#pragma once

#include "world_model.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace cupcup {

struct RobotBox {
    bool valid = false;
    double x = 0.0, y = 0.0, width = 0.0, height = 0.0, score = 0.0;
    RobotTeam team = RobotTeam::Unknown;
};

inline cv::Rect clippedRobotBox(const RobotBox &box, cv::Size size)
{
    if (!box.valid || !std::isfinite(box.x + box.y + box.width + box.height) ||
        box.width <= 0.0 || box.height <= 0.0) return {};
    // Clamp before converting to int, including malformed network coordinates.
    const int x0 = static_cast<int>(std::max(0.0, std::min(1.0, box.x - box.width / 2)) * size.width);
    const int y0 = static_cast<int>(std::max(0.0, std::min(1.0, box.y - box.height / 2)) * size.height);
    const int x1 = static_cast<int>(std::max(0.0, std::min(1.0, box.x + box.width / 2)) * size.width);
    const int y1 = static_cast<int>(std::max(0.0, std::min(1.0, box.y + box.height / 2)) * size.height);
    return {x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)};
}

// Team markers in the released world are saturated red/blue. This is a color
// evidence check inside a robot box, not a second robot detector or jersey OCR.
inline RobotTeam classifyRobotTeam(const cv::Mat &rgb, const RobotBox &box)
{
    if (rgb.empty() || rgb.type() != CV_8UC3) return RobotTeam::Unknown;
    const auto rect = clippedRobotBox(box, rgb.size());
    if (rect.area() < 64) return RobotTeam::Unknown;
    cv::Mat hsv;
    cv::cvtColor(rgb(rect), hsv, cv::COLOR_RGB2HSV);
    int red = 0, blue = 0;
    for (int row = 0; row < hsv.rows; ++row) for (int col = 0; col < hsv.cols; ++col) {
        const auto pixel = hsv.at<cv::Vec3b>(row, col);
        if (pixel[1] < 100 || pixel[2] < 45) continue;
        if (pixel[0] <= 12 || pixel[0] >= 170) ++red;
        if (pixel[0] >= 100 && pixel[0] <= 135) ++blue;
    }
    // A side-on marker occupies only ~1% of a full-body box in released RGB.
    // Keep the absolute support and color-dominance gates: a few colored noise
    // pixels, weak saturation, or mixed teams must still stay Unknown.
    const int minimum = std::max(12, rect.area() / 100);
    if (red >= minimum && red >= 3 * (blue + 1)) return RobotTeam::Red;
    if (blue >= minimum && blue >= 3 * (red + 1)) return RobotTeam::Blue;
    return RobotTeam::Unknown;
}

struct RobotMarkerPose {
    bool valid = false;
    // OpenCV camera coordinates: right, down, forward, in metres. This is the
    // front number PATCH centre, not a body pose, player ID, or world position.
    cv::Vec3d translation{0.0, 0.0, 0.0};
    double reprojectionError = 0.0, translationSpread = 0.0, minimumSide = 0.0;
};

// Released SEURobot front texture is a 0.074 m square surrounded by a team
// panel. This optional geometry measurement needs four visible corners;
// neither an arbitrary white limb nor the differently sized back panel fits
// this contract. Keep ambiguous/partial/small markers unknown, never clamp a
// failed pose into a plausible range. No OCR or extra network is involved.
inline RobotMarkerPose estimateRobotNumberPatch(const cv::Mat &rgb, const RobotBox &box)
{
    RobotMarkerPose result;
    if (rgb.empty() || rgb.type() != CV_8UC3) return result;
    const auto rect = clippedRobotBox(box, rgb.size());
    if (rect.width < 12 || rect.height < 12) return result;
    cv::Mat hsv, white;
    cv::cvtColor(rgb(rect), hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 170), cv::Scalar(179, 65, 255), white);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(white, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2d> corners;
    double minimumSide = 0.0;
    for (const auto &contour : contours) {
        std::vector<cv::Point> quad;
        cv::approxPolyDP(contour, quad, 0.035 * cv::arcLength(contour, true), true);
        if (quad.size() != 4 || !cv::isContourConvex(quad)) continue;
        double low = 1e9, high = 0.0;
        bool clipped = false;
        for (std::size_t i = 0; i < 4; ++i) {
            const double length = cv::norm(quad[i] - quad[(i + 1) % 4]);
            low = std::min(low, length); high = std::max(high, length);
            clipped |= quad[i].x <= 1 || quad[i].y <= 1 ||
                quad[i].x >= rect.width - 2 || quad[i].y >= rect.height - 2;
        }
        if (clipped || low < 8.0 || high > 2.5 * low) continue;
        cv::Mat mask(white.size(), CV_8U, cv::Scalar(0)), expanded;
        cv::fillConvexPoly(mask, quad, cv::Scalar(255));
        const int radius = std::max(2, static_cast<int>(std::round(low * 0.15)));
        cv::dilate(mask, expanded, cv::Mat::ones(2 * radius + 1, 2 * radius + 1, CV_8U));
        int dark = 0, inside = 0, red = 0, blue = 0, ring = 0;
        for (int y = 0; y < hsv.rows; ++y) for (int x = 0; x < hsv.cols; ++x) {
            const auto p = hsv.at<cv::Vec3b>(y, x);
            if (mask.at<unsigned char>(y, x)) { ++inside; dark += p[2] < 100; }
            else if (expanded.at<unsigned char>(y, x)) {
                ++ring;
                if (p[1] < 100 || p[2] < 45) continue;
                red += p[0] <= 12 || p[0] >= 170;
                blue += p[0] >= 100 && p[0] <= 135;
            }
        }
        if (inside == 0 || dark < 0.04 * inside || dark > 0.45 * inside || ring == 0) continue;
        const bool redPanel = red >= 0.5 * ring && red >= 3 * (blue + 1);
        const bool bluePanel = blue >= 0.5 * ring && blue >= 3 * (red + 1);
        if ((!redPanel && !bluePanel) || (redPanel && box.team == RobotTeam::Blue) ||
            (bluePanel && box.team == RobotTeam::Red)) continue;
        // Two supported patches in one detector ROI imply an overlap: do not
        // choose one by proximity or reprojection error and invent identity.
        if (!corners.empty()) return result;
        cv::Point2d centre;
        for (const auto &p : quad) centre += cv::Point2d(p);
        centre *= 0.25;
        std::sort(quad.begin(), quad.end(), [centre](const cv::Point &a, const cv::Point &b) {
            return std::atan2(a.y - centre.y, a.x - centre.x) <
                std::atan2(b.y - centre.y, b.x - centre.x);
        });
        // Any cyclic starting corner is valid for square translation. Start
        // top-left for reproducibility; do not infer jersey orientation.
        const auto first = std::min_element(quad.begin(), quad.end(), [](const cv::Point &a, const cv::Point &b) {
            return a.x + a.y < b.x + b.y;
        });
        std::rotate(quad.begin(), first, quad.end());
        for (const auto &p : quad) corners.emplace_back(p.x + rect.x, p.y + rect.y);
        minimumSide = low;
    }
    if (corners.empty()) return result;
    const double focal = rgb.cols / (2.0 * std::tan(1.3613 / 2.0)), half = 0.074 / 2.0;
    const cv::Matx33d camera(focal, 0, rgb.cols / 2.0, 0, focal, rgb.rows / 2.0, 0, 0, 1);
    const std::vector<cv::Point3d> object{{-half, half, 0}, {half, half, 0},
                                        {half, -half, 0}, {-half, -half, 0}};
    std::vector<cv::Mat> rotations, translations;
    cv::Mat errors;
    cv::solvePnPGeneric(object, corners, camera, cv::noArray(), rotations, translations,
                       false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), errors);
    if (translations.size() != 2 || errors.total() != 2 || !cv::checkRange(errors)) return result;
    cv::Vec3d t[2];
    double error[2];
    for (int i = 0; i < 2; ++i) {
        // Refine both geometric solutions. OpenCV 4.5.4 can leave a perfectly
        // front-facing, off-axis square with a nonzero IPPE residual; do not
        // relax the reprojection gate to hide that numerical failure.
        cv::solvePnPRefineLM(object, corners, camera, cv::noArray(), rotations[i], translations[i]);
        if (!cv::checkRange(translations[i])) return result;
        t[i] = cv::Vec3d(translations[i].at<double>(0), translations[i].at<double>(1), translations[i].at<double>(2));
        std::vector<cv::Point2d> reprojection;
        cv::projectPoints(object, rotations[i], translations[i], camera, cv::noArray(), reprojection);
        double sum = 0.0;
        for (std::size_t k = 0; k < corners.size(); ++k) {
            const auto delta = reprojection[k] - corners[k];
            sum += delta.dot(delta);
        }
        error[i] = t[i][2] > 0.0 ? std::sqrt(sum / (2 * corners.size())) : 1e9;
        if (!std::isfinite(error[i])) return result;
    }
    const int best = error[0] <= error[1] ? 0 : 1;
    const double spread = error[0] <= 1 && error[1] <= 1 ? cv::norm(t[0] - t[1]) : 0.0;
    const double distance = cv::norm(t[best]);
    if (error[best] > 1.0 || spread > 0.10 || distance < 0.15 || distance > 6.0) return result;
    result.valid = true; result.translation = t[best]; result.reprojectionError = error[best];
    result.translationSpread = spread; result.minimumSide = minimumSide;
    return result;
}

struct RobotNumberPatch {
    RobotBox panel;  // visible team panel ROI, NOT a full-body box
    RobotMarkerPose pose;
};

// A number landmark may be visible even when a model trained on other robot
// bodies misses this released model. Use saturated team components solely as
// search ROIs; require the same complete white-square geometry inside each.
inline std::vector<RobotNumberPatch> detectRobotNumberPatches(const cv::Mat &rgb)
{
    std::vector<RobotNumberPatch> result;
    if (rgb.empty() || rgb.type() != CV_8UC3) return result;
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
    for (const auto team : {RobotTeam::Red, RobotTeam::Blue}) {
        cv::Mat mask, wrapped;
        if (team == RobotTeam::Red) {
            cv::inRange(hsv, cv::Scalar(0, 100, 45), cv::Scalar(12, 255, 255), mask);
            cv::inRange(hsv, cv::Scalar(170, 100, 45), cv::Scalar(179, 255, 255), wrapped);
            mask |= wrapped;
        } else cv::inRange(hsv, cv::Scalar(100, 100, 45), cv::Scalar(135, 255, 255), mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours) {
            cv::Rect roi = cv::boundingRect(contour);
            if (roi.width < 12 || roi.height < 12 || roi.area() > 0.4 * rgb.total() ||
                roi.width > 3 * roi.height || roi.height > 3 * roi.width) continue;
            roi = (roi + cv::Size(4, 4) - cv::Point(2, 2)) & cv::Rect(0, 0, rgb.cols, rgb.rows);
            RobotBox panel{true, (roi.x + roi.width / 2.0) / rgb.cols,
                (roi.y + roi.height / 2.0) / rgb.rows, double(roi.width) / rgb.cols,
                double(roi.height) / rgb.rows, 1.0, team};
            const auto pose = estimateRobotNumberPatch(rgb, panel);
            if (pose.valid) result.push_back({panel, pose});
            if (result.size() == 4) return result;
        }
    }
    return result;
}

inline std::vector<RobotBox> filterRobotBoxes(std::vector<RobotBox> boxes, const cv::Mat &rgb)
{
    std::sort(boxes.begin(), boxes.end(), [](const RobotBox &a, const RobotBox &b) { return a.score > b.score; });
    std::vector<RobotBox> result;
    for (auto box : boxes) {
        const cv::Rect rect = clippedRobotBox(box, rgb.size());
        if (rect.area() < 64) continue;
        bool duplicate = false;
        for (const auto &accepted : result) {
            const auto other = clippedRobotBox(accepted, rgb.size());
            const double overlap = (rect & other).area();
            if (overlap / std::max(1.0, rect.area() + other.area() - overlap) > 0.4) { duplicate = true; break; }
        }
        if (duplicate) continue;
        box.team = classifyRobotTeam(rgb, box);
        result.push_back(box);
        if (result.size() == 4) break;
    }
    return result;
}

inline std::vector<RobotBox> decodeRobotBoxes(const std::vector<cv::Mat> &outputs, const cv::Mat &rgb,
                                             double minimumConfidence = 0.30)
{
    if (rgb.empty() || rgb.type() != CV_8UC3 || !std::isfinite(minimumConfidence) ||
        minimumConfidence < 0.1 || minimumConfidence > 1.0) return {};
    const int side = std::max(rgb.cols, rgb.rows);
    const int left = (side - rgb.cols) / 2, top = (side - rgb.rows) / 2;
    std::vector<RobotBox> boxes;
    auto sigmoid = [](double v) { return 1.0 / (1.0 + std::exp(-v)); };
    for (const auto &output : outputs) {
        if (output.dims != 4 || output.type() != CV_32F || output.size[1] != 7 ||
            output.size[2] <= 0 || output.size[3] <= 0) continue;
        const int rows = output.size[2], cols = output.size[3];
        const float *data = output.ptr<float>();
        for (int row = 0; row < rows; ++row) for (int col = 0; col < cols; ++col) {
            auto value = [&](int channel) { return data[channel * rows * cols + row * cols + col]; };
            const double confidence = sigmoid(value(4)) * sigmoid(value(6));
            if (confidence <= minimumConfidence || value(6) <= value(5)) continue;
            const double x = ((sigmoid(value(0)) + col) * side / cols - left) / rgb.cols;
            const double y = ((sigmoid(value(1)) + row) * side / rows - top) / rgb.rows;
            const double width = std::exp(value(2)) * 99.99983 * side / (416.0 * rgb.cols);
            const double height = std::exp(value(3)) * 99.99983 * side / (416.0 * rgb.rows);
            if (!std::isfinite(x + y + width + height + confidence) ||
                x < 0.0 || x >= 1.0 || y < 0.0 || y >= 0.85 ||
                width <= 0.025 || height <= 0.06) continue;
            boxes.push_back({true, x, y, width, height, confidence});
        }
    }
    return filterRobotBoxes(std::move(boxes), rgb);
}

}  // namespace cupcup
