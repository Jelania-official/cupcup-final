#pragma once

#include "ball_tracker.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace cupcup {

// Optional patch-model adapter. Model weights are NOT bundled. Scores are not
// calibrated map confidence; ambiguous predictions never veto a candidate.
inline bool strongBallNegative(const std::array<float, 6> &scores)
{
    for (float value : scores) if (!std::isfinite(value)) return false;
    for (int i = 0; i < 3; ++i) if (scores[i] < 0 || scores[i] > 1) return false;
    if (std::abs(scores[0] + scores[1] + scores[2] - 1) > .01) return false;
    return scores[0] >= .9f && scores[2] <= .1f;
}

class BallCandidateVerifier {
public:
    void load(const std::string &path)
    {
        net_ = cv::dnn::readNetFromONNX(path);
        const int shape[] = {1, 32, 32, 1};
        infer(cv::Mat(4, shape, CV_32F, cv::Scalar(0))); // Validate before use.
    }

    bool enabled() const { return !net_.empty(); }
    void disable() { net_ = cv::dnn::Net(); }

    std::array<float, 6> scores(const cv::Mat &rgb, const BallObservation &ball)
    {
        if (rgb.empty() || rgb.type() != CV_8UC3 || !ball.valid ||
            !std::isfinite(ball.x + ball.y + ball.radius) || ball.radius <= 0 ||
            ball.radius > 1 || ball.x < 0 || ball.x > 1 || ball.y < 0 || ball.y > 1)
            throw std::invalid_argument("invalid ball verifier input");
        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
        // Matches the offline probe, not B-Human's camera-projected crop size.
        int area = static_cast<int>(ball.radius * rgb.cols * 3.5);
        area += 4 - area % 4;
        const int cx = static_cast<int>(ball.x * rgb.cols), cy = static_cast<int>(ball.y * rgb.rows);
        std::array<unsigned char, 1024> patch, sorted;
        for (int y = 0; y < 32; ++y) for (int x = 0; x < 32; ++x) {
            const int px = std::max(0, std::min(rgb.cols - 1,
                static_cast<int>(cx - area / 2.0 + x * area / 32.0)));
            const int py = std::max(0, std::min(rgb.rows - 1,
                static_cast<int>(cy - area / 2.0 + y * area / 32.0)));
            patch[y * 32 + x] = gray.at<unsigned char>(py, px);
        }
        sorted = patch;
        std::nth_element(sorted.begin(), sorted.begin() + 20, sorted.end(), std::greater<unsigned char>());
        const float maximum = std::max(1, static_cast<int>(sorted[20]));
        const int shape[] = {1, 32, 32, 1};
        cv::Mat input(4, shape, CV_32F);
        for (int i = 0; i < 1024; ++i)
            input.ptr<float>()[i] = static_cast<unsigned char>(
                std::min(float(patch[i]), maximum) * 255.f / maximum);
        return infer(input);
    }

private:
    std::array<float, 6> infer(const cv::Mat &input)
    {
        if (net_.empty()) throw std::runtime_error("empty ball verifier model");
        net_.setInput(input);
        const cv::Mat output = net_.forward();
        if (output.type() != CV_32F || !output.isContinuous() || output.total() != 6 ||
            !cv::checkRange(output)) throw std::runtime_error("invalid ball verifier output");
        std::array<float, 6> result;
        std::copy_n(output.ptr<float>(), 6, result.begin());
        for (int i = 0; i < 3; ++i)
            if (result[i] < 0 || result[i] > 1) throw std::runtime_error("invalid class score");
        if (std::abs(result[0] + result[1] + result[2] - 1) > .01)
            throw std::runtime_error("invalid class probabilities");
        return result;
    }
    cv::dnn::Net net_;
};
} // namespace cupcup
