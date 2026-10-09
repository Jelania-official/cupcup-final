#include "robot_perception.hpp"
#include "ball_perception.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>

using namespace cupcup;

int main(int argc, char **argv) {
    const bool verifiedReplay = argc == 4 && std::string(argv[1]) == "--replay-verified";
    if (verifiedReplay || ((argc == 3 || argc == 5) && std::string(argv[1]) == "--replay")) {
        const double threshold = argc == 5 && std::string(argv[3]) == "--confidence" ?
            std::stod(argv[4]) : 0.30;
        if (!std::isfinite(threshold) || threshold < 0.1 || threshold > 1.0 ||
            (argc == 5 && std::string(argv[3]) != "--confidence")) return 2;
        cv::dnn::Net net = cv::dnn::readNetFromONNX(argv[2]);
        BallCandidateVerifier verifier;
        if (verifiedReplay) verifier.load(argv[3]);
        std::string path;
        while (std::cin >> std::quoted(path)) {
            cv::Mat bgr = cv::imread(path), rgb, square, resized;
            if (bgr.empty()) return 2;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            const int side = std::max(rgb.cols, rgb.rows), left = (side - rgb.cols) / 2, top = (side - rgb.rows) / 2;
            cv::copyMakeBorder(rgb, square, top, side - rgb.rows - top, left, side - rgb.cols - left,
                              cv::BORDER_CONSTANT, cv::Scalar());
            cv::resize(square, resized, cv::Size(416, 416), 0, 0, cv::INTER_NEAREST);
            net.setInput(cv::dnn::blobFromImage(resized, 1.0 / 255.0));
            std::vector<cv::Mat> outputs;
            net.forward(outputs, net.getUnconnectedOutLayersNames());
            std::cout << "{\"image\":" << std::quoted(path) << ",\"robots\":[";
            bool comma = false;
            for (const auto &box : decodeRobotBoxes(outputs, rgb, threshold)) {
                if (comma) std::cout << ',';
                comma = true;
                std::cout << "{\"team\":\"" << robotTeamName(box.team) << "\",\"x\":" << box.x
                    << ",\"y\":" << box.y << ",\"width\":" << box.width << ",\"height\":" << box.height
                    << ",\"score\":" << box.score << ",\"number_patch\":";
                const auto patch = estimateRobotNumberPatch(rgb, box);
                if (!patch.valid) std::cout << "null";
                else std::cout << "{\"translation\":[" << patch.translation[0] << ','
                    << patch.translation[1] << ',' << patch.translation[2]
                    << "],\"reprojection_error_px\":" << patch.reprojectionError
                    << ",\"translation_spread_m\":" << patch.translationSpread
                    << ",\"minimum_side_px\":" << patch.minimumSide << '}';
                std::cout << '}';
            }
            const auto ball = decodeBall(outputs, rgb, 0.30, true, {},
                                         verifiedReplay ? &verifier : nullptr);
            std::cout << "],\"ball\":{\"valid\":" << (ball.valid ? "true" : "false")
                << ",\"x\":" << ball.x << ",\"y\":" << ball.y << ",\"radius\":" << ball.radius
                << ",\"score\":" << ball.score << "},\"number_patches\":[";
            comma = false;
            for (const auto &marker : detectRobotNumberPatches(rgb)) {
                if (comma) std::cout << ',';
                comma = true;
                const auto &panel = marker.panel;
                const auto &pose = marker.pose;
                std::cout << "{\"team\":\"" << robotTeamName(panel.team) << "\",\"x\":" << panel.x
                    << ",\"y\":" << panel.y << ",\"width\":" << panel.width << ",\"height\":" << panel.height
                    << ",\"translation\":[" << pose.translation[0] << ',' << pose.translation[1]
                    << ',' << pose.translation[2] << "],\"reprojection_error_px\":" << pose.reprojectionError
                    << ",\"translation_spread_m\":" << pose.translationSpread
                    << ",\"minimum_side_px\":" << pose.minimumSide << '}';
            }
            std::cout << "]}\n";
        }
        return 0;
    }
    cv::Mat rgb(100, 200, CV_8UC3, cv::Scalar(100, 100, 100));
    const int ballShape[] = {1, 7, 1, 1};
    cv::Mat ballOutput(4, ballShape, CV_32F, cv::Scalar(0));
    float *ballValues = ballOutput.ptr<float>();
    ballValues[2] = ballValues[3] = -1;
    ballValues[4] = ballValues[5] = 2; ballValues[6] = -2;
    const auto decodedBall = decodeBall({ballOutput}, rgb, .30, false);
    assert(decodedBall.valid && std::abs(decodedBall.x - .5) < 1e-9);
    assert(std::abs(decodedBall.y - .5) < 1e-9 && decodedBall.radius < .11);
    assert(!decodeBall({ballOutput}, rgb, .99, false).valid);
    assert(!decodeBall({ballOutput}, rgb, .30, true).valid);  // gray has no ball pattern
    assert(!decodeBall({ballOutput}, cv::Mat(), .30, false).valid);
    assert(!decodeBall({ballOutput}, rgb, NAN, false).valid);
    // Center lies in the image, but the predicted ball box is clipped at top.
    ballValues[1] = std::log(55.0 / 145.0);  // cy=5 in the 200-pixel padded square
    assert(!decodeBall({ballOutput}, rgb, .30, false).valid);
    ballValues[1] = 0;
    ballValues[6] = std::numeric_limits<float>::quiet_NaN();
    assert(!decodeBall({ballOutput}, rgb, .30, false).valid);
    ballValues[6] = 3;
    assert(!decodeBall({ballOutput}, rgb, .30, false).valid);  // stronger robot class
    ballValues[6] = -2; ballValues[2] = ballValues[3] = 2;
    assert(!decodeBall({ballOutput}, rgb, .30, false).valid);  // oversized candidate
    const int rankedShape[] = {1, 7, 1, 2};
    cv::Mat rankedOutput(4, rankedShape, CV_32F, cv::Scalar(0));
    float *ranked = rankedOutput.ptr<float>();
    for (int cell = 0; cell < 2; ++cell) {
        ranked[4 + cell] = ranked[6 + cell] = -1;
        ranked[8 + cell] = 2; ranked[10 + cell] = cell ? 1.3 : 1;
        ranked[12 + cell] = -2;
    }
    assert(std::abs(decodeBall({rankedOutput}, rgb, .30, false).x - .75) < 1e-9);
    BallObservation priorBall;
    priorBall.valid = true; priorBall.x = .25; priorBall.y = .5;
    assert(std::abs(decodeBall({rankedOutput}, rgb, .30, false, priorBall).x - .25) < 1e-9);
    RobotBox left{true, 0.25, 0.5, 0.4, 0.8, 0.9};
    RobotBox right{true, 0.75, 0.5, 0.4, 0.8, 0.8};
    rgb(cv::Rect(35, 30, 30, 20)).setTo(cv::Scalar(180, 20, 20));
    rgb(cv::Rect(135, 30, 30, 20)).setTo(cv::Scalar(20, 20, 180));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Red);
    assert(classifyRobotTeam(rgb, right) == RobotTeam::Blue);
    auto duplicate = left; duplicate.score = 0.6;
    const auto boxes = filterRobotBoxes({left, right, duplicate}, rgb);
    assert(boxes.size() == 2 && boxes[0].team == RobotTeam::Red && boxes[1].team == RobotTeam::Blue);
    rgb.setTo(cv::Scalar(100, 100, 100));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Unknown);
    assert(decodeRobotBoxes({}, rgb, std::numeric_limits<double>::quiet_NaN()).empty());
    assert(decodeRobotBoxes({}, rgb, 0.0).empty());
    assert(decodeRobotBoxes({}, cv::Mat()).empty());
    const int dimensions[] = {1, 7, 1, 1};
    cv::Mat output(4, dimensions, CV_32F, cv::Scalar(0));
    output.ptr<float>()[4] = 1;  // objectness*robot ~= 0.366
    output.ptr<float>()[5] = -2;
    assert(decodeRobotBoxes({output}, rgb, .45).empty());
    assert(decodeRobotBoxes({output}, rgb).size() == 1);
    output.ptr<float>()[0] = std::numeric_limits<float>::quiet_NaN();
    assert(decodeRobotBoxes({output}, rgb).empty());
    cv::Mat wrongType(4, dimensions, CV_64F, cv::Scalar(0));
    assert(decodeRobotBoxes({wrongType}, rgb).empty());
    rgb(cv::Rect(35, 30, 30, 20)).setTo(cv::Scalar(180, 20, 20));
    rgb(cv::Rect(35, 50, 30, 20)).setTo(cv::Scalar(20, 20, 180));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Unknown);
    // Thin side markers between 1% and 2% of the body box are valid evidence.
    rgb.setTo(cv::Scalar(100, 100, 100));
    rgb(cv::Rect(35, 30, 5, 16)).setTo(cv::Scalar(20, 20, 180));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Blue);
    rgb(cv::Rect(35, 30, 5, 16)).setTo(cv::Scalar(180, 20, 20));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Red);
    rgb.setTo(cv::Scalar(100, 100, 100));
    rgb(cv::Rect(35, 30, 2, 5)).setTo(cv::Scalar(20, 20, 180));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Unknown);  // sparse noise
    rgb(cv::Rect(35, 30, 5, 16)).setTo(cv::Scalar(100, 100, 125));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Unknown);  // weak color
    rgb(cv::Rect(35, 30, 5, 16)).setTo(cv::Scalar(20, 20, 180));
    rgb(cv::Rect(45, 30, 5, 16)).setTo(cv::Scalar(180, 20, 20));
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Unknown);  // two teams
    left.width = std::numeric_limits<double>::quiet_NaN();
    assert(classifyRobotTeam(rgb, left) == RobotTeam::Unknown);

    // Known-size number patch geometry; not a second detector or an OCR test.
    cv::Mat markerImage(480, 640, CV_8UC3, cv::Scalar(50, 90, 50));
    const RobotBox markerBox{true, 0.5, 0.5, 0.8, 0.9, 0.9, RobotTeam::Red};
    markerImage(cv::Rect(220, 130, 114, 114)).setTo(cv::Scalar(220, 10, 10));
    markerImage(cv::Rect(240, 150, 74, 74)).setTo(cv::Scalar(240, 240, 240));
    markerImage(cv::Rect(270, 165, 12, 44)).setTo(cv::Scalar(10, 10, 10));
    const auto patch = estimateRobotNumberPatch(markerImage, markerBox);
    assert(patch.valid && std::abs(patch.translation[2] - 0.4) < 0.03);
    assert(patch.translation[0] < 0 && patch.translation[1] < 0);
    auto wrongTeam = markerBox; wrongTeam.team = RobotTeam::Blue;
    assert(!estimateRobotNumberPatch(markerImage, wrongTeam).valid);
    cv::Mat noDigit = markerImage.clone();
    noDigit(cv::Rect(270, 165, 12, 44)).setTo(cv::Scalar(240, 240, 240));
    assert(!estimateRobotNumberPatch(noDigit, markerBox).valid);
    cv::Mat greenPanel = markerImage.clone();
    greenPanel(cv::Rect(220, 130, 114, 114)).setTo(cv::Scalar(10, 220, 10));
    markerImage(cv::Rect(240, 150, 74, 74)).copyTo(greenPanel(cv::Rect(240, 150, 74, 74)));
    assert(!estimateRobotNumberPatch(greenPanel, markerBox).valid);
    cv::Mat overlap = markerImage.clone();
    markerImage(cv::Rect(220, 130, 114, 114)).copyTo(overlap(cv::Rect(360, 130, 114, 114)));
    assert(!estimateRobotNumberPatch(overlap, markerBox).valid);
    auto clippedMarker = markerBox; clippedMarker.x = 240.0 / 640; clippedMarker.width = 74.0 / 640;
    assert(!estimateRobotNumberPatch(markerImage, clippedMarker).valid);
    assert(!estimateRobotNumberPatch(cv::Mat(), markerBox).valid);
    assert(!estimateRobotNumberPatch(cv::Mat(480, 640, CV_32FC3), markerBox).valid);
    assert(detectRobotNumberPatches(markerImage).size() == 1);
    assert(detectRobotNumberPatches(overlap).size() == 2);
    assert(detectRobotNumberPatches(greenPanel).empty());
    assert(detectRobotNumberPatches(noDigit).empty());
    assert(detectRobotNumberPatches(cv::Mat()).empty());

    WorldModel map;
    const std::vector<RobotObservation> scene = {
        {{0.0, 0.0}, RobotTeam::Red, 0.8, 1.0},
        {{1.5, 0.0}, RobotTeam::Blue, 0.8, 1.0},
        {{0.0, 1.5}, RobotTeam::Unknown, 0.8, 1.0}};
    map.updateRobots(scene, 1.0);
    assert(map.robots().size() == 3 && map.robots()[0].team == RobotTeam::Unknown);
    map.updateRobots(scene, 1.0);  // repeated frame cannot confirm a team
    assert(map.robots()[0].team == RobotTeam::Unknown);
    map.updateRobots(scene, 1.1);
    assert(map.robots()[0].team == RobotTeam::Red && map.robots()[1].team == RobotTeam::Blue);
    assert(map.robots()[2].team == RobotTeam::Unknown);
    assert(map.robots()[0].trackId != map.robots()[1].trackId);
    for (const auto &robot : map.robots()) assert(robot.position.confidence <= 0.25);
    map.expire(2.0);
    for (const auto &robot : map.robots()) assert(!robot.position.valid);
    map.reset(); assert(map.robots().empty());
    std::cout << "robot perception tests passed\n";
}
