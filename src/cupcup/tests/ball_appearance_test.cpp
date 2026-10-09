#include "ball_appearance.hpp"
#include "ball_candidate_verifier.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    if (argc == 3 && std::string(argv[1]) == "--verify") {
        cupcup::BallCandidateVerifier verifier;
        verifier.load(argv[2]);
        std::string path;
        double x, y, radius;
        std::cout.precision(9);
        while (std::cin >> std::quoted(path) >> x >> y >> radius) {
            cv::Mat rgb = cv::imread(path);
            if (rgb.empty()) return 2;
            cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
            const auto scores = verifier.scores(rgb, {true, x, y, radius, 1});
            std::cout << "{\"scores\":[";
            for (int i = 0; i < 6; ++i) std::cout << (i ? "," : "") << scores[i];
            std::cout << "],\"reject\":" << (cupcup::strongBallNegative(scores) ? "true" : "false")
                      << "}\n";
        }
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--replay") {
        std::string path;
        double x, y, radius;
        while (std::cin >> std::quoted(path) >> x >> y >> radius) {
            cv::Mat rgb = cv::imread(path);
            if (rgb.empty()) return 2;
            cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
            std::cout << cupcup::hasBallPattern(rgb, x, y, radius) << '\n';
        }
        return 0;
    }
    cv::Mat rgb(100, 100, CV_8UC3, cv::Scalar(143, 143, 143));
    assert(!cupcup::hasBallPattern(rgb, .5, .5, .2));
    rgb(cv::Rect(45, 34, 5, 32)).setTo(cv::Scalar(255, 255, 255));
    assert(!cupcup::hasBallPattern(rgb, .5, .5, .2));  // own knee + white number
    rgb.setTo(cv::Scalar(230, 230, 230));
    cv::circle(rgb, cv::Point(47, 50), 6, cv::Scalar(20, 20, 20), -1);
    assert(cupcup::hasBallPattern(rgb, .5, .5, .2));
    rgb.setTo(cv::Scalar(35, 150, 35));
    assert(!cupcup::hasBallPattern(rgb, .5, .5, .2));  // grass is not white texture
    assert(cupcup::hasBallPattern(rgb, .5, .5, .005));  // too small: unknown
    assert(!cupcup::hasBallPattern(cv::Mat(), .5, .5, .2));
    assert(!cupcup::hasBallPattern(rgb, -1.0, .5, .2));
    assert(!cupcup::hasBallPattern(rgb, .5, 2.0, .2));
    assert(!cupcup::hasBallPattern(rgb, .5, .5, 1e20));
    assert(!cupcup::hasBallPattern(rgb, NAN, .5, .2));
    assert(cupcup::strongBallNegative({.95f, .01f, .04f, 16, 16, 9}));
    assert(!cupcup::strongBallNegative({.3f, .01f, .69f, 16, 16, 9}));
    assert(!cupcup::strongBallNegative({.95f, .01f, NAN, 16, 16, 9}));
    assert(!cupcup::strongBallNegative({1.1f, 0, -.1f, 16, 16, 9}));
    assert(!cupcup::strongBallNegative({.95f, .5f, .04f, 16, 16, 9}));
    cupcup::BallCandidateVerifier unloaded;
    assert(!unloaded.enabled());
    unloaded.disable();
    assert(!unloaded.enabled());
}
