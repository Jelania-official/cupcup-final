#include "camera_geometry.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    using namespace cupcup::camera;
    if ((argc == 9 || argc == 10) && std::string(argv[1]) == "--project") {
        double values[8];
        for (int i = 0; i < argc - 2; ++i) {
            values[i] = std::stod(argv[i + 2]);
            if (!std::isfinite(values[i])) return 2;
        }
        const double rootHeight = argc == 10 ? values[7] : .365;
        if (rootHeight < .25 || rootHeight > .50) return 2;
        const auto pose = seurPose(values[0], values[1], values[2], values[3], values[4], rootHeight);
        const auto point = project(pose, values[5], values[6], 640, 480);
        std::cout.precision(17);
        std::cout << "{\"valid\":" << (point.valid ? "true" : "false")
                  << ",\"x\":" << point.x << ",\"z\":" << point.z
                  << ",\"root_height_m\":" << rootHeight << "}\n";
        return 0;
    }
    // Offline evaluator uses exactly the sensor-only production FK, rather
    // than a second Python transcription or supervisor camera truth.
    if (argc == 7 && std::string(argv[1]) == "--pose") {
        double angles[5];
        for (int i = 0; i < 5; ++i) {
            angles[i] = std::stod(argv[i + 2]);
            if (!std::isfinite(angles[i])) return 2;
        }
        const auto pose = seurPose(angles[0], angles[1], angles[2], angles[3], angles[4]);
        std::cout.precision(17);
        std::cout << "{\"origin\":[" << pose.origin[0] << ',' << pose.origin[1] << ',' << pose.origin[2]
                  << "],\"rotation\":[";
        for (int i = 0; i < 9; ++i) std::cout << (i ? "," : "") << pose.rotation[i];
        std::cout << "]}\n";
        return 0;
    }
    if (argc != 1) return 2;
    const Pose front = seurPose(0.0, 0.0, 0.0, 0.0, 30.0);
    const Vector axis = rotate(front.rotation, {{1.0, 0.0, 0.0}});
    assert(axis[0] > 0.86 && axis[0] < 0.87);
    assert(std::abs(axis[1] + 0.5) < 0.003);
    assert(std::abs(axis[2]) < 0.003);
    const auto point = project(front, 0.5, 0.5, 640, 480);
    assert(point.valid && point.x > 0.7 && point.x < 1.1);
    const Pose mirror = seurPose(180.0, 0.0, 0.0, 0.0, 30.0);
    const auto mirrored = project(mirror, 0.5, 0.5, 640, 480);
    assert(mirrored.valid);
    assert(std::abs(mirrored.x + point.x) < 1e-9);
    assert(std::abs(mirrored.z + point.z) < 1e-9);
    const auto east = project(seurPose(90, 0, 0, 0, 30), .5, .5, 640, 480);
    assert(east.valid);
    assert(std::abs(east.z + point.x) < 1e-9);
    assert(std::abs(east.x - point.z) < 1e-9);
    const auto left = project(seurPose(0, 0, 0, 20, 30), .5, .5, 640, 480);
    assert(left.valid && left.z < -0.1);
    assert(!project(seurPose(0, 0, 0, 0, 0), .5, .5, 640, 480).valid);
    assert(!project(front, .5, .5, 0, 480).valid);
    assert(!project(front, std::numeric_limits<double>::quiet_NaN(), .5, 640, 480).valid);
    assert(!project(front, -0.1, .5, 640, 480).valid);
    const Pose tilted = seurPose(-35, 7, -4, 23, 41);
    for (int row = 0; row < 3; ++row)
        for (int other = 0; other < 3; ++other) {
            double dot = 0.0;
            for (int k = 0; k < 3; ++k)
                dot += tilted.rotation[3 * row + k] * tilted.rotation[3 * other + k];
            assert(std::abs(dot - (row == other ? 1.0 : 0.0)) < 1e-12);
        }
}
