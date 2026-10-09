#pragma once

#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>

namespace cupcup {

// Platform-specific consistency check for the current black/white textured
// ball. Uniform gray own-knee surfaces can get very high YOEO ball scores.
// This is not a learned detector, a body mask, or valid for arbitrary balls.
inline bool hasBallPattern(const cv::Mat &rgb, double x, double y, double radius)
{
    if (rgb.empty() || rgb.type() != CV_8UC3 ||
        !std::isfinite(x + y + radius) || radius <= 0.0 || radius > 1.0 ||
        x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0) return false;
    const double cx = x * rgb.cols, cy = y * rgb.rows;
    const double core = 0.8 * radius * rgb.cols;
    const int x0 = std::max(0, static_cast<int>(std::floor(cx - core)));
    const int x1 = std::min(rgb.cols - 1, static_cast<int>(std::ceil(cx + core)));
    const int y0 = std::max(0, static_cast<int>(std::floor(cy - core)));
    const int y1 = std::min(rgb.rows - 1, static_cast<int>(std::ceil(cy + core)));
    const int stride = std::max(1, static_cast<int>(core / 20.0));
    int samples = 0, dark = 0, light = 0;
    for (int py = y0; py <= y1; py += stride) {
        const auto *row = rgb.ptr<cv::Vec3b>(py);
        for (int px = x0; px <= x1; px += stride) {
            if ((px-cx)*(px-cx) + (py-cy)*(py-cy) > core*core) continue;
            ++samples;
            const auto &pixel = row[px];
            const int low = std::min({pixel[0], pixel[1], pixel[2]});
            const int high = std::max({pixel[0], pixel[1], pixel[2]});
            if (high - low > 40) continue;
            const int intensity = (pixel[0] + pixel[1] + pixel[2]) / 3;
            if (intensity < 80) ++dark;
            if (intensity > 135) ++light;
        }
    }
    // Insufficient pixels are unknown, not evidence for rejecting a far ball.
    return samples < 12 || (dark >= 0.015 * samples && light >= 0.015 * samples);
}

}  // namespace cupcup
