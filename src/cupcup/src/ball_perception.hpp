#pragma once

#include "ball_appearance.hpp"
#include "ball_candidate_verifier.hpp"
#include "ball_tracker.hpp"  // Observation type, not the prediction filter.
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace cupcup {

// Production decoder shared with image replay. Optional prior preserves the
// player's existing candidate ranking; replayed independent fixtures have none.
inline BallObservation decodeBall(const std::vector<cv::Mat> &outputs, const cv::Mat &rgb,
                                  double minScore, bool patternFilter,
                                  const BallObservation &prior = {},
                                  BallCandidateVerifier *verifier = nullptr) {
    BallObservation best;
    if (rgb.empty() || rgb.type() != CV_8UC3 || !std::isfinite(minScore) ||
        minScore < 0 || minScore > 1) return best;
    const int side = std::max(rgb.cols, rgb.rows), left = (side - rgb.cols) / 2,
              top = (side - rgb.rows) / 2;
    const auto sigmoid = [](double value) { return 1.0 / (1.0 + std::exp(-value)); };
    double bestRank = 0.0;
    for (const auto &output : outputs) {
        if (output.empty() || output.dims != 4 || output.size[0] != 1 || output.size[1] != 7 ||
            output.type() != CV_32F || !output.isContinuous()) continue;
        const int height = output.size[2], width = output.size[3];
        const float *data = output.ptr<float>();
        for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
            const auto value = [&](int channel) { return data[channel * height * width + y * width + x]; };
            bool finite = true;
            for (int c = 0; c < 7; ++c) finite = finite && std::isfinite(value(c));
            if (!finite) continue;
            const double objectness = sigmoid(value(4));
            const double confidence = objectness * sigmoid(value(5));
            const double robotConfidence = objectness * sigmoid(value(6));
            if (std::max(confidence, robotConfidence) < 0.30) continue;
            const double cx = (sigmoid(value(0)) + x) * side / width - left;
            const double cy = (sigmoid(value(1)) + y) * side / height - top;
            const double w = std::exp(value(2)) * 99.99983 * side / 416.0;
            const double h = std::exp(value(3)) * 99.99983 * side / 416.0;
            if (!std::isfinite(w + h + cx + cy + confidence) || cx < 0 || cy < 0 ||
                cx >= rgb.cols || cy >= rgb.rows) continue;
            const double nx = cx / rgb.cols, ny = cy / rgb.rows;
            const double radius = (w + h) / (4.0 * rgb.cols);
            if (confidence < minScore || value(5) < value(6) || radius > 0.11) continue;
            // A clipped black/white foot can satisfy the texture check. Require
            // the estimated ball disc, not an anisotropic network box, to fit.
            const double pixels = radius * rgb.cols;
            if (cx - pixels < 0 || cx + pixels > rgb.cols ||
                cy - pixels < 0 || cy + pixels > rgb.rows) continue;
            if (patternFilter && !hasBallPattern(rgb, nx, ny, radius)) continue;
            double rank = confidence;
            if (prior.valid) {
                const double dx = nx - prior.x, dy = ny - prior.y;
                rank *= 0.65 + 0.35 * std::exp(-20.0 * (dx * dx + dy * dy));
            }
            if (rank > bestRank) {
                const BallObservation candidate{true, nx, ny, radius, confidence};
                if (verifier && verifier->enabled() &&
                    strongBallNegative(verifier->scores(rgb, candidate))) continue;
                bestRank = rank;
                best.valid = true; best.x = nx; best.y = ny;
                best.radius = radius; best.score = confidence;
            }
        }
    }
    return best;
}
}  // namespace cupcup
