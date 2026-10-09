#pragma once

#include <array>
#include <cmath>

namespace cupcup {
namespace camera {

using Vector = std::array<double, 3>;
using Matrix = std::array<double, 9>;

inline Matrix multiply(const Matrix &a, const Matrix &b)
{
    Matrix result{};
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            for (int k = 0; k < 3; ++k)
                result[3 * row + column] += a[3 * row + k] * b[3 * k + column];
    return result;
}

inline Vector rotate(const Matrix &matrix, const Vector &vector)
{
    Vector result{};
    for (int row = 0; row < 3; ++row)
        for (int k = 0; k < 3; ++k) result[row] += matrix[3 * row + k] * vector[k];
    return result;
}

inline Matrix axisAngle(Vector axis, double angle)
{
    const double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    for (double &value : axis) value /= norm;
    const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
    const double x = axis[0], y = axis[1], z = axis[2];
    return {{t*x*x+c, t*x*y-s*z, t*x*z+s*y,
             t*x*y+s*z, t*y*y+c, t*y*z-s*x,
             t*x*z-s*y, t*y*z+s*x, t*z*z+c}};
}

struct Pose {
    Vector origin{{0.0, 0.0, 0.0}};
    Matrix rotation{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
};

inline void append(Pose &pose, const Vector &translation, const Matrix &rotation)
{
    const Vector offset = rotate(pose.rotation, translation);
    for (int i = 0; i < 3; ++i) pose.origin[i] += offset[i];
    pose.rotation = multiply(pose.rotation, rotation);
}

// Sensor-only SEURobot camera FK. Angles use existing ROS messages in degrees;
// world coordinates are NUE (X north, Y up, Z east). The torso's X/Z origin is
// zero, so the result never contains supervisor position or camera truth.
// Fixed transforms come from SEURobot.proto, including ALL camera ancestors.
inline Pose seurPose(double yaw, double pitch, double roll,
                     double headYaw, double headPitch, double rootHeight = 0.365)
{
    constexpr double radians = 0.017453292519943295;
    const Vector x{{1.0, 0.0, 0.0}}, y{{0.0, 1.0, 0.0}}, z{{0.0, 0.0, 1.0}};
    const Vector zero{{0.0, 0.0, 0.0}};
    const Matrix identity{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    Pose pose;
    pose.origin[1] = rootHeight;
    // R2023b's NUE implementation explicitly uses Ry(yaw) Rz(pitch) Rx(roll).
    pose.rotation = multiply(multiply(axisAngle(y, yaw * radians),
        axisAngle(z, pitch * radians)), axisAngle(x, roll * radians));
    append(pose, {{0.0, 0.16, 0.0}},
        axisAngle({{1.0, 1.0, 1.0}}, -2.094395307179586));
    append(pose, zero, axisAngle(z, headYaw * radians));
    append(pose, {{-5.658360659606089e-7, 1.8087251347035108e-7, 0.03995753752544773}},
        axisAngle({{1.0774956288450345e-6, 9.473642441671128e-8,
            -0.999999999999415}}, 1.5723890370449267));
    append(pose, zero, axisAngle(y, -headPitch * radians));
    append(pose, {{0.0, 0.0, 0.005}}, identity);
    append(pose, zero, axisAngle(x, 1.57));
    append(pose, {{0.0, 0.051, 0.0}}, axisAngle(x, -1.57));
    append(pose, zero, axisAngle(z, 1.57));
    append(pose, {{0.0, 0.011, 0.0}}, identity);
    append(pose, {{0.0, 0.005, 0.015}}, axisAngle(z, 1.57));
    return pose;
}

struct GroundPoint {
    bool valid = false;
    double x = 0.0;
    double z = 0.0;
};

inline GroundPoint project(const Pose &pose, double u, double v, int width, int height,
                           double planeHeight = 0.07, double horizontalFov = 1.3613)
{
    GroundPoint point;
    if (!std::isfinite(u + v + planeHeight + horizontalFov) || width <= 0 || height <= 0 ||
        u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 ||
        horizontalFov <= 0.0 || horizontalFov >= 3.141592653589793) return point;
    const double scale = 2.0 * std::tan(horizontalFov / 2.0);
    const Vector direction = rotate(pose.rotation,
        {{1.0, (0.5 - u) * scale, (0.5 - v) * scale * height / width}});
    // Bound grazing-ray amplification instead of inventing a huge range.
    if (!std::isfinite(direction[1]) || direction[1] >= -0.05) return point;
    const double distance = (planeHeight - pose.origin[1]) / direction[1];
    if (!std::isfinite(distance) || distance <= 0.0 || distance > 12.0) return point;
    point.x = pose.origin[0] + distance * direction[0];
    point.z = pose.origin[2] + distance * direction[2];
    point.valid = std::isfinite(point.x + point.z);
    return point;
}

}  // namespace camera
}  // namespace cupcup
