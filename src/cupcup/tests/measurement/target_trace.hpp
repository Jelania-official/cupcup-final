#pragma once
// Measurement-only natural GetAngles response logging. These are motor targets,
// not measured joints and not an extra request to the consuming motion service.
#include <common/msg/body_angles.hpp>
#include <array>
#include <ostream>

namespace cupcup_measure {
using Angles = common::msg::BodyAngles;
const std::array<float Angles::*, 12> legFields{{
    &Angles::left_hip_yaw, &Angles::left_hip_roll, &Angles::left_hip_pitch,
    &Angles::left_knee, &Angles::left_ankle_pitch, &Angles::left_ankle_roll,
    &Angles::right_hip_yaw, &Angles::right_hip_roll, &Angles::right_hip_pitch,
    &Angles::right_knee, &Angles::right_ankle_pitch, &Angles::right_ankle_roll}};
inline void header(std::ostream &out) {
    out << "left_hip_yaw,left_hip_roll,left_hip_pitch,left_knee,left_ankle_pitch,left_ankle_roll,"
           "right_hip_yaw,right_hip_roll,right_hip_pitch,right_knee,right_ankle_pitch,right_ankle_roll\n";
}
inline void angles(std::ostream &out, const Angles &body) {
    for (auto field : legFields) out << ',' << body.*field;
    out << '\n';
}
}  // namespace cupcup_measure
