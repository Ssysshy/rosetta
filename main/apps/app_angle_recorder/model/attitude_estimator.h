#pragma once
#include <array>

namespace model::angle {

using Quaternion = std::array<float, 4>;

class AttitudeEstimator {
public:
    bool initialize(const std::array<float, 3>& gravity);
    bool update(const std::array<float, 3>& accel, const std::array<float, 3>& gyro, float seconds);
    const Quaternion& orientation() const;
    static std::array<float, 3> relativeAngles(const Quaternion& reference, const Quaternion& current);

private:
    Quaternion _orientation{1.0F, 0.0F, 0.0F, 0.0F};
};

}  // namespace model::angle
