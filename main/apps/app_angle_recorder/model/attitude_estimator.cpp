#include "attitude_estimator.h"
#include <algorithm>
#include <cmath>

namespace model::angle {
namespace {
constexpr float Pi = 3.14159265358979323846F;

Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

bool normalize(Quaternion& q)
{
    float squared = 0.0F;
    for (const float value : q) {
        squared += value * value;
    }
    if (!std::isfinite(squared) || squared < 1.0e-12F) {
        return false;
    }
    const float inverse = 1.0F / std::sqrt(squared);
    for (float& value : q) {
        value *= inverse;
    }
    return true;
}
}  // namespace

bool AttitudeEstimator::initialize(const std::array<float, 3>& gravity, const std::array<float, 3>& bias)
{
    const float norm = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
    if (!std::isfinite(norm) || norm < 1.0e-6F) {
        return false;
    }
    const float roll = std::atan2(gravity[1], gravity[2]);
    const float pitch = std::atan2(-gravity[0], std::hypot(gravity[1], gravity[2]));
    const float cr = std::cos(roll * 0.5F);
    const float sr = std::sin(roll * 0.5F);
    const float cp = std::cos(pitch * 0.5F);
    const float sp = std::sin(pitch * 0.5F);
    _orientation = {cr * cp, sr * cp, cr * sp, -sr * sp};
    _bias = bias;
    return normalize(_orientation);
}

bool AttitudeEstimator::update(const std::array<float, 3>& accel, const std::array<float, 3>& gyro,
                               float seconds)
{
    if (!std::isfinite(seconds) || seconds <= 0.0F || seconds > 0.1F) {
        return false;
    }
    std::array<float, 3> rate{};
    for (size_t axis = 0; axis < rate.size(); ++axis) {
        if (!std::isfinite(accel[axis]) || !std::isfinite(gyro[axis])) {
            return false;
        }
        rate[axis] = (gyro[axis] - _bias[axis]) * (Pi / 180.0F);
    }
    const float norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    const Quaternion& q = _orientation;
    if (norm >= 0.85F && norm <= 1.15F) {
        const std::array<float, 3> measured{accel[0] / norm, accel[1] / norm, accel[2] / norm};
        const std::array<float, 3> estimated{2.0F * (q[1] * q[3] - q[0] * q[2]),
                                             2.0F * (q[0] * q[1] + q[2] * q[3]),
                                             q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]};
        constexpr float ProportionalGain = 2.0F;
        rate[0] += ProportionalGain * (measured[1] * estimated[2] - measured[2] * estimated[1]);
        rate[1] += ProportionalGain * (measured[2] * estimated[0] - measured[0] * estimated[2]);
        rate[2] += ProportionalGain * (measured[0] * estimated[1] - measured[1] * estimated[0]);
    }
    const float speed = std::sqrt(rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2]);
    const float halfAngle = speed * seconds * 0.5F;
    const float scale = speed > 1.0e-6F ? std::sin(halfAngle) / speed : seconds * 0.5F;
    Quaternion next = multiply(q, {std::cos(halfAngle), rate[0] * scale, rate[1] * scale, rate[2] * scale});
    if (!normalize(next)) {
        return false;
    }
    _orientation = next;
    return true;
}

const Quaternion& AttitudeEstimator::orientation() const
{
    return _orientation;
}

std::array<float, 3> AttitudeEstimator::relativeAngles(const Quaternion& reference, const Quaternion& current)
{
    const Quaternion conjugate{reference[0], -reference[1], -reference[2], -reference[3]};
    const Quaternion q = multiply(conjugate, current);
    constexpr float ToDegrees = 180.0F / Pi;
    const float pitchSine = std::clamp(2.0F * (q[0] * q[2] - q[3] * q[1]), -1.0F, 1.0F);
    return {std::atan2(2.0F * (q[0] * q[1] + q[2] * q[3]),
                      1.0F - 2.0F * (q[1] * q[1] + q[2] * q[2])) * ToDegrees,
            std::asin(pitchSine) * ToDegrees,
            std::atan2(2.0F * (q[0] * q[3] + q[1] * q[2]),
                      1.0F - 2.0F * (q[2] * q[2] + q[3] * q[3])) * ToDegrees};
}

}  // namespace model::angle
