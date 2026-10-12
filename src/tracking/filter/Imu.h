#pragma once
// L3: IMU sample conversion and the stillness detector used by the filter (zero-velocity updates, base
// bootstrap, tilt refinement).
//
// Scale: watchman devices run the IMU at +-8 g / +-2000 deg/s (libsurvive survive_default_devices.c uses
// 2/8192 g per LSB and 2000 deg/s for every non-HMD device; matches the ~4096 LSB we read at rest).
// Factory calibration from the device JSON "imu" block: (raw * scale - bias) * scale_factor, IMU frame.
#include "tracking/math/Linalg.h"
#include "tracking/model/DeviceModel.h"

#include <cmath>
#include <cstdint>
#include <deque>

namespace lhp::tracking::filter {

constexpr double kAccPerLsb = 8.0 / 32768.0 * 9.80665;               // m/s^2
constexpr double kGyroPerLsb = 2000.0 / 32768.0 * math::kPi / 180.0;  // rad/s
constexpr int kImuSaturation = 32000;

struct ImuSi {
    math::Vec3 acc;      // m/s^2, IMU frame, factory calibrated
    math::Vec3 gyro;     // rad/s
    bool accSaturated = false, gyroSaturated = false;
};

inline ImuSi ImuToSi(const int16_t acc[3], const int16_t gyro[3], const model::ImuCalibration* cal) {
    ImuSi s;
    math::Vec3 a{acc[0] * kAccPerLsb, acc[1] * kAccPerLsb, acc[2] * kAccPerLsb};
    math::Vec3 g{gyro[0] * kGyroPerLsb, gyro[1] * kGyroPerLsb, gyro[2] * kGyroPerLsb};
    if (cal) {
        a = {(a.x - cal->accBias.x) * cal->accScale.x, (a.y - cal->accBias.y) * cal->accScale.y, (a.z - cal->accBias.z) * cal->accScale.z};
        g = {(g.x - cal->gyroBias.x) * cal->gyroScale.x, (g.y - cal->gyroBias.y) * cal->gyroScale.y, (g.z - cal->gyroBias.z) * cal->gyroScale.z};
    }
    for (int k = 0; k < 3; ++k) {
        s.accSaturated |= std::abs(acc[k]) >= kImuSaturation;
        s.gyroSaturated |= std::abs(gyro[k]) >= kImuSaturation;
    }
    s.acc = a;
    s.gyro = g;
    return s;
}

/// "Still" = gyro and accelerometer flat over a short window (libsurvive uses light activations for the
/// same purpose; the IMU reacts faster and works without light).
class StillnessDetector {
public:
    struct Params {
        double window = 0.3;          // seconds of history
        double minDuration = 0.25;    // must be flat at least this long
        double gyroDev = 0.02;        // rad/s, max deviation from the window mean
        double gyroMean = 0.08;       // rad/s, |mean| (allows a residual gyro bias)
        double accStd = 0.06;         // m/s^2, per-axis standard deviation
    };
    StillnessDetector() = default;
    explicit StillnessDetector(Params p) : p_(p) {}

    void Add(double t, const math::Vec3& acc, const math::Vec3& gyro) {
        q_.push_back({t, acc, gyro});
        while (!q_.empty() && t - q_.front().t > p_.window) q_.pop_front();
        bool flat = Evaluate();
        if (!flat) stillSince_ = -1;
        else if (stillSince_ < 0) stillSince_ = t;
        still_ = flat && stillSince_ >= 0 && t - stillSince_ >= p_.minDuration - p_.window;
        lastT_ = t;
    }
    void Reset() {
        q_.clear();
        still_ = false;
        stillSince_ = -1;
    }
    bool Still() const { return still_; }
    double StillSeconds() const { return still_ ? lastT_ - stillSince_ + p_.window : 0.0; }
    const math::Vec3& MeanAcc() const { return meanAcc_; }
    const math::Vec3& MeanGyro() const { return meanGyro_; }
    double AccStd() const { return accStd_; }
    double GyroDev() const { return gyroDev_; }

private:
    bool Evaluate() {
        if (q_.size() < 8 || q_.back().t - q_.front().t < p_.window * 0.7) return false;
        math::Vec3 ma, mg;
        for (auto& s : q_) {
            ma += s.acc;
            mg += s.gyro;
        }
        ma = ma / (double)q_.size();
        mg = mg / (double)q_.size();
        double va[3] = {}, dev = 0;
        for (auto& s : q_) {
            for (int k = 0; k < 3; ++k) va[k] += (s.acc[k] - ma[k]) * (s.acc[k] - ma[k]);
            dev = std::max(dev, math::Norm(s.gyro - mg));
        }
        double sd = 0;
        for (double v : va) sd = std::max(sd, std::sqrt(v / q_.size()));
        meanAcc_ = ma;
        meanGyro_ = mg;
        accStd_ = sd;
        gyroDev_ = dev;
        return sd < p_.accStd && dev < p_.gyroDev && math::Norm(mg) < p_.gyroMean;
    }
    struct S {
        double t;
        math::Vec3 acc, gyro;
    };
    Params p_;
    std::deque<S> q_;
    bool still_ = false;
    double stillSince_ = -1, lastT_ = 0;
    math::Vec3 meanAcc_, meanGyro_;
    double accStd_ = 0, gyroDev_ = 0;
};

}  // namespace lhp::tracking::filter
