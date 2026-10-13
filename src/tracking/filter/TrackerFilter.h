#pragma once
// L3: per-device tracking filter -- the counterpart of SteamVR's "LighthouseDirect" tracker.
//
// What SteamVR does is not public, but its lighthouse driver names the pieces (driver_lighthouse strings):
// a per-device estimator fed by residual functors CSensorPositionAngular (one LH2 sweep angle),
// CImuAccel, CImuGyro and CQuaternionNormalizer, with a separate CLighthousePoseSolver for bootstrap and an
// IMU-only fallback tracker. This is the same structure, built as an error-state iterated Kalman filter
// (each optical update is a small Gauss-Newton problem with the filter prior, i.e. what a Ceres step with a
// prior term computes). State layout and noise model follow libsurvive's survive_kalman_tracker (MIT):
//
//   nominal: position p, orientation q (world_from_device), velocity v, angular velocity w (world),
//            linear acceleration a (world, gravity excluded), gyro bias bg, accel bias ba (IMU frame)
//   error  : [dp dtheta dv dw da dbg dba] (21), dtheta is a world-frame rotation vector (q = Exp(dtheta) * q)
//
// Measurements
//   IMU   : gyro = C*w + bg,  accel = C*(a + g_up) + ba,  C = (R_world_device * R_device_imu)^T
//   light : one angle per (sensor, plane, base); the sensor position at the hit time is extrapolated from
//           the filter time with (v, w, a), so a whole base rotation (~20 ms) is fused without motion blur
//   ZVU   : zero velocity / acceleration while the IMU says the device is still (libsurvive "zvu")
//
// Pure computation: no threads, no I/O. Times are seconds on the device clock.
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/math/Linalg.h"
#include "tracking/model/DeviceModel.h"

#include <array>
#include <string>
#include <vector>

namespace lhp::tracking::filter {

constexpr int kStateDim = 21;
enum StateIndex : int { kP = 0, kTheta = 3, kV = 6, kW = 9, kA = 12, kBg = 15, kBa = 18 };
using Covariance = std::array<double, kStateDim * kStateDim>;

constexpr double kGravity = 9.80665;

struct FilterParams {
    // ---- process noise (continuous-time spectral densities)
    double jerkPsd = 4e5;          // (m/s^3)^2 s; libsurvive process-weight-jerk = 1874161
    double angAccPsd = 60;         // (rad/s^2)^2 s; libsurvive process-weight-ang-vel = 60
    double gyroBiasRw = 1e-9;      // rad^2/s^3 (bias random walk)
    double accBiasRw = 1e-7;       // (m/s^2)^2/s
    double maxPredictDt = 0.05;    // longer gaps are integrated in steps of this size
    // ---- measurement noise
    double gyroVar = 3.05e-5;      // (rad/s)^2; libsurvive imu-gyro-variance = 0.0000304617
    double accVar = 0.12;          // (m/s^2)^2; libsurvive imu-acc-variance = 1e-3 g^2
    double lightSigma = 1.5e-4;    // rad, one sweep angle (static residuals on real hardware are ~6e-5)
    double huberK = 2.0;           // residuals beyond k sigma are down-weighted (IRLS)
    double gateSigma = 5.0;        // innovation gate, in innovation standard deviations
    double gateSigmaConverging = 12.0;
    double convergeSeconds = 0.6;  // looser gate right after (re)initialisation
    // One update = one base rotation. A handful of angles from one plane can be explained by a wrong pose and
    // would drag the state away (then the gate rejects exactly the angles that would correct it):
    int minUsed = 5;                 // libsurvive kalman-lightcap-minimum-sensors = 5
    double maxGatedFraction = 0.25;  // more rejected than this: the batch disagrees with the state as a whole
                                     // -> nothing is applied (the caller decides whether the filter is lost)
    double zvuStationaryVar = 1e-5;   // libsurvive kalman-zvu-stationary
    double zvuNoLightVar = 1e-4;      // libsurvive kalman-zvu-no-light
    int maxIterations = 4;
    // ---- initial uncertainty
    double initVelVar = 0.25, initAngVelVar = 0.05, initAccVar = 4.0;
    double initGyroBiasVar = 1e-4, initAccBiasVar = 0.04;
    double minPosVar = 1e-6, minRotVar = 1e-6;
};

/// One sweep angle. `t` is the hit time (device clock, seconds).
struct LightMeas {
    int base = 0;                // index into the LightBase array
    int sensor = 0;              // model index
    int plane = 0;
    double angle = 0;            // radians, Lh2Observation::angle
    double t = 0;
};

struct LightBase {
    math::Pose worldFromBase;
    lighthouse::AxisCal cal[2];
    double sigmaScale = 1;       // > 1 while the base has no OOTX calibration yet (ideal model)
};

struct LightUpdate {
    bool applied = false;
    bool inconsistent = false;   // the batch as a whole disagrees with the state (filter probably wrong)
    std::string note;            // why not applied
    int n = 0, used = 0, gated = 0, facingAway = 0, iterations = 0;
    double rmsBeforeRad = 0, rmsAfterRad = 0;   // over used measurements
    double nis = 0;              // normalised innovation squared per used measurement (~1 when consistent)
    double spanMs = 0;           // time span of the batch
    double lagMs = 0;            // filter time - newest hit (how far the IMU ran ahead)
    std::vector<double> residuals;     // per measurement after the update (rad); NaN when not evaluated
    std::vector<char> rejected;        // gated / facing away
};

struct ImuUpdate {
    math::Vec3 accResidual, gyroResidual;   // measured - predicted (IMU frame)
    double nis = 0;
    bool accUsed = false, gyroUsed = false;
};

class TrackerFilter {
public:
    explicit TrackerFilter(const model::DeviceModel& model, FilterParams params = {});

    bool Initialized() const { return init_; }
    void Reset();
    /// Start from a bootstrap pose (batch solve) with its 6x6 covariance [rx ry rz tx ty tz].
    void Initialize(double t, const math::Pose& worldFromDevice, const std::array<double, 36>& poseCov, const math::Vec3& angVelWorld,
                    const math::Vec3& gyroBias = {}, const math::Vec3& velocity = {});
    /// Recovery: jump to an independently solved pose, keeping the motion and IMU bias estimates.
    void Reanchor(double t, const math::Pose& worldFromDevice, const std::array<double, 36>& poseCov);

    double Time() const { return t_; }
    double SecondsSinceInit() const { return t_ - tInit_; }
    void Predict(double t);
    ImuUpdate UpdateImu(double t, const math::Vec3& accImu, const math::Vec3& gyroImu, bool useAcc = true, bool useGyro = true);
    LightUpdate UpdateLight(const std::vector<LightMeas>& meas, const std::vector<LightBase>& bases);
    /// Pseudo-measurement v = 0, a = 0 (and w = 0 when `angular`).
    void UpdateZeroVelocity(double var, bool angular);
    /// Loosen the pose (e.g. a base calibration just changed): adds variance and re-enters the converging phase.
    void Inflate(double posVar, double rotVar);
    /// Re-express the state in a new world frame (world re-levelled, floor moved ...): x_new = newFromOld * x_old.
    void TransformWorld(const math::Pose& newFromOld);

    // state
    const math::Pose& WorldFromDevice() const { return pose_; }
    math::Pose PoseAt(double t) const;   // short extrapolation (display)
    const math::Vec3& Velocity() const { return v_; }
    const math::Vec3& AngularVelocity() const { return w_; }
    const math::Vec3& Acceleration() const { return a_; }
    const math::Vec3& GyroBias() const { return bg_; }
    const math::Vec3& AccBias() const { return ba_; }
    const Covariance& P() const { return P_; }
    double Var(int i) const { return P_[i * kStateDim + i]; }
    double PositionSigma() const;   // sqrt(trace) metres
    double RotationSigma() const;   // sqrt(trace) radians
    /// Pose block [rx ry rz tx ty tz] (same layout as the batch solver).
    std::array<double, 36> PoseCovariance() const;
    bool Healthy() const;           // finite and not exploded
    const FilterParams& Params() const { return prm_; }
    FilterParams& MutableParams() { return prm_; }

    /// Predicted angle (and whether the sensor faces the base) for a pose; shared with diagnostics.
    double Predict(const LightMeas& m, const LightBase& b, const math::Pose& worldFromDevice, double* facing = nullptr) const;

private:
    struct Nominal {
        math::Pose pose;
        math::Vec3 v, w, a, bg, ba;
    };
    Nominal Get() const { return {pose_, v_, w_, a_, bg_, ba_}; }
    void Set(const Nominal& n);
    static Nominal Plus(const Nominal& x, const double* d);
    static void Minus(const Nominal& x, const Nominal& x0, double* d);
    void PredictStep(double dt);
    void Symmetrize();

    model::DeviceModel model_;
    FilterParams prm_;
    math::Quat imuToDevice_;
    bool init_ = false;
    double t_ = 0, tInit_ = 0;
    math::Pose pose_;
    math::Vec3 v_, w_, a_, bg_, ba_;
    Covariance P_{};
};

}  // namespace lhp::tracking::filter
