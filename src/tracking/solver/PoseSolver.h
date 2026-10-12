#pragma once
// L3: device pose from LH2 sweep angles (Levenberg-Marquardt, robust Huber weights, numeric Jacobian).
// Pure function of its inputs; reports every intermediate quantity for the inspector.
#include "tracking/lighthouse/BaseStation.h"
#include "tracking/math/Linalg.h"
#include "tracking/model/DeviceModel.h"

#include <array>
#include <string>
#include <vector>

namespace lhp::tracking::solver {

struct Measurement {
    int base = 0;                // index into the BaseView array
    int sensor = 0;              // model index
    int plane = 0;               // 0 / 1
    double angle = 0;            // radians (Lh2Observation::angle)
    double dt = 0;               // seconds relative to the solve's reference time (<= 0: earlier)
};

struct BaseView {
    math::Pose worldFromBase;
    lighthouse::AxisCal cal[2];  // zero = ideal model (no OOTX yet)
};

struct SolverOptions {
    int maxIterations = 30;
    double huberRad = 1.5e-3;          // residuals above this get down-weighted
    double outlierRad = 2e-2;          // residuals above this are rejected after convergence
    bool seedSearch = true;            // try several orientations when no initial pose is given
    bool estimateVelocity = true;      // also solve linear/angular velocity (removes motion blur of the observation window)
    math::Vec3 velocityPrior, angularVelocityPrior;   // e.g. the previous solve's velocities (weak prior + start value)
};

struct SolveResult {
    bool ok = false;
    std::string failReason;
    math::Pose worldFromDevice;        // at the reference time (dt = 0)
    bool velocityEstimated = false;
    math::Vec3 velocity, angularVelocity;   // world frame, m/s and rad/s
    int iterations = 0;
    int seedsTried = 0;
    double initialRmsRad = 0;
    double rmsRad = 0;
    double maxAbsRad = 0;
    int used = 0, rejected = 0;
    std::vector<double> residuals;     // per input measurement (rad); NaN for rejected
    std::vector<char> rejectedMask;
    std::array<double, 36> covariance{};   // pose block [rx ry rz tx ty tz], world frame, rad^2 / m^2
    double positionSigma = 0;          // sqrt(trace of position block), metres
    double rotationSigma = 0;          // sqrt(trace of rotation block), radians
};

SolveResult SolvePose(const model::DeviceModel& model, const std::vector<Measurement>& meas, const std::vector<BaseView>& bases,
                      const math::Pose* initial, const SolverOptions& opt = {});

/// Bearing (unit vector in the base frame) to a point whose two plane angles are known, using the
/// ideal model. Used to seed the solver and to draw rays.
math::Vec3 BearingFromAngles(double angle0, double angle1);

/// Predicted angle for one measurement given poses (exposed for residual displays).
double PredictAngle(const model::DeviceModel& model, const Measurement& m, const BaseView& base, const math::Pose& worldFromDevice);

}  // namespace lhp::tracking::solver
