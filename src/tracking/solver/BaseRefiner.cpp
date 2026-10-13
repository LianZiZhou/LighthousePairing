#include "tracking/solver/BaseRefiner.h"

#include "tracking/solver/PoseSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lhp::tracking::solver {

using math::Pose;
using math::Vec3;

namespace {

constexpr double kRadToDeg = 180.0 / math::kPi;
constexpr double kMaxRmsRad = 1.5e-3;   // a good fit over all keyframes (calibrated base: ~0.01-0.05 deg)

double AngleDeg(const math::Quat& a, const math::Quat& b) { return math::Norm((a * b.Conj()).ToRotVec()) * kRadToDeg; }

}  // namespace

bool BaseRefiner::Wants(const Pose& refFromDevice) const {
    for (const auto& k : keyframes_)
        if (math::Norm(k.refFromDevice.pos - refFromDevice.pos) < kMinSpacingM && AngleDeg(k.refFromDevice.rot, refFromDevice.rot) < kMinSpacingDeg)
            return false;
    return true;
}

void BaseRefiner::Add(RefineKeyframe k) {
    if (k.obs.empty()) return;
    if ((int)keyframes_.size() >= kMaxKeyframes) {
        // full: drop the most redundant keyframe (the one closest to its nearest neighbour)
        size_t worst = 0;
        double worstD = std::numeric_limits<double>::max();
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            double nearest = std::numeric_limits<double>::max();
            for (size_t j = 0; j < keyframes_.size(); ++j)
                if (i != j) nearest = std::min(nearest, math::Norm(keyframes_[i].refFromDevice.pos - keyframes_[j].refFromDevice.pos));
            if (nearest < worstD) {
                worstD = nearest;
                worst = i;
            }
        }
        keyframes_.erase(keyframes_.begin() + (std::ptrdiff_t)worst);
    }
    keyframes_.push_back(std::move(k));
    ++added_;
}

double BaseRefiner::ExtentM() const {
    if (keyframes_.empty()) return 0;
    Vec3 lo = keyframes_[0].refFromDevice.pos, hi = lo;
    for (const auto& k : keyframes_)
        for (int i = 0; i < 3; ++i) {
            lo[i] = std::min(lo[i], k.refFromDevice.pos[i]);
            hi[i] = std::max(hi[i], k.refFromDevice.pos[i]);
        }
    return math::Norm(hi - lo);
}

RefineResult BaseRefiner::Solve(const lighthouse::AxisCal cal[2], const Pose& seedRefFromBase) {
    added_ = 0;
    RefineResult out;
    out.keyframes = Count();
    if (out.keyframes < kMinKeyframes) {
        out.note = "位置样本不足";
        return out;
    }
    if (ExtentM() < kMinExtentM) {
        out.note = "样本位置分布范围太小";
        return out;
    }
    // All keyframes' sensors as one rigid point set in the reference frame; its pose in the base frame
    // (baseFromRef) is what SolvePose estimates when the base sits at the origin.
    model::DeviceModel cloud;
    std::vector<Measurement> meas;
    for (const auto& k : keyframes_)
        for (const auto& o : k.obs) {
            meas.push_back({0, (int)cloud.points.size(), o.plane, o.angle, 0});
            cloud.points.push_back(o.pointRef);
            cloud.normals.push_back({});
        }
    out.measurements = (int)meas.size();
    BaseView view;
    view.cal[0] = cal[0];
    view.cal[1] = cal[1];
    const Pose seed = seedRefFromBase.Inverse();
    double s2 = 0;
    int n = 0;
    for (const auto& m : meas) {
        double r = std::remainder(m.angle - PredictAngle(cloud, m, view, seed), 2 * math::kPi);
        if (std::isfinite(r)) {
            s2 += r * r;
            ++n;
        }
    }
    out.rmsBeforeRad = n ? std::sqrt(s2 / n) : 0;

    SolverOptions opt;
    opt.seedSearch = false;
    opt.estimateVelocity = false;
    opt.maxIterations = 50;
    SolveResult res = SolvePose(cloud, meas, {view}, &seed, opt);
    out.used = res.used;
    out.rmsAfterRad = res.rmsRad;
    if (!res.ok) {
        out.note = "求解失败: " + res.failReason;
        return out;
    }
    if (res.used < 0.7 * out.measurements || res.rmsRad > kMaxRmsRad) {
        out.note = "拟合不够好";
        return out;
    }
    out.refFromBase = res.worldFromDevice.Inverse();
    out.movedMm = math::Norm(out.refFromBase.pos - seedRefFromBase.pos) * 1000;
    out.rotatedDeg = AngleDeg(out.refFromBase.rot, seedRefFromBase.rot);
    out.ok = res.rmsRad <= out.rmsBeforeRad;
    if (!out.ok) out.note = "没有比现有位姿更好";
    return out;
}

}  // namespace lhp::tracking::solver
