#include "tracking/solver/Scene.h"

#include <cmath>

namespace lhp::tracking::solver {

using math::Pose;
using math::Quat;
using math::Vec3;

void Scene::MarkSeen(int ch) {
    BaseState& b = bases_[ch & 15];
    b.seen = true;
    if (ref_ < 0) {
        ref_ = ch & 15;
        b.isReference = true;
        b.hasPose = true;
        b.refFromBase = Pose{};
    }
}

void Scene::SetCalibration(int ch, const lighthouse::BaseCalibration& c, bool fromCache) {
    MarkSeen(ch);
    BaseState& b = bases_[ch & 15];
    bool changed = !b.cal || b.cal->raw != c.raw;
    b.cal = c;
    b.calFromCache = fromCache;
    if (!changed) return;
    // Bootstrap samples were computed with the old calibration: start over. A new reference calibration
    // biases every device pose that fed the other bases, so those restart as well.
    for (auto& o : bases_) {
        if (o.isReference) continue;
        if (&o == &b || b.isReference) {
            o.hasPose = false;
            o.samples = 0;
        }
    }
}

bool Scene::ReadyToBootstrap(int ch) const {
    // both the reference and this base need their real fcal (ideal-model angles would bias the result)
    return ref_ >= 0 && bases_[ref_].cal && bases_[ch & 15].cal && !bases_[ch & 15].isReference;
}

void Scene::Reset() {
    bases_ = {};
    ref_ = -1;
}

bool Scene::GravityAligned() const {
    if (ref_ < 0 || !bases_[ref_].cal) return false;
    const auto& a = bases_[ref_].cal->accelDir;
    return a[0] || a[1] || a[2];
}

Pose Scene::WorldFromRef() const {
    Pose w;
    if (GravityAligned()) {
        const auto& a = bases_[ref_].cal->accelDir;
        Vec3 upInBase = math::Normalize({(double)a[0], (double)a[1], (double)a[2]});
        Quat level = Quat::FromTo(upInBase, {0, 1, 0});
        // yaw: the base's forward (-Z) projected on the floor points to world -Z
        Vec3 fwd = level.Rotate({0, 0, -1});
        fwd.y = 0;
        if (math::Norm(fwd) > 1e-6) {
            fwd = math::Normalize(fwd);
            double yaw = std::atan2(-fwd.x, -fwd.z);   // angle from -Z to fwd about +Y
            Quat unyaw = Quat::FromRotVec({0, -yaw, 0});
            level = unyaw * level;
        }
        w.rot = level.Normalized();
    }
    w.pos = {0, refHeight_, 0};
    return w;
}

void Scene::AddBootstrapSample(int ch, const Pose& refFromDevice, const Pose& baseFromDevice) {
    BaseState& b = bases_[ch & 15];
    if (b.isReference) return;
    Pose sample = refFromDevice * baseFromDevice.Inverse();
    if (!b.hasPose) {
        b.refFromBase = sample;
        b.hasPose = true;
        b.samples = 1;
        b.lastSpreadMm = 0;
        return;
    }
    // running average (position mean, quaternion nlerp), capped so it keeps adapting slowly
    b.lastSpreadMm = math::Norm(sample.pos - b.refFromBase.pos) * 1000;
    if (b.samples >= 20 && b.lastSpreadMm > 50) {   // outlier against a settled estimate
        ++b.rejectedSamples;
        return;
    }
    double w = 1.0 / std::min(b.samples + 1, 200);
    b.refFromBase.pos = b.refFromBase.pos * (1 - w) + sample.pos * w;
    Quat q = sample.rot;
    const Quat& r = b.refFromBase.rot;
    if (q.w * r.w + q.x * r.x + q.y * r.y + q.z * r.z < 0) q = {-q.w, -q.x, -q.y, -q.z};
    b.refFromBase.rot = Quat{r.w * (1 - w) + q.w * w, r.x * (1 - w) + q.x * w, r.y * (1 - w) + q.y * w, r.z * (1 - w) + q.z * w}.Normalized();
    ++b.samples;
}

}  // namespace lhp::tracking::solver
