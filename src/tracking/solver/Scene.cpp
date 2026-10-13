#include "tracking/solver/Scene.h"

#include <algorithm>
#include <cmath>

namespace lhp::tracking::solver {

using math::Pose;
using math::Quat;
using math::Vec3;

namespace {
constexpr double kBaseUpWeight = 30;      // the base accelerometer counts like this many device samples
constexpr int kMaxUpSamples = 600;        // beyond this the average keeps adapting slowly
constexpr double kUpApplyDeg = 0.02;      // smaller tilt changes are not applied (avoid churning the world)
constexpr double kOutlierMm = 50;
constexpr int kMovedAfterRejects = 40;    // this many consecutive outliers: the base was moved

double AngleDeg(const Vec3& a, const Vec3& b) {
    return std::acos(std::clamp(math::Dot(math::Normalize(a), math::Normalize(b)), -1.0, 1.0)) * 180 / math::kPi;
}
}  // namespace

const char* ToString(PoseSource s) {
    switch (s) {
        case PoseSource::Reference: return "参考基站（坐标原点）";
        case PoseSource::Bootstrap: return "由设备观测推算";
        case PoseSource::SavedUniverse: return "上次保存的空间";
        case PoseSource::SteamVrUniverse: return "SteamVR 房间设置（lighthousedb）";
        case PoseSource::Refined: return "多位置精化";
        default: return "未知";
    }
}

void Scene::MarkSeen(int ch) {
    BaseState& b = bases_[ch & 15];
    b.seen = true;
    if (ref_ < 0) {
        ref_ = ch & 15;
        b.isReference = true;
        b.hasPose = true;
        b.source = PoseSource::Reference;
        b.refFromBase = Pose{};
        ++version_;
    }
}

void Scene::SetCalibration(int ch, const lighthouse::BaseCalibration& c, bool fromCache) {
    MarkSeen(ch);
    BaseState& b = bases_[ch & 15];
    bool changed = !b.cal || b.cal->raw != c.raw;
    bool idChanged = !b.cal || b.cal->id != c.id;
    b.cal = c;
    b.calFromCache = fromCache;
    if (!changed) return;
    // Bootstrap samples were computed with the old calibration: start over. A new reference calibration
    // biases every device pose that fed the other bases, so those restart as well.
    for (auto& o : bases_) {
        if (o.isReference) continue;
        if (&o == &b || b.isReference) {
            if (o.fixed && !idChanged && &o != &b) continue;   // a fixed universe pose does not depend on our data
            o.altRefFromBase.reset();
            o.hasPose = false;
            o.fixed = false;
            o.source = PoseSource::None;
            o.samples = 0;
            ClearRefiner((int)(&o - bases_.data()));
        }
    }
    if (b.isReference) {
        upSum_ = Vec3{};
        upCount_ = 0;
        UpdateUp();
    }
    TryApplyUniverses();
}

bool Scene::ReadyToBootstrap(int ch) const {
    // both the reference and this base need their real fcal (ideal-model angles would bias the result)
    const BaseState& b = bases_[ch & 15];
    return ref_ >= 0 && bases_[ref_].cal && b.cal && !b.isReference && (b.fixed || b.samples < 300);
}

void Scene::Reset() {
    bases_ = {};
    for (int ch = 0; ch < lighthouse::kMaxChannels; ++ch) ClearRefiner(ch);
    ref_ = -1;
    upSum_ = Vec3{};
    upCount_ = 0;
    appliedUp_ = Vec3{};
    universes_.clear();
    universeNote_.clear();
    ++version_;
}

void Scene::SetReferenceHeight(double h) {
    if (h == refHeight_) return;
    refHeight_ = h;
    ++version_;
}

void Scene::SetFloorAt(double worldY) {
    refHeight_ -= worldY;
    ++version_;
}

Vec3 Scene::BaseAccelUp() const {
    if (ref_ < 0 || !bases_[ref_].cal) return {};
    const auto& a = bases_[ref_].cal->accelDir;
    if (!(a[0] || a[1] || a[2])) return {};
    return math::Normalize({(double)a[0], (double)a[1], (double)a[2]});
}

void Scene::UpdateUp() {
    Vec3 base = BaseAccelUp();
    Vec3 sum = base * kBaseUpWeight;
    if (useDeviceTilt_) sum += upSum_;
    if (math::Norm(sum) < 1e-9) {
        if (math::Norm(appliedUp_) > 0) ++version_;
        appliedUp_ = Vec3{};
        return;
    }
    Vec3 up = math::Normalize(sum);
    if (math::Norm(appliedUp_) == 0 || AngleDeg(up, appliedUp_) > kUpApplyDeg) {
        appliedUp_ = up;
        ++version_;
    }
}

void Scene::AddUpSample(const Vec3& upInRef) {
    if (!useDeviceTilt_ || ref_ < 0 || math::Norm(upInRef) < 0.5) return;
    Vec3 u = math::Normalize(upInRef);
    if (math::Norm(appliedUp_) > 0 && AngleDeg(u, appliedUp_) > 10) return;   // not gravity (moving device / bad pose)
    upSum_ += u;
    ++upCount_;
    if (upCount_ > kMaxUpSamples) {
        upSum_ = upSum_ * ((double)kMaxUpSamples / upCount_);
        upCount_ = kMaxUpSamples;
    }
    UpdateUp();
}

void Scene::SetUseDeviceTilt(bool on) {
    if (on == useDeviceTilt_) return;
    useDeviceTilt_ = on;
    UpdateUp();
}

double Scene::TiltCorrectionDeg() const {
    Vec3 b = BaseAccelUp();
    if (math::Norm(b) == 0 || math::Norm(appliedUp_) == 0) return 0;
    return AngleDeg(b, appliedUp_);
}

bool Scene::GravityAligned() const { return math::Norm(appliedUp_) > 0; }

Pose Scene::WorldFromRef() const {
    Pose w;
    if (GravityAligned()) {
        Quat level = Quat::FromTo(appliedUp_, {0, 1, 0});
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
    if (!b.hasPose && b.altRefFromBase) {
        // external pose with an undocumented direction: let independent observations choose the reading
        double dP = math::Norm(sample.pos - b.refFromBase.pos), dA = math::Norm(sample.pos - b.altRefFromBase->pos);
        b.lastSpreadMm = std::min(dP, dA) * 1000;
        if (dP < 0.1 && dP <= dA) ++b.votesPrimary;
        else if (dA < 0.1) ++b.votesAlt;
        else ++b.votesNone;
        if (b.votesAlt >= 5 && b.votesAlt > 2 * b.votesPrimary) {
            b.refFromBase = *b.altRefFromBase;
            b.altRefFromBase.reset();
            b.hasPose = true;
            universeNote_ = "已用实测确认 SteamVR 基站位姿（反向解读）";
        } else if (b.votesPrimary >= 5 && b.votesPrimary > 2 * b.votesAlt) {
            b.altRefFromBase.reset();
            b.hasPose = true;
            universeNote_ = "已用实测确认 SteamVR 基站位姿";
        } else if (b.votesNone >= 10 && b.votesPrimary + b.votesAlt < 3) {
            b.altRefFromBase.reset();
            b.fixed = false;
            b.refFromBase = sample;
            b.hasPose = true;
            b.source = PoseSource::Bootstrap;
            b.samples = 1;
            ClearRefiner(ch);
            universeNote_ = "SteamVR 记录的基站位置与实测不符，已改为自行推算";
        }
        return;
    }
    if (!b.hasPose) {
        b.refFromBase = sample;
        b.hasPose = true;
        b.source = PoseSource::Bootstrap;
        b.samples = 1;
        b.lastSpreadMm = 0;
        b.rejectedSamples = 0;
        ClearRefiner(ch);
        return;
    }
    b.lastSpreadMm = math::Norm(sample.pos - b.refFromBase.pos) * 1000;
    bool outlier = (b.fixed || b.samples >= 20) && b.lastSpreadMm > kOutlierMm;
    if (outlier) {
        ++b.rejectedSamples;
        if (b.rejectedSamples >= kMovedAfterRejects) {
            // consistently somewhere else: the base was moved (or the remembered universe is stale)
            b.refFromBase = sample;
            b.fixed = false;
            b.source = PoseSource::Bootstrap;
            b.samples = 1;
            b.rejectedSamples = 0;
            ClearRefiner(ch);
            universeNote_ = "基站位置与记录不符，已重新推算";
        }
        return;
    }
    b.rejectedSamples = 0;
    if (b.fixed) return;   // SteamVR's universe is taken as is; samples only check it
    // running average (position mean, quaternion nlerp), capped so it keeps adapting slowly
    double w = 1.0 / std::min(b.samples + 1, 200);
    b.refFromBase.pos = b.refFromBase.pos * (1 - w) + sample.pos * w;
    Quat q = sample.rot;
    const Quat& r = b.refFromBase.rot;
    if (q.w * r.w + q.x * r.x + q.y * r.y + q.z * r.z < 0) q = {-q.w, -q.x, -q.y, -q.z};
    b.refFromBase.rot = Quat{r.w * (1 - w) + q.w * w, r.x * (1 - w) + q.x * w, r.y * (1 - w) + q.y * w, r.z * (1 - w) + q.z * w}.Normalized();
    ++b.samples;
}

void Scene::SetKnownUniverses(std::vector<UniverseRecord> u) {
    universes_ = std::move(u);
    TryApplyUniverses();
}

void Scene::TryApplyUniverses() {
    if (ref_ < 0 || !bases_[ref_].cal || universes_.empty()) return;
    const uint32_t refId = bases_[ref_].cal->id;
    for (const auto& u : universes_) {
        // where is our reference base in this universe? (both readings for formats with an ambiguous direction)
        std::optional<Pose> uRef, uRefAlt;
        if (u.referenceId == refId) uRef = uRefAlt = Pose{};
        for (auto& e : u.bases)
            if (e.id == refId) {
                uRef = e.refFromBase;
                uRefAlt = e.alternative ? *e.alternative : e.refFromBase;
            }
        if (!uRef) continue;
        Pose refFromU = uRef->Inverse();
        Pose refFromUAlt = uRefAlt->Inverse();
        int applied = 0;
        for (int ch = 0; ch < lighthouse::kMaxChannels; ++ch) {
            BaseState& b = bases_[ch];
            if (!b.seen || b.isReference || !b.cal) continue;
            if (b.altRefFromBase) continue;   // candidate already waiting for verification
            if (b.hasPose && !(b.source == PoseSource::Bootstrap && b.samples < 20)) continue;
            Pose uBase, uBaseAlt;
            bool found = false, ambiguous = false;
            if (b.cal->id == u.referenceId) {
                uBase = uBaseAlt = Pose{};
                found = true;
            }
            for (auto& e : u.bases)
                if (e.id == b.cal->id) {
                    uBase = e.refFromBase;
                    uBaseAlt = e.alternative ? *e.alternative : e.refFromBase;
                    ambiguous = e.alternative.has_value();
                    found = true;
                }
            for (auto& e : u.bases)
                if (e.id == refId && e.alternative) ambiguous = true;
            if (!found) continue;
            b.refFromBase = refFromU * uBase;
            b.votesPrimary = b.votesAlt = b.votesNone = 0;
            if (ambiguous) {
                // not trusted until live observations pick the reading (see AddBootstrapSample)
                b.altRefFromBase = refFromUAlt * uBaseAlt;
                b.hasPose = false;
            } else {
                b.altRefFromBase.reset();
                b.hasPose = true;
            }
            b.fixed = u.fixed;
            b.source = u.source == "steamvr" ? PoseSource::SteamVrUniverse : PoseSource::SavedUniverse;
            int evidence = 30;
            for (auto& e : u.bases)
                if (e.id == b.cal->id) evidence = std::max(evidence, e.samples);
            b.samples = u.fixed ? 0 : std::min(evidence, 200);
            b.rejectedSamples = 0;
            ClearRefiner(ch);
            ++applied;
        }
        if (u.hasUp && upCount_ == 0 && u.upSamples > 0) {
            upSum_ = refFromU.rot.Rotate(math::Normalize(u.upInRef)) * (double)std::min(u.upSamples, kMaxUpSamples);
            upCount_ = std::min(u.upSamples, kMaxUpSamples);
            UpdateUp();
        }
        if (applied)
            universeNote_ = std::string("已从") + (u.source == "steamvr" ? " SteamVR 房间设置" : "上次保存的空间") + "恢复 " + std::to_string(applied) + " 个基站位置" +
                            (u.source == "steamvr" ? "（等待实测确认位姿方向）" : "");
    }
}

void Scene::ClearRefiner(int ch) {
    refiners_[ch & 15].Clear();
    ++refineGen_[ch & 15];
    bases_[ch & 15].refineKeyframes = 0;
}

bool Scene::WantsRefineKeyframe(int ch, const Pose& refFromDevice) const {
    const BaseState& b = bases_[ch & 15];
    // a fixed (SteamVR) universe is taken as is; the reference base is the frame itself
    if (ref_ < 0 || !bases_[ref_].cal || b.isReference || !b.cal || b.fixed || !HasPose(ch)) return false;
    return refiners_[ch & 15].Wants(refFromDevice);
}

void Scene::AddRefineKeyframe(int ch, RefineKeyframe k) {
    refiners_[ch & 15].Add(std::move(k));
    bases_[ch & 15].refineKeyframes = refiners_[ch & 15].Count();
}

std::optional<RefineJob> Scene::TakeRefineJob(int ch) {
    BaseRefiner& r = refiners_[ch & 15];
    const BaseState& b = bases_[ch & 15];
    // solve again once a few new viewpoints arrived (the first time: as soon as there are enough)
    if (!b.cal || r.Count() < BaseRefiner::kMinKeyframes || r.Added() < 4) return std::nullopt;
    RefineJob j;
    j.channel = ch & 15;
    j.generation = refineGen_[ch & 15];
    j.refiner = r;
    j.cal[0] = b.cal->axis[0];
    j.cal[1] = b.cal->axis[1];
    j.seed = b.refFromBase;
    r.ResetAdded();
    return j;
}

bool Scene::ApplyRefineResult(const RefineJob& job, const RefineResult& res) {
    BaseState& b = bases_[job.channel];
    if (job.generation != refineGen_[job.channel]) return false;   // the base restarted meanwhile
    b.lastRefine = res;
    if (!res.ok) return false;
    b.refFromBase = res.refFromBase;
    b.source = PoseSource::Refined;
    b.samples = std::max(b.samples, 300);   // done bootstrapping: single-base averages would only drag it back
    ++b.refinements;
    return true;
}

UniverseRecord Scene::ExportUniverse() const {
    UniverseRecord u;
    u.source = "saved";
    if (ref_ < 0 || !bases_[ref_].cal) return u;
    u.referenceId = bases_[ref_].cal->id;
    for (int ch = 0; ch < lighthouse::kMaxChannels; ++ch) {
        const BaseState& b = bases_[ch];
        if (!b.cal || !b.hasPose) continue;
        if (b.isReference) {
            u.bases.push_back({b.cal->id, Pose{}, 0});
            continue;
        }
        bool worth = b.fixed || b.source == PoseSource::SavedUniverse || b.source == PoseSource::SteamVrUniverse || b.source == PoseSource::Refined ||
                     b.samples >= 30;
        if (worth) u.bases.push_back({b.cal->id, b.refFromBase, b.samples});
    }
    if (upCount_ > 0) {
        u.hasUp = true;
        u.upInRef = math::Normalize(upSum_);
        u.upSamples = upCount_;
    }
    if (u.bases.size() < 2 && !u.hasUp) u.bases.clear();
    return u;
}

}  // namespace lhp::tracking::solver
