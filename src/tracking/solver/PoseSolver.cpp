#include "tracking/solver/PoseSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace lhp::tracking::solver {

using math::Pose;
using math::Quat;
using math::Vec3;

double PredictAngle(const model::DeviceModel& model, const Measurement& m, const BaseView& b, const Pose& worldFromDevice) {
    Vec3 pw = worldFromDevice.Apply(model.points[m.sensor]);
    Vec3 pb = b.worldFromBase.Inverse().Apply(pw);
    return lighthouse::ReprojectAxis(b.cal[m.plane], pb, m.plane);
}

Vec3 BearingFromAngles(double a0, double a1) {
    // Ideal model: a0 = az - asin(t*e), a1 = az + asin(t*e), t = tan(30deg), e = Y / |XZ|, az = B - pi/2.
    double az = 0.5 * (a0 + a1);
    double s = std::sin(0.5 * (a1 - a0)) / std::tan(math::kPi / 6);
    double B = az + math::kPi / 2;
    // libsurvive frame flips Z: Z' = -z
    return math::Normalize({std::cos(B), s, -std::sin(B)});
}

namespace {

double WrapPi(double a) {
    while (a > math::kPi) a -= 2 * math::kPi;
    while (a < -math::kPi) a += 2 * math::kPi;
    return a;
}

/// Pose at the reference time plus (optionally) linear / angular velocity in the world frame.
struct State {
    Pose pose;
    Vec3 vel, angVel;
    Pose At(double dt) const {
        if (dt == 0) return pose;
        return {(Quat::FromRotVec(angVel * dt) * pose.rot).Normalized(), pose.pos + vel * dt};
    }
    State Perturbed(const double* d, int n) const {
        State s = *this;
        s.pose = pose.Perturbed(d);
        if (n >= 12) {
            s.vel += Vec3{d[6], d[7], d[8]};
            s.angVel += Vec3{d[9], d[10], d[11]};
        }
        return s;
    }
};

struct Problem {
    const model::DeviceModel& model;
    const std::vector<Measurement>& meas;
    const std::vector<BaseView>& bases;
    std::vector<Pose> baseInv;
    std::vector<char> active;
    // Weak prior on the velocity terms (keeps them from absorbing noise when weakly observable):
    // residual = k * (prior - value), k chosen so 1 m/s or 5 rad/s deviation weighs like ~3 noise-level angles.
    Vec3 velPrior, angVelPrior;
    double kVel = 3e-4, kAngVel = 6e-5;

    Problem(const model::DeviceModel& mo, const std::vector<Measurement>& me, const std::vector<BaseView>& ba) : model(mo), meas(me), bases(ba) {
        for (auto& b : bases) baseInv.push_back(b.worldFromBase.Inverse());
        active.assign(meas.size(), 1);
    }
    double Residual(size_t i, const State& s) const {
        const Measurement& m = meas[i];
        Vec3 pb = baseInv[m.base].Apply(s.At(m.dt).Apply(model.points[m.sensor]));
        if (pb.z > -1e-3) return 1.0;   // behind the base: large residual
        return WrapPi(m.angle - lighthouse::ReprojectAxis(bases[m.base].cal[m.plane], pb, m.plane));
    }
    double Cost(const State& s, double huber) const {
        double c = 0;
        for (size_t i = 0; i < meas.size(); ++i) {
            if (!active[i]) continue;
            double r = std::fabs(Residual(i, s));
            c += r <= huber ? 0.5 * r * r : huber * (r - 0.5 * huber);
        }
        return c;
    }
    double PriorCost(const State& s) const {
        Vec3 dv = (velPrior - s.vel) * kVel, dw = (angVelPrior - s.angVel) * kAngVel;
        return 0.5 * (Dot(dv, dv) + Dot(dw, dw));
    }
};

template <int N>
struct LmOut {
    State state;
    double cost = 0;
    int iterations = 0;
    std::array<double, N * N> jtj{};
};

template <int N>
LmOut<N> RunLm(const Problem& pr, State x, int maxIt, double huber) {
    LmOut<N> out;
    double lambda = 1e-3;
    auto total = [&](const State& s) { return pr.Cost(s, huber) + (N >= 12 ? pr.PriorCost(s) : 0.0); };
    double cost = total(x);
    const double h = 1e-6;
    std::array<double, N * N> jtj{};
    for (int it = 0; it < maxIt; ++it) {
        out.iterations = it + 1;
        jtj.fill(0);
        std::array<double, N> jtr{};
        for (size_t i = 0; i < pr.meas.size(); ++i) {
            if (!pr.active[i]) continue;
            double r = pr.Residual(i, x);
            double ar = std::fabs(r);
            double w = ar <= huber ? 1.0 : huber / ar;   // IRLS Huber weight
            double J[N];
            for (int k = 0; k < N; ++k) {
                double d[N] = {};
                d[k] = h;
                double rp = pr.Residual(i, x.Perturbed(d, N));
                d[k] = -h;
                double rm = pr.Residual(i, x.Perturbed(d, N));
                J[k] = -(rp - rm) / (2 * h);   // residual = meas - pred  =>  d(pred)/dx = -dr/dx
            }
            for (int a = 0; a < N; ++a) {
                jtr[a] += w * J[a] * r;
                for (int b = 0; b < N; ++b) jtj[a * N + b] += w * J[a] * J[b];
            }
        }
        if constexpr (N >= 12) {   // prior rows: pred = k*x, residual = k*(prior - x)
            for (int k = 0; k < 3; ++k) {
                jtj[(6 + k) * N + 6 + k] += pr.kVel * pr.kVel;
                jtr[6 + k] += pr.kVel * pr.kVel * (pr.velPrior[k] - x.vel[k]);
                jtj[(9 + k) * N + 9 + k] += pr.kAngVel * pr.kAngVel;
                jtr[9 + k] += pr.kAngVel * pr.kAngVel * (pr.angVelPrior[k] - x.angVel[k]);
            }
        }
        bool improved = false;
        for (int tries = 0; tries < 8 && !improved; ++tries) {
            auto A = jtj;
            for (int k = 0; k < N; ++k) A[k * N + k] += lambda * (jtj[k * N + k] + 1e-9);
            std::array<double, N> dx{};
            if (!math::SolveSpd<N>(A, jtr, dx)) {
                lambda *= 10;
                continue;
            }
            State cand = x.Perturbed(dx.data(), N);
            double c = total(cand);
            if (c < cost) {
                double step = 0;
                for (double v : dx) step += v * v;
                x = cand;
                double rel = (cost - c) / std::max(cost, 1e-30);
                cost = c;
                lambda = std::max(lambda * 0.3, 1e-9);
                improved = true;
                if (step < 1e-16 || rel < 1e-10) {
                    out.state = x;
                    out.cost = cost;
                    out.jtj = jtj;
                    return out;
                }
            } else {
                lambda *= 10;
            }
        }
        if (!improved) break;
    }
    out.state = x;
    out.cost = cost;
    out.jtj = jtj;
    return out;
}

/// Initial guesses: position from the mean bearing at a distance estimated from the angular spread,
/// orientation from 24 axis-aligned candidates.
std::vector<Pose> Seeds(const Problem& pr) {
    std::vector<Pose> seeds;
    std::map<int, int> perBase;
    for (auto& m : pr.meas) ++perBase[m.base];
    int base = std::max_element(perBase.begin(), perBase.end(), [](auto& a, auto& b) { return a.second < b.second; })->first;
    std::map<int, std::pair<double, double>> both;   // sensor -> (a0, a1)
    std::map<int, int> mask;
    for (auto& m : pr.meas) {
        if (m.base != base) continue;
        auto& e = both[m.sensor];
        (m.plane ? e.second : e.first) = m.angle;
        mask[m.sensor] |= 1 << m.plane;
    }
    std::vector<Vec3> bearings;
    for (auto& [s, a] : both)
        if (mask[s] == 3) bearings.push_back(BearingFromAngles(a.first, a.second));
    Vec3 mean{0, 0, -1};
    double dist = 2.0;
    if (!bearings.empty()) {
        Vec3 sum;
        for (auto& b : bearings) sum += b;
        mean = math::Normalize(sum);
        double spread = 0;
        for (auto& b : bearings) spread = std::max(spread, std::acos(std::clamp(Dot(b, mean), -1.0, 1.0)));
        double size = 0;
        for (auto& p : pr.model.points) size = std::max(size, Norm(p));
        if (spread > 1e-4) dist = std::clamp(size / std::tan(spread), 0.3, 8.0);
    }
    Vec3 posWorld = pr.bases[base].worldFromBase.Apply(mean * dist);
    const Vec3 axes[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (auto& up : axes)
        for (int r = 0; r < 4; ++r) {
            Quat q = Quat::FromRotVec(up * (r * math::kPi / 2)) * Quat::FromTo({0, 1, 0}, up);
            seeds.push_back({q.Normalized(), posWorld});
        }
    return seeds;
}

template <int N>
void Finish(const Problem& pr, const LmOut<N>& o, const SolverOptions& opt, SolveResult& res) {
    res.worldFromDevice = o.state.pose;
    res.velocity = o.state.vel;
    res.angularVelocity = o.state.angVel;
    double s = 0;
    for (size_t i = 0; i < pr.meas.size(); ++i) {
        double r = pr.Residual(i, o.state);
        res.residuals[i] = r;
        if (pr.active[i]) {
            s += r * r;
            res.maxAbsRad = std::max(res.maxAbsRad, std::fabs(r));
        }
    }
    res.rmsRad = std::sqrt(s / std::max(res.used, 1));
    double sigma2 = res.used > N ? s / (res.used - N) : s;
    std::array<double, N * N> inv{};
    if (math::InvertSpd<N>(o.jtj, inv)) {
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c) res.covariance[r * 6 + c] = inv[r * N + c] * sigma2;
        res.rotationSigma = std::sqrt(std::max(0.0, res.covariance[0] + res.covariance[7] + res.covariance[14]));
        res.positionSigma = std::sqrt(std::max(0.0, res.covariance[21] + res.covariance[28] + res.covariance[35]));
    }
    (void)opt;
}

}  // namespace

SolveResult SolvePose(const model::DeviceModel& model, const std::vector<Measurement>& meas, const std::vector<BaseView>& bases, const Pose* initial,
                      const SolverOptions& opt) {
    SolveResult res;
    res.residuals.assign(meas.size(), std::numeric_limits<double>::quiet_NaN());
    res.rejectedMask.assign(meas.size(), 0);
    if (meas.size() < 6) {
        res.failReason = "观测不足（需要至少 6 个角度）";
        return res;
    }
    double tMin = 0, tMax = 0;
    for (auto& m : meas) {
        if (m.sensor < 0 || m.sensor >= (int)model.points.size() || m.base < 0 || m.base >= (int)bases.size()) {
            res.failReason = "观测引用了不存在的传感器或基站";
            return res;
        }
        tMin = std::min(tMin, m.dt);
        tMax = std::max(tMax, m.dt);
    }
    Problem pr(model, meas, bases);
    pr.velPrior = opt.velocityPrior;
    pr.angVelPrior = opt.angularVelocityPrior;

    State start;
    if (initial) {
        start.pose = *initial;
    } else {
        auto seeds = Seeds(pr);
        double best = std::numeric_limits<double>::max();
        for (auto& sd : seeds) {
            ++res.seedsTried;
            State s0;
            s0.pose = sd;
            auto o = RunLm<6>(pr, s0, 8, opt.huberRad * 20);
            if (o.cost < best) {
                best = o.cost;
                start = o.state;
            }
        }
    }
    {
        double s = 0;
        for (size_t i = 0; i < meas.size(); ++i) {
            double r = pr.Residual(i, start);
            s += r * r;
        }
        res.initialRmsRad = std::sqrt(s / (double)meas.size());
    }
    start.vel = opt.velocityPrior;
    start.angVel = opt.angularVelocityPrior;
    // Velocity terms only when the observations span enough time and are plentiful.
    // With one base all angles come from (nearly) the same rotation, so a static device gains nothing from it;
    // with several bases (sampled up to a rotation apart) or a moving device it removes the motion blur.
    std::vector<char> seenBase(bases.size(), 0);
    int nBases = 0;
    for (auto& m : meas) nBases += !seenBase[m.base]++;
    bool moving = math::Norm(opt.velocityPrior) > 0.05 || math::Norm(opt.angularVelocityPrior) > 0.3;
    res.velocityEstimated = opt.estimateVelocity && (tMax - tMin) > 0.004 && meas.size() >= 18 && (nBases >= 2 || moving);
    auto solve = [&](State x, auto tag) {
        constexpr int N = decltype(tag)::value;
        auto o = RunLm<N>(pr, x, opt.maxIterations, opt.huberRad);
        res.iterations += o.iterations;
        int rejected = 0;
        for (size_t i = 0; i < meas.size(); ++i)
            if (pr.active[i] && std::fabs(pr.Residual(i, o.state)) > opt.outlierRad) {
                pr.active[i] = 0;
                res.rejectedMask[i] = 1;
                ++rejected;
            }
        if (rejected && (int)meas.size() - rejected >= N) {
            o = RunLm<N>(pr, o.state, opt.maxIterations, opt.huberRad);
            res.iterations += o.iterations;
        }
        res.rejected = rejected;
        res.used = (int)meas.size() - rejected;
        Finish<N>(pr, o, opt, res);
    };
    const auto activeInit = pr.active;
    if (res.velocityEstimated) {
        solve(start, std::integral_constant<int, 12>{});
        // implausible velocities mean the extra terms soaked up noise / model error: fall back to static
        if (math::Norm(res.velocity) > 5.0 || math::Norm(res.angularVelocity) > 20.0) {
            pr.active = activeInit;
            res.rejectedMask.assign(meas.size(), 0);
            res.maxAbsRad = 0;
            res.velocityEstimated = false;
            res.velocity = res.angularVelocity = Vec3{};
            start.vel = start.angVel = Vec3{};
            solve(start, std::integral_constant<int, 6>{});
        }
    } else {
        solve(start, std::integral_constant<int, 6>{});
    }

    Vec3 inBase = bases[0].worldFromBase.Inverse().Apply(res.worldFromDevice.pos);
    res.ok = res.used >= 6 && res.rmsRad < 5e-3 && inBase.z < 0;
    if (!res.ok) res.failReason = res.rmsRad >= 5e-3 ? "残差过大（未收敛或数据不一致）" : "解在基站背后";
    return res;
}

}  // namespace lhp::tracking::solver
