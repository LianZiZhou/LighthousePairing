#include "tracking/filter/TrackerFilter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lhp::tracking::filter {

using math::Pose;
using math::Quat;
using math::Vec3;

namespace {

constexpr int N = kStateDim;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const Vec3 kGravityUp{0, kGravity, 0};   // specific force of a resting accelerometer, world frame (Y up)

using M3 = std::array<double, 9>;   // row-major

M3 RotM(const Quat& q) {
    Vec3 c0 = q.Rotate({1, 0, 0}), c1 = q.Rotate({0, 1, 0}), c2 = q.Rotate({0, 0, 1});
    return {c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z};
}
M3 Tr(const M3& m) { return {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]}; }
M3 Mul(const M3& a, const M3& b) {
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return r;
}
Vec3 Mul(const M3& m, const Vec3& v) { return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z}; }
M3 Skew(const Vec3& v) { return {0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0}; }

double WrapPi(double a) {
    while (a > math::kPi) a -= 2 * math::kPi;
    while (a < -math::kPi) a += 2 * math::kPi;
    return a;
}

/// In-place inverse of a small dense m x m matrix (Gauss-Jordan, partial pivoting). Returns false if singular.
bool InvertSmall(double* A, int m) {   // m <= 9
    double inv[81] = {};
    for (int i = 0; i < m; ++i) inv[i * m + i] = 1;
    for (int c = 0; c < m; ++c) {
        int piv = c;
        for (int r = c + 1; r < m; ++r)
            if (std::fabs(A[r * m + c]) > std::fabs(A[piv * m + c])) piv = r;
        if (std::fabs(A[piv * m + c]) < 1e-300) return false;
        if (piv != c)
            for (int k = 0; k < m; ++k) {
                std::swap(A[c * m + k], A[piv * m + k]);
                std::swap(inv[c * m + k], inv[piv * m + k]);
            }
        double d = A[c * m + c];
        for (int k = 0; k < m; ++k) {
            A[c * m + k] /= d;
            inv[c * m + k] /= d;
        }
        for (int r = 0; r < m; ++r) {
            if (r == c) continue;
            double f = A[r * m + c];
            if (f == 0) continue;
            for (int k = 0; k < m; ++k) {
                A[r * m + k] -= f * A[c * m + k];
                inv[r * m + k] -= f * inv[c * m + k];
            }
        }
    }
    std::copy(inv, inv + m * m, A);
    return true;
}

void MatMul(const double* A, const double* B, double* C, int n) {   // C = A * B, n x n
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            double s = 0;
            for (int k = 0; k < n; ++k) s += A[i * n + k] * B[k * n + j];
            C[i * n + j] = s;
        }
}

void MatMulBt(const double* A, const double* B, double* C, int n) {   // C = A * B^T
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            double s = 0;
            for (int k = 0; k < n; ++k) s += A[i * n + k] * B[j * n + k];
            C[i * n + j] = s;
        }
}

void SetBlock(double* M, int r0, int c0, const M3& b) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M[(r0 + i) * N + c0 + j] = b[i * 3 + j];
}

/// Solver pose covariance index [rx ry rz tx ty tz] -> filter index.
int FromSolverIndex(int k) { return k < 3 ? kTheta + k : kP + (k - 3); }

}  // namespace

TrackerFilter::TrackerFilter(const model::DeviceModel& model, FilterParams params) : model_(model), prm_(params) {
    imuToDevice_ = model_.imu.mount.rot;
}

void TrackerFilter::Reset() {
    init_ = false;
    P_.fill(0);
}

void TrackerFilter::Initialize(double t, const Pose& worldFromDevice, const std::array<double, 36>& poseCov, const Vec3& angVelWorld, const Vec3& gyroBias) {
    pose_ = worldFromDevice;
    v_ = a_ = ba_ = Vec3{};
    w_ = angVelWorld;
    bg_ = gyroBias;
    P_.fill(0);
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) P_[FromSolverIndex(r) * N + FromSolverIndex(c)] = 4.0 * poseCov[r * 6 + c];
    for (int k = 0; k < 3; ++k) {
        P_[(kP + k) * N + kP + k] += prm_.minPosVar;
        P_[(kTheta + k) * N + kTheta + k] += prm_.minRotVar;
        P_[(kV + k) * N + kV + k] = prm_.initVelVar;
        P_[(kW + k) * N + kW + k] = prm_.initAngVelVar;
        P_[(kA + k) * N + kA + k] = prm_.initAccVar;
        P_[(kBg + k) * N + kBg + k] = prm_.initGyroBiasVar;
        P_[(kBa + k) * N + kBa + k] = prm_.initAccBiasVar;
    }
    t_ = tInit_ = t;
    init_ = true;
}

void TrackerFilter::Set(const Nominal& n) {
    pose_ = n.pose;
    v_ = n.v;
    w_ = n.w;
    a_ = n.a;
    bg_ = n.bg;
    ba_ = n.ba;
}

TrackerFilter::Nominal TrackerFilter::Plus(const Nominal& x, const double* d) {
    Nominal r = x;
    r.pose.pos += Vec3{d[kP], d[kP + 1], d[kP + 2]};
    r.pose.rot = (Quat::FromRotVec({d[kTheta], d[kTheta + 1], d[kTheta + 2]}) * x.pose.rot).Normalized();
    r.v += Vec3{d[kV], d[kV + 1], d[kV + 2]};
    r.w += Vec3{d[kW], d[kW + 1], d[kW + 2]};
    r.a += Vec3{d[kA], d[kA + 1], d[kA + 2]};
    r.bg += Vec3{d[kBg], d[kBg + 1], d[kBg + 2]};
    r.ba += Vec3{d[kBa], d[kBa + 1], d[kBa + 2]};
    return r;
}

void TrackerFilter::Minus(const Nominal& x, const Nominal& x0, double* d) {
    auto put = [&](int i, const Vec3& v) {
        d[i] = v.x;
        d[i + 1] = v.y;
        d[i + 2] = v.z;
    };
    put(kP, x.pose.pos - x0.pose.pos);
    put(kTheta, (x.pose.rot * x0.pose.rot.Conj()).ToRotVec());
    put(kV, x.v - x0.v);
    put(kW, x.w - x0.w);
    put(kA, x.a - x0.a);
    put(kBg, x.bg - x0.bg);
    put(kBa, x.ba - x0.ba);
}

void TrackerFilter::Symmetrize() {
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            double m = 0.5 * (P_[i * N + j] + P_[j * N + i]);
            P_[i * N + j] = P_[j * N + i] = m;
        }
}

void TrackerFilter::Predict(double t) {
    if (!init_) {
        t_ = t;
        return;
    }
    double dt = t - t_;
    if (dt <= 0) return;
    while (dt > 1e-9) {
        double step = std::min(dt, prm_.maxPredictDt);
        PredictStep(step);
        dt -= step;
    }
    t_ = t;
}

void TrackerFilter::PredictStep(double dt) {
    // nominal state (constant acceleration / constant angular velocity)
    Quat dq = Quat::FromRotVec(w_ * dt);
    pose_.pos += v_ * dt + a_ * (0.5 * dt * dt);
    v_ += a_ * dt;
    pose_.rot = (dq * pose_.rot).Normalized();

    // error-state transition
    static thread_local double F[N * N], FP[N * N], Pn[N * N];
    std::fill(F, F + N * N, 0.0);
    for (int i = 0; i < N; ++i) F[i * N + i] = 1;
    for (int k = 0; k < 3; ++k) {
        F[(kP + k) * N + kV + k] = dt;
        F[(kP + k) * N + kA + k] = 0.5 * dt * dt;
        F[(kV + k) * N + kA + k] = dt;
        F[(kTheta + k) * N + kW + k] = dt;
    }
    SetBlock(F, kTheta, kTheta, RotM(dq));
    MatMul(F, P_.data(), FP, N);
    MatMulBt(FP, F, Pn, N);

    // process noise: white jerk on (p, v, a), white angular acceleration on (theta, w), bias random walks
    const double q = prm_.jerkPsd, qa = prm_.angAccPsd;
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
    for (int k = 0; k < 3; ++k) {
        int p = kP + k, v = kV + k, a = kA + k, th = kTheta + k, w = kW + k;
        Pn[p * N + p] += q * dt5 / 20;
        Pn[p * N + v] += q * dt4 / 8;
        Pn[v * N + p] += q * dt4 / 8;
        Pn[p * N + a] += q * dt3 / 6;
        Pn[a * N + p] += q * dt3 / 6;
        Pn[v * N + v] += q * dt3 / 3;
        Pn[v * N + a] += q * dt2 / 2;
        Pn[a * N + v] += q * dt2 / 2;
        Pn[a * N + a] += q * dt;
        Pn[th * N + th] += qa * dt3 / 3;
        Pn[th * N + w] += qa * dt2 / 2;
        Pn[w * N + th] += qa * dt2 / 2;
        Pn[w * N + w] += qa * dt;
        Pn[(kBg + k) * N + kBg + k] += prm_.gyroBiasRw * dt;
        Pn[(kBa + k) * N + kBa + k] += prm_.accBiasRw * dt;
    }
    std::copy(Pn, Pn + N * N, P_.begin());
    Symmetrize();
}

namespace {

/// Standard EKF update with an m-row measurement (m <= 9), dense H (m x N). Returns the NIS.
double EkfUpdate(Covariance& P, const double* H, const double* r, const double* Rdiag, int m, double* dx) {
    static thread_local double PHt[N * 9], S[81], K[N * 9];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0;
            for (int k = 0; k < N; ++k)
                if (H[j * N + k] != 0) s += P[i * N + k] * H[j * N + k];
            PHt[i * m + j] = s;
        }
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0;
            for (int k = 0; k < N; ++k)
                if (H[i * N + k] != 0) s += H[i * N + k] * PHt[k * m + j];
            S[i * m + j] = s + (i == j ? Rdiag[i] : 0.0);
        }
    double Sinv[81];
    std::copy(S, S + m * m, Sinv);
    if (!InvertSmall(Sinv, m)) {
        std::fill(dx, dx + N, 0.0);
        return kNaN;
    }
    double nis = 0;
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) nis += r[i] * Sinv[i * m + j] * r[j];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < m; ++j) {
            double s = 0;
            for (int k = 0; k < m; ++k) s += PHt[i * m + k] * Sinv[k * m + j];
            K[i * m + j] = s;
        }
    for (int i = 0; i < N; ++i) {
        double s = 0;
        for (int j = 0; j < m; ++j) s += K[i * m + j] * r[j];
        dx[i] = s;
    }
    // P -= K S K^T  (== K H P)
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            double s = 0;
            for (int k = 0; k < m; ++k) s += K[i * m + k] * PHt[j * m + k];
            P[i * N + j] -= s;
        }
    return nis;
}

}  // namespace

ImuUpdate TrackerFilter::UpdateImu(double t, const Vec3& accImu, const Vec3& gyroImu, bool useAcc, bool useGyro) {
    ImuUpdate out;
    if (!init_) return out;
    Predict(t);
    M3 C = Tr(Mul(RotM(pose_.rot), RotM(imuToDevice_)));   // world -> IMU
    double H[6 * N] = {};
    double r[6], R[6];
    int m = 0;
    auto row3 = [&](const Vec3& meas, const Vec3& pred, int thetaCol, const M3& dTheta, int stateCol, const M3& dState, int biasCol, double var) {
        Vec3 res = meas - pred;
        for (int k = 0; k < 3; ++k) {
            double* h = H + (m + k) * N;
            for (int j = 0; j < 3; ++j) {
                h[thetaCol + j] = dTheta[k * 3 + j];
                h[stateCol + j] = dState[k * 3 + j];
            }
            h[biasCol + k] = 1;
            r[m + k] = res[k];
            R[m + k] = var;
        }
        m += 3;
        return res;
    };
    if (useGyro) {
        Vec3 pred = Mul(C, w_) + bg_;
        out.gyroResidual = row3(gyroImu, pred, kTheta, Mul(C, Skew(w_)), kW, C, kBg, prm_.gyroVar);
        out.gyroUsed = true;
    }
    if (useAcc) {
        Vec3 f = a_ + kGravityUp;
        Vec3 pred = Mul(C, f) + ba_;
        out.accResidual = row3(accImu, pred, kTheta, Mul(C, Skew(f)), kA, C, kBa, prm_.accVar);
        out.accUsed = true;
    }
    if (!m) return out;
    double dx[N];
    Covariance P0 = P_;
    double nis = EkfUpdate(P_, H, r, R, m, dx);
    // Soft robustness: an implausible IMU innovation (shock, saturation, model error) is down-weighted.
    if (std::isfinite(nis) && nis > 25.0 * m) {
        P_ = P0;
        double s = nis / (25.0 * m);
        for (int i = 0; i < m; ++i) R[i] *= s;
        nis = EkfUpdate(P_, H, r, R, m, dx);
    }
    out.nis = nis;
    if (!std::isfinite(nis)) {
        P_ = P0;
        return out;
    }
    Set(Plus(Get(), dx));
    Symmetrize();
    return out;
}

void TrackerFilter::UpdateZeroVelocity(double var, bool angular) {
    if (!init_) return;
    double H[9 * N] = {};
    double r[9], R[9];
    int m = 0;
    auto add = [&](int col, const Vec3& value) {
        for (int k = 0; k < 3; ++k) {
            H[(m + k) * N + col + k] = 1;
            r[m + k] = -value[k];
            R[m + k] = var;
        }
        m += 3;
    };
    add(kV, v_);
    add(kA, a_);
    if (angular) add(kW, w_);
    double dx[N];
    Covariance P0 = P_;
    double nis = EkfUpdate(P_, H, r, R, m, dx);
    if (!std::isfinite(nis)) {
        P_ = P0;
        return;
    }
    Set(Plus(Get(), dx));
    Symmetrize();
}

void TrackerFilter::Inflate(double posVar, double rotVar) {
    if (!init_) return;
    for (int k = 0; k < 3; ++k) {
        P_[(kP + k) * N + kP + k] += posVar;
        P_[(kTheta + k) * N + kTheta + k] += rotVar;
    }
    tInit_ = t_;
}

void TrackerFilter::TransformWorld(const Pose& T) {
    if (!init_) return;
    pose_ = T * pose_;
    v_ = T.rot.Rotate(v_);
    w_ = T.rot.Rotate(w_);
    a_ = T.rot.Rotate(a_);
    // error covariance: the world-frame blocks rotate with the frame
    static thread_local double G[N * N], GP[N * N], Pn[N * N];
    std::fill(G, G + N * N, 0.0);
    for (int i = 0; i < N; ++i) G[i * N + i] = 1;
    M3 R = RotM(T.rot);
    for (int b : {kP, kTheta, kV, kW, kA}) SetBlock(G, b, b, R);
    MatMul(G, P_.data(), GP, N);
    MatMulBt(GP, G, Pn, N);
    std::copy(Pn, Pn + N * N, P_.begin());
    Symmetrize();
}

double TrackerFilter::Predict(const LightMeas& m, const LightBase& b, const Pose& worldFromDevice, double* facing) const {
    Vec3 pw = worldFromDevice.Apply(model_.points[m.sensor]);
    if (facing) {
        Vec3 n = worldFromDevice.rot.Rotate(model_.normals[m.sensor]);
        *facing = math::Dot(n, math::Normalize(b.worldFromBase.pos - pw));
    }
    Vec3 pb = b.worldFromBase.Inverse().Apply(pw);
    if (pb.z > -1e-3) return kNaN;
    return lighthouse::ReprojectAxis(b.cal[m.plane], pb, m.plane);
}

LightUpdate TrackerFilter::UpdateLight(const std::vector<LightMeas>& meas, const std::vector<LightBase>& bases) {
    LightUpdate out;
    out.n = (int)meas.size();
    out.residuals.assign(meas.size(), kNaN);
    out.rejected.assign(meas.size(), 0);
    if (!init_) {
        out.note = "滤波器未初始化";
        return out;
    }
    if (meas.empty()) {
        out.note = "没有观测";
        return out;
    }
    double tMin = meas[0].t, tMax = meas[0].t;
    for (auto& m : meas) {
        tMin = std::min(tMin, m.t);
        tMax = std::max(tMax, m.t);
    }
    out.spanMs = (tMax - tMin) * 1000;
    if (tMax > t_) Predict(tMax);
    out.lagMs = (t_ - tMax) * 1000;
    if (t_ - tMin > 0.25) {
        out.note = "光学数据比滤波器时间早太多";
        return out;
    }

    std::vector<Pose> baseInv;
    baseInv.reserve(bases.size());
    for (auto& b : bases) baseInv.push_back(b.worldFromBase.Inverse());
    const Nominal x0 = Get();
    auto sigmaOf = [&](const LightMeas& m) { return prm_.lightSigma * bases[m.base].sigmaScale; };

    auto angleAt = [&](const LightMeas& m, const Pose& pose) {
        Vec3 pb = baseInv[m.base].Apply(pose.Apply(model_.points[m.sensor]));
        if (pb.z > -1e-3) return kNaN;
        return lighthouse::ReprojectAxis(bases[m.base].cal[m.plane], pb, m.plane);
    };
    // Residual and Jacobian row (only p, theta, v, w, a are non-zero) of one measurement at state x.
    struct Row {
        double r = kNaN;
        double h[15] = {};   // [p theta v w a]
        double facing = 1;
    };
    auto evaluate = [&](const Nominal& x, const LightMeas& m) {
        Row row;
        double dt = m.t - t_;
        Quat qdt = Quat::FromRotVec(x.w * dt);
        Pose pdt{(qdt * x.pose.rot).Normalized(), x.pose.pos + x.v * dt + x.a * (0.5 * dt * dt)};
        double pred = angleAt(m, pdt);
        if (!std::isfinite(pred)) return row;
        row.r = WrapPi(m.angle - pred);
        Vec3 pw = pdt.Apply(model_.points[m.sensor]);
        row.facing = math::Dot(pdt.rot.Rotate(model_.normals[m.sensor]), math::Normalize(bases[m.base].worldFromBase.pos - pw));
        const double h = 1e-6;
        double Jp[3], Jt[3];
        for (int k = 0; k < 3; ++k) {
            Pose q = pdt;
            q.pos[k] += h;
            Jp[k] = WrapPi(angleAt(m, q) - pred) / h;
            Vec3 e;
            e[k] = h;
            Pose qr{(Quat::FromRotVec(e) * pdt.rot).Normalized(), pdt.pos};
            Jt[k] = WrapPi(angleAt(m, qr) - pred) / h;
        }
        // chain rule back to the filter time: d(theta_dt) = Rot(w dt) d(theta) + dt d(w)
        M3 M = RotM(qdt);
        for (int k = 0; k < 3; ++k) {
            row.h[0 + k] = Jp[k];
            row.h[3 + k] = Jt[0] * M[0 * 3 + k] + Jt[1] * M[1 * 3 + k] + Jt[2] * M[2 * 3 + k];
            row.h[6 + k] = Jp[k] * dt;
            row.h[9 + k] = Jt[k] * dt;
            row.h[12 + k] = Jp[k] * 0.5 * dt * dt;
        }
        return row;
    };

    // ---- gating at the prior
    const bool converging = SecondsSinceInit() < prm_.convergeSeconds;
    const double gate = converging ? prm_.gateSigmaConverging : prm_.gateSigma;
    std::vector<Row> rows(meas.size());
    std::vector<char> use(meas.size(), 0);
    double sumR2 = 0, sumNis = 0;
    for (size_t i = 0; i < meas.size(); ++i) {
        rows[i] = evaluate(x0, meas[i]);
        Row& rw = rows[i];
        if (!std::isfinite(rw.r)) {
            out.rejected[i] = 1;
            ++out.gated;
            continue;
        }
        if (rw.facing < -0.05 && !converging) {   // a sensor cannot see a base behind its own surface
            out.rejected[i] = 1;
            ++out.facingAway;
            continue;
        }
        // innovation variance s = h P h^T + sigma^2 over the 15 active columns
        const double sg = sigmaOf(meas[i]);
        double s = sg * sg;
        for (int a = 0; a < 15; ++a) {
            if (rw.h[a] == 0) continue;
            double acc = 0;
            for (int b = 0; b < 15; ++b) acc += P_[a * N + b] * rw.h[b];
            s += rw.h[a] * acc;
        }
        double nis = rw.r * rw.r / s;
        if (nis > gate * gate) {
            out.rejected[i] = 1;
            ++out.gated;
            continue;
        }
        use[i] = 1;
        ++out.used;
        sumR2 += rw.r * rw.r;
        sumNis += nis;
    }
    out.rmsBeforeRad = out.used ? std::sqrt(sumR2 / out.used) : 0;
    out.nis = out.used ? sumNis / out.used : 0;
    // A converged filter takes even single angles (like Crazyflie's per-sweep update); right after
    // initialisation a few are needed together so one bad angle cannot drag the state.
    if (out.used < (converging ? 4 : 1)) {
        out.note = "可用观测不足";
        return out;
    }
    if (out.n >= 6 && out.gated > out.used) {
        out.note = "多数观测被新息门限拒绝";
        return out;
    }

    // ---- iterated update in information form: minimise |x - x0|^2_P0^-1 + sum w_i (r_i / sigma)^2
    Covariance lambda0;
    {
        Covariance Pj = P_;
        for (int i = 0; i < N; ++i) Pj[i * N + i] += 1e-15;
        if (!math::InvertSpd<N>(Pj, lambda0)) {
            out.note = "协方差矩阵不正定";
            return out;
        }
    }
    Nominal x = x0;
    Covariance lambda{};
    for (int it = 0; it < prm_.maxIterations; ++it) {
        out.iterations = it + 1;
        double d[N];
        Minus(x, x0, d);
        lambda = lambda0;
        std::array<double, N> b{};
        for (int i = 0; i < N; ++i) {
            double s = 0;
            for (int j = 0; j < N; ++j) s -= lambda0[i * N + j] * d[j];
            b[i] = s;
        }
        for (size_t i = 0; i < meas.size(); ++i) {
            if (!use[i]) continue;
            if (it > 0) rows[i] = evaluate(x, meas[i]);
            const Row& rw = rows[i];
            if (!std::isfinite(rw.r)) continue;
            const double sigma = sigmaOf(meas[i]);
            double ar = std::fabs(rw.r);
            double w = ar <= prm_.huberK * sigma ? 1.0 : prm_.huberK * sigma / ar;
            double ws = w / (sigma * sigma);
            for (int a = 0; a < 15; ++a) {
                if (rw.h[a] == 0) continue;
                b[a] += ws * rw.h[a] * rw.r;
                for (int c = 0; c < 15; ++c) lambda[a * N + c] += ws * rw.h[a] * rw.h[c];
            }
        }
        std::array<double, N> dx{};
        if (!math::SolveSpd<N>(lambda, b, dx)) {
            out.note = "信息矩阵求解失败";
            return out;
        }
        x = Plus(x, dx.data());
        double step = 0;
        for (double v : dx) step += v * v;
        if (step < 1e-16) break;
    }
    Covariance Pn;
    if (!math::InvertSpd<N>(lambda, Pn)) {
        out.note = "后验协方差求逆失败";
        return out;
    }
    Set(x);
    P_ = Pn;
    Symmetrize();
    out.applied = true;

    double s = 0;
    int cnt = 0;
    for (size_t i = 0; i < meas.size(); ++i) {
        Row rw = evaluate(x, meas[i]);
        out.residuals[i] = rw.r;
        if (use[i] && std::isfinite(rw.r)) {
            s += rw.r * rw.r;
            ++cnt;
        }
    }
    out.rmsAfterRad = cnt ? std::sqrt(s / cnt) : 0;
    return out;
}

Pose TrackerFilter::PoseAt(double t) const {
    double dt = std::clamp(t - t_, -0.1, 0.1);
    return {(Quat::FromRotVec(w_ * dt) * pose_.rot).Normalized(), pose_.pos + v_ * dt + a_ * (0.5 * dt * dt)};
}

double TrackerFilter::PositionSigma() const { return std::sqrt(std::max(0.0, Var(kP) + Var(kP + 1) + Var(kP + 2))); }
double TrackerFilter::RotationSigma() const { return std::sqrt(std::max(0.0, Var(kTheta) + Var(kTheta + 1) + Var(kTheta + 2))); }

std::array<double, 36> TrackerFilter::PoseCovariance() const {
    std::array<double, 36> c{};
    for (int r = 0; r < 6; ++r)
        for (int k = 0; k < 6; ++k) c[r * 6 + k] = P_[FromSolverIndex(r) * N + FromSolverIndex(k)];
    return c;
}

bool TrackerFilter::Healthy() const {
    if (!init_) return false;
    for (double v : P_)
        if (!std::isfinite(v)) return false;
    auto finite = [](const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
    if (!finite(pose_.pos) || !finite(v_) || !finite(w_) || !finite(a_)) return false;
    return PositionSigma() < 1.0 && RotationSigma() < 1.0 && math::Norm(v_) < 50 && math::Norm(w_) < 100;
}

}  // namespace lhp::tracking::filter
