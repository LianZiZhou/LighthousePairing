#pragma once
// Small fixed-size linear algebra for tracking (header-only, double precision).
// Conventions: right-handed, quaternions (w, x, y, z), Pose maps local -> parent: p_parent = R * p_local + t.
#include <array>
#include <cmath>

namespace lhp::tracking::math {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    constexpr Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
    double& operator[](int i) { return i == 0 ? x : i == 1 ? y : z; }
    double operator[](int i) const { return i == 0 ? x : i == 1 ? y : z; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { return *this = *this + o; }
    Vec3& operator-=(const Vec3& o) { return *this = *this - o; }
};
inline double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double Norm(const Vec3& a) { return std::sqrt(Dot(a, a)); }
inline Vec3 Normalize(const Vec3& a) {
    double n = Norm(a);
    return n > 0 ? a / n : Vec3{};
}

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
    static Quat Identity() { return {}; }
    Quat operator*(const Quat& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y, w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w};
    }
    Quat Conj() const { return {w, -x, -y, -z}; }
    Quat Normalized() const {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        return n > 0 ? Quat{w / n, x / n, y / n, z / n} : Quat{};
    }
    Vec3 Rotate(const Vec3& v) const {
        // v' = v + 2w(q x v) + 2 q x (q x v)
        Vec3 q{x, y, z};
        Vec3 t = Cross(q, v) * 2.0;
        return v + t * w + Cross(q, t);
    }
    /// Rotation vector (axis * angle) -> quaternion.
    static Quat FromRotVec(const Vec3& r) {
        double a = Norm(r);
        if (a < 1e-12) return Quat{1, r.x * 0.5, r.y * 0.5, r.z * 0.5}.Normalized();
        double s = std::sin(a * 0.5) / a;
        return {std::cos(a * 0.5), r.x * s, r.y * s, r.z * s};
    }
    Vec3 ToRotVec() const {
        Quat q = w < 0 ? Quat{-w, -x, -y, -z} : *this;
        double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        if (s < 1e-12) return {q.x * 2, q.y * 2, q.z * 2};
        double a = 2 * std::atan2(s, q.w);
        return Vec3{q.x, q.y, q.z} * (a / s);
    }
    /// Shortest rotation taking unit vector a onto unit vector b.
    static Quat FromTo(const Vec3& a, const Vec3& b) {
        Vec3 c = Cross(a, b);
        double d = Dot(a, b);
        if (d < -0.999999) {
            Vec3 axis = std::fabs(a.x) < 0.9 ? Cross(a, {1, 0, 0}) : Cross(a, {0, 1, 0});
            return FromRotVec(Normalize(axis) * kPi);
        }
        return Quat{1 + d, c.x, c.y, c.z}.Normalized();
    }
    /// Rotation whose local +X / +Z axes map onto the given (orthogonalised) directions.
    static Quat FromAxes(const Vec3& plusX, const Vec3& plusZ);
};

inline Quat Quat::FromAxes(const Vec3& plusX, const Vec3& plusZ) {
    Vec3 X = Normalize(plusX);
    Vec3 Z = Normalize(plusZ - X * Dot(plusZ, X));
    Vec3 Y = Cross(Z, X);
    // rotation matrix columns X, Y, Z -> quaternion
    double m00 = X.x, m01 = Y.x, m02 = Z.x, m10 = X.y, m11 = Y.y, m12 = Z.y, m20 = X.z, m21 = Y.z, m22 = Z.z;
    double tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0) {
        double s = std::sqrt(tr + 1.0) * 2;
        q = {0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s};
    } else if (m00 > m11 && m00 > m22) {
        double s = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        q = {(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s};
    } else if (m11 > m22) {
        double s = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        q = {(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s};
    } else {
        double s = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        q = {(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s};
    }
    return q.Normalized();
}

struct Pose {
    Quat rot;
    Vec3 pos;
    Vec3 Apply(const Vec3& p) const { return rot.Rotate(p) + pos; }
    Pose Inverse() const {
        Quat ri = rot.Conj();
        return {ri, -ri.Rotate(pos)};
    }
    /// (this * b): first b, then this.
    Pose operator*(const Pose& b) const { return {(rot * b.rot).Normalized(), rot.Rotate(b.pos) + pos}; }
    /// Apply a small 6-dof increment [rx ry rz tx ty tz] in the parent frame.
    Pose Perturbed(const double* d) const {
        Quat dq = Quat::FromRotVec({d[0], d[1], d[2]});
        return {(dq * rot).Normalized(), pos + Vec3{d[3], d[4], d[5]}};
    }
};

/// Solve A x = b for a symmetric positive (semi-)definite N x N system (Cholesky with tiny damping fallback).
template <int N>
bool SolveSpd(std::array<double, N * N> A, std::array<double, N> b, std::array<double, N>& x) {
    for (int k = 0; k < N; ++k) {
        double d = A[k * N + k];
        for (int j = 0; j < k; ++j) d -= A[k * N + j] * A[k * N + j];
        if (d <= 1e-18) return false;
        d = std::sqrt(d);
        A[k * N + k] = d;
        for (int i = k + 1; i < N; ++i) {
            double s = A[i * N + k];
            for (int j = 0; j < k; ++j) s -= A[i * N + j] * A[k * N + j];
            A[i * N + k] = s / d;
        }
    }
    for (int i = 0; i < N; ++i) {
        double s = b[i];
        for (int j = 0; j < i; ++j) s -= A[i * N + j] * b[j];
        b[i] = s / A[i * N + i];
    }
    for (int i = N - 1; i >= 0; --i) {
        double s = b[i];
        for (int j = i + 1; j < N; ++j) s -= A[j * N + i] * x[j];
        x[i] = s / A[i * N + i];
    }
    return true;
}

/// Inverse of an SPD matrix via column solves (used for covariance = (J^T J)^-1 * sigma^2).
template <int N>
bool InvertSpd(const std::array<double, N * N>& A, std::array<double, N * N>& inv) {
    for (int c = 0; c < N; ++c) {
        std::array<double, N> e{}, x{};
        e[c] = 1;
        if (!SolveSpd<N>(A, e, x)) return false;
        for (int r = 0; r < N; ++r) inv[r * N + c] = x[r];
    }
    return true;
}

}  // namespace lhp::tracking::math
