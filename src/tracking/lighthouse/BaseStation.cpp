#include "tracking/lighthouse/BaseStation.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace lhp::tracking::lighthouse {

double ChannelFrequency(int channel) {
    static const double f[kMaxChannels] = {50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182,
                                           51.2273, 51.6685, 52.2307, 52.6894, 52.9217, 53.2741, 53.7514, 54.1150};
    return channel >= 0 && channel < kMaxChannels ? f[channel] : 50.0;
}

namespace {

float HalfToFloat(uint16_t x) {
    uint32_t sign = (uint32_t)(x & 0x8000) << 16;
    uint32_t exp = (x >> 10) & 0x1F;
    uint32_t man = x & 0x3FF;
    uint32_t bits;
    if (exp == 0) {
        if (man == 0) bits = sign;
        else {   // denormal
            int e = -1;
            do {
                ++e;
                man <<= 1;
            } while ((man & 0x400) == 0);
            bits = sign | ((uint32_t)(112 - e) << 23) | ((man & 0x3FF) << 13);
        }
    } else if (exp == 0x1F) {
        bits = sign | 0x7F800000 | (man << 13);
    } else {
        bits = sign | ((exp + 112) << 23) | (man << 13);
    }
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
}

uint16_t U16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

}  // namespace

std::optional<BaseCalibration> ParseOotxV15(const uint8_t* d, size_t n) {
    // packed little-endian layout, 43 bytes
    if (n < 43) return std::nullopt;
    BaseCalibration c;
    size_t i = 0;
    auto h = [&]() { float v = HalfToFloat(U16(d + i)); i += 2; return (double)v; };
    c.firmware = U16(d + i), i += 2;
    c.id = (uint32_t)d[i] | ((uint32_t)d[i + 1] << 8) | ((uint32_t)d[i + 2] << 16) | ((uint32_t)d[i + 3] << 24), i += 4;
    c.axis[0].phase = h(), c.axis[1].phase = h();
    c.axis[0].tilt = h(), c.axis[1].tilt = h();
    c.sysUnlockCount = d[i++];
    c.ootxModel = d[i++];
    c.axis[0].curve = h(), c.axis[1].curve = h();
    for (int k = 0; k < 3; ++k) c.accelDir[k] = (int8_t)d[i++];
    c.axis[0].gibPhase = h(), c.axis[1].gibPhase = h();
    c.axis[0].gibMag = h(), c.axis[1].gibMag = h();
    c.mode = d[i++] & 0x7F;
    c.sysFaults = d[i++];
    c.axis[0].ogeePhase = h(), c.axis[1].ogeePhase = h();
    c.axis[0].ogeeMag = h(), c.axis[1].ogeeMag = h();
    c.nonce = U16(d + i), i += 2;
    c.raw.assign(d, d + n);
    return c;
}

namespace {

inline double Clamp1(double v) { return std::clamp(v, -1.0, 1.0); }

void CalSeries(double s, double& m, double& a) {
    const double f[6] = {-8.0108022e-06, 0.0028679863, 5.3685255000000001e-06, 0.0076069798000000001, 0, 0};
    m = f[0];
    a = 0;
    for (int i = 1; i < 6; ++i) {
        a = a * s + m;
        m = m * s + f[i];
    }
}

}  // namespace

double ReprojectAxis(const AxisCal& cal, const math::Vec3& p, int axis) {
    // libsurvive works with Z flipped (objects in front of the base have -Z).
    const double X = p.x, Y = p.y, Z = -p.z;
    const double B = std::atan2(Z, X);
    const double Ydeg = cal.tilt + (axis ? -1 : 1) * math::kPi / 6.0;
    const double tanA = std::tan(Ydeg);
    const double normXZ = std::sqrt(X * X + Z * Z);
    const double asinArg = Clamp1(tanA * Y / normXZ);
    const double sinYdeg = std::sin(Ydeg), cosYdeg = std::cos(Ydeg);
    const double sinPart = std::sin(B - std::asin(asinArg) + cal.ogeePhase) * cal.ogeeMag;
    const double normXYZ = std::sqrt(X * X + Y * Y + Z * Z);
    const double modAsinArg = Clamp1(Y / normXYZ / cosYdeg);
    const double asinOut = std::asin(modAsinArg);
    double mod, acc;
    CalSeries(asinOut, mod, acc);
    const double bCurved = sinPart + cal.curve;
    const double asinArg2 = Clamp1(asinArg + mod * bCurved / (cosYdeg - acc * bCurved * sinYdeg));
    const double asinOut2 = std::asin(asinArg2);
    const double sinOut2 = std::sin(B - asinOut2 + cal.gibPhase);
    return B - asinOut2 + sinOut2 * cal.gibMag - cal.phase - math::kPi / 2;
}

}  // namespace lhp::tracking::lighthouse
