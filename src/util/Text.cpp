#include "util/Text.h"

#include <windows.h>

#include <cstdio>
#include <ctime>

namespace lhp {

std::string ToUtf8(std::wstring_view w) {
    if (w.empty()) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), nullptr, 0, nullptr, nullptr);
    std::string s(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), s.data(), n, nullptr, nullptr);
    return s;
}

std::wstring ToWide(std::string_view s) {
    if (s.empty()) return {};
    int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0);
    std::wstring w(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), w.data(), n);
    return w;
}

double NowSeconds() {
    static const auto t0 = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

static std::tm LocalTm(int64_t t) {
    std::tm tm{};
    time_t tt = (time_t)t;
    localtime_s(&tm, &tt);
    return tm;
}

std::string FormatClock(int64_t t) {
    if (t <= 0) return "--:--:--";
    auto tm = LocalTm(t);
    char buf[16];
    std::snprintf(buf, sizeof buf, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

std::string FormatDateTime(int64_t t) {
    if (t <= 0) return "—";
    auto tm = LocalTm(t);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

std::string FormatAgo(int64_t t) {
    if (t <= 0) return "从未";
    int64_t d = UnixNow() - t;
    char buf[48];
    if (d < 5) return "刚刚";
    if (d < 60) std::snprintf(buf, sizeof buf, "%lld 秒前", (long long)d);
    else if (d < 3600) std::snprintf(buf, sizeof buf, "%lld 分钟前", (long long)(d / 60));
    else if (d < 86400) std::snprintf(buf, sizeof buf, "%lld 小时前", (long long)(d / 3600));
    else std::snprintf(buf, sizeof buf, "%lld 天前", (long long)(d / 86400));
    return buf;
}

std::string FormatTimestampForFile() {
    auto tm = LocalTm(UnixNow());
    char buf[32];
    std::snprintf(buf, sizeof buf, "%04d%02d%02d-%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

}  // namespace lhp
