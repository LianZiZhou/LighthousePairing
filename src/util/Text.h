#pragma once
#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lhp {

std::string ToUtf8(std::wstring_view w);
std::wstring ToWide(std::string_view s);

inline std::string_view Trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r' || s.front() == '\n')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) s.remove_suffix(1);
    return s;
}

inline bool StartsWith(std::string_view s, std::string_view p) { return s.size() >= p.size() && s.substr(0, p.size()) == p; }
inline bool Contains(std::string_view s, std::string_view p) { return s.find(p) != std::string_view::npos; }

inline bool IEquals(std::string_view a, std::string_view b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
               return std::tolower((unsigned char)x) == std::tolower((unsigned char)y);
           });
}

inline std::vector<std::string_view> Split(std::string_view s, char sep) {
    std::vector<std::string_view> out;
    size_t start = 0;
    for (size_t i = 0; i <= s.size(); ++i) {
        if (i == s.size() || s[i] == sep) {
            out.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    return out;
}

template <typename T>
std::optional<T> ParseNum(std::string_view s) {
    s = Trim(s);
    T v{};
    auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (r.ec != std::errc{}) return std::nullopt;
    return v;
}

/// Seconds since an arbitrary process-wide epoch (monotonic). Used for plots.
double NowSeconds();

/// Wall clock as unix seconds.
inline int64_t UnixNow() {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string FormatClock(int64_t unixSeconds);        // HH:MM:SS (local)
std::string FormatDateTime(int64_t unixSeconds);     // YYYY-MM-DD HH:MM:SS (local)
std::string FormatAgo(int64_t unixSeconds);          // "3 分钟前"
std::string FormatTimestampForFile();                // YYYYMMDD-HHMMSS

}  // namespace lhp
