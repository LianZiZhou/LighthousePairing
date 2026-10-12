#include "tracking/io/Recording.h"

#include <windows.h>

#include <atomic>
#include <cstring>
#include <filesystem>
#include <thread>

namespace lhp::tracking::io {

namespace {

constexpr char kMagic[6] = {'L', 'H', 'R', 'E', 'C', '\0'};
constexpr uint16_t kVersion = 1;
enum : uint8_t { kReport = 1, kSource = 2, kMarker = 3 };

std::wstring Widen(const std::string& s) {
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), nullptr, 0);
    std::wstring w(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), w.data(), n);
    return w;
}

void PutString(std::vector<uint8_t>& b, const std::string& s) {
    uint16_t n = (uint16_t)std::min<size_t>(s.size(), 0xFFFF);
    b.push_back((uint8_t)n);
    b.push_back((uint8_t)(n >> 8));
    b.insert(b.end(), s.begin(), s.begin() + n);
}
bool GetString(const uint8_t*& p, const uint8_t* end, std::string& s) {
    if (end - p < 2) return false;
    uint16_t n = (uint16_t)(p[0] | (p[1] << 8));
    p += 2;
    if (end - p < n) return false;
    s.assign((const char*)p, n);
    p += n;
    return true;
}
template <typename T>
void Put(std::vector<uint8_t>& b, T v) {
    for (size_t i = 0; i < sizeof(T); ++i) b.push_back((uint8_t)((uint64_t)v >> (8 * i)));
}
template <typename T>
bool Get(const uint8_t*& p, const uint8_t* end, T& v) {
    if ((size_t)(end - p) < sizeof(T)) return false;
    uint64_t x = 0;
    for (size_t i = 0; i < sizeof(T); ++i) x |= (uint64_t)p[i] << (8 * i);
    v = (T)x;
    p += sizeof(T);
    return true;
}

}  // namespace

std::unique_ptr<RecordingWriter> RecordingWriter::Create(const std::string& path, std::string* error) {
    std::unique_ptr<RecordingWriter> w(new RecordingWriter());
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(Widen(path)).parent_path(), ec);
    w->f_ = _wfopen(Widen(path).c_str(), L"wb");
    if (!w->f_) {
        if (error) *error = "cannot create " + path;
        return nullptr;
    }
    std::fwrite(kMagic, 1, sizeof kMagic, w->f_);
    uint16_t hdr[2] = {kVersion, 0};
    std::fwrite(hdr, sizeof hdr, 1, w->f_);
    return w;
}

RecordingWriter::~RecordingWriter() {
    if (f_) std::fclose(f_);
}

void RecordingWriter::Record(uint8_t type, uint64_t hostUs, uint16_t sourceId, const void* data, size_t len) {
    std::scoped_lock lk(mtx_);
    std::vector<uint8_t> b;
    b.reserve(13 + len);
    b.push_back(type);
    Put<uint64_t>(b, hostUs);
    Put<uint16_t>(b, sourceId);
    Put<uint16_t>(b, (uint16_t)len);
    b.insert(b.end(), (const uint8_t*)data, (const uint8_t*)data + len);
    std::fwrite(b.data(), 1, b.size(), f_);
    ++count_;
}

void RecordingWriter::AddSource(const SourceInfo& s) {
    std::vector<uint8_t> b;
    Put<uint16_t>(b, s.vid);
    Put<uint16_t>(b, s.pid);
    Put<int16_t>(b, (int16_t)s.interfaceNumber);
    Put<uint16_t>(b, s.inputReportLength);
    PutString(b, s.serial);
    PutString(b, s.product);
    Record(kSource, HostMicros(), s.id, b.data(), b.size());
}

void RecordingWriter::Write(const RawReport& r) { Record(kReport, r.hostUs, r.sourceId, r.bytes.data(), r.bytes.size()); }

void RecordingWriter::Marker(uint64_t hostUs, const std::string& text) { Record(kMarker, hostUs, 0, text.data(), text.size()); }

bool LoadRecording(const std::string& path, Recording& out, std::string* error) {
    out = Recording{};
    std::FILE* f = _wfopen(Widen(path).c_str(), L"rb");
    if (!f) {
        if (error) *error = "cannot open " + path;
        return false;
    }
    std::vector<uint8_t> data;
    uint8_t chunk[65536];
    size_t n;
    while ((n = std::fread(chunk, 1, sizeof chunk, f)) > 0) data.insert(data.end(), chunk, chunk + n);
    std::fclose(f);
    if (data.size() < 10 || std::memcmp(data.data(), kMagic, sizeof kMagic) != 0) {
        if (error) *error = "not an .lhrec file";
        return false;
    }
    const uint8_t* p = data.data() + 10;
    const uint8_t* end = data.data() + data.size();
    while (p < end) {
        uint8_t type = *p++;
        uint64_t t;
        uint16_t src, len;
        if (!Get(p, end, t) || !Get(p, end, src) || !Get(p, end, len) || end - p < len) {
            if (error) *error = "truncated record";
            return !out.reports.empty();   // keep what was readable (e.g. a capture cut short)
        }
        const uint8_t* rec = p;
        p += len;
        if (type == kReport) {
            RawReport r;
            r.hostUs = t;
            r.sourceId = src;
            r.bytes.assign(rec, rec + len);
            out.reports.push_back(std::move(r));
        } else if (type == kSource) {
            SourceInfo s;
            s.id = src;
            const uint8_t* q = rec;
            int16_t iface = -1;
            if (Get(q, p, s.vid) && Get(q, p, s.pid) && Get(q, p, iface) && Get(q, p, s.inputReportLength) && GetString(q, p, s.serial) &&
                GetString(q, p, s.product)) {
                s.interfaceNumber = iface;
                out.sources.push_back(std::move(s));
            }
        } else if (type == kMarker) {
            out.markers.emplace_back(t, std::string((const char*)rec, len));
        }
    }
    return true;
}

namespace {

class ReplaySource final : public ReportSource {
public:
    ReplaySource(std::shared_ptr<const Recording> rec, uint16_t id, bool realtime) : rec_(std::move(rec)), realtime_(realtime) {
        for (auto& s : rec_->sources)
            if (s.id == id) info_ = s;
        info_.id = id;
        for (size_t i = 0; i < rec_->reports.size(); ++i)
            if (rec_->reports[i].sourceId == id) index_.push_back(i);
    }
    const SourceInfo& Info() const override { return info_; }
    bool Read(RawReport& out, int timeoutMs) override {
        if (pos_ >= index_.size() || cancelled_) {
            ended_ = true;
            return false;
        }
        const RawReport& r = rec_->reports[index_[pos_]];
        if (realtime_) {
            uint64_t now = HostMicros();
            if (pos_ == 0) {
                startHost_ = now;
                startRec_ = r.hostUs;
            }
            uint64_t due = startHost_ + (r.hostUs - startRec_);
            if (due > now) {
                uint64_t waitUs = due - now;
                if (timeoutMs >= 0 && waitUs > (uint64_t)timeoutMs * 1000) {
                    Sleep((DWORD)timeoutMs);
                    return false;
                }
                Sleep((DWORD)(waitUs / 1000));
            }
        }
        out = r;
        if (realtime_) out.hostUs = startHost_ + (r.hostUs - startRec_);
        ++pos_;
        return true;
    }
    bool Ended() const override { return ended_; }
    void Cancel() override { cancelled_ = true; }

private:
    std::shared_ptr<const Recording> rec_;
    SourceInfo info_;
    std::vector<size_t> index_;
    size_t pos_ = 0;
    bool realtime_;
    uint64_t startHost_ = 0, startRec_ = 0;
    std::atomic<bool> cancelled_{false};
    std::atomic<bool> ended_{false};
};

}  // namespace

std::unique_ptr<ReportSource> OpenReplay(std::shared_ptr<const Recording> rec, uint16_t sourceId, bool realtime) {
    return std::make_unique<ReplaySource>(std::move(rec), sourceId, realtime);
}

}  // namespace lhp::tracking::io
