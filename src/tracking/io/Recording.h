#pragma once
// L0: .lhrec recording files (see docs/TRACKING.md). Used for offline analysis, tests and replay.
#include "tracking/io/ReportSource.h"

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lhp::tracking::io {

class RecordingWriter {
public:
    static std::unique_ptr<RecordingWriter> Create(const std::string& path, std::string* error);
    ~RecordingWriter();
    void AddSource(const SourceInfo& info);
    void Write(const RawReport& r);
    void Marker(uint64_t hostUs, const std::string& text);
    size_t Count() const { return count_; }

private:
    RecordingWriter() = default;
    void Record(uint8_t type, uint64_t hostUs, uint16_t sourceId, const void* data, size_t len);
    std::FILE* f_ = nullptr;
    std::mutex mtx_;
    size_t count_ = 0;
};

struct Recording {
    std::vector<SourceInfo> sources;
    std::vector<RawReport> reports;     // in file order (= arrival order across sources)
    std::vector<std::pair<uint64_t, std::string>> markers;
};

/// Loads a whole recording. Returns false (with error) on a malformed file.
bool LoadRecording(const std::string& path, Recording& out, std::string* error);

/// Replays one source of a recording. realtime=true sleeps to reproduce the original timing.
std::unique_ptr<ReportSource> OpenReplay(std::shared_ptr<const Recording> rec, uint16_t sourceId, bool realtime);

}  // namespace lhp::tracking::io
