#include "tracking/io/ReportSource.h"

#include <windows.h>

#include <atomic>

namespace lhp::tracking::io {

uint64_t HostMicros() {
    static const LONGLONG freq = [] {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return f.QuadPart;
    }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (uint64_t)(c.QuadPart / freq * 1000000 + (c.QuadPart % freq) * 1000000 / freq);
}

namespace {

class LiveHidSource final : public ReportSource {
public:
    LiveHidSource(std::unique_ptr<hid::HidDevice> dev, uint16_t id) : dev_(std::move(dev)) {
        const auto& d = dev_->Info();
        info_.id = id;
        info_.vid = d.vid;
        info_.pid = d.pid;
        info_.interfaceNumber = d.interfaceNumber;
        info_.serial = d.serial;
        info_.product = d.product;
        info_.inputReportLength = d.inputReportLength;
        buf_.resize(std::max<size_t>(d.inputReportLength, 65));
    }
    const SourceInfo& Info() const override { return info_; }
    bool Read(RawReport& out, int timeoutMs) override {
        if (ended_) return false;
        int n = dev_->Read(buf_.data(), buf_.size(), timeoutMs);
        if (n == 0) return false;
        if (n < 0) {
            ended_ = true;
            return false;
        }
        out.hostUs = HostMicros();
        out.sourceId = info_.id;
        out.bytes.assign(buf_.begin(), buf_.begin() + n);
        return true;
    }
    bool Ended() const override { return ended_; }
    void Cancel() override { dev_->Cancel(); }
    hid::HidDevice* Writable() override { return dev_->GetAccess() == hid::Access::ReadWrite ? dev_.get() : nullptr; }

private:
    std::unique_ptr<hid::HidDevice> dev_;
    SourceInfo info_;
    std::vector<uint8_t> buf_;
    std::atomic<bool> ended_{false};
};

}  // namespace

std::unique_ptr<ReportSource> OpenLive(const hid::HidDeviceInfo& info, uint16_t sourceId, hid::Access access, std::string* error) {
    auto dev = hid::HidDevice::Open(info, access, error);
    if (!dev) return nullptr;
    return std::make_unique<LiveHidSource>(std::move(dev), sourceId);
}

}  // namespace lhp::tracking::io
