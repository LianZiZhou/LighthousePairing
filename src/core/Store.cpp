#include "core/Store.h"

#include "util/Text.h"

#include <algorithm>

namespace lhp {

const char* ToString(ReceiverKind k) {
    switch (k) {
        case ReceiverKind::HmdBuiltin: return "头显内置";
        case ReceiverKind::VrRadio: return "外置 VR Radio";
        case ReceiverKind::WatchmanDongle: return "Watchman 接收器";
        case ReceiverKind::Wired: return "USB 直连设备";
        default: return "未知";
    }
}
const char* ToString(LinkState s) {
    switch (s) {
        case LinkState::Idle: return "无设备连接";
        case LinkState::Online: return "在线";
        case LinkState::Pairing: return "配对中";
        case LinkState::Busy: return "操作中";
        case LinkState::Error: return "错误";
        default: return "读取中";
    }
}
const char* ToString(PairingSource s) {
    switch (s) {
        case PairingSource::Device: return "设备读取";
        case PairingSource::Session: return "本次观察";
        case PairingSource::History: return "历史记录";
        default: return "未知";
    }
}
const char* ToString(DeviceClass c) {
    switch (c) {
        case DeviceClass::IndexController: return "Valve Index 手柄";
        case DeviceClass::ViveWand: return "HTC Vive 手柄";
        case DeviceClass::ViveTracker: return "VIVE 追踪器";
        case DeviceClass::TundraTracker: return "Tundra 追踪器";
        default: return "未知设备";
    }
}
const char* ToString(Hand h) {
    switch (h) {
        case Hand::Left: return "左手";
        case Hand::Right: return "右手";
        default: return "";
    }
}

std::string DeviceDisplayModel(const DeviceInfo& d) {
    std::string m = d.deviceClass != DeviceClass::Unknown ? ToString(d.deviceClass) : d.model;
    if (d.hand != Hand::Unknown) m += std::string("（") + ToString(d.hand) + "）";
    return m;
}

const char* ToString(MonitorState s) {
    switch (s) {
        case MonitorState::Starting: return "启动中";
        case MonitorState::Running: return "监控中";
        case MonitorState::Paused: return "已暂停";
        case MonitorState::Failed: return "失败";
        default: return "未监控";
    }
}

Receiver* AppState::Find(std::string_view serial) {
    for (auto& r : receivers)
        if (IEquals(r.serial, serial)) return &r;
    return nullptr;
}
const Receiver* AppState::Find(std::string_view serial) const {
    for (auto& r : receivers)
        if (IEquals(r.serial, serial)) return &r;
    return nullptr;
}

void Store::LogUnlocked(AppState& s, LogLevel level, std::string source, std::string text) {
    s.log.push_back({UnixNow(), level, std::move(source), std::move(text)});
    // Over capacity: drop the oldest *debug* line first so events/warnings survive much longer.
    while (s.log.size() > AppState::kMaxLog) {
        auto it = std::find_if(s.log.begin(), s.log.begin() + std::min<size_t>(s.log.size(), 2000),
                               [](const LogLine& l) { return l.level == LogLevel::Debug; });
        if (it != s.log.begin() + std::min<size_t>(s.log.size(), 2000)) s.log.erase(it);
        else s.log.pop_front();
    }
}

void Store::ToastUnlocked(AppState& s, Toast::Kind kind, std::string title, std::string text, float duration) {
    Toast t;
    t.kind = kind;
    t.title = std::move(title);
    t.text = std::move(text);
    t.createdT = NowSeconds();
    t.duration = duration;
    s.toasts.push_back(std::move(t));
    if (s.toasts.size() > 6) s.toasts.erase(s.toasts.begin());
}

void Store::Log(LogLevel level, std::string source, std::string text) {
    std::scoped_lock lk(mutex);
    LogUnlocked(state, level, std::move(source), std::move(text));
}

void Store::AddToast(Toast::Kind kind, std::string title, std::string text, float duration) {
    std::scoped_lock lk(mutex);
    ToastUnlocked(state, kind, std::move(title), std::move(text), duration);
}

void Store::PushSample(Receiver& r, const SignalSample& sample) {
    r.samples.push_back(sample);
    while (r.samples.size() > Receiver::kMaxSamples) r.samples.pop_front();
    r.lastSampleT = sample.t;
}

void Store::PushEvent(Receiver& r, SignalEvent ev) {
    switch (ev.type) {
        case SignalEvent::Type::Gap:
            ++r.gapCount;
            r.worstGap = std::max(r.worstGap, ev.value);
            r.pendingGap = std::max(r.pendingGap, ev.value);
            break;
        case SignalEvent::Type::Disconnected: ++r.disconnectCount; break;
        case SignalEvent::Type::Connected: ++r.connectCount; break;
        case SignalEvent::Type::TrackingReset: ++r.resetCount; break;
        default: break;
    }
    r.events.push_back(std::move(ev));
    while (r.events.size() > Receiver::kMaxEvents) r.events.pop_front();
}

void Store::SortReceivers(AppState& s) {
    auto rank = [](ReceiverKind k) {
        switch (k) {
            case ReceiverKind::HmdBuiltin: return 0;
            case ReceiverKind::VrRadio: return 1;
            case ReceiverKind::WatchmanDongle: return 2;
            case ReceiverKind::Wired: return 4;
            default: return 3;
        }
    };
    std::stable_sort(s.receivers.begin(), s.receivers.end(), [&](const Receiver& a, const Receiver& b) {
        if (rank(a.kind) != rank(b.kind)) return rank(a.kind) < rank(b.kind);
        return a.serial < b.serial;
    });
}

}  // namespace lhp
