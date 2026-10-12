#include "backend/SystemInfo.h"
#include "imgui_internal.h"
#include "implot.h"
#include "ui/App.h"
#include "ui/Icons.h"
#include "ui/Widgets.h"
#include "util/Text.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace lhp::ui {

namespace {

ImVec4 StateColor(const Receiver& r) {
    const auto& p = Pal();
    if (r.kind == ReceiverKind::Wired) return p.info;
    if (!r.usb.present) return p.textFaint;
    switch (r.state) {
        case LinkState::Online: return p.success;
        case LinkState::Pairing: return p.accent;
        case LinkState::Busy: return p.purple;
        case LinkState::Error: return p.danger;
        case LinkState::Idle: return p.textDim;
        default: return p.textFaint;
    }
}

const char* StateText(const Receiver& r) {
    if (r.kind == ReceiverKind::Wired) return "USB 直连";
    if (!r.usb.present) return r.seenByConsole ? "USB 未检测到" : "已拔出";
    return ToString(r.state);
}

ImVec4 KindColor(ReceiverKind k) {
    const auto& p = Pal();
    switch (k) {
        case ReceiverKind::HmdBuiltin: return p.purple;
        case ReceiverKind::VrRadio: return p.info;
        case ReceiverKind::WatchmanDongle: return p.accent;
        case ReceiverKind::Wired: return p.textDim;
        default: return p.textDim;
    }
}

/// Device line text: "Valve Index 手柄（左手）" when known, else raw model.
std::string ModelText(const DeviceInfo& d) {
    if (d.deviceClass != DeviceClass::Unknown) return DeviceDisplayModel(d);
    return d.model;
}

ImVec4 QualityColor(int q) {
    const auto& p = Pal();
    return q >= 4 ? p.success : q == 3 ? Mix(p.success, p.warning, 0.4f) : q == 2 ? p.warning : q == 1 ? p.danger : p.textFaint;
}

const char* QualityText(int q) { return q >= 4 ? "优秀" : q == 3 ? "良好" : q == 2 ? "一般" : q == 1 ? "差" : "—"; }

const char* BatteryIcon(const DeviceInfo& d) {
    if (d.charging.value_or(false)) return ICON_BATTERY_CHARGING;
    int v = d.batteryPct.value_or(0);
    return ICON_BATTERY(std::clamp((v + 5) / 10, 0, 10));
}

ImVec4 BatteryColor(int pct) {
    const auto& p = Pal();
    return pct <= 15 ? p.danger : pct <= 35 ? p.warning : p.success;
}

ImVec4 EventColor(SignalEvent::Type t) {
    const auto& p = Pal();
    switch (t) {
        case SignalEvent::Type::Gap: return p.warning;
        case SignalEvent::Type::Disconnected: return p.danger;
        case SignalEvent::Type::Connected: return p.success;
        case SignalEvent::Type::TrackingReset: return p.danger;
        case SignalEvent::Type::Bootstrapped: return p.info;
        case SignalEvent::Type::PairingSuccess: return p.success;
        case SignalEvent::Type::PairingFailed: return p.danger;
        case SignalEvent::Type::PairingStart: return p.accent;
        default: return p.textDim;
    }
}

const char* EventName(SignalEvent::Type t) {
    switch (t) {
        case SignalEvent::Type::Gap: return "丢包";
        case SignalEvent::Type::Disconnected: return "断开";
        case SignalEvent::Type::Connected: return "连接";
        case SignalEvent::Type::TrackingReset: return "追踪重置";
        case SignalEvent::Type::Bootstrapped: return "基站标定";
        case SignalEvent::Type::PairingStart: return "开始配对";
        case SignalEvent::Type::PairingSuccess: return "配对成功";
        case SignalEvent::Type::PairingFailed: return "配对失败";
        default: return "事件";
    }
}

int GapsWithin(const Receiver& r, double now, double window, float* worst = nullptr) {
    int n = 0;
    float w = 0;
    for (auto it = r.events.rbegin(); it != r.events.rend(); ++it) {
        if (now - it->t > window) break;
        if (it->type == SignalEvent::Type::Gap) {
            ++n;
            w = std::max(w, it->value);
        }
    }
    if (worst) *worst = w;
    return n;
}

/// Tiny sparkline drawn with the draw list (cheap enough for every card).
void Sparkline(const Receiver& r, double now, double window, const ImVec2& size, const ImVec4& color) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::Dummy(size);
    float maxV = 1;
    bool anyData = false;
    for (auto& s : r.samples)
        if (now - s.t <= window) {
            maxV = std::max(maxV, s.imuRate);
            anyData |= s.imuRate > 0;
        }
    if (!anyData) {
        // Dashed baseline = "no live data"
        float y = p0.y + size.y - 1;
        for (float x = p0.x; x < p0.x + size.x; x += 7)
            dl->AddLine(ImVec2(x, y), ImVec2(std::min(x + 3.5f, p0.x + size.x), y), Col(Pal().textFaint, 0.45f), 1.0f);
        return;
    }
    std::vector<ImVec2> pts;
    pts.reserve(r.samples.size());
    for (auto& s : r.samples) {
        double age = now - s.t;
        if (age > window) continue;
        float x = p0.x + size.x * (float)(1.0 - age / window);
        float y = p0.y + size.y - 1 - (size.y - 2) * (s.imuRate / maxV);
        pts.push_back(ImVec2(x, y));
        if (s.gapSeconds > 0) dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p0.y + size.y), Col(Pal().danger, 0.55f), 1.5f);
    }
    if (pts.size() >= 2) {
        // soft fill
        for (size_t i = 1; i < pts.size(); ++i)
            dl->AddQuadFilled(ImVec2(pts[i - 1].x, p0.y + size.y), pts[i - 1], pts[i], ImVec2(pts[i].x, p0.y + size.y), Col(color, 0.10f));
        dl->AddPolyline(pts.data(), (int)pts.size(), Col(color), 1.5f);
    } else {
        dl->AddLine(ImVec2(p0.x, p0.y + size.y - 1), ImVec2(p0.x + size.x, p0.y + size.y - 1), Col(Pal().textFaint, 0.4f), 1.0f);
    }
}

ImPlotSpec Line(const ImVec4& c, float w = 1.8f, ImPlotItemFlags f = 0) {
    ImPlotSpec s;
    s.LineColor = c;
    s.LineWeight = w;
    s.Flags = f;
    return s;
}

ImPlotSpec Fill(const ImVec4& c, float alpha, ImPlotItemFlags f = 0) {
    ImPlotSpec s;
    s.FillColor = c;
    s.FillAlpha = alpha;
    s.LineColor = c;
    s.Flags = f;
    return s;
}

int SecondsAgoFormatter(double value, char* buf, int size, void*) {
    double v = -value;
    if (v < 0.5) return std::snprintf(buf, size, "现在");
    if (v < 60) return std::snprintf(buf, size, "-%.0fs", v);
    return std::snprintf(buf, size, "-%.0fm%02.0fs", std::floor(v / 60), std::fmod(v, 60));
}

}  // namespace

// ============================================================================ receivers panel

void App::DrawReceivers(AppState& s) {
    const auto& p = Pal();
    ImGui::SetNextWindowSize(ImVec2(360, 600), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("接收器###receivers")) {
        ImGui::End();
        return;
    }
    // Search + filter
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##search", ICON_SEARCH "  搜索序列号 / 别名 / 设备", ui_.search, sizeof ui_.search);
    const char* filters[] = {"全部", "在线", "空闲", "需关注"};
    float segW = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 3) / 4;
    for (int i = 0; i < 4; ++i) {
        if (i) ImGui::SameLine();
        bool sel = ui_.filter == i;
        ImGui::PushStyleColor(ImGuiCol_Button, sel ? p.accentSoft : ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_Text, sel ? p.accent : p.textDim);
        if (ImGui::Button(filters[i], ImVec2(segW, 0))) ui_.filter = i;
        ImGui::PopStyleColor(2);
    }
    ImGui::Spacing();

    ImGui::BeginChild("##list", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_None);
    double now = NowSeconds();
    std::string q = ui_.search;
    for (auto& c : q) c = (char)std::toupper((unsigned char)c);
    ReceiverKind groups[] = {ReceiverKind::HmdBuiltin, ReceiverKind::VrRadio, ReceiverKind::WatchmanDongle, ReceiverKind::Unknown, ReceiverKind::Wired};
    bool any = false;
    for (auto g : groups) {
        std::vector<Receiver*> items;
        for (auto& r : s.receivers) {
            if (r.kind != g) continue;
            if (!q.empty()) {
                std::string hay = r.serial + " " + r.nickname + " " + r.pairedSerial + " " + r.pairedModel + " " + (r.device ? r.device->serial : "");
                for (auto& c : hay) c = (char)std::toupper((unsigned char)c);
                if (hay.find(q) == std::string::npos) continue;
            }
            if (ui_.filter == 1 && r.state != LinkState::Online) continue;
            if (ui_.filter == 2 && r.state == LinkState::Online) continue;
            if (ui_.filter == 3) {
                bool attention = r.monitor == MonitorState::Failed || !r.usb.present || r.usb.hasProblem || GapsWithin(r, now, 60) > 2 ||
                                 (r.device && r.device->batteryPct && *r.device->batteryPct < 20);
                if (!attention) continue;
            }
            items.push_back(&r);
        }
        if (items.empty()) continue;
        any = true;
        ImGui::PushStyleColor(ImGuiCol_Text, p.textDim);
        ImGui::PushFont(GetFonts().bold, ImGui::GetStyle().FontSizeBase * 0.9f);
        ImGui::TextUnformatted(Fmt("%s %s  ·  %d", g == ReceiverKind::HmdBuiltin ? ICON_HEADSET : g == ReceiverKind::VrRadio ? ICON_WIFI : g == ReceiverKind::Wired ? ICON_LINK : ICON_USB, ToString(g),
                                   (int)items.size())
                                   .c_str());
        ImGui::PopFont();
        ImGui::PopStyleColor();
        float w = ImGui::GetContentRegionAvail().x;
        for (auto* r : items) DrawReceiverCard(s, *r, w);
        ImGui::Spacing();
    }
    if (!any) {
        ImGui::Spacing();
        if (s.receivers.empty()) {
            if (s.refreshing || !s.backendReady) {
                ProgressRing(ImGui::GetFontSize(), 3, -1, p.accent);
                ImGui::SameLine();
                TextDim("正在扫描接收器…");
            } else {
                TextDim("没有发现任何 Lighthouse 接收器。");
                TextDim("请插入头显或 Valve 接收器后点击刷新。");
            }
        } else {
            TextDim("没有符合筛选条件的接收器。");
        }
    }
    ImGui::EndChild();
    ImGui::End();
}

void App::DrawReceiverCard(AppState& s, Receiver& r, float width) {
    const auto& p = Pal();
    double now = NowSeconds();
    float fs = ImGui::GetFontSize();
    const float pad = ImGui::GetStyle().WindowPadding.x;
    const float sparkH = fs * 0.9f;
    float h = pad * 0.8f * 2 + ImGui::GetTextLineHeightWithSpacing() * 3 + sparkH;
    ImGui::PushID(r.serial.c_str());
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::SetNextItemAllowOverlap();
    bool clicked = ImGui::InvisibleButton("card", ImVec2(width, h));
    bool hovered = ImGui::IsItemHovered();
    bool selected = ui_.selected == r.serial;
    if (clicked) {
        ui_.selected = r.serial;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) SetTab(DetailTab::Signal);
    if (ImGui::BeginPopupContextItem("ctx")) {
        ReceiverContextMenu(s, r);
        ImGui::EndPopup();
    }
    // Plain ImGui-style row block: flat background, 1px border, accent border when selected.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float rounding = ImGui::GetStyle().ChildRounding;
    ImVec4 bg = selected ? p.cardActive : hovered ? p.cardHover : p.card;
    ImVec2 a(pos.x, pos.y), b(pos.x + width, pos.y + h);
    dl->AddRectFilled(a, b, Col(bg), rounding);
    dl->AddRect(a, b, Col(selected ? Mix(p.border, p.accent, 0.8f) : p.border), rounding, 1.0f);
    ImVec4 stc = StateColor(r);

    ImGui::SetCursorScreenPos(ImVec2(a.x + pad, a.y + pad * 0.8f));
    ImGui::BeginGroup();
    // Row 1: dot + name + right side indicators
    StatusDot(stc, fs * 0.28f, r.state == LinkState::Online || r.state == LinkState::Pairing);
    ImGui::SameLine(0, fs * 0.35f);
    ImGui::PushFont(GetFonts().bold, 0.0f);
    ImGui::TextUnformatted(r.DisplayName());
    ImGui::PopFont();
    float rightX = a.x + width - pad;
    {
        // right: battery + signal bars
        std::string bat;
        if (r.device && r.device->batteryPct) bat = Fmt("%s %d%%", BatteryIcon(*r.device), *r.device->batteryPct);
        int q = SignalQuality(r, now);
        float barsW = fs * 0.9f;
        float batW = bat.empty() ? 0 : ImGui::CalcTextSize(bat.c_str()).x + fs * 0.5f;
        ImGui::SameLine(0, 0);
        ImGui::SetCursorScreenPos(ImVec2(rightX - barsW - batW, ImGui::GetCursorScreenPos().y));
        if (!bat.empty()) {
            TextColored2(BatteryColor(*r.device->batteryPct), "%s", bat.c_str());
            ImGui::SameLine(0, fs * 0.5f);
        }
        SignalBars(q, QualityColor(q), fs * 0.85f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("信号质量: %s（基于最近 60 秒丢包与抖动）", QualityText(q));
    }
    // Row 2: serial / kind / state
    ImGui::PushStyleColor(ImGuiCol_Text, p.textDim);
    if (!r.nickname.empty()) {
        ImGui::TextUnformatted(r.serial.c_str());
        ImGui::SameLine(0, fs * 0.4f);
        ImGui::TextUnformatted("·");
        ImGui::SameLine(0, fs * 0.4f);
    }
    ImGui::TextUnformatted(StateText(r));
    if (r.monitor == MonitorState::Failed) {
        ImGui::SameLine();
        TextColored2(p.warning, ICON_WARNING " 监控失败");
    }
    ImGui::PopStyleColor();
    // Row 3: device
    if (r.kind == ReceiverKind::Wired) {
        TextDim(ICON_INFO "  头显 / 有线设备的追踪模块，不是接收器");
    } else if (r.device && !r.device->serial.empty()) {
        TextColored2(p.text, ICON_GAME "  %s", r.device->serial.c_str());
        std::string m = ModelText(*r.device);
        if (!m.empty()) {
            ImGui::SameLine(0, fs * 0.4f);
            TextDim("%s", m.c_str());
        }
    } else if (r.state == LinkState::Online) {
        TextColored2(p.text, ICON_GAME "  设备在线");
        ImGui::SameLine(0, fs * 0.4f);
        TextDim("（序列号读取中）");
    } else if (!r.pairedSerial.empty()) {
        TextDim(ICON_HISTORY "  %s  %s · %s", r.pairedSerial.c_str(), r.pairedModel.c_str(), ToString(r.pairingSource));
    } else {
        ImGui::PushStyleColor(ImGuiCol_Text, p.textFaint);
        ImGui::TextUnformatted(ICON_LINK "  未知配对 / 空闲");
        ImGui::PopStyleColor();
    }
    // Row 4: sparkline + rate
    std::string rate = r.state == LinkState::Online ? Fmt("%.0f Hz", r.imu.rateHz) : "—";
    float rateW = fs * 4.2f;
    Sparkline(r, now, 60, ImVec2(width - pad * 2 - rateW, sparkH), r.state == LinkState::Online ? p.plotImu : p.textFaint);
    ImGui::SameLine(0, fs * 0.4f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() - fs * 0.05f);
    TextDim("%s", rate.c_str());
    ImGui::EndGroup();

    ImGui::SetCursorScreenPos(ImVec2(pos.x, pos.y + h + ImGui::GetStyle().ItemSpacing.y * 0.8f));
    ImGui::Dummy(ImVec2(0, 0));
    ImGui::PopID();
}

void App::ReceiverContextMenu(AppState& s, Receiver& r) {
    BoldText("%s", r.serial.c_str());
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_ADD "  配对新设备到此接收器")) OpenWizard(r.serial);
    {
        DeviceClass cls = r.device ? r.device->deviceClass : r.pairedClass;
        const char* label = CanVibrate(cls) ? ICON_VIBRATE "  震动识别" : IsTracker(cls) ? ICON_SYNC "  摇一摇识别" : ICON_SEARCH "  识别设备";
        if (ImGui::MenuItem(label, nullptr, false, r.state == LinkState::Online)) IdentifyDevice(r);
    }
    if (r.monitor == MonitorState::Running || r.monitor == MonitorState::Starting) {
        if (ImGui::MenuItem(ICON_PAUSE "  停止监控")) Post({Command::Type::StopMonitor, r.serial});
    } else if (ImGui::MenuItem(ICON_PLAY "  开始监控")) {
        Post({Command::Type::StartMonitor, r.serial});
    }
    if (ImGui::MenuItem(ICON_COPY "  复制序列号")) sys::CopyToClipboard(r.serial);
    if (ImGui::MenuItem(ICON_CHART "  查看实时信号")) {
        ui_.selected = r.serial;
        SetTab(DetailTab::Signal);
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_LINK "  解除此接收器的配对…")) OpenDanger(Command::Type::Unpair, r.serial);
    if (s.settings.expertMode && ImGui::MenuItem(ICON_DELETE "  恢复出厂设置…")) OpenDanger(Command::Type::FactoryReset, r.serial);
}

// ============================================================================ detail panel

void App::DrawDetail(AppState& s) {
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("详情###detail")) {
        ImGui::End();
        return;
    }
    Receiver* r = s.Find(ui_.selected);
    if (!r) {
        ImGui::Spacing();
        TextDim("在左侧选择一个接收器查看详情。");
        ImGui::End();
        return;
    }
    DrawDetailHeader(s, *r);
    ImGui::Spacing();
    if (ImGui::BeginTabBar("##detailtabs", ImGuiTabBarFlags_None)) {
        struct TabDef {
            DetailTab t;
            const char* label;
        } tabs[] = {
            {DetailTab::Overview, ICON_HOME "  概览###t0"},     {DetailTab::Signal, ICON_CHART "  实时信号###t1"}, {DetailTab::Device, ICON_GAME "  设备信息###t2"},
            {DetailTab::Usb, ICON_USB "  USB 拓扑###t3"},       {DetailTab::History, ICON_HISTORY "  历史与备注###t4"}, {DetailTab::Console, ICON_CMD "  控制台###t5"},
        };
        for (auto& t : tabs) {
            ImGuiTabItemFlags f = (ui_.forceTab && ui_.tab == t.t) ? ImGuiTabItemFlags_SetSelected : 0;
            if (ImGui::BeginTabItem(t.label, nullptr, f)) {
                if (!ui_.forceTab) ui_.tab = t.t;
                ImGui::Spacing();
                ImGui::BeginChild("##tabbody", ImVec2(0, 0), ImGuiChildFlags_None);
                switch (t.t) {
                    case DetailTab::Overview: DrawTabOverview(s, *r); break;
                    case DetailTab::Signal: DrawTabSignal(s, *r); break;
                    case DetailTab::Device: DrawTabDevice(s, *r); break;
                    case DetailTab::Usb: DrawTabUsb(s, *r); break;
                    case DetailTab::History: DrawTabHistory(s, *r); break;
                    case DetailTab::Console: DrawTabConsole(s, *r); break;
                    default: break;
                }
                ImGui::EndChild();
                ImGui::EndTabItem();
            }
        }
        ui_.forceTab = false;
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void App::DrawDetailHeader(AppState& s, Receiver& r) {
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    ImGui::BeginGroup();
    BigText(1.55f, "%s", r.DisplayName());
    ImGui::SameLine(0, fs * 0.6f);
    // Centre the chip row on the big title line.
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (fs * BigScale(1.55f) - ImGui::GetFrameHeight()) * 0.5f);
    Chip(ToString(r.kind), KindColor(r.kind));
    ChipSameLine(StateText(r), StateColor(r), r.state == LinkState::Online);
    ImVec4 mc = r.monitor == MonitorState::Running ? p.info : r.monitor == MonitorState::Failed ? p.warning : p.textFaint;
    ChipSameLine(ToString(r.monitor), mc);
    if (r.usb.hasProblem) ChipSameLine(Fmt("设备管理器问题 #%u", r.usb.problemCode).c_str(), p.danger);
    TextDim("%s  ·  %s  ·  射频版本 %u", r.serial.c_str(), r.pid == 0x2102 ? "28DE:2102 Valve VR Radio" : r.pid == 0x2101 ? "28DE:2101 Watchman Dongle" : "USB 未识别",
            r.radioVersion);
    if (r.usb.behindHmd) {
        ImGui::SameLine();
        TextDim("·  位于 %s", r.usb.hmdName.c_str());
    }
    ImGui::EndGroup();

    if (r.kind == ReceiverKind::Wired) {
        ImGui::Spacing();
        TextDim(ICON_INFO "  这是通过 USB 直接连接的追踪设备（通常是头显自身的 Lighthouse 模块）。");
        TextDim("lighthouse_console 会把它和接收器列在一起，但它不能配对其他设备，本工具也不会对它打开监控会话。");
        return;
    }

    // Action bar
    ImGui::Spacing();
    bool busy = s.op.Active();
    ImGui::BeginDisabled(busy);
    if (AccentButton(ICON_ADD "  配对新设备")) OpenWizard(r.serial);
    ImGui::SameLine();
    // Identify: controllers vibrate, trackers (VIVE / Tundra) have no motor -> shake-to-identify.
    DeviceClass cls = r.device ? r.device->deviceClass : r.pairedClass;
    ImGui::BeginDisabled(r.state != LinkState::Online);
    if (CanVibrate(cls)) {
        if (ImGui::Button(ICON_VIBRATE "  震动识别")) Post({Command::Type::Identify, r.serial});
        ImGui::SetItemTooltip("让连接在此接收器上的%s震动，用来确认是哪一只", ToString(cls));
    } else if (IsTracker(cls)) {
        if (ImGui::Button(ICON_SYNC "  摇一摇识别")) OpenIdentify(r.serial);
        ImGui::SetItemTooltip("%s没有震动马达。拿起追踪器摇晃，界面会实时标出是哪个接收器上的设备在动。", ToString(cls));
    } else {
        if (ImGui::Button(ICON_SEARCH "  识别设备")) OpenIdentify(r.serial);
        ImGui::SetItemTooltip("设备类型未知：可以摇晃设备识别，也可以在弹窗里尝试震动");
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_POWER "  关机")) Post({Command::Type::PowerOff, r.serial});
    ImGui::SetItemTooltip(IsTracker(cls) ? "关闭此接收器上的追踪器：指示灯熄灭的那个就是它（按电源键可重新开机）"
                                         : "关闭当前连接在此接收器上的设备");
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (r.monitor == MonitorState::Running || r.monitor == MonitorState::Starting) {
        if (ImGui::Button(ICON_PAUSE "  停止监控")) Post({Command::Type::StopMonitor, r.serial});
    } else {
        if (ImGui::Button(ICON_PLAY "  开始监控")) Post({Command::Type::StartMonitor, r.serial});
    }
    ImGui::SameLine();
    if (DangerButton(ICON_LINK "  解除配对…")) OpenDanger(Command::Type::Unpair, r.serial);
    ImGui::SameLine();
    if (ImGui::Button(ICON_CHEVRON_D "  更多")) ImGui::OpenPopup("more");
    if (ImGui::BeginPopup("more")) {
        if (ImGui::MenuItem(ICON_COPY "  复制序列号")) sys::CopyToClipboard(r.serial);
        if (ImGui::MenuItem(ICON_COPY "  复制 USB 实例路径", nullptr, false, !r.usb.instanceId.empty())) sys::CopyToClipboard(r.usb.instanceId);
        if (ImGui::MenuItem(ICON_CLEAR "  清空此接收器的统计")) Post({Command::Type::ClearStats, r.serial});
        ImGui::Separator();
        if (ImGui::MenuItem(ICON_DELETE "  恢复出厂设置…", nullptr, false, s.settings.expertMode)) OpenDanger(Command::Type::FactoryReset, r.serial);
        if (!s.settings.expertMode) ImGui::SetItemTooltip("需要在设置中开启专家模式");
        ImGui::EndPopup();
    }
    ImGui::EndDisabled();
    if (busy) {
        ImGui::SameLine();
        TextDim("（有操作正在进行）");
    }
}

void App::DrawTabOverview(AppState& s, Receiver& r) {
    const auto& p = Pal();
    double now = NowSeconds();
    float fs = ImGui::GetFontSize();
    float avail = ImGui::GetContentRegionAvail().x;
    int cols = avail > fs * 50 ? 5 : avail > fs * 30 ? 3 : 2;
    float tileW = (avail - ImGui::GetStyle().ItemSpacing.x * (cols - 1)) / cols;
    bool online = r.state == LinkState::Online;
    float worst1m = 0;
    int gaps1m = GapsWithin(r, now, 60, &worst1m);
    int q = SignalQuality(r, now);

    auto tile = [&](int i, const char* id, const char* cap, std::string val, const char* unit, ImVec4 col, const char* hint) {
        if (i % cols) ImGui::SameLine();
        MetricTile(id, cap, val.c_str(), unit, col, tileW, hint);
    };
    int i = 0;
    tile(i++, "q", "信号质量", online ? QualityText(q) : "—", "", QualityColor(q), "综合最近 60 秒丢包次数、最长丢包和 IMU 抖动");
    tile(i++, "imu", "IMU 采样率", online ? Fmt("%.0f", r.imu.rateHz) : "—", "Hz", p.plotImu, "设备通过无线发送的 IMU 数据包速率（imustats）。持续下降代表无线丢包。");
    tile(i++, "sig", "时序抖动 σ", online ? Fmt("%.2f", r.imu.sigmaMs) : "—", "ms", p.plotJitter, "IMU 数据包间隔的标准差，越小越稳定。");
    tile(i++, "gap", "最近 1 分钟丢包", Fmt("%d", gaps1m), worst1m > 0 ? Fmt("最长 %.2fs", worst1m).c_str() : "次", gaps1m ? p.warning : p.success,
         "来自 keepalive 日志 \"Packet received after Xs\"");
    std::string bat = r.device && r.device->batteryPct ? Fmt("%d", *r.device->batteryPct) : "—";
    tile(i++, "bat", "设备电量", bat, r.device && r.device->batteryPct ? "%" : "", r.device && r.device->batteryPct ? BatteryColor(*r.device->batteryPct) : p.textFaint,
         r.device ? r.device->batteryText.c_str() : nullptr);
    ImGui::Spacing();
    ImGui::Spacing();

    float colW = (avail - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    bool twoCol = avail > fs * 40;
    // --- Paired device card
    if (BeginCard("##devcard", ImVec2(twoCol ? colW : 0, 0))) {
        SectionTitle(ICON_GAME, "配对的设备");
        ImGui::Spacing();
        if (BeginPropTable("##dev")) {
            if (r.device) {
                PropRow("序列号", r.device->serial.empty() ? "（读取中）" : r.device->serial, true, &p.success);
                PropRow("型号", r.device->model);
                PropRow("设备类型", ToString(r.device->deviceClass));
                if (r.device->deviceClass == DeviceClass::IndexController)
                    PropRow("左 / 右手", r.device->hand == Hand::Unknown ? "未能判断" : ToString(r.device->hand));
                PropRow("识别方式", CanVibrate(r.device->deviceClass) ? "震动（有触觉马达）"
                                    : IsTracker(r.device->deviceClass) ? "摇一摇 / 关机看指示灯（追踪器没有震动马达）"
                                                                       : "类型未知：先试摇一摇");
                PropRow("固件", r.device->firmware);
                PropRow("硬件版本", r.device->hardware);
                PropRow("电量", r.device->batteryPct ? Fmt("%d%%%s", *r.device->batteryPct, r.device->charging.value_or(false) ? "（充电中）" : "") : r.device->batteryText);
                PropRow("连接时长", r.device->connectedSince ? Fmt("%lld 分钟", (long long)((UnixNow() - r.device->connectedSince) / 60)) : "");
                PropRow("配对信息来源", ToString(r.pairingSource));
            } else if (!r.pairedSerial.empty()) {
                ImVec4 dim = p.textDim;
                PropRow("序列号", r.pairedSerial, true, &dim);
                PropRow("型号", r.pairedModel);
                PropRow("状态", "离线");
                PropRow("最后在线", r.pairedLastSeen ? FormatDateTime(r.pairedLastSeen) + "（" + FormatAgo(r.pairedLastSeen) + "）" : "");
                PropRow("信息来源", ToString(r.pairingSource));
            } else {
                PropRow("状态", "当前没有设备在线，也没有历史记录");
                PropRow("提示", "打开设备电源；若它已配对到此接收器，几秒内会出现在这里");
            }
            EndPropTable();
        }
    }
    EndCard();
    if (twoCol) ImGui::SameLine();
    // --- Receiver card
    if (BeginCard("##rxcard", ImVec2(twoCol ? colW : 0, 0))) {
        SectionTitle(ICON_WIFI, "接收器");
        ImGui::Spacing();
        if (BeginPropTable("##rx")) {
            PropRow("序列号", r.serial, true);
            PropRow("类型", std::string(ToString(r.kind)) + (r.kindOverridden ? "（手动指定）" : ""));
            PropRow("射频版本", r.radioVersion ? std::to_string(r.radioVersion) : "");
            std::string extra;
            for (auto& e : r.dongleInfoExtra) extra += (extra.empty() ? "" : ", ") + e;
            PropRow("dongleinfo 附加字段", extra.empty() ? "（空）" : extra);
            PropRow("USB 位置", r.usb.locationInfo + (r.usb.address ? Fmt("（端口 %u）", r.usb.address) : ""));
            PropRow("所在头显", r.usb.behindHmd ? r.usb.hmdName : "否（外置）");
            PropRow("首次发现", FormatDateTime(r.firstSeen));
            PropRow("监控", std::string(ToString(r.monitor)) + (r.monitorError.empty() ? "" : " · " + r.monitorError));
            EndPropTable();
        }
    }
    EndCard();

    ImGui::Spacing();
    // --- Recent events
    if (BeginCard("##evcard", ImVec2(0, 0))) {
        SectionTitle(ICON_HISTORY, "最近事件", Fmt("连接 %u · 断开 %u · 丢包 %u · 追踪重置 %u", r.connectCount, r.disconnectCount, r.gapCount, r.resetCount).c_str());
        ImGui::Spacing();
        if (r.events.empty()) TextDim("暂无事件。监控运行时，丢包、连接/断开、追踪重置等会实时出现在这里。");
        int shown = 0;
        for (auto it = r.events.rbegin(); it != r.events.rend() && shown < 8; ++it, ++shown) {
            ImGui::AlignTextToFramePadding();   // text shares the chip's frame-height line
            TextDim("%s", FormatClock(it->unix).c_str());
            ImGui::SameLine();
            Chip(EventName(it->type), EventColor(it->type));
            ImGui::SameLine();
            ImGui::TextUnformatted(it->text.c_str());
        }
    }
    EndCard();
}

void App::DrawTabSignal(AppState& s, Receiver& r) {
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    double now = ui_.signalPaused ? ui_.pausedAt : NowSeconds();

    // Controls
    const char* windows[] = {"1 分钟", "5 分钟", "15 分钟"};
    int winS[] = {60, 300, 900};
    int wi = ui_.signalWindowS == 60 ? 0 : ui_.signalWindowS == 300 ? 1 : 2;
    ImGui::SetNextItemWidth(fs * 7);
    if (ImGui::Combo("时间窗口", &wi, windows, 3)) ui_.signalWindowS = winS[wi];
    ImGui::SameLine();
    if (ImGui::Button(ui_.signalPaused ? ICON_PLAY "  继续" : ICON_PAUSE "  暂停画面")) {
        ui_.signalPaused = !ui_.signalPaused;
        ui_.pausedAt = NowSeconds();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_CLEAR "  清空统计")) Post({Command::Type::ClearStats, r.serial});
    ImGui::SameLine();
    TextDim("每 %d ms 采样一次 · 数据来自 imustats / usbstats / keepalive 日志", s.settings.pollIntervalMs);
    if (r.monitor != MonitorState::Running) {
        ImGui::Spacing();
        TextColored2(p.warning, ICON_WARNING "  此接收器当前未在监控（%s）。", ToString(r.monitor));
        ImGui::SameLine();
        if (ImGui::SmallButton("开始监控")) Post({Command::Type::StartMonitor, r.serial});
        if (!r.monitorError.empty()) TextDim("%s", r.monitorError.c_str());
    }
    ImGui::Spacing();

    double window = ui_.signalWindowS;
    std::vector<double> xs, imu, interval, sigma, usb, usbMax;
    std::vector<double> gx, gy;
    double sumImu = 0, minImu = 1e9, maxImu = 0;
    int n = 0;
    for (auto& smp : r.samples) {
        double age = now - smp.t;
        if (age > window || age < -1) continue;
        xs.push_back(-age);
        imu.push_back(smp.imuRate);
        interval.push_back(smp.imuIntervalMs);
        sigma.push_back(smp.imuSigmaMs);
        usb.push_back(smp.usbRate);
        usbMax.push_back(smp.usbMaxIntervalMs);
        if (smp.imuRate > 0) {
            sumImu += smp.imuRate;
            minImu = std::min(minImu, (double)smp.imuRate);
            maxImu = std::max(maxImu, (double)smp.imuRate);
            ++n;
        }
    }
    int gaps = 0, disc = 0, resets = 0;
    float worst = 0;
    double gapTotal = 0;
    for (auto& e : r.events) {
        double age = now - e.t;
        if (age > window || age < -1) continue;
        if (e.type == SignalEvent::Type::Gap) {
            gx.push_back(-age);
            gy.push_back(e.value);
            ++gaps;
            worst = std::max(worst, e.value);
            gapTotal += e.value;
        }
        if (e.type == SignalEvent::Type::Disconnected) ++disc;
        if (e.type == SignalEvent::Type::TrackingReset) ++resets;
    }

    // Stats strip
    float avail = ImGui::GetContentRegionAvail().x;
    int cols = 6;
    float tileW = (avail - ImGui::GetStyle().ItemSpacing.x * (cols - 1)) / cols;
    MetricTile("avg", "平均 IMU", n ? Fmt("%.0f", sumImu / n).c_str() : "—", "Hz", p.plotImu, tileW, n ? Fmt("最低 %.0f Hz / 最高 %.0f Hz", minImu, maxImu).c_str() : nullptr);
    ImGui::SameLine();
    MetricTile("gaps", "丢包次数", Fmt("%d", gaps).c_str(), Fmt("%.1f 次/分", gaps / (window / 60.0)).c_str(), gaps ? p.warning : p.success, tileW);
    ImGui::SameLine();
    MetricTile("worst", "最长丢包", Fmt("%.2f", worst).c_str(), "秒", worst > 1 ? p.danger : worst > 0.3f ? p.warning : p.success, tileW);
    ImGui::SameLine();
    MetricTile("loss", "丢包总时长", Fmt("%.1f", gapTotal).c_str(), "秒", p.plotGap, tileW, "窗口内所有 keepalive 间隔之和");
    ImGui::SameLine();
    MetricTile("disc", "断开", Fmt("%d", disc).c_str(), "次", disc ? p.danger : p.success, tileW);
    ImGui::SameLine();
    MetricTile("rst", "追踪重置", Fmt("%d", resets).c_str(), "次", resets ? p.danger : p.success, tileW, "需要在设置中开启“追踪诊断”才会产生（pose 求解）");
    ImGui::Spacing();

    if (r.state != LinkState::Online && gx.empty() && sumImu == 0) {
        TextColored2(p.textDim, ICON_INFO "  当前没有设备连接在此接收器上。打开已配对到它的设备后，下面会实时显示 IMU 速率、抖动和丢包。");
        ImGui::Spacing();
    }
    float plotH = std::max(fs * 9.f, (ImGui::GetContentRegionAvail().y - fs * 14) / 3);
    ImPlotAxisFlags xflags = ImPlotAxisFlags_NoHighlight;
    auto setupX = [&] {
        ImPlot::SetupAxis(ImAxis_X1, nullptr, xflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, -window, 0, ImPlotCond_Always);
        ImPlot::SetupAxisFormat(ImAxis_X1, SecondsAgoFormatter, nullptr);
    };
    int cnt = (int)xs.size();
    if (ImPlot::BeginPlot("##imu", ImVec2(-1, plotH), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
        setupX();
        ImPlot::SetupAxis(ImAxis_Y1, "IMU Hz", ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
        ImPlot::SetupAxis(ImAxis_Y2, "丢包 s", ImPlotAxisFlags_AuxDefault | ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
        if (cnt) {
            ImPlot::PlotShaded("IMU 速率", xs.data(), imu.data(), cnt, 0.0, Fill(p.plotImu, 0.18f));
            ImPlot::PlotLine("IMU 速率", xs.data(), imu.data(), cnt, Line(p.plotImu, 2.0f * ImGui::GetStyle().FontScaleDpi));
        }
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
        if (!gx.empty()) {
            ImPlotSpec st = Line(p.plotGap, 2.0f);
            st.Marker = ImPlotMarker_Circle;
            st.MarkerSize = 3.5f;
            st.MarkerFillColor = p.plotGap;
            ImPlot::PlotStems("丢包", gx.data(), gy.data(), (int)gx.size(), 0.0, st);
        }
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##jit", ImVec2(-1, plotH * 0.85f), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
        setupX();
        ImPlot::SetupAxis(ImAxis_Y1, "ms", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
        if (cnt) {
            ImPlot::PlotLine("平均间隔", xs.data(), interval.data(), cnt, Line(p.info, 1.6f));
            ImPlot::PlotShaded("抖动 σ", xs.data(), sigma.data(), cnt, 0.0, Fill(p.plotJitter, 0.22f));
            ImPlot::PlotLine("抖动 σ", xs.data(), sigma.data(), cnt, Line(p.plotJitter, 1.6f));
        }
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##usb", ImVec2(-1, plotH * 0.85f), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
        setupX();
        ImPlot::SetupAxis(ImAxis_Y1, "包/秒", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y2, "最大间隔 ms", ImPlotAxisFlags_AuxDefault | ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
        if (cnt) {
            ImPlot::PlotLine("USB 包率", xs.data(), usb.data(), cnt, Line(p.plotUsb, 1.8f));
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            ImPlot::PlotLine("USB 最大间隔", xs.data(), usbMax.data(), cnt, Line(Mix(p.plotGap, p.plotJitter, 0.5f), 1.2f));
        }
        ImPlot::EndPlot();
    }

    // USB interface table
    if (!r.usbRates.empty()) {
        ImGui::Spacing();
        if (ImGui::BeginTable("##usbrates", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("VID:PID");
            ImGui::TableSetupColumn("序列号");
            ImGui::TableSetupColumn("接口");
            ImGui::TableSetupColumn("包/秒");
            ImGui::TableSetupColumn("最大间隔");
            ImGui::TableHeadersRow();
            for (auto& u : r.usbRates) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(u.vidpid.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(u.serial.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(u.iface.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%.0f", u.perSec);
                ImGui::TableNextColumn();
                ImGui::Text("%.0f ms", u.maxIntervalMs);
            }
            ImGui::EndTable();
        }
        if (r.imu.count) TextDim("IMU 累计样本 %llu · 重力 %.2f m/s² (σ %.3f)", (unsigned long long)r.imu.count, r.imu.grav, r.imu.gravSigma);
    }

    // Event log
    ImGui::Spacing();
    SectionTitle(ICON_HISTORY, "事件");
    ImGui::SameLine(0, fs);
    const char* ef[] = {"全部", "丢包", "连接/断开", "追踪"};
    for (int k = 0; k < 4; ++k) {
        ImGui::SameLine();
        if (ImGui::RadioButton(ef[k], ui_.eventFilter == k)) ui_.eventFilter = k;
    }
    if (ImGui::BeginTable("##events", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0, std::max(fs * 8, ImGui::GetContentRegionAvail().y)))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("时间", ImGuiTableColumnFlags_WidthFixed, fs * 5.5f);
        ImGui::TableSetupColumn("类型", ImGuiTableColumnFlags_WidthFixed, fs * 6);
        ImGui::TableSetupColumn("说明", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (auto it = r.events.rbegin(); it != r.events.rend(); ++it) {
            auto t = it->type;
            if (ui_.eventFilter == 1 && t != SignalEvent::Type::Gap) continue;
            if (ui_.eventFilter == 2 && t != SignalEvent::Type::Connected && t != SignalEvent::Type::Disconnected) continue;
            if (ui_.eventFilter == 3 && t != SignalEvent::Type::TrackingReset && t != SignalEvent::Type::Bootstrapped) continue;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            TextDim("%s", FormatClock(it->unix).c_str());
            ImGui::TableNextColumn();
            TextColored2(EventColor(t), "%s", EventName(t));
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(it->text.c_str());
        }
        ImGui::EndTable();
    }
}

void App::DrawTabDevice(AppState& s, Receiver& r) {
    const auto& p = Pal();
    if (!r.device) {
        TextDim("当前没有设备连接在此接收器上。");
        if (!r.pairedSerial.empty()) TextDim("最近一次见到的设备：%s %s（%s）", r.pairedSerial.c_str(), r.pairedModel.c_str(), FormatAgo(r.pairedLastSeen).c_str());
        return;
    }
    DeviceInfo& d = *r.device;
    if (BeginCard("##devsum", ImVec2(0, 0))) {
        BigText(1.3f, ICON_GAME "  %s", d.serial.empty() ? "（序列号读取中）" : d.serial.c_str());
        // Everything on this row is centred on the 1.3x title line.
        const float fsz = ImGui::GetFontSize();
        const float lineH = fsz * BigScale(1.3f);
        auto centerOnLine = [&](float itemH) { ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (lineH - itemH) * 0.5f); };
        if (!d.model.empty()) {
            ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x * 2);
            centerOnLine(ImGui::GetFrameHeight());
            Chip(d.model.c_str(), p.accent);
        }
        if (d.batteryPct) {
            ImGui::SameLine(0, fsz);
            centerOnLine(ImGui::GetTextLineHeight());
            TextColored2(BatteryColor(*d.batteryPct), "%s %d%%", BatteryIcon(d), *d.batteryPct);
            ImGui::SameLine();
            const float meterH = fsz * 0.45f;
            centerOnLine(meterH);
            Meter(*d.batteryPct / 100.f, BatteryColor(*d.batteryPct), ImVec2(fsz * 6, meterH));
        }
        TextDim("连接在 %s · 已连接 %s", r.serial.c_str(), d.connectedSince ? FormatAgo(d.connectedSince).c_str() : "—");
        ImGui::SameLine();
        if (ImGui::SmallButton(ICON_REFRESH " 重新读取")) Post({Command::Type::RawCommand, r.serial, "deviceinfo"});
        ImGui::SameLine();
        if (ImGui::SmallButton(ICON_COPY " 复制全部")) {
            std::string all = "Receiver: " + r.serial + "\n";
            for (auto& [k, v] : d.props) all += (k.empty() ? "" : k + ": ") + v + "\n";
            for (auto& l : d.rawVersion) all += l + "\n";
            all += "Battery: " + d.batteryText + "\n";
            sys::CopyToClipboard(all);
            store_.ToastUnlocked(s, Toast::Kind::Success, "已复制", "设备信息已复制到剪贴板");
        }
    }
    EndCard();
    ImGui::Spacing();
    SectionTitle(ICON_INFO, "全部属性", "（deviceinfo / version 原始解析）");
    if (BeginPropTable("##props", ImGui::GetFontSize() * 11)) {
        PropRow("序列号", d.serial, true);
        PropRow("型号", d.model);
        PropRow("固件", d.firmware, true);
        PropRow("硬件版本", d.hardware, true);
        PropRow("电量原文", d.batteryText);
        for (auto& [k, v] : d.props)
            if (!k.empty()) PropRow(k.c_str(), v, true);
        EndPropTable();
    }
    auto rawBlock = [&](const char* title, const std::vector<std::string>& lines) {
        if (lines.empty()) return;
        ImGui::Spacing();
        if (ImGui::CollapsingHeader(title)) {
            ImGui::PushFont(GetFonts().mono, 0.0f);
            for (auto& l : lines) ImGui::TextUnformatted(l.c_str());
            ImGui::PopFont();
        }
    };
    rawBlock("deviceinfo 原始输出", d.rawDeviceInfo);
    rawBlock("version 原始输出", d.rawVersion);
    std::vector<std::string> free;
    for (auto& [k, v] : d.props)
        if (k.empty()) free.push_back(v);
    rawBlock("其他输出行", free);
}

void App::DrawTabUsb(AppState& s, Receiver& r) {
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (!r.usb.present) {
        TextColored2(p.warning, ICON_WARNING "  当前在 USB 上没有找到这个接收器。");
        TextDim("它可能已被拔出，或者只出现在 lighthouse_console 的列表中（例如通过其他方式连接）。");
        return;
    }
    // Chain visualisation: controller -> hubs -> receiver
    SectionTitle(ICON_USB, "连接链路");
    ImGui::Spacing();
    std::vector<UsbNode> chain(r.usb.parents.rbegin(), r.usb.parents.rend());
    UsbNode self{r.usb.instanceId, r.usb.friendlyName.empty() ? r.serial : r.usb.friendlyName + "  (" + r.serial + ")", 0x28DE, r.pid};
    chain.push_back(self);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (size_t i = 0; i < chain.size(); ++i) {
        const auto& n = chain[i];
        bool last = i + 1 == chain.size();
        bool hmd = n.vid == 0x34A4 || n.vid == 0x0BB4 || (n.vid == 0x28DE && n.pid != 0x2101 && n.pid != 0x2102);
        ImGui::Indent(fs * 1.2f * (float)i);
        ImVec2 cp = ImGui::GetCursorScreenPos();
        if (i > 0) {
            float x = cp.x - fs * 0.6f;
            dl->AddLine(ImVec2(x, cp.y - fs * 0.6f), ImVec2(x, cp.y + fs * 0.55f), Col(p.textFaint, 0.6f), 1.5f);
            dl->AddLine(ImVec2(x, cp.y + fs * 0.55f), ImVec2(cp.x - fs * 0.1f, cp.y + fs * 0.55f), Col(p.textFaint, 0.6f), 1.5f);
        }
        ImVec4 c = last ? p.accent : hmd ? p.purple : p.textDim;
        TextColored2(c, "%s", last ? ICON_WIFI : hmd ? ICON_HEADSET : ICON_USB);
        ImGui::SameLine();
        if (last) BoldText("%s", n.name.c_str());
        else ImGui::TextUnformatted(n.name.empty() ? "(未命名设备)" : n.name.c_str());
        ImGui::SameLine();
        if (n.vid) TextDim("%04X:%04X", n.vid, n.pid);
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Text, p.textFaint);
        ImGui::TextUnformatted(n.instanceId.c_str());
        ImGui::PopStyleColor();
        ImGui::Unindent(fs * 1.2f * (float)i);
    }
    ImGui::Spacing();
    if (r.usb.behindHmd) TextColored2(p.purple, ICON_HEADSET "  判定为头显内置接收器：%s", r.usb.hmdName.c_str());
    ImGui::Spacing();

    SectionTitle(ICON_INFO, "设备属性");
    if (BeginPropTable("##usbprops", fs * 9)) {
        PropRow("实例路径", r.usb.instanceId, true);
        PropRow("名称", r.usb.friendlyName);
        PropRow("总线描述", r.usb.busDescription);
        PropRow("制造商", r.usb.manufacturer);
        PropRow("位置", r.usb.locationInfo);
        PropRow("端口号", r.usb.address ? std::to_string(r.usb.address) : "");
        PropRow("位置路径", r.usb.locationPath, true);
        PropRow("驱动服务", r.usb.service);
        PropRow("驱动版本", r.usb.driverVersion + (r.usb.driverDate.empty() ? "" : "（" + r.usb.driverDate + "）"));
        ImVec4 bad = p.danger;
        PropRow("设备状态", r.usb.hasProblem ? Fmt("问题代码 %u", r.usb.problemCode) : "正常", false, r.usb.hasProblem ? &bad : nullptr);
        std::string hid;
        for (auto& h : r.usb.hidInterfaces) hid += h + "\n";
        if (!hid.empty()) hid.pop_back();
        PropRow("HID 接口", hid, true);
        EndPropTable();
    }
    if (!r.usb.siblings.empty()) {
        ImGui::Spacing();
        SectionTitle(ICON_DEVICES, "同一 Hub 上的其他设备");
        if (ImGui::BeginTable("##sib", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerH)) {
            ImGui::TableSetupColumn("名称");
            ImGui::TableSetupColumn("VID:PID");
            ImGui::TableSetupColumn("实例");
            ImGui::TableHeadersRow();
            for (auto& sb : r.usb.siblings) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(sb.name.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%04X:%04X", sb.vid, sb.pid);
                ImGui::TableNextColumn();
                TextDim("%s", sb.instanceId.c_str());
            }
            ImGui::EndTable();
        }
    }
}

void App::DrawTabHistory(AppState& s, Receiver& r) {
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (ui_.editingFor != r.serial) {
        ui_.editingFor = r.serial;
        std::snprintf(ui_.nickBuf, sizeof ui_.nickBuf, "%s", r.nickname.c_str());
        std::snprintf(ui_.noteBuf, sizeof ui_.noteBuf, "%s", r.note.c_str());
    }
    if (BeginCard("##names", ImVec2(0, 0))) {
        SectionTitle(ICON_EDIT, "别名与备注");
        ImGui::Spacing();
        ImGui::SetNextItemWidth(fs * 16);
        if (ImGui::InputTextWithHint("别名", "例如：右手 / 腰追踪器", ui_.nickBuf, sizeof ui_.nickBuf, ImGuiInputTextFlags_EnterReturnsTrue) ||
            ImGui::IsItemDeactivatedAfterEdit())
            Post({Command::Type::SetNickname, r.serial, ui_.nickBuf});
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputTextWithHint("##note", "备注（例如：这个接收器信号偏弱，2026-10-12 已改用外置接收器）", ui_.noteBuf, sizeof ui_.noteBuf) &&
            false) {
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) Post({Command::Type::SetNote, r.serial, ui_.noteBuf});
        int kind = r.kindOverridden ? (int)r.kind : -1;
        const char* kinds[] = {"自动判断", "头显内置", "外置 VR Radio", "Watchman 接收器", "未知", "USB 直连设备"};
        int sel = kind + 1;
        ImGui::SetNextItemWidth(fs * 12);
        if (ImGui::Combo("类型", &sel, kinds, 6)) Post({Command::Type::SetKind, r.serial, "", sel - 1});
        HelpMarker("自动判断依据：USB 父 Hub 或同级设备是否属于已知头显厂商（Pimax 34A4 / HTC 0BB4 / Valve / Bigscreen）。判断错误时可以手动指定。");
    }
    EndCard();
    ImGui::Spacing();

    double now = NowSeconds();
    if (historyCacheFor_ != r.serial || now - historyCacheT_ > 2.0) {
        historyCache_ = svc_.PairingsFor(r.serial);
        historyCacheFor_ = r.serial;
        historyCacheT_ = now;
    }
    SectionTitle(ICON_HISTORY, "配对历史", "（本工具观察到的设备 ↔ 接收器关系）");
    if (historyCache_.empty()) {
        TextDim("还没有记录。设备在线时会自动记录。");
    } else if (ImGui::BeginTable("##hist", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("设备");
        ImGui::TableSetupColumn("型号");
        ImGui::TableSetupColumn("来源");
        ImGui::TableSetupColumn("首次");
        ImGui::TableSetupColumn("最后");
        ImGui::TableHeadersRow();
        for (auto& h : historyCache_) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(h.device.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(h.model.c_str());
            ImGui::TableNextColumn();
            TextDim("%s", h.source.c_str());
            ImGui::TableNextColumn();
            TextDim("%s", FormatDateTime(h.firstSeen).c_str());
            ImGui::TableNextColumn();
            TextDim("%s", FormatAgo(h.lastSeen).c_str());
        }
        ImGui::EndTable();
    }
    ImGui::Spacing();
    TextDim(ICON_SAVE "  历史文件: %s", svc_.HistoryPath().c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("打开备份目录")) sys::OpenInExplorer(svc_.BackupDir());
    (void)p;
}

void App::DrawTabConsole(AppState& s, Receiver& r) {
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    TextDim("此接收器的 lighthouse_console 输出（监控会话 + 操作会话）。");
    if (s.settings.expertMode) {
        ImGui::SetNextItemWidth(-fs * 6);
        bool enter = ImGui::InputTextWithHint("##raw", "输入 lighthouse_console 命令，例如 imustats / deviceinfo / version", ui_.rawCmd, sizeof ui_.rawCmd,
                                              ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        if ((ImGui::Button("执行", ImVec2(-FLT_MIN, 0)) || enter) && ui_.rawCmd[0]) {
            std::string c = ui_.rawCmd;
            static const char* dangerous[] = {"pair", "unpair", "pairall", "unpairall", "forcepairall", "donglereset", "dongleresetall", "reboot", "isp", "uploadconfig", "poweroff"};
            bool risky = std::any_of(std::begin(dangerous), std::end(dangerous), [&](const char* d) { return c.rfind(d, 0) == 0; });
            if (risky) store_.ToastUnlocked(s, Toast::Kind::Warning, "请使用界面上的对应功能", "为了安全，配对/解配/重置/刷写类命令只能通过带确认的按钮执行。");
            else Post({Command::Type::RawCommand, r.serial, c});
            ui_.rawCmd[0] = 0;
            ImGui::SetKeyboardFocusHere(-1);
        }
    } else {
        TextColored2(p.textFaint, ICON_LOCK "  在设置中开启专家模式后，可以在这里直接执行只读的 lighthouse_console 命令。");
    }
    ImGui::Spacing();
    ImGui::PushFont(GetFonts().mono, ImGui::GetStyle().FontSizeBase * 0.92f);
    if (ImGui::BeginChild("##rxlog", ImVec2(0, 0), ImGuiChildFlags_Borders)) {
        std::vector<const LogLine*> lines;
        for (auto& l : s.log)
            if (IEquals(l.source, r.serial)) lines.push_back(&l);
        ImGuiListClipper clip;
        clip.Begin((int)lines.size());
        while (clip.Step())
            for (int i = clip.DisplayStart; i < clip.DisplayEnd; ++i) {
                const LogLine& l = *lines[i];
                ImVec4 c = l.level == LogLevel::Error ? p.danger : l.level == LogLevel::Warning ? p.warning : l.level == LogLevel::Event ? p.info : l.level == LogLevel::Command ? p.accent : p.textDim;
                TextDim("%s", FormatClock(l.unix).c_str());
                ImGui::SameLine();
                TextColored2(c, "%s", l.text.c_str());
            }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
    ImGui::PopFont();
}

// ============================================================================ overview

void App::DrawOverview(AppState& s) {
    const auto& p = Pal();
    if (!ImGui::Begin("信号总览###overview", &ui_.overviewOpen)) {
        ImGui::End();
        return;
    }
    float fs = ImGui::GetFontSize();
    double now = NowSeconds();
    TextDim("所有接收器的实时状态。点击一行查看详情，双击打开实时信号。");
    ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable |
                         ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("##ov", 13, tf)) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("接收器", ImGuiTableColumnFlags_WidthFixed, fs * 8.5f);
        ImGui::TableSetupColumn("类型", ImGuiTableColumnFlags_WidthFixed, fs * 5.5f);
        ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthFixed, fs * 4.2f);
        ImGui::TableSetupColumn("设备", ImGuiTableColumnFlags_WidthFixed, fs * 8.5f);
        ImGui::TableSetupColumn("电量", ImGuiTableColumnFlags_WidthFixed, fs * 2.6f);
        ImGui::TableSetupColumn("质量", ImGuiTableColumnFlags_WidthFixed, fs * 2.4f);
        ImGui::TableSetupColumn("IMU Hz", ImGuiTableColumnFlags_WidthFixed, fs * 3.2f);
        ImGui::TableSetupColumn("σ ms", ImGuiTableColumnFlags_WidthFixed, fs * 2.6f);
        ImGui::TableSetupColumn("USB/s", ImGuiTableColumnFlags_WidthFixed, fs * 3.0f);
        ImGui::TableSetupColumn("丢包/分", ImGuiTableColumnFlags_WidthFixed, fs * 3.2f);
        ImGui::TableSetupColumn("最长", ImGuiTableColumnFlags_WidthFixed, fs * 2.8f);
        ImGui::TableSetupColumn("断开", ImGuiTableColumnFlags_WidthFixed, fs * 2.4f);
        ImGui::TableSetupColumn("最近 2 分钟", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (auto& r : s.receivers) {
            ImGui::PushID(r.serial.c_str());
            ImGui::TableNextRow(0, fs * 2.0f);
            ImGui::TableNextColumn();
            bool sel = ui_.selected == r.serial;
            if (ImGui::Selectable("##row", sel, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap, ImVec2(0, fs * 1.8f))) ui_.selected = r.serial;
            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                ui_.selected = r.serial;
                SetTab(DetailTab::Signal);
                ImGui::SetWindowFocus("详情###detail");
            }
            if (ImGui::BeginPopupContextItem("ctx")) {
                ReceiverContextMenu(s, r);
                ImGui::EndPopup();
            }
            ImGui::SameLine(0, 0);
            StatusDot(StateColor(r), fs * 0.25f, r.state == LinkState::Online);
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::TextUnformatted(r.DisplayName());
            if (!r.nickname.empty()) {
                ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 0.8f);
                TextDim("%s", r.serial.c_str());
                ImGui::PopFont();
            }
            ImGui::EndGroup();
            ImGui::TableNextColumn();
            TextColored2(KindColor(r.kind), "%s", ToString(r.kind));
            ImGui::TableNextColumn();
            TextColored2(StateColor(r), "%s", StateText(r));
            ImGui::TableNextColumn();
            if (r.device) {
                ImGui::TextUnformatted(r.device->serial.empty() ? "在线" : r.device->serial.c_str());
                std::string m = ModelText(*r.device);
                if (!m.empty()) {
                    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 0.8f);
                    TextDim("%s", m.c_str());
                    ImGui::PopFont();
                }
            } else if (!r.pairedSerial.empty()) {
                TextDim("%s", r.pairedSerial.c_str());
            } else {
                TextDim("—");
            }
            ImGui::TableNextColumn();
            if (r.device && r.device->batteryPct) TextColored2(BatteryColor(*r.device->batteryPct), "%d%%", *r.device->batteryPct);
            else TextDim("—");
            ImGui::TableNextColumn();
            int q = SignalQuality(r, now);
            SignalBars(q, QualityColor(q), fs * 0.8f);
            bool online = r.state == LinkState::Online;
            ImGui::TableNextColumn();
            if (online) ImGui::Text("%.0f", r.imu.rateHz);
            else TextDim("—");
            ImGui::TableNextColumn();
            if (online) TextColored2(r.imu.sigmaMs > 1.5 ? p.warning : p.text, "%.2f", r.imu.sigmaMs);
            else TextDim("—");
            ImGui::TableNextColumn();
            double usb = 0;
            for (auto& u : r.usbRates) usb += u.perSec;
            if (online) ImGui::Text("%.0f", usb);
            else TextDim("—");
            float worst = 0;
            int g = GapsWithin(r, now, 60, &worst);
            ImGui::TableNextColumn();
            TextColored2(g == 0 ? p.textDim : g > 5 ? p.danger : p.warning, "%d", g);
            ImGui::TableNextColumn();
            TextColored2(worst > 1 ? p.danger : worst > 0 ? p.warning : p.textDim, worst > 0 ? "%.2fs" : "—", worst);
            ImGui::TableNextColumn();
            TextColored2(r.disconnectCount ? p.danger : p.textDim, "%u", r.disconnectCount);
            ImGui::TableNextColumn();
            Sparkline(r, now, 120, ImVec2(std::max(fs * 6, ImGui::GetContentRegionAvail().x), fs * 1.5f), online ? p.plotImu : p.textFaint);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// ============================================================================ log

void App::DrawLog(AppState& s) {
    const auto& p = Pal();
    if (!ImGui::Begin("控制台日志###log")) {
        ImGui::End();
        return;
    }
    float fs = ImGui::GetFontSize();
    ImGui::SetNextItemWidth(fs * 16);
    ImGui::InputTextWithHint("##lf", ICON_FILTER "  过滤日志", ui_.logFilter, sizeof ui_.logFilter);
    ImGui::SameLine();
    const char* levels[] = {"调试", "信息", "事件", "警告", "错误"};
    ImGui::SetNextItemWidth(fs * 6);
    ImGui::Combo("最低级别", &ui_.logMinLevel, levels, 5);
    ImGui::SameLine();
    ImGui::Checkbox("只看选中接收器", &ui_.logSelectedOnly);
    ImGui::SameLine();
    ImGui::Checkbox("自动滚动", &ui_.logAutoScroll);
    ImGui::SameLine();
    if (ImGui::Button(ICON_COPY "  复制")) {
        std::string all;
        for (auto& l : s.log) all += FormatDateTime(l.unix) + " [" + l.source + "] " + l.text + "\n";
        sys::CopyToClipboard(all);
        store_.ToastUnlocked(s, Toast::Kind::Success, "已复制", Fmt("%d 行日志", (int)s.log.size()));
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_EXPORT "  导出")) {
        std::string all;
        for (auto& l : s.log) all += FormatDateTime(l.unix) + " [" + l.source + "] " + l.text + "\n";
        std::string path = s.dataDir + "\\log-" + FormatTimestampForFile() + ".txt";
        sys::WriteTextFile(path, all);
        store_.ToastUnlocked(s, Toast::Kind::Success, "日志已导出", path);
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_CLEAR "  清空")) s.log.clear();

    std::string f = ui_.logFilter;
    std::vector<const LogLine*> lines;
    lines.reserve(s.log.size());
    for (auto& l : s.log) {
        int lv = l.level == LogLevel::Debug ? 0 : l.level == LogLevel::Info || l.level == LogLevel::Command ? 1 : l.level == LogLevel::Event ? 2 : l.level == LogLevel::Warning ? 3 : 4;
        if (lv < ui_.logMinLevel) continue;
        if (ui_.logSelectedOnly && !IEquals(l.source, ui_.selected)) continue;
        if (!f.empty() && l.text.find(f) == std::string::npos && l.source.find(f) == std::string::npos) continue;
        lines.push_back(&l);
    }
    ImGui::PushFont(GetFonts().mono, ImGui::GetStyle().FontSizeBase * 0.92f);
    if (ImGui::BeginTable("##log", 4, ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable, ImVec2(0, 0))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("时间", ImGuiTableColumnFlags_WidthFixed, fs * 5.2f);
        ImGui::TableSetupColumn("级别", ImGuiTableColumnFlags_WidthFixed, fs * 3.2f);
        ImGui::TableSetupColumn("来源", ImGuiTableColumnFlags_WidthFixed, fs * 8.5f);
        ImGui::TableSetupColumn("内容", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        ImGuiListClipper clip;
        clip.Begin((int)lines.size());
        while (clip.Step()) {
            for (int i = clip.DisplayStart; i < clip.DisplayEnd; ++i) {
                const LogLine& l = *lines[i];
                ImVec4 c = l.level == LogLevel::Error ? p.danger : l.level == LogLevel::Warning ? p.warning : l.level == LogLevel::Event ? p.info : l.level == LogLevel::Command ? p.accent
                           : l.level == LogLevel::Debug                                        ? p.textFaint
                                                                                                : p.text;
                const char* lv = l.level == LogLevel::Error ? "错误" : l.level == LogLevel::Warning ? "警告" : l.level == LogLevel::Event ? "事件" : l.level == LogLevel::Command ? "命令"
                                 : l.level == LogLevel::Debug                                     ? "调试"
                                                                                                  : "信息";
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                TextDim("%s", FormatClock(l.unix).c_str());
                ImGui::TableNextColumn();
                TextColored2(c, "%s", lv);
                ImGui::TableNextColumn();
                TextDim("%s", l.source.c_str());
                ImGui::TableNextColumn();
                TextColored2(c, "%s", l.text.c_str());
            }
        }
        if (ui_.logAutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - fs * 2) ImGui::SetScrollHereY(1.0f);
        ImGui::EndTable();
    }
    ImGui::PopFont();
    ImGui::End();
}

}  // namespace lhp::ui
