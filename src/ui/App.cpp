#include "ui/App.h"

#include "backend/SystemInfo.h"
#include "imgui_internal.h"
#include "implot.h"
#include "ui/Icons.h"
#include "ui/SpatialView.h"
#include "ui/Widgets.h"
#include "util/Text.h"

#include <algorithm>
#include <cmath>

namespace lhp::ui {

namespace {

struct DeviceType {
    const char* name;
    const char* hint;
};
const DeviceType kDeviceTypes[] = {
    {"Valve Index 手柄", "关机状态下同时按住 B 键和系统键，直到指示灯蓝色闪烁。"},
    {"HTC Vive 手柄", "关机状态下同时按住菜单键和系统键，直到指示灯蓝色闪烁。"},
    {"VIVE Tracker 2018 / 3.0", "长按电源键约 3 秒，直到指示灯蓝色闪烁。"},
    {"Tundra Tracker", "长按按钮约 3 秒，直到指示灯蓝色闪烁。"},
    {"其他 Lighthouse 设备", "按设备说明书进入配对模式（通常指示灯会蓝色闪烁）。"},
};

}  // namespace

const char* DeviceTypeName(int idx) { return kDeviceTypes[std::clamp(idx, 0, DeviceTypeCount() - 1)].name; }
const char* DeviceTypeHint(int idx) { return kDeviceTypes[std::clamp(idx, 0, DeviceTypeCount() - 1)].hint; }
int DeviceTypeCount() { return (int)(sizeof kDeviceTypes / sizeof kDeviceTypes[0]); }

int SignalQuality(const Receiver& r, double now) {
    if (r.state != LinkState::Online) return 0;
    int gaps = 0;
    float worst = 0;
    for (auto it = r.events.rbegin(); it != r.events.rend(); ++it) {
        if (now - it->t > 60) break;
        if (it->type == SignalEvent::Type::Gap) {
            ++gaps;
            worst = std::max(worst, it->value);
        }
    }
    int q = gaps == 0 ? 4 : gaps <= 2 ? 3 : gaps <= 8 ? 2 : 1;
    if (worst > 1.0f) q = std::min(q, 2);
    if (r.imu.sigmaMs > 1.5) q = std::min(q, 2);
    return std::max(q, 1);
}

App::App(Store& store, IService& service) : store_(store), svc_(service) {}

void App::OpenWizard(const std::string& target) {
    ui_.wizard = PairWizard{};
    ui_.wizard.open = true;
    ui_.wizard.justOpened = true;
    ui_.wizard.target = target;
    ui_.wizard.step = target.empty() ? 0 : 1;
}

void App::OpenDanger(Command::Type t, const std::string& serial) {
    ui_.danger = DangerDialog{};
    ui_.danger.open = true;
    ui_.danger.justOpened = true;
    ui_.danger.type = t;
    ui_.danger.serial = serial;
}

void App::Frame() {
    {
        std::scoped_lock lk(store_.mutex);
        AppState& s = store_.state;
        fastFrames_ = s.op.Active() || !s.toasts.empty() || s.refreshing;

        if (ui_.selected.empty() || !s.Find(ui_.selected)) {
            for (auto& r : s.receivers)
                if (r.state == LinkState::Online) {
                    ui_.selected = r.serial;
                    break;
                }
            if (ui_.selected.empty() && !s.receivers.empty()) ui_.selected = s.receivers.front().serial;
        }

        DrawToolbar(s);
        DrawStatusBar(s);

        ImGuiID dock = ImGui::GetID("MainDock");
        if (!layoutChecked_) {
            layoutChecked_ = true;
            if (!ImGui::DockBuilderGetNode(dock)) BuildDefaultLayout(dock);
        }
        ImGui::DockSpaceOverViewport(dock, ImGui::GetMainViewport(), ImGuiDockNodeFlags_None);

        DrawReceivers(s);
        DrawDetail(s);
        if (ui_.overviewOpen) DrawOverview(s);
        DrawLog(s);

        DrawPairWizard(s);
        DrawDangerDialog(s);
        DrawKillDialog(s);
        DrawSettings(s);
        DrawAbout(s);
        DrawIdentifyDialog(s);
        DrawOpOverlay(s);
        DrawToasts(s);
    }
    // The spatial view reads tracking snapshots only (no store lock needed).
    if (spatial_) {
        if (ImGuiWindow* detail = ImGui::FindWindowByName("详情###detail"); detail && detail->DockId)
            ImGui::SetNextWindowDockID(detail->DockId, ImGuiCond_FirstUseEver);
        if (focusSpatial_) {
            ImGui::SetNextWindowFocus();
            focusSpatial_ = false;
        }
        spatial_->Draw();
        fastFrames_ |= spatial_->WantsFastFrames();
    }
    // Commands are posted after releasing the lock (the demo service locks the store in Post).
    auto cmds = std::move(pending_);
    pending_.clear();
    for (auto& c : cmds) svc_.Post(std::move(c));
}

void App::BuildDefaultLayout(unsigned dockId) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(dockId);
    ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockId, vp->WorkSize);
    ImGuiID left, rest, bottom, center;
    ImGui::DockBuilderSplitNode(dockId, ImGuiDir_Left, 0.29f, &left, &rest);
    ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, 0.24f, &bottom, &center);
    ImGui::DockBuilderDockWindow("接收器###receivers", left);
    ImGui::DockBuilderDockWindow("详情###detail", center);
    ImGui::DockBuilderDockWindow("信号总览###overview", center);
    ImGui::DockBuilderDockWindow(SpatialView::kWindowName, center);
    ImGui::DockBuilderDockWindow("控制台日志###log", bottom);
    ImGui::DockBuilderFinish(dockId);
}

// ---------------------------------------------------------------------------- toolbar / status bar

void App::DrawToolbar(AppState& s) {
    const auto& p = Pal();
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float h = ImGui::GetFrameHeight() + ImGui::GetFontSize() * 0.9f;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, Mix(p.bg, p.panel, 0.4f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetFontSize() * 0.6f, ImGui::GetFontSize() * 0.45f));
    ImGuiWindowFlags fl = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDocking;
    if (ImGui::BeginViewportSideBar("##toolbar", vp, ImGuiDir_Up, h, fl)) {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        // Logo: concentric rings
        ImVec2 c = ImGui::GetCursorScreenPos();
        float r = ImGui::GetFontSize() * 0.55f;
        ImVec2 cc(c.x + r, c.y + ImGui::GetFrameHeight() * 0.5f);
        dl->AddCircleFilled(cc, r, Col(p.accent, 0.18f));
        dl->AddCircle(cc, r * 0.72f, Col(p.accent), 0, 2.0f);
        dl->AddCircleFilled(cc, r * 0.30f, Col(p.accent));
        ImGui::Dummy(ImVec2(r * 2.2f, ImGui::GetFrameHeight()));
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::PushFont(GetFonts().bold, 0.0f);
        ImGui::TextUnformatted("Lighthouse 配对管理");
        ImGui::PopFont();
        // Chips use a frame-height slot; the title is 1.15x tall, so nudge the chip row onto the title's centre line.
        const float chipNudge = 0.0f;   // title is regular size now: same frame-height line as the chips
        ImGui::SameLine();
        if (s.demo) {
            ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + chipNudge);
            Chip("演示模式", p.purple, true);
        }

        int online = 0, monitored = 0;
        for (auto& rr : s.receivers) {
            online += rr.state == LinkState::Online;
            monitored += rr.monitor == MonitorState::Running;
        }
        ImGui::SameLine(0, ImGui::GetFontSize() * 1.2f);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + chipNudge);
        int receiverCount = (int)std::count_if(s.receivers.begin(), s.receivers.end(), [](const Receiver& x) { return x.kind != ReceiverKind::Wired; });
        Chip(Fmt("%d 个接收器", receiverCount).c_str(), p.textDim);
        ChipSameLine(Fmt("%d 在线", online).c_str(), p.success);
        ChipSameLine(Fmt("%d 监控中", monitored).c_str(), p.info);
        if (!s.conflicts.empty()) {
            ImGui::SameLine(0, ImGui::GetFontSize());
            std::string names;
            for (auto& cpr : s.conflicts) {
                if (names.find(cpr.category) == std::string::npos) names += (names.empty() ? "" : " / ") + cpr.category;
            }
            ImGui::PushStyleColor(ImGuiCol_Button, Mix(p.warning, ImVec4(0, 0, 0, 0), 0.82f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Mix(p.warning, ImVec4(0, 0, 0, 0), 0.7f));
            ImGui::PushStyleColor(ImGuiCol_Text, p.warning);
            // Half-height padding, centred in the same frame-height slot as the chips.
            const ImVec2 fp = ImGui::GetStyle().FramePadding;
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + fp.y * 0.5f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(fp.x, fp.y * 0.5f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, (ImGui::GetFontSize() + fp.y) * 0.5f);
            if (ImGui::Button(Fmt(ICON_WARNING "  %s 正在运行（可能占用接收器）", names.c_str()).c_str())) ui_.showKill = true;
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(3);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("点击查看并可选择结束这些进程。\n只读监控通常可以共存；配对/解配等写操作建议先关闭。");
        }

        // Right side buttons
        float bw = ImGui::GetFrameHeight();
        float right = ImGui::GetWindowWidth() - ImGui::GetStyle().WindowPadding.x;
        float total = bw * 5 + ImGui::GetStyle().ItemSpacing.x * 5 + ImGui::CalcTextSize("  配对新设备").x + ImGui::GetStyle().FramePadding.x * 2 + bw;
        ImGui::SameLine(right - total);
        ImGui::SetCursorPosY(ImGui::GetStyle().WindowPadding.y);
        if (AccentButton(ICON_ADD "  配对新设备")) OpenWizard(ui_.selected);
        ImGui::SameLine();
        bool anyMon = monitored > 0;
        if (s.refreshing) {
            ProgressRing(bw * 0.5f, 2.5f, -1, p.accent);
        } else if (IconButton(ICON_REFRESH, "重新扫描接收器 (F5)", ImVec2(bw, bw))) {
            Post({Command::Type::Refresh});
        }
        ImGui::SameLine();
        if (IconButton(anyMon && s.settings.autoMonitorAll ? ICON_PAUSE : ICON_PLAY, anyMon && s.settings.autoMonitorAll ? "停止所有实时监控" : "开始监控所有接收器",
                       ImVec2(bw, bw)))
            Post({anyMon && s.settings.autoMonitorAll ? Command::Type::StopAllMonitors : Command::Type::MonitorAll});
        ImGui::SameLine();
        if (spatial_ && IconButton(ICON_GLOBE, "空间解算与实时渲染", ImVec2(bw, bw))) focusSpatial_ = true;
        ImGui::SameLine();
        if (IconButton(ICON_SETTINGS, "设置", ImVec2(bw, bw))) ui_.showSettings = true;
        ImGui::SameLine();
        if (IconButton(ICON_HELP, "关于", ImVec2(bw, bw))) ui_.showAbout = true;
        // bottom hairline
        ImVec2 wp = ImGui::GetWindowPos(), ws = ImGui::GetWindowSize();
        dl->AddLine(ImVec2(wp.x, wp.y + ws.y - 1), ImVec2(wp.x + ws.x, wp.y + ws.y - 1), Col(p.border));
    }
    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();

    if (ImGui::IsKeyPressed(ImGuiKey_F5)) Post({Command::Type::Refresh});
}

void App::DrawStatusBar(AppState& s) {
    const auto& p = Pal();
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float h = ImGui::GetFrameHeight() * 1.05f;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, Mix(p.bg, p.panel, 0.3f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetFontSize() * 0.8f, ImGui::GetFontSize() * 0.25f));
    if (ImGui::BeginViewportSideBar("##status", vp, ImGuiDir_Down, h, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDocking)) {
        ImGui::AlignTextToFramePadding();
        if (!s.backendError.empty()) {
            TextColored2(p.danger, ICON_WARNING " %s", s.backendError.c_str());
        } else {
            TextDim(ICON_CMD " %s", s.consolePath.empty() ? "lighthouse_console 未找到" : s.consolePath.c_str());
            if (!s.consoleVersion.empty()) {
                ImGui::SameLine();
                TextDim("· 版本 %s", s.consoleVersion.c_str());
            }
        }
        ImGui::SameLine();
        TextDim("· 上次刷新 %s", s.lastRefresh ? FormatClock(s.lastRefresh).c_str() : "—");
        std::string right = Fmt("%s  ·  %.0f FPS", s.dataDir.c_str(), ImGui::GetIO().Framerate);
        float w = ImGui::CalcTextSize(right.c_str()).x;
        ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::GetStyle().WindowPadding.x - w);
        TextDim("%s", right.c_str());
        if (ImGui::IsItemClicked()) sys::OpenInExplorer(s.dataDir);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("数据目录（历史记录、备份、布局），点击打开");
    }
    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

// ---------------------------------------------------------------------------- toasts / op overlay

void App::DrawToasts(AppState& s) {
    const auto& p = Pal();
    double now = NowSeconds();
    s.toasts.erase(std::remove_if(s.toasts.begin(), s.toasts.end(), [&](const Toast& t) { return now - t.createdT > t.duration + 0.4; }), s.toasts.end());
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float pad = ImGui::GetFontSize();
    float y = vp->WorkPos.y + vp->WorkSize.y - pad;
    float width = ImGui::GetFontSize() * 22;
    int idx = 0;
    for (auto it = s.toasts.rbegin(); it != s.toasts.rend(); ++it, ++idx) {
        const Toast& t = *it;
        float age = (float)(now - t.createdT);
        float in = std::clamp(age / 0.25f, 0.f, 1.f);
        float out = std::clamp((t.duration + 0.4f - age) / 0.4f, 0.f, 1.f);
        float a = std::min(in, out);
        float ease = 1 - (1 - a) * (1 - a);
        ImVec4 col = t.kind == Toast::Kind::Success ? p.success : t.kind == Toast::Kind::Warning ? p.warning : t.kind == Toast::Kind::Error ? p.danger : p.info;
        const char* icon = t.kind == Toast::Kind::Success ? ICON_CHECK : t.kind == Toast::Kind::Error ? ICON_CANCEL : t.kind == Toast::Kind::Warning ? ICON_WARNING : ICON_INFO;
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - pad + (1 - ease) * width * 0.4f, y), ImGuiCond_Always, ImVec2(1, 1));
        ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0), ImVec2(width, FLT_MAX));
        ImGui::SetNextWindowBgAlpha(0.97f * a);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, a);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, ImGui::GetFontSize() * 0.7f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, p.dark ? ImVec4(0.13f, 0.145f, 0.17f, 1) : ImVec4(1, 1, 1, 1));
        char name[32];
        std::snprintf(name, sizeof name, "##toast%d", idx);
        ImGui::Begin(name, nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoInputs);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 wp = ImGui::GetWindowPos(), ws = ImGui::GetWindowSize();
        dl->AddRectFilled(wp, ImVec2(wp.x + ImGui::GetFontSize() * 0.25f, wp.y + ws.y), Col(col), ImGui::GetFontSize() * 0.7f, ImDrawFlags_RoundCornersLeft);
        TextColored2(col, "%s", icon);
        ImGui::SameLine();
        ImGui::BeginGroup();
        BoldText("%s", t.title.c_str());
        if (!t.text.empty()) {
            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width - ImGui::GetFontSize() * 3.5f);
            TextDim("%s", t.text.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::EndGroup();
        // remaining time bar
        float frac = std::clamp(1.f - age / t.duration, 0.f, 1.f);
        dl->AddRectFilled(ImVec2(wp.x + ImGui::GetFontSize() * 0.6f, wp.y + ws.y - 3), ImVec2(wp.x + ImGui::GetFontSize() * 0.6f + (ws.x - ImGui::GetFontSize() * 1.2f) * frac, wp.y + ws.y - 1),
                          Col(col, 0.5f), 1.0f);
        y -= ImGui::GetWindowHeight() + pad * 0.5f;
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(3);
    }
}

void App::DrawOpOverlay(AppState& s) {
    if (ui_.wizard.open) return;
    const Operation& op = s.op;
    if (op.kind == OpKind::None) return;
    const auto& p = Pal();
    bool active = op.Active();
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + ImGui::GetFontSize()), ImGuiCond_Always, ImVec2(0.5f, 0));
    ImGui::SetNextWindowBgAlpha(0.98f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, ImGui::GetFontSize());
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::Begin("##opoverlay", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking);
    ImVec4 col = op.phase == OpPhase::Succeeded ? p.success : op.phase == OpPhase::Failed ? p.danger : op.phase == OpPhase::Cancelled ? p.textDim : p.accent;
    if (active) {
        float prog = op.timeoutS > 0 && op.phase == OpPhase::WaitingForDevice ? (float)((NowSeconds() - op.startedT) / op.timeoutS) : -1.f;
        ProgressRing(ImGui::GetFontSize() * 0.75f, 3.0f, prog, col);
    } else {
        TextColored2(col, "%s", op.phase == OpPhase::Succeeded ? ICON_CHECK : op.phase == OpPhase::Failed ? ICON_CANCEL : ICON_INFO);
    }
    ImGui::SameLine();
    ImGui::BeginGroup();
    const char* kind = op.kind == OpKind::Pair ? "配对" : op.kind == OpKind::Unpair ? "解除配对" : op.kind == OpKind::FactoryReset ? "恢复出厂" : op.kind == OpKind::Identify ? "识别" : op.kind == OpKind::PowerOff ? "关机" : "操作";
    BoldText("%s · %s", kind, op.target.c_str());
    TextDim("%s", op.message.c_str());
    ImGui::EndGroup();
    ImGui::SameLine(0, ImGui::GetFontSize() * 1.5f);
    if (active) {
        if (GhostButton("取消")) Post({Command::Type::CancelOp});
    } else if (GhostButton(ICON_CANCEL)) {
        Post({Command::Type::DismissOp});
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}

}  // namespace lhp::ui
