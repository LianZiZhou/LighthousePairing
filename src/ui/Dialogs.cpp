#include "backend/SystemInfo.h"
#include "imgui_internal.h"
#include "implot.h"
#include "ui/App.h"
#include "ui/Icons.h"
#include "ui/Widgets.h"
#include "util/Text.h"

#include <algorithm>
#include <cmath>

namespace lhp::ui {

namespace {

void CenterNext(float wMul, float hMul = 0) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    float fs = ImGui::GetFontSize();
    ImGui::SetNextWindowSize(ImVec2(fs * wMul, hMul > 0 ? fs * hMul : 0), ImGuiCond_Always);
}

std::string Last4(const std::string& s) {
    std::string base = s;
    auto dash = base.find('-');
    if (dash != std::string::npos) base = base.substr(0, dash);
    return base.size() > 4 ? base.substr(base.size() - 4) : base;
}

void StepRow(bool done, bool active, const char* text) {
    const auto& p = Pal();
    if (done) TextColored2(p.success, ICON_CHECK);
    else if (active) {
        ProgressRing(ImGui::GetFontSize() * 0.45f, 2.0f, -1, p.accent);
    } else TextColored2(p.textFaint, "○");
    ImGui::SameLine();
    if (active) BoldText("%s", text);
    else if (done) ImGui::TextUnformatted(text);
    else TextDim("%s", text);
}

}  // namespace

// ============================================================================ pair wizard

void App::DrawPairWizard(AppState& s) {
    auto& w = ui_.wizard;
    if (!w.open) return;
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (w.justOpened) {
        ImGui::OpenPopup("配对新设备###pairwiz");
        w.justOpened = false;
    }
    CenterNext(40);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 1.4f, fs * 1.2f));
    bool keepOpen = true;
    if (!ImGui::BeginPopupModal("配对新设备###pairwiz", s.op.Active() ? nullptr : &keepOpen, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::PopStyleVar();
        if (!keepOpen) w.open = false;
        return;
    }
    // Stepper header
    const char* steps[] = {"选择接收器", "设备与检查", "配对"};
    int cur = std::min(w.step, 2);
    for (int i = 0; i < 3; ++i) {
        if (i) {
            ImGui::SameLine();
            TextDim("─────");
            ImGui::SameLine();
        }
        ImVec4 c = i < cur ? p.success : i == cur ? p.accent : p.textFaint;
        TextColored2(c, "%s", i < cur ? ICON_CHECK : i == cur ? "●" : "○");
        ImGui::SameLine();
        if (i == cur) BoldText("%s", steps[i]);
        else TextDim("%s", steps[i]);
    }
    ImGui::Separator();
    ImGui::Spacing();

    Receiver* target = s.Find(w.target);

    if (w.step == 0) {
        BoldText("要把新设备配对到哪个接收器？");
        TextDim("一个接收器同一时间只能配对一个设备。建议选择空闲且信号好的接收器（外置接收器通常比头显内置的更稳定）。");
        ImGui::Spacing();
        if (ImGui::BeginChild("##pick", ImVec2(0, fs * 16), ImGuiChildFlags_Borders)) {
            for (auto& r : s.receivers) {
                if ((!r.usb.present && !r.seenByConsole) || r.kind == ReceiverKind::Wired) continue;
                ImGui::PushID(r.serial.c_str());
                bool sel = w.target == r.serial;
                if (ImGui::Selectable("##r", sel, ImGuiSelectableFlags_AllowOverlap, ImVec2(0, fs * 2.2f))) w.target = r.serial;
                ImGui::SameLine(fs * 0.5f);
                ImGui::BeginGroup();
                ImGui::AlignTextToFramePadding();   // name shares the chip's frame-height line
                BoldText("%s", r.DisplayName());
                ImGui::SameLine();
                Chip(ToString(r.kind), r.kind == ReceiverKind::HmdBuiltin ? p.purple : p.info);
                if (r.state == LinkState::Online) TextColored2(p.warning, "已有设备在线：%s", r.device && !r.device->serial.empty() ? r.device->serial.c_str() : "（未知）");
                else if (!r.pairedSerial.empty()) TextDim("历史配对：%s（%s）", r.pairedSerial.c_str(), ToString(r.pairingSource));
                else TextColored2(p.success, "空闲");
                ImGui::EndGroup();
                ImGui::PopID();
            }
        }
        ImGui::EndChild();
        ImGui::Spacing();
        ImGui::BeginDisabled(w.target.empty());
        if (AccentButton("下一步 " ICON_CHEVRON_R, ImVec2(fs * 8, 0))) w.step = 1;
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (GhostButton("取消")) {
            w.open = false;
            ImGui::CloseCurrentPopup();
        }
    } else if (w.step == 1) {
        if (!target) {
            TextColored2(p.danger, "接收器 %s 不存在。", w.target.c_str());
            if (ImGui::Button("返回")) w.step = 0;
        } else {
            if (BeginCard("##tgt", ImVec2(0, 0))) {
                TextDim("目标接收器");
                BigText(1.25f, "%s", target->DisplayName());
                ImGui::SameLine();
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (fs * BigScale(1.25f) - ImGui::GetFrameHeight()) * 0.5f);
                Chip(ToString(target->kind), target->kind == ReceiverKind::HmdBuiltin ? p.purple : p.info);
                TextDim("%s · %s", target->serial.c_str(), target->usb.behindHmd ? target->usb.hmdName.c_str() : "外置");
            }
            EndCard();
            bool occupied = target->state == LinkState::Online || !target->pairedSerial.empty();
            if (occupied) {
                ImGui::Spacing();
                std::string curDev = target->device && !target->device->serial.empty() ? target->device->serial : target->pairedSerial;
                TextColored2(p.warning, ICON_WARNING "  这个接收器当前配对了 %s。", curDev.empty() ? "一个设备" : curDev.c_str());
                TextDim("配对新设备后，原来的设备将无法再连接到这个接收器，需要另找接收器重新配对。");
                ImGui::Checkbox("我明白，继续", &w.overwriteAck);
            }
            ImGui::Spacing();
            BoldText("设备类型");
            TextDim("用于显示如何让设备进入配对模式，不影响配对命令本身。");
            for (int i = 0; i < DeviceTypeCount(); ++i)
                if (ImGui::RadioButton(DeviceTypeName(i), w.deviceType == i)) w.deviceType = i;
            ImGui::Spacing();
            ImGui::SetNextItemWidth(fs * 12);
            ImGui::SliderInt("等待时间", &w.timeoutS, 15, 120, "%d 秒");
            if (!s.conflicts.empty()) {
                ImGui::Spacing();
                std::string names;
                for (auto& c : s.conflicts) names += c.exe + " ";
                TextColored2(p.warning, ICON_WARNING "  检测到可能占用接收器的程序：%s", names.c_str());
                TextDim("配对时建议先退出 SteamVR / Pimax 软件，否则命令可能失败或被它们干扰。");
                if (DangerButton(ICON_STOP "  结束这些进程")) Post({Command::Type::KillConflicts});
                ImGui::SameLine();
                ImGui::Checkbox("仍然继续", &w.conflictAck);
            }
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            bool ready = (!occupied || w.overwriteAck) && (s.conflicts.empty() || w.conflictAck) && !s.op.Active();
            if (GhostButton(ICON_CHEVRON_D " 上一步")) w.step = 0;
            ImGui::SameLine();
            ImGui::BeginDisabled(!ready);
            if (AccentButton(ICON_LINK "  开始配对", ImVec2(fs * 9, 0))) {
                Command c{Command::Type::Pair, target->serial, DeviceTypeName(w.deviceType)};
                c.seconds = w.timeoutS;
                Post(c);
                w.step = 2;
                w.opId = 0;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            TextDim("开始前会自动备份当前状态");
        }
    } else {
        const Operation& op = s.op;
        bool mine = op.kind == OpKind::Pair && IEquals(op.target, w.target);
        if (!mine) {
            ProgressRing(fs * 1.2f, 3, -1, p.accent);
            ImGui::SameLine();
            TextDim("正在启动…");
        } else {
            bool active = op.Active();
            double el = NowSeconds() - op.startedT;
            if (op.phase == OpPhase::WaitingForDevice) {
                float prog = (float)std::clamp(el / std::max(1.0, op.timeoutS), 0.0, 1.0);
                ImGui::BeginGroup();
                ImVec2 c0 = ImGui::GetCursorScreenPos();
                float R = fs * 3.2f;
                ProgressRing(R, fs * 0.35f, 1.0f - prog, p.accent);
                std::string left = Fmt("%.0f", std::max(0.0, op.timeoutS - el));
                ImFont* bf = GetFonts().bold;
                float big = fs * 2.0f;
                ImVec2 ts = bf->CalcTextSizeA(big, FLT_MAX, 0, left.c_str());
                ImGui::GetWindowDrawList()->AddText(bf, big, ImVec2(c0.x + R - ts.x * 0.5f, c0.y + R - ts.y * 0.6f), Col(p.text), left.c_str());
                ImVec2 ts2 = ImGui::CalcTextSize("秒");
                ImGui::GetWindowDrawList()->AddText(ImVec2(c0.x + R - ts2.x * 0.5f, c0.y + R + ts.y * 0.35f), Col(p.textDim), "秒");
                ImGui::EndGroup();
                ImGui::SameLine(0, fs * 1.5f);
                ImGui::BeginGroup();
                BigText(1.2f, "现在让设备进入配对模式");
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 22);
                TextColored2(p.accent, "%s", DeviceTypeHint(w.deviceType));
                ImGui::PopTextWrapPos();
                ImGui::Spacing();
                float pulse = Pulse(1.0f);
                StatusDot(Mix(p.accent, p.info, pulse), fs * 0.35f, true);
                ImGui::SameLine();
                TextDim("接收器 %s 正在等待设备…", op.target.c_str());
                if (!op.warning.empty()) {
                    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 22);
                    TextColored2(p.warning, ICON_WARNING " %s", op.warning.c_str());
                    ImGui::PopTextWrapPos();
                }
                ImGui::EndGroup();
            } else if (op.phase == OpPhase::Succeeded) {
                TextColored2(p.success, ICON_CHECK);
                ImGui::SameLine();
                BigText(1.3f, "配对成功");
                TextDim("%s", op.message.c_str());
                ImGui::Spacing();
                if (!op.resultDevice.empty()) {
                    if (ui_.editingFor != "wiz:" + op.target) {
                        ui_.editingFor = "wiz:" + op.target;
                        std::snprintf(ui_.nickBuf, sizeof ui_.nickBuf, "%s", target ? target->nickname.c_str() : "");
                    }
                    ImGui::SetNextItemWidth(fs * 14);
                    ImGui::InputTextWithHint("给这个接收器起个别名", "例如：右手", ui_.nickBuf, sizeof ui_.nickBuf);
                }
            } else if (op.phase == OpPhase::Failed || op.phase == OpPhase::Cancelled) {
                TextColored2(op.phase == OpPhase::Failed ? p.danger : p.textDim, "%s", op.phase == OpPhase::Failed ? ICON_CANCEL : ICON_INFO);
                ImGui::SameLine();
                BigText(1.3f, op.phase == OpPhase::Failed ? "配对未完成" : "已取消");
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 34);
                TextDim("%s", op.message.c_str());
                ImGui::PopTextWrapPos();
            } else {
                ProgressRing(fs * 1.2f, 3, -1, p.accent);
                ImGui::SameLine();
                ImGui::BeginGroup();
                BoldText("%s", op.message.c_str());
                TextDim("%.1f 秒", el);
                ImGui::EndGroup();
            }
            ImGui::Spacing();
            ImGui::Separator();
            int ph = op.phase == OpPhase::Preparing ? 0 : op.phase == OpPhase::Running ? 1 : op.phase == OpPhase::WaitingForDevice ? 2 : op.phase == OpPhase::Verifying ? 3 : 4;
            bool ok = op.phase == OpPhase::Succeeded;
            StepRow(ph > 0 || ok, ph == 0 && active, "备份并连接到目标接收器（暂停其监控）");
            StepRow(ph > 1 || ok, ph == 1 && active, "让接收器进入配对模式 (serial + pair)");
            StepRow(ph > 2 || ok, ph == 2 && active, "等待设备连接");
            StepRow(ok, ph == 3 && active, "读取设备信息并校验连接在目标接收器上");
            if (!op.backupFile.empty()) TextDim(ICON_SAVE "  备份: %s", op.backupFile.c_str());
            ImGui::Spacing();
            if (active) {
                if (ImGui::Button(ICON_CANCEL "  取消配对")) Post({Command::Type::CancelOp});
            } else {
                if (op.phase == OpPhase::Succeeded) {
                    if (AccentButton("完成", ImVec2(fs * 7, 0))) {
                        if (!op.resultDevice.empty() && target && ui_.nickBuf != target->nickname) Post({Command::Type::SetNickname, op.target, ui_.nickBuf});
                        Post({Command::Type::DismissOp});
                        ui_.selected = op.target;
                        w.open = false;
                        ImGui::CloseCurrentPopup();
                    }
                } else {
                    if (AccentButton(ICON_REFRESH "  重试")) {
                        Command c{Command::Type::Pair, w.target, DeviceTypeName(w.deviceType)};
                        c.seconds = w.timeoutS;
                        Post(c);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("返回修改")) {
                        Post({Command::Type::DismissOp});
                        w.step = 1;
                    }
                    ImGui::SameLine();
                    if (GhostButton("关闭")) {
                        Post({Command::Type::DismissOp});
                        w.open = false;
                        ImGui::CloseCurrentPopup();
                    }
                }
            }
        }
    }
    ImGui::EndPopup();
    ImGui::PopStyleVar();
    if (!keepOpen) w.open = false;
}

// ============================================================================ dangerous operations

void App::DrawDangerDialog(AppState& s) {
    auto& d = ui_.danger;
    if (!d.open) return;
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (d.justOpened) {
        ImGui::OpenPopup("确认###danger");
        d.justOpened = false;
    }
    CenterNext(34);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 1.4f, fs * 1.2f));
    bool open = true;
    if (ImGui::BeginPopupModal("确认###danger", &open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        bool reset = d.type == Command::Type::FactoryReset;
        Receiver* r = s.Find(d.serial);
        TextColored2(p.danger, "%s", ICON_WARNING);
        ImGui::SameLine();
        BigText(1.3f, reset ? "恢复接收器出厂设置" : "解除接收器的配对");
        ImGui::Spacing();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 31);
        if (reset)
            ImGui::TextUnformatted("将向该接收器发送 donglereset。它会清除配对并恢复出厂状态，下次上电时会自动开始配对。只在接收器工作异常时使用。");
        else
            ImGui::TextUnformatted("将向该接收器发送 unpair，清除它保存的配对记录。之后原来的设备将无法连接到这个接收器，直到重新配对。");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (BeginCard("##dz", ImVec2(0, 0))) {
            if (BeginPropTable("##dzt")) {
                PropRow("接收器", d.serial);
                if (r) {
                    PropRow("类型", ToString(r->kind));
                    std::string dev = r->device && !r->device->serial.empty() ? r->device->serial + "（在线）" : r->pairedSerial.empty() ? "未知" : r->pairedSerial + "（" + ToString(r->pairingSource) + "）";
                    PropRow("当前配对", dev);
                }
                PropRow("影响范围", "只影响这一个接收器（会先校验 console 已连接到它）");
                PropRow("备份", "执行前自动写入备份目录");
                EndPropTable();
            }
        }
        EndCard();
        if (!s.conflicts.empty()) TextColored2(p.warning, ICON_WARNING "  SteamVR / Pimax 等程序仍在运行，建议先关闭。");
        ImGui::Spacing();
        std::string need = reset ? "RESET-" + Last4(d.serial) : Last4(d.serial);
        ImGui::TextUnformatted(Fmt("请输入 %s 以确认：", need.c_str()).c_str());
        ImGui::SetNextItemWidth(fs * 12);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::InputText("##confirm", d.confirm, sizeof d.confirm, ImGuiInputTextFlags_CharsUppercase);
        bool match = IEquals(d.confirm, need);
        ImGui::Spacing();
        ImGui::BeginDisabled(!match || s.op.Active());
        if (DangerButton(reset ? ICON_DELETE "  恢复出厂" : ICON_LINK "  解除配对", ImVec2(fs * 9, 0))) {
            Post({d.type, d.serial});
            d.open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("取消", ImVec2(fs * 6, 0))) {
            d.open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar();
    if (!open) d.open = false;
}

// ============================================================================ conflicts

void App::DrawKillDialog(AppState& s) {
    if (!ui_.showKill) return;
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (!ImGui::IsPopupOpen("占用接收器的程序###kill")) ImGui::OpenPopup("占用接收器的程序###kill");
    CenterNext(32);
    bool open = true;
    if (ImGui::BeginPopupModal("占用接收器的程序###kill", &open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 29);
        TextDim("这些程序会打开 Lighthouse 接收器。实时监控一般可以和它们共存，但配对、解配、重置等写操作可能被干扰。结束 SteamVR / Pimax 会中断正在进行的 VR 游戏。");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (s.conflicts.empty()) TextColored2(p.success, ICON_CHECK "  当前没有冲突进程。");
        else if (ImGui::BeginTable("##cf", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("程序");
            ImGui::TableSetupColumn("类别");
            ImGui::TableSetupColumn("PID");
            ImGui::TableHeadersRow();
            for (auto& c : s.conflicts) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(c.exe.c_str());
                if (c.isService) {
                    ImGui::SameLine();
                    TextDim("(服务，可能需要管理员)");
                }
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(c.category.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%u", c.pid);
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        bool hasSteam = std::any_of(s.conflicts.begin(), s.conflicts.end(), [](auto& c) { return c.category == "SteamVR"; });
        bool hasPimax = std::any_of(s.conflicts.begin(), s.conflicts.end(), [](auto& c) { return c.category == "Pimax"; });
        ImGui::BeginDisabled(!hasSteam);
        if (DangerButton("结束 SteamVR")) Post({Command::Type::KillConflicts, "", "SteamVR"});
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!hasPimax);
        if (DangerButton("结束 Pimax")) Post({Command::Type::KillConflicts, "", "Pimax"});
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(s.conflicts.empty());
        if (DangerButton("全部结束")) Post({Command::Type::KillConflicts});
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("关闭")) {
            ui_.showKill = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) ui_.showKill = false;
}

// ============================================================================ identify

void App::IdentifyDevice(const Receiver& r) {
    DeviceClass cls = r.device ? r.device->deviceClass : r.pairedClass;
    if (CanVibrate(cls)) Post({Command::Type::Identify, r.serial});
    else OpenIdentify(r.serial);
}

void App::DrawIdentifyDialog(AppState& s) {
    if (!ui_.showIdentify) return;
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (!ImGui::IsPopupOpen("识别设备###identify")) ImGui::OpenPopup("识别设备###identify");
    CenterNext(38);
    bool open = true;
    auto close = [&] {
        ui_.showIdentify = false;
        Post({Command::Type::SetFastPoll, "", "", 0});
    };
    if (ImGui::BeginPopupModal("识别设备###identify", &open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        double now = NowSeconds();
        if (ui_.identifyStart == 0) ui_.identifyStart = now;
        BigText(1.3f, ICON_SYNC "  摇一摇识别");
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 35);
        TextDim("拿起要找的设备，用力摇晃几秒。被摇动的设备加速度波动（imustats 的 grav σ）会明显升高，下面会标出它连在哪个接收器上。"
                "VIVE Tracker 和 Tundra Tracker 没有震动马达，只能用这种方式或关机看指示灯来识别；手柄也可以直接点“震动”。");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();

        // Collect current motion per online receiver.
        struct Row {
            Receiver* r;
            float cur;
            float peak;
        };
        std::vector<Row> rows;
        constexpr float kThreshold = 0.30f;   // m/s² std-dev; a resting device sits around 0.02
        for (auto& r : s.receivers) {
            if (r.state != LinkState::Online) continue;
            float cur = 0;
            if (!r.samples.empty() && now - r.samples.back().t < 1.5) cur = r.samples.back().gravSigma;
            float& pk = ui_.identifyPeak[r.serial];
            pk = std::max(pk * 0.985f, cur);   // slowly decaying peak
            rows.push_back({&r, cur, pk});
        }
        std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.peak > b.peak; });
        const Row* winner = nullptr;
        if (!rows.empty() && rows[0].peak >= kThreshold && (rows.size() == 1 || rows[0].peak >= rows[1].peak * 3.0f)) winner = &rows[0];

        // Result banner
        if (BeginCard("##idres", ImVec2(0, 0))) {
            if (winner) {
                TextColored2(p.success, ICON_CHECK);
                ImGui::SameLine();
                ImGui::BeginGroup();
                BigText(1.2f, "就是它：%s", winner->r->DisplayName());
                std::string dev = winner->r->device && !winner->r->device->serial.empty() ? winner->r->device->serial : "设备";
                TextDim("%s · %s · 接收器 %s", dev.c_str(), ToString(winner->r->device ? winner->r->device->deviceClass : winner->r->pairedClass),
                        winner->r->serial.c_str());
                ImGui::EndGroup();
                ImGui::SameLine(0, fs * 1.5f);
                if (AccentButton("选中它")) {
                    ui_.selected = winner->r->serial;
                    close();
                    ImGui::CloseCurrentPopup();
                }
            } else if (rows.empty()) {
                TextColored2(p.warning, ICON_WARNING "  当前没有在线设备。请先打开设备，并确认对应接收器正在监控。");
            } else {
                ProgressRing(fs * 0.8f, 3.0f, -1, p.accent);
                ImGui::SameLine();
                ImGui::BeginGroup();
                BoldText("等待摇晃…");
                TextDim("已等待 %.0f 秒 · 采样加速到每秒 4 次", now - ui_.identifyStart);
                ImGui::EndGroup();
            }
        }
        EndCard();
        ImGui::Spacing();

        if (!rows.empty() && ImGui::BeginTable("##idrows", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("接收器", ImGuiTableColumnFlags_WidthFixed, fs * 8);
            ImGui::TableSetupColumn("设备", ImGuiTableColumnFlags_WidthFixed, fs * 9);
            ImGui::TableSetupColumn("运动强度", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("峰值", ImGuiTableColumnFlags_WidthFixed, fs * 3.5f);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, fs * 4.2f);
            ImGui::TableHeadersRow();
            for (auto& row : rows) {
                Receiver& r = *row.r;
                bool win = winner && winner->r == row.r;
                bool focus = IEquals(r.serial, ui_.identifyFocus);
                ImGui::PushID(r.serial.c_str());
                ImGui::TableNextRow(0, fs * 2.1f);
                if (win) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, Col(p.success, 0.15f));
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                if (win) BoldText("%s", r.DisplayName());
                else ImGui::TextUnformatted(r.DisplayName());
                if (focus) {
                    ImGui::SameLine();
                    TextDim("（当前）");
                }
                ImGui::TableNextColumn();
                DeviceClass cls = r.device ? r.device->deviceClass : r.pairedClass;
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(r.device && !r.device->serial.empty() ? r.device->serial.c_str() : "在线");
                ImGui::SameLine();
                {
                    std::string kindText = IsTracker(cls) ? "追踪器" : CanVibrate(cls) ? "手柄" : "";
                    if (r.device && r.device->hand != Hand::Unknown) kindText += std::string("·") + ToString(r.device->hand);
                    TextDim("%s", kindText.c_str());
                }
                ImGui::TableNextColumn();
                float frac = std::clamp(row.cur / 2.5f, 0.f, 1.f);
                float shown = Animate(ImGui::GetID("bar"), frac, 14.f);
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (ImGui::GetFrameHeight() - fs * 0.55f) * 0.5f);
                Meter(shown, win ? p.success : row.cur >= kThreshold ? p.warning : p.accent, ImVec2(ImGui::GetContentRegionAvail().x, fs * 0.55f));
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                TextColored2(row.peak >= kThreshold ? p.text : p.textDim, "%.2f", row.peak);
                ImGui::TableNextColumn();
                if (CanVibrate(cls)) {
                    if (ImGui::Button(ICON_VIBRATE " 震动")) Post({Command::Type::Identify, r.serial});
                } else {
                    TextDim("无马达");
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        TextDim("运动强度 = 加速度幅值标准差 (m/s²)。静止约 0.02，摇晃时通常大于 0.5。");
        ImGui::Spacing();
        if (ImGui::Button(ICON_CLEAR "  重新开始")) {
            ui_.identifyPeak.clear();
            ui_.identifyStart = now;
        }
        ImGui::SameLine();
        if (ImGui::Button("关闭", ImVec2(fs * 6, 0))) {
            close();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) close();
}

// ============================================================================ settings / about

void App::DrawSettings(AppState& s) {
    if (!ui_.showSettings) return;
    const auto& p = Pal();
    float fs = ImGui::GetFontSize();
    if (!ImGui::IsPopupOpen("设置###settings")) {
        ImGui::OpenPopup("设置###settings");
        ui_.settingsConsolePath = s.settings.consolePathOverride;
    }
    CenterNext(36);
    bool open = true;
    if (ImGui::BeginPopupModal("设置###settings", &open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        Settings& st = s.settings;
        bool changed = false;
        ImGui::SeparatorText("lighthouse_console");
        char buf[512];
        std::snprintf(buf, sizeof buf, "%s", ui_.settingsConsolePath.c_str());
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputTextWithHint("##cp", "留空 = 自动从 SteamVR 安装目录查找", buf, sizeof buf)) ui_.settingsConsolePath = buf;
        TextDim("当前使用: %s", s.consolePath.empty() ? "（未找到）" : s.consolePath.c_str());

        ImGui::SeparatorText("实时监控");
        changed |= ImGui::Checkbox("自动监控所有接收器", &st.autoMonitorAll);
        HelpMarker("为每个接收器保持一个 lighthouse_console 会话，持续读取 imustats / usbstats / battery。");
        ImGui::SetNextItemWidth(fs * 12);
        changed |= ImGui::SliderInt("采样间隔", &st.pollIntervalMs, 250, 5000, "%d ms");
        ImGui::SetNextItemWidth(fs * 12);
        changed |= ImGui::SliderInt("电量刷新", &st.batteryIntervalS, 5, 120, "%d 秒");
        changed |= ImGui::Checkbox("追踪诊断（在监控会话中启用 pose 求解）", &st.trackingDiagnostics);
        HelpMarker("开启后可以看到追踪重置、基站标定等事件，但会占用更多 CPU。对新开始的监控会话生效。");

        ImGui::SeparatorText("外观");
        const char* themes[] = {"深色", "浅色", "跟随系统"};
        ImGui::SetNextItemWidth(fs * 10);
        if (ImGui::Combo("主题", &st.theme, themes, 3)) {
            changed = true;
            themeDirty_ = true;
        }
        ImGui::SetNextItemWidth(fs * 10);
        if (ImGui::SliderFloat("界面缩放", &st.uiScale, 0.8f, 1.6f, "%.2fx")) {
            changed = true;
            themeDirty_ = true;
        }
        changed |= ImGui::Checkbox("半透明背景（Windows 11，重启后生效）", &st.useMica);

        ImGui::SeparatorText("安全");
        changed |= ImGui::Checkbox("专家模式", &st.expertMode);
        HelpMarker("显示“恢复出厂设置”并允许在控制台标签页执行只读的 lighthouse_console 命令。配对/解配/重置类命令始终只能通过带确认的按钮执行。");

        ImGui::SeparatorText("数据");
        TextDim("%s", s.dataDir.c_str());
        if (ImGui::SmallButton(ICON_FOLDER " 打开数据目录")) sys::OpenInExplorer(s.dataDir);
        ImGui::SameLine();
        if (ImGui::SmallButton(ICON_FOLDER " 打开备份目录")) sys::OpenInExplorer(svc_.BackupDir());

        ImGui::Spacing();
        ImGui::Separator();
        if (AccentButton("保存", ImVec2(fs * 7, 0))) {
            st.consolePathOverride = ui_.settingsConsolePath;
            Post({Command::Type::SaveSettings});
            ui_.showSettings = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("关闭", ImVec2(fs * 6, 0))) {
            Post({Command::Type::SaveSettings});
            ui_.showSettings = false;
            ImGui::CloseCurrentPopup();
        }
        (void)changed;
        (void)p;
        ImGui::EndPopup();
    }
    if (!open) {
        Post({Command::Type::SaveSettings});
        ui_.showSettings = false;
    }
}

void App::DrawAbout(AppState& s) {
    if (!ui_.showAbout) return;
    float fs = ImGui::GetFontSize();
    if (!ImGui::IsPopupOpen("关于###about")) ImGui::OpenPopup("关于###about");
    CenterNext(30);
    bool open = true;
    if (ImGui::BeginPopupModal("关于###about", &open, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize)) {
        BigText(1.4f, "Lighthouse 配对管理");
        TextDim("版本 %s%s%s", LHP_VERSION, LHP_BUILD_ID[0] ? " · 构建 " : "", LHP_BUILD_ID);
        ImGui::Spacing();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 27);
        ImGui::TextUnformatted("列出所有 Lighthouse 接收器，查看配对关系、详细设备信息和实时无线信号，并精确地把设备配对到指定接收器。");
        ImGui::Spacing();
        TextDim("需要安装 SteamVR。所有写操作都会先校验目标、自动备份并要求确认。");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (BeginPropTable("##ab")) {
            PropRow("Windows 版本", std::to_string(sys::WindowsBuild()));
            PropRow("SteamVR", s.steamVrRuntime);
            EndPropTable();
        }
        ImGui::Spacing();
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + fs * 27);
        TextDim("Copyright (C) 2026 LianZhou。本程序是自由软件，依据 GNU GPL v3 或更新版本发布，不提供任何担保。"
                "第三方组件：Dear ImGui、ImPlot、nlohmann/json（MIT），FreeType（FTL，Portions of this software are copyright © The FreeType Project）。");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (ImGui::Button("关闭", ImVec2(fs * 6, 0))) {
            ui_.showAbout = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) ui_.showAbout = false;
}

}  // namespace lhp::ui
