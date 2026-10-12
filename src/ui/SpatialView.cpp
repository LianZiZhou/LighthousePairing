#include "ui/SpatialView.h"

#include "backend/SystemInfo.h"
#include "imgui.h"
#include "implot.h"
#include "ui/Icons.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"
#include "util/Text.h"

#include <algorithm>
#include <cmath>

namespace lhp::ui {

using namespace tracking;
using service::BaseSnapshot;
using service::DeviceSnapshot;
using service::SensorSnapshot;
using service::TrackState;

namespace {

constexpr double kRad2Deg = 180.0 / math::kPi;

gfx::V3 G(const math::Vec3& v) { return {(float)v.x, (float)v.y, (float)v.z}; }

gfx::Color BaseColor(int ch, uint8_t a = 255) {
    static const gfx::Color pal[] = {gfx::Rgba(255, 170, 60), gfx::Rgba(80, 200, 255), gfx::Rgba(200, 120, 255), gfx::Rgba(120, 230, 120),
                                     gfx::Rgba(255, 110, 150), gfx::Rgba(240, 230, 90), gfx::Rgba(100, 160, 255), gfx::Rgba(255, 140, 90)};
    gfx::Color c = pal[ch % 8];
    return (c & 0x00FFFFFF) | ((gfx::Color)a << 24);
}
ImVec4 BaseColorV(int ch) { return ImGui::ColorConvertU32ToFloat4(BaseColor(ch)); }

gfx::Color ResidualColor(double absDeg, bool rejected) {
    if (rejected) return gfx::Rgba(230, 80, 230);
    if (absDeg < 0.005) return gfx::Rgba(80, 220, 120);
    if (absDeg < 0.02) return gfx::Rgba(200, 220, 80);
    if (absDeg < 0.1) return gfx::Rgba(250, 170, 60);
    return gfx::Rgba(240, 80, 70);
}

void Euler(const math::Quat& q, double& yaw, double& pitch, double& roll) {
    yaw = std::atan2(2 * (q.w * q.y + q.x * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * kRad2Deg;
    pitch = std::asin(std::clamp(2 * (q.w * q.x - q.y * q.z), -1.0, 1.0)) * kRad2Deg;
    roll = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.x * q.x + q.z * q.z)) * kRad2Deg;
}

std::string ChannelName(int ch) { return Fmt("频道 %d", ch + 1); }
std::string BaseName(const BaseSnapshot& b) {
    return b.hasCalibration ? Fmt("基站 %08X", b.cal.id) : Fmt("基站（%s）", ChannelName(b.channel).c_str());
}

std::string Vec(const math::Vec3& v, double scale, const char* fmt = "%+.1f") {
    std::string f = std::string(fmt) + "  " + fmt + "  " + fmt;
    return Fmt(f.c_str(), v.x * scale, v.y * scale, v.z * scale);
}

const char* ModeShort(protocol::LightMode m) {
    switch (m) {
        case protocol::LightMode::Raw0: return "raw0";
        case protocol::LightMode::Raw1: return "raw1";
        default: return "—";
    }
}

}  // namespace

SpatialView::SpatialView(service::TrackingService& svc, gfx::Renderer3D& renderer, std::string recordDir)
    : svc_(svc), r3d_(renderer), recordDir_(std::move(recordDir)) {}

const DeviceSnapshot* SpatialView::Selected(const Snap& s) {
    const DeviceSnapshot* current = nullptr;
    for (auto& d : s.devices)
        if (d.receiverSerial == selected_) current = &d;
    // keep the user's choice; otherwise follow the most interesting device (pose > data > nothing)
    if (current && (userPicked_ || current->hasPose)) return current;
    const DeviceSnapshot* best = current;
    auto score = [](const DeviceSnapshot& d) { return (d.hasPose ? 1e12 : 0.0) + (double)d.decoder.packets; };
    for (auto& d : s.devices)
        if (!best || score(d) > score(*best)) best = &d;
    return best;
}

void SpatialView::Draw() {
    visible_ = false;
    ImGui::SetNextWindowSize(ImVec2(900, 600), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(kWindowName)) {
        ImGui::End();
        return;
    }
    visible_ = true;
    auto snap = svc_.Snapshot();
    if (!snap) {
        TextDim("空间追踪尚未启动。");
        ImGui::End();
        return;
    }
    const Snap& s = *snap;
    if (const DeviceSnapshot* d = Selected(s)) selected_ = d->receiverSerial;
    DrawToolbar(s);
    float fs = ImGui::GetFontSize();
    if (ImGui::BeginTable("##sp", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV, ImGui::GetContentRegionAvail())) {
        ImGui::TableSetupColumn("view", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("inspector", ImGuiTableColumnFlags_WidthFixed, fs * 30);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImVec2 avail = ImGui::GetContentRegionAvail();
        DrawViewport(s, avail.x, std::max(avail.y, fs * 10));
        ImGui::TableNextColumn();
        DrawInspector(s);
        ImGui::EndTable();
    }
    ImGui::End();
}

void SpatialView::DrawToolbar(const Snap& s) {
    const auto& p = Pal();
    // mode chips
    if (s.simulated) Chip(ICON_PLAY " 模拟数据", p.purple);
    else if (s.replay) Chip(ICON_PLAY " 回放", p.info);
    else if (!s.running) Chip("已停止", p.textDim);
    else if (s.activeAllowed) Chip(ICON_SWITCH " 主动模式", p.warning);
    else Chip(ICON_SHIELD " 被动只读", p.success);
    if (s.host.steamVrRunning) ChipSameLine("SteamVR 运行中", p.purple);
    if (s.recording) ChipSameLine(Fmt(ICON_RECORD " 录制中 %zu", s.recordedReports).c_str(), p.danger, true);
    ChipSameLine(s.gravityAligned ? "已对齐重力" : "未对齐重力", s.gravityAligned ? p.success : p.textDim);

    ImGui::SameLine(0, ImGui::GetFontSize());
    bool active = s.activeRequested;
    ImGui::BeginDisabled(s.replay);
    if (ImGui::Checkbox("主动模式", &active)) svc_.SetOptions({active});
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("被动模式只读取 HID 数据，不向设备发送任何东西。\n"
                          "设备若处于 raw0（脉冲）模式就无法计算 LH2 角度；开启主动模式后，\n"
                          "在 SteamVR 未运行时会把设备切换到 raw1（LH2 解码）模式。\n"
                          "该切换不会写入设备，重新上电或 SteamVR 启动后即恢复。\n\n当前: %s",
                          s.activeNote.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    if (!s.recording) {
        ImGui::BeginDisabled(s.replay);
        if (ImGui::Button(ICON_RECORD " 录制")) {
            std::string path = recordDir_ + "\\" + FormatTimestampForFile() + ".lhrec";
            std::string err;
            if (!svc_.StartRecording(path, &err)) lastError_ = err;
        }
        ImGui::EndDisabled();
    } else if (ImGui::Button(ICON_STOP " 停止录制")) {
        svc_.StopRecording();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_OPENFILE " 回放…")) {
        std::string f = sys::PickOpenFile(L"Lighthouse 录制 (*.lhrec)\0*.lhrec\0所有文件\0*.*\0", recordDir_);
        if (!f.empty()) {
            std::string err;
            if (!svc_.StartReplay(f, &err)) lastError_ = err;
        }
    }
    if (s.replay && !s.simulated) {
        ImGui::SameLine();
        if (ImGui::Button(ICON_SYNC " 回到实时")) svc_.Start();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_REFRESH " 重置空间")) svc_.ResetWorld();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("清除基站位置估计与参考系（OOTX 校准会重新接收）");
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    baseHeight_ = (float)s.referenceHeight;
    if (ImGui::DragFloat("##h", &baseHeight_, 0.01f, 0.2f, 5.0f, "%.2f m")) svc_.SetReferenceHeight(baseHeight_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("参考基站离地高度（地面 y = 0）");
    ImGui::SameLine();
    if (ImGui::Button(ICON_LOCATION " 以设备为地面")) {
        if (!svc_.SetFloorFromDevice(selected_)) lastError_ = "所选设备当前没有位姿";
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("把所选设备最低的传感器高度设为地面（设备平放在地上时使用）");

    // view toggles
    ImGui::SameLine(0, ImGui::GetFontSize());
    if (ImGui::Button(ICON_FILTER " 显示")) ImGui::OpenPopup("##viewopts");
    if (ImGui::BeginPopup("##viewopts")) {
        ImGui::Checkbox("地面网格", &showGrid_);
        ImGui::Checkbox("基站视野", &showFrustum_);
        ImGui::Checkbox("光线（基站 → 被扫到的传感器）", &showRays_);
        ImGui::Checkbox("传感器", &showSensors_);
        ImGui::Checkbox("传感器法线", &showNormals_);
        ImGui::Checkbox("轨迹", &showTrail_);
        ImGui::Checkbox("IMU 重力方向", &showGravity_);
        ImGui::Checkbox("标签", &showLabels_);
        ImGui::Separator();
        ImGui::Checkbox("相机跟随所选设备", &followDevice_);
        if (ImGui::Button("重置视角")) framed_ = false;
        ImGui::EndPopup();
    }
    if (!lastError_.empty()) {
        ImGui::SameLine();
        TextColored2(p.danger, ICON_WARNING " %s", lastError_.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("×")) lastError_.clear();
    }
}

void SpatialView::BuildScene(const Snap& s) {
    using gfx::Rgba;
    const bool dark = Pal().dark;
    if (showGrid_) {
        r3d_.Grid(6.0f, 0.25f, dark ? Rgba(60, 64, 72) : Rgba(205, 208, 214), dark ? Rgba(92, 98, 110) : Rgba(160, 165, 175), 4);
        r3d_.Axes({0, 0.001f, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.5f);
    }
    // base stations
    for (auto& b : s.bases) {
        if (!b.hasPose) continue;
        const auto& P = b.worldFromBase;
        gfx::V3 c = G(P.pos);
        gfx::V3 ax = G(P.rot.Rotate({1, 0, 0})), ay = G(P.rot.Rotate({0, 1, 0})), az = G(P.rot.Rotate({0, 0, 1}));
        r3d_.Box(c, ax * 0.04f, ay * 0.04f, az * 0.03f, dark ? Rgba(40, 42, 48) : Rgba(70, 72, 80));
        r3d_.Box(c - az * 0.031f, ax * 0.03f, ay * 0.03f, az * 0.001f, BaseColor(b.channel));   // front face (looks along -Z)
        r3d_.Axes(c, ax, ay, az, 0.12f);
        if (showFrustum_) {
            // LH2 field of view ~150 x 110 degrees
            const float d = 0.6f, th = std::tan(75.f * 3.14159f / 180.f) * d, tv = std::tan(55.f * 3.14159f / 180.f) * d;
            gfx::V3 corners[4] = {c - az * d + ax * th + ay * tv, c - az * d - ax * th + ay * tv, c - az * d - ax * th - ay * tv, c - az * d + ax * th - ay * tv};
            for (int i = 0; i < 4; ++i) {
                r3d_.Line(c, corners[i], BaseColor(b.channel, 140));
                r3d_.Line(corners[i], corners[(i + 1) % 4], BaseColor(b.channel, 140));
                r3d_.Triangle(c, corners[i], corners[(i + 1) % 4], BaseColor(b.channel, 12));
            }
        }
    }
    // devices
    for (auto& d : s.devices) {
        if (!d.hasPose && d.trail.empty()) continue;
        bool sel = d.receiverSerial == selected_;
        const auto& P = d.worldFromDevice;
        if (showTrail_ && d.trail.size() > 1)
            for (size_t i = 1; i < d.trail.size(); ++i) {
                uint8_t a = (uint8_t)(40 + 200 * i / d.trail.size());
                r3d_.Line(G(d.trail[i - 1]), G(d.trail[i]), sel ? Rgba(110, 190, 255, a) : Rgba(160, 160, 170, a));
            }
        if (!d.hasPose) continue;
        gfx::V3 o = G(P.pos);
        r3d_.Axes(o, G(P.rot.Rotate({1, 0, 0})), G(P.rot.Rotate({0, 1, 0})), G(P.rot.Rotate({0, 0, 1})), 0.08f);
        // bounding box of the sensor constellation
        if (!d.sensors.empty()) {
            math::Vec3 lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
            for (auto& sn : d.sensors)
                for (int k = 0; k < 3; ++k) {
                    lo[k] = std::min(lo[k], sn.point[k]);
                    hi[k] = std::max(hi[k], sn.point[k]);
                }
            math::Vec3 mid = (lo + hi) * 0.5, half = (hi - lo) * 0.5;
            r3d_.WireBox(G(P.Apply(mid)), G(P.rot.Rotate({half.x, 0, 0})), G(P.rot.Rotate({0, half.y, 0})), G(P.rot.Rotate({0, 0, half.z})),
                         sel ? Rgba(110, 190, 255, 160) : Rgba(150, 150, 160, 120));
        }
        if (showSensors_)
            for (auto& sn : d.sensors) {
                math::Vec3 w = P.Apply(sn.point);
                double worst = 0;
                bool rej = false, any = false;
                for (auto& sb : sn.bases)
                    for (int pl = 0; pl < 2; ++pl)
                        if (sb.has[pl] && sb.ageMs[pl] < 60) {
                            any = true;
                            worst = std::max(worst, std::fabs(sb.residualDeg[pl]));
                            rej |= sb.rejected[pl];
                        }
                r3d_.Cube(G(w), sel ? 0.0045f : 0.0035f, any ? ResidualColor(worst, rej) : (dark ? Rgba(90, 92, 100) : Rgba(150, 152, 160)));
                if (showNormals_) r3d_.Line(G(w), G(w + P.rot.Rotate(sn.normal) * 0.02), Rgba(200, 200, 210, 160));
            }
        if (showRays_ && sel)
            for (auto& b : s.bases) {
                if (!b.hasPose) continue;
                for (auto& sn : d.sensors)
                    for (auto& sb : sn.bases)
                        if (sb.channel == b.channel && ((sb.has[0] && sb.ageMs[0] < 60) || (sb.has[1] && sb.ageMs[1] < 60)))
                            r3d_.Line(G(b.worldFromBase.pos), G(P.Apply(sn.point)), BaseColor(b.channel, 90));
            }
        if (showGravity_ && d.imu.valid && math::Norm(d.imu.gravityDevice) > 0.5)
            r3d_.Line(o, o + G(P.rot.Rotate(d.imu.gravityDevice)) * 0.15f, Rgba(250, 220, 80));
    }
}

void SpatialView::DrawViewport(const Snap& s, float width, float height) {
    const DeviceSnapshot* sel = Selected(s);
    if (!framed_) FrameScene(s);
    if (followDevice_ && sel && sel->hasPose) {
        math::Vec3 t = sel->worldFromDevice.pos;
        cam_.target = {(float)t.x, (float)t.y, (float)t.z};
    }
    float dpi = ImGui::GetIO().DisplayFramebufferScale.x;
    int pw = (int)(width * dpi), ph = (int)(height * dpi);
    const auto& bg = Pal().dark ? ImVec4(0.09f, 0.095f, 0.11f, 1) : ImVec4(0.96f, 0.965f, 0.975f, 1);
    float clear[4] = {bg.x, bg.y, bg.z, 1};
    r3d_.Begin(pw, ph, cam_, clear);
    BuildScene(s);
    r3d_.End();

    ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::Image(ImTextureRef((ImTextureID)(intptr_t)r3d_.Texture()), ImVec2(width, height));
    // camera interaction on top of the image
    ImGui::SetCursorScreenPos(origin);
    ImGui::InvisibleButton("##view3d", ImVec2(width, height), ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsItemActive()) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            cam_.yaw -= io.MouseDelta.x * 0.008f;
            cam_.pitch = std::clamp(cam_.pitch + io.MouseDelta.y * 0.008f, -1.5f, 1.5f);
        } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
            gfx::V3 eye = cam_.Eye();
            gfx::V3 fwd = gfx::Normalize(cam_.target - eye);
            gfx::V3 right = gfx::Normalize(gfx::Cross(fwd, {0, 1, 0}));
            gfx::V3 up = gfx::Cross(right, fwd);
            float k = cam_.distance * 0.0015f;
            cam_.target = cam_.target - right * (io.MouseDelta.x * k) + up * (io.MouseDelta.y * k);
            followDevice_ = false;
        }
    }
    if (ImGui::IsItemHovered() && io.MouseWheel != 0) cam_.distance = std::clamp(cam_.distance * std::pow(0.88f, io.MouseWheel), 0.2f, 40.0f);
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && sel && sel->hasPose) {
        math::Vec3 t = sel->worldFromDevice.pos;
        cam_.target = {(float)t.x, (float)t.y, (float)t.z};
        cam_.distance = 1.2f;
    }
    DrawOverlay(s, origin);
}

// Fit the camera to everything that has a position (once a device is placed, or on request).
void SpatialView::FrameScene(const Snap& s) {
    math::Vec3 lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
    bool anyDevice = false;
    auto add = [&](const math::Vec3& v) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], v[k]);
            hi[k] = std::max(hi[k], v[k]);
        }
    };
    for (auto& b : s.bases)
        if (b.hasPose) add(b.worldFromBase.pos);
    for (auto& d : s.devices)
        if (d.hasPose) {
            add(d.worldFromDevice.pos);
            anyDevice = true;
        }
    cam_ = gfx::OrbitCamera{};
    if (!anyDevice) return;   // keep the default view until something is placed
    math::Vec3 c = (lo + hi) * 0.5;
    double extent = std::max(0.5, math::Norm(hi - lo));
    cam_.target = {(float)c.x, (float)c.y, (float)c.z};
    cam_.distance = (float)std::clamp(extent * 1.6, 1.0, 30.0);
    framed_ = true;
}

void SpatialView::DrawOverlay(const Snap& s, const ImVec2& origin) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float dpi = ImGui::GetIO().DisplayFramebufferScale.x;
    const auto& p = Pal();
    auto label = [&](const math::Vec3& w, const std::string& text, ImU32 col) {
        float x, y;
        if (!r3d_.Project(G(w), x, y)) return;
        ImVec2 at(origin.x + x / dpi + 6, origin.y + y / dpi - ImGui::GetFontSize() * 0.5f);
        ImVec2 sz = ImGui::CalcTextSize(text.c_str());
        dl->AddRectFilled(ImVec2(at.x - 3, at.y - 1), ImVec2(at.x + sz.x + 3, at.y + sz.y + 1), Col(p.panel, 0.78f), 3);
        dl->AddText(at, col, text.c_str());
    };
    if (showLabels_) {
        for (auto& b : s.bases)
            if (b.hasPose) label(b.worldFromBase.pos, BaseName(b) + (b.isReference ? " · 参考" : ""), BaseColor(b.channel));
        for (auto& d : s.devices)
            if (d.hasPose)
                label(d.worldFromDevice.pos, (d.deviceSerial.empty() ? d.receiverSerial : d.deviceSerial) + "  " + d.modelNumber,
                      d.receiverSerial == selected_ ? Col(p.accent) : Col(p.text));
    }
    // corner legend
    ImVec2 at(origin.x + 8, origin.y + 6);
    size_t streaming = std::count_if(s.devices.begin(), s.devices.end(), [](const DeviceSnapshot& d) { return d.decoder.packets > 0; });
    std::string info = Fmt("%zu 个设备在发送数据（%zu 个接收器）· %zu 个基站", streaming, s.devices.size(), s.bases.size());
    const DeviceSnapshot* sel = nullptr;
    for (auto& d : s.devices)
        if (d.receiverSerial == selected_) sel = &d;
    if (sel && !sel->hasPose) info += " · " + (sel->hasModel ? (sel->solveNote.empty() ? sel->modeNote : sel->solveNote) : sel->modelNote);
    if (sel && sel->hasPose && s.gravityAligned && sel->worldFromDevice.pos.y < -0.02)
        info += " · 设备在地面以下：调整基站高度，或把设备放在地上后点「以设备为地面」";
    dl->AddText(at, Col(p.textDim), info.c_str());
    dl->AddText(ImVec2(at.x, at.y + ImGui::GetFontSize() * 1.3f), Col(p.textFaint), "左键旋转 · 右键平移 · 滚轮缩放 · 双击聚焦所选设备");
}

void SpatialView::DrawInspector(const Snap& s) {
    const DeviceSnapshot* d = Selected(s);
    // device picker
    ImGui::SetNextItemWidth(-FLT_MIN);
    std::string preview = d ? (d->receiverSerial + (d->deviceSerial.empty() ? "" : "  " + d->deviceSerial)) : "（没有设备）";
    if (ImGui::BeginCombo("##dev", preview.c_str())) {
        // receivers with a device streaming first; idle receivers after a separator
        for (int pass = 0; pass < 2; ++pass) {
            bool any = false;
            for (auto& x : s.devices) {
                bool active = x.decoder.packets > 0;
                if (active != (pass == 0)) continue;
                if (pass == 1 && !any) {
                    ImGui::SeparatorText("没有数据");
                    any = true;
                }
                std::string lbl = x.receiverSerial + (x.deviceSerial.empty() ? "" : "  " + x.deviceSerial + " " + x.modelNumber) + (x.hasPose ? "  ●" : "");
                if (ImGui::Selectable(lbl.c_str(), d && x.receiverSerial == d->receiverSerial)) {
                    selected_ = x.receiverSerial;
                    userPicked_ = true;
                }
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::BeginTabBar("##insp")) {
        auto tab = [&](int i, const char* name) {
            ImGuiTabItemFlags f = forceTab_ == i ? ImGuiTabItemFlags_SetSelected : 0;
            return ImGui::BeginTabItem(name, nullptr, f);
        };
        if (tab(0, "位姿")) {
            if (d) TabDevice(s, *d);
            ImGui::EndTabItem();
        }
        if (tab(1, "传感器")) {
            if (d) TabSensors(s, *d);
            ImGui::EndTabItem();
        }
        if (tab(2, "基站")) {
            TabBases(s, d);
            ImGui::EndTabItem();
        }
        if (tab(3, "数据流")) {
            if (d) TabDecode(*d);
            ImGui::EndTabItem();
        }
        if (tab(4, "曲线")) {
            if (d) TabPlots(*d);
            ImGui::EndTabItem();
        }
        if (tab(5, "日志")) {
            TabLog(s);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
        forceTab_ = -1;
    }
}

void SpatialView::TabDevice(const Snap& s, const Dev& d) {
    const auto& p = Pal();
    const auto& f = d.filter;
    ImGui::BeginChild("##devtab");
    if (BeginPropTable("##id", ImGui::GetFontSize() * 8)) {
        PropRow("接收器", d.receiverSerial, true);
        PropRow("设备", d.deviceSerial.empty() ? "（未知）" : d.deviceSerial + "  " + d.modelNumber, true);
        PropRow("设备模型", d.hasModel ? Fmt("%zu 个传感器", d.sensors.size()) : d.modelNote);
        PropRow("光数据模式", d.modeNote.empty() ? ModeShort(d.lightMode) : d.modeNote);
        ImVec4 sc = f.state == TrackState::Tracking ? p.success : f.state == TrackState::ImuOnly ? p.warning : f.state == TrackState::Lost ? p.danger : p.textDim;
        std::string st = std::string(service::ToString(f.state)) + " · " + f.note;
        if (f.state == TrackState::Tracking || f.state == TrackState::ImuOnly) st += Fmt("（%.0f 秒）", f.trackingSeconds);
        PropRow("跟踪状态", st, false, &sc);
        if (f.resets) PropRow("重置次数", Fmt("%llu · 最近一次：%s", (unsigned long long)f.resets, f.lastResetReason.c_str()));
        EndPropTable();
    }
    ImGui::Spacing();
    SectionTitle(ICON_LOCATION, "位姿", d.hasPose ? (s.gravityAligned ? "（世界坐标，Y 向上，单位 m）" : "（参考基站坐标系，尚未对齐重力）") : nullptr);
    if (!d.hasPose) {
        TextColored2(p.warning, "%s", d.hasModel ? (d.solveNote.empty() ? f.note.c_str() : d.solveNote.c_str()) : d.modelNote.c_str());
    }
    if (BeginPropTable("##pose", ImGui::GetFontSize() * 8)) {
        const auto& P = d.worldFromDevice;
        double yaw, pitch, roll;
        Euler(P.rot, yaw, pitch, roll);
        PropRow("位置 x y z", Vec(P.pos, 1000, "%+.1f") + " mm", true);
        PropRow("离地高度", Fmt("%.3f m", P.pos.y));
        PropRow("姿态 偏航/俯仰/横滚", Fmt("%+.2f°  %+.2f°  %+.2f°", yaw, pitch, roll));
        PropRow("四元数 w x y z", Fmt("%+.5f %+.5f %+.5f %+.5f", P.rot.w, P.rot.x, P.rot.y, P.rot.z), true);
        PropRow("速度", Fmt("%.3f m/s  (%s)", math::Norm(d.velocity), Vec(d.velocity, 1, "%+.3f").c_str()));
        if (d.velocityEstimated) {
            PropRow("角速度（滤波）", Fmt("%.1f °/s  (%s)", math::Norm(d.angularVelocity) * kRad2Deg, Vec(d.angularVelocity, kRad2Deg, "%+.1f").c_str()));
            PropRow("线加速度（不含重力）", Fmt("%.2f m/s²  (%s)", math::Norm(f.acceleration), Vec(f.acceleration, 1, "%+.2f").c_str()));
        }
        if (d.imu.valid) PropRow("角速度（陀螺仪原始）", Fmt("%.1f °/s", math::Norm(d.imu.gyro) * kRad2Deg));
        for (auto& b : s.bases)
            if (b.hasPose) PropRow(("到" + BaseName(b)).c_str(), Fmt("%.3f m", math::Norm(P.pos - b.worldFromBase.pos)));
        PropRow("距上次光学更新", Fmt("%.0f ms", d.poseAgeMs));
        if (d.imu.gravityErrorDeg >= 0) {
            ImVec4 gc = d.imu.gravityErrorDeg < 3 ? p.success : d.imu.gravityErrorDeg < 8 ? p.warning : p.danger;
            PropRow("IMU 重力 vs 光学姿态", Fmt("%.2f°", d.imu.gravityErrorDeg), false, &gc);
        }
        EndPropTable();
    }

    const bool filt = f.state == TrackState::Tracking || f.state == TrackState::ImuOnly || f.state == TrackState::Lost;
    ImGui::Spacing();
    SectionTitle(ICON_DIAG, "跟踪滤波器", "（误差状态迭代卡尔曼：光学 + IMU）");
    if (!filt) {
        TextDim("%s", f.note.c_str());
    } else if (BeginPropTable("##flt", ImGui::GetFontSize() * 8)) {
        const auto& C = f.poseCovariance;
        auto sd = [&](int i, double k) { return std::sqrt(std::max(0.0, C[i * 7])) * k; };
        PropRow("位置 σ", Fmt("%.2f mm  (x %.2f  y %.2f  z %.2f)", f.posSigmaMm, sd(3, 1000), sd(4, 1000), sd(5, 1000)));
        PropRow("姿态 σ", Fmt("%.3f°  (rx %.3f  ry %.3f  rz %.3f)", f.rotSigmaDeg, sd(0, kRad2Deg), sd(1, kRad2Deg), sd(2, kRad2Deg)));
        PropRow("速度 / 角速度 σ", Fmt("%.1f mm/s  ·  %.2f °/s", f.velSigma * 1000, f.angVelSigma * kRad2Deg));
        PropRow("陀螺仪零偏（估计）", Vec(f.gyroBias, kRad2Deg, "%+.3f") + Fmt(" °/s  ±%.3f", f.gyroBiasSigma * kRad2Deg));
        PropRow("加速度计零偏（估计）", Vec(f.accBias, 1, "%+.3f") + Fmt(" m/s²  ±%.3f", f.accBiasSigma));
        {
            std::string why;
            for (auto& [r, n] : f.skipReasons) why += (why.empty() ? "：" : "，") + r + Fmt(" %llu", (unsigned long long)n);
            PropRow("光学更新", Fmt("%.0f 次/秒 · 共 %llu 次，跳过 %llu", f.lightUpdateHz, (unsigned long long)f.lightUpdates, (unsigned long long)f.lightSkipped) + why);
        }
        PropRow("角度观测 / 被门限拒绝", Fmt("%llu / %llu  (%.2f%%)", (unsigned long long)f.lightMeasurements, (unsigned long long)f.lightGated,
                                           f.lightMeasurements ? 100.0 * f.lightGated / f.lightMeasurements : 0.0));
        if (f.lastChannel >= 0) {
            ImVec4 rc = f.lastRmsAfterDeg < 0.01 ? p.success : f.lastRmsAfterDeg < 0.05 ? p.warning : p.danger;
            PropRow("最近一次更新", Fmt("%s · 用 %d，拒 %d，背向 %d · 迭代 %d", ChannelName(f.lastChannel).c_str(), f.lastUsed, f.lastGated, f.lastFacingAway, f.lastIterations));
            PropRow("残差 RMS 更新前 → 后", Fmt("%.4f° → %.4f°", f.lastRmsBeforeDeg, f.lastRmsAfterDeg), false, &rc);
            PropRow("归一化新息 NIS", Fmt("%.2f（≈1 表示噪声模型与实际一致）", f.lastNis));
            PropRow("批次跨度 / 滞后", Fmt("%.1f ms / %.1f ms", f.lastSpanMs, f.lastLagMs));
            if (!f.lastNote.empty()) PropRow("未应用原因", f.lastNote);
        }
        PropRow("IMU 更新", Fmt("%llu 次 · 加速度计%s", (unsigned long long)f.imuUpdates, f.accUsed ? "参与" : "未参与（世界尚未对齐重力）"));
        PropRow("加速度残差（IMU 系）", Vec(f.accResidual, 1, "%+.3f") + " m/s²");
        PropRow("角速度残差（IMU 系）", Vec(f.gyroResidual, kRad2Deg, "%+.3f") + " °/s");
        PropRow("静止检测 / 零速更新", Fmt("%s%s · %s", f.still ? "静止" : "运动中", f.still ? Fmt(" %.1f 秒", f.stillSeconds).c_str() : "", f.zvu ? "零速约束生效" : "无"));
        PropRow("单次光学更新耗时", Fmt("%.0f µs", f.lightUpdateUs));
        EndPropTable();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("状态：位置、姿态、速度、角速度、线加速度、陀螺仪零偏、加速度计零偏（21 维误差状态）。\n"
                          "每圈扫描的角度按各自的命中时刻外推后一起融合；IMU 每个样本都更新一次。");

    ImGui::Spacing();
    SectionTitle(ICON_CHART, "单帧解算", "（启动滤波器、校验、推算基站）");
    if (BeginPropTable("##q", ImGui::GetFontSize() * 8)) {
        ImVec4 rc = d.rmsDeg < 0.01 ? p.success : d.rmsDeg < 0.05 ? p.warning : p.danger;
        PropRow("重投影残差 RMS", Fmt("%.4f°  (%.1f 角秒)", d.rmsDeg, d.rmsDeg * 3600), false, &rc);
        PropRow("最大残差 / 初值残差", Fmt("%.4f° / %.4f°", d.maxDeg, d.initialRmsDeg));
        PropRow("使用 / 剔除观测", Fmt("%d / %d  · %d 个基站", d.used, d.rejected, d.basesUsed));
        PropRow("迭代 / 种子", Fmt("%d / %d", d.iterations, d.seedsTried));
        PropRow("位置 / 姿态 σ", Fmt("%.2f mm / %.3f°", d.positionSigmaMm, d.rotationSigmaDeg));
        if (f.hasBatch) PropRow("与滤波器之差", Fmt("%.1f mm / %.2f°", f.batchDiffMm, f.batchDiffDeg));
        PropRow("解算耗时", Fmt("%.0f µs", d.solveUs));
        PropRow("解算次数 / 失败", Fmt("%llu / %llu", (unsigned long long)d.solves, (unsigned long long)d.solveFailures));
        if (!d.solveNote.empty()) PropRow("最近失败原因", d.solveNote);
        EndPropTable();
    }
    ImGui::Spacing();
    const std::array<double, 36>& cov = filt ? f.poseCovariance : d.covariance;
    if (ImGui::CollapsingHeader(filt ? "位姿协方差（滤波器，rx ry rz tx ty tz）" : "位姿协方差（单帧解算，rx ry rz tx ty tz）")) {
        if (ImGui::BeginTable("##cov", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
            for (int r = 0; r < 6; ++r) {
                ImGui::TableNextRow();
                for (int c = 0; c < 6; ++c) {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.2e", cov[r * 6 + c]);
                }
            }
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();
}

void SpatialView::TabSensors(const Snap& s, const Dev& d) {
    if (d.sensors.empty()) {
        TextDim("%s", d.hasModel ? "还没有传感器数据" : d.modelNote.c_str());
        return;
    }
    // base filter
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    std::string cur = sensorBaseFilter_ < 0 ? "全部基站" : ChannelName(sensorBaseFilter_);
    if (ImGui::BeginCombo("##bf", cur.c_str())) {
        if (ImGui::Selectable("全部基站", sensorBaseFilter_ < 0)) sensorBaseFilter_ = -1;
        for (auto& b : s.bases)
            if (ImGui::Selectable((BaseName(b) + " · " + ChannelName(b.channel)).c_str(), sensorBaseFilter_ == b.channel)) sensorBaseFilter_ = b.channel;
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    int lit = 0;
    for (auto& sn : d.sensors) lit += sn.litNow;
    TextDim("%d / %zu 个传感器正在被扫到", lit, d.sensors.size());
    ImGuiTableFlags f = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable;
    if (ImGui::BeginTable("##sens", 9, f, ImGui::GetContentRegionAvail())) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("#");
        ImGui::TableSetupColumn("ID");
        ImGui::TableSetupColumn("基站");
        ImGui::TableSetupColumn("角度 面1");
        ImGui::TableSetupColumn("角度 面2");
        ImGui::TableSetupColumn("残差 面1");
        ImGui::TableSetupColumn("残差 面2");
        ImGui::TableSetupColumn("命中");
        ImGui::TableSetupColumn("年龄");
        ImGui::TableHeadersRow();
        const auto& p = Pal();
        for (auto& sn : d.sensors) {
            bool printed = false;
            for (auto& sb : sn.bases) {
                if (sensorBaseFilter_ >= 0 && sb.channel != sensorBaseFilter_) continue;
                printed = true;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                StatusDot(sn.litNow ? p.success : p.textFaint, ImGui::GetFontSize() * 0.22f);
                ImGui::SameLine();
                ImGui::Text("%d", sn.index);
                ImGui::TableNextColumn();
                ImGui::Text("%d", sn.reportedId);
                ImGui::TableNextColumn();
                TextColored2(BaseColorV(sb.channel), "%d", sb.channel + 1);
                for (int pl = 0; pl < 2; ++pl) {
                    ImGui::TableNextColumn();
                    if (sb.has[pl]) ImGui::Text("%+9.4f°", sb.angleDeg[pl]);
                    else TextDim("—");
                }
                for (int pl = 0; pl < 2; ++pl) {
                    ImGui::TableNextColumn();
                    if (sb.has[pl] && sb.ageMs[pl] < 60) {
                        ImVec4 c = ImGui::ColorConvertU32ToFloat4(ResidualColor(std::fabs(sb.residualDeg[pl]), sb.rejected[pl]));
                        TextColored2(c, "%+.4f°%s", sb.residualDeg[pl], sb.rejected[pl] ? " ✕" : "");
                    } else {
                        TextDim("—");
                    }
                }
                ImGui::TableNextColumn();
                ImGui::Text("%llu/%llu", (unsigned long long)sb.hits[0], (unsigned long long)sb.hits[1]);
                ImGui::TableNextColumn();
                double age = std::min(sb.has[0] ? sb.ageMs[0] : 1e9, sb.has[1] ? sb.ageMs[1] : 1e9);
                if (age < 1e8) ImGui::Text("%.0f ms", age);
                else TextDim("—");
            }
            if (!printed && sensorBaseFilter_ < 0) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                StatusDot(p.textFaint, ImGui::GetFontSize() * 0.22f);
                ImGui::SameLine();
                ImGui::Text("%d", sn.index);
                ImGui::TableNextColumn();
                ImGui::Text("%d", sn.reportedId);
                ImGui::TableNextColumn();
                TextDim("从未被扫到");
            }
        }
        ImGui::EndTable();
    }
}

void SpatialView::TabBases(const Snap& s, const Dev* d) {
    ImGui::BeginChild("##bases");
    SectionTitle(ICON_GLOBE, "空间", "（基站布局与水平校正）");
    if (BeginPropTable("##uni", ImGui::GetFontSize() * 8)) {
        PropRow("水平基准", s.gravityAligned ? "参考基站加速度计" + (s.tiltSamples ? Fmt(" + 设备 IMU 校正（%d 个静止样本）", s.tiltSamples) : std::string())
                                            : std::string("尚未对齐（等待参考基站 OOTX）"));
        if (s.tiltSamples) PropRow("相对基站加速度计的校正", Fmt("%.2f°", s.tiltCorrectionDeg));
        PropRow("记忆的空间", s.knownUniverses ? Fmt("%d 个（SteamVR 房间设置 / 上次保存）", s.knownUniverses) : std::string("无（基站位置需由设备观测推算）"));
        if (!s.universeNote.empty()) PropRow("最近", s.universeNote);
        EndPropTable();
    }
    bool tilt = s.useDeviceTilt;
    if (ImGui::Checkbox("用静止设备的加速度计校正水平", &tilt)) svc_.SetUseDeviceTilt(tilt);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("SteamVR 也会用设备数据修正基站的水平（\"Updating tilt\"）。基站自带的加速度计只有 8 位精度。");
    ImGui::Spacing();
    if (s.bases.empty()) TextDim("还没有看到基站。设备需要处于 raw1 模式并能看到基站。");
    for (auto& b : s.bases) {
        ImGui::PushID(b.channel);
        ImGui::PushStyleColor(ImGuiCol_Text, BaseColorV(b.channel));
        bool open = ImGui::CollapsingHeader((BaseName(b) + " · " + ChannelName(b.channel) + (b.isReference ? " · 参考" : "")).c_str(), ImGuiTreeNodeFlags_DefaultOpen);
        ImGui::PopStyleColor();
        if (open && BeginPropTable("##b", ImGui::GetFontSize() * 8)) {
            PropRow("旋转频率", Fmt("%.4f Hz（标称 %.4f，偏差 %+.4f）", b.measuredHz, lighthouse::ChannelFrequency(b.channel), b.freqErrorHz));
            PropRow("同步 / 跳过", Fmt("%llu / %llu  (%.1f%%)", (unsigned long long)b.syncs, (unsigned long long)b.skippedSyncs,
                                     b.syncs ? 100.0 * b.skippedSyncs / (b.syncs + b.skippedSyncs) : 0.0));
            PropRow("扫描命中", Fmt("%llu", (unsigned long long)b.sweeps));
            if (b.conflict) {
                ImVec4 c = Pal().danger;
                PropRow("频道冲突", "两台基站使用了同一频道，追踪会失败", false, &c);
            }
            std::string seen;
            for (auto& x : b.seenBy) seen += (seen.empty() ? "" : ", ") + x;
            PropRow("被这些接收器看到", seen);
            PropRow("OOTX", b.calibrationFromCache ? Fmt("使用上次保存的校准，正在重新接收 %u / %u 字节", b.ootx.progressBytes, b.ootx.expectedBytes) : b.hasCalibration ? Fmt("已接收（%u 包，坏 CRC %u，猜测位 %u）", b.ootx.packets, b.ootx.badCrc, b.ootx.guessedBits)
                                              : Fmt("接收中 %u / %u 字节（每圈 1 bit，约需 10 秒）", b.ootx.progressBytes, b.ootx.expectedBytes));
            if (b.hasPose) {
                double yaw, pitch, roll;
                Euler(b.worldFromBase.rot, yaw, pitch, roll);
                PropRow("位置", Vec(b.worldFromBase.pos, 1, "%+.3f") + " m", true);
                PropRow("姿态 偏航/俯仰/横滚", Fmt("%+.1f°  %+.1f°  %+.1f°", yaw, pitch, roll));
                std::string src = b.poseSource;
                if (!b.isReference) {
                    src += b.poseFixed ? std::string(" · 固定") : Fmt(" · %d 个样本", b.bootstrapSamples);
                    src += Fmt(" · 最新偏差 %.1f mm", b.bootstrapSpreadMm);
                    if (b.bootstrapRejected) src += Fmt(" · 连续离群 %d", b.bootstrapRejected);
                }
                PropRow("位置来源", src);
                if (d && d->hasPose) PropRow("到所选设备", Fmt("%.3f m", math::Norm(d->worldFromDevice.pos - b.worldFromBase.pos)));
            } else {
                PropRow("位置", "未知（需要一个同时能看到参考基站和它的设备）");
            }
            if (b.hasCalibration) {
                const auto& c = b.cal;
                PropRow("基站 ID", Fmt("%08X", c.id), true);
                PropRow("固件 / 协议", Fmt("%u / %u", c.firmware >> 6, c.firmware & 0x3F));
                PropRow("模式 / 故障标志", Fmt("%u / 0x%02X", c.mode, c.sysFaults));
                PropRow("加速度计方向", Fmt("%d  %d  %d", c.accelDir[0], c.accelDir[1], c.accelDir[2]));
                PropRow("OOTX 模型 / 失锁计数", Fmt("%u / %u", c.ootxModel, c.sysUnlockCount));
                for (int a = 0; a < 2; ++a) {
                    const auto& x = c.axis[a];
                    PropRow(a ? "面2 校准" : "面1 校准", Fmt("相位 %+.5f  倾斜 %+.5f  曲率 %+.5f", x.phase, x.tilt, x.curve), true);
                    PropRow(a ? "面2 gibbous/ogee" : "面1 gibbous/ogee", Fmt("gib %+.4f / %+.5f  ogee %+.4f / %+.5f", x.gibPhase, x.gibMag, x.ogeePhase, x.ogeeMag), true);
                }
            }
            EndPropTable();
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
}

void SpatialView::TabDecode(const Dev& d) {
    ImGui::BeginChild("##dec");
    const auto& st = d.decoder;
    if (BeginPropTable("##t", ImGui::GetFontSize() * 9)) {
        PropRow("HID 设备", Fmt("%04X  %s", d.pid, d.product.c_str()));
        PropRow("报告速率", Fmt("%.0f /s  （最大间隔 %.1f ms）", d.reportRate, d.hostIntervalMaxMs));
        PropRow("报告 / 数据包", Fmt("%llu / %llu", (unsigned long long)st.reports, (unsigned long long)st.packets));
        PropRow("光数据格式", Fmt("%s  （判定票数 raw0 %llu / raw1 %llu）", protocol::ToString(d.lightMode), (unsigned long long)st.raw0Votes, (unsigned long long)st.raw1Votes));
        PropRow("IMU 样本", Fmt("%llu  （%.1f Hz，设备时钟）", (unsigned long long)st.imu, d.imu.rateHz));
        PropRow("LH2 同步 / 扫描", Fmt("%llu / %llu", (unsigned long long)st.syncs, (unsigned long long)st.sweeps));
        PropRow("raw0 脉冲", Fmt("%llu", (unsigned long long)st.pulses));
        PropRow("电量 / 按键 / 心跳", Fmt("%llu / %llu / %llu", (unsigned long long)st.battery, (unsigned long long)st.input, (unsigned long long)st.heartbeats));
        PropRow("解码错误", Fmt("%llu  （未知报告 %llu）", (unsigned long long)st.errors, (unsigned long long)st.unknownReports));
        if (d.batteryPct >= 0) PropRow("电量", Fmt("%d%%%s", d.batteryPct, d.charging ? "（充电中）" : ""));
        EndPropTable();
    }
    ImGui::Spacing();
    SectionTitle(ICON_CHART, "IMU");
    if (d.imu.valid && BeginPropTable("##imu", ImGui::GetFontSize() * 9)) {
        PropRow("原始加速度", Fmt("%6d %6d %6d", d.imu.rawAccel[0], d.imu.rawAccel[1], d.imu.rawAccel[2]), true);
        PropRow("原始角速度", Fmt("%6d %6d %6d", d.imu.rawGyro[0], d.imu.rawGyro[1], d.imu.rawGyro[2]), true);
        PropRow("加速度（校准后）", Vec(d.imu.accel, 1, "%+.3f") + Fmt(" m/s²  |a| = %.3f", math::Norm(d.imu.accel)));
        PropRow("角速度（校准后）", Vec(d.imu.gyro, kRad2Deg, "%+.2f") + " °/s");
        PropRow("重力方向（设备系）", Vec(d.imu.gravityDevice, 1, "%+.3f"));
        PropRow("量程假设", "±8 g / ±2000 °/s");
        EndPropTable();
    }
    ImGui::Spacing();
    SectionTitle(ICON_WIFI, "每个频道（本设备看到的）");
    if (ImGui::BeginTable("##ch", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("频道");
        ImGui::TableSetupColumn("Hz");
        ImGui::TableSetupColumn("同步");
        ImGui::TableSetupColumn("跳过");
        ImGui::TableSetupColumn("扫描");
        ImGui::TableSetupColumn("丢弃/边界");
        ImGui::TableSetupColumn("OOTX");
        ImGui::TableHeadersRow();
        for (int ch = 0; ch < lighthouse::kMaxChannels; ++ch) {
            const auto& c = d.channels[ch];
            if (!c.syncs && !c.sweeps) continue;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            TextColored2(BaseColorV(ch), "%d%s", ch + 1, c.conflict ? " ⚠" : "");
            ImGui::TableNextColumn();
            ImGui::Text("%.4f", c.measuredHz);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", (unsigned long long)c.syncs);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", (unsigned long long)c.skippedSyncs);
            ImGui::TableNextColumn();
            ImGui::Text("%llu", (unsigned long long)c.sweeps);
            ImGui::TableNextColumn();
            ImGui::Text("%llu/%llu", (unsigned long long)c.droppedSweeps, (unsigned long long)c.borderlineSweeps);
            ImGui::TableNextColumn();
            ImGui::Text("%u pk %u crc", c.ootx.packets, c.ootx.badCrc);
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
}

void SpatialView::TabPlots(const Dev& d) {
    if (d.history.size() < 2) {
        TextDim("还没有位姿历史。");
        return;
    }
    std::vector<double> t, rms, sig, x, y, z, used, diff, speed;
    double t0 = d.history.back().t;
    for (auto& h : d.history) {
        t.push_back(h.t - t0);
        rms.push_back(h.rmsDeg);
        sig.push_back(h.sigmaMm);
        x.push_back(h.pos.x * 1000);
        y.push_back(h.pos.y * 1000);
        z.push_back(h.pos.z * 1000);
        used.push_back(h.used);
        diff.push_back(h.batchDiffMm >= 0 ? h.batchDiffMm : std::nan(""));
        speed.push_back(h.speed * 1000);
    }
    int n = (int)t.size();
    float h = std::max(ImGui::GetFontSize() * 7, (ImGui::GetContentRegionAvail().y - ImGui::GetFontSize()) / 4);
    if (ImPlot::BeginPlot("##motion", ImVec2(-1, h), ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("秒", "mm · mm/s", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::PlotLine("滤波器 vs 单帧解算", t.data(), diff.data(), n);
        ImPlot::PlotLine("速度", t.data(), speed.data(), n);
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##rms", ImVec2(-1, h), ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("秒", "°", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y2, "mm", ImPlotAxisFlags_AuxDefault | ImPlotAxisFlags_AutoFit);
        ImPlot::PlotLine("残差 RMS", t.data(), rms.data(), n);
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
        ImPlot::PlotLine("位置 σ", t.data(), sig.data(), n);
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##pos", ImVec2(-1, h), ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("秒", "mm", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::PlotLine("x", t.data(), x.data(), n);
        ImPlot::PlotLine("y", t.data(), y.data(), n);
        ImPlot::PlotLine("z", t.data(), z.data(), n);
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##used", ImVec2(-1, h), ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("秒", "观测数", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::PlotLine("使用的角度观测", t.data(), used.data(), n);
        ImPlot::EndPlot();
    }
}

void SpatialView::TabLog(const Snap& s) {
    ImGui::BeginChild("##log");
    ImGui::PushFont(GetFonts().mono, 0.0f);
    for (auto& l : s.log) ImGui::TextUnformatted(l.c_str());
    ImGui::PopFont();
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
}

}  // namespace lhp::ui
