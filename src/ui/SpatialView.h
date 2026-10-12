#pragma once
// "空间" window: 3D view of tracked devices and base stations plus an inspector exposing every
// intermediate value of the solve. Reads TrackingSnapshots only; commands go to the TrackingService.
#include "imgui.h"
#include "platform/Renderer3D.h"
#include "tracking/service/TrackingService.h"

#include <string>

namespace lhp::ui {

class SpatialView {
public:
    SpatialView(tracking::service::TrackingService& svc, gfx::Renderer3D& renderer, std::string recordDir);
    /// Draws the window (call between ImGui::NewFrame and Render).
    void Draw();
    bool WantsFastFrames() const { return visible_; }
    /// 0 pose, 1 sensors, 2 bases, 3 data stream, 4 plots, 5 log (test hook)
    void SetInspectorTab(int t) { forceTab_ = t; }
    static constexpr const char* kWindowName = "空间###spatial";

private:
    using Snap = tracking::service::TrackingSnapshot;
    using Dev = tracking::service::DeviceSnapshot;
    void DrawToolbar(const Snap& s);
    void DrawViewport(const Snap& s, float width, float height);
    void BuildScene(const Snap& s);
    void DrawOverlay(const Snap& s, const ImVec2& origin);
    void FrameScene(const Snap& s);
    void DrawInspector(const Snap& s);
    void TabDevice(const Snap& s, const Dev& d);
    void TabSensors(const Snap& s, const Dev& d);
    void TabBases(const Snap& s, const Dev* d);
    void TabDecode(const Dev& d);
    void TabPlots(const Dev& d);
    void TabLog(const Snap& s);
    const Dev* Selected(const Snap& s);

    tracking::service::TrackingService& svc_;
    gfx::Renderer3D& r3d_;
    std::string recordDir_;
    gfx::OrbitCamera cam_;
    std::string selected_;       // receiver serial
    bool userPicked_ = false;    // the user chose a device explicitly (do not auto-follow)
    bool visible_ = false;
    bool showGrid_ = true, showFrustum_ = true, showRays_ = true, showSensors_ = true, showNormals_ = false, showTrail_ = true,
         showLabels_ = true, showGravity_ = true;
    bool followDevice_ = false;
    bool framed_ = false;
    float baseHeight_ = 2.0f;
    std::string lastError_;
    int sensorBaseFilter_ = -1;
    int forceTab_ = -1;
};

}  // namespace lhp::ui
