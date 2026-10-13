// Lighthouse Pairing Manager – entry point.
//
// Command line:
//   --demo                 use synthetic data (no hardware access)
//   --no-mica              disable the DirectComposition / Mica backdrop
//   --fresh-layout         ignore the saved docking layout
//   --size WxH             initial logical window size
//   --select SERIAL        preselect a receiver
//   --tab N                0 overview, 1 signal, 2 device, 3 usb, 4 history, 5 console
//   --open wizard|danger|settings|kill|about|record-notice
//   --screenshot FILE.bmp  render, save the back buffer after --frames frames (default 150) and exit
//   --dump-usb FILE        write the USB topology report and exit
//   --spatial              focus the 3D tracking window
//   --replay FILE.lhrec    feed the tracking pipeline from a recording instead of live HID
//   --tracking-active      start the tracking subsystem in active mode (may switch devices to raw1)
//   --inspector N          spatial inspector tab (0 pose, 1 sensors, 2 bases, 3 data, 4 plots, 5 log)
#include <windows.h>
#include <shellapi.h>

#include "backend/SystemInfo.h"
#include "backend/UsbTopology.h"
#include "core/Service.h"
#include "core/TrackingModels.h"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"
#include "platform/Renderer3D.h"
#include "platform/Window.h"
#include "tracking/service/TrackingService.h"
#include "tracking/sim/Simulator.h"
#include "ui/App.h"
#include "ui/SpatialView.h"
#include "ui/Theme.h"
#include "util/Text.h"

#include <cmath>
#include <string>
#include <vector>

using namespace lhp;

namespace {

struct Args {
    bool demo = false, noMica = false, freshLayout = false;
    int w = 1500, h = 940;
    std::string select, open, screenshot, dumpUsb, replay;
    bool spatial = false, trackingActive = false;
    int inspector = -1;
    int tab = -1, frames = 150, theme = -1;
    bool overview = false;
};

Args ParseArgs() {
    Args a;
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    for (int i = 1; i < argc; ++i) {
        std::string s = ToUtf8(argv[i]);
        auto next = [&]() -> std::string { return i + 1 < argc ? ToUtf8(argv[++i]) : std::string(); };
        if (s == "--demo") a.demo = true;
        else if (s == "--no-mica") a.noMica = true;
        else if (s == "--fresh-layout") a.freshLayout = true;
        else if (s == "--select") a.select = next();
        else if (s == "--tab") a.tab = std::atoi(next().c_str());
        else if (s == "--open") a.open = next();
        else if (s == "--screenshot") a.screenshot = next();
        else if (s == "--frames") a.frames = std::atoi(next().c_str());
        else if (s == "--dump-usb") a.dumpUsb = next();
        else if (s == "--theme") a.theme = std::atoi(next().c_str());
        else if (s == "--overview") a.overview = true;
        else if (s == "--spatial") a.spatial = true;
        else if (s == "--replay") a.replay = next();
        else if (s == "--tracking-active") a.trackingActive = true;
        else if (s == "--inspector") a.inspector = std::atoi(next().c_str());
        else if (s == "--size") {
            std::string v = next();
            sscanf_s(v.c_str(), "%dx%d", &a.w, &a.h);
        }
    }
    LocalFree(argv);
    return a;
}

}  // namespace

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int) {
    Args args = ParseArgs();
    CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

    if (!args.dumpUsb.empty()) {
        auto list = EnumerateUsbReceivers();
        sys::WriteTextFile(args.dumpUsb, DescribeUsb(list));
        return 0;
    }

    Store store;
    std::unique_ptr<IService> service = args.demo ? CreateDemoService(store) : CreateRealService(store);
    service->Start();
    if (args.theme >= 0) store.With([&](AppState& s) { s.settings.theme = args.theme; });
    Settings settings = store.With([](AppState& s) { return s.settings; });
    std::string dataDir = store.With([](AppState& s) { return s.dataDir; });

    Window win;
    bool mica = settings.useMica && !args.noMica && sys::WindowsBuild() >= 22000;
    if (!win.Create(L"Lighthouse 配对管理", args.w, args.h, mica, false)) {
        MessageBoxW(nullptr, L"无法创建 Direct3D 11 窗口。", L"Lighthouse 配对管理", MB_ICONERROR);
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    io.ConfigDockingWithShift = false;
    static std::string iniPath = dataDir + (args.demo ? "\\demo\\imgui.ini" : "\\imgui.ini");
    io.IniFilename = args.freshLayout || !args.screenshot.empty() ? nullptr : iniPath.c_str();

    ImGui_ImplWin32_Init(win.hwnd);
    ImGui_ImplDX11_Init(win.device, win.context);

    auto applyLook = [&](const Settings& st) {
        float scale = win.dpiScale * std::clamp(st.uiScale, 0.6f, 2.0f);
        ui::GetFonts().base = 15.0f;
        ui::ApplyTheme(st.theme, scale, win.composition);
        bool dark = st.theme == 0 || (st.theme == 2 && ui::SystemPrefersDark());
        win.SetDarkTitleBar(dark);
    };
    ui::LoadFonts(1.5f);   // initial guess; refined on the first frames
    applyLook(settings);

    // Spatial tracking: raw HID -> poses (see docs/TRACKING.md). Passive unless the user enables active mode.
    tracking::service::TrackingService tracker(std::make_shared<StoreModelProvider>(store),
                                               args.demo ? nullptr : std::make_shared<FileCalibrationCache>(dataDir + "\\base_stations.json"));
    if (!args.replay.empty()) {
        std::string err;
        tracker.StartReplay(args.replay, &err);
    } else if (args.demo) {
        // Demo: simulated base stations and devices drive the real decode -> solve pipeline.
        auto scene = tracking::sim::DemoScene({"D1E1CD00DA-RYB", "F51A4164A2-RYB", "68C52C2C3A-RYB", "109031C053"});
        std::vector<std::unique_ptr<tracking::io::ReportSource>> sources;
        for (size_t i = 0; i < scene.devices.size(); ++i) sources.push_back(tracking::sim::OpenSimSource(scene, i, (uint16_t)i, true));
        tracker.StartSources(std::move(sources), "模拟数据", "演示模式：模拟的基站与设备数据（走完整的解码与解算流程）", tracking::sim::MakeModelProvider(scene), true);
    } else {
        tracker.Start();
    }
    if (args.trackingActive) tracker.SetOptions({true});
    gfx::Renderer3D renderer3d;
    renderer3d.Init(win.device, win.context);
    ui::SpatialView spatial(tracker, renderer3d, dataDir + "\\recordings");
    if (args.inspector >= 0) spatial.SetInspectorTab(args.inspector);

    ui::App app(store, *service);
    app.AttachSpatial(&spatial);
    if (args.spatial) app.FocusSpatial();
    if (!args.select.empty()) app.Select(args.select);
    if (args.tab >= 0 && args.tab < (int)ui::DetailTab::Count) app.SetTab((ui::DetailTab)args.tab);

    int frame = 0;
    bool running = true;
    while (running) {
        if (!win.PumpMessages()) break;
        if (win.dpiChanged || app.ThemeDirty()) {
            win.dpiChanged = false;
            app.ClearThemeDirty();
            applyLook(store.With([](AppState& s) { return s.settings; }));
        }
        const auto& pal = ui::Pal();
        float clear[4] = {0, 0, 0, 0};
        if (!win.composition) {
            clear[0] = pal.bg.x;
            clear[1] = pal.bg.y;
            clear[2] = pal.bg.z;
            clear[3] = 1;
        }
        if (!win.BeginFrame(clear)) {
            win.WaitForEvents(50);
            continue;
        }
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        // Optical icon alignment: measure on the first frames and rebuild the atlas with the
        // corrected icon offset (converges in one or two passes).
        float iconFix = 0;
        if (frame < 3) {
            float c = ui::MeasureIconMisalignment();
            if (std::fabs(c) > 0.15f) iconFix = c;
        }
        if (frame == 2 && !args.open.empty()) {
            if (args.open == "wizard") app.OpenWizard(args.select);
            else if (args.open == "wizard-run") app.StartWizardPairing(args.select);
            else if (args.open == "settings") app.OpenSettings();
            else if (args.open == "danger") app.OpenUnpairConfirm(args.select);
            else if (args.open == "kill") app.OpenKill();
            else if (args.open == "about") app.OpenAbout();
            else if (args.open == "identify") app.OpenIdentify(args.select);
            else if (args.open == "record-notice") spatial.OpenRecordNotice();
        }
        if (args.overview && frame == 3) ImGui::SetWindowFocus("信号总览###overview");
        app.Frame();
        ImGui::Render();
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        ++frame;
        if (!args.screenshot.empty() && frame >= args.frames) {
            float bg[3] = {pal.dark ? 0.125f : 0.93f, pal.dark ? 0.13f : 0.94f, pal.dark ? 0.15f : 0.96f};
            win.SaveScreenshot(ToWide(args.screenshot), bg);
            running = false;
        }
        win.Present(true);
        if (iconFix != 0) ui::LoadFonts(ui::GetFonts().iconOffsetY + iconFix);
        if (!win.Focused() && !app.WantsFastFrames() && args.screenshot.empty()) win.WaitForEvents(120);
    }

    tracker.Stop();
    renderer3d.Shutdown();
    service->Shutdown();
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    win.Destroy();
    CoUninitialize();
    return 0;
}
