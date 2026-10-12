#pragma once
#include "core/Service.h"
#include "core/Store.h"

#include <map>
#include <string>
#include <vector>

namespace lhp::ui {

class SpatialView;

enum class DetailTab { Overview, Signal, Device, Usb, History, Console, Count };

struct DangerDialog {
    bool open = false;
    bool justOpened = false;
    Command::Type type = Command::Type::Unpair;
    std::string serial;
    char confirm[64] = {};
};

struct PairWizard {
    bool open = false;
    bool justOpened = false;
    int step = 0;              // 0 choose receiver, 1 device type, 2 checklist, 3 running/result
    std::string target;
    int deviceType = 0;
    int timeoutS = 30;
    bool overwriteAck = false;
    bool conflictAck = false;
    uint64_t opId = 0;
};

struct UiState {
    std::string selected;
    DetailTab tab = DetailTab::Overview;
    bool forceTab = false;
    char search[64] = {};
    int filter = 0;            // 0 all, 1 online, 2 idle, 3 attention
    bool showSettings = false;
    bool showAbout = false;
    bool showImGuiDemo = false;
    bool showImPlotDemo = false;
    bool showKill = false;
    DangerDialog danger;
    PairWizard wizard;
    char logFilter[128] = {};
    bool logSelectedOnly = false;
    bool logAutoScroll = true;
    int logMinLevel = 1;       // hide Debug by default
    char nickBuf[64] = {};
    char noteBuf[256] = {};
    std::string editingFor;
    char rawCmd[160] = {};
    int signalWindowS = 300;
    bool signalPaused = false;
    double pausedAt = 0;
    int eventFilter = 0;       // 0 all, 1 gaps, 2 connect/disconnect, 3 tracking
    bool overviewOpen = true;
    std::string settingsConsolePath;
    std::string lastToastKey;
    // shake-identify
    bool showIdentify = false;
    std::string identifyFocus;                    // receiver the user started from (may be empty)
    std::map<std::string, float> identifyPeak;    // receiver -> peak grav sigma during this session
    double identifyStart = 0;
};

/// Quality estimate 0..4 bars from recent samples.
int SignalQuality(const Receiver& r, double now);
const char* DeviceTypeName(int idx);
const char* DeviceTypeHint(int idx);
int DeviceTypeCount();

class App {
public:
    App(Store& store, IService& service);

    /// Draw one frame. Locks the store internally.
    void Frame();

    /// True while something animates (pairing, toasts) so the main loop renders at full rate.
    bool WantsFastFrames() const { return fastFrames_; }

    void Select(const std::string& serial) { ui_.selected = serial; }
    void SetTab(DetailTab t) {
        ui_.tab = t;
        ui_.forceTab = true;
    }
    void OpenWizard(const std::string& target);
    void OpenSettings() { ui_.showSettings = true; }
    void OpenUnpairConfirm(const std::string& serial) { OpenDanger(Command::Type::Unpair, serial); }
    void OpenKill() { ui_.showKill = true; }
    /// Shake-to-identify: works for every device (trackers have no haptic motor).
    void OpenIdentify(const std::string& focus) {
        ui_.showIdentify = true;
        ui_.identifyFocus = focus;
        ui_.identifyPeak.clear();
        ui_.identifyStart = 0;
        Post({Command::Type::SetFastPoll, "", "", 1});
    }
    /// Identify the device on `serial` the best way for its type (vibrate controllers, shake trackers).
    void IdentifyDevice(const Receiver& r);
    void OpenAbout() { ui_.showAbout = true; }
    /// Test hook: open the wizard and immediately start pairing (demo screenshots).
    void StartWizardPairing(const std::string& serial) {
        OpenWizard(serial);
        Command c{Command::Type::Pair, serial, DeviceTypeName(0)};
        c.seconds = 30;
        Post(c);
        ui_.wizard.step = 2;
    }
    bool ThemeDirty() const { return themeDirty_; }
    void ClearThemeDirty() { themeDirty_ = false; }

    void Post(Command c) { pending_.push_back(std::move(c)); }
    /// The 3D tracking window (optional; owned by the caller).
    void AttachSpatial(SpatialView* v) { spatial_ = v; }
    void FocusSpatial() { focusSpatial_ = true; }

private:
    void DrawToolbar(AppState& s);
    void DrawStatusBar(AppState& s);
    void BuildDefaultLayout(unsigned dockId);
    void DrawReceivers(AppState& s);
    void DrawReceiverCard(AppState& s, Receiver& r, float width);
    void DrawDetail(AppState& s);
    void DrawDetailHeader(AppState& s, Receiver& r);
    void DrawTabOverview(AppState& s, Receiver& r);
    void DrawTabSignal(AppState& s, Receiver& r);
    void DrawTabDevice(AppState& s, Receiver& r);
    void DrawTabUsb(AppState& s, Receiver& r);
    void DrawTabHistory(AppState& s, Receiver& r);
    void DrawTabConsole(AppState& s, Receiver& r);
    void DrawOverview(AppState& s);
    void DrawLog(AppState& s);
    void DrawToasts(AppState& s);
    void DrawOpOverlay(AppState& s);
    void DrawPairWizard(AppState& s);
    void DrawDangerDialog(AppState& s);
    void DrawKillDialog(AppState& s);
    void DrawSettings(AppState& s);
    void DrawAbout(AppState& s);
    void DrawIdentifyDialog(AppState& s);
    void ReceiverContextMenu(AppState& s, Receiver& r);
    void OpenDanger(Command::Type t, const std::string& serial);

    Store& store_;
    IService& svc_;
    UiState ui_;
    std::vector<Command> pending_;
    bool fastFrames_ = false;
    bool layoutChecked_ = false;
    bool themeDirty_ = false;
    SpatialView* spatial_ = nullptr;
    bool focusSpatial_ = false;
    std::vector<PairingRecord> historyCache_;
    std::string historyCacheFor_;
    double historyCacheT_ = -100;
};

}  // namespace lhp::ui
