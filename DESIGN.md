# LighthousePairing — 设计方案

> 状态：M1–M4 已实现（v0.4.0，见 README.md）。M0 中需要真机写操作的验证项（U1、U3、U5、U6）仍待在空闲接收器上确认。
> 目标平台：Windows 10/11 x64 · C++20 · Dear ImGui v1.92.9b（docking 分支）· Direct3D 11

---

## 1. 目标

做一个桌面工具，用来：

1. **列出所有接收器**：包括头显内置的 Watchman、外置 Valve VR Radio、老款 Watchman 接收器。显示序列号、类型、所在 USB 位置、射频版本。
2. **查看配对情况**：每个接收器配对了哪个设备（LHR-xxxx）、在线还是离线、上次在什么时候看到它。
3. **精确控制配对**：指定"某个接收器 ↔ 某个设备"进行配对，可以单独解除某个接收器的配对，也可以做恢复出厂这类危险操作。全部操作都有引导和二次确认。

**不做的事**：追踪质量分析、固件升级、修改基站设置。

---

## 2. 已经在真机上验证过的事实（2026-10-12）

| 项目 | 结果 |
|---|---|
| 工具位置 | `<SteamVR>\tools\lighthouse\bin\win64\lighthouse_console.exe`，版本 11000037 |
| 本机接收器 | 11 个：9 个 `28DE:2101`（Watchman Dongle），2 个 `28DE:2102`（Valve VR Radio，序列号带 `-LYM` 后缀） |
| 启动时的输出 | `Attached lighthouse receiver devices: N`，后面每行一个序列号，然后自动连接其中一个 |
| `help` 里写了的命令 | `serial`、`deviceinfo`、`identifycontroller`、`version`、`poweroff`、`reboot` 等 |
| **没写进 help 的命令**（从 exe 的字符串里提取） | `dongleinfo`、`pair`、`unpair`、`pairall`、`forcepairall`、`unpairall`、`donglereset`、`dongleresetall`；命令行参数还有 `/pairstop` |
| `dongleinfo` 的输出 | `Dongles:<serial>,<radioVersion>,<?>;...`，第三个字段在本机上全为空 |
| 与配对相关的输出字符串 | `Pairing Start`、`Pairing Success`、`Pairing Failed`、`Could not pair dongle %s`、`Could not unpair dongle %s`、`Skipping connected dongle %s`、`Dongle %s reset to factory defaults...` |
| Pimax 运行时里的同类数据 | `Prop_AllWirelessDongleDescriptions_String = serial,ver,type,channel;...`，以及每个设备的 `Prop_ConnectedWirelessDongle_String` |

Valve 开发人员确认过：`pair` / `unpair` 不接收序列号参数，要先用 `serial <sn>` 选中目标设备再发命令（[Steam 社区](https://steamcommunity.com/app/358720/discussions/0/1630789172630698082/)）。

---

## 3. 需要先验证的问题（M0 阶段）

| # | 问题 | 怎么验证 | 如果答案是否定的 |
|---|---|---|---|
| U1 | `dongleinfo` 第三个字段是不是"已配对设备的序列号" | 在一个已配对且设备在线的接收器上执行 | 改用"连接事件 + 本地历史记录"来推断配对关系（见 §5.4） |
| U2 | 在交互模式下 `serial <dongle>` 能不能选中没有设备在线的接收器 | 实测 | 改为每次操作单独启动一个进程，用命令行参数 `/serial X /pair` |
| U3 | `pair` 是否有超时，超时后输出什么；交互模式下有没有 `pairstop` | 实测并录下完整输出 | 由我们自己计时，超时后发 `/pairstop` 或直接结束进程 |
| U4 | SteamVR（vrserver）或 Pimax（pi_server）运行时，console 是否还能操作接收器 | 分别在两者运行时测试 | 强制要求先关闭（§5.5 冲突检测） |
| U5 | `unpair` 是只清当前选中的接收器，还是会影响其他接收器 | 用一个无关紧要的接收器测试 | 在 UI 里把它标成危险操作 |
| U6 | 设备上线、下线时 console 的输出格式（例如 `Wireless controller LHR-x connected`） | 开关一只手柄，录下输出 | 定期轮询 `deviceinfo` |

**M0 的产出**：一组真实的 console 输出录制（`tests/transcripts/*.txt`），作为解析器单元测试的固定输入。

---

## 4. 总体架构

```
┌──────────────────────────── App (UI 线程) ────────────────────────────┐
│  ImGui 视图层：ReceiverListView / DetailView / PairWizard / LogDock    │
│        ▲ 只读快照 (AppState, 不可变)        │ 用户意图 (Command)         │
└────────┼────────────────────────────────────┼─────────────────────────┘
         │                                    ▼
┌────────┴──────────────── Core (无 UI，可单测) ─────────────────────────┐
│  Store：持有状态，reduce(Event) → 新的 AppState                         │
│  PairingService：高层操作的状态机（配对、解配、识别、重置）              │
│  ConflictGuard：检测 vrserver / pi_server 等占用接收器的进程            │
│  HistoryDb：本地记录 dongle↔设备 的历史、别名、标签 (JSON)              │
├──────────────────────────── Backend 接口 ─────────────────────────────┤
│  ILighthouseBackend                                                   │
│   ├─ ConsoleBackend   ← 包装 lighthouse_console.exe（第一版）          │
│   └─ (NativeHidBackend 以后再做，接口保持不变)                         │
│  UsbTopology：SetupAPI/CfgMgr32 枚举，热插拔通知，判断内置还是外置       │
└───────────────────────────────────────────────────────────────────────┘
```

### 4.1 模块划分

| 模块 | 职责 | 关键技术 |
|---|---|---|
| `platform/Process` | 用 `CreateProcessW` 启动子进程，三个管道，异步按行读取，进程放进 Job Object（主程序退出时子进程一起结束） | Win32、`std::jthread`、`std::stop_token` |
| `backend/ConsoleSession` | 管理一个长期运行的 `lh>` 会话：命令排队、等提示符、超时、崩溃后自动重启 | 状态机 + 命令队列 |
| `backend/ConsoleParser` | 把文本行解析成强类型事件 | `std::regex` 或手写解析，全部有单测 |
| `backend/UsbTopology` | 枚举 `VID_28DE&PID_2101/2102`，找出 USB 父设备链（hub → 端口），判断是否插在头显内部 hub 上；监听热插拔 | `SetupDiGetClassDevs`、`CM_Get_Parent`、`CM_Register_Notification` |
| `core/Store` | 单一状态源，事件驱动 | `std::variant` 事件 |
| `core/PairingService` | 配对向导的状态机、超时、结果校验 | |
| `core/ConflictGuard` | 检测并可选关闭 `vrserver.exe`、`vrmonitor.exe`、`pi_server.exe`、`PiService.exe` | Toolhelp32 |
| `core/HistoryDb` | `%LOCALAPPDATA%\LighthousePairing\history.json` | nlohmann-json |
| `ui/*` | 所有 ImGui 界面代码 | ImGui 1.92.9 docking |

### 4.2 核心数据模型

```cpp
enum class ReceiverKind { HmdBuiltin, VrRadio, WatchmanDongle, Unknown };
enum class LinkState    { Unpaired, PairedOffline, Online, Pairing, Busy, Error };

struct UsbLocation {
    std::wstring instanceId;      // USB\VID_28DE&PID_2101\93F15DFEA6
    std::wstring parentHub;       // 父 hub 的友好名称 / VID:PID
    int          port = -1;
    bool         behindHmdHub = false;   // 父链上出现头显 hub（Index 28DE / Pimax 34A4 / Vive 0BB4）
};

struct PairedDevice {
    std::string serial;           // LHR-9E63AEE0
    std::string model;            // Knuckles Right / VIVE Tracker 3.0 ...
    std::optional<int> batteryPct;
    std::chrono::system_clock::time_point lastSeen;
};

struct Receiver {
    std::string  serial;          // 109031C053 / A04165BAAA-RYB
    uint16_t     pid = 0;         // 0x2101 / 0x2102
    ReceiverKind kind = ReceiverKind::Unknown;
    uint32_t     radioVersion = 0;
    UsbLocation  usb;
    LinkState    state = LinkState::Unpaired;
    std::optional<PairedDevice> paired;
    std::string  nickname;        // 用户起的别名，比如“左手 / 腰追踪器”
    enum class Source { Device, Inferred, History } pairingSource{}; // 配对信息的可信度
};
```

### 4.3 Backend 接口

```cpp
struct BackendEvent;   // std::variant<ReceiversListed, DongleInfo, DeviceConnected,
                       //   DeviceDisconnected, PairingProgress, CommandFailed, RawLine, ...>

class ILighthouseBackend {
public:
    virtual ~ILighthouseBackend() = default;
    virtual void start(std::function<void(BackendEvent)> sink) = 0;
    virtual void stop() = 0;

    // 全部异步执行，结果通过 sink 返回；同一时间只执行一个写操作
    virtual void refresh() = 0;                                   // 列表 + dongleinfo + deviceinfo
    virtual void pair(std::string_view dongle, std::chrono::seconds timeout) = 0;
    virtual void cancelPairing(std::string_view dongle) = 0;
    virtual void unpair(std::string_view dongle) = 0;
    virtual void identify(std::string_view deviceSerial) = 0;     // identifycontroller，手柄震动
    virtual void factoryReset(std::string_view dongle) = 0;       // donglereset（危险）
};
```

C++20 没有 `std::expected`，用 `tl::expected`（vcpkg 包名 `tl-expected`），或者自己写一个很小的 `Result<T>`。

---

## 5. 关键流程

### 5.1 刷新

1. `UsbTopology` 枚举 USB 设备，立刻在 UI 上显示接收器卡片，状态标为"读取中"。
2. `ConsoleSession` 启动（或复用已有会话），解析启动时的接收器列表。
3. 依次发送 `dongleinfo`、`deviceinfo`，合并结果。
4. 和 `HistoryDb` 合并：设备不在线时，显示上次记录到的配对关系，并标明"历史记录"。

### 5.2 精确配对向导

```
选择接收器 ──► 检查：这个接收器上已经有设备？──是──► 提示会覆盖，是否先解配
     │                                         否
     ▼
选择设备类型（决定提示文字和动画）
     │
     ▼
冲突检测：vrserver / pi_server 在运行？──是──► 提示关闭（可一键关闭）
     │
     ▼
serial <dongle>  →  pair  →  倒计时（默认 30s）
     │            UI 显示：“现在把 Index 手柄关机后同时按住 B + 系统键，直到指示灯蓝色闪烁”
     ▼
等待 "Pairing Success" 或 设备连接事件 ──超时──► pairstop，提示重试
     │
     ▼
校验：再执行一次 dongleinfo/deviceinfo，确认设备确实连在“这个”接收器上
     │
     ▼
写入 HistoryDb，提示用户给它起个别名
```

各设备进入配对模式的方法（内置在 UI 提示里）：

| 设备 | 操作 |
|---|---|
| Index 手柄 | 关机状态下同时按住 B + 系统键，直到指示灯蓝色闪烁 |
| Vive 手柄 | 同时按住菜单键 + 系统键 |
| Vive Tracker 2018 / 3.0 | 长按电源键，直到指示灯蓝色闪烁 |
| Tundra Tracker | 长按按钮约 3 秒，直到指示灯蓝色闪烁 |

### 5.3 解配 / 重置（危险区）

- `unpair`：清除单个接收器的配对。需要输入该接收器序列号的后 4 位才能确认。
- `unpairall`、`dongleresetall`：默认在设置里隐藏，打开"专家模式"后才显示，需要输入完整的 `RESET` 确认。
- 每次写操作前，自动把当前快照（dongleinfo 结果 + 历史记录）备份到 `backups/yyyyMMdd-HHmmss.json`。

### 5.4 配对关系从哪里来（按可信度从高到低）

1. **Device**：`dongleinfo` 直接给出（如果 U1 验证成立）。
2. **Inferred**：本次会话里观察到 `<dongle>: Wireless controller LHR-x connected` 这类事件。
3. **History**：以前观察到并保存在 HistoryDb 里的。

UI 用不同的标签区分这三种来源，避免把推断结果当成事实。

### 5.5 冲突检测

lighthouse_console 和 SteamVR、Pimax 运行时会抢同一批 HID 设备。启动时以及每次写操作前都检查：

- `vrserver.exe`、`vrmonitor.exe`、`vrcompositor.exe`
- `pi_server.exe`、`PiService.exe`、`PimaxClient.exe`

检测到时，顶部显示黄色提示条：「SteamVR 正在运行，配对操作可能失败」，并提供「关闭它们」按钮（先尝试正常退出，再强制结束）。只读操作照常允许。

---

## 6. UI 设计

### 6.1 技术选型

| 项 | 选择 | 理由 |
|---|---|---|
| ImGui 版本 | **v1.92.9b**，`docking` 分支（tag `v1.92.9-docking`） | 当前最新；1.92 起字体图集可以动态扩展，中文不用预先烘焙字形范围 |
| 平台 / 渲染后端 | `imgui_impl_win32` + `imgui_impl_dx11` | 最省事、最稳；这个工具用不上 DX12 或 Vulkan 的能力 |
| 字体 | 中文 **HarmonyOS Sans SC** 或 Noto Sans SC，合并 **Material Symbols Rounded** 图标字体 | 现代观感；图标用字体渲染，任意 DPI 下都清晰 |
| 字体光栅化 | `imgui_freetype` | 小字号更清晰，支持彩色 emoji |
| DPI | Per-Monitor V2，`style.ScaleAllSizes` + 字体按 DPI 缩放 | 高分屏不糊 |
| 窗口外观 | Win11 **Mica** 背景（`DwmSetWindowAttribute(DWMWA_SYSTEMBACKDROP_TYPE)`），沉浸式深色标题栏，ImGui 背景半透明 | 和系统原生应用风格一致；Win10 上自动退回纯色 |
| 动画 | 自己写一个很小的 tween 工具（状态切换渐变、卡片悬停抬升、配对时的呼吸灯效果） | ImGui 本身没有动画系统 |
| 提示消息 | 右下角 toast（成功、失败、警告） | |
| 可选 | ImPlot：显示每个接收器的丢包时间线（以后可以接入 Pimax / SteamVR 日志） | |

### 6.2 布局

```
┌───────────────────────────────────────────────────────────────────────┐
│ ◉ LighthousePairing         [⟳ 刷新]  [⚠ SteamVR 运行中 · 关闭它们]  ⚙ │
├──────────────────────┬────────────────────────────────────────────────┤
│ 头显内置 (2)          │  109031C053                     ● 在线          │
│ ┌──────────────────┐ │  头显内置 · Watchman · Radio 1525468496         │
│ │● 109031C053      │ │  USB: Pimax hub → 端口 3                        │
│ │  LHR-9E63AEE0    │ │ ──────────────────────────────────────────────  │
│ │  右手 · 85% 🔋    │ │  已配对设备                                      │
│ └──────────────────┘ │   LHR-9E63AEE0 · Knuckles Right · 85%           │
│ ┌──────────────────┐ │   来源：设备读取  ·  最后在线 刚刚               │
│ │○ 18848BA78B      │ │   [📳 震动识别]  [✎ 别名]                        │
│ │  未配对          │ │ ──────────────────────────────────────────────  │
│ └──────────────────┘ │  操作                                           │
│ 外置 VR Radio (4)     │   [＋ 配对新设备到此接收器]                       │
│  ...                 │   [⛓ 解除此接收器的配对]      (危险操作，需确认) │
│ 老款 Watchman (5)     │                                                 │
│  ...                 │  ▸ 专家选项                                      │
├──────────────────────┴────────────────────────────────────────────────┤
│ 控制台日志  [过滤: ______]  [只看错误]  [复制]  [导出]                    │
│ 21:34:15  lh> serial A04165BAAA-RYB                                    │
│ 21:34:15  lighthouse_console: Connected to receiver A04165BAAA-RYB     │
└───────────────────────────────────────────────────────────────────────┘
```

- 左栏按类型分组的卡片列表，支持搜索、按状态筛选；卡片左侧的圆点颜色表示状态（绿＝在线，灰＝离线，蓝色呼吸＝配对中，红＝错误）。
- 右侧是详情面板；底部是可停靠的控制台日志面板。所有面板都可以拖动、重新停靠（docking），布局保存在 `imgui.ini`。
- 配对向导是一个居中的模态窗口，分步骤显示，有进度环和倒计时。

### 6.3 主题

- 深色为主：背景 `#14161A`（叠加 Mica 半透明），面板 `#1C1F24`，强调色 `#4F8CFF`，成功 `#3DDC97`，警告 `#FFB547`，危险 `#FF5D5D`。
- 圆角：窗口 10、控件 6、卡片 12；适当加大内边距（`FramePadding` 10×6，`ItemSpacing` 10×8）。
- 支持跟随系统的浅色主题（读注册表 `AppsUseLightTheme`）。

---

## 7. 线程模型

| 线程 | 职责 |
|---|---|
| UI 线程 | Win32 消息循环 + ImGui 帧；只读 `AppState` 快照；用户操作作为 Command 投递到队列 |
| Backend 工作线程（`std::jthread`） | 消费命令队列、驱动 `ConsoleSession`，产生事件 |
| 管道读取线程 | 阻塞读子进程的 stdout / stderr，按行拆分后投递 |
| 设备通知 | `CM_Register_Notification` 回调，只投递事件，不做耗时工作 |

UI 线程每帧通过 `Store::drain()` 取出事件并更新状态。没有事件、窗口也不在前台时降低帧率（`WaitMessage` 或者 100ms 超时），节省 CPU。

---

## 8. 工程与构建

```
LighthousePairing/
├─ CMakeLists.txt
├─ CMakePresets.json          # msvc-x64-debug / release
├─ vcpkg.json                 # nlohmann-json, tl-expected, doctest, freetype
├─ cmake/FetchImGui.cmake     # FetchContent 固定 tag v1.92.9-docking
├─ assets/fonts/              # HarmonyOS Sans SC, MaterialSymbolsRounded
├─ src/
│  ├─ main.cpp                # Win32 窗口 + DX11 + ImGui 初始化
│  ├─ platform/  Process.*  Win32Window.*  Dx11Device.*  Mica.*
│  ├─ backend/   ILighthouseBackend.h  ConsoleBackend.*  ConsoleSession.*
│  │             ConsoleParser.*  UsbTopology.*  SteamVrLocator.*
│  ├─ core/      Model.h  Store.*  PairingService.*  ConflictGuard.*  HistoryDb.*
│  └─ ui/        App.*  Theme.*  Fonts.*  ReceiverListView.*  DetailView.*
│                PairWizard.*  LogDock.*  Toasts.*  Anim.h
└─ tests/
   ├─ transcripts/            # M0 录下的真实 console 输出
   ├─ ConsoleParserTests.cpp
   ├─ PairingServiceTests.cpp # 用 FakeBackend 回放录制内容
   └─ FakeBackend.h
```

- 编译选项：`/std:c++20 /permissive- /utf-8 /W4 /EHsc`；Release 加 `/O2 /GL`，链接选项 `/LTCG`；静态 CRT（`/MT`），这样单个 exe 可以直接拷走使用。
- ImGui 不走 vcpkg：vcpkg 里的版本可能落后于最新 tag，用 FetchContent 固定 tag 更可控。
- `SteamVrLocator`：通过 `%LOCALAPPDATA%\openvr\openvrpaths.vrpath` 的 `runtime` 字段找到 SteamVR，从而定位 `lighthouse_console.exe`；也允许在设置里手动指定。
- 程序清单（manifest）：声明 PerMonitorV2 DPI 感知、启用 Common Controls v6；不需要管理员权限。
- 工具链：本机已有 VS 2026 Community（自带 CMake 和 Ninja）以及 vcpkg。

---

## 9. 测试策略

| 层 | 方式 |
|---|---|
| 解析器 | doctest，输入 `tests/transcripts` 里的真实录制，断言解析出的事件序列 |
| 配对状态机 | `FakeBackend` 按时间线回放事件，覆盖成功、超时、失败、被其他程序占用、子进程崩溃 |
| USB 拓扑 | 只做手动测试清单：插拔外置接收器、拔掉头显 |
| UI | 手动测试；可选接入 `imgui_test_engine` 做冒烟测试 |
| 真机 | 每个写操作只在指定的测试接收器上验证，先用 `A04165BAAA-RYB` 这类空闲接收器 |

---

## 10. 里程碑

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| **M0 技术验证** | 命令行原型：启动 console、发命令、录输出；验证 U1–U6 | 每个问题都有明确结论和录制文件 |
| **M1 只读 GUI** | 窗口、主题、字体；接收器列表 + USB 拓扑 + dongleinfo + 冲突检测 + 日志面板 | 本机 11 个接收器全部正确列出并分组；插拔时 2 秒内更新 |
| **M2 配对** | 配对向导、`identifycontroller`、配对结果校验、HistoryDb | 能把指定手柄配对到指定接收器，校验通过 |
| **M3 危险操作** | unpair / reset、自动备份、专家模式 | 误操作必须经过两步确认；有备份可查 |
| **M4 打磨** | Mica、动画、toast、浅色主题、导出诊断包 | 视觉验收 |
| M5（可选） | `NativeHidBackend`：逆向 feature report，脱离 SteamVR 运行 | 和 ConsoleBackend 行为一致 |

---

## 11. 风险

| 风险 | 影响 | 应对 |
|---|---|---|
| 隐藏命令的行为或输出格式随 SteamVR 更新而变化 | 解析失败 | 解析器宽松匹配；未识别的行原样显示在日志里；启动时检查 console 版本号，并对未测试过的版本给出警告 |
| `dongleinfo` 拿不到"配对了谁" | 离线设备的配对关系看不到 | 推断加历史记录，并在 UI 上明确标注来源（§5.4） |
| 和 SteamVR / Pimax 抢设备 | 操作失败或干扰正在运行的 VR | 冲突检测 + 默认禁止写操作 |
| `unpairall` / `dongleresetall` 误操作 | 所有设备都要重新配对 | 默认隐藏 + 输入确认 + 自动备份 |
| 头显内置接收器被识别成外置 | 分组错误 | USB 父链判断 + 用户手动修正，并保存修正结果 |
| 子进程卡住 | UI 状态停住 | 每条命令都有超时；看门狗重启会话；子进程放进 Job Object |
