# Lighthouse 配对管理 (LighthousePairing)

一个 Windows 桌面工具，用来：

- **列出所有 Lighthouse 接收器**：头显内置的 Watchman、外置 Valve VR Radio (28DE:2102)、老款 Watchman 接收器 (28DE:2101)，按类型分组显示。
- **设备身份**：设备上线后用 `downloadconfig` 读取设备自身的 JSON 配置（与 SteamVR 读取的相同），得到准确的序列号、产品名（如 `VIVE Tracker 3.0 MV`、`Knuckles Left`）、制造商、渲染模型和左右手；读取失败时用 SteamVR 缓存 `config\lighthouse\<序列号>\config.json`，追踪器角色取自 `steamvr.vrsettings`。
- **查看配对关系与设备详情**：每个接收器当前连接 / 历史配对的设备（LHR-xxxxxxxx）、型号、固件、硬件版本、电量，以及 `deviceinfo` / `version` 的全部原始字段。
- **实时信号**：IMU 采样率、包间隔与抖动 σ、USB 包率与最大间隔、无线丢包（keepalive "Packet received after Xs"）、断开、追踪重置；1/5/15 分钟曲线 + 事件表 + 信号质量评分；“信号总览”表格同时看所有接收器。
- **空间解算与实时渲染**（「空间」窗口）：不经过 SteamVR，直接读取接收器的原始 HID 数据，自行解码 LH2 光数据与 IMU、接收基站 OOTX 校准、解算设备与基站位姿，并用 3D 视图实时渲染。可查看每个传感器在每个基站下的扫描角度 / 重投影残差 / 命中、基站校准参数与频率、解算残差 / 协方差 / 迭代、IMU 与光学姿态的重力一致性等。支持录制 `.lhrec` 与回放。详见 [docs/TRACKING.md](docs/TRACKING.md)。
- **USB 拓扑**：从 PCI 控制器 → Hub → 接收器的完整链路、端口号、位置路径、驱动、HID 接口、同 Hub 设备，并据此自动判断“头显内置 / 外置”（可手动修正）。
- **精确配对控制**：配对向导（选接收器 → 设备类型与检查 → `serial <接收器>` + `pair` → 等待设备 → 校验），震动识别、关机、解除单个接收器的配对、恢复出厂（专家模式）。
- **安全措施**：写操作前自动备份状态；会话必须确认已连接到目标接收器才会发命令；解配 / 重置需要输入序列号确认；控制台标签页禁止直接输入配对 / 解配 / 重置 / 刷写类命令；检测并可结束 SteamVR / Pimax 等占用接收器的进程。
- **其他**：别名与备注、配对历史、完整控制台日志（过滤 / 复制 / 导出）、深色 / 浅色 / 跟随系统主题、Windows 11 半透明背景、高分屏清晰显示、可拖拽停靠的布局。

## 下载

每次推送到 `main` 分支，GitHub Actions 都会从零构建单文件 exe 并发布一个 Release，版本号为该提交的短哈希
（例如 `a1b2c3d`），见本仓库的 Releases 页面。exe 的「关于」对话框中也会显示这个短哈希。

## 构建

需要 Visual Studio 2022/2026（含 C++ 桌面开发与 CMake/Ninja）。imgui / implot / freetype 是 git 子模块（固定到具体提交），
nlohmann/json 与 doctest 为单头文件直接放在 `third_party/`。

```powershell
git clone --recurse-submodules --shallow-submodules <repo-url>
# 已经 clone 过：git submodule update --init --depth 1（build.ps1 发现缺失时也会自动执行）
.\build.ps1            # Release 构建 -> build\Release\LighthousePairing.exe
.\build.ps1 -Test      # 构建并运行单元测试
.\build.ps1 -Clean     # 清理后重新构建
.\build.ps1 -Dist      # 从零构建单个独立 exe -> dist\LighthousePairing.exe（静态 CRT，只依赖系统 DLL）
```

> 中文系统上的 MSVC 用控制台代码页输出本地化的 `/showIncludes` 前缀，CMake 写进 `rules.ninja` 的前缀编码不对，
> 导致 Ninja 追踪不到头文件依赖（改头文件不重编，链接进旧对象）。构建脚本会固定控制台为 UTF-8 并用 cl 的原始输出修正该前缀，
> 所以请使用 `build.ps1` / `tools\b.ps1` 构建，而不是直接调用 `cmake --build`。

## 运行

```powershell
build\Release\LighthousePairing.exe            # 真实硬件
build\Release\LighthousePairing.exe --demo     # 演示模式（模拟数据，不访问任何接收器）
```

依赖 SteamVR 自带的 `lighthouse_console.exe`（自动从 `%LOCALAPPDATA%\openvr\openvrpaths.vrpath` 定位，也可在设置中指定）。

命令行参数（主要用于调试 / 截图）：

| 参数 | 作用 |
|---|---|
| `--demo` | 演示模式 |
| `--no-mica` | 关闭 Mica / DirectComposition |
| `--fresh-layout` | 忽略保存的停靠布局 |
| `--size 1500x940` | 初始窗口大小（逻辑像素） |
| `--select SERIAL` / `--tab N` | 预选接收器 / 详情标签页（0 概览 1 实时信号 2 设备信息 3 USB 4 历史 5 控制台） |
| `--theme 0/1/2` | 深色 / 浅色 / 跟随系统 |
| `--open wizard\|settings\|danger\|kill\|about` | 启动后打开对应对话框 |
| `--screenshot out.bmp --frames N` | 渲染 N 帧后保存后台缓冲区并退出 |
| `--dump-usb out.txt` | 输出 USB 拓扑报告后退出 |
| `--spatial` | 启动后切到「空间」窗口 |
| `--replay file.lhrec` | 空间追踪使用录制文件回放（不读取实时 HID） |
| `--tracking-active` | 空间追踪以主动模式启动（SteamVR 未运行时可把设备切到 raw1） |
| `--inspector N` | 空间检查器标签页（0 位姿 1 传感器 2 基站 3 数据流 4 曲线 5 日志） |

数据目录：`%LOCALAPPDATA%\LighthousePairing`（`history.json` 历史与设置、`backups\` 写操作前的备份、`imgui.ini` 布局、`device_configs\` 读取到的设备配置、`base_stations.json` 基站校准缓存、`recordings\` 原始 HID 录制）。

## 隐私与数据保护

**请勿录制他人未同意的追踪数据，也不要把录制文件发送给未经授权的第三方。**

录制文件（`.lhrec`）保存设备的原始 HID 数据：IMU（加速度、角速度）、基站扫描时刻、按键与手指传感器读数（如有），
以及设备和基站的序列号。由这些数据可以还原出佩戴者在录制期间头部、双手和身体的完整运动轨迹。

身体动作数据是需要特别保护的个人信息：

- 步态、手势、动作习惯等运动特征可用于识别特定个人，也可能被用来推断健康状况、身体残障、年龄或情绪状态。
- 中国《个人信息保护法》：生物识别、医疗健康、行踪轨迹等信息属于敏感个人信息（第二十八条），处理须取得个人的单独同意（第二十九条）；
  向其他个人信息处理者提供须告知接收方情况并取得单独同意（第二十三条）；向境外提供还须满足第三十八、三十九条的要求。
- 欧盟 GDPR：用于识别特定自然人的生物识别数据及健康数据属于特殊类别数据（第九条），原则上禁止处理，除非取得明确同意等例外情形。
  其他地区（如美国伊利诺伊州 BIPA）也有关于生物识别信息的法规。

请遵守：

1. 只录制你自己的设备和动作。录制他人前，先告知录制内容、用途和保存期限，并取得对方明确同意。
2. 不要把录制文件发送、上传或分享给未经授权的第三方，包括公开的 issue、论坛、网盘和聊天群。确需提供给开发者排查问题时，
   只提供必要的片段，并通过可信渠道直接交给获得授权的人。**请不要把 `.lhrec` 附加到本仓库的公开 issue 中。**
3. 录制文件只保存在本机，本程序不会上传任何数据。用完请及时删除，不要长期保存不再需要的录制。

程序在每次开始录制前都会显示这份提示（`lhp_hidtool capture` 会在控制台输出简短提示）。文案定义在
[src/tracking/io/PrivacyNotice.h](src/tracking/io/PrivacyNotice.h)。本提示不构成法律意见，具体义务以你所在地的法律为准。

## 结构

```
src/
  backend/  ConsoleSession (lighthouse_console 子进程 + 管道 + Job Object)
            ConsoleParser  (dongleinfo / imustats / usbstats / battery / 事件行解析)
            UsbTopology    (SetupAPI + CfgMgr32 枚举、父链、热插拔通知)
            SystemInfo     (SteamVR 定位、冲突进程、文件工具)
  core/     Model / Store (单一状态源) / HistoryDb (JSON 持久化与备份)
            RealService (刷新、每个接收器一个监控会话、配对/解配/重置操作状态机)
            DemoService (模拟数据)
  tracking/ 空间追踪子系统（不依赖 core / UI，分层见 docs/TRACKING.md）
            hid (HID 传输) · io (数据源 / 录制回放) · protocol (报告解码、命令) · lighthouse (LH2 角度、OOTX、基站模型)
            model (设备几何) · solver (LM 位姿解算、场景 / 基站自举) · service (管线、模式策略、快照) · sim (模拟器)
  platform/ Window (Win32 + D3D11 + DirectComposition/Mica + 截图) / Renderer3D (离屏 3D)
  ui/       App / Views / Dialogs / Widgets / Theme / Icons / SpatialView
tests/      doctest 单元测试 + 真机录制的 console 输出
tools/      b.ps1（开发构建）、shot.ps1（截图并缩小）、probe_console.py（只读探测）、make_icon.py
            hidtool/ lhp_hidtool：list / capture / dump / decode / angles / solve / live / replay / setmode
```

## 已知限制

- `lighthouse_console` 的 `pair` / `unpair` / `donglereset` 等是未写进 help 的命令，输出格式来自 exe 内字符串与 Pimax 运行时日志，尚未在真机上执行过写操作；首次使用请先在空闲接收器上验证。
- 离线设备的配对关系 `dongleinfo` 不一定能给出（实测第三个字段为空），因此用“设备读取 / 本次观察 / 历史记录”三级来源区分可信度。
- 设备在线时的 `deviceinfo` / `version` / `battery` 输出格式按通用 key:value 解析，原始输出在“设备信息”页完整保留。
- 空间追踪目前只支持接收器（28DE:2101 / 2102）后面的无线设备与 Lighthouse 2.0 基站；USB 直连的头显追踪模块（报告格式不同）与 1.0 基站尚未支持。IMU 只用作角速度先验和重力一致性检查，还没有做完整的 IMU / 光学融合滤波。

## 许可证

Copyright (C) 2026 LianZhou

本程序是自由软件：你可以依据自由软件基金会发布的 GNU 通用公共许可证第 3 版或（由你选择）任何更新的版本
再分发和/或修改它。本程序不提供任何担保。完整条款见 [LICENSE](LICENSE)。

第三方组件（Dear ImGui、ImPlot、nlohmann/json、doctest 为 MIT，FreeType 为 FTL）的版权与许可见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本项目与 Valve、HTC、Pimax 等厂商无关。
