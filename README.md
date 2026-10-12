# Lighthouse 配对管理 (LighthousePairing)

一个 C++20 + Dear ImGui (v1.92.9b docking) 的 Windows 桌面工具，用来：

- **列出所有 Lighthouse 接收器**：头显内置的 Watchman、外置 Valve VR Radio (28DE:2102)、老款 Watchman 接收器 (28DE:2101)，按类型分组显示。
- **查看配对关系与设备详情**：每个接收器当前连接 / 历史配对的设备（LHR-xxxxxxxx）、型号、固件、硬件版本、电量，以及 `deviceinfo` / `version` 的全部原始字段。
- **实时信号**：IMU 采样率、包间隔与抖动 σ、USB 包率与最大间隔、无线丢包（keepalive "Packet received after Xs"）、断开、追踪重置；1/5/15 分钟曲线 + 事件表 + 信号质量评分；“信号总览”表格同时看所有接收器。
- **USB 拓扑**：从 PCI 控制器 → Hub → 接收器的完整链路、端口号、位置路径、驱动、HID 接口、同 Hub 设备，并据此自动判断“头显内置 / 外置”（可手动修正）。
- **精确配对控制**：配对向导（选接收器 → 设备类型与检查 → `serial <接收器>` + `pair` → 等待设备 → 校验），震动识别、关机、解除单个接收器的配对、恢复出厂（专家模式）。
- **安全措施**：写操作前自动备份状态；会话必须确认已连接到目标接收器才会发命令；解配 / 重置需要输入序列号确认；控制台标签页禁止直接输入配对 / 解配 / 重置 / 刷写类命令；检测并可结束 SteamVR / Pimax 等占用接收器的进程。
- **其他**：别名与备注、配对历史、完整控制台日志（过滤 / 复制 / 导出）、深色 / 浅色 / 跟随系统主题、Windows 11 Mica 半透明背景、DPI 感知、动画与 toast 通知、可拖拽停靠的布局。

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

数据目录：`%LOCALAPPDATA%\LighthousePairing`（`history.json` 历史与设置、`backups\` 写操作前的备份、`imgui.ini` 布局）。

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
  platform/ Window (Win32 + D3D11 + DirectComposition/Mica + 截图)
  ui/       App / Views / Dialogs / Widgets / Theme / Icons
tests/      doctest 单元测试 + 真机录制的 console 输出
tools/      b.ps1（开发构建）、shot.ps1（截图并缩小）、probe_console.py（只读探测）、make_icon.py
```

## 已知限制

- `lighthouse_console` 的 `pair` / `unpair` / `donglereset` 等是未写进 help 的命令，输出格式来自 exe 内字符串与 Pimax 运行时日志，尚未在真机上执行过写操作；首次使用请先在空闲接收器上验证。
- 离线设备的配对关系 `dongleinfo` 不一定能给出（实测第三个字段为空），因此用“设备读取 / 本次观察 / 历史记录”三级来源区分可信度。
- 设备在线时的 `deviceinfo` / `version` / `battery` 输出格式按通用 key:value 解析，原始输出在“设备信息”页完整保留。

## 许可证

Copyright (C) 2026 LianZhou

本程序是自由软件：你可以依据自由软件基金会发布的 GNU 通用公共许可证第 3 版或（由你选择）任何更新的版本
再分发和/或修改它。本程序不提供任何担保。完整条款见 [LICENSE](LICENSE)。

第三方组件（Dear ImGui、ImPlot、nlohmann/json、doctest 为 MIT，FreeType 为 FTL）的版权与许可见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本项目与 Valve、HTC、Pimax 等厂商无关。
