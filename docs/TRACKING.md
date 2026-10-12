# 空间追踪子系统（tracking）架构

目标：不经过 SteamVR，直接读取 Lighthouse 接收器 / 有线设备的原始 HID 数据，自行解码、解算设备与基站的位姿，
供"空间"界面实时渲染，并暴露解算过程的全部中间数据。

## 设计原则

1. **严格分层，单向依赖**：上层只依赖下层的公开类型，下层不知道上层存在。
2. **纯函数优先**：协议解码、光学处理、解算都是不碰 I/O、不碰线程、不碰 UI 的纯逻辑，可以用录制数据做单元测试。
3. **数据源可替换**：实时 HID 与录制文件回放实现同一个接口；演示模式、回归测试、离线分析都走同一条管线。
4. **只读优先**：默认被动监听（不向设备发任何命令）。只有用户显式开启且 SteamVR 未运行时，才发送开启原始数据的配置命令；所有命令集中在一个文件里。
5. **快照发布**：后台线程算好不可变的快照，UI 只读快照，不持有后台对象、不加锁跨帧。

## 分层

```
 L5  ui/spatial/          空间界面：3D 视口、设备/基站列表、深度检查器、曲线
      platform/Renderer3D  D3D11 离屏渲染（网格、基站视锥、设备模型、传感器、轨迹）
        │ 只读 TrackingSnapshot
 L4  tracking/service/    TrackingService：每个 HID 设备一条管线（线程），模式控制，快照发布
        │
 L3  tracking/solver/     PoseSolver（LM 位姿）、SceneBootstrap（基站位姿、重力对齐）、ImuFusion
        │ 角度观测 + 设备模型 + 基站校准
 L2  tracking/lighthouse/ Lh2Processor（sync/sweep → 角度）、Lh1Processor、OotxDecoder、基站校准模型
        │ 类型化事件
 L1  tracking/protocol/   报告解码：Watchman RF(0x23/0x24) v1/v2、raw0/raw1 光数据、IMU、电量、输入；Commands（配置命令）
        │ RawReport（字节 + 主机时间戳）
 L0  tracking/hid/        HidDevice：枚举、共享打开、重叠读、Feature 报告
      tracking/io/         ReportSource 接口；LiveHidSource / FileSource；.lhrec 录制读写
      tracking/math/       Vec3 / Quat / Pose / 小矩阵与 LM（L2、L3 共用）
```

依赖方向：`L5 → L4 → L3 → L2 → L1 → L0`，`math` 被 L2/L3 共用。`tracking` 整体不依赖 `core/Store` 和 UI；
`core` 里的适配层把快照接入 AppState。

## 数据流与时间

- **主机时间**：`RawReport::hostUs`，读到报告时的 QPC 微秒，只用于统计到达间隔和延迟。
- **设备时间**：48 MHz 的 watchman 时钟（光数据 24 位、IMU 由 16 位 + 8 位拼成）。解码层把它展开成单调的 64 位
  `deviceTicks`，解算一律用设备时间，与传输延迟无关。
- **LH2 角度**：每个基站（频道）以固定频率旋转（50.05–54.12 Hz）。`sync` 给出一圈的起点，`sweep` 给出传感器被
  扫到的时刻，`angle = (t_sweep - t_sync) / period × 2π`，再按 2π/3、4π/3 分到两个斜面。
- **OOTX**：sync 里每圈带 1 bit，拼成基站校准包（相位、倾斜、曲率、gibbous、ogee、加速度计方向、基站 ID）。

## 线程模型

- 每个打开的 HID 设备一个读取线程：`ReportSource::Read` → L1 解码 → L2 处理 → L3 解算 → 写入该设备的状态。
- 基站是全局共享的（多个设备看到同一个基站），由 `TrackingService` 持有，设备管线只提交观测，基站估计在服务线程更新。
- 每 ~16 ms 生成一次 `TrackingSnapshot`（`std::shared_ptr<const ...>` 原子替换），UI 取最新指针即可。

## 模式

| 模式 | 条件 | 行为 |
|---|---|---|
| 被动 | 默认；SteamVR 运行时强制 | 只读打开 HID，不发任何报告。数据格式取决于当前主控（SteamVR / lighthouse_console）设置的模式 |
| 主动 | 用户开启，且检测到 SteamVR 未运行 | 发送 `Commands.h` 里的配置序列，切到 raw1 光数据模式 |
| 回放 | 打开 `.lhrec` 文件 | 用录制数据跑同一条管线 |

## 录制格式 `.lhrec`

```
header:  "LHREC\0" u16 version, u16 reserved
record:  u8 type (1=report, 2=device-info, 3=marker), u64 hostUs, u16 sourceId, u16 length, bytes[length]
```

device-info 记录保存 VID/PID/序列号/路径，回放时能还原设备列表。

## 真机验证结论（本机：VIVE Tracker 3.0 @ Watchman 接收器，1 台 LH2 基站，频道 4）

- 接收器 HID：`28DE:2101`，单接口，usage page `FF00`，输入报告 59 字节；无线数据走报告 `0x23`（1 包）/ `0x24`（2 包）。
- 没有任何程序发命令时也会持续上报：IMU ~245 Hz、光数据、电量。
- 光数据编码由主控决定：lighthouse_console 连接时会把设备切回 **raw0**（只有脉冲，LH2 无法定基站和角度）；
  发送 `ff 87 06 01 01 00 02 00 00`（Feature 报告）后切到 **raw1**：每圈一个 sync（含 1 bit OOTX）+ 每个传感器两个平面的 sweep。
- 静止 tracker 的扫描角噪声约 0.003°；收到 OOTX 校准后重投影残差 RMS 从 ~0.02° 降到 ~0.003°，位置 σ ≈ 3 mm。
- 用基站加速度计（OOTX `accel_dir`）定义的“上”与 tracker 自己的 IMU 重力方向相差约 2.6°，说明世界系对齐正确。

## 解算（L3）

- **观测**：每个（基站, 传感器, 平面）只保留最新角度，30 ms 内的参与解算；每次 sync 后最多每 12 ms 解一次。
- **PoseSolver**：Levenberg–Marquardt，数值雅可比，Huber 鲁棒权重，收敛后剔除 > 0.02 rad 的离群观测再解一次。
  多个基站或设备在运动时，额外解线速度 / 角速度（12 维），把观测窗口内的运动模糊消掉；速度带弱先验
  （线速度先验 = 位置有限差分，角速度先验 = 陀螺仪），不合理（> 5 m/s 或 > 20 rad/s）时退回静态解。
- **初值**：没有上一帧位姿时，由两平面角反推每个传感器的方向（理想模型），估计距离，再试 24 个朝向。
- **输出**：位姿、残差（逐观测）、RMS / 最大残差、协方差 (JᵀJ)⁻¹σ²、迭代次数、耗时。
- **Scene**：第一个看到的基站为参考；其它基站在设备**静止**且参考与该基站都已有 OOTX 校准时，
  由“同一设备分别对参考系和对该基站的解”推出，滑动平均（偏差 > 5 cm 的样本剔除）；校准变化时重新自举。
- **基站校准缓存**：`DataDir\base_stations.json`（按频道保存 OOTX 原始载荷），启动时先用缓存，OOTX 到达后替换。

## 工具：`lhp_hidtool`

| 命令 | 作用 |
|---|---|
| `list` | 列出所有 Valve HID 接口（VID/PID、序列号、报告长度） |
| `capture <serial\|all> <秒> <out.lhrec>` | 只读录制原始报告 |
| `dump <file> [n]` / `decode <file> [n]` | 报告统计 / 解码统计、OOTX、角度范围 |
| `angles <file> <sensor>` | 单个传感器的角度时间序列 |
| `solve <file> <config.json> [每 N 个 sync]` | 离线解算 |
| `live <秒> <config.json> [active]` / `replay <秒> <config.json> <file>` | 无界面运行完整 TrackingService |
| `setmode <serial> raw0\|raw1` | 唯一会写设备的命令；SteamVR 运行时拒绝 |

## 测试

`tests/TrackingTests.cpp`：四元数 / 位姿、理想重投影与方向反算互逆、OOTX 编解码往返（含丢位）、设备模型解析、
录制往返，以及**端到端**：模拟器生成的 HID 报告 → 解码 → LH2 角度 → OOTX → 解算，与真值比较（< 3 mm / 0.5°）。

## 尚未支持

- USB 直连设备（头显追踪模块 28DE:2000/2300 等）：报告格式不同（0x20 / 0x25 / 0x27 / 0x28），尚未接入。
- Lighthouse 1.0 基站（raw0 脉冲 + sync 闪光）的角度计算。
- 完整的 IMU / 光学融合（卡尔曼滤波）；目前 IMU 只用于角速度先验与重力一致性检查。
- 设备配置只从 lighthouse_console 的 `downloadconfig` 或 SteamVR 缓存获得，尚未直接通过 HID 读取。

## 参考

协议格式参考 [libsurvive](https://github.com/collabora/libsurvive)（MIT），移植的代码在文件头注明出处，
许可与版权声明见 `THIRD_PARTY_NOTICES.md`。
