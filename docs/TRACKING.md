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
 L3  tracking/filter/     TrackerFilter（每设备的误差状态迭代卡尔曼：光学 + IMU）、Imu（换算、静止检测）
      tracking/solver/     PoseSolver（单帧 LM：启动 / 校验 / 推算基站）、Scene（空间：基站位姿、水平、记忆）、UniverseIo
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
- Knuckles 的 `channelMap` 是“模型序号 → 线上传感器 ID”的置换（tracker 恰好是恒等），方向读反会让手柄解算完全错乱。
- 滤波器实测（2 台基站，tracker + Knuckles 静止在地上）：位置 σ ≈ 0.2 mm、姿态 σ ≈ 0.25°，光学更新约 230 次/秒，
  光学残差 0.003–0.005°，单次光学更新约 55 µs。tracker 估出的陀螺仪残余零偏约 0.45°/s、加速度计零偏约 0.3 m/s²
  （IMU 与光学模型之间约 1.7° 的安装偏差由零偏状态吸收；Knuckles 只有 0.2°）。
- lighthouse_console 连接时会把设备切回 raw0；主动模式切回 raw1 后解码器重建、设备时间重新展开，管线检测到后整体重新开始
  （一次重置，随后立即恢复跟踪）。

## 解算（L3）：按 SteamVR 的结构

SteamVR 的追踪代码不公开，但它的 lighthouse 驱动二进制里的类名和日志字符串（见
[SteamTracking/GameTracking-SteamVR](https://github.com/SteamTracking/GameTracking-SteamVR) 的 `driver_lighthouse_strings.txt`）
给出了结构：`CLighthousePoseSolver`（Ceres，残差 `CSensorPositionAngular` + `CQuaternionNormalizer`，用于启动）、
`CLighthouseDirect`（每设备的跟踪器，残差 `CSensorPositionAngular`、`CImuAccel`、`CImuGyro`、`CSensorNormalCosAngle`）、
`CImuOnlyTracker`（"Switching Lighthouse -> IMU Only"）、universe / tilt / "SECONDARY base ... distance" / lighthousedb。这里一一对应：

| SteamVR | 这里 |
|---|---|
| CLighthousePoseSolver | `solver::SolvePose`：单帧静态 LM，24 个朝向种子；启动滤波器、10 Hz 校验滤波器、推算新基站 |
| CLighthouseDirect | `filter::TrackerFilter`：误差状态迭代卡尔曼（每次光学更新 = 带先验的 Gauss-Newton，相当于一步 Ceres） |
| CSensorPositionAngular | 每个（传感器, 平面）一个角度；按各自命中时刻用 (v, ω, a) 外推，整圈扫描没有运动模糊 |
| CSensorNormalCosAngle | 传感器法线背向基站的观测剔除 |
| CImuAccel / CImuGyro | `UpdateImu`：gyro = C·ω + bg，acc = C·(a + g) + ba，每个 IMU 样本更新一次 |
| CImuOnlyTracker | 状态 `仅 IMU`：0.25 s 没有光学更新时姿态靠陀螺仪、零速约束保持位置；3 s 后为 `丢失` |
| universe / tilt / lighthousedb | `solver::Scene` + `UniverseIo`：基站按 ID 记忆，读取 SteamVR 的 lighthousedb，用静止设备的加速度计修正水平 |

**TrackerFilter**（`tracking/filter/`，状态布局与噪声默认值参考 libsurvive `survive_kalman_tracker`）
- 名义状态：位置、姿态（四元数）、速度、角速度（世界系）、线加速度（不含重力）、陀螺仪零偏、加速度计零偏；误差状态 21 维。
- 过程模型：匀加速 + 匀角速度；白噪声 jerk 驱动 (p, v, a)，白噪声角加速度驱动 (θ, ω)，零偏随机游走。
- 光学更新：每个基站的角度攒到 6 个或 12 ms 就融合一次；按新息协方差做 5σ 门限（刚启动 0.6 s 内放宽到 12σ），
  再做最多 4 次迭代（Huber 加权），后验协方差 = 信息矩阵求逆。
- 零速更新（libsurvive "zvu"）：IMU 判定静止时约束 v = a = ω = 0；光学中断 > 0.1 s 时约束 v = a = 0。
- 健康检查：协方差 / 状态异常、连续 10 批光学被门限拒绝、单帧解算连续 5 次与滤波器相差 > 15 cm / 15°，都会重置并重新启动。
- 世界系变化（重力对齐、地面移动、水平修正）时 `TransformWorld` 把状态和协方差一起搬过去，不重置；
  基站校准变化（理想模型 → OOTX）时 `Inflate` 放松位姿协方差，而不是把正确的新数据当离群值拒掉；
  设备时间基准变化（解码器重建、时钟跳变）时整体重新开始。

**Scene（空间）**
- 第一个看到的基站为参考；其它基站在设备**静止**、两边都有 OOTX 校准时，由“同一设备对参考系的位姿”和“对该基站单独的解”
  推出，滑动平均（> 5 cm 离群剔除；连续 40 个离群视为基站被移动，重新推算）；至少 10 个样本后才参与跟踪。
- 水平：参考基站加速度计（8 位，权重相当于 30 个样本）+ 静止且跟踪良好的设备每秒 1 个重力样本。样本在参考基站坐标系里平均，
  与当前水平无关，不会自我强化。可在界面关闭。
- 记忆：学到的布局按基站 ID 每 10 秒存进 `DataDir\base_stations.json` 的 `universe`，下次看到同样的基站立即恢复。
- SteamVR：读取 `<Steam>\config\lighthouse\lighthousedb.json` 的 `known_universes`（`target_pose.pose = [qx qy qz qw px py pz]`）。
  这个位姿是“原点←基站”还是反过来没有文档（libsurvive 按后者读），所以两种读法都保留，先不参与跟踪，
  用实时推算的样本投票决定（5 票）；都不符合就改为自行推算。
- **基站校准缓存**：同一个文件的 `channels`（按频道保存 OOTX 原始载荷），启动时先用缓存，OOTX 到达后替换。

## 工具：`lhp_hidtool`

| 命令 | 作用 |
|---|---|
| `list` | 列出所有 Valve HID 接口（VID/PID、序列号、报告长度） |
| `capture <serial\|all> <秒> <out.lhrec>` | 只读录制原始报告 |
| `dump <file> [n]` / `decode <file> [n]` | 报告统计 / 解码统计、OOTX、角度范围 |
| `angles <file> <sensor>` | 单个传感器的角度时间序列 |
| `solve <file> <config.json> [每 N 个 sync]` | 离线解算 |
| `live <秒> <config.json\|目录> [active]` / `replay <秒> <config> <file\|sim\|simfast>` | 无界面运行完整 TrackingService（目录 = 按接收器序列号命名的配置；sim = 模拟场景并与真值比较） |
| `setmode <serial> raw0\|raw1` | 唯一会写设备的命令；SteamVR 运行时拒绝 |

## 测试

`tests/TrackingTests.cpp`：四元数 / 位姿、理想重投影与方向反算互逆、OOTX 编解码往返（含丢位）、设备模型解析、
录制往返、channelMap 方向，以及：
- **端到端单帧**：模拟器生成的 HID 报告 → 解码 → LH2 角度 → OOTX → 解算，与真值比较（< 3 mm / 0.5°）。
- **滤波器**：静止收敛并学到陀螺仪零偏；~1.3 m/s、~3 rad/s 运动下平均误差 < 2 mm / 0.2°；世界系变换。
- **服务端到端**：两台模拟设备 + 两台基站走完整 TrackingService（第二台基站自举、水平修正），与真值比较
  （< 5 mm / 0.5°，不重置）。模拟器的 IMU 含线加速度、噪声和零偏；光数据按命中时刻的位姿生成。
- **空间**：SteamVR lighthousedb 解析 + 两种读法由样本投票；自有格式往返 + 按基站 ID 在不同频道上恢复。

## 尚未支持

- USB 直连设备（头显追踪模块 28DE:2000/2300 等）：报告格式不同（0x20 / 0x25 / 0x27 / 0x28），尚未接入。
- Lighthouse 1.0 基站（raw0 脉冲 + sync 闪光）的角度计算。
- 每设备 IMU 安装偏差（SteamVR 的 "IMU misalignment" / trackref_from_imu 修正）尚未单独估计，目前由加速度计零偏吸收。
- 基站位姿没有像 CLighthouseDirect 那样放进每个设备的滤波状态里联合估计，而是由 Scene 从静止设备样本平均得到。
- SteamVR 房间设置（chaperone 的 standing 平移 / 偏航）尚未套用；lighthousedb 的位姿方向靠实测投票确定（本机没有该数据，未实测）。
- 设备配置只从 lighthouse_console 的 `downloadconfig` 或 SteamVR 缓存获得，尚未直接通过 HID 读取。

## 参考

- 协议格式、gen2 重投影、卡尔曼状态布局与噪声默认值参考 [libsurvive](https://github.com/collabora/libsurvive)（MIT），
  移植的代码在文件头注明出处，许可与版权声明见 `THIRD_PARTY_NOTICES.md`。
- 扫描角测量模型与逐角度更新的思路参考 Bitcraze [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)
  的 `mm_sweep_angles.c`（GPL-3.0，未复制代码）。
- SteamVR 的结构来自其驱动的公开字符串表（SteamTracking/GameTracking-SteamVR）；lighthousedb 格式参考 libsurvive 与公开样例。
