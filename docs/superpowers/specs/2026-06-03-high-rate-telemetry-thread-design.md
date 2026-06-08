# 高频遥测独立发送线程（High-Rate Telemetry Thread）设计

- 日期：2026-06-03
- 适用固件：本仓库 ArduPlane 4.6.3 交付分支（`206固定翼交付程序4.6.3`）
- 目标载具：固定翼（ArduPlane），可推广到其它载具

## 1. 背景与问题

当前 `SYSTEM_TIME`、`GLOBAL_POSITION_INT`、`GPS_RAW_INT` 三条消息通过自定义的
`STREAM_HIGH_RATE`（`SRx_HIGH_RATE` 参数，默认 100Hz）发送，走的是标准的
**桶（bucket）+ `update_send()` 仲裁**路径：

- `GCS_MAVLINK::update_send()`（`libraries/GCS_MAVLink/GCS_Common.cpp:1508`）每个主循环
  在一个 **最多 5ms 的预算循环**里，按优先级轮流发送：特殊消息 `deferred_message[3]`
  （HEARTBEAT/NEXT_PARAM）→ `pushed_ap_message_ids` → 流消息桶 `deferred_message_bucket[10]`
  （`GCS.h:863`）。
- 三条高频消息按相同间隔被分到同一个桶（`set_ap_message_interval`，`GCS_Common.cpp:1667`）。
  即便该桶"干净"（仅这 3 条），它仍与其它所有桶/特殊消息/pushed 消息**共享同一个
  `update_send` 仲裁与 5ms 预算**，且只有一个桶能同时处于发送态（`sending_bucket_id`）。
- 叠加 `out_of_time()` 截断、TX 缓冲背压（`no_space_for_message`）与
  `stream_slowdown_ms` 全局降速，导致在 200Hz 目标下**节拍不稳、抖动大**。
- 另有硬上限：任何消息速率 ≤ `0.8 × SCHED_LOOP_RATE`（`cap_message_interval`，
  `GCS_Common.cpp:232`），即发送节拍受主循环频率约束。

**结论**：要稳定 200Hz，必须把这三条消息拆到一条独立、固定节拍、绕开 `update_send`
仲裁、且不受主循环频率限制的发送路径。

## 2. 关键可行性依据（线程安全）

MAVLink 生成的 `mavlink_msg_*_send(chan, ...)` 内部以
`MAVLINK_START_UART_SEND`→`comm_send_lock(chan,size)`（每通道一把信号量
`chan_locks[chan]`）和 `MAVLINK_END_UART_SEND`→`comm_send_unlock(chan)` 把**整帧**括起来
（`libraries/GCS_MAVLink/GCS_MAVLink.cpp:165,178`）。txspace 不足时设
`chan_discard[chan]=true`，整帧被丢弃而非半截写入。

因此：**多个线程向同一通道发送，帧不会字节交错、不会产生乱码帧**。这使得"独立线程在
共享通道上发送"成为可行且较低风险的方案。

残留并发点（均为可接受级别，实现时确认/记录）：
- 序列号 `current_tx_seq` 自增发生在锁外，多线程极小概率竞争 → 仅影响地面站丢包统计，
  不影响帧完整性。
- 从独立线程读取 AHRS/GPS 状态可能偶发"撕裂读" → 对遥测可接受。

## 3. 需求

### 功能需求
- FR1：以独立于主循环的固定节拍发送 `SYSTEM_TIME` / `GLOBAL_POSITION_INT` / `GPS_RAW_INT`。
- FR2：发送速率由每通道参数 `SRx_HIGH_RATE`（Hz）决定，可达 200Hz。
- FR3：**通道选择规则 = `SRx_HIGH_RATE > 0` 即在该通道发送**（USB/数传/独立口，谁开谁发，
  天然支持多路）。
- FR4：这三条消息**不再经由桶/`update_send` 发送**（去重，避免双发）。

### 非功能需求
- NFR1：节拍不受 `SCHED_LOOP_RATE` 影响（独立线程自计时）。
- NFR2：不破坏帧完整性（复用现有 `chan_lock`，绝不绕过 `comm_send_lock`）。
- NFR3：低带宽链路（如 57600 数传）设高频时，只表现为**整帧丢弃 / 实发不满**，
  不得阻塞主循环、不得乱码、不得影响其它消息（心跳/参数等）。
- NFR4：不改变其它流消息（POSITION/EXTRA1/2/3/RAW_SENSORS/PARAMS…）的行为。

## 4. 设计

### 4.1 组件
新增一个专用线程，逻辑上归属 GCS：

- 通过 `hal.scheduler->thread_create(...)` 在 GCS 初始化后创建一次。
- 选专用线程而非 timer 线程（1kHz，时间关键，抢锁阻塞会拖垮 1kHz）或 IO 线程
  （要干存储等杂活，抖动大）。专用线程节拍最稳。
- 线程优先级：低于主循环与 timer，给一个适中优先级（实现时按 ChibiOS 现有遥测/IO 优先级
  选取并记录）。

### 4.2 节拍与速率
- 线程基准节拍 5ms（200Hz），用 `micros64()` 自校准休眠（计算到下一个目标时刻再 sleep），
  避免累计漂移。
- 为每个通道维护 `last_sent_us`，按各自 `SRx_HIGH_RATE` 计算间隔（`1e6 / rate` 微秒）调度；
  到点才发，未到点跳过。

### 4.3 多通道
- 线程遍历所有 GCS 通道 `gcs().chan(i)`；对每个 `link != nullptr` 且其
  `SRx_HIGH_RATE > 0` 的通道，按 4.2 的节拍发送三条消息。
- 带宽自负：`comm_send_lock` 在 txspace 不足时整帧丢弃（见第 2 节），符合 NFR3。

### 4.4 去重
- 从 `ArduPlane/GCS_Mavlink.cpp` 的 `STREAM_HIGH_RATE_msgs[]` 中移除这三条
  （`MSG_SYSTEM_TIME` / `MSG_LOCATION` / `MSG_GPS_RAW`）。
- 保留 `SRx_HIGH_RATE` 参数定义（`streamRates[10]`，`GCS_Mavlink.cpp:617`），但其语义改为
  "**高频独立线程的发送速率**"。更新该参数的 `@Description`。
- 结果：桶系统不再持有这三条；独立线程成为它们唯一的发送路径。

### 4.5 数据流与发送实现
- 复用现有发送逻辑：`GCS_MAVLINK::send_system_time()`、
  `GCS_MAVLINK::send_global_position_int()`、`GCS_MAVLINK::send_gps_raw()`
  （后者内部 `AP_GPS::send_mavlink_gps_raw()`）。它们读取 AHRS/GPS 最新值并调用
  `mavlink_msg_*_send`（自动加 `chan_lock`）。
- 实现时**必须逐个核实这三条发送函数可重入**：只读取缓存状态、无主循环专用可变缓冲、
  无非线程安全的静态局部。若某条不满足，则在独立线程内改用等价的、读快照后直接
  `mavlink_msg_*_send` 的实现。

### 4.6 受影响文件（预估）
- `ArduPlane/GCS_Mavlink.cpp`：移除三条出 `STREAM_HIGH_RATE_msgs[]`；更新 `HIGH_RATE`
  参数描述。
- `libraries/GCS_MAVLink/GCS.h` / `GCS.cpp`：新增高频线程成员、创建入口、每通道
  `last_sent_us` 状态、发送函数。
- 线程创建挂载点：GCS 初始化路径（实现时确定，例如 `GCS::init` 之后或首次 `update_send`
  前的一次性初始化）。

## 5. 参数

| 参数 | 变化 | 说明 |
|---|---|---|
| `SRx_HIGH_RATE` | 语义变更（不新增） | 由"桶流速率"改为"高频独立线程速率"，Hz，范围 0–200，0=关闭该通道高频发送 |

不新增其它参数。

## 6. 测试计划

- T1（节拍稳定性，SITL）：起飞，`SR0_HIGH_RATE=200`，用 pymavlink 统计三条消息的到达
  频率与到达间隔抖动（std / 最大间隔），对比改造前桶方式应显著改善。
- T2（多通道）：同时开 `SR0_HIGH_RATE` 与 `SR1_HIGH_RATE`，验证两路独立达到各自设定。
- T3（去重）：抓包确认三条消息不再由桶路径重复发出（无双份）。
- T4（低带宽降级）：把某通道（模拟低 txspace）设高频，验证只丢帧、不阻塞主循环、不乱码、
  心跳/参数等其它消息仍正常。
- T5（关闭）：`SRx_HIGH_RATE=0` 时该通道不发这三条。

## 7. 风险与缓解

| 风险 | 缓解 |
|---|---|
| 发送函数从他线程调用不可重入 | 实现前逐个核实；不满足则改用快照后直发 |
| 序列号竞争 | 仅影响丢包统计，可接受；如需可后续将 seq 自增纳入锁 |
| AHRS/GPS 撕裂读 | 遥测可接受；如需可读一致快照 |
| 线程栈溢出 | 分配足够栈并在 SITL/实测观察高水位 |
| CPU 占用（200Hz×N通道×3条） | 评估实测；低带宽口靠丢帧自限 |

## 8. 范围之外（YAGNI）

- 不改其它流消息的发送机制。
- 不引入新的位掩码参数（通道选择直接用 `SRx_HIGH_RATE>0`）。
- 不做跨载具统一（先在 ArduPlane 落地；结构上不阻碍后续推广）。
- 不解决 GPS 实际数据率仅 5–10Hz 的问题（高频发送的是最新值/重复值 + 插值时间戳，
  与既有设计一致）。
