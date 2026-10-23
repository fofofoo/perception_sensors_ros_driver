# uwb_rtu_serial_demo

D-PDOA-Node-B 基站经 **TTL UART（USB-TTL，默认 `/dev/ttyUWB`）** 输出
**Modbus-RTU** 定位帧，本包把定位帧解码后发布为 ROS 话题。
本包替代已移除的旧 `serial_demo`，不再维护旧代码；接入方式、接线和协议细节见
[仓库 README](../../../README.md)。

## 话题

节点有两种运行模式（launch 参数 `uwb_self_collect`）：默认**旁路订阅**，不打开串口，
订阅旧 `serial_demo` 节点发布的 `/uwb_position` 并原样转发到
`/perception_sensors/uwb_position`；`uwb_self_collect:=true` 时才主动采集串口。
两种模式下对外话题相同，订阅方不需要区分。

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/perception_sensors/uwb_position` | `uwb_rtu_serial_demo/uwb_data` | 每收到一帧有效定位帧发布一次；频率跟随标签发包（动态默认约 10 Hz，静态可低至 0.05 Hz）；队列 10 |
| `/perception_sensors/uwb_status` | `std_msgs/Bool` | 链路状态：`true` 在线 / `false` 串口未打开、读写异常或超过 15 s 无有效帧；跳变时发布，latched |

**状态话题** `/perception_sensors/uwb_status`：判定用 `RtuClient::isAlive(15 s)`，已包含
「串口未打开」「读写异常」「超时无有效帧」三种情况。阈值取无帧告警值
（launch 参数 `uwb_heartbeat_timeout_ms`，默认 15 s），而不是失联重连阈值
`uwb_link_fail_timeout_ms`（默认 30 s），让下游早 15 s 知道数据失效；串口刚打开时
`last_frame_time` 被重置，有同样时长的宽限，不会因为还没收到帧
就立刻报 `false`。节点首轮必发一次，之后只在 `true`/`false` 跳变时发布，latched 保留
当前值。状态话题的发布频率反映跳变次数，不是数据频率，**不要用 `rostopic hz` 判断标签是否在工作**。

`uwb_data.msg` 原文（[msg/uwb_data.msg](msg/uwb_data.msg)）：

```text
uint8 fre
float32 x
float32 y
float32 angle
float32 distance
```

**协议字段 → 消息字段映射**：

| 协议字段（定位帧） | 帧内偏移（0 起） | 消息字段 | 原始类型 / 单位 | 发布值 |
| --- | --- | --- | --- | --- |
| 标签 X 坐标 | 13~14 | `x` | 有符号 16 位，cm | m（×0.01） |
| 标签 Y 坐标 | 15~16 | `y` | 有符号 16 位，cm | m（×0.01） |
| 测距 | 17~18 | `distance` | **无符号** 16 位，cm | m（×0.01） |
| 角度 | 19~20 | `angle` | 有符号 16 位，1/100 度 | 度（×0.01） |
| 标签 ID | 5~12 | — | 8 字节 | **未解析** |
| 收帧频率 | — | `fre` | 客户端 1 秒窗口统计 | Hz，`uint8` |

**数据语义**：

- 长度原始单位是 **cm**，发布时换算为 **m**；角度原始单位是 **1/100 度**，发布为 **度**；
- 坐标系是**基站本地坐标系**，不是 ROS 的 `map` / `odom`。从基站正面看：
  - X **负半轴是基站右边，正半轴是基站左边**；
  - Y **正半轴是基站前面**，基站背面无法得到正确坐标（手册第 3 章）；
- 协议没有定义「无目标」的无效值。**没有定位帧就不发布**，不会发 0；读到 0 可能是标签
  真实位于坐标原点，**不是通讯故障**；
- `distance` 按无符号解析，超过 327 m 也不会变成负数；
- `fre` 是客户端统计的**收帧频率**：最近一个 1 秒窗口内 CRC 校验通过的 03H 整帧数，
  包含定位帧和新标签上报帧，不是标签配置里的「动态/静态频率」；节点启动后的第一秒为 0；
- 消息没有 `header`，时间戳取发布时刻，约等于帧到达时刻；需要精确时差请在上游记录；
- 标签 ID 未解析，多标签场景下无法区分数据来源。

## 默认行为

默认运行模式是**旁路订阅**（`uwb_self_collect=false`）：

- 不打开串口、不创建 `RtuClient`、不重连，可与旧 `serial_demo` 节点同时运行，启动顺序无关；
- 订阅 `uwb_source_topic`（默认 `/uwb_position`），在回调里原样转发到
  `/perception_sensors/uwb_position`，`fre` 等字段不改写；
- 1 Hz 检查最近一次收帧时间，超过 `uwb_heartbeat_timeout_ms`（默认 15 s）把
  `/perception_sensors/uwb_status` 置 `false`；首轮必发，之后只在跳变时发布；
- 转发目标与来源话题相同时会报错退出，避免自环；
- 订阅类型用本包 `uwb_rtu_serial_demo/uwb_data` 接旧包 `serial_demo/uwb_data`：
  两边字段逐字节相同，ROS1 按 MD5 握手可以连接；**旧包一旦修改 msg 字段，连接会静默断开**，
  需要同步更新。

主动采集模式（`uwb_self_collect:=true`）的行为如下：

- 串口**只读不写**：基站定位成功后主动推送，PC 无需问询（手册 4.4（1））；
- 轮询频率 **30 Hz**（`kLoopRateHz = 30.0`），每轮把缓冲中已到达的帧全部取完再发布；
  依据：标签动态默认 10 Hz、上限 20 Hz，30 Hz 约为数据频率的 1.5~3 倍；
- 发布**跟帧走**：收一帧发一帧，不用定时器重发旧帧；话题频率等于有效定位帧频率；
- 心跳：客户端按 1 秒窗口统计收帧频率；**15 s**（`uwb_heartbeat_timeout_ms`）没有 CRC
  有效帧时打印一次 `ROS_WARN`，恢复后自动复位。默认 15 s 是因为标签静态休眠默认 10 s
  才发一包，阈值过小会误报；标签休眠调大时必须同步调大该参数；
- 串口打开失败：节点**不再退出**，打印 `ROS_ERROR` 后保持运行，由主循环每 **5 s**
  （`uwb_startup_retry_sec`）重试；单台设备掉线不应拖垮顶层 launch 里的其它传感器；
- 运行中串口断开：自动重连，**不需要重启节点**；连续 **30 s**
  （`uwb_link_fail_timeout_ms`）没有有效帧（或读写异常）触发重试，重连失败后每 **10 s**
  （`uwb_runtime_reconnect_sec`）再试，插回串口后自动恢复；
- 30 Hz 外层轮询里只做轻量判断（`isAlive()`），真正的 `open()` 由启动/运行两档间隔
  （默认 5 s / 10 s）节流控制，不会每轮都尝试打开；
- `uwb_print_frame = true` 时逐帧打印 x / y / distance / angle。

## 掉线重连方案

本节只适用于**主动采集模式**。旁路模式不打开串口、不参与重连，来源停止发布时仅把
`/perception_sensors/uwb_status` 置 `false`，来源恢复后自动跟随，无需节点做任何动作。

主动采集模式下，串口是单设备、单消费者，最终选择在 `main.cpp` 的外层轮询里直接检测并重连；
`UwbNode` 保持纯解码，`RtuClient` 只负责底层异常捕获与重开。

| 环节 | 当前实现 |
| --- | --- |
| 启动探测 | 打开 `uwb_serial_port` 指定的设备；成功则打印 `打开成功（115200 8N1）`，失败只打印 `ROS_ERROR`，节点继续运行 |
| 启动失败 | 保持运行，首轮等 **5 s** 后重试，之后每 5 s 一次；期间不发布话题，不退出进程 |
| 断线判据 | `RtuClient::isAlive(uwb_link_fail_timeout_ms)` 为假（默认 30 s）：串口未打开、读写抛异常（`link_error_`），或阈值内没有 CRC 校验通过的整帧 |
| 读写异常 | `available()` / `read()` / `write()` / `close()` 都可能因 USB 拔出抛异常，客户端内部捕获并置 `link_error_`，不再解析断线前残留的半帧 |
| 重连动作 | `main` 在 30 Hz 外层循环检测，按 `next_reconnect` 节流调用 `RtuClient::reconnect()`：启动阶段 5 s、运行阶段 10 s |
| 重连内容 | 关闭旧句柄 → 清空 `rx_buf_` 与心跳窗口 → 用最近一次 `open()` 记录的 port / baudrate / timeout 重新打开；不回放旧帧 |
| 断线期间输出 | `/perception_sensors/uwb_position` **不发布**、不补 0、不发空消息；15 s 无有效帧后 `/perception_sensors/uwb_status` 跳变为 `false`，下游应优先用它判断失效 |
| 恢复判定 | `open()` 成功（串口重新打开且没有异常）；收到下一帧有效定位帧后立即恢复发布 |
| 心跳告警 | 15 s 无帧只告警一次，不触发重连；30 s 才进入重连逻辑，避免标签静态休眠造成误重连 |
| 日志 | 启动失败、重连失败分别由 `open()` / `reconnect()` 打印具体原因；1 Hz 心跳块对“串口异常等待重连”做 10 s 限流 |

### 三种落点评估

| 方案 | 落点 | 适用场景 | 优点 | 代价 | 本项目是否采用 |
| --- | --- | --- | --- | --- | --- |
| main 直接检测 + 节流重连 | `main.cpp` 外层 `while (ros::ok())` | 单设备、main 已持有 client 和循环节奏 | 改动最少；检测与重连时机一眼可见；无新线程、无回调生命周期问题 | 重连时 `open()` 会短暂阻塞主循环（通常 < 超时时间） | **UWB 采用** |
| 中层节点类 `ensureLink()` | `UltrasonicRadarNode` 等节点类 | 多路设备、需要在类内维护每条连接的状态与错峰 | 连接状态与业务封装在一起；main 循环保持干净 | 节点类要额外持有时间戳/节流状态；单设备时属于过度分层 | 超声波采用 |
| 客户端心跳回调 | `RtuClient` / `ModbusTcpClient` 内部 | 多消费者需要感知断线、事件驱动架构，或需要尽快通知最上层做急停 | 客户端自动上报，调用方无需轮询；检测点可直接通知最上层，不必等中层执行器轮询 | 回调生命周期、重入、重试语义都要额外约定；单设备、无急停需求时收益低 | 未采用（PTZ 风格） |

选择 main 方案的原因：UWB 只有一路串口、main 本来就持有 `RtuClient` 和 30 Hz 循环，
把“是否还活着 → 到点重连”放在同一个循环里最直观；超声波有两台网关和后台重探状态，
放中层 `UltrasonicRadarNode` 更合适。三种方案不混用。

回调方案还有一个容易被忽略的价值：**急停时延**。`pantilt_controller` 当初采用回调，
主要目的就是在断线时尽快让云台停下来。它把失败通知从底层心跳线程直接送到最上层
`PanTiltTaskManager::handle_heartbeat_failure()`，处理函数只做 `signal_interrupt()`
和置重连标志，中断信号立即生效，正在执行的扫描在下一个检查点就能退出。
如果改成由中层执行器（`PTZMoveHelper` / `PatrolScanner`）去发现并通知停止，
上层要等中层下一次轮询中断标志、或当前阻塞动作返回后才能感知，急停会明显滞后；
对于“一边运动一边等数据”的设备，这段滞后就是多走的距离。UWB 没有执行器，
不需要为急停抢这一段时间，所以这里不采用回调。

### 拔串口测试方案

用于验证启动失败重试、运行中自动重连和恢复，约 5 分钟：

1. **基线**：接好 USB-TTL 与基站，启动节点，确认 `rostopic hz /perception_sensors/uwb_position`
   有数据；
2. **运行中拔出**：拔掉 USB-TTL，确认节点不退出；约 15 s 打印一次无帧告警，30 s 后日志出现
   `尝试重连串口 ...`；
3. **离线启动**：保持拔出状态重启节点，确认打印 `打开失败，节点保持运行，每 5 s 重试`，
   进程持续存活且不发布话题；
4. **自动恢复**：插回 USB-TTL，确认日志出现 `重连成功（115200 8N1）`，话题恢复发布；
   若内核重新枚举成其它设备名（如 `/dev/ttyUSB0`），需用 launch 参数指定或补 udev 规则；
5. **退出**：Ctrl+C，节点应正常退出并关闭串口，下次可立即重新打开。

判定：全过程不需要重启节点。恢复慢于 30~60 s 时，先确认设备节点是否还存在
（`ls -l /dev/ttyUWB`）、当前用户是否仍在 `dialout` 组，再看重连日志中的具体异常。

## 参数

本包有 launch 文件 [launch/uwb_node.launch](launch/uwb_node.launch)。心跳、重连与时序相关的
现场可调项都放在 `<node>` 外的全局命名空间，由 [main.cpp](src/main.cpp) 用 `nh.param` 读取，
`main.cpp` 顶部只保留同名字面量作为默认值；其余参数仍写死在源码里。

### launch 可调

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `uwb_self_collect` | `false` | 运行模式：`false`=旁路订阅外部话题（不占串口、不重连）；`true`=主动打开串口采集 |
| `uwb_source_topic` | `/uwb_position` | 旁路模式订阅的来源话题；不能与转发目标 `/perception_sensors/uwb_position` 相同 |
| `uwb_serial_port` | `/dev/ttyUWB` | 基站 TTL 转串口设备路径；无 udev 时改成 `/dev/ttyUSB0` 等；仅主动采集模式使用 |
| `uwb_print_frame` | `true` | 是否逐帧打印定位结果；正式运行可设 `false` |
| `uwb_heartbeat_timeout_ms` | `15000` | 无帧告警阈值，ms：无有效帧多久告警、状态话题置 `false`；需大于标签静态休眠间隔（默认 10 s），休眠调大时同步调大 |
| `uwb_link_fail_timeout_ms` | `30000` | 连续无有效帧触发重连的阈值，ms；必须大于 `uwb_heartbeat_timeout_ms`，且大于标签静态休眠间隔 |
| `uwb_startup_retry_sec` | `5.0` | 启动打不开串口时的重试间隔，秒 |
| `uwb_runtime_reconnect_sec` | `10.0` | 运行中失联后的最小重连间隔，秒 |

非法值不会让节点退出：`uwb_link_fail_timeout_ms <= uwb_heartbeat_timeout_ms` 时自动改为
「告警阈值 + 1 s」；其余 ≤ 0 的阈值回退到默认值，均打印 `ROS_WARN`。

参数属于全局命名空间，命令行可直接覆盖：

```bash
# 默认旁路：订阅旧节点 /uwb_position，不占串口
roslaunch uwb_rtu_serial_demo uwb_node.launch
# 主动采集：先停掉旧 serial_demo，再打开本节点串口
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_self_collect:=true \
    uwb_serial_port:=/dev/ttyUSB0
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_print_frame:=false
# 标签静态休眠调到 30 s：告警与重连阈值一起调大
roslaunch uwb_rtu_serial_demo uwb_node.launch \
    uwb_heartbeat_timeout_ms:=40000 uwb_link_fail_timeout_ms:=60000
```

### 源码固定（改后重编译）

| 常量 | 默认值 | 含义 |
| --- | --- | --- |
| `kBaudrate` | `115200` | 波特率（手册 4.1 固定 115200 8N1，不要改） |
| `kLoopRateHz` | `30.0` | 串口轮询频率，Hz；标签提频到 20 Hz 以上时再调 |
| `kHeartbeatTimeoutMs` | `15000` | `uwb_heartbeat_timeout_ms` 的默认值，launch 可覆盖 |
| `kLinkFailTimeoutMs` | `30000` | `uwb_link_fail_timeout_ms` 的默认值，launch 可覆盖 |
| `kStartupRetrySec` | `5.0` | `uwb_startup_retry_sec` 的默认值，launch 可覆盖 |
| `kRuntimeReconnectSec` | `10.0` | `uwb_runtime_reconnect_sec` 的默认值，launch 可覆盖 |
| `kSelfCollect` | `false` | `uwb_self_collect` 的默认值，launch 可覆盖 |
| `kSourceTopic` | `/uwb_position` | `uwb_source_topic` 的默认值，launch 可覆盖 |

### 代码写死

| 项目 | 值 | 位置 |
| --- | --- | --- |
| 话题名 | `/perception_sensors/uwb_position` | `main.cpp` 的 `nh.advertise<...>("/perception_sensors/uwb_position", 10)` |
| 旁路订阅队列 | `10` | `main.cpp` 的 `nh.subscribe<...>(source_topic, 10, relay)` |
| 发布队列 | `10` | 同 advertise，约 1 秒缓存 |
| 消息类型 | `uwb_rtu_serial_demo/uwb_data` | 两种模式都用本包类型 |
| 串口格式 | 8N1 | [rtu_client.cpp](src/rtu_client.cpp) 的 `open()` |
| 只接受功能码 | `03H` | [rtu_client.h](include/uwb_rtu_serial_demo/rtu_client.h) 的 `kFuncRead` |
| 接收缓冲上限 | 4096 字节 | `rtu_client.h` 的 `kMaxBuffer` |

### 协议固定值（不要改）

| 项目 | 值 | 依据 |
| --- | --- | --- |
| 波特率 / 数据位 / 停止位 / 校验 | 115200 / 8 / 1 / 无 | 手册 4.1 |
| 定位帧长度 | 23 字节 | 手册 4.4（1） |
| 定位帧数据长度 `LEN` | `0x12`（18） | 手册 4.4（1） |
| 定位标识 | `CA DA` | 手册 4.4（1） |
| 新标签上报标识 | `40 AD`，`LEN = 0x0A` | 手册 4.4（2） |
| 字段偏移 | X 13、Y 15、距离 17、角度 19 | 本文件「字段映射」表 |
| 单位换算 | cm → m `×0.01`；1/100 度 → 度 `×0.01` | 手册 4.4（1） |
| CRC16 | 初值 `0xFFFF`，多项式 `0xA001`，低字节先发 | 手册 4.2 |

> 上表参数都是 launch 全局参数；`rosrun` 的私有参数写法 `_uwb_serial_port:=...` **不生效**。

## 配置服务器

launch 文件：[launch/uwb_node.launch](launch/uwb_node.launch)。现场可调项在 `<node>` 外
定义，属于全局命名空间。

```bash
# 默认参数启动（旁路订阅，不占串口）
roslaunch uwb_rtu_serial_demo uwb_node.launch

# 主动采集：指定串口并关闭逐帧打印（先停旧 serial_demo）
roslaunch uwb_rtu_serial_demo uwb_node.launch \
  uwb_self_collect:=true \
  uwb_serial_port:=/dev/ttyUSB0 \
  uwb_print_frame:=false

# 旁路订阅其它来源话题
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_source_topic:=/uwb_position
```

轮询频率、心跳阈值、重连阈值仍需改 [main.cpp](src/main.cpp) 顶部常量后重新编译。

## 编译与运行

```bash
cd /home/liyun/code/perception_sensors_ros_driver/catkin_ws
catkin_make --only-pkg-with-deps uwb_rtu_serial_demo \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

# 默认启动（旁路订阅，先起旧 serial_demo 或之后起都可以）
roslaunch uwb_rtu_serial_demo uwb_node.launch

# 主动采集（先停掉旧 serial_demo，避免串口被占）
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_self_collect:=true \
    uwb_serial_port:=/dev/ttyUSB0

# 或直接 rosrun（使用 main.cpp 中的默认值：旁路订阅 /uwb_position）
rosrun uwb_rtu_serial_demo uwb_node
```

运行前置：

- 旁路模式（默认）：旧 `serial_demo` 节点在跑并发布 `/uwb_position` 即可，不要求串口权限；
- 主动采集模式：`/dev/ttyUWB` 存在或启动时用 `uwb_serial_port:=...` 指定实际设备；
  当前用户在 `dialout` 组，对串口有读写权限；
- 主动采集还要求标签已加入基站 K 列表，否则只会收到新标签上报帧、不会收到定位帧。

验证：

```bash
rostopic echo /perception_sensors/uwb_position
rostopic hz /perception_sensors/uwb_position
rostopic echo -n1 /perception_sensors/uwb_status
```

协议层面的硬约束：串口必须 **115200 8N1**，链路必须是 **TTL 电平**；本节点只读，
不会发送 03H / 06H / 10H / 40H / 50H 请求。

## 实现说明

### 文件职责

| 文件 | 职责 |
| --- | --- |
| [src/main.cpp](src/main.cpp) | 读取 launch 参数与字面量参数；旁路模式只订阅来源话题、原样转发并做 1 Hz 状态判定；主动采集模式打开串口、创建 `UwbNode`、`advertise("/perception_sensors/uwb_position", 10)`、30 Hz 轮询循环、逐帧发布、心跳告警、链路探针与节流重连 |
| [src/rtu_client.cpp](src/rtu_client.cpp) / [include/uwb_rtu_serial_demo/rtu_client.h](include/uwb_rtu_serial_demo/rtu_client.h) | **无 ROS 依赖**的 Modbus-RTU 客户端：打开/关闭/重开串口、读写异常捕获、字节流累积、帧头同步、CRC16、收帧心跳 |
| [src/uwb_node.cpp](src/uwb_node.cpp) / [include/uwb_rtu_serial_demo/uwb_node.h](include/uwb_rtu_serial_demo/uwb_node.h) | **无 ROS 依赖**的业务解码：过滤定位帧、校验 `CA DA`、大端转换、单位换算 |
| [msg/uwb_data.msg](msg/uwb_data.msg) | 话题消息定义 |
| [launch/uwb_node.launch](launch/uwb_node.launch) | 两个现场可调参数与节点启动 |
| [CMakeLists.txt](CMakeLists.txt) | 编译 `rtu_client` 库与 `uwb_node` 可执行文件 |

### 架构边界

- 发布器、话题名、`ros::Rate`、`ros::spinOnce()`、链路探针与重连节流只在 `main.cpp`；
- `UwbNode` 只持有 `RtuClient&`，不持有 ROS 句柄、发布器或话题名，也不参与重连决策；
- `RtuClient` 不依赖 ROS，可被其它节点或离线工具直接复用；
- `RtuClient` 负责底层状态：保存最近一次 `open()` 的参数、捕获读写异常（`link_error_`）、
  提供 `reconnect()` / `linkError()`；何时重连由 `main` 决定；
- 心跳与收帧统计在 `RtuClient` 内部，`UwbNode` 不重复计数。

### 协议实现范围

- 只解析 **03H 应答**，且只把满足「23 字节 + 03H + `LEN=0x12` + `CA DA`」的帧当定位帧；
- 新标签上报帧（`LEN=0x0A`、`40 AD`）、标签配置上报帧（`LEN=0x20`、`5C 5F`）等其它
  03H 帧由 `UwbNode::isPositionFrame()` 丢弃；
- `RtuClient::write()` 只做原始发送，**没有实现** 03H / 06H / 10H / 40H / 50H 的请求组帧；
- CRC16 已实现，并在每一帧上校验。

### 分帧能力

`RtuClient::readFrame()` 把串口当**字节流**处理，不假设一次 `read()` 等于一帧：

- 数据先追加到内部缓冲，再逐字节找候选帧头（`ADR 1~247` + `03H` + 偶数 `LEN` + `LEN ≤ 250`）；
- 帧未收全：保留半帧，等下一次读取拼接；
- 粘包：一次取出多帧，调用方循环取完；
- 垃圾字节：不是候选帧头就前进 1 字节；
- CRC 不符：丢 1 字节重新同步；
- 缓冲超过 4096 字节：只保留最近一段，避免异常数据无限增长；
- 打开串口时 `flushInput()`，避免上次运行的残留字节影响首帧。

### 失败处理

| 场景 | 行为 |
| --- | --- |
| 启动时串口打开失败 | `ROS_ERROR` 后节点保持运行，每 5 s 重试 |
| 当前没有完整帧 | `readFrame()` 返回 `false`，主循环继续下一轮 |
| CRC 错 / 垃圾字节 | 丢 1 字节重新找帧头 |
| 非定位帧 | 丢弃，继续取下一帧 |
| 15 s 无有效帧 | `ROS_WARN` 一次，恢复后复位告警；不触发重连 |
| 30 s 无有效帧 | 进入重连逻辑，运行阶段每 10 s 重试一次 |
| 运行中读/写异常 | 置 `link_error_`，立即视为失联，按 10 s 节流重连 |
| 运行中串口拔出 | 自动重连，插回后恢复发布，无需重启节点 |

## 已知限制

| 位置 | 问题 | 影响 |
| --- | --- | --- |
| `uwb_node.cpp` | 不解析标签 ID | 多标签场景无法区分来源 |
| `main.cpp` | 重连在 30 Hz 主循环内同步执行 | `open()` 失败通常很快返回，但设备异常时可能阻塞到驱动超时，期间的帧会延后处理 |
| `main.cpp` | 重连只重开同一设备路径 | USB 重新枚举成 `/dev/ttyUSB1` 等新名字时，需改 launch 参数或补 udev 规则 |
| `main.cpp` | 标签静态休眠超过 30 s 会触发无意义重连 | 重连成功会重置 `last_frame_time_`，约每 30 s 重开一次串口；不丢定位帧，但日志会周期性出现重连记录。标签静态间隔调大时，同步调大 launch 参数 `uwb_link_fail_timeout_ms` |
| `main.cpp` | 轮询频率、心跳与重连阈值仍写死 | 改这些值需重新编译（串口与打印开关已进 launch） |
| `uwb_data.msg` | 无 `header` | 无精确时间戳、无坐标系标识 |
| `rtu_client.cpp` | 只接受 03H 应答 | 将来下发 40H/50H 时需扩展应答解析 |
| `uwb_node.cpp` | 不做坐标变换、滤波、平滑 | 输出为基站本地原始坐标 |
| `main.cpp` | 旁路模式依赖两边 msg 的 MD5 相同 | 用 `uwb_rtu_serial_demo/uwb_data` 订阅旧 `serial_demo/uwb_data`；旧包一旦修改字段，连接会静默断开，转发话题和状态检测都收不到数据 |
| `main.cpp` | 旁路模式看不到串口层 CRC | 只能反映旧节点是否还在发消息，无法发现旧节点已经发布的错误数据 |
| 硬件 | 基站背面无法定位 | 标签需处于基站正面约 120° 内 |

协议、接线、排障与更多现场问题见[仓库 README](../../../README.md)。
