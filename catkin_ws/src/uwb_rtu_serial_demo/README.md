# uwb_rtu_serial_demo

D-PDOA-Node-B 基站经 **TTL UART（USB-TTL，默认 `/dev/ttyUWB`）** 输出
**Modbus-RTU** 定位帧，本包把定位帧解码后发布为 ROS 话题。
本包替代已移除的旧 `serial_demo`，不再维护旧代码；接入方式、接线和协议细节见
[仓库 README](../../../README.md)。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/uwb_position` | `uwb_rtu_serial_demo/uwb_data` | 每收到一帧有效定位帧发布一次；频率跟随标签发包（动态默认约 10 Hz，静态可低至 0.05 Hz）；队列 10 |

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

- 串口**只读不写**：基站定位成功后主动推送，PC 无需问询（手册 4.4（1））；
- 轮询频率 **30 Hz**（`kLoopRateHz = 30.0`），每轮把缓冲中已到达的帧全部取完再发布；
  依据：标签动态默认 10 Hz、上限 20 Hz，30 Hz 约为数据频率的 1.5~3 倍；
- 发布**跟帧走**：收一帧发一帧，不用定时器重发旧帧；话题频率等于有效定位帧频率；
- 心跳：客户端按 1 秒窗口统计收帧频率；**15 s** 没有 CRC 有效帧时打印一次 `ROS_WARN`，
  恢复后自动复位。阈值取 15 s 是因为标签静态休眠默认 10 s 才发一包，阈值过小会误报；
- 串口打开失败：打印 `ROS_ERROR` 后直接退出（没有数据源，原地重试没有意义）；
- 运行中串口断开：**不自动重连**，需要重启节点；
- `uwb_print_frame = true` 时逐帧打印 x / y / distance / angle。

## 参数

本包有 launch 文件 [launch/uwb_node.launch](launch/uwb_node.launch)。两个现场可调项放在
`<node>` 外的全局命名空间，由 [main.cpp](src/main.cpp) 用 `nh.param` 读取；其余参数是
`main.cpp` 顶部的字面量，修改后需要重新编译。

### launch 可调

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `uwb_serial_port` | `/dev/ttyUWB` | 基站 TTL 转串口设备路径；无 udev 时改成 `/dev/ttyUSB0` 等 |
| `uwb_print_frame` | `true` | 是否逐帧打印定位结果；正式运行可设 `false` |

参数属于全局命名空间，命令行可直接覆盖：

```bash
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_serial_port:=/dev/ttyUSB0
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_print_frame:=false
```

### 源码固定（改后重编译）

| 常量 | 默认值 | 含义 |
| --- | --- | --- |
| `kBaudrate` | `115200` | 波特率（手册 4.1 固定 115200 8N1，不要改） |
| `kLoopRateHz` | `30.0` | 串口轮询频率，Hz；标签提频到 20 Hz 以上时再调 |
| `kHeartbeatTimeoutMs` | `15000` | 无帧告警阈值，ms；标签静态休眠调大时同步调大 |

### 代码写死

| 项目 | 值 | 位置 |
| --- | --- | --- |
| 话题名 | `uwb_position` | `main.cpp` 的 `nh.advertise<...>("uwb_position", 10)` |
| 发布队列 | `10` | 同上，约 1 秒缓存 |
| 消息类型 | `uwb_rtu_serial_demo/uwb_data` | 同上 |
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

> `uwb_serial_port` / `uwb_print_frame` 是 launch 全局参数；`rosrun` 的私有参数写法
> `_uwb_serial_port:=...` **不生效**。

## 配置服务器

launch 文件：[launch/uwb_node.launch](launch/uwb_node.launch)。两个现场可调项在 `<node>` 外
定义，属于全局命名空间。

```bash
# 默认参数启动
roslaunch uwb_rtu_serial_demo uwb_node.launch

# 指定串口并关闭逐帧打印
roslaunch uwb_rtu_serial_demo uwb_node.launch \
  uwb_serial_port:=/dev/ttyUSB0 \
  uwb_print_frame:=false
```

轮询频率、心跳阈值仍需改 [main.cpp](src/main.cpp) 顶部常量后重新编译。

## 编译与运行

```bash
cd /home/liyun/code/devices/uwb_demo/catkin_ws
catkin_make --only-pkg-with-deps uwb_rtu_serial_demo
source devel/setup.bash

# 启动节点（推荐，参数见 launch 文件）
roslaunch uwb_rtu_serial_demo uwb_node.launch

# 指定串口设备
roslaunch uwb_rtu_serial_demo uwb_node.launch uwb_serial_port:=/dev/ttyUSB0

# 或直接 rosrun（使用 main.cpp 中的默认值）
rosrun uwb_rtu_serial_demo uwb_node
```

运行前置：

- `/dev/ttyUWB` 存在，或启动时用 `uwb_serial_port:=...` 指定实际设备；
- 当前用户在 `dialout` 组，对串口有读写权限；
- 标签已加入基站 K 列表，否则只会收到新标签上报帧、不会收到定位帧。

验证：

```bash
rostopic echo /uwb_position
rostopic hz /uwb_position
```

协议层面的硬约束：串口必须 **115200 8N1**，链路必须是 **TTL 电平**；本节点只读，
不会发送 03H / 06H / 10H / 40H / 50H 请求。

## 实现说明

### 文件职责

| 文件 | 职责 |
| --- | --- |
| [src/main.cpp](src/main.cpp) | 读取 launch 参数与字面量参数、打开串口、创建 `UwbNode`、`advertise("uwb_position", 10)`、30 Hz 轮询循环、逐帧发布、心跳告警 |
| [src/rtu_client.cpp](src/rtu_client.cpp) / [include/uwb_rtu_serial_demo/rtu_client.h](include/uwb_rtu_serial_demo/rtu_client.h) | **无 ROS 依赖**的 Modbus-RTU 客户端：打开/关闭串口、字节流累积、帧头同步、CRC16、收帧心跳 |
| [src/uwb_node.cpp](src/uwb_node.cpp) / [include/uwb_rtu_serial_demo/uwb_node.h](include/uwb_rtu_serial_demo/uwb_node.h) | **无 ROS 依赖**的业务解码：过滤定位帧、校验 `CA DA`、大端转换、单位换算 |
| [msg/uwb_data.msg](msg/uwb_data.msg) | 话题消息定义 |
| [launch/uwb_node.launch](launch/uwb_node.launch) | 两个现场可调参数与节点启动 |
| [CMakeLists.txt](CMakeLists.txt) | 编译 `rtu_client` 库与 `uwb_node` 可执行文件 |

### 架构边界

- 发布器、话题名、`ros::Rate`、`ros::spinOnce()` 只在 `main.cpp`；
- `UwbNode` 只持有 `RtuClient&`，不持有 ROS 句柄、发布器或话题名；
- `RtuClient` 不依赖 ROS，可被其它节点或离线工具直接复用；
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
| 串口打开失败 | `ROS_ERROR` 后退出，返回码 `-1` |
| 当前没有完整帧 | `readFrame()` 返回 `false`，主循环继续下一轮 |
| CRC 错 / 垃圾字节 | 丢 1 字节重新找帧头 |
| 非定位帧 | 丢弃，继续取下一帧 |
| 15 s 无有效帧 | `ROS_WARN` 一次，恢复后复位告警 |
| 运行中串口拔出 | 不重连，需重启节点 |

## 已知限制

| 位置 | 问题 | 影响 |
| --- | --- | --- |
| `uwb_node.cpp` | 不解析标签 ID | 多标签场景无法区分来源 |
| `main.cpp` | 串口断开不自动重连 | 需人工重启 |
| `main.cpp` | 轮询频率、心跳阈值仍写死 | 改这两个值需重新编译（串口与打印开关已进 launch） |
| `uwb_data.msg` | 无 `header` | 无精确时间戳、无坐标系标识 |
| `rtu_client.cpp` | 只接受 03H 应答 | 将来下发 40H/50H 时需扩展应答解析 |
| `uwb_node.cpp` | 不做坐标变换、滤波、平滑 | 输出为基站本地原始坐标 |
| 硬件 | 基站背面无法定位 | 标签需处于基站正面约 120° 内 |

协议、接线、排障与更多现场问题见[仓库 README](../../../README.md)。
