# perception_sensors_ros_driver

面向巡检/底盘设备的感知传感器 ROS 1 Noetic 工作区：接入速腾 Airy360 激光雷达、DYP 电应普超声波雷达
和 UWB 基站，把三类数据统一发布到 `/perception_sensors/` 命名空间，并提供 rosbag 录制控制与
底盘保存按键联动。

## 系统组成

| 包 | 一句话职责 | 入口 |
| --- | --- | --- |
| [rslidar_sdk](catkin_ws/src/rslidar_sdk-v1.5.20/) | 速腾官方驱动，UDP 收包并发布原始点云 | `launch/start.launch` |
| [lidar_preprocess](catkin_ws/src/lidar_preprocess/) | 点云滑动窗口多帧累加与雷达心跳 | `lidar_preprocess.cpp` |
| [ultrasonic_radar_modbus_tcp](catkin_ws/src/ultrasonic_radar_modbus_tcp/) | Modbus TCP 网关采集两组各 4 路超声波 | `src/main.cpp` |
| [uwb_rtu_serial_demo](catkin_ws/src/uwb_rtu_serial_demo/) | Modbus-RTU 串口解析 UWB 定位帧 | `src/main.cpp` |
| [bag_recorder](catkin_ws/src/bag_recorder/) | 服务控制的 rosbag 录制/停止 | `src/bag_recorder_node.cpp` |
| [perception_sensors_client_main](catkin_ws/src/perception_sensors_client_main/) | 顶层 launch 一键启动；底盘保存按键联动录制 | `src/main.cpp` |

各包的协议帧、寄存器、线程模型和参数细节以包内 README 与源码为准；`doc/` 下是原独立工程的文档副本，
路径可能指向已不存在的工作区。

## 硬件链路

| 传感器 | 链路 | 地址 / 设备 | 关键参数 |
| --- | --- | --- | --- |
| Airy360 激光雷达 | 以太网 UDP | 雷达 `192.168.1.68`；主机 `192.168.1.100`；MSOP 6699 / DIFOP 7788 | `lidar_type: RSAIRY`，`use_lidar_clock: true` |
| DYP 电应普超声波雷达（两组） | RS485 → M04 串口服务器 → 以太网 | 网关 `192.168.1.15`，502（车体左侧）/ 503（车体右侧） | RS485 9600 8N1，从机地址 1，每组 4 路探头 |
| UWB 基站 | TTL UART → USB-TTL | 默认 `/dev/ttyUWB` | 115200 8N1，基站主动推送，PC 只读不写 |

## 数据流

```text
Airy360 ──UDP 6699/7788──► rslidar_sdk_node ──/rslidar_points──► lidar_preprocess_node
                                                                      │
                                          /perception_sensors/processed_lidar_cloud
                                                                      ▼
DYP 电应普超声波雷达 ×2 ──RS485──► M04 ──Modbus TCP──► ultrasonic_radar_modbus_tcp ──┐
                                                                     │
UWB 基站 ──TTL 115200──► uwb_node ──/perception_sensors/uwb_position─┤
                                                                     ▼
                                                        bag_recorder_node ──rosbag──► log/rosbag/
                                                                     ▲
底盘 ──/chassis_robot/chassis_status──► perception_sensors_client_main
                                            │ start/stop 服务
                                            └──────────────────────────┘
```

- 激光雷达：驱动回调收包，预处理后台线程累加，主循环 10 Hz 取结果发布。
- 超声波：主动轮询，每轮一条聚合消息，空数组表示本轮全部网关读取失败；
  启动时读不到数据的网关先跳过，由后台线程每 30 s 重探，接好模组后自动上线。
  掉线重连细节与拔线测试步骤见
  [ultrasonic_radar_modbus_tcp/README.md](catkin_ws/src/ultrasonic_radar_modbus_tcp/README.md#掉线重连方案)。
- UWB：串口主动轮询，收一帧发一帧；启动时打不开串口不退出，每 5 s 重试；运行中读写异常或
  30 s 无有效帧时按 10 s 节流自动重连，插回 USB-TTL 后无需重启节点。判定细节、三种重连落点
  对比与拔串口测试步骤见
  [uwb_rtu_serial_demo/README.md](catkin_ws/src/uwb_rtu_serial_demo/README.md#掉线重连方案)。
- 录制：服务事件驱动；保存按键节点只负责把底盘状态翻译成 start/stop 服务调用。

## 话题与服务契约

| 名称 | 类型 | 方向 | 频率 | 特殊语义 |
| --- | --- | --- | --- | --- |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 发布（驱动） | ≈10 Hz | 原始点云，`frame_id=rslidar` |
| `/perception_sensors/processed_lidar_cloud` | `sensor_msgs/PointCloud2` | 发布 | ≈10 Hz | 窗口内 `frame_quantity` 帧逐点相加，启动后约 1 s 才有输出 |
| `/perception_sensors/processed_lidar_connection_status` | `std_msgs/Bool` | 发布 | 仅状态跳变 | latched；后启动的订阅者能立即拿到当前状态 |
| `/perception_sensors/ultrasonic_radar_datas` | `ultrasonic_radar_modbus_tcp/ultrasonic_radar_datas` | 发布 | ≈3.3 Hz | 聚合消息，`radars[]` 为空表示本轮全部失败 |
| `/perception_sensors/ultrasonic_radar_status/left_radar_502` | `std_msgs/Bool` | 发布 | 仅状态跳变 | latched；`true` 本轮读到数据，`false` 超时/离线/未上线 |
| `/perception_sensors/ultrasonic_radar_status/right_radar_503` | `std_msgs/Bool` | 发布 | 仅状态跳变 | latched；判定口径同上 |
| `/perception_sensors/uwb_position` | `uwb_rtu_serial_demo/uwb_data` | 发布 | 跟标签上报走 | 没有定位帧就不发布；消息无 `header` |
| `/perception_sensors/uwb_status` | `std_msgs/Bool` | 发布 | 仅状态跳变 | latched；`true` 在线，`false` 串口异常或 15 s 无有效帧 |
| `/perception_sensors/recording_status` | `std_msgs/Bool` | 发布 | 仅状态跳变 | latched，录制中为 `true` |
| `/perception_sensors/recording_size` | `std_msgs/Float64` | 发布 | 录制中 1 Hz + 停止时一次 | 当前批次现存分卷总大小（GB，GiB）；`delete_old` 后数值会下降 |
| `/chassis_robot/chassis_status` | `std_msgs/Bool` | 订阅 | 由底盘决定 | `true`=开始保存，`false`=停止保存 |
| `/perception_sensors/start_bag_recorder` | `std_srvs/Trigger` | 服务 | 事件 | 开始录制；重复调用返回 `Already recording!` |
| `/perception_sensors/stop_bag_recorder` | `std_srvs/Trigger` | 服务 | 事件 | 停止录制；未录制时返回 `Not recording!` |

## 设备在线状态与重连参数

三类传感器都有在线状态话题，均为 `std_msgs/Bool`、latched、**仅在状态跳变时发布**。
`true` 表示链路可用，`false` 表示超时或断开。

| 设备 | 状态话题 | 判 `false` 的条件 | 恢复方式 |
| --- | --- | --- | --- |
| 激光雷达 | `/perception_sensors/processed_lidar_connection_status` | 20 s 内没有收到点云 | 重连由 `rslidar_sdk` 内置处理，本仓库不实现；雷达恢复发包后自动恢复。`lidar_preprocess` 在超时期间清空累加窗口，恢复后要重新攒满 `frame_quantity` 帧（默认约 1 s）才输出点云 |
| 超声波左（502） | `/perception_sensors/ultrasonic_radar_status/left_radar_502` | 本轮读寄存器失败：应答超时、连接被拒、网关忙或异常帧；启动探测失败也是 `false` | 已上线断线按 1 s 重连；启动就失败的端口由后台线程每 30 s 重探 |
| 超声波右（503） | `/perception_sensors/ultrasonic_radar_status/right_radar_503` | 同上 | 同上 |
| UWB | `/perception_sensors/uwb_status` | 串口未打开、读写异常，或 15 s 内没有 CRC 有效的整帧 | 启动打不开每 5 s 重试；运行中失联每 10 s 重连，插回串口后自动恢复 |

重连相关参数：

| 设备 | 参数 | 默认值 | 位置 | 含义 |
| --- | --- | --- | --- | --- |
| 激光雷达 | `timeout_sec` | 20 s | `lidar_preprocess.cpp` 写死 | 无点云多久判超时；launch 里的同名 arg（默认 5.0）没有注入节点，改它不生效 |
| 激光雷达 | 心跳检查周期 | 1 s | `lidar_sub.cpp` 写死 | 定时器多久判一次连/断 |
| 超声波 | `poll_interval` | 0.3 s | launch | 轮询周期；状态最多晚一轮（0.3 s）变化 |
| 超声波 | `timeout` | 0.5 s | launch | TCP 连接与单次应答超时 |
| 超声波 | `reconnect_interval` | 1.0 s | launch | 已上线服务器断线后的重连间隔 |
| 超声波 | `reprobe_interval` | 30.0 s | launch | 启动探测失败端口的后台重探间隔；`0` 关闭 |
| UWB | `kHeartbeatTimeoutMs` | 15 s | `uwb_rtu_serial_demo/src/main.cpp` 写死 | 无有效帧多久告警、状态话题置 `false` |
| UWB | `kLinkFailTimeoutMs` | 30 s | 同上 | 连续无有效帧多久触发重连 |
| UWB | `kStartupRetrySec` | 5 s | 同上 | 启动打不开串口时的重试间隔 |
| UWB | `kRuntimeReconnectSec` | 10 s | 同上 | 运行中失联后的重连间隔 |

几点语义：

- 状态话题频率只反映跳变次数，**不能拿 `rostopic hz` 判断设备在线**；后启动的订阅者靠 latched 拿当前值。
- UWB 的状态阈值（15 s）小于重连阈值（30 s）：下游先知道数据失效，节点随后才重连。
- 超声波某一路 `false` 不代表节点故障：503 未接模组时它一直是 `false`，502 照常轮询。
- 激光雷达在收到第一帧之前不做超时判定，也不会发布状态消息；从未收到数据的场景要看点云话题是否更新。
- 激光雷达的重连交给 `rslidar_sdk` 内置处理，本仓库不写重连逻辑；`lidar_preprocess` 只做
  超时判定与清缓冲（在 `LidarSub` 的心跳定时器里），细节见
  [lidar_preprocess/README.md](catkin_ws/src/lidar_preprocess/README.md)。
- 超声波与 UWB 的判定细节、实测恢复耗时见各自包 README 的「掉线重连方案」章节。

## 数据语义

只列订阅方必须知道的约定，字段表见各包 README。

- **激光雷达**：`POINT_TYPE=XYZI`，`x/y/z/intensity` 均为 float32，`point_step=16`；
  `dense_points=false`，点云可能含 NaN。工程没有 TF，rviz 的 Fixed Frame 要设为 `rslidar`。
  输出时间戳 = 雷达时间 + 首帧计算的一次性偏移。
- **超声波**：寄存器原始单位 mm，节点换算成 m；读数 ≤ 0.1 m（含 0）统一发布 `-1.0`，
  表示该路无目标，订阅方用 `< 0` 判断。**读到 0 不是通讯故障**。每组 4 路依次为
  `Front / Side1 / Side2 / Back`；502 为车体左侧，503 为车体右侧。
- **UWB**：坐标 cm→m，角度 1/100 度→度，测距为无符号 16 位。坐标系是基站本地坐标系，
  从基站正面看 X 负半轴在右、正半轴在左，Y 正半轴在前；基站背面无法正确解算。
  标签未定位时不发布消息，不会发 0。
- **链路状态**：超声波每台网关一个 `std_msgs/Bool`，UWB 一个，均为「跳变时发布 + latched」：
  节点首轮必发一次，之后只在 `true`/`false` 变化时发一条，后启动的订阅者用 latched 拿当前值。
  `true` 表示链路可用（超声波=最近一轮读到寄存器，UWB=15 s 内收到过有效帧），
  `false` 表示超时或断开。超声波读到全 0 mm 仍算在线（无目标不是通讯故障）。
  **不要用 `rostopic hz` 判断设备在线**，这类话题的频率只反映状态跳变次数。
- **录制**：文件名 `perception_sensors_<6位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag`。
  序号在启动时扫描保存目录续号，时间戳只用于可读，时钟不准不会导致重名。
  单卷固定 10 GB，超过后 rosbag 自动创建下一卷；总上限与超限动作见启动参数表。
- **录制大小**：`/perception_sensors/recording_size` 单位 GB（GiB，1024³ 字节），
  统计当前批次现存分卷（含正在写入的 `.bag.active`）；`delete_old` 删除旧卷后数值会下降，
  反映的是磁盘占用而不是本次累计写入量。开始录制时发 0，停止时发一次最终值。

## 频率链

| 链路 | 硬件发出/上报 | 节点采集节奏 | 话题发布 |
| --- | --- | --- | --- |
| 激光雷达 | Airy360 600 rpm，约 10 Hz | 回调每帧入队，后台线程维护累加长度 `frame_quantity` 的窗口；主循环 10 Hz 轮询 | ≈10 Hz；前 `frame_quantity` 帧（默认 10 帧 ≈1 s）不输出；每帧点数是窗口内各帧直接相加，默认约 10 倍带宽 |
| 超声波 | 设备只应答，不主动推送 | `poll_interval=0.3 s`，每轮遍历两个网关 | ≈3.3 Hz，每轮一条聚合消息 |
| UWB | 标签驱动：动态默认约 10 Hz，上限 20 Hz；静态默认 0.1 Hz（可配置到 0.05 Hz） | 30 Hz 串口轮询，每轮取完缓冲 | 等于有效定位帧频率，不是固定 30 Hz |
| 录制 | 事件驱动 | 服务 start/stop | 无周期 |
| 保存按键 | 由底盘发布（频率未知） | 回调内按状态去重，仅跳变时调服务 | 无周期 |

轮询周期不等于话题频率；激光雷达多帧累加还存在约 `frame_quantity` 倍的数据量放大。

## 启动与部署

### 编译

```bash
cd /home/liyun/code/perception_sensors_ros_driver/catkin_ws
conda deactivate
catkin_make -j8
source devel/setup.bash
```

- 首次编译依赖：`sudo apt-get install -y libpcap-dev libeigen3-dev libboost-dev libpcl-dev`，
  UWB 包还需要 `ros-noetic-serial`。
- 无法退出 conda 时追加 `-DPYTHON_EXECUTABLE=/usr/bin/python3`。
- `catkin_make --only-pkg-with-deps <包名>` 会把 `CATKIN_WHITELIST_PACKAGES` 固化，之后全量构建
  要用 `catkin_make -DCATKIN_WHITELIST_PACKAGES=""` 复位。

### 一键启动

```bash
roslaunch perception_sensors_client_main perception_sensors_client_main.launch
roslaunch perception_sensors_client_main perception_sensors_client_main.launch \
    gateway_ip:=192.168.1.15 save_dir:=/data/rosbag max_bag_size_gb:=25 total_limit_action:=delete_old
```

顶层 launch 依次 include：`lidar_preprocess` → `ultrasonic_radar_modbus_tcp` → `uwb_rtu_serial_demo`
→ `bag_recorder`，最后启动保存按键联动节点。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gateway_ip` | `192.168.1.15` | 超声波 Modbus 网关地址 |
| `save_dir` | 空 | 空表示仓库根目录的 `log/rosbag`，目录不存在时自动创建 |
| `max_bag_size_gb` | `25.0` | 单次录制总大小上限（GB）；单卷固定 10 GB，源码硬编码 |
| `total_limit_action` | `stop` | 达到总上限时：`stop` 停止录制；`delete_old` 删除最旧分卷后继续 |

UWB 串口名在 `uwb_rtu_serial_demo/launch/uwb_node.launch` 中配置；雷达网络参数在
`rslidar_sdk-v1.5.20/config/config.yaml` 中配置。

### 单独启动

```bash
roslaunch lidar_preprocess lidar_preprocess.launch
roslaunch ultrasonic_radar_modbus_tcp ultrasonic_radar_modbus_tcp.launch
roslaunch uwb_rtu_serial_demo uwb_node.launch
roslaunch bag_recorder bag_recorder.launch
rosservice call /perception_sensors/start_bag_recorder
rosservice call /perception_sensors/stop_bag_recorder
```

### 验证

仓库没有单元测试和 lint 配置，验证方式是编译、接实机并用 `rostopic` 看频率与内容：

```bash
rostopic hz   /perception_sensors/processed_lidar_cloud
rostopic echo -n1 /perception_sensors/processed_lidar_connection_status
rostopic echo /perception_sensors/ultrasonic_radar_datas
rostopic echo -n1 /perception_sensors/ultrasonic_radar_status/left_radar_502
rostopic echo /perception_sensors/uwb_position
rostopic echo -n1 /perception_sensors/uwb_status
rostopic echo -n1 /perception_sensors/recording_status
rostopic echo /perception_sensors/recording_size
```

## 已知陷阱

1. **话题命名漂移**：代码已统一到 `/perception_sensors/*`，但 `doc/` 和部分包 README 仍写旧名
   （`/lidar/processed_cloud`、`/lidar/connection_status`、`/uwb_position` 等）。以代码中的
   `advertise/subscribe` 为准。
2. **WSL2 默认 NAT 收不到雷达 UDP**：需要 mirrored 网络，或在 Windows 侧做端口转发；
   同时确认雷达网页里的点云目的地址是 `192.168.1.100`。
3. **录制服务与状态话题已改名**：服务为 `/perception_sensors/start_bag_recorder`、
   `/perception_sensors/stop_bag_recorder`，状态话题为 `/perception_sensors/recording_status`；
   旧的 `/robot_detector/*` 名称不再存在，外部订阅端需同步修改。
4. **单卷固定 10 GB，`max_bag_size_gb=25` 是单次录制总上限**：节点每秒统计当前批次
   所有分卷（含正在写入的 `.bag.active`）。`total_limit_action=stop`（默认）时达到
   25 GB 停止录制；`delete_old` 时删除最旧的已关闭分卷后继续，正在写入的卷不删。
   删除以整卷为单位，因此 `delete_old` 下磁盘占用会在 15~25 GB 之间波动，而不是
   精确停在 25 GB。`--split` 下首个文件实际带 `_0` 后缀。
5. **保存目录默认在仓库根目录的 `log/rosbag`**：按 `bag_recorder` 包路径
   `../../../log/rosbag` 推导；现场长期运行建议用 `save_dir` 指到数据盘。
6. **`/chassis_robot/chassis_status` 按 `std_msgs/Bool` 实现**：如果底盘实际发布的是自定义消息，
   订阅端不会报错但收不到数据，需要同时修改 `perception_sensors_client_main` 的订阅类型。
7. **UWB 的重连阈值必须大于标签静态休眠间隔**：15 s 无帧只告警，30 s 才自动重连；
   标签静态休眠默认 10 s 发一包，若现场把静态间隔调到 30 s 以上，需同步调大
   `main.cpp` 的 `kLinkFailTimeoutMs`，否则会周期性做无意义的重连。断线期间不发布、
   不补 0，下游需按超时判断数据失效。
8. **`doc/` 是历史工程文档副本**：其中的 `catkin_ws/src/...` 路径指向原独立工作区，在本仓库里
   可能不存在，读取协议和排障信息可以，按路径找代码不行。
9. **终端里裸的 `ERRCODE_MSOPTIMEOUT` 来自激光雷达驱动，不是 ROS 日志**：现象是日志中间突然
   冒出一行没有 `[WARN] [时间戳]` 前缀、没有节点名的 `ERRCODE_MSOPTIMEOUT`，还常夹在超声波
   或 UWB 的输出之间，容易被误认为是它们的报错。原因是 `rs_driver` 的日志宏
   （[rs_log.hpp](catkin_ws/src/rslidar_sdk-v1.5.20/src/rs_driver/src/rs_driver/common/rs_log.hpp)）
   直接 `std::cout` 写标准输出，绕过 ROS 日志系统；多个节点的 `std::cout` 与 `rosout`
   各自刷新，行序会互相穿插。该错误码由
   [input_sock_epoll.hpp](catkin_ws/src/rslidar_sdk-v1.5.20/src/rs_driver/src/rs_driver/driver/input/unix/input_sock_epoll.hpp)
   在 `epoll_wait` 等待 MSOP 包超时 1 s 时发出，经
   [source_driver.hpp](catkin_ws/src/rslidar_sdk-v1.5.20/src/source/source_driver.hpp)
   以 WARNING 级别打印，**每超时 1 s 报一次且不限流**。零星出现（网络抖动、丢包）可忽略，
   节点不会退出；连续每秒刷说明雷达没在发包或主机收不到，按陷阱 2 检查网络与点云目的地址。
