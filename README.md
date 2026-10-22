# perception_sensors_ros_driver

面向巡检/底盘设备的感知传感器 ROS 1 Noetic 工作区：接入速腾 Airy360 激光雷达、DYP 电应普超声波雷达
和 UWB 基站，把三类数据统一发布到 `/perception_sensors/` 命名空间，并提供 rosbag 录制控制与
底盘上装电源按键联动。

## 功能说明

1. **三类传感器统一接入与状态管理**
   - 激光雷达、超声波雷达、UWB 基站各自独立节点采集，统一发布到 `/perception_sensors/` 命名空间；

     | 传感器 | 接入与管理方式 |
     | --- | --- |
     | Airy360 | 官方驱动 UDP 收包；滑动窗口多帧累加后 10 Hz 发布 |
     | 超声波雷达 | 两组 M04 网关各 4 路，0.3 s 轮询，按轮聚合成一条消息 |
     | UWB | 串口主动轮询，字节流分帧 + CRC16，收一帧发一帧 |

   - 每类设备都有 latched 的链路状态话题（激光雷达 1 个、超声波每台网关 1 个、UWB 1 个），
     只在 `true`/`false` 跳变时发布，后启动的订阅者也能立即拿到当前状态；
   - 断线自恢复：UWB 启动失败每 5 s 重试、运行中失联每 10 s 重连；超声波已上线断线按 1 s 重连，
     启动探测失败的网关由后台线程每 30 s 重探；激光雷达重连由官方驱动内置处理，预处理节点负责
     20 s 心跳判定并在超时期间清空累加窗口。详见[设备在线状态与重连参数](#设备在线状态与重连参数)。

2. **遥控器拨杆一键控制录制，存储自动循环覆盖**
   - 底盘把遥控器上装电源使能拨杆的状态发布到 `/chassis_msg`，`chassis_relay_ctrl.tops_power_enable == 1`
     自动开始录制，其余值自动停止；同一状态去重，服务失败下一帧重试；底盘消息断流超过 5 s 自动停录；
   - 录制内部有完整的存储控制：`--lz4` 压缩、单卷固定 10 GB、文件名带 6 位序号保证唯一与排序；
     单次录制默认 25 GB 上限（超限可停止或删除旧卷），保存目录默认 150 GB 高水位 / 130 GB 低水位，
     达到高水位后按最旧批次删除已关闭分卷，删到低水位停止，实现无人值守循环覆盖；
   - 录制状态与两个大小话题（当前批次、目录总量）实时可观测，`.bag.active` 永不删除。
     触发链路、目录布局、两级上限与清理规则详见[数据存储](#数据存储)。

一句话亮点：**三类传感器即插即用、断线自恢复；拨杆开关录制、磁盘占用自动封顶。**

## 系统组成

| 包 | 一句话职责 | 入口 |
| --- | --- | --- |
| [rslidar_sdk](catkin_ws/src/rslidar_sdk-v1.5.20/) | 速腾官方驱动，UDP 收包并发布原始点云 | `launch/start.launch` |
| [lidar_preprocess](catkin_ws/src/lidar_preprocess/) | 点云滑动窗口多帧累加与雷达心跳 | `lidar_preprocess.cpp` |
| [ultrasonic_radar_modbus_tcp](catkin_ws/src/ultrasonic_radar_modbus_tcp/) | Modbus TCP 网关采集两组各 4 路超声波 | `src/main.cpp` |
| [uwb_rtu_serial_demo](catkin_ws/src/uwb_rtu_serial_demo/) | Modbus-RTU 串口解析 UWB 定位帧 | `src/main.cpp` |
| [bag_recorder](catkin_ws/src/bag_recorder/) | 服务控制的 rosbag 录制/停止 | `src/main.cpp` |
| [common_msgs](catkin_ws/src/common_msgs/) | 底盘控制消息定义（`ChassisMsg` / `ChassisCtrl` / `TopMsg`） | `msg/ChassisMsg.msg` |
| [perception_sensors_client_main](catkin_ws/src/perception_sensors_client_main/) | 顶层 launch 一键启动；底盘上装电源联动录制 | `src/main.cpp` |

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
底盘 ──/chassis_msg（common_msgs/ChassisMsg）──► perception_sensors_client_main
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
- 录制：服务事件驱动；联动节点只负责把底盘上装电源使能（`tops_power_enable`）翻译成
  start/stop 服务调用。另设 5 s 断流保护：底盘停止发布 `/chassis_msg` 时自动停录，
  避免持续录制无控制键的数据。`bag_recorder` 自身拆为 `src/main.cpp`（参数、发布器、
  服务、定时器与 `ros::spin()`）和 `include/bag_recorder/bag_recorder_node.h` +
  `src/bag_recorder_node.cpp`（录制、统计、清理流水线），ROS 拓扑只在 main 中可见。

## 数据存储

三类传感器的数据不落盘到各自节点，统一由 [`bag_recorder`](catkin_ws/src/bag_recorder/)
写入 rosbag。录制默认由底盘遥控器的上装电源拨杆驱动，也可以手动调服务；
存储层设置**单次录制**与**保存目录总量**两道独立上限，后者通过高低水位清理实现无人值守循环覆盖。

### 存储链路

```text
遥控器上装电源拨杆
        │
底盘 ──/chassis_msg（tops_power_enable）──► perception_sensors_client_main
                                                    │ start / stop 服务（按目标状态去重）
                                                    ▼
                                bag_recorder_node ──拉起子进程──► rosbag record
                                                                        ▲
        3 个感知话题 ───────────────────────────────────────────────────┘
                                                   --lz4 --split ──► save_dir
```

| 触发方式 | 行为 |
| --- | --- |
| `tops_power_enable == 1` | 调 `/perception_sensors/start_bag_recorder` 开始录制 |
| 其它值（含 `0`、`-1`） | 调 `/perception_sensors/stop_bag_recorder` 停止录制 |
| 状态未跳变 | 不重复调用；服务失败时保持旧状态，下一帧消息重试 |
| 5 s 未收到 `/chassis_msg` | 正在录制则自动停录，恢复且仍为 `1` 时自动续录 |
| 手动 `rosservice call` | 直接控制录制，不依赖底盘消息 |

默认录制话题（`record_topics`）为三个感知结果话题，不含原始点云 `/rslidar_points` 与各类状态话题。

### 保存目录与文件命名

保存目录由 `save_dir` 指定；留空时按 `bag_recorder` 包路径推导为**仓库根目录的 `log/rosbag`**，
目录不存在会自动创建，建议现场用绝对路径指向数据盘。

```text
log/rosbag/
├── perception_sensors_000001_20261022_090000_0.bag      ← 第 1 次录制，第 0 卷
├── perception_sensors_000001_20261022_090000_1.bag      ← 写满 10 GB 后自动分卷
├── perception_sensors_000002_20261022_103000_0.bag
└── perception_sensors_000003_20261022_140000_0.bag.active   ← 正在写入
```

文件名组成：

| 字段 | 示例 | 说明 |
| --- | --- | --- |
| 固定前缀 | `perception_sensors_` | 目录清理只识别这个前缀的文件 |
| 6 位批次序号 | `000003` | 启动时扫描目录续号，每次录制递增；唯一性与排序主键，最大 `999999` |
| 时间戳 | `20261022_140000` | 仅用于人工识别，时钟重复或回退不影响唯一性 |
| 分卷号 | `_0`、`_1`…… | 单卷固定 10 GB，`--split` 下首卷带 `_0` |
| 后缀 | `.bag` / `.bag.active` | `.bag.active` 表示 rosbag 正在写入，停止后自动去掉 |

录制子进程固定使用 `--lz4 --split --size 10240`；bag 内数据频率等于被录话题频率，
本节点不做重采样或多帧拼接。

### 两级容量上限

| 层级 | 参数 | 默认值 | 统计范围 | 达到上限后的动作 |
| --- | --- | --- | --- | --- |
| 单次录制 | `max_bag_size_gb` | 25 GB | 当前批次现存分卷 | `stop`（默认）停止录制；`delete_old` 删除最旧已关闭分卷后继续 |
| 保存目录 | `max_dir_size_gb` / `dir_low_watermark_gb` | 150 / 130 GB | `save_dir` 下所有批次 | 超过高水位后删到低水位；`max_dir_size_gb <= 0` 关闭清理 |

两套上限**互相独立**：默认配置下单次录到 25 GB 就停，目录清理主要在多次录制累积到
150 GB 时才生效。要让录制连续运行、完全依赖目录循环覆盖，需把 `total_limit_action`
设为 `delete_old`，或调大 `max_bag_size_gb`。检查周期为 1 s，因此超限最多晚约 1 s 被发现。

### 目录级清理规则

- **高低水位状态机**：总量低于高水位时不动；一旦达到高水位进入清理，必须删到低水位才停，
  避免每秒只删一卷。
- **删除顺序**：按「批次序号 → 分卷号」从旧到新；先删历史批次，历史批次删完仍超限才删当前批次的旧分卷。
- **正在写入的分卷永不删除**：`.bag.active` 不计入删除候选；只剩它仍超限时限流告警，不再刷屏。
- **按整卷删除**：只删已关闭的完整分卷，实际总量可能比低水位再低最多一卷（10 GB）。
- **两个时机**：每次开始录制前先做一次预清理；**停止录制后不清理**，刚录完的批次至少保留到下次开录。
- 只统计和删除符合本节点命名规则的 `perception_sensors_*.bag`，目录里手动录制的其它 bag 不受影响。

### 存储可观测性

| 话题 | 单位 / 类型 | 发布时机 | 含义 |
| --- | --- | --- | --- |
| `/perception_sensors/recording_status` | `std_msgs/Bool`，latched | 仅跳变 | `true` 表示正在录制，可作素材保存互斥信号 |
| `/perception_sensors/recording_size` | GB（GiB），`Float64` | start 发 0、录制中 1 Hz、stop 后一次 | 当前批次**现存**分卷总大小；`delete_old` 后数值会下降 |
| `/perception_sensors/recording_dir_size` | GB（GiB），`Float64` | 开录前一次、录制中 1 Hz | 保存目录所有批次的总大小，是目录清理的依据 |

两个大小话题按 **1024³ 字节**换算成 GB（GiB），不是 10⁹ 字节；`recording_dir_size`
不能用 `recording_size` 推断。`recording_status` 不是周期心跳，`rostopic hz` 只反映跳变次数。

### 查看与回放

```bash
rosbag info /data/rosbag/perception_sensors_000003_20261022_140000_0.bag
rosbag play /data/rosbag/perception_sensors_000003_20261022_140000_0.bag

# 单独启动录制节点并修改保存位置 / 上限
roslaunch bag_recorder bag_recorder.launch save_dir:=/data/rosbag max_bag_size_gb:=25 \
    total_limit_action:=delete_old max_dir_size_gb:=150 dir_low_watermark_gb:=130
```

同一工作空间只应运行一个 `bag_recorder` 实例：服务名固定，多个实例还会竞争文件名序号。
逐卷规则、实现细节与完整已知限制见
[bag_recorder/README.md](catkin_ws/src/bag_recorder/README.md#已知限制)：
rosbag 子进程异常退出后节点不会自动感知（仍显示在录制）、异常退出留下的残留 `.bag.active`
需要人工处理、目录清理不可恢复等。

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
| `/perception_sensors/recording_dir_size` | `std_msgs/Float64` | 发布 | 录制中 1 Hz + 开录前一次 | 保存目录所有批次的总大小（GB，GiB），含 `.bag.active`；目录级清理的依据 |
| `/chassis_msg` | `common_msgs/ChassisMsg` | 订阅 | 由底盘决定 | `chassis_relay_ctrl.tops_power_enable == 1` 开始保存，其余值停止保存；超过 5 s 无消息自动停录；消息定义见 `catkin_ws/src/common_msgs/` |
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
- **录制目录总量**：`/perception_sensors/recording_dir_size` 统计保存目录下**所有批次**的
  `perception_sensors_*.bag`（含 `.bag.active`），是目录级清理的依据。录制期间 1 Hz 发布，
  开录前预清理后也发一次；目录清理删除文件后数值会下降。
- **目录级清理**：`max_dir_size_gb`（默认 150 GB）是目录高水位，`dir_low_watermark_gb`
  （默认 130 GB）是低水位。总量达到高水位后按「批次序号 → 分卷号」从旧到新删除已关闭分卷，
  删到低水位才停；`.bag.active` 永远不删。优先删历史批次，历史批次删完仍超限才删当前批次的
  旧分卷。开录前会先做一次预清理；停止录制后不清理。

## 频率链

| 链路 | 硬件发出/上报 | 节点采集节奏 | 话题发布 |
| --- | --- | --- | --- |
| 激光雷达 | Airy360 600 rpm，约 10 Hz | 回调每帧入队，后台线程维护累加长度 `frame_quantity` 的窗口；主循环 10 Hz 轮询 | ≈10 Hz；前 `frame_quantity` 帧（默认 10 帧 ≈1 s）不输出；每帧点数是窗口内各帧直接相加，默认约 10 倍带宽 |
| 超声波 | 设备只应答，不主动推送 | `poll_interval=0.3 s`，每轮遍历两个网关 | ≈3.3 Hz，每轮一条聚合消息 |
| UWB | 标签驱动：动态默认约 10 Hz，上限 20 Hz；静态默认 0.1 Hz（可配置到 0.05 Hz） | 30 Hz 串口轮询，每轮取完缓冲 | 等于有效定位帧频率，不是固定 30 Hz |
| 录制 | 事件驱动 | 服务 start/stop | 无周期 |
| 上装电源使能 | 由底盘发布（频率未知） | 回调内按 `tops_power_enable` 去重，仅跳变时调服务；1 Hz 看门狗检测断流，5 s 无消息自动停录 | 无周期 |

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
→ `bag_recorder`，最后启动上装电源联动节点。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gateway_ip` | `192.168.1.15` | 超声波 Modbus 网关地址 |
| `save_dir` | 空 | 空表示仓库根目录的 `log/rosbag`，目录不存在时自动创建 |
| `max_bag_size_gb` | `25.0` | 单次录制总大小上限（GB）；单卷固定 10 GB，源码硬编码 |
| `total_limit_action` | `stop` | 达到总上限时：`stop` 停止录制；`delete_old` 删除最旧分卷后继续 |
| `max_dir_size_gb` | `150.0` | 保存目录所有批次的总大小高水位（GB）；`<= 0` 关闭目录级清理 |
| `dir_low_watermark_gb` | `130.0` | 目录清理目标低水位（GB），须小于高水位；非法时回退为「高水位 − 2 个单卷」 |

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
rostopic echo /perception_sensors/recording_dir_size
```

### 打包源码

把当前提交导出成 zip 交付（不经过 `git clone`，也不含 `.git` 目录）：

```bash
cd /home/liyun/code/perception_sensors_ros_driver
git archive --format=zip --prefix=perception_sensors_ros_driver/ \
    --output=perception_sensors_ros_driver.zip HEAD
```

- 只导出 git **已跟踪**的内容：`catkin_ws/src/`、`doc/`、`README.md`、`.gitignore`。
  `catkin_ws/build`、`devel`、`log/rosbag`、`.vscode`、`CLAUDE.md` 都不会入包，
  对方解压后需要按[编译](#编译)一节重新构建。
- 打包的是提交快照：工作区里未提交的修改不会进包，要带上就先用 `git commit`。
  也可以把 `HEAD` 换成具体分支名或 tag，导出对应版本。
- `--prefix` 让解压后得到 `perception_sensors_ros_driver/` 顶层目录；不需要时去掉该项。
- `catkin_ws/src/CMakeLists.txt` 在 zip 中以符号链接条目存储，Linux 下 `unzip` 会还原成链接；
  但在 Windows 上解压时可能变成一个普通文本文件，导致 `catkin_make` 报解析错误。
  遇到这种情况在 WSL/Linux 里重建链接：
  ```bash
  ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake catkin_ws/src/CMakeLists.txt
  ```

## 已知陷阱

1. **WSL2 默认 NAT 收不到雷达 UDP**：需要 mirrored 网络，或在 Windows 侧做端口转发；
   同时确认雷达网页里的点云目的地址是 `192.168.1.100`。
2. **单卷固定 10 GB，`max_bag_size_gb=25` 是单次录制总上限**：达到上限时 `stop`（默认）停录，
   `delete_old` 按整卷删除最旧分卷后继续（占用在 15~25 GB 波动）；首个分卷文件名带 `_0` 后缀。
   它与 `max_dir_size_gb`（目录所有批次的总上限）是两套独立限制，互不替代。
3. **保存目录默认在仓库根目录的 `log/rosbag`**：按 `bag_recorder` 包路径
   `../../../log/rosbag` 推导；现场长期运行建议用 `save_dir` 指到数据盘。
4. **录制开关取自 `/chassis_msg` 的 `chassis_relay_ctrl.tops_power_enable`**：只有 `1` 算打开，
   其余值按关闭处理；消息类型 `common_msgs/ChassisMsg` 定义在独立包 `catkin_ws/src/common_msgs/`。
   正在录制时超过 5 s 没收到 `/chassis_msg` 会自动停录，恢复且仍为 `1` 时自动续录。
5. **UWB 的重连阈值必须大于标签静态休眠间隔**：15 s 无帧只告警，30 s 才自动重连；
   标签静态休眠默认 10 s 发一包，若现场把静态间隔调到 30 s 以上，需同步调大
   `main.cpp` 的 `kLinkFailTimeoutMs`，否则会周期性做无意义的重连。断线期间不发布、
   不补 0，下游需按超时判断数据失效。
6. **`doc/` 是历史工程文档副本**：其中的 `catkin_ws/src/...` 路径指向原独立工作区，在本仓库里
   可能不存在，读取协议和排障信息可以，按路径找代码不行。文档里的旧话题名
   （`/lidar/processed_cloud`、`/lidar/connection_status`、`/uwb_position` 等）同样已过时，
   以代码和本 README 的约定为准。
7. **裸的 `ERRCODE_MSOPTIMEOUT` 来自激光雷达驱动**：`rs_driver` 用 `std::cout` 直接打印、
   绕过 ROS 日志，所以没有 `[WARN]` 前缀和节点名。它是 MSOP 收包超过 1 s 的提示，零星出现可忽略；
   连续每秒刷说明雷达没在发包或主机收不到，按陷阱 1 检查网络与点云目的地址。
8. **目录级清理只删已关闭分卷，且停止录制后不清理**：`.bag.active` 永远保留，所以当历史批次
   全删完仍超限时，会继续删当前批次的旧分卷（保留正在写入的那一卷）；只剩 `.active` 时无法
   再降，节点按 1 s 限流告警。停止录制后不做清理，刚录完的批次至少保留到下一次开始录制。
   如果现场要求连续录制并完全依赖目录循环覆盖，需把 `total_limit_action` 改为 `delete_old`
   或调大 `max_bag_size_gb`，否则默认的「单次 25 GB + stop」会先把录制停掉。
