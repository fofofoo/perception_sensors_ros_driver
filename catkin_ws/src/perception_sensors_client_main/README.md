# perception_sensors_client_main

感知传感器一键启动与上装电源录制联动节点。

## 职责

1. 提供顶层 launch，一次性拉起激光雷达、超声波、UWB、bag_recorder 和本节点；
2. 订阅底盘控制消息 `/chassis_msg`（`common_msgs/ChassisMsg`）；
3. 读取 `chassis_relay_ctrl.tops_power_enable`（上装电源使能）作为录制开关：
   值为 `1` 时调用 `/perception_sensors/start_bag_recorder`，为 `0` 或其它异常值时调用
   `/perception_sensors/stop_bag_recorder`；
4. 对目标状态去重，底盘周期重复发布时不会反复调用服务；start 失败会在下一帧自动重试；
   stop 失败也会记为「已停止」，并在后续 0 帧补调，保证拨杆关再开一定能重新开录。

本节点不发布任何话题，也没有主循环；以订阅回调驱动为主，另有一个 1 Hz 看门狗定时器
负责 `/chassis_msg` 断流检测（超过 5 s 无消息自动停录）。

## 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/chassis_msg` | `common_msgs/ChassisMsg` | 订阅 | `chassis_relay_ctrl.tops_power_enable == 1` 开始保存，其余值停止保存 |
| `/perception_sensors/start_bag_recorder` | `std_srvs/Trigger` | 调用 | 由 `bag_recorder_node` 提供 |
| `/perception_sensors/stop_bag_recorder` | `std_srvs/Trigger` | 调用 | 由 `bag_recorder_node` 提供 |

## 文件

| 文件 | 作用 |
| --- | --- |
| `src/main.cpp` | 订阅 `/chassis_msg`、状态去重、服务调用 |
| `launch/perception_sensors_client_main.launch` | 一键启动全部节点 |
| `CMakeLists.txt` / `package.xml` | 包配置（依赖 `common_msgs`） |

## 编译与运行

```bash
cd /home/liyun/code/perception_sensors_ros_driver/catkin_ws
catkin_make
source devel/setup.bash

# 一键启动（参数按需覆盖）
roslaunch perception_sensors_client_main perception_sensors_client_main.launch
roslaunch perception_sensors_client_main perception_sensors_client_main.launch \
    gateway_ip:=192.168.1.12 save_dir:=/data/rosbag max_bag_size_gb:=25 total_limit_action:=delete_old
```

`save_dir` 留空时使用仓库根目录的 `log/rosbag`。UWB 串口名在
`uwb_rtu_serial_demo/launch/uwb_node.launch` 中配置。

录制参数原样透传给 `bag_recorder`：

- `max_bag_size_gb` / `total_limit_action`：单次录制的总上限与超限动作；
- `max_dir_size_gb` / `dir_low_watermark_gb`：保存目录所有批次的高/低水位（默认 150/130 GB），
  达到高水位后按最旧批次删除已关闭分卷，删到低水位停止；`.bag.active` 永不删除；
- `max_dir_size_gb <= 0` 关闭目录级清理，仅发布目录总量。

## 行为说明

- 第一帧消息无论上装电源是开还是关都会执行一次服务调用，用于让录制状态与底盘同步。
- 判据只认 `tops_power_enable == 1`；`0`、`-1` 等异常值都按关闭处理，避免无效状态误启录制。
- **断流保护**：1 Hz 看门狗检测 `/chassis_msg`，若正在录制且超过 5 s 没收到消息，
  自动调用 stop 服务停录。停录失败会在下一秒重试；底盘恢复发布且 `tops_power_enable` 仍为 1 时，
  会重新开始录制。节点启动后从未收到消息时不触发。
- **`bag_recorder` 自行停录（超限）后的行为**：单次录制达到 `max_bag_size_gb` 时由
  `bag_recorder` 内部停止，本节点收不到通知。上装电源保持 `1` 时不会自动重开，必须把拨杆
  关掉再打开；拨杆关闭那一次即使 stop 服务调用失败（`bag_recorder` 正阻塞落盘或重启），
  也会把本地状态记为「已停止」，拨回 `1` 时一定重新发出 start，停止本身在后续 0 帧上补调。
- 服务未注册时只打印 2 秒节流的告警，不阻塞节点；后续消息会继续重试。
- `bag_recorder` 返回 `Already recording!` / `Not recording!` 时视为目标状态已达成，
  不重复调用；其它失败原因（如保存目录不可用）会原样打印并等下一帧重试。
- 消息类型由 `common_msgs` 包（与本包同级）定义；底盘实际发布的话题名或消息类型不同时，
  需同步修改 `src/main.cpp` 的订阅模板参数与回调参数类型。

## 依赖的消息包

`common_msgs` 定义 `ChassisMsg` / `ChassisCtrl` / `TopMsg`，位于
`catkin_ws/src/common_msgs/msg/`。`ChassisMsg.msg` 以 `common_msgs/ChassisCtrl`、
`common_msgs/TopMsg` 内嵌另两个消息，因此包名必须固定为 `common_msgs`，
不能把这些 msg 直接放进本包的 `msg/` 目录（否则生成类型会变成
`perception_sensors_client_main/ChassisMsg`，与底盘发布端类型不匹配）。
