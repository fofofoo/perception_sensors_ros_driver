# perception_sensors_client_main

感知传感器一键启动与保存按键联动节点。

## 职责

1. 提供顶层 launch，一次性拉起激光雷达、超声波、UWB、bag_recorder 和本节点；
2. 订阅底盘保存按键状态 `/chassis_robot/chassis_status`（`std_msgs/Bool`）；
3. 状态为 `true` 时调用 `/perception_sensors/start_bag_recorder`，为 `false` 时调用
   `/perception_sensors/stop_bag_recorder`；
4. 对状态去重，底盘周期重复发布时不会反复调用服务；服务失败会在下一帧自动重试。

本节点不发布任何话题，也没有主循环，完全由订阅回调驱动。

## 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/chassis_robot/chassis_status` | `std_msgs/Bool` | 订阅 | `true`=开始保存，`false`=停止保存 |
| `/perception_sensors/start_bag_recorder` | `std_srvs/Trigger` | 调用 | 由 `bag_recorder_node` 提供 |
| `/perception_sensors/stop_bag_recorder` | `std_srvs/Trigger` | 调用 | 由 `bag_recorder_node` 提供 |

## 文件

| 文件 | 作用 |
| --- | --- |
| `src/main.cpp` | 订阅、状态去重、服务调用 |
| `launch/perception_sensors_client_main.launch` | 一键启动全部节点 |
| `CMakeLists.txt` / `package.xml` | 包配置 |

## 编译与运行

```bash
cd /home/liyun/code/perception_sensors_ros_driver/catkin_ws
catkin_make
source devel/setup.bash

# 一键启动（参数按需覆盖）
roslaunch perception_sensors_client_main perception_sensors_client_main.launch
roslaunch perception_sensors_client_main perception_sensors_client_main.launch \
    gateway_ip:=192.168.1.15 save_dir:=/data/rosbag max_bag_size_gb:=25 total_limit_action:=delete_old
```

`save_dir` 留空时使用仓库根目录的 `log/rosbag`。UWB 串口名在
`uwb_rtu_serial_demo/launch/uwb_node.launch` 中配置。

## 行为说明

- 第一帧按键消息无论 `true`/`false` 都会执行一次服务调用，用于让录制状态与底盘同步。
- 服务未注册时只打印 2 秒节流的告警，不阻塞节点；后续消息会继续重试。
- `bag_recorder` 返回 `Already recording!` / `Not recording!` 时视为目标状态已达成，
  不重复调用；其它失败原因（如保存目录不可用）会原样打印并等下一帧重试。
- 若底盘实际发布的不是 `std_msgs/Bool`，需要修改 `src/main.cpp` 中的订阅模板参数和
  回调参数类型。
