# lidar_preprocess

订阅速腾 Airy360 经 `rslidar_sdk` 发布的 `sensor_msgs/PointCloud2`，用滑动窗口做多帧点云累加，
同时监测雷达心跳，输出处理后的点云和连接状态。

本包不修改上游 `rslidar_sdk`，只消费它发布的 `/rslidar_points`。
设备、网络和驱动配置见[仓库 README](../../../README.md)。

## 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 订阅 | 驱动原始点云，约 10 Hz；可通过参数覆盖 |
| `/perception_sensors/processed_lidar_cloud` | `sensor_msgs/PointCloud2` | 发布 | 滑动窗口累加结果 |
| `/perception_sensors/processed_lidar_connection_status` | `std_msgs/Bool` | 发布 | `true` 在线 / `false` 超时；latched，仅在状态变化时发布 |

### 输入点云语义

默认 `POINT_TYPE=XYZI`，字段为 `x` / `y` / `z` / `intensity`，全部 float32，`point_step = 16`，
`frame_id = rslidar`。驱动配置 `dense_points=false`，因此点云中可能包含 NaN 点。

### 输出点云语义

- 类型：`sensor_msgs/PointCloud2`，由 PCL `PointXYZI` 通过 `pcl::toROSMsg` 转换；
- 字段：`x` / `y` / `z` / `intensity`，全部 float32，`point_step = 16`；
- `frame_id`：`rslidar`（与驱动一致）；
- 内容是最近 `frame_quantity` 帧原始点的**直接拼接**，未做去重、降采样、运动补偿；
- 时间戳：见下文“时间戳对齐”。

## 默认行为

- 启动后订阅输入话题，把每帧转成 `pcl::PointCloud<PointXYZI>` 放入原始队列；
- 后台线程维护长度为 `frame_quantity`（默认 10）的滑动窗口：
  - 窗口未满时不发布任何结果；
  - 窗口满后，每来一帧就丢掉最旧一帧、加入最新一帧，输出整窗累加结果；
  - 因此输出频率约等于输入频率（10 Hz），但每帧点数约为单帧的 10 倍；
- 心跳：每 1 s 检查一次最近收帧时间，超过 **20 s（代码写死）** 判为断连；
  - 断连时清空原始队列、窗口和处理结果缓存；
  - 恢复后重新积累满一个窗口再发布；
  - **重连由上游 `rslidar_sdk` 内置处理，本包不实现重连逻辑**；本包只负责判定与清缓冲；
- 主循环频率 10 Hz，每次循环 `spinOnce` 并通过 `getProcessedCloud` 取结果，有新结果才发布。

从雷达到 `/perception_sensors/processed_lidar_cloud` 的完整频率传递（含每帧数据量放大）见
[仓库 README 的“数据链路与频率传递”](../../../README.md#6-数据链路与频率传递)。

## 参数

节点使用私有句柄 `~`，参数位于 `/lidar_preprocess_node/` 命名空间。

| launch 参数 | 节点参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `lidar_topic` | `~input_topic` | `/rslidar_points` | 输入点云话题 |
| `frame_quantity` | `~frame_quantity` | `10` | 滑动窗口帧数 |
| `timeout_sec` | — | `5.0` | **未生效**：节点内写死 20 s |

启动时覆盖：

```bash
roslaunch lidar_preprocess lidar_preprocess.launch lidar_topic:=/rslidar_points frame_quantity:=5
```

代码内写死、不支持 launch 修改的项（[lidar_preprocess.cpp](lidar_preprocess.cpp)）：

- 输出话题 `/perception_sensors/processed_lidar_cloud`、`/perception_sensors/processed_lidar_connection_status`；
- 输出 `frame_id = rslidar`；
- 心跳检查周期 1 s、超时 20 s；
- 主循环频率 10 Hz。

## 时间戳对齐

驱动配置 `use_lidar_clock=true`，输入点云时间戳是雷达时钟。节点在拿到**第一帧**处理结果时计算：

```text
time_offset = ros::Time::now() - result.timestamp
output.stamp = result.timestamp + time_offset
```

把雷达时钟映射到 ROS 墙钟。该偏移只计算一次，后续不再更新，雷达时钟与系统时钟的漂移不会被跟踪。

## 编译与运行

```bash
cd /home/liyun/code/devices/airy360_ws/catkin_ws
catkin_make -j8
source devel/setup.bash
roslaunch lidar_preprocess lidar_preprocess.launch
```

该 launch 会先 include `rslidar_sdk` 的 `start.launch` 启动驱动，再启动本节点。

验证：

```bash
rostopic hz /rslidar_points
rostopic hz /perception_sensors/processed_lidar_cloud
rostopic echo -n1 /perception_sensors/processed_lidar_connection_status
```

### 可视化

本包自带一份 rviz 配置，`Fixed Frame` 已设为 `rslidar`，PointCloud2 直接订阅 `/perception_sensors/processed_lidar_cloud`：

```bash
rosrun rviz rviz -d $(rospack find lidar_preprocess)/rviz/lidar_preprocess.rviz
```

- 话题内容本身就是 `frame_quantity` 帧的累加结果，**不需要**再靠 rviz 的 `Decay Time` 叠加残影；
- 点数约为单帧的 10 倍，渲染较吃 CPU/GPU，卡顿时可减小 `frame_quantity` 或点大小；
- 工程没有 TF，`Fixed Frame` 必须与消息 `frame_id` 一致（`rslidar`），否则点云不显示。

## 实现说明

| 文件 | 职责 |
| --- | --- |
| [lidar_preprocess.cpp](lidar_preprocess.cpp) | `main`：参数、订阅话题、发布器、10 Hz 主循环 |
| [include/lidar_sub.h](include/lidar_sub.h) | `zl_lidar::LidarSub` 类声明，职责与接口注释 |
| [src/lidar_sub.cpp](src/lidar_sub.cpp) | 订阅回调、后台累加线程、心跳定时器 |
| [launch/lidar_preprocess.launch](launch/lidar_preprocess.launch) | 启动驱动与节点，暴露 `lidar_topic`、`frame_quantity` |
| [rviz/lidar_preprocess.rviz](rviz/lidar_preprocess.rviz) | 可视化配置，订阅 `/perception_sensors/processed_lidar_cloud` |
| [CMakeLists.txt](CMakeLists.txt) | 可执行目标 `lidar_preprocess_node` |

线程模型：

```text
ROS 回调线程 ──转换 PointCloud2→PCL──► raw_cloud_queue_
                                            │
                                     condition_variable
                                            ▼
Worker 线程 ──维护滑动窗口──► 累加 ──► latest_result_
                                            │
                                     10 Hz 主循环轮询
                                            ▼
                         /perception_sensors/processed_lidar_cloud
```

- 订阅器和发布器由 `main` 创建，类只持有订阅器和定时器；
- 队列、窗口、结果缓存分别用互斥锁保护，跨线程唤醒用 `condition_variable`；
- 断连清空逻辑在定时器回调中执行，并通知 Worker 重置窗口。

## 已知限制

| 位置 | 问题 | 影响 |
| --- | --- | --- |
| `lidar_preprocess.cpp` | `timeout_sec` 没有从参数服务器读取，写死 20 s | launch 调整超时无效 |
| `lidar_preprocess.cpp` | 时间偏移只计算一次 | 长时间运行后输出时间戳可能缓慢漂移 |
| `src/lidar_sub.cpp` | 多帧直接逐点相加，无运动补偿/去重 | 运动物体出现拖影，点数和内存随窗口线性增长 |
| `src/lidar_sub.cpp` | TODO 注释：IMU 运动补偿与滤波尚未实现 | 当前只是累加 |
| `lidar_preprocess.cpp` | 连接状态只在**变化时**发布，且初始状态为 `false` | 从未收到数据时不会有状态消息 |
| 输出话题 | 由历史命名 `/livox/...` 改为 `/lidar/...` | 下游订阅需同步修改 |

## 上游历史

本包最初按 Livox 驱动设计（类名 `LivoxSub`、话题 `/livox/processed_cloud`）。切换到 Airy360 后：

- 输入源改为 `/rslidar_points`，launch 不再启动 Livox 驱动；
- 类名、文件名、可执行名统一改为 `LidarSub` / `lidar_sub.*` / `lidar_preprocess_node`；
- 输出话题改为 `/lidar/processed_cloud`、`/lidar/connection_status`（当前已统一到 `/perception_sensors/*`）；
- 输出 `frame_id` 改为 `rslidar`。
