# Airy360 激光雷达 ROS 接入与点云预处理

速腾聚创 **Airy360** 激光雷达，经 **以太网 UDP** 接入，底层用 `rs_driver`，ROS 封装用 `rslidar_sdk` 发布
`sensor_msgs/PointCloud2`。本工作区在官方驱动之上增加 `lidar_preprocess` 节点，做多帧点云累加和雷达心跳监测。

- 运行环境：Ubuntu 20.04 + ROS Noetic（当前开发环境为 WSL2）
- 驱动版本：`rslidar_sdk` 1.5.20（压缩包内自带 `rs_driver`）
- 本工作区**不修改** `rslidar_sdk` 的驱动逻辑，只改了它的 launch（不再自动启动 rviz）

## 目录结构

```text
airy360_ws/
├── catkin_ws/                         catkin 工作区
│   └── src/
│       ├── rslidar_sdk-v1.5.20/       速腾官方 ROS 驱动（内含 src/rs_driver）
│       └── lidar_preprocess/          点云预处理节点，见其 README
├── rs_driver-main/                    单独下载的 rs_driver 源码与构建目录（历史步骤）
└── README.md
```

> `rs_driver-main` 是按官方步骤单独编译安装的版本；当前 `rslidar_sdk` 编译时会通过
> `add_subdirectory(src/rs_driver)` 构建自带的 `src/rs_driver`，工作区不依赖顶层的
> `rs_driver-main`。保留它只作参考，不要把它再拷进 `catkin_ws/src` 造成重复包。

## 1. 物理连接与网络

```text
      Airy360 雷达                          Ubuntu / WSL2 主机
    ┌──────────────┐     UDP MSOP 6699      ┌────────────────────┐
    │ 192.168.1.68 │ ─────────────────────► │ 网卡 192.168.1.100 │
    │              │     UDP DIFOP 7788     │                    │
    └──────────────┘ ─────────────────────► └─────────┬──────────┘
                                                      │
                                            rslidar_sdk_node
                                                      │ /rslidar_points
                                                      ▼
                                            lidar_preprocess_node
                                                      │ /lidar/processed_cloud
                                                      ▼
                                                  下游节点
```

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 雷达 IP | `192.168.1.68` | 未知时用 `npcap` / 抓包工具搜索，见 `airy360-test-python` 或用户手册 |
| 点云目的地址 | `192.168.1.100` | **必须在雷达网页配置正确**，否则收不到点云 |
| MSOP 端口 | `6699` | 点云数据 |
| DIFOP 端口 | `7788` | 设备信息 |
| 主机地址 | `192.168.1.100` | 主机网卡必须配置该地址（或做端口/地址转发） |

WSL2 注意：默认 NAT 模式下主机收不到雷达发往 `192.168.1.100` 的 UDP 包。需要把 WSL 配成
**镜像网络（mirrored）**，或在 Windows 侧做端口转发并保证目的地址可达；排查时可在 Windows
用 Wireshark / npcap 确认是否收到 6699 端口的数据。

## 2. 软件组成

| 组件 | 版本 / 位置 | 作用 |
| --- | --- | --- |
| `rs_driver` | `catkin_ws/src/rslidar_sdk-v1.5.20/src/rs_driver` | 底层解码库，MSOP/DIFOP 解析 |
| `rslidar_sdk` | `catkin_ws/src/rslidar_sdk-v1.5.20`（1.5.20） | ROS 节点，发布点云/IMU/原始包 |
| `lidar_preprocess` | `catkin_ws/src/lidar_preprocess` | 多帧累加 + 心跳监测，见包内 README |

安装依赖（首次编译前）：

```bash
sudo apt-get install -y libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```

> 下载 `rslidar_sdk` 时必须用 GitHub 的 **Release 压缩包**。GitHub 的 “Source code” 包不含
> 子模块 `rs_driver`，编译会失败。

## 3. 驱动配置

配置文件：[config/config.yaml](catkin_ws/src/rslidar_sdk-v1.5.20/config/config.yaml)。
`start.launch` 里 `config_path` 为空，表示使用包内这份配置。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `msg_source` | `1` | 从在线雷达收包 |
| `send_point_cloud_ros` | `true` | 发布点云话题 |
| `lidar_type` | `RSAIRY` | **必须是 `RSAIRY`**，Airy 系列的标准类型名 |
| `msop_port` / `difop_port` | `6699` / `7788` | 与雷达一致 |
| `min_distance` / `max_distance` | `0.2` / `200` | 单位 m |
| `use_lidar_clock` | `true` | 时间戳取雷达时钟 |
| `dense_points` | `false` | 保留 NaN 点，不丢弃 |
| `ros_frame_id` | `rslidar` | 点云坐标系名 |
| `ros_send_point_cloud_topic` | `/rslidar_points` | 输出点云话题 |
| `ros_queue_length` | `100` | 发布队列深度 |

若雷达型号保持默认的 `RSM1`，启动会报：

```text
ERRCODE_WRONGMSOPLEN   MSOP 包长度错误
ERRCODE_WRONGDIFOPLEN  DIFOP 包长度错误
```

把 `lidar_type` 改为 `RSAIRY` 即可。

## 4. 编译与运行

```bash
cd /home/liyun/code/devices/airy360_ws/catkin_ws
catkin_make -j8
source devel/setup.bash
roslaunch lidar_preprocess lidar_preprocess.launch
```

该 launch 会依次启动：

1. `rslidar_sdk_node`：连接雷达，发布 `/rslidar_points`；
2. `lidar_preprocess_node`：订阅点云，滑动窗口累加后发布 `/lidar/processed_cloud`。

`start.launch` 默认**不启动 rviz**。需要可视化时另开终端，按要看的点云选择配置：

```bash
# 驱动原始点云 /rslidar_points
rosrun rviz rviz -d $(rospack find rslidar_sdk)/rviz/rviz.rviz

# 累加结果 /lidar/processed_cloud（Fixed Frame 已配为 rslidar）
rosrun rviz rviz -d $(rospack find lidar_preprocess)/rviz/lidar_preprocess.rviz
```

`rslidar_sdk` 的配置也可以只启动后手动把 PointCloud2 话题改成 `/lidar/processed_cloud`；
`lidar_preprocess` 的配置细节与注意事项见[功能包 README](catkin_ws/src/lidar_preprocess/README.md#可视化)。

验证：

```bash
rostopic hz /rslidar_points          # 应有约 10 Hz
rostopic hz /lidar/processed_cloud   # 窗口累积满 10 帧后开始输出
rostopic echo -n1 /lidar/connection_status
```

## 5. 话题与数据格式

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 驱动原始点云，约 10 Hz |
| `/lidar/processed_cloud` | `sensor_msgs/PointCloud2` | 预处理结果，每帧约 10 帧原始点累加 |
| `/lidar/connection_status` | `std_msgs/Bool` | `true` 在线，`false` 超时；latched |

默认 `POINT_TYPE=XYZI`，驱动发布的 PointCloud2 字段：

| 字段 | 类型 | 偏移 |
| --- | --- | --- |
| `x` / `y` / `z` | float32 | 0 / 4 / 8 |
| `intensity` | float32 | 12 |

`point_step = 16`，`frame_id = rslidar`。

> 注意：`rslidar_sdk` 内部点结构的 `intensity` 是 `uint8_t`，但发布成 ROS 消息时会扩成
> `float32`。字段在编译期由 `POINT_TYPE` 决定，可选 `XYZI` / `XYZIF` / `XYZIRT` / `XYZIRTF`，
> 详见 [05_how_to_change_point_type_CN.md](catkin_ws/src/rslidar_sdk-v1.5.20/doc/howto/05_how_to_change_point_type_CN.md)。

与 Livox 驱动的 18 字节格式（`x,y,z,intensity` + `tag` + `line`）相比，速腾 `XYZI` 少了末尾
`tag`/`line` 两个字节。使用 `pcl::fromROSMsg` 按字段读取的下游不受影响；按 `point_step == 18`
硬编码偏移的下游不兼容。

## 6. 数据链路与频率传递

从雷达到下游，每一环的频率都由上一环驱动；工作区里没有任何“固定频率发布”的定时器。

```text
Airy360 雷达硬件
  电机 600 rpm → 10 转/s → 10 帧/s（每转 1 帧，约 856,320 pts/s）
        │ UDP MSOP 6699（点云）/ DIFOP 7788（设备信息）
        ▼
rs_driver 解码器（内嵌在 rslidar_sdk 中）
  DIFOP.rpm → rps = rpm / 60 = 10
  默认按 0° 水平角分帧 → 每转输出 1 帧（split_frame_mode=SPLIT_BY_ANGLE）
  每切出一帧回调一次 → 点云交给上层
        ▼
rslidar_sdk 发布
  没有定时器、没有发布频率参数；解码出一帧就 publish 一次
  /rslidar_points ≈ 10 Hz
        ▼
lidar_preprocess 订阅回调（输入每来 1 帧触发 1 次）
  PointCloud2 → pcl::PointCloud<PointXYZI> → raw_cloud_queue_
        ▼
Worker 线程（每输入 1 帧处理 1 次）
  窗口未满：只入窗，不产出
  满 10 帧后：丢最旧 + 加最新 → 产出 1 个累加结果
  结果产出频率 = 输入频率 = 10 Hz
        ▼
main 线程（ros::Rate 10 Hz）
  每 100 ms 轮询一次；有结果才发布
  /lidar/processed_cloud ≈ 10 Hz
```

| 环节 | 频率由什么决定 | 默认频率 | 说明 |
| --- | --- | --- | --- |
| 雷达出帧 | 电机转速 ÷ 60 | `600 rpm` → **10 Hz** | 硬件设置，驱动不控制 |
| `/rslidar_points` | 每解码出一帧发布一次 | **≈ 10 Hz** | 与雷达帧率一致，无节流 |
| 累加结果产出 | 每个输入帧更新一次窗口 | **≈ 10 Hz** | 窗口满 10 帧后才开始产出 |
| `/lidar/processed_cloud` | main 10 Hz 轮询 + 有新结果 | **≈ 10 Hz** | 与输入同频，存在相位抖动 |
| `/lidar/connection_status` | 状态变化事件 | 非周期 | 1 s 检查、20 s 超时，latched |

三个频率边界：

1. **启动填充**：前 `frame_quantity` 帧（默认 10 帧，约 1 s）不输出，这是正常现象。
2. **轮询上限**：main 固定 10 Hz。若将来把雷达转速改为 `1200 rpm`（20 Hz），两次轮询之间的
   中间结果会被新结果覆盖，输出仍约 10 Hz；届时需同步提高
   [ros::Rate](catkin_ws/src/lidar_preprocess/lidar_preprocess.cpp#L32)。当前 600 rpm 不需要改。
3. **数据量放大**：每帧输出是窗口内 10 帧的**重复拼接**（相邻两帧重叠 9 帧），按手册输入
   856,320 pts/s、每点 16 B 估算，输出约 86 万点/帧、10 帧/秒，约 137 MB/s，是输入数据量的
   10 倍。下游节点和网络需要能承受这个带宽，否则发布队列会积压。

## 7. 与官方默认配置的差异

| 文件 | 改动 |
| --- | --- |
| `rslidar_sdk-v1.5.20/launch/start.launch` | 注释 rviz 节点，启动只发话题 |
| `config/config.yaml` | `lidar_type` 改为 `RSAIRY` |
| `lidar_preprocess` | 输入源由 Livox 改为 `/rslidar_points`，详见包 README |

`rslidar_sdk` 的驱动、解码、点云格式均未修改。

## 8. 排障记录

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| 启动报 `ERRCODE_WRONGMSOPLEN` / `ERRCODE_WRONGDIFOPLEN` | `lidar_type` 不是 `RSAIRY` | 修改 `config/config.yaml` |
| `roslaunch` 后没有 rviz 窗口 | 已按需求注释 rviz | 手动 `rosrun rviz ...` |
| 没有 `/rslidar_points` | 雷达点云目的地址不是 `192.168.1.100`；主机没有该 IP；WSL NAT 拦截 UDP | 检查雷达网页配置、主机网卡地址、WSL 网络模式 |
| 有 `/rslidar_points` 但无 `/lidar/processed_cloud` | 滑动窗口未满（默认 10 帧）；输入话题参数不对 | 等约 1 s；`rosparam get /lidar_preprocess_node/input_topic` 检查 |
| `/lidar/connection_status` 一直为 `false` | 20 s 内没收到数据 | 先解决点云接入问题 |
| rviz 报 `No transform from [rslidar]` | 没有 TF | 添加 `rslidar` 到 `map`/`base_link` 的静态变换，或把 Fixed Frame 设为 `rslidar` |
| WSL 下收不到任何 UDP | WSL2 默认 NAT | 切换 mirrored 网络或做 Windows 端口转发 |

## 9. 当前状态与遗留问题

- 2026-10-20：工作区已完成首次 `catkin_make`，`rslidar_sdk_node`、`lidar_preprocess_node` 均编译通过；
  **尚未接雷达做联调**。
- `lidar_preprocess` 的 `timeout_sec` 是 launch 参数，但节点内部写死 20 s，该参数暂不生效。
- 预处理只做多帧累加，未做运动补偿、去重或滤波（源码中留有 TODO）。
- 顶层的 `rs_driver-main` 与工作区内嵌的 `rs_driver` 是两份代码，当前只使用后者。

## 参考文档

- [rslidar_sdk 中文 README](catkin_ws/src/rslidar_sdk-v1.5.20/README_CN.md)
- [rs_driver 中文 README](rs_driver-main/README_CN.md)
- [点类型修改说明](catkin_ws/src/rslidar_sdk-v1.5.20/doc/howto/05_how_to_change_point_type_CN.md)
- [参数说明](catkin_ws/src/rslidar_sdk-v1.5.20/doc/intro/02_parameter_intro_CN.md)
- [lidar_preprocess 功能包 README](catkin_ws/src/lidar_preprocess/README.md)
