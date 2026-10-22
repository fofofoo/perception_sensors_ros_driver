# bag_recorder

服务控制的 rosbag 录制节点：通过两个 `std_srvs/Trigger` 服务启停 `rosbag record`，
自动生成唯一、可排序的 bag 文件名，并限制单次录制大小与整个保存目录的总占用。
本节点不连接任何硬件、不实现通讯协议，录制开关通常由
[perception_sensors_client_main](../perception_sensors_client_main/) 根据底盘上装电源使能转发。

## 组成

```text
/start_bag_recorder ─┐
                     ├─► bag_recorder_node ──fork/execvp──► rosbag record ──lz4/split──► save_dir
/stop_bag_recorder ──┘          │
                                ├─ /perception_sensors/recording_status      Bool, latched
                                ├─ /perception_sensors/recording_size        Float64, 1 Hz
                                └─ /perception_sensors/recording_dir_size    Float64, 1 Hz
```

| 文件 | 职责 |
| --- | --- |
| [src/main.cpp](src/main.cpp) | 读取 launch 参数；创建 3 个发布器、注册 2 个服务、创建 1 s 定时器；`ros::spin()` |
| [include/bag_recorder/bag_recorder_node.h](include/bag_recorder/bag_recorder_node.h) | `BagRecorderConfig` 与 `BagRecorderNode` 声明、职责边界与线程模型说明 |
| [src/bag_recorder_node.cpp](src/bag_recorder_node.cpp) | 录制业务：fork/停止 rosbag、分卷统计、目录级清理、文件名分配 |
| [launch/bag_recorder.launch](launch/bag_recorder.launch) | 参数与节点启动 |
| [CMakeLists.txt](CMakeLists.txt) / [package.xml](package.xml) | 包配置 |

## 服务与话题

### 服务

| 服务 | 类型 | 行为 |
| --- | --- | --- |
| `/perception_sensors/start_bag_recorder` | `std_srvs/Trigger` | 开始录制；已在录制时返回 `success=false`、`Already recording!` |
| `/perception_sensors/stop_bag_recorder` | `std_srvs/Trigger` | 停止录制；未录制时返回 `success=false`、`Not recording!` |

服务回调本身始终返回 `true`，业务结果放在 `response.success` 中。start 成功时
`message` 为 `Started. Path: <首个分卷路径> | PID: <rosbag PID>`；stop 成功时为
`Recording stopped gracefully.`。stop 会向 rosbag 发 `SIGINT` 并 `waitpid`，
等 bag 索引落盘完成后才发布停止状态。

### 话题

| 话题 | 类型 | 方向 | 发布时机 | 说明 |
| --- | --- | --- | --- | --- |
| `/perception_sensors/recording_status` | `std_msgs/Bool`（latched） | 发布 | start 发 `true`，stop 发 `false`，节点析构时若在录制再发 `false` | 素材保存互斥信号；**不是周期心跳** |
| `/perception_sensors/recording_size` | `std_msgs/Float64` | 发布 | start 发 0；录制中 1 Hz；stop 后发一次最终值 | 当前批次现存分卷总大小 |
| `/perception_sensors/recording_dir_size` | `std_msgs/Float64` | 发布 | 开录前预清理后一次；录制中 1 Hz | 保存目录所有批次总大小 |

**数据语义**：

- 单位是 **GB 但按 GiB 计算**（1024³ 字节），不是 10⁹ 字节；
- `recording_size` 统计当前批次**现存**分卷（含正在写入的 `.bag.active`），
  是磁盘占用而不是本次累计写入量；`delete_old` 删除旧卷后数值会下降；
- `recording_dir_size` 只统计符合本节点命名规则的 `perception_sensors_*.bag`，
  目录里手动录制的其它 bag 既不统计也不删除；
- `.bag.active` 计入大小，但任何清理都不会删除它；
- `recording_status` 节点启动后、首次 start 前**没有消息**（构造函数不发布），
  停止后保持 `false` 直到下次 start；后启动的订阅者靠 latched 拿当前值。
  `rostopic hz` 只反映状态跳变次数，不能用来判断是否正在录制。

## 默认行为

### 录制命令与文件命名

子进程执行：

```bash
rosbag record --lz4 --split --size 10240 -O <save_dir>/perception_sensors_<序号>_<时间戳> <话题...>
```

- 单卷固定 **10 GB**（`BagRecorderConfig::kSplitSizeMb = 10 * 1024`，源码硬编码）；
- `--lz4` 压缩；`--split` 开启分卷，首个分卷实际文件名带 `_0` 后缀；
- 文件名规则：`perception_sensors_<6 位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag`；
- 序号是唯一性与排序主键：启动时扫描保存目录恢复最大序号，每次录制递增；
  时间戳只用于人工识别，工控机时钟重复或回退也不影响唯一性；
- 分配文件名时会同时检查裸 `<前缀>.bag` 与首个分卷 `<前缀>_0.bag`，避免 rosbag 的
  `-O` 覆盖已有文件。

### 单次录制上限

- 默认 `max_bag_size_gb = 25.0`，达到后按 `total_limit_action` 处理：
  - `stop`（默认）：停止录制；
  - `delete_old`：按整卷删除最旧的**已关闭**分卷后继续，占用在约 15~25 GB 波动；
- 正在写入的 `.bag.active` 永远不删，所以首卷写入期间 `delete_old` 无法生效；
- 检查周期 1 s，由 main 创建的定时器驱动。

### 目录级高低水位清理

用于防止多次录制累积占满磁盘：

- 默认高水位 `max_dir_size_gb = 150.0`、低水位 `dir_low_watermark_gb = 130.0`；
- 未进入清理时，目录总量达到高水位才开始删；进入后必须删到低水位才停，
  避免每秒只删一卷；
- 删除顺序：先删**历史批次**（按批次序号 → 分卷号从旧到新），历史批次删完仍超限
  才删**当前批次的旧分卷**；两轮都跳过 `.bag.active`；
- 按整卷删除，因此实际结果可能比低水位再低最多一卷（10 GB）；
- 开录前会先做一次预清理（此时没有当前批次，全部按历史批次处理）；
- **停止录制后不清理**，刚录完的批次至少保留到下次开录；
- `max_dir_size_gb <= 0` 关闭清理，但仍会统计并发布 `recording_dir_size`；
- 只剩 `.bag.active` 或删除失败时无法继续降低，打印一次限流告警，不反复刷屏。

两套上限互相独立：默认配置下单次录到 25 GB 就停，目录清理主要在多次录制累积到
150 GB 时生效。要让录制连续跑、完全靠目录循环覆盖，可把 `total_limit_action`
设为 `delete_old` 或调大 `max_bag_size_gb`。

### 频率链

录制是事件驱动，没有硬件上报或固定采集周期：

| 环节 | 频率 | 决定因素 |
| --- | --- | --- |
| 触发 start/stop | 事件驱动 | 服务调用方；联动节点按 `/chassis_msg` 的 `tops_power_enable` 跳变去重后调用 |
| rosbag 写入 | 连续写入 | 上游话题频率与磁盘带宽；单卷写满 10 GB 时切分卷 |
| `recording_size` / `recording_dir_size` | 1 Hz | main 中的 `ros::Duration(1.0)` 定时器；停止后不再更新 |
| `recording_status` | 仅跳变 | start / stop / 节点析构 |

- 启动填充：start 成功后父进程立即发布大小 0，rosbag 随后创建 `.bag.active`；
  第一次 1 Hz tick 后才有实际大小；`recording_dir_size` 在开录前预清理后也会先发一次。
- 数据量放大：本节点不做多帧累加或重复拼接，bag 体积由上游消息和 lz4 压缩率决定；
  目录总占用是所有批次分卷之和，不是单次录制大小。
- 参数位置：单卷 10 GB 与检查周期在源码写死；两个上限与水位在 launch；
  实际录制数据频率由上游话题决定，本节点不改变。

## 参数

参数全部放在 `<node>` 外的**全局命名空间**，节点用主句柄 `nh` 读取。因此
`rosrun bag_recorder bag_recorder_node _save_dir:=...` 这类私有覆盖**不生效**，
必须用 `roslaunch ... save_dir:=...`。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `record_topics` | 原始点云 + 两个感知话题 | 字符串数组；也兼容 `[/a, /b]` 字符串；缺失或为空时回退源码默认值 |
| `save_dir` | 空 | 空表示按 `bag_recorder` 包路径推导 `<pkg>/../../../log/rosbag`，即仓库根目录的 `log/rosbag`；目录不存在时自动创建 |
| `max_bag_size_gb` | `25.0` | 单次录制总大小上限（GB）；`<= 0` 时回退到 25.0 |
| `total_limit_action` | `stop` | `stop` 停止录制；`delete_old` 删除最旧已关闭分卷后继续；其它值回退 `stop` |
| `max_dir_size_gb` | `150.0` | 保存目录所有批次总大小高水位（GB）；`<= 0` 关闭目录级清理 |
| `dir_low_watermark_gb` | `130.0` | 目录清理目标低水位（GB）；须在 `(0, 高水位)` 之间，非法时回退为「高水位 − 2 个单卷」，不足则取高水位一半 |

源码写死、不支持 launch 修改的项：

- 单卷大小 `10240` MB、`--lz4`、`--split`；
- 文件名前缀 `perception_sensors_`、6 位序号、最大序号 `999999`；
- 默认录制话题：`/rslidar_points`、
  `/perception_sensors/ultrasonic_radar_datas`、`/perception_sensors/uwb_position`；
  激光雷达录原始点云而不是 `/perception_sensors/processed_lidar_cloud`——后者是
  `frame_quantity` 帧（默认 10 帧）逐点累加的结果，带宽约高一个数量级，
  实测会导致 rosbag 写入跟不上（`buffer exceeded`）并丢帧；
- 大小检查周期 1 s；
- 话题名、服务名、队列长度、latched 属性。

## 配置服务器

launch 用 `<param>` 配置标量参数，用 `<rosparam>` 配置话题数组：

```xml
<rosparam param="record_topics">
  - /rslidar_points
  - /perception_sensors/ultrasonic_radar_datas
  - /perception_sensors/uwb_position
</rosparam>
```

命令行覆盖：

```bash
roslaunch bag_recorder bag_recorder.launch \
    save_dir:=/data/rosbag \
    max_bag_size_gb:=25 \
    total_limit_action:=delete_old \
    max_dir_size_gb:=150 \
    dir_low_watermark_gb:=130
```

`save_dir` 建议使用绝对路径。同一工作空间只应运行一个本节点实例：服务名固定，
多个实例还会竞争同一条文件名序号。

## 编译与运行

```bash
cd /home/liyun/code/perception_sensors_ros_driver/catkin_ws
conda deactivate
catkin_make --only-pkg-with-deps bag_recorder -DPYTHON_EXECUTABLE=/usr/bin/python3
# 之后用全量构建复位白名单
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
source devel/setup.bash
roslaunch bag_recorder bag_recorder.launch
```

录制控制与验证：

```bash
rosservice call /perception_sensors/start_bag_recorder
rosservice call /perception_sensors/stop_bag_recorder

rostopic echo /perception_sensors/recording_status
rostopic hz   /perception_sensors/recording_size
rostopic hz   /perception_sensors/recording_dir_size
```

一键启动全部传感器并启用底盘上装电源联动，见
[perception_sensors_client_main/README.md](../perception_sensors_client_main/README.md)
和仓库根 [README.md](../../../README.md)。

## 实现说明

### 文件职责与架构边界

- [src/main.cpp](src/main.cpp)：参数读取（`loadConfig`）、3 个发布器、2 个服务、
  1 s 定时器、`ros::spin()`。话题名、服务名、队列与定时周期全部写在调用处。
- [include/bag_recorder/bag_recorder_node.h](include/bag_recorder/bag_recorder_node.h)：
  `BagRecorderConfig` 与 `BagRecorderNode` 声明；类不持有 NodeHandle、service、timer，
  只保存 publisher 副本。
- [src/bag_recorder_node.cpp](src/bag_recorder_node.cpp)：录制业务流水线，包括
  `startCallback` / `stopCallback` / `checkTotalSize`，以及目录清理、分卷统计、
  文件名分配等私有函数。

节点是单线程模型：服务回调与定时器回调都在 `ros::spin()` 的同一队列中串行执行，
因此 `record_pid_`、`is_recording_`、清理状态和序号不需要加锁。若改成
`AsyncSpinner` 或独立回调队列，必须重新划分锁边界。

### 进程模型

- `startCallback` 用 `fork()` 复制进程，子进程 `execvp("rosbag", ...)` 替换成
  rosbag record；父进程保存子进程 PID、批次前缀和录制标志，随后返回服务回调。
- `stopRecording` 向 PID 发 `SIGINT`，`waitpid` 等 rosbag 完成落盘，再发布
  `recording_status=false` 和最终大小。
- 子进程 exec 失败时用 `_exit(1)` 退出，不执行从父进程继承的析构与 `atexit` 回调，
  也不会回到 `spin()` 变成第二个节点。

### 目录清理实现

1. 用正则匹配 `perception_sensors_<6 位序号>_<8 位日期>_<6 位时间>_<分卷号>.bag[.active]`；
2. 按批次序号 → 分卷号升序排序；
3. 未进入清理态且总量低于高水位时直接返回；否则置清理态；
4. 第 0 轮只删非当前批次，第 1 轮才允许删当前批次旧卷，直到总量 ≤ 低水位；
5. 两轮都跳过 `.bag.active`；删除后统一发布一次目录大小。

### 失败处理

- 保存目录不存在：自动创建；路径存在但不是目录：start 返回失败；
- `fork()` 失败：start 返回失败，不产生 bag 文件；
- `execvp()` 失败：子进程打印错误并 `_exit(1)`；
- 删除文件失败：打印告警并继续尝试下一个，不中断录制。

## 已知限制

| 位置 | 问题 | 影响 |
| --- | --- | --- |
| `checkTotalSize` / `stopRecording` | 没有周期检查 rosbag 子进程是否存活 | rosbag 异常退出后节点仍认为在录制：start 会返回 `Already recording!`，大小话题停在最后值，直到人工调用 stop |
| 目录清理 | 跳过 `.bag.active` | 异常退出留下的残留 `.bag.active` 不会被自动删除，需人工确认后用 `rosbag reindex` 或直接清理 |
| 目录清理 | 停止录制后不清理 | 刚录完的批次至少保留到下次开录，期间目录可能短时超过高水位 |
| 目录清理 | 只按整卷删除 | 结果可能比低水位再低最多一卷（10 GB） |
| 单次上限 | 默认 `max_bag_size_gb=25` 与目录上限独立 | 默认配置下录制到 25 GB 就停，目录清理只在多次录制累积到 150 GB 时生效 |
| `delete_old` | 只能删除已关闭分卷 | 总上限小于单卷时首卷写入期间无法生效 |
| 参数读取 | 使用全局命名空间 | `rosrun ... _save_dir:=...` 不生效，必须用 `roslaunch` |
| 实例数量 | 服务名、文件名前缀固定 | 同时运行多个实例会冲突，序号也可能竞争 |
| 录制大小统计 | 只认本节点命名规则 | 目录里手动录制的 bag 不计入大小，也不会被清理 |
| 删除操作 | 直接 `fs::remove` | 清理不可恢复，删前无法确认内容 |
