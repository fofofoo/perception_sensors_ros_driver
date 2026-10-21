# ultrasonic_radar_modbus_tcp

DYP 电应普超声波雷达的 Modbus TCP 接入节点。通过艾莫迅 RS485-ETH-M04 等串口服务器的
Modbus 网关端口（默认 502/503）读取 4 路探头寄存器，支持同时接入多个服务器。

原有的串口 DYP 电应普节点保留在 `ultrasonic_radar_dyp`，本包不修改它。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/perception_sensors/ultrasonic_radar_datas` | `ultrasonic_radar_modbus_tcp/ultrasonic_radar_datas` | 每轮一条，`radars[]` 聚合本轮所有读取成功的服务器，`name` 区分左右：`left_radar_502` / `right_radar_503` |

`radars[]` 中每个元素的字段：

```text
string name
float32 Front   # 前
float32 Side1   # 侧1
float32 Side2   # 侧2
float32 Back    # 后
```

`ultrasonic_radar_datas` 字段：

```text
ultrasonic_radar_data[] radars
```

探头映射：1 号 → `Front`，2 号 → `Side1`，3 号 → `Side2`，4 号 → `Back`。

现场接线固定为 502 接车体左侧四个探头、503 接车体右侧四个探头，因此
`name` 分别为 `left_radar_502`、`right_radar_503`；若 launch 把端口改成
502/503 之外的值，`name` 退化为 `radar_<端口>`，不标左右。

距离单位米；读数 ≤ 0.1 m（含 0，表示无回波）时发布 `-1.0`，表示该路无目标。

## 默认行为

节点固定检测两个网关端口：

- `192.168.1.15:503`
- `192.168.1.15:502`

两个端口共用同一台串口服务器和同一个从机地址。节点启动时会逐个探测：能正常读到寄存器的才进入轮询，
读不到的（TCP 连不上、网关返回异常帧等）先跳过，**不占用主循环**；之后由后台线程每
`reprobe_interval`（默认 30 s）重探一次，接好模组或恢复接线后自动上线，无需重启节点。
前台探测失败的服务器首次重探时间按启动后 `reprobe_interval + i*2 s` 错开，避免多台同时连接。
运行过程中已上线服务器断线，仍由主循环按 `reconnect_interval` 重连。

轮询周期默认 `0.3 s`。实测 250 ms 时 M04 会每几次请求返回一帧
`00 80 FF` 网关忙应答，导致数据时断时续；300 ms 及以上稳定。

## 掉线重连方案

| 环节 | 当前实现 |
| --- | --- |
| 启动探测 | 每台服务器「TCP 连接 + 读 4 个保持寄存器」，最多 2 次（间隔 0.1 s）；只连上不读寄存器不算成功 |
| 启动失败 | 标记 `available=false`，不进入主循环；后台线程按 `reprobe_interval` 重探（默认 30 s，`0` 关闭），接好模组/恢复接线后无需重启 |
| 首次重探错峰 | 第 i 台（从 0 计）在启动后 `reprobe_interval + i*2 s` 首次重探，避免多台同时连接 |
| 断线检测 | 单次读取超时 0.5 s（`timeout`）、connect 超时或被拒；读失败立即关闭 socket |
| 重连动作 | 已上线服务器由主循环 `ensureConnected()` 重连，间隔 `reconnect_interval=1 s`；后台重探每次只试 1 次，失败顺延 30 s |
| 并发边界 | 主线程只操作 `available=true` 的连接，后台线程只操作离线连接；状态由 `state_mutex_` 保护，重探不阻塞 0.3 s 轮询 |
| 断线期间输出 | 每轮照常发布聚合话题，`radars[]` 为空；实测拔线期间话题仍约 3.8 Hz |
| 恢复判定 | 连接成功且能读到 4 路寄存器；恢复后下一轮即发布，无需重启 |
| 日志 | 连接/读取/重探失败均为 5 s 限流 WARN；日志 5~6 s 一条不代表实际重试间隔（实际约 1 s） |

实测（2026-10-21，M04 网线拔插）：

- 网线插回后，M04 在链路恢复初期约 30 s 内仍拒绝 TCP（日志 `connect: 拒绝连接`），
  此时 ping 已通；节点每秒重试，网关恢复接受连接后自动上线。不要把这段等待误判为
  节点故障或重探失效。
- 502 在线时拔线：节点不退出，空数组继续发布，插回后自动恢复，无需重启。
- 503 未接模组：启动跳过，后台每 30 s 重探失败（`unexpected unit id 0`），不影响 502 的 3.3 Hz 轮询。

### 拔线测试方案

用于验证启动探测、后台重探和运行中断线恢复，约 5 分钟：

1. **基线**：网线插好启动节点，确认 502 有数据；503 未接模组时应打印 `未检测到 ... 后台重探`；
2. **运行中拔线**：拔掉 M04 网线，确认节点不退出、`radars[]` 变空、日志出现连接/读取失败；
3. **离线启动**：保持拔线状态重启节点，确认两台都打印 `未检测到 ... 30.0 秒后后台重探`，
   且持续发布空数组；
4. **自动上线**：插回网线，约 30 s（502 错峰 +2 s）后应出现 `重新检测到 ... 已加入轮询`，数据恢复；
5. **运行中恢复**：再次拔线并插回，确认按 1 s 节流重连后自动恢复（网关可能额外拒绝 TCP 约 30 s）；
6. **退出**：Ctrl+C，节点应在 1 s 内退出，后台线程正常 join。

判定：全过程不需要重启节点；恢复明显慢于 30~60 s 时，先检查网线、网关供电和 M04 端口状态，
再看后台重探日志。

## 参数

参数定义在 launch 的**全局命名空间**（`<param>` 放在 `<node>` 外），节点用主句柄
`nh` 读取，不使用 `~` 私有参数。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gateway_ip` | `192.168.1.15` | 串口服务器 IP，两个端口共用 |
| `port_1` | `503` | 第一个 Modbus 网关端口（HC1） |
| `port_2` | `502` | 第二个 Modbus 网关端口（HC0） |
| `poll_interval` | `0.3` | 轮询周期，秒；实测 0.25 会触发网关忙错误 |
| `timeout` | `0.5` | 连接和应答超时，秒 |
| `reconnect_interval` | `1.0` | 已上线服务器断线后的重连间隔，秒 |
| `reprobe_interval` | `30.0` | 启动探测失败服务器的后台重探间隔，秒；`0` 关闭周期重探（恢复永久跳过） |
| `print_data` | `true` | 每轮打印 4 路原始 mm 值和换算距离 |
| `debug_frames` | `false` | 打印 Modbus TCP 收发字节 |

话题名、从机地址（1）、1 号探头寄存器（`0x0106`）、读取数量（4）
以及距离阈值（0.1 m / -1.0 m）都固定写在 [src/main.cpp](src/main.cpp) 中，
不支持通过 launch 修改。

## 配置服务器

默认网关和端口写在 launch 顶部的 `arg` 里，可以直接改默认值，也可以在命令行覆盖：

```bash
roslaunch ultrasonic_radar_modbus_tcp ultrasonic_radar_modbus_tcp.launch \
  gateway_ip:=192.168.1.15 port_1:=503 port_2:=502
```

从机地址固定为 `1`，写在 [src/main.cpp](src/main.cpp) 中。参数位于全局命名空间，
因此 `rosrun ... _gateway_ip:=...` 这类私有覆盖不会生效，请通过 launch 或
`rosparam set` 设置。

节点固定使用两个端口。需要接第三台服务器时，在
[src/main.cpp](src/main.cpp) 组装 `servers` 的位置按同样方式追加一条
`ServerConfig`，然后重新编译。

## 编译与运行

```bash
cd /home/liyun/code/perception_sensors_ros_driver/catkin_ws
conda deactivate
catkin_make -j8
source devel/setup.bash
roslaunch ultrasonic_radar_modbus_tcp ultrasonic_radar_modbus_tcp.launch
```

只编译本包：

```bash
catkin_make --only-pkg-with-deps ultrasonic_radar_modbus_tcp \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

查看数据：

```bash
rostopic echo /perception_sensors/ultrasonic_radar_datas
rostopic hz /perception_sensors/ultrasonic_radar_datas
```

串口服务器的 502~505 是 Modbus 网关端口，网口侧必须发 Modbus TCP
（MBAP + PDU，无 CRC），DYP 电应普自定义帧不能直接通过 502 端口。

## 实现说明

- 源码结构：
  - `src/main.cpp`：字面量默认参数、读取 launch 全局参数、组装服务器列表、
    创建发布器，并驱动事件循环（问询读取 → 发布聚合话题 → `spinOnce`）；
  - `src/ultrasonic_radar_node.cpp`：启动探测、按服务器维护连接与重连节流、
    后台重探线程（`reprobeLoop` / `tryReProbe`）、读取寄存器、mm→m 换算、打印；
    不持有句柄、发布器或话题名；
  - `src/modbus_tcp_client.cpp`：POSIX socket 实现的 Modbus TCP。
- 发布器只建在 main 中，每轮发布一条 `ultrasonic_radar_datas`；
  节点类只返回组装好的消息。
- 使用 POSIX socket 直接实现 Modbus TCP，不依赖 libmodbus。
- 只实现功能码 `0x03`（读保持寄存器），请求 12 字节，无 CRC。
- 有接收缓冲区，能处理半帧、粘包和超时后迟到的旧应答。
- 连接失败或读取失败时关闭 socket，下一轮自动重连。
- 启动探测失败的端口（如未接模组的 503）不进入主循环轮询，由后台线程每
  `reprobe_interval` 重探；重探的 connect/read 在主循环之外执行，单次最长约
  `timeout`（0.5 s），不会拖慢健康服务器的 0.3 s 轮询。`available` 与
  `next_probe_attempt` 由 `state_mutex_` 保护，主线程只访问已上线连接，
  后台线程只访问离线连接，同一 socket 不会被两端并发使用。
- 后台线程在节点析构时通过 `probe_stop_` + `join()` 退出；`reprobe_interval <= 0`
  时不启动线程，退回“失败即永久跳过”的旧行为。
