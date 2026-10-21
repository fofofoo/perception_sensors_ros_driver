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

两个端口共用同一台串口服务器和同一个从机地址。节点启动时会逐个探测：能正常读到寄存器的才保留，读不到的
（TCP 连不上、网关返回异常帧等）直接跳过，不再参与轮询，避免占用总线或
拖慢其他组。新增服务器后需要重启节点重新探测。运行过程中已检测到的服务器
若临时断线，仍会按 `reconnect_interval` 自动重连。

轮询周期默认 `0.3 s`。实测 250 ms 时 M04 会每几次请求返回一帧
`00 80 FF` 网关忙应答，导致数据时断时续；300 ms 及以上稳定。

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
| `reconnect_interval` | `1.0` | 断线重连间隔，秒 |
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
    读取寄存器、mm→m 换算、打印；不持有句柄、发布器或话题名；
  - `src/modbus_tcp_client.cpp`：POSIX socket 实现的 Modbus TCP。
- 发布器只建在 main 中，每轮发布一条 `ultrasonic_radar_datas`；
  节点类只返回组装好的消息。
- 使用 POSIX socket 直接实现 Modbus TCP，不依赖 libmodbus。
- 只实现功能码 `0x03`（读保持寄存器），请求 12 字节，无 CRC。
- 有接收缓冲区，能处理半帧、粘包和超时后迟到的旧应答。
- 连接失败或读取失败时关闭 socket，下一轮自动重连。
- 启动探测失败的端口（如未接模组的 503）永久跳过，不占用轮询周期。
