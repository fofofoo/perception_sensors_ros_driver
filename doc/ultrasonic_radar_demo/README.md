# 超声波雷达通讯说明（DYP 4 路测距模组）

本文说明 DYP 超声波测距模组的两种接入方式、物理连接、底层协议、通信库和问询流程。

两种方式都需要保留，按现场条件选择：

| 方式 | 连接 | 协议 | 使用的库 |
| --- | --- | --- | --- |
| **A. 串口直连** | 模组 RS485 → 上位机 RS485 接口（板载或 USB 转 485） | DYP `55 AA` | `serial` 库 |
| **B. 网口接入** | 模组 RS485 → 串口服务器 → 以太网 → 上位机 | Modbus TCP（502）或 DYP over TCP（8801） | POSIX socket |

文末保留精简后的排障记录。

## 1. 两种接入方式

### 1.1 方式 A：串口直连（DYP 协议）

```
DYP 测距模组 ──RS485──> 上位机 RS485 接口（板载或 USB 转 485）
从机地址 0x01           /dev/ttyS*、/dev/ttyUSB*、/dev/ttyTHS* 等
```

- 上位机直接操作串口设备；
- RS485 接口可以是工控机板载的 DB9/端子，也可以是 USB 转 485 模块；
- 使用 ROS 的 `serial` 库；
- 应用层使用 DYP `55 AA` 协议；
- 适合没有串口服务器、或需要本地直连调试的场景；
- 当前 ROS 节点采用的就是这种方式。

### 1.2 方式 B：网口接入（经串口服务器）

```
DYP 测距模组 ──RS485──> 艾莫迅 RS485-ETH-M04 ──以太网──> 上位机
从机地址 0x01           192.168.1.15                    192.168.1.15:502
```

- 上位机通过 TCP 连接串口服务器；
- 使用 POSIX socket；
- 串口服务器有两种可用模式：
  - **502 通用 TCP 转 RTU**：网口侧走 Modbus TCP，模组侧由网关转成 Modbus RTU；
  - **8801 Server 透传**：网口侧原始字节流，DYP 帧可以原样透传。
- 适合设备已经联网、或上位机与模组距离较远的场景。

### 1.3 方式 C：网口透传（DYP over TCP，可选）

这是方式 B 的一个子模式，单列出来是因为协议和 502 不同：

```
上位机 ──TCP:8801──> M04 透传 ──原始 DYP 帧──> 模组
```

- 仍然用 socket；
- 仍然用 DYP `55 AA` 协议；
- 需要自己做 TCP 收包缓冲和帧重同步；
- 适合"已经走网口，但想保留 DYP 逻辑"的场景。

### 1.4 如何选择

| 现场条件 | 选择 | 协议 | 库 |
| --- | --- | --- | --- |
| 模组直接接上位机 RS485（板载或 USB-485） | A | DYP（或 Modbus RTU） | serial |
| 有 M04，且已配置为 502 | B | Modbus TCP | socket |
| 有 M04，想继续用 DYP | C（8801 透传） | DYP over TCP | socket |
| 串口和网口都要支持 | 双模式，见第 8 节 | 按连接选择 | serial / socket |

## 2. 物理连接与参数

### 2.1 串口直连接线

模组 `A+/B-` 接上位机 RS485 接口的 `A/B`，常见有两种形态：

| 形态 | 设备名示例 | 说明 |
| --- | --- | --- |
| 工控机板载 RS485 | `/dev/ttyS0`、`/dev/ttyS1`、`/dev/ttyTHS1`（Jetson） | DB9 或端子引出，部分机型需在 BIOS/跳线/软件里切换 RS232/RS422/RS485 |
| USB 转 RS485 | `/dev/ttyUSB0`、`/dev/ttyACM0`、`/dev/ttyCH343USB0` | CH343、FT232、CP2102 等芯片，插入后按驱动枚举 |

几个注意点：

- 先确认接口真的是 **2 线半双工 RS485**，不是 RS232；RS422 是 4 线，不能直接当 2 线用；
- 板载串口的设备名不一定是 `ttyUSB*`，代码里不要写死，按实际设备修改；
- 普通用户访问串口需要加入 `dialout` 组：`sudo usermod -aG dialout $USER`；
- 同一时刻只允许一个主机在 RS485 总线上发送。

### 2.2 网口接线

- 模组 `A+/B-` 接到 M04 的第一路 `A0+/B0-`；
- 端子、指示灯、端口都从 **0** 开始编号：

  ```
  A0+/B0-  =  HC0  =  端口 502（或 8801）  =  CH0 指示灯
  ```

- 盯 "CH1" 的灯去看第一路，是看错地方了。

### 2.3 串口参数

| 项目 | 值 |
| --- | --- |
| 波特率 | 9600 |
| 数据位 / 停止位 | 8 / 1 |
| 校验位 | 无 |
| 从机地址 | 默认 `0x01` |
| 模组供电 | 9～36 V |

无论是方式 A 还是方式 B/C，模组这一侧的串口参数都必须是 9600 8-N-1。

### 2.4 从机地址

- 默认 `0x01`，可设置为 `0x01~0xFE`；
- `0xFF` 是广播地址，从站按规范不应答；
- 地址可以改，但正常运行时不操作，见 DYP `0x03` 和 Modbus `0x0200`。

## 3. 模组输出方式

三种输出方式是**出厂选型、硬件互斥**的，一个模组只能是一种：

| 型号 | 特征 | 输出方式 |
| --- | --- | --- |
| DYP-A05LYU-V1.1 | 串口自动输出 | UART（TTL） |
| **DYP-A05LY4-V1.1** | **RS485 受控输出** | **RS485** |
| DYP-A05LYJ-V1.1 | 继电器干接点输出 | 继电器 |

本项目用的是 RS485 受控输出型号。

### 3.1 UART 自动输出（仅 LYU 型号）

上电后自动推送，10 字节一帧：

```
FF | D1H D1L | D2H D2L | D3H D3L | D4H D4L | SUM
```

- 每路 2 字节大端，单位 mm；
- `SUM` 是前面所有字节求和取低 8 位；
- 收到 Modbus 指令后会切换到受控模式，断电重启又回到自动输出；
- 这是 TTL 电平 UART，不是 RS-232；
- **RS485 型号没有这个模式，不需要关闭**。

### 3.2 RS485 受控输出（LY4 型号）

- 上电后不主动发数据，只在收到问询后回一帧；
- 同一个 RS485 接口同时支持 **DYP 协议**和 **Modbus RTU**；
- 具体使用哪种协议、能否走到模组，取决于采用方式 A、B 还是 C。

## 4. 底层协议

### 4.1 DYP 协议（`55 AA`）

通用帧格式：

```
55 AA | 地址 | 控制字 | 数据... | 校验和
校验和 = 前面所有字节累加 & 0xFF
```

控制字：

| 控制字 | 含义 | 发送周期 | 应答长度 |
| --- | --- | --- | --- |
| `0x01` | 全部 4 路探头测距 | ≥ 250 ms | 13 字节 |
| `0x10` | 只测 1 号探头 | ≥ 100 ms | 7 字节 |
| `0x11` | 只测 2 号探头 | ≥ 100 ms | 7 字节 |
| `0x12` | 只测 3 号探头 | ≥ 100 ms | 7 字节 |
| `0x13` | 只测 4 号探头 | ≥ 100 ms | 7 字节 |
| `0x03` | 修改传感器地址 | 一次性 | 5 字节回显 |
| `0xFF` | 广播地址 | — | — |

全部探头问询示例（地址 `0x01`）：

```
主机发：55 AA 01 01 01
                        └─ 校验和 = (0x55 + 0xAA + 0x01 + 0x01) & 0xFF = 0x01

从机回：55 AA 01 01 | D1H D1L | D2H D2L | D3H D3L | D4H D4L | checksum
                     4 路数据，每路 2 字节大端，单位 mm
```

注意应答是 **13 字节**，最后一位是校验和。

单探头问询示例（以 1 号为例）：

```
主机发：55 AA 01 10 checksum
从机回：55 AA 01 10 DataH DataL checksum
```

### 4.2 Modbus RTU

帧格式见 [doc/超声波雷达/超声波协议.txt](doc/超声波雷达/超声波协议.txt)。模组支持：

- 读功能码：`0x03`
- 写功能码：`0x06`
- 校验：CRC-16/MODBUS（低位在前）
- 从机地址：默认 `0x01`，`0xFF` 广播

| 寄存器 | 功能 | 读写 | 说明 |
| --- | --- | --- | --- |
| `0x0106` / `0x0010` | 1 号探头测量值 | 只读 | 16 位无符号，单位 mm |
| `0x0107` / `0x0011` | 2 号探头测量值 | 只读 | 同上 |
| `0x0108` / `0x0012` | 3 号探头测量值 | 只读 | 同上 |
| `0x0109` / `0x0013` | 4 号探头测量值 | 只读 | 同上 |
| `0x0200` | 从机地址 | 读写 | 范围 `0x01~0xFE`，默认 `0x01` |

读请求和应答：

```
主机发：地址 | 03 | 寄存器地址(2) | 数量(2) | CRC16
从机回：地址 | 03 | 字节数 | 数据(N) | CRC16
```

示例（读取 1 号探头）：

```
主机：01 03 01 06 00 01 65 F7
从机：01 03 02 01 3C B9 C5
      01 3C = 316 mm
```

- `0x0106~0x0109` 支持连续读取，最多读到 `0x0109`；
- 每次收到读指令会启动一次测距，响应约 100 ms；
- 读到 `0` 表示该路没有有效回波，不是通讯故障。

### 4.3 Modbus TCP（仅网口 502 模式）

方式 B 的网口侧不是 RTU 帧，而是标准 Modbus TCP：

```
MBAP(7 字节) | 功能码 | PDU
```

MBAP 包含：

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| 事务标识 | 2 | 主机自增，应答原样返回 |
| 协议标识 | 2 | Modbus 固定为 `0x0000` |
| 长度 | 2 | 后续字节数 = 单元标识 + PDU |
| 单元标识 | 1 | 对应从机地址，默认 `0x01` |

- 网口侧 **没有 CRC**；
- 由 M04 负责转换成带 CRC 的 Modbus RTU；
- 读取寄存器的 PDU 与 RTU 相同：`03 01 06 00 04`。

### 4.4 两种协议的共同点

不管 DYP 还是 Modbus，RS485 型号的行为都是：

```
上位机发一帧问询  →  模组回一帧应答
```

没有主动上报，也没有事件推送。区别只在帧头、寻址方式、校验方式和寄存器/控制字的表达。

## 5. 通信库：serial 与 socket

两种接入方式对应两套完全不同的 API。它们都只负责"搬运字节"，不负责理解协议。

### 5.1 serial 库（方式 A）

ROS1 的 `serial` 功能包（`ros-noetic-serial`），C++ 接口是 `serial::Serial`。

它封装的是 Linux 的 **termios** 串口 API，负责：

- 打开 `/dev/tty*` 设备；
- 配置波特率、数据位、停止位、校验位；
- 设置读超时；
- 读写原始字节。

典型用法：

```cpp
serial::Serial sp;
sp.setPort("/dev/ttyCH343USB0");   // 按实际设备修改，可能是 /dev/ttyS0、/dev/ttyUSB0 等
sp.setBaudrate(9600);
sp.setBytesize(serial::eightbits);
sp.setStopbits(serial::stopbits_one);
sp.setTimeout(serial::Timeout::simpleTimeout(1000));
sp.open();

sp.write(send, 5);              // 发送 DYP 问询帧
int n = sp.available();         // 有多少字节可读
sp.read(buffer, n);             // 读取应答
```

特点：

- 面向**串口设备文件**，地址是 `/dev/tty*`；
- 不涉及 IP、端口、TCP；
- 不会帮你分帧，DYP 帧的帧头、长度、校验都需要自己处理；
- 串口是字节流，同样存在半帧、粘帧问题。

### 5.2 socket（方式 B / C）

socket 不是第三方库，而是**操作系统网络协议栈的系统调用接口**，由 libc 提供。Linux 下直接包含：

```cpp
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
```

核心调用：

```cpp
int fd = socket(AF_INET, SOCK_STREAM, 0);   // TCP 套接字
connect(fd, addr, addr_len);                // 连接 IP:端口
send(fd, request, len, MSG_NOSIGNAL);       // 发字节
recv(fd, buffer, sizeof(buffer), 0);        // 收字节
close(fd);
```

特点：

- 面向**网络连接**，地址是 `IP:端口`；
- 通过参数可以选择 TCP、UDP、IPv4、IPv6、本机进程间通信等；
- Linux 下不需要额外安装库，也不需要额外链接选项；
- 同样不帮你分帧，Modbus TCP 的 MBAP、长度、DYP 的帧头都要自己处理；
- TCP 是字节流，必须自己做收包缓冲和重同步；
- 连接可能断开，需要处理超时和重连；
- 可用 `select` / `poll` / `epoll` 做多路复用。

### 5.3 对比

| | serial 库 | socket |
| --- | --- | --- |
| 面向对象 | 串口设备 | 网络连接 |
| 地址形式 | `/dev/ttyUSB0`、`/dev/ttyCH343USB0` | `192.168.1.15:502` |
| 底层机制 | termios | TCP/IP 协议栈 |
| 常见搭配协议 | DYP、Modbus RTU | Modbus TCP、DYP over TCP、HTTP |
| 额外依赖 | `ros-noetic-serial` | 无（系统 API） |
| 分帧责任 | 应用自己处理 | 应用自己处理 |
| 断开检测 | 打开失败、读超时、设备拔出 | 连接断开、读写超时 |
| 重连 | 重新 `open()` | 重新 `connect()` |

### 5.4 为什么两种都保留

它们不是替代关系，而是对应不同现场：

- 设备在本地、只有 USB-485 → 只能用 serial；
- 设备已接入串口服务器 → 只能走 socket；
- 两者都存在的项目 → 在传输层做抽象，协议和发布逻辑复用。

因此不要把串口代码删掉，而是让节点具备两种传输方式，按配置选择。

## 6. TCP 中介：串口服务器

### 6.1 工作模式与端口

M04 的几种模式对网口侧的要求完全不同：

| 模式 | 端口 | 网口侧协议 | 能否承载 DYP |
| --- | --- | --- | --- |
| 通用 TCP 转 RTU | 502 / 503 / 504 / 505 | Modbus TCP | ❌ |
| Server 透传 | 8801 / 8802 / 8803 / 8804 | 原始字节流 | ✅ |
| RTU 主站 | 5502（仅 CH0） | 由网关主动轮询 | ❌ |

通道编号和端口一一对应：HC0 → 502（或 8801），HC1 → 503（或 8802），以此类推。

### 6.2 502：Modbus 网关

这个模式不是透传，而是**协议转换**：

```
上位机 ──Modbus TCP（MBAP + PDU，无 CRC）──>
M04 解析 MBAP/PDU，重新组 Modbus RTU 帧（加 CRC）──>
模组
```

- 网关负责 CRC、从机地址映射和 RTU 时序；
- 网口侧只能发合法的 Modbus TCP 报文；
- 发 DYP 的 `55 AA` 帧会被当作非法报文丢弃，一个字节都不回；
- 因此方式 B 只能用 Modbus。

### 6.3 8801：Server 透传

- 网口侧收到的字节原样转发到串口，不解析协议；
- DYP 帧可以正常通过；
- 但 TCP 是字节流，没有消息边界，上位机仍需自己做收包缓冲和帧重同步；
- 若改这个模式，需要重新配置 M04，并把端口从 502 换成 8801。

### 6.4 配置注意事项

1. 网页上点"保存并重启"**不够**，参数不会真正生效；
2. 改完配置必须**断电重启**，这是早期"通讯永远失败"的真正原因；
3. 串口服务器串口参数必须是 9600 8-N-1；
4. 确认接线是 `A0+/B0-` 这一路，不要看错通道号。

## 7. 问询流程

三种连接的共同点：主机发问询，模组回应答。不同点在于帧格式和分帧方式。

### 7.1 方式 A：串口 + DYP

```
上位机：serial.write("55 AA 01 01 01", 5)
模组：  200~210 ms 后返回 13 字节
上位机：读取、校验帧头、取 4 路数据
```

- 问询周期 ≥ 250 ms；
- 串口读到的可能不是完整一帧，需要按 `55 AA` 帧头做重同步；
- 应答最后一字节是校验和，建议校验。

### 7.2 方式 B：TCP + Modbus

```
上位机：socket.send("00 01 00 00 00 06 01 03 01 06 00 04")
M04：   转换成 Modbus RTU 发给模组
模组：  约 100 ms 后返回 RTU 应答
M04：   转换成 Modbus TCP 返回 17 字节
上位机：解析 MBAP，取 8 字节数据
```

- 单次响应约 100 ms，超时一般设 1 s；
- 4 Hz 轮询即可，每次读取都会触发一次测距；
- 需要处理 TCP 粘包/拆包；
- 断线后重新 `connect()`。

### 7.3 方式 C：TCP + DYP

```
上位机：socket.send("55 AA 01 01 01")
M04：   原样转发到 RS485
模组：  返回 13 字节 DYP 应答
上位机：从 TCP 字节流中找出完整帧并校验
```

- 帧格式和方式 A 完全一样；
- 只是传输从串口换成了 TCP；
- TCP 没有消息边界，必须自己做缓冲和重同步。

### 7.4 方式 B 的请求/应答示例

```
请求：00 01 00 00 00 06 01 03 01 06 00 04
      │     │        │  │  │     │
      │     │        │  │  │     └─ 数量 = 4
      │     │        │  │  └─ 起始寄存器 = 0x0106
      │     │        │  └─ 功能码 0x03
      │     │        └─ 单元标识 / 从机地址 = 1
      │     └─ 长度 = 6（单元标识 + PDU）
      └─ 协议标识 = 0（Modbus）

应答：00 01 00 00 00 0B 01 03 08 00 00 00 00 00 00 02 BC
      │                    │  │  │  └─ 8 字节数据（4 路 × 2 字节，大端）
      │                    │  │  └─ 字节数 = 8
      │                    │  └─ 功能码 0x03
      │                    └─ 单元标识 = 1
      └─ 长度 = 11（单元标识 + PDU）
```

上面示例中前 3 路为 0，4 号探头为 `0x02BC = 700 mm`。

## 8. 代码与工具

### 8.1 ROS 节点现状

`catkin_ws` 内有两个互不依赖的功能包：

| 功能包 | 目录 | 接入方式 | 协议 | 话题 / 消息 |
| --- | --- | --- | --- | --- |
| `radar_demo`（旧，保持原样） | `catkin_ws/src/ultrasonic_radar_dyp` | A：串口直连 | DYP `55 AA` | `ultrasonic_radar` / `radar_demo/radar_data` |
| `ultrasonic_radar_modbus_tcp`（新） | `catkin_ws/src/ultrasonic_radar_modbus_tcp` | B：M04 502/503 网关 | Modbus TCP | `ultrasonic_radar_demo/ultrasonic_radar_datas` |

- 旧包串口硬编码 `/dev/ttyCH343USB0`（9600 8N1），问询帧 `55 AA 01 01 01`，每 250 ms 一次；
- 旧包本次未做任何修改，其已知问题见第 11 节；
- 新包通过网口读取 DYP 寄存器，支持同时接两台串口服务器，启动时逐台探测，
  读不到的端口先跳过、再由后台线程周期重探（默认 30 s），无需重启节点。

### 8.2 双模式设计建议（历史方案，未采用）

最初考虑用参数在串口 / 网口、DYP / Modbus 之间切换，当前实现选择了更直接的方案：
**新包固定走 Modbus TCP 的两个网关端口，旧包保持串口直连**。以下思路仅作参考：

```
传输层：SerialTransport  / TcpTransport
协议层：DypProtocol      / ModbusProtocol
发布层：radar_data 组装与 publish（两种方式共用）
```

对应参数原本设想为 `~transport`、`~protocol`、`~port`、`~host`、`~tcp_port`、
`~slave_id`、`~poll_interval`、`~timeout`；实际落地时只保留了现场会变的项，
且参数放在全局命名空间，详见第 9 节和包内 README。

### 8.3 测试脚本

| 文件 | 用途 |
| --- | --- |
| [test_comm_tcp.py](test_comm_tcp.py) | 裸 socket 版 Modbus TCP，无第三方依赖，排查首选 |
| [test_comm_pymodbus.py](test_comm_pymodbus.py) | pymodbus 版，自动适配 `slave` / `device_id` 参数名 |

两者都默认连接 `192.168.1.15:502`，读取 `0x0106` 起的 4 个寄存器。

### 8.4 编译与运行

```bash
cd catkin_ws
conda deactivate                 # 必须用系统 Python，否则找不到 empy
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

# 新节点：Modbus TCP
roslaunch ultrasonic_radar_modbus_tcp ultrasonic_radar_modbus_tcp.launch

# 旧节点：串口 DYP
rosrun radar_demo radar_node
```

- 工作空间目录已重命名为 `ultrasonic_radar_demo`，路径中不再有空格；
- 新包可执行文件名为 `radar_node`，实际二进制位于
  `devel/lib/ultrasonic_radar_modbus_tcp/radar_node`。

## 9. ROS 节点实现简介

新节点 `ultrasonic_radar_modbus_tcp` 按「**main 直白定义参数与节奏，节点类封装数据流水线**」组织。

### 9.1 文件结构

| 文件 | 职责 |
| --- | --- |
| `src/main.cpp` | 字面量默认参数、读取 launch 全局参数、组装服务器列表、创建发布器、事件循环 |
| `include/.../ultrasonic_radar_node.h` | 节点类、`ServerConfig`、`RadarNodeConfig` 定义 |
| `src/ultrasonic_radar_node.cpp` | 启动探测、连接/重连、读寄存器、mm→m 换算、打印 |
| `include/.../modbus_tcp_client.h` | Modbus TCP 客户端接口 |
| `src/modbus_tcp_client.cpp` | POSIX socket 实现 MBAP + 功能码 `0x03`，处理半帧/粘包 |
| `msg/` | 单服务器数据 `ultrasonic_radar_data` 与聚合消息 `ultrasonic_radar_datas` |
| `launch/` | 现场参数：网关 IP、两个端口、轮询/超时/重连、打印开关 |

### 9.2 运行流程

```
main()
  ├─ 读取全局参数 → 组装 servers（left_radar_502、right_radar_503）
  ├─ UltrasonicRadarNode node(servers, config)
  │    └─ start()：校验参数 → 逐台探测 → 记录 available
  └─ while (ros::ok())
       ├─ queryRadars()        // 必要时重连 → 读寄存器 → 换算，返回一帧聚合数据
       ├─ array_pub.publish()  // 每轮一条 ultrasonic_radar_datas
       └─ spinOnce() + rate.sleep()
```

### 9.3 设计要点

- **参数分层**：网关 IP、端口、轮询周期、超时、重连/重探间隔、打印开关放 launch
  （写在 `<node>` 外的全局命名空间，main 用主句柄 `nh` 读取，不使用 `pnh`）；
  话题名、从机地址、寄存器地址/数量、距离阈值写死在 `main.cpp`，不建 launch 参数。
- **单一发布话题**：每轮发布一条 `ultrasonic_radar_datas`，`radars[]` 中每个元素
  对应一台服务器，`name` 区分左右来源（`left_radar_502` / `right_radar_503`）；
  元素内字段为 `Front` / `Side1` / `Side2` / `Back`，对应寄存器 1~4 号探头。
- **节点类不持有 ROS 组件**：发布器和 `ros::Rate` 都在 main；类只负责探测、连接、
  读取、换算和打印，返回组装好的消息。
- **启动探测 + 后台重探 + 断线重连**：启动时每台最多试 2 次；失败端口先跳过，
  由后台线程每 `reprobe_interval`（默认 30 s，`0` 关闭）重探，上线后自动加入轮询；
  已探测成功的端口运行中断线时按 `reconnect_interval` 重连。重探在独立线程执行，
  不阻塞 main 的 0.3 s 轮询。
- **协议实现**：Modbus TCP 为 MBAP + PDU、无 CRC；只实现 `0x03`；接收侧有缓冲区，
  按 MBAP 长度切帧，并丢弃超时后迟到的旧应答。
- **数据语义**：原始值单位 mm，除以 1000 换算为米；0 mm 表示该路无回波；
  ≤ 0.1 m 统一发布 `-1.0`，订阅方用 `< 0` 判断无目标。

模块参数、话题字段和排障细节见
[catkin_ws/src/ultrasonic_radar_modbus_tcp/README.md](catkin_ws/src/ultrasonic_radar_modbus_tcp/README.md)。

## 10. 排障记录（精简）

### 9.1 pymodbus 参数名写错会静默失败

| 参数名 | 版本 |
| --- | --- |
| `slave` | 3.0 ~ 3.8 |
| `device_id` | 3.9 以后 |

老版本遇到不认识的 `device_id=` **不报错、直接忽略**，单元标识退成默认 `0`。而 `0` 是广播地址，从站按规范不应答，现象就是"TCP 连上了，但 0 字节返回"。

- 不能用 `try/except TypeError` 兜底，因为根本不会抛异常；
- 要检查方法签名里有没有这个名字；
- `trace_packet` 是 3.7 才加入的，老版本看不到实际发出的字节。

### 9.2 M04 改配置后必须断电重启

网页"保存并重启"不够，参数不会真正保存。症状是参数看着全对、TCP 也能连，就是不通。断电重插即可。

### 9.3 502 是协议网关，不是透传

- 通用 TCP 转 RTU 模式下，网口侧必须发 Modbus TCP；
- 发裸 DYP/RTU 帧会被丢弃；
- 透传要用 8801~8804。

### 9.4 读到 0 不是通讯错误

0 表示该路没有有效回波（未接探头、无目标、超量程）。只要帧结构和校验正确，就是真实值。

### 9.5 第一轮调试先看字节

优先用裸 socket 或串口监听，把实际收发的字节打出来，再套库。看不见字节的库不适合做第一次通讯验证。

## 11. 遗留问题

**1~3 号探头始终读 0，只有 4 号有值。** 与通讯无关。若那三路确实接了探头，需要检查探头与模组之间的接线，或确认探头前方有有效反射目标。

另外，旧串口包 `radar_demo` 还有几个已知问题（新 Modbus TCP 包不涉及这些帧解析问题）：

- `while (ros::ok)` 缺少括号，条件恒真；
- 帧长判断 `n >= 11` 却读取 `buffer[11]`，应为至少 `n >= 12`，完整帧为 13 字节；
- 未校验 DYP 帧最后一位 checksum；
- 无帧缓冲和重同步机制，半帧或粘包会解析错位。

新节点当前现场只检测到 `192.168.1.15:502`（`left_radar_502`）；
`503` 未接模组，启动探测时返回网关异常帧，按设计跳过。

## 参考文档

- [doc/超声波雷达/超声波协议.txt](doc/超声波雷达/超声波协议.txt)：Modbus 协议与寄存器
- `doc/超声波雷达/超声波雷达_DYP-RD-产品规格书(DYP-A05-V1.1).pdf`：DYP 协议、UART 自动输出、型号说明
- `doc/串口服务器/串口服务器 ETH-RS485-M04产品手册.pdf`：工作模式、端口映射、配置步骤


git archive --format=zip --output=ultrasonic_radar_modbus_tcp.zip HEAD  
