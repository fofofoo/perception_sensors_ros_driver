#pragma once

#include <ros/ros.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ultrasonic_radar_modbus_tcp/ultrasonic_radar_data.h>
#include <ultrasonic_radar_modbus_tcp/ultrasonic_radar_datas.h>

#include "ultrasonic_radar_modbus_tcp/modbus_tcp_client.h"

namespace ultrasonic_radar_modbus_tcp {

// 一台串口服务器（Modbus 网关端口）的连接参数。
// name 用于消息里区分数据来源；host/port 是网关地址；slave_id 是 RS485 侧
// DYP 电应普超声波雷达模组的从机地址。
struct ServerConfig {
  std::string name;
  std::string host;
  int port = 502;
  int slave_id = 1;
};

// 节点运行参数。默认值与 launch 文件保持一致，main 中按现场覆盖。
struct RadarNodeConfig {
  // —— 寄存器 ——
  int start_register = 0x0106;  // 1 号探头寄存器（十进制 262），4 路连续
  int probe_count = 4;          // 连续读取数量，1~4

  // —— 轮询 ——
  double poll_interval = 0.3;       // 周期（s）：0.25 会让 M04 周期性返回网关忙
  double timeout = 0.5;             // 连接 / 单次应答超时（s）
  double reconnect_interval = 1.0;  // 已探测到的服务器断线后的重连间隔（s）
  // 启动探测失败的服务器重新探测间隔（s）；<= 0 表示关闭周期重探，
  // 恢复“失败即永久跳过”的旧行为。
  double reprobe_interval = 30.0;

  // —— 距离（单位：米）——
  double min_valid_distance = 0.1;  // ≤ 该值视为无有效回波
  double invalid_distance = -1.0;   // 无效读数发布 -1.0，表示该路无目标

  // —— 调试 ——
  bool print_data = true;     // 每轮打印原始 mm 值与换算后的距离
  bool debug_frames = false;  // 打印 Modbus TCP 收发字节
};

// 超声波雷达节点。
// 职责边界：启动时探测服务器，之后由 main 的事件循环驱动——
// 每轮调用 queryRadars() 问询并读取寄存器，发布由 main 直接完成。
// 类内部只保留单台服务器的连接、读取、换算和打印细节。
class UltrasonicRadarNode {
 public:
  UltrasonicRadarNode(const std::vector<ServerConfig>& servers,
                      const RadarNodeConfig& config);

  // 停止后台重探线程并等待其退出；必须在 servers_ 销毁前完成 join。
  ~UltrasonicRadarNode();

  // 校验参数、打印启动配置、探测服务器。
  // 参数非法或 servers 为空时返回 false；全部探测失败仍返回 true，
  // 由后台重探线程继续尝试，节点保持运行并发布空数组。
  bool start(std::string* error);

  // 问询并读取所有可用服务器，返回本轮可发布的数据。
  // 已上线服务器断线时在内部按 reconnect_interval 重连；未上线的服务器
  // 直接跳过，由后台线程重探，不阻塞本轮其他服务器。
  // 同时刷新每台服务器的 online 状态（本轮读成功即 true）。
  ultrasonic_radar_datas queryRadars();

  // 单台服务器的在线状态快照，供 main 发布状态话题。
  // online 的含义是「最近一轮 queryRadars() 是否读到了有效寄存器」，
  // 因此启动探测失败、后台重探中、读超时、网关忙/异常帧都会是 false。
  struct ServerStatus {
    std::string name;  // 与数据消息里的 name 一致，如 left_radar_502
    bool online = false;
  };

  // 返回所有服务器的状态，顺序与构造时传入的 servers 一致。
  // 必须在 queryRadars() 之后、同一线程调用；online 只由主线程写，
  // 后台重探线程不修改它，因此这里不需要加 state_mutex_。
  std::vector<ServerStatus> serverStatuses() const;

 private:
  // 一台服务器的运行时状态：静态配置 + 连接对象 + 重连节流 + 探测结果。
  // 由构造函数按 main 传入的 ServerConfig 逐个创建，之后随连接和探测变化。
  struct ServerRuntime {
    ServerConfig config;  // 静态连接参数：name / host / port / slave_id
    std::unique_ptr<ModbusTcpClient> client;  // 该服务器独享的 TCP 连接
    ros::Time next_attempt;  // 下一次允许重连的时间；未到则本轮跳过
    // 当前是否参与主循环轮询。启动探测失败时为 false，由后台线程重探；
    // 成功上线后置 true，主循环随即开始读取。由 state_mutex_ 保护。
    bool available = false;
    // 下一次允许重新探测的时间。仅对 available=false 的服务器有意义，
    // 由后台线程在失败后顺延 reprobe_interval；由 state_mutex_ 保护。
    ros::Time next_probe_attempt;
    // 最近一轮 queryRadars() 是否读到了有效寄存器，发布状态话题用。
    // 与 available 的区别：available 表示「是否加入轮询」（重探成功后由后台
    // 线程置 true）；online 表示「本轮是否真的读到数据」（主线程每轮刷新）。
    // 只由主线程写（queryRadars 置位、serverStatuses 读取），后台线程不碰，
    // 因此不受 state_mutex_ 保护。
    bool online = false;
  };

  // 启动探测：最多尝试 attempts 次（默认 2），能读到寄存器才算成功。
  // 每次失败都会关闭 socket，避免把半开连接留给后续轮询。
  bool probeServer(ServerRuntime* server, std::string* error, int attempts = 2);

  // 后台重探线程主体：轮询所有 available=false 且到期的服务器。
  // 只做连接 + 读寄存器，不参与主循环发布，也不触碰 available=true 的连接。
  void reprobeLoop();

  // 重新探测一台离线服务器；成功时置 available=true，失败时顺延下次重探时间。
  // 返回 true 表示本轮已上线，主循环可以开始读取它。
  bool tryReProbe(ServerRuntime* server);

  // 确保连接可用；断线时按 reconnect_interval 节流重连。
  bool ensureConnected(ServerRuntime* server);

  // 读取一台服务器的 4 路寄存器；失败时关闭连接并安排重连。
  bool readServerRegisters(ServerRuntime* server,
                           std::vector<uint16_t>* registers);

  // 把寄存器值换算成消息。探头顺序：1 号→Front，2 号→Side1，
  // 3 号→Side2，4 号→Back；左右侧由配置中的 name 区分。
  ultrasonic_radar_data buildData(
      const ServerConfig& config,
      const std::vector<uint16_t>& registers) const;

  // 单路距离换算：mm → m；≤ min_valid_distance 时返回 invalid_distance。
  float toDistance(uint16_t millimeters) const;

  // 打印原始 mm 值和换算后的四路距离。
  void printRadarData(const ultrasonic_radar_data& data,
                      const std::vector<uint16_t>& registers) const;

  // 所有服务器的运行时状态，元素数量与 main 传入的 servers 一致。
  // start() 的探测结果、每轮 queryRadars() 的读取、重连节流以及客户端释放
  // 都以这个 vector 为准；元素在构造后不再增删，只更新内部状态。
  std::vector<ServerRuntime> servers_;

  // 保护 servers_ 中 available / next_probe_attempt 的跨线程读写。
  // 主循环在 queryRadars() 里短临界区读取 available；后台线程在探测前后
  // 更新状态。阻塞的 connect/read 不持锁，避免拖慢主循环。
  std::mutex state_mutex_;

  // 后台重探线程的退出标志；析构时先置 true 再 join。
  std::atomic<bool> probe_stop_{false};

  // 后台重探线程；reprobe_interval <= 0 时不会启动。
  std::thread probe_thread_;

  // 全局运行参数（寄存器范围、轮询/超时、距离阈值、打印开关），
  // 由 main 组装后传入，节点内部只读。
  RadarNodeConfig config_;
};

}  // namespace ultrasonic_radar_modbus_tcp
