// 超声波雷达节点的流水线实现。
//
// 职责边界：
//   本类负责「串口服务器 → 一批雷达数据」的编排：启动探测、按服务器维护连接、
//   读取保持寄存器、把 mm 换算成米、组装消息。消息发布不在类内，由 main 的
//   事件循环直接完成。Modbus TCP 的帧格式、半帧/粘包处理和 socket 细节都在
//   ModbusTcpClient 中，本文件不直接操作 socket。
//
// 线程模型：
//   主线程：main 用 ros::Rate 驱动事件循环，每轮调用 queryRadars()（阻塞读取）
//   并发布返回值。
//   后台线程：只在 reprobe_interval > 0 时启动，周期性重探启动时未上线的服务器。
//   available / next_probe_attempt 由 state_mutex_ 保护；后台线程只访问
//   available=false 的 client，主线程只访问 available=true 的 client，
//   两边不会并发操作同一条连接。
//
// 数据流（由 main 的 ros::Rate 事件循环驱动）：
//   启动探测（每台最多 2 次）；失败的服务器交给后台线程每 reprobe_interval 秒重探
//   每轮：queryRadars() → 跳过未上线服务器 → ensureConnected（必要时重连）
//         → readServerRegisters → buildData →（可选）printRadarData
//         main 发布聚合话题
//
// 数据语义：
//   - 寄存器原始值为 16 位无符号整数，单位 mm，大端；
//   - 探头顺序：1 号 → Front（前），2 号 → Side1（侧1），
//     3 号 → Side2（侧2），4 号 → Back（后）；
//     左右侧（车体左侧 / 右侧）由服务器 name 区分，本消息不区分左右；
//   - 读数 ≤ min_valid_distance（0.1 m）视为无有效回波，发布
//     invalid_distance（-1.0 m）。0 mm 通常表示该路没接探头、无目标或
//     超量程，是正常读数，不是通讯故障。

#include "ultrasonic_radar_modbus_tcp/ultrasonic_radar_node.h"

#include <chrono>
#include <sstream>
#include <system_error>
#include <utility>

namespace ultrasonic_radar_modbus_tcp {

// ==================== 构造 ====================

UltrasonicRadarNode::UltrasonicRadarNode(
    const std::vector<ServerConfig>& servers, const RadarNodeConfig& config)
    : config_(config) 
{

    // 构造阶段做静态准备：保存配置、为每台服务器创建独立的 Modbus 客户端。
    // 每台服务器用一个 ServerRuntime 表示，它把以下信息放在一起：
    //   config             —— main 传来的静态连接参数（name / host / port / slave_id）；
    //   client             —— 该服务器独享的 TCP 连接；
    //   next_attempt       —— 下一次允许重连的时间，用于重连节流；
    //   available          —— 是否已上线并参与主循环轮询；
    //   next_probe_attempt —— 离线服务器的下一次重探时间。

    // servers_是所有串口服务器runtime的容器。
    servers_.reserve(servers.size());
    for (const ServerConfig& server : servers) {
        ServerRuntime runtime;

        // 静态参数原样保存；后续轮询、打印和消息里的 name 都从这里取。
        runtime.config = server;

        // 每台超声波服务器持有一个独立的 ModbusTcpClient（unique_ptr）：
        // 各自一条 TCP 连接、独立的事务号和接收缓冲，互不干扰；
        runtime.client.reset(new ModbusTcpClient());

        // 每台超声波服务器独立开关报文打印，便于只盯住有问题的那一路。
        runtime.client->setDebug(config_.debug_frames);

        // 允许立即重连；探测成功后会在 start() 中重置为当前时间。
        runtime.next_attempt = ros::Time::now();
        // 初始探测在 start() 中立即执行；失败后才会把重探时间顺延一个间隔。
        // 这里先填当前时间，避免未初始化时间被后台线程误判。
        runtime.next_probe_attempt = ros::Time::now();

        // ServerRuntime 含 unique_ptr，不可拷贝，只能移动到成员 vector 中；
        // 移动后 runtime 本身不再持有 client，节点析构时由 servers_ 负责释放。
        servers_.push_back(std::move(runtime));
    }
}

// 停止后台重探线程并等待退出。先置停止标志再 join，保证线程不会在
// servers_ / client 析构后继续访问它们。线程未启动时 joinable() 为 false。
UltrasonicRadarNode::~UltrasonicRadarNode() {
  probe_stop_.store(true);
  if (probe_thread_.joinable()) {
    probe_thread_.join();
  }
}

// ==================== 启动 ====================
// 启动阶段的完整流程：参数校验 → 打印配置 → 逐台探测。
// 参数非法时返回 false，error 带回原因，由 main 打印 ROS_FATAL 并退出。
// 探测失败的服务器只打 WARN 并置 available=false，不阻止节点启动。
bool UltrasonicRadarNode::start(std::string* error) {
  // —— 一、参数校验 ——
  // 这些值来自 main 的字面量或 launch，配置错误时尽早失败，
  // 避免节点跑起来后才在轮询中反复报错。
  if (servers_.empty()) {
    if (error) *error = "至少需要一个服务器配置";
    return false;
  }

  // DYP 电应普超声波雷达模组最多 4 路探头，寄存器从 1 号探头起连续排列。
  if (config_.probe_count < 1 || config_.probe_count > 4) {
    if (error) *error = "probe_count 必须在 1~4 之间";
    return false;
  }

  // Modbus 保持寄存器地址是 16 位；起始地址 + 数量不能越过 0xFFFF。
  if (config_.start_register < 0 || config_.start_register > 0xFFFF ||
      config_.start_register + config_.probe_count - 1 > 0xFFFF) {
    if (error) *error = "start_register 或 probe_count 超出寄存器范围";
    return false;
  }

  // 周期/超时必须为正，否则 ros::Rate 和 poll 的行为不可预期；
  // 重连间隔允许为 0（每轮都尝试重连）。
  if (config_.poll_interval <= 0.0 || config_.timeout <= 0.0 ||
      config_.reconnect_interval < 0.0) {
    if (error) {
      *error = "poll_interval / timeout 必须大于 0，reconnect_interval 不能为负";
    }
    return false;
  }

  // 逐台检查网络参数。slave_id 0 是广播地址、从站不会应答；
  // Modbus 规范的有效从机地址范围是 1~247。
  for (const ServerRuntime& server : servers_) {
    if (server.config.host.empty()) {
      if (error) *error = server.config.name + " 的 host 为空";
      return false;
    }
    if (server.config.port < 1 || server.config.port > 65535) {
      if (error) *error = server.config.name + " 的 port 非法";
      return false;
    }
    if (server.config.slave_id < 1 || server.config.slave_id > 247) {
      if (error) *error = server.config.name + " 的 slave_id 非法";
      return false;
    }
  }

  // —— 二、打印启动配置 ——
  // 现场排障时先看这一段，确认 IP、端口、寄存器、周期与预期一致。
  ROS_INFO("超声波雷达节点启动，共 %zu 个服务器", servers_.size());
  for (const ServerRuntime& server : servers_) {
    ROS_INFO("  [%s] %s:%d, slave_id=%d", server.config.name.c_str(),
             server.config.host.c_str(), server.config.port,
             server.config.slave_id);
  }
  ROS_INFO("寄存器: 起始=0x%04X(%d), 数量=%d", config_.start_register,
           config_.start_register, config_.probe_count);
  ROS_INFO("轮询=%.2fs, 超时=%.2fs, 重连=%.2fs, 重探=%.1fs, 无效值=%.2fm",
           config_.poll_interval, config_.timeout, config_.reconnect_interval,
           config_.reprobe_interval, config_.invalid_distance);

  // —— 三、启动探测 ——
  // 读不到寄存器的端口不再永久跳过：标记 available=false，交给后台线程
  // 每 reprobe_interval 秒重探。主循环只轮询已上线的服务器，离线端口不会
  // 拖慢健康服务器；运行期间已上线服务器断线仍走 reconnect_interval 重连。
  const bool reprobe_enabled = config_.reprobe_interval > 0.0;
  const ros::Time startup_time = ros::Time::now();
  int available_count = 0;
  for (size_t i = 0; i < servers_.size(); ++i) {
    ServerRuntime& server = servers_[i];
    std::string probe_error;
    server.available = probeServer(&server, &probe_error);

    if (server.available) {
      ++available_count;
      ROS_INFO("检测到 %s (%s:%d)", server.config.name.c_str(),
               server.config.host.c_str(), server.config.port);
      server.next_attempt = ros::Time::now();
    } else {
      // 按服务器序号错开首次重探，避免多台离线网关在同一时刻被同时连接；
      // 每台偏移 2 秒，对 30 秒量级的重探间隔可以忽略。
      server.next_probe_attempt =
          startup_time + ros::Duration(config_.reprobe_interval + i * 2.0);
      if (reprobe_enabled) {
        ROS_WARN("未检测到 %s (%s:%d)，%.1f 秒后后台重探: %s",
                 server.config.name.c_str(), server.config.host.c_str(),
                 server.config.port, config_.reprobe_interval,
                 probe_error.c_str());
      } else {
        ROS_WARN("未检测到 %s (%s:%d)，周期重探已关闭，保持跳过: %s",
                 server.config.name.c_str(), server.config.host.c_str(),
                 server.config.port, probe_error.c_str());
      }
    }
  }

  if (available_count == 0) {
    ROS_ERROR("没有检测到任何超声波服务器，节点继续运行但不发布有效数据%s",
              reprobe_enabled ? "；后台线程将持续重探" : "");
  }

  // —— 四、启动后台重探线程 ——
  // 线程创建失败不阻断节点：退回“离线端口永久跳过”的旧行为，健康服务器
  // 仍可正常轮询。线程本身只处理 available=false 的服务器，不参与发布。
  if (reprobe_enabled) {
    try {
      probe_thread_ = std::thread(&UltrasonicRadarNode::reprobeLoop, this);
      ROS_INFO("后台重探已启动：每 %.1f 秒检查一次离线服务器（首次按序号错开）",
               config_.reprobe_interval);
    } catch (const std::system_error& e) {
      ROS_ERROR("后台重探线程启动失败: %s；离线服务器将保持跳过状态", e.what());
    }
  } else {
    ROS_INFO("reprobe_interval=%.1f，周期重探已关闭", config_.reprobe_interval);
  }

  return true;
}

// 探测一台服务器是否真的能读到 4 路寄存器。
// 只验证 TCP 连上还不够：M04 在某个通道没接模组时仍可能建立连接，
// 但会返回网关异常帧（例如 00 80 FF），所以必须完整走一次
// 「连接 + 读保持寄存器」。
//
// attempts 由调用方决定：启动探测用 2 次（首次失败可能是网关刚上电或上一条
// 连接尚未释放，重试一次能明显降低误判）；后台周期重探用 1 次，失败就等
// 下一个间隔，避免单次重探占用过久。
// 每次失败都关闭 socket，避免把半开连接留给后续轮询。
bool UltrasonicRadarNode::probeServer(ServerRuntime* server, std::string* error,
                                      int attempts) {
  for (int attempt = 0; attempt < attempts; ++attempt) {
    std::vector<uint16_t> registers;
    if (server->client->connectTo(server->config.host, static_cast<uint16_t>(server->config.port),config_.timeout, error)) 
    {
        if (server->client->readHoldingRegisters(static_cast<uint8_t>(server->config.slave_id), static_cast<uint16_t>(config_.start_register),static_cast<uint16_t>(config_.probe_count), &registers, config_.timeout, error)) 
        {
            return true;
        }
    }
        
    server->client->close();
    // 给网关一点恢复时间再重试；最后一次失败后不再等待。
    if (attempt + 1 < attempts) ros::Duration(0.1).sleep();
  }
  return false;
}

// 后台重探线程主体。
// 每 200 ms 检查一次所有服务器：只挑 available=false 且 next_probe_attempt
// 已到期的，调用一次 tryReProbe()。锁只包住状态读取，探测期间不持锁，
// 因此主循环读取 available 不会被阻塞。
void UltrasonicRadarNode::reprobeLoop() {
  while (ros::ok() && !probe_stop_.load()) {
    for (ServerRuntime& server : servers_) {
      if (probe_stop_.load() || !ros::ok()) break;

      bool due = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        due = !server.available &&
              ros::Time::now() >= server.next_probe_attempt;
      }
      if (!due) continue;

      tryReProbe(&server);
    }

    // 分片睡眠，让析构 join 不必等满一个完整周期；200 ms 相对 30 s 的
    // 重探间隔可以忽略，也不会空转烧 CPU。
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

// 重新探测一台离线服务器。
// 成功：probeServer 已建立连接并读到寄存器，这里置 available=true 并把
// next_attempt 重置为当前时间，主循环下一轮即可读取它。
// 失败：probeServer 已关闭 socket，这里只顺延 next_probe_attempt。
// 日志用 THROTTLE，避免每 30 秒刷屏。
bool UltrasonicRadarNode::tryReProbe(ServerRuntime* server) {
  std::string error;
  const bool ok = probeServer(server, &error, 1);
  const ros::Time now = ros::Time::now();

  if (!ok) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      server->next_probe_attempt =
          now + ros::Duration(config_.reprobe_interval);
    }
    ROS_WARN_THROTTLE(30.0, "重新探测 %s (%s:%d) 失败: %s",
                      server->config.name.c_str(),
                      server->config.host.c_str(), server->config.port,
                      error.c_str());
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    server->available = true;
    server->next_attempt = now;
  }
  ROS_INFO("重新检测到 %s (%s:%d)，已加入轮询",
           server->config.name.c_str(), server->config.host.c_str(),
           server->config.port);
  return true;
}

// ==================== 轮询 ====================

// 执行一轮问询：对每台已探测成功的服务器读取 4 路寄存器，组装成一批数据。
// 单台服务器的失败（断线、超时、异常帧）只影响自己，不中断其他服务器。
// 返回的 batch 即使为空也会由 main 照常发布，订阅方据此判断节点仍在运行。
ultrasonic_radar_datas UltrasonicRadarNode::queryRadars() {
  ultrasonic_radar_datas batch;
  batch.radars.reserve(servers_.size());

  for (ServerRuntime& server : servers_) {
    // 离线服务器由后台线程重探，主循环不在这里等待连接。
    // 只做一次短临界区读取，避免和后台线程的 available 更新竞争。
    bool available = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      available = server.available;
    }
    if (!available) continue;

    // 连接断开时尝试重连；受 reconnect_interval 节流，失败不影响本轮其他服务器。
    if (!ensureConnected(&server)) continue;

    // 读取失败时 readServerRegisters() 已关闭连接并安排重连。
    std::vector<uint16_t> registers;
    if (!readServerRegisters(&server, &registers)) continue;

    // 原始 mm 值 → 四路距离；寄存器数量异常时由 buildData 填无效值。
    const ultrasonic_radar_data data = buildData(server.config, registers);

    // 打印原始值与换算值，便于和 Modbus 调试工具逐字节对照。
    if (config_.print_data) printRadarData(data, registers);

    batch.radars.push_back(data);
  }

  return batch;
}

// 保证连接可用：已连接直接返回，断线时尝试重新建立 TCP 连接。
//
// next_attempt 用于重连节流。连接失败可能一直阻塞到 timeout（默认 0.5 s），
// 如果每个轮询周期都重连，会把事件循环拖慢并让日志刷屏。
// 连接失败只告警（5 s 节流）并安排下一次重连，不关闭节点。
bool UltrasonicRadarNode::ensureConnected(ServerRuntime* server) {
  if (server->client->isConnected()) return true;

  // 还没到允许重连的时间，本轮直接跳过。
  if (ros::Time::now() < server->next_attempt) return false;

  std::string error;
  if (!server->client->connectTo(
          server->config.host, static_cast<uint16_t>(server->config.port),
          config_.timeout, &error)) {
    ROS_WARN_THROTTLE(5.0, "连接 %s (%s:%d) 失败: %s",
                      server->config.name.c_str(),
                      server->config.host.c_str(), server->config.port,
                      error.c_str());
    server->next_attempt =
        ros::Time::now() + ros::Duration(config_.reconnect_interval);
    return false;
  }

  ROS_INFO("已连接 %s (%s:%d)", server->config.name.c_str(),
           server->config.host.c_str(), server->config.port);
  return true;
}

// 读取一台服务器的保持寄存器。
//
// 失败原因可能是应答超时、对端关闭连接或 Modbus 异常帧；无论哪种都关闭
// socket，让下一轮通过 ensureConnected() 重新建连，避免在坏连接上反复超时。
// 返回值只表示本次读取是否成功，重连节流在上层统一处理。
bool UltrasonicRadarNode::readServerRegisters(
    ServerRuntime* server, std::vector<uint16_t>* registers) {
  std::string error;
  if (!server->client->readHoldingRegisters(
          static_cast<uint8_t>(server->config.slave_id),
          static_cast<uint16_t>(config_.start_register),
          static_cast<uint16_t>(config_.probe_count), registers,
          config_.timeout, &error)) {
    ROS_WARN_THROTTLE(5.0, "读取 %s 失败: %s", server->config.name.c_str(),
                      error.c_str());
    server->client->close();
    server->next_attempt =
        ros::Time::now() + ros::Duration(config_.reconnect_interval);
    return false;
  }
  return true;
}

// ==================== 消息组装 ====================

// 把寄存器数组转换成 ROS 消息。
//
// 探头映射（现场接线固定为「前、侧1、侧2、后」）：
//   1 号 → Front   2 号 → Side1
//   3 号 → Side2   4 号 → Back
// 左右侧由服务器 name（left_radar_502 / right_radar_503）区分，
// 同一套字段名左右两侧通用。
//
// 正常情况下 registers.size() == probe_count；若数量不足（协议异常或配置
// 改成少于 4 路），缺失的探头填 invalid_distance，避免数组越界，也避免
// 发布 0 被下游误判成"贴脸障碍物"。
ultrasonic_radar_data UltrasonicRadarNode::buildData(
    const ServerConfig& config, const std::vector<uint16_t>& registers) const {
  const auto distance_at = [&](size_t index) -> float {
    if (index >= registers.size()) {
      return static_cast<float>(config_.invalid_distance);
    }
    return toDistance(registers[index]);
  };

  ultrasonic_radar_data data;
  data.name = config.name;
  data.Front = distance_at(0);
  data.Side1 = distance_at(1);
  data.Side2 = distance_at(2);
  data.Back = distance_at(3);
  return data;
}

// 单路距离换算：mm → m。
//
// DYP 电应普超声波雷达模组原始单位为 mm，读到 0 表示该路无有效回波（未接探头、无目标、
// 超量程），是正常读数而不是通讯故障。
// 阈值判断用严格大于：恰好 0.1 m 也按无效处理，与旧 DYP 电应普节点保持一致。
// 无效时返回 -1.0 m，订阅方可以用 < 0 统一判断"该路无目标"。
float UltrasonicRadarNode::toDistance(uint16_t millimeters) const {
  const double meters = static_cast<double>(millimeters) / 1000.0;
  return static_cast<float>(meters > config_.min_valid_distance
                                ? meters
                                : config_.invalid_distance);
}

// 打印本轮的原始寄存器值和换算后的四路距离。
//
// 原始值保留 mm，便于直接和 Modbus 调试工具（或串口服务器网页）对照；
// 换算值保留 3 位小数，正好对应 mm 精度。输出示例：
//   [left_radar_502] 1号=558mm 2号=452mm 3号=371mm 4号=640mm |
//   Front=0.558m Side1=0.452m Side2=0.371m Back=0.640m
void UltrasonicRadarNode::printRadarData(
    const ultrasonic_radar_data& data,
    const std::vector<uint16_t>& registers) const {
  std::ostringstream raw;
  for (size_t i = 0; i < registers.size(); ++i) {
    if (i > 0) raw << " ";
    raw << (i + 1) << "号=" << registers[i] << "mm";
  }

  ROS_INFO("[%s] %s | Front=%.3fm Side1=%.3fm Side2=%.3fm Back=%.3fm",
           data.name.c_str(), raw.str().c_str(), data.Front, data.Side1,
           data.Side2, data.Back);
}

}  // namespace ultrasonic_radar_modbus_tcp
