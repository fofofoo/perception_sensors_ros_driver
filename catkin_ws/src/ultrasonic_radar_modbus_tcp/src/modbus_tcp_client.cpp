// Modbus TCP 客户端实现：POSIX socket + 手写 MBAP/PDU。
//
// 只实现功能码 0x03（读保持寄存器）。请求帧固定 12 字节：
//   事务号(2) 协议号(2)=0 长度(2)=6 单元标识(1) 功能码(1) 起始寄存器(2) 数量(2)
// 应答由 receiveResponse() 从接收缓冲区里按 MBAP 长度切帧，校验事务号、
// 单元标识、功能码和字节数；非法帧逐字节丢弃后重同步。
//
// 读失败一律关闭 socket，由调用方（节点）按 reconnect_interval 重连。

#include "ultrasonic_radar_modbus_tcp/modbus_tcp_client.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>

namespace ultrasonic_radar_modbus_tcp {
namespace {

// 单调时钟，单位秒。
// 用于计算 connect / poll / recv 的截止时间：不受系统时间调整影响，
// 也不会像 ros::Time 那样依赖 ROS 时间源（仿真或 /clock 关闭时仍然可靠）。
double nowSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

// 构造时不建立连接、不分配 socket：连接由 connectTo() 显式发起，
// 便于节点先完成参数校验，再决定探测哪些服务器。
ModbusTcpClient::ModbusTcpClient()
    : fd_(-1), transaction_id_(0), pending_transaction_id_(0), debug_(false) {}

// 析构兜底关闭 fd，避免调用方忘记 close() 时泄漏 socket。
ModbusTcpClient::~ModbusTcpClient() { close(); }

// ==================== 连接管理 ====================

// 建立到 host:port 的 TCP 连接，timeout_sec 是整体连接超时（含 DNS 解析）。
//
// 流程：getaddrinfo 解析出所有候选地址 → 逐个尝试 → 非阻塞 connect + poll
// 等待可写，再用 SO_ERROR 判断真实结果。
// 之所以用非阻塞 connect：网络不可达时阻塞 connect 可能卡住几十秒，会直接
// 拖垮节点的轮询周期；poll 能把等待上限控制在 timeout_sec 内。
// 任一地址连接成功即保存 fd；全部失败返回 false，error 带回最后一次错误。
bool ModbusTcpClient::connectTo(const std::string& host, uint16_t port,
                                double timeout_sec, std::string* error) {
  // 重复连接前先清理旧 socket 和接收缓冲，保证从干净状态开始。
  close();

  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;   // IPv4 / IPv6 都接受，由系统解析结果决定
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* result = nullptr;
  const std::string port_text = std::to_string(port);
  const int gai =
      ::getaddrinfo(host.c_str(), port_text.c_str(), &hints, &result);
  if (gai != 0) {
    if (error) *error = std::string("getaddrinfo: ") + ::gai_strerror(gai);
    return false;
  }

  // 逐个候选地址尝试；只有全部失败才把 last_error 返回给调用方。
  std::string last_error = "no usable address";
  for (struct addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
    const int sock = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock < 0) {
      last_error = std::string("socket: ") + std::strerror(errno);
      continue;
    }

    // 改为非阻塞：connect 立即返回 EINPROGRESS，再交给下面的 poll 等待。
    const int flags = ::fcntl(sock, F_GETFL, 0);
    if (flags < 0 || ::fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
      last_error = std::string("fcntl: ") + std::strerror(errno);
      ::close(sock);
      continue;
    }

    // Modbus 请求/应答都是几十字节的小包，关闭 Nagle 可避免额外延迟。
    int one = 1;
    ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    int ret = ::connect(sock, ai->ai_addr, ai->ai_addrlen);
    if (ret < 0 && errno != EINPROGRESS) {
      // 明确的失败（拒绝连接、网络不可达等），换下一个候选地址。
      last_error = std::string("connect: ") + std::strerror(errno);
      ::close(sock);
      continue;
    }

    if (ret < 0) {
      // EINPROGRESS：连接仍在进行，在截止时间内等 socket 可写。
      const double deadline = nowSeconds() + timeout_sec;
      bool connected = false;
      while (true) {
        const double remaining = deadline - nowSeconds();
        if (remaining <= 0.0) {
          last_error = "connect timeout";
          break;
        }

        // poll 用毫秒；向上取整到 1 ms，避免剩余时间不足 1 ms 时忙等。
        int timeout_ms = static_cast<int>(remaining * 1000.0);
        if (timeout_ms <= 0) timeout_ms = 1;

        struct pollfd pfd;
        pfd.fd = sock;
        pfd.events = POLLOUT;
        pfd.revents = 0;
        const int poll_ret = ::poll(&pfd, 1, timeout_ms);
        if (poll_ret > 0) {
          // 可写不等于连接成功，必须用 SO_ERROR 取真实错误码：
          // 0 表示成功，非 0 则是 ECONNREFUSED / EHOSTUNREACH 等。
          int so_error = 0;
          socklen_t so_len = sizeof(so_error);
          if (::getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &so_len) <
              0) {
            last_error = std::string("getsockopt: ") + std::strerror(errno);
          } else if (so_error != 0) {
            last_error = std::string("connect: ") + std::strerror(so_error);
          } else {
            connected = true;
          }
          break;
        }
        if (poll_ret == 0) {
          last_error = "connect timeout";
          break;
        }
        // 被信号打断不算失败，重新计算剩余时间继续等。
        if (errno != EINTR) {
          last_error = std::string("poll: ") + std::strerror(errno);
          break;
        }
      }

      if (!connected) {
        ::close(sock);
        continue;
      }
    }

    // 连接成功：保存 fd，停止遍历其他候选地址。
    fd_ = sock;
    break;
  }

  ::freeaddrinfo(result);

  if (fd_ < 0) {
    if (error) *error = last_error;
    return false;
  }

  // 新连接从干净状态开始，避免上一条连接的旧事务号或半帧干扰解析。
  transaction_id_ = 0;
  pending_transaction_id_ = 0;
  rx_buffer_.clear();
  return true;
}

// 关闭连接并清空接收缓冲，可重复调用（fd_ 已为 -1 时只清缓冲）。
//
// 接收缓冲必须一起清掉：否则重连后可能把上一条连接残留的字节当成本次应答，
// 造成事务号/帧长错位。调用方无需区分当前是否已连接。
void ModbusTcpClient::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rx_buffer_.clear();
}

// 在 timeout_sec 内等待 fd_ 进入 events 指定的状态（POLLIN / POLLOUT）。
//
// 返回 true 表示关注的事件已就绪；false 表示超时、fd 无效或 poll 出错。
// 被信号打断（EINTR）时重新计算剩余时间继续等，不把信号当作连接失败。
// 错误/挂断（POLLERR/POLLHUP）也会走到这里返回 false，由后续 recv/send
// 或关闭逻辑给出具体原因。
bool ModbusTcpClient::waitFor(short events, double timeout_sec) {
  if (fd_ < 0) return false;

  const double deadline = nowSeconds() + timeout_sec;
  while (true) {
    const double remaining = deadline - nowSeconds();
    if (remaining <= 0.0) return false;

    int timeout_ms = static_cast<int>(remaining * 1000.0);
    if (timeout_ms <= 0) timeout_ms = 1;

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = events;
    pfd.revents = 0;

    const int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret > 0) {
      // 对端关闭时仍可能带着最后一批数据，优先让 recv 取走。
      if ((pfd.revents & events) != 0) return true;
      return false;
    }
    if (ret == 0) return false;
    if (errno != EINTR) return false;
  }
}

// ==================== 收发 ====================

// 在 timeout_sec 内把 length 字节全部写出去。
//
// send 可能只写一部分（尤其是缓冲区紧张时），所以循环到发完为止；每次发送前
// 用 poll 等可写，整体共享同一个截止时间。
// MSG_NOSIGNAL：对端已关闭时返回 EPIPE，而不是触发 SIGPIPE 直接杀掉进程。
// EAGAIN/EWOULDBLOCK/EINTR 属于暂时状态，继续等剩余时间即可。
bool ModbusTcpClient::sendAll(const uint8_t* data, size_t length,
                              double timeout_sec, std::string* error) {
  size_t sent = 0;
  const double deadline = nowSeconds() + timeout_sec;

  while (sent < length) {
    const double remaining = deadline - nowSeconds();
    if (!waitFor(POLLOUT, remaining)) {
      if (error) *error = "send timeout";
      return false;
    }

    const ssize_t n =
        ::send(fd_, data + sent, length - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n == 0) {
      // 发送 0 字节不是正常状态，直接按失败处理，避免死循环。
      if (error) *error = "send returned 0";
      return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;

    if (error) *error = std::string("send: ") + std::strerror(errno);
    return false;
  }

  return true;
}

// ==================== 请求与应答 ====================

// 读取从 start_register 开始的 register_count 个保持寄存器（功能码 0x03）。
//
// 请求固定 12 字节 = MBAP 头 7 字节 + PDU 5 字节，网口侧不带 CRC：
//   事务号(2) 协议号(2)=0 长度(2)=6 单元标识(1) 功能码(1)
//   起始寄存器(2) 寄存器数量(2)
// register_count 上限 125 是 Modbus 对 0x03 的规范限制（应答 PDU 最大 256 字节）。
// 事务号每次自增，用于把应答和本次请求对应起来。
// 发送或解析失败一律 close()：连接可能已半死，继续用只会在后续轮询里反复超时，
// 交给上层按 reconnect_interval 重新建连。
bool ModbusTcpClient::readHoldingRegisters(
    uint8_t slave_id, uint16_t start_register, uint16_t register_count,
    std::vector<uint16_t>* values, double timeout_sec, std::string* error) {
  if (values == nullptr) {
    if (error) *error = "values is null";
    return false;
  }
  // 失败时保持空数组，避免调用方读到上一次的旧值。
  values->clear();

  if (fd_ < 0) {
    if (error) *error = "not connected";
    return false;
  }
  // 0 个寄存器没有意义；超过 125 超出单帧应答容量。
  if (register_count == 0 || register_count > 125) {
    if (error) *error = "invalid register count";
    return false;
  }

  // 记录本次事务号：应答必须带回同一个值，否则视为迟到/串路的旧帧。
  pending_transaction_id_ = ++transaction_id_;

  uint8_t request[12];
  // —— MBAP 头 ——
  request[0] = static_cast<uint8_t>(pending_transaction_id_ >> 8);
  request[1] = static_cast<uint8_t>(pending_transaction_id_ & 0xFF);
  request[2] = 0;  // Protocol ID 高字节，Modbus 固定为 0
  request[3] = 0;  // Protocol ID 低字节
  request[4] = 0;  // Length 高字节：Unit ID + PDU = 6
  request[5] = 6;
  request[6] = slave_id;  // RS485 侧模组的从机地址

  // —— PDU ——
  request[7] = 0x03;  // 读保持寄存器
  request[8] = static_cast<uint8_t>(start_register >> 8);
  request[9] = static_cast<uint8_t>(start_register & 0xFF);
  request[10] = static_cast<uint8_t>(register_count >> 8);
  request[11] = static_cast<uint8_t>(register_count & 0xFF);

  logBytes("TX", request, sizeof(request));

  // 发送失败说明连接已不可用，关闭后由上层重连。
  if (!sendAll(request, sizeof(request), timeout_sec, error)) {
    close();
    return false;
  }

  // 应答超时、格式错误或异常码同样说明本次交互不可信，关闭连接。
  if (!receiveResponse(slave_id, register_count, values, timeout_sec, error)) {
    close();
    return false;
  }

  return true;
}

// 等待并解析一条 0x03 应答，把寄存器值按大端序写入 values。
//
// 这里要处理 TCP 字节流的两个现实问题：
//   1. 半帧：一次 recv 可能只收到 MBAP 头的一部分或部分数据，必须累积到
//      rx_buffer_，再按 MBAP 的 Length 字段判断整帧是否到齐；
//   2. 粘包/残帧：可能一次收到多帧，或上一轮超时后迟到的旧应答，需要按
//      事务号挑出属于本次请求的帧，其余整帧丢弃。
// 帧头本身不合法（协议号非 0、长度越界）时逐字节丢弃，重新寻找帧起点。
// 单元标识、功能码、异常码、字节数任一不符都返回 false；调用方会关闭连接。
// 超时按整条应答共享一个截止时间，不会因为陆续收到碎片而无限续期。
bool ModbusTcpClient::receiveResponse(uint8_t slave_id,
                                      uint16_t register_count,
                                      std::vector<uint16_t>* values,
                                      double timeout_sec, std::string* error) {
  const double deadline = nowSeconds() + timeout_sec;

  while (true) {
    // 先在已有缓冲区里找完整、合法的应答帧；不足一帧时跳出，继续 recv。
    // MBAP 头固定 7 字节：事务号(2) 协议号(2) 长度(2) 单元标识(1)。
    while (rx_buffer_.size() >= 7) {
      const uint16_t protocol_id =
          (static_cast<uint16_t>(rx_buffer_[2]) << 8) | rx_buffer_[3];
      const uint16_t length =
          (static_cast<uint16_t>(rx_buffer_[4]) << 8) | rx_buffer_[5];

      // 协议号必须为 0；Length 至少覆盖 Unit ID + 功能码，上限 260 是
      // Modbus TCP 单帧的最大长度。不合法说明帧起点错了，丢 1 字节重同步。
      if (protocol_id != 0 || length < 3 || length > 260) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      // MBAP Length 不包含前 6 字节（事务号 + 协议号 + 长度本身）。
      const size_t total = 6 + length;
      if (rx_buffer_.size() < total) break;  // 帧还没收全，继续 recv

      const uint16_t transaction =
          (static_cast<uint16_t>(rx_buffer_[0]) << 8) | rx_buffer_[1];
      if (transaction != pending_transaction_id_) {
        // 超时后迟到的旧应答，整帧丢弃后继续找。
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total);
        continue;
      }

      // 事务号已匹配：先把整帧取出来，后续校验失败也不会污染接收缓冲。
      const uint8_t unit_id = rx_buffer_[6];
      const uint8_t function = rx_buffer_[7];
      const std::vector<uint8_t> frame(rx_buffer_.begin(),
                                       rx_buffer_.begin() + total);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total);
      logBytes("RX", frame.data(), frame.size());

      // 单元标识必须是本次寻址的从机。M04 在某通道未接模组时可能返回
      // 单元标识 0 或网关异常帧，据此可以判定该端口不可用。
      if (unit_id != slave_id) {
        if (error) {
          *error = "unexpected unit id " + std::to_string(unit_id);
        }
        return false;
      }

      // 0x83 = 0x03 | 0x80，表示从机返回异常应答；第 9 字节是异常码
      // （如 0x02 非法数据地址、0x04 从机故障）。
      if (function == 0x83) {
        const uint8_t code = frame.size() > 8 ? frame[8] : 0;
        if (error) {
          *error = "modbus exception " + std::to_string(code);
        }
        return false;
      }

      if (function != 0x03) {
        if (error) {
          *error = "unexpected function " + std::to_string(function);
        }
        return false;
      }

      // 正常应答至少包含：功能码(1) + 字节数(1)，帧内偏移 7、8。
      if (frame.size() < 9) {
        if (error) *error = "short response";
        return false;
      }

      // 0x03 应答的字节数必须等于 2×寄存器数量，且数据确实到齐。
      const uint8_t byte_count = frame[8];
      const size_t expected_bytes = 2 * static_cast<size_t>(register_count);
      if (byte_count != expected_bytes || frame.size() < 9 + expected_bytes) {
        if (error) *error = "bad byte count";
        return false;
      }

      // 寄存器值是大端：高字节在前，低字节在后。
      values->resize(register_count);
      for (uint16_t i = 0; i < register_count; ++i) {
        const size_t offset = 9 + 2 * static_cast<size_t>(i);
        (*values)[i] = (static_cast<uint16_t>(frame[offset]) << 8) |
                       frame[offset + 1];
      }
      return true;
    }

    const double remaining = deadline - nowSeconds();
    if (remaining <= 0.0) {
      if (error) *error = "response timeout";
      return false;
    }

    if (!waitFor(POLLIN, remaining)) {
      if (error) *error = "response timeout";
      return false;
    }

    // 一次最多读 256 字节（Modbus TCP 单帧上限）；读到多少追加多少，
    // 不做假设——TCP 不保证一次 recv 对应一帧。
    uint8_t buffer[256];
    const ssize_t n = ::recv(fd_, buffer, sizeof(buffer), 0);
    if (n > 0) {
      rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + n);
      continue;
    }
    if (n == 0) {
      // 对端正常关闭（FIN）：网关重启或连接被回收。
      if (error) *error = "connection closed";
      return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;

    if (error) *error = std::string("recv: ") + std::strerror(errno);
    return false;
  }
}

// 调试打印一帧原始字节，格式：`[modbus_tcp] TX (12): 00 01 ...`。
//
// 只在 debug_frames=true 时输出；写到 stderr 并立即 flush，避免和
// ROS_INFO 的 stdout 缓冲交错，便于与抓包/网关日志按时间对齐排查。
void ModbusTcpClient::logBytes(const char* direction, const uint8_t* data,
                               size_t length) {
  if (!debug_) return;

  std::fprintf(stderr, "[modbus_tcp] %s (%zu):", direction, length);
  for (size_t i = 0; i < length; ++i) {
    std::fprintf(stderr, " %02X", data[i]);
  }
  std::fprintf(stderr, "\n");
  std::fflush(stderr);
}

}  // namespace ultrasonic_radar_modbus_tcp
