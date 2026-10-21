#ifndef ULTRASONIC_RADAR_MODBUS_TCP_MODBUS_TCP_CLIENT_H_
#define ULTRASONIC_RADAR_MODBUS_TCP_MODBUS_TCP_CLIENT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ultrasonic_radar_modbus_tcp {

// 轻量 Modbus TCP 客户端：只实现功能码 0x03（读保持寄存器）。
// 不依赖 libmodbus，直接使用 POSIX socket。
class ModbusTcpClient {
 public:
  ModbusTcpClient();
  ~ModbusTcpClient();

  ModbusTcpClient(const ModbusTcpClient&) = delete;
  ModbusTcpClient& operator=(const ModbusTcpClient&) = delete;

  // 建立连接。timeout_sec 同时用于 TCP 连接超时。
  bool connectTo(const std::string& host, uint16_t port, double timeout_sec,
                 std::string* error);

  void close();
  bool isConnected() const { return fd_ >= 0; }

  // 读取一段连续的保持寄存器。失败时返回 false，并关闭连接，由调用方重连。
  bool readHoldingRegisters(uint8_t slave_id, uint16_t start_register,
                            uint16_t register_count,
                            std::vector<uint16_t>* values,
                            double timeout_sec, std::string* error);

  // 打印收发的原始字节，便于现场排障。
  void setDebug(bool debug) { debug_ = debug; }

 private:
  bool waitFor(short events, double timeout_sec);
  bool sendAll(const uint8_t* data, size_t length, double timeout_sec,
               std::string* error);
  bool receiveResponse(uint8_t slave_id, uint16_t register_count,
                       std::vector<uint16_t>* values, double timeout_sec,
                       std::string* error);
  void logBytes(const char* direction, const uint8_t* data, size_t length);

  int fd_;
  uint16_t transaction_id_;
  uint16_t pending_transaction_id_;
  std::vector<uint8_t> rx_buffer_;
  bool debug_;
};

}  // namespace ultrasonic_radar_modbus_tcp

#endif  // ULTRASONIC_RADAR_MODBUS_TCP_MODBUS_TCP_CLIENT_H_
