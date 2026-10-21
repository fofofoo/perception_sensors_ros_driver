#include "uwb_rtu_serial_demo/rtu_client.h"

#include <iostream>

/**
 * Modbus-RTU 串口客户端实现。
 *
 * 协议依据：D-PDOA-Node-B 基站的「PDOA 定位跟随系统手册」——
 *   4.1 串口参数（115200, 8 数据位, 1 停止位, 无校验）
 *   4.2 功能码 03H 应答帧格式
 *   4.4（1）定位数据帧示例
 *
 * 03H 应答帧（手册 4.2）：
 *   下标   0      1     2      3 ~ 2+LEN      3+LEN   4+LEN
 *   字段   ADR    03H   LEN    DATA           CRC低   CRC高
 *   总长 = 3 + LEN + 2 字节。LEN 是寄存器数据的总字节数，
 *   寄存器按 16 位组织，因此 LEN 总是偶数（定位帧为 0x12，即 18 字节）。
 *
 * 定位帧示例（手册 4.4（1））：
 *   01 03 12 CA DA 96 24 4B 6E 76 00 58 08 FF 55 00 38 00 B5 E8 EA 18 3E
 *   ADR 03 LEN 标识      标签 64 位 ID       X      Y      距离   角度   CRC
 */
namespace uwb_rtu_serial_demo
{

// ==================== 串口管理 ====================

// 按手册 4.1 配置 8N1 并打开串口。打开过程中的异常全部在内部捕获。
// @return true 打开成功，接收缓冲与心跳窗口已重置；false 打开失败（已打印原因）
bool RtuClient::open(const std::string& port, uint32_t baudrate, int timeout_ms)
{
    try
    {
        sp_.setPort(port);
        sp_.setBaudrate(baudrate);
        sp_.setBytesize(serial::eightbits);       // 手册 4.1：8 数据位
        sp_.setStopbits(serial::stopbits_one);    // 手册 4.1：1 停止位
        sp_.setParity(serial::parity_none);       // 手册 4.1：无校验
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
        sp_.setTimeout(timeout);                  // 该接口要求非 const 引用
        sp_.open();
    }
    catch (const std::exception& e)
    {
        std::cerr << "打开串口 " << port << " 失败：" << e.what() << std::endl;
        return false;
    }

    sp_.flushInput();     // 丢掉上次运行残留的字节，避免首帧从半截开始
    rx_buf_.clear();

    // 心跳统计从头开始，避免把上次运行的帧数算进第一个窗口
    frame_count_ = 0;
    window_start_ = std::chrono::steady_clock::now();
    last_frame_time_ = window_start_;
    return true;
}

// 关闭串口并清空接收缓冲、心跳计数。未打开时调用也安全（只做状态复位）。
void RtuClient::close()
{
    if (sp_.isOpen())
    {
        sp_.close();
    }
    rx_buf_.clear();
    frame_count_ = 0;
    window_start_ = std::chrono::steady_clock::now();
    last_frame_time_ = window_start_;
}

// 串口状态查询，不改变任何内部状态。
bool RtuClient::isOpen() const
{
    return sp_.isOpen();
}

// ==================== 收帧 ====================

// 把串口当前可读字节追加到 rx_buf_。非阻塞：没数据直接返回。
// 实际读到的长度以 read() 返回值为准（available() 只是上限，可能少于它）。
void RtuClient::drainSerial()
{
    const size_t n = sp_.available();
    if (n == 0)
    {
        return;
    }

    const size_t old_size = rx_buf_.size();
    rx_buf_.resize(old_size + n);
    const size_t got = sp_.read(rx_buf_.data() + old_size, n);
    rx_buf_.resize(old_size + got);

    // 长时间收到无效数据时只保留最近一段，避免内存无限增长
    if (rx_buf_.size() > kMaxBuffer)
    {
        discard(rx_buf_.size() - kMaxBuffer);
    }
}

// 从缓冲头部丢弃 n 字节；n 不小于当前长度时直接清空，不会越界。
void RtuClient::discard(size_t n)
{
    if (n >= rx_buf_.size())
    {
        rx_buf_.clear();
        return;
    }
    rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + n);
}

// 从字节流中同步出一整帧 Modbus-RTU 应答。
// 步骤：追加串口数据 → 逐字节找候选帧头 → 长度够则校验 CRC →
//       成功则取出整帧并消耗缓冲，失败则丢 1 字节继续找。
// @param frame 输出完整帧（ADR ~ CRC 高字节），仅在返回 true 时有效
// @return true 取到一帧；false 当前没有完整且 CRC 正确的帧（半帧保留在缓冲）
bool RtuClient::readFrame(std::vector<uint8_t>& frame)
{
    frame.clear();
    if (!sp_.isOpen())
    {
        return false;
    }

    drainSerial();

    // 注意：基站主动上报都用 03H（手册 4.2），既包括定位帧（LEN=0x12），
    // 也包括新标签上报帧（LEN=0x0A）。本层只保证「整帧 + CRC 正确」，
    // 具体是哪类帧由 UwbNode 按 CA DA 标识判断。
    //
    // 从缓冲头部逐字节找候选帧头：ADR(1~247) + 功能码 03H + 偶数 LEN。
    // ADR 范围来自手册 4.2「从机地址码 001~255」，Modbus 规范有效地址是 1~247。
    size_t i = 0;
    while (i + 2 < rx_buf_.size())
    {
        const uint8_t addr = rx_buf_[i];
        const uint8_t func = rx_buf_[i + 1];
        const uint8_t len  = rx_buf_[i + 2];

        const bool plausible = addr >= 1 && addr <= 247
                               && func == kFuncRead
                               && len > 0
                               && (len % 2 == 0)
                               && len <= kMaxPayload;
        if (!plausible)
        {
            ++i;
            continue;
        }

        const size_t total = 3 + static_cast<size_t>(len) + 2;   // ADR 功能码 LEN DATA CRC×2
        if (rx_buf_.size() - i < total)
        {
            // 帧还没收全：丢掉候选帧头之前的垃圾字节，保留半帧等下次拼接
            discard(i);
            return false;
        }

        // 手册 4.2：CRC 覆盖从 ADR 到最后一个数据字节（不含 CRC 本身），
        // 校验和的低字节先发、高字节后发。
        const uint16_t crc_calc = crc16(&rx_buf_[i], 3 + static_cast<size_t>(len));
        const uint16_t crc_recv = static_cast<uint16_t>(
            rx_buf_[i + 3 + len] | (rx_buf_[i + 4 + len] << 8));
        if (crc_calc == crc_recv)
        {
            frame.assign(rx_buf_.begin() + i, rx_buf_.begin() + i + total);
            discard(i + total);

            // 心跳：只有 CRC 通过的整帧才算一次有效接收
            ++frame_count_;
            last_frame_time_ = std::chrono::steady_clock::now();
            return true;
        }

        // CRC 不符：该位置不是真帧头，丢 1 字节重新同步
        ++i;
    }

    // 找不到任何候选帧头：只保留末尾 2 字节，帧头可能被拆到两次读取之间
    if (rx_buf_.size() > 2)
    {
        discard(rx_buf_.size() - 2);
    }
    return false;
}

// 发送接口只负责原样写出，帧格式由调用方按手册 4.2 组：
//   03H 读寄存器、06H 写单寄存器、10H 写多寄存器，末尾都要附 CRC 低/高字节。
// @param data 完整请求帧（含 CRC）；空数据或串口未打开时返回 false
// @return true 全部字节写出；false 未打开 / 空数据 / 写异常
bool RtuClient::write(const std::vector<uint8_t>& data)
{
    if (!sp_.isOpen() || data.empty())
    {
        return false;
    }
    try
    {
        return sp_.write(data.data(), data.size()) == data.size();
    }
    catch (const std::exception& e)
    {
        std::cerr << "串口写失败：" << e.what() << std::endl;
        return false;
    }
}

// ==================== 心跳 ====================

// 按 1 秒窗口结算收帧频率；窗口未满时返回 false，不影响计数。
// @param hz 输出最近一个窗口的频率，单位 Hz；仅在返回 true 时有效
// @return true 完成一次结算（hz 已更新、计数清零）；false 窗口未满
bool RtuClient::updateHeartbeat(uint32_t& hz)
{
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - window_start_).count();
    if (elapsed_ms < 1000)
    {
        return false;   // 窗口未满：继续累积
    }

    // 用实际经过的时间换算，避免主循环抖动导致窗口不是整 1 秒
    hz = static_cast<uint32_t>(frame_count_ * 1000 / elapsed_ms);
    frame_count_ = 0;
    window_start_ = now;
    return true;
}

// 链路存活判断：最近 timeout_ms 内是否收到过 CRC 正确的整帧。
// @return true 存活；false 串口未打开或超时无帧
// 注意阈值要大于标签静态休眠间隔，否则静止时会被误判为掉线。
bool RtuClient::isAlive(int timeout_ms) const
{
    if (!sp_.isOpen())
    {
        return false;
    }
    return std::chrono::steady_clock::now() - last_frame_time_
           <= std::chrono::milliseconds(timeout_ms);
}

// ==================== CRC16 ====================

// Modbus-RTU CRC16：初值 0xFFFF，反向多项式 0xA001（即 0x8005 的位反转），
// 逐字节异或后右移 8 位。与手册 4.4（1）示例帧的计算结果一致。
uint16_t RtuClient::crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;   // Modbus-RTU 固定初值
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001)
            {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

}  // namespace uwb_rtu_serial_demo
