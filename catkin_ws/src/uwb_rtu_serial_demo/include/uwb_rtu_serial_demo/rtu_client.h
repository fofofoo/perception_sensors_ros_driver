#pragma once

#include <serial/serial.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace uwb_rtu_serial_demo
{

/**
 * @brief 通用 Modbus-RTU 串口客户端：只负责收发与成帧，不含 UWB 业务。
 *
 * 职责边界
 *   - 管理串口打开 / 关闭（8N1，基站固定 115200）；
 *   - 把收到的字节累积进内部缓冲，按 Modbus-RTU 应答格式同步出一整帧；
 *   - 校验 CRC16，CRC 不符时丢弃 1 字节重新找帧头，可自愈半帧、粘包和垃圾字节；
 *   - 提供原始发送接口（预留 03H/06H/10H 等配置指令使用）。
 *
 * 本类不依赖 ROS，可被其他节点直接复用。
 *
 * 应答帧格式（基站主动上报，功能码 03H）：
 *   ADR(1) 03H(1) LEN(1) DATA(LEN) CRC低(1) CRC高(1)，总长 3 + LEN + 2
 *   其中 LEN 为寄存器数据字节数，必须为偶数（手册 4.2 / 4.4）。
 */
class RtuClient
{
public:
    RtuClient() = default;

    /**
     * @brief 配置并打开串口。
     *
     * @param port       串口设备路径，如 /dev/ttyUWB（main 提供）
     * @param baudrate   波特率，D-PDOA-Node-B 固定 115200（手册 4.1）
     * @param timeout_ms 读超时，单位 ms，默认 1000
     * @return true 打开成功；false 打开失败（异常已在内部捕获并打印原因）
     */
    bool open(const std::string& port, uint32_t baudrate, int timeout_ms = 1000);

    /// 关闭串口并清空接收缓冲；重复调用安全。
    /// 设备已拔出时 close 可能抛异常，内部捕获，保证退出路径不崩溃。
    void close();

    /// 串口是否已打开。
    bool isOpen() const;

    /**
     * @brief 重新打开串口。
     *
     * 用最近一次 open() 记录的 port / baudrate / timeout 重新打开：
     * 先关闭旧句柄（设备已拔出时 close 可能抛异常，内部捕获），
     * 清空接收缓冲与心跳窗口，再调用 open()。
     * 主要供 main 在启动失败或运行中链路失联时按节流周期调用。
     *
     * @return true 重新打开成功；false 未记录设备路径或 open 失败（原因已打印）
     */
    bool reconnect();

    /// 最近一次串口读写是否发生异常；open()/reconnect() 成功后清除。
    bool linkError() const { return link_error_; }

    /**
     * @brief 尝试取出一整帧 Modbus-RTU 应答。
     *
     * 非阻塞：当前没有完整帧时立即返回 false，已收到的半帧保留在内部缓冲，
     * 等后续字节到达再拼帧。CRC 校验通过才返回，返回的 frame 含 CRC 两字节。
     *
     * @param frame [out] 完整帧（ADR ~ CRC高）
     * @return true 取到一帧；false 当前没有完整且 CRC 正确的帧
     */
    bool readFrame(std::vector<uint8_t>& frame);

    /**
     * @brief 原始发送，调用方自行组帧并附带 CRC。
     *
     * @param data 待发送字节，低字节在前
     * @return true 全部写出；false 串口未打开或写失败
     */
    bool write(const std::vector<uint8_t>& data);

    /// 计算 Modbus-RTU CRC16：多项式 0xA001，初值 0xFFFF，发送时低字节在前。
    static uint16_t crc16(const uint8_t* data, size_t len);

    /**
     * @brief 心跳统计：按 1 秒窗口统计成功收到的整帧数。
     *
     * 每次调用检查距上次结算是否已满 1 秒；满则把窗口内帧数换算成 Hz 写入 hz 并清零。
     * 只统计 CRC 校验通过的整帧，不区分业务类型。
     *
     * @param hz [out] 本窗口的收帧频率，仅在返回 true 时有效
     * @return true 本次完成了一个统计窗口（可以打印/使用 hz）；false 窗口未满
     */
    bool updateHeartbeat(uint32_t& hz);

    /**
     * @brief 链路心跳：最近 timeout_ms 内是否收到过 CRC 通过的整帧。
     *
     * 用于判断基站是否还在正常推送。注意标签静止时会进入休眠
     * （默认 10 s 才发一包），超时阈值应大于标签静态发包间隔，否则会误报。
     *
     * @param timeout_ms 超时时间，单位 ms
     * @return true 链路存活；false 串口未打开、读写异常或超时无数据
     */
    bool isAlive(int timeout_ms) const;

private:
    /// 把串口当前可读字节追加到 rx_buf_，并限制缓冲上限。
    void drainSerial();

    /// 从 rx_buf_ 头部丢弃 n 字节。
    void discard(size_t n);

    serial::Serial sp_;                            ///< 底层串口对象
    std::vector<uint8_t> rx_buf_;                  ///< 接收累积缓冲（按字节流处理）

    std::string port_;                             ///< 最近一次 open() 的设备路径，供 reconnect() 使用
    uint32_t baudrate_ = 0;                        ///< 最近一次 open() 的波特率
    int timeout_ms_ = 1000;                        ///< 最近一次 open() 的读超时，ms
    bool link_error_ = false;                      ///< 读写异常标志；open()/reconnect() 成功后清除

    uint32_t frame_count_ = 0;                     ///< 当前 1 秒窗口内的成功收帧数
    std::chrono::steady_clock::time_point window_start_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_frame_time_ = std::chrono::steady_clock::now();

    static constexpr size_t kMaxBuffer = 4096;     ///< 缓冲上限，防止异常数据无限增长
    static constexpr uint8_t kFuncRead = 0x03;     ///< 本客户端只解析 03H 应答
    static constexpr uint8_t kMaxPayload = 250;    ///< Modbus 单帧数据字节上限（偶数）
};

}  // namespace uwb_rtu_serial_demo
