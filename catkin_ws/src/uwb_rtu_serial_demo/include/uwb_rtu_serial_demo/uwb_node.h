#pragma once

#include <cstdint>
#include <vector>

#include "uwb_rtu_serial_demo/rtu_client.h"

namespace uwb_rtu_serial_demo
{

/// UWB 定位结果，长度单位 m，角度单位 度，坐标系为基站本地坐标系。
struct UwbPosition
{
    float x        = 0.0f;   ///< 标签 X 坐标，m（原始 cm / 100）
    float y        = 0.0f;   ///< 标签 Y 坐标，m（原始 cm / 100）
    float distance = 0.0f;   ///< 基站与标签测距，m（原始 cm / 100，无符号）
    float angle    = 0.0f;   ///< 标签角度，度（原始 100 度 / 100）
};

/**
 * @brief UWB 基站定位帧解析：把 RTU 客户端取到的整帧解码成 UwbPosition。
 *
 * 职责边界
 *   - 过滤非定位帧（如 LEN=0x0A 的新标签上报帧）；
 *   - 校验定位帧标识 CA DA；
 *   - 按手册 4.4（1）解码 X / Y / 距离 / 角度，并统一换算成 m 与 度。
 *
 * 收帧频率与链路存活由 RtuClient 的心跳接口负责，本类不重复统计。
 *
 * 本类不依赖 ROS；CRC 已由 RtuClient 校验，这里只做业务层校验。
 *
 * 定位帧布局（23 字节，下标从 0 开始）：
 *   0     ADR = 01
 *   1     功能码 = 03
 *   2     寄存器数据字节数 = 0x12
 *   3~4   定位标识 = CA DA
 *   5~12  标签 64 位 ID
 *   13~14 X 坐标，有符号 16 位，cm
 *   15~16 Y 坐标，有符号 16 位，cm
 *   17~18 测距，无符号 16 位，cm
 *   19~20 角度，有符号 16 位，单位 1/100 度
 *   21~22 CRC16
 */
class UwbNode
{
public:
    /**
     * @brief 绑定一个已打开的 RTU 客户端。
     * @param client 生命周期由 main 管理，必须比本对象活得久
     */
    explicit UwbNode(RtuClient& client);

    /**
     * @brief 读取并解析下一帧有效定位数据。
     *
     * 会连续从客户端取帧，跳过非定位帧，直到取到定位帧或暂时无帧可读。
     *
     * @param position [out] 解析结果，仅在返回 true 时有效
     * @return true 取到一帧有效定位帧；false 当前没有可用的定位帧
     */
    bool readPosition(UwbPosition& position);

private:
    /// 是否为 UWB 定位帧：长度为 23 字节，LEN=0x12 且带 CA DA 标识。
    static bool isPositionFrame(const std::vector<uint8_t>& frame);

    /// 按手册 4.4（1）解码一帧定位数据。
    static void decodeFrame(const std::vector<uint8_t>& frame, UwbPosition& position);

    /// 大端有符号 16 位（高字节在前）。
    static float toInt16(uint8_t hi, uint8_t lo);

    /// 大端无符号 16 位（高字节在前），用于测距字段。
    static float toUint16(uint8_t hi, uint8_t lo);

    RtuClient& client_;              ///< 串口客户端（不持有）
};

}  // namespace uwb_rtu_serial_demo
