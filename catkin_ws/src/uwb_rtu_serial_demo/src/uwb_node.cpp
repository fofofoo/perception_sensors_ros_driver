/**
 * UWB 定位数据解码实现。
 *
 * 本文件只做一件事：把 RtuClient 交来的整帧（CRC 已校验）解码成定位数据。
 *   串口读取 / 成帧 / CRC / 心跳：rtu_client.cpp
 *   ROS 消息填充与发布：main.cpp
 *
 * 处理流程（readPosition）：
 *   1. 向客户端连续取帧；
 *   2. isPositionFrame() 过滤：只保留 23 字节、LEN=0x12、带 CA DA 标识的定位帧，
 *      新标签上报帧（LEN=0x0A、标识 40 AD）等其它 03H 帧直接丢弃；
 *   3. decodeFrame() 按手册 4.4（1）的字节偏移解出 X / Y / 距离 / 角度并换算单位。
 *
 * 帧布局（23 字节）：
 *   0 ADR=01 | 1 功能码=03 | 2 LEN=0x12 | 3~4 CA DA | 5~12 标签 ID |
 *   13~14 X | 15~16 Y | 17~18 距离 | 19~20 角度 | 21~22 CRC
 *
 * 边界说明：
 *   - 输出为基站本地坐标系，不做坐标变换、滤波、平滑；
 *   - 标签 ID（下标 5~12）暂不解析，多标签场景无法区分；
 *   - 帧本身不带时间戳，时间信息由发布方在发布时刻决定。
 */
#include "uwb_rtu_serial_demo/uwb_node.h"

namespace uwb_rtu_serial_demo
{

namespace
{
    // 定位帧常量（手册 4.4（1）），仅本文件使用
    constexpr uint8_t kFuncRead     = 0x03;  ///< 功能码：读寄存器，定位帧固定 03H
    constexpr uint8_t kFrameLength  = 0x12;  ///< 寄存器数据字节数 18（不是整帧 23 字节）
    constexpr uint8_t kFlagCa       = 0xCA;  ///< 定位帧标识高字节（新标签上报帧为 0x40 0xAD）
    constexpr uint8_t kFlagDa       = 0xDA;  ///< 定位帧标识低字节
    // 整帧长度 = ADR(1) + 功能码(1) + LEN(1) + 数据(18) + CRC(2)
    constexpr size_t  kPositionFrameSize = 23;
    constexpr size_t  kOffsetX       = 13;   ///< X 高字节；5~12 为标签 64 位 ID，本节点不使用
    constexpr size_t  kOffsetY       = 15;
    constexpr size_t  kOffsetDist    = 17;
    constexpr size_t  kOffsetAngle   = 19;
    constexpr float   kRawToMeter    = 0.01f;   ///< X/Y/距离：cm -> m
    constexpr float   kRawToDegree   = 0.01f;   ///< 角度：1/100 度 -> 度
}  // namespace

// 绑定串口客户端：只保存引用，不负责打开/关闭串口。
// client 由 main 持有，生命周期必须覆盖本对象，否则 readPosition 会访问悬空引用。
UwbNode::UwbNode(RtuClient& client)
    : client_(client)
{
}

// 取帧 → 过滤 → 解码。取到定位帧立刻返回；其它 03H 帧丢弃后继续取，
// 这样即使新标签上报帧夹在定位帧之间，也不会漏掉后面的定位数据。
// @param position 输出定位结果，仅在返回 true 时有效；返回 false 时不修改
// @return true 取到一帧定位帧；false 当前缓冲里没有定位帧（非阻塞）
bool UwbNode::readPosition(UwbPosition& position)
{
    std::vector<uint8_t> frame;
    while (client_.readFrame(frame))
    {
        if (!isPositionFrame(frame))
        {
            continue;   // 新标签上报等其它 03H 帧：丢弃后继续取下一帧
        }
        decodeFrame(frame, position);
        return true;
    }
    return false;
}

// 定位帧判定：23 字节 + 03H + LEN=0x12 + CA DA。
// 逐项校验是因为同一串口上还会出现新标签上报帧（03H、LEN=0x0A、标识 40 AD），
// 只靠「03H + CRC 正确」无法区分两者。
// @param frame 已通过 CRC 校验的整帧
// @return true 是定位帧；false 其它帧或长度不符
bool UwbNode::isPositionFrame(const std::vector<uint8_t>& frame)
{
    return frame.size() == kPositionFrameSize
           && frame[1] == kFuncRead
           && frame[2] == kFrameLength
           && frame[3] == kFlagCa
           && frame[4] == kFlagDa;
}

// 按手册 4.4（1）解码。各字段均为大端（高字节在前）、占 2 字节：
//   X / Y / 角度：有符号 16 位；距离：无符号 16 位（超过 327 m 也不会变负）。
// 单位统一换算：cm -> m，1/100 度 -> 度。
// @param frame    已确认是定位帧（isPositionFrame 返回 true）的整帧
// @param position 输出参数，四个字段全部覆盖写；不做滤波、不保留历史值
void UwbNode::decodeFrame(const std::vector<uint8_t>& frame, UwbPosition& position)
{
    position.x        = toInt16(frame[kOffsetX],     frame[kOffsetX + 1])     * kRawToMeter;
    position.y        = toInt16(frame[kOffsetY],     frame[kOffsetY + 1])     * kRawToMeter;
    position.distance = toUint16(frame[kOffsetDist], frame[kOffsetDist + 1])  * kRawToMeter;
    position.angle    = toInt16(frame[kOffsetAngle], frame[kOffsetAngle + 1]) * kRawToDegree;
}

// 大端有符号 16 位：hi 为高字节、lo 为低字节，拼成 0~65535 后按补码转成 -32768~32767。
// 用于 X / Y / 角度字段（手册 4.4（1）标注为「有符号」）。
float UwbNode::toInt16(uint8_t hi, uint8_t lo)
{
    int32_t value = (static_cast<int32_t>(hi) << 8) | lo;
    if (value > 32767)
    {
        value -= 65536;   // 两字节补码 -> 有符号数
    }
    return static_cast<float>(value);
}

// 大端无符号 16 位：hi 为高字节、lo 为低字节，结果 0~65535，不做符号扩展。
// 仅用于测距字段（手册 4.4（1）标注为「无符号」），避免距离大于 327 m 时被解析成负数。
float UwbNode::toUint16(uint8_t hi, uint8_t lo)
{
    return static_cast<float>((static_cast<uint32_t>(hi) << 8) | lo);
}

}  // namespace uwb_rtu_serial_demo
