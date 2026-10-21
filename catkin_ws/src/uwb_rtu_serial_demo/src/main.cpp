#include <locale.h>
#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cstdint>

#include <ros/ros.h>

#include "uwb_rtu_serial_demo/rtu_client.h"
#include "uwb_rtu_serial_demo/uwb_data.h"
#include "uwb_rtu_serial_demo/uwb_node.h"

/**
 * UWB 定位数据串口节点（D-PDOA-Node-B 基站，Modbus-RTU over TTL UART）。
 *
 * 数据流：
 *   基站主动推送 03H 应答帧 → RtuClient 成帧 + CRC 校验 → UwbNode 过滤 CA DA 定位帧并换算单位
 *    → main 发布 /perception_sensors/uwb_position（uwb_rtu_serial_demo/uwb_data）。
 *
 * 本节点是「主动轮询型」：串口只读不写。基站定位成功后自动输出坐标，
 * PC 无需发送任何指令开启数据输出（手册 4.4（1））。
 */
// ==================== 节点参数 ====================
// 现场可调项放 launch 全局参数（launch/uwb_node.launch），main 用 nh.param 读取：
//   uwb_serial_port：基站 TTL 转串口设备路径
//   uwb_print_frame：是否逐帧打印定位结果
// 其余参数属于协议固定值或内部调优值，直接写在 main 里。
const uint32_t kBaudrate            = 115200;   // 手册 4.1：固定 115200 8N1，不要改
const double   kLoopRateHz          = 30.0;     // 串口轮询频率，Hz；为标签默认 10 Hz / 上限 20 Hz 的 1.5~3 倍
const int      kHeartbeatTimeoutMs  = 15000;    // 无帧告警阈值，需大于标签静态休眠间隔（默认 10 s）
const int      kLinkFailTimeoutMs   = 30000;    // 连续无有效帧触发重连的阈值，需大于告警值并留出休眠余量
const double   kStartupRetrySec     = 5.0;      // 启动打不开串口时的重试间隔，秒；首轮也等这个间隔
const double   kRuntimeReconnectSec = 10.0;     // 运行中失联后的最小重连间隔，秒

namespace
{

/// Ctrl+C 时让 ros::ok() 返回 false，主循环正常退出并关闭串口。
void mySigintHandler(int /*sig*/)
{
    ros::shutdown();
}

}  // namespace

// 节点入口：打开串口 → 创建解析器与发布者 → 主动轮询并逐帧发布。
// 失败策略：串口打不开不再退出，节点保持运行并由主循环按 5 s 重试；
//           运行中读写异常或 30 s 无有效帧时，主循环按 10 s 节流重连。
// Ctrl+C 由 mySigintHandler 调用 ros::shutdown，主循环退出后关闭串口。
int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");                     // 终端中文输出
    ros::init(argc, argv, "uwb_node");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    // —— 一、读取现场参数 ——
    // 两个现场可调项放 launch 全局命名空间（launch/uwb_node.launch），
    // 默认值仍在此字面量可见；直接 rosrun 时使用默认值。
    std::string serial_port;
    bool        print_frame;
    nh.param<std::string>("uwb_serial_port", serial_port, "/dev/ttyUWB");
    nh.param<bool>       ("uwb_print_frame", print_frame, true);

    // —— 二、打开串口 ——
    // 打不开也继续运行：单设备掉线不应该拖垮顶层 launch 里的其它传感器；
    // 主循环会用同一个 client 按 kStartupRetrySec 周期重试。
    uwb_rtu_serial_demo::RtuClient client;
    if (!client.open(serial_port, kBaudrate))
    {
        ROS_ERROR("串口 %s 打开失败，节点保持运行，每 %.0f s 重试",
                  serial_port.c_str(), kStartupRetrySec);
    }
    else
    {
        ROS_INFO("串口 %s 打开成功（%u 8N1）", serial_port.c_str(), kBaudrate);
    }

    uwb_rtu_serial_demo::UwbNode node(client);

    // 发布定位结果话题，话题名称 uwb_position，消息类型 uwb_rtu_serial_demo/uwb_data。
    // 发布节奏跟帧走（收一帧发一帧），默认约 10 Hz；队列 10 约等于 1 秒缓存。
    ros::Publisher pub = nh.advertise<uwb_rtu_serial_demo::uwb_data>("/perception_sensors/uwb_position", 10);

    // —— 三、心跳与重连状态 ——
    uint32_t frequency = 0;      // 最近一个 1 秒窗口内的收帧数，写入消息 fre 字段
    bool stale_warned = false;   // 链路超时告警只打一次，恢复后复位

    // link_ever_ok 区分两种失败：启动就没打开（5 s 重试）和运行中掉线（10 s 重连）。
    // 初始值取 open() 的结果；reconnect 成功后置 true。
    bool link_ever_ok = client.isOpen();
    auto next_reconnect = std::chrono::steady_clock::now();
    if (!link_ever_ok)
    {
        next_reconnect += std::chrono::milliseconds(
            static_cast<int64_t>(kStartupRetrySec * 1000.0));
    }

    // —— 四、主动轮询 + 发布 ——
    // 每轮把缓冲里已到达的帧全部取完再发布：正常情况下 30 Hz 轮询 > 10~20 Hz 数据，
    // 每轮 0~1 帧；系统卡顿或标签提频导致积压时也能一次清空，不会留下旧数据。
    ros::Rate loop_rate(kLoopRateHz);
    while (ros::ok())
    {
        uwb_rtu_serial_demo::UwbPosition position;
        while (node.readPosition(position))
        {
            uwb_rtu_serial_demo::uwb_data msg;
            // fre 为 uint8（0~255），实际收帧频率 10~50 Hz，截断分支正常不会触发
            msg.fre      = static_cast<uint8_t>(std::min<uint32_t>(frequency, 255));
            msg.x        = position.x;
            msg.y        = position.y;
            msg.angle    = position.angle;
            msg.distance = position.distance;
            // 消息无 header 字段，时间戳取发布时刻（≈ 帧到达时刻）；下游若需精确时差请自行记录
            pub.publish(msg);

            if (print_frame)
            {
                ROS_INFO("x: %.2f m, y: %.2f m, distance: %.2f m, angle: %.2f deg",
                         position.x, position.y, position.distance, position.angle);
            }
        }

        // 每秒结算一次收帧频率，并检查基站是否还在推送。
        // 读写异常（linkError）和“只是没帧”分开提示：前者不必等满 15 s。
        if (client.updateHeartbeat(frequency))
        {
            if (client.linkError())
            {
                // 串口都没打开时不打印 0 Hz，避免每 5 s 重试期间刷屏
                ROS_WARN_THROTTLE(10.0, "串口未打开或读写异常，等待重连");
                stale_warned = true;
            }
            else
            {
                ROS_INFO("收帧频率：%u Hz", frequency);
                const bool alive = client.isAlive(kHeartbeatTimeoutMs);
                if (!alive && !stale_warned)
                {
                    ROS_WARN("超过 %d ms 未收到有效帧，请检查串口连接、标签是否开机并已在 K 列表中",
                             kHeartbeatTimeoutMs);
                    stale_warned = true;
                }
                else if (alive)
                {
                    stale_warned = false;
                }
            }
        }

        // —— 五、链路探针与重连 ——
        // 外层循环 30 Hz，检测很快；真正 open 按 5 s / 10 s 节流，不会每轮都重连。
        // isAlive() 已包含 link_error_ 判断，拔线异常会立即进入该分支。
        if (!client.isAlive(kLinkFailTimeoutMs))
        {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_reconnect)
            {
                const double interval = link_ever_ok
                                      ? kRuntimeReconnectSec
                                      : kStartupRetrySec;
                next_reconnect = now + std::chrono::milliseconds(
                    static_cast<int64_t>(interval * 1000.0));

                if (client.reconnect())
                {
                    link_ever_ok = true;
                    stale_warned = false;
                    frequency = 0;   // 新窗口尚未结算，避免把断线前的收帧数带进重连后的首帧
                }
            }
        }

        // 当前节点没有订阅者，spinOnce 保留用于处理 ROS 内部回调与退出流程
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前显式关闭串口，方便下次立即重新打开
    client.close();
    return 0;
}
