#include <locale.h>
#include <signal.h>

#include <algorithm>
#include <cstdint>

#include <ros/ros.h>

#include "uwb_rtu_serial_demo/rtu_client.h"
#include "uwb_rtu_serial_demo/uwb_data.h"
#include "uwb_rtu_serial_demo/uwb_node.h"

/**
 * UWB 定位数据串口节点（D-PDOA-Node-B 基站，Modbus-RTU over TTL UART）。
 *
 * 数据流：
 *   基站主动推送 03H 应答帧 → RtuClient 成帧 + CRC 校验 → UwbNode 过滤 CA DA定位帧并换算单位 
 *    → main 发布 /uwb_position（uwb_rtu_serial_demo/uwb_data）。
 *
 * 本节点是「主动轮询型」：串口只读不写。基站定位成功后自动输出坐标，
 * PC 无需发送任何指令开启数据输出（手册 4.4（1））。
 */
// ==================== 节点参数 ====================
// 现场可调项放 launch 全局参数（launch/uwb_node.launch），main 用 nh.param 读取：
//   uwb_serial_port：基站 TTL 转串口设备路径
//   uwb_print_frame：是否逐帧打印定位结果
// 其余参数属于协议固定值或内部调优值，直接写在 main 里。
const uint32_t kBaudrate           = 115200;   // 手册 4.1：固定 115200 8N1，不要改
const double   kLoopRateHz         = 30.0;     // 串口轮询频率，Hz；为标签默认 10 Hz / 上限 20 Hz 的 1.5~3 倍
const int      kHeartbeatTimeoutMs = 15000;    // 无帧告警阈值，需大于标签静态休眠间隔（默认 10 s）

namespace
{

/// Ctrl+C 时让 ros::ok() 返回 false，主循环正常退出并关闭串口。
void mySigintHandler(int /*sig*/)
{
    ros::shutdown();
}

}  // namespace

// 节点入口：打开串口 → 创建解析器与发布者 → 主动轮询并逐帧发布。
// 失败策略：串口打不开直接退出（没有数据源，原地重试没有意义）；
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
    uwb_rtu_serial_demo::RtuClient client;
    if (!client.open(serial_port, kBaudrate))
    {
        ROS_ERROR("串口 %s 打开失败，节点退出", serial_port.c_str());
        return -1;
    }
    ROS_INFO("串口 %s 打开成功（%u 8N1）", serial_port.c_str(), kBaudrate);

    uwb_rtu_serial_demo::UwbNode node(client);

    // 发布定位结果话题，话题名称 uwb_position，消息类型 uwb_rtu_serial_demo/uwb_data。
    // 发布节奏跟帧走（收一帧发一帧），默认约 10 Hz；队列 10 约等于 1 秒缓存。
    ros::Publisher pub = nh.advertise<uwb_rtu_serial_demo::uwb_data>("/perception_sensors/uwb_position", 10);

    // —— 三、心跳：1 秒频率窗口由 RtuClient 内部结算 ——
    uint32_t frequency = 0;      // 最近一个 1 秒窗口内的收帧数，写入消息 fre 字段
    bool stale_warned = false;   // 链路超时告警只打一次，恢复后复位

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

        // 每秒结算一次收帧频率，并检查基站是否还在推送
        if (client.updateHeartbeat(frequency))
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

        // 当前节点没有订阅者，spinOnce 保留用于处理 ROS 内部回调与退出流程
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前显式关闭串口，方便下次立即重新打开
    client.close();
    return 0;
}
