#include <locale.h>
#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

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
 * 话题：
 *   /perception_sensors/uwb_position —— 定位数据，收一帧发一帧
 *   /perception_sensors/uwb_status   —— 链路状态 std_msgs/Bool，跳变时发布（latched）
 *
 * 本节点是「主动轮询型」：串口只读不写。基站定位成功后自动输出坐标，
 * PC 无需发送任何指令开启数据输出（手册 4.4（1））。
 */
// ==================== 节点参数 ====================
// 现场可调项放 launch 全局参数（launch/uwb_node.launch），main 用 nh.param 读取，
// 下面同名字面量仅作为默认值（直接 rosrun 时生效）：
//   uwb_serial_port            ：基站 TTL 转串口设备路径
//   uwb_print_frame            ：是否逐帧打印定位结果
//   uwb_heartbeat_timeout_ms   ：无帧告警 / 状态话题置 false 的阈值，ms
//   uwb_link_fail_timeout_ms   ：连续无有效帧触发重连的阈值，ms
//   uwb_startup_retry_sec      ：启动打不开串口时的重试间隔，秒
//   uwb_runtime_reconnect_sec  ：运行中失联后的最小重连间隔，秒
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
    // 现场可调项放 launch 全局命名空间（launch/uwb_node.launch），
    // 默认值仍以顶部字面量可见；直接 rosrun 时使用默认值。
    std::string serial_port;
    bool        print_frame;
    int         heartbeat_timeout_ms;
    int         link_fail_timeout_ms;
    double      startup_retry_sec;
    double      runtime_reconnect_sec;
    nh.param<std::string>("uwb_serial_port", serial_port, "/dev/ttyUWB");
    nh.param<bool>       ("uwb_print_frame", print_frame, true);
    nh.param<int>        ("uwb_heartbeat_timeout_ms", heartbeat_timeout_ms, kHeartbeatTimeoutMs);
    nh.param<int>        ("uwb_link_fail_timeout_ms", link_fail_timeout_ms, kLinkFailTimeoutMs);
    nh.param<double>     ("uwb_startup_retry_sec", startup_retry_sec, kStartupRetrySec);
    nh.param<double>     ("uwb_runtime_reconnect_sec", runtime_reconnect_sec, kRuntimeReconnectSec);

    // 参数一致性校验：非法值直接回退，避免运行期出现「状态还没报失效就先重连」
    // 或 while 循环里每轮都尝试重连（间隔 <= 0）。
    // 告警阈值 <= 0 时 isAlive(0) 恒为假，会让状态话题一直 false，也按非法处理。
    if (heartbeat_timeout_ms <= 0)
    {
        ROS_WARN("uwb_heartbeat_timeout_ms=%d 非法（需 > 0），回退为默认值 %d",
                 heartbeat_timeout_ms, kHeartbeatTimeoutMs);
        heartbeat_timeout_ms = kHeartbeatTimeoutMs;
    }
    // 重连阈值必须严格大于告警阈值：保留「先报失效、后重连」的时序语义。
    // 回退到「告警阈值 + 1 s」而不是固定默认值，保证告警阈值被调大时依然成立。
    if (link_fail_timeout_ms <= heartbeat_timeout_ms)
    {
        ROS_WARN("uwb_link_fail_timeout_ms=%d 必须大于 uwb_heartbeat_timeout_ms=%d，改为 %d",
                 link_fail_timeout_ms, heartbeat_timeout_ms, heartbeat_timeout_ms + 1000);
        link_fail_timeout_ms = heartbeat_timeout_ms + 1000;
    }
    if (startup_retry_sec <= 0.0)
    {
        ROS_WARN("uwb_startup_retry_sec=%.2f 非法（需 > 0），回退为默认值 %.2f",
                 startup_retry_sec, kStartupRetrySec);
        startup_retry_sec = kStartupRetrySec;
    }
    if (runtime_reconnect_sec <= 0.0)
    {
        ROS_WARN("uwb_runtime_reconnect_sec=%.2f 非法（需 > 0），回退为默认值 %.2f",
                 runtime_reconnect_sec, kRuntimeReconnectSec);
        runtime_reconnect_sec = kRuntimeReconnectSec;
    }
    ROS_INFO("UWB 参数：串口=%s，告警 %d ms，失联重连 %d ms，启动重试 %.1f s，运行重连 %.1f s",
             serial_port.c_str(), heartbeat_timeout_ms, link_fail_timeout_ms,
             startup_retry_sec, runtime_reconnect_sec);

    // —— 二、打开串口 ——
    // 打不开也继续运行：单设备掉线不应该拖垮顶层 launch 里的其它传感器；
    // 主循环会用同一个 client 按 startup_retry_sec 周期重试。
    uwb_rtu_serial_demo::RtuClient client;
    if (!client.open(serial_port, kBaudrate))
    {
        ROS_ERROR("串口 %s 打开失败，节点保持运行，每 %.0f s 重试",
                  serial_port.c_str(), startup_retry_sec);
    }
    else
    {
        ROS_INFO("串口 %s 打开成功（%u 8N1）", serial_port.c_str(), kBaudrate);
    }

    uwb_rtu_serial_demo::UwbNode node(client);

    // 发布定位结果话题，话题名称 uwb_position，消息类型 uwb_rtu_serial_demo/uwb_data。
    // 发布节奏跟帧走（收一帧发一帧），默认约 10 Hz；队列 10 约等于 1 秒缓存。
    ros::Publisher pub = nh.advertise<uwb_rtu_serial_demo::uwb_data>("/perception_sensors/uwb_position", 10);

    // 链路状态话题：true=在线，false=串口未打开 / 读写异常 / 超时无有效帧。
    // latched=true：后启动的订阅者能立即拿到最近一次状态，不必等下一次跳变。
    ros::Publisher status_pub =
        nh.advertise<std_msgs::Bool>("/perception_sensors/uwb_status", 10, true);

    // —— 三、心跳与重连状态 ——
    uint32_t frequency = 0;      // 最近一个 1 秒窗口内的收帧数，写入消息 fre 字段
    bool stale_warned = false;   // 链路超时告警只打一次，恢复后复位

    // 状态话题的发布控制：首轮必发一次，之后只在 true/false 跳变时发布。
    bool last_online = false;      // 上一轮已发布的链路状态
    bool status_published = false; // 是否已发过首条

    // link_ever_ok 区分两种失败：启动就没打开（startup_retry_sec 重试）和运行中掉线
    // （runtime_reconnect_sec 重连）。初始值取 open() 的结果；reconnect 成功后置 true。
    bool link_ever_ok = client.isOpen();
    auto next_reconnect = std::chrono::steady_clock::now();
    if (!link_ever_ok)
    {
        next_reconnect += std::chrono::milliseconds(
            static_cast<int64_t>(startup_retry_sec * 1000.0));
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
                ROS_WARN_THROTTLE(10.0, "UWB串口未打开或读写异常，等待重连");
                stale_warned = true;
            }
            else
            {
                // ROS_INFO("UWB收帧频率：%u Hz", frequency);
                const bool alive = client.isAlive(heartbeat_timeout_ms);
                if (!alive && !stale_warned)
                {
                    ROS_WARN("超过 %d ms 未收到有效帧，请检查串口连接、标签是否开机并已在 K 列表中",
                             heartbeat_timeout_ms);
                    stale_warned = true;
                }
                else if (alive)
                {
                    stale_warned = false;
                }
            }
        }

        // —— 五、链路探针与重连 ——
        // 外层循环 30 Hz，检测很快；真正 open 按启动/运行两档间隔节流，不会每轮都重连。
        // isAlive() 已包含 link_error_ 判断，拔线异常会立即进入该分支。
        if (!client.isAlive(link_fail_timeout_ms))
        {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_reconnect)
            {
                const double interval = link_ever_ok
                                      ? runtime_reconnect_sec
                                      : startup_retry_sec;
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

        // —— 六、发布链路状态 ——
        // isAlive() 已包含「串口未打开 / 读写异常 / 超时无有效帧」三种情况。
        // 阈值取无帧告警阈值 heartbeat_timeout_ms，而不是失联重连阈值 link_fail_timeout_ms：
        // 下游要比重连早一步知道数据已失效。串口刚打开时 last_frame_time 被重置，
        // 有同样时长的宽限，不会因为还没有帧就立刻报 false。
        // 只在 true/false 跳变时发布一条，latched 保留当前值。
        const bool online = client.isAlive(heartbeat_timeout_ms);
        if (!status_published || online != last_online)
        {
            std_msgs::Bool status_msg;
            status_msg.data = online;
            status_pub.publish(status_msg);

            if (online)
            {
                ROS_INFO("UWB 链路在线");
            }
            else
            {
                ROS_WARN("UWB 链路超时/断开：串口未打开、读写异常或超过 %d ms 无有效帧",
                         heartbeat_timeout_ms);
            }

            last_online = online;
            status_published = true;
        }

        // 当前节点没有订阅者，spinOnce 保留用于处理 ROS 内部回调与退出流程
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前显式关闭串口，方便下次立即重新打开
    client.close();
    return 0;
}
