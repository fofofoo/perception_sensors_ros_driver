/**
 * @file main.cpp
 * @brief 感知传感器一键启动与底盘上装电源按键联动节点。
 *
 * 职责边界：
 *   1. 订阅底盘控制消息 /chassis_msg（common_msgs/ChassisMsg）；
 *   2. 读取其中的 chassis_relay_ctrl.tops_power_enable（上装电源使能）作为录制开关：
 *      值为 1 时调用 /perception_sensors/start_bag_recorder，
 *      为 0 或其它异常值时调用 /perception_sensors/stop_bag_recorder；
 *   3. 对目标状态去重，避免底盘周期发布时反复调用服务；
 *   4. 断流保护：超过 kChassisMsgTimeoutSec（5 s）未收到 /chassis_msg 时，
 *      若仍在录制则自动调用 stop 服务，防止底盘失联后持续录无效数据。
 *
 * 本节点不发布话题；以订阅回调驱动为主，另有一个 1 Hz 看门狗定时器负责断流检测。
 * 传感器节点和 bag_recorder 由配套 launch 一并拉起，本节点只负责“按键 → 服务”。
 */

#include <locale.h>

#include <common_msgs/ChassisMsg.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace
{

// 最近一次“服务执行成功”的保存状态：1=已开始录制，0=已停止。
// 初值 -1 表示尚未同步：第一帧消息无论真假都会执行一次服务调用，
// 这样本节点中途重启时也能把 bag_recorder 拉回与底盘一致的状态。
int g_last_applied_state = -1;

// 最近一次收到 /chassis_msg 的本地时间，断流看门狗据此判断底盘是否仍在发送。
// 用本地时钟而非 msg->header.stamp：底盘与主机时钟可能不同步，
// 这里只关心“距上一次收到消息过了多久”，无需绝对时间。
ros::Time g_last_msg_time;

// 断流阈值（秒）：超过该时长没有收到 /chassis_msg，视为底盘链路异常并停止录制。
// 取值依据：底盘按固定周期发布，5 s 可容忍数十帧丢失而不误停；
// 1 Hz 看门狗下，底盘失联后最迟约 6 s 停录，避免持续录制无控制键的数据。
constexpr double kChassisMsgTimeoutSec = 5.0;

// 服务调用结果。kAlreadyApplied 用于 bag_recorder 的幂等拒绝：
// 目标状态已经达成，同样算成功，避免周期消息反复重试和刷日志。
enum class CallResult
{
    kFailed,
    kApplied,
    kAlreadyApplied,
};

// 调用 Trigger 服务并打印结果。want_recording 用于识别“已经是目标状态”的拒绝。
CallResult callTrigger(ros::ServiceClient& client, const char* action, bool want_recording)
{
    // bag_recorder 可能比本节点晚启动；服务未注册时只告警，不阻塞等待。
    if (!client.exists())
    {
        ROS_WARN_THROTTLE(2.0, "[perception_sensors_client] 服务 %s 不可用，等待 bag_recorder 启动…",
                          client.getService().c_str());
        return CallResult::kFailed;
    }

    std_srvs::Trigger srv;
    if (!client.call(srv))
    {
        ROS_ERROR("[perception_sensors_client] 调用 %s 失败（%s）",
                  client.getService().c_str(), action);
        return CallResult::kFailed;
    }
    if (!srv.response.success)
    {
        // bag_recorder 的两种幂等拒绝，说明要去的状态已经达成：
        //   start → "Already recording!"；stop → "Not recording!"。
        const bool already_in_state =
            want_recording ? srv.response.message == "Already recording!"
                           : srv.response.message == "Not recording!";
        if (already_in_state)
        {
            ROS_INFO("[perception_sensors_client] %s：目标状态已达成（%s）",
                     action, srv.response.message.c_str());
            return CallResult::kAlreadyApplied;
        }

        // 其它原因（保存目录不可用等）是真失败，下一帧消息会重试。
        ROS_WARN("[perception_sensors_client] %s 被拒绝：%s",
                 action, srv.response.message.c_str());
        return CallResult::kFailed;
    }

    ROS_INFO("[perception_sensors_client] %s 成功：%s", action, srv.response.message.c_str());
    return CallResult::kApplied;
}

// 底盘控制消息回调：上装电源使能 tops_power_enable == 1 开始保存，其余值停止保存。
// 只在目标状态发生变化时动作；服务失败则保持旧状态，等下一帧重试。
void chassisMsgCallback(const common_msgs::ChassisMsg::ConstPtr& msg, ros::ServiceClient* start_client, ros::ServiceClient* stop_client)
{
    g_last_msg_time = ros::Time::now();  // 刷新断流看门狗的计时基准

    // 只认 1：0 表示关闭；-1 或其它异常值同样按关闭处理，避免把无效状态当成录制指令。
    const bool want_recording = msg->chassis_relay_ctrl.tops_power_enable == 1;
    const int want = want_recording ? 1 : 0;
    if (want == g_last_applied_state)
    {
        return;  // 周期重复消息，忽略
    }

    const CallResult result = want_recording
        ? callTrigger(*start_client, "开始录制", true)
        : callTrigger(*stop_client, "停止录制", false);

    if (result != CallResult::kFailed)
    {
        g_last_applied_state = want;
    }
}

// 断流看门狗：正在录制且超过 kChassisMsgTimeoutSec 没收到 /chassis_msg 时主动停录。
// 由 1 Hz 定时器驱动，与订阅回调处于同一 ROS 回调队列（单线程 spin），无需加锁。
// 从未收到过消息时不动作：节点刚启动、bag 尚未录制，无法区分“底盘没发”与“本节点刚启动”。
void checkChassisTimeout(ros::ServiceClient* stop_client)
{
    if (g_last_msg_time.isZero() || g_last_applied_state != 1)
    {
        return;
    }

    const double silent_sec = (ros::Time::now() - g_last_msg_time).toSec();
    if (silent_sec < kChassisMsgTimeoutSec)
    {
        return;
    }

    // 持续断流时看门狗每秒都会进来；停录成功后状态置 0，不会再重复打印。
    ROS_WARN_THROTTLE(5.0,
                      "[perception_sensors_client] %.1f s 未收到 /chassis_msg（阈值 %.1f s），自动停止录制",
                      silent_sec, kChassisMsgTimeoutSec);

    // 停录失败（服务未就绪等）保持录制状态，下一轮看门狗继续重试。
    if (callTrigger(*stop_client, "断流停止录制", false) != CallResult::kFailed)
    {
        g_last_applied_state = 0;
    }
}

}  // namespace

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");  // 终端中文日志
    ros::init(argc, argv, "perception_sensors_client_main");
    ros::NodeHandle nh;

    // —— 一、保存控制服务 ——
    // 服务由 bag_recorder_node 提供，使用绝对名称，避免节点命名空间影响。
    ros::ServiceClient start_client =
        nh.serviceClient<std_srvs::Trigger>("/perception_sensors/start_bag_recorder");
    ros::ServiceClient stop_client =
        nh.serviceClient<std_srvs::Trigger>("/perception_sensors/stop_bag_recorder");

    // —— 二、订阅底盘上装电源状态 ——
    // /chassis_msg 由底盘节点发布（common_msgs/ChassisMsg）。
    // 取 chassis_relay_ctrl.tops_power_enable 作为录制开关：1=上装电源打开→开始录制。
    ros::Subscriber chassis_msg_sub = nh.subscribe<common_msgs::ChassisMsg>(
        "/chassis_msg", 10,
        [&start_client, &stop_client](const common_msgs::ChassisMsg::ConstPtr& msg) {
            chassisMsgCallback(msg, &start_client, &stop_client);
        });

    ROS_INFO("[perception_sensors_client] 已订阅 /chassis_msg，等待上装电源状态…");

    // —— 三、断流看门狗 ——
    // 1 Hz 检查底盘消息是否超时。这是本节点唯一的定时器，
    // 用于“没有新消息”这种无法由订阅回调触发的场景。
    ros::Timer watchdog_timer = nh.createTimer(
        ros::Duration(1.0), [&stop_client](const ros::TimerEvent&) { checkChassisTimeout(&stop_client); });

    // —— 四、回调驱动 ——
    // 本节点没有主循环和发布逻辑，交给 ROS 回调队列即可（定时器也由同一队列驱动）。
    ros::spin();
    return 0;
}
