/**
 * @file main.cpp
 * @brief 感知传感器一键启动与底盘保存按键联动节点。
 *
 * 职责边界：
 *   1. 订阅底盘提供的保存按键状态 /chassis_robot/chassis_status（std_msgs/Bool）；
 *   2. 状态为 true 时调用 /perception_sensors/start_bag_recorder，
 *      状态为 false 时调用 /perception_sensors/stop_bag_recorder；
 *   3. 对状态去重，避免底盘周期发布时反复调用服务。
 *
 * 本节点不发布话题、不持有定时器，完全由订阅回调驱动；
 * 传感器节点和 bag_recorder 由配套 launch 一并拉起，本节点只负责“按键 → 服务”。
 */

#include <locale.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

namespace
{

// 最近一次“服务执行成功”的保存状态：1=已开始录制，0=已停止。
// 初值 -1 表示尚未同步：第一帧消息无论真假都会执行一次服务调用，
// 这样本节点中途重启时也能把 bag_recorder 拉回与底盘一致的状态。
int g_last_applied_state = -1;

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

// 底盘保存按键回调：true=开始保存，false=停止保存。
// 只在状态发生变化时动作；服务失败则保持旧状态，等下一帧重试。
void chassisStatusCallback(const std_msgs::Bool::ConstPtr& msg,
                           ros::ServiceClient* start_client,
                           ros::ServiceClient* stop_client)
{
    const int want = msg->data ? 1 : 0;
    if (want == g_last_applied_state)
    {
        return;  // 周期重复消息，忽略
    }

    const CallResult result = msg->data
        ? callTrigger(*start_client, "开始录制", true)
        : callTrigger(*stop_client, "停止录制", false);

    if (result != CallResult::kFailed)
    {
        g_last_applied_state = want;
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

    // —— 二、订阅底盘保存按键状态 ——
    // 话题约定为 std_msgs/Bool：true=开始保存，false=停止保存。
    // 若底盘实际发布的是自定义消息类型，需要同时修改这里的模板参数与回调参数。
    ros::Subscriber chassis_status_sub = nh.subscribe<std_msgs::Bool>(
        "/chassis_robot/chassis_status", 10,
        [&start_client, &stop_client](const std_msgs::Bool::ConstPtr& msg) {
            chassisStatusCallback(msg, &start_client, &stop_client);
        });

    ROS_INFO("[perception_sensors_client] 已订阅 /chassis_robot/chassis_status，等待保存按键状态…");

    // —— 三、回调驱动 ——
    // 本节点没有主循环和发布逻辑，交给 ROS 回调队列即可。
    ros::spin();
    return 0;
}
