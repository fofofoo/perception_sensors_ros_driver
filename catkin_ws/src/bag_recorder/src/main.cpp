/**
 * @file main.cpp
 * @brief rosbag 录制控制节点入口：参数读取 + ROS 拓扑编排。
 *
 * 本文件是节点的「一眼可见」层：
 *   - 在这里读取全部 launch 参数，并写明默认值；
 *   - 在这里创建 3 个发布器、注册 2 个服务、创建 1 s 定时器；
 *   - 在这里进入 ros::spin()，服务与定时器共用单线程回调队列。
 *
 * 录制业务（fork rosbag、大小统计、目录清理、文件名分配）全部在
 * BagRecorderNode 内，见 include/bag_recorder/bag_recorder_node.h。
 *
 * 参数全部放在 <node> 外的全局命名空间，因此必须用
 *   roslaunch bag_recorder bag_recorder.launch save_dir:=...
 * 覆盖；rosrun 的 _save_dir:=... 私有参数不会生效。
 */

#include "bag_recorder/bag_recorder_node.h"

#include <locale.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <sstream>
#include <string>
#include <vector>

namespace {

// 没有配置 record_topics 时使用的默认话题，与 launch 默认值保持一致。
// launch 中的 record_topics 优先级更高；这里只作为兜底，避免参数缺失时录不到数据。
// 激光雷达录原始点云而非预处理累加结果：单帧约 1.5 MB / 10 Hz，
// 比 10 帧累加话题小约一个数量级，避免 rosbag 写入跟不上导致丢帧。
std::vector<std::string> defaultTopics() {
    return {
        "/rslidar_points",
        "/perception_sensors/ultrasonic_radar_datas",
        "/perception_sensors/uwb_position",
    };
}

// 去掉字符串首尾空白，供解析 "[/a, /b]" 形式的字符串参数使用。
std::string trim(const std::string& text) {
    const std::string whitespace = " \t\r\n";
    const std::size_t begin = text.find_first_not_of(whitespace);
    if (begin == std::string::npos) {
        return "";
    }
    const std::size_t end = text.find_last_not_of(whitespace);
    return text.substr(begin, end - begin + 1);
}

// 解析 "[/a, /b]" 或 "/a,/b" 形式的字符串参数。
// roslaunch 的 <param value="[...]"/> 在某些版本里会被存成字符串而不是数组，
// 因此除了 XmlRpc 数组外还要兼容这种写法。
std::vector<std::string> parseTopicList(const std::string& raw) {
    std::string body = trim(raw);
    if (!body.empty() && body.front() == '[') {
        body.erase(body.begin());
    }
    if (!body.empty() && body.back() == ']') {
        body.pop_back();
    }

    std::vector<std::string> topics;
    std::stringstream ss(body);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (item.size() >= 2 &&
            ((item.front() == '"' && item.back() == '"') ||
             (item.front() == '\'' && item.back() == '\''))) {
            item = item.substr(1, item.size() - 2);
        }
        item = trim(item);
        if (!item.empty()) {
            topics.push_back(item);
        }
    }
    return topics;
}

// 读取 launch 全局参数并填入配置结构。
// record_topics 优先按 XmlRpc 数组读取；若 roslaunch 把它存成了字符串，
// 再按 "[a, b]" / "a,b" 解析一次；都没有则使用默认的三个感知话题。
// 非法值的回退与告警在 BagRecorderNode::normalizeConfig() 里处理。
BagRecorderConfig loadConfig(ros::NodeHandle& nh) {
    BagRecorderConfig config;

    XmlRpc::XmlRpcValue topics_value;
    if (nh.getParam("record_topics", topics_value)) {
        if (topics_value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < topics_value.size(); ++i) {
                if (topics_value[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                    const std::string topic =
                        static_cast<std::string>(topics_value[i]);
                    if (!topic.empty()) {
                        config.topics.push_back(topic);
                    }
                }
            }
        } else if (topics_value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            config.topics = parseTopicList(static_cast<std::string>(topics_value));
        }
    }
    if (config.topics.empty()) {
        config.topics = defaultTopics();
        ROS_WARN("[BagRecorder] record_topics 未配置或为空，使用默认的 %zu 个感知话题。",
                 config.topics.size());
    }

    nh.param<std::string>("save_dir", config.save_dir, "");
    nh.param<double>("max_bag_size_gb", config.max_bag_size_gb, 25.0);
    nh.param<std::string>("total_limit_action", config.total_limit_action, "stop");
    nh.param<double>("max_dir_size_gb", config.max_dir_size_gb, 150.0);
    nh.param<double>("dir_low_watermark_gb", config.dir_low_watermark_gb, 130.0);

    return config;
}

}  // namespace

int main(int argc, char** argv) {
    // 终端按 UTF-8 输出中文日志。缺少这一行时 roslaunch 环境下中文会变成 "?"，
    // 与工作区其它三个节点保持一致。
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "bag_recorder_node");
    ros::NodeHandle nh;

    // —— 一、参数（全部来自 launch 全局命名空间）——
    const BagRecorderConfig config = loadConfig(nh);

    // —— 二、发布器（话题名、队列、latch 写在调用处）——
    // 录制状态：start/stop 时刻翻转，latched 让后到订阅者立即拿到当前状态；
    // 素材保存互斥由发布端消费。
    ros::Publisher record_status_pub = nh.advertise<std_msgs::Bool>(
        "/perception_sensors/recording_status", 5, true);
    // 当前批次已录文件总大小（GB，GiB 口径）：录制中 1 Hz，开始发 0，
    // 停止时发一次最终值；不 latch，避免新订阅者误以为录制仍在进行。
    ros::Publisher recording_size_pub = nh.advertise<std_msgs::Float64>(
        "/perception_sensors/recording_size", 5);
    // 保存目录所有批次的总大小（GB）：录制中 1 Hz，开录前预清理后也发一次；
    // 它是目录级清理的依据，不能由 recording_size 推断。
    ros::Publisher recording_dir_size_pub = nh.advertise<std_msgs::Float64>(
        "/perception_sensors/recording_dir_size", 5);

    // —— 三、节点类：只做录制业务，不碰 ROS 拓扑 ——
    BagRecorderNode node(config, record_status_pub,
                         recording_size_pub, recording_dir_size_pub);

    // —— 四、服务（触发入口与幂等语义集中在 main 可见）——
    ros::ServiceServer start_srv = nh.advertiseService(
        "perception_sensors/start_bag_recorder",
        &BagRecorderNode::startCallback, &node);
    ros::ServiceServer stop_srv = nh.advertiseService(
        "perception_sensors/stop_bag_recorder",
        &BagRecorderNode::stopCallback, &node);

    // —— 五、定时处理（1 s：目录清理 + 单次大小检查 + 两个大小话题）——
    // rosbag 没有总大小上限参数，只能在录制期间由节点周期统计。
    ros::Timer total_size_timer = nh.createTimer(
        ros::Duration(1.0), &BagRecorderNode::checkTotalSize, &node);

    // —— 六、事件循环：服务与定时器同处单线程队列，类内状态无需加锁 ——
    ros::spin();
    return 0;
}
