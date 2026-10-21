/**
 * @file bag_recorder_node.cpp
 * @brief 独立的 rosbag 录制控制节点 (极简单线程版)
 *
 * 职责边界：
 *   - 提供 start / stop 两个 Trigger 服务，fork 子进程执行 rosbag record；
 *   - 录制期间发布 latched 的 recording_status，供素材发布端做互斥；
 *   - 负责生成唯一、可排序的 bag 文件名，并保证不会覆盖已有文件。
 *
 * 文件名规则：
 *   perception_sensors_<6 位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag
 *   序号是唯一性与排序主键，时间戳只用于人工识别。工控机无网络、时钟可能
 *   重复或回退时，序号仍能保证每次录制不重名、按创建顺序排序。
 *   注意：rosbag 开启 --split 后，第一个分卷名会追加 _0（即使没有真正分卷），
 *   例如 perception_sensors_000008_20261021_102030_0.bag。
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <ros/package.h>
#include <xmlrpcpp/XmlRpcValue.h>

// C++ 标准库
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

// Linux 进程控制相关头文件
#include <unistd.h>      // fork, execvp, _exit
#include <sys/types.h>   // pid_t
#include <sys/wait.h>    // waitpid
#include <signal.h>      // kill, SIGINT

namespace fs = std::filesystem;

namespace {

// bag 文件名的固定前缀与序号宽度。修改前缀时需同步更新 scanExistingSequence()
// 里的正则，否则重启后无法从已有文件恢复序号。
const char* const kBagPrefix = "perception_sensors_";
const int kBagSeqWidth = 6;              // 6 位十进制，最大 999999
const uint32_t kBagSeqMax = 999999;      // 序号耗尽后拒绝录制，避免回绕覆盖旧文件

// 没有配置 record_topics 时使用的默认话题，与当前感知链路保持一致。
// launch 中的 record_topics 优先级更高；这里只作为兜底，避免参数缺失时录不到数据。
std::vector<std::string> defaultTopics() {
    return {
        "/perception_sensors/processed_lidar_cloud",
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

// 生成文件名中的时间戳部分，格式 YYYYmmdd_HHMMSS。
// 使用 localtime_r 而不是 localtime，避免以后多线程化时的静态缓冲区问题。
std::string formatTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&time_now, &tm_buf);

    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return ss.str();
}

}  // namespace

class BagRecorderNode {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer start_srv_;
    ros::ServiceServer stop_srv_;
    ros::Publisher record_status_pub_;   // 录制状态广播（素材保存互斥信号源）

    std::atomic<pid_t> record_pid_{-1};
    std::atomic<bool> is_recording_{false};

    // —— 现场可调参数（launch 全局命名空间）——
    std::vector<std::string> topics_;   // 录制话题列表；record_topics 覆盖默认值
    std::string save_dir_;              // 保存目录；空表示按 bag_recorder 包路径推导
    double max_bag_size_gb_ = 25.0;     // 单个分卷大小上限（GB）
    int max_splits_ = 2;                // 最多保留的分卷数量，超出删除最旧

    // —— 文件名序号状态 ——
    uint32_t next_bag_seq_ = 1;         // 下一次录制使用的序号
    bool sequence_scanned_ = false;     // 是否已从保存目录恢复过序号
    std::string scanned_dir_;           // 已扫描的目录，目录变化时重新扫描

public:
    BagRecorderNode() {
        loadParams();

        // 启动时扫描一次保存目录，恢复已有文件的最大序号。
        // 此时目录可能还不存在（首次运行），scanExistingSequence() 会保持序号 1；
        // 真正创建目录留给 startCallback，避免节点启动就产生目录副作用。
        std::string dir;
        std::string error;
        if (resolveSaveDir(&dir, &error)) {
            scanExistingSequence(dir);
            sequence_scanned_ = true;
            scanned_dir_ = dir;
        } else {
            ROS_WARN("[BagRecorder] 启动时无法确定保存目录：%s；开始录制时会再次尝试。",
                     error.c_str());
        }

        start_srv_ = nh_.advertiseService("perception_sensors/start_bag_recorder", &BagRecorderNode::startCallback, this);
        stop_srv_  = nh_.advertiseService("perception_sensors/stop_bag_recorder", &BagRecorderNode::stopCallback, this);
        // 录制状态广播：start/stop 时刻翻转，素材互斥由发布端消费
        // （/robot_detector/recording_status，std_msgs/Bool）
        record_status_pub_ = nh_.advertise<std_msgs::Bool>("robot_detector/recording_status", 5, true);  // latched: 后到订阅者立即拿到当前状态
        ROS_INFO("[BagRecorder] Standalone bag recorder node started (single-threaded version)...");
    }

    bool startCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        (void)req;  // Trigger 请求为空

        // 1. 状态检查：防止重复开启
        if (is_recording_.load()) {
            res.success = false;
            res.message = "Already recording!";
            ROS_WARN("[BagRecorder] Already recording, ignoring start request.");
            return true;
        }

        // 2. 确定保存目录并确保存在
        std::string target_dir;
        std::string error;
        if (!resolveSaveDir(&target_dir, &error)) {
            res.success = false;
            res.message = error;
            ROS_ERROR_STREAM("[BagRecorder] " << error);
            return true;
        }
        if (!ensureDirectory(target_dir, &error)) {
            res.success = false;
            res.message = error;
            ROS_ERROR_STREAM("[BagRecorder] " << error);
            return true;
        }

        // 启动时目录可能还不存在；若当时没扫到，或目录被参数改过，这里补扫一次。
        if (!sequence_scanned_ || scanned_dir_ != target_dir) {
            scanExistingSequence(target_dir);
            sequence_scanned_ = true;
            scanned_dir_ = target_dir;
        }

        // 3. 生成唯一文件名：序号 + 时间戳。rosbag 的 -O 在目标已存在时会
        //    直接覆盖（实测），所以先检查 <前缀>.bag 与首个分卷 <前缀>_0.bag。
        std::string bag_prefix;
        if (!allocateBagPath(target_dir, &bag_prefix, &error)) {
            res.success = false;
            res.message = error;
            ROS_ERROR_STREAM("[BagRecorder] " << error);
            return true;
        }
        // 开启 --split 后，rosbag 的第一个分卷固定追加 _0，后续为 _1、_2……
        // 这里用首个分卷作为日志与服务返回的路径，避免再创建一个不会被使用的
        // 裸 <前缀>.bag 占位文件。
        const std::string first_bag_path = bag_prefix + "_0.bag";
        ROS_INFO_STREAM("[BagRecorder] Preparing to record to file: " << first_bag_path);

        // 4. rosbag --size 的单位是 MB；launch 以 GB 暴露给现场。
        const int max_size_mb = static_cast<int>(std::llround(max_bag_size_gb_ * 1024.0));

        // 5. 核心：使用 fork 创建子进程
        pid_t pid = fork();

        if (pid < 0) {
            // 此时还没有创建任何 bag 文件（文件名只在 rosbag 子进程中落盘），
            // 直接返回失败即可；序号已被消耗，下次录制会顺延，不影响唯一性。
            res.success = false;
            res.message = "Failed to fork process.";
            ROS_ERROR("[BagRecorder] fork() failed!");
            return true;
        }
        else if (pid == 0) {
            // ================= 子进程执行区 =================
            // 先把所有参数放到 std::string 容器里，再构造 char* 数组，
            // 保证 execvp 执行期间字符串仍然有效。
            std::vector<std::string> argv_strings;
            argv_strings.push_back("rosbag");
            argv_strings.push_back("record");
            argv_strings.push_back("--lz4");

            // ================= 分卷大小与保留数量 =================
            argv_strings.push_back("--split");                       // 开启文件分割
            argv_strings.push_back("--size");                        // 单个分卷上限（MB）
            argv_strings.push_back(std::to_string(max_size_mb));
            argv_strings.push_back("--max-splits");                  // 最多保留的分卷数
            argv_strings.push_back(std::to_string(max_splits_));
            // ======================================================

            // 使用 -O 指定精确路径：节点已经生成唯一文件名，不再让 rosbag
            // 追加自己的时间戳（-o 会追加，导致文件名出现两个时间戳）。
            argv_strings.push_back("-O");
            argv_strings.push_back(bag_prefix);

            for (const auto& topic : topics_) {
                argv_strings.push_back(topic);
            }

            std::vector<char*> args;
            args.reserve(argv_strings.size() + 1);
            for (auto& item : argv_strings) {
                args.push_back(const_cast<char*>(item.c_str()));
            }
            args.push_back(nullptr);

            execvp("rosbag", args.data());

            // execvp 失败：直接退出。此时还没有 bag 文件落盘，无需清理。
            ROS_ERROR("[BagRecorder] Child process execvp rosbag failed!");
            _exit(1);
        }
        else {
            // ================= 父进程执行区 =================
            record_pid_.store(pid);
            is_recording_.store(true);

            // 广播录制开始（互斥信号：素材保存节点/发布端据此停发素材）
            std_msgs::Bool status_msg;
            status_msg.data = true;
            record_status_pub_.publish(status_msg);

            res.success = true;
            res.message = "Started. Path: " + first_bag_path + " | PID: " + std::to_string(pid);
            ROS_INFO_STREAM("[BagRecorder] Recording started (Background PID: " << pid << ")");
        }
        return true;
    }

    bool stopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        (void)req;  // Trigger 请求为空

        if (!is_recording_.load()) {
            res.success = false;
            res.message = "Not recording!";
            ROS_WARN("[BagRecorder] Not currently recording, ignoring stop request.");
            return true;
        }

        pid_t pid = record_pid_.load();
        ROS_INFO_STREAM("[BagRecorder] Stopping recording (Sending signal to PID: " << pid << ")...");

        if (kill(pid, SIGINT) == 0) {
            int status;
            waitpid(pid, &status, 0);

            is_recording_.store(false);
            record_pid_.store(-1);

            // 广播录制结束（素材保存恢复条件；rosbag 已落盘完成，无重叠帧）
            std_msgs::Bool status_msg;
            status_msg.data = false;
            record_status_pub_.publish(status_msg);

            res.success = true;
            res.message = "Recording stopped gracefully.";
            ROS_INFO("[BagRecorder] Recording stopped gracefully, bag file finalized.");
        } else {
            res.success = false;
            res.message = "Failed to send signal to process.";
            ROS_ERROR("[BagRecorder] Failed to send SIGINT, the process may have already exited.");
            is_recording_.store(false);
            // rosbag 已退出：广播录制结束，素材保存恢复
            std_msgs::Bool status_msg;
            status_msg.data = false;
            record_status_pub_.publish(status_msg);
        }
        return true;
    }

    ~BagRecorderNode() {
        if (is_recording_.load()) {
            ROS_WARN("[BagRecorder] Node shutting down, safely stopping background recording...");
            kill(record_pid_.load(), SIGINT);
            // 进程退出前广播停止，在线的素材发布端即时恢复（不依赖 latch 重连）
            std_msgs::Bool status_msg;
            status_msg.data = false;
            record_status_pub_.publish(status_msg);
        }
    }

private:
    // 读取 launch 全局参数。参数全部放在 <node> 外，由主句柄 nh_ 读取；
    // 这样与工作区其它传感器节点的参数约定保持一致。
    void loadParams() {
        // record_topics 优先按 XmlRpc 数组读取；若 roslaunch 把它存成了字符串，
        // 再按 "[a, b]" / "a,b" 解析一次。都没有则使用默认的三个感知话题。
        topics_.clear();
        XmlRpc::XmlRpcValue topics_value;
        if (nh_.getParam("record_topics", topics_value)) {
            if (topics_value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < topics_value.size(); ++i) {
                    if (topics_value[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                        const std::string topic = static_cast<std::string>(topics_value[i]);
                        if (!topic.empty()) {
                            topics_.push_back(topic);
                        }
                    }
                }
            } else if (topics_value.getType() == XmlRpc::XmlRpcValue::TypeString) {
                topics_ = parseTopicList(static_cast<std::string>(topics_value));
            }
        }
        if (topics_.empty()) {
            topics_ = defaultTopics();
            ROS_WARN("[BagRecorder] record_topics 未配置或为空，使用默认的 %zu 个感知话题。",
                     topics_.size());
        }

        nh_.param<std::string>("save_dir", save_dir_, "");
        nh_.param<double>("max_bag_size_gb", max_bag_size_gb_, 25.0);
        nh_.param<int>("max_splits", max_splits_, 2);

        // 参数兜底：非法值直接回退到默认值，避免把 rosbag 参数传成 0 或负数。
        if (max_bag_size_gb_ <= 0.0) {
            ROS_WARN("[BagRecorder] max_bag_size_gb=%.3f 非法，回退到 25.0 GB。",
                     max_bag_size_gb_);
            max_bag_size_gb_ = 25.0;
        }
        if (max_splits_ < 1) {
            ROS_WARN("[BagRecorder] max_splits=%d 非法，回退到 2。", max_splits_);
            max_splits_ = 2;
        }

        ROS_INFO("[BagRecorder] 录制话题（%zu 个）：", topics_.size());
        for (const auto& topic : topics_) {
            ROS_INFO("  - %s", topic.c_str());
        }
        ROS_INFO("[BagRecorder] save_dir='%s'，分卷上限 %.3f GB（%d MB），最多保留 %d 个分卷。",
                 save_dir_.empty() ? "<按 bag_recorder 包路径推导>" : save_dir_.c_str(),
                 max_bag_size_gb_,
                 static_cast<int>(std::llround(max_bag_size_gb_ * 1024.0)),
                 max_splits_);
    }

    // 解析保存目录：save_dir 非空时直接用；为空时按 bag_recorder 自身包路径推导
    // <pkg>/../../../log/rosbag。包位于 catkin_ws/src/bag_recorder 时，结果就是
    // 仓库根目录下的 log/rosbag，不再依赖外部的 robot_detector_ros 包。
    bool resolveSaveDir(std::string* dir, std::string* error) const {
        if (!save_dir_.empty()) {
            *dir = save_dir_;
            return true;
        }

        const std::string package_path = ros::package::getPath("bag_recorder");
        if (package_path.empty()) {
            if (error) {
                *error = "无法获取 bag_recorder 包路径，请配置 save_dir 或 source 工作空间。";
            }
            return false;
        }

        std::error_code ec;
        const fs::path raw_path = fs::path(package_path) / ".." / ".." / ".." / "log" / "rosbag";
        const fs::path normalized = fs::weakly_canonical(raw_path, ec);
        *dir = ec ? raw_path.lexically_normal().string() : normalized.string();
        return true;
    }

    // 确保保存目录存在；已存在但不是目录时返回失败。
    bool ensureDirectory(const std::string& dir, std::string* error) const {
        std::error_code ec;
        if (fs::is_directory(dir, ec)) {
            return true;
        }
        if (fs::exists(dir, ec)) {
            if (error) {
                *error = "保存路径已存在但不是目录：" + dir;
            }
            return false;
        }
        if (!fs::create_directories(dir, ec) || ec) {
            if (error) {
                *error = "创建保存目录失败：" + dir + "（" + ec.message() + "）";
            }
            return false;
        }
        ROS_INFO_STREAM("[BagRecorder] Automatically created recording directory: " << dir);
        return true;
    }

    // 启动时扫描保存目录，取 perception_sensors_<6 位序号>_ 的最大序号 + 1。
    // 只解析文件名，不读 bag 内容；目录不存在时保持序号 1。
    void scanExistingSequence(const std::string& dir) {
        next_bag_seq_ = 1;

        std::error_code ec;
        if (!fs::is_directory(dir, ec)) {
            ROS_INFO("[BagRecorder] 保存目录尚不存在，序号从 1 开始：%s", dir.c_str());
            return;
        }

        const std::regex pattern(std::string("^") + kBagPrefix + "(\\d{" +
                                 std::to_string(kBagSeqWidth) + "})_");
        uint32_t max_seq = 0;
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            if (ec) {
                break;
            }
            if (!entry.is_regular_file(ec)) {
                continue;
            }
            const std::string name = entry.path().filename().string();
            std::smatch match;
            if (std::regex_search(name, match, pattern)) {
                try {
                    const uint32_t seq = static_cast<uint32_t>(std::stoul(match[1].str()));
                    max_seq = std::max(max_seq, seq);
                } catch (const std::exception&) {
                    // 极端情况下文件名数字溢出，忽略该文件即可。
                }
            }
        }

        next_bag_seq_ = max_seq + 1;
        ROS_INFO("[BagRecorder] 扫描 %s，已有最大序号 %06u，下一次使用 %06u。",
                 dir.c_str(), max_seq, next_bag_seq_);
    }

    // 分配一个未被占用的 bag 前缀（不含 .bag 后缀，供 rosbag -O 使用）。
    // rosbag 开启 --split 时首卷为 <前缀>_0.bag，未开启时才是 <前缀>.bag；
    // 两种都检查，防止 -O 覆盖已有文件。序号由启动扫描恢复，正常情况下
    // 循环第一次就能命中；并发启动时靠序号递增自然错开。
    bool allocateBagPath(const std::string& dir, std::string* prefix, std::string* error) {
        for (;;) {
            if (next_bag_seq_ > kBagSeqMax) {
                if (error) {
                    *error = "bag 序号已耗尽（最大 999999），请清理保存目录或更换前缀。";
                }
                return false;
            }

            std::ostringstream name;
            name << kBagPrefix
                 << std::setw(kBagSeqWidth) << std::setfill('0') << next_bag_seq_
                 << "_" << formatTimestamp();

            const std::string candidate = (fs::path(dir) / name.str()).string();

            // 同时检查裸文件名和首个分卷名。--split 下 rosbag 只写 _0，
            // 但保留裸文件检查可兼容将来去掉 --split 的情况。
            std::error_code ec_plain;
            std::error_code ec_split;
            const bool plain_exists = fs::exists(candidate + ".bag", ec_plain);
            const bool split_exists = fs::exists(candidate + "_0.bag", ec_split);
            if (ec_plain || ec_split) {
                if (error) {
                    *error = "检查 bag 文件是否存在失败：" + candidate +
                             "（" + (ec_plain ? ec_plain.message() : ec_split.message()) + "）";
                }
                return false;
            }

            if (!plain_exists && !split_exists) {
                *prefix = candidate;
                ++next_bag_seq_;
                return true;
            }

            // 该序号已被已有文件占用（例如上次异常退出留下的分卷），换下一个。
            ++next_bag_seq_;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_recorder_node");
    BagRecorderNode node;
    ros::spin();
    return 0;
}
