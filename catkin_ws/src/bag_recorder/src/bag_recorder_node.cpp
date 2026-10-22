/**
 * @file bag_recorder_node.cpp
 * @brief 独立的 rosbag 录制控制节点 (极简单线程版)
 *
 * 职责边界：
 *   - 提供 start / stop 两个 Trigger 服务，fork 子进程执行 rosbag record；
 *   - 录制期间发布 latched 的 recording_status，供素材发布端做互斥；
 *   - 录制期间每秒发布当前批次已录文件总大小（GB），停止时发布一次最终值；
 *   - 目录级清理：按 save_dir 下所有批次 bag 的总大小做高低水位循环覆盖，
 *     防止多次录制累积把磁盘占满；
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
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <xmlrpcpp/XmlRpcValue.h>

// C++ 标准库
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
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

// 单个分卷大小固定 10 GB，不再通过 launch 配置。
// rosbag 的 --size 单位是 MB，因此直接写 10 * 1024。
const int kSplitSizeMb = 10 * 1024;

// 总大小检查周期（秒）。rosbag 只支持单卷大小和分卷数量，没有“总大小上限”
// 参数，所以由节点在录制期间定时统计当前批次的文件总大小。
const double kTotalSizeCheckPeriodSec = 1.0;

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
    ros::Publisher record_status_pub_;        // 录制状态广播（素材保存互斥信号源）
    ros::Publisher recording_size_pub_;       // 当前批次已录文件总大小（GB，std_msgs/Float64）
    ros::Publisher recording_dir_size_pub_;   // 保存目录所有批次总大小（GB，std_msgs/Float64）

    std::atomic<pid_t> record_pid_{-1};
    std::atomic<bool> is_recording_{false};

    // —— 现场可调参数（launch 全局命名空间）——
    std::vector<std::string> topics_;   // 录制话题列表；record_topics 覆盖默认值
    std::string save_dir_;              // 保存目录；空表示按 bag_recorder 包路径推导
    double max_bag_size_gb_ = 25.0;     // 单次录制总大小上限（GB）
    std::string total_limit_action_ = "stop";  // 达到总上限后的动作：stop / delete_old
    double max_dir_size_gb_ = 150.0;    // 保存目录所有批次的高水位（GB，GiB 口径）；<= 0 关闭
    double dir_low_watermark_gb_ = 130.0;  // 清理目标低水位（GB）：进入清理后删到这里才停

    // —— 当前录制批次 ——
    std::string current_bag_prefix_;    // 当前批次的 bag 前缀（不含 _N.bag）
    ros::Timer total_size_timer_;       // 录制期间定时检查单次与目录总大小
    bool limit_warning_shown_ = false;  // delete_old 无法删除时避免每秒刷日志
    bool dir_cleanup_active_ = false;   // 已进入目录清理，未降到低水位前持续删除
    bool dir_limit_warning_shown_ = false;  // 目录超限但无可删文件时避免每秒刷日志

    // 当前批次的一个分卷文件。
    struct SplitFile {
        fs::path path;
        unsigned long index = 0;   // rosbag 追加的分卷号
        bool active = false;       // true 表示 .bag.active，正在写入
        uintmax_t size = 0;        // 字节数
    };

    // 保存目录下的一个 bag 分卷（跨批次），供目录级清理使用。
    // 与 SplitFile 的区别是带 batch_seq：清理时要先删最旧批次。
    struct DirBagFile {
        fs::path path;
        uint32_t batch_seq = 0;        // 文件名中的 6 位批次序号（录制次数）
        unsigned long split_index = 0; // rosbag 追加的分卷号
        bool active = false;           // true 表示 .bag.active，正在写入
        uintmax_t size = 0;            // 字节数
    };

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
        // （/perception_sensors/recording_status，std_msgs/Bool）。
        // 2026-10-21 由 /robot_detector/recording_status 迁入感知命名空间，
        // 外部订阅端需同步修改话题名。
        record_status_pub_ = nh_.advertise<std_msgs::Bool>("/perception_sensors/recording_status", 5, true);  // latched: 后到订阅者立即拿到当前状态
        // 当前批次已录文件总大小，单位 GB（GiB，1024³ 字节）。
        // 录制期间由总大小定时器每秒发布；开始录制发 0，停止时发一次最终值。
        // 与状态话题不同，不 latch，避免新订阅者误以为录制仍在进行。
        recording_size_pub_ = nh_.advertise<std_msgs::Float64>("/perception_sensors/recording_size", 5);
        // 保存目录所有批次的总大小，单位 GB（GiB，1024³ 字节）。
        // 录制期间由总大小定时器每秒发布，开录前预清理后也发一次；
        // 与 recording_size 的区别是它统计整个 save_dir，是目录级清理的依据。
        recording_dir_size_pub_ = nh_.advertise<std_msgs::Float64>(
            "/perception_sensors/recording_dir_size", 5);

        // 总大小检查定时器：rosbag 没有总大小上限参数，录制期间由节点统计
        // 当前批次所有分卷（含正在写入的 .active）的总字节数。
        total_size_timer_ = nh_.createTimer(
            ros::Duration(kTotalSizeCheckPeriodSec), &BagRecorderNode::checkTotalSize, this);
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

        // 开录前按目录总量做一次预清理：历史批次若已把目录撑到高水位以上，
        // 先删最旧批次，避免本次录制一开始就超额。此时 current_bag_prefix_
        // 为空，所有历史文件都按「非当前批次」处理；.bag.active 仍然跳过。
        enforceDirLimit(target_dir, true);

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

        // 4. 单卷大小固定为 10 GB；总上限由定时器在录制期间检查，
        //    不通过 rosbag 的分卷数量参数实现。
        const int max_size_mb = kSplitSizeMb;

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

            // ================= 分卷大小 =================
            // 只按单卷大小切分；总上限与超限动作由父进程定时器处理。
            // 不使用 --max-splits，避免 rosbag 的删除策略与节点开关冲突。
            argv_strings.push_back("--split");                       // 开启文件分割
            argv_strings.push_back("--size");                        // 单个分卷大小（MB）
            argv_strings.push_back(std::to_string(max_size_mb));
            // ============================================

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
            current_bag_prefix_ = bag_prefix;
            limit_warning_shown_ = false;
            record_pid_.store(pid);
            is_recording_.store(true);

            // 广播录制开始（互斥信号：素材保存节点/发布端据此停发素材）
            std_msgs::Bool status_msg;
            status_msg.data = true;
            record_status_pub_.publish(status_msg);

            // 新会话从 0 开始，订阅端据此重置显示；随后定时器每秒刷新实际大小。
            publishRecordingSize(0.0);

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

        std::string message;
        res.success = stopRecording(&message);
        res.message = message;
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
        nh_.param<std::string>("total_limit_action", total_limit_action_, "stop");
        nh_.param<double>("max_dir_size_gb", max_dir_size_gb_, 150.0);
        nh_.param<double>("dir_low_watermark_gb", dir_low_watermark_gb_, 130.0);

        // 参数兜底：非法值直接回退到默认值，避免把 rosbag 参数传成 0 或负数。
        if (max_bag_size_gb_ <= 0.0) {
            ROS_WARN("[BagRecorder] max_bag_size_gb=%.3f 非法，回退到 25.0 GB。",
                     max_bag_size_gb_);
            max_bag_size_gb_ = 25.0;
        }
        if (total_limit_action_ != "stop" && total_limit_action_ != "delete_old") {
            ROS_WARN("[BagRecorder] total_limit_action='%s' 非法，回退到 stop。"
                     "可选值：stop / delete_old。",
                     total_limit_action_.c_str());
            total_limit_action_ = "stop";
        }

        // 目录级清理参数校验：
        // - max_dir_size_gb_ <= 0 表示关闭清理，只统计和发布目录总量；
        // - 低水位必须落在 (0, 高水位) 之间。非法时回退到「高水位 - 2 个单卷」，
        //   保证一次清理至少能删掉两卷，避免高低水位过近导致每秒都触发删除；
        //   若高水位本身不足两个单卷，则退化为高水位的一半。
        if (max_dir_size_gb_ <= 0.0) {
            ROS_WARN("[BagRecorder] max_dir_size_gb=%.3f <= 0，关闭目录级清理，"
                     "仅发布目录总量。", max_dir_size_gb_);
        } else if (dir_low_watermark_gb_ <= 0.0 ||
                   dir_low_watermark_gb_ >= max_dir_size_gb_) {
            const double volume_gb = kSplitSizeMb / 1024.0;
            const double fallback = max_dir_size_gb_ - 2.0 * volume_gb;
            const double old_low = dir_low_watermark_gb_;
            dir_low_watermark_gb_ =
                (fallback > 0.0) ? fallback : max_dir_size_gb_ / 2.0;
            ROS_WARN("[BagRecorder] dir_low_watermark_gb=%.3f 非法（须在 0 与高水位 "
                     "%.3f 之间），回退到 %.3f GB。",
                     old_low, max_dir_size_gb_, dir_low_watermark_gb_);
        }

        ROS_INFO("[BagRecorder] 录制话题（%zu 个）：", topics_.size());
        for (const auto& topic : topics_) {
            ROS_INFO("  - %s", topic.c_str());
        }
        ROS_INFO("[BagRecorder] save_dir='%s'，单次总上限 %.3f GB，单卷 %.0f GB，超限动作 %s。",
                 save_dir_.empty() ? "<按 bag_recorder 包路径推导>" : save_dir_.c_str(),
                 max_bag_size_gb_,
                 kSplitSizeMb / 1024.0,
                 total_limit_action_.c_str());
        ROS_INFO("[BagRecorder] 目录级清理：高水位 %.3f GB，低水位 %.3f GB%s。",
                 max_dir_size_gb_, dir_low_watermark_gb_,
                 max_dir_size_gb_ <= 0.0 ? "（已关闭）" : "");

        // 单卷固定 10 GB。delete_old 只能删除已经关闭的旧分卷，如果总上限
        // 比单卷还小，第一卷一写完就已经超限，此时没有旧卷可删。
        if (total_limit_action_ == "delete_old" &&
            max_bag_size_gb_ < kSplitSizeMb / 1024.0) {
            ROS_WARN("[BagRecorder] 总上限 %.3f GB 小于单卷 %.0f GB，delete_old "
                     "在首卷写入期间无法生效，建议把 max_bag_size_gb 调到至少单卷大小。",
                     max_bag_size_gb_, kSplitSizeMb / 1024.0);
        }
    }

    // 停止当前录制：给 rosbag 发 SIGINT 并等待其落盘，广播状态、清理当前批次。
    // 返回 true 表示信号发送成功；失败时也会把内部状态置为未录制。
    bool stopRecording(std::string* message) {
        const pid_t pid = record_pid_.load();
        if (!is_recording_.load() || pid <= 0) {
            if (message) *message = "Not recording!";
            return false;
        }

        ROS_INFO_STREAM("[BagRecorder] Stopping recording (Sending signal to PID: " << pid << ")...");
        bool ok = true;
        if (kill(pid, SIGINT) == 0) {
            int status = 0;
            waitpid(pid, &status, 0);
        } else {
            ok = false;
            ROS_ERROR("[BagRecorder] Failed to send SIGINT, the process may have already exited.");
        }

        // rosbag 已落盘，前缀仍然有效，此时统计最终大小；kill 失败时只能取当前值。
        const double stopped_size_gb = currentRecordingGb();

        is_recording_.store(false);
        record_pid_.store(-1);
        current_bag_prefix_.clear();

        // 广播录制结束（素材保存恢复条件；rosbag 已落盘完成，无重叠帧）
        std_msgs::Bool status_msg;
        status_msg.data = false;
        record_status_pub_.publish(status_msg);

        // 停止后再发一次最终大小，之后不再更新；订阅端可据此显示本次结果。
        publishRecordingSize(stopped_size_gb);

        if (message) {
            *message = ok ? "Recording stopped gracefully."
                          : "Failed to send signal to process.";
        }
        if (ok) {
            ROS_INFO("[BagRecorder] Recording stopped gracefully, bag file finalized.");
        }
        return ok;
    }

    // 列出当前批次的全部分卷文件（含正在写入的 .bag.active），按分卷号升序。
    // 只匹配 current_bag_prefix_，避免误扫同目录下其它批次的 bag。
    std::vector<SplitFile> currentSplits() const {
        std::vector<SplitFile> splits;
        if (current_bag_prefix_.empty()) {
            return splits;
        }

        const fs::path prefix_path(current_bag_prefix_);
        const std::string dir = prefix_path.parent_path().string();
        const std::string base = prefix_path.filename().string() + "_";
        std::error_code ec;
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            if (ec) {
                break;
            }
            std::error_code entry_ec;
            if (!entry.is_regular_file(entry_ec) || entry_ec) {
                continue;
            }

            const std::string name = entry.path().filename().string();
            if (name.rfind(base, 0) != 0) {
                continue;
            }

            std::string rest = name.substr(base.size());
            const std::string active_ext = ".bag.active";
            const std::string closed_ext = ".bag";
            bool active = false;
            if (rest.size() > active_ext.size() &&
                rest.compare(rest.size() - active_ext.size(),
                             active_ext.size(), active_ext) == 0) {
                rest.resize(rest.size() - active_ext.size());
                active = true;
            } else if (rest.size() > closed_ext.size() &&
                       rest.compare(rest.size() - closed_ext.size(),
                                    closed_ext.size(), closed_ext) == 0) {
                rest.resize(rest.size() - closed_ext.size());
            } else {
                continue;
            }

            // 分卷号必须是纯数字；rosbag 的格式是 <前缀>_<序号>.bag[.active]。
            if (rest.empty() ||
                !std::all_of(rest.begin(), rest.end(),
                             [](unsigned char c) { return std::isdigit(c) != 0; })) {
                continue;
            }

            unsigned long index = 0;
            try {
                index = std::stoul(rest);
            } catch (const std::exception&) {
                continue;
            }

            std::error_code size_ec;
            const uintmax_t size = fs::file_size(entry.path(), size_ec);
            if (size_ec) {
                continue;
            }

            SplitFile split;
            split.path = entry.path();
            split.index = index;
            split.active = active;
            split.size = size;
            splits.push_back(std::move(split));
        }

        std::sort(splits.begin(), splits.end(),
                  [](const SplitFile& a, const SplitFile& b) {
                      return a.index < b.index;
                  });
        return splits;
    }

    // 当前批次所有分卷（含正在写入的 .active）的总字节数。
    uintmax_t currentRecordingBytes() const {
        uintmax_t total = 0;
        for (const auto& split : currentSplits()) {
            total += split.size;
        }
        return total;
    }

    // 当前批次现存分卷的总大小，单位 GB（GiB，1024³ 字节）。
    // 注意：delete_old 删除旧分卷后，这个值会下降，反映的是磁盘占用而不是
    // 本次累计写入量。
    double currentRecordingGb() const {
        const double bytes_per_gb = 1024.0 * 1024.0 * 1024.0;
        return static_cast<double>(currentRecordingBytes()) / bytes_per_gb;
    }

    // 发布当前录制大小（std_msgs/Float64，单位 GB）。
    // 录制期间每秒调用；开始录制时发 0；停止时在 rosbag 落盘后发一次最终值。
    void publishRecordingSize(double size_gb) {
        std_msgs::Float64 msg;
        msg.data = size_gb;
        recording_size_pub_.publish(msg);
    }

    // 发布保存目录所有批次的总大小（std_msgs/Float64，单位 GB）。
    // 录制期间每秒调用；开录前预清理后也发一次。不 latch。
    void publishDirSize(uintmax_t bytes) {
        std_msgs::Float64 msg;
        msg.data = static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0);
        recording_dir_size_pub_.publish(msg);
    }

    // 扫描保存目录下所有由本节点命名的 bag 分卷（含正在写入的 .bag.active），
    // 按「批次序号 → 分卷号」升序返回，供目录级清理从旧到新删除。
    // 只认 perception_sensors_<6 位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag 格式；
    // 目录里手动录制的其它 bag、临时文件既不统计也不删除。
    std::vector<DirBagFile> scanDirBags(const std::string& dir) const {
        std::vector<DirBagFile> files;
        std::error_code ec;
        if (!fs::is_directory(dir, ec)) {
            return files;
        }

        const std::regex pattern(
            std::string("^") + kBagPrefix + "(\\d{" + std::to_string(kBagSeqWidth) +
            "})_\\d{8}_\\d{6}_(\\d+)\\.bag(\\.active)?$");

        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            if (ec) {
                break;
            }
            std::error_code entry_ec;
            if (!entry.is_regular_file(entry_ec) || entry_ec) {
                continue;
            }

            const std::string name = entry.path().filename().string();
            std::smatch match;
            if (!std::regex_match(name, match, pattern)) {
                continue;
            }

            DirBagFile file;
            try {
                file.batch_seq = static_cast<uint32_t>(std::stoul(match[1].str()));
                file.split_index = std::stoul(match[2].str());
            } catch (const std::exception&) {
                continue;
            }
            file.active = match[3].matched;

            std::error_code size_ec;
            file.size = fs::file_size(entry.path(), size_ec);
            if (size_ec) {
                continue;
            }
            file.path = entry.path();
            files.push_back(std::move(file));
        }

        std::sort(files.begin(), files.end(),
                  [](const DirBagFile& a, const DirBagFile& b) {
                      if (a.batch_seq != b.batch_seq) {
                          return a.batch_seq < b.batch_seq;
                      }
                      return a.split_index < b.split_index;
                  });
        return files;
    }

    // 从当前批次前缀解析文件名里的 6 位批次序号；未录制时返回 0。
    // 只用于目录级清理区分「当前正在录的批次」和「历史批次」；
    // 前缀由 allocateBagPath() 生成，格式固定，解析失败只可能是内存状态异常。
    uint32_t currentBatchSeq() const {
        if (current_bag_prefix_.empty()) {
            return 0;
        }
        const std::string name = fs::path(current_bag_prefix_).filename().string();
        const std::regex pattern(std::string("^") + kBagPrefix + "(\\d{" +
                                 std::to_string(kBagSeqWidth) + "})_");
        std::smatch match;
        if (!std::regex_search(name, match, pattern)) {
            return 0;
        }
        try {
            return static_cast<uint32_t>(std::stoul(match[1].str()));
        } catch (const std::exception&) {
            return 0;
        }
    }

    // 目录级清理：把 save_dir 下所有本节点 bag 的总大小压到低水位。
    // 采用高低水位状态机：未激活时总量达到 max_dir_size_gb_ 才进入清理，
    // 进入后必须删到 dir_low_watermark_gb_ 才复位，避免每秒只删一卷。
    // 删除顺序：先删非当前批次，仍超限再删当前批次的旧分卷；两轮都跳过
    // .bag.active（正在写入，删除后空间也不会释放）。max_dir_size_gb_ <= 0
    // 时只统计和发布大小，不删除。返回本次删除的文件数。
    int enforceDirLimit(const std::string& dir, bool publish_size) {
        const double bytes_per_gb = 1024.0 * 1024.0 * 1024.0;
        const std::vector<DirBagFile> files = scanDirBags(dir);

        uintmax_t total = 0;
        for (const auto& file : files) {
            total += file.size;
        }

        // 清理关闭时只保留统计，不做任何删除。
        if (max_dir_size_gb_ <= 0.0) {
            if (publish_size) {
                publishDirSize(total);
            }
            return 0;
        }

        const uintmax_t high_bytes =
            static_cast<uintmax_t>(max_dir_size_gb_ * bytes_per_gb);
        const uintmax_t low_bytes =
            static_cast<uintmax_t>(dir_low_watermark_gb_ * bytes_per_gb);

        // 未进入清理态且没到高水位：保持现状。
        if (!dir_cleanup_active_ && total < high_bytes) {
            if (publish_size) {
                publishDirSize(total);
            }
            return 0;
        }
        dir_cleanup_active_ = true;

        const uint32_t current_seq = currentBatchSeq();
        int deleted = 0;

        // 两轮删除：第 0 轮只删历史批次，第 1 轮才允许删当前批次的旧卷。
        // 这样当前录制尽量保留，只有历史批次全删完仍超限时才动它。
        // files 已按批次序号和分卷号升序，从旧到新删。
        for (int pass = 0; pass < 2; ++pass) {
            for (const auto& file : files) {
                if (total <= low_bytes) {
                    break;
                }
                if (file.active) {
                    continue;  // 正在写入的文件永远不删
                }
                const bool is_current = (file.batch_seq == current_seq);
                if (is_current != (pass == 1)) {
                    continue;
                }

                std::error_code ec;
                if (!fs::remove(file.path, ec) || ec) {
                    ROS_WARN("[BagRecorder] 目录清理删除失败：%s（%s）",
                             file.path.c_str(), ec.message().c_str());
                    continue;
                }

                total -= file.size;
                ++deleted;
                ROS_INFO("[BagRecorder] 目录总量超过高水位，删除最旧分卷：%s（%.3f GB）",
                         file.path.c_str(),
                         static_cast<double>(file.size) / bytes_per_gb);
            }
        }

        // 删除后统一刷新一次，保证每次检查最多发布一条目录大小。
        if (publish_size) {
            publishDirSize(total);
        }

        if (total <= low_bytes) {
            dir_cleanup_active_ = false;
            dir_limit_warning_shown_ = false;
            ROS_INFO("[BagRecorder] 目录清理完成，当前总量 %.3f GB（低水位 %.3f GB）。",
                     static_cast<double>(total) / bytes_per_gb,
                     dir_low_watermark_gb_);
        } else if (!dir_limit_warning_shown_) {
            // 只剩 active（或删除失败）时无法再降，限流告警，避免每秒刷屏。
            ROS_WARN("[BagRecorder] 目录总量 %.3f GB 仍高于高水位 %.3f GB，但已没有"
                     "可删除的已关闭分卷（只剩正在写入的 .bag.active 或残留文件）。",
                     static_cast<double>(total) / bytes_per_gb, max_dir_size_gb_);
            dir_limit_warning_shown_ = true;
        }
        return deleted;
    }

    // 删除当前批次最旧的已关闭分卷，直到总大小不超过 cap_bytes。
    // 正在写入的 .bag.active 永远不删，避免破坏 rosbag 的输出文件。
    // 返回实际删除的分卷数量。
    int deleteOldestClosedSplits(uintmax_t cap_bytes) {
        const std::vector<SplitFile> splits = currentSplits();
        uintmax_t total = 0;
        for (const auto& split : splits) {
            total += split.size;
        }

        int deleted = 0;
        for (const auto& split : splits) {
            if (total <= cap_bytes) {
                break;
            }
            if (split.active) {
                continue;
            }

            std::error_code ec;
            if (!fs::remove(split.path, ec) || ec) {
                ROS_WARN("[BagRecorder] 删除旧分卷失败：%s（%s）",
                         split.path.c_str(), ec.message().c_str());
                continue;
            }

            total -= split.size;
            ++deleted;
            ROS_INFO("[BagRecorder] 总大小超过上限，删除最旧分卷：%s（%.3f GB）",
                     split.path.c_str(),
                     static_cast<double>(split.size) / (1024.0 * 1024.0 * 1024.0));
        }
        return deleted;
    }

    // 定时检查当前批次总大小；达到 max_bag_size_gb_ 后按开关处理：
    // stop 停止录制，delete_old 删除最旧的已关闭分卷后继续。
    // 由 ros::spin() 单线程调用，与服务回调天然串行，不需要额外加锁。
    void checkTotalSize(const ros::TimerEvent&) {
        if (!is_recording_.load() || current_bag_prefix_.empty()) {
            return;
        }

        // 先做目录级清理：它可能删掉当前批次的旧分卷，之后再统计单次大小，
        // 保证两个上限基于同一时刻的目录状态。目录大小话题也在这里发布。
        const std::string dir = fs::path(current_bag_prefix_).parent_path().string();
        enforceDirLimit(dir, true);

        const double bytes_per_gb = 1024.0 * 1024.0 * 1024.0;
        const uintmax_t cap_bytes =
            static_cast<uintmax_t>(max_bag_size_gb_ * bytes_per_gb);
        const uintmax_t total = currentRecordingBytes();
        const double total_gb = static_cast<double>(total) / bytes_per_gb;

        // 无论是否达到总上限，每秒都刷新一次当前批次大小（含正在写入的 .active）。
        publishRecordingSize(total_gb);

        if (total < cap_bytes) {
            return;
        }
        if (total_limit_action_ == "delete_old") {
            const int deleted = deleteOldestClosedSplits(cap_bytes);
            if (deleted == 0 && !limit_warning_shown_) {
                ROS_WARN("[BagRecorder] 总大小 %.3f GB 已达上限 %.3f GB，但没有可删除的"
                         "已关闭分卷（当前卷仍在写入）。", total_gb, max_bag_size_gb_);
                limit_warning_shown_ = true;
            } else if (deleted > 0) {
                limit_warning_shown_ = false;
            }
            return;
        }

        ROS_WARN("[BagRecorder] 总大小 %.3f GB 达到上限 %.3f GB，按 stop 策略停止录制。",
                 total_gb, max_bag_size_gb_);
        std::string message;
        if (!stopRecording(&message)) {
            ROS_ERROR("[BagRecorder] 达到总上限后停止失败：%s", message.c_str());
        }
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
