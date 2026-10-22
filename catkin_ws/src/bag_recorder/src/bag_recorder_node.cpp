/**
 * @file bag_recorder_node.cpp
 * @brief BagRecorderNode 类实现（录制业务流水线）。
 *
 * 本文件只包含业务实现：
 *   - fork / SIGINT / waitpid 管理 rosbag 子进程；
 *   - 当前批次分卷统计与单次大小上限；
 *   - 保存目录跨批次高低水位清理；
 *   - bag 文件名序号恢复与分配。
 *
 * ROS 拓扑不在这里：publisher、service、timer 由 main.cpp 创建，
 * 对应话题名、服务名、队列与周期也在 main.cpp 可见。
 *
 * 文件名规则：
 *   perception_sensors_<6 位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag
 *   序号是唯一性与排序主键，时间戳只用于人工识别。工控机无网络、时钟可能
 *   重复或回退时，序号仍能保证每次录制不重名、按创建顺序排序。
 *   注意：rosbag 开启 --split 后，第一个分卷名会追加 _0（即使没有真正分卷），
 *   例如 perception_sensors_000008_20261021_102030_0.bag。
 */

#include "bag_recorder/bag_recorder_node.h"

#include <ros/package.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Linux 进程控制相关头文件
#include <signal.h>     // kill, SIGINT
#include <sys/types.h>  // pid_t
#include <sys/wait.h>   // waitpid
#include <unistd.h>     // fork, execvp, _exit

namespace fs = std::filesystem;

namespace {

// bag 文件名的固定前缀与序号宽度。修改前缀时需同步更新 scanExistingSequence()
// 里的正则，否则重启后无法从已有文件恢复序号。
const char* const kBagPrefix = "perception_sensors_";
const int kBagSeqWidth = 6;              // 6 位十进制，最大 999999
const uint32_t kBagSeqMax = 999999;      // 序号耗尽后拒绝录制，避免回绕覆盖旧文件

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

// 构造：保存配置与发布器副本，校验参数并恢复 bag 序号；不创建目录、不启动录制。
BagRecorderNode::BagRecorderNode(const BagRecorderConfig& config,
                                 ros::Publisher record_status_pub,
                                 ros::Publisher recording_size_pub,
                                 ros::Publisher recording_dir_size_pub)
    : record_status_pub_(std::move(record_status_pub)),
      recording_size_pub_(std::move(recording_size_pub)),
      recording_dir_size_pub_(std::move(recording_dir_size_pub)),
      config_(config) {
    normalizeConfig();
    logConfig();

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

    ROS_INFO("[BagRecorder] Standalone bag recorder node started (single-threaded version)...");
}

// 析构：录制中先让 rosbag 落盘，并广播停止状态，避免素材发布端一直互斥。
BagRecorderNode::~BagRecorderNode() {
    if (is_recording_.load()) {
        ROS_WARN("[BagRecorder] Node shutting down, safely stopping background recording...");
        kill(record_pid_.load(), SIGINT);
        // 进程退出前广播停止，在线的素材发布端即时恢复（不依赖 latch 重连）
        std_msgs::Bool status_msg;
        status_msg.data = false;
        record_status_pub_.publish(status_msg);
    }
}

// 开始录制：状态检查、目录准备、序号恢复、目录预清理、分配文件名、
// fork 子进程执行 rosbag record；父进程更新状态并广播。
bool BagRecorderNode::startCallback(std_srvs::Trigger::Request& req,
                                    std_srvs::Trigger::Response& res) {
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
    const int max_size_mb = BagRecorderConfig::kSplitSizeMb;

    // 5. 核心：fork + execvp 拉起后台 rosbag 进程。
    // fork() 是系统调用，不是普通函数：内核把当前进程复制出一份子进程，
    // 父子两个进程都从 fork() 的下一行继续执行，所以这一行会“返回两次”：
    //   - 父进程（现在的节点进程）：返回新子进程的 PID，一定 > 0，该节点负责发布录制状态
    //   - 子进程（复制出来的新进程）：返回 0，该进程独立执行rosbag record，之后不再执行本节点的任何 ROS 逻辑。
    //   - 创建失败：只有父进程返回 -1，此时不存在子进程。
    pid_t pid = fork();

    if (pid < 0) {
        // —— 分支一：fork 失败（只在父进程中发生）——
        // 内核没有创建出子进程，也就不存在 rosbag 进程和 bag 文件，无需清理，报错退出。
        res.success = false;
        res.message = "Failed to fork process.";
        ROS_ERROR("[BagRecorder] fork() failed!");
        return true;
    } else if (pid == 0) {
        // —— 分支二：子进程（fork() 返回 0）, 子进程触发rosbag录制命令 ——
        // 会用 execvp() 把自己替换成 rosbag 程序，之后不再执行本节点的任何 ROS 逻辑。
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

        for (const auto& topic : config_.topics) {
            argv_strings.push_back(topic);
        }

        std::vector<char*> args;
        args.reserve(argv_strings.size() + 1);
        for (auto& item : argv_strings) {
            args.push_back(const_cast<char*>(item.c_str()));
        }
        args.push_back(nullptr);

        execvp("rosbag", args.data());

        // execvp() 成功时不会返回：当前进程已经被 rosbag record 替换，
        // 这行之后的代码都不存在了。只有失败（找不到 rosbag 或参数非法）才会走到这里。此时还没有 bag 文件落盘，无需清理。
        // 必须用 _exit() 而不是 return 或 exit()，避免调用 C++ 静态对象析构函数，破坏父进程的 ROS 运行环境。
        ROS_ERROR("[BagRecorder] Child process execvp rosbag failed!");
        _exit(1);
    } else {
        // —— 分支三：父进程（fork() 返回子进程 PID）——
        // 父进程不执行 rosbag，只做三件事：
        //   1. 记账：保存子进程 PID（stopRecording() 靠它发 SIGINT 并 waitpid 等落盘）、记录当前批次前缀、置录制标志；
        //   2. 通知：发布 recording_status=true（素材保存互斥）和大小 0；
        //   3. 回复服务调用方，结束回调；
        //   后续：返回 main 里面的 ros::spin(),由 1 s 定时器检查大小并完成单次录制超限控制、文件夹超限清理、已录制文件大小和保存文件夹大小话题更新。
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

// 停止录制：未录制时返回幂等失败文案；否则调用 stopRecording() 发信号并等待落盘。
bool BagRecorderNode::stopCallback(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& res) {
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

// 参数校验：非法值回退到默认值，避免把 rosbag 参数传成 0 或负数。
// 低水位非法时回退为「高水位 - 2 个单卷」，保证一次清理至少能删两卷。
void BagRecorderNode::normalizeConfig() {
    if (config_.max_bag_size_gb <= 0.0) {
        ROS_WARN("[BagRecorder] max_bag_size_gb=%.3f 非法，回退到 25.0 GB。",
                 config_.max_bag_size_gb);
        config_.max_bag_size_gb = 25.0;
    }
    if (config_.total_limit_action != "stop" &&
        config_.total_limit_action != "delete_old") {
        ROS_WARN("[BagRecorder] total_limit_action='%s' 非法，回退到 stop。"
                 "可选值：stop / delete_old。",
                 config_.total_limit_action.c_str());
        config_.total_limit_action = "stop";
    }

    // 目录级清理参数校验：
    // - max_dir_size_gb <= 0 表示关闭清理，只统计和发布目录总量；
    // - 低水位必须落在 (0, 高水位) 之间。非法时回退到「高水位 - 2 个单卷」，
    //   保证一次清理至少能删掉两卷，避免高低水位过近导致每秒都触发删除；
    //   若高水位本身不足两个单卷，则退化为高水位的一半。
    if (config_.max_dir_size_gb <= 0.0) {
        ROS_WARN("[BagRecorder] max_dir_size_gb=%.3f <= 0，关闭目录级清理，"
                 "仅发布目录总量。", config_.max_dir_size_gb);
    } else if (config_.dir_low_watermark_gb <= 0.0 ||
               config_.dir_low_watermark_gb >= config_.max_dir_size_gb) {
        const double volume_gb = BagRecorderConfig::kSplitSizeMb / 1024.0;
        const double fallback = config_.max_dir_size_gb - 2.0 * volume_gb;
        const double old_low = config_.dir_low_watermark_gb;
        config_.dir_low_watermark_gb =
            (fallback > 0.0) ? fallback : config_.max_dir_size_gb / 2.0;
        ROS_WARN("[BagRecorder] dir_low_watermark_gb=%.3f 非法（须在 0 与高水位 "
                 "%.3f 之间），回退到 %.3f GB。",
                 old_low, config_.max_dir_size_gb, config_.dir_low_watermark_gb);
    }
}

// 打印生效参数：话题、保存目录、单次上限、单卷大小、超限动作、目录水位。
// 在 normalizeConfig() 之后调用，日志里是最终生效值而不是原始输入值。
void BagRecorderNode::logConfig() const {
    const double volume_gb = BagRecorderConfig::kSplitSizeMb / 1024.0;

    ROS_INFO("[BagRecorder] 录制话题（%zu 个）：", config_.topics.size());
    for (const auto& topic : config_.topics) {
        ROS_INFO("  - %s", topic.c_str());
    }
    ROS_INFO("[BagRecorder] save_dir='%s'，单次总上限 %.3f GB，单卷 %.0f GB，超限动作 %s。",
             config_.save_dir.empty() ? "<按 bag_recorder 包路径推导>"
                                      : config_.save_dir.c_str(),
             config_.max_bag_size_gb,
             volume_gb,
             config_.total_limit_action.c_str());
    ROS_INFO("[BagRecorder] 目录级清理：高水位 %.3f GB，低水位 %.3f GB%s。",
             config_.max_dir_size_gb, config_.dir_low_watermark_gb,
             config_.max_dir_size_gb <= 0.0 ? "（已关闭）" : "");

    // 单卷固定 10 GB。delete_old 只能删除已经关闭的旧分卷，如果总上限
    // 比单卷还小，第一卷一写完就已经超限，此时没有旧卷可删。
    if (config_.total_limit_action == "delete_old" &&
        config_.max_bag_size_gb < volume_gb) {
        ROS_WARN("[BagRecorder] 总上限 %.3f GB 小于单卷 %.0f GB，delete_old "
                 "在首卷写入期间无法生效，建议把 max_bag_size_gb 调到至少单卷大小。",
                 config_.max_bag_size_gb, volume_gb);
    }
}

// 停止当前录制：给 rosbag 发 SIGINT 并等待其落盘，广播状态、清理当前批次。
// 返回 true 表示信号发送成功；失败时也会把内部状态置为未录制。
bool BagRecorderNode::stopRecording(std::string* message) {
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
std::vector<BagRecorderNode::SplitFile> BagRecorderNode::currentSplits() const {
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
uintmax_t BagRecorderNode::currentRecordingBytes() const {
    uintmax_t total = 0;
    for (const auto& split : currentSplits()) {
        total += split.size;
    }
    return total;
}

// 当前批次现存分卷的总大小，单位 GB（GiB，1024³ 字节）。
// 注意：delete_old 删除旧分卷后，这个值会下降，反映的是磁盘占用而不是
// 本次累计写入量。
double BagRecorderNode::currentRecordingGb() const {
    const double bytes_per_gb = 1024.0 * 1024.0 * 1024.0;
    return static_cast<double>(currentRecordingBytes()) / bytes_per_gb;
}

// 发布当前录制大小（std_msgs/Float64，单位 GB）。
// 录制期间每秒调用；开始录制时发 0；停止时在 rosbag 落盘后发一次最终值。
void BagRecorderNode::publishRecordingSize(double size_gb) {
    std_msgs::Float64 msg;
    msg.data = size_gb;
    recording_size_pub_.publish(msg);
}

// 发布保存目录所有批次的总大小（std_msgs/Float64，单位 GB）。
// 录制期间每秒调用；开录前预清理后也发一次。不 latch。
void BagRecorderNode::publishDirSize(uintmax_t bytes) {
    std_msgs::Float64 msg;
    msg.data = static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0);
    recording_dir_size_pub_.publish(msg);
}

// 扫描保存目录下所有由本节点命名的 bag 分卷（含正在写入的 .bag.active），
// 按「批次序号 → 分卷号」升序返回，供目录级清理从旧到新删除。
// 只认 perception_sensors_<6 位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag 格式；
// 目录里手动录制的其它 bag、临时文件既不统计也不删除。
std::vector<BagRecorderNode::DirBagFile> BagRecorderNode::scanDirBags(
    const std::string& dir) const {
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
uint32_t BagRecorderNode::currentBatchSeq() const {
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
// 采用高低水位状态机：未激活时总量达到 max_dir_size_gb 才进入清理，
// 进入后必须删到 dir_low_watermark_gb 才复位，避免每秒只删一卷。
// 删除顺序：先删非当前批次，仍超限再删当前批次的旧分卷；两轮都跳过
// .bag.active（正在写入，删除后空间也不会释放）。max_dir_size_gb <= 0
// 时只统计和发布大小，不删除。返回本次删除的文件数。
int BagRecorderNode::enforceDirLimit(const std::string& dir, bool publish_size) {
    const double bytes_per_gb = 1024.0 * 1024.0 * 1024.0;
    const std::vector<DirBagFile> files = scanDirBags(dir);

    uintmax_t total = 0;
    for (const auto& file : files) {
        total += file.size;
    }

    // 清理关闭时只保留统计，不做任何删除。
    if (config_.max_dir_size_gb <= 0.0) {
        if (publish_size) {
            publishDirSize(total);
        }
        return 0;
    }

    const uintmax_t high_bytes =
        static_cast<uintmax_t>(config_.max_dir_size_gb * bytes_per_gb);
    const uintmax_t low_bytes =
        static_cast<uintmax_t>(config_.dir_low_watermark_gb * bytes_per_gb);

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
                 config_.dir_low_watermark_gb);
    } else if (!dir_limit_warning_shown_) {
        // 只剩 active（或删除失败）时无法再降，限流告警，避免每秒刷屏。
        ROS_WARN("[BagRecorder] 目录总量 %.3f GB 仍高于高水位 %.3f GB，但已没有"
                 "可删除的已关闭分卷（只剩正在写入的 .bag.active 或残留文件）。",
                 static_cast<double>(total) / bytes_per_gb, config_.max_dir_size_gb);
        dir_limit_warning_shown_ = true;
    }
    return deleted;
}

// 删除当前批次最旧的已关闭分卷，直到总大小不超过 cap_bytes。
// 正在写入的 .bag.active 永远不删，避免破坏 rosbag 的输出文件。
// 返回实际删除的分卷数量。
int BagRecorderNode::deleteOldestClosedSplits(uintmax_t cap_bytes) {
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

// 定时检查当前批次总大小；达到 max_bag_size_gb 后按开关处理：
// stop 停止录制，delete_old 删除最旧的已关闭分卷后继续。
// 由 ros::spin() 单线程调用，与服务回调天然串行，不需要额外加锁。
void BagRecorderNode::checkTotalSize(const ros::TimerEvent&) {
    if (!is_recording_.load() || current_bag_prefix_.empty()) {
        return;
    }

    // 先做目录级清理：它可能删掉当前批次的旧分卷，之后再统计单次大小，
    // 保证两个上限基于同一时刻的目录状态。目录大小话题也在这里发布。
    const std::string dir = fs::path(current_bag_prefix_).parent_path().string();
    enforceDirLimit(dir, true);

    const double bytes_per_gb = 1024.0 * 1024.0 * 1024.0;
    const uintmax_t cap_bytes =
        static_cast<uintmax_t>(config_.max_bag_size_gb * bytes_per_gb);
    const uintmax_t total = currentRecordingBytes();
    const double total_gb = static_cast<double>(total) / bytes_per_gb;

    // 无论是否达到总上限，每秒都刷新一次当前批次大小（含正在写入的 .active）。
    publishRecordingSize(total_gb);

    if (total < cap_bytes) {
        return;
    }
    if (config_.total_limit_action == "delete_old") {
        const int deleted = deleteOldestClosedSplits(cap_bytes);
        if (deleted == 0 && !limit_warning_shown_) {
            ROS_WARN("[BagRecorder] 总大小 %.3f GB 已达上限 %.3f GB，但没有可删除的"
                     "已关闭分卷（当前卷仍在写入）。", total_gb,
                     config_.max_bag_size_gb);
            limit_warning_shown_ = true;
        } else if (deleted > 0) {
            limit_warning_shown_ = false;
        }
        return;
    }

    ROS_WARN("[BagRecorder] 总大小 %.3f GB 达到上限 %.3f GB，按 stop 策略停止录制。",
             total_gb, config_.max_bag_size_gb);
    std::string message;
    if (!stopRecording(&message)) {
        ROS_ERROR("[BagRecorder] 达到总上限后停止失败：%s", message.c_str());
    }
}

// 解析保存目录：save_dir 非空时直接用；为空时按 bag_recorder 自身包路径推导
// <pkg>/../../../log/rosbag。包位于 catkin_ws/src/bag_recorder 时，结果就是
// 仓库根目录下的 log/rosbag，不再依赖外部的 robot_detector_ros 包。
bool BagRecorderNode::resolveSaveDir(std::string* dir, std::string* error) const {
    if (!config_.save_dir.empty()) {
        *dir = config_.save_dir;
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
    const fs::path raw_path =
        fs::path(package_path) / ".." / ".." / ".." / "log" / "rosbag";
    const fs::path normalized = fs::weakly_canonical(raw_path, ec);
    *dir = ec ? raw_path.lexically_normal().string() : normalized.string();
    return true;
}

// 确保保存目录存在；已存在但不是目录时返回失败。
bool BagRecorderNode::ensureDirectory(const std::string& dir, std::string* error) const {
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
void BagRecorderNode::scanExistingSequence(const std::string& dir) {
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
bool BagRecorderNode::allocateBagPath(const std::string& dir, std::string* prefix,
                                      std::string* error) {
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
