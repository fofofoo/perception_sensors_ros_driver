/**
 * @file bag_recorder_node.h
 * @brief rosbag 录制控制节点类声明。
 *
 * 职责边界：
 *   - 只负责录制业务：fork/停止 rosbag 子进程、分卷统计、目录级清理、
 *     生成唯一 bag 文件名；
 *   - 不持有 NodeHandle、service server、timer，也不包含任何话题名/服务名；
 *   - ROS 拓扑（publisher、service 注册、定时器周期、事件循环）全部由
 *     main.cpp 创建，publisher 以副本传入，回调由 main 绑定。
 *
 * 调用时机：
 *   - 构造：main 读取参数后创建；只做参数校验、启动扫描 bag 序号，
 *     不创建目录，避免节点启动就产生文件副作用；
 *   - startCallback / stopCallback：由 main 注册的两个 Trigger 服务触发；
 *   - checkTotalSize：由 main 创建的 1 s 定时器在录制期间触发。
 *
 * 线程模型：
 *   单线程。服务回调与定时器回调都在 ros::spin() 的同一回调队列里执行，
 *   天然串行，因此类内状态（pid、is_recording、清理状态、序号）不需要加锁。
 *   若将来改成 AsyncSpinner 或独立回调队列，必须重新划分锁边界。
 *
 * 生命周期：
 *   publisher 副本由 main 持有，且 node 在 main 中晚于 publisher 定义，
 *   因此 node 析构时 publisher 仍然有效，析构函数可以安全发布停止状态。
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <string>
#include <sys/types.h>
#include <vector>

/**
 * @brief 录制节点的运行参数，全部来自 launch 全局命名空间。
 *
 * 默认值与 launch/bag_recorder.launch 保持一致；main.cpp 用这些默认值读取
 * 参数，因此参数缺失时行为与显式传默认值相同。
 */
struct BagRecorderConfig {
    /// 单个分卷固定大小（MB）。rosbag 的 --size 单位是 MB，因此 10 GB = 10240。
    /// 该值同时用于高低水位非法时的回退计算（一个清理周期至少删两卷）。
    static constexpr int kSplitSizeMb = 10 * 1024;

    std::vector<std::string> topics;          ///< 录制话题；空表示使用默认三话题
    std::string save_dir;                     ///< 保存目录；空表示按包路径推导
    double max_bag_size_gb = 25.0;            ///< 单次录制总上限（GB）
    std::string total_limit_action = "stop";  ///< 达到单次上限后的动作：stop / delete_old
    double max_dir_size_gb = 150.0;           ///< 目录所有批次总大小高水位（GB）；<= 0 关闭清理
    double dir_low_watermark_gb = 130.0;      ///< 目录清理目标低水位（GB）
};

/**
 * @brief rosbag 录制控制节点。
 *
 * 对外只暴露三个入口：
 *   - startCallback()：校验状态、预清理目录、生成文件名、fork rosbag；
 *   - stopCallback()：向 rosbag 发 SIGINT 并等待落盘；
 *   - checkTotalSize()：周期检查单次上限与目录高水位。
 *
 * 文件名规则：
 *   perception_sensors_<6 位序号>_<YYYYmmdd_HHMMSS>_<分卷号>.bag
 *   序号是唯一性与排序主键，时间戳只用于人工识别；工控机时钟重复或回退
 *   也不影响唯一性。--split 下首个分卷带 _0 后缀。
 */
class BagRecorderNode {
public:
    /**
     * @brief 构造录制节点。
     * @param config                  main 已读取的 launch 参数
     * @param record_status_pub       /perception_sensors/recording_status（latched）
     * @param recording_size_pub      /perception_sensors/recording_size
     * @param recording_dir_size_pub  /perception_sensors/recording_dir_size
     *
     * 构造内部会：校验并回退非法参数、打印生效参数、扫描保存目录恢复
     * bag 序号。不会创建保存目录，也不会启动录制。
     */
    BagRecorderNode(const BagRecorderConfig& config,
                    ros::Publisher record_status_pub,
                    ros::Publisher recording_size_pub,
                    ros::Publisher recording_dir_size_pub);

    /**
     * @brief 析构。
     *
     * 若仍在录制，向 rosbag 发 SIGINT 让它落盘，并发布一次
     * recording_status=false，避免素材发布端一直处于互斥状态。
     * 不等待 waitpid，因为节点退出时 rosbag 会接收同一终端信号自行收尾。
     */
    ~BagRecorderNode();

    /**
     * @brief 开始录制服务回调。
     * @param req Trigger 空请求
     * @param res success/message；重复调用时 success=false，message="Already recording!"
     * @return 始终 true（服务调用本身成功），业务结果放在 res.success。
     *
     * 流程：状态检查 → 确定/创建目录 → 恢复序号 → 目录预清理 →
     * 分配唯一前缀 → fork + execvp rosbag record。
     */
    bool startCallback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res);

    /**
     * @brief 停止录制服务回调。
     * @param req Trigger 空请求
     * @param res success/message；未录制时 success=false，message="Not recording!"
     * @return 始终 true，业务结果放在 res.success。
     *
     * 向 rosbag 发 SIGINT 并 waitpid，保证 bag 索引落盘完成后再发布状态。
     */
    bool stopCallback(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);

    /**
     * @brief 录制期间的周期检查（main 以 1 s 定时器绑定）。
     * @param event ROS 定时器事件，未使用
     *
     * 未录制时直接返回。先做目录级清理（可能删除当前批次旧卷），再统计
     * 当前批次大小并按 total_limit_action 停止或删除旧卷；同时发布
     * recording_size 与 recording_dir_size。
     */
    void checkTotalSize(const ros::TimerEvent& event);

private:
    /// 当前批次的一个分卷文件。
    struct SplitFile {
        std::filesystem::path path;
        unsigned long index = 0;  ///< rosbag 追加的分卷号
        bool active = false;      ///< true 表示 .bag.active，正在写入
        uintmax_t size = 0;       ///< 字节数
    };

    /// 保存目录下的一个 bag 分卷（跨批次），供目录级清理使用。
    /// 与 SplitFile 的区别是带 batch_seq：清理时要先删最旧批次。
    struct DirBagFile {
        std::filesystem::path path;
        uint32_t batch_seq = 0;         ///< 文件名中的 6 位批次序号（录制次数）
        unsigned long split_index = 0;  ///< rosbag 追加的分卷号
        bool active = false;            ///< true 表示 .bag.active，正在写入
        uintmax_t size = 0;             ///< 字节数
    };

    /// 校验参数并回退非法值（单次上限、超限动作、高低水位）。
    void normalizeConfig();

    /// 打印生效参数；在 normalizeConfig() 之后、扫描序号之前调用。
    void logConfig() const;

    /// 停止当前录制：发 SIGINT、waitpid、广播状态、发布最终大小。
    /// 返回 true 表示信号发送成功；失败时也会把内部状态置为未录制。
    bool stopRecording(std::string* message);

    /// 列出当前批次的全部分卷（含 .bag.active），按分卷号升序。
    std::vector<SplitFile> currentSplits() const;

    /// 当前批次所有分卷（含 .active）的总字节数。
    uintmax_t currentRecordingBytes() const;

    /// 当前批次现存分卷总大小（GB，GiB 口径）。
    double currentRecordingGb() const;

    /// 发布当前批次大小（GB）。
    void publishRecordingSize(double size_gb);

    /// 发布保存目录所有批次总大小（GB）。
    void publishDirSize(uintmax_t bytes);

    /// 扫描 save_dir 下所有本节点命名的分卷，按批次序号 → 分卷号升序。
    std::vector<DirBagFile> scanDirBags(const std::string& dir) const;

    /// 从当前批次前缀解析 6 位批次序号；未录制时返回 0。
    uint32_t currentBatchSeq() const;

    /// 目录级高低水位清理；返回本次删除的文件数。
    int enforceDirLimit(const std::string& dir, bool publish_size);

    /// 删除当前批次最旧的已关闭分卷，直到总大小不超过 cap_bytes。
    int deleteOldestClosedSplits(uintmax_t cap_bytes);

    /// 解析保存目录：save_dir 非空直接用，否则按包路径推导 log/rosbag。
    bool resolveSaveDir(std::string* dir, std::string* error) const;

    /// 确保保存目录存在；已存在但不是目录时返回失败。
    bool ensureDirectory(const std::string& dir, std::string* error) const;

    /// 扫描目录已有文件名，恢复下一个可用批次序号。
    void scanExistingSequence(const std::string& dir);

    /// 分配未被占用的 bag 前缀（不含 _N.bag），并递增序号。
    bool allocateBagPath(const std::string& dir, std::string* prefix,
                         std::string* error);

    // —— 由 main 注入的发布器（副本，生命周期覆盖本对象）——
    ros::Publisher record_status_pub_;        ///< 录制状态（素材保存互斥信号源）
    ros::Publisher recording_size_pub_;       ///< 当前批次总大小（GB）
    ros::Publisher recording_dir_size_pub_;   ///< 保存目录总大小（GB）

    // —— 参数 ——
    BagRecorderConfig config_;

    // —— 录制进程状态 ——
    std::atomic<pid_t> record_pid_{-1};
    std::atomic<bool> is_recording_{false};

    // —— 当前录制批次 ——
    std::string current_bag_prefix_;         ///< 当前批次前缀（不含 _N.bag）
    bool limit_warning_shown_ = false;       ///< delete_old 无法删除时避免每秒刷日志
    bool dir_cleanup_active_ = false;        ///< 已进入目录清理，降到低水位前持续删除
    bool dir_limit_warning_shown_ = false;   ///< 目录超限但无可删文件时避免刷日志

    // —— 文件名序号状态 ——
    uint32_t next_bag_seq_ = 1;       ///< 下一次录制使用的序号
    bool sequence_scanned_ = false;   ///< 是否已从保存目录恢复过序号
    std::string scanned_dir_;         ///< 已扫描的目录，目录变化时重新扫描
};
