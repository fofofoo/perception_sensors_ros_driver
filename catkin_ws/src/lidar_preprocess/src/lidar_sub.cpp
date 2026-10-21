#include "lidar_sub.h"

namespace zl_lidar {

LidarSub::LidarSub(ros::NodeHandle& nh, const std::string& topic, int frame_quantity, double timeout_sec)
    : nh_(nh), topic_name_(topic), frame_quantity_(frame_quantity), timeout_sec_(timeout_sec),
      last_message_time_(ros::Time(0)) 
{
    sub_lidar_ = nh_.subscribe(topic_name_, 10, &LidarSub::lidarCallback, this);
    
    // 初始化心跳定时器，周期 1.0 秒
    timer_ = nh_.createTimer(ros::Duration(1.0), &LidarSub::timerCallback, this);

    // 启动后台处理线程
    worker_thread_ = std::thread(&LidarSub::workerThreadFunc, this);

    ROS_INFO("[LidarSub] Initialized. Topic: %s | Frames: %d | Timeout: %.1fs", 
             topic_name_.c_str(), frame_quantity_, timeout_sec_);
}

LidarSub::~LidarSub() {
    running_ = false;
    raw_cv_.notify_one(); 
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    
    sub_lidar_.shutdown();
    timer_.stop(); 
}

// ==========================================
// ROS 点云订阅回调：执行消息转换并推入原始数据队列
// ==========================================
void LidarSub::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_data) {
    // 在锁外执行耗时的消息转换
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    temp_cloud->reserve(40000); 
    pcl::fromROSMsg(*laser_data, *temp_cloud);

    // 更新心跳时间戳
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_message_time_ = ros::Time::now();
    }

    // 若雷达处于断连状态，标记需要重置滑动窗口
    if (!is_lidar_connect_.load()) {
        need_reset_window_ = true; 
    }

    // 将转换后的点云推入原始数据队列
    {
        std::lock_guard<std::mutex> lock(raw_mutex_);
        raw_cloud_queue_.push(temp_cloud);
    }
    raw_cv_.notify_one(); // 唤醒后台处理线程
}

// ==========================================
// 后台处理线程：执行多帧累加及后续滤波/补偿算法
// ==========================================
void LidarSub::workerThreadFunc() {
    while (running_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud;

        // 等待原始数据队列更新或收到重置/退出信号
        {
            std::unique_lock<std::mutex> lock(raw_mutex_);
            raw_cv_.wait(lock, [this] { 
                return !raw_cloud_queue_.empty() || need_reset_window_.load() || !running_; 
            });
            
            if (!running_) break; 

            // 处理滑动窗口重置逻辑
            if (need_reset_window_.load()) {
                sliding_window_.clear();
                need_reset_window_ = false;
                if (raw_cloud_queue_.empty()) continue; 
            }

            new_cloud = raw_cloud_queue_.front();
            raw_cloud_queue_.pop();
        }

        // 维护多帧滑动窗口
        if (static_cast<int>(sliding_window_.size()) < frame_quantity_) {
            sliding_window_.push_back(new_cloud);
        } else {
            sliding_window_.pop_front();
            sliding_window_.push_back(new_cloud);
        }

        // 若窗口未满，等待下一帧
        if (static_cast<int>(sliding_window_.size()) < frame_quantity_) continue; 

        // 核心处理逻辑：多帧物理累加
        ProcessedCloud current_frame;
        current_frame.timestamp = pcl_conversions::fromPCL(new_cloud->header.stamp); 

        current_frame.cloud->clear();
        for (const auto& cloud_ptr : sliding_window_) {
            *(current_frame.cloud) += *cloud_ptr;
        }
        
        // TODO: 在此处扩展 IMU 运动补偿与点云滤波算法

        // 更新处理结果缓存
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            latest_result_ = std::move(current_frame); 
            has_new_result_ = true;
        }
    }
}


// ==========================================
// 定时器回调：Watchdog 心跳监测
// ==========================================
void LidarSub::timerCallback(const ros::TimerEvent&) {
    ros::Time last_time;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_time = last_message_time_;
    }

    // 初始状态保护：未收到首帧数据前不判定超时
    if (last_time == ros::Time(0)) return; 

    bool is_connected = (ros::Time::now() - last_time).toSec() < timeout_sec_;
    
    // 状态边沿检测
    if (is_connected != is_lidar_connect_.load()) {
        is_lidar_connect_ = is_connected;
        status_changed_flag_ = true; 
        
        if (!is_connected) {
            ROS_ERROR("[LidarSub] LiDAR timeout. Clearing buffers.");
            
            // 清空原始数据队列
            {
                std::lock_guard<std::mutex> lock(raw_mutex_);
                while(!raw_cloud_queue_.empty()) raw_cloud_queue_.pop();
            }
            // 清空处理结果缓存
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                has_new_result_ = false;
                latest_result_.cloud->clear();
            }
            // 通知后台线程清空滑动窗口
            need_reset_window_ = true;
            raw_cv_.notify_one(); 
            
        } else {
            ROS_INFO("[LidarSub] LiDAR connection recovered.");
        }
    }
}


// ==========================================
// 暴露给主线程的接口：获取最新的处理结果
// ==========================================
bool LidarSub::getProcessedCloud(ProcessedCloud& out_frame) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    if (has_new_result_) {
        out_frame = latest_result_; 
        has_new_result_ = false;    
        return true;
    }
    return false;
}

bool LidarSub::isConnected() const {
    return is_lidar_connect_.load();
}

bool LidarSub::checkStatusChange() {
    return status_changed_flag_.exchange(false); 
}

} // namespace zl_lidar