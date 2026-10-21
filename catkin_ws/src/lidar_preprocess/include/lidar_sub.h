#ifndef LIDAR_SUB_H
#define LIDAR_SUB_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <deque>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <pcl_conversions/pcl_conversions.h>

namespace zl_lidar {

// 定义处理后的点云数据结构，包含点云指针与对应的时间戳
struct ProcessedCloud {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    ros::Time timestamp;
    ProcessedCloud() : cloud(new pcl::PointCloud<pcl::PointXYZI>()) {}
};

/**
 * @brief 激光雷达预处理组件
 * 
 * 职责：
 * 1. 订阅原始点云话题，进行 ROS 到 PCL 的格式转换。
 * 2. 维护滑动窗口，执行多帧点云物理累加（去噪）。
 * 3. 监测雷达心跳状态（Watchdog）。
 * 
 * 架构设计：
 * 采用多线程异步处理模型。ROS 回调线程负责数据接收，后台 Worker 线程负责核心计算，
 * 主线程通过轮询接口获取处理结果，实现数据接收与处理的解耦。
 */
class LidarSub {
public:
    LidarSub(ros::NodeHandle& nh, const std::string& topic, int frame_quantity = 10, double timeout_sec = 5.0);
    ~LidarSub();
   
    /**
     * @brief 获取最新处理完成的点云数据
     * @param out_frame 输出参数，若获取成功则填充点云与时间戳
     * @return 若存在新的处理结果返回 true，否则返回 false
     */
    bool getProcessedCloud(ProcessedCloud& out_frame);

    /**
     * @brief 获取雷达当前连接状态
     */
    bool isConnected() const;

    /**
     * @brief 检查连接状态是否发生变化（边沿触发）
     * @return 若状态发生跳变返回 true 并重置标志位，否则返回 false
     */
    bool checkStatusChange();

private:
    // ==========================================
    // 内部回调与后台线程函数
    // ==========================================
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_data);
    void timerCallback(const ros::TimerEvent& event); 
    void workerThreadFunc(); 

    // ==========================================
    // 成员变量
    // ==========================================
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Timer timer_; 

    std::string topic_name_;
    int frame_quantity_;
    double timeout_sec_;

    // 原始点云数据队列 (ROS回调线程 -> Worker线程)
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> raw_cloud_queue_;
    std::mutex raw_mutex_;
    std::condition_variable raw_cv_; 

    // 多帧滑动窗口 (Worker线程内部维护)
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> sliding_window_;
    std::atomic<bool> need_reset_window_{false}; // 通知 Worker 线程清空滑动窗口的标志

    // 处理结果缓存 (Worker线程 -> 主线程)
    ProcessedCloud latest_result_;
    std::mutex result_mutex_;
    bool has_new_result_ = false;

    // 线程与状态管理
    std::thread worker_thread_;
    std::atomic<bool> running_{true};
    
    ros::Time last_message_time_;
    std::mutex state_mutex_; // 保护心跳时间戳的互斥锁

    std::atomic<bool> is_lidar_connect_{false};
    std::atomic<bool> status_changed_flag_{false}; // 连接状态变化标志
};

} // namespace zl_lidar

#endif // LIDAR_SUB_H