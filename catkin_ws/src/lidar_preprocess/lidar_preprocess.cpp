#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include "lidar_sub.h"

int main(int argc, char** argv) {
    // 1. ROS 节点初始化
    ros::init(argc, argv, "lidar_preprocess_node");
    ros::NodeHandle nh("~");

    // 2. 配置组件参数
    std::string lidar_topic = "/rslidar_points";  // 点云话题名称（launch 可通过 ~input_topic 覆盖）
    int frame_quantity = 10;                 // 滑动窗口累加帧数
    double timeout_sec = 20.0;          // 心跳超时时间（秒）

    // 从参数服务器读取配置（launch 中设置的私有参数，节点句柄为 "~"）
    nh.param<std::string>("input_topic", lidar_topic, lidar_topic);
    nh.param<int>("frame_quantity", frame_quantity, 10);
    ROS_INFO("[Main] Config loaded -> Topic: %s, Frames: %d, Timeout: %.1fs", 
             lidar_topic.c_str(), frame_quantity, timeout_sec);

    // 4. 实例化雷达预处理组件
    zl_lidar::LidarSub lidar_sub(nh, lidar_topic, frame_quantity, timeout_sec);

    // 5. 初始化 ROS 发布者
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/perception_sensors/processed_lidar_cloud", 10);
    // latched=true 确保后启动的 downstream 节点能立即收到最新的心跳状态
    ros::Publisher pub_status = nh.advertise<std_msgs::Bool>("/perception_sensors/processed_lidar_connection_status", 10, true); 

    // 6. 设定主循环频率
    ros::Rate loop_rate(10); 

    // 定义时间偏移量及初始化标志
    ros::Duration time_offset(0.0);
    bool offset_initialized = false;

    ROS_INFO("[Main] LiDAR preprocess node started. Control loop active at 10Hz.");

    // 7. 主循环
    while (ros::ok()) {
        // 触发 ROS 底层回调，接收雷达数据并唤醒后台处理线程
        ros::spinOnce(); 

        // 获取处理完成的点云数据
        zl_lidar::ProcessedCloud result;
        if (lidar_sub.getProcessedCloud(result)) {

            // ================= 计算并应用时间偏移 =================
            // 仅在第一次成功获取到点云时，计算时间基准偏移量
            if (!offset_initialized) {
                // 偏移量 = 当前ROS绝对时间 - 雷达硬件相对时间
                time_offset = ros::Time::now() - result.timestamp;
                offset_initialized = true;
                ROS_INFO("[Main] LiDAR time offset initialized: %.3f seconds (Aligned to ROS absolute time)", 
                         time_offset.toSec());
            }

            sensor_msgs::PointCloud2 out_msg;
            // 填充点云
            pcl::toROSMsg(*result.cloud, out_msg);
            // 填充id和时间戳
            out_msg.header.frame_id = "rslidar";
            out_msg.header.stamp = result.timestamp + time_offset;   // 将雷达的相对时间，转换为与视觉同基准的 ROS 绝对时间
            // 发布处理后的点云
            pub_cloud.publish(out_msg);
        }

        // 检查雷达连接状态是否发生变化
        if (lidar_sub.checkStatusChange()) {
            std_msgs::Bool status_msg;
            status_msg.data = lidar_sub.isConnected();
            // 发布激光雷达状态
            pub_status.publish(status_msg);
            if (!status_msg.data) {
                ROS_WARN("[Main] LiDAR disconnected. Buffers cleared.");
            } else {
                ROS_INFO("[Main] LiDAR connection recovered.");
            }
        }

        // 按照设定频率休眠
        loop_rate.sleep();
    }

    return 0;
}