#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include "lidar_sub.h"
#include "pointcloud_processor.h"

int main(int argc, char** argv) {
    // 1. ROS 节点初始化
    ros::init(argc, argv, "lidar_preprocess_node");
    ros::NodeHandle nh("~");

    // 2. 配置组件参数
    std::string lidar_topic = "/rslidar_points";  // 点云话题名称（launch 可通过 ~input_topic 覆盖）
    int frame_quantity = 1;                  // 滑动窗口累加帧数：1 表示不累加，逐帧输出
    double timeout_sec = 20.0;          // 心跳超时时间（秒）

    // 点云处理参数（全部可通过 launch 覆盖，改参数不需要重新编译）
    double leaf_size = 0.02;            // 体素降采样叶大小（m）：越大越省带宽、细节越少
    // 安装姿态修正（度，按 Z-Y-X 顺序旋转点云）：修正雷达出厂坐标系与装车姿态的差异。
    // 当前车型为「雷达翻转立起、+Z（旋转轴）朝车头」，现场实测值 pitch=-90、yaw=180 由
    // launch 提供；这里的三元组是代码默认值，全 0 表示不旋转。
    // 角度定义在车体坐标系（+X 前 / +Y 左 / +Z 上），作用对象是点云本身。
    double rotate_roll = 0.0;           // 绕车体前后轴（X）旋转角（度）
    double rotate_pitch = 0.0;          // 绕车体左右轴（Y）旋转角（度）
    double rotate_yaw = 0.0;            // 绕车体垂直轴（Z）旋转角（度）
    bool range_filter_enable = false;   // 空间范围滤波总开关
    double range_x_min = -20.0;         // 前后范围（m），+X 为前方
    double range_x_max = 20.0;
    double range_y_min = -20.0;         // 左右范围（m），+Y 为左侧
    double range_y_max = 20.0;
    double range_z_min = -1.0;          // 高度范围（m），+Z 向上
    double range_z_max = 5.0;

    // 从参数服务器读取配置（launch 中设置的私有参数，节点句柄为 "~"）
    nh.param<std::string>("input_topic", lidar_topic, lidar_topic);
    nh.param<int>("frame_quantity", frame_quantity, frame_quantity);
    nh.param<double>("leaf_size", leaf_size, leaf_size);
    nh.param<double>("rotate_roll", rotate_roll, rotate_roll);
    nh.param<double>("rotate_pitch", rotate_pitch, rotate_pitch);
    nh.param<double>("rotate_yaw", rotate_yaw, rotate_yaw);
    nh.param<bool>("range_filter_enable", range_filter_enable, range_filter_enable);
    nh.param<double>("range_x_min", range_x_min, range_x_min);
    nh.param<double>("range_x_max", range_x_max, range_x_max);
    nh.param<double>("range_y_min", range_y_min, range_y_min);
    nh.param<double>("range_y_max", range_y_max, range_y_max);
    nh.param<double>("range_z_min", range_z_min, range_z_min);
    nh.param<double>("range_z_max", range_z_max, range_z_max);

    ROS_INFO("[Main] Config loaded -> Topic: %s, Frames: %d, Timeout: %.1fs",
             lidar_topic.c_str(), frame_quantity, timeout_sec);
    ROS_INFO("[Main] Cloud processing -> leaf: %.3fm, RPY rotation: %.2f/%.2f/%.2f (deg), "
             "RangeFilter: %d (x %.1f~%.1f, y %.1f~%.1f, z %.1f~%.1f)",
             leaf_size, rotate_roll, rotate_pitch, rotate_yaw, range_filter_enable,
             range_x_min, range_x_max, range_y_min, range_y_max, range_z_min, range_z_max);

    // 4. 实例化雷达预处理组件
    zl_lidar::LidarSub lidar_sub(nh, lidar_topic, frame_quantity, timeout_sec);

    // 点云处理器：目前只做体素降采样，后续可在此串联旋转、地面剔除、BEV 投影等步骤。
    // 无状态、无 ROS 依赖，直接在主线程使用。
    PointCloudProcessor cloud_processor;

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

            // 在这里扩展点云旋转和滤波
            // 体素降采样：默认 2 cm 体素合并近邻点，降低下游带宽与后续滤波计算量。
            // leaf_size 由 launch 参数 ~leaf_size 提供。
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;
            cloud_processor.Leaf_Filter(result.cloud, filtered_cloud, static_cast<float>(leaf_size));

            // ============ 激光雷达标准坐标系参考 ============
            //  右手系，速腾 RS 系列（含 Airy360）驱动输出一致：
            //   原点：雷达旋转中心（安装基座中心上方）
            //   +X  ：雷达正前方，即方位角 0° 的线束零点方向
            //   +Y  ：雷达左侧
            //   +Z  ：垂直向上，与旋转轴重合
            // ================================================
            // 安装姿态修正：按 Z-Y-X 顺序绕车体三轴旋转点云，把雷达本体坐标系转到车体坐标系。
            // 角度来自 launch（~rotate_roll / ~rotate_pitch / ~rotate_yaw，单位度），
            // 当前车型实测值 pitch=-90、yaw=180；三值全 0 时跳过，避免一次多余的点云拷贝。
            // RotateRPY 支持输入输出传同一指针，因此只用一个 processed_cloud。
            pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud = filtered_cloud;
            if (rotate_roll != 0.0 || rotate_pitch != 0.0 || rotate_yaw != 0.0) {
                cloud_processor.RotateRPY(processed_cloud, processed_cloud,
                                          static_cast<float>(rotate_roll),
                                          static_cast<float>(rotate_pitch),
                                          static_cast<float>(rotate_yaw));
            }

            // 空间范围滤波：由 launch 开关 ~range_filter_enable 控制（现场默认开启）。
            // 在姿态修正之后、车体坐标系下，按前后（x）、左右（y）、上下（z）三段依次裁剪，
            // 范围来自 launch 参数 ~range_x_min/max、~range_y_min/max、~range_z_min/max。
            if (range_filter_enable) {
                cloud_processor.FilterX(processed_cloud, processed_cloud,
                                        static_cast<float>(range_x_min), static_cast<float>(range_x_max));
                cloud_processor.FilterY(processed_cloud, processed_cloud,
                                        static_cast<float>(range_y_min), static_cast<float>(range_y_max));
                cloud_processor.FilterHeight(processed_cloud, processed_cloud,
                                             static_cast<float>(range_z_min), static_cast<float>(range_z_max));
            }

            sensor_msgs::PointCloud2 out_msg;
            // 填充点云（发布降采样、方向修正、范围滤波后的最终结果）
            pcl::toROSMsg(*processed_cloud, out_msg);
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