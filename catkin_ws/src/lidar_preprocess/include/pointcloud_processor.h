#pragma once
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>  
#include <pcl/common/transforms.h>  
#include <pcl/common/centroid.h>
#include <Eigen/Dense> // 确保包含 Eigen 核心
#include <pcl/filters/approximate_voxel_grid.h>
#include <opencv2/opencv.hpp>

class PointCloudProcessor {
public:
    PointCloudProcessor() = default;
    ~PointCloudProcessor() = default;

    // ── I/O ──
    // 加载单帧
    pcl::PointCloud<pcl::PointXYZI>::Ptr LoadPCD(const std::string& path);
    // 合并文件夹下多帧 (核心需求)
    bool MergePCDFolder(const std::string& dir, int max_frames, pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    // ── 姿态修正 ──
    /**
     * @brief 绕车体三轴按 Z-Y-X 顺序旋转任意角度，修正激光雷达安装姿态。
     *
     * 用于修正雷达出厂坐标系与装车姿态的差异（如「出厂 +Z 朝上、装车后 +Z 朝前」
     * 这类 90° 安装差异）。等价旋转矩阵 R = Rz(yaw)·Ry(pitch)·Rx(roll)，作用到点上
     * 等价于先 roll、再 pitch、最后 yaw（与 pcl::getTransformation 的约定一致）。
     *
     * 角度定义在车体坐标系（+X 前 / +Y 左 / +Z 上），作用对象是点云本身；180° 也是合法
     * 输入（如 yaw=180 等价于点云绕垂直轴转半圈）。
     *
     * 常见安装形态：
     *   - 雷达顶面朝车头（旋转轴指向前方，本体 +X 朝下）：roll=0,   pitch=+90, yaw=0
     *   - 雷达翻转立起（旋转轴指向前方，本体 +X 朝上）：roll=0,   pitch=-90, yaw=180
     * 当前车型为后者，实测值写在 launch 里（pitch=-90、yaw=180）。实际安装总有几度误差，
     * 角度可直接填实测值（如 88.5）。
     *
     * @param in         输入点云；可与 out 传同一指针（原地旋转）
     * @param out        输出点云；in 为空时输出空云
     * @param roll_deg   绕车体 X 轴（前后轴）旋转角，单位：度
     * @param pitch_deg  绕车体 Y 轴（左右轴）旋转角，单位：度
     * @param yaw_deg    绕车体 Z 轴（垂直轴）旋转角，单位：度
     *
     * @note 只做旋转、不做平移，雷达安装位置偏移需由下游或外参另行处理；
     *       header（时间戳、frame_id）与 is_dense 原样保留。
     */
    void RotateRPY(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                   float roll_deg, float pitch_deg, float yaw_deg);

    // ── 滤波 ──
    // 体素下采样
    void Leaf_Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float leaf_size = 0.02f);

    // RANSAC地面剔除，同时输出地面和非地面
    void FilterGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& out_nonground,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& out_ground,
                      float dist_threshold = 0.15f);
    
    // 前后方向滤波：只保留 x ∈ [min_x, max_x]（+X 为雷达正前方）
    void FilterX(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                 float min_x, float max_x);

    // 左右方向滤波：只保留 y ∈ [min_y, max_y]（+Y 为雷达左侧）
    void FilterY(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                 float min_y, float max_y);

    // 高度滤波 [min_z, max_z]
    void FilterHeight(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                      float min_z, float max_z);

    // ── BEV投影 ──
    // 通用点云→二值占用图 (CV_8UC1)
    cv::Mat ProjectToBEV(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
                         float x_min, float x_max, 
                         float y_min, float y_max, 
                         float resolution);
    
private:
    // 内部辅助：世界坐标→图像像素
    inline cv::Point worldToPixel(float x, float y, 
                                  float x_min, float y_min, float resolution) const {
        int col = static_cast<int>((x - x_min) / resolution);
        int row = static_cast<int>((y - y_min) / resolution);
        return cv::Point(col, row);
    }
};