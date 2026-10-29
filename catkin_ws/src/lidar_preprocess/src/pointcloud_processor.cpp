#include "pointcloud_processor.h"
#include <pcl/io/pcd_io.h>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <cctype>

// ── 辅助：检查路径 ──
static bool pathExists(const std::string& path) 
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0);
}

// ── 辅助：列出目录下所有 .pcd 文件 ──
static std::vector<std::string> listPCDFiles(const std::string& folder) 
{
    std::vector<std::string> files;
    DIR* dir = opendir(folder.c_str());
    if (!dir) return files;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name == "." || name == "..") continue;
        if (name.size() > 4) {
            std::string ext = name.substr(name.size() - 4);
            for (char& c : ext) c = std::tolower(c);
            if (ext == ".pcd") {
                files.push_back(folder + "/" + name);
            }
        }
    }
    closedir(dir);
    std::sort(files.begin(), files.end()); // 保证顺序一致
    return files;
}

// ── I/O 实现 ──
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudProcessor::LoadPCD(const std::string& path) 
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcl::io::loadPCDFile(path, *cloud) == -1) {
        std::cerr << "[Warn] Failed to load: " << path << std::endl;
        return nullptr;
    }
    return cloud;
}

bool PointCloudProcessor::MergePCDFolder(const std::string& dir, int max_frames, pcl::PointCloud<pcl::PointXYZI>::Ptr& out) 
{
    if (!pathExists(dir)) {
        std::cerr << "[Error] Folder not found: " << dir << std::endl;
        return false;
    }
    auto files = listPCDFiles(dir);
    if (files.empty()) {
        std::cerr << "[Error] No .pcd files in: " << dir << std::endl;
        return false;
    }
    
    // 预分配内存 (按每帧 1 万点估算)
    out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    out->reserve(std::min(max_frames, static_cast<int>(files.size())) * 10000);
    
    int count = 0;
    for (const auto& f : files) {
        if (count >= max_frames) break;
        auto cloud = LoadPCD(f);
        if (!cloud) continue;
        *out += *cloud;  // PCL 重载了 +=
        ++count;
    }
    out->height = 1;
    out->width = out->points.size();
    std::cout << "[Info] Merged " << count << " frames, total " << out->size() << " points." << std::endl;
    return !out->empty();
}

// ── 姿态修正实现 ──
// 绕车体三轴旋转任意角度：按 Z-Y-X 顺序组合（R = Rz·Ry·Rx），角度单位为度。
// 用于修正雷达出厂坐标系与装车姿态的差异（如 +Z 朝前这类 90° 安装）。
// 只做旋转、不做平移；先算到局部云再赋给 out，因此 in 与 out 可传同一指针；
// in 为空时输出空云。header 与 is_dense 原样保留。
void PointCloudProcessor::RotateRPY(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                                    float roll_deg, float pitch_deg, float yaw_deg) {
    if (!in) {
        out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        return;
    }

    // 平移量固定为 0：本函数只修正姿态，安装位置偏移不在这里处理
    const float kDegToRad = static_cast<float>(M_PI) / 180.0f;
    Eigen::Affine3f tf = pcl::getTransformation(0.0f, 0.0f, 0.0f,
                                                roll_deg  * kDegToRad,
                                                pitch_deg * kDegToRad,
                                                yaw_deg   * kDegToRad);

    auto rotated = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*in, *rotated, tf);  // width/height 由 PCL 按输出点数重设
    rotated->header = in->header;                 // 保留时间戳与 frame_id
    rotated->is_dense = in->is_dense;             // NaN 点仍按输入标记，不改变语义
    out = rotated;
}

// ── 滤波实现 ──
// 点云下采样
void PointCloudProcessor::Leaf_Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
                                      pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, 
                                      float leaf_size) {
    // 1. 健壮性检查
    if (!cloud_in || cloud_in->empty()) {
        std::cerr << "[Warn] Leaf_Filter: Input cloud is empty." << std::endl;
        cloud_out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        return;
    }
    
    // 2. 确保输出指针有效
    if (!cloud_out) {
        cloud_out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    // 3. 使用 ApproximateVoxelGrid (近似体素滤波)
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    
    // 可选：设置下采样方法 (LEAF_CENTER 取体素中心点，CENTROID 取质心)
    // sor.setDownsampleAllData(true); // 如果点云有强度/颜色字段，设为 true 会计算平均值
    
    sor.filter(*cloud_out);

    // 4. 重置元数据 (PCL 算法强依赖正确的 height/width)
    cloud_out->height = 1;
    cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());

    // // 5. 日志
    // std::cout << "[Filter] ApproxVoxelDownsample: leaf=" << leaf_size << "m, "
    //           << "pts: " << cloud_in->size() << " -> " << cloud_out->size() << std::endl;
}

void PointCloudProcessor::FilterGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in, pcl::PointCloud<pcl::PointXYZI>::Ptr& out_nonground, pcl::PointCloud<pcl::PointXYZI>::Ptr& out_ground, float dist_threshold) {
    out_nonground = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    out_ground = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (in->empty()) return;

    // RANSAC 平面拟合
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_threshold);
    seg.setMaxIterations(200);         // 防止稀疏点云 early stop
    seg.setProbability(0.99f);
    // 法向约束：期望地面法向 = (0,0,1)
    Eigen::Vector3f vertical_axis(0.0f, 0.0f, 1.0f);
    seg.setAxis(vertical_axis);
    seg.setEpsAngle(10.0f * M_PI / 180.0f);  // ±15°

    seg.setInputCloud(in);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.empty()) {
        std::cerr << "[Warn] RANSAC found no ground plane." << std::endl;
        *out_nonground = *in;
        return;
    }
    
    // 提取地面/非地面
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(in);
    extract.setIndices(inliers);
    
    extract.setNegative(false);
    extract.filter(*out_ground);   // 地面
    extract.setNegative(true);
    extract.filter(*out_nonground); // 非地面
}

// 前后方向滤波：只保留 x ∈ [min_x, max_x] 的点，其余丢弃。
// 在雷达坐标系下 +X 为前方；in 为空时输出空云。
// 与旋转函数一致，先算到局部云再写回 out，支持 in/out 传同一个指针。
void PointCloudProcessor::FilterX(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                                  float min_x, float max_x) {
    auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (in) {
        for (const auto& p : in->points) {
            if (p.x >= min_x && p.x <= max_x) {
                filtered->points.push_back(p);
            }
        }
    }
    filtered->height = 1;
    filtered->width = filtered->points.size();
    out = filtered;
}

// 左右方向滤波：只保留 y ∈ [min_y, max_y] 的点，其余丢弃。
// 在雷达坐标系下 +Y 为左侧；in 为空时输出空云。
void PointCloudProcessor::FilterY(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                                  float min_y, float max_y) {
    auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (in) {
        for (const auto& p : in->points) {
            if (p.y >= min_y && p.y <= max_y) {
                filtered->points.push_back(p);
            }
        }
    }
    filtered->height = 1;
    filtered->width = filtered->points.size();
    out = filtered;
}

// 高度滤波：只保留 z ∈ [min_z, max_z] 的点，其余丢弃。
// 水平安装时可用 [-1, 5] 之类区间裁掉地面噪声与高层杂点；in 为空时输出空云。
void PointCloudProcessor::FilterHeight(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                                       float min_z, float max_z) {
    auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (in) {
        for (const auto& p : in->points) {
            if (p.z >= min_z && p.z <= max_z) {
                filtered->points.push_back(p);
            }
        }
    }
    filtered->height = 1;
    filtered->width = filtered->points.size();
    out = filtered;
}

// ── BEV 投影实现 ──
cv::Mat PointCloudProcessor::ProjectToBEV(const pcl::PointCloud<pcl::PointXYZI>::Ptr& points,float x_min, float x_max, float y_min, float y_max, float resolution) {
    int w = static_cast<int>((x_max - x_min) / resolution);
    int h = static_cast<int>((y_max - y_min) / resolution);
    cv::Mat bev = cv::Mat::zeros(h, w, CV_8UC1);
    
    if (!points || points->empty()) return bev;
    
    for (const auto& p : points->points) {
        // 边界检查
        if (p.x < x_min || p.x > x_max || p.y < y_min || p.y > y_max) continue;
        
        int col = static_cast<int>((p.x - x_min) / resolution);
        int row = static_cast<int>((p.y - y_min) / resolution);
        
        if (row >= 0 && row < h && col >= 0 && col < w) {
            bev.at<uchar>(row, col) = 255;  // 占用=白
        }
    }
    return bev;
}


