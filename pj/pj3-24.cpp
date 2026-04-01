#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// std::string File="../source/";

// 从csv文件读取数据
std::vector<std::vector<float>> read(std::string File)
{
    std::vector<std::vector<float>> res;
    std::ifstream file(File);

    if (!file.is_open()) {
        std::cerr << "错误：文件 " << File << " 未打开" << std::endl;
        return res;
    }

    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string v;
        std::vector<float> row;
        while (std::getline(ss, v, ',')) {
            row.push_back(std::stof(v));
        }
        res.push_back(row);
    }
    file.close();
    if (res.empty()) {
        std::cerr << "错误：文件读入结果为空" << std::endl;
        return res;
    }

    int h = res.size(), w = res[0].size();
    std::cout << "已经读入" << h << "x" << w << "大小的csv文件" << std::endl;

    return res;
}

void Cloud_Build(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<std::vector<float>>& data) // 将data读入cloud点云数据
{
    if (data.empty()) {
        std::cerr << "错误：点云数据为空，无法生成对应点云" << std::endl
                  << "方法：Cloud_Build 未执行" << std::endl;
        return;
    }
    for (auto line : data) {
        float X = line[0], Y = line[1], Z = line[2];
        pcl::PointXYZ point;
        point = { X, Y, Z };
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = 0;
}

// 从点云构建深度图 参数：生成图像的高度与宽度 相机内参 点云对象
cv::Mat Depth_Map_Build(
    int h, int w,
    float fx, float fy, float cx, float cy,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    cv::Mat depth_map = cv::Mat::zeros(h, w, CV_32FC1);
    for (auto& point : cloud->points) {
        float x = point.x, y = point.y, z = point.z;

        // 如果点在相机背面可以过滤掉
        if (z <= 0.001f) {
            continue;
        }
        // 投影公式
        int u = fx * (x / z) + cx;
        int v = fy * (y / z) + cy;
        // 检查边界
        if (u >= 0 && u < w && v >= 0 && v < h) {
            float depth = depth_map.at<float>(v, u);

            // Z-Buffering: 如果当前点更近 (z 更小)，或者该像素还没有值 (0)

            // 注意：如果初始化为0，需要特殊处理第一个点

            if (depth == 0.0f || z < depth) {
                depth_map.at<float>(v, u) = z;
            }
        }
    }

    return depth_map;
}

void Savecsv_From_DepthMap(
    cv::Mat& depth_map, std::string save_path)
{
    if (depth_map.empty()) {
        std::cerr << "错误：传入的深度图数据为空" << std::endl;
        return;
    }

    std::ofstream file(save_path);
    if (!file.is_open()) {
        std::cerr << "错误：无法创建 csv 文件: " << save_path << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);
    int rows = depth_map.rows;
    int cols = depth_map.cols;

    for (int r = 0; r < rows; ++r) {
        // 获取当前行的指针
        const float* row_ptr = depth_map.ptr<float>(r);

        for (int c = 0; c < cols; ++c) {
            float val = row_ptr[c]; // 获取真实的深度值
            if(val>0.0001f)
            file << val;
            else file<<INFINITY;

            if (c < cols - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "成功保存 CSV 文件: " << save_path << std::endl;
}

int main()
{
    std::string File = "../source/pc_rot.csv";

    auto depth = read(File);
    if (depth.empty()) {
        std::cout << "data is empty, process will be killed" << std::endl;
        return 0;
    }

    float fx = 795.209, fy = 793.957, cx = 332.031, cy = 231.308;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Cloud_Build(cloud, depth);

    int h = 480, w = 750;
    cv::Mat Depth_Map = Depth_Map_Build(h, w, fx, fy, cx, cy, cloud);

    std::string img_path = "build/depth_map.png";
    std::string save_path = "dep_rot.csv";
    Savecsv_From_DepthMap(Depth_Map, save_path);

    cv::Mat img;
    cv::normalize(Depth_Map, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite("depth_map.png", img);
    cv::imshow("Depth Map", img);
    cv::waitKey(0);

    return 0;
}