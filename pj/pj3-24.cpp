#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// std::string File="../source/";

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

void Cloud_Build(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
}

int main()
{
    std::string File = "../source/";

    return 0;
}