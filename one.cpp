#include<iostream>
#include<cmath>
#include<random>
#include<pcl-1.10/pcl/io/pcd_io.h>
#include<pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.10/pcl/visualization/point_cloud_color_handlers.h>

#define PointXYZ PointXYZRGB

int main(){

    std::srand(std::time(0));
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width    = 0; // 设置点云宽度
    cloud->height   = 1; // 设置点云高度（对于非组织化点云，高度为1）
    cloud->is_dense = false; // 点云可能包含无效的点（例如NaN）
    //cloud->points.resize(cloud->width * cloud->height); // 分配内存

    // // 填充点云数据
    // for (size_t i = 0; i < cloud->points.size(); ++i) {
    //     cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0); // 随机生成x坐标
    //     cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0); // 随机生成y坐标
    //     cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0); // 随机生成z坐标
    // }
    double r=10;
    for(double th=0;th<2*M_PI;th+=0.1){
        for(double phi=0;phi<2*M_PI;phi+=0.1){
            pcl::PointXYZ point;
            point.x=r*sin(phi)*cos(th);
            point.y=r*sin(phi)*sin(th);
            point.z=r*cos(phi);
            cloud->points.push_back(point);  
            cloud->width++;
        }
    }
    
    for(auto &point:cloud->points){
        point.r= std::rand()%255;
        point.b= std::rand()%255;
        point.g= std::rand()%255;
    }

    // 保存点云到文件
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud); // 使用ASCII格式保存点云到文件
    std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"); // 将点云添加到视图器中
    viewer->setBackgroundColor (0, 0, 0); // 设置背景颜色为黑色
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud"); // 设置点的大小
    viewer->addCoordinateSystem (1.0); // 显示坐标轴
    viewer->initCameraParameters (); // 设置相机参数以更好地显示点云
    while (!viewer->wasStopped ()) { // 循环直到用户关闭视图器窗口
        viewer->spinOnce (100); // 处理视图器事件并更新屏幕内容
    }

  return 0; 

}