#include<iostream>
#include<fstream>
#include<sstream>
#include<cmath>
#include<random>
#include<pcl-1.10/pcl/io/pcd_io.h>
#include<pcl-1.10/pcl/point_types.h>
#include<pcl-1.10/pcl/point_cloud.h>
#include<pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.10/pcl/visualization/point_cloud_color_handlers.h>
#include<pcl-1.10/pcl/io/pcd_io.h>
#include<eigen3/Eigen/Dense>
#include<opencv2/opencv.hpp>

//read csv
std::string File="/home/cuckoo/VSC/vsc cpp/source/pc_rot.csv";

std::vector<std::vector<double>> Read(std::string &File){
    //读入csv文件，csv文件可以看作是一个二维的字符串类型数组，每次取一行来处理
    std::vector<std::vector<double>> res;
    
    std::ifstream file(File);

    if(!file.is_open()){
        std::cerr<<"错误：文件 "<<File<<" 未打开"<<std::endl;
        return res;
    }

    std::string line;

    while(std::getline(file,line)){
        std::stringstream ss(line);//读入字符串流中方便分开处理
        std::string v;
        std::vector<double> row;
        while(std::getline(ss,v,',')){//从字符串流中不断读入逗号之间的数字
            row.push_back(std::stof(v));//stof可以将字符串转换为float
        }
        res.push_back(row);
    }
    file.close();
    if(res.empty()){
        std::cerr<<"错误：文件读入结果为空"<<std::endl;
        return res;
    }
    
    int h = res.size(),w = res[0].size(); 
    std::cout<<"已经读入"<<h<<"x"<<w<<"大小的csv文件"<<std::endl;

    return res;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_build(
    std::vector< std::vector<double> > &data ){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(auto line : data){
        double X = line[0],Y = line[1],Z = line[2];
        pcl::PointXYZ point;
        point.x=X;
        point.y=Y;
        point.z=Z;
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = 0;
    return cloud;

}

void savecsv(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            std::string &ofile){
        //把读建立好的点云再保存回csv文件
        std::ofstream outfile(ofile);
        outfile<<std::fixed<<std::setprecision(6);//防止科学记数法，6位小数精度
        
        auto &points=cloud->points;
        size_t total=points.size();

        for(size_t i=0;i<total;i++){
            outfile<<points[i].x<<","<<points[i].y<<","<<points[i].z<<endl;
        }
        outfile.close();
}

void depth_map_build(
    int h, int w,
    double fx, double fy, double cx, double cy,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

    
    cv::Mat depth_map = cv::Mat::zeros(h,w,CV_32FC1);
    for (const auto& point : cloud->points) {

        float x = point.x;
        float y = point.y;
        float z = point.z;
        // 过滤掉相机背后的点 (Z <= 0)
        if (z <= 0.001f) continue;

        // 投影公式: u = fx * (X/Z) + cx, v = fy * (Y/Z) + cy

        int u = static_cast<int>(fx * (x / z) + cx);

        int v = static_cast<int>(fy * (y / z) + cy);

        // 检查边界

        if (u >= 0 && u < w && v >= 0 && v < h) {

            float current_depth = depth_map.at<float>(v, u);

            // Z-Buffering: 如果当前点更近 (z 更小)，或者该像素还没有值 (0)

            // 注意：如果初始化为0，需要特殊处理第一个点

            if (current_depth == 0.0f || z < current_depth) {

                depth_map.at<float>(v, u) = z;

            }

        }
    }
    cv::imwrite("depth_map.png",depth_map);

    cv::Mat display_img;

    cv::normalize(depth_map, display_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::imshow("Depth Map", display_img);

    cv::waitKey(0);
}



int main(){
    auto depth=Read(File);
    
    if(depth.empty()){
        cout<<"depth is empty"<<endl;
        return 0;
    }

    double fx=795.209,fy=793.957,cx=332.031,cy=231.308;
    
    auto cloud=cloud_build(depth);
    int h = 640,w = 480;

    depth_map_build(h,w,fx,fy,cx,cy,cloud);

    // std::string outputcsv="pc.csv";
    // savecsv(cloud,outputcsv);

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"); // 将点云添加到视图器中
    // viewer->setBackgroundColor (0, 0, 0); // 设置背景颜色为黑色
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.2, "sample cloud"); // 设置点的大小
    // viewer->addCoordinateSystem (0.1); // 显示坐标轴
    // viewer->initCameraParameters (); // 设置相机参数以更好地显示点云
    // while (!viewer->wasStopped ()) { // 循环直到用户关闭视图器窗口
    //     viewer->spinOnce (100); // 处理视图器事件并更新屏幕内容
    // }

  return 0; 

}