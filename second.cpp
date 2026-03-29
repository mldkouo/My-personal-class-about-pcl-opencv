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

//read csv
std::string File="/home/cuckoo/VSC/vsc cpp/img_dep_640x480(2).csv";

std::vector<std::vector<double>> Read(std::string &File){
    //读入csv文件，csv文件可以看作是一个二维的字符串类型数组，每次取一行来处理
    std::vector<std::vector<double>> res;
    
    std::ifstream file(File);
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
    return res;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_build(
    const std::vector<std::vector<double>> &depth,
    double fx, double fy, double cx, double cy){
    //参数：读好的csv文件（也就是深度图）和四个相机内参
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    //Ptr本质上是个cpp的智能指针

    int height = depth.size();//第一维的容量大小
    int width = depth[0].size();//第二维的容量大小

    for (int u = 0; u < width; ++u)  {
        for (int v = 0; v < height; ++v){

            float Z=depth[v][u];

            if (Z<=0) continue; 
            //如果csv的单位是米，那么要记得除以1000
            //Z /= 1000.0;
            float X = (u - cx) * Z / fx;//公式
            float Y = (v - cy) * Z / fy;

            pcl::PointXYZ point;
            point.x = X;
            point.y = Y;
            point.z = Z;

            cloud->points.push_back(point);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

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


int main(){
    auto depth=Read(File);
    
    if(depth.empty()){
        cout<<"depth is empty"<<endl;
        return 0;
    }

    double fx=795.209,fy=793.957,cx=332.031,cy=231.308;
    auto cloud=cloud_build(depth,fx,fy,cx,cy);
    pcl::io::savePCDFileASCII("pc.pcd",*cloud);


    std::string outputcsv="pc.csv";
    savecsv(cloud,outputcsv);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"); // 将点云添加到视图器中
    viewer->setBackgroundColor (0, 0, 0); // 设置背景颜色为黑色
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud"); // 设置点的大小
    viewer->addCoordinateSystem (0); // 显示坐标轴
    viewer->initCameraParameters (); // 设置相机参数以更好地显示点云
    while (!viewer->wasStopped ()) { // 循环直到用户关闭视图器窗口
        viewer->spinOnce (100); // 处理视图器事件并更新屏幕内容
    }

  return 0; 

}