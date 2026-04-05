#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat read(std::string file_path)
{

    cv::Mat img;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "错误：从文件路径 " << file_path << " 未能打开" << std::endl;
        return img;
    }

    // 读取图像数据
    // 每行代表一行像素，每个像素值用逗号分隔
    std::vector<std::vector<float>> data;
    std::string line;
    while (std::getline(file, line)) {

        std::stringstream ss(line);

        std::string value;
        std::vector<float> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stof(value));
        }
        data.push_back(row);
    }

    if (data.empty()) {
        std::cerr << "错误：从文件路径 " << file_path << " 读取的数据为空" << std::endl;
        return img;
    }
    file.close();

    // mat函数将二维vector数据转换为cv::Mat
    cv::Mat mat(data.size(), data[0].size(), CV_32F);

    for (int i = 0; i < data.size(); i++) {
        for (int j = 0; j < data[0].size(); j++) {
            mat.at<float>(i, j) = data[i][j];
        }
    }
    return mat;
}


//使用滑动窗口和SSD进行视差计算
cv::Mat search(cv::Mat& imgL, cv::Mat& imgR,
    int window_size, int max_disparity)
{
    cv::Mat match = cv::Mat::zeros(imgL.size(), CV_32F);
    //窗口半径
    int half_window = window_size / 2;

    for (int y = half_window; y < imgL.rows - half_window; y++) {
        for (int x = half_window; x < imgL.cols - half_window; x++) {

            //左图像中的窗口区域
            cv::Rect left_rect(x - half_window, y - half_window, window_size, window_size);
            cv::Mat left_window = imgL(left_rect);//提取窗口图像

            //确定搜索范围
            int start_pos = std::max(half_window, x - max_disparity);
            int end_pos = std::min(imgR.cols - half_window, x + max_disparity);
            //初始化SSD值与最佳匹配位置
            double min_val = 1e9;
            int best_x = x;

            for (int d = start_pos; d < end_pos; d++) {
                cv::Rect right_rect(d - half_window, y - half_window, window_size, window_size);
                cv::Mat right_window = imgR(right_rect);
                cv::Mat diff;
                cv::absdiff(left_window, right_window, diff);
                //计算SSD
                double ssd = cv::norm(diff, cv::NORM_L2);
                if (ssd < min_val) {//如果SSD更小就更新最佳位置
                    min_val = ssd;
                    best_x = d;
                }
            }
            float disparity = x - best_x; // 计算视差，注：正视差表示右图像中的匹配点在左图象点点左侧
            // 将视差值存储到视差图中
            match.at<float>(y, x) = disparity;
        }
    }
    return match;
}

void save_csv(const std::string& filename, const cv::Mat& mat)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            // 写入数据
            file << mat.at<float>(i, j);
            if (j < mat.cols - 1) {
                file << ",";
            }
        }
        //记得加换行符
        file << "\n";
    }
    file.close();
    std::cout << "成功保存文件: " << filename << std::endl;
}

int main()
{

    std::string left_image_path = "../source/aL_gray.csv";
    std::string right_image_path = "../source/aR_gray.csv";
    cv::Mat imgL = read(left_image_path);
    cv::Mat imgR = read(right_image_path);
    if (imgL.empty() || imgR.empty()) {
        std::cerr << "错误：未能成功读取图像数据" << std::endl;
        return -1;
    }
    //注：CV_8U是unsign的8位/像素 一个像素的值在0-255区间（可视）
    //CV_32F是32位float类型 每个像素值的取值范围即为float类型的范围，如果要显示图像必须转化为CV_8U格式
    //使用cv::imshow用来显示CV_32F图像时，OpenCV默认将[0.0,1.0]的数据范围映射到[0,255]
    cv::Mat match_img = search(imgL, imgR, 5, 64);
    save_csv("../source/match.csv", match_img);

    //为了使得图像可见，需要对得到的图像进行归一化处理
    cv::normalize(match_img, match_img, 0, 255, cv::NORM_MINMAX, CV_8U);

    //因为读取的时候统一使用了CV_32F的图像类型储存，为了显示图像，必须将其转化为CV_8U图像
    imgL.convertTo(imgL, CV_8U);
    imgR.convertTo(imgR, CV_8U);

    cv::imshow("Left Image", imgL);
    cv::imshow("Right Image", imgR);
    cv::imshow("Disparity", match_img);

    cv::waitKey(0);

    return 0;
}
