#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// --- 1. 核心数据提取逻辑 ---
bool prepareData(const std::vector<std::string>& leftImages, 
                 const std::vector<std::string>& rightImages,
                 cv::Size boardSize, float squareSize,
                 std::vector<std::vector<cv::Point2f>>& imgPointsL,
                 std::vector<std::vector<cv::Point2f>>& imgPointsR,
                 std::vector<std::vector<cv::Point3f>>& objPoints,
                 cv::Size& imgSize) {
    
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            obj.push_back(cv::Point3f(static_cast<float>(j) * squareSize, static_cast<float>(i) * squareSize, 0));

    for (size_t i = 0; i < leftImages.size(); i++) {
        
        cv::Mat imgL = cv::imread(leftImages[i], cv::IMREAD_GRAYSCALE);
        cv::Mat imgR = cv::imread(rightImages[i], cv::IMREAD_GRAYSCALE);

        if (imgL.empty() || imgR.empty()) continue;
        imgSize = imgL.size();

        std::vector<cv::Point2f> cornersL, cornersR;
        bool foundL = cv::findChessboardCorners(imgL, boardSize, cornersL);
        bool foundR = cv::findChessboardCorners(imgR, boardSize, cornersR);

        if (foundL && foundR) {
            cv::cornerSubPix(imgL, cornersL, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::cornerSubPix(imgR, cornersR, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            imgPointsL.push_back(cornersL);
            imgPointsR.push_back(cornersR);
            objPoints.push_back(obj);
        }
    }
    return !objPoints.empty();
}

// --- 2. 主执行逻辑 ---
void stereoProcess(const std::string& leftDir, const std::string& rightDir, const std::string& outDir) {
    cv::Size boardSize(9, 6); 
    float squareSize = 25.0f; 

    std::vector<std::string> leftPaths, rightPaths;
    cv::glob(leftDir + "/*.jpg", leftPaths);
    cv::glob(rightDir + "/*.jpg", rightPaths);

    std::vector<std::vector<cv::Point2f>> imgPointsL, imgPointsR;
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Size imgSize;

    if (!prepareData(leftPaths, rightPaths, boardSize, squareSize, imgPointsL, imgPointsR, objPoints, imgSize)) {
        std::cerr << "无法提取足够的角点数据。" << std::endl;
        return;
    }

    // A. 双目标定
    cv::Mat K1, D1, K2, D2, R, T, E, F;
    double rms = cv::stereoCalibrate(objPoints, imgPointsL, imgPointsR, K1, D1, K2, D2, imgSize, R, T, E, F,
                                     cv::CALIB_FIX_ASPECT_RATIO, 
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

    // 输出基本参数
    std::cout << "RMS Error: " << rms << std::endl;
    std::cout << "K1:\n" << K1 << "\nD1:\n" << D1 << std::endl;
    std::cout << "K2:\n" << K2 << "\nD2:\n" << D2 << std::endl;
    std::cout << "R:\n" << R << "\nT:\n" << T << std::endl;

    // B. 立体校正 (Stereo Rectification)
    // 计算校正映射，使左右图像行对齐
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, imgSize, R, T, R1, R2, P1, P2, Q);

    cv::Mat mapL1, mapL2, mapR1, mapR2;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, imgSize, CV_16SC2, mapL1, mapL2);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, imgSize, CV_16SC2, mapR1, mapR2);

    // C. 应用校正并保存
    for (size_t i = 0; i < leftPaths.size(); i++) {
        cv::Mat srcL = cv::imread(leftPaths[i]);
        cv::Mat srcR = cv::imread(rightPaths[i]);
        cv::Mat dstL, dstR;

        cv::remap(srcL, dstL, mapL1, mapL2, cv::INTER_LINEAR);
        cv::remap(srcR, dstR, mapR1, mapR2, cv::INTER_LINEAR);

        cv::imwrite(outDir + "/rect_L_" + std::to_string(i) + ".jpg", dstL);
        cv::imwrite(outDir + "/rect_R_" + std::to_string(i) + ".jpg", dstR);
    }
    
    std::cout << "校正图像已保存至: " << outDir << std::endl;
}

int main() {
    stereoProcess("../source/stereoCalibratio/data1/L", "../source/stereoCalibratio/data1/R", "../source/save");
    return 0;
}