#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Core>
#include <string>
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    // 1. Check arguments
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return -1;
    }

    const std::string image_path = argv[1];  // First argument is image path
    double param_ = std::stod(argv[2]); // Second argument is size
    // const std::string output_path = (argc >= 3) ? argv : "undistorted_result.jpg";

    // 2. 读取相机内参和畸变系数（实际项目中建议从配置文件读取）
    double param[12] = {0.033369358628988266, -0.013455653563141823, -0.0049120308831334114, 0.00067755661439150572, 
        0.0, 0.0, 0.0, 0.0, 223.89411926269531, 223.90921020507812, 325.479248046875, 325.74505615234375};
    Eigen::Matrix<double, 3, 3> eK;
    eK << param[8],      0,  param[10],
            0,  param[9],   param[11],
            0,          0,         1;
    Eigen::Matrix<double, 4, 1> edistCoeffs;
    edistCoeffs << param[0], param[1], param[2], param[3];
    
    // cv::Mat cam_K = (cv::Mat_<double>(3, 3) << param[8],      0,  param[10],
    //                                             0,  param[9],   param[11],
    //                                             0,          0,         1);
    // cv::Mat cam_distCoeffs = (cv::Mat_<double>(4, 1) << param[0], param[1], param[2], param[3]);

    cv::Mat cam_K, cam_distCoeffs;
    cv::eigen2cv(eK, cam_K);
    cv::eigen2cv(edistCoeffs, cam_distCoeffs);


    // 3. 读取输入图像
    cv::Mat input_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (input_image.empty()) {
        std::cerr << "Error: Could not load image at " << image_path << std::endl;
        std::cerr << "Supported formats: JPEG, PNG, TIFF, etc." << std::endl;
        return -1;
    }

    // 4. 设置输出尺寸（可修改为自定义尺寸）
    cv::Size new_size = input_image.size(); // 保持原尺寸
    // cv::Size new_size = cv::Size(input_image.size().width * 2, input_image.size().height * 2); // 可选更大尺寸试试
    double balance = 0.0; // 平衡参数：0.0最小化黑边，1.0保留最大FOV
    std::cout << "Balance: " << balance << std::endl;

    // for (double balance : {0.0, 0.5, 1.0}) {
    //     cv::Mat new_k;
    //     cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_K, cam_distCoeffs, input_image.size(), cv::Mat(), new_k, balance);
    //     cv::Mat map1, map2, undistorted;
    //     cv::fisheye::initUndistortRectifyMap(cam_K, cam_distCoeffs, cv::Mat(), new_k, input_image.size(), CV_16SC2, map1, map2);
    //     cv::remap(input_image, undistorted, map1, map2, cv::INTER_LINEAR);
    //     // cv::imwrite("undistorted_balance_" + std::to_string(balance) + ".png", undistorted);
    //     cv::imshow("undistorted_balance_" + std::to_string(balance), undistorted);
    // }
    // cv::waitKey(0);

    // const double g_alpha = 1;
    // int g_centerX = 0;
    // int g_centerY = 0;
    // cv::Mat newCameraMatrix = getOptimalNewCameraMatrix(cam_K, cam_distCoeffs, input_image.size(), g_alpha, new_size, 0); // （针孔模型）

    // 5. 计算新相机矩阵
    cv::Mat new_k;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_K, cam_distCoeffs, input_image.size(), cv::Mat(), new_k, balance, new_size);
    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_K, cam_distCoeffs, input_image.size(), cv::Mat(), new_k, balance, new_size, /*fov_scale=*/ param_);
    
    cv::Mat test_k;
    for (double scale_in : {3.0, 3.5, 3.8, 4.0, 4.2, 4.5, 5.0}) {
        test_k = new_k.clone(); // ⚠️ 一定要 clone，不然 new_k 会被覆盖
        test_k.at<double>(0, 0) *= scale_in; // fx
        test_k.at<double>(1, 1) *= scale_in; // fy
        // 或者你也可以尝试下面这种方式让它回到图像中心：
        test_k.at<double>(0, 2) = new_size.width / 2.0;
        test_k.at<double>(1, 2) = new_size.height / 2.0;

        // 初始化映射表并去畸变
        cv::Mat map1_in, map2_in, undistorted_in;
        cv::fisheye::initUndistortRectifyMap(cam_K, cam_distCoeffs, cv::Mat(), test_k, new_size, CV_16SC2, map1_in, map2_in);

        cv::remap(input_image, undistorted_in, map1_in, map2_in, cv::INTER_LINEAR);

        // 可视化：画图像中心点
        cv::circle(undistorted_in, cv::Point(new_size.width / 2, new_size.height / 2), 2, cv::Scalar(255), -1);
        cv::imshow("Undistorted (scale=" + std::to_string(scale_in) + ")", undistorted_in);
    }
    // cv::waitKey(0);



    // // 手动缩放焦距（0.9 表示再收紧 10%） 我用的较好的参数是4
    // double scale = param_;
    // new_k.at<double>(0, 0) *= scale; // fx
    // new_k.at<double>(1, 1) *= scale; // fy
    // // 或者你也可以尝试下面这种方式让它回到图像中心：
    // new_k.at<double>(0, 2) = new_size.width / 2.0;
    // new_k.at<double>(1, 2) = new_size.height / 2.0;

    // // 修复：同步缩放光心或设定居中
    // new_k.at<double>(0, 2) *= scale; // cx
    // new_k.at<double>(1, 2) *= scale; // cy


    // 6. 生成去畸变映射
    cv::Mat map1, map2;
    cv::Mat UndistortRectifyMap1, UndistortRectifyMap2;
    cv::fisheye::initUndistortRectifyMap(cam_K, cam_distCoeffs, cv::Mat(), new_k, new_size, CV_16SC2, map1, map2);
    // cv::fisheye::initUndistortRectifyMap(cam_K, cam_distCoeffs, cv::Mat(), cam_K, new_size, CV_16SC2, map1, map2);

    cv::fisheye::initUndistortRectifyMap(cam_K, cam_distCoeffs, cv::Mat(), cam_K, input_image.size(), CV_16SC2, UndistortRectifyMap1, UndistortRectifyMap2);


    // 7. 应用去畸变
    cv::Mat undistorted_image, undistorted_image2;
    cv::remap(input_image, undistorted_image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(input_image, undistorted_image2, UndistortRectifyMap1, UndistortRectifyMap2, cv::INTER_LINEAR);

    // // 8. 保存并显示结果
    // if (!cv::imwrite(output_path, undistorted_image)) {
    //     std::cerr << "Error: Failed to save result to " << output_path << std::endl;
    //     return -1;
    // }
    // std::cout << "Undistorted image saved to: " << output_path << std::endl;
    // cv::resize(undistorted_image, undistorted_image, cv::Size(undistorted_image.cols / 2, undistorted_image.rows / 2));
    // cv::resize(undistorted_image2, undistorted_image2, cv::Size(undistorted_image2.cols / 2, undistorted_image2.rows / 2));
    // cv::resize(input_image, input_image, cv::Size(input_image.cols / 2, input_image.rows / 2));

    cv::circle(input_image, cv::Point(input_image.cols / 2, input_image.rows / 2), 2, cv::Scalar(255, 255, 255), -1);
    cv::circle(undistorted_image, cv::Point(undistorted_image.cols / 2, undistorted_image.rows / 2), 2, cv::Scalar(255, 255, 255), -1);
    cv::circle(undistorted_image2, cv::Point(undistorted_image2.cols / 2, undistorted_image2.rows / 2), 2, cv::Scalar(255, 255, 255), -1);

    cv::imshow("Original Image", input_image);
    cv::imshow("Undistorted Image", undistorted_image);
    cv::imshow("keep K and image size", undistorted_image2);
    cv::waitKey(0);

    return 0;
}
