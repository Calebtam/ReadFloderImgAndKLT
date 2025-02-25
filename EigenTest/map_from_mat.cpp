#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <iostream>

int main() {
    // 假设我们有一个 4x4 的 OpenCV 矩阵（例如，相机的位姿矩阵）
    cv::Mat T_cv = (cv::Mat_<float>(4, 4) <<
                     1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0);

    // 输出 cv::Mat 的内容，检查矩阵是否正确
    std::cout << "OpenCV Mat (T_cv):\n" << T_cv << std::endl;

    // 使用 Eigen::Map 来将 cv::Mat 映射为 Eigen 矩阵（RowMajor 表示行主序布局）
    Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> T_gtsam(T_cv.ptr<float>(), T_cv.rows, T_cv.cols);

    // 输出 Eigen::Matrix 的内容，检查映射是否正确
    std::cout << "Mapped Eigen Matrix (T_gtsam):\n" << T_gtsam << std::endl;

    // 将 Eigen::Matrix<float, 4, 4> 转换为 Eigen::Matrix<double, 4, 4>，因为 GTSAM 的 Pose3 需要 double 类型
    gtsam::Pose3 cam_pose(T_gtsam.cast<double>());

    // 输出 Pose3 矩阵，检查构造是否成功
    std::cout << "GTSAM Pose3 Matrix:\n" << cam_pose.matrix() << std::endl;

    return 0;
}