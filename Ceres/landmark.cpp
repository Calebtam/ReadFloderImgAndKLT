#include <ceres/ceres.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>

// // Define the reprojection error cost function for Ceres.
// struct ReprojectionError {
//     ReprojectionError(const Eigen::Vector2d& observed_point, const Eigen::Matrix3d& intrinsic, const Eigen::Matrix4d& extrinsic)
//         : observed_point_(observed_point), intrinsic_(intrinsic), extrinsic_(extrinsic) {}

//     template <typename T>
//     bool operator()(const T* const landmark, T* residuals) const {
//         // Convert landmark position to homogeneous coordinates
//         Eigen::Matrix<T, 4, 1> landmark_homo(T(landmark[0]), T(landmark[1]), T(landmark[2]), T(1.0));

//         // Transform the landmark to camera coordinates using extrinsic
//         Eigen::Matrix<T, 4, 1> camera_point = extrinsic_.template cast<T>() * landmark_homo;

//         // Project the 3D point into the image plane using camera intrinsics
//         T u = intrinsic_(0, 0) * camera_point[0] / camera_point[2] + intrinsic_(0, 2);
//         T v = intrinsic_(1, 1) * camera_point[1] / camera_point[2] + intrinsic_(1, 2);

//         // Calculate the residuals (difference between observed and projected points)
//         residuals[0] = u - T(observed_point_(0));
//         residuals[1] = v - T(observed_point_(1));

//         return true;
//     }

//     static ceres::CostFunction* Create(const Eigen::Vector2d& observed_point, const Eigen::Matrix3d& intrinsic, const Eigen::Matrix4d& extrinsic) {
//         return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
//             new ReprojectionError(observed_point, intrinsic, extrinsic)));
//     }

//     Eigen::Vector2d observed_point_;
//     Eigen::Matrix3d intrinsic_;
//     Eigen::Matrix4d extrinsic_;
// };
// 定义重投影误差的代价函数

struct ReprojectionError {
    ReprojectionError(const Eigen::Vector2d& observed_point, const Eigen::Matrix3d& intrinsic, const Eigen::Matrix4d& extrinsic)
        : observed_point_(observed_point), intrinsic_(intrinsic), extrinsic_(extrinsic) {}

    template <typename T>
    bool operator()(const T* const landmark, T* residuals) const {
        // 将地标位置转换为齐次坐标
        // Eigen::Matrix<T, 3, 1> landmark_T(landmark.cast<T>());
        Eigen::Matrix<T, 4, 1> landmark_homo(landmark[0], landmark[1], landmark[2], T(1.0));

        // 使用外参矩阵将地标转换到相机坐标系
        Eigen::Matrix<T, 4, 1> camera_point = extrinsic_.template cast<T>() * landmark_homo;

        // 使用相机内参将三维点投影到图像平面
        T u = intrinsic_(0, 0) * camera_point[0] / camera_point[2] + intrinsic_(0, 2);
        T v = intrinsic_(1, 1) * camera_point[1] / camera_point[2] + intrinsic_(1, 2);

        // 计算残差（观测点与投影点之间的差异）
        residuals[0] = u - T(observed_point_(0));
        residuals[1] = v - T(observed_point_(1));

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& observed_point, const Eigen::Matrix3d& intrinsic, const Eigen::Matrix4d& extrinsic) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
            new ReprojectionError(observed_point, intrinsic, extrinsic)));
    }

    Eigen::Vector2d observed_point_;
    Eigen::Matrix3d intrinsic_;
    Eigen::Matrix4d extrinsic_;
};
int main() {
    // Observations for each landmark from multiple camera poses
    std::vector<std::vector<Eigen::Vector2d>> observations = {
        {Eigen::Vector2d(100, 150), Eigen::Vector2d(98, 148), Eigen::Vector2d(102, 151)},
        {Eigen::Vector2d(200, 250), Eigen::Vector2d(198, 248), Eigen::Vector2d(202, 251)},
        {Eigen::Vector2d(300, 350), Eigen::Vector2d(298, 348), Eigen::Vector2d(302, 351)}
    };

    // Camera intrinsics (assumed identical for all cameras)
    Eigen::Matrix3d intrinsic;
    intrinsic << 500, 0, 320,
                 0, 500, 240,
                 0, 0, 1;

    // Multiple camera extrinsics (poses)
    std::vector<Eigen::Matrix4d> extrinsics = {
        Eigen::Matrix4d::Identity(),
        (Eigen::Matrix4d() << 1, 0, 0, 0.1,
                              0, 1, 0, 0.0,
                              0, 0, 1, 0.0,
                              0, 0, 0, 1).finished(),
        (Eigen::Matrix4d() << 1, 0, 0, 0.05,
                              0, 1, 0, 0.05,
                              0, 0, 1, 0.0,
                              0, 0, 0, 1).finished(),
    };

    // Initial guesses for landmark positions (x, y, z)
    std::vector<std::array<double, 3>> landmarks = {
        {0.0, 0.0, 5.0},
        {1.0, 1.0, 5.0},
        {2.0, 2.0, 5.0}
    };

    // Set up the problem
    ceres::Problem problem;
    for (size_t i = 0; i < landmarks.size(); ++i) {
        for (size_t j = 0; j < extrinsics.size(); ++j) {
            ceres::CostFunction* cost_function = 
                ReprojectionError::Create(observations[i][j], intrinsic, extrinsics[j]);
            problem.AddResidualBlock(cost_function, nullptr, landmarks[i].data());
        }
    }

    // Set up solver options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Output the results
    std::cout << summary.FullReport() << "\n";
    for (size_t i = 0; i < landmarks.size(); ++i) {
        std::cout << "Landmark " << i << " position: (" 
                  << landmarks[i][0] << ", " << landmarks[i][1] << ", " << landmarks[i][2] << ")\n";
    }

    return 0;
}