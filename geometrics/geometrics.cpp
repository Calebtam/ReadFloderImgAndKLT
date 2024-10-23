/******************************************************************************
 *
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * This software and associated documentation files (the "Software") are
 * proprietary and confidential to MyCompany and are made available under
 * commercial license only. Unauthorized reproduction, distribution, or
 * disclosure of the Software, or any part thereof, is strictly prohibited.
 *
 ******************************************************************************/

#include "geometrics.h"

#include <math.h>
#include <set>


bool TriangulatePoint(const std::shared_ptr<vo::PinholeCamera4d>& camera, 
    const std::vector<Eigen::Matrix4d> &T_cam_world, 
    const std::vector<Eigen::Vector2d> &pixels, 
    Eigen::Vector3d &point, 
    const double min_movement,
    const double max_dist, 
    const double reprojection_threshold,
    const double min_parallax)
{
    CHECK(camera != nullptr);
    CHECK_GT(T_cam_world.size(), 1);
    if (pixels.size() != T_cam_world.size()) {
        return false; 
    }

    // 每一个观测能够构建两个约束
    int rows = pixels.size() * 2;
    Eigen::MatrixXd A(rows, 4);

    for (uint32_t i = 0; i < pixels.size(); i++) {
        Eigen::Vector3d m = camera->UnprojectFromPixelCornerConv(pixels[i]);
        double mx = m[0], my = m[1], mz = m[2];
        // std::cout << mx << " " << my << " " << mz << std::endl;
        // mz = 1;
        Eigen::Matrix4d T = T_cam_world[i];
        Eigen::Vector4d P1 = T.row(0), 
                        P2 = T.row(1), 
                        P3 = T.row(2);
        A.row(2 * i) = (mx * P3) - (mz * P1);
        A.row((2 * i) + 1) = (my * P3) - (mz * P2);
    }
    std::cout << "A: " << A.rows() << "*" << A.cols()  <<  "\n" << A << std::endl;
    // A: 6*4
    // 1         0 -0.841597  0.810535
    // 0         1 -0.138965  0.310307
    // 1         0  -0.76898  0.471569
    // 0         1  -0.14434  0.321593
    // 1         0 -0.689194  0.118398
    // 0         1 -0.145877  0.324821
    /* Solve the system by finding the right nullspace of A using the SVD
    decomposition*/
    // 线性方程求解
    // Eigen::Vector4d x;
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // // auto x = svd.matrixV().col(A.cols() - 1);
    auto x = svd.matrixV().rightCols<1>();
    std::cout << "x: " << x.transpose() << std::endl;
    // // 将坐标转换为 3+1 维度的齐次坐标
    // point = x.hnormalized();
    
    Eigen::Matrix4d A1_sub = A.block<4, 4>(0, 0);
    std::cout << "A1_sub:\n" << A1_sub.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
    // auto x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().rightCols<1>();
    point = A1_sub.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().hnormalized();

    std::cout << "point: " << point.transpose() << std::endl;

    // 检查这个点是否合理
    double movement = 0.0;
    double parallax = 0.0;
    std::set<double> reproj_errors;
    for (size_t i = 0; i < T_cam_world.size(); i++) {
        // point.homogeneous()：将一个三维点point转换为齐次坐标。在齐次坐标中，三维点(x, y, z)被表示为(x, y, z, 1)。这样做是为了方便进行仿射变换。
        // hnormalized()：将齐次坐标点归一化。归一化操作是将齐次坐标点除以最后一个元素（w），得到一个三维向量。这个操作通常用于将齐次坐标点转换为三维笛卡尔坐标点。
        Eigen::Vector3d point_cam_i = (T_cam_world[i] * point.homogeneous()).hnormalized();

        // check if result is in front of all cameras
        if (point_cam_i[2] < 0) {
            // LOG(INFO) << "Triangulation failed: the point is behind the camera";
            return false;
        }

        if (point_cam_i[2] > max_dist) {
            // LOG(INFO) << "Triangulation failed: the point is too far away";
            return false;
        }

        // check reprojection
        if (reprojection_threshold > 0.0) {
            Eigen::Vector2d reproj_pixel = camera->ProjectToPixelCornerConv(point_cam_i);
            double reprojection_error = (reproj_pixel - pixels[i]).norm();
            reproj_errors.insert(reprojection_error);
        }

        // check movement 最大移动距离
        if (min_movement > 0.0) {
            for (size_t j = i+1; j < T_cam_world.size(); ++j) {
                double m_ij = (T_cam_world[i].inverse() * T_cam_world[j]).block<3,1>(0,3).norm();
                if (m_ij > movement) {
                    movement = m_ij;
                }
            }
        }

        // check parallax 
        if (min_parallax > 0.0) {
            for (size_t j = i+1; j < T_cam_world.size(); ++j) {
                Eigen::Vector3d point_cam_j =  (T_cam_world[j] * point.homogeneous()).hnormalized();

                // 由位置产生的夹角， 旋转矩阵能够确定cami下的坐标点能够转换到camj的姿态，但是位置依旧是cami
                // 意思是求解的是两个向量的夹角, cami原点 到 3dpoint, camj原点 到 3dpoint
                //                              R_cj_w   *  R_w_ci
                Eigen::Matrix3d R_Cj_Ci = T_cam_world[j].block(0, 0, 3, 3) * T_cam_world[i].block(0, 0, 3, 3).transpose();
                Eigen::Vector3d point_cami_in_camj = R_Cj_Ci * point_cam_i;

                double cos_parallax = point_cami_in_camj.normalized().dot(point_cam_j.normalized());
                const double PI = std::atan(1.0)*4;
                double p_ij = std::acos(cos_parallax) * 180.0 / PI;
                if (p_ij > parallax) {
                    parallax = p_ij;
                }
            }
        }

    }

    // Check if the camera moved enough
    if (min_movement > 0.0 && movement < min_movement) {
        LOG(INFO) << "Triangulation failed: movement is too small: " << movement << "(cond. >= " << min_movement << ")";
        return false;
    }

    if (min_parallax > 0.0 && parallax < min_parallax) {
        LOG(INFO) << "Triangulation failed: insufficient parallax with " << parallax << "(cond. >= " << min_parallax << ") degrees";
        return false;
    }

    if (reprojection_threshold > 0.0) {
        const double max_reproj_error = *reproj_errors.rbegin();
        const double second_small_reproj_error = *(++reproj_errors.begin());

        if (max_reproj_error > 2 * reprojection_threshold || second_small_reproj_error > reprojection_threshold) {
            std::stringstream ss;
            for (auto& err : reproj_errors) {
                ss << err << ", ";
            }
            ss << " > " << reprojection_threshold;
            // #tam :三角化重投影误差
            LOG(INFO) << "Triangulation failed: reprojection errors (pixels) " << ss.str();
            return false;
        }
    }

    return true;
}

bool TriangulatePointWithCovariance(        
    const std::shared_ptr<vo::PinholeCamera4d>& camera,
    const std::vector<Eigen::Matrix4d>& T_cam_world,
    const std::vector<Eigen::Vector2d>& pixels,
    Eigen::Vector3d& point,
    Eigen::Matrix3d& covariance,
    const double min_movement,
    const double max_dist, 
    const double reprojection_threshold,
    const double min_parallax) {

    // Currently only calculate covariance for two views triangulation
    CHECK_EQ(T_cam_world.size(), 2);
    CHECK_EQ(pixels.size(), 2);

    // TODO (qin shi): should return a invalid covariance matrix if triangulation failed
    covariance = Eigen::Matrix3d::Identity();
    bool res = TriangulatePoint(camera, T_cam_world, pixels, point, min_movement, max_dist, reprojection_threshold, min_parallax);
    const double baseline = (T_cam_world[1].block(0, 3, 3, 1) - T_cam_world[0].block(0, 3, 3, 1)).norm();

    const double* fxfycxcy = static_cast<const double*>(camera->fxfycxcy());
    const double& fx = fxfycxcy[0];
    const double& fy = fxfycxcy[1];
    const double& cx = fxfycxcy[2];
    const double& cy = fxfycxcy[3];
    
    // For 2D case, the point depth in camera frame is the z-coordinate
    const double z = point.z();
    double u = pixels[0].x();
    double v = pixels[0].y();
    // standard deviation of depth
    const double depth_std = z * z / (baseline*fx) * 0.1;
    const double pixel_std = 0.1; // 0.2 pixel error
    Eigen::Matrix3d J; 
    Eigen::Vector3d cov_pixel(pixel_std, pixel_std, depth_std);
    J << z/fx, 0, u/fx - cx/fx,
         0, z/fy, v/fy - cy/fy,
         0, 0, 1;
    covariance = J * Eigen::Matrix3d(cov_pixel.array().pow(2).matrix().asDiagonal()) * J.transpose();
    return res; 
}

int main() {
    std::cout << "Hello World!\n" << std::endl;
    
    if(1)
    {
        // template<typename Scalar>
        // using Position = Eigen::Matrix<Scalar, 3, 1>;
        Eigen::MatrixXd A1(6, 4), A2(6, 4), A3(6, 4);
        // 第一个矩阵数据
        A1 << 1.00000,  0.00000, -0.84160,  0.79233,
            -0.00000, 1.00000, -0.13897,  0.31031,
            1.00000, -0.00000, -0.76898, 0.47162,
            0.00000, 1.00000, -0.14434, 0.32159,
            1.00000, -0.00000, -0.68919, 0.11845,
            0.00000, 1.00000, -0.14588, 0.32482;

        Eigen::Matrix4d A1_sub = A1.block<4, 4>(0, 0);
        std::cout << "A1:\n" << A1.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
        std::cout << "A1_sub:\n" << A1_sub.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;

        // 第二个矩阵数据
        A2 << 1.00000,  0.00000, -0.84160,  0.81054,
            0.00000,  1.00000, -0.13897,  0.31031,
            1.00000, -0.00000, -0.76898,  0.47157,
            0.00000,  1.00000, -0.14434,  0.32159,
            1.00000, -0.00000, -0.68919,  0.11840,
            0.00000,  1.00000, -0.14588,  0.32482;
        Eigen::Matrix4d A2_sub = A2.block<4, 4>(0, 0);
        std::cout << "A2:\n" << A2.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
        std::cout << "A2_sub:\n" << A2_sub.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;

        // // 第三个矩阵数据
        // A3 << 1.00000,  0.00000, -0.84160,  0.81054,
        //     0.00000,  1.00000, -0.13897,  0.31031,
        //     1.00000, -0.00000, -0.76898,  0.47157,
        //     0.00000,  1.00000, -0.14434,  0.32159,
        //     1.00000, -0.00000, -0.68919,  0.11840,
        //     0.00000,  1.00000, -0.14588,  0.32482;
        // Eigen::Matrix4d A3_sub = A3.block<4, 4>(0, 0);
        // std::cout << "A3:\n" << A3.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
        // std::cout << "A3_sub:\n" << A3_sub.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
        A3 << 1.00000, -0.00000,  0.06657, -0.74021,
            0.00000,  1.00000,  0.60829,  1.27741,
            1.00000, -0.00000,  0.08883, -0.73423,
            0.00000,  1.00000,  0.60718,  1.27509,
            1.00000, -0.00000,  0.18634, -1.49486,
            0.00000,  1.00000,  0.60080, -1.26167;
        Eigen::Matrix4d A3_sub = A3.block<4, 4>(0, 0);
        std::cout << "A3:\n" << A3.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
        std::cout << "A3_sub:\n" << A3_sub.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().hnormalized().transpose() << std::endl;
    }

    if(0)
    {
        // Observations for each landmark from multiple camera poses
        // 无畸变的像素平面
        std::vector<std::vector<Eigen::Vector2d>> observations = {
            {Eigen::Vector2d(535.79298,954.53189), 
            Eigen::Vector2d(579.50468,951.29612), 
            Eigen::Vector2d(627.53174,950.37062)},

            {Eigen::Vector2d(531.41897,895.03735), 
            Eigen::Vector2d(577.25681,892.25756), 
            Eigen::Vector2d(624.12050,888.89212)},

            {Eigen::Vector2d(531.41897,895.03735), 
            Eigen::Vector2d(577.25681,894.25756), 
            Eigen::Vector2d(624.12050,893.89212)}
            // {Eigen::Vector2d(300, 350), Eigen::Vector2d(298, 348), Eigen::Vector2d(302, 351)}
        };
        std::cout << "observations: " << std::endl;
        // Multiple camera extrinsics (poses)
        std::vector<std::vector<Eigen::Matrix4d>> extrinsics = {
            {
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  0.95682,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
                // 1.14329−0.95682 = 0.18647
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  1.14329,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
                // 1.32891−1.14329 = 0.18562
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  1.32891,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
            },
            {
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  2.02719,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
                // 2.19844 - 2.02719 = 0.17125
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  2.19844,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
                // 2.38907 - 2.19844 = 0.19063
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  2.38907,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
            },
            {
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  2.02719,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
                // 2.02719 + 0.185 = 2.21219
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  2.21219,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
                // 2.21219 + 0.185 = 2.39719
                (Eigen::Matrix4d() <<  -1.00000,  0.00000,  0.00000,  2.39719,
                                        0.00000, -1.00000,  0.00000, -0.01848,
                                        0.00000,  0.00000,  1.00000,  2.10000,
                                        0.00000,  0.00000,  0.00000,  1.00000).finished().inverse(),
            }
        };
        std::cout << "Camera poses: " << std::endl;
        // Camera intrinsics (assumed identical for all cameras)
        // k1, k2, k3, k4, p1, p2, sx1, sy1, fx, fy, cx, cy
        // double fisheye_camera_params_[12] = {3.05418428e-02, -3.56408791e-03, -1.80240031e-02, 6.03048922e-03, 
        //         0.0, 0.0, 0.0, 0.0, 5.99024902e+02, 5.99141663e+02, 4.94848450e+02, 5.03196930e+02};
        double params_[12] = {3.16444859e-02, -7.04552373e-03, -1.41365882e-02, 4.73309727e-03, 0.0, 0.0, 0.0, 0.0, 6.01948425e+02, 6.02096313e+02, 5.09349335e+02, 5.07302551e+02};
        vo::CameraPtr camera_ = std::make_shared<vo::ThinPrismFisheyeCamera12d>(992, 992, params_);
        std::cout << "Camera intrinsics: " << std::endl;
        Eigen::Matrix3d intrinsic;
        intrinsic << params_[8],           0,          params_[10],
                    0,           params_[9],          params_[11],
                    0,                    0,                    1;

        std::shared_ptr<vo::PinholeCamera4d> camera_undistorted_;
        vo::DecideForUndistortedCamera(camera_, camera_undistorted_, false);
        std::cout << "Undistorted camera intrinsics: " << std::endl;
        // //fx, fy, cx, cy
        // double undistorted_initial_intrinsics[4] = {6.01948425e+02, 6.02096313e+02, 5.09349335e+02, 5.07302551e+02};
        // camera_undistorted = std::make_shared<vo::PinholeCamera4d>(992, 992, undistorted_initial_intrinsics);

        Eigen::Vector3d point_3d;
        std::vector<Eigen::Vector2d> pixels;
        std::vector<Eigen::Matrix4d> T_cam_world;
        std::cout << "Camera extrinsics: " << std::endl;
        TriangulatePoint(camera_undistorted_, extrinsics.at(0), observations.at(0), point_3d, 0.2, 3.0, 5, 1.5);
        TriangulatePoint(camera_undistorted_, extrinsics.at(1), observations.at(1), point_3d, 0.2, 3.0, 5, 1.5);
        TriangulatePoint(camera_undistorted_, extrinsics.at(1), observations.at(2), point_3d, 0.2, 3.0, 5, 1.5);
        TriangulatePoint(camera_undistorted_, extrinsics.at(2), observations.at(1), point_3d, 0.2, 3.0, 5, 1.5);
        TriangulatePoint(camera_undistorted_, extrinsics.at(2), observations.at(2), point_3d, 0.2, 3.0, 5, 1.5);

    }

    return 0;
}