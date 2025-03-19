
#include "geometrics.h"

#include <math.h>
#include <set>

int main() {
    std::cout << "Hello World!\n" << std::endl;

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
    Eigen::Matrix3d camera_undistorted_intrinsic;
    camera_undistorted_intrinsic << 601.948425,           0,          1042.391255,
                                    0,           602.096313,          1038.202480,
                                    0,                    0,                    1;
    std::cout << "Undistorted camera intrinsics: " << std::endl;
    // //fx, fy, cx, cy
    // double undistorted_initial_intrinsics[4] = {6.01948425e+02, 6.02096313e+02, 5.09349335e+02, 5.07302551e+02};
    // camera_undistorted = std::make_shared<vo::PinholeCamera4d>(992, 992, undistorted_initial_intrinsics);    

    if(1){
        // Observations for each landmark from multiple camera poses
        // 无畸变的像素平面
        std::vector<std::vector<Eigen::Vector2d>> observations = {
            {Eigen::Vector2d(1197.5, 647.922), 
            Eigen::Vector2d(1121.62, 649.619), 
            Eigen::Vector2d(1067.7, 649.842)}
        };

        std::cout << "Observations: " << std::endl;

        // Multiple camera extrinsics (poses)  T_cam_world
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
                (Eigen::Matrix4d() <<  
                    -0.996829, -0.00424964,  0.0794646,  1.84093,
                    0.00625126, -0.999669,  0.024957,   1.18041,
                    0.0793322,  0.0253746,  0.996525,  -0.177428,
                    0.00000,    0.00000,    0.00000,    1.00000).finished(),

                (Eigen::Matrix4d() <<  
                    -0.995223, -0.00453176,  0.0975194,  3.08797,
                    0.00740047, -0.99955,   0.0290753,  1.82815,
                    0.0973438,  0.0296581,  0.994809,  -0.260198,
                    0.00000,    0.00000,    0.00000,    1.00000).finished(),

                (Eigen::Matrix4d() <<  
                    -0.993988, -0.00535201,  0.109362,   4.30014,
                    0.008843,  -0.999466,   0.0314614,  2.39963,
                    0.109135,  0.0322393,   0.993504,  -0.359186,
                    0.00000,   0.00000,     0.00000,    1.00000).finished()
            }
        };

        Eigen::Vector3d point_3d;
        std::vector<Eigen::Vector2d> pixels;
        std::vector<Eigen::Matrix4d> T_cam_world;
        std::cout << "Camera extrinsics: " << std::endl;
        TriangulatePoint(camera_undistorted_, extrinsics.at(0), observations.at(0), point_3d, 0.2, 3.0, 5, 1.5);
        TriangulatePoint(camera_undistorted_, extrinsics.at(1), observations.at(0), point_3d, 0.2, 3.0, 5, 1.5);
    }

    if(0)
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
        // Multiple camera extrinsics (poses)  T_cam_world
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