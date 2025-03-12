#include <iostream>
#include <ctime>
 
#include <iomanip> // Header file needed to use setprecision

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

using namespace Eigen;


// 格式化打印输出
//precision 精度：主要包括StreamPrecision与FullPrecision，默认为0
//flags 设置标号，默认为0

//coeffSeparator 同一行两个数的分隔符
//rowSeparator  两行之间的分隔符

//rowPrefix 每一行开头符号
//rowSuffix 每一行结束符号

//matPrefix 矩阵开始时符号
//matSuffix 矩阵结束时符号


IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

IOFormat MyFmt(FullPrecision, 0, ", " , "\n", " - [" , "]");
IOFormat ImuFmt(FullPrecision, 0, ", " , "\n", "[" , "]");

#define MATRIX_SIZE 50
 
using namespace std;
  
int main(int argc, char **argv){


    // cout << "Quaternion from vector4d(1, 2, 3, 4) is:\n"
    // << Quaterniond(Vector4d(1, 2, 3, 4)).coeffs().transpose() << endl;
    // cout << "Quaternion from (1, 2, 3, 4) is:\n"
    // << Quaterniond(1, 2, 3, 4).coeffs().transpose() << endl;

    // Quaternion from vector4d(1, 2, 3, 4) is:
    // 1 2 3 4
    // Quaternion from (1, 2, 3, 4) is:
    // 2 3 4 1 
        
    // 四元数初始化有多种形式，容易出错的是：由于Eigen中四元数存储顺序（从打印可知）是xyzw，导致赋值出现的错误：
    // 方式一：4个标量
    // Quaterniond q1(1, 2, 3, 4);                               // 第一种方式  实部为1 ，虚部234
    // 方式二：Vector4d
    // Quaterniond q2(Vector4d(1, 2, 3, 4));             // 第二种方式  实部为4 ，虚部123

    // Eigen::Quaterniond R_c_i2_quaternion(T_c_i2.block<3,3>(0,0));
    // std::cout << std::fixed << std::setprecision(10) << "R_c_i2_quaternion: " << R_c_i2_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    
    // ======================
    // "T_lidar_imu": [     -0.00423168,     -0.00840343,     -0.0500412,
    //     0.000493062,     -0.00368547,     0.00545724,      0.999978    //   ],
    Eigen::Vector3d translation(-0.00423168, -0.00840343, -0.0500412);
    Eigen::Quaterniond quaternion(0.999978, 0.000493062, -0.00368547, 0.00545724); // (w, x, y, z)
    quaternion.normalize();
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
    Eigen::Matrix4d T_lidar_imu = Eigen::Matrix4d::Identity();
    T_lidar_imu.block<3,3>(0,0) = rotationMatrix;
    T_lidar_imu.block<3,1>(0,3) = translation;
    std::cout << std::fixed << std::setprecision(10) << "T_lidar_imu:\n" << T_lidar_imu.format(MyFmt) << std::endl;

    Eigen::Vector3d T_lidar_imu_eulerAngle=T_lidar_imu.block<3,3>(0,0).eulerAngles(2,1,0);
    Eigen::Quaterniond T_lidar_imu_quaternion(T_lidar_imu.block<3,3>(0,0));
    std::cout << std::endl << std::fixed << std::setprecision(10) << "T_lidar_imu_eulerAngle_y-p-r: " << T_lidar_imu_eulerAngle.transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_lidar_imu_quaternion: " << T_lidar_imu_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    
    // ======================
    // "T_wheel_imu": [     0.96,     0.01,     1.9499,
    //     0.00522274,     -0.00126581,     0.00882262,     0.999947   ],
    // 平移向量 (x, y, z)
    translation = Eigen::Vector3d(0.96, 0.01, 1.9499);
    quaternion = Eigen::Quaterniond(0.999947, 0.00522274, -0.00126581, 0.00882262); // (w, x, y, z)
    quaternion.normalize();
    rotationMatrix = quaternion.toRotationMatrix();
    Eigen::Matrix4d T_wheel_imu = Eigen::Matrix4d::Identity();
    T_wheel_imu.block<3,3>(0,0) = rotationMatrix;
    T_wheel_imu.block<3,1>(0,3) = translation;
    std::cout << std::fixed << std::setprecision(10) << "T_wheel_imu:\n" << T_wheel_imu.format(MyFmt) << std::endl;

    // ======================
    // "T_wheel_lidar"
    //     [LOG_DEBUG]  | Rotation:
    //   0.999986 -0.00510222 -0.00150444
    //   0.00508015    0.999884  -0.0143315
    //   0.00157739   0.0143237    0.999896 
    //  [LOG_DEBUG]  | Quaternion:0.00716402i + -0.00077048j + 0.00254567k + 0.999971 
    //  [LOG_DEBUG]  | roll: 0.0143242 pitch: -0.00157739 yaw: 0.00508017 
    Eigen::Matrix4d T_wheel_lidar = T_wheel_imu * T_lidar_imu.inverse();
    std::cout << std::fixed << std::setprecision(10) << "T_wheel_lidar:\n" << T_wheel_lidar.format(MyFmt) << std::endl;

    // ======================
    // T_wheel_camera   top_rgb_link
    // <origin xyz="1.026 -0.006994 2.1" rpy="0.0068064 -0.0064479 -3.1317553"/>
    // [LOG_DEBUG]  | lx:0.966032 m [LOG_DEBUG]  | ly:-0.00699419 m [LOG_DEBUG]  | ltheta:-3.1318 rad 
    // 平移向量 (x, y, z)
    // translation = Eigen::Vector3d(1.026, -0.006994, 2.1);
    translation = Eigen::Vector3d(0.966032, -0.00699419, 2.1);

    Eigen::AngleAxisd rollAngle(0.0068064, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-0.0064479, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-3.1317553, Eigen::Vector3d::UnitZ());
    // rotationMatrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    rotationMatrix = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
    Eigen::Matrix4d T_wheel_camera = Eigen::Matrix4d::Identity();
    T_wheel_camera.block<3,3>(0,0) = rotationMatrix;
    T_wheel_camera.block<3,1>(0,3) = translation;
    std::cout << std::fixed << std::setprecision(10) << "T_wheel_camera:\n" << T_wheel_camera.format(MyFmt) << std::endl;

    Eigen::Vector3d T_wheel_camera_eulerAngle=T_wheel_camera.block<3,3>(0,0).eulerAngles(2,1,0);
    Eigen::Quaterniond T_wheel_camera_quaternion(T_wheel_camera.block<3,3>(0,0));
    std::cout << std::endl << std::fixed << std::setprecision(10) << "T_wheel_camera_y-p-r: " << T_wheel_camera_eulerAngle.transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_wheel_camera_quaternion: " << T_wheel_camera_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    
    // ======================
    Eigen::Matrix4d T_camera_imu = T_wheel_camera.inverse() * T_wheel_imu;
    std::cout << std::fixed << std::setprecision(10) << "T_camera_imu:\n" << T_camera_imu.format(MyFmt) << std::endl;

    Eigen::Vector3d T_camera_imu_eulerAngle=T_camera_imu.block<3,3>(0,0).eulerAngles(2,1,0);
    Eigen::Quaterniond T_camera_imu_quaternion(T_camera_imu.block<3,3>(0,0));
    std::cout << std::endl << std::fixed << std::setprecision(10) << "T_camera_imu_y-p-r: " << T_camera_imu_eulerAngle.transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_camera_imu_quaternion: " << T_camera_imu_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    
    

    return 0;
}
