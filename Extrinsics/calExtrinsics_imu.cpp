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
    //定义变量
    Matrix<float,2,3> matrix_23;
    
    Vector3d v_3d;//与下面一样是列向量 double
    Matrix<float,3,1> vd_3d;//lie
    
    Matrix3d matrix_33 = Matrix3d::Zero();

    double ARC2DEC = 57.29578;
    
    // 三轴90度
    Vector3d v_3d_1(90/57.29578, 90/57.29578, 90/57.29578);
    // Vector3d v_3d_1(1, 1, 1);
    Matrix3d ch030_acc,icm40608_acc, ch030_gyr, icm40608_gyr;
    ch030_acc << 0.99997057974219561,     0.00080871659439779315, -0.000043439969991280537, 
                0.0023539588579533956,   0.99715894021963458,    0.0031377515760083641,
                0.00077579781873370361, -0.0033565136577209374,  0.99942758163831091;
    // 超核  陀螺轴偏 尺度偏移
    ch030_gyr <<  1.0002101399944603,                          0.,                     0., 
                -0.00005578440739421717,       1.0021655266290543,                     0.,
                -0.00057229960194921895,       0.0009093160800406,    0.99806393571890961;

    icm40608_acc << 0.99405432160386986   ,  -0.017398903146890243 , -0.011654335270268240  , 
                    -0.0052559878292822351 ,   1.0088449057919222   , -0.0021251401954012795,
                    -0.0054742119267912094 ,  0.00056708000800976687,  1.0513641049364815;
    // 40608  陀螺轴偏 尺度偏移
    icm40608_gyr << 1.0598204797595738   ,                      0.,                       0., 
                    -0.060907440894186031 ,  0.97156130224344672   ,                       0., 
                    -0.0069146349091346124, -0.033885065278017192  ,  1.0290295071221283   ;
    // std::cout << ch030_acc.block<1,3>(0,0).transpose().cwiseProduct(v_3d_1).transpose() * ARC2DEC << std::endl;
    // std::cout << ch030_acc.block<1,3>(1,0).transpose().cwiseProduct(v_3d_1).transpose() * ARC2DEC << std::endl;
    // std::cout << ch030_acc.block<1,3>(2,0).transpose().cwiseProduct(v_3d_1).transpose() * ARC2DEC << std::endl;
    std::cout << ch030_gyr.inverse() * v_3d_1 * ARC2DEC << std::endl;
    // 原始值       90          90              90
    // 超核轴偏值    89.9811     89.8105         90.1444
    // std::cout << icm40608_acc.block<1,3>(0,0).transpose().cwiseProduct(v_3d_1).transpose() * ARC2DEC << std::endl;
    // std::cout << icm40608_acc.block<1,3>(1,0).transpose().cwiseProduct(v_3d_1).transpose() * ARC2DEC << std::endl;
    // std::cout << icm40608_acc.block<1,3>(2,0).transpose().cwiseProduct(v_3d_1).transpose() * ARC2DEC << std::endl;
    std::cout << icm40608_gyr.inverse() * v_3d_1 * ARC2DEC << std::endl;
    // 原始值        90          90              90
    // 40608轴偏值   84.92       97.9581         91.2574

    Matrix<double,Dynamic,Dynamic> matrix_dynamic;
    MatrixXd matrix_x;

    Matrix4d T_c_i0, T_c_i1, T_c_i2, T_c_i3;
    Matrix4d T_c_c1, T_c1_c, T_robot_cam;

    // 固定
    T_robot_cam <<  0.016954, -0.466968, 0.884112, 0.438583
                  ,-0.999855, -0.006797, 0.015583, 0.073411
                  ,-0.001267, -0.884248, -0.467016, 0.2662
                  ,0        , 0        , 0        , 1;
    // Vector3d p_imu_rtk(0.0399348, 0.211008, 0.342694);
    // Vector3d p_robot_rtk(0.005, 0.113, 0.230);       // 定位盒子的值
    Vector3d p_robot_rtk(0.029, 0.121, 0.238);          // 固定
    
    // ICM40608
    T_c_i0 <<   -9.9536133339461363e-01, -9.6206779701979619e-02, 2.6743648197638677e-04, 5.4680667309668371e-03
                ,-9.6207107534286893e-02, 9.9535437880358146e-01, -3.7219694279559522e-03, 1.0963968020146111e-02
                ,9.1884619425839726e-05, -3.7304337430443171e-03, -9.9999303768641590e-01, -4.7308101533619961e-03
                ,0, 0, 0, 1;
    T_c_c1 <<    0.999385, 0.00383497, 0.0348471, 0.101697
                ,-0.00317308, 0.999814, -0.0190296, -0.000419059
                ,-0.0349135, 0.0189074, 0.999211, -0.00407236
                ,0.0, 0.0, 0.0, 1.0;
    std::cout << " --------------------------------------------------------------------------------- " << std::endl;
    Matrix4d T_c1_i0 = T_c_c1.inverse() * T_c_i0;
    std::cout << " ======== icm40608 原先 ========" << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_c0_i:\n" << T_c1_i0.format(MyFmt) << std::endl;

    Matrix4d T_i0_robot = (T_robot_cam * T_c_i0).inverse();
    Vector3d p_i0_robot = T_i0_robot.block<3,1>(0,3);
    Vector3d pi0_robot_rtk = T_i0_robot.block<3,3>(0,0) * p_robot_rtk;  // 向量转换坐标系  
    Vector3d p_i0_rtk =  p_i0_robot+pi0_robot_rtk;              // i_r + r_rtk  = i_rtk
    std::cout << std::fixed << std::setprecision(10) << "p_i0_rtk:\n" << p_i0_rtk.transpose().format(ImuFmt) << std::endl;

    std::cout << " ----------------------------------- CG05 ---------------------------------------- " << std::endl;
    // CG05
    T_c1_c <<    9.9929362019459733e-01, 3.6563663679893665e-02, -8.6809640409060036e-03, -9.9857449085087535e-02
                ,-3.6562751166395319e-02, 9.9933132424168847e-01, 2.6384938396979533e-04, 1.6100191294004516e-03
                ,8.6848065908306966e-03, 5.3736922018791152e-05, 9.9996228491219763e-01, 4.3249286893142625e-04
                ,0.0, 0.0, 0.0, 1.0;
    T_c_c1 <<    0.999293620194598, -0.036562751166395,  0.008684806590831,  0.099842022411518
                ,0.036563663679894,  0.999331324241689,  0.000053736922019,  0.002042188394805
                ,-0.008680964040906,  0.000263849383970,  0.999962284912198, -0.001299760284705
                ,-0.000000000000000, -0.000000000000000, -0.000000000000000,  1.000000000000000;
    std::cout << " --------------------------------------------------------------------------------- " << std::endl;
    // CG05 icm
    T_c_i1 << -9.9976458775839849e-01, -2.1056059618633140e-02, -5.2355914294924411e-03, -5.2769634243991703e-03
              ,-2.1014508286477940e-02, 9.9974820631745753e-01, -7.8685708043991706e-03, -3.6288241036312177e-04
              ,5.3999542366169866e-03, -7.7566950670282233e-03, -9.9995533609050757e-01, -8.3947850718082695e-03
              ,0.0 , 0.0 , 0.0 , 1.0;

    Matrix4d T_c1_i1 = T_c1_c * T_c_i1;
    std::cout << " ======== CG05 icm40608 ========" << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_c1_i1:\n" << T_c1_i1.format(MyFmt) << std::endl;

    Matrix4d T_i1_robot = (T_robot_cam * T_c_i1).inverse();
    Vector3d p_i1_robot = T_i1_robot.block<3,1>(0,3);
    Vector3d pi1_robot_rtk = T_i1_robot.block<3,3>(0,0) * p_robot_rtk;  // 向量转换坐标系  
    Vector3d p_i1_rtk =  p_i1_robot+pi1_robot_rtk;              // i_r + r_rtk  = i_rtk
    std::cout << std::fixed << std::setprecision(10) << "p_i0_rtk:\n" << p_i1_rtk.transpose().format(ImuFmt) << std::endl;

    Eigen::Vector3d R_c_i1_eulerAngle=T_c_i1.block<3,3>(0,0).eulerAngles(2,1,0);
    Eigen::Quaterniond R_c_i1_quaternion(T_c_i1.block<3,3>(0,0));
    //输出欧拉角 Z Y X  
    std::cout << std::endl << std::fixed << std::setprecision(10) << "R_c_i1_y-p-r: " << R_c_i1_eulerAngle.transpose() << std::endl;
    //输出四元数
    std::cout << std::fixed << std::setprecision(10) << "R_c_i1_quaternion: " << R_c_i1_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

    std::cout << " --------------------------------------------------------------------------------- " << std::endl;
    // CG05 ch0x0 early
    T_c_i2 << -5.3600845749410819e-03, -9.9939196268917763e-01, 3.4452494911438980e-02, 5.4409151811733655e-02
              ,6.3354543429109123e-02, -3.4723163635731469e-02, -9.9738683755803370e-01, -6.0586000592539090e-03
              ,9.9797668876594436e-01, -3.1633557181361405e-03, 6.3502140596691126e-02, -1.0242074294945365e-02
              ,0.0 , 0.0 , 0.0 , 1.0;

    Matrix4d T_c1_i2 = T_c1_c * T_c_i2;
    std::cout << " ======== CG05 ch0x0 early ========" << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_c1_i2:\n" << T_c1_i2.format(MyFmt) << std::endl;

    Matrix4d T_i2_robot = (T_robot_cam * T_c_i2).inverse();
    Vector3d p_i2_robot = T_i2_robot.block<3,1>(0,3);
    Vector3d pi2_robot_rtk = T_i2_robot.block<3,3>(0,0) * p_robot_rtk;  // 向量转换坐标系  
    Vector3d p_i2_rtk =  p_i2_robot+pi2_robot_rtk;              // i_r + r_rtk  = i_rtk
    std::cout << std::fixed << std::setprecision(10) << "p_i2_rtk:\n" << p_i2_rtk.transpose().format(ImuFmt) << std::endl;

    Eigen::Vector3d R_c_i2_eulerAngle=T_c_i2.block<3,3>(0,0).eulerAngles(2,1,0);
    Eigen::Quaterniond R_c_i2_quaternion(T_c_i2.block<3,3>(0,0));
    std::cout << std::endl << std::fixed << std::setprecision(10) << "R_c_i2_y-p-r: " << R_c_i2_eulerAngle.transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "R_c_i2_quaternion: " << R_c_i2_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

    std::cout << " --------------------------------------------------------------------------------- " << std::endl;
    // ch0x0 now
    T_c_i3 <<    -9.4993443002331368e-03, -9.9994395665702529e-01, 4.6739707914257734e-03, 4.5845156257491068e-02
                ,-9.9960941692875183e-01, 9.3731034638246946e-03, -2.6327903805993196e-02, 1.3306726298177101e-02
                ,2.6282618690435459e-02, -4.9222430405156791e-03, -9.9964243381232265e-01, -2.5554352081653249e-02
                ,0, 0, 0, 1;

    Matrix4d T_c1_i3 = T_c1_c * T_c_i3;
    std::cout << " ======== CG05 ch0x0 now ========" << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "T_c1_i3:\n" << T_c1_i3.format(MyFmt) << std::endl;

    Matrix4d T_i3_robot = (T_robot_cam * T_c_i3).inverse();
    Vector3d p_i3_robot = T_i3_robot.block<3,1>(0,3);
    Vector3d pi3_robot_rtk = T_i3_robot.block<3,3>(0,0) * p_robot_rtk;  // 向量转换坐标系  
    Vector3d p_i3_rtk =  p_i3_robot+pi3_robot_rtk;              // i_r + r_rtk  = i_rtk
    std::cout << std::fixed << std::setprecision(10) << "p_i3_rtk:\n" << p_i3_rtk.transpose().format(ImuFmt) << std::endl;

    Eigen::Vector3d R_c_i3_eulerAngle=T_c_i3.block<3,3>(0,0).eulerAngles(2,1,0);
    Eigen::Quaterniond R_c_i3_quaternion(T_c_i3.block<3,3>(0,0));
    std::cout << std::endl << std::fixed << std::setprecision(10) << "R_c_i3_y-p-r: " << R_c_i3_eulerAngle.transpose() << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "R_c_i3_quaternion: " << R_c_i3_quaternion.coeffs().transpose() << std::endl;   // coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

    return 0;
}
