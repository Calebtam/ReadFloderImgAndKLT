#include <iostream>
#include <ctime>
 
#include <iomanip> // Header file needed to use setprecision

#include <Eigen/Core>
#include <Eigen/Dense>
 
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

#define MATRIX_SIZE 50
 
using namespace std;
  
int main(int argc, char **argv){
    //定义变量
    Matrix<float,2,3> matrix_23;
    
    Vector3d v_3d;//与下面一样是列向量 double
    Matrix<float,3,1> vd_3d;//lie
    
    Matrix3d matrix_33 = Matrix3d::Zero();
    Matrix<double,Dynamic,Dynamic> matrix_dynamic;
    MatrixXd matrix_x;
    
    Vector3d p_imu_rtk(0.0399348, 0.211008, 0.342694);
    // Vector3d p_robot_rtk(0.005, 0.113, 0.230);       // 定位盒子的值
    Vector3d p_robot_rtk(0.029, 0.121, 0.238);          // 机器人本身的值

    Matrix4d T_c_i1, T_c_i2, T_robot_cam;
    Matrix4d T_c_c1;
    // icm
    // T_c_i1 << -9.9976458775839849e-01, -2.1056059618633140e-02, -5.2355914294924411e-03, -5.2769634243991703e-03
    //           ,-2.1014508286477940e-02, 9.9974820631745753e-01, -7.8685708043991706e-03, -3.6288241036312177e-04
    //           ,5.3999542366169866e-03, -7.7566950670282233e-03, -9.9995533609050757e-01, -8.3947850718082695e-03
    //           ,0.0 , 0.0 , 0.0 , 1.0;
    T_c_i1 <<   -9.9536133339461363e-01, -9.6206779701979619e-02, 2.6743648197638677e-04, 5.4680667309668371e-03
                ,-9.6207107534286893e-02, 9.9535437880358146e-01, -3.7219694279559522e-03, 1.0963968020146111e-02
                ,9.1884619425839726e-05, -3.7304337430443171e-03, -9.9999303768641590e-01, -4.7308101533619961e-03
                ,0, 0, 0, 1;
    // T_c_i1 <<   -9.9903654037223144e-01, -4.3876769977994383e-02, -9.0557019636171364e-04, 1.1722312812024486e-03
    //             ,-4.3743951805060473e-02, 9.9724783259726113e-01, -5.9860062316553712e-02, 3.5761864668978557e-04
    //             ,3.5295441007182323e-03, -5.9762776344170272e-02, -9.9820636788295292e-01, -5.4287220305360465e-04
    //             ,0, 0, 0, 1;

    T_c_c1 << 9.9929362019459733e-01, 3.6563663679893665e-02, -8.6809640409060036e-03, -9.9857449085087535e-02
              ,-3.6562751166395319e-02, 9.9933132424168847e-01, 2.6384938396979533e-04, 1.6100191294004516e-03
              ,8.6848065908306966e-03, 5.3736922018791152e-05, 9.9996228491219763e-01, 4.3249286893142625e-04
              ,0.0 , 0.0 , 0.0 , 1.0;
    // ch0x0
    T_c_i2 << -5.3600845749410819e-03, -9.9939196268917763e-01, 3.4452494911438980e-02, 5.4409151811733655e-02
              ,6.3354543429109123e-02, -3.4723163635731469e-02, -9.9738683755803370e-01, -6.0586000592539090e-03
              ,9.9797668876594436e-01, -3.1633557181361405e-03, 6.3502140596691126e-02, -1.0242074294945365e-02
              ,0.0 , 0.0 , 0.0 , 1.0;

    T_robot_cam <<  0.016954, -0.466968, 0.884112, 0.438583
                  ,-0.999855, -0.006797, 0.015583, 0.073411
                  ,-0.001267, -0.884248, -0.467016, 0.2662
                  ,0        , 0        , 0        , 1;

    Matrix4d T_i1_robot = (T_robot_cam * T_c_i2).inverse();
    Vector3d p_i1_robot = T_i1_robot.block<3,1>(0,3);
    
    Vector3d pi1_robot_rtk = T_i1_robot.block<3,3>(0,0) * p_robot_rtk;  // 向量转换坐标系
    // Matrix4d T_c1_i = T_c_c1.inverse() * T_c_i;

    std::cout << std::fixed << std::setprecision(10) << "T_c1_i:\n" << T_i1_robot.format(MyFmt) << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "p_i1_robot:\n" << p_i1_robot.transpose().format(MyFmt) << std::endl;
    std::cout << std::fixed << std::setprecision(10) << "p_i1_rtk:\n" << (p_i1_robot+pi1_robot_rtk).transpose().format(MyFmt) << std::endl;
    // //初始化变量
    // matrix_23 << 1,2,3,4,5,6;
    // v_3d << 3, 2, 1;
    // vd_3d << 4, 5, 6;
    
    // cout<<"matrix_23:\n"<<matrix_23<<endl;
    // cout<<"v_3d: "<<v_3d.transpose()<<endl;
    // cout<<"vd_3d: "<<vd_3d.transpose()<<endl;
    // // for(int i=0;i<2;i++)
    // //     for(int j=0;j<3;j++)
    // //         cout<<matrix_23(i,j)<<"\t";
    // cout<<endl;
    
    // //一个float，一个double，不能直接相乘，应该显式转换
    // Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    // cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << endl;

    // //一个float，一个float
    // Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    // cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;
    
    // matrix_33 = Matrix3d::Random();      // 随机数矩阵
    // cout << "random matrix: \n" << matrix_33 << endl;
    // // cout << "transpose: \n" << matrix_33.transpose() << endl;      // 转置
    // cout << "sum: " << matrix_33.sum() << endl;            // 各元素和
    // cout << "trace: " << matrix_33.trace() << endl;          // 迹
    // // cout << "times 10: \n" << 10 * matrix_33 << endl;               // 数乘
    // // cout << "inverse: \n" << matrix_33.inverse() << endl;        // 逆
    // cout << "det: " << matrix_33.determinant() << endl;    // 行列式
    
    return 0;
}
