//
// Created by zyn on 2023/3/21.
//
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Matrix3d parametersToRotationMatrix(const Eigen::Vector3d & parameters) //, Eigen::Matrix3d * S)
{
  Eigen::Matrix3d C;

  double angle = parameters.norm();

  if(angle < 1e-14)
  {
    // The angle goes to zero.
    C = Eigen::Matrix3d::Identity();
//    if(S)
//    {
//      *S = Eigen::Matrix3d::Identity();
//    }
  }
  else
  {
    Eigen::Vector3d axis;
    double recip_angle = 1.0/angle;
    axis = parameters * recip_angle;

    double ax = axis[0];
    double ay = axis[1];
    double az = axis[2];
    double sa = sin(angle);
    double ca = cos(angle);
    double ax2 = ax*ax;
    double ay2 = ay*ay;
    double az2 = az*az;

    C <<
      ax2+ca*(1.0-ax2),     ax*ay-ca*ax*ay+sa*az, ax*az-ca*ax*az-sa*ay,
        ax*ay-ca*ax*ay-sa*az, ay2+ca*(1.0-ay2),     ay*az-ca*ay*az+sa*ax,
        ax*az-ca*ax*az+sa*ay, ay*az-ca*ay*az-sa*ax, az2+ca*(1.0-az2);


//    if(S)
//    {
//      *S = parametersToSMatrix(parameters);
//    }
  }

  return C;
}

Eigen::Vector3d rotationMatrixToParameters(const Eigen::Matrix3d & C)
{
  Eigen::Vector3d p;
  // Sometimes, because of roundoff error, the value of tr ends up outside
  // the valid range of arccos. Truncate to the valid range.
  double tr = std::max(-1.0, std::min( (C(0,0) + C(1,1) + C(2,2) - 1.0) * 0.5, 1.0));
  double a = acos( tr ) ;

  if(fabs(a) < 1e-14){
    return Eigen::Vector3d::Zero();
  }

  p[0] = (C(2,1) - C(1,2));
  p[1] = (C(0,2) - C(2,0));
  p[2] = (C(1,0) - C(0,1));
  double n2 = p.norm();
  if(fabs(n2) < 1e-14)
    return Eigen::Vector3d::Zero();

  double scale = -a/n2;
//            double scale = 0.5/sin(a);
//  std::cout << "p: " << p.transpose() << std::endl;
  p = scale * p;
  Eigen::Matrix3d R = parametersToRotationMatrix(p);
//  std::cout << "scale================: " << scale << ", " << a*0.5/sin(a) << ", " << a << std::endl;

//            std::cout << "##############" << std::endl;
  if(!R.isApprox(C, 0.001)){
    std::cout << "scale================: " << scale << ", " << a*0.5/sin(a) << ", " << a << std::endl;
    scale = a/n2;
    p = scale * p;
  }

  return p;
}
using namespace std;
int main(){
  Eigen::Vector3d vectorOri(2.89268, -0.115323,  0.129738);
  cout << "origin vector: " << vectorOri.transpose() << endl;
  cout << "origin rotation: " << endl << parametersToRotationMatrix(vectorOri) << endl;
  Eigen::Matrix3d rotation;
  rotation << 0.9930,   -0.0671,    0.0977,
             -0.0887,   -0.9674,    0.2373,
             0.0785,   -0.2443,   -0.9665;
  Eigen::Vector3d vectorCal = rotationMatrixToParameters(rotation);
  cout << "vectorCal: " << vectorCal.transpose() << endl;
  cout << "to rotation: " << endl << parametersToRotationMatrix(vectorCal)<< endl;
  /// close to Pi
  Eigen::Vector3d thetaPi = (vectorOri)*(1/vectorOri.norm())* 3.14159265358;
  Eigen::Matrix3d rotationPi = parametersToRotationMatrix(thetaPi);
  cout << "origin Pi: " << thetaPi.transpose() << endl;
  cout << "to rotationPi: " << endl << parametersToRotationMatrix(thetaPi)<< endl;
  Eigen::Vector3d vectorPi = rotationMatrixToParameters(rotation);
  cout << "to vectorPi: " << vectorPi.transpose()<< endl;
  cout << "to vectorPi: " << endl << parametersToRotationMatrix(vectorPi)<< endl;
  /// test the transformation
  double r = -0.952, p = -0.056, y = 1.571;
  /// euler angle to rotation matrix
  Eigen::AngleAxisd rotation_x(r, Eigen::Vector3d::UnitX());
  cout << "rotationX: " << endl << rotation_x.matrix() << endl;
  Eigen::AngleAxisd rotation_y(p, Eigen::Vector3d::UnitY());
  cout << "rotationY: " << endl << rotation_y.matrix() << endl;
  Eigen::AngleAxisd rotation_z(y, Eigen::Vector3d::UnitZ());
  cout << "rotationZ: " << endl << rotation_z.matrix() << endl;
  cout << "rotationXYZ: " << endl << rotation_x.matrix()*rotation_y.matrix()*rotation_z.matrix() << endl;
  /// 从imu坐标系到小车坐标系的旋转矩阵, this calculation is right : right mutiply
  cout << "rotationZYX: " << endl << rotation_z.matrix()*rotation_y.matrix()*rotation_x.matrix() << endl;
  ///
  Eigen::Isometry3d rotation_3d = Eigen::Isometry3d::Identity();
  rotation_3d.prerotate(Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()));
  rotation_3d.prerotate(Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()));
  rotation_3d.prerotate(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));
  /// result is the same with the ZYX, it says the prerotate is the right multiply
  cout << "## rotation_3d: " << endl << rotation_3d.rotation() << endl;
  cout << "## rotation_3d linear: " << endl << rotation_3d.linear() << endl;
  /// 使用 eigen 将旋转矩阵转换为欧拉角
  Eigen::Vector3d eulerAngle1 =
    rotation_3d.rotation().eulerAngles(2, 1, 0); //zyx顺序, regarding to right mutiply
  /// this output: -0.952 -0.056 1.571 , the same with input
  cout << "roll_2, pitch_2, yaw_2 = " << eulerAngle1[2]
       << " " << eulerAngle1[1]
       << " " << eulerAngle1[0] << endl << endl;
  /// verify the XY MUTIPLY
  cout << "## rotation  YX: " << endl << rotation_y.matrix()*rotation_x.matrix() << endl;
  cout << "## rotation  -Y-X: " << endl << rotation_y.matrix().transpose()*rotation_x.matrix().transpose() << endl;
  {
    Eigen::Vector3d P_imu_rtk(0.1059, 0.2789, 0.2900);
    std::cout << "P_imu_rtk distance: " << P_imu_rtk.squaredNorm() << std::endl;
    Eigen::Matrix4d R; R.setIdentity();
    cout << R.topLeftCorner<3,3>(0,0) << std::endl;
    cout << R(3,3) << std::endl;
    R.block<1,3>(3,1) = P_imu_rtk.transpose();
    cout << endl << R << endl;
    Eigen::Matrix3d R1; R1.setIdentity();
//    cout << R1(3,3) << endl;  /// release mode don't tell error, but debug can
    P_imu_rtk.tail(2) = P_imu_rtk.tail(2) + P_imu_rtk.tail(2);
    cout << "tail P_imu_rtk: " << P_imu_rtk.tail(2) << endl;
  }
  return 0;
}
