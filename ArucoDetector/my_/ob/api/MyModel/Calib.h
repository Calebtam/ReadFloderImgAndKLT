#pragma once

#include <istream>
#include <sophus/se3.hpp>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
// #include "aruco/Aruco.h"

// using namespace calibration;

// NOTE: camera frame: right down forward
//       vehicle frame: right down forward
class RotVertex : public g2o::BaseVertex<3, Sophus::SO3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override
  {
    // R_C_V rotation from vehicle frame to camera frame
    _estimate = Sophus::SO3d();
  }

  virtual void oplusImpl(const double *update) override
  {
    Eigen::Vector3d delta(update);
    if(delta != Eigen::Vector3d(0,0,0))
      setEstimate(Sophus::SO3d::exp(delta) * estimate());
  }

  virtual bool read(std::istream &in) { return true; }
  virtual bool write(std::ostream &out) const { return true; }
};

class ScaleVertex : public g2o::BaseVertex<1, double>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override
  {
    // R_C_V rotation from vehicle frame to camera frame
    _estimate = 0.0;
  }

  virtual void oplusImpl(const double *update) override
  {
    setEstimate(estimate() + update[0]);
  }

  virtual bool read(std::istream &in) { return true; }
  virtual bool write(std::ostream &out) const { return true; }
};

class PlaneDataEdge : public g2o::BaseMultiEdge<1, double>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlaneDataEdge(Eigen::Vector2d &p_w, Eigen::Vector3d &p_c, Eigen::Matrix3f new_K)
      // : BaseMultiEdge<1, double>(), p_w_(p_w), p_c_(p_c)
      //                                世界坐标    归一化坐标
      : g2o::BaseMultiEdge<1, double>(), p_w_(p_w), p_c_(p_c), new_K_(new_K)
  {
    resize(4);
  }

  virtual void computeError() override
  {
    // rot:  R_C_V
    const RotVertex *rot = static_cast<const RotVertex *>(_vertices[0]);
    const ScaleVertex *scale = static_cast<const ScaleVertex *>(_vertices[1]);
    const ScaleVertex *offset_x = static_cast<const ScaleVertex *>(_vertices[2]);
    const ScaleVertex *offset_y = static_cast<const ScaleVertex *>(_vertices[3]);

    Eigen::Vector3d offset(offset_x->estimate(), offset_y->estimate(), scale->estimate());
    Eigen::Vector3d pw(p_w_.x(), p_w_.y(), 0);
    pw += offset;
    pw = rot->estimate() * pw;
    pw.normalize();
    _error(0) = 1.0 - pw.dot(p_c_);

    // Eigen::Vector2d pcpc(p_c_.x() * new_K_(0, 0) + new_K_(0, 2), p_c_.y() * new_K_(1, 1) + new_K_(1, 2));
    // Eigen::Vector2d pwpw(pw.x() * new_K_(0, 0) + new_K_(0, 2), pw.y() * new_K_(1, 1) + new_K_(1, 2));
    // std::cout << "  cam " << pcpc.transpose() << "   sample " << pwpw.transpose() << "           " << _error(0) <<std::endl; 
  }

  // //linearizeOplus函数是边（误差）对于各个顶点（优化变量）的偏导数（雅可比矩阵）的解析求导函数，若此函数不被定义，则采用G2O内置的数值求导，但是数值求导很慢
  // virtual void linearizeOplus() override
  // {          
  //     const RotVertex *rot = static_cast<const RotVertex *>(_vertices[0]);
  //     const ScaleVertex *scale = static_cast<const ScaleVertex *>(_vertices[1]);
  //     const ScaleVertex *offset_x = static_cast<const ScaleVertex *>(_vertices[2]);
  //     const ScaleVertex *offset_y = static_cast<const ScaleVertex *>(_vertices[3]);

  //     // Eigen::Vector3d rot = Eigen::Vector3d(0., 0., 0.);
  //     // rot << parameters[0][0],parameters[0][1],parameters[0][2];

  //     Eigen::Vector3d offset(offset_x->estimate(), offset_y->estimate(), scale->estimate());
  //     Eigen::Vector3d pw(p_w_.x(), p_w_.y(), 0);
  //     pw += offset;
  //     pw = rot->estimate() * pw;
  //     pw.normalize();
  //     // 计算雅克比
  //     // if (jacobians != NULL && jacobians[0] != NULL) {
  //       // 1 * 6 
  //       auto f_R_px = -p_c_.transpose();

  //       Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  //       Eigen::Vector3d phi = -1 * rot->estimate().log();

  //       double phin = phi.norm() + 1e-8;
  //       Eigen::Matrix3d r = rot->estimate().matrix();//Sophus::SO3d::exp(rot).matrix();
  //       Eigen::Matrix3d rr = r * r;
  //       Eigen::Matrix3d J_l = I - (1 - cos(phin)) / std::pow(phin,2) * r + (phin - sin(phin))/std::pow(phin,3) * rr; 

  //       // Eigen::Vector3d f_r = (f_R_px * -1 * Sophus::SO3d::hat(pw) * J_l).transpose();
  //       // Eigen::Vector3d f_t = (f_R_px * r).transpose();
  //       // Eigen::Matrix<double, 6, 1> f;
  //       // f << f_r(0), f_r(1), f_r(2), f_t(0), f_t(1), f_t(2); 
  //       _jacobianOplus[0].block<3, 1>(0, 0) = (f_R_px * -1 * Sophus::SO3d::hat(pw) * J_l).transpose(); 
  //       _jacobianOplus[0].block<3, 1>(3, 0) = (f_R_px * r).transpose(); 

  //       // std::cout << "  J " << f.transpose() <<std::endl; 

  //     // }
  // }

  virtual bool read(std::istream &in) { return true; }
  virtual bool write(std::ostream &out) const { return true; }

private:
  Eigen::Vector2d p_w_;
  Eigen::Vector3d p_c_;
  Eigen::Matrix3f new_K_;
};