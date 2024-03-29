#pragma once

//#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace cg {

const double kDegreeToRadian = M_PI / 180.;

class Utils {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static Eigen::Isometry3d getTransformEigen(const std::string &field) {
    Eigen::Isometry3d T;
    cv::Mat c = getTransformCV(field);

    T.linear()(0, 0) = c.at<double>(0, 0);
    T.linear()(0, 1) = c.at<double>(0, 1);
    T.linear()(0, 2) = c.at<double>(0, 2);
    T.linear()(1, 0) = c.at<double>(1, 0);
    T.linear()(1, 1) = c.at<double>(1, 1);
    T.linear()(1, 2) = c.at<double>(1, 2);
    T.linear()(2, 0) = c.at<double>(2, 0);
    T.linear()(2, 1) = c.at<double>(2, 1);
    T.linear()(2, 2) = c.at<double>(2, 2);
    T.translation()(0) = c.at<double>(0, 3);
    T.translation()(1) = c.at<double>(1, 3);
    T.translation()(2) = c.at<double>(2, 3);
    return T;
  }

  static cv::Mat getTransformCV(const std::string &field) {
    cv::Mat T;
    try {
      // first try reading kalibr format
      T = getKalibrStyleTransform(field);
    } catch (std::runtime_error &e) {
      // maybe it's the old style format?
      std::cout << "cannot read transform " << field << " in kalibr format, trying old one!" << std::endl;
      try {
        T = getVec16Transform(field);
      } catch (std::runtime_error &e) {
        std::string msg = "cannot read transform " + field + " error: " + e.what();
//        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
      }
    }
    return T;
  }

  static cv::Mat getVec16Transform(const std::string &field) {
    std::vector<double> v;
//    nh.getParam(field, v);
    if (v.size() != 16) {
      throw std::runtime_error("invalid vec16!");
    }
    cv::Mat T = cv::Mat(v).clone().reshape(1, 4);  // one channel 4 rows
    return T;
  }

  static cv::Mat getKalibrStyleTransform(const std::string &field) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
//    XmlRpc::XmlRpcValue lines;
//    if (!nh.getParam(field, lines)) {
//      throw(std::runtime_error("cannot find transform " + field));
//    }
//    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
//      throw(std::runtime_error("invalid transform " + field));
//    }
//    for (int i = 0; i < lines.size(); i++) {
//      if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
//        throw(std::runtime_error("bad line for transform " + field));
//      }
//      for (int j = 0; j < lines[i].size(); j++) {
//        if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
//          throw(std::runtime_error("bad value for transform " + field));
//        } else {
//          T.at<double>(i, j) = static_cast<double>(lines[i][j]);
//        }
//      }
//    }
    return T;
  }

  static double condition_number(const Eigen::MatrixXd &A) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValues;
    singularValues.resize(svd.singularValues().rows(), 1);
    singularValues = svd.singularValues();
    double cond = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);
    return std::abs(cond);
  }

  static Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
    return w;
  }

  static Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
  }

  static Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = -skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
  }

  static Eigen::Vector3d rot_mat_to_vec(const Eigen::Matrix3d &R) {
    Eigen::Vector3d vec_r;
    cv::Mat Rmat, rvec;
    cv::eigen2cv(R, Rmat);
    cv::Rodrigues(Rmat, rvec);
    cv::cv2eigen(rvec, vec_r);
    return vec_r;
  }

  static Eigen::Matrix3d rot_vec_to_mat(const Eigen::Vector3d &rvec) {
    return Eigen::AngleAxisd(rvec.norm(), rvec.normalized()).toRotationMatrix();
  }

  static  Eigen::Matrix4d icp_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B){
        /*
        Notice:
        1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
        2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
        */
        Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4);
        Eigen::Vector3d centroid_A(0,0,0);
        Eigen::Vector3d centroid_B(0,0,0);
        Eigen::MatrixXd AA = A;
        Eigen::MatrixXd BB = B;
        int row = A.rows();

        for(int i=0; i<row; i++){
            centroid_A += A.block<1,3>(i,0).transpose();
            centroid_B += B.block<1,3>(i,0).transpose();
        }
        centroid_A /= row;
        centroid_B /= row;
        for(int i=0; i<row; i++){
            AA.block<1,3>(i,0) = A.block<1,3>(i,0) - centroid_A.transpose();
            BB.block<1,3>(i,0) = B.block<1,3>(i,0) - centroid_B.transpose();
        }

        Eigen::MatrixXd H = AA.transpose()*BB;
        Eigen::MatrixXd U;
        Eigen::VectorXd S;
        Eigen::MatrixXd V;
        Eigen::MatrixXd Vt;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

      Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        S = svd.singularValues();
        V = svd.matrixV();
        Vt = V.transpose();

        R = Vt.transpose()*U.transpose();

        if (R.determinant() < 0 ){
            Vt.block<1,3>(2,0) *= -1;
            R = Vt.transpose()*U.transpose();
        }

        t = centroid_B - R*centroid_A;

        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        return T;
  }
  static  Eigen::Matrix4d icp_transform(std::vector<Eigen::Vector3d> &rtk_align,
                                                std::vector<Eigen::Vector3d> &vio_align){
      int size = rtk_align.size();
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size,3);
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(size,3);
      for(int index = 0; index < size; index++){
          A.block<1,3>(index,0) = rtk_align[index].transpose();
          B.block<1,3>(index,0) = vio_align[index].transpose();
      }
      return icp_transform(A,B);
  }

  static  Eigen::Matrix4d icp_2d_transform(std::vector<Eigen::Vector3d> &rtk_align,
                                             std::vector<Eigen::Vector3d> &vio_align){
      int size_num = rtk_align.size();
      bool multiData = false;
      int size  = multiData ? size_num : 1;
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size,2);
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(size,2);

      if(multiData){
          for(int index = 0; index < size; index++){
              A.block<1,2>(index,0) = Eigen::Vector2d(rtk_align[index].x(), rtk_align[index].y()).transpose();
              B.block<1,2>(index,0) = Eigen::Vector2d(vio_align[index].x(), vio_align[index].y()).transpose();
          }
      }else{   /// just use the one data
          A.block<1,2>(0,0) = Eigen::Vector2d(rtk_align[size_num-1].x(), rtk_align[size_num-1].y()).transpose();
          B.block<1,2>(0,0) = Eigen::Vector2d(vio_align[size_num-1].x(), vio_align[size_num-1].y()).transpose();
      }
      return icp_2d_transform(A,B);
  }
  static  Eigen::Matrix4d icp_2d_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
      Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
      Eigen::MatrixXd AA = A;
      Eigen::MatrixXd BB = B;
      int row = A.rows();
      double up = 0.0, down = 0.0;
//      bool flag_just_use_one_data = true;
      Eigen::Vector2d centroid_A(0, 0);
      Eigen::Vector2d centroid_B(0, 0);

      for (int i = 0; i < row; i++) {
          centroid_A += A.block<1, 2>(i, 0).transpose();
          centroid_B += B.block<1, 2>(i, 0).transpose();
      }
//      if (row > 1) {
//          centroid_A /= row;
//          centroid_B /= row;
//          /// can not use the following average process, vector(A) x vector(B)/vector(A)*vector(B)
//          for (int i = 0; i < row; i++) {
//              AA.block<1, 2>(i, 0) = A.block<1, 2>(i, 0) - centroid_A.transpose();
//              BB.block<1, 2>(i, 0) = B.block<1, 2>(i, 0) - centroid_B.transpose();
//          }
//      }
      for (int i = 0; i < row; i++) {
          up += AA.block<1, 2>(i, 0).y() * BB.block<1, 2>(i, 0).x() -
                AA.block<1, 2>(i, 0).x() * BB.block<1, 2>(i, 0).y();
          down += AA.block<1, 2>(i, 0).x() * BB.block<1, 2>(i, 0).x() +
                  AA.block<1, 2>(i, 0).y() * BB.block<1, 2>(i, 0).y();
      }
      double theta = atan(up / down);
      if (down <= 0) {
          theta = 3.14159265358 + theta;
      }
      std::cout << "## row num: " << row << ",up: " << up << ",down: " << down << ",theta: " << theta << std::endl;
      T(0, 0) = cos(theta);
      T(0, 1) = -sin(theta);
      T(1, 0) = sin(theta);
      T(1, 1) = cos(theta);
      T.block<2, 1>(0, 3) = centroid_A - T.topLeftCorner(2, 2) * centroid_B;  // no use
      return T;
  }
};

}  // namespace cg