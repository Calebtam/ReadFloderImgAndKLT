/*****************************************************************************
 *  Orbbec openvins
 *  Copyright (C) 2024 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec openvins.
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 * parties who have NOT received written authorization from the owner.
 *
 *  Description
 ****************************************************************************/

#ifndef __OPENVINS_ARUCO_DETECTOR_H__
#define __OPENVINS_ARUCO_DETECTOR_H__
#include <opencv2/aruco.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "options/VioManagerOptions.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "utils/draw_cube.h"

namespace ob_map {
// self printed tag with dict as vorx
struct SimVorxTag {
  cv::aruco::PREDEFINED_DICTIONARY_NAME dict = cv::aruco::DICT_APRILTAG_36h11;
  double half_square_length = 0.074;//self printed
  // real top-left, top-right, bottom-right, bottom-left corner id for detectMark
  int corner_order[4] = {2, 3, 0, 1};
};

struct VorxTag {
  cv::aruco::PREDEFINED_DICTIONARY_NAME dict = cv::aruco::DICT_APRILTAG_36h11;
  double half_square_length = 0.03;//vorx car
  // real top-left, top-right, bottom-right, bottom-left corner id for detectMarkers
  int corner_order[4] = {2, 3, 0, 1};
};

struct ObTag {
  cv::aruco::PREDEFINED_DICTIONARY_NAME dict = cv::aruco::DICT_6X6_50;
  double half_square_length = 0.1; // when print with A4 paper
  // real top-left, top-right, bottom-right, bottom-left corner id for detectMarkers
  int corner_order[4] = {0, 1, 2, 3};
};


struct MyTag {
  cv::aruco::PREDEFINED_DICTIONARY_NAME dict = cv::aruco::DICT_APRILTAG_36h11;
  double half_square_length = 0.045/2; // when print with A4 paper
  // real top-left, top-right, bottom-right, bottom-left corner id for detectMarkers
  int corner_order[4] = {2, 3, 0, 1};
};
class ArucoDetector {
public:
  ArucoDetector(const std::shared_ptr<ov_core::CamBase> &cam);
  ~ArucoDetector(){};
  void create_maker();
  bool detect(const cv::Mat &image);
  bool get_pose(const cv::Mat &image, Eigen::Isometry3d &Tcw, double devAng);
  bool get_pose_aruco(const cv::Mat &image, Eigen::Isometry3d &Tcw, double devAng);
  double CheckPosLoc(const cv::Mat &image, int width, double distance);

private:
  MyTag tag;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
  std::vector<cv::Point3f> corners_3d; // 3d position of corners in top-left, top-right, bottom-right, bottom-left order(must!)
  std::vector<std::vector<cv::Point3f>> corners_3d_my; // 3d position of corners in top-left, top-right, bottom-right, bottom-left order(must!)
  std::vector<cv::Point3f> corners_3d_match;
  std::vector<int> ids_aruco;
  std::vector<std::vector<cv::Point2f>> corners, rejects;
  std::shared_ptr<ov_core::CamBase> cam_intrinsic;
};
}
#endif //__OPENVINS_ARUCO_DETECTOR_H__
