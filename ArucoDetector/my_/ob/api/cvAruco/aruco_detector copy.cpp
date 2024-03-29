/*****************************************************************************
 *  Orbbec openvins
 *  Copyright (C) 2024 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec openvins.
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 *  parties who have NOT received written authorization from the owner.
 *
 *  Description
 ****************************************************************************/
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco_detector.h"
#include <opencv2/core/eigen.hpp>
using namespace ob_map;
#define USE_SOLVEPNP_IPPE

ArucoDetector::ArucoDetector(const std::shared_ptr<ov_core::CamBase>& cam):cam_intrinsic(cam) {
  aruco_dict = cv::aruco::getPredefinedDictionary(tag.dict);
  aruco_params = cv::aruco::DetectorParameters::create();
//  create_maker();//run only once
  // 3d position for corners
  corners_3d.resize(4);
  corners_3d[0] = cv::Point3f(-tag.half_square_length, tag.half_square_length, 0);
  corners_3d[1] = cv::Point3f(tag.half_square_length, tag.half_square_length, 0);
  corners_3d[2] = cv::Point3f(tag.half_square_length, -tag.half_square_length, 0);
  corners_3d[3] = cv::Point3f(-tag.half_square_length, -tag.half_square_length, 0);
}

void ArucoDetector::create_maker() {
  //create maker
  cv::Mat marker_img;
  cv::aruco::drawMarker(aruco_dict, 0, 600, marker_img, 1);
  cv::imwrite("/home/minjian/projects/ros_ws/aruco_maker_DICT_APRILTAG_36h11.png", marker_img);
  marker_img = cv::imread("/home/minjian/projects/ros_ws/aruco_maker_DICT_APRILTAG_36h11.png", cv::IMREAD_GRAYSCALE);
  cv::imshow("aruco_maker", marker_img);
  cv::waitKey(5);
  //check detector
//  cv::Mat im_bg = marker_img;
  cv::Mat im_bg = 255*cv::Mat::ones(2*marker_img.rows, 2*marker_img.cols, CV_8UC1);
  marker_img.copyTo(im_bg(cv::Rect(300, 100, marker_img.cols, marker_img.rows)));
  detect(im_bg);
  std::cout <<  "aruco_detector check result: " << corners.size() << std::endl;
  cv::Mat im_show = im_bg.clone();
  if(im_bg.channels() != 3){
    cv::cvtColor(im_show, im_show, cv::COLOR_GRAY2BGR);
  }
  for(auto idc:corners){
    for(auto c:idc){
      cv::circle(im_show, c, 5, cv::Scalar(0,0,255), -1);
      cv::imshow("aruco detect corners",  im_show);
      cv::waitKey(0);
    }
  }
  cv::imshow("aruco detect corners",  im_show);
  cv::waitKey(0);
}

bool ArucoDetector::detect(const cv::Mat &image) {
  corners.clear();
  ids_aruco.clear();
  rejects.clear();
  // top-left, top-right, bottom-right, bottom-left order
#ifdef USE_SOLVEPNP_IPPE
  auto rT1 = boost::posix_time::microsec_clock::local_time();
  cv::aruco::detectMarkers(image, aruco_dict, corners, ids_aruco, aruco_params, rejects);//typically 15.452 ms in pc, 8~25ms
  auto rT2 = boost::posix_time::microsec_clock::local_time();
  // printf("[TRAJ]: detect keypoints: %.3fms\n", (rT2 - rT1).total_microseconds() * 1e-3);
#else
  std::vector<std::vector<cv::Point2f>> corners_ori, rejects_ori;
  auto rT1 = boost::posix_time::microsec_clock::local_time();
  cv::aruco::detectMarkers(image, aruco_dict, corners_ori, ids_aruco, aruco_params, rejects_ori);//typically 15.452 ms in pc, 8~25ms
  auto rT2 = boost::posix_time::microsec_clock::local_time();
  printf("[TRAJ]: detect keypoints: %.3fms\n", (rT2 - rT1).total_microseconds() * 1e-3);
  corners.resize(corners_ori.size());
  rejects.resize(rejects_ori.size());
  for(size_t i=0; i<corners_ori.size(); i++) {
    corners[i].resize(4);
    rejects[i].resize(4);
    for (size_t j = 0; j < 4; j++) {
      corners[i][tag.corner_order[j]] = corners_ori[i][j];
      rejects[i][tag.corner_order[j]] = rejects_ori[i][j];
    }
  }
#endif

#ifdef PC_VISUALIZATION
  cv::Mat im_show = image.clone();
  if(image.channels() != 3){
    cv::cvtColor(im_show, im_show, cv::COLOR_GRAY2BGR);
  }
  for(auto idc:corners){
    for(auto c:idc){
      cv::circle(im_show, c, 3, cv::Scalar(0,255,0), -1);
    }
  }
  for(auto idc:rejects){
    for(auto c:idc){
      cv::circle(im_show, c, 2, cv::Scalar(0,0,255), -1);
    }
  }

  cv::Mat imageCopy = image.clone();
  if(image.channels() != 3){
    cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2BGR);
  }

  if (ids_aruco.size() > 0) 
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids_aruco);
    
  cv::resize(imageCopy, imageCopy, imageCopy.size());
  cv::imshow("aruco detect out", imageCopy); 

  cv::resize(im_show, im_show, im_show.size());
  cv::imshow("aruco detect corners", im_show);
  cv::waitKey(10);

  if(corners.empty() || corners.at(0).empty())
  {
    std::cout << RED;
    if(!corners.empty())
      std::cout << "corner.size = " << corners.size() << "  point  " << corners.at(0).size() << std::endl;
    else
      std::cout << "corner.size = " << corners.size() << std::endl;
    
    std::cout << RESET;
    return false;
  }

#endif

  return corners.size() > 0;
}

double ArucoDetector::CheckPosLoc(const cv::Mat &image, int width, double distance){

  if(corners.size() > 0){
    float mid = width / 2.0;
    // int pix_th;
    // if(distance > 1)
    //   pix_th = 200;
    // else
    //   pix_th = 100;

    for(int i = 0; i < ids_aruco.size(); i++)
    {
      if(ids_aruco[i] == 0){
        
        cv::Point2f rectangle_mid;
        float factor = 1.0 / corners.at(i).size();

        for(auto p:corners[i]){
          cv::Point2f uv = cam_intrinsic->undistort_cv(p);
          // std::cout << " == "<< uv << std::endl << (uv * factor) << std::endl;

          rectangle_mid += (uv * factor) ;
        }

#ifdef PC_VISUALIZATION1
        cv::Point2f mid_rec_cv = cam_intrinsic->distort_cv(rectangle_mid);
        cv::Mat imageCube = image.clone();
        if(image.channels() != 3){
          cv::cvtColor(imageCube, imageCube, cv::COLOR_GRAY2BGR);
        }
        cv::circle(imageCube, mid_rec_cv, 2, cv::Scalar(0,0,255),2);
        cv::resize(imageCube, imageCube, imageCube.size());
        cv::imshow("rectangle center point", imageCube); 
        cv::waitKey(10);      
#endif
        double ang = -atan2(rectangle_mid.x, 1.0);
        // std::cout << "[aruco]: Deviation Angle = "  << ang*57.3 << " degree" << std::endl;
        return ang;
      }
    }
  }
  return -999; //奇怪的结果
}

bool ArucoDetector::get_pose(const cv::Mat &image, Eigen::Isometry3d& Tcw, double devAng){
  // get undistort 2d measurements
  std::vector<cv::Point2f> p_2ds;
  for(auto p:corners[0]){
    cv::Point2f uv = cam_intrinsic->undistort_cv(p);
    p_2ds.push_back(uv);
  }
  // calculate pose with pnp
  if(corners.size() > 0){
    // std::vector<cv::Vec3d> rvec, tvec;
    cv::Vec3d rvec, tvec;
//    auto rT1 = boost::posix_time::microsec_clock::local_time();
#ifdef USE_SOLVEPNP_IPPE
    bool valid = cv::solvePnP(corners_3d, p_2ds, cv::Matx33d::eye(), cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_IPPE);//0.116 ms in pc
    // std::cout << "sol2: r=" << rvec << "   t=" << tvec << std::endl << valid << std::endl;
#else
    //    auto rT2 = boost::posix_time::microsec_clock::local_time();
    bool valid = cv::solvePnP(corners_3d, p_2ds, cv::Matx33d::eye(), cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);//0.170ms in pc
    // std::cout << "sol1: r=" << rvec << "   t=" << tvec << std::endl << valid << std::endl;
#endif
//    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cam_intrinsic->get_K(), cam_intrinsic->get_D(), rvec, tvec);//no support fisheye model
//    auto rT3 = boost::posix_time::microsec_clock::local_time();
//    printf("[TRAJ]: pnp: %.3f vs %.3f ms\n", (rT2 - rT1).total_microseconds() * 1e-3, (rT3 - rT2).total_microseconds() * 1e-3);
    Eigen::Vector3d t_cw;
    cv::cv2eigen(tvec, t_cw);
    // if(valid && t_cw(2) < 0.6 && t_cw.norm() < 1)
    if(t_cw(2) > 0.8 || t_cw.norm() > 1.2 ||  abs(devAng)*57.3 > 20)
    { 
      PRINT_INFO("[aruco] pnp: aruco is too far away, norm= %fm, z= %fm, devAng=%f\n", t_cw(2), t_cw.norm(), devAng*57.3);
    }
    else if(!valid)
    {
      PRINT_INFO("[aruco] pnp: cant solve the pnp \n");
    }
    else
    {
      cv::Mat R_cv;
      cv::Rodrigues(rvec, R_cv);
      Eigen::Matrix3d R_cw;
      cv::cv2eigen(R_cv, R_cw);
      Tcw = Eigen::Isometry3d::Identity();
      Tcw.linear() = R_cw;
      Tcw.translation() = t_cw;
      bool pnp_valid = true;
      for(size_t i = 0; i < p_2ds.size(); ++i){
        Eigen::Vector3d pw(corners_3d[i].x, corners_3d[i].y, corners_3d[i].z);
        Eigen::Vector2d uv(p_2ds[i].x, p_2ds[i].y);
        auto pc =  R_cw * pw + t_cw;
        Eigen::Vector2d pc2 = pc.head(2).transpose()/pc.z();

        if((pc2 - uv).norm() > 0.01){

          std::cout << RED << "pc est: " << pc2.transpose() << std::endl;
          std::cout << "pc mea: " << uv.transpose() << RESET << std::endl;
          pnp_valid = false;
          break;
        }
      }
#ifdef PC_VISUALIZATION
      cv::Mat imageCopy = image.clone();
      if(image.channels() != 3){
        cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2BGR);
      }
      // draw axis for each marker 
      for(int i=0; i<ids_aruco.size(); i++) 
      {
        cv::drawFrameAxes(imageCopy, cam_intrinsic->get_K(), cam_intrinsic->get_D(), rvec, tvec, tag.half_square_length*2); 
      }
      cv::resize(imageCopy, imageCopy, imageCopy.size());
      cv::imshow("pnp drawAxis", imageCopy); 
      cv::waitKey(10);


      cv::Mat imageCube = image.clone();
      if(image.channels() != 3){
        cv::cvtColor(imageCube, imageCube, cv::COLOR_GRAY2BGR);
      }
      for(int i=0; i<ids_aruco.size(); i++){
          mpe::draw_cube(imageCube, ids_aruco[i], cam_intrinsic->get_K(), cam_intrinsic->get_D(), rvec, tvec, tag.half_square_length*2);
      }
      cv::resize(imageCube, imageCube, imageCube.size());
      cv::imshow("pnp cube", imageCube); 
      cv::waitKey(10);      
#endif
      return pnp_valid;
    }
  }
  return false;
}

bool ArucoDetector::get_pose_aruco(const cv::Mat &image, Eigen::Isometry3d &Tcw, double devAng){

  if(corners.empty() || corners.at(0).empty())
    return false;

  // std::cout << "point : " << corners.at(0).at(0).x << " " << corners.at(0).at(0).y << std::endl;

  // get undistort 2d measurements
  std::vector<std::vector<cv::Point2f>> p_2ds(1);
  for(auto& p:corners[0]){
    cv::Point2f uv = cam_intrinsic->undistort_cv(p);
    p_2ds[0].push_back(uv);
  }
  // calculate pose with pnp
  if(corners.size() > 0){
    std::vector<cv::Vec3d> rvec, tvec;
    cv::aruco::estimatePoseSingleMarkers(p_2ds, tag.half_square_length*2, cv::Matx33d::eye(), cv::Vec4d::zeros(), rvec, tvec);
//    auto rT1 = boost::posix_time::microsec_clock::local_time();
    // if(rvec[0])
    //   std::cout<< "sol3: r=" << rvec[0] << "     t=" << tvec[0] << std::endl;

//    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cam_intrinsic->get_K(), cam_intrinsic->get_D(), rvec, tvec);//no support fisheye model
//    auto rT3 = boost::posix_time::microsec_clock::local_time();
//    printf("[TRAJ]: pnp: %.3f vs %.3f ms\n", (rT2 - rT1).total_microseconds() * 1e-3, (rT3 - rT2).total_microseconds() * 1e-3);
    Eigen::Vector3d t_cw;
    cv::cv2eigen(tvec[0], t_cw);
    if(0)
    // if(t_cw(2) > 0.8 || t_cw.norm() > 1.2 ||  abs(devAng)*57.3 > 20)
    { 
      PRINT_INFO("[aruco] aruco is too far away, norm= %fm, z= %fm, mdevAng=%f\n", t_cw(2), t_cw.norm(), devAng*57.3);
    }
    else
    {
      cv::Mat R_cv;
      cv::Rodrigues(rvec[0], R_cv);
      Eigen::Matrix3d R_cw;
      cv::cv2eigen(R_cv, R_cw);
      Tcw = Eigen::Isometry3d::Identity();
      Tcw.linear() = R_cw;
      Tcw.translation() = t_cw;
      bool pnp_valid = true;
      for(size_t i = 0; i < p_2ds[0].size(); ++i){
        Eigen::Vector3d pw(corners_3d[i].x, corners_3d[i].y, corners_3d[i].z);
        Eigen::Vector2d uv(p_2ds[0][i].x, p_2ds[0][i].y);
        auto pc =  R_cw * pw + t_cw;
        Eigen::Vector2d pc2 = pc.head(2).transpose()/pc.z();

        if((pc2 - uv).norm() > 0.01){
          pnp_valid = false;
          std::cout << RED << "pc est: " << pc2.transpose() << std::endl;
          std::cout << "pc mea: " << uv.transpose() << RESET << std::endl;
          break;
        }
      }
#ifdef PC_VISUALIZATION
      // draw axis for each marker
      cv::Mat imageCopy = image.clone();
      if(image.channels() != 3){
        cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2BGR);
      }      
      for(int i=0; i<ids_aruco.size(); i++) 
      {
        // cv::drawAxis
        // cv::drawFrameAxes
        // cv::drawDetectedMarkers
        // cv::drawMarker
        cv::drawFrameAxes(imageCopy, cam_intrinsic->get_K(), cam_intrinsic->get_D(), rvec[0], tvec[0], tag.half_square_length*2); 
      }
      cv::resize(imageCopy, imageCopy, imageCopy.size());
      cv::imshow("Mark out", imageCopy); 
      cv::waitKey(10);


      cv::Mat imageCube = image.clone();
      if(image.channels() != 3){
        cv::cvtColor(imageCube, imageCube, cv::COLOR_GRAY2BGR);
      }
      for(int i=0; i<ids_aruco.size(); i++){
          mpe::draw_cube(imageCube, ids_aruco[i], cam_intrinsic->get_K(), cam_intrinsic->get_D(), rvec[0], tvec[0], tag.half_square_length*2);
      }
      cv::resize(imageCube, imageCube, imageCube.size());
      cv::imshow("Mark cube", imageCube); 
      cv::waitKey(10);      
#endif
      return pnp_valid;
    }
  }
  return false;
}