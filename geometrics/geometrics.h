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

#ifndef OBSLAM_VO_TRIANGULATION_H_
#define OBSLAM_VO_TRIANGULATION_H_

#include "camera.h"
#include "undistortion.h"
#include <iostream>
#include <Eigen/Dense>


    /**
     * @brief Triangulate a 3D point given a camera model and a set of 2D measurements
     * @param camera The camera model
     * @param T_cam_world List of the transformation matrix from world to camera coordinates
     * @param pixels List of the 2D measurements in each camera
     * @param point The resulting triangulated 3D point
     * @param min_movement The minimum movement of camera to be a valid triangulation
     * @param max_dist The maximum distance from camera to be a valid triangulation (-1 for no threshold)
     * @param reprojection_threshold The reprojection threshold to be a valid triangulation (-1 for no threshold)
     * @param min_parallax The minimum parallax angle (in degrees) to be a valid triangulation (-1 for no threshold)
     * @return True if triangulation is successful, false otherwise
     */
    bool TriangulatePoint(
        const std::shared_ptr<vo::PinholeCamera4d>& camera,
        const std::vector<Eigen::Matrix4d>& T_cam_world,
        const std::vector<Eigen::Vector2d>& pixels,
        Eigen::Vector3d& point,
        const double min_movement = -1,
        const double max_dist = 100, 
        const double reprojection_threshold = -1,
        const double min_parallax = -1);

    bool TriangulatePointWithCovariance(        
        const std::shared_ptr<vo::PinholeCamera4d>& camera,
        const std::vector<Eigen::Matrix4d>& T_cam_world,
        const std::vector<Eigen::Vector2d>& pixels,
        Eigen::Vector3d& point,
        Eigen::Matrix3d& covariance,
        const double min_movement,
        const double max_dist, 
        const double reprojection_threshold,
        const double min_parallax);

#endif // OBSLAM_VO_TRIANGULATION_H_