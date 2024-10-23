// Copyright 2019 ETH Zürich, Thomas Schöps
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// The file is part of BADSLAM 

#include "undistortion.h"

namespace vo {

    void DecideForUndistortedCamera(
        const CameraPtr& camera,
        std::shared_ptr<PinholeCamera4d>& camera_undistorted,
        bool avoid_invalid_pixels) {

    // 初始化无畸变相机的初始内参：
    //     从输入相机对象中获取焦距和主点坐标，并复制到undistorted_initial_intrinsics数组中。
    double undistorted_initial_intrinsics[4];
    const double* fxfycxcy = static_cast<const double*>(camera->fxfycxcy());
    // copy fyfycxcy to undistorted_initial_intrinsics using memcpy
    memcpy(undistorted_initial_intrinsics, fxfycxcy, 4 * sizeof(double));

    // 创建无畸变相机对象：
    //     使用初始内参创建一个PinholeCamera4d对象，并设置其宽度和高度。
    camera_undistorted = std::make_shared<PinholeCamera4d>(camera->width(),
                                                            camera->height(),
                                                            undistorted_initial_intrinsics);
    // 计算无畸变相机的边界：
    //     遍历相机的每个像素点，计算其在无畸变相机中的投影位置。
    //     根据avoid_invalid_pixels参数，更新最小和最大像素坐标。 
    Eigen::Vector2d min_pixel;
    Eigen::Vector2d max_pixel;
    if (avoid_invalid_pixels) {
        min_pixel = Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
        max_pixel = Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    } else {
        min_pixel = Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        max_pixel = Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
    }
    
    // 调整无畸变相机的内参：
    //      根据计算出的边界，调整相机的宽度和高度，并更新主点坐标。  
    for (uint32_t x = 0; x < camera->width(); ++ x) {
        // top point (x, 0)
        Eigen::Vector3d dir = camera->UnprojectFromPixelCornerConv(Eigen::Vector2d(x, 0)).cast<double>();
        Eigen::Vector2d pixel = camera_undistorted->ProjectToPixelCornerConv(dir);
        min_pixel.y() = avoid_invalid_pixels ? std::max(min_pixel.y(), pixel.y()) : std::min(min_pixel.y(), pixel.y());
        
        // bottom point (x, height - 1)
        dir = camera->UnprojectFromPixelCornerConv(Eigen::Vector2d(x, camera->height() - 1)).cast<double>();
        pixel = camera_undistorted->ProjectToPixelCornerConv(dir);
        max_pixel.y() = avoid_invalid_pixels ? std::min(max_pixel.y(), pixel.y()) : std::max(max_pixel.y(), pixel.y());
    }
    
    for (uint32_t y = 0; y < camera->height(); ++ y) {
        // left point (0, y)
        Eigen::Vector3d dir = camera->UnprojectFromPixelCornerConv(Eigen::Vector2d(0, y)).cast<double>();
        Eigen::Vector2d pixel = camera_undistorted->ProjectToPixelCornerConv(dir);
        min_pixel.x() = avoid_invalid_pixels ? std::max(min_pixel.x(), pixel.x()) : std::min(min_pixel.x(), pixel.x());
        
        // right point (width - 1, y)
        dir = camera->UnprojectFromPixelCornerConv(Eigen::Vector2d(camera->width() - 1, y)).cast<double>();
        pixel = camera_undistorted->ProjectToPixelCornerConv(dir);
        max_pixel.x() = avoid_invalid_pixels ? std::min(max_pixel.x(), pixel.x()) : std::max(max_pixel.x(), pixel.x());
    }
    
    // The first pixel center will be placed on the coordinates of min_pixel, the last not farther away than max_pixel.
    int target_undistorted_width = static_cast<int>(max_pixel.x() - min_pixel.x());
    int target_undistorted_height = static_cast<int>(max_pixel.y() - min_pixel.y());
    double undistorted_final_intrinsics[4];
    undistorted_final_intrinsics[0] = undistorted_initial_intrinsics[0];
    undistorted_final_intrinsics[1] = undistorted_initial_intrinsics[1];
    undistorted_final_intrinsics[2] = undistorted_initial_intrinsics[2] - min_pixel.x();
    undistorted_final_intrinsics[3] = undistorted_initial_intrinsics[3] - min_pixel.y();

    camera_undistorted = std::make_shared<PinholeCamera4d>(target_undistorted_width,
                                                            target_undistorted_height,
                                                            undistorted_final_intrinsics);
    }

    void UndistortPixel(
        const Eigen::Vector2d& pixel,
        Eigen::Vector2d& pixel_undistorted,
        const CameraPtr& camera,
        const std::shared_ptr<PinholeCamera4d>& camera_undistorted) {
        
        pixel_undistorted = camera_undistorted->ProjectToPixelCornerConv(camera->UnprojectFromPixelCornerConv(pixel)).cast<double>();    
    }

} // namespace vo
