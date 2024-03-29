//
// Created by zyn on 2023/05/22.
// Modified by zhongda on 2024/03/11 ： add aruco detector interface 
//

#ifndef TEST_H
#define TEST_H

#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <regex>

#include "test.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "obsystem.h"

// class OB_SYSTEM {

// public:
//     OB_SYSTEM() {};
//     ~OB_SYSTEM();
//     OB_SYSTEM( std::string config_path);

//     void rgbCallback(double ts, cv::Mat image);
//     // void stereoCallback(double ts0, cv::Mat image0, double ts1, cv::Mat image1);
//     bool setCallbackForAruco(obsystem_aruco_callback callback);
//     bool StatusCheck();
//     bool start();
//     bool pause();
//     const char *getFusionVersion();
//     const ob_status getStatus();

//     std::shared_ptr<ov_core::YamlParser> getParse();

// };



#endif //TEST_H
