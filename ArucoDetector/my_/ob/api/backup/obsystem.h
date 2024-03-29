//
// Created by zyn on 2023/05/22.
// Modified by zhongda on 2024/03/11 ： add aruco detector interface 
//

#ifndef OPENVINS_OBSYSTEM_H
#define OPENVINS_OBSYSTEM_H

// #include "core/VioManager.h"
#include <Eigen/StdVector>
#include <algorithm>
#include <atomic>
#include <boost/filesystem.hpp>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <deque>

#include "utils/utils.hpp"

#include "options/VioManagerOptions.h"
// #include "ros/ROS1Visualizer.h"
#include "utils/dataset_reader.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
// #include "fusion/fusion.hpp"
#include <signal.h>

#include "obaruco_api.h"
#include "aruco_detector.h"
// #include "nmea_converter.h"

// #include "RosbagStorage/rosbag/bag.h"
// #include "MessageType/sensor_msgs/CompressedImage.h"
// #include "MessageType/nav_msgs/Odometry.h"
// #include "MessageType/geometry_msgs/Vector3Stamped.h"
// #include "MessageType/geometry_msgs/Vector3.h"

#include <boost/date_time/posix_time/posix_time.hpp>

// using namespace ov_msckf;
// #ifndef USE_ROS
// // using namespace ob_slam;
// #endif

class OB_SYSTEM {

public:
    OB_SYSTEM() {};
    ~OB_SYSTEM();
    OB_SYSTEM( std::string config_path);

    void rgbCallback(double ts, cv::Mat image);
    // void stereoCallback(double ts0, cv::Mat image0, double ts1, cv::Mat image1);
    bool setCallbackForAruco(obsystem_aruco_callback callback);
    bool StatusCheck();
    bool start();
    bool pause();
    const char *getFusionVersion();
    const ob_aruco_status getStatus();

    std::shared_ptr<ov_core::YamlParser> getParse();

private:  
    /// Publish the current state
    void publish_state();

    /// Image processing
    void processing();

    // proocess img and pub
    bool m_processFlag = true;
    int waitCount = 0;

    ob_aruco_status sta = ob_aruco_status::SILENT;

    std::shared_ptr<ov_msckf::VioManagerOptions> params;
    std::shared_ptr<ov_core::YamlParser> parser;
    std::shared_ptr<ob_map::ArucoDetector> detector;

    std::thread *record_thread = nullptr;
    std::thread* processing_thread = nullptr;
    std::mutex processing_thread_mtx;
    std::condition_variable processing_thread_cv;

    // ob_slam::rosbag::Bag bag;
    // thread
    std::mutex img_queue_mtx;
    std::deque<std::pair<double, cv::Mat>> img_queue;
    // std::deque<std::pair<double, cv::Mat>> img_left_queue;
    // std::deque<std::pair<double, cv::Mat>> img_right_queue;

    Eigen::Isometry3d Trc =Eigen::Isometry3d::Identity();

    obsystem_aruco_callback pub_result_func = nullptr;
};



#endif //OPENVINS_OBSYSTEM_H
