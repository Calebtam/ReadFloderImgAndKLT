//
// Created by zyn on 2023/5/24.
// Modified by zhongda on 2024/03/11 ： add aruco detector interface 
//
#include "obsystem.h"
#include <RosbagStorage/rosbag/bag.h>
#include <rosbag/view.h>
#include "cv_bridge/include/cv_bridge/cv_bridge.h"

void arucoCallback(const ob_aruco_result *result) {
  PRINT_INFO( GREEN "#### callback stamp:  %lld\n", result->timestamp);
  // std::cout << GREEN << "#### callback stamp: " << result->timestamp;
  // std::cout << std::endl << "#### camera_pose = ";
  // for(auto i : result->camera_pose)
  //   std::cout << i << " ";
  PRINT_INFO( "#### rpyxyz :  ");

  // std::cout << std::endl << "#### rpyxyz = ";
  for(int j = 0; j < 6; j++ )
    if(j < 3)
    {
      // std::cout << result->rpyxyz[j] * 57.3 << " ";
      PRINT_INFO("%.6f  ", result->rpyxyz[j] * 57.3);
    }
    else
    {
      PRINT_INFO("%.6f  ", result->rpyxyz[j]);
      // std::cout << result->rpyxyz[j] << " ";
    }

  // std::cout << RESET << std::endl;
  PRINT_INFO(RESET"\n");
  // PRINT_INFO("#### callback fusion status:  %d\n", result->fusion_status);
}

FUSION_HANDLE fusion_system;

void signal_callback_handler(int signum) {
  //  fusion_system->visualize_final();
  ob_stop(fusion_system);
  ob_release(fusion_system);
  PRINT_INFO(" Ctr + C Interrupt, Quit perfectly!!");
  exit(signum);
}

// set stack capture
extern void dump(void);
extern void signal_handler(int signo);
int add(int num) {
  int ret = 0x00;
  int *pTemp = NULL;

  /* 这将导致一个段错误，致使程序崩溃退出 */
  *pTemp = 0x01;

  ret = num + *pTemp;

  return ret;
}

int main(int argc, char **argv) {
  const char *config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }
  signal(SIGABRT, signal_handler); /* 为SIGSEGV信号安装新的处理函数 */ // SIGABRT
  signal(SIGSEGV, signal_handler);
  //  add(1);
  signal(SIGINT, signal_callback_handler); // control + c capture
  ob_create(&fusion_system, config_path);

  std::shared_ptr<ov_core::YamlParser> parser = ((OB_SYSTEM *)fusion_system)->getParse();

  //===================================================================================
  // Our camera topics
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < 1; i++) {
    std::string cam_topic;
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam: %s\n", cam_topic.c_str());
  }

  // Location of the ROS bag we want to read in
  std::string path_to_bag;
  parser->parse_config("path_bag", path_to_bag);
  PRINT_DEBUG("[SERIAL]: ros bag path is: %s\n", path_to_bag.c_str());

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  parser->parse_config("bag_start", bag_start);
  parser->parse_config("bag_durr", bag_durr);

  PRINT_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr);

  //===================================================================================
  // Load rosbag here, and find messages we can play
  rosbag::Bag bag;
  bag.open(path_to_bag, (uint32_t)rosbag::bagmode::BagMode::Read);

  // We should load the bag as a view
  // Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  auto time_init = view_full.getBeginTime();
  time_init += ob_slam::Duration(bag_start);

  auto time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ob_slam::Duration(bag_durr);

  PRINT_DEBUG("time start = %.6f\n", time_init.toSec());
  PRINT_DEBUG("time end   = %.6f\n", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    PRINT_ERROR(RED "[SERIAL]: No messages to play on specified topics.  Exiting.\n" RESET);
    return EXIT_FAILURE;
  }

  ob_register_aruco_result_callback(fusion_system, arucoCallback);

  ob_start(fusion_system);

  std::string version = ob_get_version(fusion_system);
  PRINT_INFO(RED "vesion: %s\n" RESET, version.c_str());

  bool flag = false;

  rosbag::View::iterator iter, next_iter;
  for (iter = view.begin(), next_iter = view.begin(); iter != view.end(); iter++) {
// #ifdef PC_VISUALIZATION
//     while (viz->viewer_ptr->mbPausePlayBag) {
//       usleep(5000);
//     }
// #endif

    // PRINT_INFO(GREEN "loop :\n" RESET);

    if (iter->getTopic() == topic_cameras.at(0)) {
        // PRINT_INFO(GREEN "topic_cameras: %s\n" RESET, topic_cameras.at(0).c_str());

      // if (cg::FUSION_TYPE::FUSE_RTK_ONLY != ((OB_SYSTEM *)fusion_system)->getFusionType()) {
        if (iter->isType<sensor_msgs::Image>()) {
          const auto image_ptr = iter->instantiate<sensor_msgs::Image>();
          cv_bridge::CvImageConstPtr cv_ptr;
          cv_ptr = cv_bridge::toCvShare(image_ptr, sensor_msgs::image_encodings::MONO8);
          cv::Mat mat = cv_ptr->image;
          // if (camera_num == 2) {
            // ob_add_image_stereo_single(fusion_system, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows, mat.data, mat.channels());
          // } else {
            ob_add_image(fusion_system, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows, mat.data, mat.channels());
            // ob_add_image(fusion_system, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows / 2, mat.data, mat.channels());
          // }
          // cv::imshow("sensor_msgs::Image", mat);
        } else {
          const auto image_ptr = iter->instantiate<ob_slam::sensor_msgs::CompressedImage>();
          cv::Mat mat = cv::imdecode(image_ptr->data, cv::IMREAD_GRAYSCALE);
          // if (camera_num == 2) {
            // ob_add_image_stereo_single(fusion_system, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows, mat.data, mat.channels());
          // } else {
            ob_add_image(fusion_system, image_ptr->header.stamp.toNSec(), mat.cols, mat.rows / 2, mat.data, mat.channels());
          // }
          // cv::imshow("sensor_msgs::CompressedImage", mat);
        }
      // }
      //      static int image_num = 0;
      //      image_num++;
      //      if(image_num > 200) break;
      // cv::waitKey(0);
    }
   
    if (++next_iter != view.end()) {
      double sleep_time = next_iter->getTime().toSec() - iter->getTime().toSec();
      // sleep_time = sleep_time / ((OB_SYSTEM *)fusion_system)->getBagSpeed();
      sleep_time = sleep_time / 0.5;

      struct timespec ts;
      ts.tv_sec = (long)sleep_time;
      ts.tv_nsec = (long)((sleep_time - (long)sleep_time) * 1e9);
      nanosleep(&ts, 0);
    }
  }
  PRINT_WARNING(RED "[SERIAL]: play bag finished\n" RESET);
   ob_stop(fusion_system);   /// can check the path
   ob_release(fusion_system);
  // Final visualization
  // viz->visualize_final();
  // don't exit
  while (1) {
    usleep(500000);
  }
  return 0;
}