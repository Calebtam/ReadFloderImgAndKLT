//
// Created by zyn on 2023/05/22.
// Modified by zhongda on 2024/03/11 ： add aruco detector interface 
//
#include "obsystem.h"
OB_SYSTEM::~OB_SYSTEM() {}

OB_SYSTEM::OB_SYSTEM(std::string config_path) {
  // parse config params
  parser = std::make_shared<ov_core::YamlParser>(config_path);

  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  // setting the params
  ov_core::Printer::setPrintLevel(verbosity);

  params = std::make_shared<ov_msckf::VioManagerOptions>();
  params->print_and_load(parser);

  // cv::setNumThreads(0);

  detector = std::make_shared<ob_map::ArucoDetector>(params->camera_intrinsics[0]);

  sta = ob_aruco_status::SILENT;
  processing_thread = new std::thread(&OB_SYSTEM::processing, this);

  // params->use_multi_threading_subs = false;
  { /// add the fusion parameter
    Eigen::Matrix4d Trc_temp;
    parser->parse_config("T_robot_cam", Trc_temp);
    std::cout << "Trc: " << std::endl << Trc_temp << std::endl;

    Trc.rotate(Trc_temp.block<3,3>(0,0));
    Trc.pretranslate(Eigen::Vector3d(Trc_temp(0,3),Trc_temp(1,3),Trc_temp(2,3)));
  }
  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }
}

void OB_SYSTEM::stationCallback(double ts, ChargestationInfo station) {

  if(sta != ob_aruco_status::SILENT)
  {
    std::unique_lock<std::mutex> lck(station_mtx);      
    while(station_queue.size() > 5)
    {
      station_queue.pop_front();
    }
    station_queue.push_back(std::make_pair(ts, station));
    PRINT_INFO("============================== get station:  %s \n", std::to_string(ts).c_str());

    // PRINT_DEBUG("get image:  %s \n", std::to_string(ts).c_str());
    // std::sort(img_queue.begin(), img_queue.end());
  }
  // processing_thread_cv.notify_all();
}

void OB_SYSTEM::rgbCallback(double ts, cv::Mat image) {

  if(sta != ob_aruco_status::SILENT)
  {
    std::unique_lock<std::mutex> lck(img_queue_mtx);      
    while(img_queue.size() > 5)
    {
      img_queue.pop_front();
    }
    img_queue.push_back(std::make_pair(ts, image));
    // PRINT_INFO("============================== get image:  %s \n", std::to_string(ts).c_str());

    // PRINT_DEBUG("get image:  %s \n", std::to_string(ts).c_str());
    // std::sort(img_queue.begin(), img_queue.end());
  }
  processing_thread_cv.notify_all();
}

bool OB_SYSTEM::start() {
  if(sta == ob_aruco_status::SILENT || sta == ob_aruco_status::PAUSE)
  {
    sta = ob_aruco_status::RUNNING;
    PRINT_INFO(GREEN "[aruco] status = RUNNING..... \n" RESET);
    return true;
  }
  else
    return false;
}

bool OB_SYSTEM::pause() {
  if(sta == ob_aruco_status::RUNNING || sta == ob_aruco_status::WAITING)
  {  
    sta = ob_aruco_status::PAUSE;
    PRINT_INFO(RED "[aruco] status = PAUSE..... \n" RESET);
    return true;
  }
  else
    return false;
}

const ob_aruco_status OB_SYSTEM::getStatus() {
  return sta;
}

const char *OB_SYSTEM::getFusionVersion() {
  const char *vesion = "V1.0.0";
  return vesion;
}

std::shared_ptr<ov_core::YamlParser> OB_SYSTEM::getParse() { return parser; }

bool OB_SYSTEM::setCallbackForAruco(obsystem_aruco_callback callback) {
  if (pub_result_func == nullptr)
    pub_result_func = callback;
  else
    return false;

  return true;
}
void OB_SYSTEM::SetOutData(Eigen::Isometry3d &Tra, ob_aruco_result &result){
    Eigen::Matrix3d Twa;
    Twa << 0 , 0 , -1 , 1 , 0 , 0 , 0 , -1 , 0;

    Eigen::Isometry3d T_t = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d T_t_r;
    T_t_r << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    T_t.linear() = T_t_r;
    Eigen::Isometry3d tar = T_t * Tra.inverse();
    std::cout << " tar " << std::endl << tar.matrix() << std::endl;

    // T_t << 
    Eigen::Matrix3d t = Tra.rotation() * Twa.inverse();

    Eigen::Matrix<double, 3, 1> rpy = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 4, 1> q = Eigen::Matrix<double, 4, 1>::Zero();
    rpy = ov_core::rot2rpy(t);
    q = ov_core::rot_2_quat(t);

    // std::cout << " Tra " << std::endl << Tra.matrix() << std::endl;

    result.camera_pose[0] = q.x();
    result.camera_pose[1] = q.y();
    result.camera_pose[2] = q.z();
    result.camera_pose[3] = q.w();
    result.camera_pose[4] = Tra.inverse().translation().z();
    result.camera_pose[5] = Tra.inverse().translation().x();
    result.camera_pose[6] = -Tra.inverse().translation().y();
    result.rpyxyz[0] = rpy.x();
    result.rpyxyz[1] = rpy.y();
    result.rpyxyz[2] = rpy.z();
    result.rpyxyz[3] = Tra.inverse().translation().z();
    result.rpyxyz[4] = Tra.inverse().translation().x();
    result.rpyxyz[5] = -Tra.inverse().translation().y();
}

bool OB_SYSTEM::StatusCheck() {
  if(sta == ob_aruco_status::RUNNING || sta == ob_aruco_status::WAITING)
  {
    if(img_queue.empty()) 
    {
      usleep(5000);                     // 5ms   Waiting img
      sta = ob_aruco_status::WAITING;
      return false;
    }
    else
    
    {
      sta = ob_aruco_status::RUNNING;
      return true;
    }
  }
  else if(sta == ob_aruco_status::SILENT)
  {
    struct timespec ts;
    ts.tv_sec = (long)1;
    ts.tv_nsec = (long)0;
    nanosleep(&ts, 0);
    // usleep(1000000);                    //1s      // waiting start
    return false;
  }
  else if(sta == ob_aruco_status::PAUSE)
  {
    usleep(50000);                      //50ms    // waiting start         
    { 
      // we dont want to process, so we will pop
      std::unique_lock<std::mutex> lck(img_queue_mtx);  
      img_queue.pop_front();
    }
    return false;
  }
}

void OB_SYSTEM::processing(){

  double max_cost = 0.0;
  double ave_cost = 0.0;
  uint32_t count = 0;
  double last_dis = 10.0;
  while(m_processFlag) {
  
    if(!StatusCheck())  continue;
    
    // thread_update_running = true;
    double latest_imu_msg_timestamp_cpy = -1;
    cv::Mat image;
    {
      std::unique_lock<std::mutex> lck(img_queue_mtx);  
      latest_imu_msg_timestamp_cpy = img_queue.back().first;
      image = img_queue.back().second;
      while(img_queue.front().first < latest_imu_msg_timestamp_cpy)
      {  
        img_queue.pop_front();
      }
    }
    // std::cout << " latest_imu_msg_timestamp_cpy " << latest_imu_msg_timestamp_cpy << std::endl;

    // 
    if(latest_imu_msg_timestamp_cpy > 0){
      ob_aruco_result result;
      result.timestamp = latest_imu_msg_timestamp_cpy;

      Eigen::Isometry3d Tca = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Tra = Eigen::Isometry3d::Identity();

      auto rT1 = boost::posix_time::microsec_clock::local_time();
      if(detector->detect(image))
      {
        result.DevAng = detector->CheckPosLoc(image, image.cols, last_dis);
        // if(detector->get_pose_aruco(image, Tca, result.DevAng))
        // {
        //   Tra = Trc * Tca;//Tca.inverse();

        //   last_dis = Tra.translation().norm();
        //   // std::cout << " Tca " << std::endl << Tca.matrix() << std::endl;
        //   result.sta = 3;
        // }
        // else 
        if(detector->get_pose(image, Tca, result.DevAng))
        {
          Tra = Trc * Tca;//Tca.inverse();

          last_dis = Tra.translation().norm();
          result.sta = 2;
        }
        else
        {
          last_dis = 10.0;
          result.sta = 1;
          PRINT_INFO(BLUE "[aruco]: cant solve pose \n"RESET);
        }

      }
      else
      {
        last_dis = 10.0;
        result.sta = 0;
        PRINT_INFO(BLUE "[aruco]: dont discover the mark \n"RESET);
      }

      SetOutData(Tra, result);

      PRINT_INFO( GREEN "#### callback stamp:  %lld\n", result.timestamp);
      if(result.sta > 1)
        for(int j = 0; j < 6; j++ )
          if(j < 3)
          {
            if(j==0)
              PRINT_INFO("#### sta %d   rpy(degree) ", result.sta);
            std::cout << result.rpyxyz[j] * 57.3 << " ";
            // PRINT_INFO("%.6f  ", result.rpyxyz[j] * 57.3);
          }
          else
          {
            if(j==3)
              std::cout << "  xyz(m) ";
            // PRINT_INFO("%.6f  ", result.rpyxyz[j]);
            std::cout << result.rpyxyz[j] << " ";
          }
      PRINT_INFO("derivation angle = %.6f degree"RESET, result.DevAng * 57.3);
      std::cout << std::endl << std::endl;
      // PRINT_INFO(RESET"\n\n");

      auto rT2 = boost::posix_time::microsec_clock::local_time();
      double cost = (rT2 - rT1).total_microseconds() * 1e-3;
      if(cost > max_cost) max_cost = cost;
      count++;
      ave_cost = (ave_cost * (count-1) + cost) / count;
      // printf("[OB_SYS]: cost: %.3fms  ave: %.3fms  max: %.3fms\n", cost, ave_cost, max_cost);


      if (pub_result_func != nullptr){
        pub_result_func(&result);
      }
#ifdef PC_VISUALIZATION
      // cv::imshow("img ", image);
      // cv::waitKey(10);
#endif
    }
    // thread_update_running = false;
    if(!m_processFlag) { break;}
    unique_lock<mutex> lock(processing_thread_mtx);
    // printf(RED " thread: wait for image !!\n" RESET);
    processing_thread_cv.wait(lock);
  }

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
//  if (!_app->get_params()->use_multi_threading_subs) {  // can happen to join itself when quit
//    processing_thread->join();
//  } else {
//    processing_thread->detach();
//  }
}