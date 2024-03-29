//
// Created by zyn on 2023/05/22.
// Modified by zhongda on 2024/03/11 ： add aruco detector interface 
//
#include "obsystem.h"

// cg::ANGULAR_ERROR cg::State::kAngError = cg::ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

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


    // Eigen::Matrix4d Tci;
    // parser->parse_external("relative_config_imucam", "cam0", "T_cam_imu", Tci);
    // std::cout << "Tci: " << std::endl << Tci << std::endl;
  }
  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }
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
    PRINT_DEBUG("get image:  %s \n", std::to_string(ts).c_str());
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

bool OB_SYSTEM::StatusCheck() {
  if(sta == ob_aruco_status::RUNNING || sta == ob_aruco_status::WAITING)
  {
    if(img_queue.empty()) 
    {
      usleep(5000);                     // 5ms   Waiting img
      sta = ob_aruco_status::WAITING;
      waitCount++;
      if(waitCount > 200)
        PRINT_ERROR(RED"[aruco] ERROR: dont have img import \n"RESET);
      return false;
    }
    else
    {
      sta = ob_aruco_status::RUNNING;
      waitCount=0;
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

  Eigen::Matrix3d Twa;
  Twa << 0 , 0 , -1 , -1 , 0 , 0 , 0 , -1 , 0;
  double max_cost = 0.0;
  double ave_cost = 0.0;
  uint32_t count = 0;
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
      Eigen::Isometry3d Tca = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d Tra = Eigen::Isometry3d::Identity();
      Eigen::Matrix<double, 3, 1> rpy = Eigen::Matrix<double, 3, 1>::Zero();
      Eigen::Matrix<double, 4, 1> q = Eigen::Matrix<double, 4, 1>::Zero();

      auto rT1 = boost::posix_time::microsec_clock::local_time();
      if(detector->detect(image))
      {
        if(detector->get_pose_aruco(image,Tca))
        {
          Tra = Trc * Tca;//Tca.inverse();
          Eigen::Matrix3d t = Tra.rotation() * Twa.transpose();
          rpy = ov_core::rot2rpy(t);
          q = ov_core::rot_2_quat(t);
        
          std::cout << " Tca " << std::endl << Tca.matrix() << std::endl;
          // std::cout << " Tra " << std::endl << Tra.matrix() << std::endl;
          // std::cout << " t " << std::endl << t << std::endl;
          result.is_chargestation = true;
        }
        else if(detector->get_pose(image,Tca))
        {
          
        }

      }
      result.timestamp = latest_imu_msg_timestamp_cpy;
      result.camera_pose[0] = q.x();
      result.camera_pose[1] = q.y();
      result.camera_pose[2] = q.z();
      result.camera_pose[3] = q.w();
      result.camera_pose[4] = Tra.translation().x();
      result.camera_pose[5] = Tra.translation().y();
      result.camera_pose[6] = Tra.translation().z();
      result.rpyxyz[0] = rpy.x();
      result.rpyxyz[1] = rpy.y();
      result.rpyxyz[2] = rpy.z();
      result.rpyxyz[3] = Tra.translation().x();
      result.rpyxyz[4] = Tra.translation().y();
      result.rpyxyz[5] = Tra.translation().z();

      PRINT_INFO( GREEN "#### callback stamp:  %lld\n", result.timestamp);
      for(int j = 0; j < 6; j++ )
        if(j < 3)
        {
          // std::cout << result->rpyxyz[j] * 57.3 << " ";
          PRINT_INFO("%.6f  ", result.rpyxyz[j] * 57.3);
        }
        else
        {
          if(j==3)
            PRINT_INFO("       ");
          PRINT_INFO("%.6f  ", result.rpyxyz[j]);
          // std::cout << result->rpyxyz[j] << " ";
        }
      PRINT_INFO(RESET"\n\n");

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