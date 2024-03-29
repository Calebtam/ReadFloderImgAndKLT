//
// Created by zyn on 2023/05/22.
// Modified by zhongda on 2024/03/11 ： add aruco detector interface 
//

#include "obsystem.h"

#ifdef __cplusplus
extern "C" {
#endif

  ORBBEC_API void ob_register_aruco_result_callback(FUSION_HANDLE handle, obsystem_aruco_callback proc)
  {
      if( !((OB_SYSTEM*)handle)->setCallbackForAruco(proc) )
          PRINT_ERROR(RED "[SERIAL]: need to start the slam system !!")
      ;
  }

  ORBBEC_API void ob_create(FUSION_HANDLE *handle, const char *config_path)
  {

      PRINT_INFO(GREEN " start to create FUSION !! \n" RESET);
      *handle = new OB_SYSTEM(config_path);  // create is default state = silent
      if(*handle){
        PRINT_INFO(GREEN " create FUSION success !! \n" RESET);
      }
      else{
        PRINT_INFO(RED " create VIO failure !! \n" RESET);
      }
  }

  ORBBEC_API void ob_release(FUSION_HANDLE handle)
  {
    if(handle)
    {
        delete (OB_SYSTEM*)handle;
    }
  }

  ORBBEC_API int32_t ob_start(FUSION_HANDLE handle)
  {
    if(handle){
      return ((OB_SYSTEM*)handle)->start();
    }
    else{
      PRINT_INFO(RED " please create VIO firstly !! \n" RESET);
      return 0;
    }
  }

  ORBBEC_API int32_t ob_pause(FUSION_HANDLE handle)
  {
    if(handle){
      ((OB_SYSTEM*)handle)->pause();
      return 1;
    }
    else{
      PRINT_INFO(RED " please create VIO firstly !! \n" RESET);
      return 0;
    }

  }

  ORBBEC_API int32_t ob_add_image(FUSION_HANDLE handle, uint64_t timestamp,
                                      uint32_t width, uint32_t height, uint8_t *data, uint64_t size)
  {
      std::cout << std::endl;
      // PRINT_INFO(GREEN "ob_add_image: %f\n" RESET, timestamp);
      double imgTimeStamp = timestamp/1e6;  /// todo: becareful for the time unit
      if(size == 3)
      {
          cv::Mat img(height, width, CV_8UC3, data);
          ((OB_SYSTEM*)handle)->rgbCallback(imgTimeStamp, img.clone());
      } else
      {
          cv::Mat img(height, width, CV_8UC1, data);
          ((OB_SYSTEM*)handle)->rgbCallback(imgTimeStamp, img.clone());
      }
      return 1;
  }

  ORBBEC_API int32_t ob_add_station(FUSION_HANDLE handle, uint64_t timestamp, ChargestationInfo station_info)
  {
    double imgTimeStamp = timestamp/1e6;  /// todo: becareful for the time unit
    ((OB_SYSTEM*)handle)->stationCallback(timestamp,station_info);
    return 1;
  }


  ORBBEC_API const ob_aruco_status ob_get_status(FUSION_HANDLE handle){
    return ((OB_SYSTEM*)handle)->getStatus();
  }

  ORBBEC_API const char *ob_get_version(FUSION_HANDLE handle){
    return ((OB_SYSTEM*)handle)->getFusionVersion();
  }


#ifdef __cplusplus
}
#endif