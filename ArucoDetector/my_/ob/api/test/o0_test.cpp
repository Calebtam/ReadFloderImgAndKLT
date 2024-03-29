#include <opencv2/opencv.hpp>
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

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
// #include "obsystem.h"
#include "obaruco_api.h"

using namespace std;
using namespace cv;

FUSION_HANDLE fusion_system;

void arucoCallback(const ob_aruco_result *result) {
//   PRINT_INFO( "#### callback stamp:  %lld\n", result->timestamp);
  std::cout << "#### callback stamp: " << result->timestamp;
  // std::cout << std::endl << "#### camera_pose = ";
  // for(auto i : result->camera_pose)
    std::cout << "#### rpyxyz :  ";
//   PRINT_INFO( "#### rpyxyz :  ");

  // std::cout << std::endl << "#### rpyxyz = ";
  for(int j = 0; j < 6; j++ )
    if(j < 3)
    {
      std::cout << result->rpyxyz[j] * 57.3 << " ";
    //   PRINT_INFO("%.6f  ", result->rpyxyz[j] * 57.3);
    }
    else
    {
    //   PRINT_INFO("%.6f  ", result->rpyxyz[j]);
      std::cout << result->rpyxyz[j] << " ";
    }

  std::cout << std::endl;
//   PRINT_INFO("\n");
  // PRINT_INFO("#### callback fusion status:  %d\n", result->fusion_status);
}

struct FN {
    std::string name;
    int number;
};
bool compare(const std::string& s1, const std::string& s2)
{
    std::regex re("\\d+"); // 正则表达式匹配数字
    std::smatch match1, match2;
    std::regex_search(s1, match1, re);
    std::regex_search(s2, match2, re);
    return std::stoi(match1.str()) < std::stoi(match2.str()); // 比较数字大小
}
bool cmp(const FN& a, const FN& b) {
    if(a.number < b.number)
        return true;
    else if(a.number > b.number)
        return false;
    else if(a.number == b.number)
        if(a.name[0] == 'L')        // 判断字符串第一个字母是否是L
            return true;
    // return a.number < b.number;
}
std::vector<FN> GetFileNamesInDir(const std::string& folderPath)
{
    DIR* dir = opendir(folderPath.c_str());
    // std::vector<std::string> fileNames;
    std::vector<FN> files;
    std::regex re("\\d+"); // 正则表达式匹配数字

    if (dir != NULL)
    {
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL)
        {
            std::string fileName = entry->d_name;
            if (fileName != "." && fileName != ".." && fileName.find(".jpg"))
            {
                // fileNames.push_back(fileName);
                FN tmp;
                std::smatch match;
                if(std::regex_search(fileName, match, re)){
                    tmp.name = fileName;   
                    // tmp.number =  std::stoi(match[0].str());
                    // std::cout << fileName << "  " << match[0] << std::endl;
                    // if(tmp.number > 70)
                        files.push_back(tmp);
                }
            }
        }
        closedir(dir);
    }
    // std::sort(files.begin(), files.end(), cmp);

    // for(auto tmp : files)
        // std::cout << tmp.name << "  " << tmp.number << std::endl;

    // std::sort(fileNames.begin(), fileNames.end());
    // std::sort(fileNames.begin(), fileNames.end(), []{
    //     std::regex re("\\d+"); // 正则表达式匹配数字
    //     std::smatch match1, match2;
    //     std::regex_search(s1, match1, re);
    //     std::regex_search(s2, match2, re);
    //     return std::stoi(match1.str()) < std::stoi(match2.str()); // 比较数字大小
    // }); // 排序
    return files;
}
 
int main(int argc, char **argv)
{
    std::string config_path = "../config/dcw3_15/estimator_config.yaml";
    std::string img_dir = "/root/aruco/img/";
    double n = 10;
    int th = 0;
    bool torf = false;
	if (argc > 1) {
		config_path = argv[1];
        // std::cout << " = " << config_path.c_str() << " = "<< std::endl;
        if(argv[2])
        {
            img_dir = argv[2];
            std::cout << " = " << config_path.c_str() << std::endl << img_dir << " = " << std::endl;
        }
    }
    ob_create(&fusion_system, config_path.c_str());

    ob_register_aruco_result_callback(fusion_system, arucoCallback);
    ob_start(fusion_system);

    // std::shared_ptr<OB_SYSTEM> detector = std::make_shared<OB_SYSTEM>(config_path);
    // detector->start();

    // std::string img_dir = "/root/aruco/img/";
    std::vector<FN> fileNames = GetFileNamesInDir(img_dir);  

    // cv::Rect img_roi = cv::Rect(0, 0, 1280, 800);
    std::vector<cv::Mat> imgs;
    for(int i=0; i < fileNames.size(); i++)
    {
        cv::Mat img1 = imread(img_dir + fileNames[i].name, 0);
        if(img1.empty())   
        {
            std::cout << "  " << img_dir + fileNames[i].name << "  is empty()  "<< std::endl;
            continue;
        }
        else
        {
            imgs.push_back(img1);
            std::cout << " " << fileNames[i].name << " "<< std::endl;

        }
    }

    // 设置环境变量以禁用多线程
    cv::setUseOptimized(torf);
    cv::setNumThreads(th);

    for(int ij = 0; ij < 1000; ij++)
    {
        for(int j = 0; j < imgs.size(); j++)
        {
            ob_add_image(fusion_system, fileNames[j].number, imgs[j].cols, imgs[j].rows, imgs[j].data, imgs[j].channels());

            // detector->rgbCallback(fileNames[j].number, imgs[j]);
            // 40ms
            // usleep(n*10000);
            if (j != imgs.size()-1) {
                double sleep_time = 1.0 / n; //帧间隔

                struct timespec ts;
                ts.tv_sec = (long)sleep_time;
                ts.tv_nsec = (long)((sleep_time - (long)sleep_time) * 1e9);
                //   std::cout << "ts.tv_nsec: " << ts.tv_nsec << std::endl;
                nanosleep(&ts, 0);
            }
        }
        double sleep_time = 1.0 / n; //帧间隔

        struct timespec ts;
        ts.tv_sec = (long)sleep_time;
        ts.tv_nsec = (long)((sleep_time - (long)sleep_time) * 1e9);
        //   std::cout << "ts.tv_nsec: " << ts.tv_nsec << std::endl;
        nanosleep(&ts, 0);
    }

    std::cout << " END +++++++ END " << std::endl;
    return 0;
}