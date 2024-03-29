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

void SetParams(cv::SimpleBlobDetector::Params &params_)
{
    params_.thresholdStep = 10;    //二值化的阈值步长，即公式1的t
    params_.minThreshold = 50;   //二值化的起始阈值，即公式1的T1
    params_.maxThreshold = 220;    //二值化的终止阈值，即公式1的T2

    //重复的最小次数，只有属于灰度图像斑点的那些二值图像斑点数量大于该值时，该灰度图像斑点才被认为是特征点
    params_.minRepeatability = 2;   

    //最小的斑点距离，不同二值图像的斑点间距离小于该值时，被认为是同一个位置的斑点，否则是不同位置上的斑点
    params_.minDistBetweenBlobs = 10;
    
    params_.filterByColor = true;    //斑点颜色的限制变量
    params_.blobColor = 0;    //表示只提取黑色斑点；如果该变量为255，表示只提取白色斑点
 
    params_.filterByArea = true;    //斑点面积的限制变量
    params_.minArea = 200;    //斑点的最小面积
    params_.maxArea = 5000;    //斑点的最大面积
 
    params_.filterByCircularity = false;    //斑点圆度的限制变量，默认是不限制
    params_.minCircularity = 0.8f;    //斑点的最小圆度
    //斑点的最大圆度，所能表示的float类型的最大值
    params_.maxCircularity = std::numeric_limits<float>::max();
 
    // params_.filterByInertia = true;    //斑点惯性率的限制变量
    // params_.minInertiaRatio = 0.1f;    //斑点的最小惯性率
    // params_.maxInertiaRatio = std::numeric_limits<float>::max();    //斑点的最大惯性率
 
    params_.filterByConvexity = true;    //斑点凸度的限制变量
    params_.minConvexity = 0.95f;    //斑点的最小凸度
    params_.maxConvexity = std::numeric_limits<float>::max();    //斑点的最大凸度
}

    int main(int argc, char **argv)
    {

        std::string config_path = "../config/dcw3_15/estimator_config.yaml";
        std::string img_path = "/home/tam/wk/aruco/version/mark/891772.000000.jpg";
        int n = 20;
        if (argc > 1) {
            config_path = argv[1];
            // std::cout << " = " << config_path.c_str() << " = "<< std::endl;
            if(argv[2])
            {
                n = double(std::stod(argv[2]));
                std::cout << " = " << config_path << " = " << n << " = " << std::endl;
            }
        
        }
        int show_ = 1;

        cv::Mat img = cv::imread(img_path, 1);
        // cv::Mat img1 = cv::imread(img_path, 0);


        if (img.empty())
        {
            return 0;
        }

        // if (show_)
        // {
            cv::imshow("camera jpg", img);
            // cv::waitKey(0);
        // }
            //使用灰度图像进行角点检测
        cv::Mat img1;
        cv::cvtColor(img, img1, cv::COLOR_BGR2GRAY);

        cv::SimpleBlobDetector::Params params_;
        SetParams(params_);

        auto detector = cv::SimpleBlobDetector::create(params_);


        // std::vector<cv::Point2f> &detect_points;
        std::vector<cv::KeyPoint> keypoints;
        // auto detector = cv::SimpleBlobDetector::create(params_);
        detector->detect(img1, keypoints);
        std::cout << " keypoints = " << keypoints.size() << std::endl;
        // if (show_)
        // {
            int max_x = 0, max_y = 0, min_x = 2000, min_y = 2000;
            cv::Mat temp_img = img.clone();
            for (auto &kpt : keypoints)
            {
                cv::circle(temp_img, kpt.pt, 3, cv::Scalar(0, 255, 0), 2);
                if(kpt.pt.x > max_x)    max_x = kpt.pt.x;
                if(kpt.pt.x < min_x)    min_x = kpt.pt.x;
                if(kpt.pt.y > max_y)    max_y = kpt.pt.y;
                if(kpt.pt.y < min_y)    min_y = kpt.pt.y;
            }
            cv::imshow("camera _feapg", temp_img);
            // cv::waitKey(0);
        // }

        // // if (target_.method == TargetInfo::METHOD::CORNER)
        // {
            // CornerDetect(post_img, target_.roi, 20, detect_points);
            std::vector<cv::Point2f> corner_pts(0);
            // cv::goodFeaturesToTrack(img, corner_pts, 20, 0.1, 30);

            //设置角点检测参数
            std::vector<cv::Point2f> corners;
            int max_corners = 20;           // 最大角点数目
            double quality_level = 0.01;    // 质量水平系数（小于1.0的正数，一般在0.01-0.1之间）
            double min_distance = 3.0;      // 最小距离，小于此距离的点忽略
            int block_size = 3;             // 使用的邻域数
            bool use_harris = false;        // false ='Shi Tomasi metric'
            double k = 0.04;                // Harris角点检测时使用
            cv::Point2f offset(min_x-25, min_y-25);
            cv::Point2f offset2(max_x+25, max_y+25);


            auto roi = cv::Rect(offset.x, offset.y, offset2.x-offset.x, offset2.y-offset.y);
            std::cout << " " << min_x << " "<< min_y << " " << max_x << " "<< max_y << " " << roi << std::endl;

            //角点检测
            cv::goodFeaturesToTrack(img1(roi), 
                                    corners, 
                                    max_corners, 
                                    quality_level, 
                                    min_distance, 
                                    cv::Mat(), 
                                    block_size, 
                                    use_harris, 
                                    k);


            std::cout << " corner_pts = " << corners.size() << std::endl;

            cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(img1(roi), corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
            // pts.clear();
   
            cv::Mat img2 = img.clone();
            //将检测到的角点绘制到原图上
            for (int i = 0; i < corners.size(); i++)
            {
                cv::circle(img2(roi), corners[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
            }
            cv::rectangle(img2, offset, offset2, cv::Scalar(0, 255, 255), 2, 8);

            cv::imshow("camera _feapg3123", img2);

            cv::waitKey(0);


            // for (auto &pt : corner_pts)
            // {
            //     // pts.emplace_back(attention_region.x + pt.x, attention_region.y + pt.y);
            //     pts.emplace_back(pt.x, pt.y);

            // }
            // std::cout << " detect_points  " << detect_points.size() << std::endl;
            // if (show_)
            // {
            //     cv::Mat temp_img = data_->clone();//= cv::imread("camera" + std::to_string(camera_id_) + "_fea" + ".jpg");
            //     for (auto &pt : detect_points)
            //     {
            //         cv::circle(temp_img, pt, 3, cv::Scalar(0, 0, 255));
            //     }
            //     cv::imshow("camera" + std::to_string(camera_id_) + "_fea" + ".jpg", temp_img);
            //     cv::waitKey(0);
            // }
        // }
        return 1;
    }