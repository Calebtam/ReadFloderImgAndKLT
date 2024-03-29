#pragma once

#include "MyModel/Calib.h"
#include "MyModel/km.h"
#include "unistd.h"
#include <sophus/se3.hpp>
#include "utils/utils.hpp"
#include "options/VioManagerOptions.h"
#include <thread>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace calibration
{
    enum STATE
    {
        INITIALIZING = 0,
        DATA_COLLECTING = 1,
        CALCULATING = 2,
        FINISHED = 3
    };

    struct TargetInfo
    {
        enum METHOD
        {
            CIRCLE = 0,
            CORNER = 1
        };

        METHOD method;
        cv::Rect roi;
        // camera style, virtual camera orientation
        Sophus::SO3d R_prior;
        Eigen::Vector3d init_center;
        cv::Mat fixed_flag;
        // camera style, in virtual camera coordinate
        std::vector<Eigen::Vector2d> template_points;
        std::vector<cv::Point2f> detect_points;
    };

    class tAruco
    {
    public:
        tAruco(std::shared_ptr<ov_msckf::VioManagerOptions> &cam);
        ~tAruco();
        void InsertImg(cv::Mat &data);
        void GetResult(Sophus::SO3d &rot);
        STATE GetStatus();
        double GetProgress();

    private:
        void Run();
        void CollectData();
        void Optimize();
        void Detection();
        void BlobDetect(cv::Mat &img, cv::Rect &roi, std::vector<cv::Point2f> &pts);
        void CornerDetect(cv::Mat &img, cv::Rect &attention_region, int max_num, std::vector<cv::Point2f> &pts);
        bool FindBBox(std::vector<cv::Point2f> &pts, cv::Rect &bbox);
        void GetMatch(std::vector<Eigen::Vector2d> &template_points, double distance, double offset_x, double offset_y,
                        Sophus::SO3d rot, std::vector<Eigen::Vector3d> camera_pts, std::vector<std::pair<int, int>> &match_ret);

        bool show = true;
        std::mutex img_mutex_, sta_mutex_;
        std::deque<std::shared_ptr<cv::Mat>> img_deque_;
        STATE sta_;
        bool input_flag_;
        bool flag_run_;
        int num_image_thr_;
        int camera_id_;
        double progress_;
        Eigen::Matrix3f new_K_;


        cv::Rect roi_;
        // std::shared_ptr<CameraModle> cam_;
        std::shared_ptr<ov_core::CamBase> cam_;
        std::shared_ptr<std::thread> thr_;
        std::shared_ptr<cv::Mat> data_;
        bool save_raw_img_;
        bool save_fea_img_;
        TargetInfo target_;
        float valid_ratio_;
        cv::SimpleBlobDetector::Params params_;
        float attention_width_scale_, attention_height_scale_;
        double quality_level_;
        double min_distance_;
        int window_size_;

        SORT::KMDispatch km_;
        int optimize_time_;
        double huber_thre_;
        Sophus::SO3d R_camera_body_;

    };
}