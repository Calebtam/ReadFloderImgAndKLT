#include "MyModel/Aruco.h"

#include "g2o/core/block_solver.h"
#include <g2o/core/sparse_optimizer.h>                      
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
// #include "aruco/Calib.h"
#include <opencv2/core/eigen.hpp>
// #include "aruco/Calib.h"

// #include "opencv2/core/eigen.hpp"

namespace calibration
{
    tAruco::tAruco(std::shared_ptr<ov_msckf::VioManagerOptions> &params)
    {
        cam_ = params->camera_intrinsics[0];
        new_K_ << cam_->get_K()(0, 0), cam_->get_K()(0, 1), cam_->get_K()(0, 2),
                    cam_->get_K()(1, 0), cam_->get_K()(1, 1), cam_->get_K()(1, 2),
                    cam_->get_K()(2, 0), cam_->get_K()(2, 1), cam_->get_K()(2, 2);

        input_flag_ = true;

        target_.method = TargetInfo::METHOD::CORNER;

        target_.roi = cv::Rect(cv::Point(0, 0), cv::Point(1280, 680));

        target_.R_prior = Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.0, 0.0));
        // std::cout << "" << std::endl << target_.R_prior.matrix() << std::endl;

        {
            target_.template_points.emplace_back(-0.199, -0.044);
            target_.template_points.emplace_back(-0.155, -0.044);
            target_.template_points.emplace_back(-0.111, -0.044);
            target_.template_points.emplace_back(-0.067, -0.044);
            target_.template_points.emplace_back( 0.111, -0.044);
            target_.template_points.emplace_back( 0.155, -0.044);

            target_.template_points.emplace_back(-0.199, 0.0);
            target_.template_points.emplace_back(-0.155, 0.0);
            target_.template_points.emplace_back(-0.111, 0.0);
            target_.template_points.emplace_back(-0.067, 0.0);
            target_.template_points.emplace_back( 0.067, 0.0);
            target_.template_points.emplace_back( 0.111, 0.0);
            target_.template_points.emplace_back( 0.155, 0.0);
            target_.template_points.emplace_back( 0.199, 0.0);

            target_.template_points.emplace_back(-0.155, 0.044);
            target_.template_points.emplace_back(-0.111, 0.044);
            target_.template_points.emplace_back( 0.067, 0.044);
            target_.template_points.emplace_back( 0.111, 0.044);
            target_.template_points.emplace_back( 0.155, 0.044);
            target_.template_points.emplace_back( 0.199, 0.044);
        }
        // 可供匹配的点个数
        valid_ratio_ = 0.8;

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
            params_.minArea = 50;    //斑点的最小面积
            params_.maxArea = 10000;    //斑点的最大面积
            params_.filterByCircularity = false;    //斑点圆度的限制变量，默认是不限制
            params_.minCircularity = M_PI/4.0;    //斑点的最小圆度      圆的为圆度为1，正方形的圆度为PI/4
            //斑点的最大圆度，所能表示的float类型的最大值
            params_.maxCircularity = std::numeric_limits<float>::max();
            params_.filterByInertia = true;    //斑点惯性率的限制变量
            params_.minInertiaRatio = 0.8f;    //斑点的最小惯性率
            params_.maxInertiaRatio = std::numeric_limits<float>::max();    //斑点的最大惯性率
            params_.filterByConvexity = true;    //斑点凸度的限制变量
            params_.minConvexity = 0.95f;    //斑点的最小凸度
            params_.maxConvexity = std::numeric_limits<float>::max();    //斑点的最大凸度
        }
        optimize_time_ = 2;
        huber_thre_ = 0.1;

        flag_run_ = true;
        sta_mutex_.lock();
        sta_ = STATE::INITIALIZING;
        progress_ = 0;
        sta_mutex_.unlock();

        thr_ = std::make_shared<std::thread>(&tAruco::Run, this);
    }

    tAruco::~tAruco()
    {
        flag_run_ = false;
        if (thr_ != NULL)
        {
            if (thr_->joinable())
            {
                thr_->join();
                thr_ = NULL;
            }
        }
    }

    void tAruco::Run()
    {

        std::cout << "  123 = " <<  target_.detect_points.size() << "    " << std::endl;

        // data_ = NULL;
        // sta_mutex_.lock();
        // sta_ = STATE::DATA_COLLECTING;
        // sta_mutex_.unlock();
        CollectData();
        
        // std::cout << "  ====================================================== 234= " << std::endl;
        // sta_mutex_.lock();
        // sta_ = STATE::CALCULATING;
        // sta_mutex_.unlock();
        // Optimize();

        cv::waitKey(100);
        // sta_mutex_.lock();
        // sta_ = STATE::FINISHED;
        // sta_mutex_.unlock();
    }

    void tAruco::CollectData()
    {
        while (flag_run_)
        {
            if (img_deque_.empty())
            {
                // input_flag_ = true;
                usleep(1000);
                continue;
            }
            {
                // input_flag_ = false;
                std::lock_guard<std::mutex> lock(img_mutex_);
                data_ = img_deque_.back();
                img_deque_.pop_back();
            }
            {
                cv::Mat temp_img = data_->clone();
                cv::imshow("source ", temp_img);
            }
            auto rT1 = boost::posix_time::microsec_clock::local_time();

            Detection();
            auto rT2 = boost::posix_time::microsec_clock::local_time();

            Optimize();
            auto rT3 = boost::posix_time::microsec_clock::local_time();
            printf("[model]: detect: %.3f ms      optimize  %.3f ms\n", (rT2 - rT1).total_microseconds() * 1e-3, (rT3 - rT2).total_microseconds() * 1e-3);

            // input_flag_ = true;
            cv::waitKey(10);
            // usleep(10000);
            // flag_run_ = true;
        }
    }

    void tAruco::Detection()
    {
        int show_ = 1;
        if (data_->empty())
        {
            return;
        }
        target_.detect_points.clear();

        cv::Mat img = data_->clone();
        //使用灰度图像进行角点检测
        cv::Mat img1;
        cv::cvtColor(img, img1, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> keypoints;
        auto detector1 = cv::SimpleBlobDetector::create(params_);
        
        detector1->detect(img1, keypoints);
        // std::cout << " keypoints = " << keypoints.size() << std::endl;

        if(keypoints.size() < 4) return;

        int max_x = 0, max_y = 0, min_x = 2000, min_y = 2000;
        if (show_)
        {
            cv::Mat temp_img = data_->clone();
            for (auto &kpt : keypoints)
            {
                cv::circle(temp_img, kpt.pt, 3, cv::Scalar(0, 255, 0), 2);
                if(kpt.pt.x > max_x)    max_x = kpt.pt.x;
                if(kpt.pt.x < min_x)    min_x = kpt.pt.x;

                if(kpt.pt.y > max_y)    max_y = kpt.pt.y;
                if(kpt.pt.y < min_y)    min_y = kpt.pt.y;
            }
            cv::imshow("camera _feapg", temp_img);
        }
  
        std::vector<cv::Point2f> corner_pts(0);

        //设置角点检测参数
        std::vector<cv::Point2f> corners;
        int max_corners = 20;           // 最大角点数目
        double quality_level = 0.01;    // 质量水平系数（小于1.0的正数，一般在0.01-0.1之间）
        double min_distance = 3.0;      // 最小距离，小于此距离的点忽略
        int block_size = 3;             // 使用的邻域数
        bool use_harris = false;        // false ='Shi Tomasi metric'
        double k = 0.04;                // Harris角点检测时使用

        // std::cout << " " << min_x << " "<< min_y << " " << max_x << " "<< max_y << " " << roi_ << " " << img1.cols << " " << img1.rows<< std::endl;

        if(min_x - 25 < 0)  min_x = 25;
        if(min_y - 25 < 0)  min_y = 25;
        if(max_x + 25 > img.cols)  max_x = img.cols-26; //横轴 对应cols
        if(max_y + 25 > img.rows)  max_y = img.rows-26;
        
        cv::Point2f offset(min_x-25, min_y-25);
        cv::Point2f offset2(max_x+25, max_y+25);
        roi_ = cv::Rect(offset.x, offset.y, offset2.x-offset.x, offset2.y-offset.y);
        // roi_ = cv::Rect(offset.y, offset.x, offset2.y-offset.y, offset2.x-offset.x);
        // std::cout << " " << min_x << " "<< min_y << " " << max_x << " "<< max_y << " " << roi_ << " " << img1.cols << " " << img1.rows<< std::endl;

        // cv::imshow("source 2", img1);
        // cv::waitKey(0);
        //角点检测
        cv::goodFeaturesToTrack(img1(roi_), 
                                corners, 
                                max_corners, 
                                quality_level, 
                                min_distance, 
                                cv::Mat(), 
                                block_size, 
                                use_harris, 
                                k);

        // cv::imshow("source 3", img1);
        // cv::waitKey(0);


        std::cout << " corner_pts = " << corners.size() << std::endl;

        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
        cv::cornerSubPix(img1(roi_), corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        // pts.clear();
        if(show_)
        {
            cv::Mat img2 = data_->clone();
            //将检测到的角点绘制到原图上
            std::vector<cv::Point2f> &detect_points = target_.detect_points;
        
            for (int i = 0; i < corners.size(); i++)
            {
                cv::circle(img2, corners[i]+offset, 1, cv::Scalar(0, 0, 255), 2, 8, 0);
                detect_points.push_back(cam_->undistort_cv(corners[i]+offset));

            }
            cv::rectangle(img2, offset, offset2, cv::Scalar(0, 255, 255), 2, 8);

            cv::imshow("camera _feapg3123", img2);
            // cv::waitKey(2);
        }
        // cv::imshow("source 4", img1);
        // cv::waitKey(0);

    }

    void tAruco::Optimize()
    {
        if (target_.detect_points.size() < valid_ratio_ * target_.template_points.size())
        {
            std::cout << "  " <<  target_.detect_points.size() << "    " << valid_ratio_ * target_.template_points.size() << std::endl;
            return;
        }

        // g2o::OptimizationAlgorithmLevenberg *solver;

        /*************** 第1步：创建一个线性求解器LinearSolver*************************/
        using LinearSolver = g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>;
        // solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<LinearSolver>()));
        std::unique_ptr<LinearSolver> linearSolver(new LinearSolver);
        // LinearSolverCholmod ：使用sparse cholesky分解法，继承自LinearSolverCCS。
        // LinearSolverCSparse：使用CSparse法，继承自LinearSolverCCS。
        // LinearSolverPCG ：使用preconditioned conjugate gradient 法，继承自LinearSolver。
        // LinearSolverDense ：使用dense cholesky分解法，继承自LinearSolver。
        // LinearSolverEigen： 依赖项只有eigen，使用eigen中sparse Cholesky 求解，因此编译好后可以方便的在其他地方使用，性能和CSparse差不多，继承自LinearSolver。

        /*************** 第2步：创建BlockSolver。并用上面定义的线性求解器初始化**********/
        std::unique_ptr<g2o::BlockSolverX> blockSolver(new g2o::BlockSolverX(std::move(linearSolver)));
        // BlockSolver有两种定义方式：
        // // 固定变量的solver。 p代表pose的维度（是流形manifold下的最小表示），l表示landmark的维度
        // using BlockSolverPL = BlockSolver< BlockSolverTraits<p, l> >;
        // // 可变尺寸的solver。Pose和Landmark在程序开始时并不能确定，所有参数都在中间过程中被确定。
        // using BlockSolverX = BlockSolverPL<Eigen::Dynamic, Eigen::Dynamic>;

        // 此外g2o还预定义了以下几种常用类型：
        // BlockSolver_6_3 ：表示pose 是6维，观测点是3维，用于3D SLAM中的BA。
        // BlockSolver_7_3：在BlockSolver_6_3 的基础上多了一个scale。
        // BlockSolver_3_2：表示pose 是3维，观测点是2维。


        /*************** 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化****/
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
        // 注意看程序中只使用了一行代码进行创建：右侧是初始化；左侧含有我们选择的迭代策略，在这一部分，我们有三迭代策略可以选择：
        // g2o::OptimizationAlgorithmGaussNewton
        // g2o::OptimizationAlgorithmLevenberg
        // g2o::OptimizationAlgorithmDogleg

        /*************** 第4步：创建图优化的核心：稀疏优化器（SparseOptimizer）**********/
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);     // 设置求解器
        optimizer.setVerbose(false);         // 调试输出
        // optimizer.setMaxIterations(100); // Set maximum number of iterations
        // optimizer.setForceStopFlag(false); // Reset force stop flag
        // optimizer.setStopThreshold(1e-4); // Set the threshold for convergence
        double stopTh = 1e-4;
        // （4）往下看，SparseOptimizer包含一个优化算法部分OptimizationAlgorithm，它是通过OptimizationWithHessian 来实现的。
        // 其中迭代策略可以从Gauss-Newton（高斯牛顿法，简称GN）、 Levernberg-Marquardt（简称LM法）,、Powell's dogleg 三者中间选择一个（常用的是GN和LM）。

        /*************** 第5步：定义图的顶点和边。并添加到SparseOptimizer中**********/
        // 旋转矩阵
        auto vertex_so3 = new RotVertex();
        vertex_so3->setId(0);
        optimizer.addVertex(vertex_so3);

        auto vertex_scale = new ScaleVertex();
        vertex_scale->setId(1);
        vertex_scale->setEstimate(0.01);
        optimizer.addVertex(vertex_scale);

        // 光心与二维码坐标x轴偏移量
        auto vertex_x_offset = new ScaleVertex();
        vertex_x_offset->setId(2);
        vertex_x_offset->setEstimate(0.01);
        // 光心与二维码坐标t轴偏移量
        optimizer.addVertex(vertex_x_offset);

        auto vertex_y_offset = new ScaleVertex();
        vertex_y_offset->setId(3);
        vertex_y_offset->setEstimate(0.01);
        optimizer.addVertex(vertex_y_offset);

        std::vector<Eigen::Vector3d> cam_pts(0);
        for (auto &ele : target_.detect_points)
        {
            Eigen::Vector3f upt;
            upt(0) = ele.x;//(ele.x - new_K_(0, 2)) / new_K_(0, 0);
            upt(1) = ele.y;//(ele.y - new_K_(1, 2)) / new_K_(1, 1);
            upt(2) = 1.0;
            upt.normalize();
            cam_pts.emplace_back(upt.cast<double>());
        }
        // std::cout << "  开始优化 " <<  std::endl;

        double lastChi2 = optimizer.activeChi2();
        int nonDecreaseCount = 0;
        const int maxNonDecreaseCount = 5; // Set the maximum number of consecutive iterations with non-decreasing cost
        for (int i = 0; i < optimize_time_; ++i)
        {
            optimizer.edges().clear();
            std::vector<std::pair<int, int>> match_ret(0);
            GetMatch(target_.template_points, vertex_scale->estimate(), vertex_x_offset->estimate(),
                     vertex_y_offset->estimate(), vertex_so3->estimate(), cam_pts, match_ret);
            for (auto &ele : match_ret)
            {
                auto data_edge = new PlaneDataEdge(target_.template_points[ele.first], cam_pts[ele.second], new_K_);
                data_edge->setVertex(0, vertex_so3);
                data_edge->setVertex(1, vertex_scale);
                data_edge->setVertex(2, vertex_x_offset);
                data_edge->setVertex(3, vertex_y_offset);
                data_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(huber_thre_);
                data_edge->setRobustKernel(rk);
                optimizer.addEdge(data_edge);
            }
            optimizer.initializeOptimization();
            optimizer.optimize(5);

            // Check if the optimization is not making significant progress
            double currentChi2 = optimizer.activeChi2();
            double deltaChi2 = lastChi2 - currentChi2;
            if (deltaChi2 < stopTh) {
                nonDecreaseCount++;
            } else {
                nonDecreaseCount = 0;
            }

            // if (nonDecreaseCount >= maxNonDecreaseCount) {
            //     std::cout << "Optimization stopped: No significant decrease in cost function" << std::endl;

            //     // Restore initial estimate
            //     for (const auto& v : optimizer.activeVertices()) {
            //         // optimizer->inititialEstimate
            //         // optimizer->initialEstimate
            //         // optimizer->restoreInitialEstimate();
            //         // optimizer.setEstimate(optimizer.estima)
            //         if (optimizer->estimate().find(v->id()) != optimizer->estimate().end()) {
            //             v->setEstimate(optimizer.initialEstimate[v->id()]);
            //         }
            //     } 
            // }

            // // Check if the optimization is not making significant progress
            // double currentChi2 = optimizer.activeChi2();
            // double deltaChi2 = lastChi2 - currentChi2;
            // if (deltaChi2 < optimizer.stopThreshold()) {
            //     nonDecreaseCount++;
            // } else {
            //     nonDecreaseCount = 0;
            // }
            // if (nonDecreaseCount >= maxNonDecreaseCount) {
            //     std::cout << "Optimization stopped: No significant decrease in cost function" << std::endl;
            //     // Restore initial estimate
            //     optimizer.restoreInitialEstimate();
            //     break;
            // }
            lastChi2 = currentChi2;
        }

        /*************** 第6步：设置优化参数，开始执行优化**********/
        R_camera_body_ = vertex_so3->estimate();
        target_.init_center(2) = vertex_scale->estimate();
        target_.init_center(0) = vertex_x_offset->estimate();
        target_.init_center(1) = vertex_y_offset->estimate();
        sta_mutex_.lock();
        progress_ = 1.0;
        sta_mutex_.unlock();
                // std::cout << "  6" <<  std::endl;

        if (show)
        {
            // std::cout << "init center 1: " << target_.init_center.transpose() << std::endl;
            cv::Mat temp_img = data_->clone();// = cv::imread("camera" + std::to_string(camera_id_) + "_fea" + ".jpg");
            for (auto &pt : target_.template_points)
            {
                Eigen::Vector3d upt(pt.x(), pt.y(), 0);
                upt += target_.init_center;
                upt = R_camera_body_ * upt;
                upt /= upt(2);
                cv::Point2f tpt;
                tpt.x = upt.x() * new_K_(0, 0) + new_K_(0, 2);
                tpt.y = upt.y() * new_K_(1, 1) + new_K_(1, 2);
                if(tpt.x < 0)    tpt.x = 0;
                if(tpt.y < 0)    tpt.y = 0;
                if(tpt.x > temp_img.cols) tpt.x = temp_img.cols-10;
                if(tpt.y > temp_img.rows) tpt.y = temp_img.rows-10;
                cv::circle(temp_img, tpt, 2, cv::Scalar(255, 0, 0), -1);
                // std::cout << "tpt : " << tpt << std::endl;

            }
            // std::cout << "init center 2: " << target_.init_center.transpose() << std::endl;

            cv::imshow(" result .jpg", temp_img);
            // cv::waitKey(0);
        }
    }

    void tAruco::GetResult(Sophus::SO3d &rot)
    {
        rot = R_camera_body_ * target_.R_prior.inverse();
    }

    void tAruco::BlobDetect(cv::Mat &img, cv::Rect &roi, std::vector<cv::Point2f> &pts)
    {
        std::vector<cv::KeyPoint> keypoints;
        auto detector = cv::SimpleBlobDetector::create(params_);
        detector->detect(img(roi), keypoints);
        // std::cout << " keypoints = " << keypoints.size() << std::endl;
        pts.clear();
        for (auto &kp : keypoints)
        {
            pts.emplace_back(roi.x + kp.pt.x, roi.y + kp.pt.y);
        }
    }

    void tAruco::CornerDetect(cv::Mat &img, cv::Rect &attention_region, int max_num, std::vector<cv::Point2f> &pts)
    {
        // cv::Rect bbox;
        // if (!FindBBox(region, bbox))
        // {
        //     pts.clear();
        //     return;
        // }

        // cv::Point2f center(bbox.x + bbox.width * 0.5, bbox.y + bbox.height * 0.5);
        // float new_width = attention_width_scale_ * bbox.width;
        // float new_height = attention_height_scale_ * bbox.height;
        // cv::Rect attention_region(center.x - new_width * 0.5, center.y - new_height * 0.5, new_width, new_height);
        std::vector<cv::Point2f> corner_pts(0);
        cv::goodFeaturesToTrack(img(attention_region), corner_pts, max_num, quality_level_, min_distance_);
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
        cv::cornerSubPix(img(attention_region), corner_pts, cv::Size(window_size_, window_size_), cv::Size(-1, -1), criteria);
        pts.clear();
        for (auto &pt : corner_pts)
        {
            // pts.emplace_back(attention_region.x + pt.x, attention_region.y + pt.y);
            pts.emplace_back(pt.x, pt.y);

        }
    }

    bool tAruco::FindBBox(std::vector<cv::Point2f> &pts, cv::Rect &bbox)
    {
        if (pts.size() < 2)
        {
            return false;
        }

        float min_x = 1e6, min_y = 1e6, max_x = -1e6, max_y = -1e6;
        for (auto pt : pts)
        {
            min_x = std::min(pt.x, min_x);
            min_y = std::min(pt.y, min_y);
            max_x = std::max(pt.x, max_x);
            max_y = std::max(pt.y, max_y);
        }
        bbox = cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y));
        return true;
    }

    void tAruco::GetMatch(std::vector<Eigen::Vector2d> &template_points, double distance, double offset_x, double offset_y,
                           Sophus::SO3d rot, std::vector<Eigen::Vector3d> camera_pts, std::vector<std::pair<int, int>> &match_ret)
    {
        cv::Mat cost_matrix1 = cv::Mat_<double>(template_points.size(), camera_pts.size());
        cv::Mat cost_matrix2;

        Eigen::Vector3d offset(offset_x, offset_y, distance);
        for (int i = 0; i < template_points.size(); ++i)
        {
            Eigen::Vector3d pw(template_points[i].x(), template_points[i].y(), 0);
            pw += offset;
            pw = rot * pw;
            pw.normalize();
            for (int j = 0; j < camera_pts.size(); ++j)
            {
                cost_matrix1.at<double>(i, j) = pw.dot(camera_pts[j]);
            }
        }

        bool is_transpose = cost_matrix1.cols < cost_matrix1.rows;
        if (is_transpose)
            cv::transpose(cost_matrix1, cost_matrix2);
        else
            cost_matrix2 = cost_matrix1;

        std::vector<std::pair<int, int>> ret(0);
        km_.GetMatch(cost_matrix2, ret);
        if (is_transpose)
        {
            for (auto &ele : ret)
            {
                match_ret.emplace_back(ele.second, ele.first);
            }
        }
        else
        {
            match_ret = ret;
        }

        if(show)
        {
            // std::cout << "Match : " << match_ret.size() << std::endl;
            cv::Mat temp_img= data_->clone();// = cv::imread("camera" + std::to_string(camera_id_) + "_fea" + ".jpg");
            for (int i = 0; i < camera_pts.size(); i++)
            {
                // 从归一化平面到像素平面
                cv::Point2f tpt;
                tpt.x = camera_pts[match_ret.at(i).second].x() * new_K_(0, 0) + new_K_(0, 2);
                tpt.y = camera_pts[match_ret.at(i).second].y() * new_K_(1, 1) + new_K_(1, 2);

                cv::circle(temp_img, tpt, 2, cv::Scalar(0, 0, 100 + 155 * i / camera_pts.size()), 2);

                // 
                cv::Point2f tpt2;
                Eigen::Vector3d pw(template_points[match_ret.at(i).first].x(), template_points[match_ret.at(i).first].y(), 1);
                pw = rot * pw;
                pw.normalize();
                tpt2.x = pw.x() * new_K_(0, 0) + new_K_(0, 2);
                tpt2.y = pw.y() * new_K_(1, 1) + new_K_(1, 2);
                // std::cout << "  cam " << tpt << "   sample " << tpt2 <<std::endl; 
                cv::circle(temp_img, tpt2, 2, cv::Scalar(0, 100 + 155 * i / camera_pts.size(), 0), 2);

                cv::line(temp_img, tpt, tpt2, cv::Scalar(235, 0, 0));
            }
            cv::imshow(" 红色相机 绿色样本", temp_img);
            // cv::waitKey(0);
        }
    }

    void tAruco::InsertImg(cv::Mat &data)
    {
        // if (input_flag_)
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            std::shared_ptr<cv::Mat> img(std::make_shared<cv::Mat>(data));
            img_deque_.emplace_back(img);
            while(img_deque_.size() > 3)
                img_deque_.pop_front();
            // input_flag_ = img_deque_.size() < num_image_thr_;
        }
    }

    STATE tAruco::GetStatus()
    {
        std::lock_guard<std::mutex> lock(sta_mutex_);
        return sta_;
    }

    double tAruco::GetProgress()
    {
        std::lock_guard<std::mutex> lock(sta_mutex_);
        return progress_;
    }
}