#include "window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"

#include "test.hpp"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "MyModel/Aruco.h"
#include "MyModel/Calib.h"

// #include <opencv2/imgcodecs/imgcodecs_c.h>
#include <atomic>
#include <chrono>

std::atomic<bool> shouldRun(true);

std::shared_ptr<OB_SYSTEM> detector = nullptr;
std::shared_ptr<std::thread> worker_ = nullptr;
std::deque<std::pair<uint64_t, cv::Mat>> img__que;
std::mutex img__mtx;

// int main(int argc, char **argv) try {
int ob_driver() try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
        auto profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
        try {
            // Find the corresponding Profile according to the specified format, and choose the RGB888 format first
            colorProfile = profiles->getVideoStreamProfile(1280, OB_HEIGHT_ANY, OB_FORMAT_MJPG, 30);
        }
        catch(ob::Error &e) {
            // If the specified format is not found, select the first one (default stream profile)
            colorProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
        }
        config->enableStream(colorProfile);
    }
    catch(ob::Error &e) {
        std::cerr << "Current device is not support color sensor!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Start the pipeline with config
    pipe.start(config);

    // Create a window for rendering and set the resolution of the window
    // Window app("ColorViewer", colorProfile->width(), colorProfile->height());

    while(shouldRun) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }
        auto frame = frameSet->colorFrame();
        cv::Mat rstMat;
        if(frame->type() == OB_FRAME_COLOR) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            // std::cout << " ================ " << frame->type() << " == " << videoFrame->format() << std::endl;
            switch(videoFrame->format()) {
            case OB_FORMAT_MJPG: {
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
            } break;
            case OB_FORMAT_NV21: {
                cv::Mat rawMat(videoFrame->height() * 3 / 2, videoFrame->width(), CV_8UC1, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
            } break;
            case OB_FORMAT_YUYV:
            case OB_FORMAT_YUY2: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
            } break;
            case OB_FORMAT_RGB: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC3, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
            } break;
            case OB_FORMAT_UYVY: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
            } break;
            default:
                break;
            }
        }
        {
            std::unique_lock<std::mutex> lck(img__mtx);  
            img__que.push_back(std::make_pair(frame->timeStamp(), rstMat));
            while(img__que.size() > 5)
            {  
                img__que.pop_front();
            }
        }        
        // cv::imshow("12346", rstMat);
        // cv::waitKey(2);
        // app.addToRender(frameSet->colorFrame());

    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

void signal_callback_handler(int signum) {

#ifdef PC_VISUALIZATION
    cv::destroyAllWindows();
#endif

    // 设置标志，请求线程停止
    shouldRun = false;
    // 等待线程结束
    worker_->join();
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    detector.reset();
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    PRINT_INFO(" Ctr + C Interrupt, Quit perfectly!!");
    exit(signum);

}

int main(int argc, char **argv)
{

    std::string config_path = "../config/dcw3_15/estimator_config.yaml";
    double n = 10;
    if (argc > 1) {
        config_path = argv[1];
        // std::cout << " = " << config_path.c_str() << " = "<< std::endl;
        if(argv[2])
        {
            n = double(std::stod(argv[2]));
            // std::cout << " = " << config_path << " = " << n << " = " << std::endl;
        }
    
    }
    
    signal(SIGINT, signal_callback_handler); // control + c capture

    worker_ = std::make_shared<std::thread>(ob_driver);

    // parse config params
    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    std::string verbosity = "DEBUG";
    parser->parse_config("verbosity", verbosity);
    // setting the params
    ov_core::Printer::setPrintLevel(verbosity);

    auto params = std::make_shared<ov_msckf::VioManagerOptions>();
    params->print_and_load(parser);

    auto detector = std::make_shared<calibration::tAruco>(params);

    while(1)
    {
        if(!img__que.empty()){
            cv::Mat img;
            uint64_t time;
            {
                std::unique_lock<std::mutex> lck(img__mtx);  
                time = img__que.back().first;
                img = img__que.back().second.clone();

                // while(img__que.front().first <= time)
                {  
                    img__que.pop_back();
                }
            }    
        
            // if(!img.empty()) {
            //     // cv::imshow(" 123", img);
            //     // cv::waitKey(2);
            // }
            // else
            // {
            //     std::cout << " o3 : img empty " << std::endl;
            //     continue;
            // }
            // detector->rgbCallback(double(time), img);
            detector->InsertImg(img);
        }

        // 50ms
        double sleep_time = 1.0 / n; //帧间隔
        struct timespec ts;
        ts.tv_sec = (long)sleep_time;
        ts.tv_nsec = (long)((sleep_time - (long)sleep_time) * 1e9);
        //   std::cout << "ts.tv_nsec: " << ts.tv_nsec << std::endl;
        nanosleep(&ts, 0);

    }

    // cv::Rect img_roi = cv::Rect(0, 0, 1280, 800);
    // cv::Mat img1;
    // for(int i=0; i < fileNames.size(); i++)
    // {
    //     std::cout << " ====================== " << fileNames[i].name << "  ======================  "<< std::endl;

    //     img1 = imread(img_dir + fileNames[i].name, 0);

    //     detector->rgbCallback(fileNames[i].number, img1);
    //     // 40ms
    //     // usleep(n*10000);
    //     if (i != fileNames.size()-1) {
    //         double sleep_time = fileNames.at(i+1).number - fileNames.at(i).number;
    //         // sleep_time = sleep_time / ((OB_SYSTEM *)fusion_system)->getBagSpeed();
    //         sleep_time = sleep_time / 1e5;

    //         struct timespec ts;
    //         ts.tv_sec = (long)sleep_time;
    //         ts.tv_nsec = (long)((sleep_time - (long)sleep_time) * 1e9);
    //         nanosleep(&ts, 0);
    //     }
    // }

    std::cout << " END +++++++ END " << std::endl;
    return 0;
}