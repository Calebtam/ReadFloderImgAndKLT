#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
// ===
#include <stdlib.h>
#include "image_types.h"
#include "workerpool.h"

#include <thread>
#include <chrono>
// #include <boost/thread/thread.hpp>

#if EVALUATION_TIME
#include "TicToc.hpp"
#endif

// #include "print.h"
#include "colors.h"

using namespace std;
using namespace cv;


image_u8_t *image_u8_create_stride(unsigned int width, unsigned int height, unsigned int stride)
{
    uint8_t *buf = (uint8_t *)calloc(height*stride, sizeof(uint8_t));

    // const initializer
    image_u8_t tmp = { .width = width, .height = height, .stride = stride, .buf = buf };

    image_u8_t *im = (image_u8_t *)calloc(1, sizeof(image_u8_t));
    memcpy(im, &tmp, sizeof(image_u8_t));
    return im;
}

struct minmax_task {
    int ty;

    image_u8_t *im;
    uint8_t *im_max;
    uint8_t *im_min;
};
struct blur_task {
    int ty;

    image_u8_t *im;
    uint8_t *im_max;
    uint8_t *im_min;
    uint8_t *im_max_tmp;
    uint8_t *im_min_tmp;
};
struct threshold_task {
    int ty;

    // apriltag_detector_t *td;
    image_u8_t *im;
    image_u8_t *threshim;
    uint8_t *im_max;
    uint8_t *im_min;
};

void do_minmax_task(void *p)
{
    const int tilesz = 4;
    struct minmax_task* task = (struct minmax_task*) p;
    int s = task->im->stride;
    int ty = task->ty;                  // 当前处理的 tile 行 （不同线程处理不同行）
    int tw = task->im->width / tilesz;  // tile 列数
    image_u8_t *im = task->im;

    // 每tile列
    for (int tx = 0; tx < tw; tx++) {
        uint8_t max = 0, min = 255;
        // tilesz x tilesz 的区域范围内的最大和最小值
        for (int dy = 0; dy < tilesz; dy++) {
            for (int dx = 0; dx < tilesz; dx++) {
                // ty对应tile行，ty*tilesz对应图像tile起始行，ty*tilesz+dy对应现在要处理的图像行，s对应图像宽度stride， 这样得到了行起始指针
                // tx对应tile列，tx*tilesz对应图像tile起始列，tx*tilesz+dx对应现在要处理的图像列，这样就找到了要获取的像素
                uint8_t v = im->buf[(ty*tilesz+dy)*s + tx*tilesz + dx];
                if (v < min)
                    min = v;
                if (v > max)
                    max = v;
            }
        }
        // 映射到 tile 索引中的值
        task->im_max[ty*tw+tx] = max;
        task->im_min[ty*tw+tx] = min;
    }
}
void do_blur_task(void *p)
{
    const int tilesz = 4;
    struct blur_task* task = (struct blur_task*) p;
    int ty = task->ty;                      // 当前处理的 tile 行 （不同线程处理不同行）
    int tw = task->im->width / tilesz;      // tile 列数
    int th = task->im->height / tilesz;     // tile 行数
    uint8_t *im_max = task->im_max;         // 每个 tile 的最大值
    uint8_t *im_min = task->im_min;         // 每个 tile 的最小值

    // 每tile列
    for (int tx = 0; tx < tw; tx++) {
        uint8_t max = 0, min = 255;

        // 在tilesz x tilesz 的图块中，找3x3邻域的最大和最小值
        for (int dy = -1; dy <= 1; dy++) {
            if (ty+dy < 0 || ty+dy >= th)
                continue;
            for (int dx = -1; dx <= 1; dx++) {
                if (tx+dx < 0 || tx+dx >= tw)
                    continue;

                // printf("ty+dy=%d, tx+dx=%d\n", tx+dy, tx+dx);

                uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                if (m > max)
                    max = m;
                    
                m = im_min[(ty+dy)*tw+tx+dx];
                if (m < min)
                    min = m;
            }
        }

        // 映射到 tile 索引中的值
        task->im_max_tmp[ty*tw + tx] = max;
        task->im_min_tmp[ty*tw + tx] = min;
    }
}
void do_threshold_task(void *p)
{
    const int tilesz = 4;
    struct threshold_task* task = (struct threshold_task*) p;
    int ty = task->ty;                                          // 当前处理的 tile 行 （不同线程处理不同行）
    int tw = task->im->width / tilesz;                          // tile 列数
    int s = task->im->stride;                                   // 图像宽度stride
    uint8_t *im_max = task->im_max;
    uint8_t *im_min = task->im_min;
    image_u8_t *im = task->im;
    image_u8_t *threshim = task->threshim;                      // 输出图像

    // int min_white_black_diff = task->td->qtp.min_white_black_diff;
    int min_white_black_diff = 70;

    // 每tile列
    for (int tx = 0; tx < tw; tx++) {
        int min = im_min[ty*tw + tx];
        int max = im_max[ty*tw + tx];

        // low contrast region? (no edges)
        if (max - min < min_white_black_diff) {
            for (int dy = 0; dy < tilesz; dy++) {
                int y = ty*tilesz + dy;

                for (int dx = 0; dx < tilesz; dx++) {
                    int x = tx*tilesz + dx;

                    threshim->buf[y*s+x] = 127;
                }
            }
            continue;
        }

        // otherwise, actually threshold this tile.

        // argument for biasing towards dark; specular highlights
        // can be substantially brighter than white tag parts
        uint8_t thresh = min + (max - min) / 2;

        for (int dy = 0; dy < tilesz; dy++) {
            int y = ty*tilesz + dy;

            for (int dx = 0; dx < tilesz; dx++) {
                int x = tx*tilesz + dx;

                uint8_t v = im->buf[y*s+x];
                if (v > thresh)
                    threshim->buf[y*s+x] = 255;
                else
                    threshim->buf[y*s+x] = 0;
            }
        }
    }
}

struct grad_task {
    int ty;
    int thread;
    
    image_u8_t *im;      // 输入图像（灰度）
    float *theta;         // 输出角度 (rad)
    uint16_t *mag;           // 输出梯度幅值 (Ix^2 + Iy^2)
};

void do_grad_task(void *p) {
    grad_task *task = (grad_task*) p;
    image_u8_t *im = task->im;
    int rows = im->height;
    int cols = im->width;
    int stride = im->stride;

    float *theta = task->theta;
    uint16_t *mag   = task->mag;

    int ty = task->ty;
    int thread = task->thread;
    int th;
    if(ty == thread - 1)
        th = im->height - (rows / thread) * ty;
    else
        th = rows / thread;

    int row_start = (rows / thread) * ty;
    int row_end = row_start + th - 1;

    // printf("ty: %d, thread: %d, th: %d, row_start: %d, row_end: %d\n", ty, thread, th, row_start, row_end);

    // for (int y = 0; y < img.rows; y++) {
    //     for (int x = 0; x < img.cols; x++) {}}


    // 处理 [row_start, row_end)
    for (int i = 1; i < rows-1; i++) {
        if (i < row_start || i > row_end) continue;  // 边界跳过
        if(i == 1 || i == 49 || i == 99 || i == 149 || i == 199 || i > 199 || i < 1)
            std::cout << " " << i << " ";  // image size: 401x201 199
        // std::this_thread::sleep_for(std::chrono::milliseconds(15));

        for (int j = 1; j < cols-1; j++) {
            int idx = i*stride + j;
            // std::cout << " " << j << std::endl;  // image size: 401x201 399
            // std::cout << " " << i << "-" << j << std::endl; // image size: 401x201 [199 - 399]

            float Ix = (float)im->buf[i*stride + (j+1)] - (float)im->buf[i*stride + (j-1)];
            float Iy = (float)im->buf[(i+1)*stride + j] - (float)im->buf[(i-1)*stride + j];
            // std::cout << "idx: " << idx << " i: " << i << " j: " << j << " Ix: " << Ix << " Iy: " << Iy << std::endl;

            theta[idx] = std::atan2(Iy, Ix);
            mag[idx]   = Ix*Ix + Iy*Iy;
            // if(mag[idx] != Ix*Ix + Iy*Iy)
            //     std::cout << "mag[idx]: " << mag[idx] << " Ix*Ix + Iy*Iy: " << Ix*Ix + Iy*Iy << " Ix: " << Ix << " Iy: " << Iy << std::endl;
        }
    }
    std::cout << std::endl;
}


int main(int argc, char **argv)
{
    if (argc != 2){
        cout << "Plese intput: [exe] [img_path]" << endl;
        return -1;
    }
    std::cout << "\n\n\t\tTEST START!!!" << std::endl;

    int td_nthreads = 10; // thread num
    workerpool_t *td_wp = workerpool_create(td_nthreads);
    int get_thread = workerpool_get_nthreads(td_wp);
    if (td_wp == NULL || td_nthreads != workerpool_get_nthreads(td_wp)) {
        workerpool_destroy(td_wp);
        td_wp = workerpool_create(td_nthreads);
        if (td_wp == NULL) {
            // creating workerpool failed - return empty zarray
            // return zarray_create(sizeof(apriltag_detection_t*));
            return 0;
        }
    }
    std::cout << "\n\n\t\t===== Create Workerpool =====\n" << std::endl;

    for(int i = 0; i < 1; i++){
        printf("\n\n\t\t===== Frame %d =====\n", i);

        {
            cv::Mat img = cv::imread(argv[1], -1);
            // cv::imshow("Image Show all", img);
            // cv::waitKey(0);

            uint64_t time = 0 + 1.0 * i;

            cv::Rect rect(500, 300, 401, 204);
            cv::Mat roi_img = img(rect);

            cv::imshow("Image Show roi", roi_img);
            // cv::waitKey(0);

            std::cout << "Type: " << cv::typeToString(roi_img.type()) << std::endl;
            if(roi_img.type() != CV_8UC1){
                cv::cvtColor(roi_img, roi_img, cv::COLOR_BGR2GRAY);
            }



            uint8_t *aligned_data = nullptr;
            cv::Mat img_aligned;
            if (roi_img.isContinuous() &&
                (reinterpret_cast<uintptr_t>(roi_img.data) % 64 == 0)) {
                // PRINT_INFO("roi_img.isContinuous\n");
                img_aligned = roi_img;
            } else {
                if (posix_memalign((void **)&aligned_data, 64,
                                roi_img.cols * roi_img.rows) != 0) {
                // PRINT_INFO("posix_memalign failed");
                return 0;
                }
                img_aligned = cv::Mat(roi_img.rows, roi_img.cols, CV_8UC1, aligned_data);
                roi_img.copyTo(img_aligned);
            }

            // ============== 下面是apriltag1的操作 ==============
            {
                cv::Mat filteredSeg; 
                img_aligned.convertTo(filteredSeg, CV_32FC1, 1.0/255.0 );
                cv::Mat filteredTheta = cv::Mat( filteredSeg.size(), CV_32FC1 );
                cv::Mat filteredMag = cv::Mat( filteredSeg.size(), CV_32FC1 );

                auto start = std::chrono::high_resolution_clock::now();
                for( int i = 1; i < filteredSeg.rows-1; i++ )
                {
                    for( int j = 1; j < filteredSeg.cols-1; j++ )
                    {
                        float Ix = filteredSeg.at<float>(i,j+1) - filteredSeg.at<float>(i,j-1);
                        float Iy = filteredSeg.at<float>(i+1,j) - filteredSeg.at<float>(i-1,j);
                        filteredTheta.at<float>(i,j) = std::atan2(Iy, Ix);
                        filteredMag.at<float>(i,j) = Ix*Ix + Iy*Iy;
                    }	
                }
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::micro> elapsed = end - start;
                std::cout << "Single thread Grad time: " << elapsed.count() << " us " << std::endl;

                cv::imshow("filteredMag", filteredMag);
                cv::imshow("filteredTheta", filteredTheta);
            }

            // ============== 下面是apriltag3的多线程操作 ==============

            image_u8_t img_ = {
                .width = img_aligned.cols,
                .height = img_aligned.rows,
                .stride = img_aligned.cols,
                .buf = img_aligned.data};
            auto im = &img_;

            int w = img_.width, h = img_.height, s = img_.stride;
            image_u8_t *threshim = image_u8_create_stride(w, h, s);
            // zarray_t *detections = apriltag_detector_detect(td, &im);
            
            // tile（小方块）
            const int tilesz = 4;
            // the last (possibly partial) tiles along each row and column will
            // just use the min/max value from the last full tile.
            int tw = w / tilesz;
            int th = h / tilesz;                                                // 200/4 = 50
            uint8_t *im_max = (uint8_t*) calloc(tw*th, sizeof(uint8_t));        // 100*50
            uint8_t *im_min = (uint8_t*) calloc(tw*th, sizeof(uint8_t));        // 100*50
            
            printf("image size: %dx%d stride: %d\n", w, h, s);   // 400*200
            printf("tiles size: %dx%d\n", tw, th);               // 100*50
            printf("tile size: %d\n", tilesz);                   // 4
            
            // first, compute min/max for each tile
            if(0)
            {
                struct minmax_task *minmax_tasks = (struct minmax_task*) malloc(sizeof(struct minmax_task)*th);

                // first, collect min/max statistics for each tile
                for (int ty = 0; ty < th; ty++) {
                    minmax_tasks[ty].im = im;
                    minmax_tasks[ty].im_max = im_max;
                    minmax_tasks[ty].im_min = im_min;
                    minmax_tasks[ty].ty = ty;

                    workerpool_add_task(td_wp, do_minmax_task, &minmax_tasks[ty]);
                }
                workerpool_run(td_wp);
                free(minmax_tasks);
            }

            // next, blur the min/max values
            if(0)
            {
                uint8_t *im_max_tmp = (uint8_t*) calloc(tw*th, sizeof(uint8_t));
                uint8_t *im_min_tmp = (uint8_t*) calloc(tw*th, sizeof(uint8_t));
        
                struct blur_task *blur_tasks = (struct blur_task*) malloc(sizeof(struct blur_task)*th);
                for (int ty = 0; ty < th; ty++) {
                    blur_tasks[ty].im = im;
                    blur_tasks[ty].im_max = im_max;
                    blur_tasks[ty].im_min = im_min;
                    blur_tasks[ty].im_max_tmp = im_max_tmp;
                    blur_tasks[ty].im_min_tmp = im_min_tmp;
                    blur_tasks[ty].ty = ty;
        
                    workerpool_add_task(td_wp, do_blur_task, &blur_tasks[ty]);
                }
                workerpool_run(td_wp);
                free(blur_tasks);
                free(im_max);
                free(im_min);
                im_max = im_max_tmp;
                im_min = im_min_tmp;
            }

            if(1)
            {
                // uint8_t *seg = (uint8_t*) calloc(w*h, sizeof(uint8_t));
                float *theta = (float*) calloc(w*h, sizeof(float));
                uint16_t *mag = (uint16_t*) calloc(w*h, sizeof(uint16_t));
                
                struct grad_task *grad_tasks = (struct grad_task*) malloc(sizeof(struct grad_task)*th);
                for (int ty = 0; ty < td_nthreads; ty++) 
                // int ty = 0;
                {
                    grad_tasks[ty].im = im;
                    grad_tasks[ty].theta = theta;
                    grad_tasks[ty].mag = mag;
                    grad_tasks[ty].ty = ty;
                    grad_tasks[ty].thread = td_nthreads;
            
                    workerpool_add_task(td_wp, do_grad_task, &grad_tasks[ty]);
                }
                std::cout << "\n\n\t\t===== Run Grad Task =====\n" << std::endl;
                auto start = std::chrono::high_resolution_clock::now();
                workerpool_run(td_wp);
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::micro> elapsed = end - start;
                std::cout << "Multi thread Grad time: " << elapsed.count() << " us " << std::endl;
                
                free(grad_tasks);

                // ======== 显示还原结果 ========
                cv::Mat mag_img(h, w, CV_16UC1, mag);
                cv::Mat theta_img(h, w, CV_32FC1, theta);

                double minVal, maxVal;
                cv::minMaxLoc(mag_img, &minVal, &maxVal);
                cv::Mat mag_norm;
                mag_img.convertTo(mag_norm, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

                // cv::Mat theta_norm;
                // theta_img.convertTo(theta_norm, CV_8UC1, 255.0/(2*M_PI), 255.0/2.0);
                // cv::applyColorMap(theta_norm, theta_norm, cv::COLORMAP_HSV);

                cv::imshow("mag", mag_img);
                cv::imshow("theta", theta_img);
                cv::imshow("mag_norm", mag_norm);
                // cv::imshow("theta_norm", theta_norm);
                cv::waitKey(0);

            }

            // we skipped over the non-full-sized tiles above. Fix those now.
            if (0) 
            {
                for (int y = 0; y < h; y++) {
                    // what is the first x coordinate we need to process in this row?
                    int x0;

                    if (y >= th*tilesz) {
                        x0 = 0; // we're at the bottom; do the whole row.
                    } else {
                        x0 = tw*tilesz; // we only need to do the right most part.
                    }

                    // compute tile coordinates and clamp.
                    int ty = y / tilesz;
                    if (ty >= th)
                        ty = th - 1;

                    for (int x = x0; x < w; x++) {
                        int tx = x / tilesz;
                        if (tx >= tw)
                            tx = tw - 1;

                        int max = im_max[ty*tw + tx];
                        int min = im_min[ty*tw + tx];
                        int thresh = min + (max - min) / 2;

                        uint8_t v = im->buf[y*s+x];
                        if (v > thresh)
                            threshim->buf[y*s+x] = 255;
                        else
                            threshim->buf[y*s+x] = 0;
                    }
                }
            }

            free(im_min);
            free(im_max);

        }

        double sleep_time = 1.0; // 帧间隔，default 0.1s

        struct timespec ts;
        ts.tv_sec = (long)sleep_time;
        ts.tv_nsec = (long)((sleep_time - (long)sleep_time) * 1e9);
        nanosleep(&ts, 0);
    }

    workerpool_destroy(td_wp);

    // cv::imshow("Image Show", image_show);
    // cv::waitKey();

    // cv::imwrite("/home/yunye/Documents/2.png", image_show);
    return 0;
}
