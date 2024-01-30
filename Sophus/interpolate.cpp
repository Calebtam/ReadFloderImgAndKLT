#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/interpolate.hpp>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <mutex>
#include <deque>
// #include <uni
#include <condition_variable>
#include "colors.h"

using namespace std;
using namespace Eigen;
 
std::condition_variable cv;
std::mutex mtx;
std::deque<std::shared_ptr<Eigen::Isometry3d>, Eigen::aligned_allocator<std::shared_ptr<Eigen::Isometry3d>>> poses;

int show()
{
    // string trajectory_file = "../../examples/trajectory.txt";

    // ifstream fin(trajectory_file);
    // if (!fin)
    // {
    //     cout << "cannot find trajectory file at " << trajectory_file << endl;
    //     return EXIT_FAILURE;
    // }

    // 初始化视窗
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    double time, tx, ty, tz, qx, qy, qz, qw;
    vector<Eigen::Vector3d> traj;
    // vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    int i = 0;
    bool loop = true;
    while (loop)
    {
        // ============== 常规操作 ===============
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
/*
    .rotation():无参数，返回只读的当前变换的旋转部分，以旋转矩阵表示
    .translation():无参数，返回当前变换平移部分的向量表示(可修改)，可以索引[]获取各分量
    .translationExt():无参数，如果当前变换是仿射的话，返回平移部分(可修改)；如果是射影变换的话，返回最后一列(可修改)
    .affine():无参数，返回当前变换的仿射部分(可修改)，对于Isometry3d而言，返回的是一个4×3的矩阵
    
    .rotate():有参数，用于设置当前变换的旋转部分，输入参数可以为角轴、四元数、旋转矩阵等。A.rotate(B)等价于A×B
    .translate():有参数，用于在当前变换上应用右乘，A.translate(B)等价于A×B
    .scale():有参数，用于在当前变换上应用右乘，A.scale(B)等价于A×B

    .prerotate():有参数，用于设置当前变换的旋转部分，输入参数可以为角轴、四元数、旋转矩阵等。A.prerotate(B)等价于B×A
    .pretranslate():有参数，用于在当前变换上应用左乘，A.pretranslate(B)等价于B×A
    .prescale():有参数，用于在当前变换上应用左乘，A.prescale(B)等价于B×A

    .matrix():返回变换对应的矩阵(可修改)
    .cols():变换对应矩阵的列数
    .rows():变换对应矩阵的行数
    .linear()&.linearExt():返回变换的线性部分，对于Isometry而言就是旋转对应的旋转矩阵，Eigen::Block类型
    .inverse():返回当前变换的逆变换
    .Identity():返回一个单位变换，一般可用于Isometry的初始化中，将变换设为一个单位阵：Isometry3d::Identity()
    .setIdentity():将当前变换变成单位变换，变换对应的矩阵变成单位阵，也可用于初始化赋值
    .cast():将当前类型的变换转换为其它类型的变换，如Isometry3d转Isometry3f
    .data():返回一个指向变换内部矩阵的指针，矩阵按照列优先存储
*/      

        Eigen::Isometry3d Twr = Eigen::Isometry3d::Identity();;
        {
          // std::lock_guard<std::mutex> lck(mtx);  
          if(mtx.try_lock())
          {
            std::cout << BLUE << " show " << RESET << std::endl;
            if (!poses.empty()){
              Twr = *poses.front();
              poses.pop_front();
              i=0;
            }
            else
            {
              i++;
              usleep(10000);
              if(i==1000)
              {
                loop = false;
                std::cout << RED << " 挂掉了 " << RESET << std::endl;
              }
              mtx.unlock();
              continue;
            }
            mtx.unlock();
          }
          else
          {
            usleep(10000);
            continue;
          }
        }
        // poses.push_back(Twr);

        // 绘制坐标系
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0.3, 0, 0);
        glColor3f(0.f, 1.0f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0.3, 0);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 0.3);
        glEnd();

        // ------------------- 绘制随位姿变换的相机模型 ----------
        glPushMatrix();

        glMultMatrixd(Twr.data());

        // 绘制相机轮廓线
        const float w = 0.2;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();

        // // -------- 绘制相机轨迹 --------//
        // glLineWidth(2);
        // glBegin(GL_LINES);

        // glColor3f(1.0f, 0.0f, 0.0f);
        // for (size_t i = 0; i < poses.size() - 1; i++)
        // {
        //     auto p1 = poses[i], p2 = poses[i + 1];
        //     /* 这边说明一下:
        //       p1 是 4x4的齐次矩阵,比如这个是  poses[2].matrix() 打印出来的
        //         0.997863 -0.00369598   0.0652398    0.013979
        //         0.00932699    0.996232  -0.0862205  -0.0130823
        //         -0.0646754   0.0866447    0.994138  -0.0108696
        //         0           0           0           1

        //         而下面用到的poses[i].translation() 表示从4x4的欧式变换矩阵中提取出 平移向量
        //         同理,poses[i].rotation() 表示从4x4的欧式变换矩阵中提取出  旋转向量

        //     */
        //     glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        //     glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        // }

        // // if (i++ == 2)
        // // {
        // //     std::cout << poses[2].matrix() << std::endl;
        // //     std::cout << std::endl
        // //               << poses[2].translation().transpose() << std::endl;
        // //     std::cout << std::endl
        // //               << poses[2].rotation() << std::endl;
        // // }

        // glEnd();

        pangolin::FinishFrame();

        if (pangolin::ShouldQuit())
            break;

        usleep(20000);
    }
}




int main(int argc, char **argv) {
 
  std::shared_ptr<std::thread> thread_new = std::make_shared<std::thread>(show);

  // 定义起点和终点位姿
  Vector3d t1(1, 1, 1);
  Quaterniond q1(AngleAxisd(M_PI / 4, Vector3d(0, 0, 1)));
  Sophus::SE3d pose1(q1, t1);
 
  Vector3d t2(-1, -1, -1);
  Quaterniond q2(AngleAxisd(M_PI / 2, Vector3d(1, 0, 0)));
  Sophus::SE3d pose2(q2, t2);

  while (1)
  {
    // 演示位姿插值
    for (double t = 0; t <= 1; t += 0.1) {
      Sophus::SE3d interp_pose = Sophus::interpolate(pose1, pose2, t);
      // cout << "interpolated pose at t=" << t << ":\n" << interp_pose.matrix() << endl;
      {
          std::lock_guard<std::mutex> lck(mtx);
          std::shared_ptr<Eigen::Isometry3d> Twr(new Eigen::Isometry3d(interp_pose.matrix()));
          poses.push_back(Twr);
          std::cout << GREEN << " push1 " << RESET << std::endl;
      }
      usleep(50000);
    }

    // 演示位姿插值
    for (double t = 0; t <= 1; t += 0.1) {
      Sophus::SE3d interp_pose = Sophus::interpolate(pose2, pose1, t);
      // cout << "interpolated pose at t=" << t << ":\n" << interp_pose.matrix() << endl;
      {
          std::lock_guard<std::mutex> lck(mtx);
          std::shared_ptr<Eigen::Isometry3d> Twr(new Eigen::Isometry3d(interp_pose.matrix()));
          poses.push_back(Twr);
          std::cout << GREEN << " push2 " << RESET << std::endl;
      }
      usleep(50000);
    }
  // // 演示路径生成
  // vector<Sophus::SE3d> path;
  // for (double x = 0; x <= 1; x += 0.1) {
  //   for (double y = 0; y <= 1; y += 0.1) {
  //     Vector3d t(x, y, 0);
  //     Quaterniond q(AngleAxisd(M_PI / 4, Vector3d(0, 0, 1)));
  //     Sophus::SE3d pose(q, t);
  //     path.push_back(pose);
  //   }
  // }
 
  // // 输出路径中的位姿
  // for (size_t i = 0; i < path.size(); ++i) {
  //   cout << "pose " << i << ":\n" << path[i].matrix() << endl;
  // }

    cout << "pose " << endl;
    usleep(3000);
  }
  
  return 0;
}