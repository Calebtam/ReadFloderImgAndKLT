#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <unistd.h>

using namespace std;

int main(int argc, char const *argv[])
{

    string trajectory_file = "../../examples/trajectory.txt";

    ifstream fin(trajectory_file);
    if (!fin)
    {
        cout << "cannot find trajectory file at " << trajectory_file << endl;
        return EXIT_FAILURE;
    }

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
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    int i = 0;
    while (!fin.eof())
    {
        // ============== 常规操作 ===============
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);

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

        // -------- 绘制相机轨迹 --------//
        glLineWidth(2);
        glBegin(GL_LINES);

        glColor3f(1.0f, 0.0f, 0.0f);
        for (size_t i = 0; i < poses.size() - 1; i++)
        {
            auto p1 = poses[i], p2 = poses[i + 1];
            /* 这边说明一下:
              p1 是 4x4的齐次矩阵,比如这个是  poses[2].matrix() 打印出来的
                0.997863 -0.00369598   0.0652398    0.013979
                0.00932699    0.996232  -0.0862205  -0.0130823
                -0.0646754   0.0866447    0.994138  -0.0108696
                0           0           0           1

                而下面用到的poses[i].translation() 表示从4x4的欧式变换矩阵中提取出 平移向量
                同理,poses[i].rotation() 表示从4x4的欧式变换矩阵中提取出  旋转向量

            */
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        }

        if (i++ == 2)
        {
            std::cout << poses[2].matrix() << std::endl;
            std::cout << std::endl
                      << poses[2].translation().transpose() << std::endl;
            std::cout << std::endl
                      << poses[2].rotation() << std::endl;
        }

        glEnd();

        pangolin::FinishFrame();

        if (pangolin::ShouldQuit())
            break;

        usleep(30000);
    }

    return 0;
}

