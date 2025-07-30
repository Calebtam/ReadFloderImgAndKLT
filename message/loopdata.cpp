#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

struct CamData {
  cv::Mat image;                            // 图像数据：mono8
  std::vector<double> intrinsic_param;      // k1 k2 k3 k4 fx fy cx cy
  std::vector<double> extrinsic_param;      // Tx Ty Tz Qx Qy Qz Qw (JPL)
  std::vector<cv::Point3f> point_3d;        // 3D点
  std::vector<cv::Point2f> point_2d_uv;     // 像素坐标
  std::vector<cv::Point2f> point_2d_normal; // 归一化坐标
  std::vector<double> point_id;             // 点ID

  // 构造函数
  CamData() = default;

  // 序列化函数（只序列化元数据，不含图像）
  std::string toString(const std::string& prefix = "") const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    auto vecToStr = [](const std::vector<double>& vec) {
      std::ostringstream oss;
      for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i + 1 < vec.size()) oss << ", ";
      }
      return oss.str();
    };

    oss << prefix << "intrinsic_param: [" << vecToStr(intrinsic_param) << "]\n";
    oss << prefix << "extrinsic_param: [" << vecToStr(extrinsic_param) << "]\n";

    oss << prefix << "point_3d: [";
    for (size_t i = 0; i < point_3d.size(); ++i) {
      const auto& p = point_3d[i];
      oss << "[" << p.x << ", " << p.y << ", " << p.z << "]";
      if (i + 1 < point_3d.size()) oss << ", ";
    }
    oss << "]\n";

    oss << prefix << "point_2d_uv: [";
    for (size_t i = 0; i < point_2d_uv.size(); ++i) {
      const auto& p = point_2d_uv[i];
      oss << "[" << p.x << ", " << p.y << "]";
      if (i + 1 < point_2d_uv.size()) oss << ", ";
    }
    oss << "]\n";

    oss << prefix << "point_2d_normal: [";
    for (size_t i = 0; i < point_2d_normal.size(); ++i) {
      const auto& p = point_2d_normal[i];
      oss << "[" << p.x << ", " << p.y << "]";
      if (i + 1 < point_2d_normal.size()) oss << ", ";
    }
    oss << "]\n";

    oss << prefix << "point_id: [" << vecToStr(point_id) << "]\n";

    return oss.str();
  }
};

struct LoopData {
  double timestamp = 0.0;                       // 时间戳，秒
  std::vector<double> extrinsic_param;          // cam0ToIMU
  std::vector<double> loop_pose;                // IMU to World
  CamData cam_front;                            // 前置相机
  CamData cam_left;                             // 左侧相机
  CamData cam_right;                            // 右侧相机

  // 构造函数
  LoopData() = default;

  // 序列化函数（不包含图像内容）
  std::string toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    auto vecToStr = [](const std::vector<double>& vec) {
      std::ostringstream oss;
      for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i + 1 < vec.size()) oss << ", ";
      }
      return oss.str();
    };

    oss << "timestamp: " << timestamp << "\n";
    oss << "extrinsic_param: [" << vecToStr(extrinsic_param) << "]\n";
    oss << "loop_pose: [" << vecToStr(loop_pose) << "]\n";

    oss << "cam_front:\n" << cam_front.toString("  ");
    oss << "cam_left:\n"  << cam_left.toString("  ");
    oss << "cam_right:\n" << cam_right.toString("  ");

    return oss.str();
  }
};


int main() {
  LoopData data;
  data.timestamp = 123456.789;
  data.extrinsic_param = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
  data.loop_pose = {1, 2, 3, 0, 0, 0, 1};

  data.cam_front.intrinsic_param = {0.01, 0.01, 0, 0, 400, 400, 320, 240};
  data.cam_front.point_id = {1, 2, 3};
  data.cam_front.point_3d = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  data.cam_front.point_2d_uv = {{100, 200}, {120, 220}, {140, 240}};
  data.cam_front.point_2d_normal = {{0.1, 0.2}, {0.2, 0.3}, {0.3, 0.4}};

  data.cam_left.intrinsic_param = {0.01, 0.01, 0, 0, 400, 400, 320, 240};
  data.cam_left.point_id = {4, 5, 6};
  data.cam_left.point_3d = {{4, 5, 6}, {7, 8, 9}, {10, 11, 12}};
  data.cam_left.point_2d_uv = {{300, 400}, {320, 420}, {340, 440}};
  data.cam_left.point_2d_normal = {{0.3, 0.4}, {0.4, 0.5}, {0.5, 0.6}};

  data.cam_right.intrinsic_param = {0.01, 0.01, 0, 0, 400, 400, 320, 240};
  data.cam_right.point_id = {7, 8, 9};
  data.cam_right.point_3d = {{7, 8, 9}, {10, 11, 12}, {13, 14, 15}};
  data.cam_right.point_2d_uv = {{500, 600}, {520, 620}, {540, 640}};
  data.cam_right.point_2d_normal = {{0.5, 0.6}, {0.6, 0.7}, {0.7, 0.8}};

  std::cout << data.toString() << std::endl;

  auto cam_front_str = data.cam_front.toString("  ");

  auto cam_data = data.toString();

  

  return 0;
}