#include <Eigen/Geometry>
#include <vector>
#include <limits>

class Camera {
public:
    Eigen::Vector3d UnprojectFromPixelCornerConv(const Eigen::Vector2i& pixel) const {
        // 在相机坐标系中计算像素点的三维坐标
        // 这里假设你有一个实现
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    int width_;
    int height_;
};

class CameraFrustum {
public:
    void Create(const Camera& camera, double min_depth, double max_depth, double expansion_radius) {
        CHECK_GT(min_depth, 0.0);
        CHECK_LT(max_depth, 100.0);
        CHECK_GT(expansion_radius, 0.0);
        CHECK_LT(expansion_radius, 3.0);

        std::vector<Eigen::Vector3d> base_points;

        // 计算视锥体的基础顶点
        Eigen::Vector3d top_left = camera.UnprojectFromPixelCornerConv(Eigen::Vector2i(0, 0));
        Eigen::Vector3d top_right = camera.UnprojectFromPixelCornerConv(Eigen::Vector2i(camera.width(), 0));
        Eigen::Vector3d bottom_left = camera.UnprojectFromPixelCornerConv(Eigen::Vector2i(0, camera.height()));
        Eigen::Vector3d bottom_right = camera.UnprojectFromPixelCornerConv(Eigen::Vector2i(camera.width(), camera.height()));

        // 在近端和远端的深度下计算视锥体顶点
        base_points.push_back(min_depth * top_left);
        base_points.push_back(max_depth * top_left);
        base_points.push_back(min_depth * top_right);
        base_points.push_back(max_depth * top_right);
        base_points.push_back(min_depth * bottom_left);
        base_points.push_back(max_depth * bottom_left);
        base_points.push_back(min_depth * bottom_right);
        base_points.push_back(max_depth * bottom_right);

        // 计算视平面法线（近端和远端）
        // 近端平面（Near Plane）: // 定义: 近端平面是距离相机最近的一个平面，通常用于裁剪视锥体前面的部分。
        // 解释: 这里 base_points[0] 通常表示视锥体的近端平面的一个角点（比如左下角），base_points[1] 和 base_points[4] 分别表示近端平面的另两个角点。
        //       通过计算 base_points[1] - base_points[0] 和 base_points[4] - base_points[0] 这两个向量的叉积，可以获得近端平面的法线。
        Eigen::Vector3d near_normal = (base_points[1] - base_points[0]).cross(base_points[4] - base_points[0]).normalized();
        Eigen::Vector3d far_normal = (base_points[5] - base_points[4]).cross(base_points[7] - base_points[4]).normalized();

        // 扩展视锥体
        std::vector<Eigen::Vector3d> expanded_points;
        for (const auto& point : base_points) {
            Eigen::Vector3d expanded_point_near = point + near_normal * expansion_radius;
            Eigen::Vector3d expanded_point_far = point + far_normal * expansion_radius;
            expanded_points.push_back(expanded_point_near);
            expanded_points.push_back(expanded_point_far);
        }

        // 计算扩展后的包围盒
        Eigen::Vector3d min_point = expanded_points[0];
        Eigen::Vector3d max_point = expanded_points[0];
        for (const auto& point : expanded_points) {
            min_point = min_point.cwiseMin(point);
            max_point = max_point.cwiseMax(point);
        }

        // 更新 points_ 数组
        points_[0] = min_point;
        points_[1] = Eigen::Vector3d(min_point.x(), min_point.y(), max_point.z());
        points_[2] = Eigen::Vector3d(min_point.x(), max_point.y(), min_point.z());
        points_[3] = Eigen::Vector3d(min_point.x(), max_point.y(), max_point.z());
        points_[4] = Eigen::Vector3d(max_point.x(), min_point.y(), min_point.z());
        points_[5] = Eigen::Vector3d(max_point.x(), min_point.y(), max_point.z());
        points_[6] = Eigen::Vector3d(max_point.x(), max_point.y(), min_point.z());
        points_[7] = max_point;

        // 计算视锥体的轴和面
        ComputeAxesAndPlanes();
    }

private:
    void ComputeAxesAndPlanes() {
        axes_[0] = points_[7] - points_[6];  // bottom right edge
        axes_[1] = points_[3] - points_[2];  // top right edge
        axes_[2] = points_[5] - points_[4];  // bottom left edge
        axes_[3] = points_[1] - points_[0];  // top left edge
        axes_[4] = points_[2] - points_[6];  // top <-> down edge
        axes_[5] = points_[0] - points_[2];  // left <-> right edge

        // Plane normals point away from the frustum.
        Eigen::Vector3d forward_normal_dir = axes_[5].cross(axes_[4]);
        planes_[0] = Hyperplane<double, 3>(forward_normal_dir, -forward_normal_dir.dot(points_[1]));

        planes_[1] = Hyperplane<double, 3>(-forward_normal_dir, forward_normal_dir.dot(points_[0]));

        Eigen::Vector3d right_normal_dir = axes_[0].cross(axes_[4]);
        planes_[2] = Hyperplane<double, 3>(right_normal_dir, -right_normal_dir.dot(points_[6]));

        Eigen::Vector3d top_normal_dir = axes_[1].cross(axes_[5]);
        planes_[3] = Hyperplane<double, 3>(top_normal_dir, -top_normal_dir.dot(points_[2]));

        Eigen::Vector3d left_normal_dir = axes_[4].cross(axes_[2]);
        planes_[4] = Hyperplane<double, 3>(left_normal_dir, -left_normal_dir.dot(points_[4]));

        Eigen::Vector3d bottom_normal_dir = axes_[5].cross(axes_[0]);
        planes_[5] = Hyperplane<double, 3>(bottom_normal_dir, -bottom_normal_dir.dot(points_[6]));

        axes_and_planes_computed_ = true;
    }

    Eigen::Vector3d points_[8];
    Eigen::Vector3d axes_[6];
    Eigen::Hyperplane<double, 3> planes_[6];
    bool axes_and_planes_computed_;
};
