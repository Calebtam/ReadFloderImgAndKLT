#include <Eigen/Dense>
#include <iostream>
#include <vector>

class Frustum {
public:
    Frustum(const Eigen::Matrix3d& K, const std::vector<Eigen::Vector2d>& corners, double near, double far) {
        // 反投影像素点到相机坐标系
        for (const auto& corner : corners) {
            Eigen::Vector3d camPoint = K.inverse() * Eigen::Vector3d(corner.x(), corner.y(), 1.0);
            nearPlanePoints_.push_back(camPoint * near / camPoint.z());
            farPlanePoints_.push_back(camPoint * far / camPoint.z());
        }
        computePlanes();
    }

    // 判断点是否在视锥体内
    bool contains(const Eigen::Vector3d& point) const {
        for (const auto& plane : planes_) {
            if (plane.head<3>().dot(point) + plane[3] < 0) {
                return false;
            }
        }
        return true;
    }

    // 判断两个视锥体是否相交
    bool intersects(const Frustum& other) const {
        for (const auto& corner : other.getCorners()) {
            if (contains(corner)) {
                return true;
            }
        }

        for (const auto& corner : getCorners()) {
            if (other.contains(corner)) {
                return true;
            }
        }

        return false;
    }

    void printPoints() const {
        std::cout << "Near Plane Points:\n";
        for (const auto& point : nearPlanePoints_) {
            std::cout << point.transpose() << std::endl;
        }
        std::cout << "Far Plane Points:\n";
        for (const auto& point : farPlanePoints_) {
            std::cout << point.transpose() << std::endl;
        }
    }

private:
    std::vector<Eigen::Vector3d> nearPlanePoints_;
    std::vector<Eigen::Vector3d> farPlanePoints_;
    std::vector<Eigen::Vector4d> planes_;

    void computePlanes() {
        // 从反投影点计算视锥体的平面
        Eigen::Vector3d nearTopLeft = nearPlanePoints_[0];
        Eigen::Vector3d nearTopRight = nearPlanePoints_[1];
        Eigen::Vector3d nearBottomLeft = nearPlanePoints_[2];
        Eigen::Vector3d nearBottomRight = nearPlanePoints_[3];

        Eigen::Vector3d farTopLeft = farPlanePoints_[0];
        Eigen::Vector3d farTopRight = farPlanePoints_[1];
        Eigen::Vector3d farBottomLeft = farPlanePoints_[2];
        Eigen::Vector3d farBottomRight = farPlanePoints_[3];

        planes_.push_back(computePlane(nearTopLeft, nearTopRight, nearBottomLeft)); // Near plane
        planes_.push_back(computePlane(farTopRight, farTopLeft, farBottomLeft));    // Far plane
        planes_.push_back(computePlane(nearTopLeft, nearBottomLeft, farBottomLeft)); // Left plane
        planes_.push_back(computePlane(nearTopRight, farTopRight, farBottomRight));  // Right plane
        planes_.push_back(computePlane(nearTopLeft, farTopLeft, farTopRight));       // Top plane
        planes_.push_back(computePlane(nearBottomLeft, farBottomLeft, farBottomRight)); // Bottom plane
    }

    Eigen::Vector4d computePlane(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) const {
        Eigen::Vector3d normal = (p2 - p1).cross(p3 - p1).normalized();
        double d = -normal.dot(p1);
        return Eigen::Vector4d(normal[0], normal[1], normal[2], d);
    }

    std::vector<Eigen::Vector3d> getCorners() const {
        std::vector<Eigen::Vector3d> corners = nearPlanePoints_;
        corners.insert(corners.end(), farPlanePoints_.begin(), farPlanePoints_.end());
        return corners;
    }
};

int main() {
    // 假设相机内参矩阵已知
    Eigen::Matrix3d K;
    K << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;

    // 图像四个角的像素点
    std::vector<Eigen::Vector2d> corners = {
        {0, 0}, {640, 0}, {0, 480}, {640, 480}
    };

    // 近裁剪面和远裁剪面的深度
    double near = 0.1, far = 100.0;

    // 构建视锥体
    Frustum frustum(K, corners, near, far);

    // 测试视锥体是否包含某个点
    Eigen::Vector3d point(0, 0, 5);
    if (frustum.contains(point)) {
        std::cout << "Point is inside the frustum" << std::endl;
    } else {
        std::cout << "Point is outside the frustum" << std::endl;
    }

    // 创建另一个视锥体并测试是否相交
    Frustum otherFrustum(K, corners, near, far);
    if (frustum.intersects(otherFrustum)) {
        std::cout << "Frustums intersect" << std::endl;
    } else {
        std::cout << "Frustums do not intersect" << std::endl;
    }

    return 0;
}
