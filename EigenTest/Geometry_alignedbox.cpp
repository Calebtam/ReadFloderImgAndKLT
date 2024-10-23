#include <iostream>
#include <Eigen/Geometry>

int main() {
    // // Define two 2D points
    // Eigen::Vector2d p1(1.0, 2.0);
    // Eigen::Vector2d p2(3.0, 4.0);

    // // Create an empty AlignedBox2d
    // Eigen::AlignedBox2d box;

    // // Extend the box with the points
    // box.extend(p1);
    // box.extend(p2);

    // // Print the min and max corners of the box
    // std::cout << "Min corner: " << box.min().transpose() << std::endl;
    // std::cout << "Max corner: " << box.max().transpose() << std::endl;

    // // Testing other functions
    // std::cout << "Center: " << box.center().transpose() << std::endl;
    // std::cout << "Sizes: " << box.sizes().transpose() << std::endl;
    // std::cout << "Volume: " << box.volume() << std::endl; // 包围盒的面积
    // std::cout << "Contains p1? " << (box.contains(p1) ? "Yes" : "No") << std::endl;
    // std::cout << "Contains p2? " << (box.contains(p2) ? "Yes" : "No") << std::endl;

    // // Testing the intersection with another box
    // Eigen::AlignedBox2d box2(Eigen::Vector2d(2.0, 3.0), Eigen::Vector2d(4.0, 5.0));
    // Eigen::AlignedBox2d intersection = box.intersection(box2);
    // std::cout << "Intersection min corner: " << intersection.min().transpose() << std::endl;
    // std::cout << "Intersection max corner: " << intersection.max().transpose() << std::endl;


    std::cout << " =========================================================================== " << std::endl;
    // 定义旋转向量 (单位为弧度)
    Eigen::Vector3d rotation_vector(M_PI / 2, 0, 0);  // 绕 X 轴旋转 90 度
    // 使用旋转向量构建旋转矩阵
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized()).toRotationMatrix();
    std::cout << "Rotation Matrix:\n" << rotation_matrix << std::endl;
    std::cout << "Rotation Matrix:\n" << rotation_matrix.inverse() << std::endl;


    // 创建一个初始的三维边界框，使用两个对角点初始化
    Eigen::Vector3d minCorner(0.0, 0.0, 0.0);
    Eigen::Vector3d maxCorner(1.0, 1.0, 1.0);

    Eigen::Vector3d minCorner_transformed = rotation_matrix * minCorner;
    Eigen::Vector3d maxCorner_transformed = rotation_matrix * maxCorner;
    std::cout << "Min corner transformed: " << minCorner_transformed.transpose() << std::endl;
    std::cout << "Max corner transformed: " << maxCorner_transformed.transpose() << std::endl;

    Eigen::AlignedBox<double, 3> bbox;//(minCorner, maxCorner);
    bbox.extend(minCorner_transformed);
    bbox.extend(maxCorner_transformed);

    // 打印初始边界框的最小和最大角点
    std::cout << "Initial bounding box:" << std::endl;
    std::cout << "Min corner: " << bbox.min().transpose() << std::endl;
    std::cout << "Max corner: " << bbox.max().transpose() << std::endl;

    // 扩展边界框以包含一个新的点
    Eigen::Vector3d point4(2.0, 2.0, 2.0);
    bbox.extend(point4);
    std::cout << "\nAfter extending with point (2, 2, 2):" << std::endl;
    std::cout << "Min corner: " << bbox.min().transpose() << std::endl;
    std::cout << "Max corner: " << bbox.max().transpose() << std::endl;

    // 计算边界框的中心点
    Eigen::Vector3d center = bbox.center();
    std::cout << "\nCenter of the bounding box: " << center.transpose() << std::endl;

    // 计算边界框的体积
    double volume = bbox.volume();
    std::cout << "Volume of the bounding box: " << volume << std::endl;

    // 平移边界框
    bbox.translate(Eigen::Vector3d(1.0, 1.0, 1.0));
    std::cout << "\nAfter translating by (1, 1, 1):" << std::endl;
    std::cout << "Min corner: " << bbox.min().transpose() << std::endl;
    std::cout << "Max corner: " << bbox.max().transpose() << std::endl;

    // // 缩放边界框（以中心为基准）
    // bbox.scale(2.0);
    // std::cout << "\nAfter scaling by 2.0:" << std::endl;
    // std::cout << "Min corner: " << bbox.min().transpose() << std::endl;
    // std::cout << "Max corner: " << bbox.max().transpose() << std::endl;

    // 合并另一个边界框
    Eigen::AlignedBox<double, 3> bbox2(Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(0.5, 0.5, 0.5));
    bbox.extend(bbox2);
    std::cout << "\nAfter merging with another bounding box:" << std::endl;
    std::cout << "Min corner: " << bbox.min().transpose() << std::endl;
    std::cout << "Max corner: " << bbox.max().transpose() << std::endl;

    // 裁剪边界框（裁剪掉超出指定范围的部分）
    Eigen::AlignedBox<double, 3> clippingBox(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 2.0, 2.0));
    // bbox.clip(clippingBox);
    // std::cout << "\nAfter clipping with a box from (0, 0, 0) to (2, 2, 2):" << std::endl;
    // std::cout << "Min corner: " << bbox.min().transpose() << std::endl;
    // std::cout << "Max corner: " << bbox.max().transpose() << std::endl;

    // 检查是否包含另一个边界框
    bool contains = bbox.contains(clippingBox);
    std::cout << "\nDoes the bounding box contain the clipping box? " << (contains ? "Yes" : "No") << std::endl;

    return 0;
}

