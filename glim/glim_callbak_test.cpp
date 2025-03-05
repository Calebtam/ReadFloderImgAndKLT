#include <iostream>
#include <memory>
#include <vector>
#include <functional>

#include "callbacks.hpp"

using namespace glim;

// 模拟 IMU 数据
struct IMUData {
    double timestamp;
    // std::vector<double> linear_acceleration;  // [ax, ay, az]
    // std::vector<double> angular_velocity;     // [wx, wy, wz]
    Eigen::Vector3d linear_acceleration;  // [ax, ay, az]
    Eigen::Vector3d angular_velocity;     // [wx, wy, wz]
};

// ✅方法 1：直接传入普通函数
void imuCallback1(double stamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
    std::cout << "Processing IMU data by 方法 1：普通函数 " << std::endl;
}

// ✅方法 2：使用 std::bind 绑定成员函数
class IMUProcessor1 {
public:
    void processIMU(double stamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
        std::cout << "IMU Processor handling data by 方法 2：类中使用std::bind 绑定成员函数 " << std::endl;
    }

    void registerCallback() {
        glim::OdometryEstimationCallbacks::on_insert_imu.add(
            std::bind(&IMUProcessor1::processIMU, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );
    }
};

// ✅ 方法 3：使用 std::function 绑定 lambda
auto imuLambda = [](double stamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
    std::cout << "Lambda IMU Callback 方法 3：使用 std::function 绑定 lambda " << std::endl;
};

// ✅ 方法 4：绑定静态成员函数
class IMUProcessor2 {
public:
    static void processIMU(double stamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
        std::cout << "Static IMU Processor handling data 方法 4：绑定静态成员函数 " << std::endl;
    }
};

// 基类，处理 IMU 数据插入
class OdometryEstimationBase {
public:
    virtual ~OdometryEstimationBase() = default;

    // 插入 IMU 数据（虚函数，可以在子类中重写）
    virtual void insert_imu(const IMUData& imu_data) {
        OdometryEstimationCallbacks::on_insert_imu(imu_data.timestamp, imu_data.linear_acceleration, imu_data.angular_velocity);
        std::cout << "基类 Linear Acceleration: [" 
                  << imu_data.linear_acceleration(0) << ", " 
                  << imu_data.linear_acceleration(1) << ", " 
                  << imu_data.linear_acceleration(2) << "]" << std::endl;
        std::cout << "基类 Angular Velocity: [" 
                  << imu_data.angular_velocity(0) << ", " 
                  << imu_data.angular_velocity(1) << ", " 
                  << imu_data.angular_velocity(2) << "]" << std::endl;
    }
};

// 一代子类：OdometryEstimationIMU，扩展了基类的 insert_imu 方法
class OdometryEstimationIMU : public OdometryEstimationBase {
public:
    // 重写插入 IMU 数据的处理方式
    virtual void insert_imu(const IMUData& imu_data) override {
        OdometryEstimationCallbacks::on_insert_imu(imu_data.timestamp, imu_data.linear_acceleration, imu_data.angular_velocity);
        std::cout << "一代子类 IMU Data received: Timestamp = " << imu_data.timestamp << std::endl;
    }
};

// 二代子类
class OdometryEstimationCPU : public OdometryEstimationIMU {

};

int main() {
    IMUProcessor1 imu_;

    // 多回调函数统一注册
    glim::OdometryEstimationCallbacks::on_insert_imu.add(imuCallback1);                // ✅ 方法 1：直接传入普通函数
    imu_.registerCallback();                                                           // ✅ 方法 2：使用 std::bind 绑定成员函数
    glim::OdometryEstimationCallbacks::on_insert_imu.add(imuLambda);                   // ✅ 方法 3：使用 std::function 绑定 lambda
    glim::OdometryEstimationCallbacks::on_insert_imu.add(IMUProcessor2::processIMU);   // ✅ 方法 4：绑定静态成员函数
    glim::OdometryEstimationCallbacks::on_insert_imu.add([](double stamp, Eigen::Vector3d acc, Eigen::Vector3d gyro) {
        std::cout << "Processing IMU data 方法 5： " << std::endl;
    });

    // 创建一个具体的 IMU 估计对象
    std::shared_ptr<OdometryEstimationBase> odometry_estimation = std::make_shared<OdometryEstimationCPU>();

    // 创建 IMU 数据
    IMUData imu_data = {123.456, {0.1, 0.2, 0.3}, {0.01, 0.02, 0.03}};

    // 插入 IMU 数据
    odometry_estimation->insert_imu(imu_data);
    std::cout << std::endl;
    odometry_estimation->OdometryEstimationBase::insert_imu(imu_data);

    return 0;
}
