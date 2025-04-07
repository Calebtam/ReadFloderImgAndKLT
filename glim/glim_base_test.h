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

// 基类，处理 IMU 数据插入
class OdometryEstimationBase {
    public:
        virtual ~OdometryEstimationBase() = default;
    
        // 插入 IMU 数据（虚函数，可以在子类中重写）
        virtual void insert_imu(const IMUData& imu_data) {
            OdometryEstimationCallbacks::on_insert_imu(imu_data.timestamp, imu_data.linear_acceleration, imu_data.angular_velocity);
            std::cout << "基类和一代子类有声明和实现, 此次调用的是基类,Timestamp = " << imu_data.timestamp << std::endl;
    
            // std::cout << "基类 Linear Acceleration: [" 
            //           << imu_data.linear_acceleration(0) << ", " 
            //           << imu_data.linear_acceleration(1) << ", " 
            //           << imu_data.linear_acceleration(2) << "]" << std::endl;
            // std::cout << "基类 Angular Velocity: [" 
            //           << imu_data.angular_velocity(0) << ", " 
            //           << imu_data.angular_velocity(1) << ", " 
            //           << imu_data.angular_velocity(2) << "]" << std::endl;
        }
        virtual void print_for_b(){};
    
        virtual void base_and_cpu()=0;
    };
    
    // 一代子类：OdometryEstimationIMU，扩展了基类的 insert_imu 方法
    class OdometryEstimationIMU : public OdometryEstimationBase {
    public:
        static std::shared_ptr<OdometryEstimationIMU> Create();
    
        // 重写插入 IMU 数据的处理方式
        virtual void insert_imu(const IMUData& imu_data) override {
            OdometryEstimationCallbacks::on_insert_imu(imu_data.timestamp, imu_data.linear_acceleration, imu_data.angular_velocity);
            std::cout << "基类和一代子类有声明和实现, 此次调用的是一代子类,Timestamp = " << imu_data.timestamp << std::endl;
        }
        virtual int print_for_c()=0;
        // virtual int print_for_c(){};
    
        virtual void print_for_b() override{
            std::cout << "print_for_b 声明在基类, 实现在一代子类, 在一代子类中调用print_for_c" << std::endl;
            if(print_for_c())
                ;
        }
    };