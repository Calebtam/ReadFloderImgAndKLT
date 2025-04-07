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

// 二代子类
class OdometryEstimationCPU : public OdometryEstimationIMU {

    virtual int print_for_c() {
        std::cout << "print_for_c 声明在一代子类, 实现在二代子类" << std::endl;
        return 1;
    }
    void base_and_cpu(){
        std::cout << "base_and_cpu 声明在基类, 实现在二代子类" << std::endl;
    };

};


std::shared_ptr<OdometryEstimationIMU> OdometryEstimationIMU::Create() {
    return std::make_shared<OdometryEstimationCPU>();
}

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
    // 基类指针指向 二代子类 对象
    std::shared_ptr<OdometryEstimationBase> odometry_estimation = std::make_shared<OdometryEstimationCPU>();

    // create 里面创建了一个二代子类对象, 并返回一代子类的指针, 最终使用基类指针指向
    std::shared_ptr<OdometryEstimationBase> odometry_estimation_imu = OdometryEstimationIMU::Create();
    std::shared_ptr<OdometryEstimationIMU> odometry_estimation_imu2 = OdometryEstimationIMU::Create();


    // 创建 IMU 数据
    IMUData imu_data = {123.456, {0.1, 0.2, 0.3}, {0.01, 0.02, 0.03}};

    // 插入 IMU 数据
    std::cout << " =====================基类指针指向 二代子类 对象======================== " << std::endl;  
    // 由于二代子类没有重写 insert_imu 方法，因此会调用一代子类的 insert_imu 方法  
    odometry_estimation->insert_imu(imu_data);
    std::cout << " --------------------------------------------- " << std::endl;    
    std::cout << std::endl;
    // 指向使用基类的方法
    odometry_estimation->OdometryEstimationBase::insert_imu(imu_data);
    std::cout << " --------------------------------------------- " << std::endl;    
    odometry_estimation->print_for_b();
    std::cout << " ====================== create 里面创建了一个二代子类对象, 并返回一代子类的指针, 最终使用基类指针指向 ======================= " << std::endl;  
    odometry_estimation_imu->insert_imu(imu_data);
    odometry_estimation_imu->print_for_b();
    odometry_estimation_imu->base_and_cpu();

    // 基类指针没有办法调用子类的方法
    // odometry_estimation_imu->print_for_c();
    std::cout << " ===================== create 里面创建了一个二代子类对象, 并返回一代子类的指针 ======================== " << std::endl;
    odometry_estimation_imu2->insert_imu(imu_data);
    odometry_estimation_imu2->print_for_c();
    odometry_estimation_imu2->base_and_cpu();

    return 0;
}
