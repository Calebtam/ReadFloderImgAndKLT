#include "glim_base_test.h"

// 二代子类
class OdometryEstimationCPU : public OdometryEstimationIMU {

    virtual int print_for_c() {
        std::cout << "print_for_c 声明在一代子类, 实现在二代子类" << std::endl;
        return 1;
    }
    void base_and_cpu() override;

};

void OdometryEstimationCPU::base_and_cpu(){
    std::cout << "base_and_cpu 声明在基类, 实现在二代子类" << std::endl;
};

std::shared_ptr<OdometryEstimationIMU> OdometryEstimationIMU::Create() {
    return std::make_shared<OdometryEstimationCPU>();
}

int main() {
    // create 里面创建了一个二代子类对象, 并返回一代子类的指针, 最终使用基类指针指向
    std::shared_ptr<OdometryEstimationBase> odometry_estimation_imu = OdometryEstimationIMU::Create();


    // 创建 IMU 数据
    IMUData imu_data = {123.456, {0.1, 0.2, 0.3}, {0.01, 0.02, 0.03}};

    std::cout << " ====================== create 里面创建了一个二代子类对象, 并返回一代子类的指针, 最终使用基类指针指向 ======================= " << std::endl;  
    odometry_estimation_imu->insert_imu(imu_data);
    odometry_estimation_imu->print_for_b();
    odometry_estimation_imu->base_and_cpu();
    // odometry_estimation_imu->print_for_c();


    return 0;
}
