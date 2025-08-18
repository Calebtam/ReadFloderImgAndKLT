#include <iostream>
#include <Eigen/Dense>

#include <iomanip> // 
int propagate() {
    // Define parameters
    double linear_vel_mid = 0.0;        // Start at 0, will test up to 1.3 m/s
    double time_delta = 0.0125;         // 采样周期 80hz
    double linear_vel_sigma = 0.02;     // 线速度噪声标准差
    double angular_vel_sigma = 0.002;   // 角速度噪声标准差

    Eigen::Matrix3d jocabian_x;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;

    // Initialize P_ matrix (example values, adjust as needed)
    P_ << 1e-6, 0, 0,
          0, 1e-6, 0,
          0, 0, 1e-6;

    // 循环测试不同的 linear_vel_mid, 从 0 m/s 到 1.3 m/s 每步增加 0.1 m/s。
    for (linear_vel_mid = 0.0; linear_vel_mid <= 1.3; linear_vel_mid += 0.1) {

        // Define state 固定朝向 θ = 45°。
        struct {
            double theta = M_PI / 4; // Example value: 45 degrees
        } state_;

        // Compute jocabian_x   这是基于差分运动学模型推导的。 前两行表示位置对角度的偏导。
        jocabian_x << 1, 0, -linear_vel_mid * time_delta * sin(state_.theta),
                      0, 1,  linear_vel_mid * time_delta * cos(state_.theta),
                      0, 0, 1;
        // Print jocabian_x
        std::cout << "jocabian_x: \n" << jocabian_x << std::endl;

        // Compute noise_std   把噪声标准差映射到状态空间，并平方成方差。  
        Eigen::Vector3d noise_std;
        noise_std << std::cos(state_.theta) * linear_vel_sigma * time_delta,
                     std::sin(state_.theta) * linear_vel_sigma * time_delta,
                     angular_vel_sigma * time_delta;
        noise_std = noise_std.array().square();
        // Print noise_std
        std::cout << "noise_std: " << noise_std.transpose() << std::endl;

        // Compute Q_  Q_ 是对角阵，假设各状态噪声独立。
        Q_ = noise_std.asDiagonal();

        // Print old P_
        std::cout << "p_old: \n" << P_ << std::endl;

        // Propagate error-state covariance   协方差预测公式
        P_ = jocabian_x * P_ * jocabian_x.transpose() + Q_;

        // Print updated P_
        std::cout << "Updated P_: \n" << P_ << std::endl;
    }

    return 0;
}

struct State2d{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1e-6;
};

void NormalizeAngle(double& angle) {
    while (angle < -M_PI) {
        angle += 2*M_PI;
    }

    while (angle > M_PI) {
        angle -= 2*M_PI;
    }
}
Eigen::Matrix3d propagate_cov(State2d state_, double linear_vel, double angular_vel) {
    double dt =  0.0125;                // 0.5;//
    double linear_vel_sigma = 0.02;     // 0.2;      //
    double angular_vel_sigma = 0.002;   // 0.02;    //

    double x = state_.x;
    double y = state_.y;
    double theta = state_.theta;

    // 状态转移雅可比矩阵 F
    // propagate state
    state_.x += linear_vel * std::cos(state_.theta) * dt;
    state_.y += linear_vel * std::sin(state_.theta) * dt;
    state_.theta += angular_vel * dt;
    NormalizeAngle(state_.theta);

    Eigen::Matrix3d jocabian_x;
    jocabian_x << 1, 0, -linear_vel * dt * sin(theta),      // 线速度 [0, 5] * 0.0125 * [0, 1]
                0, 1, linear_vel * dt * cos(theta),         // 
                0, 0, 1;

    // 下面两种转换方式都会按照列来进行打印
    // Eigen::Map<const Eigen::RowVectorXd> flat(jocabian_x.data(), jocabian_x.size());
    // Eigen::Map<const Eigen::RowVectorXd> flat_row(jocabian_x.transpose().data(), jocabian_x.size());
    // std::cout << " jocabian_x " << flat_row;

    // 过程噪声方差 Q
    // 留意角度只会改变系数，系数0-1
    Eigen::Vector3d noise_std;                              // 1 / (2^1/2) = 1.414213562373095049 / 2 = 0.707106781186547525
    noise_std << std::cos(theta) * linear_vel_sigma * dt,   // 该项只和 theta [-3.14, 3.14]有关， std::cos(theta) 系数范围 [0, 1],  系数 * 0.02 * 0.0125 = 系数 * 0.00025 = 系数 * 2.5 * 1e-4 
                 std::sin(theta) * linear_vel_sigma * dt,   // 该项只和 theta [-3.14, 3.14]有关， std::sin(theta) 系数范围 [0, 1]， 系数 * 0.02 * 0.0125 = 系数 * 0.00025 = 系数 * 2.5 * 1e-4
                 angular_vel_sigma * dt;                    // 0.002*0.0125 = 0.000025 = 1e-5 * 2.5

    // // 增加角速度对角度噪声的影响
    // noise_std(2) += std::abs(angular_vel) * 0.0005;

    // 从向量/矩阵视图转成**逐元素操作（Array）**视图。    
    //      在矩阵模式下，* 是矩阵乘法    
    //      在 Array 模式下，* 是逐元素相乘  
    //      同理，.square() 在 Array 模式下表示每个元素平方
    Eigen::Vector3d noise = noise_std.array().square();     
    // 系数*系数 = 0.707106781186547525 * 0.707106781186547525 = 0.5 
    // 系数*系数 * 6.25 * 1e-8                    系数*系数 * 6.25 * 1e-8                 6.25 * 1e-10
    // std::cos(theta)*std::cos(theta)          std::sin(theta)*std::sin(theta)

    Eigen::Matrix3d Q = noise.matrix().asDiagonal();

    // 下面这种转换方式都会按照列来进行打印
    Eigen::Map<const Eigen::RowVectorXd> flat2(Q.data(), Q.size());
    std::cout << "  Q " << flat2*10e6 << std::endl;

    // 协方差预测公式
    Eigen::Matrix3d P_new = jocabian_x * state_.cov * jocabian_x.transpose() + Q;
    state_.cov = P_new;

    return P_new;

}

Eigen::Matrix3d propagate_cov(double linear_vel, double angular_vel) {
    double dt =  0.0125;                // 0.5;//
    double linear_vel_sigma = 0.02;     // 0.2;      //
    double angular_vel_sigma = 0.002;   // 0.02;    //
    double theta = M_PI / 4;            // 45°

    // 初始协方差
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() * 1e-6;

    // // 状态转移雅可比矩阵 F
    // Eigen::Matrix3d F;
    // F << 1, 0, -linear_vel * dt * std::sin(theta),
    //      0, 1,  linear_vel * dt * std::cos(theta),
    //      0, 0, 1;

    Eigen::Matrix3d jocabian_x;
    jocabian_x << 1, 0, -linear_vel * dt * sin(theta),      // 线速度 [0, 5] * 0.0125 * [0, 1]
                0, 1, linear_vel * dt * cos(theta),         // 
                0, 0, 1;

    // 下面两种转换方式都会按照列来进行打印
    // Eigen::Map<const Eigen::RowVectorXd> flat(jocabian_x.data(), jocabian_x.size());
    // Eigen::Map<const Eigen::RowVectorXd> flat_row(jocabian_x.transpose().data(), jocabian_x.size());
    // std::cout << " jocabian_x " << flat_row;

    // 过程噪声方差 Q
    // 留意角度只会改变系数，系数0-1
    Eigen::Vector3d noise_std;                              // 1 / (2^1/2) = 1.414213562373095049 / 2 = 0.707106781186547525
    noise_std << std::cos(theta) * linear_vel_sigma * dt,   // 该项只和 theta [-3.14, 3.14]有关， std::cos(theta) 系数范围 [0, 1],  系数 * 0.02 * 0.0125 = 系数 * 0.00025 = 系数 * 2.5 * 1e-4 
                 std::sin(theta) * linear_vel_sigma * dt,   // 该项只和 theta [-3.14, 3.14]有关， std::sin(theta) 系数范围 [0, 1]， 系数 * 0.02 * 0.0125 = 系数 * 0.00025 = 系数 * 2.5 * 1e-4
                 angular_vel_sigma * dt;                    // 0.002*0.0125 = 0.000025 = 1e-5 * 2.5

    // // 增加角速度对角度噪声的影响
    // noise_std(2) += std::abs(angular_vel) * 0.0005;

    // 从向量/矩阵视图转成**逐元素操作（Array）**视图。    
    //      在矩阵模式下，* 是矩阵乘法    
    //      在 Array 模式下，* 是逐元素相乘  
    //      同理，.square() 在 Array 模式下表示每个元素平方
    Eigen::Vector3d noise = noise_std.array().square();     
    // 系数*系数 = 0.707106781186547525 * 0.707106781186547525 = 0.5 
    // 系数*系数 * 6.25 * 1e-8                    系数*系数 * 6.25 * 1e-8                 6.25 * 1e-10
    // std::cos(theta)*std::cos(theta)          std::sin(theta)*std::sin(theta)

    Eigen::Matrix3d Q = noise.matrix().asDiagonal();

    // 下面这种转换方式都会按照列来进行打印
    Eigen::Map<const Eigen::RowVectorXd> flat2(Q.data(), Q.size());
    std::cout << "  Q " << flat2 << std::endl;

    // 协方差预测公式
    Eigen::Matrix3d P_new = jocabian_x * P * jocabian_x.transpose() + Q;

    return P_new;

}


int test_propagate() {
    // 1. 验证：只改变线速度
    std::cout << "============ Var(x) vs Linear Velocity ============\n";
    for (double v = 0.0; v <= 5.0; v += 0.5) {
        std::cout << std::fixed << std::setw(12) << std::setprecision(10) << "v=" << v << " m/s";
        Eigen::Matrix3d P_new = propagate_cov(v, 0.0);
        // Eigen::Map<const Eigen::RowVectorXd> flat(P_new.data(), P_new.size());
        // std::cout <<  "\t Var=" << flat*10e6 << std::endl;
                //   << P_new(0, 0) << " " << P_new(0, 1) << " " << P_new(0, 2) << " | "
                //   << P_new(0, 0) << " " << P_new(0, 0) << " " << P_new(1, 2) << " | "
                //   << P_new(0, 0) << " " << P_new(0, 0) << " " << P_new(2, 2) << std::endl;
    }

    std::cout << "============ Var(x) vs Linear Velocity ============\n";
    State2d sta_;
    for (double v = 0.0; v <= 5.0; v += 0.5) {
        std::cout << std::fixed << std::setw(12) << std::setprecision(10) << "v= 1.0 m/s";
        Eigen::Matrix3d P_new = propagate_cov(sta_, 1.0, 0.0);
        // Eigen::Map<const Eigen::RowVectorXd> flat(P_new.data(), P_new.size());
        // std::cout <<  "\t Var=" << flat*10e6 << std::endl;
                //   << P_new(0, 0) << " " << P_new(0, 1) << " " << P_new(0, 2) << " | "
                //   << P_new(0, 0) << " " << P_new(0, 0) << " " << P_new(1, 2) << " | "
                //   << P_new(0, 0) << " " << P_new(0, 0) << " " << P_new(2, 2) << std::endl;
    }

    // 2. 验证：只改变角速度
    std::cout << "\n============ Var(theta) vs Angular Velocity ============\n";
    for (double w = 0.0; w <= 1.0; w += 0.3) {
        Eigen::Matrix3d P_new = propagate_cov(0.8, w);
        std::cout << std::fixed << std::setw(12) << std::setprecision(10) << "w=" << w << " rad/s \t Var=";
        Eigen::Map<const Eigen::RowVectorXd> flat(P_new.data(), P_new.size());
        std::cout << std::fixed << std::setw(12) << std::setprecision(10) << flat*10e6 << std::endl;
    }

    // 3. 验证：线速度 + 角速度联合变化
    std::cout << "\n============ Var(x) & Var(theta) vs v & w ============\n";
    for (double v = 0.0; v <= 1.0; v += 0.5) {
        for (double w = 0.0; w <= 1.0; w += 0.5) {
            Eigen::Matrix3d P_new = propagate_cov(v, w);
            std::cout << "v=" << v << " m/s,\t w=" << w
                      << " rad/s \t Var=";
            Eigen::Map<const Eigen::RowVectorXd> flat(P_new.data(), P_new.size());
            std::cout << std::fixed << std::setw(12) << std::setprecision(10) << flat*10e6 << std::endl;
        }
    }

    return 0;
}

int main() {
    std::cout << "============================= propagate test ================================" << std::endl;

    // propagate();
    test_propagate();
    std::cout << "\n============================= update test1 ======================" << std::endl;

    // Define P_ matrix (p_old)
    Eigen::Matrix3d P_;
    P_ << 9.49693e-07, 7.53089e-13, -7.9886e-12,
          7.53089e-13, 0.147561e-07, -4.79442e-10,
         -7.9886e-12, -4.79442e-10, 0.0950118e-07;

    // Define cov matrix
    Eigen::Matrix3d cov;
    cov << 10.00000e-07, 0.000000e+00, 0.000000e+00,
           0.000000e+00, 10.00000e-07, 0.000000e+00,
           0.000000e+00, 0.000000e+00, 0.1000000e-07;

    // Compute K matrix
    Eigen::Matrix3d K = P_ * (P_ + cov).inverse();
    Eigen::Matrix3d P_1 = (Eigen::Matrix3d::Identity() - K) * P_;

    // Print K diagonal elements
    std::cout << " 6 6 8" << std::endl;
    std::cout << "cov:\n" << cov << std::endl;
    std::cout << "P_:\n" << P_ << std::endl;
    std::cout << "P_+ cov:\n" << P_ + cov << std::endl;
    // std::cout << "K.diag: " << K.diagonal().transpose() << std::endl;
    std::cout << "K:\n" << K << std::endl;
    // Print P_ and cov matrices for verification
    std::cout << "p_new:\n" << P_1 << std::endl;

    std::cout << "\n============================= update test2 ======================" << std::endl;

    Eigen::Matrix3d cov2;
    cov2 << 0.100000e-07, 0.000000e+00, 0.000000e+00,
            0.000000e+00, 0.100000e-07, 0.000000e+00,
            0.000000e+00, 0.000000e+00, 0.010000e-07;

    Eigen::Matrix3d K2 = P_ * (P_ + cov2).inverse();
    Eigen::Matrix3d P_2 = (Eigen::Matrix3d::Identity() - K2) * P_;

    std::cout << "8 8 9" << std::endl;
    std::cout << "cov:\n" << cov2 << std::endl;
    std::cout << "P_:\n" << P_ << std::endl;
    std::cout << "P_+ cov:\n" << P_ + cov2 << std::endl;
    // std::cout << "K.diag: " << K2.diagonal().transpose() << std::endl;
    std::cout << "K:\n" << K2 << std::endl;
    std::cout << "p_new:\n" << P_2 << std::endl;

    return 0;
}
