#include <iostream>
#include <Eigen/Dense>

int propagate() {
    // Define parameters
    double linear_vel_mid = 0.0; // Start at 0, will test up to 1.3 m/s
    double time_delta = 0.0125;
    double linear_vel_sigma = 0.02;
    double angular_vel_sigma = 0.002;

    Eigen::Matrix3d jocabian_x;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;

    // Initialize P_ matrix (example values, adjust as needed)
    P_ << 1e-6, 0, 0,
          0, 1e-6, 0,
          0, 0, 1e-6;

    for (linear_vel_mid = 0.0; linear_vel_mid <= 1.3; linear_vel_mid += 0.1) {
        // Define state
        struct {
            double theta = M_PI / 4; // Example value: 45 degrees
        } state_;

        // Compute jocabian_x
        jocabian_x << 1, 0, -linear_vel_mid * time_delta * sin(state_.theta),
                      0, 1,  linear_vel_mid * time_delta * cos(state_.theta),
                      0, 0, 1;

        // Print jocabian_x
        std::cout << "jocabian_x: \n" << jocabian_x << std::endl;

        // Compute noise_std
        Eigen::Vector3d noise_std;
        noise_std << std::cos(state_.theta) * linear_vel_sigma * time_delta,
                     std::sin(state_.theta) * linear_vel_sigma * time_delta,
                     angular_vel_sigma * time_delta;
        noise_std = noise_std.array().square();

        // Print noise_std
        std::cout << "noise_std: " << noise_std.transpose() << std::endl;

        // Compute Q_
        Q_ = noise_std.asDiagonal();

        // Print old P_
        std::cout << "p_old: \n" << P_ << std::endl;

        // Propagate error-state covariance
        P_ = jocabian_x * P_ * jocabian_x.transpose() + Q_;

        // Print updated P_
        std::cout << "Updated P_: \n" << P_ << std::endl;

        std::cout << "========================================" << std::endl;
    }

    return 0;
}


int main() {
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
    std::cout << "p_1:\n" << P_1 << std::endl;

    Eigen::Matrix3d cov2;
    cov2 << 0.100000e-07, 0.000000e+00, 0.000000e+00,
            0.000000e+00, 0.100000e-07, 0.000000e+00,
            0.000000e+00, 0.000000e+00, 0.010000e-07;

    Eigen::Matrix3d K2 = P_ * (P_ + cov2).inverse();
    Eigen::Matrix3d P_2 = (Eigen::Matrix3d::Identity() - K2) * P_;

    std::cout << "\n 8 8 9" << std::endl;
    std::cout << "P_:\n" << P_ << std::endl;
    std::cout << "cov:\n" << cov2 << std::endl;
    std::cout << "P_+ cov:\n" << P_ + cov2 << std::endl;
    // std::cout << "K.diag: " << K2.diagonal().transpose() << std::endl;
    std::cout << "K:\n" << K2 << std::endl;
    std::cout << "p_2:\n" << P_2 << std::endl;

    return 0;
}
