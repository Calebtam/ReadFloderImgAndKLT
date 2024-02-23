#include<iostream>
#include<cstdio>
#include<cstring>
#include<vector>
#include<cmath>

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;


int main()
{

    Eigen::Matrix<double, 4, 4> A = Eigen::Matrix4d::Ones(); 
    Eigen::Matrix<double, 4, 4> B; 
    B << 1, 2, 3, 1,    // Initialize A. The elements can also be
         4, 5, 6, 4,   // matrices, which are stacked along cols
         7, 8, 9, 7,
         0, 1, 2, 3;     // and then the rows are stacked.

    A += B;
    Eigen::Matrix<double, 4, 4> C = Eigen::MatrixXd::Zero(4, 4);
    C += A.block(0, 1, 4, 2) * B.block(2, 0, 2, 4);

    std::cout << "A =" << std::endl << A << std::endl;
    std::cout << "B =" << std::endl << B << std::endl;
    std::cout << "A.block(0, 1, 4, 2) =" << std::endl << A.block(0, 1, 4, 2) << std::endl;
    std::cout << "B.block(2, 0, 2, 4) =" << std::endl << B.block(2, 0, 2, 4) << std::endl;
    std::cout << "C =" << std::endl << C << std::endl;

    return 0;
}