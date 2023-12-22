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




// int visited_x_[MaxN], visited_y_[MaxN];
// int Slack[MaxN];
int num_x_, num_y_;

const double epsilon = 1e-10;
int w[4][4];
#define cost_matrix w

int main()
{

        num_x_ = 4;
        num_y_ = 4;
        w[0][0] = 2;  w[0][1] = 5;   w[0][2] = 1;  w[0][3] = 2; 
        w[1][0] = 8;   w[1][1] = 3;   w[1][2] = 9;  w[1][3] = 3;  
        w[2][0] = 4;  w[2][1] = 9;  w[2][2] = 2; w[2][3] = 4; 
        w[3][0] = 3;   w[3][1] = 5;  w[3][2] = 6; w[3][3] = 5; 

/*
    Eigen::Matrix<double, 4, 4> A = Eigen::Matrix4d::Ones(); 
    Eigen::Matrix<double, 4, 4> B; 
    B << 1, 2, 3, 1,    // Initialize A. The elements can also be
         4, 5, 6, 4,   // matrices, which are stacked along cols
         7, 8, 9, 7,
         0, 1, 2, 3;     // and then the rows are stacked.

    A += B;
    Eigen::Matrix<double, 4, 4> C = Eigen::MatrixXd::Zero(4, 4);
    C += A.block(1, 1, 2, 2) * B.block(2, 2, 2, 2);

    std::cout << "A =" << std::endl << A << std::endl;
    std::cout << "B =" << std::endl << B << std::endl;
    std::cout << "C =" << std::endl << C << std::endl;
*/
    // // 与稠密矩阵类似，triangularView()函数可以处理稀疏矩阵的三角部分，进行右侧是稠密矩阵的三角求解，例如：
    // dm2 = sm1.triangularView<Lower>(dm1);
    // dv2 = sm1.transpose().triangularView<Upper>(dv1);

    // //  对称的稠密-稀疏矩阵相乘
    // //  稀疏对称矩阵与稠密矩阵或向量相乘 ，可以用selfadjointView()来进行优化：
    // // dm2 = sm1.selfadjointView<>() * dm1;          // if all coefficients of sm1 are stored
    // // dm2 = sm1.selfadjointView<Upper>() * dm1;     // if only the upper part of sm1 is stored
    // // dm2 = sm1.selfadjointView<Lower>() * dm1;     // if only the lower part of sm1 is stored
    // C = B.selfadjointView();
    // std::cout << C << std::endl;


	Eigen::MatrixXf zzz(4, 4);
	Eigen::MatrixXf l, m;
	zzz << 1, 2, 3, 4, 5, 6, 7, 8, 9, 7, 8, 9, 8, 8, 4, 5;
	cout << "MatrixXf" << endl << zzz << endl;

    // 矩阵下三角
	l = zzz.triangularView<Eigen::Lower>();
	cout << "Lower" << endl << l << endl;

	m = l.selfadjointView<Eigen::Lower>();
	cout << "selfadjointView" << endl << m << endl;

    // 矩阵上三角
	l = zzz.triangularView<Eigen::Upper>();
	cout << "Upper" << endl << l << endl;

    // 矩阵上三角 + 对角线上元素全零
	l = zzz.triangularView<Eigen::ZeroDiag>();  //后来发现在某些情况下不对，找不到原因，只能改用StrictlyUpper
	cout << "ZeroDiag" << endl << l << endl;

    // 矩阵下三角 + 对角线全1
	l = zzz.triangularView<Eigen::UnitLower>();
	cout << "UnitLower" << endl << l << endl;

    // 矩阵上三角 + 对角线全1
	l = zzz.triangularView<Eigen::UnitUpper>();
	cout << "UnitUpper" << endl << l << endl;

    // 矩阵下三角 + 对角线上元素全零
	l = zzz.triangularView<Eigen::StrictlyLower>();
	cout << "StrictlyLower" << endl << l << endl;

    // 矩阵上三角 + 对角线上元素全零
	l = zzz.triangularView<Eigen::StrictlyUpper>();
	cout << "StrictlyUpper" << endl << l << endl;

    // 
	l = zzz.triangularView<Eigen::SelfAdjoint>();
	cout << "SelfAdjoint" << endl << l << endl;


	l = zzz.triangularView<Eigen::Symmetric>();
	cout << "Symmetric" << endl << l << endl;

    // l = zzz.selfadjointView<Eigen::Upper>();
    // zzz.selfadjointView<Eigen::Upper>().llt().solveInPlace(l);
	// cout << "selfadjointView llt" << endl << l << endl;
    
	// cout << "selfadjointView" << endl << l.selfadjointView<Eigen::Upper>() << endl;

    // 向下三角部分写数据
    // m1.triangularView<Eigen::Lower>() = m2 + m3

    // 转换成矩阵，并将另一部分设置为0： 
    // m2 = m1.triangularView<Eigen::UnitUpper>()

    // 乘法
    // m3 += s1 * m1.adjoint().triangularView<Eigen::UnitUpper>() * m2
    // m3 -= s1 * m2.conjugate() * m1.adjoint().triangularView<Eigen::Lower>()

    // 对称/伴随矩阵(Symmetric/selfadjoint views)  
    // 可以引用方阵的任何三角形部分 以将其视为自伴矩阵并执行特殊的优化操作

    // 转换为稠密矩阵：  Conversion to a dense matrix:
    // m2 = m.selfadjointView<Eigen::Lower>();      
                                
    // Product with another general matrix or vector:
    // m3  = s1 * m1.conjugate().selfadjointView<Eigen::Upper>() * m3;    
    // m3 -= s1 * m3.adjoint() * m1.selfadjointView<Eigen::Lower>();

    // Solving linear equations: m2 = m1^-1 * m2
    // via a standard Cholesky factorization
    // m2 = m1.selfadjointView<Eigen::Upper>().llt().solve(m2);
    // via a Cholesky factorization with pivoting
    // m2 = m1.selfadjointView<Eigen::Lower>().ldlt().solve(m2);
    return 0;
}