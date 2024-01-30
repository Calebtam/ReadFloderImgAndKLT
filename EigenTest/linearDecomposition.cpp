#include <iostream>
#include <Eigen/Dense>
 
using namespace std;
using namespace Eigen;
/*
Linear Algebra and Decomposition

    The problem: You have a system of equations, that you have written as a single matrix equation
    Ax = b
    Where A and b are matrices (b could be a vector, as a special case). You want to find a solution x.
    The solution: You can choose between various decompositions, depending on the properties of your matrix A, and depending on whether you favor speed or accuracy. However, let's start with an example that works in all cases, and is a good compromise:

    https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html

    1.  ColPivHouseholderQR 是具有列旋转的 QR 分解。它适用于所有矩阵，同时速度相当快。

    2.  SVD Least squares solving
        Eigen 提供了两种实现。推荐的类是 BDCSVD 类，它可以很好地解决大型问题，并自动回退到 JacobiSVD 类来处理较小的问题。
        对于这两个类，它们的solve()方法在最小二乘意义上求解线性系统。

        An alternative to the SVD, which is usually faster and about as accurate,
        is CompleteOrthogonalDecomposition.
        https://eigen.tuxfamily.org/dox/classEigen_1_1CompleteOrthogonalDecomposition.html


    3.  Computing inverse and determinant
        矩阵具有非常小的固定大小（最多 4x4），这允许Eigen避免执行LU分解，而是使用对此类小矩阵更有效的公式。

null-space and rank    

    4.  LLT Separating the computation from the construction
    
    5.  LU separating

    6.  Rank and the threshold

    7.  Computing eigenvalues and eigenvectors



*/
void fun1()
{
    cout << "================================= QR .inverse" << endl;
    Matrix3f A;
    Vector3f b;
    A << 1,2,3,  4,5,6,  7,8,10;
    b << 3, 3, 4;
    cout << "Here is the matrix A:\n" << A << endl;
    cout << "Here is the vector b:\n" << b << endl;
    //    Vector3f x = A.colPivHouseholderQr().solve(b);
    // the colPivHouseholderQr() method returns an object of class ColPivHouseholderQR. 
    // Since here the matrix is of type Matrix3f, this line could have been replaced by:
    ColPivHouseholderQR<Matrix3f> dec(A);
    Vector3f x = dec.solve(b);
    cout << "The solution of Ax = b is:\n" << x << endl;
}
/*
Least squares solving
The most general and accurate method to solve under- or over-determined linear systems in the least squares sense, 
is the SVD decomposition. Eigen provides two implementations. The recommended one is the BDCSVD class, which scales well for large problems and automatically falls back to the JacobiSVD class for smaller problems. 
For both classes, their solve() method solved the linear system in the least-squares sense.
https://eigen.tuxfamily.org/dox/classEigen_1_1CompleteOrthogonalDecomposition.html
*/
//    The least-squares solution is:
void fun2()
{
   cout << "================================= 最小二乘 Least-squares solving" << endl;
   MatrixXf A = MatrixXf::Random(3, 2);
   cout << "Here is the matrix A:\n" << A << endl;
   VectorXf b = VectorXf::Random(3);
   cout << "Here is the right hand side b:\n" << b << endl;
   cout << "The least-squares solution of Ax = b is:\n"
        << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
}
/*
Computing eigenvalues and eigenvectors
You need an eigendecomposition here, see available such decompositions on this page. Make sure to check if your matrix is self-adjoint, as is often the case in these problems. Here's an example using SelfAdjointEigenSolver, 
it could easily be adapted to general matrices using EigenSolver or ComplexEigenSolver.
The computation of eigenvalues and eigenvectors does not necessarily converge, 
but such failure to converge is very rare. The call to info() is to check for this possibility.
*/
void fun7()
{
    cout << "================================= 计算特征值和特征向量" << endl;
    Matrix2f A;
    A << 1, 2, 2, 3;
    cout << "Here is the matrix A:\n" << A << endl;
    SelfAdjointEigenSolver<Matrix2f> eigensolver(A);
    if (eigensolver.info() != Success) abort();
    cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;
    cout << "Here's a matrix whose columns are eigenvectors of A \n"
            << "corresponding to these eigenvalues:\n"
            << eigensolver.eigenvectors() << endl;
}
/*
 逆 计算通常有利地被solve()操作取代，并且行列式通常 不是 检查矩阵是否可逆的好方法。
然而，对于 非常 小的 矩阵，上述情况可能不成立，逆矩阵和行列式可能非常有用。

虽然某些分解（例如 PartialPivLU 和 FullPivLU）也提供 inverse() 和 行列式() 方法，
但是您也可以直接在矩阵上调用inverse()和 行列式() 。
如果您的矩阵具有非常小的固定大小（最多 4x4），这允许Eigen避免执行LU分解，而是使用对此类小矩阵更有效的公式。
*/
void fun3()
{
    cout << "================================= .inverse" << endl;
    Matrix3f A;
    A << 1, 2, 1,
            2, 1, 0,
            -1, 1, 2;
    cout << "这是矩阵 A:\n" << A << endl;
    cout << "A 的行列式是 " << A.determinant() << endl;
    cout << "A 的逆是：\n" << A.inverse() << endl;
}
/*
在上面的示例中，在构造分解对象的同时计算分解。
然而，在某些情况下，您可能希望将这两件事分开，例如，如果您在构造时不知道要分解的矩阵；或者如果您想重用现有的分解对象。
使这一切成为可能的是：
    所有分解都有一个默认构造函数，
    所有分解都有一个进行计算的compute(matrix)方法，并且可以在已经计算的分解上再次调用该方法，重新初始化它。

最后，您可以告诉分解构造函数为分解给定大小的矩阵预先分配存储空间，这样当您随后分解此类矩阵时，
就不会执行动态内存分配（当然，如果您使用的是固定大小的矩阵，则不会执行动态内存分配）分配完全发生）。
这是通过将大小传递给分解构造函数来完成的，如下例所示：
*/
void fun4()
{
    cout << "================================= LLT  .inverse" << endl;

    Matrix2f A,b;
    LLT<Matrix2f> llt;
    A << 2,-1,-1,3;
    b << 1, 2, 3, 1;
    cout << "这是矩阵 A:\n" << A << endl;
    cout << "这是右侧 b:\n" << b << endl;

    cout << "计算 LLT 分解..." << endl;
    llt.compute(A);
    cout << "解为：\n" << llt.solve(b) << endl;

    A(1,1)++;
    cout << "现在矩阵 A 是：\n" << A << endl;
    cout << "计算 LLT 分解..." << endl;
    llt.compute(A);
    cout << "现在的解是：\n" << llt.solve(b) << endl;

    HouseholderQR<MatrixXf> qr(50,50);
    MatrixXf B = MatrixXf::Random (50,50);
    qr.compute(B); // 没有动态内存分配
    VectorXf d = VectorXf::Random (50);
    // cout << "50 * 50 矩阵的解是：\n" << qr.solve(d) << endl;
    
}
/*
显示秩的分解
某些分解是揭示秩的，即能够计算矩阵的秩。这些通常也是在面对非满秩矩阵（在方形情况下意味着奇异矩阵）时表现最好的分解。
在 此表中 ，您可以看到我们所有的分解，无论它们是否揭示等级。

Rank揭示分解至少提供了一个rank()方法。它们还可以提供方便的方法，例如 isInvertible()，
有些还提供计算矩阵的kernel（零空间）和image（列空间）的方法，如 FullPivLU的情况：
*/
void fun5()
{
    cout << "================================= nullspace and columns space" << endl;

    Matrix3f A;
    A << 1, 2, 5,
            2, 1, 4,
            3, 0, 3;
    cout << "这是矩阵 A:\n" << A << endl;
    FullPivLU<Matrix3f> lu_decomp(A);
    cout << "A 的秩为 " << lu_decomp.rank() << endl;
    cout << "这是一个矩阵，其列构成 A 的零空间的基础：\n"
         << "Here is a matrix whose columns form a basis of the null-space of A:\n"
            << lu_decomp.kernel() << endl;
    cout << "这是一个矩阵，其列构成 A 的列空间的基础：\n"
         << "Here is a matrix whose columns form a basis of the column-space of A:\n"
            << lu_decomp.image(A) << endl; // 是的，必须传递原来的A
}
/*
当然，任何秩计算都取决于任意阈值的选择，因为实际上没有浮点矩阵 完全是 秩不足的。 
Eigen 选择一个合理的默认阈值，该阈值取决于分解，但通常是对角线大小乘以机器 epsilon。
虽然这是我们可以选择的最佳默认值，但只有您知道适合您的应用程序的正确阈值。
您可以通过在调用rank() 或任何其他需要使用此类阈值的方法之前对分解对象调用setThreshold() 来设置此阈值。
分解本身，即compute()方法，与阈值无关。更改阈值后无需重新计算分解。
*/
void fun6()
{
    cout << "================================= rank " << endl;
    Matrix2d A;
    A << 2, 1,
            2,0.9999999999;
    FullPivLU<Matrix2d> lu(A);
    cout << "默认情况下，A的rank为" << lu.rank() << endl;
    // lu.matrixLU()
    lu.setThreshold(1e-5);
    cout << "以1e-5为阈值，A的rank为" << lu.rank() << endl;
}
int main()
{
    fun1();

    fun2();

    fun3();

    fun4();

    fun5();

    fun6();

    fun7();

    return 0;
}