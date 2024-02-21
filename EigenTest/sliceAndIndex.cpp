#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;
 
int main()
{
    MatrixXf B = MatrixXf::Random (10,10);
    cout << "现在矩阵 B 是：\n" << B << endl;

/*
function	                    example                   description
seq(firstIdx,lastIdx)           seq(2,5) <=> {2,3,4,5}     represents the sequence of integers ranging from firstIdx to lastIdx	
seq(firstIdx,lastIdx,incr)      seq(2,8,2) <=> {2,4,6,8}   same but using the increment incr to advance from one index to the next	
seqN(firstIdx,size)             seqN(2,5) <=> {2,3,4,5,6}  represents the sequence of size integers starting from firstIdx	
seqN(firstIdx,size,incr)        seqN(2,3,3) <=> {2,5,8}    same but using the increment incr to advance from one index to the next	



Code	                    Block-API equivalence               Intent	
A(seq(i,last), seqN(0,n))   A.bottomLeftCorner(A.rows()-i,n)    Bottom-left corner starting at row i with n columns	
A(seqN(i,m), seqN(i,n))     A.block(i,j,m,n)                    Block starting at i,j having m rows, and n columns	
A(seq(i0,i1), seq(j0,j1)    A.block(i0,j0,i1-i0+1,j1-j0+1)      Block starting at i0,j0 and ending at i1,j1	
A(all, seq(0,last,2))                                           Even columns of A	
A(seqN(1,n,2), all)                                             First n odd rows A	
A(all, last-1)              A.col(A.cols()-2)                   The last past one column	                	
A(last/2,all)               A.row((A.rows()-1)/2)               The middle row
v(seq(i,last))              v.tail(v.size()-i)                  Last elements of v starting at i
v(seq(last+1-n,last))       v.tail(n)                           Last n elements of v
*/
    int i = 0;
    // Bottom-left corner starting at row i with n columns
    cout << "B.bottomLeftCorner(B.rows()-3, 3)  " << endl << B.bottomLeftCorner(B.rows()-3, 3) << endl;

// https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
    MatrixXcf a = MatrixXcf::Random(2,2);   // std::complex<float>
    cout << "Here is the matrix a\n" << a << endl; 
    cout << "Here is the matrix a^T\n" << a.transpose() << endl;  
    cout << "Here is the conjugate 共轭 of a\n" << a.conjugate() << endl;  
    cout << "Here is the matrix 伴随 a^*\n" << a.adjoint() << endl;

    Matrix2i b; 
    b << 1, 2, 3, 4;
    cout << "Here is the matrix a:\n" << b << endl;
    // b = b.transpose(); // !!! do NOT do this !!!
    // cout << "and the result of the aliasing effect:\n" << b << endl;
    b = b.transpose().eval(); // do this instead
    cout << "the result :\n" << b << endl;
    // If you know your matrix product can be safely evaluated into the destination matrix without aliasing issue, then you can use the noalias() function to avoid the temporary, e.g.:
    // c.noalias() += a * b;

    MatrixXf c(2,3); 
    c << 1, 2, 3, 4, 5, 6;
    cout << "Here is the initial matrix a:\n" << c << endl; 
    c.transposeInPlace();
    cout << "and after being transposed:\n" << c << endl;


    Eigen::Matrix2d mat;    
    mat << 1, 2,
            3, 4;
    cout << "Here is mat.sum():       " << mat.sum()       << endl;
    cout << "Here is mat.prod() 所有值的乘积:      " << mat.prod()      << endl;
    cout << "Here is mat.mean():      " << mat.mean()      << endl;
    cout << "Here is mat.minCoeff():  " << mat.minCoeff()  << endl;
    cout << "Here is mat.maxCoeff():  " << mat.maxCoeff()  << endl;
    cout << "Here is mat.trace():     " << mat.trace()     << endl;

    // Matrix3f m = Matrix3f::Random();
    // std::ptrdiff_t i, j;
    // float minOfM = m.minCoeff(&i,&j);
    // cout << "Here is the matrix m:\n" << m << endl;
    // cout << "Its minimum coefficient (" << minOfM 
    //     << ") is at position (" << i << "," << j << ")\n\n";
    
    // RowVector4i v = RowVector4i::Random();
    // int maxOfV = v.maxCoeff(&i);
    // cout << "Here is the vector v: " << v << endl;
    // cout << "Its maximum coefficient (" << maxOfV 
    //     << ") is at position " << i << endl;
}