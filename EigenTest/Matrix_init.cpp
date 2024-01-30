#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;
 
int main()
{
  MatrixXd m = MatrixXd::Random(3,3);
  cout << "1. m =" << endl << m << endl;

  m = m + MatrixXd::Constant(3,3,1.2);
  cout << "2. m+1.2 =" << endl << m << endl;

  m = m.eval() * 50;
  cout << "3. m*50 =" << endl << m << endl;
  
  VectorXd v(3);
  v << 1, 2, 3;
  cout << "4. m * v =" << endl << m * v << endl;
}