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


  MatrixXd n = MatrixXd::Zero(6,6);
  VectorXd v_idx(6);
  VectorXd v_ones = VectorXd::Ones(6);
  v_idx << 2, 3, 4, 5, 6, 7;
  n << v_idx, (v_idx+v_ones) ,(v_idx+2*v_ones), (v_idx+3*v_ones), (v_idx+4*v_ones), (v_idx+5*v_ones);
  cout << "Here is n.sum():       " << n.sum()       << endl;
  cout << "Here is n.sum() / 36:       " << n.sum() / n.cols() / n.rows()       << endl;

  MatrixXd n2 = MatrixXd::Zero(6,6);
  VectorXd v_idx2(6);
  VectorXd v_ones2 = VectorXd::Ones(6);
  v_idx2 << 1, 2, 3, 4, 5, 6;
  n2 = v_idx2 * v_idx2.transpose();
  cout << "Here is n.sum():       " << n2.sum()       << endl;
  cout << "Here is n.sum() / 36:       " << n2.sum() / n2.cols() / n2.rows()       << endl;
}