//
// Created by zyn on 2023/12/22.
//

#include <Eigen/Eigen>
#include <iostream>
#include <Eigen/Dense>
#include <ctime>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
int main(){
  Eigen::MatrixXcd randvalue1 = Eigen::MatrixXcd::Random(200, 200);
  Eigen::MatrixXcd randResult = Eigen::MatrixXcd::Random(200, 200);
  auto rT1 = boost::posix_time::microsec_clock::local_time();
  randResult = randvalue1*randvalue1;
  auto rT2 = boost::posix_time::microsec_clock::local_time();
  double dt = (double) ( (rT2 - rT1).total_microseconds() );
  std::cout << "just multiply: " << dt << std::endl;
  return 1;
}