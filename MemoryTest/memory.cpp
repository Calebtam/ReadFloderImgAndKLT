//
// Created by zyn on 2022/8/8.
//


#include <iostream>
#include <vector>
#include <math.h>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;

int main()
{
    vector<int> vector_;
    vector_.resize(4,0);
    int temp = 10;
    vector_[100] = 7;
    cout << vector_[100] << endl;
    cout << temp << endl;
    ///
//    float * array;
    void *array;
    array = (new float[16]);
//    float array[16];
    Eigen::Matrix<float, 4, 4> mat4;
    mat4.setIdentity();
    mat4(0,0) = 2, mat4(1,0) = 3, mat4(2,0) = 4, mat4(3,0) = 5;
        memcpy(array,mat4.data(),16*sizeof(float));
    std::cout << "mat4 data: " << mat4 << std::endl;
//    std::cout << "array[0]: " << array[0] << std::endl;
    std::cout << "array: "<< ((float*)array)[0] << ","<< ((float*)array)[1] <<
     ","<< ((float*)array)[2] << ","<< ((float*)array)[3] << ","
        << ((float*)array)[12] << ","<< ((float*)array)[13] << ","
        << ((float*)array)[14] << ","<< ((float*)array)[15] << std::endl;

    double array1[16];
    std::cout << "size: " << sizeof(array1) << std::endl;


  {
    vector<int> vector_a;
    vector_a.push_back(1);
    int &b =  vector_a[0];
    cout << "before b: " << b << ",*b: " << &b << std::endl;
    vector_a.pop_back();
    vector_a.push_back(2);
    cout << "after1 b: " << b << ",*b: " << &b  << std::endl;
    vector_a.clear();
    cout << "after2 b: " << b << ",*b: " << &b  << std::endl;
  }
  void *pointer_;
  std::cout << "point point: " << &pointer_ << std::endl;
  return 0;
}