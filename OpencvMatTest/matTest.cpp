//
// Created by zyn on 2021/11/12.
//
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
int main()
{
    int *p;
    Mat mat1;
    uint8_t *data;
    {
        Mat mat2(2,2,CV_8UC3, Scalar(1));
        cout << "mat2: " << mat2 << endl;
        mat1 = mat2;
        int a = 2;
        p = &a;
        data = mat2.data;
    }
    int b[1000000] = {1,1,1};
//    cout << "mat2: " << mat1 << endl;
    cout << "*p: " << *p << ",p" << p << endl;
    cout << int(*data) << endl;
    return 1;
}