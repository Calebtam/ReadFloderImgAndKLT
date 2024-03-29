#include <iostream>
#include "Utils.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ArucoDetector.h"

using namespace std;
using namespace cv;

int main() 
{
	Mat marker = imread("imgs/marker.png");
	ArucoDetector detector(marker, 36);

	VideoCapture cap(0);
	while (true) {
		Mat frame;
		cap >> frame;

		Mat gray;
		cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
		vector<ArucoResult> ars = detector.detectArucos(gray, 1);
		//Utils::drawArucos(frame, ars);
		Utils::drawAxisWithPose(frame, ars, detector.m_dict);

		imshow("frame", frame);
		int k = waitKey(50);
		if (k >= 0) {
			break;
		}
	}
	return 1;
}