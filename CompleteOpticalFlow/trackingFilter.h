#pragma once

// Standard libraries
#include <iostream>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>


class trackingFilter
{
private:
	int missingData;
	
	cv::KalmanFilter KF;
	float dt;
	float dv;

public:
	trackingFilter (float x, float y, float velocityFactor = 1, float accelerationFactor = -1, int maxMissingData = 20);
	cv::Mat applyFilter(float x, float y);
	cv::Mat updateRelativePosition(float x, float y,float relativeX, float relativeY, float &deltaX, float &deltaY);
};


