#pragma once

// Standard libraries
#include <iostream>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>


class targetTrackingFilter
{
private:
	std::vector<int> missingData;
	std::vector<cv::Mat> targetsModel;
	std::vector<cv::Mat> predictions;
	std::vector<cv::KalmanFilter> KFs;
	std::vector<int> noOfTarget;
	int nbOfTargets;
	float dt;
	float dv;

public:
	targetTrackingFilter (float velocityFactor = 1, float accelerationFactor = -1, int maxMissingData = 5);
	~targetTrackingFilter();
	
	void applyFilter(cv::Mat &image,std::vector<cv::Rect> targets);
	void drawTargets(cv::Mat &image,cv::Scalar color = CV_RGB(255,0,0), int thickness = 1);
};


