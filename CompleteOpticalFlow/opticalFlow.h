#pragma once

// Standard libraries
#include <iostream>
#include <fstream>
#include <math.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"


class opticalFlow
{
private:
	
	std::string detectorName;
	int cornerBackgroundSize;
	int bestPointToKeep;
	float updateRate;
	bool refreshAllMode;
	
	cv::Ptr<cv::FeatureDetector> detector;
	
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point2f> kptPrev,kpt,kptNext;
	std::vector<uchar> status; 
	cv::Mat hStatus;
	std::vector<float> err;
	
public:
	opticalFlow(std::string detectorName , int cornerBackgroundSize =-1 ,int bestPointToKeep = 200 , float updateRate = 0.86 , bool refreshAllMode = true);
	
	void drawOpticalflowArrows (cv::Mat image, int scale= 7, cv::Scalar color=CV_RGB(255,0,0));
	void drawDots(cv::Mat centersMatrix, cv::Mat &image, cv::Scalar color = cv::Scalar(0,200,0) , int thickness = 15);
	
	void markersMaskUpdate(cv::Mat matrix , cv::Mat &mask);
	void FeatureDetection(cv::Mat frame , cv::Mat mask);
	void findProjectiveMatrix(cv::Mat framePrev, cv::Mat frame, cv::Mat &homography);
	void keyPointsUpdate(cv::Mat frame, cv::Mat mask);
	
	float rmsError(cv::Mat coordinate1 , cv::Mat coordinate2);
	
};