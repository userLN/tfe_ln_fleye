/*
 * Author:	Hélène Loozen 
 * Date:	2016
 * 
 */


// Standard libraries
#include <iostream>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

//Header
#include "targetTrackingFilter.h"

// Global variables
int MAX_MISSING_DATA;
int THRESHOLD;
int BORDERS;
float CORR_FACTOR;

// Initialization of the Kalman filter	
void initKalman(cv::KalmanFilter *KF, float x , float y, float dt = 1,float dv = -1)
{
	if ( dv<0 )
	{
		// KalmanFilter 2D traker with position and constant velocity
		KF->init(4,2,0);
		KF->transitionMatrix = *(cv::Mat_<float>(4, 4) << 
		1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1);
	}
	
	else
	{
		// KalmanFilter 2D traker with position, velocity and constant acceleration
		KF->init(6,2,0);
		KF->transitionMatrix = *(cv::Mat_<float>(6, 6) <<
		1, 0, dt, 0, 0.5*dv, 0,
		0, 1, 0, dt, 0, 0.5*dv, 
		0, 0, 1, 0, dv, 0, 
		0, 0, 0, 1, 0, dv,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);
		
		KF->statePre.at<float>(4) = 0;
		KF->statePre.at<float>(5) = 0;
		
		KF->statePost.at<float>(4) = 0;
		KF->statePost.at<float>(5) = 0;
	}
	
	KF->statePre.at<float>(0) = x;
	KF->statePre.at<float>(1) = y;
	KF->statePre.at<float>(2) = 0;
	KF->statePre.at<float>(3) = 0;
	
	KF->statePost.at<float>(0) = x;
	KF->statePost.at<float>(1) = y;
	KF->statePost.at<float>(2) = 0;
	KF->statePost.at<float>(3) = 0;
	
	cv::setIdentity(KF->measurementMatrix);
	cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(1e-4));
	cv::setIdentity(KF->measurementNoiseCov, cv::Scalar::all(1e-1));
	cv::setIdentity(KF->errorCovPost, cv::Scalar::all(.1));	
}


// Update of the transition matrix to createmore complex models	
void transitionMatrixUpdate (cv::KalmanFilter *KF, float dt, float dv)
{
 	if ( dv<0 )
	{
		KF->transitionMatrix = *(cv::Mat_<float>(4, 4) << 
		1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1);
	}
	
	else
	{
		KF->transitionMatrix = *(cv::Mat_<float>(6, 6) <<
		1, 0, dt, 0, 0.5*dv, 0,
		0, 1, 0, dt, 0, 0.5*dv, 
		0, 0, 1, 0, dv, 0, 
		0, 0, 0, 1, 0, dv,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);
	}
}

// Kalman prediction and model update

cv::Mat KalmanModelUpdate (cv::KalmanFilter *KF, float x , float y, int &missingData)
{
	cv::Mat predictMatrix = KF->predict();
	
	if(missingData < MAX_MISSING_DATA && (x<0 || y<0))
	{
			x = predictMatrix.at<float>(0);
			y = predictMatrix.at<float>(1);
			missingData ++;
	}
	
	else if ((x>=0 && y>=0))
	{		
		cv::Mat correctMatrix = KF->correct((cv::Mat_<float>(2,1)<<x,y));
		KF->statePost.at<float>(0) = x;
		KF->statePost.at<float>(1) = y;
		missingData = 0;
	}
	return (cv::Mat_<float> (1,2) << x, y);
}


targetTrackingFilter::targetTrackingFilter(float velocityFactor,float accelerationFactor, int maxMissingData)
{
	MAX_MISSING_DATA = 18;
	// MAX_MISSING_DATA down if fps down
	THRESHOLD = 25;
	BORDERS = 10;
	CORR_FACTOR = 0,97;
	// CORR_FACTOR down if fps down
	nbOfTargets =0;
	dt = velocityFactor;
	dv = accelerationFactor;
}

targetTrackingFilter::~targetTrackingFilter(){}


cv::Point center(cv::Rect square)
{
	cv::Point center;
	center.x = square.x + square.width/2;
	center.y = square.y + square.height/2;
	return center;
}


void targetTrackingFilter::applyFilter(cv::Mat &image, std::vector<cv::Rect> targets)
{
	predictions.clear();
	for(int i=0 ; i<KFs.size(); ++i)
	{
		predictions.push_back(KFs.at(i).predict());
		missingData.at(i) ++;
	}
	
	
	
	for(int i = 0 ; i<targets.size() ; ++i)
	{
		bool found = false;
		bool close = false;
		for(int j=0 ; j<predictions.size(); ++j)
		{
			// Check if the observation is in the square THRESHOLD of the track
			float deltaX = center(targets.at(i)).x-predictions.at(j).at<float>(0);
			float deltaY = center(targets.at(i)).y-predictions.at(j).at<float>(1);
			if (abs(deltaX) < THRESHOLD && abs(deltaY) < THRESHOLD)
			{
				//correlation
				close = true;
				cv::Mat correlation;
				double minVal, maxVal;
				cv::Point minLoc, maxLoc;
				cv::Mat  toCompare = cv::Mat(image,targets.at(i));
				cv::resize(toCompare,toCompare,targetsModel.at(j).size());
				cv::matchTemplate(toCompare, targetsModel.at(j), correlation, CV_TM_CCORR_NORMED);
				cv::minMaxLoc(correlation, &minVal,&maxVal,&minLoc,&maxLoc);
				std::cout << maxVal << std::endl;
				
// 				cv::Mat hist1,hist2;
// 				cv::calcHist(&cv::Mat(image,targets.at(i)));
// 				cv::calcHist(&targetsModel.at(j),1,0,cv::Mat(),hist1);
// 				cv::compareHist();
				
				if(maxVal>CORR_FACTOR)
				{
					predictions.at(j) = KFs.at(j).correct((cv::Mat_<float>(2,1)<< center(targets.at(i)).x ,center(targets.at(i)).y ));
					missingData.at(j) = 0;
					targetsModel.at(j) = cv::Mat(image,targets.at(i));
					found = true;
					break;
				}
			}
				
		}
		
		// If no tak is found for the target a new tracking filter is created 
		if(! found && ! close ) // &&(targets.at(i).x < BORDERS || targets.at(i).x > image.rows - BORDERS || targets.at(i).y < BORDERS || targets.at(i).y > image.cols - BORDERS))
		{
			cv::KalmanFilter newKalman;
			initKalman (&newKalman,  center(targets.at(i)).x, center(targets.at(i)).y, 1.5);
			KFs.push_back(newKalman);
			missingData.push_back(0);
			targetsModel.push_back(cv::Mat(image,targets.at(i)));
			noOfTarget.push_back(++nbOfTargets); 
		}
		
		// If the track of the target is lost for more than MAX_MISSING_DATA frames the tracking filter is deleted
		for(int i=0 ; i<missingData.size(); ++i)
			if(missingData.at(i)>MAX_MISSING_DATA)
			{
// 				if(missingData.size() < 2)
// 				{
// 					KFs.clear();
// 					missingData.clear();
// 					predictions.clear();
// 					
// 				}
// 				else
				{
					KFs.erase(KFs.begin()+i);
					missingData.erase(missingData.begin()+i);
					predictions.erase(predictions.begin()+i);
					targetsModel.erase(targetsModel.begin()+i);
					noOfTarget.erase(noOfTarget.begin()+i);
				}
		
			}
	}
}


void targetTrackingFilter::drawTargets(cv::Mat &image,cv::Scalar color, int thickness)
{
	for (int i =0; i<KFs.size();++i)
	{
		std::stringstream s;
		s<<noOfTarget.at(i);
		cv::Point label;
		label.x=KFs.at(i).statePost.at<float>(0);
		label.y=KFs.at(i).statePost.at<float>(1);
		
		cv::Rect target;
		target.width = targetsModel.at(i).cols;
		target.height = targetsModel.at(i).rows;
		target.x=KFs.at(i).statePost.at<float>(0)-target.width/2;
		target.y=KFs.at(i).statePost.at<float>(1)-target.height/2;

		cv::rectangle(image, target, color , thickness); 
		cv::putText(image, s.str(), label,CV_FONT_NORMAL, 0.7, color,thickness );
	}
}