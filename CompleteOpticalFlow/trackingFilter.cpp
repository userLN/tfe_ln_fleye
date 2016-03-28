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
#include "trackingFilter.h"

// Global variables
int MAX_MISSING_DATA;

// Initialization of the Kalman filter	
void initKalman(cv::KalmanFilter *KF, float x , float y, float dt,float dv)
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


trackingFilter::trackingFilter(float x, float y, float velocityFactor,float accelerationFactor, int maxMissingData)
{
	missingData = 0;
	MAX_MISSING_DATA = maxMissingData;
	
	dt = velocityFactor;
	dv = accelerationFactor;
	
	initKalman(&KF, x, y, dt, dv);
}


cv::Mat trackingFilter::applyFilter(float x, float y)
{
	transitionMatrixUpdate(&KF, missingData/5 + dt , dv);
	cv::Mat coordinates = KalmanModelUpdate(&KF, x, y, missingData);
	return (cv::Mat_<float> (1,2) << coordinates.at<float>(0) , coordinates.at<float>(1));
}


// Update the popsition of the corners according to the center position	

cv::Mat trackingFilter::updateRelativePosition(float x, float y,float relativeX, float relativeY, float &deltaX, float &deltaY)
{
	if((missingData < MAX_MISSING_DATA) && (relativeX<0 ||relativeY<0))
	{
		relativeX = deltaX + x;
		relativeY = deltaY + y;
	}
	else if ((relativeX>=0 && relativeY >=0))
	{
		deltaX = relativeX - x;
		deltaY = relativeY - y;
	}
	return (cv::Mat_<float> (1,2) << relativeX , relativeY);
}
