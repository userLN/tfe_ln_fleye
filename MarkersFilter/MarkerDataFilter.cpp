/*
 * Author:	Hélène Loozen 
 * Date:	2016
 * 
 * << MarkerDataFilter >> fill the missing ArUco marker position thanks to a simple Kalman filter (position -- speed [-- acceleration])
 * 
 * This program works alongside two others : << CameraMotion.cpp >> and << MarkersDetector.cpp >>
 * 
 * How to use :
 * 1. Choose a video containing ArUco markers
 * 2. Use << MarkersDetector >> on the chosen video
 * 3. Use << MarkerDataFilter >> on the output YAML files of << MarkersDetector >> 
 * 4. Use << CameraMotion >> with the output YAML files of << MarkerDataFilter >>
 * 
 */


// Standard libraries
#include <iostream>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"

// Namespaces
using namespace cv;
using namespace std;

// Global parametres
int const MAX_MISSING_DATA = 20;

//////////////////////////////////////////
// Initialization of the Kalman filter	
/////////////////////////////////////////

void initKalman (KalmanFilter *KF, float x , float y, float dt ,float dv=-1.);
void initKalman (KalmanFilter *KF, float x , float y, float dt,float dv)
{
	if ( dv<0 )
	{
		// KalmanFilter 2D traker with position and constant velocity
		KF->init(4,2,0);
		KF->transitionMatrix = *(Mat_<float>(4, 4) << 
		1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1);
	}
	
	else
	{
		// KalmanFilter 2D traker with position, velocity and constant acceleration
		KF->init(6,2,0);
		KF->transitionMatrix = *(Mat_<float>(6, 6) <<
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
	
	setIdentity(KF->measurementMatrix);
	setIdentity(KF->processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF->errorCovPost, Scalar::all(.1));	
}


//////////////////////////////////////////////////////////////////
// Update of the transition matrix to createmore complex models	
/////////////////////////////////////////////////////////////////

void transitionMatrixUpdate (KalmanFilter *KF, float dt)
{
		KF->transitionMatrix = *(Mat_<float>(4, 4) << 
		1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1);
}

void transitionMatrixUpdate (KalmanFilter *KF, float dt,float dv)
{
		KF->transitionMatrix = *(Mat_<float>(6, 6) <<
		1, 0, dt, 0, 0.5*dv, 0,
		0, 1, 0, dt, 0, 0.5*dv, 
		0, 0, 1, 0, dv, 0, 
		0, 0, 0, 1, 0, dv,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);
}

//////////////////////////////////////////
// Kalman prediction and model update	
/////////////////////////////////////////

Mat KalmanModelUpdate (KalmanFilter *KF, float x , float y, int &missingData)
{
	Mat predictMatrix = KF->predict();
	
	if(missingData < MAX_MISSING_DATA && (x<0 || y<0))
	{
			x = predictMatrix.at<float>(0);
			y = predictMatrix.at<float>(1);
			missingData ++;
	}
	
	else if ((x>=0 && y>=0))
	{		
		Mat correctMatrix = KF->correct((Mat_<float>(2,1)<<x,y));
		KF->statePost.at<float>(0) = x;
		KF->statePost.at<float>(1) = y;
		missingData = 0;
	}
	return (Mat_<float> (1,2) << x, y);
}


//////////////////////////////////////////////////////////////////////////
// Update the popsition of the corners according to the center position	
/////////////////////////////////////////////////////////////////////////

Mat updateCornerPosition(Mat center, float x, float y, float &deltaX, float &deltaY, int missingData)
{
	if(missingData < MAX_MISSING_DATA && (x<0 || y<0))
	{
		x = deltaX + center.at<float>(0);
		y = deltaY + center.at<float>(1);
	}
	else if ((x>=0 && y>=0))
	{
		deltaX = x - center.at<float>(0);
		deltaY = y - center.at<float>(1);
	}
	return (Mat_<float> (1,2) << x, y);
}


//////////////////////
// Main function	
/////////////////////

int main(int argc, char **argv) 
{
    // Open the datafiles
    FileStorage markersCenter("markers_centers.yml", FileStorage::READ);
	FileStorage markersCorners("markers_corners.yml", FileStorage::READ);
	if(! markersCenter.isOpened() || ! markersCorners.isOpened())
	{
		cerr <<"Error opening data files !" <<endl;
		return -1;
	}
	
	// Creation of the filtered datafiles
	FileStorage filteredMarkersCenter("filtered_markers_centers.yml", FileStorage::WRITE);
	FileStorage filteredMarkersCorners("filtered_markers_corners.yml", FileStorage::WRITE);
	
	// Creation of the working matrices and other variable
	Mat centersMatrix, cornersMatrix;
	int frameCount = (int) markersCenter["frameCount"];
	stringstream frameNumber;
	Mat missingData = Mat::zeros(1,10, CV_32S);
	Mat deltaC = Mat::zeros(8,10, CV_32F);
	
	
	// First lines of the YAML file
	Mat ids;
	markersCenter["ids"]>> ids;
	filteredMarkersCenter << "ids" << ids;
	filteredMarkersCorners << "ids" << ids;
	
	markersCenter ["frame1"] >> centersMatrix;
	markersCorners ["frame1"] >> cornersMatrix;
	filteredMarkersCenter << "frame1" << centersMatrix;
	filteredMarkersCorners << "frame1" << cornersMatrix;
		
	

	// Kalman Filters initialization (One Kalman per id)

	vector <KalmanFilter> KFS;
	for (int i=0; i<centersMatrix.cols; ++i)
	{
		KalmanFilter KF;
		initKalman (&KF, centersMatrix.at<float>(0,i), centersMatrix.at<float>(1,i), 1);
		KFS.push_back(KF);
	}

	// Main loop that goes through all the frames
	for(int frame = 2; frame <= frameCount ; ++frame)
	{
		frameNumber << "frame" << frame;
		markersCenter [frameNumber.str()] >> centersMatrix;
		markersCorners [frameNumber.str()] >> cornersMatrix;
		
		// Loop that update all the center of the markers
		for (int i =0; i<centersMatrix.cols; ++i)
		{
			transitionMatrixUpdate(&(KFS.at(i)), (missingData.at<int>(i)/5 +1));
			Mat updatedCenter = KalmanModelUpdate(&(KFS.at(i)), centersMatrix.at<float>(0,i), centersMatrix.at<float>(1,i), missingData.at<int>(i));
			centersMatrix.at<float>(0,i) = updatedCenter.at<float>(0);
			centersMatrix.at<float>(1,i) = updatedCenter.at<float>(1);
			
			for(int j=0; j<cornersMatrix.rows/2 ; ++j)
			{
				Mat updatedCorner = updateCornerPosition( updatedCenter, cornersMatrix.at<float>((2*j),i), cornersMatrix.at<float>((2*j)+1,i), deltaC.at<float>((2*j),i), deltaC.at<float>((2*j)+1,i), missingData.at<int>(i));
				cornersMatrix.at<float>((2*j),i) = updatedCorner.at<float>(0);
				cornersMatrix.at<float>((2*j)+1,i) = updatedCorner.at<float>(1);
			}
		}
		
		filteredMarkersCenter << frameNumber.str() << centersMatrix;
		filteredMarkersCorners << frameNumber.str() << cornersMatrix;
		frameNumber.str("");
	}
	
	markersCenter.release();
	markersCorners.release();
	
	filteredMarkersCenter << "frameCount" << frameCount;
	filteredMarkersCorners << "frameCount" << frameCount;
	filteredMarkersCenter.release();
	filteredMarkersCorners.release();
	
    return 0;
}
