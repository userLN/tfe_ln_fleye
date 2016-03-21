/*
 * Author:	Hélène Loozen 
 * Date:	2016
 * 
 * << CameraMotion >> finds the homography transformation for each frame of a video
 * 
 * This program works alongside two others : << MarkersDetector.cpp >> and << MarkerDataFilter.cpp >>
 * 
 * How to use :
 * 1. Choose a video containing ArUco markers
 * 2. Use << MarkersDetector >> on the chosen video
 * 3. Use << MarkerDataFilter >> on the output YAML files of << MarkersDetector >> 
 * 4. Use << CameraMotion >> with the output YAML files of << MarkerDataFilter >>
 * 
 * 
 * Note:
 * If the program crashes during execution, consider modifying the global parametres
 * as well as modifying the detector parametres in << initFeatureDetector >> 
 * 
 */



// Standard libraries
#include <iostream>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

// Others
#include "OutputControl.h"
#include "TicToc.h"

// Namespaces
using namespace cv;
using namespace std;

// Global parametres
float const DEFAULT_FPS = 60;


//////////////////////////////////////
// Create red arrows on the frames	
/////////////////////////////////////

void opticalflowArrows (Mat image, vector<Point2f>  keypoints1, vector<Point2f> keypoints2, int scale = 7, CvScalar color=CV_RGB(255,0,0))
{
	// Source : Stavens_opencv_optical_flow
		for(int i = 0; i < keypoints1.size(); i++)
		{
			// Points and their properties that will be used to draw the lines
			Point2f p,q; 
			p.x = (int) keypoints1.at(i).x;
			p.y = (int) keypoints1.at(i).y;
			q.x = (int) keypoints2.at(i).x;
			q.y = (int) keypoints2.at(i).y;
			
			double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse = sqrt( pow((p.y - q.y),2) + pow((p.x - q.x),2) );
			
			// Scale the arrows to actually see them
			q.x = (int) (p.x - scale * hypotenuse * cos(angle));
			q.y = (int) (p.y - scale * hypotenuse * sin(angle));
			
			// Draw on the image
			arrowedLine(image, q, p, color, 2, 8, 0, 0.35);
		}
}


//////////////////////////////////////////////
// Used to draw the center of the markers
/////////////////////////////////////////////

void drawDots(Mat centersMatrix, Mat &image, Scalar color = Scalar(0,200,0) , int thickness = 15);
void drawDots(Mat centersMatrix, Mat &image, Scalar color, int thickness)
{
	for(int i=0; i<centersMatrix.cols; ++i)
		if(!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0))
			circle(image,Point(centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)),8,color,thickness);
}


//////////////////////////////////////////////////////////
// Clean the feature according to their correspondance	
/////////////////////////////////////////////////////////

vector<Point2f> cleanFeatures(vector<Point2f> kpt, vector<uchar> status)
{
	vector<Point2f> kptclean;
	for(int i=0 ; i < kpt.size(); ++i)
		if(status.at(i) == 1)
			kptclean.push_back(kpt.at(i));
		
	return kptclean;
}

Mat findTranslation(vector<Point2f>  keypoints1, vector<Point2f> keypoints2)
{
	Mat translationMatrix = (Mat_<float> (1,2) ) ;
	float x = 0;
	float y = 0;
	for(int i = 0; i < keypoints1.size(); i++)
	{
		x+=keypoints2.at(i).x-keypoints1.at(i).x;
		y+=keypoints2.at(i).y-keypoints1.at(i).y;
	}
	translationMatrix.at<float>(0) = x/keypoints1.size();
	translationMatrix.at<float>(1) = y/keypoints1.size();
	
	return translationMatrix;
}


////////////////////
// Main function
///////////////////
int main(int argc, char **argv) 
{
	// Open the video file
	VideoCapture capture("marker_video_2.mp4");
	
	if (!capture.isOpened())
	{
		cerr << "Failed to open the video file" << endl;
		return -1;
    }
    
	double fps = capture.get(CV_CAP_PROP_FPS);
	if (fps!=fps)
		fps=DEFAULT_FPS;
    
	
    // Open the datafiles
	FileStorage markersCenter("_markers_centers.yml", FileStorage::READ);
	if(! markersCenter.isOpened())
	{
		cerr <<"Error opening data file !" <<endl;
		return -1;
	}
	
	FileStorage translation("camera_translation_data.yml", FileStorage::WRITE);
	FileStorage affine("camera_affine_data.yml", FileStorage::WRITE);
	FileStorage projective("camera_projective_data.yml", FileStorage::WRITE);
	FileStorage foreground("foreground_translation_data.yml", FileStorage::WRITE);
	
	// Variables initialization
	vector<KeyPoint> keypoints;
	vector<Point2f> kpt1,kpt2,kpt_tmp;
	vector<uchar> status;
	vector<float> err;
	Mat frame;
	Mat ids;
	Mat foundHomography, perspectiveIm, hStatus;
	OutputControl option;
	
			
	// Do the first frame out of the main loop
	capture >> frame;
	
	markersCenter["ids"] >> ids;
	
	stringstream frameNumber;
	int frameCount = (int)  markersCenter["frameCount"];
	
	for(int Noframe = 2; Noframe <= frameCount; Noframe++ )
	{
		Mat centersMatrix, centersMatrix_1;
		// Acquire new frame
		capture >> frame;
		
		// End when video finishes
		if (frame.empty() )
			break;
		
		// Retrive information about the center and the corners of the markers
		frameNumber.str("");
		frameNumber << "frame" << Noframe-1;
		markersCenter [frameNumber.str()] >> centersMatrix_1;
		
		frameNumber.str("");
		frameNumber << "frame" << Noframe;
		markersCenter [frameNumber.str()] >> centersMatrix;
		kpt2.clear();
		kpt1.clear();
		
		
		for(int i=0; i<centersMatrix.cols; ++i)
			if((ids.at<int>(i) != 50) && (!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0)))
			{
				kpt2.push_back(Point (centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)));
				if(!(centersMatrix_1.at<float>(0,i)<0 || centersMatrix_1.at<float>(1,i)<0))
					kpt1.push_back(Point (centersMatrix_1.at<float>(0,i),centersMatrix_1.at<float>(1,i)));
				else
					kpt1.push_back(Point (centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)));
			}
			
		
// 		// Fancy output view
// 		// Draw the center of the mask and the arrows of the flow vthen show the video
// 		drawDots(centersMatrix, frame);
// 		opticalflowArrows (frame,kpt1, kpt2);
// 		namedWindow("opticalflow Image",0);
// 		resizeWindow("opticalflow Image", 600,380);
// 		imshow("opticalflow Image", frame);
// 		char c = waitKey(1000/fps);
		
		if (kpt2.size() < 3)
		{
			foundHomography.release();
			projective << frameNumber.str() << foundHomography;
			affine << frameNumber.str() << foundHomography;
			
			foundHomography = findTranslation(kpt1,kpt2);
			translation << frameNumber.str() << foundHomography;
		}
		
		else if (kpt2.size() == 3)
		{
			foundHomography.release();
			projective << frameNumber.str() << foundHomography;
			
			foundHomography = getAffineTransform(kpt1,kpt2);
			affine << frameNumber.str() << (Mat_<float> (1,2) << foundHomography.at<double>(0,2), foundHomography.at<double>(1,2));
			
			foundHomography = findTranslation(kpt1,kpt2);
			translation << frameNumber.str() << foundHomography;
			
		}
		
		else if (kpt2.size() > 3)
		{
			foundHomography = findHomography(kpt1,kpt2,CV_RANSAC,3);
			projective << frameNumber.str() << (Mat_<float> (1,2) << foundHomography.at<double>(0,2), foundHomography.at<double>(1,2));
			
			foundHomography = findTranslation(kpt1,kpt2);
			translation << frameNumber.str() << foundHomography;
			
			kpt2.clear();
			kpt1.clear();
			for(int i=0; i<centersMatrix.cols; ++i)
			if((ids.at<int>(i) == 30 || ids.at<int>(i) == 70 || ids.at<int>(i) == 80) && (!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0)))
			{
				kpt2.push_back(Point (centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)));
				if(!(centersMatrix_1.at<float>(0,i)<0 || centersMatrix_1.at<float>(1,i)<0))
					kpt1.push_back(Point (centersMatrix_1.at<float>(0,i),centersMatrix_1.at<float>(1,i)));
				else
					kpt1.push_back(Point (centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)));
			}
			
			foundHomography = getAffineTransform(kpt1,kpt2);
			affine << frameNumber.str() <<(Mat_<float> (1,2) << foundHomography.at<double>(0,2), foundHomography.at<double>(1,2));
			
		}
		
		kpt2.clear();
		kpt1.clear();
		
		
		for(int i=0; i<centersMatrix.cols; ++i)
			if((ids.at<int>(i) == 50) && (!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0)))
			{
				kpt2.push_back(Point (centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)));
				if(!(centersMatrix_1.at<float>(0,i)<0 || centersMatrix_1.at<float>(1,i)<0))
					kpt1.push_back(Point (centersMatrix_1.at<float>(0,i),centersMatrix_1.at<float>(1,i)));
				else
					kpt1.push_back(Point (centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)));
			}
		
		foundHomography = findTranslation(kpt1,kpt2);
		if (foundHomography.at<float>(0) != foundHomography.at<float>(0))
			foundHomography.release();
		foreground << frameNumber.str() << foundHomography;
		
		
// 		// Program control
// 		
// 		if(option.quitProgram(c))
// 			break;
// 		option.pauseProgram(c);
// 		option.screenshot(c, frame);
	}
	
	translation << "frameCount" << frameCount;
	affine << "frameCount" << frameCount;
	projective << "frameCount" << frameCount;
	foreground << "frameCount" << frameCount;
	
	
	// close the data files
	markersCenter.release();
	translation.release();
	affine.release();
	projective.release();
	foreground.release();
	
	
    return 0;
}