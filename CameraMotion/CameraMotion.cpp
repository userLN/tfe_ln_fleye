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
#include <math.h>

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
int const CORNER_BACK = 55;
int const CORNER_FORE = 110;
string const DETECTOR_NAME = "FAST";
int const BEST_POINTS = 200;
float const PERCENT = 0.86;
bool const REFRESH_MODE_ALL = 1;
bool const HAS_CORNERS = 0;


//////////////////////////////////////
// Create red arrows on the frames	
/////////////////////////////////////

void opticalflowArrows (Mat image, vector<uchar> status, vector<Point2f>  keypoints1, vector<Point2f> keypoints2, int scale = 7, CvScalar color=CV_RGB(255,0,0));
void opticalflowArrows (Mat image, vector<uchar> status, vector<Point2f>  keypoints1, vector<Point2f> keypoints2, int scale, CvScalar color)
{
	// Source : Stavens_opencv_optical_flow
		for(int i = 0; i < status.size(); i++)
		{
			// Skip if no correspoendance found
			if ( status.at(i) == 0 ) 
				continue;
			
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


//////////////////////////////////////////////////////////////////////////
// Choose between multiple detectors and set parametres in consequence	
/////////////////////////////////////////////////////////////////////////

void initFeatureDetector(Ptr<FeatureDetector> &detector , string detectorName)
{
	// From the faster to the slower detector
	
	if (detectorName == "FAST")
	{
		detector= new FastFeatureDetector();
		detector->set("threshold", 30);
		detector->set("nonmaxSuppression",true);
	}
	
	else if (detectorName == "ORB")
	{
		// Need to have REFRESH_MODE_ALL = 0;
		detector = new OrbFeatureDetector();
	}	
	
	else if (detectorName == "BRISK")
	{
		detector = new BRISK();
	}
	
	else if (detectorName == "HARRIS")
	{
		detector = new GoodFeaturesToTrackDetector(200);
		detector->set("qualityLevel",0.01);
		detector->set("minDistance",10);	
		detector->set("useHarrisDetector",true);
		detector->set("k",0.04);
	}
	
	else if (detectorName == "STAR")
	{
		detector = new StarFeatureDetector();
		detector->set("maxSize",45);
		detector->set("responseThreshold",30);
		detector->set("lineThresholdProjected",10);
		detector->set("lineThresholdBinarized",8);
		detector->set("suppressNonmaxSize",5);  
	}
	
		else if (detectorName == "SIFT")
	{
		detector = new SiftFeatureDetector();
	}
	
	else if (detectorName == "SURF")
	{
		detector = new SurfFeatureDetector();
	}
	
	else if (detectorName == "MSER")
	{
		detector = new MserFeatureDetector();
        detector->set("delta",5);
		detector->set("minArea",60);
		detector->set("maxArea",14400);
		detector->set("maxVariation",0.7);
		detector->set("minDiversity",0.2);
		detector->set("maxEvolution",200);
		detector->set("areaThreshold",1.01);
		detector->set("minMargin",0.003);
		detector->set("edgeBlurSize",5);
	}
}


//////////////////////////////////////////////////////
// Update of the mask of the markers at each frame
/////////////////////////////////////////////////////

void maskUpdate(Mat cornersMatrix, Mat &mask )
{
	for(int i=0; i<cornersMatrix.cols; ++i)
		if(!(cornersMatrix.at<float>(0,i)<0 || cornersMatrix.at<float>(1,i)<0))
		{
			vector < vector<Point> > contour;
			contour.push_back(vector<Point>());
			
			for(int j=0; j<(int) cornersMatrix.rows/2 ; j++)
					contour[0].push_back(Point (cornersMatrix.at<float>((2*j),i),cornersMatrix.at<float>((2*j)+1,i)));
				
			int thickness = max(abs(contour[0].at(0).x -contour[0].at(1).x),abs(contour[0].at(0).y - contour[0].at(2).y))+25;
			drawContours(mask,contour,-1,Scalar::all(0),thickness);	
		}
}


void maskUpdate(Mat centersMatrix, Mat &mask, int mask_size )
{
	int x[4] = {-1, +1, +1, -1};
	int y[4] = {+1, +1, -1, -1};
	for(int i=0; i<centersMatrix.cols; ++i)
		if(!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0))
		{
			vector < vector<Point> > contour;
			contour.push_back(vector<Point>());
			
			for(int j=0; j< 4 ; j++)
					contour[0].push_back(Point (centersMatrix.at<float>(0,i)+(mask_size*x[j]),centersMatrix.at<float>(1,i)+(mask_size*y[j])));
				
			int thickness = max(abs(contour[0].at(0).x -contour[0].at(1).x),abs(contour[0].at(0).y - contour[0].at(2).y))+25;
			drawContours(mask,contour,-1,Scalar::all(0),thickness);	
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

////////////////////////////////////////
// Compute the translation matrix only	
////////////////////////////////////////
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


////////////////////////////
// Compute the RMS error	
////////////////////////////
float rmsError(Mat coordinate1 , Mat coordinate2)
{
	float x_square = pow(coordinate2.at<float>(0) - coordinate1.at<float>(0),2);
	float y_square = pow(coordinate2.at<float>(1) - coordinate1.at<float>(1),2);
	return sqrt(x_square+y_square);
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
	FileStorage markersCorners,backgroundGT,foregroundGT;
	if (HAS_CORNERS)
	{
		markersCorners.open("filtered_markers_corners.yml", FileStorage::READ);
		if(! markersCenter.isOpened() ||! markersCorners.isOpened())
		{
			cerr <<"Error opening data files !" <<endl;
			return -1;
		}
	}
	
	else
	{
		backgroundGT.open("camera_translation_data.yml", FileStorage::READ);
		foregroundGT.open("foreground_translation_data.yml", FileStorage::READ);
		
		if(! markersCenter.isOpened() || ! backgroundGT.isOpened() || ! foregroundGT.isOpened())
		{
			cerr <<"Error opening data files !" <<endl;
			return -1;
		}
	}
	
	//Feature detector initialization
	Ptr<FeatureDetector> detector;
	initFeatureDetector(detector , DETECTOR_NAME);

	// Variables initialization
	vector<KeyPoint> keypoints;
	vector<Point2f> kpt1,kpt2,kpt_tmp;
	vector<uchar> status;
	vector<float> err;
	Mat frame1, frame2;
	Mat foundHomography, perspectiveIm, hStatus;
	OutputControl option;
	TicToc time;
	
	Mat centersMatrix, cornersMatrix;
	markersCenter ["frame1"] >> centersMatrix;
	markersCorners ["frame1"] >> cornersMatrix;
	
	// Mask creation
	Mat mask(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1,Scalar::all(225));
	if(HAS_CORNERS)
		maskUpdate(cornersMatrix,mask);
	else
		maskUpdate(centersMatrix,mask, CORNER_BACK);
	
	// Do the first frame out of the main loop
	capture >> frame1;
	detector->detect(frame1, keypoints, mask);
	KeyPointsFilter::retainBest(keypoints,BEST_POINTS);
	KeyPoint::convert(keypoints, kpt1);
	
	stringstream frameNumber;
	int frameCount = (int)  markersCenter["frameCount"];

	for(int frame = 2; frame <= frameCount; frame++ )
	{
	//	time.tic();
		// Acquire new frame
		capture >> frame2;
		
		// End when video finishes
		if (frame2.empty() )
			break;
		
		// Retrive information about the center and the corners of the markers
		frameNumber.str("");
		frameNumber << "frame" << frame;
		markersCenter [frameNumber.str()] >> centersMatrix;
		markersCorners [frameNumber.str()] >> cornersMatrix;
		
		
		// Mask update
		mask = Mat(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1,Scalar::all(225));
		if(HAS_CORNERS)
			maskUpdate(cornersMatrix,mask);
		else
			maskUpdate(centersMatrix,mask, CORNER_BACK);
		
		// Detecting keypoints
		detector->detect(frame2, keypoints,mask);
		KeyPointsFilter::retainBest(keypoints,BEST_POINTS);
		KeyPoint::convert(keypoints, kpt2);
		kpt_tmp = kpt2;
		
		// Compute opticalflow
		calcOpticalFlowPyrLK(frame1,frame2,kpt1,kpt2,status,err);
		
		// Keep only the feature with status == 1
		kpt1 = cleanFeatures(kpt1,status);
		kpt2 = cleanFeatures(kpt2, status);
		
		//Find Homography
		foundHomography = findHomography(kpt1,kpt2,CV_RANSAC,3,hStatus);
// 		cout << foundHomography << endl;
		
		Mat backGT;
		backgroundGT[frameNumber.str()] >> backGT;
		
		float absError = rmsError(backGT,  (Mat_<float> (1,2) << foundHomography.at<double>(0,2), foundHomography.at<double>(1,2)));
		cout << absError <<endl;
// 		//Create the perspective image
// 		warpPerspective(frame2,perspectiveIm,foundHomography,frame2.size(), CV_INTER_LINEAR | CV_WARP_INVERSE_MAP);
// 		namedWindow("Perspective Image",0);
// 		resizeWindow("Perspective Image", 600,380);
// 		imshow("Perspective Image", perspectiveIm);
		
		frame2.copyTo(frame1);
		
		// Fancy output view
		// Draw the center of the mask and the arrows of the flow vthen show the video
		drawDots(centersMatrix, frame2);
		opticalflowArrows (frame2, hStatus, kpt1, kpt2);
		namedWindow("opticalflow Image",0);
		resizeWindow("opticalflow Image", 600,380);
		imshow("opticalflow Image", frame2);
		
		// If the percentage of correspondance drops under 85% refresh the features tracked
		//cout << (float) kpt2.size()/BEST_POINTS <<endl;
		if( (float) kpt2.size()/BEST_POINTS < PERCENT)
		{
			if (REFRESH_MODE_ALL)
				kpt1 = kpt_tmp;
			
			else
			{
				kpt1 = kpt2;
				detector->detect(frame2, keypoints,mask);
				KeyPointsFilter::retainBest(keypoints,(int) BEST_POINTS*(1-PERCENT));
				KeyPoint::convert(keypoints, kpt_tmp);
				kpt1.insert(kpt1.end(),kpt_tmp.begin(),kpt_tmp.end());
			}
			
			cout<< "REFRESH ! " << endl;
		}
		else
			kpt1 = kpt2;
		
		// Program control
		char c = waitKey(1000/fps);
		if(option.quitProgram(c))
			break;
		option.pauseProgram(c);
		option.screenshot(c, frame2);
//		cout << (double) mask.rows*mask.cols/time.getToc() << " pixels/second" << endl;
// 		time.toc();
	}
	
	// close the data files
	markersCenter.release();
	markersCorners.release();
	
    return 0;
}