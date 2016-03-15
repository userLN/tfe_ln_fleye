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

// Namespaces
using namespace cv;
using namespace std;

// Global parametres
int const BEST_POINTS = 200;

///////////////////////////////////////
// Create red arrows on the frames	// 
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

///////////////////////////////////////////////////////////////////////////
// Choose between multiple detectors and set parametres in consequence	//
/////////////////////////////////////////////////////////////////////////

void initFeatureDetector(Ptr<FeatureDetector> &detector , string detectorName)
{
	if (detectorName == "FAST")
		detector= new FastFeatureDetector(50);
	
	else if (detectorName == "HARRIS")
	{
		detector = new GoodFeaturesToTrackDetector(200);
		detector->set("useHarrisDetector",true);
		detector->set("minDistance",10.);	
	}
	else if (detectorName == "MSER")
	{
		detector = new MserFeatureDetector();
		detector->set("minDiversity",0.7);
		detector->set("maxVariation",0.2);
	}
	else if (detectorName == "STAR");
	else if (detectorName == "SIFT");
	else if (detectorName == "SURF");
	else if (detectorName == "ORB");	
	else if (detectorName == "BRISK");
	else if (detectorName == "BLOB");
}

///////////////////////////////////////////////////////
// Update of the mask of the markers at each frame	//
/////////////////////////////////////////////////////

void maskUpdate(Mat cornersMatrix, Mat &mask )
{
	for(int i=0; i<10; ++i)
	if(!(cornersMatrix.at<float>(0,i)<0 || cornersMatrix.at<float>(1,i)<0))
	{
		
		vector < vector<Point> > contour;
		contour.push_back(vector<Point>());
		for(int j=0; j<4 ; j++)
			{
				contour[0].push_back(Point (cornersMatrix.at<float>((2*j),i),cornersMatrix.at<float>((2*j)+1,i)));
			}
		int thickness = max(abs(contour[0].at(0).x -contour[0].at(1).x),abs(contour[0].at(0).y - contour[0].at(2).y))+10;
		drawContours(mask,contour,-1,Scalar::all(0),thickness);	
	}
}

///////////////////////////////////////////////
// Used to draw the center of the markers	//
/////////////////////////////////////////////

void drawDots(Mat centersMatrix, Mat &image, Scalar color = Scalar(0,200,0) , int thickness = 15);
void drawDots(Mat centersMatrix, Mat &image, Scalar color, int thickness)
{
	for(int i=0; i<10; ++i)
		if(!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0))
			circle(image,Point(centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)),8,color,thickness);
}


int main(int argc, char **argv) 
{
	// Open the video file
	VideoCapture capture("marker_video_1.mp4");
	
	if (!capture.isOpened())
	{
		cerr << "Failed to open the video file" << endl;
		return -1;
    }
    
	double fps = capture.get(CV_CAP_PROP_FPS);
	if (fps!=fps)
		fps=60;
    
	
    // Open the datafiles
	FileStorage markersCenter("filtered_markers_centers.yml", FileStorage::READ);
	FileStorage markersCorners("filtered_markers_corners.yml", FileStorage::READ);
	if(! markersCenter.isOpened() || ! markersCorners.isOpened())
	{
		cerr <<"Error opening data files !" <<endl;
		return -1;
	}
	
	//Feature detector initialization
	Ptr<FeatureDetector> detector;
	initFeatureDetector(detector , "FAST");

	// Variables initialization
	vector<KeyPoint> keypoints;
	vector<Point2f> kpt1,kpt2,kpt_tmp;
	vector<uchar> status;
	vector<float> err;
	Mat frame1, frame2;
	OutputControl option;
	
	Mat centersMatrix, cornersMatrix;
	markersCenter ["frame1"] >> centersMatrix;
	markersCorners ["frame1"] >> cornersMatrix;
	
	// Mask creation
	Mat mask(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1,Scalar::all(225));
	maskUpdate(cornersMatrix,mask);
			
	// Do the first frame out of the main loop
	capture >> frame1;
	detector->detect(frame1, keypoints, mask);
	KeyPointsFilter::retainBest(keypoints,BEST_POINTS);
	KeyPoint::convert(keypoints, kpt1);
	
	stringstream frameNumber;
	int frameCount = (int)  markersCorners["frameCount"];
	
	for(int frame = 2; frame <= frameCount; frame++ )
	{
		// Acquire new frame
		capture >> frame2;
		
		// End when video finishes
		if (frame2.empty() )
			break;
		
		// Retrive information about the center and the corners of the markers
		frameNumber << "frame" << frame;
		markersCenter [frameNumber.str()] >> centersMatrix;
		markersCorners [frameNumber.str()] >> cornersMatrix;
		frameNumber.str("");
		
		// Mask update
		mask= Mat(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1,Scalar::all(225));
		maskUpdate(cornersMatrix,mask);
		
		
		// Detecting keypoints
		detector->detect(frame2, keypoints,mask);
		KeyPointsFilter::retainBest(keypoints,BEST_POINTS);
		KeyPoint::convert(keypoints, kpt2);
		kpt_tmp = kpt2;
		
		// Compute opticalflow
		calcOpticalFlowPyrLK(frame1,frame2,kpt1,kpt2,status,err);
		cout << (float) count(status.begin(),status.end(),1)/status.size() <<endl;
		
		frame2.copyTo(frame1);
		
		
		// Fancy output view
		// Draw the center of the mask and the arrows of the flow vthen show the video
		drawDots(centersMatrix, frame2);
		opticalflowArrows (frame2, status, kpt1, kpt2);
		namedWindow("opticalflow Image",0);
		resizeWindow("opticalflow Image", 600,380);
		imshow("opticalflow Image", frame2);
		
		
		// If the percentage of correspondance drops under 85% refresh the features tracked
		if((float) count(status.begin(),status.end(),1)/status.size() < (float) 0.86)
		{
			kpt1 = kpt_tmp;
			cout<< "REFRESH ! " << endl;
		}
		else
			kpt1 = kpt2;
		
		// Program control
		char c = waitKey(1000/fps);
		option.pauseProgram(c);
		option.screenshot(c, frame2);
		if(option.quitProgram(c))
			break;
		
	}
	// close the data files
	markersCenter.release();
	markersCorners.release();
	
    return 0;
}
