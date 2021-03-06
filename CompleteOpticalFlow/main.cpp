/*
 * Author:	Hélène Loozen 
 * Date:	2016
 * 
 */

// Standard libraries
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

// Others
#include "outputControl.h"
#include "ticToc.h"
#include "markersDetector.h"
#include "trackingFilter.h"
#include "opticalFlow.h"

// Namespaces
using namespace cv;
using namespace std;


// My functions
void help();

// Global parametres
float const DEFAULT_FPS = 60;


// Main funtion
int main(int argc, char **argv) 
{
	if (argc != 2)
	{
		help();
		return 0;
	}
    
    // Open the video or images sequence
    string sequence = argv[1];
    VideoCapture capture(sequence);
	
	if (!capture.isOpened())
	{
		cerr << "\nFailed to open the video file or image sequence \n" << endl;
		return -1;
    }
    
    // Get the video properties
    double width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    double fps = capture.get(CV_CAP_PROP_FPS);
	if (fps!=fps)
		fps=DEFAULT_FPS;
	
	
	// Load markers
	markersDetector markers;
	FileStorage centers("_markers_centers.yml", FileStorage::READ);
	markers.readMarkersFiles(centers);
	
	// Variables initialization
	Mat frame, frameGray,framePrev;
	ticToc time;
	
	outputControl control;
	control.outputControlHelp(1,0,0);
	
	opticalFlow opticalFlow("FAST", 200, 0.86, true);
	opticalFlow.detector->set("threshold", 30);
	//opticalFlow.detector->set(3, FastFeatureDetector::TYPE_9_16);
	
	// Mask creation and update
	Mat mask(height,width, CV_8UC1,Scalar::all(225));
	opticalFlow.markersMaskUpdate(markers.getCentersMatrix(), mask, 50);
	
	// Detect the feature for the fisrt frame
	capture >>frame;
	cvtColor(frame, frameGray, CV_BGR2GRAY);
	opticalFlow.FeatureDetection(frameGray, mask);
	frameGray.copyTo(framePrev);
	
	while(true)
	{
		time.tic();
		// Acquire new frame
		capture >> frame;
		cvtColor(frame, frameGray, CV_BGR2GRAY);
		markers.newFrame();
		markers.readMarkersFiles(centers);
		
		// End when video finishes
		if (frame.empty())
			break;
		
		Mat mask(height,width, CV_8UC1,Scalar::all(225));
		opticalFlow.markersMaskUpdate(markers.getCentersMatrix(), mask, 50);
		opticalFlow.FeatureDetection(frameGray, mask);
		Mat foundHomography, deltaX, deltaY;
		
		
		opticalFlow.findProjectiveMatrix(framePrev, frameGray,foundHomography);
		opticalFlow.pixelDisplacment(foundHomography, deltaX, deltaY, width, height );
		
	
		
		frameGray.copyTo(framePrev);
		
		opticalFlow.drawDots(markers.getCentersMatrix(),frameGray);
		opticalFlow.drawOpticalflowArrows(frameGray);
		
		
		opticalFlow.keyPointsUpdate(frameGray, mask);
		
		// Control of the output
		char c = waitKey(1000/fps);
		control.showVideo("Output", frameGray, (int) height/3, (int) width/3 );
		if(control.quitProgram(c))
			break;
		
		cout << (double) mask.rows*mask.cols/time.toc() << " pixels/second" << endl;
		
		
	}
	
    centers.release();
    return 0;
}






// Help function
void help()
{
	cout
	<< "\nUsage: ./program <video file or image sequence>" << endl
    << "Examples: " << endl
    << "Passing a video file : ./program myvideo.avi" << endl
    << "Passing an image sequence : ./program image%03d.jpg  (if the images are numbered with 3 digits) \n" << endl;	
}

