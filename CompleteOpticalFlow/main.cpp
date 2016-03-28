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
	Mat frame, framePrev;
	ticToc time;
	
	outputControl control;
	control.outputControlHelp(1,0,0);
	
	opticalFlow opticalFlow("FAST", 55);
	
	// Mask creation and update
	Mat mask(height,width, CV_8UC1,Scalar::all(225));
	opticalFlow.markersMaskUpdate(markers.getCentersMatrix(), mask);
	
	// Detect the feature for the fisrt frame
	capture >>framePrev;
	opticalFlow.FeatureDetection(framePrev, mask);
	
	while(true)
	{
		// Acquire new frame
		capture >> frame;
		markers.newFrame();
		markers.readMarkersFiles(centers);
		
		// End when video finishes
		if (frame.empty())
			break;
		
		Mat mask(height,width, CV_8UC1,Scalar::all(225));
		opticalFlow.markersMaskUpdate(markers.getCentersMatrix(), mask);
		opticalFlow.FeatureDetection(frame, mask);
		Mat foundHomography;
		
		opticalFlow.findProjectiveMatrix(framePrev, frame,foundHomography);
		
		frame.copyTo(framePrev);
		
		opticalFlow.drawDots(markers.getCentersMatrix(),frame);
		opticalFlow.drawOpticalflowArrows(frame);
		
		
		opticalFlow.keyPointsUpdate(frame, mask);
		
		// Control of the output
		char c = waitKey(1000/fps);
		control.showVideo("Output", frame, (int) height/3, (int) width/3 );
		if(control.quitProgram(c))
			break;
		
		
		
		
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

