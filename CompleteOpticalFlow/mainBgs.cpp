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
#include "Vibe.h"

// Namespaces
using namespace cv;
using namespace std;


// My functions
void help();

// Global parametres
float const DEFAULT_FPS = 60;



void BgsPostprocess(const cv::Mat &src, cv::Mat &dst)
{
	// Implemented by Michael Fonder
	
	Mat tmp;

	Mat struct_el = getStructuringElement(MORPH_ELLIPSE, Point(7, 23));

	//Noise reduction step
	medianBlur(src, tmp, 3);
	medianBlur(tmp, tmp, 3);	

	//Fill holes in foreground
	morphologyEx(tmp, dst, MORPH_CLOSE, struct_el);
}


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
	
	::BackgroundSubtractor *bgsVibe = new Vibe;
	
	// Variables initialization
	Mat frame,frameGray, framePrev;
	ticToc time;
	
	outputControl control;
	control.outputControlHelp(1,0,1);
	
	
	// Detect the feature for the fisrt frame
	capture >>framePrev;
	
	while(true)
	{
		time.tic();
		// Acquire new frame
		capture >> frame;
		cvtColor(frame, frameGray, CV_BGR2GRAY);
		
		
		// End when video finishes
		if (frame.empty())
			break;
		
		
		Mat bgsMask = bgsVibe->process(frameGray);
		//BgsPostprocess(bgsMask,bgsMask);
		
		frame.copyTo(framePrev);
		
		
		
		// Control of the output
		char c = waitKey(1000/fps);
		if (! bgsMask.empty())
		{
			control.showVideo("Bgs mask", bgsMask, (int) height/3, (int) width/3 );
		}
		

		control.showVideo("Output", frame, (int) height/3, (int) width/3 );
		if(control.quitProgram(c))
			break;
		
		control.screenshot(c, frame);
		control.screenshot(c, bgsMask);
		
		cout << (double) width*height/time.toc() << " pixels/second" << endl;
		
		
	}
	delete bgsVibe;
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

