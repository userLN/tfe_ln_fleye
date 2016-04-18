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
#include "targetTrackingFilter.h"
#include "Vibe.h"

// Namespaces
using namespace cv;
using namespace std;


// My functions
void help();

// Global parametres
float const DEFAULT_FPS = 120;

void blobsFinder(Mat &image, vector<Rect> &blobs)
{
	vector< vector< Point> > contours;  
		findContours(image,contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		
		for (int i =0; i<contours.size();++i)
		{
			Rect newBlob = boundingRect(contours.at(i));
			if (newBlob.area() > image.total()/80)
				blobs.push_back(newBlob);
		}
}

void drawBlobs(Mat &image, vector<Rect> &blobs, Scalar color = CV_RGB(255,0,0), int thickness = 4)
{
	
	for (int i =0; i<blobs.size();++i)
		rectangle(image, blobs.at(i), color , thickness); 
}

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
	
	
	
	// Variables initialization
	Mat frame,frameGray;
	::BackgroundSubtractor *bgsVibe = new Vibe;
	targetTrackingFilter trackingFilters;
	
	outputControl control;
	control.outputControlHelp(1,1,1);
	
	
	while(true)
	{
		// Acquire new frame
		capture >> frame;
		
		cvtColor(frame, frameGray, CV_BGR2GRAY);
		
		
		// End when video finishes
		if (frame.empty())
			break;
		
		Mat motion(height,width, CV_8UC1,Scalar::all(225));
		
		Mat bgsMask = bgsVibe->process(frameGray,motion,motion );
		BgsPostprocess(bgsMask,bgsMask);
		
		if (! bgsMask.empty())
		{
			control.showVideo("Bgs mask", bgsMask, (int) 420, (int) 640);
		}

		vector<Rect> blobs;
		blobsFinder(bgsMask,blobs);		
		//drawBlobs(frame,blobs);
		
		trackingFilters.applyFilter(frame, blobs);
		trackingFilters.drawTargets(frame);
		
		
		// Control of the output
		char c = waitKey(1000/fps);
		control.showVideo("Output", frame, (int) 420, (int) 640 );
		if(control.quitProgram(c))
			break;
		
		control.screenshot(c, frame);
		control.screenshot(c, bgsMask);
		
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

