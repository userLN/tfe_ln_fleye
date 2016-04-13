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
#include "markersDetector.h"

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
	
	// Variables initialization
	Mat frame;
	outputControl control;
	control.outputControlHelp(1,1,1);
	
	markersDetector foundMarkers;
	
	foundMarkers.createMarkersFiles();
	
	while(true)
	{
		// Acquire new frame
		capture >> frame;
		
		// End when video finishes
		if (frame.empty())
			break;
		
		foundMarkers.findMarkers(frame);
		
		
		foundMarkers.writeMarkersFiles();
		foundMarkers.drawMarkers(frame);
		foundMarkers.newFrame();
		
		// Control of the output
		char c = waitKey(1000/fps);
		control.showVideo("Output", frame, (int) height/3, (int) width/3 );
		if(control.quitProgram(c))
			break;
		control.pauseProgram(c);
		control.screenshot(c, frame);
	}
	
	foundMarkers.closeMarkersFiles();
    
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

