// Standard libraries
#include <iostream>
#include <algorithm>
#include <fstream>

// OpenCV libraries
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

// ArUco libraries
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

// Others
#include "OutputControl.h"

// Namespaces
using namespace cv;
using namespace std;
using namespace aruco;

// Global Variables
float const THRESHOLD_X = 6.;
float const THRESHOLD_Y = 6.;

///////////////////////////////////////////////////////////////
// Function that sorts the markers according to their id	//
/////////////////////////////////////////////////////////////

bool sort_markers(const Marker &a, const Marker &b) 
{ 
	return ((a.id < b.id));
}


///////////////////////
// Main function	//
/////////////////////

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

	int frameCount = 0;
	stringstream frameNumber;
	OutputControl option;
	
	FileStorage markers_centers("markers_centers.yml", FileStorage::WRITE);
	FileStorage markers_corners("markers_corners.yml", FileStorage::WRITE);
	
	// The first line of the file contains the markers ids
	Mat ids = (Mat_<int> (1,10) <<10,20,30,40,50,60,70,80,90,100);
	markers_centers << "ids" << ids;
	markers_corners << "ids" << ids;
	
	// Markerdetector initialization
	MarkerDetector MDetector;
	MDetector.setThresholdParams(THRESHOLD_X,THRESHOLD_Y);
	vector<Marker> Markers;
	
	Mat frame;
	while(true)
	{
		// Acquire new frame
		capture >> frame;
		
		// End when video finishes
		if (frame.empty())
			break;
		
		// Detect markers
		MDetector.detect(frame,Markers);
		sort(Markers.begin(),Markers.end(),sort_markers);
		
		Mat centersMatrix(2,10,CV_32F, Scalar::all(-1.));
		Mat cornersMatrix(8,10,CV_32F, Scalar::all(-1.));
		
		// Put the centers in a matrix
		int k =0;
		for(int j=0; j<10; ++j)
			if(k<Markers.size() && Markers[k].id==(j+1)*10)
			{
				centersMatrix.at<float>(0,j)=Markers[k].getCenter().x;
				centersMatrix.at<float>(1,j)=Markers[k].getCenter().y;
				k++;
			}
			
		// Put the corners in a matrix	
		for(int i=0;i<4;i++)
		{
			int k=0;
			for(int j=0; j<10; ++j)
			if(k<Markers.size() && Markers[k].id==(j+1)*10)
			{
				cornersMatrix.at<float>(0+(2*i),j)=Markers[k].at(i).x;
				cornersMatrix.at<float>(1+(2*i),j)=Markers[k].at(i).y;
				k++;
			}
		}

		// Draw markers
		for(int i =0;i<Markers.size();i++)
			Markers[i].draw(frame,Scalar(0,0,225),8);
		
		// Write the center and the corners information in the file
		frameNumber << "frame" << ++frameCount;
		markers_centers << frameNumber.str() << centersMatrix;
		markers_corners << frameNumber.str() << cornersMatrix;
		frameNumber.str("");
		
		//Show treshholded image
		namedWindow("Thresholded Image",0);
		resizeWindow("Thresholded Image", 600,380);
		imshow("Thresholded Image",MDetector.getThresholdedImage());
		
		// Show video with markers
		namedWindow("Marked Image",0);
		resizeWindow("Marked Image", 600,380);
		imshow("Marked Image",frame);
		
		
		// Program control
		char c = waitKey(1000/fps);
		option.pauseProgram(c);
		option.screenshot(c, frame);
		if(option.quitProgram(c))
			break;
	}
	
	// Add the framecount to be able to loop in other programs
	markers_centers << "frameCount" << frameCount;
	markers_corners << "frameCount" << frameCount;
	
	markers_centers.release();
	markers_corners.release();
	
	return 0;
}
