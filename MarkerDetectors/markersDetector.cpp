// Standard libraries
#include <iostream>
#include <algorithm>
#include <fstream>

// OpenCV libraries
#include "opencv2/opencv.hpp"

// ArUco libraries
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

// Namespaces
using namespace cv;
using namespace std;
using namespace aruco;


// Function that sorts the markers according to their id
bool sort_markers(const Marker &a, const Marker &b)
{
	return ((a.id < b.id));
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
	double fps = 25;
	int o=0;
	
	// Filename initialization for the screenshot option
	string filename;
	stringstream ss;
	
	int frameCount = 0;
	stringstream frameNumber;
	
	
	// Files creation that saves the markers positions over time	
	FileStorage markers_centers("markers_centers.yml", FileStorage::WRITE);
	FileStorage markers_corners("markers_corners.yml", FileStorage::WRITE);
	Mat ids = (Mat_<int> (1,10) <<10,20,30,40,50,60,70,80,90,100);
	
	markers_centers << "ids" << ids;
	markers_corners << "ids" << ids;
	
	// Markerdetector initialization
	MarkerDetector MDetector;
	MDetector.setThresholdParams(6.,6.);
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
		
		int k =0;
		for(int j=0; j<10; ++j)
			if(k<Markers.size() && Markers[k].id==(j+1)*10)
			{
				centersMatrix.at<float>(0,j)=Markers[k].getCenter().x;
				centersMatrix.at<float>(1,j)=Markers[k].getCenter().y;
				k++;
			}
			
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
		
		// Save position od the centers and the corners
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
		
		
		// Keeps the realtime speed of the video
		char c = (char)waitKey(1000/fps);
		
		// End the program at user will
		if( c  == 27 || c == 'q' || c == 'Q' )
			break;
		
		// Saves screenshot when pressing "p"
		else if (c == 'p' || c == 'P')
		{
			ss<<"screenshot"<<++o<<".png";
			filename = ss.str();
			ss.str("");
			imwrite(filename, frame);
		}
	}
	
	markers_centers << "frameCount" << frameCount;
	markers_corners << "frameCount" << frameCount;
	
	markers_centers.release();
	markers_corners.release();
	
	return 0;
}
