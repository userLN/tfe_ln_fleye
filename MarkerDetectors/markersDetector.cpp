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
	
	
	// Files creation that saves the markers positions over time
	ofstream output1, output2;
	output1.open("markers_position.txt");
	output2.open("markers_corners.txt");
	for(int i=10; i<101; i+=10)
	{
		output1<<i <<"\t";
		output2<<i <<"\t";
	}
	output1<<"\n";
	output2<<"\n";
	
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
		
		// Draw markers
		for(int i =0;i<Markers.size();i++)
			Markers[i].draw(frame,Scalar(0,0,225),8);
		
		// Write central position of the markers in "markers_position.txt"
		int k=0;
		for(int j=10; j<101; j+=10)
		{
			if(k<Markers.size() && Markers[k].id==j)
				output1<< Markers[k++].getCenter() <<"\t";
			
			else if (Markers[k].id % 10 != 0)
			{
				output1<<"\t";
				k++;
			}
			
			else
				output1<<"\t";
		}
		output1 << "\n";
		
		
		// Write corners position of the markers in "markers_corners.txt"
		for(int l=0;l<4;l++)
		{
			int k=0;
			for(int j=10; j<101; j+=10)
			{
				if(k<Markers.size() && Markers[k].id==j)
					output2<< Markers[k++].at(l) <<"\t";

				else if (Markers[k].id % 10 != 0)
				{
					output2<<"\t";
					k++;
				}
				else
					output2<<"\t";
			}
			output2 << "\n";
		}
		
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
	output1.close();
	return 0;
}
