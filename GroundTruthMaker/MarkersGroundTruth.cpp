// Standard libraries
#include <iostream>

// OpenCV libraries
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

// Others
#include "OutputControl.h"

// Namespaces
using namespace std;
using namespace cv;

static void onMouse( int event, int x, int y, int, void* ptr )
{
    if( event != EVENT_LBUTTONDOWN )
        return;
	
	Point*p = (Point*)ptr;
    p->x = x;
    p->y = y;
}


int main(int argc, char **argv) 
{
   	// Open the video file
	VideoCapture capture("marker_video_2.mp4");
	
	if (!capture.isOpened())
	{
		cerr << "Failed to open the video file" << endl;
		return -1;
	}

	int frameCount = 0;
	stringstream frameNumber,idNumbre;
	OutputControl option;
	
	FileStorage gtMarkersCenters("GT_markers_centers.yml", FileStorage::WRITE);
	FileStorage markersCenters("_markers_centers.yml", FileStorage::READ);
	
	// The first line of the file contains the markers ids
	Mat ids = (Mat_<int> (1,10) <<10,20,30,40,50,60,70,80,90,100);
	gtMarkersCenters << "ids" << ids;
	
	Mat frame, frame_tmp;
	char c;
	while(! option.quitProgram(c))
	{
		// Acquire new frame
		capture >> frame;
		
		// End when video finishes
		if (frame.empty())
			break;
		
		Mat centersMatrix(2,10,CV_32F, Scalar::all(-1.));
		frameNumber << "frame" << ++frameCount;
		markersCenters [frameNumber.str()] >> centersMatrix;
		
		for(int i=0; i<ids.cols; ++i)
		{
			frame.copyTo(frame_tmp); 
			
			idNumbre << ids.at<int>(i);
			putText(frame_tmp, frameNumber.str(), Point (50,50), 1, 4 ,Scalar::all(250), 5 );
			putText(frame_tmp,"ID "+idNumbre.str() , Point (50,100), 1, 4 ,Scalar::all(250), 5 );
			
			
			// Show frame on wich the center of the markers can be detected
			namedWindow("Set centers",0);
			resizeWindow("Set centers", 1000,700);
			imshow("Set centers",frame_tmp);
			
			
			Mat marker = imread(idNumbre.str()+".png");
			circle(marker, Point (marker.rows/2, marker.cols/2), 4, Scalar (0,225,0), 200);
			// Show marker
			namedWindow("Marker",0);
			resizeWindow("Marker", 100,100);
			imshow("Marker",marker);
			Point p = Point (centersMatrix.at<float>(0,i), centersMatrix.at<float>(1,i));
			
			setMouseCallback("Set centers",onMouse, & p); 
			while(true)
			{
				Mat frame_tmp_tmp;
				frame_tmp.copyTo(frame_tmp_tmp);
				circle(frame_tmp_tmp,p, 3, Scalar (250,0,250), 3);
				imshow("Set centers",frame_tmp_tmp);
				c = waitKey(10);
				if (c == 32 || c == 27)
					break;
				if (c == 'r' || c == 'R')
					p = Point (centersMatrix.at<float>(0,i), centersMatrix.at<float>(1,i));
				
			}
			
			cout << p <<endl;
			centersMatrix.at<float>(0,i)=p.x;
			centersMatrix.at<float>(1,i)=p.y;
			
			
			idNumbre.str("");
			
		}
		// Program control
		c = waitKey(10);
		option.quitProgram(c);

		
		
		
		
		// Write the center and the corners information in the file
		
		gtMarkersCenters << frameNumber.str() << centersMatrix;
		frameNumber.str("");
	}
	gtMarkersCenters << "frameCount" << frameCount ;
	gtMarkersCenters.release();
	
    return 0;
}
