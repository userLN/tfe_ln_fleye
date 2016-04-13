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
	// Variables initialization
	markersDetector foundMarkers;
	foundMarkers.createMarkersFiles("markers_centers_test_new.yml", "markers_corners_test_new.yml");
	
	FileStorage centers("markers_centers_test.yml", FileStorage::READ);
	FileStorage corners("markers_corners_test.yml", FileStorage::READ);
	foundMarkers.readMarkersFiles(centers, corners);
	
	int maxFrameCount;
	centers ["frameCount"] >> maxFrameCount;
	
	Mat deltaC = Mat::zeros(8,10, CV_32F);
	
	vector <trackingFilter> KFS;
	for (int i=0; i<foundMarkers.getCentersMatrix().cols; ++i)
	{
		trackingFilter KF(foundMarkers.getCentersMatrix().at<float>(0,i),foundMarkers.getCentersMatrix().at<float>(1,i));
		KFS.push_back(KF);	
	}
	foundMarkers.writeMarkersFiles();
	
	while(true)
	{
		
		// End when video finishes
		if (foundMarkers.newFrame()>maxFrameCount)
			break;
		
		foundMarkers.readMarkersFiles(centers, corners);
		
		for (int i =0; i<foundMarkers.getCentersMatrix().cols; ++i)
		{
			Mat newCoordinates = KFS.at(i).applyFilter(foundMarkers.getCentersMatrix().at<float>(0,i),
													   foundMarkers.getCentersMatrix().at<float>(1,i));
			foundMarkers.getCentersMatrix().at<float>(0,i) = newCoordinates.at<float>(0);
			foundMarkers.getCentersMatrix().at<float>(1,i) = newCoordinates.at<float>(1);
			
			
			for(int j=0; j<foundMarkers.getCornersMatrix().rows/2 ; ++j)
			{
				Mat updatedCorner =  KFS.at(i).updateRelativePosition(foundMarkers.getCentersMatrix().at<float>(0,i),
																	  foundMarkers.getCentersMatrix().at<float>(1,i), 
																	  foundMarkers.getCornersMatrix().at<float>((2*j),i),
																	  foundMarkers.getCornersMatrix().at<float>((2*j)+1,i),
																	  deltaC.at<float>((2*j),i) , 
																	  deltaC.at<float>((2*j)+1,i));
				foundMarkers.getCornersMatrix().at<float>((2*j),i) = updatedCorner.at<float>(0);
				foundMarkers.getCornersMatrix().at<float>((2*j)+1,i) = updatedCorner.at<float>(1);
			}
		}
		
		
		
		foundMarkers.writeMarkersFiles();
		
		
		
		// Control of the output
// 		char c = waitKey(1000/fps);
// 		control.showVideo("Output", frame, (int) height/3, (int) width/3 );
// 		if(control.quitProgram(c))
// 			break;
		
		
		
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

