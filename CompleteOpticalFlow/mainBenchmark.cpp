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
#include "opticalFlow.h"
#include "ticToc.h"

// Namespaces
using namespace cv;
using namespace std;


// My functions
void help();

// Global parametres
float const DEFAULT_FPS = 60;


// 

double getOnes(std::vector<uchar> vector)
{
	double output = 0;
	for(int i=0; i< vector.size(); ++i)
		if(vector.at(i) == 1)
			output++;
	
	return output/vector.size();
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
	ticToc time;
	outputControl control;
	
	Mat frame, framePrev;
	std::vector<uchar> status;
	double ExecutionTimeFast1, ExecutionTimeFast2, ExecutionTimeSift, ExecutionTimeSurf, ExecutionTimeOrb1, ExecutionTimeOrb2, ExecutionTimeHarris1, ExecutionTimeHarris2, ExecutionTimeMser = 0;
	int nbFeaturesFast1, nbFeaturesFast2, nbFeaturesSift, nbFeaturesSurf, nbFeaturesOrb1, nbFeaturesOrb2, nbFeaturesHarris1, nbFeaturesHarris2, nbFeaturesMser = 0;
	
	

	opticalFlow FASTfeatures1("FAST");
	opticalFlow FASTfeatures2("FAST");
	FASTfeatures2.detector->set("threshold", 30);
	
	opticalFlow ORBfeatures1("ORB");
	opticalFlow ORBfeatures2("ORB");
	ORBfeatures2.detector->set("nFeatures", 200);
	
	opticalFlow HARRISfeatures1("HARRIS");
	opticalFlow HARRISfeatures2("HARRIS");
	HARRISfeatures2.detector->set("nfeatures",200);

	opticalFlow MSERfeatures("MSER");
	
	opticalFlow SURFfeatures("SURF");
	
	opticalFlow SIFTfeatures("SIFT");
	
	
	int framecount;
	
	for (framecount = 0 ; framecount<50; ++ framecount )
	{
		cout << framecount+1 << endl;
		
		// Acquire new frame
		capture >> frame;
		if (framePrev.empty())
			frame.copyTo(framePrev);
		
		// End when video finishes
		if (frame.empty())
			break;
		
		
		time.tic();
		nbFeaturesFast1 += FASTfeatures1.FeatureDetectorOnly(frame);
		ExecutionTimeFast1 += time.toc();
		
		time.tic();
		nbFeaturesFast2 += FASTfeatures2.FeatureDetectorOnly(frame);
		ExecutionTimeFast2 += time.toc();
		
		time.tic();
		nbFeaturesSift +=SIFTfeatures.FeatureDetectorOnly(frame);
		ExecutionTimeSift += time.toc();
		
		time.tic();
		nbFeaturesSurf += SURFfeatures.FeatureDetectorOnly(frame);
		ExecutionTimeSurf += time.toc();
		
		time.tic();
		nbFeaturesOrb1 += ORBfeatures1.FeatureDetectorOnly(frame);
		ExecutionTimeOrb1 += time.toc();
		
		time.tic();
		nbFeaturesOrb2 += ORBfeatures2.FeatureDetectorOnly(frame);
		ExecutionTimeOrb2 += time.toc();
		
		time.tic();
		nbFeaturesHarris1 += HARRISfeatures1.FeatureDetectorOnly(frame);
		ExecutionTimeHarris1 += time.toc();

		time.tic();
		nbFeaturesHarris2 += HARRISfeatures2.FeatureDetectorOnly(frame);
		ExecutionTimeHarris2 += time.toc();
		
		time.tic();
		nbFeaturesMser += MSERfeatures.FeatureDetectorOnly(frame);
		ExecutionTimeMser += time.toc();

		
		
		// Control of the output
		char c = waitKey(1);
		if(control.quitProgram(c))
			break;
	}
	
	cout << "Relative execution time FAST : "<< ExecutionTimeFast1/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesFast1/framecount <<  endl;
	cout << "Relative execution time FAST (30) : "<< ExecutionTimeFast2/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesFast2/framecount <<  endl;
	cout << "Relative execution time ORB : "<< ExecutionTimeOrb1/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesOrb1/framecount <<  endl;
	cout << "Relative execution time ORB (200) : "<< ExecutionTimeOrb2/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesOrb2/framecount <<  endl;
	cout << "Relative execution time HARRIS : "<< ExecutionTimeHarris1/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesHarris1/framecount <<endl;
	cout << "Relative execution time HARRIS (200) : "<< ExecutionTimeHarris2/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesHarris2/framecount <<endl;
	cout << "Relative execution time SIFT : "<< ExecutionTimeSift/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesSift/framecount <<   endl;
    cout << "Relative execution time SURF : "<< ExecutionTimeSurf/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesSurf/framecount << endl;
    cout << "Relative execution time MSER : "<< ExecutionTimeMser/ExecutionTimeFast1 << " -- Features :"<< nbFeaturesMser/framecount <<  endl;
	
	
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