// Standard libraries
#include <iostream>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

// Namespaces
using namespace cv;
using namespace std;

const char ESC_KEY = 27;

int main(int argc, char **argv) 
{
	// Open the video file
	VideoCapture capture("marker_video_1.mp4");
	
	if (!capture.isOpened())
	{
		cerr << "Failed to open the video file" << endl;
		return -1;
    }
    
    // Open the datafile
    ifstream data_position("markers_position.txt");
	ifstream data_corners("markers_corners.txt");
	if(! data_position.is_open() || ! data_corners.is_open())
	{
		cerr <<"Error opening data files !" <<endl;
		return -1;
	}
	
	//Feature detector initialization
	Ptr<FeatureDetector> detector = new FastFeatureDetector(10);
// 	Ptr<FeatureDetector> detector = new MserFeatureDetector();
// 	detector->set("minDiversity",0.7);
// 	detector->set("maxVariation",0.2);
	
    // Mask creation
	Mat mask(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1,Scalar::all(225));
	//mask(ROI).setTo(Scalar::all(0));
	
	
	// Variables initialization
	double fps = capture.get(CV_CAP_PROP_FPS);
	if (fps!=fps)
	{
		fps=30;	waitKey(0);
	}
	vector<KeyPoint> keypoints;
	vector<Point2f> kpt1,kpt2,kpt_tmp;
	vector<uchar> status;
	vector<float> err;
	Mat frame1, frame2;
	Mat frame_keypoints_out, frame_flow_out;
	
	// Parameters
	int bestPoints = 200;
	int thickness = 2;
	int scale = 7;
	CvScalar color = CV_RGB(255,0,0);
	
	// Variables initialization for the screenshot option
	string filename;
	stringstream ss;
	int o=0;
	
	
	
// 	vector<Point2f> corners[10];
// 	//Point corners1[4], corners2[4], corners3[4], corners4[4], corners5[4], corners6[4] , corners7[4], corners8[4], corners9[4], corners10[4];
// 	for(int i=0 ; i<4 ; ++i)
// 	{
// 		for(int j=0 ; j<10 ; ++j)
// 			data_corners >> corners[j].push_back();
// 		//data_corners >> corners1[i]>> corners2[i]>> corners3[i]>> corners4[i]>> corners5[i]>> corners6[i] >> corners7[i]>> corners8[i]>> corners9[i]>> corners10[i];
// 	}
// 	
// 	cout<< corners[3] << endl;
// 	
	// Do the first frame out of the main loop
	capture >> frame1;
	detector->detect(frame1, keypoints, mask);
	KeyPointsFilter::retainBest(keypoints,bestPoints);
	KeyPoint::convert(keypoints, kpt1);
	
	while(true)
	{
		// Acquire new frame
		capture >> frame2;
		
		// End when video finishes
		if (frame2.empty())
			break;
		
		vector< vector<Point> > corners;
		corners.push_back(vector<Point>());
		
		
		// Detecting keypoints
		detector->detect(frame2, keypoints,mask);
		KeyPointsFilter::retainBest(keypoints,bestPoints);
		KeyPoint::convert(keypoints, kpt2);
		kpt_tmp = kpt2;
		
// 		// Show video with keypoints
// 		drawKeypoints(frame2, keypoints, frame_keypoints_out,Scalar::all(0));
// 		namedWindow("Keypoints Image",0);
// 		resizeWindow("Keypoints Image", 600,380);
// 		imshow("Keypoints Image", frame_keypoints_out);
		
		
		// Compute opticalflow
		calcOpticalFlowPyrLK(frame1,frame2,kpt1,kpt2,status,err);
		cout << (float) count(status.begin(),status.end(),1)/status.size() <<endl;
		
		// Showing arrow flow
		// Source : Stavens_opencv_optical_flow
		frame2.copyTo(frame_flow_out);
		for(int i = 0; i < status.size(); i++)
		{
			// Skip if no correspoendance found
			if ( status.at(i) == 0 ) 
				continue;
			
			// Points that will be used to draw the lines
			Point2f p,q; 
			p.x = (int) kpt1.at(i).x;
			p.y = (int) kpt1.at(i).y;
			q.x = (int) kpt2.at(i).x;
			q.y = (int) kpt2.at(i).y;
			
			double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse = sqrt( pow((p.y - q.y),2) + pow((p.x - q.x),2) );
			
			// Stretch the arrows to actually see them
			q.x = (int) (p.x - scale * hypotenuse * cos(angle));
			q.y = (int) (p.y - scale * hypotenuse * sin(angle));
			
			// Draw on frame1
			arrowedLine( frame_flow_out, q, p, color, thickness, 8, 0, 0.35);
		}
		
		
		// Show video with opticalflow
		namedWindow("opticalflow Image",0);
		resizeWindow("opticalflow Image", 600,380);
		imshow("opticalflow Image", frame_flow_out);
		
		
		frame2.copyTo(frame1);
		
		// If the percentage of correspondance drops under 85% refresh the features tracked
		if((float) count(status.begin(),status.end(),1)/status.size() < (float) 0.86)
		{
			kpt1 = kpt_tmp;
			cout<< "REFRESH ! " << endl;
		}
		else
			kpt1 = kpt2;
		
		// End the program at user will
		char c = (char)waitKey(1);
		if( c  == ESC_KEY || c == 'q' || c == 'Q' )
			break;
		if( c == 'p' || c == 'P' )
		{
			ss<<"screenshot"<<++o<<".png";
			filename = ss.str();
			ss.str("");
			imwrite(filename, frame_flow_out);
		}
	}
    
    return 0;
}
