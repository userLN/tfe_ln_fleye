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
string dataline;

void opticalflowArrows (Mat frame, vector<uchar> status, vector<Point2f>  keypoints1, vector<Point2f> keypoints2)
{
	// Source : Stavens_opencv_optical_flow
	int scale = 7;
	CvScalar color = CV_RGB(255,0,0);
		for(int i = 0; i < status.size(); i++)
		{
			// Skip if no correspoendance found
			if ( status.at(i) == 0 ) 
				continue;
			
			// Points that will be used to draw the lines
			Point2f p,q; 
			p.x = (int) keypoints1.at(i).x;
			p.y = (int) keypoints1.at(i).y;
			q.x = (int) keypoints2.at(i).x;
			q.y = (int) keypoints2.at(i).y;
			
			double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse = sqrt( pow((p.y - q.y),2) + pow((p.x - q.x),2) );
			
			// Stretch the arrows to actually see them
			q.x = (int) (p.x - scale * hypotenuse * cos(angle));
			q.y = (int) (p.y - scale * hypotenuse * sin(angle));
			
			// Draw on frame1
			arrowedLine( frame, q, p, color, 2, 8, 0, 0.35);
		}
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
    
    // Open the datafiles
	FileStorage markersCenter("markers_centers.yml", FileStorage::READ);
	FileStorage markersCorners("markers_corners.yml", FileStorage::READ);
	if(! markersCenter.isOpened() || ! markersCorners.isOpened())
	{
		cerr <<"Error opening data files !" <<endl;
		return -1;
	}
	
	//Feature detector initialization
	Ptr<FeatureDetector> detector = new FastFeatureDetector(50);
	
// 	Ptr<FeatureDetector> detector = new GoodFeaturesToTrackDetector(200);
// 	detector->set("useHarrisDetector",true);
// 	detector->set("minDistance",10.);
	
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
	Point2f centers[10], corners[10][4];
	
	// Parameters
	int bestPoints = 200;
	
	// Variables initialization for the screenshot option
	string filename;
	stringstream ss;
	int o=0;	
	
	Mat centersMatrix, cornersMatrix;
	markersCenter ["frame1"] >> centersMatrix;
	markersCorners ["frame1"] >> cornersMatrix;
	for(int i=0; i<10; ++i)
		if(!(cornersMatrix.at<float>(0,i)<0 || cornersMatrix.at<float>(1,i)<0))
		{
			
			vector < vector<Point> > contour;
			contour.push_back(vector<Point>());
			for(int j=0; j<4 ; j++)
				{
					contour[0].push_back(Point (cornersMatrix.at<float>((2*j),i),cornersMatrix.at<float>((2*j)+1,i)));
				}
			cout << cornersMatrix;	
			cout << contour[0] << endl;
			int thickness = max(abs(contour[0].at(0).x -contour[0].at(1).x),abs(contour[0].at(0).y - contour[0].at(2).y))+10;
			drawContours(mask,contour,-1,Scalar::all(0),thickness);
		}
	
			
	// Do the first frame out of the main loop
	capture >> frame1;
	detector->detect(frame1, keypoints, mask);
	KeyPointsFilter::retainBest(keypoints,bestPoints);
	KeyPoint::convert(keypoints, kpt1);
	
	stringstream frameNumber;
	int frameCount = (int)  markersCenter["frameCount"];
	
	for(int frame = 2; frame <= frameCount; frame++ )
	{
		// Acquire new frame
		capture >> frame2;
		
		// End when video finishes
		if (frame2.empty() )
			break;
		
		// Mask creation
		mask= Mat(capture.get(CV_CAP_PROP_FRAME_HEIGHT),capture.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1,Scalar::all(225));
		
		// Retrive information about the center and the corners of the markers
			
		
		for(int frame = 2; frame <= frameCount; frame++ )
		{
			frameNumber << "frame" << frame;
			markersCenter [frameNumber.str()] >> centersMatrix;
			markersCorners [frameNumber.str()] >> cornersMatrix;
			frameNumber.str("");
		}
		
		
		// Create the mask to avoid taking feature in the markers
		for(int i=0; i<10; ++i)
			if(!(cornersMatrix.at<float>(0,i)<0 || cornersMatrix.at<float>(1,i)<0))
			{
				
				vector < vector<Point> > contour;
				contour.push_back(vector<Point>());
				for(int j=0; j<4 ; j++)
					{
						contour[0].push_back((Point2f) (cornersMatrix.at<float>((2*j),i),cornersMatrix.at<float>((2*j)+1,i)));
					}
				
				int thickness = max(abs(contour[0].at(0).x -contour[0].at(1).x),abs(contour[0].at(0).y - contour[0].at(2).y))+10;
				drawContours(mask,contour,-1,Scalar::all(0),thickness);
			}
		
		// Detecting keypoints
		detector->detect(frame2, keypoints,mask);
		KeyPointsFilter::retainBest(keypoints,bestPoints);
		KeyPoint::convert(keypoints, kpt2);
		kpt_tmp = kpt2;
		
		
		// Compute opticalflow
		calcOpticalFlowPyrLK(frame1,frame2,kpt1,kpt2,status,err);
		cout << (float) count(status.begin(),status.end(),1)/status.size() <<endl;
		
		// Showing arrow flow
		frame2.copyTo(frame_flow_out);
		opticalflowArrows (frame_flow_out, status, kpt1, kpt2);
		
		
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

