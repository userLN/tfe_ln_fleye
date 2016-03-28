// Standard libraries
#include <iostream>
#include <fstream>
#include <math.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

// Header
#include "opticalFlow.h"




void initFeatureDetector(cv::Ptr<cv::FeatureDetector> &detector , std::string detectorName)
{
	// From the faster to the slower detector
	
	if (detectorName == "FAST")
	{
		detector= new cv::FastFeatureDetector();
		detector->set("threshold", 30);
		detector->set("nonmaxSuppression",true);
	}
	
	else if (detectorName == "ORB")
	{
		// Need to have REFRESH_MODE_ALL = 0;
		detector = new cv::OrbFeatureDetector();
	}	
	
	else if (detectorName == "BRISK")
	{
		detector = new cv::BRISK();
	}
	
	else if (detectorName == "HARRIS")
	{
		detector = new cv::GoodFeaturesToTrackDetector(200);
		detector->set("qualityLevel",0.01);
		detector->set("minDistance",10);	
		detector->set("useHarrisDetector",true);
		detector->set("k",0.04);
	}
	
	else if (detectorName == "STAR")
	{
		detector = new cv::StarFeatureDetector();
		detector->set("maxSize",45);
		detector->set("responseThreshold",30);
		detector->set("lineThresholdProjected",10);
		detector->set("lineThresholdBinarized",8);
		detector->set("suppressNonmaxSize",5);  
	}
	
		else if (detectorName == "SIFT")
	{
		detector = new cv::SiftFeatureDetector();
	}
	
	else if (detectorName == "SURF")
	{
		detector = new cv::SurfFeatureDetector();
	}
	
	else if (detectorName == "MSER")
	{
		detector = new cv::MserFeatureDetector();
        detector->set("delta",5);
		detector->set("minArea",60);
		detector->set("maxArea",14400);
		detector->set("maxVariation",0.7);
		detector->set("minDiversity",0.2);
		detector->set("maxEvolution",200);
		detector->set("areaThreshold",1.01);
		detector->set("minMargin",0.003);
		detector->set("edgeBlurSize",5);
	}
}



// Update of the mask of the markers at each frame
void maskUpdate(cv::Mat cornersMatrix, cv::Mat &mask )
{
	for(int i=0; i<cornersMatrix.cols; ++i)
		if(!(cornersMatrix.at<float>(0,i)<0 || cornersMatrix.at<float>(1,i)<0))
		{
			std::vector < std::vector<cv::Point> > contour;
			contour.push_back(std::vector<cv::Point>());
			
			for(int j=0; j<(int) cornersMatrix.rows/2 ; j++)
					contour[0].push_back(cv::Point (cornersMatrix.at<float>((2*j),i),cornersMatrix.at<float>((2*j)+1,i)));
				
			int thickness = std::max(std::abs(contour[0].at(0).x -contour[0].at(1).x),std::abs(contour[0].at(0).y - contour[0].at(2).y))+25;
			cv::drawContours(mask,contour,-1,cv::Scalar::all(0),thickness);	
		}
}


void maskUpdate(cv::Mat centersMatrix, cv::Mat &mask, int mask_size )
{
	int x[4] = {-1, +1, +1, -1};
	int y[4] = {+1, +1, -1, -1};
	for(int i=0; i<centersMatrix.cols; ++i)
		if(!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0))
		{
			std::vector < std::vector<cv::Point> > contour;
			contour.push_back(std::vector<cv::Point>());
			
			for(int j=0; j< 4 ; j++)
					contour[0].push_back(cv::Point (centersMatrix.at<float>(0,i)+(mask_size*x[j]),centersMatrix.at<float>(1,i)+(mask_size*y[j])));
				
			int thickness = std::max(std::abs(contour[0].at(0).x -contour[0].at(1).x),std::abs(contour[0].at(0).y - contour[0].at(2).y))+25;
			cv::drawContours(mask,contour,-1,cv::Scalar::all(0),thickness);	
		}
}



// Clean the feature according to their correspondance	
std::vector<cv::Point2f> cleanFeatures(std::vector<cv::Point2f> kpt, std::vector<uchar> status)
{
	std::vector<cv::Point2f> kptclean;
	for(int i=0 ; i < kpt.size(); ++i)
		if(status.at(i) == 1)
			kptclean.push_back(kpt.at(i));
		
	return kptclean;
}


// Compute the translation matrix only	
void findTranslation(cv::Mat &translationMatrix, std::vector<cv::Point2f>  keypoints1, std::vector<cv::Point2f> keypoints2)
{
	translationMatrix = (cv::Mat_<float> (1,2) ) ;
	float x = 0;
	float y = 0;
	for(int i = 0; i < keypoints1.size(); i++)
	{
		x+=keypoints2.at(i).x-keypoints1.at(i).x;
		y+=keypoints2.at(i).y-keypoints1.at(i).y;
	}
	translationMatrix.at<float>(0) = x/keypoints1.size();
	translationMatrix.at<float>(1) = y/keypoints1.size();
}


// Constructor
opticalFlow::opticalFlow(std::string detectorName, int cornerBackgroundSize, int bestPointToKeep, float updateRate, bool refreshAllMode)
{
	this->detectorName = detectorName;
	this->cornerBackgroundSize = cornerBackgroundSize;
	this->bestPointToKeep = bestPointToKeep;
	this->updateRate = updateRate;
	this->refreshAllMode = refreshAllMode;
	
	initFeatureDetector(detector , detectorName);
	

}


void opticalFlow::FeatureDetection(cv::Mat frame, cv::Mat mask)
{
	detector->detect(frame, keypoints, mask);
	cv::KeyPointsFilter::retainBest(keypoints,bestPointToKeep);
	cv::KeyPoint::convert(keypoints, kptNext);
	kpt = kptNext;
	
	if(kptPrev.empty())
		kptPrev = kptNext;
}


void opticalFlow::findProjectiveMatrix(cv::Mat framePrev, cv::Mat frame, cv::Mat &homography)
{
	// Compute opticalflow
		cv::calcOpticalFlowPyrLK(framePrev,frame,kptPrev,kptNext,status,err);
		
		// Keep only the feature with status == 1
		kptPrev = cleanFeatures(kptPrev,status);
		kptNext = cleanFeatures(kptNext, status);
		
		homography = cv::findHomography(kptPrev,kptNext,CV_RANSAC,3,hStatus);
}

void opticalFlow::keyPointsUpdate(cv::Mat frame, cv::Mat mask)
{
	if( (float) kptNext.size()/bestPointToKeep < updateRate)
	{
		if (refreshAllMode)
			kptPrev = kpt;
		
		else
		{
			kptPrev = kptNext;
			detector->detect(frame, keypoints,mask);
			cv::KeyPointsFilter::retainBest(keypoints,(int) bestPointToKeep*(1-updateRate));
			cv::KeyPoint::convert(keypoints, kpt);
			kptPrev.insert(kptPrev.end(),kpt.begin(),kpt.end());
		}
		
		std::cout<< "REFRESH ! " << std::endl;
	}
	else
		kptPrev = kptNext;
}


void opticalFlow::markersMaskUpdate(cv::Mat matrix , cv::Mat &mask)
{
	if(cornerBackgroundSize <0)
		maskUpdate(matrix,mask);
	else
		maskUpdate(matrix, mask, cornerBackgroundSize );
}


void opticalFlow::drawOpticalflowArrows (cv::Mat image, int scale, cv::Scalar color)
{
	// Source : Stavens_opencv_optical_flow
		for(int i = 0; i < hStatus.rows; i++)
		{
			// Skip if no correspoendance found
			if ( hStatus.at<bool>(i) == 0 ) 
				continue;
			
			// Points and their properties that will be used to draw the lines
			cv::Point2f p,q; 
			p.x = (int) kptPrev.at(i).x;
			p.y = (int) kptPrev.at(i).y;
			q.x = (int) kptNext.at(i).x;
			q.y = (int) kptNext.at(i).y;
			
			double angle = std::atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse = std::sqrt( std::pow((p.y - q.y),2) + std::pow((p.x - q.x),2) );
			
			// Scale the arrows to actually see them
			q.x = (int) (p.x - scale * hypotenuse * std::cos(angle));
			q.y = (int) (p.y - scale * hypotenuse * std::sin(angle));
			
			// Draw on the image
			cv::arrowedLine(image, q, p, color, 2, 8, 0, 0.35);
		}
}


void opticalFlow::drawDots(cv::Mat centersMatrix, cv::Mat& image, cv::Scalar color, int thickness)
{
	for(int i=0; i<centersMatrix.cols; ++i)
		if(!(centersMatrix.at<float>(0,i)<0 || centersMatrix.at<float>(1,i)<0))
			cv::circle(image,cv::Point(centersMatrix.at<float>(0,i),centersMatrix.at<float>(1,i)),8,color,thickness);
}


// Compute the RMS error	
float opticalFlow::rmsError(cv::Mat coordinate1 , cv::Mat coordinate2)
{
	float x_square = std::pow(coordinate2.at<float>(0) - coordinate1.at<float>(0),2);
	float y_square = std::pow(coordinate2.at<float>(1) - coordinate1.at<float>(1),2);
	return std::sqrt(x_square+y_square);
}
