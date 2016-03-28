/*
 * Author:	Hélène Loozen 
 * Date:	2016
 * 
 */

// Standard libraries
#include <iostream>
#include <algorithm>
#include <fstream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// ArUco libraries
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

//Header
#include "markersDetector.h"

// Global Variables


// Sorts markers according to their id
bool sortMarkers(const aruco::Marker &a, const aruco::Marker &b) 
{ 
	return ((a.id < b.id));
}


void onMouse( int event, int x, int y, int, void* ptr )
{
    if( event != cv::EVENT_LBUTTONDOWN )
        return;
	
	cv::Point*p = (cv::Point*)ptr;
    p->x = x;
    p->y = y;
}



// Constructor
markersDetector::markersDetector(float thresholdX, float thresholdY,cv::Mat ids)
{
	this->ids = ids;
	MDetector.setThresholdParams(thresholdX,thresholdY);
	centersMatrix = cv::Mat(2,ids.cols,CV_32F, cv::Scalar::all(-1.));
	cornersMatrix = cv::Mat(8,ids.cols,CV_32F, cv::Scalar::all(-1.));
	frameCount = 1;
}

// Find the markers automaticaly 
void markersDetector::findMarkers(cv::Mat &image)
{
	centersMatrix = cv::Mat(2,ids.cols,CV_32F, cv::Scalar::all(-1.));
	cornersMatrix = cv::Mat(8,ids.cols,CV_32F, cv::Scalar::all(-1.));
	MDetector.detect(image,Markers);
	sort(Markers.begin(),Markers.end(),sortMarkers);
	
	// Put the centers in a matrix
	int k=0;
	for(int j=0; j<centersMatrix.cols; ++j)
		if(k<Markers.size() && Markers[k].id==ids.at<int>(j))
		{
			centersMatrix.at<float>(0,j)=Markers[k].getCenter().x;
			centersMatrix.at<float>(1,j)=Markers[k].getCenter().y;
			k++;
		}

	// Put the corners in a matrix	
	for(int i=0;i<cornersMatrix.rows/2;i++)
	{
		int k=0;
		for(int j=0; j<cornersMatrix.cols; ++j)
			if(k<Markers.size() && Markers[k].id==ids.at<int>(j))
			{
				cornersMatrix.at<float>(0+(2*i),j)=Markers[k].at(i).x;
				cornersMatrix.at<float>(1+(2*i),j)=Markers[k].at(i).y;
				k++;
			}
	}
}

// Do the groundtruth of the markers
void markersDetector::setMarkersPosition(cv::Mat& image)
{
	stringstream frameNumber,idNumbre;
	frameNumber << "frame" << frameCount;
	char c;
		for(int i=0; i<ids.cols; ++i)
		{
			cv::Mat workImage;
			image.copyTo(workImage); 
			
			idNumbre << ids.at<int>(i);
			cv::putText(workImage, frameNumber.str(), cv::Point (50,50), 1, 4 ,cv::Scalar::all(250), 5 );
			cv::putText(workImage,"ID "+idNumbre.str() , cv::Point (50,100), 1, 4 ,cv::Scalar::all(250), 5 );
			
			
			// Show frame on wich the center of the markers can be detected
			cv::namedWindow("Set Markers Position",0);
			cv::resizeWindow("Set Markers Position", 1000,700);
			cv::imshow("Set Markers Position",workImage);
			
			
			cv::Mat marker = cv::imread("mkImages/"+idNumbre.str()+".png");
			if (marker.empty())
			{
				std::cerr << "\nFail to load the markers images ! \n " << std::endl;
				exit(-1);
 			}
			circle(marker, cv::Point (marker.rows/2, marker.cols/2), 4,cv::Scalar (0,225,0), 200);
			
			// Show marker
			cv::namedWindow("Marker",0);
			cv::resizeWindow("Marker", 100,100);
			cv::imshow("Marker",marker);
			cv::Point p = cv::Point (centersMatrix.at<float>(0,i), centersMatrix.at<float>(1,i));
			
			setMouseCallback("Set Markers Position",onMouse, & p); 
			while(true)
			{
				cv::Mat imageUpdate;
				workImage.copyTo(imageUpdate);
				cv::circle(imageUpdate,p, 3, cv::Scalar (250,0,250), 3);
				cv::imshow("Set Markers Position",imageUpdate);
				
				c = cv::waitKey(100);
				if (c == 32)
					break;
				
				if (c == 'd' || c == 'D')
					p = cv::Point (-1 , -1);
				
				if (c == 'r' || c == 'R')
					p = cv::Point (centersMatrix.at<float>(0,i), centersMatrix.at<float>(1,i));
				
				if ( c == 'n'|| c == 'N'|| c == 27 || c == 'q' || c == 'Q' )
				{
					centersMatrix.at<float>(0,i)=p.x;
					centersMatrix.at<float>(1,i)=p.y;
					return;
				}
			}
			
			cout << p <<endl;
			centersMatrix.at<float>(0,i)=p.x;
			centersMatrix.at<float>(1,i)=p.y;
			
			idNumbre.str("");
			
		}
}

// draw the markers on the screen
void  markersDetector::drawMarkers(cv::Mat &image, cv::Scalar color, int tickness)
{
	for(int i =0;i<Markers.size();i++)
		Markers[i].draw(image,color,tickness);
}


// Create the yml file to store the markers
void markersDetector::createMarkersFiles(std::string markersCenterFilename, std::string markersCornersFilename)
{
	markersCenters.open(markersCenterFilename, cv::FileStorage::WRITE);
	markersCorners.open(markersCornersFilename, cv::FileStorage::WRITE);
	markersCenters << "ids" << ids;
	markersCorners << "ids" << ids;
}


// Write the value of the marker in a file
void markersDetector::writeMarkersFiles()
{
	std::stringstream frameNumber;
	frameNumber << "frame" << frameCount;
	markersCenters << frameNumber.str() << centersMatrix;
	markersCorners << frameNumber.str() << cornersMatrix;
}

// Read the value of the marker in a file
void markersDetector::readMarkersFiles(cv::FileStorage markersCenters, cv::FileStorage markersCorners)
{
	std::stringstream frameNumber;
	frameNumber << "frame" << frameCount;
	markersCenters[frameNumber.str()] >> centersMatrix;
	
	if (markersCorners.isOpened())
		markersCorners[frameNumber.str()] >> cornersMatrix;
}


// Close the file where the marker are stored
void  markersDetector::closeMarkersFiles()
{
	markersCenters << "frameCount" << frameCount-1;
	markersCorners << "frameCount" << frameCount-1;
	
	markersCenters.release();
	markersCorners.release();
}


void markersDetector::helpSetMarkersPosition()
{
	std::cout
	<< "\nUse the left button of the mouse to chose position of the marker" << std::endl
	<< "Press SPACE BAR to go to the next marker" << std::endl
	<< "Press N to go to the next frame" << std::endl
	<< "Press D to delete the marker" << std::endl
	<< "Press R to reset the marker position \n" << std::endl;
}

void markersDetector::setCentersMatrix(cv::Mat &centersMatrix)
{
	centersMatrix.copyTo(this->centersMatrix) ;
}

cv::Mat markersDetector::getCentersMatrix()
{
	return centersMatrix ;
}


void markersDetector::setCornersMatrix(cv::Mat &cornersMatrix)
{
	cornersMatrix.copyTo(this->cornersMatrix) ;
}


cv::Mat markersDetector::getCornersMatrix()
{
	return cornersMatrix ;
}

int markersDetector::newFrame()
{
	return ++frameCount;
}
