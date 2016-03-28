#pragma once

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


class markersDetector
{
private:
	cv::Mat ids;
	int frameCount;
	
	aruco::MarkerDetector MDetector;
	std::vector<aruco::Marker> Markers;
	
	cv::Mat centersMatrix;
	cv::Mat cornersMatrix;
	
	cv::FileStorage markersCenters;
	cv::FileStorage markersCorners;
	
public:
	markersDetector(float thresholdX = 4., float thresholdY = 4., cv::Mat ids = (cv::Mat_<int> (1,10) <<10,20,30,40,50,60,70,80,90,100));
	
	void findMarkers(cv::Mat &image);
	void setMarkersPosition(cv::Mat &image);
	void helpSetMarkersPosition();
	
	void drawMarkers(cv::Mat &image, cv::Scalar color = cv::Scalar(0,0,225) , int tickness = 8);
	
	void setCentersMatrix(cv::Mat &centersMatrix);
	void setCornersMatrix(cv::Mat &cornersMatrix);
	
	cv::Mat getCentersMatrix();
	cv::Mat getCornersMatrix();
	
	void createMarkersFiles(std::string markersCenterFilename = "centers.yml" , std::string markersCornersFilename = "corners.yml");
	void readMarkersFiles(cv::FileStorage markersCenters, cv::FileStorage markersCorners = NULL);
	void writeMarkersFiles();
	void closeMarkersFiles();
	int newFrame();
};