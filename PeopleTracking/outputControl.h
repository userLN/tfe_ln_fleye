#pragma once

// Standard libraries
#include <iostream>
#include <fstream>
#include <string.h>

// OpenCV libraries
#include "opencv2/highgui/highgui.hpp"

class outputControl
{
private:
	std::string filename;
	std::stringstream ss;
	int NoScreenshot;
		
public:
	outputControl();
	~outputControl();
	
	void outputControlHelp(bool quit, bool pause, bool screenshot);
	bool quitProgram (char c);
	void pauseProgram (char c);
	void screenshot(char c , cv::Mat &image);
	void showVideo(std::string name, cv::Mat &image, int heigh, int width);
};

