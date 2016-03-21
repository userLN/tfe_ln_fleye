#pragma once
#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"

const char ESC_KEY = 27;

class OutputControl
{
private:
	std::string filename;
	std::stringstream ss;
	int NoScreenshot;
		
public:
	OutputControl();
	
	bool quitProgram (char c);
	void pauseProgram (char c);
	void screenshot(char c , cv::Mat image);
	void help();
	
};

