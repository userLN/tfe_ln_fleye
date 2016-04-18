// Standard libraries
#include <iostream>
#include <fstream>
#include <string.h>

// OpenCV libraries
#include "opencv2/highgui/highgui.hpp"

// header
#include "outputControl.h"

// Global variables
const char ESC_KEY = 27;


// Constructor
outputControl::outputControl()
{
	NoScreenshot = 0;
}

outputControl::~outputControl(){}


// Print the commands usage 
void outputControl::outputControlHelp(bool quit, bool pause, bool screenshot)
{
	std::cout << "\nKeyboard controls:" << std::endl;
	if(quit)
		std::cout << "Press ESC or Q to quit" << std::endl;
	if(pause)	
		std::cout << "Press P to pause the program" << std::endl;
	if(screenshot)
		std::cout << "Press S to take a screenshot" << std::endl;
	std::cout << std::endl;
}

// Quit the program
bool outputControl::quitProgram(char c)
{
	bool quit = false;
	if( c  == ESC_KEY || c == 'q' || c == 'Q' )
		quit = true;
	return quit;
}


// Pause the program
void outputControl::pauseProgram(char c)
{
	if (c == 'p' || c == 'P')
		cv::waitKey(0);
}


// Make a screenshot of the current frame
void outputControl::screenshot(char c , cv::Mat &image)
{
	if (c == 's' || c == 'S')
	{
		ss<<"screenshot"<<++NoScreenshot<<".png";
		filename = ss.str();
		ss.str("");
		imwrite(filename, image);
	}
}

// Show Images on the screen
void outputControl::showVideo(std::string name, cv::Mat &image, int heigh, int width)
{
	cv::namedWindow(name,0);
	cv::resizeWindow(name, width,heigh);
	cv::imshow(name, image);
}


