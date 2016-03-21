#include <iostream>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"

#include "OutputControl.h"

OutputControl::OutputControl()
{
	NbScreenshot = 0;
}


bool OutputControl::quitProgram (char c)
{
	bool quit = false;
	if( c  == ESC_KEY || c == 'q' || c == 'Q' )
		quit = true;
	return quit;
}

void OutputControl::pauseProgram (char c)
{
	if (c == 'p' || c == 'P')
		cv::waitKey(0);
}

void OutputControl::screenshot(char c , cv::Mat image)
{
	if (c == 's' || c == 'S')
	{
		ss<<"screenshot"<<++NbScreenshot<<".png";
		filename = ss.str();
		ss.str("");
		imwrite(filename, image);
	}
}