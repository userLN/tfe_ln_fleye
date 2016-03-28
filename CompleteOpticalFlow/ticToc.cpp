/*
 * Source : http://ideone.com/fork/bO2tPZ
 * 
 */

#include <iostream>
#include <ctime>
#include <vector>

#include "ticToc.h"


ticToc::ticToc()
{
// 	int firstClock = std::clock();
// 	int firstTime = std::time(NULL);
// 	
// 	while(std::time(NULL) <= firstTime) {}
// 	
// 	int secondTime = std::time(NULL);
// 	int secondClock = std::clock();
// 	
// 	static int ACTUAL_CLOCKS_PER_SEC = (secondClock-firstClock)/(secondTime-firstTime);
	
}

void ticToc::tic()
{
	ticTocStack.push_back(std::clock());
}

double ticToc::toc()
{
	double toc = ((double) (std::clock() - ticTocStack.back()))/ CLOCKS_PER_SEC;
	ticTocStack.pop_back();
	return toc;
}