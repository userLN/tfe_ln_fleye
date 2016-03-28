/*
 * Source : http://ideone.com/fork/bO2tPZ
 * 
 */

#pragma once
#include <iostream>
#include <ctime>
#include <vector>

class ticToc
{
private:
	std::vector<std::clock_t> ticTocStack;
	int ACTUAL_CLOCKS_PER_SEC;
		
public:
	ticToc();
	void tic();
	double toc();
};