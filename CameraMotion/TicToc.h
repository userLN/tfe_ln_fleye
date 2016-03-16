/*
 * Source : http://ideone.com/fork/bO2tPZ
 * 
 */

#pragma once
#include <iostream>
#include <ctime>
#include <vector>

class TicToc
{
private:
	std::vector<std::clock_t> ticTocStack;
		
public:
	TicToc();
	void tic();
	void toc();
	double getToc();
};