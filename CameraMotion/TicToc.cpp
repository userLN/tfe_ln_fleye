/*
 * Source : http://ideone.com/fork/bO2tPZ
 * 
 */

#include <iostream>
#include <ctime>
#include <vector>

#include "TicToc.h"


TicToc::TicToc()
{
}

void TicToc::tic()
{
	ticTocStack.push_back(std::clock());
}

void TicToc::toc()
{
	std::cout 
	<< "Time elapsed: "
	<< ((double) (std::clock() - ticTocStack.back()))/ CLOCKS_PER_SEC
	<< " seconds"
	<< std::endl;
	ticTocStack.pop_back();
}

double TicToc::getToc()
{
	double toc = ((double) (std::clock() - ticTocStack.back()))/ CLOCKS_PER_SEC;
	ticTocStack.pop_back();
	return toc;
}