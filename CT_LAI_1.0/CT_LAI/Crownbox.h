#pragma once
#include "commdef.h"
#include "Crowncube.h"

class Crownbox
{

public:
	Crownbox() {};
	~Crownbox() {};


private:
	
	bool remove_empty;
	int block_num;
	Crowncube* block;
};