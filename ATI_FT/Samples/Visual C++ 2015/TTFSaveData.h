/*
TTFSAveData.h
defines TTFSaveData structure, which is used to save tool
transform data to a file
*/
#ifndef TTFSAVEDATA_H
#define TTFSAVEDATA_H

struct TTFSaveData
{	
	//the name of the axis this ttf applies to
	char axis[3];
	//the value of this ttf
	double value;
	//the units this ttf uses
	char units[10];	
};

#endif