/*
ConfigureSaveData.h
defines structure of data that is saved in configuration files
*/

#ifndef CONFIGURESAVEDATA_H
#define CONFIGURESAVEDATA_H

#include "TTFSaveData.h"

struct ConfigureSaveData
{
	/*the file version (currently 1)*/
	unsigned int fileVersion;
	/*the DAQ device name */
	char DAQDeviceName[100];
	/* the DAQ starting channel */
	short DAQStartChannel;
	/* the DAQ scan rate */
	double DAQScanRate;
	/* the milliseconds between screen refreshes */
	int screenTime;
	/* whether or not to beep on ADC saturation */
	bool ADCBeep;
	/* whether or not to show the thermistor readout */
	bool showTherm;
	/* the path to the calibration file */
	char calFilePath[200];
	/* the bias voltages */
	double biasVoltages[7];
	/* the force units */
	char forceUnits[6];
	/* the torque units */
	char torqueUnits[6];
	/* whether or not temperature compensation is enabled */
	BOOL tempComp;
	/* the number of tool transforms stored after this structure in the file */
	unsigned int numTTFs;	

};

#endif