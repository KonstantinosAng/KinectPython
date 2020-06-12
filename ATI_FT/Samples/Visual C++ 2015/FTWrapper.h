// FTWrapper.h: interface for the FTWrapper class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_FTWRAPPER_H__F659A09E_BC56_40EE_8D2B_360B538AEECC__INCLUDED_)
#define AFX_FTWRAPPER_H__F659A09E_BC56_40EE_8D2B_360B538AEECC__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "atidaqft.h"
#include "DAQSys.h"

class FTWrapper  
{
public:
	
	enum DAQFT_STATUS { DAQ_ERROR=0, DAQ_READY=1, CALFILE_LOADED=2, ALL_READY=3 , UNKNOWN=4};
	FTWrapper();
	virtual ~FTWrapper();
	//sets the bias values from the argument
	void SetBiasVoltages(double biases[]);
	//puts the bias voltages in argument, fills with 0s if bias is not enabled	
	void GetBiasVoltages(double biases[]);
	// this function returns a value of type FTWrapper::DAQFT_STATUS
	DAQFT_STATUS GetStatus() { return status; }
	// returns the daq err message if status == DAQ_ERROR, else ""
	CString GetDAQErrorMessage() { return daqerrmsg; }
	// returns the _FTSensor object held by this FTWrapper object
	_FTSensor* GetFTSensor() { return &mFTSensor; }
	// reads a Calibration File and resets instance variables
	int LoadCalFile(CString filename);
	// fills a 7-element array of doubles with Force and Torque measurements
	int GetForcesAndTorques(double[7]);
	// fills a 7-element array of doubles with Raw Voltages
	int GetRawVoltages(double[7]);
	// specifies whether force/torque and voltages will be biased or not
	void Bias(bool bias);
	// returns the active calibration of the _FTSensor object
	_Calibration GetActiveCalibration();
	// returns the index of the active calibration of the _FTSensor object
	short GetActiveCalibrationIndex() { return ActiveCalibration; }
	// sets the index of the active calibration of the _FTSensor object
	void SetActiveCalibration(short index)
	{
		ActiveCalibration = index;
		setupMinMax();
	}
	/* when the calibration changes, so might the Min and Max values for A/D saturation */
	void setupMinMax();
	/* adds a tool transform to the active calibration */
	void AddToolTransform(CString axis, double Value, CString units);
	/* returns a pointer to the DAQSys object */
	DAQSys* getDAQSys() { return mDAQSys; }
	/* returns the polarity of the Active Calibration */
	bool isBipolar()
	{
		if (ActiveCalibration == -1) {
			return true;
		}
		return (GetActiveCalibration().GetOutputBipolar() == TRUE);
	}
	/* returns the output range of the Active Calibration */
	double getOutputRange()
	{
		if (ActiveCalibration == -1) {
			return 20.0;
		}
		return GetActiveCalibration().GetOutputRange();
	}

private:
	double biasVoltage[7];
	CString daqerrmsg;
	// DAQSys object used to aquire data from the NI-DAQ device
	DAQSys* mDAQSys;
	// _FTSensor object used to read calibration files and get the Active Calibration
	_FTSensor mFTSensor;
	// an index into the _FTSensor's Calibrations list of the Active Calibration
	short ActiveCalibration;
	// a boolean representing whether force/torque and voltages should be biased
	bool bias;
	DAQFT_STATUS status;
};

#endif // !defined(AFX_FTWRAPPER_H__F659A09E_BC56_40EE_8D2B_360B538AEECC__INCLUDED_)
