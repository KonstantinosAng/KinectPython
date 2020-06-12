// FTWrapper.cpp: implementation of the FTWrapper class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Visual C Sample.h"
#include "FTWrapper.h"
#include <direct.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

FTWrapper::FTWrapper()
{
	// CreateDispatch is required to be called exactly once
	// therefore, creating multiple instances of FTWrapper will probably not work
	mFTSensor.CreateDispatch("ATIDAQFT.FTSensor");
	ActiveCalibration = -1;
	status = UNKNOWN;
	daqerrmsg = "";
	bias = false;
	mDAQSys = NULL;
	for (int i=0; i<7; i++) {
		biasVoltage[i] = 0.0;
	}
	try {		
		mDAQSys = new DAQSys();
		status = DAQ_READY;
	}
	catch (DAQException* error) {
		daqerrmsg = error->GetErrMsg();
		if (mDAQSys != NULL) {
			delete mDAQSys;
			mDAQSys = NULL;
		}
		status = DAQ_ERROR;
	}
}

FTWrapper::~FTWrapper()
{
	if (NULL != mDAQSys)
		delete mDAQSys;
}

// reads a Calibration File and resets instance variables
int FTWrapper::LoadCalFile(CString filename)
{
	// only allow *.cal files
	CString filter = "ATI DAQ F/T Calibration Files (*.cal)|*.cal||";
	CFileDialog fileMenu(true, NULL, NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, filter, NULL);
	if (filename == "") {
		// using a GUI, go get the file
		char workingDirectory[200]; /*the working directory*/
		_getdcwd( _getdrive(), workingDirectory, 200 );
		fileMenu.m_ofn.lpstrInitialDir = workingDirectory;
		fileMenu.DoModal();
		filename = fileMenu.GetPathName();
	}
//	if (fileMenu.GetFileExt() != "cal") {
//		// bad cal file
//	}
	if (filename == "") {
		return -1;
	}
	mFTSensor.SetCalFilePath(filename);
	// default to the first calibration listed in the file
	ActiveCalibration = 1;
	// set the bit that indicates the calibration file has been successfully loaded
	status = (FTWrapper::DAQFT_STATUS)(status | CALFILE_LOADED);
	setupMinMax();
	return 0;
}

void FTWrapper::setupMinMax()
{
	// this method should never be called unless the active calibration has been set, but just in case....
	if (ActiveCalibration != -1) {
		_Calibration calibration = this->GetActiveCalibration();		
	}
}

// returns a new 7-element array of doubles containing Force and Torque measurements
int FTWrapper::GetForcesAndTorques(double output[7])
{
	if (status == ALL_READY) {
		// this array is used to create the COleSafeArray below
		// it is used to indicate that there are 7 elements in the first diemnsion
		DWORD numElements[] = {7};
		COleSafeArray voltages;
		// VT_R8 represents an 8-byte real, i.e. a double in C++
		// there is 1 dimension with a size of 'numElements[0]'
		voltages.Create(VT_R8, 1, numElements);

		double ft_voltages[7];
		int mDAQstatus = mDAQSys->ScanGauges(ft_voltages, false);
		long i;
		for(i=0;i<7;i++) {
			// COleSafeArrays use pass-by-reference methods to read or write
			voltages.PutElement(&i,&(ft_voltages[i]));
		}
		// this method uses the calibration matrix (and possibly temperature compensation)
		// to convert the 6 gauge readings into forces and torques
		COleSafeArray forces = GetActiveCalibration().ConvertToFT(voltages,"lb","in-lb");
		for (i=0; i<6; i++) {
			// COleSafeArrays use pass-by-reference methods to read or write
			forces.GetElement(&i,&(output[i]));
		}
		// copy the thermistor voltage over to the output for display
		output[6] = ft_voltages[6];
		return mDAQstatus;
	}
	else {
		return -2;
	}
}

// returns a new 7-element array of doubles containing Raw Voltages
int FTWrapper::GetRawVoltages(double voltages[7])
{
	if (status & DAQ_READY) {
		// initiate a new scan
		int mDAQstatus = mDAQSys->ScanGauges(voltages, false);
		if (bias) {
			// don't bias voltage[6], it is the thermistor
			for(int i=0;i<6;i++) {
				voltages[i] = voltages[i] - biasVoltage[i];
			}
		}
		return mDAQstatus;
	}
	else {
		return -1;
	}
}

// specifies whether force/torque and voltages will be biased or not
void FTWrapper::Bias(bool newbias)
{
	bias = newbias;
	if (status & DAQ_READY) {
		if (bias == true) {
			// this array is used to create the COleSafeArray below
			// it is used to indicate that there are 7 elements in the first diemnsion
			DWORD numElements[] = {7};
			COleSafeArray voltages;
			// VT_R8 represents an 8-byte real, i.e. a double in C++
			// there is 1 dimension with a size of 'numElements[0]'
			voltages.Create(VT_R8, 1, numElements);

			mDAQSys->ScanGauges(biasVoltage, true);
//			if (mDAQstatus) {
				// bad bias!
//			}
			for(long i=0;i<7;i++) {
				// COleSafeArrays use pass-by-reference methods to read or write
				voltages.PutElement(&i,&(biasVoltage[i]));
			}
			// check the bits that indicate that the calibration file has been loaded and the DAQ is ready
			if (status == ALL_READY) {
				GetActiveCalibration().Bias(voltages);
			}
		}
		else {
			// check the bit that indicates that the calibration file has been loaded
			if (status & CALFILE_LOADED) {
				GetActiveCalibration().ClearBias();
			}
		}
	}
}

// returns the active calibration of the _FTSensor object
_Calibration FTWrapper::GetActiveCalibration()
{
	// check the bit that indicates that the calibration file has been loaded
	if (status & CALFILE_LOADED) {
		_Calibrations calibrations = mFTSensor.GetCalibrations();
		// have to use a COleVariant as an index into the list (joy of dll's)
		// this COleVariant represents a short (VT_I2)
		// this array is 1-based, not 0-based like in C++
		return calibrations.GetItem(COleVariant(ActiveCalibration, VT_I2));
	}
	else {
		return NULL;
	}
}

// adds a tool transform to the active calibration
void FTWrapper::AddToolTransform(CString axis, double Value, CString units)
{
	// check the bit that indicates that the calibration file has been loaded
	if (status & CALFILE_LOADED) {
		_ToolTransforms tools = GetActiveCalibration().GetToolTransforms();
		tools.Add(axis, Value, units);
	}
}

void FTWrapper::GetBiasVoltages(double biases[])
{
	int i;
	for (i = 0; i < 7; i++)
	{
		/*set bias to 0 if bias is not active*/
		biases[i] = bias ? biasVoltage[i] : 0;
	}
}


void FTWrapper::SetBiasVoltages(double biases[])
{
	long i;

	COleSafeArray safeBias; /*we do the SafeArray dance to pass
							the data through COM*/	
	DWORD numElements[] = {7};
	safeBias.Create(VT_R8, 1, numElements);

	bias = true;

	/*set the stored bias and set up the safearray to pass to COM*/
	for (i = 0; i < 7; i++)
	{
		biasVoltage[i] = biases[i];
		safeBias.PutElement(&i, &biases[i]);
	}
	
	if (status == ALL_READY) 
		GetActiveCalibration().Bias(safeBias);	
	
}