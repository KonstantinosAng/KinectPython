// DAQSys.h: interface for the DAQSys class.
//
//////////////////////////////////////////////////////////////////////
#if !defined(AFX_DAQSYS_H__7532000D_2E31_4585_AB12_BB18D4864927__INCLUDED_)
#define AFX_DAQSYS_H__7532000D_2E31_4585_AB12_BB18D4864927__INCLUDED_

#include "NIDAQmx.h"

//#include <nidaqex.h>

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define NUM_CHANNELS 7
#define ERROR_MESSAGE_SIZE 2048
#define SCAN_BUFFER_SIZE 280

#ifdef _AFXDLL
class DAQException : public CException
#else
class AFX_NOVTABLE DAQException : public CException
#endif
{
public:
	DAQException(CString new_err_msg="", BOOL bAutoDelete=true) : CException(bAutoDelete)
	{
		err_msg = new_err_msg;
		err_no = 0;
	}
	DAQException(int new_err_no, BOOL bAutoDelete = true) : CException(bAutoDelete)
	{
		err_no = new_err_no;
		CString integer;
		integer.Format("%d",err_no);
		char acErrMessage[ERROR_MESSAGE_SIZE];
		DAQmxGetErrorString( new_err_no, acErrMessage, ERROR_MESSAGE_SIZE );
		err_msg = acErrMessage;


		
	}
	CString GetErrMsg()		{ return err_msg; }
	int GetErrNo()			{ return err_no; }
private:
	CString err_msg;
	int err_no;
};

/* this is used for development only, so that a DAQ is not required to test the GUI */
#define TEST_WITHOUT_DAQ 0

class DAQSys  
{
public:
	/* this method is used to setup the DAQ during object construction of when changed in the Program Options */
	int init(CString changeDeviceName, short firstChannel, double scanRate);	
	/* scans all 7 gauges, or returns last scan if boolean is true, returns 0 for success or -1 for saturation */
	int ScanGauges(double voltages[7], bool useStoredValues);	
	/* return the device number of the DAQ - originally defaults to 1 */
	CString getDeviceName() { return m_cstrDeviceName; }
	/* return the first channel to be scanned by the DAQ - originally defaults to 1 */
	short getFirstChannel() { return firstChannel; }
	/* returns the scan rate to be used when scanning the gauges */
	double getScanRate() { return scanRate; }
	DAQSys();
	virtual ~DAQSys();

private:
	TaskHandle m_th; /* The task handle to the NI-DAQmx scan. */
	double range;
	bool bipolarity;
	double scanRate;		
	CString m_cstrDeviceName;
	short firstChannel;	
	unsigned short numChannels;	
	long scanBuffer[SCAN_BUFFER_SIZE]; /*memory buffer used by scanning operation*/
	unsigned int scanCount; /*the number of scans in a reading.  averaged out to get
								the final result*/
	double m_dUpperSaturation; /* The upper level past which a gauge is considered
							   saturated. */
	double m_dLowerSaturation; /* The lower level past which a gauge is considered
							   saturated. */
#if TEST_WITHOUT_DAQ
	bool random_gen;
#endif
};

#endif // !defined(AFX_DAQSYS_H__7532000D_2E31_4585_AB12_BB18D4864927__INCLUDED_)
