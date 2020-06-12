// DAQSys.cpp: implementation of the DAQSys class.
//
//////////////////////////////////////////////////////////////////////
/* History
 * sep.23.2006a - Sam Skuce (ATI Industrial Automation) - updated to use NI-DAQmx
 */

#include "stdafx.h"
#include "Visual C Sample.h"
#include "DAQSys.h"



#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


DAQSys::DAQSys()
{
	m_th = 0;
	scanCount = 2;
	numChannels = 7;
	m_cstrDeviceName = "dev1";
	firstChannel = 0;
	scanRate = 100;
	range = 20.0;
	bipolarity = true;
	this->init(m_cstrDeviceName,0,100);
}

int DAQSys::init(CString changeDeviceName, short changeFirstChannel, double changeScanRate)
{
	const double SATURATION_THRESHOLD = 0.995; /* level at which gauges are 
											   considered saturated. */
	int status;
	double dMaxVoltage, dMinVoltage;
	m_cstrDeviceName = changeDeviceName;
	firstChannel = changeFirstChannel;
	scanRate = changeScanRate;

	/*first stop any acquisition that is already happening*/
	DAQmxStopTask( m_th );	


	status = DAQmxCreateTask( "", &m_th );
	if ( !status )
	{
		/* The list of channels on the DAQ device to read. */
		CString cstrChannelList;
		cstrChannelList.Format( "/ai%d:%d", changeFirstChannel, 
			changeFirstChannel + numChannels - 1 );
		/* Calculate maximum and minimum voltages. */
		dMaxVoltage = range;
		dMinVoltage = 0;
		if ( bipolarity )
		{
			dMaxVoltage -= (range / 2);
			dMinVoltage -= (range / 2);			
		}
		m_dUpperSaturation = dMaxVoltage - ( ( 1 - SATURATION_THRESHOLD ) * range );
		m_dLowerSaturation = dMinVoltage + ( ( 1 - SATURATION_THRESHOLD ) * range );

		
		/* Set up scan to read differential voltages. */
		status = DAQmxCreateAIVoltageChan( m_th, m_cstrDeviceName + cstrChannelList, "", 
			DAQmx_Val_Diff, dMinVoltage, dMaxVoltage, DAQmx_Val_Volts, NULL );
	}
	if ( ! status )
	{
		/* Configure the sample clock, and specify that this task does not end until
		 * it is told to stop.  Sample on clock's rising edge, with at least 2 samples
		 * in the buffer. */
		status = DAQmxCfgSampClkTiming( m_th, NULL, scanRate, DAQmx_Val_Rising,
			DAQmx_Val_ContSamps, 2 );
	}
	if ( !status )
	{
		/* Configure to read relative to the next sample to be read. */
		status = DAQmxSetReadRelativeTo( m_th, DAQmx_Val_MostRecentSamp );
	}
	if ( !status )
	{
		/* Configure to read at offset 0 from next sample, meaning we wait for the next
		 * sample everytime we try to read a sample. */
		status = DAQmxSetReadOffset( m_th, 0 );
	}
	if ( !status )
	{
		/* Start scanning. */
		status = DAQmxStartTask( m_th );
	}

	/* If any error occurred, throw it. */
	if ( status )
	{
		throw new DAQException( status );
	}
	
	return status;
}

DAQSys::~DAQSys()
{
	DAQmxStopTask( m_th );
}



int DAQSys::ScanGauges(double voltages[7], bool useStoredValues)
{
	
	long lSampsRead; /* number of samples read. */
	int i; /* generic loop/array index. */
	int iSaturated = 0; /* whether or not we saturated. */
	/* status of operations. */
	int status = DAQmxReadAnalogF64( m_th, 1, 1, DAQmx_Val_GroupByScanNumber,
		voltages, 7, &lSampsRead, NULL );
	if ( status )
	{
		throw new DAQException( status );
	}
	/* Precondition: iSaturated = 0, voltages = list of gauge voltages, m_dUpperSaturation
	 * and m_dLowerSaturation are the upper and lower voltage saturation levels.
	 * Postcondition: iSaturated = 1 if any gauge was saturated, i == 6. */
	for ( i = 0; i < 6; i++ )
	{
		if ( ( voltages[i] > m_dUpperSaturation ) ||
			( voltages[i] < m_dLowerSaturation ) )
		{
			iSaturated = 1;
		}
	}
	
	
	return iSaturated;
}


