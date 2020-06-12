// Machine generated IDispatch wrapper class(es) created with ClassWizard

#include "stdafx.h"
#include "atidaqft.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



/////////////////////////////////////////////////////////////////////////////
// _FTSensor properties

/////////////////////////////////////////////////////////////////////////////
// _FTSensor operations

CString _FTSensor::GetCalFilePath()
{
	CString result;
	InvokeHelper(0x68030018, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _FTSensor::SetCalFilePath(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x68030018, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

short _FTSensor::GetNumChannels()
{
	short result;
	InvokeHelper(0x68030017, DISPATCH_PROPERTYGET, VT_I2, (void*)&result, NULL);
	return result;
}

CString _FTSensor::GetSerial()
{
	CString result;
	InvokeHelper(0x68030016, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

CString _FTSensor::GetBodyStyle()
{
	CString result;
	InvokeHelper(0x68030015, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

CString _FTSensor::GetFamily()
{
	CString result;
	InvokeHelper(0x68030014, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

LPDISPATCH _FTSensor::GetCalibrations()
{
	LPDISPATCH result;
	InvokeHelper(0x68030013, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

CString _FTSensor::GetVersion()
{
	CString result;
	InvokeHelper(0x68030012, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _FTSensor::LoadScript(LPCTSTR ScriptPath)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x60030019, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 ScriptPath);
}

void _FTSensor::SaveScript(LPCTSTR ScriptPath)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x6003001a, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 ScriptPath);
}


/////////////////////////////////////////////////////////////////////////////
// _Calibrations properties

/////////////////////////////////////////////////////////////////////////////
// _Calibrations operations

LPDISPATCH _Calibrations::GetItem(const VARIANT& Index)
{
	LPDISPATCH result;
	static BYTE parms[] =
		VTS_VARIANT;
	InvokeHelper(0x0, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, parms,
		&Index);
	return result;
}

double _Calibrations::GetCount()
{
	double result;
	InvokeHelper(0x68030002, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, NULL);
	return result;
}


/////////////////////////////////////////////////////////////////////////////
// _Calibration properties

/////////////////////////////////////////////////////////////////////////////
// _Calibration operations

VARIANT _Calibration::ConvertToFT(VARIANT* Voltages, LPCTSTR ForceUnits, LPCTSTR TorqueUnits)
{
	VARIANT result;
	static BYTE parms[] =
		VTS_PVARIANT VTS_BSTR VTS_BSTR;
	InvokeHelper(0x60030040, DISPATCH_METHOD, VT_VARIANT, (void*)&result, parms,
		Voltages, ForceUnits, TorqueUnits);
	return result;
}

VARIANT _Calibration::GetMatrix(LPCTSTR ForceUnits, LPCTSTR TorqueUnits)
{
	VARIANT result;
	static BYTE parms[] =
		VTS_BSTR VTS_BSTR;
	InvokeHelper(0x60030041, DISPATCH_METHOD, VT_VARIANT, (void*)&result, parms,
		ForceUnits, TorqueUnits);
	return result;
}

void _Calibration::Bias(VARIANT* Voltages)
{
	static BYTE parms[] =
		VTS_PVARIANT;
	InvokeHelper(0x60030043, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 Voltages);
}

void _Calibration::ClearBias()
{
	InvokeHelper(0x60030044, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
}

DATE _Calibration::GetCalDate()
{
	DATE result;
	InvokeHelper(0x6803003f, DISPATCH_PROPERTYGET, VT_DATE, (void*)&result, NULL);
	return result;
}

CString _Calibration::GetForceUnits()
{
	CString result;
	InvokeHelper(0x6803003e, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _Calibration::SetForceUnits(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x6803003e, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

CString _Calibration::GetTorqueUnits()
{
	CString result;
	InvokeHelper(0x6803003d, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _Calibration::SetTorqueUnits(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x6803003d, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

CString _Calibration::GetDistUnits()
{
	CString result;
	InvokeHelper(0x6803003c, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _Calibration::SetDistUnits(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x6803003c, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

CString _Calibration::GetAngleUnits()
{
	CString result;
	InvokeHelper(0x6803003b, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _Calibration::SetAngleUnits(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x6803003b, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

CString _Calibration::GetPartNumber()
{
	CString result;
	InvokeHelper(0x6803003a, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

short _Calibration::GetIndex()
{
	short result;
	InvokeHelper(0x68030039, DISPATCH_PROPERTYGET, VT_I2, (void*)&result, NULL);
	return result;
}

LPDISPATCH _Calibration::GetToolTransforms()
{
	LPDISPATCH result;
	InvokeHelper(0x68030038, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

BOOL _Calibration::GetTempCompAvailable()
{
	BOOL result;
	InvokeHelper(0x68030037, DISPATCH_PROPERTYGET, VT_BOOL, (void*)&result, NULL);
	return result;
}

BOOL _Calibration::GetTempCompEnabled()
{
	BOOL result;
	InvokeHelper(0x68030036, DISPATCH_PROPERTYGET, VT_BOOL, (void*)&result, NULL);
	return result;
}

void _Calibration::SetTempCompEnabled(BOOL bNewValue)
{
	static BYTE parms[] =
		VTS_BOOL;
	InvokeHelper(0x68030036, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 bNewValue);
}

double _Calibration::GetBiasSlopes(short Index)
{
	double result;
	static BYTE parms[] =
		VTS_I2;
	InvokeHelper(0x68030035, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, parms,
		Index);
	return result;
}

double _Calibration::GetGainSlopes(short Index)
{
	double result;
	static BYTE parms[] =
		VTS_I2;
	InvokeHelper(0x68030034, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, parms,
		Index);
	return result;
}

double _Calibration::GetThermistor()
{
	double result;
	InvokeHelper(0x68030033, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, NULL);
	return result;
}

void _Calibration::ResetDefaults()
{
	InvokeHelper(0x60030045, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
}

CString _Calibration::GetAxisName(const VARIANT& Index)
{
	CString result;
	static BYTE parms[] =
		VTS_VARIANT;
	InvokeHelper(0x68030032, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, parms,
		&Index);
	return result;
}

short _Calibration::GetNumAxes()
{
	short result;
	InvokeHelper(0x68030031, DISPATCH_PROPERTYGET, VT_I2, (void*)&result, NULL);
	return result;
}

double _Calibration::GetAxisMaxLoad(const VARIANT& Index)
{
	double result;
	static BYTE parms[] =
		VTS_VARIANT;
	InvokeHelper(0x68030030, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, parms,
		&Index);
	return result;
}

CString _Calibration::GetOutputMode()
{
	CString result;
	InvokeHelper(0x6803004a, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

double _Calibration::GetOutputRange()
{
	double result;
	InvokeHelper(0x68030049, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, NULL);
	return result;
}

BOOL _Calibration::GetOutputBipolar()
{
	BOOL result;
	InvokeHelper(0x68030048, DISPATCH_PROPERTYGET, VT_BOOL, (void*)&result, NULL);
	return result;
}


/////////////////////////////////////////////////////////////////////////////
// _ToolTransform properties

/////////////////////////////////////////////////////////////////////////////
// _ToolTransform operations

void _ToolTransform::SetAxis(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x6803000b, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

CString _ToolTransform::GetAxis()
{
	CString result;
	InvokeHelper(0x6803000b, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}

void _ToolTransform::SetValue(double newValue)
{
	static BYTE parms[] =
		VTS_R8;
	InvokeHelper(0x6803000a, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 newValue);
}

double _ToolTransform::GetValue()
{
	double result;
	InvokeHelper(0x6803000a, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, NULL);
	return result;
}

void _ToolTransform::SetUnits(LPCTSTR lpszNewValue)
{
	static BYTE parms[] =
		VTS_BSTR;
	InvokeHelper(0x68030009, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms,
		 lpszNewValue);
}

CString _ToolTransform::GetUnits()
{
	CString result;
	InvokeHelper(0x68030009, DISPATCH_PROPERTYGET, VT_BSTR, (void*)&result, NULL);
	return result;
}


/////////////////////////////////////////////////////////////////////////////
// _ToolTransforms properties

/////////////////////////////////////////////////////////////////////////////
// _ToolTransforms operations

void _ToolTransforms::SetAsArray(VARIANT* Values, LPCTSTR DistUnits, LPCTSTR AngleUnits)
{
	static BYTE parms[] =
		VTS_PVARIANT VTS_BSTR VTS_BSTR;
	InvokeHelper(0x6003000b, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 Values, DistUnits, AngleUnits);
}

LPDISPATCH _ToolTransforms::Add(LPCTSTR Axis, double Value, LPCTSTR Units)
{
	LPDISPATCH result;
	static BYTE parms[] =
		VTS_BSTR VTS_R8 VTS_BSTR;
	InvokeHelper(0x6003000c, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, parms,
		Axis, Value, Units);
	return result;
}

LPDISPATCH _ToolTransforms::GetItem(const VARIANT& Index)
{
	LPDISPATCH result;
	static BYTE parms[] =
		VTS_VARIANT;
	InvokeHelper(0x0, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, parms,
		&Index);
	return result;
}

long _ToolTransforms::GetCount()
{
	long result;
	InvokeHelper(0x6803000a, DISPATCH_PROPERTYGET, VT_I4, (void*)&result, NULL);
	return result;
}

void _ToolTransforms::Remove(const VARIANT& Index)
{
	static BYTE parms[] =
		VTS_VARIANT;
	InvokeHelper(0x6003000d, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 &Index);
}

void _ToolTransforms::RemoveAll()
{
	InvokeHelper(0x6003000e, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
}

