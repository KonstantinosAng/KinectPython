#ifndef __ATIDAQFT_H__
#define __ATIDAQFT_H__
// Machine generated IDispatch wrapper class(es) created with ClassWizard
/////////////////////////////////////////////////////////////////////////////
// _FTSensor wrapper class

class _FTSensor : public COleDispatchDriver
{
public:
	_FTSensor() {}		// Calls COleDispatchDriver default constructor
	_FTSensor(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	_FTSensor(const _FTSensor& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	CString GetCalFilePath();
	void SetCalFilePath(LPCTSTR lpszNewValue);
	short GetNumChannels();
	CString GetSerial();
	CString GetBodyStyle();
	CString GetFamily();
	LPDISPATCH GetCalibrations();
	CString GetVersion();
	void LoadScript(LPCTSTR ScriptPath);
	void SaveScript(LPCTSTR ScriptPath);
};
/////////////////////////////////////////////////////////////////////////////
// _Calibrations wrapper class

class _Calibrations : public COleDispatchDriver
{
public:
	_Calibrations() {}		// Calls COleDispatchDriver default constructor
	_Calibrations(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	_Calibrations(const _Calibrations& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	LPDISPATCH GetItem(const VARIANT& Index);
	double GetCount();
};
/////////////////////////////////////////////////////////////////////////////
// _Calibration wrapper class

class _Calibration : public COleDispatchDriver
{
public:
	_Calibration() {}		// Calls COleDispatchDriver default constructor
	_Calibration(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	_Calibration(const _Calibration& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	VARIANT ConvertToFT(VARIANT* Voltages, LPCTSTR ForceUnits, LPCTSTR TorqueUnits);
	VARIANT GetMatrix(LPCTSTR ForceUnits, LPCTSTR TorqueUnits);
	void Bias(VARIANT* Voltages);
	void ClearBias();
	DATE GetCalDate();
	CString GetForceUnits();
	void SetForceUnits(LPCTSTR lpszNewValue);
	CString GetTorqueUnits();
	void SetTorqueUnits(LPCTSTR lpszNewValue);
	CString GetDistUnits();
	void SetDistUnits(LPCTSTR lpszNewValue);
	CString GetAngleUnits();
	void SetAngleUnits(LPCTSTR lpszNewValue);
	CString GetPartNumber();
	short GetIndex();
	LPDISPATCH GetToolTransforms();
	BOOL GetTempCompAvailable();
	BOOL GetTempCompEnabled();
	void SetTempCompEnabled(BOOL bNewValue);
	double GetBiasSlopes(short Index);
	double GetGainSlopes(short Index);
	double GetThermistor();
	void ResetDefaults();
	CString GetAxisName(const VARIANT& Index);
	short GetNumAxes();
	double GetAxisMaxLoad(const VARIANT& Index);
	CString GetOutputMode();
	double GetOutputRange();
	BOOL GetOutputBipolar();
};
/////////////////////////////////////////////////////////////////////////////
// _ToolTransform wrapper class

class _ToolTransform : public COleDispatchDriver
{
public:
	_ToolTransform() {}		// Calls COleDispatchDriver default constructor
	_ToolTransform(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	_ToolTransform(const _ToolTransform& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	void SetAxis(LPCTSTR lpszNewValue);
	CString GetAxis();
	void SetValue(double newValue);
	double GetValue();
	void SetUnits(LPCTSTR lpszNewValue);
	CString GetUnits();
};
/////////////////////////////////////////////////////////////////////////////
// _ToolTransforms wrapper class

class _ToolTransforms : public COleDispatchDriver
{
public:
	_ToolTransforms() {}		// Calls COleDispatchDriver default constructor
	_ToolTransforms(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	_ToolTransforms(const _ToolTransforms& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Attributes
public:

// Operations
public:
	void SetAsArray(VARIANT* Values, LPCTSTR DistUnits, LPCTSTR AngleUnits);
	LPDISPATCH Add(LPCTSTR Axis, double Value, LPCTSTR Units);
	LPDISPATCH GetItem(const VARIANT& Index);
	long GetCount();
	void Remove(const VARIANT& Index);
	void RemoveAll();
};
#endif // __ATIDAQFT_H__