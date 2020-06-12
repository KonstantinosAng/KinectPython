#if !defined(AFX_SENSORINFORMATIONDLG_H__3344C52E_7C53_4D37_99F7_648110704639__INCLUDED_)
#define AFX_SENSORINFORMATIONDLG_H__3344C52E_7C53_4D37_99F7_648110704639__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "atidaqft.h"
#include "FTWrapper.h"
// SensorInformationDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// SensorInformationDlg dialog
class SensorInformationDlg : public CDialog
{
// Construction
public:
	SensorInformationDlg(FTWrapper* ft, CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(SensorInformationDlg)
	enum { IDD = IDD_SENSOR_INFORMATION };
	CStatic	mLabelTemperatureCompensation;
	CStatic	mLabelOutputRange;
	CStatic	mLabelOutputPolarity;
	CStatic	mLabelOutputMode;
	CStatic	mLabelCalibrationDate;
	CListBox	mListCalibrations;
	CStatic	mLabelNumChannels;
	CStatic	mLabelFamily;
	CStatic	mLabelSerialNumber;
	CStatic	mLabelBodyStyle;
	//}}AFX_DATA
	CStatic	mLabelAxisMax[6];
	CStatic	mLabelAxis[6];


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(SensorInformationDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL
	BOOL OnInitDialog();

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(SensorInformationDlg)
	afx_msg void OnSensorInformationOk();
	afx_msg void OnSelchangeCalibrations();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
private:
	/* a method that is called when a new active calibration is to be displayed (or on startup) */
	void UpdateCalibrationInfo();
	_FTSensor* mFTSensor;
	/* a pointer to the FTWrapper that hides much of the complexity of the _FTSensor class */
	FTWrapper* mFTWrapper;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SENSORINFORMATIONDLG_H__3344C52E_7C53_4D37_99F7_648110704639__INCLUDED_)
