// Visual C SampleDlg.h : header file
//
//{{AFX_INCLUDES()
#include "atidaqft.h"	// Added by ClassView
#include "FTWrapper.h"	// Added by ClassView
//}}AFX_INCLUDES
#include "ProgramOptionsDlg.h"
#include "SensorInformationDlg.h"
#include "Settings.h"

#if !defined(AFX_VISUALCSAMPLEDLG_H__8234507D_6619_49CF_A1BD_CDA8F400D457__INCLUDED_)
#define AFX_VISUALCSAMPLEDLG_H__8234507D_6619_49CF_A1BD_CDA8F400D457__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CVisualCSampleDlgAutoProxy;
class DAQSys;

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleDlg dialog

class CVisualCSampleDlg : public CDialog
{
	DECLARE_DYNAMIC(CVisualCSampleDlg);
	friend class CVisualCSampleDlgAutoProxy;

// Construction
public:
	CVisualCSampleDlg(CWnd* pParent = NULL);	// standard constructor
	virtual ~CVisualCSampleDlg();

// Dialog Data
	//{{AFX_DATA(CVisualCSampleDlg)
	enum { IDD = IDD_VISUALCSAMPLE_DIALOG };
	CButton	m_buttonLog;
	CStatic	mLabelThermistor;
	CButton	mGroupTorque;
	CButton	mGroupForce;
	CButton	mUnbias;
	CButton	mBias;
	CStatic	mLabelMessage;
	int		m_display;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CVisualCSampleDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	afx_msg BOOL OnQueryNewPalette();
	afx_msg void OnPaletteChanged(CWnd* pFocusWnd);
	CPalette* SetPalette(CPalette* pPalette);
	CPalette* m_pPalette;
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	CVisualCSampleDlgAutoProxy* m_pAutoProxy;
	HICON m_hIcon;

	BOOL CanExit();

	// Generated message map functions
	//{{AFX_MSG(CVisualCSampleDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnClose();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnABOUT();
	afx_msg void OnBias();
	afx_msg void OnUnbias();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnOpenCalibration();
	afx_msg void OnSensorInformation();
	afx_msg void OnProgramOptions();
	afx_msg void OnEXIT();
	afx_msg void OnSaveConfiguration();
	afx_msg void OnSETTINGS();
	afx_msg void OnVoltages();
	afx_msg void OnFt();
	afx_msg void OnOpenConfiguration();
	afx_msg void OnButtonlog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
private:
	/*writes field headers to log file (i.e. "G0..G6" or "Fx..Tz")*/
	void WriteLogDataHeaders();
	/*adds the readings to the log file*/
	void AddToLogFile( double readings[] );
	/*whether or not we're currently logging data*/
	bool mLoggingEnabled;
	/*the file logged data is stored in*/
	CFile mLogFile;
	/* asks whether or not to auto-load file at startup, writes value to registry */
	void AskAboutAutoLoad( CString filename );
	/* the configuration file to auto-load at startup */
	CString mAutoLoadFilename;
	/* loads a configuration file */
	void loadConfiguration(LPCTSTR fileName);
	/* an object that handles selection of the Settings menu item */
	CSettings* dlgSettings;
	/* an object that handles selection of the Program Options menu item */
	CProgramOptionsDlg* dlgProgramOptions;
	/* a pointer to the FTWrapper that hides much of the complexity of the _FTSensor class */
	FTWrapper mFTWrapper;
	/* this procedure enables/disables various controls base on the current state */
	void enableInputs();
	/* this bool tracks whether raw voltages or forces/torques should be displayed */
	bool show_raw_voltages;
//	/* an internal function used to convert progress bar readings back to floats */
//	double itof(int i);
	/* an internal function used to convert floats into progress bar readings */
	int ftoi(double f, short channel);
	/* a list of the static text controls used to display the voltages or forces/torques */
	CStatic mValues[6];
	/* a list of static text controls used to display the gauge id's of the F/T axes */
	CStatic	mLabels[6];
	/* a list of the progress bar controls that (poorly) displays the voltage reading */
	CProgressCtrl mVoltmeter[6];
	/* the heard and soul of the screen update, this procedure reads the voltages passed in and updates the various components on the screen */
	void UpdateGauges(double* readings);
	/* called during OnInitDialog - a centralized place for added startup code */
	void CustomInitialization();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_VISUALCSAMPLEDLG_H__8234507D_6619_49CF_A1BD_CDA8F400D457__INCLUDED_)
