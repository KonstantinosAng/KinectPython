#if !defined(AFX_SETTINGS_H__3A91AFED_8E5E_44DA_981A_ECB10996FD06__INCLUDED_)
#define AFX_SETTINGS_H__3A91AFED_8E5E_44DA_981A_ECB10996FD06__INCLUDED_

#include "TabOutputOptions.h"
#include "TabToolTransforms.h"
#include "FTWrapper.h"
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Settings.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CSettings dialog

class CSettings : public CDialog
{
// Construction
public:
	CSettings(FTWrapper* wrapper, CWnd* pParent = NULL);

// Dialog Data
	//{{AFX_DATA(CSettings)
	enum { IDD = IDD_SETTINGS };
	CStatic	mPalette;
	CTabCtrl	mSettingsTab;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CSettings)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CSettings)
	afx_msg void OnSelchangeSettingsTab(NMHDR* pNMHDR, LRESULT* pResult);
	virtual void OnOK();
	virtual void OnCancel();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
private:
	/* a pointer to the FTWrapper that hides much of the complexity of the _FTSensor class */
	FTWrapper* wrapper;
	/* this variable records the index for the list of torque units */
	int torque;
	/* this variable records the index for the list of force units */
	int force;
	/* this is an object representing one of the tab frames - Tool Transforms */
	CTabToolTransforms mToolTransforms;
	/* this is an object representing one of the tab frames - Output Options */
	CTabOutputOptions mOutputOptions;
	BOOL OnInitDialog();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SETTINGS_H__3A91AFED_8E5E_44DA_981A_ECB10996FD06__INCLUDED_)
