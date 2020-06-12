#if !defined(AFX_TABSTARTUP_H__9B5679AD_22DD_49D5_A01A_DFF659522476__INCLUDED_)
#define AFX_TABSTARTUP_H__9B5679AD_22DD_49D5_A01A_DFF659522476__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// TabStartup.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CTabStartup dialog

class CTabStartup : public CDialog
{
	/* grant CProgramOptionsDlg access to private members */
	friend class CProgramOptionsDlg;
// Construction
public:
	BOOL OnInitDialog();
	CTabStartup(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CTabStartup)
	enum { IDD = TAB_STARTUP };
	CEdit	mEditConfigPath;
	CButton	mCheckLoadConfig;
	CButton	mButtonLoadConfig;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTabStartup)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CTabStartup)
	afx_msg void OnLoadConfig();
	afx_msg void OnCheckLoadConfig();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TABSTARTUP_H__9B5679AD_22DD_49D5_A01A_DFF659522476__INCLUDED_)
