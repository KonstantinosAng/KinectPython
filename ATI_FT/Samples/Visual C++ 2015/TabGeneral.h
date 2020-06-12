#if !defined(AFX_TABGENERAL_H__462CC0EC_164E_403A_8854_2103FF4A8B33__INCLUDED_)
#define AFX_TABGENERAL_H__462CC0EC_164E_403A_8854_2103FF4A8B33__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// TabGeneral.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CTabGeneral dialog

class CTabGeneral : public CDialog
{
	/* grant CProgramOptionsDlg access to private members */
	friend class CProgramOptionsDlg;
// Construction
public:
	CTabGeneral(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CTabGeneral)
	enum { IDD = TAB_GENERAL };
	CButton	m_checkAutoload;
	CEdit	mEditTimeInterval;
	CButton	mCheckThermistor;
	CButton	mCheckBeep;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTabGeneral)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CTabGeneral)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TABGENERAL_H__462CC0EC_164E_403A_8854_2103FF4A8B33__INCLUDED_)
