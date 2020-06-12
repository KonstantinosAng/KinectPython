#if !defined(AFX_TABNIDAQ_H__1966B68E_C2D9_4C84_9E07_872DB0361773__INCLUDED_)
#define AFX_TABNIDAQ_H__1966B68E_C2D9_4C84_9E07_872DB0361773__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// TabNIDAQ.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CTabNIDAQ dialog

class CTabNIDAQ : public CDialog
{
	/* grant CProgramOptionsDlg access to private members */
	friend class CProgramOptionsDlg;
// Construction
public:
	CTabNIDAQ(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CTabNIDAQ)
	enum { IDD = TAB_NIDAQ };
	CEdit	mEditDeviceNumber;
	CEdit	mEditScanRate;
	CEdit	mEditFirstChannelIndex;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTabNIDAQ)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CTabNIDAQ)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TABNIDAQ_H__1966B68E_C2D9_4C84_9E07_872DB0361773__INCLUDED_)
