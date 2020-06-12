#if !defined(AFX_TABOUTPUTOPTIONS_H__22E7538B_16F1_43A2_BFEE_5B334500F778__INCLUDED_)
#define AFX_TABOUTPUTOPTIONS_H__22E7538B_16F1_43A2_BFEE_5B334500F778__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// TabOutputOptions.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CTabOutputOptions dialog

class CTabOutputOptions : public CDialog
{
	/* grant CSettings access to private members */
	friend class CSettings;
// Construction
public:
	BOOL OnInitDialog();
	CTabOutputOptions(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CTabOutputOptions)
	enum { IDD = TAB_OUTPUT_OPTIONS };
	CComboBoxEx	mComboTorque;
	CComboBoxEx	mComboForce;
	CStatic	mMessageTorque;
	CStatic	mMessageForce;
	CButton	mCheckboxTempComp;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTabOutputOptions)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CTabOutputOptions)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TABOUTPUTOPTIONS_H__22E7538B_16F1_43A2_BFEE_5B334500F778__INCLUDED_)
