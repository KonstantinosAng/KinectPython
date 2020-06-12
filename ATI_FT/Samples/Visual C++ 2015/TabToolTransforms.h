#if !defined(AFX_TABTOOLTRANSFORMS_H__129A487D_982F_4D4D_9304_E108A86B82A8__INCLUDED_)
#define AFX_TABTOOLTRANSFORMS_H__129A487D_982F_4D4D_9304_E108A86B82A8__INCLUDED_

#include "atidaqft.h"	// Added by ClassView
#include "FTWrapper.h"
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// TabToolTransforms.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CTabToolTransforms dialog

class CTabToolTransforms : public CDialog
{
	/* grant CSettings access to private members */
	friend class CSettings;
// Construction
public:
	BOOL OnInitDialog();
	void SetFTWrapper(FTWrapper* wrapper);
	CTabToolTransforms(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CTabToolTransforms)
	enum { IDD = TAB_TOOL_TRANSFORMS };
	CListBox	mListTransforms;
	CButton	mButtonRemoveTransform;
	CButton	mButtonRemoveAllTransforms;
	CButton	mButtonEditTransform;
	CButton	mButtonAddTransform;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CTabToolTransforms)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CTabToolTransforms)
	afx_msg void OnAddTransform();
	afx_msg void OnEditTransform();
	afx_msg void OnRemoveAllTransforms();
	afx_msg void OnRemoveTransform();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
private:
	/* update the list of tool transforms for the active calibration */
	void updateList();
	/* the list of Tool Transforms for the active Calibration */
	_ToolTransforms tools;
	/* the active Calibration */
	_Calibration calibration;
	/* a pointer to the FTWrapper that hides much of the complexity of the _FTSensor class */
	FTWrapper* wrapper;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_TABTOOLTRANSFORMS_H__129A487D_982F_4D4D_9304_E108A86B82A8__INCLUDED_)
