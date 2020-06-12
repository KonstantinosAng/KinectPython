#if !defined(AFX_POPUPTOOLTRANSFORM_H__20A612A8_81E9_4B4B_B067_63DB8EBEF334__INCLUDED_)
#define AFX_POPUPTOOLTRANSFORM_H__20A612A8_81E9_4B4B_B067_63DB8EBEF334__INCLUDED_

#include "atidaqft.h"
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// PopupToolTransform.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CPopupToolTransform dialog

class CPopupToolTransform : public CDialog
{
// Construction
public:
	BOOL OnInitDialog();
	/* This constructor is used for editing an existing Tool Transform */
	CPopupToolTransform(_ToolTransform* tool, CWnd* pParent = NULL);
	/* This constructor is used for creating a new Tool Transform */
	CPopupToolTransform(_ToolTransforms* tools, CWnd* pParent = NULL);

// Dialog Data
	//{{AFX_DATA(CPopupToolTransform)
	enum { IDD = IDD_TOOL_TRANSFORM };
	CEdit	mEditValue;
	CComboBoxEx	mComboUnits;
	CComboBoxEx	mComboAxis;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CPopupToolTransform)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CPopupToolTransform)
	virtual void OnOK();
	virtual void OnCancel();
	afx_msg void OnSelchangeAxis();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
private:
	/* a pointer to the list of Tool Transforms, NULL if editing */
	_ToolTransforms* tools;
	/* a pointer to an existing Tool Transform, NULL if creating new */
	_ToolTransform* tool;
	/* this boolean tracks whether this object is editing (true) or creating (false) a tool transform */
	bool edit;
	/* common code to both constructors */
	void AFX_DATA_INIT();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_POPUPTOOLTRANSFORM_H__20A612A8_81E9_4B4B_B067_63DB8EBEF334__INCLUDED_)
