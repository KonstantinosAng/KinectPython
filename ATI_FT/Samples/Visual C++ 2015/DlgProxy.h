// DlgProxy.h : header file
//

#if !defined(AFX_DLGPROXY_H__516C3176_345D_4DAC_918F_3543F3A9A18F__INCLUDED_)
#define AFX_DLGPROXY_H__516C3176_345D_4DAC_918F_3543F3A9A18F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CVisualCSampleDlg;

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleDlgAutoProxy command target

class CVisualCSampleDlgAutoProxy : public CCmdTarget
{
	DECLARE_DYNCREATE(CVisualCSampleDlgAutoProxy)

	CVisualCSampleDlgAutoProxy();           // protected constructor used by dynamic creation

// Attributes
public:
	CVisualCSampleDlg* m_pDialog;

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CVisualCSampleDlgAutoProxy)
	public:
	virtual void OnFinalRelease();
	//}}AFX_VIRTUAL

// Implementation
protected:
	virtual ~CVisualCSampleDlgAutoProxy();

	// Generated message map functions
	//{{AFX_MSG(CVisualCSampleDlgAutoProxy)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
	DECLARE_OLECREATE(CVisualCSampleDlgAutoProxy)

	// Generated OLE dispatch map functions
	//{{AFX_DISPATCH(CVisualCSampleDlgAutoProxy)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_DISPATCH
	DECLARE_DISPATCH_MAP()
	DECLARE_INTERFACE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DLGPROXY_H__516C3176_345D_4DAC_918F_3543F3A9A18F__INCLUDED_)
