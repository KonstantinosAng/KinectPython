// Visual C Sample.h : main header file for the VISUAL C SAMPLE application
//

#if !defined(AFX_VISUALCSAMPLE_H__8E04017D_2F06_48D1_8632_C7EB7C57DD94__INCLUDED_)
#define AFX_VISUALCSAMPLE_H__8E04017D_2F06_48D1_8632_C7EB7C57DD94__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleApp:
// See Visual C Sample.cpp for the implementation of this class
//

class CVisualCSampleApp : public CWinApp
{
public:
	CVisualCSampleApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CVisualCSampleApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CVisualCSampleApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_VISUALCSAMPLE_H__8E04017D_2F06_48D1_8632_C7EB7C57DD94__INCLUDED_)
