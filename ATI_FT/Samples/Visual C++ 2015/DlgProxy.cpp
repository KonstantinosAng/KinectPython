// DlgProxy.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "DlgProxy.h"
#include "Visual C SampleDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleDlgAutoProxy

IMPLEMENT_DYNCREATE(CVisualCSampleDlgAutoProxy, CCmdTarget)

CVisualCSampleDlgAutoProxy::CVisualCSampleDlgAutoProxy()
{
	EnableAutomation();
	
	// To keep the application running as long as an automation 
	//	object is active, the constructor calls AfxOleLockApp.
	AfxOleLockApp();

	// Get access to the dialog through the application's
	//  main window pointer.  Set the proxy's internal pointer
	//  to point to the dialog, and set the dialog's back pointer to
	//  this proxy.
	ASSERT (AfxGetApp()->m_pMainWnd != NULL);
	ASSERT_VALID (AfxGetApp()->m_pMainWnd);
	ASSERT_KINDOF(CVisualCSampleDlg, AfxGetApp()->m_pMainWnd);
	m_pDialog = (CVisualCSampleDlg*) AfxGetApp()->m_pMainWnd;
	m_pDialog->m_pAutoProxy = this;
}

CVisualCSampleDlgAutoProxy::~CVisualCSampleDlgAutoProxy()
{
	// To terminate the application when all objects created with
	// 	with automation, the destructor calls AfxOleUnlockApp.
	//  Among other things, this will destroy the main dialog
	if (m_pDialog != NULL)
		m_pDialog->m_pAutoProxy = NULL;
	AfxOleUnlockApp();
}

void CVisualCSampleDlgAutoProxy::OnFinalRelease()
{
	// When the last reference for an automation object is released
	// OnFinalRelease is called.  The base class will automatically
	// deletes the object.  Add additional cleanup required for your
	// object before calling the base class.

	CCmdTarget::OnFinalRelease();
}

BEGIN_MESSAGE_MAP(CVisualCSampleDlgAutoProxy, CCmdTarget)
	//{{AFX_MSG_MAP(CVisualCSampleDlgAutoProxy)
		// NOTE - the ClassWizard will add and remove mapping macros here.
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

BEGIN_DISPATCH_MAP(CVisualCSampleDlgAutoProxy, CCmdTarget)
	//{{AFX_DISPATCH_MAP(CVisualCSampleDlgAutoProxy)
		// NOTE - the ClassWizard will add and remove mapping macros here.
	//}}AFX_DISPATCH_MAP
END_DISPATCH_MAP()

// Note: we add support for IID_IVisualCSample to support typesafe binding
//  from VBA.  This IID must match the GUID that is attached to the 
//  dispinterface in the .ODL file.

// {AFDA24E8-1F0E-4F31-8B0A-794B7A3FEA42}
static const IID IID_IVisualCSample =
{ 0xafda24e8, 0x1f0e, 0x4f31, { 0x8b, 0xa, 0x79, 0x4b, 0x7a, 0x3f, 0xea, 0x42 } };

BEGIN_INTERFACE_MAP(CVisualCSampleDlgAutoProxy, CCmdTarget)
	INTERFACE_PART(CVisualCSampleDlgAutoProxy, IID_IVisualCSample, Dispatch)
END_INTERFACE_MAP()

// The IMPLEMENT_OLECREATE2 macro is defined in StdAfx.h of this project
// {43F944CE-AC8E-4923-B010-9608D3420FC6}
IMPLEMENT_OLECREATE2(CVisualCSampleDlgAutoProxy, "VisualCSample.Application", 0x43f944ce, 0xac8e, 0x4923, 0xb0, 0x10, 0x96, 0x8, 0xd3, 0x42, 0xf, 0xc6)

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleDlgAutoProxy message handlers
