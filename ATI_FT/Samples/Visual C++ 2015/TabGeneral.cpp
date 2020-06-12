// TabGeneral.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "TabGeneral.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CTabGeneral dialog


CTabGeneral::CTabGeneral(CWnd* pParent /*=NULL*/)
	: CDialog(CTabGeneral::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTabGeneral)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CTabGeneral::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTabGeneral)
	DDX_Control(pDX, IDC_CHECKAUTOLOAD, m_checkAutoload);
	DDX_Control(pDX, EDIT_TIME_INTERVAL, mEditTimeInterval);
	DDX_Control(pDX, CHECK_THERMISTOR, mCheckThermistor);
	DDX_Control(pDX, CHECK_BEEP, mCheckBeep);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CTabGeneral, CDialog)
	//{{AFX_MSG_MAP(CTabGeneral)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTabGeneral message handlers
