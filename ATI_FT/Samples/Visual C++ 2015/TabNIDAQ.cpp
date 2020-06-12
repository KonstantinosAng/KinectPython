// TabNIDAQ.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "TabNIDAQ.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CTabNIDAQ dialog


CTabNIDAQ::CTabNIDAQ(CWnd* pParent /*=NULL*/)
	: CDialog(CTabNIDAQ::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTabNIDAQ)
	//}}AFX_DATA_INIT
}


void CTabNIDAQ::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTabNIDAQ)
	DDX_Control(pDX, EDIT_DEVICE_NUMBER, mEditDeviceNumber);
	DDX_Control(pDX, EDIT_SCAN_RATE, mEditScanRate);
	DDX_Control(pDX, EDIT_FIRST_CHANNEL_INDEX, mEditFirstChannelIndex);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CTabNIDAQ, CDialog)
	//{{AFX_MSG_MAP(CTabNIDAQ)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTabNIDAQ message handlers
