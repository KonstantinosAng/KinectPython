// TabStartup.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "TabStartup.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CTabStartup dialog


CTabStartup::CTabStartup(CWnd* pParent /*=NULL*/)
	: CDialog(CTabStartup::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTabStartup)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CTabStartup::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTabStartup)
	DDX_Control(pDX, EDIT_CONFIG_PATH, mEditConfigPath);
	DDX_Control(pDX, CHECK_LOAD_CONFIG, mCheckLoadConfig);
	DDX_Control(pDX, BUTTON_LOAD_CONFIG, mButtonLoadConfig);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CTabStartup, CDialog)
	//{{AFX_MSG_MAP(CTabStartup)
	ON_BN_CLICKED(BUTTON_LOAD_CONFIG, OnLoadConfig)
	ON_BN_CLICKED(CHECK_LOAD_CONFIG, OnCheckLoadConfig)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTabStartup message handlers

CString ChooseConfigFile()
{
	CString filename;
	CString filter = "ATI DAQ F/T Configuration Files (*.cfg)|*.cfg||";
	CFileDialog fileMenu(true, NULL, NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, filter, NULL);
	fileMenu.DoModal();
	filename = fileMenu.GetPathName();
//	if (fileMenu.GetFileExt() != "cfg") {
//		// bad cal file
//	}
	return filename;
}

void CTabStartup::OnLoadConfig() 
{
	CString filename = ChooseConfigFile();
	if (filename != "") {
		mEditConfigPath.SetWindowText(filename);
	}
}

void CTabStartup::OnCheckLoadConfig() 
{
	bool enable = mCheckLoadConfig.GetCheck() ? true : false;
	mButtonLoadConfig.EnableWindow(enable);
	mEditConfigPath.EnableWindow(enable);
}

BOOL CTabStartup::OnInitDialog()
{
	CDialog::OnInitDialog();
	OnCheckLoadConfig();
	return TRUE;
}
