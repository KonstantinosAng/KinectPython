// TabOutputOptions.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "TabOutputOptions.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CTabOutputOptions dialog


CTabOutputOptions::CTabOutputOptions(CWnd* pParent /*=NULL*/)
	: CDialog(CTabOutputOptions::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTabOutputOptions)
	//}}AFX_DATA_INIT
}


void CTabOutputOptions::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTabOutputOptions)
	DDX_Control(pDX, COMBOBOX_TORQUE, mComboTorque);
	DDX_Control(pDX, COMBOBOX_FORCE, mComboForce);
	DDX_Control(pDX, CHECK_TEMP_COMP, mCheckboxTempComp);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CTabOutputOptions, CDialog)
	//{{AFX_MSG_MAP(CTabOutputOptions)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTabOutputOptions message handlers

BOOL CTabOutputOptions::OnInitDialog()
{
	CDialog::OnInitDialog();
	CString t;

	COMBOBOXEXITEM item;
	item.mask = CBEIF_TEXT;
	item.iItem = 0;
	item.pszText = "lb";
	mComboForce.InsertItem(&item);
//	mComboForce.InsertString(0,"lb");
	item.iItem = 1;
	item.pszText = "N";
	mComboForce.InsertItem(&item);
//	mComboForce.InsertString(1, "N");
	item.iItem = 2;
	item.pszText = "kN";
	mComboForce.InsertItem(&item);
//	mComboForce.InsertString(2, "kN");
	item.iItem = 3;
	item.pszText = "g";
	mComboForce.InsertItem(&item);
//	mComboForce.InsertString(3, "g");
	item.iItem = 4;
	item.pszText = "kg";
	mComboForce.InsertItem(&item);
//	mComboForce.InsertString(4, "kg");
//	mComboForce.ShowDropDown(true);

	item.iItem = 0;
	item.pszText = "in-lb";
	mComboTorque.InsertItem(&item);
//	mComboTorque.InsertString(0, "in-lb");
	item.iItem = 1;
	item.pszText = "ft-lb";
	mComboTorque.InsertItem(&item);
//	mComboTorque.InsertString(1, "ft-lb");
	item.iItem = 2;
	item.pszText = "Nm";
	mComboTorque.InsertItem(&item);
//	mComboTorque.InsertString(2, "Nm");
	item.iItem = 3;
	item.pszText = "Nmm";
	mComboTorque.InsertItem(&item);
//	mComboTorque.InsertString(3, "Nmm");
	item.iItem = 4;
	item.pszText = "kgcm";
	mComboTorque.InsertItem(&item);
//	mComboTorque.InsertString(4, "kgcm");
//	mComboTorque.ShowDropDown(true);

	return TRUE;
}
