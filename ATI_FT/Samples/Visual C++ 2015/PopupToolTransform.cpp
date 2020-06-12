// PopupToolTransform.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "PopupToolTransform.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CPopupToolTransform dialog

void CPopupToolTransform::AFX_DATA_INIT()
{
	//{{AFX_DATA_INIT(CPopupToolTransform)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}

// this is the constructor which edits an existing tool transform
CPopupToolTransform::CPopupToolTransform(_ToolTransform* tool, CWnd* pParent /*=NULL*/)
	: CDialog(CPopupToolTransform::IDD, pParent)
{
	this->AFX_DATA_INIT();
	edit = true;
	this->tool = tool;
	this->tools = NULL;
}

// this is the constructor which creates a new tool transform to append to the list
CPopupToolTransform::CPopupToolTransform(_ToolTransforms* tools, CWnd* pParent /*=NULL*/)
	: CDialog(CPopupToolTransform::IDD, pParent)
{
	this->AFX_DATA_INIT();
	edit = false;
	this->tool = NULL;
	this->tools = tools;
}

void CPopupToolTransform::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CPopupToolTransform)
	DDX_Control(pDX, EDIT_VALUE, mEditValue);
	DDX_Control(pDX, COMBO_UNITS, mComboUnits);
	DDX_Control(pDX, COMBO_AXIS, mComboAxis);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CPopupToolTransform, CDialog)
	//{{AFX_MSG_MAP(CPopupToolTransform)
	ON_CBN_SELCHANGE(COMBO_AXIS, OnSelchangeAxis)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CPopupToolTransform message handlers

void CPopupToolTransform::OnOK() 
{
	CString axis;
	CString value;
	CString units;
	int index;

	mEditValue.GetWindowText(value);
	index = mComboAxis.GetCurSel();
	if (index != LB_ERR) {
		mComboAxis.GetLBText(index, axis);
	}
	index = mComboUnits.GetCurSel();
	if (index != LB_ERR) {
		mComboUnits.GetLBText(index, units);
	}
	if (value != "") {
		double d = atof(value);
		if (edit) {
			tool->SetAxis(axis);
			tool->SetValue(d);
			tool->SetUnits(units);
		}
		else {
			tools->Add(axis, d, units);
		}
	}
	CDialog::OnOK();
}

void CPopupToolTransform::OnCancel() 
{
	CDialog::OnCancel();
}

BOOL CPopupToolTransform::OnInitDialog()
{
	CDialog::OnInitDialog();
	COMBOBOXEXITEM item;
	item.mask = CBEIF_TEXT;

	item.iItem = 0;
	item.pszText = "Dx";
	mComboAxis.InsertItem(&item);

	item.iItem = 1;
	item.pszText = "Dy";
	mComboAxis.InsertItem(&item);

	item.iItem = 2;
	item.pszText = "Dz";
	mComboAxis.InsertItem(&item);

	item.iItem = 3;
	item.pszText = "Rx";
	mComboAxis.InsertItem(&item);

	item.iItem = 4;
	item.pszText = "Ry";
	mComboAxis.InsertItem(&item);

	item.iItem = 5;
	item.pszText = "Rz";
	mComboAxis.InsertItem(&item);

	mComboAxis.SetCurSel(0);
	if (edit) {
		int i;
		CString axis = tool->GetAxis();
		for (i=0; i<mComboAxis.GetCount(); i++) {
			CString temp;
			mComboAxis.GetLBText(i, temp);
			if (axis == temp) {
				mComboAxis.SetCurSel(i);
				break;
			}
		}
		CString value;
		value.Format("%2.4f", tool->GetValue());
		mEditValue.SetWindowText(value);
		CString units = tool->GetUnits();
		for (i=0; i<mComboUnits.GetCount(); i++) {
			CString temp;
			mComboUnits.GetLBText(i, temp);
			if (units == temp) {
				mComboUnits.SetCurSel(i);
				break;
			}
		}
	}

	OnSelchangeAxis();
	return TRUE;
}

void CPopupToolTransform::OnSelchangeAxis() 
{
	COMBOBOXEXITEM item;
	item.mask = CBEIF_TEXT;

	int index = mComboUnits.GetCurSel(); // LB_ERR means none selected
	CString units = "";
	if (index != LB_ERR) {
		mComboUnits.GetLBText(index, units);
	}

	mComboUnits.ResetContent();

	int axis = mComboAxis.GetCurSel();
	if (axis <3) {
		item.iItem = 0;
		item.pszText = "in";
		mComboUnits.InsertItem(&item);

		item.iItem = 1;
		item.pszText = "ft";
		mComboUnits.InsertItem(&item);

		item.iItem = 2;
		item.pszText = "m";
		mComboUnits.InsertItem(&item);

		item.iItem = 3;
		item.pszText = "cm";
		mComboUnits.InsertItem(&item);

		item.iItem = 4;
		item.pszText = "mm";
		mComboUnits.InsertItem(&item);
	}
	else {
		item.iItem = 0;
		item.pszText = "degrees";
		mComboUnits.InsertItem(&item);

		item.iItem = 1;
		item.pszText = "radians";
		mComboUnits.InsertItem(&item);
	}

	mComboUnits.SetCurSel(0);
	if (index != LB_ERR && index < mComboUnits.GetCount()) {
		CString temp;
		mComboUnits.GetLBText(index, temp);
		if (temp == units) {
			mComboUnits.SetCurSel(index);
		}
	}
}
