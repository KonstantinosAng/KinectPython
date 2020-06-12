// TabToolTransforms.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "TabToolTransforms.h"
#include "PopupToolTransform.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CTabToolTransforms dialog


CTabToolTransforms::CTabToolTransforms(CWnd* pParent /*=NULL*/)
	: CDialog(CTabToolTransforms::IDD, pParent)
{
	//{{AFX_DATA_INIT(CTabToolTransforms)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CTabToolTransforms::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CTabToolTransforms)
	DDX_Control(pDX, LIST_TRANSFORMS, mListTransforms);
	DDX_Control(pDX, BUTTON_REMOVE_TRANSFORM, mButtonRemoveTransform);
	DDX_Control(pDX, BUTTON_REMOVE_ALL_TRANSFORMS, mButtonRemoveAllTransforms);
	DDX_Control(pDX, BUTTON_EDIT_TRANSFORM, mButtonEditTransform);
	DDX_Control(pDX, BUTTON_ADD_TRANSFORM, mButtonAddTransform);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CTabToolTransforms, CDialog)
	//{{AFX_MSG_MAP(CTabToolTransforms)
	ON_BN_CLICKED(BUTTON_ADD_TRANSFORM, OnAddTransform)
	ON_BN_CLICKED(BUTTON_EDIT_TRANSFORM, OnEditTransform)
	ON_BN_CLICKED(BUTTON_REMOVE_ALL_TRANSFORMS, OnRemoveAllTransforms)
	ON_BN_CLICKED(BUTTON_REMOVE_TRANSFORM, OnRemoveTransform)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CTabToolTransforms message handlers

void CTabToolTransforms::OnAddTransform() 
{
//	tools.Add("Dx", 1.0, "in");
	CPopupToolTransform popup(&tools);
	popup.DoModal();
	updateList();
}

void CTabToolTransforms::OnEditTransform()
{
	int index = mListTransforms.GetCurSel();
	if (index != LB_ERR) {
		_ToolTransform tool = tools.GetItem(COleVariant((short)(index+1),VT_I2));
		CPopupToolTransform popup(&tool);
		popup.DoModal();
		updateList();
	}
}

void CTabToolTransforms::OnRemoveAllTransforms() 
{
	tools.RemoveAll();
	updateList();
}

void CTabToolTransforms::OnRemoveTransform() 
{
	int index = mListTransforms.GetCurSel();
	if (index != LB_ERR) {
		// delete the transform with this index
		tools.Remove(COleVariant((short)(index+1), VT_I2));
		updateList();
	}
}

void CTabToolTransforms::SetFTWrapper(FTWrapper *wrapper)
{
	this->wrapper = wrapper;
	this->calibration = wrapper->GetActiveCalibration();
	this->tools = calibration.GetToolTransforms();
//	updateList();
}

void CTabToolTransforms::updateList()
{
	long num = tools.GetCount();
	mListTransforms.ResetContent();
	for (short i=0; i<(short)(num); i++) {
		_ToolTransform tool = tools.GetItem(COleVariant((short)(i+1),VT_I2));
		CString axis = tool.GetAxis();
		double value = tool.GetValue();
		CString units = tool.GetUnits();
		CString f;
		f.Format("%s: %2.4f %s", axis, value, units);
		mListTransforms.InsertString(i, f);
	}
}

BOOL CTabToolTransforms::OnInitDialog()
{
	CDialog::OnInitDialog();

	updateList();

	return TRUE;
}
