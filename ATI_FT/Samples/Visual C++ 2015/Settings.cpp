// Settings.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "Settings.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CSettings dialog


CSettings::CSettings(FTWrapper* wrapper, CWnd* pParent /*=NULL*/)
	: CDialog(CSettings::IDD, pParent)
{
	//{{AFX_DATA_INIT(CSettings)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
	this->wrapper = wrapper;
	force = 0;
	torque = 0;
}


void CSettings::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CSettings)
	DDX_Control(pDX, PALETTE, mPalette);
	DDX_Control(pDX, IDC_SETTINGS_TAB, mSettingsTab);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CSettings, CDialog)
	//{{AFX_MSG_MAP(CSettings)
	ON_NOTIFY(TCN_SELCHANGE, IDC_SETTINGS_TAB, OnSelchangeSettingsTab)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CSettings message handlers


BOOL CSettings::OnInitDialog()
{
	CDialog::OnInitDialog();
	mToolTransforms.SetFTWrapper(wrapper);

	TCITEM tcOutputOptions;
	tcOutputOptions.mask = TCIF_TEXT;
	tcOutputOptions.pszText = "Output Options";
	mSettingsTab.InsertItem(0, &tcOutputOptions);
	mOutputOptions.Create(TAB_OUTPUT_OPTIONS, &mPalette);
	mOutputOptions.EnableWindow(true);

	TCITEM tcToolTransforms;
	tcToolTransforms.mask = TCIF_TEXT;
	tcToolTransforms.pszText = "Tool Transforms";
	mSettingsTab.InsertItem(1, &tcToolTransforms);
	mToolTransforms.Create(TAB_TOOL_TRANSFORMS, &mPalette);
	mToolTransforms.EnableWindow(true);

	mOutputOptions.ShowWindow(true);

	_Calibration calibration = wrapper->GetActiveCalibration();
	for (int i=0; i<mOutputOptions.mComboForce.GetCount(); i++) {
		CString forceUnits;
		mOutputOptions.mComboForce.GetLBText(i, forceUnits);
		if (calibration.GetForceUnits() == forceUnits) {
			force = i;
			break;
		}
	}
	for (int j=0; j<mOutputOptions.mComboTorque.GetCount(); j++) {
		CString torqueUnits;
		mOutputOptions.mComboTorque.GetLBText(j, torqueUnits);
		if (calibration.GetTorqueUnits() == torqueUnits) {
			torque = j;
			break;
		}
	}
	
	mOutputOptions.mCheckboxTempComp.SetCheck(wrapper->GetActiveCalibration().GetTempCompEnabled());
	mOutputOptions.mComboForce.SetCurSel(force);
	mOutputOptions.mComboTorque.SetCurSel(torque);

	return TRUE;
}

void CSettings::OnSelchangeSettingsTab(NMHDR* pNMHDR, LRESULT* pResult) 
{
	switch(mSettingsTab.GetCurSel()) {
	case 0:
		mOutputOptions.ShowWindow(true);
		mToolTransforms.ShowWindow(false);
		mOutputOptions.SetForegroundWindow();
		break;
	case 1:
		mOutputOptions.ShowWindow(false);
		mToolTransforms.ShowWindow(true);
		mToolTransforms.SetForegroundWindow();
		break;
	default:
		;
	};
	
	*pResult = 0;
}

void CSettings::OnOK() 
{
	_Calibration calibration = wrapper->GetActiveCalibration();
	calibration.SetTempCompEnabled(mOutputOptions.mCheckboxTempComp.GetCheck());
	force = mOutputOptions.mComboForce.GetCurSel();
	if (force != LB_ERR) {
		CString forceUnits;
		mOutputOptions.mComboForce.GetLBText(force, forceUnits);
		calibration.SetForceUnits(forceUnits);
	}
	torque = mOutputOptions.mComboTorque.GetCurSel();
	if (torque != LB_ERR) {
		CString torqueUnits;
		mOutputOptions.mComboTorque.GetLBText(torque, torqueUnits);
		calibration.SetTorqueUnits(torqueUnits);
	}
	CDialog::OnOK();
}

void CSettings::OnCancel() 
{
	CDialog::OnCancel();
}
