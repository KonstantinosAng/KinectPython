// Visual C SampleDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Visual C Sample.h"
#include "Visual C SampleDlg.h"
#include "DlgProxy.h"
#include <fstream> 
#include "ConfigureSaveData.h"
#include "RegAccess.h"
#include <direct.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define PEAK_FT 20
#define PROGRESS_GRANALITY 1000

using namespace std;

/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleDlg dialog

// just a random number to use as a signal for the timer
// doesn't matter what it is, as long as they all use the same timer
#define TIMER_1 42

IMPLEMENT_DYNAMIC(CVisualCSampleDlg, CDialog);

CVisualCSampleDlg::CVisualCSampleDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CVisualCSampleDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CVisualCSampleDlg)
	m_display = -1;
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_pAutoProxy = NULL;
	dlgProgramOptions = NULL;
	dlgSettings = NULL;
}

CVisualCSampleDlg::~CVisualCSampleDlg()
{
	// If there is an automation proxy for this dialog, set
	//  its back pointer to this dialog to NULL, so it knows
	//  the dialog has been deleted.
	if (m_pAutoProxy != NULL)
		m_pAutoProxy->m_pDialog = NULL;
	if (NULL != dlgProgramOptions)
		delete dlgProgramOptions;
	if (NULL != dlgSettings)
		delete dlgSettings;
}

void CVisualCSampleDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CVisualCSampleDlg)
	DDX_Control(pDX, IDC_BUTTONLOG, m_buttonLog);
	DDX_Control(pDX, LABEL_THERMISTOR, mLabelThermistor);
	DDX_Control(pDX, GROUP_TORQUE, mGroupTorque);
	DDX_Control(pDX, GROUP_FORCE, mGroupForce);
	DDX_Control(pDX, BUTTON_UNBIAS, mUnbias);
	DDX_Control(pDX, BUTTON_BIAS, mBias);
	DDX_Control(pDX, LABEL_MESSAGE, mLabelMessage);
	DDX_Radio(pDX, RADIO_VOLTAGES, m_display);
	//}}AFX_DATA_MAP
	// needed to hand-code this in order to set the controls up in arrays
	DDX_Control(pDX, LABEL_FX, mLabels[0]);
	DDX_Control(pDX, LABEL_FY, mLabels[1]);
	DDX_Control(pDX, LABEL_FZ, mLabels[2]);
	DDX_Control(pDX, LABEL_TX, mLabels[3]);
	DDX_Control(pDX, LABEL_TY, mLabels[4]);
	DDX_Control(pDX, LABEL_TZ, mLabels[5]);
	DDX_Control(pDX, LABEL_FX_VAL, mValues[0]);
	DDX_Control(pDX, LABEL_FY_VAL, mValues[1]);
	DDX_Control(pDX, LABEL_FZ_VAL, mValues[2]);
	DDX_Control(pDX, LABEL_TX_VAL, mValues[3]);
	DDX_Control(pDX, LABEL_TY_VAL, mValues[4]);
	DDX_Control(pDX, LABEL_TZ_VAL, mValues[5]);
	DDX_Control(pDX, PROGRESS_FX, mVoltmeter[0]);
	DDX_Control(pDX, PROGRESS_FY, mVoltmeter[1]);
	DDX_Control(pDX, PROGRESS_FZ, mVoltmeter[2]);
	DDX_Control(pDX, PROGRESS_TX, mVoltmeter[3]);
	DDX_Control(pDX, PROGRESS_TY, mVoltmeter[4]);
	DDX_Control(pDX, PROGRESS_TZ, mVoltmeter[5]);
}

BEGIN_MESSAGE_MAP(CVisualCSampleDlg, CDialog)
	ON_WM_QUERYNEWPALETTE()
	ON_WM_PALETTECHANGED()
	ON_WM_CREATE()
	//{{AFX_MSG_MAP(CVisualCSampleDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_CLOSE()
	ON_WM_TIMER()
	ON_COMMAND(ABOUT, OnABOUT)
	ON_BN_CLICKED(BUTTON_BIAS, OnBias)
	ON_BN_CLICKED(BUTTON_UNBIAS, OnUnbias)
	ON_WM_CTLCOLOR()
	ON_COMMAND(OPEN_CALIBRATION, OnOpenCalibration)
	ON_COMMAND(SENSOR_INFORMATION, OnSensorInformation)
	ON_COMMAND(PROGRAM_OPTIONS, OnProgramOptions)
	ON_COMMAND(EXIT, OnEXIT)
	ON_COMMAND(SAVE_CONFIGURATION, OnSaveConfiguration)
	ON_COMMAND(SETTINGS, OnSETTINGS)
	ON_BN_CLICKED(RADIO_VOLTAGES, OnVoltages)
	ON_BN_CLICKED(RADIO_FT, OnFt)
	ON_COMMAND(OPEN_CONFIGURATION, OnOpenConfiguration)
	ON_BN_CLICKED(IDC_BUTTONLOG, OnButtonlog)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CVisualCSampleDlg message handlers

BOOL CVisualCSampleDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	
	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	CustomInitialization();	
	
	

	/*set voltages to be default display*/
	m_display = 0;
	UpdateData(FALSE);
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CVisualCSampleDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CVisualCSampleDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CVisualCSampleDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

// Automation servers should not exit when a user closes the UI
//  if a controller still holds on to one of its objects.  These
//  message handlers make sure that if the proxy is still in use,
//  then the UI is hidden but the dialog remains around if it
//  is dismissed.

BOOL CVisualCSampleDlg::OnQueryNewPalette()
{
	// CG: This function was added by the Palette Support component

	if (m_pPalette == NULL)
		return FALSE;
	
	// BLOCK
	{
		CClientDC dc(this);
		CPalette* pOldPalette = dc.SelectPalette(m_pPalette,
			GetCurrentMessage()->message == WM_PALETTECHANGED);
		UINT nChanged = dc.RealizePalette();
		dc.SelectPalette(pOldPalette, TRUE);

		if (nChanged == 0)
			return FALSE;
	}
	
	Invalidate();
	
	return TRUE;
}

void CVisualCSampleDlg::OnPaletteChanged(CWnd* pFocusWnd)
{
	// CG: This function was added by the Palette Support component

	if (pFocusWnd == this || IsChild(pFocusWnd))
		return;
	
	OnQueryNewPalette();
}

CPalette* CVisualCSampleDlg::SetPalette(CPalette* pPalette)
{
	// CG: This function was added by the Palette Support component

	// Call this function when the palette changes.  It will
	// realize the palette in the foreground to cause the screen
	// to repaint correctly.  All calls to CDC::SelectPalette in
	// painting code should select palettes in the background.

	CPalette* pOldPalette = m_pPalette;
	m_pPalette = pPalette;
	OnQueryNewPalette();
	return pOldPalette;
}

int CVisualCSampleDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	// CG: This function was added by the Palette Support component

	if (CDialog::OnCreate(lpCreateStruct) == -1)
		return -1;

	m_pPalette = NULL;
	return 0;
}

void CVisualCSampleDlg::OnClose() 
{
	if (CanExit())
		CDialog::OnClose();
}

BOOL CVisualCSampleDlg::CanExit()
{
	// If the proxy object is still around, then the automation
	//  controller is still holding on to this application.  Leave
	//  the dialog around, but hide its UI.
	if (m_pAutoProxy != NULL)
	{
		ShowWindow(SW_HIDE);
		return FALSE;
	}

	return TRUE;
}

void CVisualCSampleDlg::CustomInitialization()
{
	CMenu* menu = this->GetMenu();
	// go ahead and turn off all of the menu operations that we don't want to allow unless certain conditions have been met
	// we can turn them back on as needed with the MF_ENABLED flag
	menu->EnableMenuItem(OPEN_CALIBRATION, MF_GRAYED);
	menu->EnableMenuItem(PROGRAM_OPTIONS, MF_GRAYED);
	menu->EnableMenuItem(SETTINGS, MF_GRAYED);
	menu->EnableMenuItem(SENSOR_INFORMATION, MF_GRAYED);
	menu->EnableMenuItem(OPEN_CONFIGURATION, MF_ENABLED);
	menu->EnableMenuItem(SAVE_CONFIGURATION, MF_GRAYED);
	if (mFTWrapper.GetStatus() & FTWrapper::DAQ_READY) {
		// DAQ loaded fine
		dlgProgramOptions = new CProgramOptionsDlg(&mFTWrapper, this);
		dlgSettings = new CSettings(&mFTWrapper, this);
		this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
		// if the DAQ is okay then we are ready to load a calibration file and/or make changes to the DAQ settings
		menu->EnableMenuItem(OPEN_CALIBRATION, MF_ENABLED);
		menu->EnableMenuItem(PROGRAM_OPTIONS, MF_ENABLED);
	}
	else {
		// something bad happened
		CString error = mFTWrapper.GetDAQErrorMessage();
		mLabelMessage.SetWindowText(error);
	}
	enableInputs();
	// Fonts are confusing.  I just wanted a larger, bolder text, but this is what I had to go through.
	// It would have been easier (if possible) to take the existing font and modify it, but I didn't see a way.
	CFont font;
	font.CreateFont(24,							// nHeight
					0,							// nWidth
					0,							// nEscapement
					0,							// nOrientation
					FW_BOLD,					// nWeight
					FALSE,						// bItalic
					FALSE,						// bUnderline
					0,							// cStrikeOut
					ANSI_CHARSET,				// nCharSet
					OUT_DEFAULT_PRECIS,			// nOutPrecision
					CLIP_DEFAULT_PRECIS,		// nClipPrecision
					DEFAULT_QUALITY,			// nQuality
					DEFAULT_PITCH | FF_SWISS,	// nPitchAndFamily
					"Arial");					// lpszFacename	
	mLabelMessage.SetFont(&font, true);

	char autoLoadFile[200]; /* the file which contains the 
							configuration to auto-load */
	if ( 0 ==
		GetRegValue( "AUTOLOAD", autoLoadFile, 200 ) )
	{
		/*Set the checkbox setting on the program options dialog*/
		dlgProgramOptions->setAutoLoad( true );
		mAutoLoadFilename = autoLoadFile;
		/*autoload the file*/
		loadConfiguration(autoLoadFile);
	}else
	{
		this->dlgProgramOptions->setAutoLoad( false );
		mAutoLoadFilename = "";
	}

	mLoggingEnabled = false;
}

void CVisualCSampleDlg::UpdateGauges(double* readings)
{
	// ignore temperature (readings[6]) for now
	for (long i=0; i<6; i++) {
		double t = readings[i];
		if (t>1e5) {
			t = 1e5;
		}
		if (t<-1e5) {
			t = -1e5;
		}
		int display = ftoi(t,(short)i);
		// clamp to the maximum saturation value
		if (display > PROGRESS_GRANALITY) {
			display = PROGRESS_GRANALITY;
//			t = (show_raw_voltages)?PEAK_VOLTAGE:PEAK_FT;
		}
		// clamp to the minimum saturation value
		if (display < 0) {
			display = 0;
//			t = -((show_raw_voltages)?PEAK_VOLTAGE:PEAK_FT);
		}
		CString str;
		str.Format("%4.4f",t);
		mValues[i].SetWindowText(str);
		mVoltmeter[i].SetPos(display);
	}
	if (dlgProgramOptions->isThermistorVisible()) {
		CString str;
		str.Format("Thermistor: %4.4f V",readings[6]);
		mLabelThermistor.SetWindowText(str);
//		mLabelThermistor.EnableWindow(true);
//		mLabelThermistor.ShowWindow(true);
	}
	else {
		mLabelThermistor.SetWindowText("");
//		mLabelThermistor.EnableWindow(false);
//		mLabelThermistor.ShowWindow(false);
	}
}

void CVisualCSampleDlg::OnTimer(UINT nIDEvent) 
{
	static bool alreadyScanning = false;	// using this boolean will help prevent 2 timer events from being processed at the same time
	if (nIDEvent == TIMER_1) {
		if (alreadyScanning == false) {
			alreadyScanning = true;
			try {
				double readings[7];
				int mDAQstatus;
				if (show_raw_voltages) {
					mDAQstatus = mFTWrapper.GetRawVoltages(readings);
				}
				else {
					mDAQstatus = mFTWrapper.GetForcesAndTorques(readings);
				}
				if (mDAQstatus) {
					if (dlgProgramOptions->getBeep()) {
						Beep(440,50);	//0x25 through 0x7FFF
					}
					mLabelMessage.SetWindowText("Saturation!");
				}
				else {
					mLabelMessage.SetWindowText("");
				}
				UpdateGauges(readings);
				if ( mLoggingEnabled )
				{
					AddToLogFile(readings);
				}
				alreadyScanning = false;
			}
			catch(CException* e)
			{
				const int N = 80;
				char errmsg[N];;
				e->GetErrorMessage(errmsg,N,NULL);
				alreadyScanning = false;
				e->Delete();
			}
			catch(char* e)
			{
				alreadyScanning = false;
			}
			catch(int e)
			{
				alreadyScanning = false;
			}
			catch (DAQException* e)
			{
				e->GetErrMsg();
				alreadyScanning = false;
			}
			catch(void* e)
			{
				alreadyScanning = false;
			}
			catch(...)
			{
				alreadyScanning = false;
			}
		}
	}
	CDialog::OnTimer(nIDEvent);
}

int CVisualCSampleDlg::ftoi(double f, short channel)
{
	if (show_raw_voltages) {
		double offset = 0.0;
		double voltageRange = mFTWrapper.getOutputRange();
		if (mFTWrapper.isBipolar()) {
			offset = voltageRange/2;
		}
		f = f+offset;				// align the signal so that 0 is the maximum negative signal
		f = f/(voltageRange);		// normalize the signal;
		f = f*PROGRESS_GRANALITY;	// scale the signal
	}
	else {
		double max_ft = PEAK_FT;
		/* mFTWrapper.GetActiveCalibrationIndex() should never return -1 here b/c a calfile has been loaded to get here (show_raw_voltages = false) */
		if (mFTWrapper.GetActiveCalibrationIndex() != -1) {
			channel++;
			max_ft = mFTWrapper.GetActiveCalibration().GetAxisMaxLoad(COleVariant(channel, VT_I2));
		}
		f = f+max_ft;				// align the signal so that 0 is the maximum negative signal
		f = f/(2*max_ft);			// normalize the signal;
		f = f*PROGRESS_GRANALITY;	// scale the signal
	}
	return (int)(f);			// convert the signal to int (this will truncate the signal, i.e., round down)
}

/*
double CVisualCSampleDlg::itof(int i)
{
	double f = i;
	if (show_raw_voltages) {
		double offset = 0.0;
		double voltageRange = mFTWrapper.getOutputRange();
		if (mFTWrapper.isBipolar()) {
			offset = voltageRange/2;
		}
		f = f/PROGRESS_GRANALITY;	// normalize the value
		f = f*(voltageRange);		// scale the signal;
		f = f-offset;				// recenter the signal
	}
	else {
		f = f/PROGRESS_GRANALITY;	// normalize the value
		f = f*(2*PEAK_FT);			// scale the signal;
		f = f-PEAK_FT;				// recenter the signal
	}
	return f;
}
*/

void CVisualCSampleDlg::OnABOUT() 
{
	bool need_to_reset_timer = false;
	if (this->KillTimer(TIMER_1)) {
		need_to_reset_timer = true;
	}
	CAboutDlg dlgAbout;
	dlgAbout.DoModal();
	if (need_to_reset_timer) {
		this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
	}
}

void CVisualCSampleDlg::OnBias() 
{
	mFTWrapper.Bias(true);
}

void CVisualCSampleDlg::OnUnbias() 
{
	mFTWrapper.Bias(false);
}



HBRUSH CVisualCSampleDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor) 
{
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);

	int i;
	for (i=0; i<6; i++) {
		CStatic* cstatic = &mValues[i];
		if (cstatic->m_hWnd == pWnd->m_hWnd) {
			char t[12];
			cstatic->GetWindowText(t,11);
			double x = atof(t);
			if (x<0) {
				pDC->SetTextColor(RGB(255,0,0));
			}
			else {
				pDC->SetTextColor(RGB(0,127,0));
			}
			return (HBRUSH) hbr;
		}
	}
	return (HBRUSH) hbr;
}

void CVisualCSampleDlg::OnOpenCalibration() 
{
	if ((mFTWrapper.GetStatus() & FTWrapper::DAQ_READY) == 0) {
		return;
	}
	bool need_to_reset_timer = false;
	if (this->KillTimer(TIMER_1)) {
		need_to_reset_timer = true;
	}
	mFTWrapper.LoadCalFile("");
//	mFTWrapper.LoadCalFile("I:/Engineering/Software/FT/DAQ FT Customer Software/Visual C Sample/input/dual.cal");
	CString filename = mFTWrapper.GetFTSensor()->GetCalFilePath();
	if (filename == "") {
//		mLabelMessage.SetWindowText("Calibration File was not loaded");
	}
	else {
//		mLabelMessage.SetWindowText("Loaded a new Calibration File: " + filename);
		CMenu* menu = this->GetMenu();
		menu->EnableMenuItem(SETTINGS, MF_ENABLED);
		menu->EnableMenuItem(SENSOR_INFORMATION, MF_ENABLED);
//		need_to_reset_timer = true;
	}
	enableInputs();
	if (need_to_reset_timer) {
		this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
	}
}

void CVisualCSampleDlg::enableInputs()
{
	bool visibility = (bool)(mFTWrapper.GetStatus() & FTWrapper::DAQ_READY);
	mBias.EnableWindow(visibility);
	mUnbias.EnableWindow(visibility);
	for (int i=0; i<6; i++) {
		mValues[i].EnableWindow(visibility);
		mVoltmeter[i].EnableWindow(visibility);
		mLabels[i].EnableWindow(visibility);
	}
	visibility = visibility && !show_raw_voltages;
	mGroupForce.EnableWindow(visibility);
	mGroupTorque.EnableWindow(visibility);
	
	visibility = (bool)( (mFTWrapper.GetStatus() & FTWrapper::ALL_READY) == FTWrapper::ALL_READY );

	/*Enable F/T Display*/
	CButton FTButton;
	HWND buttonHWND;
	GetDlgItem(RADIO_FT, &buttonHWND);
	FTButton.Attach(buttonHWND);
	FTButton.EnableWindow(visibility);
	FTButton.Detach();

	/*enable save configuration menu item*/
	CMenu * menu = this->GetMenu();	
	menu->EnableMenuItem(SAVE_CONFIGURATION, visibility ? MF_ENABLED : MF_GRAYED);
}

void CVisualCSampleDlg::OnSensorInformation() 
{
	if (mFTWrapper.GetStatus() & FTWrapper::CALFILE_LOADED) {
		bool need_to_reset_timer = false;
		if (this->KillTimer(TIMER_1)) {
			need_to_reset_timer = true;
		}
		static SensorInformationDlg sensorInfo(&mFTWrapper, this);
		sensorInfo.DoModal();
		if (need_to_reset_timer) {
			this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
		}
	}
}

void CVisualCSampleDlg::OnProgramOptions() 
{
	if (mFTWrapper.GetStatus() & FTWrapper::DAQ_READY) {
		bool need_to_reset_timer = false;
		if (this->KillTimer(TIMER_1)) {
			need_to_reset_timer = true;
		}
		dlgProgramOptions->DoModal();
		if ( dlgProgramOptions->getAutoLoad() )
		{
			if ( "" != mAutoLoadFilename )
			{
				SetRegValue( "AUTOLOAD", (LPCTSTR)mAutoLoadFilename );
			}else
			{
				MessageBox( "No auto-load file name has been specified.  Please save this configuration to auto-load it at startup.", "No Auto-load File Specified" );
			}
		}else
		{
			DeleteRegValue( "AUTOLOAD" );
		}
		if (need_to_reset_timer) {
			this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
		}
	}	
}



void CVisualCSampleDlg::OnEXIT()
{
	CVisualCSampleDlg::EndDialog(0);
}

void CVisualCSampleDlg::OnSaveConfiguration()
{
	// When Saving a config, why not ask the user if they want to use that by default, i.e. change program options	
	
	CFileDialog fileChooser(FALSE, "*.ati", NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, "ATI Save Files (*.ati)|*.ati||", this);	
	char workingDirectory[200]; /*the working directory*/
	_getdcwd( _getdrive(), workingDirectory, 200 );
	fileChooser.m_ofn.lpstrInitialDir = workingDirectory;
	if (fileChooser.DoModal() != IDOK)
		return;
	
	ofstream fileStream; /* the file we're saving data to */
	fileStream.open( (LPCTSTR)fileChooser.GetPathName(), ios::binary );		

	ConfigureSaveData csd; /*holds the information we want saved, 
						with the exception of tool transforms */
	
	/*unfortunately, we're saving more than just the calibration
	information, so we can't use the loadscript and savescript methods
	of the ftsensor object*/

	csd.fileVersion = 1;	
	CString cstrDevName = 	mFTWrapper.getDAQSys()->getDeviceName();
	//csd.DAQDeviceName = mFTWrapper.getDAQSys()->getDeviceName();
	strcpy( csd.DAQDeviceName, cstrDevName.GetBuffer(100) );
	csd.DAQStartChannel = mFTWrapper.getDAQSys()->getFirstChannel();
	csd.DAQScanRate = mFTWrapper.getDAQSys()->getScanRate();	
	csd.screenTime = dlgProgramOptions->getTimer();
	csd.ADCBeep = dlgProgramOptions->getBeep();
	csd.showTherm = dlgProgramOptions->isThermistorVisible();
	strcpy( csd.calFilePath, (LPCTSTR)mFTWrapper.GetFTSensor()->GetCalFilePath() );	
	mFTWrapper.GetBiasVoltages( csd.biasVoltages );	
	strcpy( csd.forceUnits, (LPCTSTR)mFTWrapper.GetActiveCalibration().GetForceUnits() );
	strcpy( csd.torqueUnits, (LPCTSTR)mFTWrapper.GetActiveCalibration().GetTorqueUnits() );	
	csd.tempComp = 
		mFTWrapper.GetActiveCalibration().GetTempCompEnabled();	
	_ToolTransforms TTFs = 
		mFTWrapper.GetActiveCalibration().GetToolTransforms();
	csd.numTTFs = TTFs.GetCount();
	
	fileStream.write( (char *)&csd, sizeof(csd) );
	
	unsigned int i;
	/*precondition: all data besides tool transforms already
		written to file. TTFs initialized to the tool transform
		group being saved
	  postcondition: all tool transform data in TTFs will be written
		to file. i = number of tool transforms
	*/
	for (i = 0; i < csd.numTTFs; i++)
	{
		_ToolTransform ttf = TTFs.GetItem( COleVariant( (short)(i+1), VT_I2 ) );
		TTFSaveData ttfSave;
		strcpy( ttfSave.axis, (LPCTSTR)(ttf.GetAxis()) );
		strcpy( ttfSave.units, (LPCTSTR)(ttf.GetUnits()) );
		ttfSave.value = ttf.GetValue();
		
		fileStream.write( (char *)(&ttfSave), sizeof(TTFSaveData) );
	}

	fileStream.close();

	AskAboutAutoLoad( fileChooser.GetPathName() );
}

void CVisualCSampleDlg::OnSETTINGS()
{
	if (mFTWrapper.GetStatus() & FTWrapper::CALFILE_LOADED) {
		bool need_to_reset_timer = false;
		if (this->KillTimer(TIMER_1)) {
			need_to_reset_timer = true;
		}
		dlgSettings->DoModal();
		if (need_to_reset_timer) {
			this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
		}
	}
}

void CVisualCSampleDlg::OnVoltages() 
{
	int i;
	show_raw_voltages = true;
	for (i=0; i<6; i++) {
		mVoltmeter[i].SetRange(0,PROGRESS_GRANALITY);
	}
	mGroupForce.SetWindowText("");
	mGroupTorque.SetWindowText("");
	mLabels[0].SetWindowText("G0");
	mLabels[1].SetWindowText("G1");
	mLabels[2].SetWindowText("G2");
	mLabels[3].SetWindowText("G3");
	mLabels[4].SetWindowText("G4");
	mLabels[5].SetWindowText("G5");
	if ( mLoggingEnabled )
	{
		char changeMessage[] = "Data Type Changed to Gages\n";
		mLogFile.Write( changeMessage, strlen(changeMessage) );		
		WriteLogDataHeaders();
	}
}

void CVisualCSampleDlg::OnFt() 
{
	int i;
	show_raw_voltages = false;
	for (i=0; i<6; i++) {
		mVoltmeter[i].SetRange(0,PROGRESS_GRANALITY);
	}
	mGroupForce.SetWindowText("Force");
	mGroupTorque.SetWindowText("Torque");
	mLabels[0].SetWindowText("X");
	mLabels[1].SetWindowText("Y");
	mLabels[2].SetWindowText("Z");
	mLabels[3].SetWindowText("X");
	mLabels[4].SetWindowText("Y");
	mLabels[5].SetWindowText("Z");
	if ( mLoggingEnabled )
	{
		char changeMessage[] = "Data Type Changed to Force/Torque\n";
		mLogFile.Write( changeMessage, strlen(changeMessage) );
		WriteLogDataHeaders();
	}
}

void CVisualCSampleDlg::OnOpenConfiguration() 
{
	CFileDialog fileChooser(TRUE, "*.ati", NULL, OFN_FILEMUSTEXIST, "ATI Files (*.ati)|*.ati||", this);
	char workingDirectory[200]; /*the working directory*/
	_getdcwd( _getdrive(), workingDirectory, 200 );
	fileChooser.m_ofn.lpstrInitialDir = workingDirectory; 
	if ( IDOK != fileChooser.DoModal() )
		return;

	loadConfiguration( (LPCTSTR)fileChooser.GetPathName() );
	
	AskAboutAutoLoad( fileChooser.GetPathName() );
}

/*loadConfiguration
loads a configuration, which consists of F/T settings as well
as program options.
arguments:  fileName - the file which contains the configuration
	data
*/
void CVisualCSampleDlg::loadConfiguration(LPCTSTR fileName)
{
	bool need_to_reset_timer = false;
	if (this->KillTimer(TIMER_1)) {
		need_to_reset_timer = true;
	}

	ifstream fileStream; /*input file stream*/
	fileStream.open( fileName, ios::binary);

	/*unfortunately, we're saving more than just the calibration
	information, so we can't use the loadscript and savescript methods
	of the ftsensor object*/

	ConfigureSaveData csd;

	try
	{
		fileStream.read( (char *)&csd, sizeof(csd) );	

		mFTWrapper.getDAQSys()->init( csd.DAQDeviceName,
			csd.DAQStartChannel, csd.DAQScanRate );	
		dlgProgramOptions->setTimer( csd.screenTime );
		dlgProgramOptions->setBeep( csd.ADCBeep );
		dlgProgramOptions->setThermistorVisible( csd.showTherm );
		mFTWrapper.LoadCalFile( csd.calFilePath );	
		mFTWrapper.SetBiasVoltages( csd.biasVoltages );		
		mFTWrapper.GetActiveCalibration().SetForceUnits( csd.forceUnits );
		mFTWrapper.GetActiveCalibration().SetTorqueUnits( csd.torqueUnits );	
		mFTWrapper.GetActiveCalibration().SetTempCompEnabled( csd.tempComp );
	
		_ToolTransforms TTFs = mFTWrapper.GetActiveCalibration().GetToolTransforms();
		unsigned int i;
		/*precondition: fileStream input is pointing to beginning of
			ttf section in file. TTFs is the tool transform collection
			of the sensor. csd contains the number of tool transforms
			in the file
		  postcondition:  all ttfs in file will be added to TTFs
		*/
		for (i = 0; i < csd.numTTFs; i++)
		{
			TTFSaveData ttfSave;
			fileStream.read ( (char*)(&ttfSave), sizeof(TTFSaveData) );
			TTFs.Add( ttfSave.axis, ttfSave.value, ttfSave.units );
		}	
		fileStream.close();

	} catch ( CException * e)
	{
		char caErrorMessage[2048];
		e->GetErrorMessage( caErrorMessage, 2048 );
		MessageBox( caErrorMessage, "Error Loading Configuration" );
		strcpy(csd.DAQDeviceName, "dev1" );
		strcpy(csd.torqueUnits, "N-m" );
		strcpy(csd.forceUnits, "N" );
		csd.DAQScanRate = 100;
		csd.DAQStartChannel = 0;
		csd.screenTime = 100;
		csd.ADCBeep = 1;
		csd.showTherm = true;
		mFTWrapper.getDAQSys()->init(csd.DAQDeviceName, csd.DAQStartChannel,
			csd.DAQScanRate);
		dlgProgramOptions->setTimer( csd.screenTime );
		dlgProgramOptions->setBeep( csd.ADCBeep );
		dlgProgramOptions->setThermistorVisible( csd.showTherm );	
		
	}

	

	CMenu* menu = this->GetMenu();
	menu->EnableMenuItem(SETTINGS, MF_ENABLED);
	menu->EnableMenuItem(SENSOR_INFORMATION, MF_ENABLED);
	enableInputs();
	if (need_to_reset_timer) {
		this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms
	}
}

void CVisualCSampleDlg::AskAboutAutoLoad(CString filename)
{
	/*see if they want to load this configuration next time they
		start the application */
	int msgBoxResult = MessageBox(
		"Would you like to load this configuration the next time you start the demo?",
		"Auto-Load Configuration?",
		MB_YESNO );

	if ( IDYES == msgBoxResult )
	{
		SetRegValue( "AUTOLOAD", 
				(LPCTSTR)filename );
		mAutoLoadFilename = filename;
		dlgProgramOptions->setAutoLoad( true );
	}
}

void CVisualCSampleDlg::OnButtonlog() 
{	
	bool need_to_reset_timer = false;
	if (this->KillTimer(TIMER_1)) {
		need_to_reset_timer = true;
	}
	mLoggingEnabled = !mLoggingEnabled;
	if ( mLoggingEnabled )
	{
		CFileDialog fileChooser(FALSE, "*.log", NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, "Log Files (*.log)|*.log||", this);	
		char workingDirectory[200]; /*the working directory*/
		_getdcwd( _getdrive(), workingDirectory, 200 );
		fileChooser.m_ofn.lpstrInitialDir = workingDirectory;
		if (fileChooser.DoModal() != IDOK)
			return;
		mLogFile.Open( (LPCTSTR)fileChooser.GetPathName(), 
			CFile::modeCreate | CFile::modeWrite | CFile::shareDenyWrite);
		WriteLogDataHeaders();		
		m_buttonLog.SetWindowText( "Stop Logging" );
	}else{
		mLogFile.Close();
		m_buttonLog.SetWindowText( "Log Data" );
	}
	if (need_to_reset_timer) {
		this->SetTimer(TIMER_1,dlgProgramOptions->getTimer(),NULL); // the timer is actually rounded up to the nearest 55 ms, so this will be 110 ms				
	}
	UpdateData( FALSE );
}

void CVisualCSampleDlg::AddToLogFile( double readings[] )
{
	CString dataString; /* the string written to file */	
	if ( dlgProgramOptions->isThermistorVisible() )
	{
		dataString.Format( "%-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n",
			readings[0], readings[1], readings[2], readings[3],
			readings[4], readings[5], readings[6]);
	}else
	{
		dataString.Format( "%-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n",
			readings[0], readings[1], readings[2], readings[3],
			readings[4], readings[5]);
	}
	mLogFile.Write( (LPCSTR)dataString, dataString.GetLength() );
	
}

void CVisualCSampleDlg::WriteLogDataHeaders()
{
	CString headerString; /* header line written to log file */
	if ( show_raw_voltages )
	{
		headerString.Format("%-10s %-10s %-10s %-10s %-10s %-10s",
			"G0", "G1", "G2", "G3", "G4", "G5");
	}else
	{
		headerString.Format("%-10s %-10s %-10s %-10s %-10s %-10s",
			"Fx", "Fy", "Fz", "Tx", "Ty", "Tz");
	}
	if ( dlgProgramOptions->isThermistorVisible() )
	{
		headerString = headerString + " Thermistor\n";
	}else
	{
		headerString = headerString + "\n";
	}
	mLogFile.Write( (LPCTSTR)headerString, headerString.GetLength() );
	
}
