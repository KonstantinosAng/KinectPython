; CLW file contains information for the MFC ClassWizard

[General Info]
Version=1
LastClass=CVisualCSampleDlg
LastTemplate=CDialog
NewFileInclude1=#include "stdafx.h"
NewFileInclude2=#include "Visual C Sample.h"
ODLFile=Visual C Sample.odl

ClassCount=17
Class1=CVisualCSampleApp
Class2=CVisualCSampleDlg
Class3=CAboutDlg
Class4=CVisualCSampleDlgAutoProxy

ResourceCount=12
Resource1=TAB_STARTUP
Resource2=IDD_TOOL_TRANSFORM
Class5=CProgramOptionsDlg
Class6=SensorInformationDlg
Resource3=TAB_OUTPUT_OPTIONS
Resource4=IDD_PROGRAM_OPTIONS
Resource5=TAB_GENERAL
Resource6=IDD_SETTINGS
Resource7=IDD_SENSOR_INFORMATION
Class7=CTabStartup
Class8=CTabNIDAQ
Class9=CTabGeneral
Resource8=IDD_ABOUTBOX
Class10=CSettings
Class11=COutputOptions
Class12=CToolTransforms
Class13=CPropertyFrame
Resource9=TAB_TOOL_TRANSFORMS
Resource10=TAB_NIDAQ
Class14=ToolTransforms
Class15=CTabOutputOptions
Class16=CTabToolTransforms
Resource11=IDD_VISUALCSAMPLE_DIALOG
Class17=CPopupToolTransform
Resource12=IDR_MAINMENU

[CLS:CVisualCSampleApp]
Type=0
HeaderFile=Visual C Sample.h
ImplementationFile=Visual C Sample.cpp
Filter=N
LastObject=ABOUT

[CLS:CVisualCSampleDlg]
Type=0
HeaderFile=Visual C SampleDlg.h
ImplementationFile=Visual C SampleDlg.cpp
Filter=D
LastObject=RADIO_VOLTAGES
BaseClass=CDialog
VirtualFilter=dWC

[CLS:CAboutDlg]
Type=0
HeaderFile=Visual C SampleDlg.h
ImplementationFile=Visual C SampleDlg.cpp
Filter=D

[DLG:IDD_ABOUTBOX]
Type=1
Class=CAboutDlg
ControlCount=4
Control1=IDC_STATIC,static,1342177283
Control2=IDC_STATIC,static,1342308480
Control3=IDC_STATIC,static,1342308352
Control4=IDOK,button,1342373889

[CLS:CVisualCSampleDlgAutoProxy]
Type=0
HeaderFile=DlgProxy.h
ImplementationFile=DlgProxy.cpp
BaseClass=CCmdTarget
Filter=N

[DLG:IDD_VISUALCSAMPLE_DIALOG]
Type=1
Class=CVisualCSampleDlg
ControlCount=28
Control1=GROUP_FORCE,button,1342177287
Control2=LABEL_FX,static,1342308352
Control3=LABEL_FY,static,1342308352
Control4=LABEL_FZ,static,1342308352
Control5=LABEL_FX_VAL,static,1342308352
Control6=LABEL_FY_VAL,static,1342308352
Control7=LABEL_FZ_VAL,static,1342308352
Control8=PROGRESS_FX,msctls_progress32,1342177281
Control9=PROGRESS_FY,msctls_progress32,1342177281
Control10=PROGRESS_FZ,msctls_progress32,1342177281
Control11=GROUP_TORQUE,button,1342177287
Control12=LABEL_TX,static,1342308352
Control13=LABEL_TY,static,1342308352
Control14=LABEL_TZ,static,1342308352
Control15=LABEL_TX_VAL,static,1342308352
Control16=LABEL_TY_VAL,static,1342308352
Control17=LABEL_TZ_VAL,static,1342308352
Control18=PROGRESS_TX,msctls_progress32,1342177281
Control19=PROGRESS_TY,msctls_progress32,1342177281
Control20=PROGRESS_TZ,msctls_progress32,1342177281
Control21=LABEL_MESSAGE,static,1342308352
Control22=BUTTON_BIAS,button,1342242816
Control23=BUTTON_UNBIAS,button,1342242816
Control24=LABEL_THERMISTOR,static,1476526592
Control25=GROUP_DISPLAY,button,1342177287
Control26=RADIO_VOLTAGES,button,1342308361
Control27=RADIO_FT,button,1342177289
Control28=IDC_BUTTONLOG,button,1342242816

[MNU:IDR_MAINMENU]
Type=1
Class=CVisualCSampleDlg
Command1=OPEN_CALIBRATION
Command2=OPEN_CONFIGURATION
Command3=SAVE_CONFIGURATION
Command4=EXIT
Command5=SETTINGS
Command6=SENSOR_INFORMATION
Command7=PROGRAM_OPTIONS
Command8=ABOUT
CommandCount=8

[DLG:IDD_PROGRAM_OPTIONS]
Type=1
Class=CProgramOptionsDlg
ControlCount=4
Control1=FOLDER,static,1342177280
Control2=PROGRAM_OPTIONS_OK,button,1342242817
Control3=PROGRAM_OPTIONS_CANCEL,button,1342242816
Control4=IDC_PROGRAM_OPTIONS_TAB,SysTabControl32,1342177280

[CLS:CProgramOptionsDlg]
Type=0
HeaderFile=ProgramOptionsDlg.h
ImplementationFile=ProgramOptionsDlg.cpp
BaseClass=CDialog
Filter=D
LastObject=ABOUT
VirtualFilter=dWC

[DLG:IDD_SENSOR_INFORMATION]
Type=1
Class=SensorInformationDlg
ControlCount=35
Control1=LIST_CALIBRATIONS,listbox,1352728833
Control2=SENSOR_INFORMATION_OK,button,1342242817
Control3=IDC_SENSOR_GROUP,button,1342177287
Control4=IDC_STATIC,static,1342308352
Control5=IDC_STATIC,static,1342308352
Control6=IDC_STATIC,static,1342308352
Control7=IDC_STATIC,static,1342308352
Control8=LABEL_SERIAL_NUMBER,static,1342308352
Control9=LABEL_BODY_STYLE,static,1342308352
Control10=LABEL_FAMILY,static,1342308352
Control11=LABEL_NUM_CHANNELS,static,1342308352
Control12=IDC_CALIBRATIONS_GROUP,button,1342177287
Control13=IDC_STATIC,static,1342308352
Control14=IDC_STATIC,static,1342308352
Control15=IDC_STATIC,static,1342308352
Control16=IDC_STATIC,static,1342308352
Control17=IDC_STATIC,static,1342308352
Control18=LABEL_CALIBRATION_DATE,static,1342308352
Control19=LABEL_TEMP_COMPENSATION,static,1342308352
Control20=LABEL_OUTPUT_MODE,static,1342308352
Control21=LABEL_OUTPUT_RANGE,static,1342308352
Control22=LABEL_OUTPUT_POLARITY,static,1342308352
Control23=IDC_MAX_LOADS_GROUP,button,1342177287
Control24=LABEL_AXIS_1,static,1342308352
Control25=LABEL_AXIS_MAX_1,static,1342308352
Control26=LABEL_AXIS_2,static,1342308352
Control27=LABEL_AXIS_MAX_2,static,1342308352
Control28=LABEL_AXIS_3,static,1342308352
Control29=LABEL_AXIS_MAX_3,static,1342308352
Control30=LABEL_AXIS_4,static,1342308352
Control31=LABEL_AXIS_MAX_4,static,1342308352
Control32=LABEL_AXIS_5,static,1342308352
Control33=LABEL_AXIS_MAX_5,static,1342308352
Control34=LABEL_AXIS_6,static,1342308352
Control35=LABEL_AXIS_MAX_6,static,1342308352

[CLS:SensorInformationDlg]
Type=0
HeaderFile=SensorInformationDlg.h
ImplementationFile=SensorInformationDlg.cpp
BaseClass=CDialog
Filter=D
LastObject=LABEL_FX_MAX
VirtualFilter=dWC

[DLG:TAB_GENERAL]
Type=1
Class=CTabGeneral
ControlCount=6
Control1=IDC_STATIC,static,1342308352
Control2=EDIT_TIME_INTERVAL,edit,1350631552
Control3=CHECK_BEEP,button,1342242819
Control4=CHECK_THERMISTOR,button,1342242819
Control5=IDC_STATIC,static,1342308352
Control6=IDC_CHECKAUTOLOAD,button,1342251011

[DLG:TAB_NIDAQ]
Type=1
Class=CTabNIDAQ
ControlCount=6
Control1=EDIT_DEVICE_NUMBER,edit,1350631552
Control2=IDC_STATIC,static,1342308352
Control3=IDC_STATIC,static,1342308352
Control4=IDC_STATIC,static,1342308352
Control5=EDIT_FIRST_CHANNEL_INDEX,edit,1350631552
Control6=EDIT_SCAN_RATE,edit,1350631552

[DLG:TAB_STARTUP]
Type=1
Class=CTabStartup
ControlCount=4
Control1=IDC_STATIC,button,1342177287
Control2=CHECK_LOAD_CONFIG,button,1342242819
Control3=EDIT_CONFIG_PATH,edit,1350631552
Control4=BUTTON_LOAD_CONFIG,button,1342242816

[CLS:CTabStartup]
Type=0
HeaderFile=TabStartup.h
ImplementationFile=TabStartup.cpp
BaseClass=CDialog
Filter=D
LastObject=ABOUT
VirtualFilter=dWC

[CLS:CTabNIDAQ]
Type=0
HeaderFile=TabNIDAQ.h
ImplementationFile=TabNIDAQ.cpp
BaseClass=CDialog
Filter=D
VirtualFilter=dWC
LastObject=ABOUT

[CLS:CTabGeneral]
Type=0
HeaderFile=TabGeneral.h
ImplementationFile=TabGeneral.cpp
BaseClass=CDialog
Filter=D
VirtualFilter=dWC
LastObject=CHECK_THERMISTOR

[DLG:IDD_SETTINGS]
Type=1
Class=CSettings
ControlCount=5
Control1=IDOK,button,1342242817
Control2=IDCANCEL,button,1342242816
Control3=IDC_SETTINGS_TAB,SysTabControl32,1342177280
Control4=PALETTE,static,1342177280
Control5=IDC_EDIT1,edit,1350631552

[CLS:CSettings]
Type=0
HeaderFile=Settings.h
ImplementationFile=Settings.cpp
BaseClass=CDialog
Filter=D
VirtualFilter=dWC
LastObject=ABOUT

[CLS:COutputOptions]
Type=0
HeaderFile=SettingsPropertySheet.h
ImplementationFile=SettingsPropertySheet.cpp
BaseClass=CPropertyPage

[CLS:CToolTransforms]
Type=0
HeaderFile=SettingsPropertySheet.h
ImplementationFile=SettingsPropertySheet.cpp
BaseClass=CPropertyPage
LastObject=ABOUT

[CLS:CPropertyFrame]
Type=0
HeaderFile=PropertyFrame.h
ImplementationFile=PropertyFrame.cpp
BaseClass=CMiniFrameWnd

[DLG:TAB_OUTPUT_OPTIONS]
Type=1
Class=CTabOutputOptions
ControlCount=6
Control1=CHECK_TEMP_COMP,button,1342242819
Control2=IDC_STATIC,button,1342177287
Control3=IDC_STATIC,static,1342308865
Control4=IDC_STATIC,static,1342308865
Control5=COMBOBOX_FORCE,ComboBoxEx32,1344340226
Control6=COMBOBOX_TORQUE,ComboBoxEx32,1344340226

[CLS:ToolTransforms]
Type=0
HeaderFile=ToolTransforms.h
ImplementationFile=ToolTransforms.cpp
BaseClass=CDialog
Filter=D

[CLS:CTabOutputOptions]
Type=0
HeaderFile=TabOutputOptions.h
ImplementationFile=TabOutputOptions.cpp
BaseClass=CDialog
Filter=D
VirtualFilter=dWC

[DLG:TAB_TOOL_TRANSFORMS]
Type=1
Class=CTabToolTransforms
ControlCount=5
Control1=LIST_TRANSFORMS,listbox,1352728835
Control2=BUTTON_ADD_TRANSFORM,button,1342242816
Control3=BUTTON_EDIT_TRANSFORM,button,1342242816
Control4=BUTTON_REMOVE_TRANSFORM,button,1342242816
Control5=BUTTON_REMOVE_ALL_TRANSFORMS,button,1342242816

[CLS:CTabToolTransforms]
Type=0
HeaderFile=TabToolTransforms.h
ImplementationFile=TabToolTransforms.cpp
BaseClass=CDialog
Filter=D
VirtualFilter=dWC
LastObject=BUTTON_ADD_TRANSFORM

[DLG:IDD_TOOL_TRANSFORM]
Type=1
Class=CPopupToolTransform
ControlCount=8
Control1=IDOK,button,1342242817
Control2=IDCANCEL,button,1342242816
Control3=IDC_STATIC,static,1342308865
Control4=COMBO_AXIS,ComboBoxEx32,1344340226
Control5=IDC_STATIC,static,1342308865
Control6=EDIT_VALUE,edit,1350631552
Control7=IDC_STATIC,static,1342308865
Control8=COMBO_UNITS,ComboBoxEx32,1344340226

[CLS:CPopupToolTransform]
Type=0
HeaderFile=PopupToolTransform.h
ImplementationFile=PopupToolTransform.cpp
BaseClass=CDialog
Filter=D
LastObject=COMBO_AXIS
VirtualFilter=dWC

