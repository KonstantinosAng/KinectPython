# Microsoft Developer Studio Generated NMAKE File, Based on Visual C Sample.dsp
!IF "$(CFG)" == ""
CFG=Visual C Sample - Win32 Debug
!MESSAGE No configuration specified. Defaulting to Visual C Sample - Win32 Debug.
!ENDIF 

!IF "$(CFG)" != "Visual C Sample - Win32 Release" && "$(CFG)" != "Visual C Sample - Win32 Debug"
!MESSAGE Invalid configuration "$(CFG)" specified.
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "Visual C Sample.mak" CFG="Visual C Sample - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "Visual C Sample - Win32 Release" (based on "Win32 (x86) Application")
!MESSAGE "Visual C Sample - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 
!ERROR An invalid configuration is specified.
!ENDIF 

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE 
NULL=nul
!ENDIF 

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"

OUTDIR=.\Release
INTDIR=.\Release
# Begin Custom Macros
OutDir=.\Release
# End Custom Macros

ALL : "$(OUTDIR)\Visual C Sample.exe" "$(OUTDIR)\Visual C Sample.tlb"


CLEAN :
	-@erase "$(INTDIR)\atidaqft.obj"
	-@erase "$(INTDIR)\DAQSys.obj"
	-@erase "$(INTDIR)\DlgProxy.obj"
	-@erase "$(INTDIR)\FTWrapper.obj"
	-@erase "$(INTDIR)\PopupToolTransform.obj"
	-@erase "$(INTDIR)\ProgramOptionsDlg.obj"
	-@erase "$(INTDIR)\SensorInformationDlg.obj"
	-@erase "$(INTDIR)\Settings.obj"
	-@erase "$(INTDIR)\StdAfx.obj"
	-@erase "$(INTDIR)\TabGeneral.obj"
	-@erase "$(INTDIR)\TabNIDAQ.obj"
	-@erase "$(INTDIR)\TabOutputOptions.obj"
	-@erase "$(INTDIR)\TabStartup.obj"
	-@erase "$(INTDIR)\TabToolTransforms.obj"
	-@erase "$(INTDIR)\vc60.idb"
	-@erase "$(INTDIR)\Visual C Sample.obj"
	-@erase "$(INTDIR)\Visual C Sample.pch"
	-@erase "$(INTDIR)\Visual C Sample.res"
	-@erase "$(INTDIR)\Visual C Sample.tlb"
	-@erase "$(INTDIR)\Visual C SampleDlg.obj"
	-@erase "$(OUTDIR)\Visual C Sample.exe"

"$(OUTDIR)" :
    if not exist "$(OUTDIR)/$(NULL)" mkdir "$(OUTDIR)"

CPP=cl.exe
CPP_PROJ=/nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /Fp"$(INTDIR)\Visual C Sample.pch" /Yu"stdafx.h" /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /c 

.c{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.c{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

MTL=midl.exe
MTL_PROJ=/nologo /D "NDEBUG" /mktyplib203 /win32 
RSC=rc.exe
RSC_PROJ=/l 0x409 /fo"$(INTDIR)\Visual C Sample.res" /i "C:\Program Files\National Instruments\NI - DAQ\Include\\" /d "NDEBUG" /d "_AFXDLL" 
BSC32=bscmake.exe
BSC32_FLAGS=/nologo /o"$(OUTDIR)\Visual C Sample.bsc" 
BSC32_SBRS= \
	
LINK32=link.exe
LINK32_FLAGS=/nologo /subsystem:windows /incremental:no /pdb:"$(OUTDIR)\Visual C Sample.pdb" /machine:I386 /out:"$(OUTDIR)\Visual C Sample.exe" 
LINK32_OBJS= \
	"$(INTDIR)\atidaqft.obj" \
	"$(INTDIR)\DAQSys.obj" \
	"$(INTDIR)\DlgProxy.obj" \
	"$(INTDIR)\FTWrapper.obj" \
	"$(INTDIR)\PopupToolTransform.obj" \
	"$(INTDIR)\ProgramOptionsDlg.obj" \
	"$(INTDIR)\SensorInformationDlg.obj" \
	"$(INTDIR)\Settings.obj" \
	"$(INTDIR)\StdAfx.obj" \
	"$(INTDIR)\TabGeneral.obj" \
	"$(INTDIR)\TabNIDAQ.obj" \
	"$(INTDIR)\TabOutputOptions.obj" \
	"$(INTDIR)\TabStartup.obj" \
	"$(INTDIR)\TabToolTransforms.obj" \
	"$(INTDIR)\Visual C Sample.obj" \
	"$(INTDIR)\Visual C SampleDlg.obj" \
	"$(INTDIR)\Visual C Sample.res" \
	"C:\Program Files\National Instruments\NI - DAQ\Lib\nidaq32.lib" \
	"C:\Program Files\National Instruments\NI - DAQ\Lib\nidex32.lib"

"$(OUTDIR)\Visual C Sample.exe" : "$(OUTDIR)" $(DEF_FILE) $(LINK32_OBJS)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(LINK32_OBJS)
<<

!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"

OUTDIR=.\Debug
INTDIR=.\Debug
# Begin Custom Macros
OutDir=.\Debug
# End Custom Macros

ALL : "$(OUTDIR)\Visual C Sample.exe" "$(OUTDIR)\Visual C Sample.tlb" "$(OUTDIR)\Visual C Sample.bsc"


CLEAN :
	-@erase "$(INTDIR)\atidaqft.obj"
	-@erase "$(INTDIR)\atidaqft.sbr"
	-@erase "$(INTDIR)\DAQSys.obj"
	-@erase "$(INTDIR)\DAQSys.sbr"
	-@erase "$(INTDIR)\DlgProxy.obj"
	-@erase "$(INTDIR)\DlgProxy.sbr"
	-@erase "$(INTDIR)\FTWrapper.obj"
	-@erase "$(INTDIR)\FTWrapper.sbr"
	-@erase "$(INTDIR)\PopupToolTransform.obj"
	-@erase "$(INTDIR)\PopupToolTransform.sbr"
	-@erase "$(INTDIR)\ProgramOptionsDlg.obj"
	-@erase "$(INTDIR)\ProgramOptionsDlg.sbr"
	-@erase "$(INTDIR)\SensorInformationDlg.obj"
	-@erase "$(INTDIR)\SensorInformationDlg.sbr"
	-@erase "$(INTDIR)\Settings.obj"
	-@erase "$(INTDIR)\Settings.sbr"
	-@erase "$(INTDIR)\StdAfx.obj"
	-@erase "$(INTDIR)\StdAfx.sbr"
	-@erase "$(INTDIR)\TabGeneral.obj"
	-@erase "$(INTDIR)\TabGeneral.sbr"
	-@erase "$(INTDIR)\TabNIDAQ.obj"
	-@erase "$(INTDIR)\TabNIDAQ.sbr"
	-@erase "$(INTDIR)\TabOutputOptions.obj"
	-@erase "$(INTDIR)\TabOutputOptions.sbr"
	-@erase "$(INTDIR)\TabStartup.obj"
	-@erase "$(INTDIR)\TabStartup.sbr"
	-@erase "$(INTDIR)\TabToolTransforms.obj"
	-@erase "$(INTDIR)\TabToolTransforms.sbr"
	-@erase "$(INTDIR)\vc60.idb"
	-@erase "$(INTDIR)\vc60.pdb"
	-@erase "$(INTDIR)\Visual C Sample.obj"
	-@erase "$(INTDIR)\Visual C Sample.pch"
	-@erase "$(INTDIR)\Visual C Sample.res"
	-@erase "$(INTDIR)\Visual C Sample.sbr"
	-@erase "$(INTDIR)\Visual C Sample.tlb"
	-@erase "$(INTDIR)\Visual C SampleDlg.obj"
	-@erase "$(INTDIR)\Visual C SampleDlg.sbr"
	-@erase "$(OUTDIR)\Visual C Sample.bsc"
	-@erase "$(OUTDIR)\Visual C Sample.exe"
	-@erase "$(OUTDIR)\Visual C Sample.ilk"
	-@erase "$(OUTDIR)\Visual C Sample.pdb"

"$(OUTDIR)" :
    if not exist "$(OUTDIR)/$(NULL)" mkdir "$(OUTDIR)"

CPP=cl.exe
CPP_PROJ=/nologo /MDd /W4 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /FR"$(INTDIR)\\" /Fp"$(INTDIR)\Visual C Sample.pch" /Yu"stdafx.h" /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /GZ /c 

.c{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{$(INTDIR)}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.c{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{$(INTDIR)}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

MTL=midl.exe
MTL_PROJ=/nologo /D "_DEBUG" /mktyplib203 /win32 
RSC=rc.exe
RSC_PROJ=/l 0x409 /fo"$(INTDIR)\Visual C Sample.res" /i "C:\Program Files\National Instruments\NI - DAQ\Include\\" /d "_DEBUG" /d "_AFXDLL" 
BSC32=bscmake.exe
BSC32_FLAGS=/nologo /o"$(OUTDIR)\Visual C Sample.bsc" 
BSC32_SBRS= \
	"$(INTDIR)\atidaqft.sbr" \
	"$(INTDIR)\DAQSys.sbr" \
	"$(INTDIR)\DlgProxy.sbr" \
	"$(INTDIR)\FTWrapper.sbr" \
	"$(INTDIR)\PopupToolTransform.sbr" \
	"$(INTDIR)\ProgramOptionsDlg.sbr" \
	"$(INTDIR)\SensorInformationDlg.sbr" \
	"$(INTDIR)\Settings.sbr" \
	"$(INTDIR)\StdAfx.sbr" \
	"$(INTDIR)\TabGeneral.sbr" \
	"$(INTDIR)\TabNIDAQ.sbr" \
	"$(INTDIR)\TabOutputOptions.sbr" \
	"$(INTDIR)\TabStartup.sbr" \
	"$(INTDIR)\TabToolTransforms.sbr" \
	"$(INTDIR)\Visual C Sample.sbr" \
	"$(INTDIR)\Visual C SampleDlg.sbr"

"$(OUTDIR)\Visual C Sample.bsc" : "$(OUTDIR)" $(BSC32_SBRS)
    $(BSC32) @<<
  $(BSC32_FLAGS) $(BSC32_SBRS)
<<

LINK32=link.exe
LINK32_FLAGS=/nologo /subsystem:windows /incremental:yes /pdb:"$(OUTDIR)\Visual C Sample.pdb" /debug /machine:I386 /out:"$(OUTDIR)\Visual C Sample.exe" /pdbtype:sept /libpath:"C:\Program Files\National Instruments\NI-DAQ\Lib\\" 
LINK32_OBJS= \
	"$(INTDIR)\atidaqft.obj" \
	"$(INTDIR)\DAQSys.obj" \
	"$(INTDIR)\DlgProxy.obj" \
	"$(INTDIR)\FTWrapper.obj" \
	"$(INTDIR)\PopupToolTransform.obj" \
	"$(INTDIR)\ProgramOptionsDlg.obj" \
	"$(INTDIR)\SensorInformationDlg.obj" \
	"$(INTDIR)\Settings.obj" \
	"$(INTDIR)\StdAfx.obj" \
	"$(INTDIR)\TabGeneral.obj" \
	"$(INTDIR)\TabNIDAQ.obj" \
	"$(INTDIR)\TabOutputOptions.obj" \
	"$(INTDIR)\TabStartup.obj" \
	"$(INTDIR)\TabToolTransforms.obj" \
	"$(INTDIR)\Visual C Sample.obj" \
	"$(INTDIR)\Visual C SampleDlg.obj" \
	"$(INTDIR)\Visual C Sample.res" \
	"C:\Program Files\National Instruments\NI - DAQ\Lib\nidaq32.lib" \
	"C:\Program Files\National Instruments\NI - DAQ\Lib\nidex32.lib"

"$(OUTDIR)\Visual C Sample.exe" : "$(OUTDIR)" $(DEF_FILE) $(LINK32_OBJS)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(LINK32_OBJS)
<<

!ENDIF 


!IF "$(NO_EXTERNAL_DEPS)" != "1"
!IF EXISTS("Visual C Sample.dep")
!INCLUDE "Visual C Sample.dep"
!ELSE 
!MESSAGE Warning: cannot find "Visual C Sample.dep"
!ENDIF 
!ENDIF 


!IF "$(CFG)" == "Visual C Sample - Win32 Release" || "$(CFG)" == "Visual C Sample - Win32 Debug"
SOURCE=.\atidaqft.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\atidaqft.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\atidaqft.obj"	"$(INTDIR)\atidaqft.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\DAQSys.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\DAQSys.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\DAQSys.obj"	"$(INTDIR)\DAQSys.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\DlgProxy.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\DlgProxy.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\DlgProxy.obj"	"$(INTDIR)\DlgProxy.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\FTWrapper.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\FTWrapper.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\FTWrapper.obj"	"$(INTDIR)\FTWrapper.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\PopupToolTransform.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\PopupToolTransform.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\PopupToolTransform.obj"	"$(INTDIR)\PopupToolTransform.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\ProgramOptionsDlg.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\ProgramOptionsDlg.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\ProgramOptionsDlg.obj"	"$(INTDIR)\ProgramOptionsDlg.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\SensorInformationDlg.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\SensorInformationDlg.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\SensorInformationDlg.obj"	"$(INTDIR)\SensorInformationDlg.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\Settings.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\Settings.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\Settings.obj"	"$(INTDIR)\Settings.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\StdAfx.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"

CPP_SWITCHES=/nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /Fp"$(INTDIR)\Visual C Sample.pch" /Yc"stdafx.h" /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /c 

"$(INTDIR)\StdAfx.obj"	"$(INTDIR)\Visual C Sample.pch" : $(SOURCE) "$(INTDIR)"
	$(CPP) @<<
  $(CPP_SWITCHES) $(SOURCE)
<<


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"

CPP_SWITCHES=/nologo /MDd /W4 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /FR"$(INTDIR)\\" /Fp"$(INTDIR)\Visual C Sample.pch" /Yc"stdafx.h" /Fo"$(INTDIR)\\" /Fd"$(INTDIR)\\" /FD /GZ /c 

"$(INTDIR)\StdAfx.obj"	"$(INTDIR)\StdAfx.sbr"	"$(INTDIR)\Visual C Sample.pch" : $(SOURCE) "$(INTDIR)"
	$(CPP) @<<
  $(CPP_SWITCHES) $(SOURCE)
<<


!ENDIF 

SOURCE=.\TabGeneral.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\TabGeneral.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\TabGeneral.obj"	"$(INTDIR)\TabGeneral.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\TabNIDAQ.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\TabNIDAQ.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\TabNIDAQ.obj"	"$(INTDIR)\TabNIDAQ.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\TabOutputOptions.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\TabOutputOptions.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\TabOutputOptions.obj"	"$(INTDIR)\TabOutputOptions.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\TabStartup.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\TabStartup.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\TabStartup.obj"	"$(INTDIR)\TabStartup.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=.\TabToolTransforms.cpp

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\TabToolTransforms.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\TabToolTransforms.obj"	"$(INTDIR)\TabToolTransforms.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=".\Visual C Sample.cpp"

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\Visual C Sample.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\Visual C Sample.obj"	"$(INTDIR)\Visual C Sample.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 

SOURCE=".\Visual C Sample.odl"

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"

MTL_SWITCHES=/nologo /D "NDEBUG" /tlb "$(OUTDIR)\Visual C Sample.tlb" /mktyplib203 /win32 

"$(OUTDIR)\Visual C Sample.tlb" : $(SOURCE) "$(OUTDIR)"
	$(MTL) @<<
  $(MTL_SWITCHES) $(SOURCE)
<<


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"

MTL_SWITCHES=/nologo /D "_DEBUG" /tlb "$(OUTDIR)\Visual C Sample.tlb" /mktyplib203 /win32 

"$(OUTDIR)\Visual C Sample.tlb" : $(SOURCE) "$(OUTDIR)"
	$(MTL) @<<
  $(MTL_SWITCHES) $(SOURCE)
<<


!ENDIF 

SOURCE=".\Visual C Sample.rc"

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\Visual C Sample.res" : $(SOURCE) "$(INTDIR)"
	$(RSC) /l 0x409 /fo"$(INTDIR)\Visual C Sample.res" /i "C:\Program Files\National Instruments\NI - DAQ\Include\\" /i "Release" /d "NDEBUG" /d "_AFXDLL" $(SOURCE)


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\Visual C Sample.res" : $(SOURCE) "$(INTDIR)"
	$(RSC) /l 0x409 /fo"$(INTDIR)\Visual C Sample.res" /i "C:\Program Files\National Instruments\NI - DAQ\Include\\" /i "Debug" /d "_DEBUG" /d "_AFXDLL" $(SOURCE)


!ENDIF 

SOURCE=".\Visual C SampleDlg.cpp"

!IF  "$(CFG)" == "Visual C Sample - Win32 Release"


"$(INTDIR)\Visual C SampleDlg.obj" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ELSEIF  "$(CFG)" == "Visual C Sample - Win32 Debug"


"$(INTDIR)\Visual C SampleDlg.obj"	"$(INTDIR)\Visual C SampleDlg.sbr" : $(SOURCE) "$(INTDIR)" "$(INTDIR)\Visual C Sample.pch"


!ENDIF 


!ENDIF 

