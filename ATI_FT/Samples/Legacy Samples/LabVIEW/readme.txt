ATI Industrial Automation
DAQ F/T VI Library

This VI library is provided to help you integrate your DAQ Force/Torque transducer system into your LabVIEW application.  The library includes a several VIs which encapsulate the functionality of the ATIDAQFT Automation server, plus a Getting Started VI that demonstrates the VI library in a simple application.

Requirements:
LabVIEW 7 or later
ATIDAQFT ActiveX Automation server (0.6.5 or later), included with ATI's Traditional NI-DAQ software.
Traditional NI-DAQ drivers (7.4 or later) from National Instruments.

Basic VIs
---------
For most applications, the following basic set of VIs will be sufficient:
-DAQFT GetCalibration.vi
-DAQFT GetAxisInfo.vi
-DAQFT ConfigDAQ.vi
-DAQFT Bias.vi
-DAQFT GetF/T.vi

You may have to modify the VIs (as described below) if your application involves:
-hardware devices other than National Instruments E Series devices
-high speed (buffered) DAQ operations

Using this library with another brand of DAQ device
---------------------------------------------------
To use another brand of DAQ device, you must change the implementation in the following subVIs:
-DAQFT ConfigDAQ.vi--configure your DAQ device with an appropriate input range, etc.
-DAQFT Acquire.vi--acquire an array of voltages from the DAQ device
You must also implement ADC saturation checking for your system.  Some DAQ devices may provide saturation condition as a return value from the read operation, as an exception error, or as a separate function call.  If none of these methods is supported (as is the case with National Instruments devices), each channel must be tested individually by comparing to the channel's minimum and maximum readings.  Checking for saturation is important, because an F/T measurement is invalid if any one fo the input channels is saturated.

High-Speed Data Acquisition Applications
----------------------------------------
For high speed applications, set up a buffered data acquisition operation, then use the Convert VI to convert the readings to forces and torques.


Overview of VIs
---------------
DAQFT Acquire.vi
This VI performs the NI-DAQ AI Single Scan VI, then checks the binary reading for a saturation condition.  This VI must be modified if you are not using a National Intruments DAQ device.

DAQFT Bias.vi
This VI stores a bias measurement (baseline reading) that will be subtracted from future calculations by the ConvertToFT method (Convert VI).  This VI uses the DAQ F/T Acquire VI, so it must be modified if you are not using a National Instruments DAQ device.

DAQFT ConfigDAQ.vi
This VI configures a National Instruments E Series DAQ device for typical use with a DAQ F/T system.  Channels 0-6 are used by default.

DAQFT Convert.vi
This VI uses the ATIDAQFT component to convert raw AI voltages into forces and torques.  This VI requires a reference to a valid Calibration object from the GetCalibration VI.

DAQFT Get F/T.vi
This high-level VI provides the most current F/T data record by acquiring raw voltages from a National Instruments DAQ device, then converting the measurement to forces and torques.This VI must be modified if other brands of DAQ device are used.

DAQFT GetAxisInfo.vi
This VI retrieves axis information from a DAQ F/T Calibration object by the index of the axis, including the maximum load, units, and name of the axis.  Axis indices are typically 1-6 for Fx, Fy, Fz, Tx, Ty, Tz.

DAQFT GetBinaryRange.vi
This VI collects the minimum and maximum binary readings from the NI DAQ device analog inputs based on the current configuration.  This information is used by the Saturation Check VI later to check for a saturated gage condition.  This condition is necessary to monitor because F/T data is invalid if any analog channels are saturated.

DAQFT GetCalibration.vi
This VI uses the ATIDAQFT component to load a DAQ F/T calibration file and retrieve a reference to a Calibration object that can be used for configuration and data conversion for the DAQ F/T system.

DAQFT GetOutputInfo.vi
This VI retrieves output information from a DAQ F/T Calibration object, including the output range, output mode, and output polarity of the transducer system.

DAQFT Getting Started.vi
This VI is intended to give an overview of the VIs included in the ATI DAQ F/T VI Library.  The front panel contains basic instructions for using the VI.

DAQFT SaturationCheck.vi
This VI checks for saturation error on all channels of a DAQ F/T reading.  This VI must be modified if you are not using a National Instruments DAQ device.