
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <errno.h>
// #include <getopt.h>
#include <analogy/analogy.h>
// #include <native/task.h>
// #include <native/pipe.h>
// #include <native/timer.h>
// #include <sys/mman.h>
// #include <signal.h>
// #include <fcntl.h>
// #include <boost/thread.hpp>
// #include <getopt.h>
// #include <execinfo.h>
#include "ftconfig.h" //ati daq

#define BUF_SIZE 10000

class ATIanalogy
{
public:
	ATIanalogy(); //Constuctor opens device
	~ATIanalogy();
	float read(int idx_chan);
	void loadCalibration(); //Loads calibration file
	void readBias(float T=1.0, float tafb=0.0); //bias initial values (tool weight)
	void readFT(float T=1.0, float tafb=0.0, bool swapXZ=TRUE); //read from the device
	float getFTi(int chan); //return single value
	float* getFT(); //return pointer
	void calcDeltaFTi(int chan, float T, float tafb); //calculate F/T derivative
	float getDeltaFTi(int chan); //return F/T time derivative
	void setVirtualFT(int chan, float value, float T=0.001, float tafb=0.0); //replace measurement with a virtual force/torque
	void superimposeFT(int chan, float value); //add to the measurement a virtual force/torque
private:
	Calibration *cal;
	a4l_desc_t dsc;// = { .sbdata = NULL };
	int idx_rng; //range 1 for -5V...+5V
	int idx_subd; //subdevice (0 is the Analog in)
	float bias_dev_volt[6];	//Array to save initial f/t voltages for bias
	float msr_dev_volt[6]; //Array to save current f/t voltages for filtering
	float msr_dev_FT[6];
	float msr_dev_FT_prev[6]; //save previous values
	float msr_dev_FT_prev2[6]; //save 2-previous values
	float deltaFT_prev[6]; //save 2-previous values
	
};