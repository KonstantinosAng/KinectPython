#include "ATIanalogy.h"

#define FILENAME "analogy0" //select device name
static char *filename = FILENAME;

/// Constructor. Used to connect to the device.
ATIanalogy::ATIanalogy()
{
	idx_subd = 0; //select Subdevice
	idx_rng = 0; //select range (0 for 10V, 1 for 5V, 2 for ... , 3 for ...)
	for (int i=0; i<6; i++) {
		bias_dev_volt[i] = 0.0;
		msr_dev_volt[i] = 0.0;
		msr_dev_FT[i] = 0.0;
		msr_dev_FT_prev[i] = 0.0; //save previous values
		msr_dev_FT_prev2[i] = 0.0;
		deltaFT_prev[i] = 0.0;
	}

	int err = 0;
	a4l_sbinfo_t *sbinfo;
	a4l_chinfo_t *chinfo;
	a4l_rnginfo_t *rnginfo;

	// static unsigned char buf[BUF_SIZE];
	//Open device
	//cout << "Opening analog in device... ";
	err = a4l_open(&dsc, filename);
	if (err < 0) {
		fprintf(stderr,
			"insn_read: a4l_open %s failed (err=%d)\n",
			filename, err);
		return;
	}
	 // Allocate a buffer so as to get more info (subd, chan, rng) 
	dsc.sbdata = malloc(dsc.sbsize);
	if (dsc.sbdata == NULL) {
		err = -ENOMEM;
		fprintf(stderr, "insn_read: info buffer allocation failed\n");
		return;
	}

	 // Get this data 
	err = a4l_fill_desc(&dsc);
	if (err < 0) {
		fprintf(stderr, "insn_read: a4l_fill_desc failed (err=%d)\n",
			err);
		return;
	}

	 // We must check that the subdevice is really an AI one (in case, the subdevice index was set with the option -s) 
	err = a4l_get_subdinfo(&dsc, idx_subd, &sbinfo);
	if (err < 0) {
		fprintf(stderr,
			"insn_read: get_sbinfo(%d) failed (err = %d)\n",
			idx_subd, err);
		err = -EINVAL;
		return;
	}
	printf("Successfully opened ATI device.\n");

}

ATIanalogy::~ATIanalogy()
{
	destroyCalibration(cal);
	printf("Closing ATI device.\n");
	a4l_close(&dsc);
}

/// Read the raw values from a certain channel and return the voltage
float ATIanalogy::read(int idx_chan)
{
	char buf[128];
	a4l_sync_read(&dsc, idx_subd, CHAN(idx_chan), 0, buf, 2);

	int err = 0, width, tmp_size = 0;
	a4l_chinfo_t *chan;
	a4l_rnginfo_t *rng;

	 // Retrieve the channel info 
	err = a4l_get_chinfo(&dsc, idx_subd, idx_chan, &chan);
	if (err < 0) {
		fprintf(stderr,
			"insn_read: info for channel %d "
			"on subdevice %d not available (err=%d)\n",
			idx_chan, idx_subd, err);
		return -1;
	}

	 // Retrieve the range info 
	err = a4l_get_rnginfo(&dsc, idx_subd, idx_chan, idx_rng, &rng);
	if (err < 0) {
		fprintf(stderr,
			"insn_read: failed to recover range descriptor\n");
		return -1;
	}

	width = a4l_sizeof_chan(chan);
	if (width < 0) {
		fprintf(stderr,
			"insn_read: incoherent info for channel %d\n",
			idx_chan);
		err = width;
		return -1;
	}

	double value[1];
	
	err = a4l_rawtod(chan, rng, value, buf, 1);
	if (err < 0)
		return -1;

	// // fprintf(stdout, "%f\n", value);


	return (float)value[0];
}

// void ATIanalogy::read_raw(int idx_chan, char* buf)
// {
// 	a4l_sync_read(&dsc, idx_subd, CHAN(idx_chan), 0, buf, 2);
// 	return;
// }

/** \brief Bias initial values.
- To get better bias run this function a few times using the follwing argumengs:
1nd argument is the sampling period
2st argument is the time constant for 1st order filter in bias (for better bias)
- Calling this function without arguments simply saves the bias without any filtering.
*/
void ATIanalogy::readBias(float T, float tafb){
	for(int chan = 0; chan < 6; ++chan) {
		bias_dev_volt[chan]=(read(chan)*T+bias_dev_volt[chan]*tafb)/(tafb+T);
	}
	Bias(cal,bias_dev_volt);  //store an unloaded measurement and remove the effect of tooling weight
}

/** \brief Measure Forces & Torques from external device
Calling this function without arguments simply reads the forces/torques without any filtering.
*/
void ATIanalogy::readFT(float T, float tafb, bool swapXZ){
		
	for(int chan = 0; chan < 6; ++chan)
	{
		msr_dev_volt[chan]=read(chan); //read and 1st order low-pass
	}
	ConvertToFT(cal,msr_dev_volt,msr_dev_FT); //voltages to forces/torques
	
	for(int chan = 0; chan < 6; ++chan)
	{
		msr_dev_FT[chan]=(msr_dev_FT[chan]*T+msr_dev_FT_prev[chan]*tafb)/(tafb+T); //1st order low-pass

		//calc rate of change in FT (filtered)
		calcDeltaFTi(chan, T, 5.0*tafb);

		msr_dev_FT_prev2[chan] = msr_dev_FT_prev[chan];
		msr_dev_FT_prev[chan] = msr_dev_FT[chan];
	}

	if (swapXZ) { //swap Ax with Az (because of the Jacobian Euler convention)
		float tmp = msr_dev_FT[3];
		msr_dev_FT[3] = msr_dev_FT[5];
		msr_dev_FT[5] = tmp;
	}
}

/// Return the force torque value of the selected channel
float ATIanalogy::getFTi(int chan){
	return msr_dev_FT[chan];
}

/// Return the pointer of the FT vector
float* ATIanalogy::getFT(){
	return msr_dev_FT;
}

/// Calculate the force/torque derivative of a channel with a low pass filtering
void ATIanalogy::calcDeltaFTi(int chan, float T, float tafb){
	deltaFT_prev[chan] = ((msr_dev_FT[chan]-msr_dev_FT_prev[chan])+deltaFT_prev[chan]*tafb)/(tafb+T);
}

/// Return the calculated derivative
float ATIanalogy::getDeltaFTi(int chan){
	return deltaFT_prev[chan]; //actually this is the current filtered value
}

/// Superimpose a virtual force/torque to the measurement of the desired channel with a low pass filtering for smoothness
void ATIanalogy::setVirtualFT(int chan, float value, float T, float tafb)
{
	// msr_dev_FT[chan] = value;
	msr_dev_FT[chan] = (value*T+msr_dev_FT[chan]*tafb)/(tafb+T);

	// //calc acceleration (filtered)
	calcDeltaFTi(chan, T, tafb);

	msr_dev_FT_prev2[chan] = msr_dev_FT_prev[chan];
	msr_dev_FT_prev[chan] = msr_dev_FT[chan];
}

// Superimpose a virtual force/torque to the measurement of the desired channel (without any filtering)
void ATIanalogy::superimposeFT(int chan, float value)
{
	msr_dev_FT[chan] += value;
}

void ATIanalogy::loadCalibration(){
	/// Load ATI F/T Sensor calibration file
	
	std::cout << "Loading F/T Calibration... ";
	char cal_filename[] = "../include/atidaq/FT8585.cal";
	cal=createCalibration(cal_filename,1);   //Second arg is Index of calibration file
		if (cal==NULL) {
			printf("Specified calibration could not be loaded.\n");
			return;
		}
		else
			{std::cout << "Calibration file loaded!\n";	}

	//Tool coordinate system transformation
	short sts; 
	float SampleTT[6]={0.0, 0.0, 0.0, 0.0, 0.0, 90.0}; //translate/rotate
	char dist[] = "mm";
	char angle[] = "degrees";
	sts = SetToolTransform(cal,SampleTT,dist,angle);
}