/*
BSD 2-Clause License

Copyright (c) 2011, Brett Beauregard
Modifiziert von Thomas Müller, 2025
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION	1.0 // Original von Brett Beauregard, modifiziert von Thomas Müller, 2025

class PID_ATune
{


  public:
  //commonly used functions **************************************************************************
    PID_ATune(double*, double*);                       	// * Constructor.  links the Autotune to a given PID
    int Runtime();						   			   	// * Similar to the PID Compue function, returns non 0 when done
	void Cancel();									   	// * Stops the AutoTune	
	
	void SetOutputStep(double);						   	// * how far above and below the starting value will the output step?	
	double GetOutputStep();							   	// 
	
	void SetControlType(int); 						   	// * Determies if the tuning parameters returned will be PI (D=0)
	int GetControlType();							   	//   or PID.  (0=PI, 1=PID)			
	
	void SetLookbackSec(int);							// * how far back are we looking to identify peaks
	int GetLookbackSec();								//
	
	void SetNoiseBand(double);							// * the autotune will ignore signal chatter smaller than this value
	double GetNoiseBand();								//   this should be acurately set
	
	double GetKp();										// * once autotune is complete, these functions contain the
	double GetKi();										//   computed tuning parameters.  
	double GetKd();										//
	
	// ----- NEUE GETTER FUNKTIONEN -----
    int getPeakCount() { return peakCount; }
    int getPeakType() { return peakType; }
    // ----------------------------------
	
  private:
    void FinishUp();
	bool isMax, isMin;
	double *input, *output;
	double setpoint;
	double noiseBand;
	int controlType;
	bool running;
	unsigned long peak1, peak2, lastTime;
	int sampleTime;
	int nLookBack;
	int peakType;
	double lastInputs[101];
    double peaks[10];
	int peakCount;
	bool justchanged;
	bool justevaled;
	double absMax, absMin;
	double oStep;
	double outputStart;
	double Ku, Pu;
	
};
#endif

