#include "internalTime.h" 

internalTime::internalTime(){
    t1 = high_resolution_clock::now();
	}

double internalTime::getSampleTime(){
   
    high_resolution_clock::time_point l_t2 = high_resolution_clock::now();
    duration<double> elapsed = duration_cast<duration<double>>(l_t2 - t1);
    double l_dSampleTime = (double)(elapsed.count());    
    t1 = high_resolution_clock::now();
    
	return l_dSampleTime;
	
	}
