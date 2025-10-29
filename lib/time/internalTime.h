#ifndef INTERNAL_TIME_H_INCLUDED 
#define INTERNAL_TIME_H_INCLUDED

#include "ext_dep.h"

using namespace std::chrono;

class internalTime{
	
     public:
         internalTime();
         double getSampleTime();
         
	 private:
         high_resolution_clock::time_point t1;
	 
};

#endif

