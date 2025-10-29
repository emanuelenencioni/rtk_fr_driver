#include <ext_dep.h>
#include <internalTime.h>
#include <thread_functions.h>
#include <Com.h>
#include <typeDef.h>


sched_param mypar;
task_par tp[2];
pthread_attr_t att[2]; 
pthread_rwlock_t communicationLock;
thread_functions communication_period_management; 
bool bCommunicationThread = false;
float fCommunicationHz = 100; //Hertz
double dSampleTime;
RX_Frame RxFrameShared;

attitude quaternionToEuler(quaternion l_Q);

void *threadCommunication(void *arg){
     
     double l_dSampleTime = 0;
     pidSignals l_pidsData;
  
     struct task_par *tp;
     tp = (struct task_par *)arg;
     communication_period_management.set_period(tp); 
     
     internalTime *sampleTime;
     sampleTime = new internalTime();
    
     Com *c;
     c = new Com();
     
     while(1){
		 //!******** delta time *********!//
	 l_dSampleTime = sampleTime->getSampleTime()/1000;
         		 		 
         l_pidsData.CONTRIBUTE.PITCH = -0.2;
         l_pidsData.CONTRIBUTE.ROLL = 1.33;
         l_pidsData.CONTRIBUTE.THRUST = 0.888;
         l_pidsData.CONTRIBUTE.YAW = -2.3;

         bool l_bStatus = true; //decidere una nuova politica per definire quando si trasmette sul bus UART 
	 bool l_bStop = false;
	 
	 //!*********Read State***********// 
	 RXData l_RxFrame = c->uartComRx();
	 RX_Frame l_RxFrameHLFormat;
	 /*l_RxFrameHLFormat.AUTOHOVERING = l_RxFrame.STATEBIN[4];
	 l_RxFrameHLFormat.MISSIONSTART = l_RxFrame.STATEBIN[5];
	 l_RxFrameHLFormat.FLIGHTMODE = l_RxFrame.STATEBIN[6];
	 l_RxFrameHLFormat.RC_PITCH = l_RxFrame.RC_PITCH;
	 l_RxFrameHLFormat.RC_ROLL = l_RxFrame.RC_ROLL;
	 l_RxFrameHLFormat.RC_YAW = l_RxFrame.RC_YAW;
	 l_RxFrameHLFormat.RC_THROTTLE = l_RxFrame.RC_THROTTLE;*/
	 l_RxFrameHLFormat.Q.W = (double)(l_RxFrame.Q0);
	 l_RxFrameHLFormat.Q.X = (double)(l_RxFrame.Q1);
	 l_RxFrameHLFormat.Q.Y = (double)(l_RxFrame.Q2);
	 l_RxFrameHLFormat.Q.Z = (double)(l_RxFrame.Q3);

	 l_RxFrameHLFormat.Q_GYRO.W = (double)(l_RxFrame.Q0_TMP);
	 l_RxFrameHLFormat.Q_GYRO.X = (double)(l_RxFrame.Q1_TMP);
	 l_RxFrameHLFormat.Q_GYRO.Y = (double)(l_RxFrame.Q2_TMP);
	 l_RxFrameHLFormat.Q_GYRO.Z = (double)(l_RxFrame.Q3_TMP);

	 attitude l_Euler = quaternionToEuler(l_RxFrameHLFormat.Q);
	 l_RxFrameHLFormat.ACC.X = (double)(l_RxFrame.AX);
	 l_RxFrameHLFormat.ACC.Y = (double)(l_RxFrame.AY);
	 l_RxFrameHLFormat.ACC.Z = (double)(l_RxFrame.AZ);
	 l_RxFrameHLFormat.ANGULAR_VELOCITY.X = (double)(l_RxFrame.GX);
	 l_RxFrameHLFormat.ANGULAR_VELOCITY.Y = (double)(l_RxFrame.GY);
	 l_RxFrameHLFormat.ANGULAR_VELOCITY.Z = (double)(l_RxFrame.GZ);
	 //l_RxFrameHLFormat.IMU_TEMP = l_RxFrame.TEMP;
	 l_RxFrameHLFormat.GPS_LATITUDE = (double)(l_RxFrame.GPS_LATITUDE)/10000000 + (double)(l_RxFrame.GPS_LATITUDE_HP)/1000000000;
	 l_RxFrameHLFormat.GPS_LONGITUDE = (double)(l_RxFrame.GPS_LONGITUDE)/10000000 + (double)(l_RxFrame.GPS_LONGITUDE_HP)/1000000000; 
	 l_RxFrameHLFormat.GPS_ALTITUDE = l_RxFrame.GPS_ALTITUDE;

	 l_RxFrameHLFormat.GPS_EPH = l_RxFrame.GPS_EPH;
	 l_RxFrameHLFormat.GPS_EPV = l_RxFrame.GPS_EPV;
	
	 l_RxFrameHLFormat.GPS_1_FIXMODE = l_RxFrame.GPS_1_FIXMODE;
	 l_RxFrameHLFormat.GPS_2_FIXMODE = l_RxFrame.GPS_2_FIXMODE;
	 l_RxFrameHLFormat.GPS_SYSTEM_ERROR = l_RxFrame.GPS_SYSTEM_ERROR;
	
	 l_RxFrameHLFormat.RELPOSN = l_RxFrame.RELPOSN;
	 l_RxFrameHLFormat.RELPOSE = l_RxFrame.RELPOSN;
	 l_RxFrameHLFormat.RELPOSD = l_RxFrame.RELPOSN;
	 //l_RxFrameHLFormat.RELPOSLENGTH = l_RxFrame.RELPOSN;
	 l_RxFrameHLFormat.ACCN = l_RxFrame.RELPOSN;
	 l_RxFrameHLFormat.ACCE = l_RxFrame.RELPOSN;
	 l_RxFrameHLFormat.ACCD = l_RxFrame.RELPOSN;
	 //l_RxFrameHLFormat.ACCLENGTH = l_RxFrame.RELPOSN;
	 l_RxFrameHLFormat.VELN = l_RxFrame.VELN;
	 l_RxFrameHLFormat.VELE = l_RxFrame.VELE;
	 l_RxFrameHLFormat.VELD = l_RxFrame.VELD;
	 l_RxFrameHLFormat.VELACC = l_RxFrame.VELACC;

     l_RxFrameHLFormat.RELPOSN2 = l_RxFrame.RELPOSN2;
	 l_RxFrameHLFormat.RELPOSE2 = l_RxFrame.RELPOSE2;
	 l_RxFrameHLFormat.RELPOSD2 = l_RxFrame.RELPOSD2;
	 //l_RxFrameHLFormat.RELPOSLENGTH2 = l_RxFrame.RELPOSLENGTH2;
	 l_RxFrameHLFormat.ACCN2 = l_RxFrame.ACCN2;
	 l_RxFrameHLFormat.ACCE2 = l_RxFrame.ACCE2;
	 l_RxFrameHLFormat.ACCD2 = l_RxFrame.ACCD2;
	 //l_RxFrameHLFormat.ACCLENGTH2 = l_RxFrame.ACCLENGTH2;
	 l_RxFrameHLFormat.VELN2 = l_RxFrame.VELN2;
	 l_RxFrameHLFormat.VELE2 = l_RxFrame.VELE2;
	 l_RxFrameHLFormat.VELD2 = l_RxFrame.VELD2;
	 l_RxFrameHLFormat.VELACC2 = l_RxFrame.VELACC2;

	 //l_RxFrameHLFormat.BARO_ALTITUDE = l_RxFrame.BARO_ALTITUDE;
	 //l_RxFrameHLFormat.BARO_PRESSURE = l_RxFrame.BARO_PRESSURE;
	 //l_RxFrameHLFormat.BARO_TEMP = l_RxFrame.BARO_TEMP;
	
	 l_RxFrameHLFormat.TIME_OF_WEEK = l_RxFrame.TIME_OF_WEEK;
	 l_RxFrameHLFormat.DELTA_TIME_IMU_MS = l_RxFrame.DELTA_TIME_IMU_MS;
	
/*l_RxFrameHLFormat.AUTOHOVERING = l_RxFrame.STATEBIN[4];
		l_RxFrameHLFormat.MISSIONSTART = l_RxFrame.STATEBIN[5];
		l_RxFrameHLFormat.FLIGHTMODE = l_RxFrame.STATEBIN[6];
		l_RxFrameHLFormat.RC_PITCH = l_RxFrame.RC_PITCH;
		l_RxFrameHLFormat.RC_ROLL = l_RxFrame.RC_ROLL;
		l_RxFrameHLFormat.RC_YAW = l_RxFrame.RC_YAW;
		l_RxFrameHLFormat.RC_THROTTLE = l_RxFrame.RC_THROTTLE;*/
		l_RxFrameHLFormat.Q.W = (double)(l_RxFrame.Q0);
		l_RxFrameHLFormat.Q.X = (double)(l_RxFrame.Q1);
		l_RxFrameHLFormat.Q.Y = (double)(l_RxFrame.Q2);
		l_RxFrameHLFormat.Q.Z = (double)(l_RxFrame.Q3);

		l_RxFrameHLFormat.Q_GYRO.W = (double)(l_RxFrame.Q0_TMP);
		l_RxFrameHLFormat.Q_GYRO.X = (double)(l_RxFrame.Q1_TMP);
		l_RxFrameHLFormat.Q_GYRO.Y = (double)(l_RxFrame.Q2_TMP);
		l_RxFrameHLFormat.Q_GYRO.Z = (double)(l_RxFrame.Q3_TMP);

		//attitude l_Euler = quaternionToEuler(l_RxFrameHLFormat.Q);
		l_RxFrameHLFormat.ACC.X = (double)(l_RxFrame.AX);
		l_RxFrameHLFormat.ACC.Y = (double)(l_RxFrame.AY);
		l_RxFrameHLFormat.ACC.Z = (double)(l_RxFrame.AZ);
		l_RxFrameHLFormat.ANGULAR_VELOCITY.X = (double)(l_RxFrame.GX);
		l_RxFrameHLFormat.ANGULAR_VELOCITY.Y = (double)(l_RxFrame.GY);
		l_RxFrameHLFormat.ANGULAR_VELOCITY.Z = (double)(l_RxFrame.GZ);
		//l_RxFrameHLFormat.IMU_TEMP = l_RxFrame.TEMP;
		l_RxFrameHLFormat.GPS_LATITUDE = (double)(l_RxFrame.GPS_LATITUDE)/10000000 + (double)(l_RxFrame.GPS_LATITUDE_HP)/1000000000;
		l_RxFrameHLFormat.GPS_LONGITUDE = (double)(l_RxFrame.GPS_LONGITUDE)/10000000 + (double)(l_RxFrame.GPS_LONGITUDE_HP)/1000000000; 
		l_RxFrameHLFormat.GPS_ALTITUDE = l_RxFrame.GPS_ALTITUDE;

		l_RxFrameHLFormat.GPS_EPH = l_RxFrame.GPS_EPH;
		l_RxFrameHLFormat.GPS_EPV = l_RxFrame.GPS_EPV;
		
		l_RxFrameHLFormat.GPS_1_FIXMODE = l_RxFrame.GPS_1_FIXMODE;
        l_RxFrameHLFormat.GPS_2_FIXMODE = l_RxFrame.GPS_2_FIXMODE;
        l_RxFrameHLFormat.GPS_SYSTEM_ERROR = l_RxFrame.GPS_SYSTEM_ERROR;
		
		l_RxFrameHLFormat.RELPOSN = l_RxFrame.RELPOSN;
		l_RxFrameHLFormat.RELPOSE = l_RxFrame.RELPOSE;
		l_RxFrameHLFormat.RELPOSD = l_RxFrame.RELPOSD;
		//l_RxFrameHLFormat.RELPOSLENGTH = l_RxFrame.RELPOSLENGTH;
		l_RxFrameHLFormat.ACCN = l_RxFrame.ACCN;
		l_RxFrameHLFormat.ACCE = l_RxFrame.ACCE;
		l_RxFrameHLFormat.ACCD = l_RxFrame.ACCD;
		//l_RxFrameHLFormat.ACCLENGTH = l_RxFrame.ACCLENGTH;
        l_RxFrameHLFormat.VELN = l_RxFrame.VELN;
		l_RxFrameHLFormat.VELE = l_RxFrame.VELE;
		l_RxFrameHLFormat.VELD = l_RxFrame.VELD;
		l_RxFrameHLFormat.VELACC = l_RxFrame.VELACC;

        l_RxFrameHLFormat.RELPOSN2 = l_RxFrame.RELPOSN2;
		l_RxFrameHLFormat.RELPOSE2 = l_RxFrame.RELPOSE2;
		l_RxFrameHLFormat.RELPOSD2 = l_RxFrame.RELPOSD2;
		//l_RxFrameHLFormat.RELPOSLENGTH2 = l_RxFrame.RELPOSLENGTH2;
		l_RxFrameHLFormat.ACCN2 = l_RxFrame.ACCN2;
		l_RxFrameHLFormat.ACCE2 = l_RxFrame.ACCE2;
		l_RxFrameHLFormat.ACCD2 = l_RxFrame.ACCD2;
		//l_RxFrameHLFormat.ACCLENGTH2 = l_RxFrame.ACCLENGTH2;
        l_RxFrameHLFormat.VELN2 = l_RxFrame.VELN2;
		l_RxFrameHLFormat.VELE2 = l_RxFrame.VELE2;
		l_RxFrameHLFormat.VELD2 = l_RxFrame.VELD2;
		l_RxFrameHLFormat.VELACC2 = l_RxFrame.VELACC2;

		/*l_RxFrameHLFormat.BARO_ALTITUDE = l_RxFrame.BARO_ALTITUDE;
		l_RxFrameHLFormat.BARO_PRESSURE = l_RxFrame.BARO_PRESSURE;
		l_RxFrameHLFormat.BARO_TEMP = l_RxFrame.BARO_TEMP;*/
		
		l_RxFrameHLFormat.TIME_OF_WEEK = l_RxFrame.TIME_OF_WEEK;
		l_RxFrameHLFormat.DELTA_TIME_IMU_MS = l_RxFrame.DELTA_TIME_IMU_MS;
		//l_RxFrameHLFormat.DELTA_TIME_IMU_MS = l_RxFrame.DELTA_TIME_IMU_MS;
		
		//c->uartComRxPrint(148);//64//36);//148
		/*
		std::cout << "" << std::endl;
		std::cout << "status4: " << l_RxFrame.STATEBIN[4] << std::endl; 
		std::cout << "status5: " << l_RxFrame.STATEBIN[5] << std::endl; 
		std::cout << "status6: " << l_RxFrame.STATEBIN[6] << std::endl;
		std::cout << "RC PITCH: " << l_RxFrame.RC_PITCH << std::endl;
		std::cout << "RC ROLL: " << l_RxFrame.RC_ROLL << std::endl;
		std::cout << "RC YAW: " << l_RxFrame.RC_YAW << std::endl;
		std::cout << "RC THROTTLE: " << l_RxFrame.RC_THROTTLE << std::endl;*/
		
		/*std::cout << "Q0: " << l_RxFrame.Q0 << std::endl;
		std::cout << "Q1: " << l_RxFrame.Q1 << std::endl;
		std::cout << "Q2: " << l_RxFrame.Q2 << std::endl;
		std::cout << "Q3: " << l_RxFrame.Q3 << std::endl;*/

		std::cout << "-- Q GYRO ONLY: --" << std::endl;

		std::cout << "Q0: " << l_RxFrame.Q0_TMP << std::endl;
		std::cout << "Q1: " << l_RxFrame.Q1_TMP << std::endl;
		std::cout << "Q2: " << l_RxFrame.Q2_TMP << std::endl;
		std::cout << "Q3: " << l_RxFrame.Q3_TMP << std::endl;

		/*std::cout << "PITCH: " << (180/M_PI)*l_Euler.PITCH << std::endl;
		std::cout << "ROLL: " << (180/M_PI)*l_Euler.ROLL << std::endl;
		std::cout << "YAW: " << (180/M_PI)*l_Euler.YAW << std::endl;*/
		
		std::cout << "AX: " << l_RxFrame.AX << std::endl;
		std::cout << "AY: " << l_RxFrame.AY << std::endl;
		std::cout << "AZ: " << l_RxFrame.AZ << std::endl;
		std::cout << "GX: " << l_RxFrame.GX << std::endl;
		std::cout << "GY: " << l_RxFrame.GY << std::endl;
		std::cout << "GZ: " << l_RxFrame.GZ << std::endl;
		
		//std::cout << "TEMP: " << l_RxFrame.TEMP << std::endl;
		
		std::cout << "Latitude: " << l_RxFrame.GPS_LATITUDE  << std::endl;
		std::cout << "Latitude HP: " << l_RxFrame.GPS_LATITUDE_HP << std::endl;
		std::cout << "Latitude res: " << std::setprecision(11) << l_RxFrameHLFormat.GPS_LATITUDE << std::endl;
		std::cout << "Longitude: " << l_RxFrame.GPS_LONGITUDE  << std::endl;
		std::cout << "Longitude HP: " << l_RxFrame.GPS_LONGITUDE_HP  << std::endl;
		std::cout << "Longitude res: " << l_RxFrameHLFormat.GPS_LONGITUDE  << std::endl;

		std::cout << "GPS Altitude: " << std::setprecision(6) << l_RxFrame.GPS_ALTITUDE  << std::endl;    
		std::cout << "GPS Altitude HP: " << l_RxFrame.GPS_ALTITUDE_HP  << std::endl;    
		
		std::cout << "EPH: " << l_RxFrameHLFormat.GPS_EPH  << " mm * 0.1" << std::endl;
		std::cout << "EPV: " << l_RxFrameHLFormat.GPS_EPV  << " mm * 0.1" << std::endl;	 
		
		std::cout << "FIX MODE GPS 1: " << l_RxFrame.GPS_1_FIXMODE << std::endl;
		std::cout << "FIX MODE GPS 2: " << l_RxFrame.GPS_2_FIXMODE << std::endl;
		std::cout << "GPS SYSTEM ERROR: " << l_RxFrame.GPS_SYSTEM_ERROR << std::endl;
		
		std::cout << "RELPOSN: " << ((float)(l_RxFrame.RELPOSN))/100 << " cm" << std::endl;
		std::cout << "RELPOSE: " << ((float)(l_RxFrame.RELPOSE))/100 << " cm" << std::endl;
		std::cout << "RELPOSD: " << ((float)(l_RxFrame.RELPOSD))/100 << " cm" << std::endl;
		
		//std::cout << "RELPOSLENGTH: " << ((float)(l_RxFrame.RELPOSLENGTH))/100 << " cm" << std::endl;
		
		/*std::cout << "ACCN: " << ((float)(l_RxFrame.ACCN))/100 << " cm" << std::endl;
		std::cout << "ACCE: " << ((float)(l_RxFrame.ACCE))/100 << " cm" << std::endl;
		std::cout << "ACCD: " << ((float)(l_RxFrame.ACCD))/100 << " cm" << std::endl;*/
		
		//std::cout << "ACCLENGTH: " << ((float)(l_RxFrame.ACCLENGTH))/100 << " cm" << std::endl;

        /*std::cout << "VELN: " << l_RxFrame.VELN << " cm/s" << std::endl;
		std::cout << "VELE: " << l_RxFrame.VELE << " cm/s" << std::endl;
		std::cout << "VELD: " << l_RxFrame.VELD << " cm/s" << std::endl;
		std::cout << "VELACC: " << l_RxFrame.VELACC << " cm/s" << std::endl;

        std::cout << "RELPOSN 2: " << ((float)(l_RxFrame.RELPOSN2))/100 << " cm" << std::endl;
		std::cout << "RELPOSE 2: " << ((float)(l_RxFrame.RELPOSE2))/100 << " cm" << std::endl;
		std::cout << "RELPOSD 2: " << ((float)(l_RxFrame.RELPOSD2))/100 << " cm" << std::endl;*/
		
		//std::cout << "RELPOSLENGTH 2: " << ((float)(l_RxFrame.RELPOSLENGTH2))/100 << " cm" << std::endl;
		
		/*std::cout << "ACCN 2: " << ((float)(l_RxFrame.ACCN2))/100 << " cm" << std::endl;
		std::cout << "ACCE 2: " << ((float)(l_RxFrame.ACCE2))/100 << " cm" << std::endl;
		std::cout << "ACCD 2: " << ((float)(l_RxFrame.ACCD2))/100 << " cm" << std::endl;*/
		
		//std::cout << "ACCLENGTH 2: " << ((float)(l_RxFrame.ACCLENGTH2))/100 << " cm" << std::endl;

        /*std::cout << "VELN 2: " << l_RxFrame.VELN2 << " cm/s" << std::endl;
		std::cout << "VELE 2: " << l_RxFrame.VELE2 << " cm/s" << std::endl;
		std::cout << "VELD 2: " << l_RxFrame.VELD2 << " cm/s" << std::endl;
		std::cout << "VELACC 2: " << l_RxFrame.VELACC2 << " cm/s" << std::endl;*/


		/*std::cout << "BARO ALTITUDE: " << l_RxFrame.BARO_ALTITUDE << std::endl;
		std::cout << "PRESSURE: " << l_RxFrame.BARO_PRESSURE  << " hPa" << std::endl;
		std::cout << "BARO TEMP: " << l_RxFrame.BARO_TEMP << std::endl;
		std::cout << "TIME OF WEEK: " << l_RxFrame.TIME_OF_WEEK  << " ms" << std::endl;*/
		
		std::cout << "IMU DELTA TIME: " << l_RxFrame.DELTA_TIME_IMU_MS << " ms" << std::endl;	 
		
		//std::cout << "BARO DELTA TIME: " << l_RxFrame.DELTA_TIME_BARO_MS << " ms" << std::endl;	 
		
		std::cout << "LINEAR ACTUATOR DELTA TIME: " << l_RxFrame.DELTA_TIME_LINEAR_ACTUATOR_MS << " ms" << std::endl;	 

		std::cout << "" << std::endl;
		std::cout << "FRAMES OK: " << c->iPacchettiBuoni << std::endl;
		std::cout << "HEADERS SEARCHED: " << c->iContatoreHeader << std::endl;
		std::cout << "RECONSTRUCTED FRAMES: " << c->iContatorePacchettiRicostruiti << std::endl;
		std::cout << "DISCARDED FRAMES: " << c->iContatorePacchettiScartati << std::endl;
	 //!********Send Auto Commands********!//
	 //int *l_piPpmSend = c->uartComTx(0, 170, 1810, 90, l_bStop, l_bStatus, l_pidsData.CONTRIBUTE); //aggiunto il controllo di consistenza sui dati
	 //!************************************//
         pthread_rwlock_wrlock(&communicationLock);
         RxFrameShared = l_RxFrameHLFormat;
         dSampleTime = l_dSampleTime;
         pthread_rwlock_unlock(&communicationLock);	                  
         
         communication_period_management.wait_for_period(tp);
     }
     return 0;
    
}     

attitude quaternionToEuler(quaternion l_Q){
  attitude l_Euler;
  
  l_Euler.PITCH = asin(2*(l_Q.W*l_Q.Y - l_Q.X*l_Q.Z));
  l_Euler.ROLL = -atan(2*(l_Q.W*l_Q.X + l_Q.Y*l_Q.Z)/(l_Q.W*l_Q.W - l_Q.X*l_Q.X - l_Q.Y*l_Q.Y + l_Q.Z*l_Q.Z));
  l_Euler.YAW = atan(2*(l_Q.W*l_Q.Z + l_Q.X*l_Q.Y)/(l_Q.W*l_Q.W + l_Q.X*l_Q.X - l_Q.Y*l_Q.Y - l_Q.Z*l_Q.Z));
  return l_Euler;
}

int main(int argc, const char **argv){	
	 ///////////////////////////////////////////////////////////////////
	 //
	 //                   Thread Com init:
	 //
	 ///////////////////////////////////////////////////////////////////
	 pthread_rwlock_init(&communicationLock, NULL);	      
	 pthread_t communicationThread_ID;
	 void *communicationExit_status;
	 int l_iCommunicationPeriod = (int)((1/fCommunicationHz)*1000);
	 tp[0].arg = 0; 
	 tp[0].period = l_iCommunicationPeriod; //ms
	 tp[0].deadline = 80; 
	 tp[0].priority = 40;
	 tp[0].dmiss = 0;
	 pthread_attr_init(&att[0]);
	 pthread_attr_setinheritsched(&att[0], PTHREAD_EXPLICIT_SCHED);
	 //pthread_attr_setschedpolicy(&att[0], SCHED_FIFO);
	 pthread_attr_setschedpolicy(&att[0], SCHED_OTHER);	 
	 mypar.sched_priority = tp[0].priority;
	 pthread_attr_setschedparam(&att[0], &mypar);
	 pthread_attr_setdetachstate(&att[0], PTHREAD_CREATE_DETACHED);	   
	 int l_iErrComm;
	 if (( l_iErrComm = pthread_create(&communicationThread_ID, &att[0], threadCommunication, &tp[0])) != 0) {
		char const *emsg = ((l_iErrComm == EAGAIN) ? "EAGAIN" : ((l_iErrComm == EINVAL) ? "EINVAL" : ((l_iErrComm == EPERM) ? "EPERM" : "unknown")));
		fprintf(stderr, "pthread_create() failed (%d %s); are you sure you're root?\n", l_iErrComm, emsg);
		fprintf(stderr, "You may also need to do:\n");
		fprintf(stderr, "echo -1 > /proc/sys/kernel/sched_rt_runtime_us\n");
		exit(1);
	 }	 
	 else{
	   bCommunicationThread = true;
	   std::cout << "communication task launched \n" << std::endl; 
	 }
	 while(1)
	    usleep(5000*1000);  
} 
