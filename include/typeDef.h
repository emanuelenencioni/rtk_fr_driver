#ifndef TYPEDEF_H_INCLUDED 
#define TYPEDEF_H_INCLUDED 
//#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>
#include "ext_dep.h"

struct tagPos{ //tag position in tag frame
int ID;
double X;
double Y;
double Z;
double PITCH;
double ROLL;
double YAW;	
};

struct meanData{
bool STATUS;
double VALUE;
};

struct setPoint{
double XVELDES;
double YVELDES;
double ZVELDES;
double XENDPOINT;
double YENDPOINT;
double ZENDPOINT;
};

struct statusData{
double X;
double Y;
double Z;
double YAWBODY;
bool AUTOMODESTATE;
int MISSIONSTATE;
bool bStatus;
double SAMPLETIME;
};

struct velocityDes{
double LINEAR_VELDES;
double ANGULAR_VELDES;
};

struct missionPoint{
double XSETPOINT;
double YSETPOINT;
double ZSETPOINT;
double VXSETPOINT;
double VYSETPOINT;
double VZSETPOINT;
double YAWSETPOINT;
double XENDPOINT;
double YENDPOINT;
double ZENDPOINT;
double YAWENDPOINT;
int GATEID;
};

struct telemetry{
int ID;
bool AUTOMODESTATE;
int MISSIONSTATE;
double BODY_X;    
double BODY_Y; 
double BODY_Z;
double END_X;    
double END_Y; 
double END_Z;
double ERROR_X;    
double ERROR_Y; 
double ERROR_Z;
double ERROR_YAW;
double PITCH;    
double ROLL; 
double YAW;
};

struct point2D{ 
double X;
double Y;
};

struct point3D{ 
double X;
double Y;
double Z;
};

struct point4D{ 
double X;
double Y;
double Z;
double YAW;
};

struct point6D{ 
double X;
double Y;
double Z;
double PITCH;
double ROLL;
double YAW;
};

struct trajectory6D{
double PX;
double PY;
double PZ;
double VX;
double VY;
double VZ;
};
	
struct contributes{
double ROLL;
double THRUST;
double PITCH;
double YAW;
};

struct commandsLl{
double COM_0;
double COM_1;
double COM_2;
double COM_3;    
};

struct gatePoints{
int ID;
point3D POINT1;
point3D POINT2;
double YAW;
};

struct attitude{ 
double PITCH;
double ROLL;
double YAW;
};

struct quaternion{
double W;
double X;
double Y;
double Z;
};

struct endPoints{ 
double X;
double Y;
double Z;
double YAW;
};

struct pidSignals{
//!******X P-I-D signals******!//		 
double XCP;
double XCI;
double XCD;
//!******THRUST P-I-D signals******!//	
double YCP;
double YCI;
double YCD;	 
//!******Z P-I-D signals******!//
double ZCP;
double ZCI;
double ZCD;	 		 
//!******Yaw P-I-D signals******!//		 
double YAWCP;
double YAWCI;
double YAWCD;	 
//!******Total contributes******!//		 
double XC;
double YC;
double ZC;
double YAWC;
//!******P-I-D DF signals******!//
double DF_XCD;
double DF_YCD;		 	 
double DF_ZCD;
//!******P-I-D DL signals******!//		 	 
double DL_XCD;
double DL_YCD;
double DL_ZCD;
//!*****Total contributes******!//		 	 
contributes CONTRIBUTE;
};

struct imu_Data{
double ACC[3];
double GYRO[3];
};

struct state{
point3D POSITION;
point3D VELOCITY;
point3D ACCELERATION;
attitude ATTITUDE;
attitude ANGULAR_VELOCITY;
attitude ANGULAR_ACCELERATION;
quaternion QUATERNION;
};

struct RX_Frame{
int AUTOHOVERING; //status[4]
int MISSIONSTART; //status[5]
int FLIGHTMODE;   //status[6]
int RC_PITCH;
int RC_ROLL;
int RC_YAW;
int RC_THROTTLE;
bool FAILSAFE;	
quaternion Q;
quaternion Q_GYRO;
attitude ATTITUDE;
point3D ACC;
point3D ANGULAR_VELOCITY;
float IMU_TEMP;
double GPS_LATITUDE;
double GPS_LONGITUDE;
int GPS_ALTITUDE;    
//int GPS_SPEED;
//int GPS_HEADING;
uint32_t GPS_EPH;
uint32_t GPS_EPV;
//float GPS_PDOP;
//int GPS_SIV;
int GPS_1_FIXMODE;
int GPS_2_FIXMODE;
int GPS_SYSTEM_ERROR;
int32_t RELPOSN;
int32_t RELPOSE;
int32_t RELPOSD;
//int32_t RELPOSLENGTH;
uint32_t ACCN;
uint32_t ACCE;
uint32_t ACCD;
//uint32_t ACCLENGTH;
int32_t VELN;
int32_t VELE;
int32_t VELD;
uint32_t VELACC;
//Second RTK
int32_t RELPOSN2;
int32_t RELPOSE2;
int32_t RELPOSD2;
//int32_t RELPOSLENGTH2;
uint32_t ACCN2;
uint32_t ACCE2;
uint32_t ACCD2;
//uint32_t ACCLENGTH2;
int32_t VELN2;
int32_t VELE2;
int32_t VELD2;
uint32_t VELACC2;
double BARO_ALTITUDE;
double BARO_PRESSURE;
float BARO_TEMP;
uint32_t TIME_OF_WEEK;
int32_t DELTA_TIME_IMU_MS;
int32_t DELTA_TIME_BARO_MS;
int32_t DELTA_TIME_LINEAR_ACTUATOR_MS;
};

struct PLOTThread{
double SAMPLE_TIME;
pthread_rwlock_t plotLock;
};

struct PLANNERThread{
point3D EPOINT_T_FRAME;    //** USED FOR GATES MISSION
gatePoints GPOINT_T_FRAME; //**
missionPoint SETPOINT;
statusData STATE;
bool MISSION_OVER = false;
double SAMPLE_TIME;
pthread_rwlock_t TrajectoryLock;
};

struct DISPLAYThread{
int RET_VAL;
double SAMPLE_TIME;
pthread_rwlock_t displayLock;
};

struct CONTROLThread{
pidSignals PIDS_DATA;
double SAMPLE_TIME;
pthread_rwlock_t controlLock;
};

struct LOGThread{
double SAMPLE_TIME;
pthread_rwlock_t logLock;
};

struct COMMUNICATIONThread{
//int STATUS[6];
RX_Frame RXFRAME;
double SAMPLE_TIME;
pthread_rwlock_t communicationLock;
};

struct Options{
bool WIND = false;
bool PLOT = false;
bool REC = false;
bool LOG = false;  
bool DISPLAY_SAMPLE_TIME = false;               
bool SIM = false;     
bool SSH = false;      
bool MIPI = false;   
bool WIFI = false;                        
};

struct ActiveThread{
bool VISION = false;
int VISION_HZ; 
bool PLANNER = false;
int PLANNER_HZ;
bool DISPLAY = false;
int DISPLAY_HZ;
bool CONTROL = false;
int CONTROL_HZ;
bool COMMUNICATION = false;
int COMMUNICATION_HZ;
bool LOG = false;
int LOG_HZ;
bool SLAM = false;
int SLAM_HZ;
bool SIM = false;
int SIM_HZ;
bool PLOT = false;
int PLOT_HZ;
bool MIPI = false;
int MIPI_HZ;
pthread_rwlock_t threadStateLock;
};

#endif
