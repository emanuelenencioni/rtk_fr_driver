#ifndef COM_H_INCLUDED 
#define COM_H_INCLUDED

#include "ext_dep.h"
#include "typeDef.h"
#include "internalTime.h"



struct RXData{
   int HEADER;
   float Q0;
   float Q1;
   float Q2;
   float Q3; 
   float Q0_TMP;
   float Q1_TMP;
   float Q2_TMP;
   float Q3_TMP;   
   float AX;
   float AY;
   float AZ;
   float GX;
   float GY;
   float GZ;
   float MX;
   float MY;
   float MZ;
   //float TEMP;
   int GPS_LATITUDE;
   int GPS_LATITUDE_HP;
   int GPS_LONGITUDE;
   int GPS_LONGITUDE_HP;
   int GPS_ALTITUDE;   
   int GPS_ALTITUDE_HP;   
   uint32_t GPS_EPH;
   uint32_t GPS_EPV;
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
   int CHECKSUM;
   int FOOTER;
};

class Com{

   public:
     
	 //Contatore di quante volte si è rilevata un'inconsistenza e si è cercato l'header.
	 int iContatoreHeader = 0;
    
	 //Contatore di quante volte si è rilevata un'inconsistenza e si è ricomposto il pacchetto.
	 int iContatorePacchettiRicostruiti = 0;
	 
	 int iContatorePacchettiScartati = 0;
	 
	 int iPacchettiBuoni = 0;
	 
    double dNumber[6]; // definisco il mio numero su cui effettuare le operazioni di scomposizione
    int iNumber;
    int iPpmNumber[6], iPpmNumberLog[6];;
    int * piPpmNumber;
    unsigned int uiBHH[6]; // byte alto più significativo
    unsigned int uiBH[6];  // byte alto meno significativo
    unsigned int uiBL[6];  // byte basso
    unsigned char ucTxBuffer[24];
    unsigned char *pTxBuffer;
	 int iUart0Filestream;
	 int iCount;
    double dPitch, dRoll, dYaw, dThrottle, dPpm, dRangeDeg;
    int iPwmMax, iPwmMin, iOffsetWrite; 
    int iPitchAlign, iRollAlign, iYawAlign;
	 int i8bitConv[8];
	 int iRxInfoBin[8];
	 int iRebootCount;
	       
	 Com(); 
    int* uartComTx(int l_iOffsetWrite ,int l_iPMin, int l_iPMax, double l_dRD, bool l_bComClose, bool l_bStatus, commandsLl l_dU); 
    RXData uartComRx();
    void uartComRxPrint(int l_iFrameSize);

   private:
    void printReceivedFrame(int* l_ucRxBuffer);
    uint8_t iHeader, iFooter;
	 int iToHighLevelHeader, iToHighLevelFooter;
	 
	 ////////////
	 //Qui ci sono le nuove funzioni
	 RXData RxFrame;
	 int iBytesRead = 0;
	 uint8_t ucRxBuffer[233];
	 bool buildData(int* l_ucRxBuffer, int l_iFrameSize, int* l_iCounter);
	 bool verifyChecksum(int* l_piFrame, int l_iChecksum, int l_iFrameSize);
    bool verifyChecksum_tmp(int* l_piFrame_words, int l_iChecksum_word, int l_iNWords);
	 uint8_t computeChecksumTx(uint16_t l_iChecksum);
	 RXData ignoreData();

    void OpenUart();
    void CloseUart();
    void getSwitchState(int l_iRx);
    void getFloat(int* l_piData, int l_iNDataToBeReconstructed, float* l_pfDataCoverted);
    bool consistencyCheckHpF(int* l_pucRxBuffer, int l_iFrameSize, int l_iHeader, int l_iFooter, int l_iChecksum);
    void UartReboot();

};

#endif
