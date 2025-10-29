#include "Com.h"

Com::Com(std::string portName): 
portName(portName)
{
	 
     iNumber = 0;
	 iUart0Filestream = -2;
     iPitchAlign = 0;
     iRollAlign = 0;
     iYawAlign = 0;
     i8bitConv[0] =1;
     i8bitConv[1] =2;
     i8bitConv[2] =4;
     i8bitConv[3] =8;
     i8bitConv[4] =16;
     i8bitConv[5] =32;
     i8bitConv[6] =64;
     i8bitConv[7] =128;
     iRxInfoBin[0] = 0;
     iRxInfoBin[1] = 0;
     iRxInfoBin[2] = 0;
     iRxInfoBin[3] = 0;
     iRxInfoBin[4] = 0;
     iRxInfoBin[5] = 0;
     iRxInfoBin[6] = 0;
     iRxInfoBin[7] = 0;
     iHeader = 36; // #### NUMERO DI CONTROLLO CONSISTENZA MESSAGGI ####
     iFooter = 42;
     iToHighLevelHeader = 453;
     iToHighLevelFooter = 761;
     iRebootCount = 0;
     this->OpenUart();
}

void Com::OpenUart()
{
    this->CloseUart();

	//iUart0Filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);	//Raspberry Port //Open in non blocking read/write mode //	/dev/ttyAMA0
    //iUart0Filestream = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY);// | O_NDELAY);	//Jetson nano Port //Open in non blocking read/write mode //	/dev/ttyAMA0  
    iUart0Filestream = open(this->portName.c_str(), O_RDWR | O_NOCTTY);
    if (iUart0Filestream == -1)
    {
        std::cerr << "Error - Unable to open UART. Ensure it is not in use by another application\n";
        return;
    }

    struct termios options;
    tcgetattr(iUart0Filestream, &options);
    cfmakeraw(&options);  // disattiva tutte le elaborazioni (modalità binaria)

    cfsetspeed(&options, B460800);  // o il baud che vuoi usare

    // Configurazione 8N1
    options.c_cflag &= ~PARENB;   // nessuna parità
    options.c_cflag &= ~CSTOPB;   // 1 bit di stop
    options.c_cflag &= ~CSIZE;    // reset dimensione bit
    options.c_cflag |= CS8;       // 8 bit di dati
    options.c_cflag |= (CLOCAL | CREAD);  // abilita ricezione, ignora modem ctrl
    options.c_cflag &= ~CRTSCTS;  // disattiva hardware flow control

    // Nessun controllo software (XON/XOFF)
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Applica subito le impostazioni
    tcsetattr(iUart0Filestream, TCSANOW, &options);

    // Svuota buffer
    tcflush(iUart0Filestream, TCIOFLUSH);

    std::cout << "UART /dev/ttyTHS0 open in 8N1 mode, 460800 baud" << std::endl;
}

void Com::CloseUart(){
	 //!----- CLOSE THE UART -----
	 close(iUart0Filestream);
	 //
}

int* Com::uartComTx(int l_iOffsetWrite ,int l_iPMin, int l_iPMax, double l_dRD, bool l_bComClose, bool l_bStatus, commandsLl l_U){         
         
         if (l_bStatus) { 
			uint16_t l_iChecksum = 0;
			int iFrameSize = 11;
			//----- TX BYTES -----
			pTxBuffer = &ucTxBuffer[0];
			//********************* Ch0 *********************//
			dPpm = ((l_iPMax - l_iPMin)/1)*l_U.COM_1 - l_iOffsetWrite; 
			iPpmNumber[0] = int(dPpm);
			//********************* Ch1 *********************//
			dPpm = ((l_iPMax - l_iPMin)/l_dRD)*l_U.COM_0 - l_iOffsetWrite;
			iPpmNumber[1] = int(dPpm);
			//********************* Ch2 *********************//
			dPpm = ((l_iPMax - l_iPMin)/l_dRD)*l_U.COM_2 - l_iOffsetWrite;
			iPpmNumber[2] = int(dPpm);
			//********************* Ch3 ************************//
			dPpm = ((l_iPMax - l_iPMin)/l_dRD)*l_U.COM_3 + l_iOffsetWrite;
			iPpmNumber[3] = -int(dPpm);
			*pTxBuffer++ = iHeader; //###### HEADER BYTE #######
			
			for (int i = 0; i < 4 ; i++) {
			   iPpmNumberLog[i] = iPpmNumber[i] + iOffsetWrite;
			   if(iPpmNumber[i] < 0){
				 
				  iPpmNumber[i] = -iPpmNumber[i];
				  uiBH[i] = iPpmNumber[i] >> 8;   //right shift
				  uiBH[i] = uiBH[i] & 0xFF;
				  uiBH[i] = uiBH[i] | 0x80;       // sign bit
				  uiBL[i] = iPpmNumber[i] & 0xFF; // set the leftmost 8 bits to zero
			 
				  *pTxBuffer++ = uiBL[i]; 
				  l_iChecksum += uiBL[i];
				  *pTxBuffer++ = uiBH[i];
				  l_iChecksum += uiBH[i];
			    }
				else{
			   
				   uiBH[i] = iPpmNumber[i] >> 8; 
				   uiBH[i] = uiBH[i] & 0xFF;
				   uiBL[i] = iPpmNumber[i] & 0xFF; 
			 
				   *pTxBuffer++ = uiBL[i]; 
				   l_iChecksum += uiBL[i];
				   *pTxBuffer++ = uiBH[i];
				   l_iChecksum += uiBH[i];
				}
			}
			*pTxBuffer++ = computeChecksumTx(l_iChecksum);
			*pTxBuffer = iFooter; //###### FOOTER BYTE #######
			
			if (iUart0Filestream != -1){
				for(int i=0; i<iFrameSize; i++) 
					iCount += write(iUart0Filestream, &ucTxBuffer[i], 1);
				if (iCount < 0){
					std::cout << "UART TX error\n" << std::endl;
				}
			}
		}	
	 if(l_bComClose){
		 //!----- CLOSE THE UART -----
	     close(iUart0Filestream);
	     //
	 }  
	 piPpmNumber = &iPpmNumberLog[0];  
	 return piPpmNumber;
}

/*
 * Nuova funzione preposta a calcolare il checksum.
 */
uint8_t Com::computeChecksumTx(uint16_t l_i16Sum) {
	uint16_t l_i16Diff = 0;
	int l_i8Sum = 0;
	l_i16Diff = l_i16Sum - (l_i16Sum & 0xFF);
	l_i16Sum -= l_i16Diff;
	l_i8Sum = (uint8_t)l_i16Sum;
	l_i8Sum += (l_i16Diff >> 8);
	return l_i8Sum;
}

/*
 * Nuova funzione preposta a verificare la correttezza del checksum.
 */
bool Com::verifyChecksum(int* l_piFrame, int l_iChecksum, int l_iFrameSize) {
	int64_t l_i64Sum = 0;
	int64_t l_i64Diff = 0;
	int32_t l_i32Sum = 0;
	l_iChecksum = (int32_t)l_iChecksum;
	for (int i=1; i<l_iFrameSize-2; i++)
		l_i64Sum += l_piFrame[i];
	//
	// 0x7FFFFFFF è il massimo numero rappresentabile con 32 bit
	//
	l_i64Diff = l_i64Sum - (l_i64Sum & 0x7FFFFFFF);
	l_i64Sum -= l_i64Diff;
	l_i32Sum = (int32_t)l_i64Sum;
	l_i32Sum += l_i64Diff >> 31;
	return (l_i32Sum == l_iChecksum);
}

bool Com::verifyChecksum_tmp(int* frame_words, int checksum_word, int nWords)
{
    // frame da 232 B = 58 word (0..57):
    // 0: HEADER, 56: CHECKSUM, 57: FOOTER
    if (nWords < 58) return false;
    if (frame_words[57] != 761) return false; // sanity: footer

    long long sum = 0;                 // = int64_t
    // Somma esattamente come sul Mixer: parole 1..55 nell’ordine.
    // SOLO EPH (idx 24) ed EPV (idx 25) sono uint32_t sul TX.
    for (int i = 1; i <= 55; ++i) {
        if (i == 24 || i == 25) {
            // EPH/EPV unsigned
            sum += (unsigned int)frame_words[i];
        } else {
            // tutti gli altri campi "int" (signed)
            sum += (int)frame_words[i];
        }
    }

    // Fold a 31 bit identico al TX
    long long diff = sum - (sum & 0x7FFFFFFFLL);
    sum -= diff;
    int folded = (int)sum;
    folded += (int)(diff >> 31);

    return folded == (int)checksum_word;   // checksum_word è frame_words[56]
}

void Com::printReceivedFrame(int* l_ucRxBuffer){
        std::cout << "Q0: " << ((float)l_ucRxBuffer[1])/10000 << std::endl;
		std::cout << "Q1: " << ((float)l_ucRxBuffer[2])/10000 << std::endl;
		std::cout << "Q2: " << ((float)l_ucRxBuffer[3])/10000 << std::endl;
		std::cout << "Q3: " << ((float)l_ucRxBuffer[4])/10000 << std::endl;

		std::cout << "Q0 TMP: " << ((float)l_ucRxBuffer[5])/10000 << std::endl;
		std::cout << "Q1 TMP: " << ((float)l_ucRxBuffer[6])/10000 << std::endl;
		std::cout << "Q2 TMP: " << ((float)l_ucRxBuffer[7])/10000 << std::endl;
		std::cout << "Q3 TMP: " << ((float)l_ucRxBuffer[8])/10000 << std::endl;

		std::cout << "AX: " << ((float)l_ucRxBuffer[9])/10000 << std::endl;
		std::cout << "AY: " << ((float)l_ucRxBuffer[10])/10000 << std::endl;
		std::cout << "AZ: " << ((float)l_ucRxBuffer[11])/10000 << std::endl;
		std::cout << "GX: " << ((float)l_ucRxBuffer[12])/10000 << std::endl;
		std::cout << "GY: " << ((float)l_ucRxBuffer[13])/10000 << std::endl;
		std::cout << "GZ: " << ((float)l_ucRxBuffer[14])/10000 << std::endl;

		std::cout << "MX: " << ((float)l_ucRxBuffer[15])/10000 << std::endl;
		std::cout << "MY: " << ((float)l_ucRxBuffer[16])/10000 << std::endl;
		std::cout << "MZ: " << ((float)l_ucRxBuffer[17])/10000 << std::endl;

		std::cout << "GPS LATITUDE: " << l_ucRxBuffer[18] << std::endl;
		std::cout << "GPS LATITUDE HP: " << l_ucRxBuffer[19] << std::endl;

		std::cout << "GPS LONGITUDE: " << l_ucRxBuffer[20] << std::endl;
		std::cout << "GPS LONGITUDE HP: " << l_ucRxBuffer[21] << std::endl;
		
		std::cout << "GPS ALTITUDE: " << l_ucRxBuffer[22] << std::endl;
		std::cout << "GPS ALTITUDE HP: " << l_ucRxBuffer[23] << std::endl;
		
		std::cout << "GPS EPH: " << (uint32_t)l_ucRxBuffer[24] << std::endl;
	    std::cout << "GPS EPV: " << (uint32_t)l_ucRxBuffer[25] << std::endl;
		
		std::cout << "GPS 1 FIXMODE: " << l_ucRxBuffer[26] << std::endl;
        std::cout << "GPS 2 FIXMODE: " << l_ucRxBuffer[27] << std::endl;
        std::cout << "GPS SYSTEM ERROR: " << l_ucRxBuffer[28] << std::endl;
		
		std::cout << "RELPOSEN: " << l_ucRxBuffer[29] << std::endl;
		std::cout << "RELPOSEE: " << l_ucRxBuffer[30] << std::endl;
		std::cout << "RELPOSED: " << l_ucRxBuffer[31] << std::endl;
		std::cout << "ACCN: " << (uint32_t)l_ucRxBuffer[32] << std::endl;
		std::cout << "ACCE: " << (uint32_t)l_ucRxBuffer[33] << std::endl;
		std::cout << "ACCD: " << (uint32_t)l_ucRxBuffer[34] << std::endl;

        std::cout << "VELN: " << l_ucRxBuffer[35] << std::endl;
		std::cout << "VELE: " << l_ucRxBuffer[36] << std::endl;
		std::cout << "VELD: " << l_ucRxBuffer[37] << std::endl;
		std::cout << "VELACC: " << (uint32_t)l_ucRxBuffer[38] << std::endl;

		std::cout << "RELPOSEN2: " << l_ucRxBuffer[39] << std::endl;
		std::cout << "RELPOSEE2: " << l_ucRxBuffer[40] << std::endl;
		std::cout << "RELPOSED2: " << l_ucRxBuffer[41] << std::endl;
		std::cout << "ACCN2: " << (uint32_t)l_ucRxBuffer[42] << std::endl;
		std::cout << "ACCE2: " << (uint32_t)l_ucRxBuffer[43] << std::endl;
		std::cout << "ACCD2: " << (uint32_t)l_ucRxBuffer[44] << std::endl;

        std::cout << "VELN2: " << l_ucRxBuffer[45] << std::endl;
		std::cout << "VELE2: " << l_ucRxBuffer[46] << std::endl;
		std::cout << "VELD2: " << l_ucRxBuffer[47] << std::endl;
		std::cout << "VELACC2: " << (uint32_t)l_ucRxBuffer[48] << std::endl;

		std::cout << "BARO ALTITUDE: " <<  l_ucRxBuffer[49] << std::endl;
		std::cout << "PRESSURE: " <<   l_ucRxBuffer[50]  << " hPa" << std::endl;
		std::cout << "BARO TEMP: " <<  l_ucRxBuffer[51] << std::endl;
		
        std::cout << "TIME OF WEEK: " <<  (uint32_t)l_ucRxBuffer[52] << std::endl;
        std::cout << "DELTA TIME IMU: " << l_ucRxBuffer[53] << std::endl;
        std::cout << "DELTA TIME BARO: " <<  l_ucRxBuffer[54] << std::endl;
        std::cout << "DELTA TIME LINEAR ACTUATOR MS: " << l_ucRxBuffer[55] << std::endl;
}

bool Com::buildData(int* l_ucRxBuffer, int l_iFrameSize, int* l_iCounter) {
	RXData l_midLevel;
	int l_iI = 0;
	l_midLevel.HEADER = l_ucRxBuffer[l_iI++];
	l_midLevel.FOOTER = l_ucRxBuffer[l_iFrameSize/4 - 1];
	l_midLevel.CHECKSUM = l_ucRxBuffer[l_iFrameSize/4 - 2];
	//std::cout << "HEADER: " << l_midLevel.HEADER << std::endl;
	//printReceivedFrame(l_ucRxBuffer);
	//std::cout << "FOOTER: " << l_midLevel.FOOTER << std::endl;

	if(consistencyCheckHpF(l_ucRxBuffer, l_iFrameSize, l_midLevel.HEADER, l_midLevel.FOOTER, l_midLevel.CHECKSUM)) {
		if(l_iCounter != NULL)
			(*l_iCounter)++;
        
		l_midLevel.Q0 = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.Q1 = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.Q2 = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.Q3 = ((float)l_ucRxBuffer[l_iI++])/10000;	

		l_midLevel.Q0_TMP = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.Q1_TMP = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.Q2_TMP = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.Q3_TMP = ((float)l_ucRxBuffer[l_iI++])/10000;	

		l_midLevel.AX = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.AY = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.AZ = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.GX = ((float)l_ucRxBuffer[l_iI++])/10000;			
		l_midLevel.GY = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.GZ = ((float)l_ucRxBuffer[l_iI++])/10000;
	    l_midLevel.MX = ((float)l_ucRxBuffer[l_iI++])/10000;			
		l_midLevel.MY = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.MZ = ((float)l_ucRxBuffer[l_iI++])/10000;
		l_midLevel.GPS_LATITUDE = l_ucRxBuffer[l_iI++];
		l_midLevel.GPS_LATITUDE_HP = l_ucRxBuffer[l_iI++];

		l_midLevel.GPS_LONGITUDE = l_ucRxBuffer[l_iI++];
		l_midLevel.GPS_LONGITUDE_HP = l_ucRxBuffer[l_iI++];
		
		l_midLevel.GPS_ALTITUDE = l_ucRxBuffer[l_iI++];    
		l_midLevel.GPS_ALTITUDE_HP = l_ucRxBuffer[l_iI++];    
		
		l_midLevel.GPS_EPH = l_ucRxBuffer[l_iI++];
	    l_midLevel.GPS_EPV = l_ucRxBuffer[l_iI++];
		
		l_midLevel.GPS_1_FIXMODE = l_ucRxBuffer[l_iI++];
        l_midLevel.GPS_2_FIXMODE = l_ucRxBuffer[l_iI++];
        l_midLevel.GPS_SYSTEM_ERROR = l_ucRxBuffer[l_iI++];
		
		l_midLevel.RELPOSN = l_ucRxBuffer[l_iI++];
		l_midLevel.RELPOSE = l_ucRxBuffer[l_iI++];
		l_midLevel.RELPOSD = l_ucRxBuffer[l_iI++];
		l_midLevel.ACCN = l_ucRxBuffer[l_iI++];
		l_midLevel.ACCE = l_ucRxBuffer[l_iI++];
		l_midLevel.ACCD = l_ucRxBuffer[l_iI++];

        l_midLevel.VELN = l_ucRxBuffer[l_iI++];
		l_midLevel.VELE = l_ucRxBuffer[l_iI++];
		l_midLevel.VELD = l_ucRxBuffer[l_iI++];
		l_midLevel.VELACC = l_ucRxBuffer[l_iI++];

		l_midLevel.RELPOSN2 = l_ucRxBuffer[l_iI++];
		l_midLevel.RELPOSE2 = l_ucRxBuffer[l_iI++];
		l_midLevel.RELPOSD2 = l_ucRxBuffer[l_iI++];

		l_midLevel.ACCN2 = l_ucRxBuffer[l_iI++];
		l_midLevel.ACCE2 = l_ucRxBuffer[l_iI++];
		l_midLevel.ACCD2 = l_ucRxBuffer[l_iI++];

        l_midLevel.VELN2 = l_ucRxBuffer[l_iI++];
		l_midLevel.VELE2 = l_ucRxBuffer[l_iI++];
		l_midLevel.VELD2 = l_ucRxBuffer[l_iI++];
		l_midLevel.VELACC2 = l_ucRxBuffer[l_iI++];

		l_midLevel.BARO_ALTITUDE = ((double)l_ucRxBuffer[l_iI++])/100;
		l_midLevel.BARO_PRESSURE = ((double)l_ucRxBuffer[l_iI++])/1000;
		l_midLevel.BARO_TEMP = ((float)l_ucRxBuffer[l_iI++])/100;

        l_midLevel.TIME_OF_WEEK = (uint32_t)l_ucRxBuffer[l_iI++];
		l_midLevel.DELTA_TIME_IMU_MS = (int32_t)l_ucRxBuffer[l_iI++];
        l_midLevel.DELTA_TIME_BARO_MS = (int32_t)l_ucRxBuffer[l_iI++];	        	
		l_midLevel.DELTA_TIME_LINEAR_ACTUATOR_MS = (int32_t)l_ucRxBuffer[l_iI++];

		RxFrame = l_midLevel;
		return true;
	}
	if(l_ucRxBuffer[0] != iToHighLevelHeader) {
		uint8_t l_buffer[4];
		int i = -1;
		int *l_iCercatoreDiHeader = (int*)(l_buffer);
		while(i<l_iFrameSize && *l_iCercatoreDiHeader!= iToHighLevelHeader) {
			i++;
			for(int j=0; j<3; j++)
				l_buffer[j] = l_buffer[j+1];
			l_buffer[3] = ucRxBuffer[i];
		}
		if(i>2 && *l_iCercatoreDiHeader == iToHighLevelHeader) {
			iContatoreHeader++;
			iBytesRead = l_iFrameSize - i;
			return false;
		}
	}
	RxFrame = ignoreData();
	return false;
}

RXData Com::ignoreData() {
	iContatorePacchettiScartati++;
	iBytesRead = 0;
	RXData l_midLevel;
	l_midLevel.Q0 = 0;
	l_midLevel.Q1 = 0;
	l_midLevel.Q2 = 0;
	l_midLevel.Q3 = 0;	
	l_midLevel.Q0_TMP = 0;
	l_midLevel.Q1_TMP = 0;
	l_midLevel.Q2_TMP = 0;
	l_midLevel.Q3_TMP = 0;		
	l_midLevel.AX = 0;
	l_midLevel.AY = 0;
	l_midLevel.AZ = 0;
	l_midLevel.GX = 0;			
	l_midLevel.GY = 0;
	l_midLevel.GZ = 0;
	l_midLevel.GPS_LATITUDE = 0;
	l_midLevel.GPS_LONGITUDE = 0;
	l_midLevel.GPS_ALTITUDE = 0;    
	l_midLevel.GPS_EPH = 0;
	l_midLevel.GPS_EPV = 0;
	l_midLevel.BARO_ALTITUDE = 0;
	l_midLevel.BARO_PRESSURE = 0;
	l_midLevel.BARO_TEMP = 0;
    l_midLevel.TIME_OF_WEEK = 0;
    l_midLevel.DELTA_TIME_IMU_MS = 0;	
    l_midLevel.DELTA_TIME_BARO_MS = 0;	
	return l_midLevel;
}

void Com::uartComRxPrint(int l_iFrameSize) {
  
  uint8_t l_ucRxBuffer[l_iFrameSize];
  int read__a = 0;
  while (read__a < l_iFrameSize) {
	int l_iRxLength = read(iUart0Filestream, &l_ucRxBuffer[read__a], 1); //Filestream, buffer to store in, number of bytes to read (max)
	//std::cout << "Rx length: " << std::dec << l_iRxLength << std::endl;
	if (l_iRxLength == 1)
		read__a++;
	//else
	  //usleep(1000);
  }
  std::cout << "Byte read :" << std::dec << read__a << std::endl;//*(l_pucRxBuffer + i) << std::endl;
  for(int i = 0; i < read__a; i = i+1){
    std::cout << "Data :" << std::hex <<  (int32_t)(l_ucRxBuffer[i]) << std::endl;//*(l_pucRxBuffer + i) << std::endl;
  }
}

RXData Com::uartComRx() {
	int l_iFrameSize = 232;
	//--------------------
	/*l_iFrameSize è il numero di byte inviati nella struttura
	 * dati chiamata "toHighLevel" in typeDef.h nel codice
	 * Mixer. Dentro ci sono 27 interi (32 bit, 4 byte, ciascuno).
	 * Da 108 siamo passati a 112 perché ho aggiunto un int di
	 * checksum.
	 */
	//--------------------
	int* l_piRxInfoBin;
	int l_iNDataRx = (l_iFrameSize-3)/2;
	int l_iAttitude[l_iNDataRx];
	bool l_bConsistencyCheck = false;
	int l_iRxLength = 0;

	/*for(int i = 0 ; i < 8 ; i++ )
		RxFrame.STATEBIN[i] = 0;*/
		
	int l_cercatoreDiHeader = 0;
	int l_len = 0;
	uint8_t l_bufferProvvisorio[4];
	uint8_t l_byte;
		
	//----- CHECK FOR ANY RX BYTES -----
	if (iUart0Filestream != -1)
	{
		// Read up to 255 characters from the port if they are there
		int l_iRxInfo;
		if(iBytesRead == 0) {
			while(l_cercatoreDiHeader != iToHighLevelHeader) {
				l_len = read(iUart0Filestream, &l_byte, 1);
				if(l_len == 1) {
					for(int i=0; i<3; i++) //memorizziamo byte a byte
						l_bufferProvvisorio[i] = l_bufferProvvisorio[i+1];
					l_bufferProvvisorio[3] = l_byte;
					memcpy(&l_cercatoreDiHeader, l_bufferProvvisorio, 4);
				}
			}
			memcpy(ucRxBuffer, &l_cercatoreDiHeader, 4);
			iBytesRead = 4;
		}
		l_iRxLength = read(iUart0Filestream, &ucRxBuffer[iBytesRead], l_iFrameSize-iBytesRead); //Filestream, buffer to store in, number of bytes to read (max)
		if (l_iRxLength < 0)
		{
		  //An error occured (will occur if there are no bytes)
		  std::cout << "COM LOST" << std::endl;
		  for(int i = 0 ; i < l_iNDataRx ; i++)
			l_iAttitude[i] = 0;
			//RxFrame.FAILSAFE = true;
		}
		else if (l_iRxLength == 0)
		{
		  //No data waiting
		  std::cout << "MIXER ERROR " << std::endl;	
		  for(int i = 0 ; i < 4 ; i++)
			l_iAttitude[i] = 0;
		  //RxFrame.FAILSAFE = true;
		}	
		else if(l_iRxLength == l_iFrameSize-iBytesRead) {
			int* l_iCounter = &iPacchettiBuoni;
			if(l_iRxLength < l_iFrameSize-4)
				l_iCounter = &iContatorePacchettiRicostruiti;
			if(buildData((int*)ucRxBuffer, l_iFrameSize, l_iCounter))
				iBytesRead = 0;
		}
		else
			iBytesRead += l_iRxLength;
	}
	//std::cout << "byte in: " << l_iRxLength << std::endl;	
    //std::cout << "failsafe: " << l_midLevel.FAILSAFE << std::endl;
	return RxFrame;
}

void Com::UartReboot(){
    this->CloseUart();
    //usleep(10000);		 
    this->OpenUart();
}

void Com::getSwitchState(int l_iRx){
	int l_i = 7;
	while( l_i >= 0){
	 if(i8bitConv[l_i] <= l_iRx){
		 iRxInfoBin[l_i] = 1;
		 l_iRx = l_iRx - i8bitConv[l_i];
		}
	 else 
		iRxInfoBin[l_i] = 0;
	 l_i = l_i - 1;
	}
}

void Com::getFloat(int* l_piData, int l_iNDataToBeReconstructed, float* l_pfDataCoverted){
   int l_iData[l_iNDataToBeReconstructed];
   float l_fDataCoverted[l_iNDataToBeReconstructed];
   for(int i = 0 ; i < l_iNDataToBeReconstructed; i++)
	  *(l_fDataCoverted+i) = ((float)(*(l_piData+i)))/10;
}

bool Com::consistencyCheckHpF(int* l_pucRxBuffer, int l_iFrameSize, int l_iHeader, int l_iFooter, int l_iChecksum){
     bool l_bOk = false;
     if(l_iHeader == iToHighLevelHeader && l_iFooter == iToHighLevelFooter)
        l_bOk = true;
     //return l_bOk && verifyChecksum(l_pucRxBuffer, l_iChecksum, l_iFrameSize/4);
	 return l_bOk && verifyChecksum_tmp(l_pucRxBuffer, l_iChecksum, l_iFrameSize/4);
}

