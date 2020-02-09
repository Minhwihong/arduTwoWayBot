
#include "main.h"


UDP_pkt_t rcvPkts[5];
UDP_rcvSts_t UDP_rcvSts = eUDP_wait_frmId;



static double dt_PID = 0.0;


static bool sts_sendIMU = false;
static bool sts_sendPID = false;





static uint8_t CS_Calculation(uint8_t *inArr, uint16_t leng)
{
	uint16_t ret = 0;
	
	for(int idx=0; idx < leng; ++ idx)
	{
		ret += (uint16_t)inArr[idx];

		if (ret > 0xFF)
			ret -= 0xFF;
	}

	return (uint8_t)ret;

}


static void sendUDP_dummy()
{
	uint8_t myBuffer[100] = {0};
	int offset = 0;

	//Frame ID
	myBuffer[offset++] = (uint8_t)FRM_NORMAL;

	//Frame Idx
	myBuffer[offset++] = FRM_NORMAL_IDX;

	//Packet ID
	myBuffer[offset++] = PKT_ID_NORMAL;

	//Packet CMD
	myBuffer[offset++] = uCMD_SYS_SEND_ALIVE;

	//Data length
	myBuffer[offset++] = 0;

	uint8_t cs = CS_Calculation(myBuffer, offset);

	myBuffer[offset++] = cs;

	char chAddr[myUDPClient.addr.length()+1] = {'\0'};
	myUDPClient.addr.toCharArray(chAddr, myUDPClient.addr.length()+1);

	udp_client.beginPacket(chAddr, myUDPClient.port);
	udp_client.write(myBuffer, offset);

	udp_client.endPacket();
}



static void UDP_initPacketStructure(UDP_pkt_t* inPkt)
{
	inPkt->checksum = 0;
	inPkt->frmId = 0;
	inPkt->frmIdx = 0;
	inPkt->pktCmd = 0;
	inPkt->pktId = 0;
	inPkt->pktLen = 0;
	inPkt->pktOK = false;
	memset(inPkt->data,0,256);
}





signed char Serial_Message_Task(uint8_t id, uint16_t param)
{

	SerialReadHandle();
	return 0;
}




signed char WIFI_Handle_Task(uint8_t id, uint16_t param)
{

	return 0;
}






void UDP_cmdControl(uint8_t cmd, uint8_t *payload)
{
	uint8_t u8Val = 0;
	uint8_t	arrBuffer[16];
	uint8_t add = 0;
	float fVal =0.0f;

	PRINT_DBG("--- cmd control start ---\r\n");
	PRINT_DBG(" cmd: "); 	PRINTLN_DBG(cmd);
	PRINT_DBG("-------------------------\r\n");
	//PRINT_DBG("cmd: "); 	PRINT_DBG(cmd);

	switch(cmd)
	{
		case uCMD_MT_SPD:
			mSpd = payload[0];
			//myMotor.motorForward(0, mSpd);
			
			PRINT_DBG("[Set Motor Speed]val: ");	PRINTLN_DBG(mSpd);
			break;
			
		case uCMD_MT_STOP:  		//MotorSpeed Control
			mSpd = payload[0];
			myMotor.motorForward(0, 0);
		
			PRINTLN_DBG("Motor Stop");
			break;
			
		case uCMD_MT_MOVE:  			//Motor Dir Change
			dir = payload[0];

			PRINT_DBG("[Motor Speed]val: ");	PRINTLN_DBG(mSpd);
			if(dir == 0)
			{
				myMotor.motorForward(0, mSpd);
				PRINTLN_DBG(" Forward");
			}
			else
			{
				myMotor.motorReverse(0, mSpd);
				PRINTLN_DBG(" Backward");
			}
			
			break;

		case uCMD_PID_ONOFF:		// start stop PID Control
			u8Val = payload[0];
			if(u8Val == 1)
				PID_Start = true;
			else {
				PID_Start = false;
				PID_Init();
			}

			PRINT_DBG("[Set PID OnOff]");	
			(PID_Start == true) ? PRINTLN_DBG(" PID Start~") : PRINTLN_DBG(" PID Stop!");
			break;

		case uCMD_PID_RST:	// PID Reset
			PID_Init();
			PRINTLN_DBG("[PID Reset]");
			break;

		case uCMD_PID_SET_K_PARAM:	// Set Ki, Kp, Kd value
			add = 0;
#if 0
						fVal = 0.0f;
			Serial.println("-------- payload ---------");
			for(uint8_t idx =0; idx < 4*3; ++idx)	
			{
				Serial.print(payload[idx], HEX);
				Serial.print(", ");
			}
			Serial.print("\r\n");
			Serial.println("--------------------------");	
#endif	
			
			for(uint8_t idx =0; idx < (uint8_t)sizeof(double); ++idx)	arrBuffer[idx] = payload[add++];
			memcpy(&pid_cfg.Kp, arrBuffer, sizeof(double));

			for(uint8_t idx =0; idx < (uint8_t)sizeof(double); ++idx)	arrBuffer[idx] = payload[add++];
			memcpy(&pid_cfg.Ki, arrBuffer, sizeof(double));

			for(uint8_t idx =0; idx < (uint8_t)sizeof(double); ++idx)	arrBuffer[idx] = payload[add++];
			memcpy(&pid_cfg.Kd, arrBuffer, sizeof(double));
			
			PRINT_DBG("[Set PID param] Kp: ");	Serial.println(pid_cfg.Kp,4); //PRINTLN_DBG(pid_cfg.Kp,4); 			
			PRINT_DBG("[Set PID param] Ki: ");	Serial.println(pid_cfg.Ki,4);//PRINTLN_DBG(pid_cfg.Ki,4);
			PRINT_DBG("[Set PID param] Kd: ");	Serial.println(pid_cfg.Kd,4);//PRINTLN_DBG(pid_cfg.Kd,4);
			break;

		case uCMD_PID_SET_TRT_ANGLE:	// Target Angle set
			add = 0;
			for(uint8_t idx =0; idx < (uint8_t)sizeof(double); ++idx)	arrBuffer[idx] = payload[add++];	
			memcpy(&pid_cfg.targetAngle, arrBuffer, sizeof(double));
			
			PRINT_DBG("[Set target Angle] angle: ");	Serial.println(pid_cfg.targetAngle,4);//PRINTLN_DBG(pid_cfg.targetAngle,4);
			break;

		case uCMD_PID_SEND_PARAM:	// send err parameter
			if(sts_sendPID == false)	sts_sendPID = true;
			else 						sts_sendPID = false;

			(sts_sendPID == true) ? PRINTLN_DBG("PID parameter send start~") : PRINTLN_DBG("PID parameter send stop~") ;
			
			break;

		case uCMD_IMU_SEND_VALUE:
			if(sts_sendIMU == false)	sts_sendIMU = true;
			else 						sts_sendIMU = false;

			(sts_sendIMU == true) ? PRINTLN_DBG("IMU logging start~") : PRINTLN_DBG("IMU logging stop~") ;
			break;


	}
  
}




static void UDP_Send_Arrive()
{
	static uint16_t aliveSend = 0;

	aliveSend++;
	
	if(aliveSend >= 20)
	{
		aliveSend = 0;
		sendUDP_dummy();
	}

}


signed char UDP_Message_Task(uint8_t id, uint16_t param)
{
	byte  packetBuffer[255];
	int   leng;
	uint8_t		pktValid; 
	UDP_pkt_t* rcvPkt = NULL;


	if(sts_sendIMU == false)	UDP_Send_Arrive();


	rcvPkt = UDP_queue_get_data(&pktValid);

	if(rcvPkt == NULL || rcvPkt->pktOK == false)
		return -1;

	rcvPkt->pktOK = false;
	PRINT_DBG("rcv cmd: ");
	PRINTLN_DBG(rcvPkt->pktCmd);
	PRINTLN_DBG("\r\n");
	
	UDP_cmdControl(rcvPkt->pktCmd , rcvPkt->data);
	

	return 0;
}


signed char Balacing_Msg_Task(uint8_t id, uint16_t param)
{
	
	if(sts_sendPID == true)
	{
		send_PID_Parameter();
	}

	if(sts_sendIMU == true)
		sendIMUData();

	return 0;
}



signed char Balancing_Task(uint8_t id, uint16_t param)
{
	static uint16_t tickIMU_conn = 0;

	if (IMU_status < 0) {
		tickIMU_conn++;
		if(tickIMU_conn >= 40)
		{
			tickIMU_conn = 0;
			Serial.println("[IMUtask]trying IMU conn");
			IMU_Init();
			Serial.println("\r\n");
		}

		return 0;
	}

	if(getIMUData() < 0)
	{
		sts_sendIMU = false;
		return -1;
	}


	if(PID_Start)	PID_Control();
		

	
	
	//getIMUData();
	return 0;
}


signed char Config_Message_Task(uint8_t id, uint16_t param)
{


	return 0;
}





signed char LedToggle_Task(uint8_t id, uint16_t param)
{

	return 0;
}


signed char Task_UDP_Rcv_Handler(uint8_t id, uint16_t param)
{
	UDP_rcvHandler();

	return 0;
}




double PID_Control()
{
  	double output 	= 0.0;
	double theta  	= atan(valMPU9255.AccelY / valMPU9255.AccelZ);
	last_err 		= theta;

	
	double err 		= pid_cfg.targetAngle - last_err;
	//accum_err 		+= (err*PID_DT);   
	accum_err 		+= (err*dt_PID);   

	pid_cfg.P_Reg 	= pid_cfg.Kp * theta;
	pid_cfg.I_Reg 	= pid_cfg.Ki * accum_err;

	PID_val 		= pid_cfg.P_Reg + pid_cfg.I_Reg;
	output 			= PID_val;
	return output;
  
}






void sendData_toServer(uint8_t cmd, uint8_t *inByte, int leng, UDP_Client_t myClient)
{
	uint8_t myBuffer[100] = {0};
	int offset = 0;

	//Frame ID
	myBuffer[offset++] = (uint8_t)FRM_NORMAL;

	//Frame Idx
	myBuffer[offset++] = FRM_NORMAL_IDX;

	//Packet ID
	myBuffer[offset++] = PKT_ID_NORMAL;

	//Packet CMD
	myBuffer[offset++] = cmd;

	//Data length
	myBuffer[offset++] = (uint8_t)leng;

	memcpy(&myBuffer[offset], inByte, leng);
	offset += leng;

	uint8_t cs = CS_Calculation(myBuffer, offset);

	myBuffer[offset++] = cs;

	char chAddr[myClient.addr.length()+1] = {'\0'};
	myClient.addr.toCharArray(chAddr, myClient.addr.length()+1);

	//Serial.print("  IP: ");
	//Serial.println(chAddr);
	//Serial.print("Port: ");
	//Serial.println(myClient.port);

	udp_client.beginPacket(chAddr, myClient.port);
	udp_client.write(myBuffer, offset);

	udp_client.endPacket();
  //if(udp_client.endPacket() == 1)
    //Serial.println("Data Send OK!");
  //else
   // Serial.println("Data Send Failed..");
} 



uint8_t getIMUData()
{
	int ret = inIMU.readSensor();
	
	// display the data

	if(ret < 0)
		return (uint8_t)ret;

	dt_PID 		= tmIntvIMU;	//Time elapse after previous getting IMU data (unit: 1ms)
	tmIntvIMU 	= 0;			//Reset elapse time
	
	
	// G value output unit : g
	valMPU9255.AccelX = inIMU.getAccelX_mss(); 
	valMPU9255.AccelY = inIMU.getAccelY_mss(); 
	valMPU9255.AccelZ = inIMU.getAccelZ_mss(); 

	// Gyro value output unit : radian
	valMPU9255.GyroX = inIMU.getGyroX_rads(); 
	valMPU9255.GyroY = inIMU.getGyroY_rads(); 
	valMPU9255.GyroZ = inIMU.getGyroZ_rads(); 

	return 0;
  
}



void send_PID_Parameter()
{
	byte payload[sizeof(double)*5] = {0};
	uint8_t add = 0;
	uint8_t cmd = uCMD_PID_SEND_PARAM;

	memcpy(&payload[add], &last_err, sizeof(double));
	add += sizeof(double);

	memcpy(&payload[add], &accum_err, sizeof(double));
	add += sizeof(double);

	memcpy(&payload[add], &pid_cfg.P_Reg, sizeof(double));
	add += sizeof(double);

	memcpy(&payload[add], &pid_cfg.I_Reg, sizeof(double));
	add += sizeof(double);

	memcpy(&payload[add], &pid_cfg.D_Reg, sizeof(double));
	add += sizeof(double);


	sendData_toServer(cmd, payload, add, myUDPClient);
}


void sendIMUData()
{
	int valMPU9255_leng = sizeof(IMU_6axis_t);
	int leng = valMPU9255_leng;
	int add = 0;

	uint8_t payload[valMPU9255_leng] = {0};
	uint8_t cmd = uCMD_IMU_SEND_VALUE;

	
	memcpy(&payload[add], &valMPU9255.AccelX, sizeof(float));
	add += sizeof(float);
	
	memcpy(&payload[add], &valMPU9255.AccelY, sizeof(float));
	add += sizeof(float);
	
	memcpy(&payload[add], &valMPU9255.AccelZ, sizeof(float));
	add += sizeof(float);
	
	memcpy(&payload[add], &valMPU9255.GyroX, sizeof(float));
	add += sizeof(float);
	
	memcpy(&payload[add], &valMPU9255.GyroY, sizeof(float));
	add += sizeof(float);
	
	memcpy(&payload[add], &valMPU9255.GyroZ, sizeof(float));
	add += sizeof(float);

	sendData_toServer(cmd, payload, add, myUDPClient);
}








void SerialReadHandle()
{
  if(Serial.available()){
    String inString = Serial.readStringUntil('\n');
    Serial.print("serial read: ");
    Serial.print(inString);
    Serial.print("\n");
    
    int separator = inString.indexOf(','); 
    int len       = inString.length();
    String strCmd = inString.substring(0, separator);
    String strVal = inString.substring(separator+1,len);

    if(strCmd == "m_spd")
    {
      mSpd = strVal.toInt();
      Serial.print("motor speed: ");
      Serial.println(mSpd);

      if(mSpd <= 0)
        myMotor.motorForward(0, 0);
      else if(mSpd >= 255)
        myMotor.motorForward(0, 255);
      else
        myMotor.motorForward(0, mSpd);

    }
    else if(strCmd == "m_dir")
    {
      Serial.print("motor direction: ");
      Serial.println(strVal);

      if(strVal == "0")
        myMotor.motorForward(0, mSpd);
      else if(strVal == "1")
        myMotor.motorReverse(0, mSpd);
    } 
 }
  
}



		

void UDP_rcvHandler()
{
	static int 			dataIdx = 0;
	static UDP_pkt_t 	rcvingPkt;

	byte  packetBuffer[255];
	int   leng;
	int   validPktLeng = 0;
	bool  rcvOK = receiveData_fromServer(packetBuffer, &leng);

	if(rcvOK == false)	return;
	
	uint16_t add 	= 0;


	while(leng != add)
	{
		switch(UDP_rcvSts)
		{
			case eUDP_wait_frmId:
				UDP_initPacketStructure(&rcvingPkt);
				rcvingPkt.frmId 	= packetBuffer[add++];
#ifdef ISR_DBG_MODE
				PRINT_LOG(add,leng);
				PRINT_DBG("frmId :");
				PRINTLN_DBG(rcvingPkt.frmId);
#endif				
				switch(rcvingPkt.frmId)
				{
					case FRM_NORMAL:
					case FRM_DEBUG:
					case FRM_UPDATE:	UDP_rcvSts	= eUDP_wait_frmIdx;	break;
					default:			UDP_rcvSts 	= eUDP_wait_frmId;	break;
				}

				break;
				
			case eUDP_wait_frmIdx:
				rcvingPkt.frmIdx 	= packetBuffer[add++];

#ifdef ISR_DBG_MODE
				PRINT_LOG(add,leng);
				PRINT_DBG("frmIdx:");
				PRINTLN_DBG(rcvingPkt.frmIdx);
#endif				
				switch(rcvingPkt.frmIdx)
				{
					case FRM_NORMAL_IDX:UDP_rcvSts	= eUDP_wait_pktId;	break;
					default:			UDP_rcvSts	= eUDP_wait_frmId;	break;
				}

				break;
				
			case eUDP_wait_pktId:
				rcvingPkt.pktId 	= packetBuffer[add++];
#ifdef ISR_DBG_MODE				
				PRINT_LOG(add,leng);
				PRINT_DBG("pktId :");
				PRINTLN_DBG(rcvingPkt.pktId);
#endif				
				switch(rcvingPkt.pktId)
				{
					case PKT_ID_NORMAL:	UDP_rcvSts	= eUDP_wait_pktCmd;	break;
					default:			UDP_rcvSts	= eUDP_wait_frmId;	break;
				}
				break;
				
			case eUDP_wait_pktCmd:
				rcvingPkt.pktCmd 	= packetBuffer[add++];
				
#ifdef ISR_DBG_MODE
				PRINT_LOG(add,leng);
				PRINT_DBG("pktCmd:");
				PRINTLN_DBG(rcvingPkt.pktCmd);
#endif				
				switch(rcvingPkt.pktId)
				{
					case uCMD_MT_SPD:
					case uCMD_MT_STOP:
					case uCMD_MT_MOVE:
					case uCMD_PID_ONOFF:
					case uCMD_PID_RST:
					case uCMD_PID_SET_K_PARAM:
					case uCMD_PID_SET_TRT_ANGLE:UDP_rcvSts	= eUDP_wait_pktLen;	break;
					default:					UDP_rcvSts	= eUDP_wait_frmId;	break;
				}
				
				UDP_rcvSts 			= eUDP_wait_pktLen;
				break;
				
			case eUDP_wait_pktLen:
				rcvingPkt.pktLen 	= packetBuffer[add++];
#ifdef ISR_DBG_MODE
				PRINT_LOG(add,leng);			
				PRINT_DBG("pktLen:");
				PRINTLN_DBG(rcvingPkt.pktLen);
#endif			

				if(rcvingPkt.pktLen == 0)	UDP_rcvSts 		= eUDP_wait_cs;
				else						UDP_rcvSts 		= eUDP_wait_data;
				
				dataIdx				= 0;
				break;
				
			case eUDP_wait_data:
				rcvingPkt.data[dataIdx++] = packetBuffer[add++];

#ifdef ISR_DBG_MODE
				PRINT_LOG(add,leng);
				PRINT_DBG("pktLen:");
				PRINTLN_DBG(rcvingPkt.pktLen);
#endif			
				if(dataIdx == rcvingPkt.pktLen)
				{
					UDP_rcvSts 			= eUDP_wait_cs;
				}
				break;
				
			case eUDP_wait_cs:
				rcvingPkt.checksum = packetBuffer[add++];
				validPktLeng = rcvingPkt.pktLen + 5;
				uint8_t calCs = CS_Calculation(packetBuffer, validPktLeng);
#ifdef ISR_DBG_MODE
				PRINT_LOG(add,leng);
				PRINT_DBG("calCs :");
				PRINT_DBG(calCs);
				PRINT_DBG(", pktCs :");
				PRINTLN_DBG(rcvingPkt.checksum);
#endif				
				if(rcvingPkt.checksum == calCs)
				{
					rcvingPkt.pktOK = true;
					//PRINTLN_DBG("enqueue");
					UDP_queue_push_back(&rcvingPkt);
					
				}

				UDP_rcvSts	= eUDP_wait_frmId;				
				break;
				
				
		}

	}


}


bool receiveData_fromServer(byte *rcvPkt, int *leng)
{
  // if there's data available, read a packet
	int packetSize = udp_client.parsePacket();

	if (packetSize) {
		IPAddress remoteIp = udp_client.remoteIP();
		
	
#ifdef ISR_DBG_MODE	
		PRINT_DBG(remoteIp);
		PRINT_DBG("Received packet of size ");
		PRINTLN_DBG(packetSize);
		PRINT_DBG("From ");
		PRINT_DBG(", port ");
		PRINTLN_DBG(udp_client.remotePort());
#endif
		// read the packet into packetBufffer
		*leng = udp_client.read(rcvPkt, 255);
		if (*leng > 0) {
			rcvPkt[*leng] = 0;
		}

		//Serial.println(packetBuffer);
		return true;
	}
	else
	{
#ifdef ISR_DBG_MODE	
		PRINT_DBG("No UDP receive data\r\n");
#endif
		return false;
	}
		
  
}









