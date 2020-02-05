
#include "main.h"


UDP_pkt_t rcvPkts[5];
UDP_rcvSts_t UDP_rcvSts = eUDP_wait_frmId;


static uint8_t CS_Calculation(uint8_t *inArr, uint16_t leng)
{
	uint8_t ret = 0;
	
	for(int idx=0; idx < leng - 1; ++ idx)
	{
		ret += inArr[idx];

		if (ret > 0xFF)
			ret -= 0xFF;
	}

	return ret;

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



signed char UDP_Message_Task(uint8_t id, uint16_t param)
{
	
	byte  packetBuffer[255];
	int   leng;
	bool  rcvOK = receiveData_fromServer(packetBuffer, &leng);

	if(rcvOK == true)
	{
		byte cmd		  = packetBuffer[0];
		byte payload[254] = {0};

		uint16_t add = 0;
		

		while(leng != add)
		{
			switch(UDP_rcvSts)
			{
				case eUDP_wait_frmId:
					break;

				case eUDP_wait_frmId:
					break;

				case eUDP_wait_frmId:
					break;

				case eUDP_wait_frmId:
					break;

				case eUDP_wait_frmId:
					break;

				case eUDP_wait_frmId:
					break;

			}

		}






		

		
		memcpy(payload, &packetBuffer[1], leng);
		UDP_cmdControl(cmd, payload);
	}


	if(UDP_RcvSts.rcvReq_IMU == true)
	{
		UDP_RcvSts.rcvReq_IMU = false;
		sendIMUData();
	}

	if(UDP_RcvSts.rcvReq_PID == true)
	{
		UDP_RcvSts.rcvReq_PID = false;
		send_PID_Parameter();
	}


	return 0;
}


signed char Balancing_Proc_Task(uint8_t id, uint16_t param)
{
	if(PID_Start)
		PID_Control();


	return 0;
}



signed char IMU_Handle_Task(uint8_t id, uint16_t param)
{
	getIMUData();
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






double PID_Control()
{
  	double output 	= 0.0;
	double theta  	= atan(valMPU9255.AccelY / valMPU9255.AccelZ);
	last_err 		= theta;

	
	double err 		= pid_cfg.targetAngle - last_err;
	accum_err 		+= (err*PID_DT);   


	pid_cfg.P_Reg 	= pid_cfg.Kp * theta;
	pid_cfg.I_Reg 	= pid_cfg.Ki * accum_err;

	PID_val 		= pid_cfg.P_Reg + pid_cfg.I_Reg;
	output 			= PID_val;
	return output;
  
}



bool receiveData_fromServer(byte *rcvPkt, int *leng)
{
  // if there's data available, read a packet
  int packetSize = udp_client.parsePacket();
  
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = udp_client.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(udp_client.remotePort());

    // read the packet into packetBufffer
    *leng = udp_client.read(rcvPkt, 255);
    if (*leng > 0) {
      rcvPkt[*leng] = 0;
    }
    Serial.println("Contents:");
    //Serial.println(packetBuffer);
    return true;
  }
  else
    return false;
  
}


void sendData_toServer(byte *inByte, int leng, UDP_Client_t myClient)
{
  uint8_t myBuffer[100] = {0};
  uint8_t offset = 0;

  //Packet ID
  myBuffer[offset++] = 0x21;

  //Packet CMD
  myBuffer[offset++] = 0x01;

  //Data length
  myBuffer[offset++] = (byte)leng;
  
  memcpy(&myBuffer[offset], inByte, leng);

  char chAddr[myClient.addr.length()+1] = {'\0'};
  myClient.addr.toCharArray(chAddr, myClient.addr.length()+1);

  //Serial.print("  IP: ");
  //Serial.println(chAddr);
  //Serial.print("Port: ");
  //Serial.println(myClient.port);
  
  udp_client.beginPacket(chAddr, myClient.port);
  udp_client.write(myBuffer, leng);

  udp_client.endPacket();
  //if(udp_client.endPacket() == 1)
    //Serial.println("Data Send OK!");
  //else
   // Serial.println("Data Send Failed..");
} 



void getIMUData()
{
  IMU.readSensor();
  // display the data
  
  // G value output unit : g
  valMPU9255.AccelX = IMU.getAccelX_mss(); 
  valMPU9255.AccelY = IMU.getAccelY_mss(); 
  valMPU9255.AccelZ = IMU.getAccelZ_mss(); 
  
  // Gyro value output unit : radian
  valMPU9255.GyroX = IMU.getGyroX_rads(); 
  valMPU9255.GyroY = IMU.getGyroY_rads(); 
  valMPU9255.GyroZ = IMU.getGyroZ_rads(); 

  
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
	static int dataIdx = 0;

	byte  packetBuffer[255];
	int   leng;
	bool  rcvOK = receiveData_fromServer(packetBuffer, &leng);
	UDP_pkt_t rcvingPkt;
	uint16_t add = 0;

	rcvingPkt.pktOK = false;

	while(leng != add)
	{
		switch(UDP_rcvSts)
		{
			case eUDP_wait_frmId:
				UDP_initPacketStructure(&rcvingPkt);
				rcvingPkt.frmId 	= packetBuffer[add++];

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

				switch(rcvingPkt.frmIdx)
				{
					case FRM_NORMAL_IDX:UDP_rcvSts	= eUDP_wait_pktId;	break;
					default:			UDP_rcvSts	= eUDP_wait_frmId;	break;
				}

				break;
				
			case eUDP_wait_pktId:
				//PKT_ID_NORMAL
				rcvingPkt.pktId 	= packetBuffer[add++];

				switch(rcvingPkt.pktId)
				{
					case PKT_ID_NORMAL:	UDP_rcvSts	= eUDP_wait_pktCmd;	break;
					default:			UDP_rcvSts	= eUDP_wait_frmId;	break;
				}
				break;
				
			case eUDP_wait_pktCmd:
				rcvingPkt.pktCmd 	= packetBuffer[add++];

				switch(rcvingPkt.pktId)
				{
					case uCMD_MT_SPD:
					case uCMD_MT_STOP:
					case uCMD_MT_REV:
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
				UDP_rcvSts 			= eUDP_wait_data;
				dataIdx				= 0;
				break;
				
			case eUDP_wait_data:
				rcvingPkt.data[dataIdx++] = packetBuffer[add++];

				if(dataIdx == rcvingPkt.pktLen)
				{
					UDP_rcvSts 			= eUDP_wait_cs;
				}
				break;
				
			case eUDP_wait_cs:
				rcvingPkt.checksum = packetBuffer[add++];
				uint8_t calCs = CS_Calculation(packetBuffer, leng);

				if(rcvingPkt.checksum == calCs)
				{
					rcvingPkt.pktOK = true;
				}

				
				break;
				
				
		}

	}


}






void UDP_cmdControl(byte cmd, byte *payload)
{
	uint8_t u8Val = 0;
	uint8_t	arrFloat[4];
	uint8_t add = 0;

	switch(cmd)
	{
		case uCMD_MT_SPD:
			mSpd = payload[0];
			myMotor.motorForward(0, mSpd);
			break;
		case uCMD_MT_STOP:  		//MotorSpeed Control
			mSpd = payload[0];
			myMotor.motorForward(0, 0);
			break;
		case uCMD_MT_REV:  			//Motor Dir Change
			dir = payload[0];
			dir == 0 ? myMotor.motorForward(0, mSpd) : myMotor.motorReverse(0, mSpd);
			break;

		case uCMD_PID_ONOFF:		// start stop PID Control
			u8Val = payload[0];
			if(u8Val == 1)
				PID_Start = true;
			else {
				PID_Start = false;
				PID_Init();
			}
			break;

		case uCMD_PID_RST:	// PID Reset
			PID_Init();
			break;

		case uCMD_PID_SET_K_PARAM:	// Set Ki, Kp, Kd value
			add = 0;
			for(uint8_t idx =0; idx <4; ++idx)	arrFloat[idx++] = payload[add++];
			
			memcpy(&pid_cfg.Kp, arrFloat, 4);

			for(uint8_t idx =0; idx <4; ++idx)	arrFloat[idx++] = payload[add++];
			
			memcpy(&pid_cfg.Ki, arrFloat, 4);

			for(uint8_t idx =0; idx <4; ++idx)	arrFloat[idx++] = payload[add++];
			
			memcpy(&pid_cfg.Kd, arrFloat, 4);
			break;

		case uCMD_PID_SET_TRT_ANGLE:	// Target Angle set
			add = 0;
			for(uint8_t idx =0; idx <4; ++idx)	arrFloat[idx++] = payload[add++];
			
			memcpy(&pid_cfg.targetAngle, arrFloat, 4);
			break;

		case uCMD_PID_SEND_PARAM:	// send err parameter
			UDP_RcvSts.rcvReq_PID = true;
			//send_PID_Parameter();
			break;

		case uCMD_IMU_SEND_VALUE:
			UDP_RcvSts.rcvReq_IMU = true;
			break;


	}
  
}




void send_PID_Parameter()
{
	byte sendData[sizeof(double)*5 + 1] = {0};
	uint8_t add = 0;

	sendData[add++] = (uint8_t)uCMD_PID_SEND_PARAM;
		
	memcpy(&sendData[add], &last_err, sizeof(double));
	add += sizeof(double);

	memcpy(&sendData[add], &accum_err, sizeof(double));
	add += sizeof(double);

	memcpy(&sendData[add], &pid_cfg.P_Reg, sizeof(double));
	add += sizeof(double);

	memcpy(&sendData[add], &pid_cfg.I_Reg, sizeof(double));
	add += sizeof(double);

	memcpy(&sendData[add], &pid_cfg.D_Reg, sizeof(double));
	add += sizeof(double);


	sendData_toServer(sendData, add, myUDPClient);
}


void sendIMUData()
{
  int valMPU9255_leng = sizeof(IMU_6axis_t);
  int leng = valMPU9255_leng + 1;
  byte sendData[valMPU9255_leng + 1] = {0};

  sendData[0] = (uint8_t)uCMD_IMU_SEND_VALUE;
  
  memcpy(&sendData[1], &valMPU9255.AccelX, valMPU9255_leng);
  
  sendData_toServer(sendData, leng+4, myUDPClient);
}




