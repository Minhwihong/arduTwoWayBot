#ifndef __MAIN_H
#define __MAIN_H


#include <Wire.h>
#include "MPU9250.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

#include <ESP32MotorControl.h>
#include <ESP32Servo.h>

#include "queue.h"
#include "task_setting.h"
#include "task_operation.h"



#define TARGET_DEG -2.0
#define PID_DT      0.1




//#define ISR_DBG_MODE

#define PRINT_DBG(A)	Serial.print((A))
#define PRINTLN_DBG(A)	Serial.println((A))

#define PRINT_LOG(A,B) 	Serial.print("["); 	\
				Serial.print((A));			\
				Serial.print("/");			\
				Serial.print((B));			\
				Serial.print("]");	


//Serial.print


typedef enum {
	uCMD_SYS_SEND_ALIVE = 0x05,
	uCMD_MT_SPD = 0x10,
	uCMD_MT_STOP,
	uCMD_MT_MOVE,
	uCMD_PID_ONOFF = 0x14,
	uCMD_PID_RST,
	uCMD_PID_SET_K_PARAM,
	uCMD_PID_SET_TRT_ANGLE,
	uCMD_PID_SEND_PARAM,
	uCMD_IMU_SEND_VALUE
}UDP_cmd_t;


// ---------------------- Setup Wifi & UDP client --------------------- //
typedef struct UDP_Client {
  String addr;
  int port;
}UDP_Client_t;

#if 0
typedef struct {
	bool rcvReq_IMU;
	bool rcvReq_PID;

}UDP_rcvSts_t;
#endif

typedef enum  {
	eUDP_wait_frmId = 0,
	eUDP_wait_frmIdx,
	eUDP_wait_pktId ,
	eUDP_wait_pktCmd ,
	eUDP_wait_pktLen ,
	eUDP_wait_data ,
	eUDP_wait_cs ,
}UDP_rcvSts_t;



typedef struct {
  double Kp, Ki, Kd; // PID variables
  double targetAngle; // Resting angle of the robot
  double P_Reg, I_Reg, D_Reg;
  //uint8_t turningScale; // Set the turning scale value
  //double Qangle, Qbias, Rmeasure; // Kalman filter values
  // TODO: Implement this:
  //double accYzero, accZzero; // Accelerometer zero values
  //double leftMotorScaler, rightMotorScaler; // Used if there is difference between the motors
} PID_cfg_t;



// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
typedef struct IMU_6axis {
  float AccelX;
  float AccelY ;
  float AccelZ;
  
  float GyroX ;
  float GyroY ;
  float GyroZ  ;
}IMU_6axis_t;





extern UDP_rcvSts_t UDP_rcvSts;





//create UDP instance
extern WiFiUDP udp_client;
extern UDP_Client_t myUDPClient;
extern UDP_rcvSts_t UDP_RcvSts;


extern PID_cfg_t pid_cfg;
extern double accum_err;
extern double last_err;
extern double PID_val;
extern bool PID_Start;


////double roll  = atan2(accY, accZ) * RAD_TO_DEG;

extern ESP32MotorControl myMotor;
extern MPU9250 inIMU;


extern IMU_6axis_t valMPU9255;
extern int IMU_status;

extern double tmIntvIMU;



extern int MotorPinHigh;
extern int MotorPinLow ;
extern int motorSpd  ;
extern byte mSpd ;
extern byte dir ;


//extern UDP_pkt_t rcvPkt;






void init_UDPClient();
void sendData_toServer(uint8_t cmd, uint8_t *inByte, int leng, UDP_Client_t myClient);
void WiFiEvent(WiFiEvent_t event);
bool receiveData_fromServer(byte *rcvPkt, int *leng);
void UDP_cmdControl(uint8_t cmd, uint8_t *payload);

double PID_Control();
void PID_Init();
void IMU_Init();

uint8_t getIMUData();
void sendIMUData();
void SerialReadHandle();
void send_PID_Parameter();

#endif



