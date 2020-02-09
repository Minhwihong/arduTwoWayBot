#if 0
#include <Wire.h>
#include "MPU9250.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

#include <ESP32MotorControl.h>
#include <ESP32Servo.h>
#endif

#include "main.h"




const char * ssid = "Hwi_Yujin";
const char * pwd = "gnlghd0923";
String udpAddress = "192.168.0.19";
int udpPort = 44444;




WiFiUDP udp_client;
UDP_Client_t myUDPClient;
UDP_rcvSts_t UDP_RcvSts;
UDP_cmd_t UDP_cmd;


PID_cfg_t pid_cfg;
double accum_err = 0.0;
double last_err = 0.0;
double PID_val = 0.0;
bool PID_Start = false;
double tmIntvIMU = 0.0;


////double roll  = atan2(accY, accZ) * RAD_TO_DEG;

ESP32MotorControl myMotor;
MPU9250 inIMU(Wire,0x68);

IMU_6axis_t valMPU9255;
int IMU_status;



int MotorPinHigh  = 26;
int MotorPinLow   = 25;
int motorSpd      = 0;
byte mSpd = 0;
byte dir = 0;


volatile unsigned short dbgVar1 = 0;
volatile unsigned short dbgVar2 = 0;



// ---------------------------- Setup Timer -------------------------- //

hw_timer_t *tm_1sec = NULL;
hw_timer_t *tm_1ms 	= NULL;
hw_timer_t *tm_10ms = NULL;



volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;



void IRAM_ATTR tmISR_1sec(){

#if 0	
	// Increment the counter and set the time of ISR
	portENTER_CRITICAL_ISR(&timerMux);
	isrCounter++;
	//lastIsrAt = millis();
	portEXIT_CRITICAL_ISR(&timerMux);

	dbgVar1++;

	Serial.print("tm1 tick cnt: ");
	Serial.println(dbgVar1);

	// Give a semaphore that we can check in the loop
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
	// It is safe to use digitalRead/Write here if you want to toggle an output
#endif
}


void IRAM_ATTR tmISR_1ms(){
	//UDP_rcvHandler();
	tmIntvIMU++;
}


void IRAM_ATTR tmISR_10ms(){

	dbgVar2++;
	Task_Timecount();

#if 0	
	if(dbgVar2 >= 100)
	{
		dbgVar2 = 0;
		Serial.print("tm2 tick cnt: ");
		Serial.println(dbgVar2);
	}
#endif	

}



void timer_init()
{
	// Set BTN_STOP_ALARM to input mode
	//pinMode(BTN_STOP_ALARM, INPUT);
	
	// Create semaphore to inform us when the timer has fired
	timerSemaphore = xSemaphoreCreateBinary();
	
	// Use 1st timer of 4 (counted from zero).
	// Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
	// info).
	tm_1sec = timerBegin(0, 80, true);
	tm_1ms 	= timerBegin(1, 80, true);
	tm_10ms = timerBegin(2, 80, true);
	
	// Attach onTimer function to our timer.
	timerAttachInterrupt(tm_1sec, &tmISR_1sec, true);
	timerAttachInterrupt(tm_1ms, &tmISR_1ms, true);
	timerAttachInterrupt(tm_10ms, &tmISR_10ms, true);
	
	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter)
	timerAlarmWrite(tm_1sec, 1000000, true);
	timerAlarmWrite(tm_1ms,  1000,    true);
	timerAlarmWrite(tm_10ms, 10000,   true);
	
	// Start an alarm
	timerAlarmEnable(tm_1sec);
	delay(250);
	timerAlarmEnable(tm_1ms);
	delay(250);
	timerAlarmEnable(tm_10ms);


}


void timer_stop()
{
	// If timer is still running
	if (tm_1sec) {
		// Stop and free timer
		timerEnd(tm_1sec);
		tm_1sec = NULL;
	}


}
// ------------------------------------------------------------------- //











void setup() {

	// serial to display data
	Serial.begin(115200);
	while(!Serial) {}

	

	myMotor = ESP32MotorControl();



	// ---------- wifi & UDP Init ------------ //
	init_UDPClient();
	// --------------------------------------- //


	IMU_Init();

	// ------------- Motor Init -------------- //
	myMotor.attachMotor(MotorPinHigh, MotorPinLow);
	myMotor.motorForward(0, 0);
	// --------------------------------------- //


 	PID_Init();
	
	UDP_queue_init();
	UDP_queue_clear();
	UDP_use_queue(true);

	timer_init();
}

void loop() {

	Task_Main();
  
}



void PID_Init()
{
	accum_err 	= 0.0;
	last_err	= 0.0;
	PID_val		= 0.0;

	pid_cfg.P_Reg = 0.0;
	pid_cfg.I_Reg = 0.0;
	pid_cfg.D_Reg = 0.0;
}


void IMU_Init()
{
	// ----------- IMU-9250 Init ------------- //
	// start communication with IMU 
	IMU_status = inIMU.begin();
	if (IMU_status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("IMU Status: ");
		Serial.println(IMU_status);
		//while(1) {}
	}
	else
	{
		inIMU.setAccelRange(inIMU.ACCEL_RANGE_8G);
		inIMU.calibrateAccel();
		Serial.print("IMU Status: ");
		Serial.println(IMU_status);
	}

	// --------------------------------------- //


}



void init_UDPClient(){
  //Connect to the WiFi network
	WiFi.begin(ssid, pwd);
	Serial.println("");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	//This initializes udp_client and transfer buffer
	if(udp_client.begin(udpPort) == 1)
		Serial.println("UDP Begin OK");
	else
		Serial.println("UDP Begin Failed");



	myUDPClient.addr = udpAddress;
	myUDPClient.port = udpPort;


  
}















