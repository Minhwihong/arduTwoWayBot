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


////double roll  = atan2(accY, accZ) * RAD_TO_DEG;

ESP32MotorControl myMotor;
MPU9250 IMU(Wire,0x68);

IMU_6axis_t valMPU9255;
int status;



int MotorPinHigh  = 26;
int MotorPinLow   = 25;
int motorSpd      = 0;
byte mSpd = 0;
byte dir = 0;





// ---------------------------- Setup Timer -------------------------- //

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;



void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
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
	timer = timerBegin(0, 80, true);
	
	// Attach onTimer function to our timer.
	timerAttachInterrupt(timer, &onTimer, true);
	
	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter)
	timerAlarmWrite(timer, 1000000, true);
	
	// Start an alarm
	timerAlarmEnable(timer);


}


void timer_stop()
{
	// If timer is still running
	if (timer) {
		// Stop and free timer
		timerEnd(timer);
		timer = NULL;
	}


}
// ------------------------------------------------------------------- //











void setup() {

	// serial to display data
	Serial.begin(115200);
	while(!Serial) {}

	ESP32MotorControl myMotor = ESP32MotorControl();


	IMU_Init();


	// ---------- wifi & UDP Init ------------ //
	init_UDPClient();
	// --------------------------------------- //

	// ------------- Motor Init -------------- //
	myMotor.attachMotor(MotorPinHigh, MotorPinLow);
	myMotor.motorForward(0, 0);
	// --------------------------------------- //


 	PID_Init();
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
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		//while(1) {}
	}
	else
	{
		IMU.setAccelRange(IMU.ACCEL_RANGE_8G);
		IMU.calibrateAccel();
		Serial.print("Status: ");
		Serial.println(status);
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

	UDP_RcvSts.rcvReq_IMU = false;
	UDP_RcvSts.rcvReq_PID = false;
  
}















