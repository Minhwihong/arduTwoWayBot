#ifndef __TASK_OP__
#define __TASK_OP__







signed char  		Serial_Message_Task(uint8_t id, uint16_t param);
signed char  		WIFI_Handle_Task(uint8_t id, uint16_t param);
signed char  		UDP_Message_Task(uint8_t id, uint16_t param);
signed char  		Balancing_Proc_Task(uint8_t id, uint16_t param);
signed char  		IMU_Handle_Task(uint8_t id, uint16_t param);
signed char  		Config_Message_Task(uint8_t id, uint16_t param);
signed char 		LedToggle_Task(uint8_t id, uint16_t param);




#endif


