#ifndef __TASK_OP__
#define __TASK_OP__




/***************************************************************/
// UDP cmd

#define FRM_NORMAL	'N'
#define FRM_DEBUG	'D'
#define FRM_UPDATE	'U'

#define FRM_NORMAL_IDX	0

#define PKT_ID_NORMAL	0xAB





/****************************************************************/





signed char  		Serial_Message_Task(uint8_t id, uint16_t param);
signed char  		WIFI_Handle_Task(uint8_t id, uint16_t param);
signed char  		UDP_Message_Task(uint8_t id, uint16_t param);
signed char  		Balacing_Msg_Task(uint8_t id, uint16_t param);
signed char  		Balancing_Task(uint8_t id, uint16_t param);
signed char  		Config_Message_Task(uint8_t id, uint16_t param);
signed char 		LedToggle_Task(uint8_t id, uint16_t param);
signed char 	Task_UDP_Rcv_Handler(uint8_t id, uint16_t param);


void UDP_rcvHandler();



#endif


