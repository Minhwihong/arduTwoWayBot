#ifndef __TASK_SETTING__
#define __TASK_SETTING__




enum {
	ID_SERIAL_MSG,			
	ID_WIFI_MSG,
	ID_UDP_MSG,
	ID_BALANCING_MSG,
	ID_BALANCING,
	ID_CONFIG_MSG,
	ID_LED_TOGGLE,
	ID_UDP_RCV_HANDLE,
	ID_MAX,
};

enum {
	T_RUN = 1,
    T_HOLD,
    T_DEL,
    T_SUPER
};

enum {
    REPEAT_ENABLE = 1,
    REPEAT_DISABLE
};

enum {
    T_ERR_TASK = -1,
    T_BLANK,
    T_RESTART_TASK,
    T_NEW_TASK,
    T_STOP_TASK,
    T_RUNNING_TASK,
    T_GOOD_TASK
};


typedef struct _tagTask {
    unsigned char 	id;
    unsigned char 	runflag;
    unsigned short 	curtick;
    unsigned short 	matchtick;
    unsigned char 	repeat;
    unsigned short 	param;
    signed char 	(* execfunc)(unsigned char id, unsigned short pm);
} Task;




signed char Task_Main(void);
void 		Task_Timecount(void);
bool 		Task_Run_State(unsigned char id);
signed char Task_Restart(unsigned char id, unsigned char repeat, unsigned short pm);
signed char Task_Newstart(unsigned char id, unsigned short mt,
                        /*  unsigned char ltf, unsigned short mlt,  */
                            unsigned char repeat, unsigned short pm,
                            signed char (* func)(unsigned char id, unsigned short pm));
signed char Task_Stop(unsigned char id);
signed char Task_Stop_All(void);
signed char Task_Tick_Change(unsigned char id, unsigned short tick);
signed char Task_Change_Order(unsigned char id);










#endif
