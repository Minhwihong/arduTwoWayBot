
#include "main.h"

volatile Task ts[ID_MAX] = {
		
	{ID_SERIAL_MSG,			T_RUN,	0, 5,	REPEAT_ENABLE,	0,	Serial_Message_Task},
	{ID_WIFI_MSG,			T_RUN,	0, 0,	REPEAT_ENABLE,	0,	WIFI_Handle_Task}, 
	{ID_UDP_MSG,			T_RUN,	0, 0,	REPEAT_ENABLE,	0,	UDP_Message_Task},
	{ID_BALANCING_MSG,		T_RUN,	0, 0,	REPEAT_ENABLE,	0,	Balancing_Proc_Task},
	{ID_IMU_MSG,			T_RUN,	0, 50,	REPEAT_ENABLE,	0,	IMU_Handle_Task}, 
	{ID_CONFIG_MSG,			T_HOLD,	0, 50,	REPEAT_DISABLE,	0,	Config_Message_Task},
	{ID_LED_TOGGLE,			T_RUN,	0, 100,	REPEAT_ENABLE,	0,	LedToggle_Task},
};


signed char PriorTask = -1;
signed char NextTask = -1;


signed char Task_Change_Order(unsigned char id)
{
	PriorTask = id;
	ts[id].curtick = ts[id].matchtick+1;
	return 0;
}



signed char Task_Main(void)
{
    unsigned char i;
	unsigned short matchtick, curtick;

    for(i=0; i<ID_MAX; i++)
    {
		//if(PriorTask != -1)
		//	i = PriorTask;
	
        switch (ts[i].runflag)
        {
	        case T_RUN:			

				if(PriorTask != -1)
				{	
					ts[PriorTask].execfunc(PriorTask, 0);
					PriorTask = -1;
				}	

				curtick 					= ts[i].curtick;
				matchtick 					= ts[i].matchtick;
				
				
	            if (matchtick < curtick)
	            {
	            	unsigned char 	id 		= ts[i].id;
					unsigned short 	param	= ts[i].param;

					
	               	ts[i].execfunc(id, param);

					
	                if (ts[i].repeat != REPEAT_ENABLE) 
					{
						ts[i].runflag 		= T_HOLD;
	                }
	                ts[i].curtick 			= 0;
	            }
	        break;
			
	        case T_SUPER:
				{
					unsigned char 	id 		= ts[i].id;
					unsigned short 	param 	= ts[i].param;
					
	            	ts[i].execfunc(id, param);
	        	}
	        break;
        }

		
		
    }

    if (i == ID_MAX) 
	{
		return T_ERR_TASK;
    }
	
    return T_GOOD_TASK;
}

void Task_Timecount(void)
{
    unsigned char i;

    for(i=0; i<ID_MAX; i++)
    {
        if (ts[i].runflag == T_RUN)
        {
            ts[i].curtick++;
        }
    }
}

bool Task_Run_State(unsigned char id)
{
	unsigned char i;
	
	for(i=0; i<ID_MAX; i++)
    {
        if (ts[i].id == id)
        {
            if (ts[i].runflag == T_RUN) 
			{
				return true;
            }
			else
			{
				return false;
			}
        }
    }

	return false;
}

signed char Task_Restart(unsigned char id, unsigned char repeat, unsigned short pm)
{
    unsigned char i;

    for(i=0; i<ID_MAX; i++)
    {
        if (ts[i].id == id)
        {
            if (ts[i].runflag == T_RUN) return T_RUNNING_TASK;

            ts[i].runflag 					= T_RUN;
            ts[i].curtick 					= 0;
            ts[i].repeat 					= repeat;
            ts[i].param 					= pm;

            break;
        }
    }

    if (i == ID_MAX) 
	{
		return T_ERR_TASK;
    }

    return T_RESTART_TASK;
}







signed char Task_Newstart(unsigned char id, unsigned short mt,
                        /*  unsigned char ltf, unsigned short mlt,  */
                            unsigned char repeat, unsigned short pm,
                            signed char (* func)(unsigned char id, unsigned short pm))
{
    unsigned char i;

    for(i=0; i<ID_MAX; i++)
    {
        if (ts[i].id == id)
        {
            ts[i].runflag 					= T_RUN;
            ts[i].curtick 					= 0;
            ts[i].matchtick 				= mt;
            ts[i].repeat 					= repeat;
            ts[i].param 					= pm;
            ts[i].execfunc 					= func;

            break;
        }
    }

    if (i == ID_MAX) 
	{
		return T_ERR_TASK;
    }
	
    return T_NEW_TASK;
}

signed char Task_Stop(unsigned char id)
{
    unsigned char i;

    for(i=0; i<ID_MAX; i++)
    {
        if (ts[i].id == id)
        {
            ts[i].runflag 					= T_HOLD;
            ts[i].curtick 					= 0;

            break;
        }
    }

    if (i == ID_MAX) 
	{
		return T_ERR_TASK;
    }
	
    return T_STOP_TASK;
}

signed char Task_Stop_All(void)
{
    unsigned char i;

    for(i=0; i<ID_MAX; i++)
    {
        ts[i].runflag 						= T_HOLD;
        ts[i].curtick 						= 0;
    }
	
    return T_STOP_TASK;
}





signed char Task_Tick_Change(unsigned char id, unsigned short tick)
{
    unsigned char i;

    for(i=0; i<ID_MAX; i++)
    {
        if (ts[i].id == id)
        {
//            if (ts[i].runflag == T_RUN) return T_RUNNING_TASK;

            ts[i].runflag 					= T_RUN;
            ts[i].matchtick					= tick;

            break;
        }
    }

    if (i == ID_MAX) 
	{
		return T_ERR_TASK;
    }

    return T_RESTART_TASK;
}


