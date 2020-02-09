#include "main.h"


UDP_queue_t	UDP_queue;





void UDP_queue_init(void)
{
    //front = rear = 0;
	UDP_queue.front = UDP_queue.rear = 0;
	UDP_queue.start = 1;
	//PRINT_DBG("[Queue] Queue init\r\n");
}

void UDP_queue_clear(void)
{
    //front = rear;
	UDP_queue.front 		= UDP_queue.rear;
	UDP_queue.start 		= 0;
	UDP_queue.use_single 	= 0;
	//PRINT_DBG("[Queue] Queue clear\r\n");
}

void UDP_use_queue(bool onoff)
{
	if(onoff) 	UDP_queue.use_single = 0;
	else 		UDP_queue.use_single = 1;

	//PRINT_DBG("[Queue] Use Queue?: %s\r\n",UDP_queue.use_single == 1 ? "not use" : "use");

	UDP_queue.front = UDP_queue.rear = 0;
	UDP_queue.start = 1;
}


int8_t UDP_queue_push_back(UDP_pkt_t* inPtr)
{
	if(UDP_queue.start == 0)	return 0;

	if(UDP_queue.use_single == 1)
	{
		UDP_queue.buf[0] = *inPtr;
	}
	else
	{
		if ((UDP_queue.rear + 1) % MAX_QUEUE == UDP_queue.front)
		{
	       	//PRINT_DBG("Queue overflow\r\n");
	        return -1;
	    }

		UDP_queue.buf[UDP_queue.rear++] = *inPtr;
    	UDP_queue.rear = UDP_queue.rear % MAX_QUEUE;
	}

    return 1;
}


UDP_pkt_t* UDP_queue_get_data(uint8_t* valid_pkt)
{
	UDP_pkt_t* ret = NULL;


	if(UDP_queue.start == 0)	return ret;

	if(UDP_queue.use_single == 1)
	{
		ret 			= &UDP_queue.buf[0];
	}
	else
	{
			
	    if (UDP_queue.front == UDP_queue.rear)
		{
			*valid_pkt 	= 0;
			//PRINT_DBG("Queue underflow\r\n");
	        return ret;
	    }

	    ret 			= &UDP_queue.buf[UDP_queue.front];
		*valid_pkt 		= abs(UDP_queue.front-UDP_queue.rear);
		UDP_queue.front++;
	    UDP_queue.front = UDP_queue.front % MAX_QUEUE;
	}
	
    return ret;
}
