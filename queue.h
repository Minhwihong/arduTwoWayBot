#ifndef __QUEUE__
#define __QUEUE__


#define MAX_QUEUE 8

typedef struct UDP_packet {
	uint8_t	frmId;
	uint8_t frmIdx;
	uint8_t pktId;
	uint8_t pktCmd;
	uint8_t pktLen;
	uint8_t data[256];
	uint8_t checksum;
	bool    pktOK;
}UDP_pkt_t;

typedef struct tag_UDP_queue
{
	uint8_t			front;
	uint8_t			rear;
	uint8_t			start;
	uint8_t			use_single;
	UDP_pkt_t 		buf[MAX_QUEUE];
}UDP_queue_t;



extern UDP_queue_t	UDP_queue;


void UDP_queue_init(void);
void UDP_queue_clear(void);
void UDP_use_queue(bool onoff);
int8_t UDP_queue_push_back(UDP_pkt_t* inPtr);
UDP_pkt_t* UDP_queue_get_data(uint8_t* valid_pkt);


#endif