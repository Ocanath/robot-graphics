#ifndef WINSERIAL_H
#define WINSERIAL_H

#include <Windows.h>
#include "PPP.h"

#define PPP_BUFFER_SIZE 512
#define PPP_UNSTUFF_BUFFER_SIZE (PPP_BUFFER_SIZE * 2 + 2)
typedef struct com_ppp_buffer_t
{
	uint8_t ser_readbuf[PPP_BUFFER_SIZE];
	uint8_t ppp_payload_buffer[PPP_BUFFER_SIZE];	//buffer
	uint8_t ppp_unstuffing_buffer[PPP_UNSTUFF_BUFFER_SIZE];
	int ppp_bidx;
}com_ppp_buffer_t;


int auto_connect_com_port(HANDLE* serial_handle, unsigned long baud);
int connect_to_usb_serial(HANDLE* serial_handle, const char* com_port_name, unsigned long baud);
int get_ppp_pld(HANDLE * serialhandle, com_ppp_buffer_t* cb);

#endif