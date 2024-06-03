#include "winserial.h"
#include <stdint.h>
#include<stdio.h>

int auto_connect_com_port(HANDLE* serial_handle, unsigned long baud)
{
	char namestr[16] = { 0 };
	uint8_t found = 0;
	for (int i = 0; i < 255; i++)
	{
		int rl = sprintf_s(namestr, "\\\\.\\COM%d", i);
		int rc = connect_to_usb_serial(serial_handle, namestr, baud);
		if (rc != 0)
		{
			printf("Connected to COM port %s successfully\n", namestr);
			found = 1;
			return found;
		}
	}
	return found;
}

int connect_to_usb_serial(HANDLE* serial_handle, const char* com_port_name, unsigned long baud)
{
	/*First, connect to com port.
	TODO: add a method that scans  this and filters based on the device descriptor.
	*/
	(*serial_handle) = CreateFileA(com_port_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	DCB serial_params = { 0 };
	serial_params.DCBlength = sizeof(serial_params);
	serial_params.BaudRate = baud;
	serial_params.ByteSize = DATABITS_8;
	serial_params.StopBits = ONESTOPBIT;
	serial_params.Parity = PARITY_NONE;

	int rc = 0;
	rc |= SetCommState((*serial_handle), &serial_params);

	if (rc != 0)
	{
		COMMTIMEOUTS timeouts = { 0 };
		timeouts.ReadIntervalTimeout = MAXDWORD;
		timeouts.ReadTotalTimeoutConstant = 0;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0;
		timeouts.WriteTotalTimeoutMultiplier = 0;
		SetCommTimeouts((*serial_handle), &timeouts);
	}
	return rc;
}


int get_ppp_pld(HANDLE * serialhandle, com_ppp_buffer_t * cb)
{
	LPDWORD num_bytes_read = 0;
	int pld_size = 0;
	int rc = ReadFile(*serialhandle, cb->ser_readbuf, PPP_BUFFER_SIZE, (LPDWORD)(&num_bytes_read), NULL);	//should be a DOUBLE BUFFER!
	for (int i = 0; i < (int)num_bytes_read; i++)
	{
		uint8_t new_byte = cb->ser_readbuf[i];
		pld_size = parse_PPP_stream(new_byte, cb->ppp_payload_buffer, PPP_BUFFER_SIZE, cb->ppp_unstuffing_buffer, PPP_UNSTUFF_BUFFER_SIZE, &cb->ppp_bidx);
		if (pld_size > 0)
		{
			return pld_size;
		}
	}
	return pld_size;
}