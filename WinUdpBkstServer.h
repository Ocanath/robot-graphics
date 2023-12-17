#ifndef WINUDP_SERVER_H
#define WINUDP_SERVER_H

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

#include<stdio.h>
#include<winsock2.h>
#include <WS2tcpip.h>
#include <stdint.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 512

/*Dead simple blocking broadcast-channel udp server*/
class WinUdpBkstServer
{
public:
	uint16_t port;
	int recv_len;
	uint8_t r_buf[BUFLEN];
	uint8_t t_buf[BUFLEN];
	SOCKET s;
	struct sockaddr_in server, si_other;
	int slen;


	WinUdpBkstServer(uint16_t arg_port)
	{
		port = arg_port;
		slen = sizeof(si_other);

		//Initialise winsock
		printf("\nInitialising Winsock...");
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		{
			printf("Failed. Error Code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}
		printf("Initialised.\n");

		//Create a socket
		if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
		{
			printf("Could not create socket : %d", WSAGetLastError());
		}
		printf("Socket created.\n");

		//Prepare the sockaddr_in structure
		server.sin_family = AF_INET;
		server.sin_addr.s_addr = inet_addr("0.0.0.0");
		//server.sin_addr.s_addr = inet_addr(inet_addr_buf);	//assign the server with one of the IPV4 addresses (the last one) in the host address list.
		//server.sin_addr = in4addr_any;	//assign the desired port # to the special address 0.0.0.0, which is a 'meta address' used to specify that the server can have any IP address we like on the LAN!
		server.sin_port = htons(port);

		char inet_addr_buf[256] = { 0 };	/*Buffer to be used for displaying string-format IP addresses*/
		inet_ntop(AF_INET, &server.sin_addr.s_addr, (PSTR)inet_addr_buf, 256);	//convert again the value we copied thru and display
		printf("Binding to server address: %s with port %d\r\n", inet_addr_buf, port);

		//Bind
		if (bind(s, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
		{
			printf("Bind failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}
		puts("Bind done");
		is_blocking = 0;
	}
	~WinUdpBkstServer()
	{
	}

	int read(void)
	{
		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(s, (char *)r_buf, BUFLEN, 0, (struct sockaddr*)&si_other, &slen)) == SOCKET_ERROR)
		{
			return WSAGetLastError();
			//exit(EXIT_FAILURE);
		}
		else
		{
			return 0;
		}
	}

	int set_nonblocking(void)
	{
		//set to blocking and clear
		u_long iMode = 1;
		return ioctlsocket(s, FIONBIO, &iMode);
	}

private:
	WSADATA wsa;
	uint8_t is_blocking;
};


#endif

