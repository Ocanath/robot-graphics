
//Using SDL and standard IO
#include <stdio.h>
#include <math.h>
#include <vector>
#include "winserial.h"
#include "PPP.h"
#include "args-parsing.h"
#include "sin_fast.h"
#include <algorithm>
#include "magsensor.h"

#define PAYLOAD_SIZE 512
#define UNSTUFFING_BUFFER_SIZE (PAYLOAD_SIZE * 2 + 2)

//Screen dimension constants
const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 800;

typedef struct fpoint_t
{
	float x;
	float y;
}fpoint_t;

static int gl_ppp_bidx = 0;
static uint8_t gl_ppp_payload_buffer[PAYLOAD_SIZE] = { 0 };	//buffer
static uint8_t gl_ppp_unstuffing_buffer[UNSTUFFING_BUFFER_SIZE] = { 0 };
static uint8_t gl_ser_readbuf[512] = { 0 };
static float gl_valdump[PAYLOAD_SIZE / sizeof(float)] = { 0 };

uint8_t gl_ser_pkt_done = 0;
float gl_arm_angles[6] = { 0 };

float gl_magsensor_xyz[3] = {};

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint8_t get_checksum(uint8_t* arr, int size)
{

	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
 */
uint16_t get_checksum16(uint16_t* arr, int size)
{
	int16_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int16_t)arr[i];
	return -checksum;
}


static const float offsets[] = { -2.700014, -1.099270, -1.576392, 1.654050, -2.082925, -0.178326 };
static const float signs[] = { -1,-1,1,-1,-1,-1 };


/*
* Inputs:
*	input_buf: raw unstuffed data buffer
* Outputs:
*	parsed_data: floats, parsed from input buffer
* Returns: number of parsed values
*/
void parse_magsensor_response(uint8_t* input_buf, int payload_size, float* parsed_data, int parsed_data_array_size, int* parsed_data_size)
{
	uint32_t* pbu32 = (uint32_t*)(&input_buf[0]);
	int32_t* pbi32 = (int32_t*)(&input_buf[0]);
	int wordsize = payload_size / sizeof(uint32_t);
	if (wordsize != 5)
		return;
	if (wordsize > parsed_data_array_size)
		return;	//array bounds safety
	int i = 0;
	for (i = 0; i < wordsize - 1; i++)
	{
		parsed_data[i] = ((float)pbi32[i]);
		//printf("%d ", pbi32[i]);
	}
	//printf("\n");
	parsed_data[i] = ((float)pbu32[i]) / 1000.f;

	*parsed_data_size = wordsize;
}

/*
* Inputs:
*	input_buf: raw unstuffed data buffer
* Outputs:
*	parsed_data: floats, parsed from input buffer
* Returns: number of parsed values
*/
void parse_read(uint8_t* input_buf, int input_size, float* parsed_data, int parsed_size)
{
	uint16_t* pbuf16 = (uint16_t*)input_buf;

	uint16_t chk = get_checksum16(pbuf16, 3);
	if (chk == pbuf16[3])
	{
		//printf("Address:%d, cos:%d, sin:%d\n", pbuf16[0], pbuf16[1], pbuf16[2]);
		uint16_t address = pbuf16[0] - 1;	//start everything at 1
		if (address >= 0 && address < parsed_size)
		{
			if (address < (sizeof(offsets) / sizeof(float)) && address < (sizeof(signs) / sizeof(float)))	//bounds check on signs and offsets arrays
			{
				double sin = (double)pbuf16[2] - 1995.;
				double cos = (double)pbuf16[1] - 1995.;
				float angle = wrap_2pi((float)atan2(sin, cos) - offsets[address]) * signs[address];
				parsed_data[address] = angle;
			}
		}

	}
	//else
	//{
	//	printf("Checksum Mismatch\n");
	//}
}


void write_encoder_command(HANDLE* pSer, uint16_t address)
{
	uint8_t stuff_buf[sizeof(address) * 2 + 2] = { 0 };

	int nb = PPP_stuff((uint8_t*)(&address), sizeof(address), stuff_buf, sizeof(stuff_buf));
	LPDWORD written = 0;
	int wfrc = WriteFile(*pSer, stuff_buf, nb, written, NULL);
}

void delay(uint32_t ms)
{
	uint32_t start = GetTickCount();
	while ((GetTickCount() - start) < ms);
}

uint8_t gl_subtraction_enable_flag = 0;
uint8_t gl_subtraction_disable_flag = 0;

void main_loop(HANDLE* pSer)
{
	int pld_size = 0;
	int previous_wordsize = 0;
	int wordsize = 0;
	int wordsize_match_count = 0;

	gl_serialwrite_serialport = *pSer;

	mlx_write(0x77, MT_RESET);
	delay(100);
	mlx_write(0x77, MT_EXIT_MODE);
	delay(100);
	mlx_write(0x77, MT_RESET);
	printf("Sensor Reset Complete\r\n");

	uint8_t gain = 0 & 0x7;
	mlx_write_register(0x77, 0x0, (gain << 4));
	delay(100);
	printf("Write Gain Complete\r\n");
	
	uint8_t res = 3;
	uint16_t regw = (res << 5) | (res << 7) | (res << 9);
	uint8_t dig_filt = 2;	//datashit says that dig_filt = 0 and OSR=0, chip won't work right so have to select non-default settings!?!?!?!!?!?!?!?!?
	regw |= (dig_filt & 0b111) << 2;
	mlx_write_register(0x77, 0x2, regw);	//set res. works
	delay(100);
	printf("Write Res Complete\r\n");


	//uint16_t addresses[] = { 1,2,3,4 ,5, 6, MAGSENSOR_RS485ADDRESS };
	//uint16_t addresses[] = { 1,2,3,4 ,5, 6 };
	uint16_t addresses[] = { MAGSENSOR_RS485ADDRESS };
	int skipcount[sizeof(addresses) / sizeof(uint16_t)] = {};

	int num_addresses = (sizeof(addresses) / sizeof(uint16_t));
	float angles[(sizeof(addresses) / sizeof(uint16_t))] = {  };
	int addr_idx = 0;
	uint64_t tx_ts = 0;
	uint8_t done = 0;
	while (1)
	{
		if (addresses[addr_idx] != MAGSENSOR_RS485ADDRESS)
		{
			write_encoder_command(pSer, addresses[addr_idx]);
		}
		else
		{
			mlx_write(MAGSENSOR_RS485ADDRESS, MT_READ_XYZ);
		}


		if (gl_subtraction_disable_flag)
		{
			delay(100);
			gl_subtraction_disable_flag = 0;
			mlx_write(MAGSENSOR_RS485ADDRESS, MT_DISABLE_SUBTRACTION);
			printf("sent disable command\r\n");
			delay(100);
		}
		if (gl_subtraction_enable_flag)
		{
			delay(100);
			gl_subtraction_enable_flag = 0;
			mlx_write(MAGSENSOR_RS485ADDRESS, MT_ENABLE_SUBTRACTION);
			printf("sent enable command\r\n");
			delay(100);
		}


		uint8_t poll_for_response = 1;
		uint64_t start_ts = GetTickCount64();
		float raw_magsense[5] = {};	//xyzt, ms
		while (poll_for_response != 0)
		{
			uint64_t tick = GetTickCount64();
			LPDWORD num_bytes_read = 0;
			pld_size = 0;
			int rc = ReadFile(*pSer, gl_ser_readbuf, 512, (LPDWORD)(&num_bytes_read), NULL);	//should be a DOUBLE BUFFER!
			for (int i = 0; i < (int)num_bytes_read; i++)
			{
				uint8_t new_byte = gl_ser_readbuf[i];
				pld_size = parse_PPP_stream(new_byte, gl_ppp_payload_buffer, PAYLOAD_SIZE, gl_ppp_unstuffing_buffer, UNSTUFFING_BUFFER_SIZE, &gl_ppp_bidx);
				if (pld_size > 0)
				{
					if (addresses[addr_idx] != MAGSENSOR_RS485ADDRESS)
					{
						parse_read(gl_ppp_payload_buffer, pld_size, angles, num_addresses);
					}
					else
					{
						parse_magsensor_response(gl_ppp_payload_buffer, pld_size, raw_magsense, sizeof(raw_magsense)/sizeof(float), &wordsize);
						gl_magsensor_xyz[0] = raw_magsense[0] * gain_res_xy[gain][res];
						gl_magsensor_xyz[1] = raw_magsense[1] * gain_res_xy[gain][res];
						gl_magsensor_xyz[2] = raw_magsense[2] * gain_res_z[gain][res];
					}
					poll_for_response = 0;

					addr_idx = (addr_idx + 1);
					if (addr_idx >= num_addresses)
					{
						addr_idx = 0;
						done = 1;
					}	
				}
			}
			uint64_t timeout = 1;
			if (addresses[addr_idx] == MAGSENSOR_RS485ADDRESS)
				timeout = 20;

			if (tick - start_ts > timeout)
			{
				skipcount[addr_idx]++;
				poll_for_response = 0;
				addr_idx = (addr_idx + 1);
				if (addr_idx >= num_addresses)
				{
					addr_idx = 0;
					done = 1;
				}

			}
		}
		uint32_t delstart = GetTickCount();
		while  ((GetTickCount() - delstart) <= 15);


		if (done != 0)
		{
			if (gl_ser_pkt_done == 0)
			{
				for (int i = 0; i < 6 && i < num_addresses; i++)
				{
					gl_arm_angles[i] = angles[i];
					//printf("%f\n", angles[5]);
					printf("%f uT\n", gl_magsensor_xyz[2]);
				}
				gl_ser_pkt_done = 1;
			}
			//for (int i = 0; i < num_addresses; i++)
			//{
			//	printf("%.2f, ", angles[i] * 180. / 3.14159265);
			//}
			//printf("\n");
			done = 0;
		}
	}
}


int serial_thread(void)
{
	HANDLE serialport;
	char namestr[16] = { 0 };
	uint8_t found = 0;
	for (int i = 0; i < 255; i++)
	{
		int rl = sprintf_s(namestr, "\\\\.\\COM%d", i);
		int rc = connect_to_usb_serial(&serialport, namestr, gl_options.baud_rate);
		if (rc != 0)
		{
			if (!(gl_options.csv_header == 1 && (gl_options.print_only == 1 || gl_options.print_in_parser == 1)))
			{
				printf("Connected to COM port %s successfully\n", namestr);
			}
			found = 1;
			break;
		}
	}
	if (found == 0)
	{
		if (gl_options.csv_header == 0)
			printf("No COM ports found\n");
	}

	main_loop(&serialport);

	//close serial port
	CloseHandle(serialport);
	return 0;
}
