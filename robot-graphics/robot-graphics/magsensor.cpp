#include "PPP.h"
#include "winserial.h"
#include "magsensor.h"



uint8_t pld[32] = {};
uint8_t stuff_pld[sizeof(pld) * 2 + 2] = {};

double gain_res_xy[8][4] = {
	{0.751, 1.502, 3.004, 6.009},
	{0.601, 1.202, 2.403, 4.840},
	{0.451, 0.901, 1.803, 3.605},
	{0.376, 0.751, 1.502, 3.004},
	{0.300, 0.601, 1.202, 2.403},
	{0.250, 0.501, 1.001, 2.003},
	{0.200, 0.401, 0.801, 1.602},
	{0.150, 0.300, 0.601, 1.202}
};

double gain_res_z[8][4] = {
	{1.210, 2.420, 4.840, 9.680},
	{0.968, 1.936, 3.872, 7.744},
	{0.726, 1.452, 2.904, 5.808},
	{0.605, 1.210, 2.420, 4.840},
	{0.484, 0.968, 1.936, 3.872},
	{0.403, 0.807, 1.613, 3.227},
	{0.323, 0.645, 1.291, 2.581},
	{0.242, 0.484, 0.968, 1.936}
};


void mlx_write(uint16_t address, uint8_t mstgtype)
{
	uint16_t* pbu16 = (uint16_t*)(&pld[0]);
	pld[0] = address;
	pld[2] = mstgtype;

	int num_bytes = PPP_stuff(pld, sizeof(pld), stuff_pld, sizeof(stuff_pld));
	serial_write(stuff_pld, num_bytes);
}

void mlx_write_register(uint16_t rs485addr, uint8_t regaddr, uint16_t regval)
{
	uint16_t* pbu16 = (uint16_t*)(&pld[0]);
	pld[0] = rs485addr;
	pld[2] = MT_WRITE_REGISTER;
	pld[3] = regaddr;
	uint16_t* pregv = (uint16_t*)(&pld[4]);
	pregv[0] = regval;


	int num_bytes = PPP_stuff(pld, sizeof(pld), stuff_pld, sizeof(stuff_pld));
	serial_write(stuff_pld, num_bytes);

}
