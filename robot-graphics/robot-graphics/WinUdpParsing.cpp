#include "WinUdpParsing.h"

uint8_t parse_abh_htmat(WinUdpBkstServer* soc, mat4_t* m)
{
	int rc = soc->read();
	if (rc != WSAEWOULDBLOCK && soc->recv_len == 64)
	{
		int bidx = 0;
		u32_fmt_t pfmt;
		for (int r = 0; r < 4; r++)
		{
			for (int c = 0; c < 4; c++)
			{
				for (int i = 0; i < sizeof(float); i++)
				{
					pfmt.ui8[i] = soc->r_buf[bidx++];
				}
				m->m[r][c] = pfmt.f32;
			}
		}
		return 1;
	}
	return 0;
}

void parse_abh_fpos_udp_cmd(WinUdpBkstServer* soc, float* q)
{
	int rc = soc->read();
	if (rc != WSAEWOULDBLOCK && soc->recv_len == 15)
	{
		//u32_fmt_t* pfmt = (u32_fmt_t*)((uint8_t*)udp_server.r_buf);
		u32_fmt_t pfmt;
		int bidx = 2;
		for (int ch = 0; ch < 6; ch++)
		{
			for (int i = 0; i < sizeof(int16_t); i++)
			{
				pfmt.ui8[i] = soc->r_buf[bidx];
				bidx++;
			}
			q[ch] = ((float)pfmt.i16[0]) * 150.f / 32767.f;
		}
	}
}
