#ifndef  WIN_UDP_PARSING_H
#define WIN_UDP_PARSING_H
#include "WinUdpBkstServer.h"
#include "spatialAlgebra.h"
#include "u32_fmt_t.h"


uint8_t parse_abh_htmat(WinUdpBkstServer* soc, mat4_t* m);
void parse_abh_fpos_udp_cmd(WinUdpBkstServer* soc, float* q);



#endif // ! WIN_UDP_PARSING_H

