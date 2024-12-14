#ifndef SERIAL_THREAD_H
#define SERIAL_THREAD_H
#include <stdint.h>

extern uint8_t gl_ser_pkt_done;
extern float gl_arm_angles[6];

int serial_thread(void);

#endif
