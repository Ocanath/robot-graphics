#ifndef USB_COM_HANDLE_H
#define USB_COM_HANDLE_H
#include <iostream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

extern volatile float q_share[];
extern volatile float tau_share[];

extern uint8_t exit_signal;

void usb_COM_handle_thread();

#endif