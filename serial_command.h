#ifndef SERIAL_COMMAND_H_
#define SERIAL_COMMAND_H_

#include "udoo_motor_control.h"

char input;
char motor_driver_output;

//ISR for the serial port
void serialEvent();

//ISR for the serial1 port
void serialEvent1();

#endif
