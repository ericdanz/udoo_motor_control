#ifndef UDOO_MOTOR_CONTROL_H_
#define UDOO_MOTOR_CONTROL_H_

#include "PID_v1.h"

#define RX_PIN 158 // Yellow, goes to TX on Udoo
#define TX_PIN 159 // Orange, goes to RX on Udoo
#define WHEEL_DIAMETER 100 // 100mm
#define SERIAL_BUFFER_LEN 32

double PID_input = 128;
double PID_output = 128;
double PID_target = 128;
double Kp = 0.1;
double Ki = 2.0;
double Kd = 0.01;
PID speedPID(&PID_input, &PID_output, &PID_target, Kp, Ki, Kd, DIRECT);


// Contains the serial output
char serial1_buffer[SERIAL_BUFFER_LEN];
int serial1_buffer_position = 0;

#endif
