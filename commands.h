#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "udoo_motor_control.h"

#define LOOP_MASK 0x01
#define BIT_DELAY 100


/** Byte commands determined from data sheet
http://www.robot-electronics.co.uk/htm/md25ser.htm
*/
#define SYNC (byte)0x00
#define GET_SPEED_1 0x21
#define GET_SPEED_2 0x22
#define GET_ENCODER_1 0x23
#define GET_ENCODER_2 0x23
#define GET_ENCODERS 0x25
#define GET_VOLTS 0x26
#define GET_CURRENT_1 0x27
#define GET_CURRENT_2 0x28
#define GET_VERSION 0x29
#define GET_ACCELERATION 0x2A
#define GET_MODE 0x2B
#define GET_VI 0x2C
#define SET_SPEED_1 0x31
#define SET_SPEED_2 0x32
#define TURN 0x32
#define SET_ACCELERATION 0x33
#define SET_MODE 0x34
#define RESET_ENCODERS 0x35
#define DISABLE_REGULATORS 0x36
#define ENABLE_REGULATORS 0x37
#define DISABLE_TIMEOUT 0x38
#define ENABLE_TIMEOUT 0x39

// Define the modes that determine what the set speed and turn commands do
#define INDIVIDUAL_MOTOR_CONTROL 0
#define COMBINED_MOTOR_CONTROL 2

void sendCommand(char command, char commandArgument);
void setMode(int mode);
int getSpeed();
void setSpeedBoth();

int currentMode;

#endif

