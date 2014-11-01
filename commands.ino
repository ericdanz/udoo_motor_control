#include "commands.h"

void sendCommand(char command, char commandArgument = 0) {
  // Write SYNC command
  Serial1.write((byte)SYNC);
  delay(10);
  Serial1.write(command);
  delay(10);
  if (commandArgument) {
    Serial1.write(commandArgument);
  }
}

void setMode(int mode) {
  currentMode = mode;
  sendCommand(SET_MODE, currentMode);
}

int getSpeed(int motor) {
  if (motor == 1) {
    sendCommand(GET_SPEED_1);
  } else {
    sendCommand(GET_SPEED_2);
  }
  return convertSpeed(readSerial1());
}

void setSpeed1(int spd) {
  sendCommand(SET_SPEED_1, spd);
}

void setSpeed2(int spd) {
  sendCommand(SET_SPEED_2, spd);
}

void setSpeedBoth(int spd) {
  if (currentMode != COMBINED_MOTOR_CONTROL) {
    setMode(COMBINED_MOTOR_CONTROL);
  }
  // Map output to new range
  spd = convertSpeed(spd);
  sendCommand(SET_SPEED_1, spd);
}

// Return speed converted to PID limits
int convertSpeed(int input) {
  return map(input, PID_output_lower, PID_output_upper, 0, 255);
}
