#include "serial_command.h"

//void serialEvent() {
//  while (Serial.available() > 0) {
//    input = Serial.parseInt();
//    delay(10);
//    PID_target = input;
//    delay(10);
//  }
//}

void serialEvent1() {
  if (Serial1.available() > 0) {
    motor_driver_output = Serial1.read();
    Serial.print("Receive: ");
    Serial.println(motor_driver_output);
    if (serial1_buffer_position = SERIAL_BUFFER_LEN) {
      serial1_buffer_position = 0;
    } else {
      serial1_buffer_position++;
    }
    serial1_buffer[serial1_buffer_position] = motor_driver_output;
  }
}

int readSerial1() {
  if (Serial1.available() > 0) {
    return Serial1.read();
    delay(10);
  } else {
    return 0;
  }
}
