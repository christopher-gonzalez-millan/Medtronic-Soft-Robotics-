/* This is a test script for communicating between Python and Arduino
   The code follow this website https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0
   This is the Arduino side of the code
*/

#include <Wire.h>               // library for I2C connections

int x;

void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);
}

void loop() {
 while (!Serial.available());
 x = Serial.readString().toInt();
 Serial.print(x + 1);
}