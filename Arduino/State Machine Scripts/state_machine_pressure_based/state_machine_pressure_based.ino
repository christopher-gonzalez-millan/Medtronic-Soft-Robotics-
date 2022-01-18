/*
 * @file    state_machine_pressure
 * @author  CU Boulder Medtronic Team 7
 * @brief   This state machine controls two pumps and two valves through code, 
 *          but there is no implementation of the pressure sensor yet
 */

#include <Wire.h>
#include "Adafruit_MPRLS.h"
// Library for sensor: VEML6075

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Initialize the pressure reading variables
unsigned long lastPressureReadTime = millis();
unsigned long currentTime;

void setup() {
  // Pressure Sensor setup
  Serial.begin(115200);
  
  // Init sensor
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor");
    while (1) {
      delay(10);
    }
  }

  // Setup pins for 2 pumps and 2 valves
  pinMode(3, OUTPUT); // inflation pump
  pinMode(5, OUTPUT); // solenoid for pump 3
  pinMode(6, OUTPUT); // deflation pump
  pinMode(9, OUTPUT); // solenoid for pump 6

  // Set the initial state of the pumps and valves to be closed
  analogWrite(3, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(9, 0);
}


void loop() {
  // Inflate the robot
  disp_pressure(100);
  analogWrite(3, 255);
  analogWrite(5, 255);
  delay(400);

  // Hold the inflation0
  disp_pressure(100);
  analogWrite(5, 0);
  analogWrite(3, 0);
  delay(3000);
  
  // Deflate the robot
  disp_pressure(100);
  analogWrite(6, 255);
  analogWrite(9, 255);
  delay(400);

  // Hold the deflation
  disp_pressure(100);
  analogWrite(9, 0);
  analogWrite(6, 0);
  delay(3000);
}

// function to display the pressure
void disp_pressure(unsigned long readDelay) {
  unsigned long currentTime = millis();

  if ((currentTime - lastPressureReadTime) > readDelay)
  {
    Serial.print("Pressure (PSI): "); Serial.println(mpr.readPressure() / 68.947572932);
    lastPressureReadTime = millis();
  }
}
