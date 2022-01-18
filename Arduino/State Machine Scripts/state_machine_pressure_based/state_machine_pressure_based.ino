/*
 * @file    state_machine_pressure
 * @author  CU Boulder Medtronic Team 7
 * @brief   This state machine is like Ver 2 but actually functions using the pressure sensor
 */

#include <Wire.h>
#include "Adafruit_MPRLS.h"
// Library for sensor: VEML6075

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// I/O Pins
#define BUTTON  13
#define POSITIVE_PUMP 3
#define SOLENOID  5
#define NEGATIVE_PUMP 6

// Possible States
typedef enum {
  RESET,
  INFLATE,
  HOLD,
  DEFLATE
} state;

// Init states
state currentState = RESET;
state previousState = DEFLATE;

// Last time pressure was read
unsigned long lastPressureReadTime = millis();
unsigned long currentTime = 0;
unsigned long startHoldTime = 0;
unsigned long startResetTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Init sensor
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor");
    while (1) {
      delay(10);
    }
  }
  
  pinMode(BUTTON, INPUT);
  pinMode(POSITIVE_PUMP, OUTPUT);
  pinMode(NEGATIVE_PUMP, OUTPUT);
  pinMode(SOLENOID, OUTPUT);
}

void loop() {
  primary();
}

/*
 * Function to display the pressure sensor
 */
void disp_pressure(unsigned long readDelay)
{
  unsigned long currentTime = millis();

  if ((currentTime - lastPressureReadTime) > readDelay)
  {
    // Serial.print("Pressure (PSI): "); Serial.println(mpr.readPressure() / 68.947572932);
    lastPressureReadTime = millis();
  }
}

/**
 * Primary State machine
 */
void primary() 
{
  
  switch(currentState) {
    
    case RESET:
      disp_pressure(1000);
      
      currentTime = millis();
      if(previousState != currentState)
      {
        startResetTime = currentTime;
        previousState = currentState;
      }
      
      analogWrite(SOLENOID, 0);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);
      
      if ((currentTime - startResetTime) > 5000)
      {
        Serial.print("HERE");
        currentState = INFLATE;
      }
      break;
    
    case INFLATE:
      disp_pressure(100);
      analogWrite(SOLENOID, 255);
      analogWrite(POSITIVE_PUMP, 255);
      if ((mpr.readPressure() / 68.947572932) >= 12.67) 
      {
        previousState = currentState;
        currentState = HOLD;
      }
      break;

    case HOLD:
      disp_pressure(1000);

      currentTime = millis();
      if(previousState != currentState)
      {
        startHoldTime = currentTime;
        previousState = currentState;
      }
      analogWrite(SOLENOID, 0);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);
      if ((currentTime - startHoldTime) > 5000)
      {
        currentState = DEFLATE;
      }
      break;

    case DEFLATE:
      disp_pressure(100);
      analogWrite(SOLENOID, 255);
      analogWrite(NEGATIVE_PUMP, 255);
      if ((mpr.readPressure() / 68.947572932) <= 11.63)       
      {
        previousState = currentState;
        currentState = RESET;
      }
      break;

  }
  
}
