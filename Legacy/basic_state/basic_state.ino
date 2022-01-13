/**
 * @file  basic_state.c
 * @brief proof of concept for state machine
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
state previousState = RESET;

// Last time pressure was read
unsigned long lastPressureReadTime = millis();

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
    Serial.print("Pressure (PSI): "); Serial.println(mpr.readPressure() / 68.947572932);
    lastPressureReadTime = millis();
  }
}

/**
 * Primary State machine
 */
void primary() 
{
  previousState = currentState;
  
  switch(currentState) {
    
    case RESET:
      disp_pressure(1000);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);
      if (digitalRead(BUTTON) == LOW) 
      {
        currentState = INFLATE;
      }
      break;
    
    case INFLATE:
      disp_pressure(100);
      analogWrite(POSITIVE_PUMP, 255);
      if (digitalRead(BUTTON) == HIGH) 
      {
        currentState = HOLD;
      }
      break;

    case HOLD:
      disp_pressure(1000);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);
      if (digitalRead(BUTTON) == LOW) 
      {
        currentState = DEFLATE;
      }
      break;

    case DEFLATE:
      disp_pressure(100);
      analogWrite(NEGATIVE_PUMP, 255);
      if (digitalRead(BUTTON) == HIGH) 
      {
        currentState = RESET;
      }
      break;
      
  }
  
}
