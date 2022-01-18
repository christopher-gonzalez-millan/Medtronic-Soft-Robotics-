/*
 * @file    state_machine_pressure
 * @author  CU Boulder Medtronic Team 7
 * @brief   This version attempts to integrate a button to control the start and stop states
 *          of the robot. It is currently not working in this version
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
  START,
  INFLATE,
  HOLD,
  DEFLATE,
  STOP
} state;

bool enter_INFLATE = true;
bool enter_DEFLATE = true;

// Init states
state currentState = STOP;
state previousState = START;

// Last time pressure was read
unsigned long lastPressureReadTime = millis();
unsigned long currentTime = 0;
unsigned long startHoldTime = 0;
unsigned long startTime = 0;

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
    Serial.print("Pressure (PSI): "); Serial.println(mpr.readPressure() / 68.947572932);
    lastPressureReadTime = millis();
  }
}

bool is_button_pressed(void)
{
  return (digitalRead(BUTTON) == LOW);
}

/**
 * Primary State machine
 */
void primary() 
{
  switch(currentState) {
    
    case START:
      disp_pressure(10);
      
      currentTime = millis();
      if(startTime == 0)
      {
        startTime = currentTime;
      }
      
      analogWrite(SOLENOID, 0);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);

      if ((currentTime - startTime) > 1000)
      {
        previousState = currentState;
        currentState = INFLATE;
        startTime = 0;
      }
      break;
    
    case INFLATE:
      disp_pressure(10);
      analogWrite(SOLENOID, 255);
      if(enter_INFLATE == true)
      {
        analogWrite(POSITIVE_PUMP, 120); 
        enter_INFLATE = false;
      }
      if ((mpr.readPressure() / 68.947572932) >= 12.67) 
      {
        previousState = currentState;
        currentState = HOLD;
        delay(1000);
        enter_INFLATE = true;
      }
      break;

    case HOLD:
      disp_pressure(10);

      currentTime = millis();
      if(startHoldTime == 0)
      {
        startHoldTime = currentTime;
      }
      analogWrite(SOLENOID, 0);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);
      if ((currentTime - startHoldTime) > 1000)
      {
        previousState = currentState;
        currentState = DEFLATE;
        startHoldTime = 0;
      }
      break;

    case DEFLATE:
      disp_pressure(10);
      analogWrite(SOLENOID, 255);
      if(enter_DEFLATE == true)
      {
        analogWrite(NEGATIVE_PUMP, 255);
        enter_DEFLATE = false;
      }
      if ((mpr.readPressure() / 68.947572932) <= 11.63)       
      {
        previousState = currentState;
        currentState = START;
        delay(1000);
        enter_DEFLATE = true;
      }
      break;

     case STOP:
      disp_pressure(10);
      analogWrite(SOLENOID, 255);
      analogWrite(POSITIVE_PUMP, 0);
      analogWrite(NEGATIVE_PUMP, 0);
      if (digitalRead(BUTTON) == LOW) 
      {
        delay(1000);
        currentState = START;
      }
      break;
  }
  
}
