/*
 * @file    state_machine_basic_button
 * @author  CU Boulder Medtronic Team 7
 * @brief   Proof of concept for state machine for one channel control
 *          Note: No multiplexer!
 */

#include <Wire.h>             // library for I2C connections
#include "Adafruit_MPRLS.h"   // library for the pressure sensor

#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

// Instantiate mpr class for pressure sensor
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// I/O Pins
#define BUTTON  13
#define POSITIVE_PUMP 3
#define POSITIVE_SOLENOID 5
#define NEGATIVE_PUMP 6
#define NEGATIVE_SOLENOID 9

// Solenoid states
#define SOLENOID_CLOSED 0
#define SOLENOID_OPEN 255

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

/*
 * @name  setup
 * @desc  called once on startup
 */
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
    pinMode(POSITIVE_SOLENOID, OUTPUT);
    pinMode(NEGATIVE_PUMP, OUTPUT);
    pinMode(NEGATIVE_SOLENOID, OUTPUT);
}

/*
 * @name  loop
 * @desc  called indefinitely
 */
void loop() {
    // Call primary state machine
    primary();
}

/*
 * @name    disp_pressure
 * @desc    Function to display the pressure sensor
 * @param   readDelay - How frequently we should read sensor pressure
 * @return  None
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

/*
 * @name  primary
 * @desc  Primary State machine
 */
void primary() 
{
    previousState = currentState;
  
    switch(currentState) {
    
      case RESET:
          disp_pressure(1000);
          
          // Solenoids off
          analogWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
          analogWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
          // Pumps off
          analogWrite(POSITIVE_PUMP, 0);
          analogWrite(NEGATIVE_PUMP, 0);
    
          // If button is pressed, transition to inflate state
          if (digitalRead(BUTTON) == LOW) 
          {
            currentState = INFLATE;
          }
          break;
          
      case INFLATE:
          disp_pressure(100);
          
          // Solenoids
          analogWrite(POSITIVE_SOLENOID, SOLENOID_OPEN);
          analogWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
          // Pumps
          analogWrite(POSITIVE_PUMP, 255/2);
          analogWrite(NEGATIVE_PUMP, 0);
    
          // Transition to hold state once button is released
          if (digitalRead(BUTTON) == HIGH) 
          {
            currentState = HOLD;
          }
          break;
  
      case HOLD:
          disp_pressure(1000);
          
          // Solenoids
          analogWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
          analogWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
          // Pumps
          analogWrite(POSITIVE_PUMP, 0);
          analogWrite(NEGATIVE_PUMP, 0);
    
          // Transition to deflate state if button is pressed again
          if (digitalRead(BUTTON) == LOW) 
          {
            currentState = DEFLATE;
          }
          break;
  
      case DEFLATE:
          disp_pressure(100);
          
          // Solenoids
          analogWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
          // Digitally PWM solenoid
          digitalWrite(NEGATIVE_SOLENOID, HIGH);
          delay(2);
          digitalWrite(NEGATIVE_SOLENOID, LOW);
          delay(2);
          
          // Pumps
          analogWrite(POSITIVE_PUMP, 0);
          analogWrite(NEGATIVE_PUMP, 0);
    
          // Transition to reset state once button is released
          if (digitalRead(BUTTON) == HIGH) 
          {
              currentState = RESET;
          }
          break;
    }
}
