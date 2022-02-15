/*
 * @file    pwm_test
 * @author  CU Boulder Medtronic Team 7
 * @brief   This is a PWM test for two pumps and one valve that reads the pressures 
 *          from a pressure sensor, but does not use it to control anything.
 *          This only works with a one channel setup to test the components.
 *          Note: No Multiplexer!
 */
 
#include <Wire.h>             // library for I2C connections
#include "Adafruit_MPRLS.h"   // library for the pressure sensor

#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

// Instantiate mpr object for sensor
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

/*
 * @name  setup
 * @desc  called once on startup
 */
void setup() {
    Serial.begin(115200);
    Serial.println("MPRLS Simple Test");
    
    if (! mpr.begin()) {
      Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
      while (1) {
        delay(10);
      }
    }
    Serial.println("Found MPRLS sensor");
    pinMode(3, OUTPUT);   // negative pump
    pinMode(5, OUTPUT);   // solenoid valve
    pinMode(6, OUTPUT);   // positive pump
}

/*
 * @name    disp_pressure
 * @desc    Output current pressure to the serial monitor
 * @param   None
 * @return  None
 */
void disp_pressure(void)
{
    Serial.print("Pressure (PSI): "); Serial.println(mpr.readPressure() / 68.947572932);
}

/*
 * @name    pwm_pump
 * @desc    PWM the inflate and deflate pumps
 * @param   dutyCycle - cycle used for both positive and negative pump
 * @return  None
 */
void pwm_pump(int dutyCycle) {
    // Solenoid always on (letting air through)
    analogWrite(5, 255);

    // Convert duty cycle to analog value (0-255)
    int analogOut = dutyCycle/100*255;

    // Positive pump on for 3 sec
    analogWrite(6, analogOut);
    delay(3000);
    analogWrite(6, 0);
    disp_pressure();

    // Negative pump on for 3 sec
    analogWrite(3, analogOut);
    delay(3000);
    analogWrite(3, 0);
    disp_pressure();

    // Pause until next run
    delay(3000);
}

/*
 * @name    pwm_solenoid
 * @desc    PWM a single solenoid valve. Because the solenoid valves are
 *          connected to digital pins, replicate PWM by sending digital commands
 *          with delays
 * @param   dutyCycle - cycle used for the valve
 * @return  None
 */
void pwm_solenoid(int dutyCycle) {
    // Replicate PWM with 10ms intervals
    // So 50% duty cycle would be 5ms ON -> 5ms off
    int dutyOn = dutyCycle/10;
    int dutyOff = 10 - dutyCycle/10;

    // Positive pump on
    analogWrite(6, 255);

    // PWM for a total of 3 seconds
    int numSec = 3;
    for(int i = 0; i < numSec*100; i++) {
      analogWrite(5, 255);
      delay(dutyOn);
      analogWrite(5, 0);
      delay(dutyOff);
    }

    // positive pump off
    analogWrite(6, 0);

    //  <=== Let some air out ===>
    // Open solenoid
    analogWrite(5, 255);
    // Negative pump on for 3 sec
    analogWrite(3, 255);
    delay(3000);
    analogWrite(3, 0);  

    // Pause until next run
    delay(3000);
}

/*
 * @name  loop
 * @desc  called indefinitely
 */
void loop() {
    // basic inflate deflate test - Solenoid always on
    pwm_pump(100);

    // Basic solenoid PWM test
    pwm_solenoid(40);
}
