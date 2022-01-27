/*
 * @file    pressure_sensor
 * @author  CU Boulder Medtronic Team 7
 * @brief   Methods for interacting with pressure sensor
 */
#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Wire.h>               // library for I2C connections
#include "Adafruit_MPRLS.h"     // library for the pressure sensor

// Pin definitions for the pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

#define TCAADDR 0x70   // address for the mux

extern void scanner(void);
extern void sensor_initialization(Adafruit_MPRLS mpr);
extern void tcaselect(uint8_t i);
extern void disp_pressure(Adafruit_MPRLS mpr, unsigned long readDelay, uint8_t channelNum);
extern float get_pressure(Adafruit_MPRLS mpr, uint8_t channelNum);

#endif PRESSURE_SENSOR_H
