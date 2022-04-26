/*
 * @file    mux_test
 * @author  CU Boulder Medtronic Team 7
 * @brief   Test code that gets the three pressure sensors with the multiplexer running.
 *          Based on https://playground.arduino.cc/Main/I2cScanner/
 */

#include <Wire.h>             // library for I2C connections
#include "Adafruit_MPRLS.h"   // library for the pressure sensor

#define TCAADDR 0x70   // TCA Address for multiplexer
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

// Instantiate mpr class for each sensor
Adafruit_MPRLS mpr0 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr1 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr2 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

/*
 * @name    scanner
 * @desc    Scan for connections to the multiplexer 
 * @param   None
 * @return  None
 */
void scanner()
{
    while (!Serial);
    delay(1000);
    Wire.begin();
    
    Serial.begin(115200);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
        tcaselect(t);
        Serial.print("TCA Port #"); 
        Serial.println(t);

        for (uint8_t addr = 0; addr<=127; addr++) 
        {
            if (addr == TCAADDR) 
            {
              continue;
            }
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
                Serial.print("Found I2C 0x");  
                Serial.println(addr,HEX);
            }
        }
    }   
    Serial.println("\nFinished scanning mux ports\n");
    Serial.println("------------------------------------------------------");
    delay(500);
}

/*
 * @name    sensor_initialization
 * @desc    Used to initialize each of the pressure sensors
 * @param   None
 * @return  None
 */
void sensor_initialization()
{
    Serial.println("Initializing pressure sensors:"); Serial.println("");
      
    // Init first sensor
    tcaselect(0);
    if (!mpr0.begin())
    {
      Serial.println("Error initializing Sensor 0");
      while(1);
    }
    Serial.println("Initialized Sensor 0");
    
    // Init second sensor
    tcaselect(1);
    if (!mpr1.begin())
    {
      Serial.println("Error initializing Sensor 1");
      while(1);
    }
    Serial.println("Initialized Sensor 1");
    
    // Init third sensor
    tcaselect(2);
    if (!mpr2.begin())
    {
      Serial.println("Error initializing Sensor 2");
      while(1);
    }
    
    Serial.println("Initialized Sensor 2\n");
    Serial.println("------------------------------------------------------");
    delay(500);
}

/*
 * @name    tcaselect
 * @desc    Select device TCA is connected to
 * @param   i - which port you want to connect to
 * @return  None
 */
void tcaselect(uint8_t i) {
    if (i > 7)
    {
      return;
    }
   
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

/*
 * @name    pressure_reading
 * @desc    Function that obtains the pressure readings from each of the sensors
 * @param   counter - current log number
 * @return  None
 */ 
void pressure_reading(int counter)
{
    Serial.print("Sample Number: "); Serial.println(counter); Serial.println("");
      
    // Obtain pressure sensor reading from Sensor 0
    Serial.println("Pressure from Sensor 0:");
    tcaselect(0);
    float pressure_hPa0 = mpr0.readPressure();
    Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa0 / 68.947572932);
    

    // obtain pressure reading from Sensor 1
    Serial.println("\nPressure from Sensor 1:");
    tcaselect(1);
    float pressure_hPa1 = mpr1.readPressure();
    Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa1 / 68.947572932);
    
    // obtain pressure reading from Sensor 2
    Serial.println("\nPressure from Sensor 2:");
    tcaselect(2);
    float pressure_hPa2 = mpr2.readPressure();
    Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa2 / 68.947572932);

    Serial.println("------------------------------------------------------");
    delay(1000);
}

// Log count for how many times you have displayed the pressures
int counter = 0;

/*
 * @name  setup
 * @desc  called once on startup
 */
void setup()
{
    // Run scanner to see the used mux ports
    scanner();

    // initialize the three pressure sensors
    sensor_initialization();
}

/*
 * @name  loop
 * @desc  called indefinitely
 */
void loop() 
{
    // count the samples
    counter++;
    
    // get a pressure reading from three sensors
    pressure_reading(counter);
}
