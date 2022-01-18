/*
 * @file    mux_test
 * @author  CU Boulder Medtronic Team 7
 * @brief   Test code that gets the three pressure sensors with the multiplexer running.
 *          Based on https://playground.arduino.cc/Main/I2cScanner/
 */

#include "Wire.h"
#include "Adafruit_MPRLS.h"

#define TCAADDR 0x70   // TCA Address for multiplexer
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

// Instantiate mpr objects for each sensor
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
    Serial.println("\ndone");
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
 * @name  setup
 * @desc  called once on startup
 */
void setup()
{
    Wire.begin();
    Serial.begin(115200);

    // Init first sensor
    tcaselect(1);
    if (!mpr1.begin())
    {
      Serial.println("Error beginning sensor 1");
      while(1);
    }
}

/*
 * @name  loop
 * @desc  called indefinitely
 */
void loop() 
{
  float pressure_hPa = mpr0.readPressure();
  Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa / 68.947572932);
  delay(1000);
}
