/*
 * @file    pressure_sensor
 * @author  CU Boulder Medtronic Team 7
 * @brief   Methods for interacting with pressure sensor
 */

#include "pressure_sensor.h"

// Times used to vary the display speed of the pressure sensors
unsigned long lastPressureReadTime = millis();

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
 * @name    scanner
 * @desc    Scan for connections to the multiplexer 
 * @param   None
 * @return  None
 */
void scanner(void)
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
    delay(1000);
}

/*
 * @name    sensor_initialization
 * @desc    Used to initialize each of the pressure sensors
 * @param   None
 * @return  None
 */
void sensor_initialization(Adafruit_MPRLS mpr)
{
    Serial.println("Initializing pressure sensors:"); Serial.println("");
    
    // Init first sensor
    tcaselect(0);
    if (!mpr.begin())
    {
      Serial.println("Error initializing Sensor 0");
      while(1);
    }
    Serial.println("Initialized Sensor 0");

    // Init second sensor
    tcaselect(1);
    if (!mpr.begin())
    {
      Serial.println("Error initializing Sensor 1");
      while(1);
    }
    Serial.println("Initialized Sensor 1");
    
    // Init third sensor
    tcaselect(2);
    if (!mpr.begin())
    {
      Serial.println("Error initializing Sensor 2");
      while(1);
    }
    Serial.println("Initialized Sensor 2\n");
    Serial.println("------------------------------------------------------");
    delay(1000);
}

/*
 * @name    disp_pressure
 * @desc    Function to display the pressure sensors
 * @param   readDelay - how frequent pressure will be logged
 * @return  None
 */ 
void disp_pressure(Adafruit_MPRLS mpr, unsigned long readDelay, uint8_t channelNum)
{
    unsigned long currentTime = millis();
    
    if ((currentTime - lastPressureReadTime) > readDelay)
    {
        // Obtain pressure sensor reading from Sensor 0
        Serial.println("Pressure from Sensor: " + channelNum);
        tcaselect(channelNum);
        float pressure_hPa0 = mpr.readPressure();
        //Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa0);
        Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa0 / 68.947572932);
        lastPressureReadTime = millis();
    }
}

/*
 * @name    disp_pressure
 * @desc    Function to display the pressure sensors
 * @param   readDelay - how frequent pressure will be logged
 * @return  None
 */ 
float get_pressure(Adafruit_MPRLS mpr, uint8_t channelNum)
{
    tcaselect(channelNum);
    float pressure_hPa0 = mpr.readPressure();
    float presssure_PSI = (pressure_hPa0 / 68.947572932);
    return presssure_PSI;
}
