#include <Wire.h>               // library for I2C connections
#include "Adafruit_MPRLS.h"     // library for the pressure sensor

// Pin definitions for the pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

#define TCAADDR 0x70   // address for the mux

// Instantiate mpr class for pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

const int ledPin = 13;
//Store Incoming serial data
int incomingByte;

// Times used to vary the display speed of the pressure sensors
unsigned long lastPressureReadTime = millis();
unsigned long currentTime = 0;

void setup() {
    //scan for mux ports and begin serial communication
    scanner();
 
    // initialize all three pressures sensors
    sensor_initialization();
    pinMode(ledPin, OUTPUT);

}

void loop() { 
  disp_pressure(1000);
}

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
 * @name    tcaselect
 * @desc    Select device TCA is connected to
 * @param   i - which port you want to connect to
 * @return  None
 */
void tcaselect(uint8_t i) {
    if (i > 7) return;
    
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

/*
 * @name    disp_pressure
 * @desc    Function to display the pressure sensors
 * @param   readDelay - how frequent pressure will be logged
 * @return  None
 */ 
void disp_pressure(unsigned long readDelay)
{
    unsigned long currentTime = millis();
    
    if ((currentTime - lastPressureReadTime) > readDelay)
    {
        // Obtain pressure sensor reading from Sensor 0
        Serial.println("Pressure from Sensor 0:");
        tcaselect(0);
        float pressure_hPa0 = mpr.readPressure();
        //Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa0);
        Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa0 / 68.947572932);
        
    
        // obtain pressure reading from Sensor 1
        Serial.println("\nPressure from Sensor 1:");
        tcaselect(1);
        float pressure_hPa1 = mpr.readPressure();
        //Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa1);
        Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa1 / 68.947572932);
        
    
        // obtain pressure reading from Sensor 2
        Serial.println("\nPressure from Sensor 2:");
        tcaselect(2);
        float pressure_hPa2 = mpr.readPressure();
        //Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa2);
        Serial.print("Pressure (PSI): "); Serial.println( pressure_hPa2 / 68.947572932);
    
        Serial.println("------------------------------------------------------");
        lastPressureReadTime = millis();
    }
}
