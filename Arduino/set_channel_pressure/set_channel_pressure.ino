/*
 * @file    set_channel_pressure
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic method to hit desired pressure in any channel
 */

#include <stdlib.h>
#include <string.h>
#include <Wire.h>               // library for I2C connections
#include "Adafruit_MPRLS.h"     // library for the pressure sensor

// Pin definitions for the pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

#define TCAADDR 0x70   // address for the mux

// Instantiate mpr class for pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Channel/pressure related defines
#define NUM_CHANNELS  1 // will be 3
#define DEFAULT_PRESSURE 12.0
#define PRESSURE_HOLD_TOLERANCE 0.15
#define PRESSURE_TOLERANCE 0.03
#define HOLD_TIME_MS 500

// I/O related defines
#define SOLENOID_CLOSED LOW
#define SOLENOID_OPEN HIGH
#define PUMP_PWM_POS 100
#define PUMP_PWM_NEG 100
#define PUMP_OFF 0

// Serial related defines
#define EXPECTED_MSG_LENGTH 4 // will be 15 bytes
#define COMMAND_FREQUENCY_MS 1000 // milliseconds
 
// Possible States
typedef enum {
    INFLATE,
    HOLD,
    DEFLATE
} state;

struct channelData 
{
    state currentState;
    float currentPressure;
    float desiredPressure;
    int positivePump;
    int negativePump;
    int positiveSolenoid;
    int negativeSolenoid;
    unsigned long startHoldTime;
};

channelData channels[NUM_CHANNELS] =
{
    // {HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 3,  5,  2,  4},
    {HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 6,  9,  7,  8, 0},
    // {HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 10, 11, 12, 13},
};

unsigned long currentTime = 0;
unsigned long lastCommandTime = 0;

/*
 * @name  setup
 * @desc  called once on startup
 */
void setup() {
  
    // scan for mux ports and begin serial communication
    scanner();
  
    // initialize all three pressures sensors
    sensor_initialization();
    
    for (int8_t cNum = 0; cNum < NUM_CHANNELS; cNum++)
    {
        pinMode(channels[cNum].positivePump, OUTPUT);
        pinMode(channels[cNum].negativePump, OUTPUT);
        pinMode(channels[cNum].positiveSolenoid, OUTPUT);
        pinMode(channels[cNum].negativeSolenoid, OUTPUT);
    }
}

/*
 * @name  loop
 * @desc  called indefinitely
 */
void loop() {
    currentTime = millis();

    // Read in serial command if needed...
    // Arduino serial buffer holds 64 bytes
    // available() gets number of bytes available for reading in serial port
    if (((currentTime - lastCommandTime) >= COMMAND_FREQUENCY_MS) &&
        (Serial.available() >= EXPECTED_MSG_LENGTH))
    {
        // Recieve command (format: "xx1000-2000-3000-")
        // Contains desired PSI (implied decimal XX.XX) fr channels 0,1,2
        
        // Find start of command in serial buffer
        char previous;
        bool foundCommand = false;
        while(Serial.available()) 
        {
            if ((Serial.available() >= EXPECTED_MSG_LENGTH))
            {
                foundCommand = true;
                break;
            }
            break;
        }

        if (foundCommand == true)
        {
            char pressureVal[4];
            // Read in pressure values for all channels
            for (int8_t cNum = 0; cNum < NUM_CHANNELS; cNum++)
            {
                // Parse 4 characters for given channel
                for (int8_t i = 0; i < 4; i++)
                {
                    char asc = Serial.read();
                    pressureVal[i] = asc;
                }
                // Parse out delimeter'
                // char delimeter = Serial.read();

                char pressure[5];
                sprintf(pressure, "%c%c.%c%c", pressureVal[0], pressureVal[1], pressureVal[2], pressureVal[3]);
                
                if (strncmp(pressure, "99.99", 5) == 0)
                {
                    // Read pressure command
                    channels[cNum].currentPressure = get_pressure(mpr, 1);
                    String pVal = String(channels[cNum].currentPressure, 2);
                    Serial.println(pVal);
                } 
                else
                {
                    // Set pressure command
                    channels[0].desiredPressure = ((String(pressure)).toFloat());
                }
                // Serial.println(channels[0].desiredPressure);
            }
            
            // Flush rest of input buffer
            while(Serial.available()) 
            {
                char t = Serial.read();
                // Serial.println(t);
            }

            // Update last command time
            lastCommandTime = millis();
        }
     }

    // Update I/O if needed based on last command
    for (int8_t cNum = 0; cNum < NUM_CHANNELS; cNum++)
    {
        channels[cNum].currentPressure = get_pressure(mpr, cNum);
        // Serial.println(channels[cNum].currentPressure);
        // Serial.println(channels[0].desiredPressure);

        switch (channels[cNum].currentState)
        {
            case INFLATE:
                if ((channels[cNum].currentPressure >= (channels[cNum].desiredPressure - PRESSURE_TOLERANCE)) &&
                    (channels[cNum].currentPressure <= (channels[cNum].desiredPressure + PRESSURE_TOLERANCE)) )
                {
                    channels[cNum].currentState = HOLD;
                    channels[cNum].startHoldTime = currentTime;
                }
                else if (channels[cNum].currentPressure >= (channels[cNum].desiredPressure + PRESSURE_HOLD_TOLERANCE))
                {
                    channels[cNum].currentState = DEFLATE;
                }
                break; 
                
            case HOLD:
                //if ((currentTime - channels[cNum].startHoldTime) >= HOLD_TIME_MS)
                {
                    if (channels[cNum].currentPressure <= (channels[cNum].desiredPressure - PRESSURE_HOLD_TOLERANCE))
                    {
                        channels[cNum].currentState = INFLATE;
                    }
                    else if (channels[cNum].currentPressure > (channels[cNum].desiredPressure + PRESSURE_HOLD_TOLERANCE))
                    {
                        channels[cNum].currentState = DEFLATE;
                    }
                }
                break;
                
            case DEFLATE:
                if ((channels[cNum].currentPressure >= (channels[cNum].desiredPressure - PRESSURE_TOLERANCE)) &&
                    (channels[cNum].currentPressure <= (channels[cNum].desiredPressure + PRESSURE_TOLERANCE)) )
                {
                    channels[cNum].currentState = HOLD;
                    channels[cNum].startHoldTime = currentTime;
                }
                else if (channels[cNum].currentPressure <= (channels[cNum].desiredPressure - PRESSURE_HOLD_TOLERANCE))
                {
                    channels[cNum].currentState = INFLATE;
                }
                break;
        }     

        switch (channels[cNum].currentState)
        {
            case INFLATE:
                // Solenoids
                digitalWrite(channels[cNum].positiveSolenoid, SOLENOID_OPEN);
                digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                
                // Pumps
                analogWrite(channels[cNum].positivePump, PUMP_PWM_POS);
                analogWrite(channels[cNum].negativePump, PUMP_OFF);
                break; 
                
            case HOLD:
                // Solenoids
                digitalWrite(channels[cNum].positiveSolenoid, SOLENOID_CLOSED);
                digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                
                // Pumps
                analogWrite(channels[cNum].positivePump, PUMP_OFF);
                analogWrite(channels[cNum].negativePump, PUMP_OFF);
                break;
            
            case DEFLATE:
                // Solenoids
                digitalWrite(channels[cNum].positiveSolenoid, SOLENOID_CLOSED);
                if (channels[cNum].currentPressure >= 12.28)
                {
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_OPEN);
                    delay(2);
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                    delay(10);
                }
                else
                {
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_OPEN);
                }

                // Pumps
                analogWrite(channels[cNum].positivePump, PUMP_OFF);
                analogWrite(channels[cNum].negativePump, PUMP_PWM_NEG); 
                break;
        }
        
    }

}

void scanner()
{
    while (!Serial);
    delay(1000);
    Wire.begin();
    
    Serial.begin(115200);
    // Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
        tcaselect(t);
        // Serial.print("TCA Port #"); 
        // Serial.println(t);

        for (uint8_t addr = 0; addr<=127; addr++) 
        {
            if (addr == TCAADDR) 
            {
              continue;
            }
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
                //Serial.print("Found I2C 0x");  
                // Serial.println(addr,HEX);
            }
        }
    }   
    //Serial.println("\nFinished scanning mux ports\n");
    //Serial.println("------------------------------------------------------");
    //delay(1000);
}

void sensor_initialization()
{
    //Serial.println("Initializing pressure sensors:"); Serial.println("");
    
    // Init first sensor
    tcaselect(0);
    if (!mpr.begin())
    {
      //Serial.println("Error initializing Sensor 0");
      while(1);
    }
    //Serial.println("Initialized Sensor 0");

    // Init second sensor
    tcaselect(1);
    if (!mpr.begin())
    {
      //Serial.println("Error initializing Sensor 1");
      while(1);
    }
    //Serial.println("Initialized Sensor 1");
    
    // Init third sensor
    tcaselect(2);
    if (!mpr.begin())
    {
      //Serial.println("Error initializing Sensor 2");
      while(1);
    }
    //Serial.println("Initialized Sensor 2\n");
    //Serial.println("------------------------------------------------------");
    //delay(1000);
}

void tcaselect(uint8_t i) {
    if (i > 7)
    {
      return;
    }
   
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

float get_pressure(Adafruit_MPRLS mpr, uint8_t channelNum)
{
    tcaselect(1);
    // tcaselect(channelNum);
    float pressure_hPa0 = mpr.readPressure();
    float presssure_PSI = (pressure_hPa0 / 68.947572932);
    // Serial.println(presssure_PSI);
    return presssure_PSI;
}
