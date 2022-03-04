/*
 * @file    pressure_feedback_algorithm.ino
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic method to hit desired pressure in any channel
 */

#include <stdlib.h>
#include <string.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"

// Channel/pressure related defines
#define NUM_CHANNELS  3
#define ON 1
#define OFF 0
#define DEFAULT_PRESSURE 12.25
#define PRESSURE_HOLD_TOLERANCE 0.05
#define PRESSURE_TOLERANCE 0.01

// I/O related defines
#define SOLENOID_CLOSED LOW
#define SOLENOID_OPEN HIGH
#define PUMP_PWM_POS 110
#define PUMP_PWM_NEG 110
#define PUMP_OFF 0

// Serial related defines
#define MSG_HEADER_SIZE 2
#define MSG_BODY_SIZE 4
#define EXPECTED_MSG_LENGTH (MSG_HEADER_SIZE + MSG_BODY_SIZE)
// Special message for selecting which channels you want to run on
#define CHANNEL_SELECTION_MSG_LENGTH 3

// Enabled when pressure sensor functions are defined here in file
#define LOCAL_PRESSURE_SENSOR_FUNCTIONS

// Pin definitions for the pressure sensor
#define RESET_PIN  -1   // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1   // set to any GPIO pin to read end-of-conversion by pin
#define TCAADDR    0x70 // address for the mux

// Instantiate mpr class for pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
 
// Possible States
typedef enum {
    INFLATE,
    HOLD,
    DEFLATE
} state;

// Data stored for each channel
struct channelData 
{
    uint8_t active;
    state currentState;
    float currentPressure;
    float desiredPressure;
    int positivePump;
    int negativePump;
    int positiveSolenoid;
    int negativeSolenoid;
};

channelData channels[NUM_CHANNELS] =
{
    {OFF, HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 3,  5,  2,  4},  // Channel 0
    {OFF, HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 6,  9,  7,  8},  // Channel 1
    {OFF, HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 10, 11, 12, 13}, // Channel 2
};

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

    Serial.println("Arduino Setup Complete");
}

/*
 * @name  loop
 * @desc  called indefinitely
 */
void loop() {

    // Read in serial command if needed...
    // Arduino serial buffer holds 64 bytes
    if (Serial.available() >= EXPECTED_MSG_LENGTH)
    {
        handleCommand();
    }

    // Update I/O if needed based on last command
    for (int8_t cNum = 0; cNum < NUM_CHANNELS; cNum++)
    {
        if(channels[cNum].active)
        {
            channels[cNum].currentPressure = get_pressure(mpr, cNum);

            switch (channels[cNum].currentState)
            {
                case INFLATE:
                    if ((channels[cNum].currentPressure >= (channels[cNum].desiredPressure - PRESSURE_TOLERANCE)) &&
                        (channels[cNum].currentPressure <= (channels[cNum].desiredPressure + PRESSURE_TOLERANCE)) )
                    {
                        channels[cNum].currentState = HOLD;
                    }
                    else if (channels[cNum].currentPressure >= (channels[cNum].desiredPressure + PRESSURE_HOLD_TOLERANCE))
                    {
                        channels[cNum].currentState = DEFLATE;
                    }
                    break; 
                    
                case HOLD:
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
                    // Pumps
                    analogWrite(channels[cNum].positivePump, PUMP_PWM_POS);
                    analogWrite(channels[cNum].negativePump, PUMP_OFF);

                    // Solenoids
                    digitalWrite(channels[cNum].positiveSolenoid, SOLENOID_OPEN);
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                    
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
                    if (channels[cNum].currentPressure >= 12.5)
                    {
                        digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_OPEN);
                        delay(2);
                        digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                        delay(4);
                    }
                    //else if (channels[cNum].currentPressure >= 12)
                    //{
                    //    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_OPEN);
                    //    delay(4);
                    //    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                    //    delay(2);
                    // }
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

}

/*
 * @name    handleCommand
 * @desc    Handle command if found in serial buffer
 * @param   None
 * @return  None
 */
void handleCommand(void)
{
    char header[2];
    char msg[4];

    // Read first two bytes as header
    header[0] = Serial.read();
    header[1] = Serial.read();

    // Read header to find channel number
    uint8_t cNum;
    if (strncmp(header, "c0", 2) == 0)
    {
        cNum = 0;
    }
    else if (strncmp(header, "c1", 2) == 0)
    {
        cNum = 1;
    }
    else if (strncmp(header, "c2", 2) == 0)
    {
        cNum = 2;
    }
    else if (strncmp(header, "sc", 2) == 0)
    {
        handleSelectionCommand();
        return;
    }
    else
    {
        Serial.println("CMD ERROR");
        return;
    }

    // Read in body of message
    for (int8_t i = 0; i < 4; i++)
    {
        msg[i] = Serial.read();
    }

    if (strncmp(msg, "read", 4) == 0)
    {
        // Read pressure command
        channels[cNum].currentPressure = get_pressure(mpr, cNum);
        String pVal = String(channels[cNum].currentPressure, 2);
        Serial.println(pVal);
    }
    else
    {
        // Set pressure command...
        // Contains desired PSI (implied decimal XX.XX) for channels that are enabled
        char pressure[5];
        sprintf(pressure, "%c%c.%c%c", msg[0], msg[1], msg[2], msg[3]);
        channels[cNum].desiredPressure = ((String(pressure)).toFloat());
        // Send confirmation back
        Serial.println("rx");
    }

}

/*
 * @name    handleSelctionCommand
 * @desc    Handle command for channel selection
 * @param   None
 * @return  None
 */
void handleSelectionCommand(void)
{
    for (int8_t cNum = 0; cNum < 3; cNum++)
    {
        char enabled = Serial.read();
        if (enabled == '1')
        {
            channels[cNum].active = ON;
        }
    }
    // Extra character currently not being used
    char reserved = Serial.read();

    // Send confirmation back
    Serial.println("rx");
}

#if defined(LOCAL_PRESSURE_SENSOR_FUNCTIONS)
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
    
    for (uint8_t t=0; t<8; t++) 
    {
        tcaselect(t);

        for (uint8_t addr = 0; addr<=127; addr++) 
        {
            if (addr == TCAADDR) 
            {
                continue;
            }
            Wire.beginTransmission(addr);
            Wire.endTransmission();
        }
    }
}

/*
 * @name    sensor_initialization
 * @desc    Used to initialize each of the pressure sensors
 * @param   None
 * @return  None
 */
void sensor_initialization()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        tcaselect(i);
        if (!mpr.begin())
        {
            // Error Initializing sensor
            while(1);
        }
    }
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
 * @name    get_pressure
 * @desc    Function to return the pressure readings
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
#endif // LOCAL_PRESSURE_SENSOR_FUNCTIONS
