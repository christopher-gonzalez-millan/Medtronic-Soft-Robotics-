/*
 * @file    set_channel_pressure
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic method to hit desired pressure in any channel
 */

#include "pressure_sensor.h"
// Instantiate mpr class for pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Channel/pressure related defines
#define NUM_CHANNELS  1 // will be 3
#define DEFAULT_PRESSURE 12.2
#define PRESSURE_TOLERANCE 0.1

// I/O related defines
#define SOLENOID_CLOSED LOW
#define SOLENOID_OPEN HIGH
#define PUMP_PWM_POS 100
#define PUMP_PWM_NEG 100
#define PUMP_OFF 0

// Serial related defines
#define EXPECTED_MSG_LENGTH 5 // will be 15 bytes
#define COMMAND_FREQUENCY_MS 5000 // milliseconds
 
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
};

channelData channels[NUM_CHANNELS] =
{
    {HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 3,  5,  2,  4},
    // {HOLD, DEFAULT_PRESSURE, DEFAULT_PRESSURE, 6,  9,  7,  8},
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
    sensor_initialization(mpr);
    
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
            char c = Serial.read();
            if ((c == 'x') && (previous == 'x') &&
                (Serial.available() >= EXPECTED_MSG_LENGTH))
            {
                foundCommand = true;
                break;
            }
            previous = c;
        }

        if (foundCommand == true)
        {
            String pressure;
            // Read in pressure values for all channels
            for (int8_t cNum = 0; cNum < NUM_CHANNELS; cNum++)
            {
                // Parse 4 characters for given channel
                for (int8_t i = 0; i < 4; i++)
                {
                    pressure[i] = Serial.read();
                }
                // Parse out delimeter
                char delimeter = Serial.read();
                channels[cNum].desiredPressure = ((float)pressure.toInt()/100);
            }
            
            // Flush rest of input buffer
            while(Serial.available()) 
            {
                char t = Serial.read();
            }

            // Update last command time
            lastCommandTime = millis();
        }
     }

    // Update I/O if needed based on last command
    for (int8_t cNum = 0; cNum < NUM_CHANNELS; cNum++)
    {
        channels[cNum].currentPressure = get_pressure(mpr, cNum);
        if (channels[cNum].currentPressure < (channels[cNum].desiredPressure - PRESSURE_TOLERANCE))
        {
            channels[cNum].currentState = INFLATE;
        }
        else if (channels[cNum].currentPressure > (channels[cNum].desiredPressure + PRESSURE_TOLERANCE))
        {
            channels[cNum].currentState = DEFLATE;
        }
        else
        {
            channels[cNum].currentState = HOLD;
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
            
            case DEFLATE:
                // Solenoids
                digitalWrite(channels[cNum].positiveSolenoid, SOLENOID_CLOSED);
                if (channels[cNum].currentPressure >= 12.1)
                {
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_OPEN);
                    delay(3);
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_CLOSED);
                    delay(7);
                }
                else
                {
                    digitalWrite(channels[cNum].negativeSolenoid, SOLENOID_OPEN);
                }

                // Pumps
                analogWrite(channels[cNum].positivePump, PUMP_OFF);
                analogWrite(channels[cNum].negativePump, PUMP_OFF);
        }
        
    }

}
