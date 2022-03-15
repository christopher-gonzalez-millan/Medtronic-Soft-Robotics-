/*
 * @file    state_machine_pressure_based
 * @author  CU Boulder Medtronic Team 7
 * @brief   This version of the state machine is them first attempt to integrate the state
 *          machine code with the three channel output. Note the button will not be able to
 *          be used in this version. In addition, the code for the mux has been added as well.
 *          Lastly, code to PWM the solenoid valve has been implemented as well.
 */

#include <Wire.h>               // library for I2C connections
#include "Adafruit_MPRLS.h"     // library for the pressure sensor

// Pin definitions for the pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

#define TCAADDR 0x70   // address for the mux

// Instantiate mpr class for pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// I/O Pins

// defining hardware pins
#define POSITIVE_PUMP 10
#define POSITIVE_SOLENOID 12
#define NEGATIVE_PUMP 11
#define NEGATIVE_SOLENOID 13

// defining hardware states
#define SOLENOID_CLOSED LOW
#define SOLENOID_OPEN HIGH
#define PUMP_PWM_POS 100
#define PUMP_PWM_NEG 100

//defining
#define ATMOSPHERIC_PRESSURE 12.1
#define HI_PRESSURE 13.2
#define LOW_PRESSURE 9
#define HOLD_TIME 100 // in milliseconds
#define SENSOR_ID 2

//simulate valve pwm values
float INFLATE_VALVE_HI = 0;
float INFLATE_VALVE_LOW = 0;
float DEFLATE_VALVE_HI = 0;
float DEFLATE_VALVE_LOW = 0;

int TRAIL = 0;
float CYCLE_TIME = 5000;

unsigned long currentTime = 0;
unsigned long startTime = 0;

/*
 * @name  setup
 * @desc  called once on startup
 */
void setup() {
    // scan for mux ports and begin serial communication
    scanner();

    // initialize all three pressures sensors
    sensor_initialization();

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
    for(int i = 100; i >= 0; i-=5){
        TRAIL = i;
        INFLATE_VALVE_HI = CYCLE_TIME*(TRAIL*0.01);
        INFLATE_VALVE_LOW = CYCLE_TIME - INFLATE_VALVE_HI;
        DEFLATE_VALVE_HI = CYCLE_TIME*(TRAIL*0.01);
        DEFLATE_VALVE_LOW = CYCLE_TIME - DEFLATE_VALVE_HI;
        
        if(reset()){
          Serial.print("Start of Trail, "); Serial.println(TRAIL);
        }
        float x = getPressure();

        //inflate test
        startTime = millis();

        while(x < HI_PRESSURE){
          //Serial.print("Pressure (PSI): "); Serial.println(x);
          inflate();
          x = getPressure();
        }
        
        Serial.print("Inflate(ms), "); Serial.println(millis() - startTime);
        hold;
        delay(HOLD_TIME);
        
        //inflate test
        startTime = millis();
        
        while(x > LOW_PRESSURE){
          //Serial.print("Pressure (PSI): "); Serial.println(x);
          deflate();
          x = getPressure();
        }
        
        Serial.print("Deflate(ms), "); Serial.println(millis() - startTime);
        hold;
        delay(HOLD_TIME);
    }
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
    delay(1000);
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
    if (i > 7)
    {
      return;
    }

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

float getPressure(){
    tcaselect(SENSOR_ID);

    float pressure_hPa2 = mpr.readPressure();

    return pressure_hPa2 / 68.947572932;
}

bool reset(){
    //TODO: allow for tollerance
    float x = getPressure();
    while(getPressure() > ATMOSPHERIC_PRESSURE + 0.4 && getPressure() < ATMOSPHERIC_PRESSURE - 0.4){
        Serial.print("Pressure (PSI): "); Serial.println(x);
        // Pumps
        analogWrite(POSITIVE_PUMP, 0);
        analogWrite(NEGATIVE_PUMP, 0);
        // Solenoids
        analogWrite(POSITIVE_SOLENOID, SOLENOID_OPEN);
        analogWrite(NEGATIVE_SOLENOID, SOLENOID_OPEN);
        x = getPressure();
    }
    // Solenoids
    analogWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
    analogWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);

    return true;
}

void inflate(){
    // Solenoids
    digitalWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
    digitalWrite(POSITIVE_SOLENOID, SOLENOID_OPEN);
    delayMicroseconds(INFLATE_VALVE_HI);
    digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
    delayMicroseconds(INFLATE_VALVE_LOW);

    // Pumps
    analogWrite(POSITIVE_PUMP, PUMP_PWM_POS);
    analogWrite(NEGATIVE_PUMP, 0);
}

void deflate(){
    if (getPressure() >= 12.1)
    {
      digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
      digitalWrite(NEGATIVE_SOLENOID, HIGH);
      delayMicroseconds(DEFLATE_VALVE_HI);
      digitalWrite(NEGATIVE_SOLENOID, LOW);
      delayMicroseconds(DEFLATE_VALVE_LOW);
    }
    else
    {
      digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
      digitalWrite(NEGATIVE_SOLENOID, SOLENOID_OPEN);
    }

    // Pumps
    analogWrite(POSITIVE_PUMP, 0);
    analogWrite(NEGATIVE_PUMP, PUMP_PWM_NEG);
}

void hold(){

    // Solenoids
    digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
    digitalWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);

    // Pumps
    analogWrite(POSITIVE_PUMP, 0);
    analogWrite(NEGATIVE_PUMP, 0);
}
