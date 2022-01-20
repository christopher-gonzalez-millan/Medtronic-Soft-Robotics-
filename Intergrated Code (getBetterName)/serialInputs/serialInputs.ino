#include <Wire.h>               // library for I2C connections

// Pin definitions for the pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

#define TCAADDR 0x70   // address for the mux

// Instantiate mpr class for pressure sensors
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// I/O Pins
//#define BUTTON  13
#define POSITIVE_PUMP 3
#define POSITIVE_SOLENOID 2 
#define NEGATIVE_PUMP 5
#define NEGATIVE_SOLENOID 4

#define SOLENOID_CLOSED LOW
#define SOLENOID_OPEN HIGH
#define PUMP_PWM_POS 100
#define PUMP_PWM_NEG 100
#define HI_PRESSURE 13.15
#define LOW_PRESSURE 11
#define HOLD_TIME 5000  // in milliseconds
#define SENSOR_ID_0 0
#define SENSOR_ID_1 1

// Possible States
typedef enum {
    START,
    INFLATE,
    HOLD,
    DEFLATE,
    STOP
} state;

// Init states
state currentState = START;
state previousState = STOP;

// Times used to vary the display speed of the pressure sensors
unsigned long lastPressureReadTime = millis();
unsigned long currentTime = 0;

// Times used to verify the hold time for the hold states
unsigned long startHoldTime = 0;
unsigned long startTime = 0;

int incomingByte;

void setup() {
// scan for mux ports and begin serial communication
    scanner();
  
    // initialize all three pressures sensors
    sensor_initialization();
    
    pinMode(POSITIVE_PUMP, OUTPUT);
    pinMode(POSITIVE_SOLENOID, OUTPUT);
    pinMode(NEGATIVE_PUMP, OUTPUT);
    pinMode(NEGATIVE_SOLENOID, OUTPUT);  // initialize serial communication:
}

void loop() {
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      primary0();
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L') {
      primary1();
    }
  }
}

void primary0() 
{
    switch(currentState) 
    {
        case START:
            disp_pressure(1000);
      
            currentTime = millis();
            if(startTime == 0)
            {
              startTime = currentTime;
            }
            
            // Solenoids
             analogWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
             analogWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
//            analogWrite(7, SOLENOID_CLOSED);
//            analogWrite(8, SOLENOID_CLOSED);
            // analogWrite(12, SOLENOID_CLOSED);
            // analogWrite(13, SOLENOID_CLOSED);
            
            // Pumps
             analogWrite(POSITIVE_PUMP, 0);
             analogWrite(NEGATIVE_PUMP, 0);
//            analogWrite(6, 0);
//            analogWrite(9, 0);
            // analogWrite(10, 0);
            // analogWrite(11, 0);
            
            if ((currentTime - startTime) > HOLD_TIME)
            {
              previousState = currentState;
              currentState = INFLATE;
              startTime = 0;
            }
            break;
    
        case INFLATE:
            disp_pressure(1000);
            
            // Solenoids
             digitalWrite(POSITIVE_SOLENOID, SOLENOID_OPEN);
             digitalWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
//            digitalWrite(7, SOLENOID_OPEN);
//            digitalWrite(8, SOLENOID_CLOSED);
            // digitalWrite(12, SOLENOID_OPEN);
            // digitalWrite(13, SOLENOID_CLOSED);
            
            // Pumps
             analogWrite(POSITIVE_PUMP, PUMP_PWM_POS);
             analogWrite(NEGATIVE_PUMP, 0);
//            analogWrite(6, PUMP_PWM_POS);
//            analogWrite(9, 0);
            // analogWrite(10, PUMP_PWM_POS);
            // analogWrite(11, 0);
      
            tcaselect(SENSOR_ID_0);
            if ((mpr.readPressure() / 68.947572932) >= HI_PRESSURE) 
            {
              previousState = currentState;
              currentState = HOLD;
              // delay(1000);
              // enter_INFLATE = true;
            } 
        
            break;

        case HOLD:
            disp_pressure(1000);

            currentTime = millis();
            if(startHoldTime == 0)
            {
              startHoldTime = currentTime;
            }
            
            // Solenoids
             digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
             digitalWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
//            digitalWrite(7, SOLENOID_CLOSED);
//            digitalWrite(8, SOLENOID_CLOSED);
            // digitalWrite(12, SOLENOID_CLOSED);
            // digitalWrite(13, SOLENOID_CLOSED);
            
            // Pumps
             analogWrite(POSITIVE_PUMP, 0);
             analogWrite(NEGATIVE_PUMP, 0);
//            analogWrite(6, 0);
//            analogWrite(9, 0);
            // analogWrite(10, 0);
            // analogWrite(11, 0);
            
            if ((currentTime - startHoldTime) > HOLD_TIME)
            {
              previousState = currentState;
              currentState = DEFLATE;
              startHoldTime = 0;
            }
            break;

        case DEFLATE:
            disp_pressure(1000);

            // Solenoids
            // digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
            // digitalWrite(NEGATIVE_SOLENOID, SOLENOID_OPEN);
            tcaselect(SENSOR_ID_0);
            if ( (mpr.readPressure() / 68.947572932) >= 12.1 )
            {
              digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
              digitalWrite(NEGATIVE_SOLENOID, HIGH);
              delay(3);
              digitalWrite(8, LOW);
              delay(10);
            }
            else
            {
              digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
              digitalWrite(NEGATIVE_SOLENOID, SOLENOID_OPEN);
            }
            // digitalWrite(7, SOLENOID_CLOSED);      
            // digitalWrite(8, SOLENOID_OPEN);
            // digitalWrite(12, SOLENOID_CLOSED);
            // digitalWrite(13, SOLENOID_OPEN);
            
            // Pumps
             analogWrite(POSITIVE_PUMP, 0);
             analogWrite(NEGATIVE_PUMP, PUMP_PWM_NEG);
//            analogWrite(6, 0);
//            analogWrite(9, PUMP_PWM_NEG);
            // analogWrite(10, 0);
            // analogWrite(11, PUMP_PWM_NEG);
      
            tcaselect(SENSOR_ID_0);
            if ((mpr.readPressure() / 68.947572932) <= LOW_PRESSURE)       
            {
              previousState = currentState;
              currentState = START;
              // delay(1000);
              // enter_DEFLATE = true;
            }
            break;
      }
}

void primary1() 
{
    switch(currentState) 
    {
        case START:
            disp_pressure(1000);
      
            currentTime = millis();
            if(startTime == 0)
            {
              startTime = currentTime;
            }
            
            // Solenoids
            // analogWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
            // analogWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
            analogWrite(7, SOLENOID_CLOSED);
            analogWrite(8, SOLENOID_CLOSED);
            // analogWrite(12, SOLENOID_CLOSED);
            // analogWrite(13, SOLENOID_CLOSED);
            
            // Pumps
            // analogWrite(POSITIVE_PUMP, 0);
            // analogWrite(NEGATIVE_PUMP, 0);
            analogWrite(6, 0);
            analogWrite(9, 0);
            // analogWrite(10, 0);
            // analogWrite(11, 0);
            
            if ((currentTime - startTime) > HOLD_TIME)
            {
              previousState = currentState;
              currentState = INFLATE;
              startTime = 0;
            }
            break;
    
        case INFLATE:
            disp_pressure(1000);
            
            // Solenoids
            // digitalWrite(POSITIVE_SOLENOID, SOLENOID_OPEN);
            // digitalWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
            digitalWrite(7, SOLENOID_OPEN);
            digitalWrite(8, SOLENOID_CLOSED);
            // digitalWrite(12, SOLENOID_OPEN);
            // digitalWrite(13, SOLENOID_CLOSED);
            
            // Pumps
            // analogWrite(POSITIVE_PUMP, PUMP_PWM_POS);
            // analogWrite(NEGATIVE_PUMP, 0);
            analogWrite(6, PUMP_PWM_POS);
            analogWrite(9, 0);
            // analogWrite(10, PUMP_PWM_POS);
            // analogWrite(11, 0);
      
            tcaselect(SENSOR_ID_1);
            if ((mpr.readPressure() / 68.947572932) >= HI_PRESSURE) 
            {
              previousState = currentState;
              currentState = HOLD;
              // delay(1000);
              // enter_INFLATE = true;
            } 
        
            break;

        case HOLD:
            disp_pressure(1000);

            currentTime = millis();
            if(startHoldTime == 0)
            {
              startHoldTime = currentTime;
            }
            
            // Solenoids
            // digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
            // digitalWrite(NEGATIVE_SOLENOID, SOLENOID_CLOSED);
            digitalWrite(7, SOLENOID_CLOSED);
            digitalWrite(8, SOLENOID_CLOSED);
            // digitalWrite(12, SOLENOID_CLOSED);
            // digitalWrite(13, SOLENOID_CLOSED);
            
            // Pumps
            // analogWrite(POSITIVE_PUMP, 0);
            // analogWrite(NEGATIVE_PUMP, 0);
            analogWrite(6, 0);
            analogWrite(9, 0);
            // analogWrite(10, 0);
            // analogWrite(11, 0);
            
            if ((currentTime - startHoldTime) > HOLD_TIME)
            {
              previousState = currentState;
              currentState = DEFLATE;
              startHoldTime = 0;
            }
            break;

        case DEFLATE:
            disp_pressure(1000);

            // Solenoids
            // digitalWrite(POSITIVE_SOLENOID, SOLENOID_CLOSED);
            // digitalWrite(NEGATIVE_SOLENOID, SOLENOID_OPEN);
            tcaselect(SENSOR_ID_1);
            if ( (mpr.readPressure() / 68.947572932) >= 12.1 )
            {
              digitalWrite(7, SOLENOID_CLOSED);
              digitalWrite(8, HIGH);
              delay(3);
              digitalWrite(8, LOW);
              delay(10);
            }
            else
            {
              digitalWrite(7, SOLENOID_CLOSED);
              digitalWrite(8, SOLENOID_OPEN);
            }
            // digitalWrite(7, SOLENOID_CLOSED);      
            // digitalWrite(8, SOLENOID_OPEN);
            // digitalWrite(12, SOLENOID_CLOSED);
            // digitalWrite(13, SOLENOID_OPEN);
            
            // Pumps
            // analogWrite(POSITIVE_PUMP, 0);
            // analogWrite(NEGATIVE_PUMP, PUMP_PWM_NEG);
            analogWrite(6, 0);
            analogWrite(9, PUMP_PWM_NEG);
            // analogWrite(10, 0);
            // analogWrite(11, PUMP_PWM_NEG);
      
            tcaselect(SENSOR_ID_1);
            if ((mpr.readPressure() / 68.947572932) <= LOW_PRESSURE)       
            {
              previousState = currentState;
              currentState = START;
              // delay(1000);
              // enter_DEFLATE = true;
            }
            break;
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
