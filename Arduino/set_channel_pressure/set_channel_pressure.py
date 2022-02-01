'''
 * @file    set_channel_pressure.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic method to send pressure commmands to python
'''

import serial as pys
ser = pys.Serial()
ser.baudrate = 115200
ser.port = 'COM4'

# Open serial
ser.open()
if (ser.is_open != True):
    print("Could not open serial")
    quit()

while(True):
    print("Enter pressure (XX.XX implied):")
    nextPressure = input()
    
    # quit
    if (nextPressure == "q"):
        break
    
    # Convert string to utf-8 and send over serial
    bytesSent = ser.write(nextPressure.encode('utf-8'))

ser.close()