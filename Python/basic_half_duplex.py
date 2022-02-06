'''
 * @file    basic_half_duplex.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic method to send pressure commmands to python
'''

import serial as pys
import time
ser = pys.Serial()
ser.baudrate = 115200
ser.port = 'COM4'

# Open serial
ser.open()
if (ser.is_open != True):
    print("Could not open serial")
    quit()

while(True):
    print("Enter pressure (XX.XX implied) or 'r' for read pressure:")
    command = input()
    
    # quit
    if (command == "q"):
        break

    if (command == "r"):
        # Special flag to send to arduino
        command = "9999"
    
    # Convert string to utf-8 and send over serial
    bytesSent = ser.write(command.encode('utf-8'))

    if (command == "9999"):
        time.sleep(.1)
        temp = ser.readline().decode("utf-8")
        print("Channel 1 Pressure: %(val)s" % {"val": temp.rstrip()})

ser.close()