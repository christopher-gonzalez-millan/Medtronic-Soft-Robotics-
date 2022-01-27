import serial
import time
import serial.tools.list_ports
import threading

# Define the serial port and baud rate.
ser = serial.Serial('COM3', 115200)
time.sleep(2) # wait for the serial connection to initialize
        
def askInput()
    userInput = input()

    if userInput:
	ser.write(b'userInput')  

    elif userInput ==quit or userInput == q
        print(Exiting)
        ser.close()
        return False
        
    return True

while askInput()
    pass

        
                          




    
