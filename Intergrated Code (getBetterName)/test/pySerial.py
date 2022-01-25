import serial
import time
import serial.tools.list_ports
import warnings
import threading


def findArduino():
    arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'Arduino' in p.description  # may need tweaking to match new arduinos
    ]
    
    if not arduino_ports:
        raise IOError("Arduino not found")
        
    if len(arduino_ports) > 1:
        warnings.warn('Multiple Arduinos found(using the first)')
        
    return arduino_ports


# Define the serial port and baud rate.
ser = serial.Serial('COM3', 115200)
time.sleep(2) # wait for the serial connection to initialize

def background():
    while True:
        arduinoString = ser.readline().decode("utf-8") #.strip()    
        print(arduinoString)
        
def action(userInput):
    if userInput =="h":
        time.sleep(0.1) 
        ser.write(b'H') 
    elif userInput =="l":
        time.sleep(0.1)
        ser.write(b'L')
    elif userInput =="quit" or userInput == "q":
        print("Exiting")
        time.sleep(0.1)
        ser.write(b'L')
        ser.close()
        
    return
    

# now threading1 runs regardless of user input
threading1 = threading.Thread(target=background)
threading1.daemon = True
threading1.start()  

while True:
    x = input()
    if x: 
        action(x)
        

        
                          




    

