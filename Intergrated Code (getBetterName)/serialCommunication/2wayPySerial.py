import serial
import time
import serial.tools.list_ports
import threading


class globalVars():
    pass

G = globalVars() #technically not a gloabl var
G.lock = threading.Lock() #not really necessary in this case, but useful none the less
G.value = 0
G.kill = False

# Define the serial port and baud rate.
ser = serial.Serial('COM3', 115200)
time.sleep(2) # wait for the serial connection to initialize

def printer():
    while True:
        arduinoString = ser.readline().decode("utf-8") #.strip()    
        print(arduinoString)
        time.sleep(0.01) #To see diffrence between prints
        
t1 = threading.Thread(target=printer)
t1.start()
        
def askInput():
    userInput = input()

    if userInput =="h":
        ser.write(b'H') 
    elif userInput =="l":
        ser.write(b'L')
    elif userInput =="quit" or userInput == "q":
        print("Exiting")
        ser.write(b'L')
        ser.close()
        G.kill = True
        return False
        
    return True

while askInput():
    pass

        
                          




    

