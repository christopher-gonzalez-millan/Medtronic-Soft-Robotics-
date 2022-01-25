import serial
import time
import serial.tools.list_ports
import warnings

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

def test():
    # set up manual user input 
    userInput = input("\n Type add / subtract / quit : ")
    print('current input:{}'.format(userInput))
    
    if userInput =="add":
        time.sleep(0.1) 
        ser.write(b'H') 
        test()
    elif userInput =="subtract":
        time.sleep(0.1)
        ser.write(b'L')
        test()
    elif userInput =="quit" or userInput == "q":
        print("Exiting")
        time.sleep(0.1)
        ser.write(b'L')
        ser.close()
    else:
        print("Invalid input.")
        test()


# Define the serial port and baud rate.
ser = serial.Serial('COM3', 115200)
time.sleep(2) # wait for the serial connection to initialize

# Define Variables
Accx = []
Accy = []
Accz = []

len = 51

for i in range (len):               # Wait until the buffer is filled up to 50 values
    while (ser.inWaiting() == 0):   
        pass                        
        
    arduinoString = ser.readline().decode("utf-8") #.strip()
    
    dataArray = (arduinoString.split(','))      # Since the values are being sent consecutively, 
                                                # the values must be split into x,y,and z
    # Data conversion from string to float
    temp1 = float(dataArray[0])             
    temp2 = float(dataArray[1]) 
    temp3 = float(dataArray[2])
    
    # Convert second element to floating number and put in P
    Accx.append(temp1)                      
    Accy.append(temp2)                      
    Accz.append(temp3)

print(Accx)    




    

