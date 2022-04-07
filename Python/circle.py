'''
 * @file    circle.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Theortically will move the robot in a circle 
'''
from Py_Arduino_Communication.arduino_control import arduino_control
import time

# Init EM Nav and Arduino
# ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)

startTime = time.time()
sleep = 0.2

#c0
minPSI0 = 14.0
maxPSI0 = 15.6
psiRange0 =int( (maxPSI0 - minPSI0) / 0.1)
psiDelta0 = 0.1
#c1
minPSI1 = 14.0
maxPSI1 = 14.7
psiRange1 =int( (maxPSI1 - minPSI1) / 0.1)
psiDelta1 = 0.1
#c2
minPSI2 = 14.0
maxPSI2 = 14.8
psiRange2 =int( (maxPSI2 - minPSI2) / 0.1)
psiDelta2 = 0.1

arduino.sendDesiredPressure(arduino.channel0, 12.0)
arduino.sendDesiredPressure(arduino.channel1, 12.0)
arduino.sendDesiredPressure(arduino.channel2, 12.0)

try:
    #start first channel at max
    arduino.sendDesiredPressure(arduino.channel0, maxPSI0)
    time.sleep(sleep)
    #increase second channel
    print("channel 1 increase")
    for i in range(psiRange1 + 1):
        psi = minPSI1 + (psiDelta1 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel1, psi)
        time.sleep(1)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel1)))
    
    #decrease first channel
    print("")
    print("channel 0 decrease")
    for i in range(psiRange0 + 1):
        psi = maxPSI0 - (psiDelta0 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel0, psi)
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel0)))
    arduino.sendDesiredPressure(arduino.channel0, 12.25)
        
    #increase third channel
    print("")
    print("channel 2 increase")
    for i in range(psiRange2):
        psi = minPSI2 + (psiDelta2 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel2, psi)  
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel2)))

    
    #decrease second channel
    print("")
    print("channel 1 decrease")
    for i in range(psiRange1):
        psi = maxPSI1 - (psiDelta1 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel1, psi)
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel1)))
    arduino.sendDesiredPressure(arduino.channel1, 12.25)

        
    #increase first channel
    print("")
    print("channel 0 increase")
    for i in range(psiRange0):
        psi = minPSI0 + (psiDelta0 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel0, psi) 
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel0)))
    
        
    #decrease third channel
    print("")
    print("channel 2 decrease")
    for i in range(psiRange2):
        psi = maxPSI2 - (psiDelta2 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel2, psi)
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel2)))
    arduino.sendDesiredPressure(arduino.channel2, 12.25)

    
    arduino.sendDesiredPressure(arduino.channel0, minPSI0)
except:
    print("Error")
else:
    print('Elapsed Time: {}'.format(time.time() - startTime))
finally:
    arduino.close()
