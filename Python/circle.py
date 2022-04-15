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
sleep = 0.15

#c0
minPSI0 = 13.5 # 14.0
maxPSI0 = 15.3 # 15.2
psiDelta0 = 0.05
psiRange0 =int( (maxPSI0 - minPSI0) / psiDelta0)
#c1
minPSI1 = 14.0 # 14.0
maxPSI1 = 15.6 # 15.1
psiDelta1 = 0.05
psiRange1 =int( (maxPSI1 - minPSI1) / psiDelta1)
#c2
minPSI2 = 13.6 # 14.0
maxPSI2 = 15.4 # 15.3
psiDelta2 = 0.05
psiRange2 =int( (maxPSI2 - minPSI2) / psiDelta2)

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
        time.sleep(sleep)
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
    for i in range(psiRange2 + 1):
        psi = minPSI2 + (psiDelta2 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel2, psi)  
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel2)))

    
    #decrease second channel
    print("")
    print("channel 1 decrease")
    for i in range(psiRange1 + 1):
        psi = maxPSI1 - (psiDelta1 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel1, psi)
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel1)))
    arduino.sendDesiredPressure(arduino.channel1, 12.25)

        
    #increase first channel
    print("")
    print("channel 0 increase")
    for i in range(psiRange0 + 1):
        psi = minPSI0 + (psiDelta0 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel0, psi) 
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel0)))
        
    #decrease third channel
    print("")
    print("channel 2 decrease")
    for i in range(psiRange2 + 1):
        psi = maxPSI2 - (psiDelta2 * i)
        print('thertotical psi: {}'.format(psi))
        arduino.sendDesiredPressure(arduino.channel2, psi)
        time.sleep(sleep)
        print('actual psi: {}'.format(arduino.getActualPressure(arduino.channel2)))
    
    arduino.sendDesiredPressure(arduino.channel0, minPSI0)
except:
    print("Error")
else:
    print('Elapsed Time: {}'.format(time.time() - startTime))
finally:
    arduino.close()
