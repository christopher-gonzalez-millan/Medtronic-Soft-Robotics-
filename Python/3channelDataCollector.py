'''
 * @file    circle.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Theortically will move the robot in a circle 
'''
from Py_Arduino_Communication.arduino_control import arduino_control
import time
import logging
from csv_logger import CsvLogger

header = ['Date', 'Elpased Time', 'PSI0', 'PSI1', 'PSI2', 'X Position', 'Y Position' , 'Z Position']
csv_logger = CsvLogger(filename='data.csv',
                        level=logging.INFO, fmt='%(asctime)s,%(message)s', header=header)

# Init EM Nav and Arduino
ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.OFF, arduino.ON)

startTime = time.time()

minPSI = 14.0
maxPSI = 15.5
psiRange = (maxPSI - minPSI) / 0.1
psiDelta = 0.1

def changePSI(channelNum, psi):
    if channelNum == 0:
        arduino.sendDesiredPressure(arduino.channel0, psi)
    elif channelNum == 1:
        arduino.sendDesiredPressure(arduino.channel1, psi)
    elif channelNum == 2:
        arduino.sendDesiredPressure(arduino.channel2, psi)
    else:
        print('invalid channel')
        
def collectData():
    #time data
    time_diff = time.time() - startTime
    
    #channel pressure data
    psi0 = arduino.getActualPressure(arduino.channel0)
    psi1 = arduino.getActualPressure(arduino.channel1)
    psi2 = arduino.getActualPressure(arduino.channel2)

    
    position = ndi.getPosition()
    X = position.deltaX
    Y = position.deltaY
    Z = position.deltaZ
    csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (time_diff, psi0, psi1, psi2, X, Y, Z))
        
while True:
    #start first channel at max
    changePSI(0, maxPSI)
    time.sleep(0.5)
    collectData()
    
    for increaseChannel, decreaseChannel in zip([1,2,0], range(0,3)): 
        #increase channel
        for i in range(psiRange):
            psi = minPSI + (psiDelta * i)
            changePSI(increaseChannel, psi)
            time.sleep(0.5)
            collectData()
    
        #decrease channel
        for i in range(psiRange):
            psi = maxPSI - (psiDelta * i)
            changePSI(decreaseChannel, psi)
            time.sleep(0.5)
            collectData()
    
    changePSI(0, minPSI)
    time.sleep(0.5)
    collectData()


print('Elapsed Time: {}'.format(time.time() - startTime))
