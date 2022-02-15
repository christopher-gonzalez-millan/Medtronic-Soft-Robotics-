import NDISensor
import serial
import time
import csv
import datetime
import os

"""Data Acquisition Variables:

These will need to be change for diffretnt expirment setups
"""
#output file
fileName  = datetime.datetime.now().strftime('%m/%d/%Y_%H:%M:%S')
headerRow = ['Time', 'Theoretical PSI', 'Actual PSi', 'X Position', 'Y Position' , 'Z Position']
currentDirectory = os.getcwd()
outputDirectory = os.path.join(currentDirectory, fileName)


#soft robot
deafaultPressure = 12.0
pressureDelta = 0.1
stabilizationTime = 1 #this number is in seconds.
maxPressure = 20.0
minPressure = 8.0


"""Setup Variables

These varibale will hopefully never need to be changed
"""
ndi = NDISensor.NDISensor()
ser = pys.Serial()
ser.baudrate = 115200
#TODO find a way to automate finding the COM port.
ser.port = 'COM4'

"""Helper Functions:
"""
def floatToCommand(float):
    temp = str(float)
    temp = temp.replace('.', '')

    if len(temp) == 2:
        temp = '0' + temp

    if len(temp) == 3:
        temp = temp + '0'

    return command


""" This should be MAIN but im lazy right now.
"""
# Open and check serial
ser.open()
if (not ser.is_open):
    print("Serial Port Could not be Opened")
    quit()

with open(outputDirectory, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=headerRow
    writer.writeheader()

    #first set up expiremnt
    bytesSent = ser.write(floatToCommand(minPressure).encode('utf-8'))
    deafaultPressure = minPressure
    time.sleep(stabilizationTime)


    while(deafaultPressure < maxPressure):
        tempData = {}

        #gather the current infromation about the current state of the robot

        #what the PSI should be
        tempData['Theoretical PSI'] = deafaultPressure
        # time of this state
        tempData['Time'] = datetime.datetime.now().strftime('%H:%M:%S')

        # store the 3 cartesian positions of the robot
        try:
            position = ndi.getPosition()
            tempData['X Position'] = position.deltaX
            tempData['Y Position'] = position.deltaY
            tempData['Z Position'] = position.deltaZ

        except:
            print("There was an issue with NDISensor's parser")

        #find what the actual psi of the system is
        ser.write("9999".encode('utf-8')) # tell arduino we want a psi
        actualPSI = ser.readline().decode("utf-8")
        tempData['Actual PSi'] = actualPSI

        #change the state the robot is in
        deafaultPressure += pressureDelta
        bytesSent = ser.write(floatToCommand(deafaultPressure).encode('utf-8'))
        time.sleep(stabilizationTime)



ser.close()
