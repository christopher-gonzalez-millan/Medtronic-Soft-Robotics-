import NDISensor
import serial as pys
import time
import csv
import datetime
import os

"""Data Acquisition Variables:

These will need to be change for diffretnt expirment setups
"""
#output file
print('Please enter file name: ')
fileName = input()
fileName  = fileName + '.csv'
headerRow = ['Time', 'Theoretical PSI', 'Actual PSI', 'X Position', 'Y Position' , 'Z Position']

#soft robot
deafaultPressure = 12.25
pressureDelta = .05
stabilizationTime = 10 #this number is in seconds.
maxPressure = 13.25
minPressure = 11.5
readCommand = '9999'


"""Setup Variables

These varibale will hopefully never need to be changed
"""
ndi = NDISensor.NDISensor()
ser = pys.Serial()
ser.baudrate = 115200
#TODO find a way to automate finding the COM port.
ser.port = 'COM3'

"""Helper Functions:
"""
def floatToCommand(float):
    float = round(float, 2)       # round desired pressure to 2 decimal points
    lessThanTen = False                     # checks to see if the P_des is less than 10

    if float < 10.0:
        lessThanTen = True

    temp = str(float)
    temp = temp.replace('.', '')

    if lessThanTen:
        temp = '0' + temp

    if len(temp) == 3:
        temp = temp + '0'

    return temp


""" This should be MAIN but im lazy right now.
"""
# Open and check serial
ser.open()
time.sleep(5)
if (not ser.is_open):
    print("Serial Port Could not be Opened")
    quit()

with open(fileName, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=headerRow)
    writer.writeheader()

    #first set up expiremnt
    bytesSent = ser.write(floatToCommand(minPressure).encode('utf-8'))
    deafaultPressure = minPressure
    print("pressure set at: {}".format(deafaultPressure))
    time.sleep(stabilizationTime)


    while(deafaultPressure < maxPressure):
        tempData = {}

        #gather the current infromation about the current state of the robot

        #what the PSI should be
        tempData['Theoretical PSI'] = deafaultPressure
        print('Desired Pressure: ', deafaultPressure)
        # time of this state
        tempData['Time'] = datetime.datetime.now().strftime('%H:%M:%S')

        # store the 3 cartesian positions of the robot
        try:
            position = ndi.getPosition()
            tempData['X Position'] = position.deltaX
            tempData['Y Position'] = position.deltaY
            tempData['Z Position'] = position.deltaZ

            #find what the actual psi of the system is
            ser.write(readCommand.encode('utf-8'))# tell arduino we want a psi
            time.sleep(0.2)
            actualPSI = float(ser.readline().decode("utf-8"))
            tempData['Actual PSI'] = actualPSI
            print('Actual Pressure: ', actualPSI)
            print('x position: ', position.deltaX)

            print("writing data to CSv")
            writer.writerow(tempData)

            #change the state the robot is in
            deafaultPressure += pressureDelta
            bytesSent = ser.write(floatToCommand(deafaultPressure).encode('utf-8'))
            time.sleep(stabilizationTime)

        except:
            print("There was an issue with NDISensor's parser")


ser.write(floatToCommand(12.25).encode('utf-8'))
print("Done collecting data")
ser.close()
