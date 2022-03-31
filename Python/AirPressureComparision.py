'''
 * @file    AirPressureComparision.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Data collector of all channel psi's during acutiation of one channel
'''
from NDI_Code.NDISensor import NDISensor
from Py_Arduino_Communication.arduino_control import arduino_control
import threading
from queue import Queue
import ctypes
import time
from tkinter import *
from tkinter import ttk
from ttkthemes import ThemedStyle
import logging
from csv_logger import CsvLogger
from math import sin, pi
from scipy import signal as sg

# create csv logging object to store the data collected
header = ['time', 'theoreticalPSI/Channel0', 'actualPSI/Channel0', 'actualPSI/Channel1', 'actualPSI/Channel2']
csv_logger = CsvLogger(filename='Data Collection/Tracking Curves/data.csv',
                        level=logging.INFO, fmt='%(asctime)s,%(message)s', header=header)

# Init EM Nav and Arduino
# ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)
startTime = time.time()
psi = 12.0
maxPSI = 16

while (psi <= maxPSI):
    #change pressure
    time.sleep(0.5) # seconds
    arduino.sendDesiredPressure(arduino.channel0, channel0)

    time_diff = time.time() - start_time
    # get the actual pressure from the pressure sensor
    psi0 = arduino.getActualPressure(arduino.channel0)
    psi1 = arduino.getActualPressure(arduino.channel1)
    psi2 = arduino.getActualPressure(arduino.channel2)
    print(psi0)
    print(psi1)
    print(psi2)

    #log data
    csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f' % (time_diff, psi, psi0, psi1, psi02))

    psi += 0.1 #increase psi for next channel
