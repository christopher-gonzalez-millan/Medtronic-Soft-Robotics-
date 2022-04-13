'''
 * @file    one_channel_PI_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 1D proportional controller
'''
from NDI_Code.NDISensor import NDISensor
from Py_Arduino_Communication.arduino_control import arduino_control
import threading
from queue import Queue
import ctypes
import time
import logging
from csv_logger import CsvLogger
from math import sin, pi
from scipy import signal as sg

# Init EM Nav and Arduino
# ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)

while (True):

    if (input("Command: ") == "R"):
        print(arduino.getActualPressure(arduino.channel0))
        print(arduino.getActualPressure(arduino.channel1))
        print(arduino.getActualPressure(arduino.channel2))
    elif (input("Command: ") == "S"):
        channel0 = input("Channel0: ")
        if (channel0 != 0):
            if (float(channel0) >= 16):
                channel0 = 16
            arduino.sendDesiredPressure(arduino.channel0, float(channel0))

        channel1 = input("Channel1: ")
        if (channel1 != 0):
            if (float(channel1) >= 16):
                channel1 = 16
            arduino.sendDesiredPressure(arduino.channel1, float(channel1))

        channel2 = input("Channel2: ")
        if (channel2 != 0):
            if (float(channel2) >= 16):
                channel2 = 16
            arduino.sendDesiredPressure(arduino.channel2, float(channel2))

        # get the actual pressure from the pressure sensor
        print(arduino.getActualPressure(arduino.channel0))
        print(arduino.getActualPressure(arduino.channel1))
        print(arduino.getActualPressure(arduino.channel2))
