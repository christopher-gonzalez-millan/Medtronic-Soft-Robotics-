'''
 * @file    arduino_communication_header.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Methods to control Arduino communication
'''
import time
import serial as pys
import atexit
import logging
import serial.tools.list_ports


READ_PRESSURE_CMD = "9999"       # Predefined string to send Arduino to read pressure
SETUP_WAIT_TIME_SEC = 10         # Forcing python to wait 10 seconds before asking the Arduino if connection
                                 # is complete. We want to avoid using the serial line while pressure sensors
                                 # Are being initialized on the arduino side. TODO: Determine if value can be decreased
SETUP_COMPLETE_MSG_LENGTH = 23   # Message length for confirmation sent back by Arduino
MSG_RECIEVED_BY_ARDUINO = "rx"   # Message sent back from Arduino when it has processed a serial message from Python
PRESSURE_RESPONSE_MSG_LENGTH = 4 # Number of bytes expected when Arduino is sending back a pressure value
DEFAULT_PRESSURE_PSI = "1225"    # Pressure near atmospheric

def getPort(deviceName):
    """ use to find port with the given port description

    Parameters
    ----------
    deviceName : string
        name of device

    Returns
    -------
    tuple
        Return tuple where the first element is port name as string, second is device name as string

    Raises
    ------
    NoPortsException
        If the matrix is not numerically invertible.

    """
    COMports = serial.tools.list_ports.comports()

    #check if there is aviable ports else throw exception
    if not COMports:
        raise Exception("Could Not Find Any Ports")

    for port in COMports:
        if deviceName in port.description:
            return (port.device, port.description)

    raise Exception("Could not find device with {} as device name".format(deviceName))

class arduino:
    def __init__(self):
        self.P_act = 12.25
        self.ser = pys.Serial()
        self.startCommunication()

        atexit.register(self.close)

    def startCommunication(self):
        # Open Serial to Arduino
        self.ser.baudrate = 115200
        self.ser.port = getPort('Serial')[0]
        self.ser.open()
        if (self.ser.is_open != True):
            print("Could not open serial")
            quit()

        # Give time for Arduino to use serial line to setup pressure sensors
        print("Waiting for Arduino Initialization...")
        time.sleep(SETUP_WAIT_TIME_SEC)
        while True:
            if(self.ser.in_waiting >= SETUP_COMPLETE_MSG_LENGTH):
                arduinoSetup = self.ser.readline().decode('utf-8')
                print(arduinoSetup)
                if(arduinoSetup.rstrip() == "Arduino Setup Complete"):
                    print("Arduino Serial Established")
                    break

    def getActualPressure(self):
        '''
        Obtains actual pressure from pressure sensor
        '''
        self.ser.write(READ_PRESSURE_CMD.encode('utf-8'))
        while True:
            if(self.ser.in_waiting > 4):
                self.P_act = float(self.ser.readline().decode('utf-8'))  # convert pressures from string to float
                break

        return self.P_act

    def sendDesiredPressure(self, desiredPressure):
        '''
        convert desiredPressure and send this pressure into the Arduino
        '''

        # Turn float pressure into strings of right format to send to Adruino
        desiredPressure = round(desiredPressure, 2) # round desired pressure to 2 decimal points
        lessThanTen = False                         # checks to see if the P_des is less than 10

        if desiredPressure < 10.0:
            lessThanTen = True

        command = str(desiredPressure)     # turn float into a string
        command = command.replace('.', '') # Remove decimal before sending over serial

        # append leading zero to pressures less than 10 psi
        if lessThanTen:
            command = '0' + command

        # add following zero if we are missing the second decimal point
        if len(command) == 3:
            command = command + '0'

        # Send over desired pressure to Arduino
        logging.debug("Writing command to arduino: ", command.encode('utf-8'))
        self.ser.write(command.encode('utf-8'))
        while True:
            if (self.ser.in_waiting == PRESSURE_RESPONSE_MSG_LENGTH):
                if ((self.ser.readline().decode('utf-8')).rstrip() == MSG_RECIEVED_BY_ARDUINO):
                    break

    def close(self):
        # Send command to reset to default pressure before terminating
        self.ser.write(DEFAULT_PRESSURE_PSI.encode('utf-8'))
        self.ser.close()
