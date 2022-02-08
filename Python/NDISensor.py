# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

'''
 * @file    em_sensor_api.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Methods to control EM sensor via the NDI capi API
'''

#TODO: conditional import in class
import ndicapy

from ndicapy import (
    ndiDeviceName, ndiProbe, NDI_OKAY,
    ndiOpen, ndiClose, ndiCommand, ndiGetError,
    ndiErrorString, NDI_115200,
    NDI_8N1, NDI_NOHANDSHAKE,
)

# Class used to send cleaned parsed data
class parsedReply:
    def __init__(self, deltaX, deltaY, deltaZ):
        self.deltaX = deltaX
        self.deltaY = deltaY
        self.deltaZ = deltaZ

class NDISensor:
    #TODO: create better constructor
    #TODO: create deconstructor 
    
    def __init__(self):
        self.name = None
        self.device = None
        
        self.findPorts()
        self.deviceSetup()
        self.checkCommunication()
        self.setCommunication()
        self.setupPort()
        self.startTracking()
    
    def __del__(self):
        self.stopTracking()
        self.closeDevice()
                
    #TODO: change to dynamic sensing current computer instead of chekcin all possible ports. 
    def findPorts(self):
        # Number ports to check
        MAX_SERIAL_PORTS = 20
        name = ''
        
        for port_no in range(MAX_SERIAL_PORTS):
            name = ndiDeviceName(port_no)
            
            if not name:
                continue
                
            result = ndiProbe(name) #Returns 257 if port not found, otherwise returns 1
            
            if result == NDI_OKAY: #NDI_OKAY == 0
                print(port_no)
                break
                
        if result != NDI_OKAY:
            raise IOError(
                'Could not find any NDI device in '
                '{} serial port candidates checked. '
                'Please check the following:\n'
                '\t1) Is an NDI device connected to your computer?\n'
                '\t2) Is the NDI device switched on?\n'
                '\t3) Do you have sufficient privilege to connect to '
                'the device? (e.g. on Linux are you part of the "dialout" '
                'group?)'.format(MAX_SERIAL_PORTS)
            )
    
    def setupPort(self):
        num_setup_complete = 0
        while(num_setup_complete < 2):
            To_Free = ndiCommand(self. device, 'PHSR:01')
        
            if(To_Free == '00'):
                To_Initialized = ndiCommand(self.device, 'PHSR:02')
        
                if(To_Initialized == '00'):
                    To_enable = ndiCommand(self.device, 'PHSR:03')
                    print(To_enable[2:4])
                    if(To_enable == '00'):
                        break
                    else:
                        if(To_enable[2:4] == '0B'):
                            ndiCommand(self. device, 'PENA:{}{}'.format(To_enable[2:4], 'S'))
                        else:
                            ndiCommand(self.device, 'PENA:{}{}'.format(To_enable[2:4], 'D'))
                        
                        print("Success:")
                        print(ndiCommand(self.device, 'PHINF:{}{}'.format(To_enable[2:4], '0001')))
                        num_setup_complete += 1
                else:
                    #There needs to be ports to be initialized
                    ndiCommand(self. device, 'PINIT:{}'.format(To_Initialized[2:4]))
            else:
                #ports need to be freed
                print("free ports please")

    def deviceSetup(self):
        self.device = ndiOpen(self.name) #seems like this command starts communication for API 
        if not self.device:
            raise IOError(
                'Could not connect to NDI device found on '
                '{}'.format(self.name)
            )
            
    #TODO: change to allow diffrent configuration.        
    def setCommunication(self):
        # Sets serial communication settings
        reply = ndiCommand(
            self.device,
            'COMM:{:d}{:03d}{:d}'.format(NDI_115200, NDI_8N1, NDI_NOHANDSHAKE)
        )
        print(reply)

    def checkCommunication(self):
        #Ensures the system configuuration was determine successfully
        reply = ndiCommand(self.device, 'INIT:') 
        error = ndiGetError(self.device)
        if reply.startswith('ERROR') or error != NDI_OKAY:
            raise IOError(
                'Error when sending command: '
                '{}'.format(ndiErrorString(error))
            )
        print(reply)
                
    def startTracking(self):
        '''
        Funciton to start tracking with ports currently set up
        '''
        reply = ndiCommand(self.device, 'TSTART:')
        if reply != NDI_OKAY:
            raise IOError('Error starting tracking')


    def stopTracking(self):
        '''
        Funciton to stop tracking with ports currently set up
        '''
        ndiCommand(self.device, 'TSTOP:')
    
    
    def closeDevice(self):
        '''
        Close out communication with opened device
        '''
        ndiCommand(self.device, 'TSTOP:')
    
    
    def getPosition(self):
        '''
        Recieve tool transformation from EM sensor
        '''
        reply = ndiCommand(self.device, 'TX:')
        return self.parser(reply)    
      
    #TODO: figure out regex for easier debugging  
    def parser(self, reply):
        '''
        Handles conversion of raw data from EM sensor into cleaned class
        '''
        numHandles = reply[0:2]
        print("numHandles:" + numHandles)
    
        # <== Handle 1 ==>
        # handle1 = reply[2:4]
        # print("\nHandle1: " + handle1)
    
        # Tool positioning
        Tx1 = float(reply[29:33] + "." + reply[33:35]); 
        if (reply[28] == "-"):
            Tx1 *= -1
    
        Ty1 = float(reply[36:40] + "." + reply[40:42]); 
        if (reply[35] == "-"):
            Ty1 *= -1
            
        Tz1 = float(reply[43:47] + "." + reply[47:49]); 
        if (reply[42] == "-"):
            Tz1 *= -1
    
        # print("\tTx: " + str(Tx) + "\tTy: " + str(Ty) + "\tTz:" + str(Tz))
    
        # <== Handle 2 ==>
        # handle2 = reply[71:73]; 
        # print("\nHandle2: " + handle2)
    
        # Tool positioning
        Tx2 = float(reply[98:102] + "." + reply[102:104])
        if (reply[97] == "-"):
            Tx2 *= -1
    
        Ty2 = float(reply[105:109] + "." + reply[109:111])
        if (reply[104] == "-"):
            Ty2 *= -1
            
        Tz2 = float(reply[112:116] + "." + reply[116:118])
        if (reply[111] == "-"):
            Tz2 *= -1
    
        # print("\tTx: " + str(Tx) + "\tTy: " + str(Ty) + "\tTz:" + str(Tz))
    
        return parsedReply((Tx2 - Tx1), (Ty2 - Ty1), (Tz2 - Tz1))