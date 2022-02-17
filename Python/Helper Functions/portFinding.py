# -*- coding: utf-8 -*-
"""
Created on Wed Feb 16 12:30:46 2022

@author: Christopher
"""
import serial
import serial.tools.list_ports


def findPorts():
    """ Used to find all the ports aviable to current device
    
    Returns
    -------
    list of Tuples
        Return tuples with first element is port name as string, second is device name as string
    
    Raises
    ------
    NoPortsException
        If the matrix is not numerically invertible.

    """
    devicesAvailable = []
    COMports = serial.tools.list_ports.comports()
    
    #check if there is aviable ports else throw exception
    if not COMports:
        raise Exception("Could Not Find Any Ports")
        
    for port in COMports:
        devicesAvailable.append((port.device, port.description))
   
    return devicesAvailable

print(findPorts())