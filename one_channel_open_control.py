'''
 * @file    basic_GUI_Implementation.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Integrates Arduino with EM sensor in one python script.
            Provides GUI with simple commands like read pressure, write pressure, and read position
            for one channel robots only
'''
from NDISensor import NDISensor
from arduino_control import arduino_control
import threading
from queue import Queue
import ctypes
import time
import serial as pys
from tkinter import *
from tkinter import ttk

# Init EM Nav and Arduino
ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()

# Queue for inter-thread communication
commandsFromGUI = Queue()

class command:
    '''
    Basic command format to be used in the queue. We pass along
    id's and any other important info in field1 and field2
    ''' 
    def __init__(self, id, field1, field2):
        self.id = id
        self.field1 = field1
        self.field2 = field2


# <===================== Building GUI =====================>
root = Tk()
root.geometry("400x300")
root.title('GUI')
root['bg'] = '#0059b3'

def GUI_handleArduinoCommand():
    '''
    Handling Arduino related commands from GUI
    '''
    global arduino_entry
    newCmd = command("Arduino", float(arduino_entry.get()), 0)
    commandsFromGUI.put(newCmd)

labelText=StringVar()
labelText.set("Enter desired pressure [psi]:")
labelDir=Label(root, textvariable=labelText, height=1)
labelDir.pack()

# Create an Entry widget to accept User Input
arduino_entry= Entry(root, width= 20)
arduino_entry.focus_set()
arduino_entry.pack()
ttk.Button(root, text= "Send",width= 10, command=GUI_handleArduinoCommand).pack(pady=20)

def GUI_handlePressureRead():
    newCmd = command("Arduino", "read", 0)
    commandsFromGUI.put(newCmd)

ttk.Button(root, text= "Read Pressure from Arduino",width= 30, command=GUI_handlePressureRead).pack(pady=20)

# Handling EM Sensor = realted commands from GUI
def GUI_handleEMcommand():
    newCmd = command("EM_Sensor", 0, 0)
    commandsFromGUI.put(newCmd)

ttk.Button(root, text= "Read Position from EM Sensor",width= 30, command=GUI_handleEMcommand).pack(pady=20)
# <==========================================================>

class controllerThread(threading.Thread):
    '''
    Implements open controller for 1D control
    '''
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        if (newCmd.id == "Arduino"):
            global ser
            if (newCmd.field1 == "read"):
                P_act = arduino.getActualPressure()
                print("Current Pressure: ", P_act)
            else:
                P_des = newCmd.field1

                if P_des < 9.0:
                    # lower limit of the pressure we are sending into the controller
                    P_des = 9.0
                elif P_des > 13.25:
                    # higher limit of the pressure we are sending into the controller
                    P_des = 13.25

                print("Setting pressure to : ", P_des)
                arduino.sendDesiredPressure(P_des)

        elif (newCmd.id == "EM_Sensor"):
            print("controller wants to read position")
            global ndi
            while True:
                position = ndi.getPosition()
                if position:
                    print("Delta Z: ", position.deltaZ)
                    break

    def run(self):
        '''
        target function of the thread class
        '''
        try:
            while True:
                if (commandsFromGUI.empty() == False):
                    newCmd = commandsFromGUI.get()
                    self.handleGUICommand(newCmd)
                    time.sleep(.07)
            
        finally:
            # global ser
            print('Controller thread teminated')
            ser.close()

    def get_id(self):
        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
              ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print('Exception raise failure') 

def main():
    '''
    Starting point for script
    '''

    # Spin up controller
    t1 = controllerThread('Thread 1')
    t1.start()

    # Designate main thread for GUI
    root.mainloop()

    # Kill controller once GUI is excited
    t1.raise_exception()
    t1.join()

if __name__ == "__main__":
    main()
