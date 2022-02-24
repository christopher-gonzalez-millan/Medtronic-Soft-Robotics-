'''
 * @file    basic_GUI_Implementation.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Integrates Arduino with EM sensor in one python script.
            Provides GUI with simple commands like read pressure, write pressure, and read position
'''
import NDISensor
import threading
from queue import Queue
import ctypes
import time
import serial as pys
from tkinter import *
from tkinter import ttk

# Globals shared between threads
cmdQueue = Queue()

ser = pys.Serial()
ser.baudrate = 115200
ser.port = 'COM4'

ndi = NDISensor.NDISensor()

# Class used for all commands
class command:
    def __init__(self, id, field1, field2):
        self.id = id
        self.field1 = field1
        self.field2 = field2
    
    def callback(self, func):
        func(self)
        return self


# <===================== Building GUI =====================>
root = Tk()
root.geometry("400x300")
root.title('GUI')
root['bg'] = '#0059b3'

# Handling Arduino related commands from GUI
def GUI_handleArduinoCommand():
    global arduino_entry
    newCmd = command("Arduino", arduino_entry.get(), 0)
    # field1 = 0
    # field2 = 0
    # def cb(self):
    #     field1 = self.field1
    #     field2 = self.field2
    # newCmd.callback(cb)
    cmdQueue.put(newCmd)

labelText=StringVar()
labelText.set("Enter desired pressure [psi]:")
labelDir=Label(root, textvariable=labelText, height=1)
labelDir.pack()

#Create an Entry widget to accept User Input
arduino_entry= Entry(root, width= 20)
arduino_entry.focus_set()
arduino_entry.pack()
ttk.Button(root, text= "Send",width= 10, command=GUI_handleArduinoCommand).pack(pady=20)

def GUI_handlePressureRead():
    newCmd = command("Arduino", "read", 0)
    cmdQueue.put(newCmd)

ttk.Button(root, text= "Read Pressure from Arduino",width= 30, command=GUI_handlePressureRead).pack(pady=20)

# Handling EM Sensor = realted commands from GUI
def GUI_handleEMcommand():
    newCmd = command("EM_Sensor", 0, 0)
    # field1 = 0
    # field2 = 0
    # def cb(self):
    #     field1 = self.field1
    #     field2 = self.field2
    # newCmd.callback(cb)
    cmdQueue.put(newCmd)

ttk.Button(root, text= "Read Position from EM Sensor",width= 30, command=GUI_handleEMcommand).pack(pady=20)
# <==========================================================>

class controllerThread(threading.Thread):
    '''
    Implements proportional controller
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
                # Convert string to utf-8 and send over serial
                command = "9999"
                bytesSent = ser.write(command.encode('utf-8'))
                time.sleep(.1)
                temp = ser.readline().decode("utf-8")
                print("Pressure: %(val)s" % {"val": temp.rstrip()})
            else:
                # Convert string to utf-8 and send over serial
                bytesSent = ser.write(newCmd.field1.encode('utf-8'))
        elif (newCmd.id == "EM_Sensor"):
            # print("controller wants to read position")
            global ndi
            while True:
                position = ndi.getPosition()
                if position:
                    print("Delta Z: ", position.deltaZ)
                    break
    
    def one_D_feedback(self, z_des, z_act, P_act, P_o):
        # define the proportional gain
        k_p = 1

        # Calculate the error between current and desired positions
        epsi_z = z_des - z_act

        # Multiply by the proportional gain k_p
        P_des = k_p * epsi_z

        # < ------- Our feedback method --------- >
        # del_P_des = k_p * epsi_z
        # P_des = P_act + del_P_des

        # < -------- Shalom delta P method ------- >
        # Figure out how to utilize del_P_act instead of P_des (on Arduino side?)
        # del_P_des = k_p*epsi_z
        # P_des = P_o + del_P_des
        # del_P_act = P_des - P_act

        return P_des

    def run(self):
        # target function of the thread class
        try:            
            while True:
                if (cmdQueue.empty() == False):
                    newCmd = cmdQueue.get()
                    self.handleGUICommand(newCmd)
                    # time.sleep(1)
            
        finally:
            global ser
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
        
# < =============================== Distance vector function ================================== >
def distance_calc(x_base, y_base, z_base, x_tip, y_tip, z_tip):
    x_act = x_tip - x_base
    y_act = y_tip - y_base
    z_act = z_tip - z_base
# < =========================================================================================== >

def main():
    '''
    Starting point for script
    '''

    # Open Serial to Arduino
    global ser
    ser = pys.Serial()
    ser.baudrate = 115200
    ser.port = 'COM4'
    ser.open()
    if (ser.is_open != True):
        print("Could not open serial")
        quit()

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