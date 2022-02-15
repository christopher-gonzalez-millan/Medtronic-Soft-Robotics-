'''
 * @file    one_channel_prop_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 1D proportional controller
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
root.geometry("300x200")
root.title('GUI')
root['bg'] = '#0059b3'

# <=============== Setting desired position ===============>
def GUI_handleSetPositionCommand():
    global position_entry
    newCmd = command("EM_Sensor", "setPosition", float(position_entry.get()))
    # field1 = 0
    # field2 = 0
    # def cb(self):
    #     field1 = self.field1
    #     field2 = self.field2
    # newCmd.callback(cb)
    cmdQueue.put(newCmd)

labelText=StringVar()
labelText.set("Enter desired Z [mm]:")
labelDir=Label(root, textvariable=labelText, height=1)
labelDir.pack()

#Create an Entry widget to accept User Input
position_entry= Entry(root, width= 20)
position_entry.focus_set()
position_entry.pack()
ttk.Button(root, text= "Send",width= 10, command=GUI_handleSetPositionCommand).pack(pady=(0,40))

# <=============== Setting Gain ===============>
def GUI_handleSetGainCommand():
    global gain_entry
    newCmd = command("EM_Sensor", "setGain", float(gain_entry.get()))
    # field1 = 0
    # field2 = 0
    # def cb(self):
    #     field1 = self.field1
    #     field2 = self.field2
    # newCmd.callback(cb)
    cmdQueue.put(newCmd)

labelText_gain=StringVar()
labelText_gain.set("Enter Gain:")
labelDir_gain=Label(root, textvariable=labelText_gain, height=1)
labelDir_gain.pack()

#Create an Entry widget to accept User Input
gain_entry= Entry(root, width= 10)
gain_entry.focus_set()
gain_entry.pack()
ttk.Button(root, text= "Send",width= 10, command=GUI_handleSetGainCommand).pack(pady=(0,20))


# <==========================================================>

class controllerThread(threading.Thread):
    '''
    Implements proportional controller
    '''
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name
        self.z_des = 0.0                  # stores the desired z position input by user
        self.z_act = 0.0                  # actual z_position from EM sensor
        self.k_p = 1.0                    # proportional controller gain
        self.P_act = 0.0                  # actual pressure read from the pressure sensor
        self.P_des = 12.0                 # desired pressure we're sending to the Arduino

    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        if (newCmd.id == "EM_Sensor"):
            if (newCmd.field1 == "setPosition"):
                self.z_des = newCmd.field2
                print("\nCommand recieved to set position to ", self.z_des)
            elif (newCmd.field1 == "setGain"):
                self.k_p = newCmd.field2
                print("\nCommand recieved to set gain to", self.k_p)
    
    def one_D_main(self):
        '''
        main function used in thread to perform 1D algorithm
        '''
        print("\n")
        # get the actual pressure from the pressure sensor
        self.getActualPressure()

        # get actual position from EM sensor
        self.getActualPosition()
            
        # perform 1D proportional control
        self.one_D_algorithm()

        # send the desired pressure into Arduino
        self.sendDesiredPressure()

    def getActualPressure(self):
        '''     
        Obtains actual pressure from pressure sensor
        '''
        global ser
        command = "9999"                             # let the Arduino know we want to read a pressure value
        bytesSent = ser.write(command.encode('utf-8'))
        time.sleep(0.1)
        self.P_act = float(ser.readline().decode('utf-8'))  # convert pressures from string to float
        print("P_act: ", self.P_act)

    def getActualPosition(self):
        '''
        Obtains actual position reading from EM sensor
        '''
        global ndi
        while True:
            position = ndi.getPosition()
            if position:
                self.z_act = position.deltaX
                break
        print("z_act: ", self.z_act)

    def one_D_algorithm(self):
        '''
        Proportional feedback loop algorithm (includes our method and Shalom's del P)
        '''
        print("z_des: ", self.z_des)
        # TODO: Figure out to obtain z_des and k_p from the user side of the code
        # Calculate the error between current and desired positions
        epsi_z = self.z_des - self.z_act

        # Multiply by the proportional gain k_p
        self.P_des = self.k_p * epsi_z
        print("P_des: ", self.P_des)

        # < ------- Our feedback method --------- >
        # del_P_des = k_p * epsi_z
        # P_des = P_act + del_P_des

        # < -------- Shalom delta P method ------- >
        # Figure out how to utilize del_P_act instead of P_des (on Arduino side?)
        # del_P_des = k_p*epsi_z
        # P_des = P_o + del_P_des
        # del_P_act = P_des - P_act
    
    def sendDesiredPressure(self):
        '''
        convert P_des and send this pressure into the Arduino
        '''
        # lower limit of the pressure we are sending into the controller
        if self.P_des < 9.0:
            self.P_des = 9.0

        # higher limit of the pressure we are sending into the controller
        if self.P_des > 13.0:
            self.P_des = 13.0      
        
        # Turn float pressure into strings of right format to send to Adruino
        self.P_des = round(self.P_des, 2)       # round desired pressure to 2 decimal points
        lessThanTen = False                     # checks to see if the P_des is less than 10

        if self.P_des < 10.0:
            lessThanTen = True

        command = str(self.P_des)               # turn float into a string
        command = command.replace('.', '')

        if lessThanTen:                         # append leading zero to pressures less than 10 psi
            command = '0' + command

        if len(command) == 3:                   # add following zero if we are missing the second decimal point
            command = command + '0'

        # Send over desired pressure to Arduino
        global ser
        print("Writing command to arduino: ", command.encode('utf-8'))
        bytesSent = ser.write(command.encode('utf-8'))

    def run(self):
        # target function of the thread class
        print("Waiting for Arduino Initialization...")
        time.sleep(6)

        try:            
            while True:
                if (cmdQueue.empty() == False):
                    newCmd = cmdQueue.get()
                    self.handleGUICommand(newCmd)
                
                self.one_D_main() 
                # time.sleep(.1)

        finally:
            global ser
            command = "1225"
            bytesSent = ser.write(command.encode('utf-8'))
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