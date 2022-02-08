'''
 * @file    one_channel_prop_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 1D proportional controller
'''
import threading
import ctypes
import serial as pys
import time
from tkinter import *
from tkinter import ttk

# Globals shared between threads
newCommand = False


# <===================== Building GUI =====================>
def display_text():
   global entry
   global userInput
   global newCommand
   userInput = entry.get()
   newCommand = True

root = Tk()
root.geometry("400x200")
root.title('GUI')

labelText=StringVar()
labelText.set("Send Command to Arduino")
labelDir=Label(root, textvariable=labelText, height=4)
labelDir.pack()

#Create an Entry widget to accept User Input
entry= Entry(root, width= 20)
entry.focus_set()
entry.pack()

ttk.Button(root, text= "Send",width= 10, command= display_text).pack(pady=20)
# <==========================================================>

class controllerThread(threading.Thread):
    '''
    Implements proportional controller
    '''
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def handleGUICommand(self):
        global newCommand
        global userInput 
        print(userInput)
        newCommand = False

    def run(self):
        # target function of the thread class
        try:
            global newCommand
            global userInput
            
            while True:
                if newCommand is True:
                    self.handleGUICommand()
                    # time.sleep(1)
            
        finally:
            print('Controller thread teminated')
          
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
        
# < =============================== 1D Feedback Algorithm ===================================== >
def one_D_feedback(z_des, z_act, P_act, P_o):
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

# < ========================================================================================= >

def main():
    '''
    Starting point for script
    '''

    # Open Serial to Arduino
    # ser = pys.Serial()
    # ser.baudrate = 115200
    # ser.port = 'COM4'
    # ser.open()
    # if (ser.is_open != True):
    #     print("Could not open serial")
    #     quit()

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