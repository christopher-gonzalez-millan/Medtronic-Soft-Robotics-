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
from tkinter import *
from tkinter import ttk
from ttkthemes import ThemedStyle
import logging
from csv_logger import CsvLogger
from math import sin, pi
from scipy import signal as sg
import numpy as np
import random

logging.basicConfig(filename = 'data.log', level = logging.WARNING, 
    format = '%(asctime)s,%(message)s')


# Init Arduino
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)

P_des = np.array([12.25, 12.25, 12.25])     # desired pressure we're sending to the Arduino (c0, c1, c2)
P_act = np.array([0.0, 0.0, 0.0])           # actual pressure read from the pressure sensor (c0, c1, c2)

# Queue for inter-thread communication (between GUI thread and controller thread)
commandsFromGUI = Queue()

class command:
    '''
    Basic command format to be used in the queue. We pass along
    id's and any other important info in field1 and field2
    '''
    def __init__(self, id, field1, field2, field3):
        self.id = id
        self.field1 = field1
        self.field2 = field2
        self.field3 = field3

class GUI:
    '''
    Class for building GUI
    '''
    def __init__(self, master):
        self.master = master
        master.title('GUI')
        master.geometry("400x350")
        master['bg'] = '#474747'

        # <=== ROW 0 ===>
        command_label = ttk.Label(master, text = "Send Commands")
        command_label.grid(row = 0, column = 0, sticky = W, pady = 2, padx = (2,0))

        # <=== ROW 1 ===>
        # Text label for position entry
        channel0_label = ttk.Label(master, text = "Channel 0 Pressure:")
        channel0_label.grid(row = 1, column = 0, sticky = W, pady = 2, padx = (30,0))
        # Entry widget for position
        self.channel0_entry= ttk.Entry(master, width= 10)
        self.channel0_entry.grid(row = 1, column = 1, sticky = W, pady = 2)
        self.channel0_entry.bind("<Return>", self.GUI_setChannel0)

        # <=== ROW 2 ===>
        # Text label for proportional gain entry
        channel1_label = ttk.Label(master, text = "Channel 1 Pressure:")
        channel1_label.grid(row = 2, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.channel1_entry = ttk.Entry(master, width= 10)
        self.channel1_entry.grid(row = 2, column = 1, sticky = W, pady = 2)
        self.channel1_entry.bind("<Return>", self.GUI_setChannel1)

        # <=== ROW 3 ===>
        # Text label for integral gain entry
        channel2_label = ttk.Label(master, text = "Channel 2 Pressure:")
        channel2_label.grid(row = 3, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.channel2_entry = ttk.Entry(master, width= 10)
        self.channel2_entry.grid(row = 3, column = 1, sticky = W, pady = 2)
        self.channel2_entry.bind("<Return>", self.GUI_setChannel2)

        # Diplaying data
        display_label = ttk.Label(master, text = "Press/hold enter to display")
        display_label.grid(row = 4, column = 0, sticky = W, pady = (20,2), padx = (2,0))
        self.display_entry = ttk.Entry(master, width= 1)
        self.display_entry.grid(row = 4, column = 1, sticky = W, pady = (20,2))
        self.display_entry.bind("<Return>", self.GUI_handleDataDisplay)

        self.p_act_0_label = ttk.Label(master, text = "P0: ")
        self.p_act_0_label.grid(row = 5, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.p_act_1_label = ttk.Label(master, text = "P1: ")
        self.p_act_1_label.grid(row = 6, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.p_act_2_label = ttk.Label(master, text = "P2: ")
        self.p_act_2_label.grid(row = 7, column = 0, sticky = W, pady = 2, padx = (30,0))

        # Record data
        command_label = ttk.Label(master, text = "Data Recording")
        command_label.grid(row = 10, column = 0, sticky = W, pady = (20,2), padx = (2,0))

        start_log_button = ttk.Button(master, text = "Start Logging", width = 12, command = lambda: self.GUI_handleLoggingCommand("start"))
        start_log_button.grid(row = 11, column = 0, sticky = W, pady = 2, padx = (2,0))

        stop_log_button = ttk.Button(master, text = "Stop Logging", width = 12, command = lambda: self.GUI_handleLoggingCommand("stop"))
        stop_log_button.grid(row = 11, column = 1, sticky = W, pady = 2, padx = (2,0))

        clear_log_button = ttk.Button(master, text = "Clear Log File", width = 12, command = lambda: self.GUI_handleLoggingCommand("clear"))
        clear_log_button.grid(row = 11, column = 2, sticky = W, pady = 2, padx = (50,0))

    def GUI_setChannel0(self, *args):
        '''
        Handle setting the channel in pressure 0
        '''
        newCmd = command("Arduino", "setPressure", 0, float(self.channel0_entry.get())) # arduino.channel0
        commandsFromGUI.put(newCmd)

    def GUI_setChannel1(self, *args):
        '''
        Handle setting the channel in pressure 1
        '''
        newCmd = command("Arduino", "setPressure", 1, float(self.channel1_entry.get())) # arduino.channel1
        commandsFromGUI.put(newCmd)
    
    def GUI_setChannel2(self, *args):
        '''
        Handle setting the channel in pressure 2
        '''
        newCmd = command("Arduino", "setPressure", 2, float(self.channel2_entry.get())) # arduino.channel2
        commandsFromGUI.put(newCmd)

    def GUI_handleDataDisplay(self, *args):
        '''
        Display control algorithm parameters to the GUI
        '''
        global P_act

        self.p_act_0_label.configure(text = "P0: " + str(round(P_act[0],3)))
        self.p_act_1_label.configure(text = "P1: " + str(round(P_act[1],3)))
        self.p_act_2_label.configure(text = "P2: " + str(round(P_act[2],3)))

    def GUI_handleLoggingCommand(self, status):
        '''
        Start logging
        '''
        global start_time

        if (status == "start"):
            logging.getLogger().setLevel(logging.INFO)
            start_time = time.time()                    # start time for ramp and sinusoid signals
        elif (status == "stop"):
            logging.getLogger().setLevel(logging.WARNING)
            start_time = 0
        elif (status == "clear"):
            # Clear contents of log file
            with open('data.log', 'w'):
                pass

class controllerThread(threading.Thread):
    '''
    This thread is where the main controller is run
    for the open controller 
    '''
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def run(self):
        '''
        Infinite loop for controller until turned off.
        Continues to look for new commands from the GUI.
        and runs the three channel main.
        '''
        try:
            while True:
                # Look for new commands
                if (commandsFromGUI.empty() == False):
                    newCmd = commandsFromGUI.get()
                    self.handleGUICommand(newCmd)

                self.three_channel_main()
                # Slow down controller so we give the arduino some
                # time to relax. Don't want to be sending serial commands
                # every loop the arduino executes
                time.sleep(.07)

        finally:
            print('Controller thread teminated')

    def three_channel_main(self):
        '''
        main function used in thread to perform 3 channel control
        '''
        global P_des, P_act, r_act, csv_logger

        # Safety check so we don't the arduino a super high or low pressure!
        # it will blow up if you have the wrong bounds
        for channel in range(3): 
            if P_des[channel] < 9.0:
                # lower limit of the pressure we are sending into the controller
                P_des[channel] = 9.0
            elif P_des[channel] > 16.0:
                # higher limit of the pressure we are sending into the controller
                P_des[channel] = 16.0

        # get the actual pressure from the pressure sensor
        P_act[0] = arduino.getActualPressure(arduino.channel0)
        P_act[1] = arduino.getActualPressure(arduino.channel1)
        P_act[2] = arduino.getActualPressure(arduino.channel2)

        # send the desired pressure into Arduino
        arduino.sendDesiredPressure(arduino.channel0, float(P_des[0]))
        arduino.sendDesiredPressure(arduino.channel1, float(P_des[1]))
        arduino.sendDesiredPressure(arduino.channel2, float(P_des[2]))

    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        global P_des
        
        if (newCmd.id == "Arduino"):
            if (newCmd.field1 == "setPressure"):
                P_des[newCmd.field2] = newCmd.field3

    def get_id(self):
        '''
        returns id of the respective thread
        '''
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        '''
        raise exception for controller thread
        '''
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
    # Spin up controller thread
    cThread = controllerThread('Thread 1')
    cThread.start()

    # Designate main thread to GUI
    root = Tk()
    style = ThemedStyle(root)
    style.set_theme("equilux")
    GUI(root)
    root.mainloop()

    # Kill controller once GUI is exited
    cThread.raise_exception()
    cThread.join()

if __name__ == "__main__":
    main()
