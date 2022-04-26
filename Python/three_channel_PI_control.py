'''
 * @file    three_channel_PI_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 2D proportional and/or PID controller
            used for 3 channel robots
'''
from cmath import cos
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
from math import sin, pi, sqrt, cos
from scipy import signal as sg
import scipy.optimize as sp_opt
import numpy as np

# Data Collection
logging.basicConfig(filename = 'data.log', level = logging.WARNING, 
    format = '%(asctime)s,%(message)s')
header = ['date', 'sample_num', 'time_diff', 'z_des', 'x_des', 'z_act', 'x_act', 'P_des[0]', 'P_des[1]', 'P_des[2]', 'P_act[0]', 'P_act[1]', 'P_act[2]']
csv_logger = CsvLogger(filename='Data Collection/Tracking Curves/data.csv',
                        level=logging.INFO, fmt='%(asctime)s,%(message)s', header=header)
sample_num = 0          # variable to keep track of the samples for any data collection

# Init EM Nav and Arduino
ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)

# Times used for running predefined sequences like tracking a circle
start_time = 0      # start time for the circle signals
time_diff = 0       # time difference betweeen the start and current times

# <== 2X Robot Parameters ==>
'''
These gain values have been tuned for the 2x robots
that we were made. It can vary for all the different robots, 
especially the max pressure. Last value is bound for int sum.
This is where we reset the integral term to 0 to avoid windup
'''
k_p = np.array([.04, .04, .04])
k_i = np.array([0.01, 0.01, 0.01]) 
k_d = np.array([0.001, 0.001, 0.001])
max_pressure = np.array([16, 16, 16])
# int_sum (max, min) = (10, -10)

# <== 1X Robot Parameters ==>
'''
These gain values have been tuned for the 2x robots
that we were made. It can vary for all the different robots, 
especially the max pressure. Last value is bound for int sum.
This is where we reset the integral term to 0 to avoid windup
'''
# k_p = np.array([.03, .03, .03])
# k_i = np.array([0.01, 0.01, 0.01]) 
# k_d = np.array([0.001, 0.001, 0.001])
# max_pressure = np.array([15.5, 15.2, 15.5])
# int_sum = (max, min) = (5, -5)

# Parameters for the 3 channel controller
'''
These values are all stored in array format because we have 3
channels. Each index corresponds to channel 0, channel 1, and
channel 2, respectively.

You will note that the coordinates for the controller are specified in (z,x)
format. This is a byproduct of how we oriented the EM field generator and our robot.
We are using global positioning in our control system, however it should be converted in the future
to relative positioning. With global, if you orient the robot or the EM in the wrong way, the
controllers predictions will not work and the robot will get sent in entirely the wrong direction.
'''
P_des = np.array([12.25, 12.25, 12.25])     # desired pressure we're sending to the Arduino (c0, c1, c2)
P_act = np.array([0.0, 0.0, 0.0])           # actual pressure read from the pressure sensor (c0, c1, c2)
r_des = np.array([0.0, 0.0])                # desired position of robot in form (z, x)
r_act = np.array([0.0, 0.0])                # actual position of the robot using EM sensor (z, x)
dT = np.array([0.125, 0.125, 0.125])        # estimated time between cycles (seconds)
int_sum = np.array([0.0, 0.0, 0.0])         # sum of the integral term
err_r = np.array([0.0, 0.0])                # error between measured position and actual position
epsi = np.array([0.0, 0.0, 0.0])            # stores the solution to the force vector algorithm
epsi_prev = np.array([0.0, 0.0, 0.0])       # modified error in r (after force vector solution) for the previous time step # TODO: figure out if this should be a global value

# Queue for inter-thread communication (between GUI thread and controller thread)
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
        x_position_label = ttk.Label(master, text = "Enter desired X [mm]:")
        x_position_label.grid(row = 1, column = 0, sticky = W, pady = 2, padx = (30,0))
        # Entry widget for position
        self.x_position_entry= ttk.Entry(master, width= 10)
        self.x_position_entry.grid(row = 1, column = 1, sticky = W, pady = 2)
        self.x_position_entry.bind("<Return>", self.GUI_handleSetXPositionCommand)

        # <=== ROW 2 ===>
        y_position_label = ttk.Label(master, text = "Enter desired Y [mm]:")
        y_position_label.grid(row = 2, column = 0, sticky = W, pady = 2, padx = (30,0))
        # Entry widget for positionx_position_label
        self.z_position_entry= ttk.Entry(master, width= 10)
        self.z_position_entry.grid(row = 2, column = 1, sticky = W, pady = 2)
        self.z_position_entry.bind("<Return>", self.GUI_handlesetZPositionCommand)

        # <=== ROW 3 ===>
        # Text label for proportional gain entry
        kp_label = ttk.Label(master, text = "Enter kp:")
        kp_label.grid(row = 3, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.kp_entry = ttk.Entry(master, width= 10)
        self.kp_entry.grid(row = 3, column = 1, sticky = W, pady = 2)
        self.kp_entry.bind("<Return>", self.GUI_handleSetKpCommand)

        # <=== ROW 4 ===>
        # Text label for integral gain entry
        ki_label = ttk.Label(master, text = "Enter ki:")
        ki_label.grid(row = 4, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.ki_entry = ttk.Entry(master, width= 10)
        self.ki_entry.grid(row = 4, column = 1, sticky = W, pady = 2)
        self.ki_entry.bind("<Return>", self.GUI_handleSetKiCommand)

        # <=== ROW 5 ===>
        # Text label for derivative gain entry
        kd_label = ttk.Label(master, text = "Enter kd:")
        kd_label.grid(row = 5, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.kd_entry = ttk.Entry(master, width= 10)
        self.kd_entry.grid(row = 5, column = 1, sticky = W, pady = 2)
        self.kd_entry.bind("<Return>", self.GUI_handleSetKdCommand)

        # Diplaying data
        display_label = ttk.Label(master, text = "Press/hold enter to display")
        display_label.grid(row = 6, column = 0, sticky = W, pady = (20,2), padx = (2,0))
        self.display_entry = ttk.Entry(master, width= 1)
        self.display_entry.grid(row = 6, column = 1, sticky = W, pady = (20,2))
        self.display_entry.bind("<Return>", self.GUI_handleDataDisplay)

        self.x_des_label = ttk.Label(master, text = "X desired: ")
        self.x_des_label.grid(row = 7, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.x_act_label = ttk.Label(master, text = "X actual: ")
        self.x_act_label.grid(row = 8, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.z_des_label = ttk.Label(master, text = "Z desired: ")
        self.z_des_label.grid(row = 9, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.z_act_label = ttk.Label(master, text = "Z actual: ")
        self.z_act_label.grid(row = 10, column = 0, sticky = W, pady = 2, padx = (30,0))

        # Record data
        command_label = ttk.Label(master, text = "Data Recording")
        command_label.grid(row = 11, column = 0, sticky = W, pady = (20,2), padx = (2,0))

        start_log_button = ttk.Button(master, text = "Start Logging", width = 12, command = lambda: self.GUI_handleLoggingCommand("start"))
        start_log_button.grid(row = 12, column = 0, sticky = W, pady = 2, padx = (2,0))

        stop_log_button = ttk.Button(master, text = "Stop Logging", width = 12, command = lambda: self.GUI_handleLoggingCommand("stop"))
        stop_log_button.grid(row = 12, column = 1, sticky = W, pady = 2, padx = (2,0))

        clear_log_button = ttk.Button(master, text = "Clear Log File", width = 12, command = lambda: self.GUI_handleLoggingCommand("clear"))
        clear_log_button.grid(row = 12, column = 2, sticky = W, pady = 2, padx = (50,0))

    def GUI_handleSetXPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        newCmd = command("GuiToController", "setXPosition", float(self.x_position_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handlesetZPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        newCmd = command("GuiToController", "setZPosition", float(self.z_position_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetKpCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("GuiToController", "setKp", float(self.kp_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetKiCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("GuiToController", "setKi", float(self.ki_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetKdCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("GuiToController", "setKd", float(self.kd_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleDataDisplay(self, *args):
        '''
        Display control algorithm parameters to the GUI. When the GUI is open, 
        you can press and hold enter to display all the values
        '''
        global r_des, r_act, y_des, y_act, int_sum

        self.x_des_label.configure(text = "X desired: " + str(round(r_des[1],3)))
        self.x_act_label.configure(text = "X actual: " + str(round(r_act[1],3)))
        self.z_des_label.configure(text = "Z desired: " + str(round(r_des[0],3)))
        self.z_act_label.configure(text = "Z actual: " + str(round(r_act[0],3)))

    def GUI_handleLoggingCommand(self, status):
        '''
        This function handles starting of the logging. Starting the logging
        sequence also starts running a predefined trajectory like a circle.
        You can uncomment the circle_signal call if you want to record data
        while still maintaining manual control
        '''
        global start_time, sample_num

        if (status == "start"):
            # Change logging level to INFO on start in order to start
            # outputting the logging commands
            logging.getLogger().setLevel(logging.INFO)
            start_time = time.time()
            sample_num = 0
        elif (status == "stop"):
            # Only display error level messages when you are not running
            # the logger
            logging.getLogger().setLevel(logging.WARNING)
            start_time = 0
        elif (status == "clear"):
            # Wipe the contents of the log file
            with open('data.log', 'w'):
                pass

class controllerThread(threading.Thread):
    '''
    This thread is where the main controller is run.
    This is where our PID controller lives
    '''
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def run(self):
        '''
        Infinite loop for controller until turned off.
        Continues to look for new commands from the GUI
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

    def circle_signal(self):
        '''
        Function will send position commands to follow a circle
        '''
        global start_time, r_des, time_diff

        current_time = time.time()              # current time compared to start time    

        time_diff = current_time - start_time   # time difference used in the signal
        
        radius = 15                             # radius of the circle in mm

        T = 60                                  # period of the circle (in seconds)
        center = np.array([0, 0])           # center of the circle script (z, x)       

        # parametric equations that represent circle as a function of time
        r_des[0] = center[0] + radius*cos((2*pi/T)*time_diff)        # in z
        r_des[1] = center[1] + radius*sin((2*pi/T)*time_diff)        # in x

    def fig_eight_signal(self):
        '''
        Function will try to send robot in a figure 8
        '''
        global start_time, r_des, time_diff

        current_time = time.time()              # current time compared to start time    

        time_diff = current_time - start_time   # time difference used in the signal
        
        radius = 15                             # "radius" of the figure eight in mm

        T = 60                                  # period of the figure eight (in seconds)
        center = np.array([0, 0])           # center of the figure eight script (z, x)       

        # parametric equations that represent figure eight as a function of time
        r_des[0] = center[0] + radius*sin((2*pi/T)*time_diff)                                # in z
        r_des[1] = center[1] + radius*sin((2*pi/T)*time_diff)*cos((2*pi/T)*time_diff)        # in x

    def three_channel_main(self):
        '''
        main function used in thread to perform 3 channel algorithm
        '''
        global time_diff, r_des, r_act, P_des, P_act, csv_logger, sample_num

        # get the actual pressure from the pressure sensor
        P_act[0] = arduino.getActualPressure(arduino.channel0)
        P_act[1] = arduino.getActualPressure(arduino.channel1)
        P_act[2] = arduino.getActualPressure(arduino.channel2)

        # get actual position from EM sensor
        position = ndi.getPositionInRange()
        r_act[1] = position.deltaX          # x dim
        r_act[0] = position.deltaZ          # y dim
        # print("r_act[0]: " + str(r_act[0]) + "r_act[1]: " + str(r_act[1]))

        # perform 3 channel control algorithm
        self.three_channel_algorithm()

        # send the desired pressure into Arduino
        self.sendDesiredPressure()

        # Log all control variables if needed
        logging.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % \
            (sample_num, time_diff, r_des[0], r_des[1], r_act[0], r_act[1], P_des[0], P_des[1], P_des[2], P_act[0], P_act[1], P_act[2]))
        if logging.getLogger().getEffectiveLevel() == logging.INFO:
            csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % \
                (sample_num, time_diff, r_des[0], r_des[1], r_act[0], r_act[1], P_des[0], P_des[1], P_des[2], P_act[0], P_act[1], P_act[2]))

        # update sample number for next data point
        sample_num = sample_num + 1

    def three_channel_algorithm(self):
        '''
        Proportional/PID feedback loop algorithm (vector based solution -- includes dot product and bounded least squares solution)
        '''
        global r_des, r_act, err_r, P_des, P_act, k_p, k_i, k_d, dT, int_sum, epsi, epsi_prev, start_time

        # If user has started logging, start the circle signal. You could
        # also run the figure 8 here if you would like.
        if start_time > 0:
            self.circle_signal()

        # Calculate the error between current and desired positions
        err_r = r_des - r_act

        # perform force vector calculations
        self.forceVectorCalc()

        # Calculate the integral sum for integral control
        int_sum = int_sum + 0.5*(epsi + epsi_prev)*dT   # these are all element-wise operations
        for i in range(len(int_sum)):                   # checks the integral sum to see if it's in range (hardcoded integral windup)
            if int_sum[i] > 10:
                int_sum[i] = 10
            elif int_sum[i] < -10:
                int_sum[i] = -10
        
        # Calculate the derivative term of the controller
        deriv = (epsi - epsi_prev)/dT

        # < ------- Feedback controller --------- >
        del_P_des = k_p*epsi + k_i*(int_sum) + k_d*deriv    # should be element-wise operations
        P_des = P_act + del_P_des

        # Update the error value for next iteration of epsi_prev
        epsi_prev = epsi

    def forceVectorCalc(self):
        '''
        calculates the force vector solution for the controller given the error vector and the unit vectors of each channel
        '''
        global err_r, epsi

        # <----- Bounded least squares implementation ----->
        # array that contains C1, C2, C3 unit vectors
        A = np.array([[sqrt(3)/2, -sqrt(3)/2, 0], [1/2, 1/2, -1]])

        # perform unbounded least squares
        sol = sp_opt.lsq_linear(A, err_r)

        # return the solution to the optimization (m, n, p)
        epsi = sol.x
        # print("epsi: ", epsi)

        # # <----- Dot product method ----->
        # # This method has very similar performance to our least squares
        # # method, but we chose to stick with least squares.

        # # develop channel unit vectors
        # C0 = A[:, 0]
        # C1 = A[:, 1]
        # C2 = A[:, 2]

        # # dot product of error vector along the channel vectors
        # epsi[0] = np.dot(err_r, C0)
        # epsi[1] = np.dot(err_r, C1)
        # epsi[2] = np.dot(err_r, C2)
        # print("epsi: ", epsi)

    def sendDesiredPressure(self):
        '''
        convert P_des and send this pressure into the Arduino
        '''
        global P_des
        # Safety check so we don't the arduino a super high or low pressure!
        # it will blow up if you have the wrong bounds
        for i in range(len(P_des)):
            if P_des[i] < 9.0:
                # lower limit of the pressure we are sending into the controller
                P_des[i] = 9.0
            elif P_des[i] > max_pressure[i]:
                # higher limit of the pressure we are sending into the controller
                P_des[i] = max_pressure[i]

        # send each channel pressure
        # We chose to send them individually so that we can control
        # any number of channels at a time
        arduino.sendDesiredPressure(arduino.channel0, float(P_des[0]))
        arduino.sendDesiredPressure(arduino.channel1, float(P_des[1]))
        arduino.sendDesiredPressure(arduino.channel2, float(P_des[2]))

    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        global r_des, k_p, k_i, k_d

        if (newCmd.id == "GuiToController"):
            if (newCmd.field1 == "setXPosition"):
                r_des[1] = newCmd.field2
            elif (newCmd.field1 == "setZPosition"):
                r_des[0] = newCmd.field2
            elif (newCmd.field1 == "setKp"):
                # This will set Kp for all channels at once
                k_p[0] = newCmd.field2
                k_p[1] = newCmd.field2
                k_p[2] = newCmd.field2
                logging.debug("\nCommand recieved to set proportional gain to", k_p)
            elif (newCmd.field1 == "setKi"):
                # This will set Ki for all channels at once
                k_i[0] = newCmd.field2
                k_i[1] = newCmd.field2
                k_i[2] = newCmd.field2
                logging.debug("\nCommand recieved to set integral gain to", k_i)
            elif (newCmd.field1 == "setKd"):
                # This will set Kd for all channels at once
                k_d[0] = newCmd.field2
                k_d[1] = newCmd.field2
                k_d[2] = newCmd.field2
                logging.debug("\nCommand recieved to set integral gain to", k_d)

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
