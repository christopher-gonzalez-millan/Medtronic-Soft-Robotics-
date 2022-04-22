'''
 * @file    three_channel_PI_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 3D proportional and/or PI controller
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

logging.basicConfig(filename = 'data.log', level = logging.WARNING, 
    format = '%(asctime)s,%(message)s')

# create csv logging object to store the data collected
header = ['date', 'sample_num', 'time_diff', 'z_des', 'x_des', 'z_act', 'x_act', 'P_des[0]', 'P_des[1]', 'P_des[2]', 'P_act[0]', 'P_act[1]', 'P_act[2]']
csv_logger = CsvLogger(filename='Data Collection/Tracking Curves/data.csv',
                        level=logging.INFO, fmt='%(asctime)s,%(message)s', header=header)
sample_num = 0          # variable to keep track of the samples for any data collection

# Init EM Nav and Arduino
ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)

# Parameters for controller
z_des = 40.0     # stores the desired z position input by user
z_act = 0.0     # actual z_position from EM sensor
start_time = 0      # start time for the ramp and sinusoid signals
time_diff = 0       # time difference betweeen the start and current times

# <== 2X Robot Parameters ==>
# k_p = np.array([.04, .04, .04])
# k_i = np.array([0.01, 0.01, 0.01]) 
# k_d = np.array([0.001, 0.001, 0.001])
# max_pressure = np.array([16, 16, 16])
# int_sum = 10, -10

# <== 1X Robot Parameters ==>
# k_p = np.array([.03, .03, .03])
# k_i = np.array([0.01, 0.01, 0.01]) 
# k_d = np.array([0.001, 0.001, 0.001])
# max_pressure = np.array([15.5, 15.2, 15.5])
# int_sum = 5, -5

# Parameters for the 3 channel controller
P_des = np.array([12.25, 12.25, 12.25])     # desired pressure we're sending to the Arduino (c0, c1, c2)
P_act = np.array([0.0, 0.0, 0.0])           # actual pressure read from the pressure sensor (c0, c1, c2)
r_des = np.array([0.0, 0.0])                # desired position of robot in form (x, y)
r_act = np.array([0.0, 0.0])                # actual position of the robot using EM sensor (x, y)
k_p = np.array([.03, .03, .03])             # 1X Scale - proportional controller gain for c0, c1, c2
k_i = np.array([0.01, 0.01, 0.01])          # 2X Scale - integral gain # TODO: figure out how to pass in integral gain and what is best gain value
k_d = np.array([0.001, 0.001, 0.001])          # Test derivative gain (TODO: figure out if this helps tracking)
dT = np.array([0.125, 0.125, 0.125])        # time between cycles (seconds) # TODO: find a way to clock the cycles to get this value (may be different between each channel)
int_sum = np.array([0.0, 0.0, 0.0])         # sum of the integral term # TODO: figure out if this should be a global value
err_r = np.array([0.0, 0.0])                # error between measured position and actual position
epsi = np.array([0.0, 0.0, 0.0])            # stores the solution to the force vector algorithm
epsi_prev = np.array([0.0, 0.0, 0.0])       # modified error in r (after force vector solution) for the previous time step # TODO: figure out if this should be a global value
max_pressure = np.array([15.5, 15.2, 15.5])

# Queue for inter-thread communication
commandsFromGUI = Queue()

# Class used for all commands
class command:
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
        self.y_position_entry= ttk.Entry(master, width= 10)
        self.y_position_entry.grid(row = 2, column = 1, sticky = W, pady = 2)
        self.y_position_entry.bind("<Return>", self.GUI_handleSetYPositionCommand)

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

        self.y_des_label = ttk.Label(master, text = "Z desired: ")
        self.y_des_label.grid(row = 9, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.y_act_label = ttk.Label(master, text = "Z actual: ")
        self.y_act_label.grid(row = 10, column = 0, sticky = W, pady = 2, padx = (30,0))

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
        newCmd = command("EM_Sensor", "setXPosition", float(self.x_position_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetYPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        newCmd = command("EM_Sensor", "setYPosition", float(self.y_position_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetKpCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("EM_Sensor", "setKp", float(self.kp_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetKiCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("EM_Sensor", "setKi", float(self.ki_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleSetKdCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("EM_Sensor", "setKd", float(self.kd_entry.get()))
        commandsFromGUI.put(newCmd)

    def GUI_handleDataDisplay(self, *args):
        '''
        Display control algorithm parameters to the GUI
        '''
        global r_des, r_act, y_des, y_act, int_sum

        self.x_des_label.configure(text = "X desired: " + str(round(r_des[1],3)))
        self.x_act_label.configure(text = "X actual: " + str(round(r_act[1],3)))
        self.y_des_label.configure(text = "Z desired: " + str(round(r_des[0],3)))
        self.y_act_label.configure(text = "Z actual: " + str(round(r_act[0],3)))
        # self.int_sum_label.configure(text = "int_sum: " + str(round(int_sum,3)))

    def GUI_handleLoggingCommand(self, status):
        '''
        Start logging
        '''
        global start_time, sample_num

        if (status == "start"):
            logging.getLogger().setLevel(logging.INFO)
            start_time = time.time()                    # start time for ramp and sinusoid signals
            sample_num = 0
        elif (status == "stop"):
            logging.getLogger().setLevel(logging.WARNING)
            start_time = 0
        elif (status == "clear"):
            # Clear contents of log file
            with open('data.log', 'w'):
                pass


class controllerThread(threading.Thread):
    '''
    Implements proportional controller
    '''
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def run(self):
        try:
            while True:
                # Look for new commands
                if (commandsFromGUI.empty() == False):
                    newCmd = commandsFromGUI.get()
                    self.handleGUICommand(newCmd)

                # self.one_D_main()
                self.three_channel_main()
                time.sleep(.07)

        finally:
            print('Controller thread teminated')
    
    def sinusoid_signal(self):
        '''
        Sinusoidal input function with regard to position for the 1D channel
        '''
        global start_time, z_des, time_diff

        current_time = time.time()

        time_diff = current_time - start_time

        A = 5       # amplitude of the sine signal [mm]
        C = 60      # offset of the sine function [mm]
        f = .1     # frequency of the signal [Hz]

        z_des = A*sin(2*pi*f*time_diff) + C      # resulting sinusoidal z_des [mm]

    def ramp_signal(self):
        global start_time, z_des, time_diff

        current_time = time.time()              # current time measured compared to start time
    
        time_diff = current_time - start_time   # time difference used for the signal

        A = (80 - 50)/2                         # amplitude of the ramp signal
        C = (80 - 50)/2 + 50                    # shifts the signal up to range of 50 mm to 90 mm
        T = 30                                # period of the signal in seconds

        z_des = A*sg.sawtooth((2*pi/T)*time_diff, width = 0.5) + C       # ramp signal set as a triangle wave 

    def circle_signal(self):
        global start_time, r_des, time_diff

        current_time = time.time()              # current time compared to start time    

        time_diff = current_time - start_time   # time difference used in the signal / TODO: incorporate the data logger into this function
        
        radius = 15                             # radius of the circle in mm

        T = 60                                  # period of the circle (in seconds)
        center = np.array([0, 0])           # center of the circle script (z, x)       

        # parametric equations that represent circle as a function of time
        r_des[0] = center[0] + radius*cos((2*pi/T)*time_diff)        # in z
        r_des[1] = center[1] + radius*sin((2*pi/T)*time_diff)        # in x

    def fig_eight_signal(self):
        global start_time, r_des, time_diff

        current_time = time.time()              # current time compared to start time    

        time_diff = current_time - start_time   # time difference used in the signal / TODO: incorporate the data logger into this function
        
        radius = 15                             # "radius" of the figure eight in mm

        T = 60                                  # period of the figure eight (in seconds)
        center = np.array([0, 0])           # center of the figure eight script (z, x)       

        # parametric equations that represent figure eight as a function of time
        r_des[0] = center[0] + radius*sin((2*pi/T)*time_diff)                                # in z
        r_des[1] = center[1] + radius*sin((2*pi/T)*time_diff)*cos((2*pi/T)*time_diff)        # in x

    def one_D_main(self):
        '''
        main function used in thread to perform 1D algorithm
        '''
        global P_act, z_act, time_diff, csv_logger

        # get the actual pressure from the pressure sensor
        P_act = arduino.getActualPressure(arduino.channel0)

        # get actual position from EM sensor
        position = ndi.getPositionInRange()
        z_act = position.deltaX

        # perform 1D proportional control
        self.one_D_algorithm()

        # send the desired pressure into Arduino
        self.sendDesiredPressure()

        # Log all control variables if needed
        logging.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (time_diff, z_des, z_act, P_des, P_act, k_p, k_i))
        if logging.getLogger().getEffectiveLevel() == logging.INFO:
            csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (time_diff, z_des, z_act, P_des, P_act, k_p, k_i))

    def one_D_algorithm(self):
        '''
        Proportional feedback loop algorithm (includes our method and Shalom's del P)
        '''
        global z_des, z_act, P_des, P_act, k_p, dT, int_sum, epsi_z_prev, k_i, start_time

        if start_time > 0:
            # self.ramp_signal()
            self.sinusoid_signal()

        # Calculate the error between current and desired positions
        epsi_z = z_des - z_act

        # Calculate the integral sum
        int_sum = int_sum + 0.5*(epsi_z + epsi_z_prev)*dT
        if int_sum > 3:
            int_sum = 3
        elif int_sum < -3:
            int_sum = -3

        # < -------- Shalom P_absolute method --------- >
        # Utilize the proportional and integral controller values for P_des
        # P_des = k_p*epsi_z + k_i*int_sum

        # logging.debug("P_des: ", self.P_des)

        # < ------- Our feedback method --------- >
        del_P_des = k_p * epsi_z + k_i*(int_sum)
        P_des = P_act + del_P_des

        # < -------- Shalom delta P method ------- >
        # Figure out how to utilize del_P_act instead of P_des (on Arduino side?)
        # del_P_des = k_p*epsi_z
        # P_des = P_o + del_P_des
        # del_P_act = P_des - P_act

        # Check for windup of the integrator
        # if (P_des >= 13.25) or (P_des <= 9.0):
        #     P_des = k_p*epsi_z

        # Update the error value for next iteration of epsi_z_prev
        epsi_z_prev = epsi_z

    def three_channel_main(self):
        '''
        main function used in thread to perform 3 channel algorithm
        '''
        global time_diff, r_des, r_act, P_des, P_act, csv_logger, sample_num

        # get the actual pressure from the pressure sensor
        P_act[0] = arduino.getActualPressure(arduino.channel0)
        P_act[1] = arduino.getActualPressure(arduino.channel1)
        P_act[2] = arduino.getActualPressure(arduino.channel2)
        # print("P_act", P_act)

        # get actual position from EM sensor
        position = ndi.getPositionInRange()
        r_act[1] = position.deltaX          # x dim
        r_act[0] = position.deltaZ          # y dim
        # print("r_act[0]: " + str(r_act[0]) + "r_act[1]: " + str(r_act[1]))

        # perform 3 channel control algorithm
        self.three_channel_algorithm()

        # send the desired pressure into Arduino
        self.sendDesiredPressure()

        # Log all control variables if needed / TODO: find out how to re-implement time_diff variable
        # TODO: figure out if logging works with vectors/matrices
        logging.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % \
            (sample_num, time_diff, r_des[0], r_des[1], r_act[0], r_act[1], P_des[0], P_des[1], P_des[2], P_act[0], P_act[1], P_act[2]))
        if logging.getLogger().getEffectiveLevel() == logging.INFO:
            csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % \
                (sample_num, time_diff, r_des[0], r_des[1], r_act[0], r_act[1], P_des[0], P_des[1], P_des[2], P_act[0], P_act[1], P_act[2]))

        # update sample number for next data point
        sample_num = sample_num + 1

    def three_channel_algorithm(self):
        '''
        Proportional/PI feedback loop algorithm (vector based solution -- includes dot product and bounded least squares solution)
        '''
        global r_des, r_act, err_r, P_des, P_act, k_p, k_i, k_d, dT, int_sum, epsi, epsi_prev, start_time

        if start_time > 0:
            self.circle_signal()

        # Calculate the error between current and desired positions
        err_r = r_des - r_act
        # print("err_r: ", err_r)

        # perform force vector calculations
        self.forceVectorCalc()

        # Calculate the integral sum for integral control
        int_sum = int_sum + 0.5*(epsi + epsi_prev)*dT               # these are all element-wise operations / TODO: check the matrix math here
        for i in range(len(int_sum)):                               # checks the integral sum to see if it's in range (hardcoded integral windup)
            if int_sum[i] > 5:
                int_sum[i] = 5
            elif int_sum[i] < -5:
                int_sum[i] = -5
        
        # Calculate the derivative term of the controller (TODO: figure out if this term helps tracking)
        deriv = (epsi - epsi_prev)/dT

        # < ------- Feedback controller --------- >
        del_P_des = k_p*epsi + k_i*(int_sum) + k_d*deriv                     # should be element-wise operations / TODO: check the matrix math here
        P_des = P_act + del_P_des

        # print("del_P_des: ", del_P_des)
        # print("P_act: ", P_act)
        # print("P_des: ", P_des)

        # Check for windup of the integrator
        # if (P_des >= 13.25) or (P_des <= 9.0):
        #     P_des = k_p*epsi_z

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
        for i in range(len(P_des)):
            # TODO: check the range limits for the pressure being sent for the smaller robot
            if P_des[i] < 9.0:
                # lower limit of the pressure we are sending into the controller
                P_des[i] = 9.0
            elif P_des[i] > max_pressure[i]:
                # higher limit of the pressure we are sending into the controller
                P_des[i] = max_pressure[i]

        # send each channel pressure
        arduino.sendDesiredPressure(arduino.channel0, float(P_des[0]))
        arduino.sendDesiredPressure(arduino.channel1, float(P_des[1]))
        arduino.sendDesiredPressure(arduino.channel2, float(P_des[2]))

    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        global r_des, k_p, k_i, k_d

        if (newCmd.id == "EM_Sensor"):
            if (newCmd.field1 == "setXPosition"):
                r_des[1] = newCmd.field2
            elif (newCmd.field1 == "setYPosition"):
                r_des[0] = newCmd.field2
                logging.debug("\nCommand recieved to set position to ", z_des)
            elif (newCmd.field1 == "setKp"):
                k_p[0] = newCmd.field2
                k_p[1] = newCmd.field2
                k_p[2] = newCmd.field2
                logging.debug("\nCommand recieved to set proportional gain to", k_p)
            elif (newCmd.field1 == "setKi"):
                k_i[0] = newCmd.field2
                k_i[1] = newCmd.field2
                k_i[2] = newCmd.field2
                logging.debug("\nCommand recieved to set integral gain to", k_i)
            elif (newCmd.field1 == "setKd"):
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
