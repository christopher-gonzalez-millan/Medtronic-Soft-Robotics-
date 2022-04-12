'''
 * @file    three_channel_PI_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 3D proportional and/or PI controller
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
from math import sin, pi, sqrt
from scipy import signal as sg
import scipy.optimize as sp_opt
import numpy as np

logging.basicConfig(filename = 'data.log', level = logging.WARNING, 
    format = '%(asctime)s,%(message)s')

# create csv logging object to store the data collected
header = ['date', 'time_diff', 'z_des', 'z_act', 'P_des', 'P_act', 'k_p', 'k_i']
csv_logger = CsvLogger(filename='Data Collection/Tracking Curves/data.csv',
                        level=logging.INFO, fmt='%(asctime)s,%(message)s', header=header)

# Init EM Nav and Arduino
ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()
arduino.selectChannels(arduino.ON, arduino.OFF, arduino.OFF)

# Parameters for controller
z_des = 40.0     # stores the desired z position input by user
z_act = 0.0     # actual z_position from EM sensor
start_time = 0      # start time for the ramp and sinusoid signals
time_diff = 0       # time difference betweeen the start and current times

# Parameters for the 3 channel controller
P_des = np.array([12.25, 12.25, 12.25])     # desired pressure we're sending to the Arduino (c0, c1, c2)
P_act = np.array([0.0, 0.0, 0.0])           # actual pressure read from the pressure sensor (c0, c1, c2)
r_des = np.array([0.0, 0.0])                # desired position of robot in form (x, y)
r_act = np.array([0.0, 0.0])                # actual position of the robot using EM sensor (x, y)
k_p = np.array([.012, .012, .012])          # proportional controller gain for c0, c1, c2
k_i = np.array([.012, .012, .012])          # integral gain # TODO: figure out how to pass in integral gain and what is best gain value
dT = np.array([0.125, 0.125, 0.125])        # time between cycles (seconds) # TODO: find a way to clock the cycles to get this value (may be different between each channel)
int_sum = np.array([0.0, 0.0, 0.0])         # sum of the integral term # TODO: figure out if this should be a global value
err_r = np.array([0.0, 0.0])                # error between measured position and actual position
epsi = np.array([0.0, 0.0, 0.0])            # stores the solution to the force vector algorithm
epsi_prev = np.array([0.0, 0.0, 0.0])       # modified error in r (after force vector solution) for the previous time step # TODO: figure out if this should be a global value

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

        # tkinter.Frame.__init__(self, master)
        self.master.bind('<Left>', self.left_key)
        self.master.bind('<Right>', self.right_key)

        # <=== ROW 0 ===>
        command_label = ttk.Label(master, text = "Send Commands")
        command_label.grid(row = 0, column = 0, sticky = W, pady = 2, padx = (2,0))

        # <=== ROW 1 ===>
        # Text label for position entry
        position_label = ttk.Label(master, text = "Enter desired Z [mm]:")
        position_label.grid(row = 1, column = 0, sticky = W, pady = 2, padx = (30,0))
        # Entry widget for position
        self.position_entry= ttk.Entry(master, width= 10)
        self.position_entry.grid(row = 1, column = 1, sticky = W, pady = 2)
        self.position_entry.bind("<Return>", self.GUI_handleSetPositionCommand)

        # <=== ROW 2 ===>
        # Text label for proportional gain entry
        kp_label = ttk.Label(master, text = "Enter kp:")
        kp_label.grid(row = 2, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.kp_entry = ttk.Entry(master, width= 10)
        self.kp_entry.grid(row = 2, column = 1, sticky = W, pady = 2)
        self.kp_entry.bind("<Return>", self.GUI_handleSetKpCommand)

        # <=== ROW 3 ===>
        # Text label for integral gain entry
        ki_label = ttk.Label(master, text = "Enter ki:")
        ki_label.grid(row = 3, column = 0, sticky = W, pady = 2, padx = (30,0))
        #Create an Entry widget to accept User Input
        self.ki_entry = ttk.Entry(master, width= 10)
        self.ki_entry.grid(row = 3, column = 1, sticky = W, pady = 2)
        self.ki_entry.bind("<Return>", self.GUI_handleSetKiCommand)

        self.master.bind("<Left>", self.left_key)
        self.master.bind("<Right>", self.right_key)

        # Diplaying data
        display_label = ttk.Label(master, text = "Press/hold enter to display")
        display_label.grid(row = 4, column = 0, sticky = W, pady = (20,2), padx = (2,0))
        self.display_entry = ttk.Entry(master, width= 1)
        self.display_entry.grid(row = 4, column = 1, sticky = W, pady = (20,2))
        self.display_entry.bind("<Return>", self.GUI_handleDataDisplay)

        self.z_des_label = ttk.Label(master, text = "Z desired: ")
        self.z_des_label.grid(row = 5, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.z_act_label = ttk.Label(master, text = "Z actual: ")
        self.z_act_label.grid(row = 6, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.p_des_label = ttk.Label(master, text = "P desired: ")
        self.p_des_label.grid(row = 7, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.p_act_label = ttk.Label(master, text = "P actual: ")
        self.p_act_label.grid(row = 8, column = 0, sticky = W, pady = 2, padx = (30,0))

        self.int_sum_label = ttk.Label(master, text = "int_sum : ")
        self.int_sum_label.grid(row = 9, column = 0, sticky = W, pady = 2, padx = (30,0))

        # Record data
        command_label = ttk.Label(master, text = "Data Recording")
        command_label.grid(row = 10, column = 0, sticky = W, pady = (20,2), padx = (2,0))

        start_log_button = ttk.Button(master, text = "Start Logging", width = 12, command = lambda: self.GUI_handleLoggingCommand("start"))
        start_log_button.grid(row = 11, column = 0, sticky = W, pady = 2, padx = (2,0))

        stop_log_button = ttk.Button(master, text = "Stop Logging", width = 12, command = lambda: self.GUI_handleLoggingCommand("stop"))
        stop_log_button.grid(row = 11, column = 1, sticky = W, pady = 2, padx = (2,0))

        clear_log_button = ttk.Button(master, text = "Clear Log File", width = 12, command = lambda: self.GUI_handleLoggingCommand("clear"))
        clear_log_button.grid(row = 11, column = 2, sticky = W, pady = 2, padx = (50,0))


    def left_key(self, *args):
        newCmd = command("EM_Sensor", "adjustPosition", .5)
        commandsFromGUI.put(newCmd)

    def right_key(self, *args):
        newCmd = command("EM_Sensor", "adjustPosition", -.5)
        commandsFromGUI.put(newCmd)

    def GUI_handleSetPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        newCmd = command("EM_Sensor", "setPosition", float(self.position_entry.get()))
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

    def GUI_handleDataDisplay(self, *args):
        '''
        Display control algorithm parameters to the GUI
        '''
        global z_des, z_act, P_des, P_act

        self.z_des_label.configure(text = "Z desired: " + str(round(z_des,3)))
        self.z_act_label.configure(text = "Z actual: " + str(round(z_act,3)))
        self.p_des_label.configure(text = "P desired: " + str(round(P_des,3)))
        self.p_act_label.configure(text = "P actual: " + str(round(P_act,3)))
        self.int_sum_label.configure(text = "int_sum: " + str(round(int_sum,3)))

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
                self.three_channel_algorithm()
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
        global P_act, r_act, csv_logger

        # get the actual pressure from the pressure sensor
        P_act[0] = arduino.getActualPressure(arduino.channel0)
        P_act[1] = arduino.getActualPressure(arduino.channel1)
        P_act[2] = arduino.getActualPressure(arduino.channel2)

        # get actual position from EM sensor
        position = ndi.getPositionInRange()
        r_act[0] = position.deltaX          # x dim
        r_act[1] = position.deltaY          # y dim

        # perform 3 channel control algorithm
        self.three_channel_algorithm()

        # send the desired pressure into Arduino
        self.sendDesiredPressure()

        # Log all control variables if needed / TODO: find out how to re-implement time_diff variable
        # TODO: figure out if logging works with vectors/matrices
        logging.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (r_des, r_act, P_des, P_act, k_p, k_i))
        if logging.getLogger().getEffectiveLevel() == logging.INFO:
            csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (r_des, r_act, P_des, P_act, k_p, k_i))

    def three_channel_algorithm(self):
        '''
        Proportional/PI feedback loop algorithm (vector based solution -- includes dot product and bounded least squares solution)
        '''
        global r_des, r_act, err_r, P_des, P_act, k_p, k_i, dT, int_sum, epsi_prev, start_time

        # if start_time > 0:
        #     # self.ramp_signal()
        #     self.sinusoid_signal()

        # Calculate the error between current and desired positions
        err_r = r_des - r_act

        # perform force vector calculations
        self.forceVectorCalc()

        # Calculate the integral sum for integral control
        int_sum = int_sum + 0.5*(epsi + epsi_prev)*dT               # these are all element-wise operations / TODO: check the matrix math here
        for i in range(len(int_sum)):                               # checks the integral sum to see if it's in range (hardcoded integral windup)
            if int_sum[i] > 3:
                int_sum[i] = 3
            elif int_sum[i] < -3:
                int_sum[i] = -3

        # < ------- Feedback controller --------- >
        del_P_des = k_p*epsi + k_i*(int_sum)                    # should be element-wise operations / TODO: check the matrix math here
        P_des = P_act + del_P_des

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

        # <----- Dot product method ----->
        # develop channel unit vectors
        C0 = A[:, 0]
        C1 = A[:, 1]
        C2 = A[:, 2]

        # dot product of error vector along the channel vectors
        epsi[0] = np.dot(err_r, C0)
        epsi[1] = np.dot(err_r, C1)
        epsi[2] = np.dot(err_r, C2)

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
            elif P_des[i] > 13.25:
                # higher limit of the pressure we are sending into the controller
                P_des[i] = 13.25

        # send each channel pressure
        arduino.sendDesiredPressure(arduino.channel0, P_des[0])
        arduino.sendDesiredPressure(arduino.channel1, P_des[1])
        arduino.sendDesiredPressure(arduino.channel2, P_des[2])

    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        global z_des, k_p, k_i

        if (newCmd.id == "EM_Sensor"):
            if (newCmd.field1 == "adjustPosition"):
                z_des += newCmd.field2
            elif (newCmd.field1 == "setPosition"):
                z_des = newCmd.field2
                logging.debug("\nCommand recieved to set position to ", z_des)
            elif (newCmd.field1 == "setKp"):
                k_p = newCmd.field2
                logging.debug("\nCommand recieved to set proportional gain to", k_p)
            elif (newCmd.field1 == "setKi"):
                k_i = newCmd.field2
                logging.debug("\nCommand recieved to set integral gain to", k_i)


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
