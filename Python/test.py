# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 14:52:22 2022

@author: Christopher
"""
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from ttkthemes import ThemedStyle
from math import sin, pi, sqrt, cos
import ctypes
import threading
from queue import Queue
import logging
from csv_logger import CsvLogger
from NDI_Code.NDISensor import NDISensor
from Py_Arduino_Communication.arduino_control import arduino_control
import numpy as np 
from PIL import Image,  ImageTk
import time
from scipy import signal as sg
import scipy.optimize as sp_opt
import matplotlib.pyplot as plt
#from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)

OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path("./assets")

"""
GLOBAL HELPER FUNCTIONS
"""
#TODO remove from global memory
def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

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

# Parameters for the 3 channel controller
P_des = np.array([12.25, 12.25, 12.25])     # desired pressure we're sending to the Arduino (c0, c1, c2)
P_act = np.array([0.0, 0.0, 0.0])           # actual pressure read from the pressure sensor (c0, c1, c2)
r_des = np.array([0.0, 0.0])                # desired position of robot in form (x, y)
r_act = np.array([0.0, 0.0])                # actual position of the robot using EM sensor (x, y)
k_p = np.array([.04, .04, .04])             # 2X Scale - proportional controller gain for c0, c1, c2
# k_p = np.array([.01, .01, .01])             # 1X Scale - proportional controller gain for c0, c1, c2
k_i = np.array([0.01, 0.01, 0.01])          # 2X Scale - integral gain # TODO: figure out how to pass in integral gain and what is best gain value
# k_i = np.array([0.0, 0.0, 0.0])          # 1X Scale - integral gain # TODO: figure out how to pass in integral gain and what is best gain value
# k_d = np.array([0.001, 0.001, 0.001])          # Test derivative gain (TODO: figure out if this helps tracking)
k_d = np.array([0.001, 0.001, 0.001])          # Test derivative gain (TODO: figure out if this helps tracking)
dT = np.array([0.125, 0.125, 0.125])        # time between cycles (seconds) # TODO: find a way to clock the cycles to get this value (may be different between each channel)
int_sum = np.array([0.0, 0.0, 0.0])         # sum of the integral term # TODO: figure out if this should be a global value
err_r = np.array([0.0, 0.0])                # error between measured position and actual position
epsi = np.array([0.0, 0.0, 0.0])            # stores the solution to the force vector algorithm
epsi_prev = np.array([0.0, 0.0, 0.0])       # modified error in r (after force vector solution) for the previous time step # TODO: figure out if this should be a global value
max_pressure = np.array([16.5, 16.5, 16.5])


# Queue for inter-thread communication
commandsFromGUI = Queue()

# Class used for all commands
class command:
    def __init__(self, id, field1, field2):
        self.id = id
        self.field1 = field1
        self.field2 = field2

# a subclass of Canvas for dealing with resizing of windows
class ResizingCanvas(tk.Canvas):
    def __init__(self,parent,**kwargs):
        tk.Canvas.__init__(self, parent,**kwargs)
        self.bind("<Configure>", self.on_resize)
        self.height = self.winfo_reqheight()
        self.width = self.winfo_reqwidth()

    def on_resize(self,event):
        # determine the ratio of old width/height to new width/height
        wscale = float(event.width)/self.width
        hscale = float(event.height)/self.height
        self.width = event.width
        self.height = event.height
        # resize the canvas 
        self.config(width=self.width, height=self.height)
        # rescale all the objects tagged with the "all" tag
        self.scale("all",0,0,wscale,hscale)

class shalomWindow(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        
        self.image  = Image.open(relative_to_assets("shalom.png"))
        self.img_copy= self.image.copy()


        self.background_image = ImageTk.PhotoImage(self.image)

        self.background = tk.Label(self, image=self.background_image)
        self.background.pack(expand=True, fill='both',)
        self.background.bind('<Configure>', self._resize_image)
        
    def _resize_image(self,event):

        new_width = event.width
        new_height = event.height

        self.image = self.img_copy.resize((new_width, new_height))

        self.background_image = ImageTk.PhotoImage(self.image)
        self.background.configure(image =  self.background_image)
        
class controlWindow(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        self.frame = tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent
        self.width, self.height = parent.winfo_screenwidth(), parent.winfo_screenheight()
        self.canvas = ResizingCanvas(
            self, 
            bg = "#626262", 
            height = self.height,
            width = self.width,
            bd = 0,
            highlightthickness = 0,
            borderwidth=2,
            relief = "raised")
        self.canvas.pack(expand = 1, fill ="both")
        self.buildUX()
        self.buildUI()
        self.GUI_handleDataDisplay()
        
    def buildUX(self):        
        """
        UX Rectangle: making pretty but unnecessary rectangles
        """
        #TODO label what each rectangle represents, we can do this by changing the color of each and see what happens
        self.canvas.create_rectangle(1464.5130615234375, 1278.018798828125, 2694.99951171875, 1639.447021484375, width=1, fill="#424242", outline="black")
        self.canvas.create_rectangle(40.0,116.35806274414062, 721.0739135742188, 989.8941955566406, width=1, fill="#424242", outline="black")
        self.canvas.create_rectangle(750.7725219726562, 767.9468383789062, 1431.846435546875, 1639.44677734375, width=1, fill="#424242", outline="black")

        """
        UX Text: defining text that remains constant
        """
        #data loggin widget
        self.canvas.create_text(1953.540283203125,1315.688720703125,anchor="nw",text="Data Recording",fill="#ffffff", font=("TkDefaultFont", 32 * -1))
        
        #position widget
        self.canvas.create_text(967.5676879882812,787.2908935546875,anchor="nw",text="Positioning",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(793.3386840820312,845.3230590820312,anchor="nw",text="Current Position:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(899.2613525390625,930.8441772460938,anchor="nw",text="x coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625,1016.3651733398438,anchor="nw",text="y coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625,1100.867919921875,anchor="nw",text="z coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(793.3386840820312,1204.715087890625,anchor="nw",text="Desired Position:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(899.2613525390625,1290.236083984375,anchor="nw",text="x coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625,1374.739013671875,anchor="nw",text="y coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625,1460.260009765625,anchor="nw",text="z coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))

        #air pressure widget
        self.canvas.create_text(232.0,140.0,anchor="nw",text="Channel Pressures",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(81.57742309570312,195.77041625976562,anchor="nw",text="Current Pressures:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(186.5096893310547,281.29144287109375,anchor="nw",text="Channel 1",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547,365.79437255859375,anchor="nw",text="Channel 2",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547,451.31536865234375,anchor="nw",text="Channel 3",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547,638.647216796875,anchor="nw",text="Channel 1",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547,725.1863403320312,anchor="nw",text="Channel 2",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547,810.7073364257812,anchor="nw",text="Channel 3 ",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(81.57742309570312,556.1624145507812,anchor="nw",text="Desired Pressures:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        
        #Graph titles
        self.canvas.create_text(1936,40.0,anchor="nw",text="Position Projection ",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(922,40.0,anchor="nw",text="Curvature Visualization",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        
    def buildUI(self):

        #position text
        self.xPosText = ttk.Label(self, text='0.0')
        self.xPosText.place(relx=1113/2736,rely=928/1839,relwidth=169/2736,relheight=47/1839)
        
        self.yPosText = ttk.Label(self,  text='0.0')
        self.yPosText.place(relx=1113/2736,rely=1014/1839,relwidth=169/2736,relheight=47/1839)
        
        self.zPosText = ttk.Label(self,  text='0.0')
        self.zPosText.place(relx=1113/2736,rely=1099/1839,relwidth=169/2736,relheight=47/1839)
        
        #position entry boxes
        self.xPosEntry = ttk.Entry(self)
        self.xPosEntry.place(relx=1113/2736,rely=1288/1839,relwidth=169/2736,relheight=47/1839)
        self.xPosEntry.bind("<Return>", self.GUI_handleSetXPositionCommand)
        
        self.yPosEntry = ttk.Entry(self)
        self.yPosEntry.place(relx=1113/2736,rely=1373/1839,relwidth=169/2736,relheight=47/1839)
        self.xPosEntry.bind("<Return>", self.GUI_handleSetYPositionCommand)
        
        self.zPosEntry = ttk.Entry(self, state='disabled')
        self.zPosEntry.place(relx=1113/2736,rely=1460/1839,relwidth=169/2736,relheight=47/1839)
        self.xPosEntry.bind("<Return>", self.GUI_handleSetZPositionCommand)
        
        #Pressure text
        self.channel0Text = ttk.Label(self)
        self.channel0Text.place(relx=402/2736,rely=279/1839,relwidth=169/2736,relheight=47/1839)
        
        self.channel1Text = ttk.Label(self)
        self.channel0Text.place(relx=402/2736,rely=364/1839,relwidth=169/2736,relheight=47/1839)
        
        self.channel2Text = ttk.Label(self)
        self.channel2Text.place(relx=402/2736,rely=449/1839,relwidth=169/2736,relheight=47/1839)
        
        #Pressure Entry Boxes
        self.channel0Entry = ttk.Entry(self)
        self.channel0Entry.place(relx=402/2736,rely=648/1839,relwidth=169/2736,relheight=47/1839)
        
        self.channel1Entry = ttk.Entry(self)
        self.channel1Entry.place(relx=402/2736,rely=732/1839,relwidth=169/2736,relheight=47/1839)
        
        self.channel2Entry = ttk.Entry(self)
        self.channel2Entry.place(relx=402/2736,rely=818/1839,relwidth=169/2736,relheight=47/1839)
        
        """
        Logging Buttons
        """
        self.stop = ttk.Button(self, text ="Stop Logging",command=lambda: self.GUI_handleLoggingCommand("stop"))
        self.stop.place(relx=1921/2736,rely=1439/1839,relwidth=314/2736,relheight=97/1839)
        
        self.clear = ttk.Button(self, text ="Clear Log File",command=lambda: self.GUI_handleLoggingCommand("clear"))
        self.clear.place(relx=2281/2736, rely=1439/1839,relwidth=314/2736,relheight=97/1839)
        
        self.start = ttk.Button(self, text ="Start Logging",command=lambda: lambda: self.GUI_handleLoggingCommand("start"))
        self.start.place(relx=1565/2736,rely=1439/1839,relwidth=314/2736,relheight=97/1839)
        
        
    def GUI_handleDataDisplay(self, *args):
        global r_des, r_act, y_des, y_act, int_sum

        self.xPosText.configure(text = str(round(r_act[1],3)))
        self.zPosText.configure(text = str(round(r_act[0],3)))
        self.yPosText.configure(text = str(round(z_act,3)))


        # call again after 100 ms
        self.parent.after(100, self.GUI_handleDataDisplay)
    
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
        
    def GUI_handleSetZPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        print('error z entry box not disabled')
        
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

class App:
    def __init__(self, parent):
        #parent window setup 
        self.width, self.height = parent.winfo_screenwidth(), parent.winfo_screenheight()
        parent.geometry('%dx%d' % (self.width, self.height))   
        parent.title('GUI')
        self.parent = parent
        # create a navagiation bar
        self.navbar = ttk.Notebook(self.parent)
        self.controlFrame = controlWindow(self.navbar)
        self.shalomFrame = shalomWindow(self.navbar) 
        
        self.navbar.pack(expand = 1, fill ="both")
        self.controlFrame.pack(expand = 1, fill ="both")
        self.shalomFrame.pack(expand = 1, fill ="both")
        
        self.navbar.add(self.controlFrame, text='Controls')
        self.navbar.add(self.shalomFrame, text='Shalom')
        
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

    def three_channel_main(self):
        '''
        main function used in thread to perform 3 channel algorithm
        '''
        global time_diff, r_des, r_act, P_des, P_act, csv_logger, sample_num, z_act

        # get the actual pressure from the pressure sensor
        P_act[0] = arduino.getActualPressure(arduino.channel0)
        P_act[1] = arduino.getActualPressure(arduino.channel1)
        P_act[2] = arduino.getActualPressure(arduino.channel2)
        # print("P_act", P_act)

        # get actual position from EM sensor
        position = ndi.getPositionInRange()
        r_act[1] = position.deltaX          # x dim
        r_act[0] = position.deltaZ          # y dim
        z_act = position.deltaZ          # y dim
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
            if int_sum[i] > 10:
                int_sum[i] = 10
            elif int_sum[i] < -10:
                int_sum[i] = -10
        
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

    # Spin up controller thread
    cThread = controllerThread('Thread 1')
    cThread.start()

    #function to calibrate the correct DPI of current computer
    ctypes.windll.shcore.SetProcessDpiAwareness(1)
    # Designate main thread to GUI
    root = tk.Tk() 
    style = ThemedStyle(root)
    style.set_theme("black")
    App(root)
    root.resizable(True, True)
    root.mainloop()

    # Kill controller once GUI is exited
    cThread.raise_exception()
    cThread.join()


if __name__ == '__main__':
    main()