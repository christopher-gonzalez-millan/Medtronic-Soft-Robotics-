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
import NDI_communication
import arduino_communcation
import numpy as np
from PIL import Image,  ImageTk
import time
from scipy import signal as sg
import scipy.optimize as sp_opt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)

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
try:
    ndi = NDI_communication.NDISensor()
    arduino = arduino_communcation.arduino()
    arduino.selectChannels(arduino.ON, arduino.ON, arduino.ON)
except:
  print("Arduino or NDI sensor not connected")

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
k_p = np.array([.03, .03, .03])             # 1X Scale - proportional controller gain for c0, c1, c2
k_i = np.array([0.01, 0.01, 0.01])          # 2X Scale - integral gain # TODO: figure out how to pass in integral gain and what is best gain value
k_d = np.array([0.001, 0.001, 0.001])          # Test derivative gain (TODO: figure out if this helps tracking)
dT = np.array([0.125, 0.125, 0.125])        # time between cycles (seconds) # TODO: find a way to clock the cycles to get this value (may be different between each channel)
int_sum = np.array([0.0, 0.0, 0.0])         # sum of the integral term # TODO: figure out if this should be a global value
err_r = np.array([0.0, 0.0])                # error between measured position and actual position
epsi = np.array([0.0, 0.0, 0.0])            # stores the solution to the force vector algorithm
epsi_prev = np.array([0.0, 0.0, 0.0])       # modified error in r (after force vector solution) for the previous time step # TODO: figure out if this should be a global value
max_pressure = np.array([15.5, 15.2, 15.5])

#thread for controller
cThread = None

# Queue for inter-thread communication
commandsFromGUI = Queue()

# Class used for all commands
class command:
    def __init__(self, *args):
        if len(args) == 3:
            self.id = args[0]
            self.field1 = args[1]
            self.field2 = args[2]

        elif len(args) == 4:
            self.id = args[0]
            self.field1 = args[1]
            self.field2 = args[2]
            self.field3 = args[3]


class projectPostition:
    def __init__(self, parent):
        self.parent = parent
        self.center, self.radius = self.defineCircle((0,50), (50,0), (0,-50))
        self.x = 0
        self.y = 0
        self.buildProjectionWidget()

    def updatePosition(self, x, y):
        self.x = x
        self.y = y

    def buildProjectionWidget(self):
        self.figure, self.axes  = plt.subplots()
        self.figure.set_facecolor("#424242",)
        # change all spines
        for axis in ['top','bottom','left','right']:
            self.axes .spines[axis].set_linewidth(2)
        # increase tick width
        self.axes.tick_params(width=1)
        #postiotn projection
        self.posProjectionPlot = FigureCanvasTkAgg(self.figure, master=self.parent)
        self.posProjectionPlot.get_tk_widget().place(relx=1464.5130615234375/2736,rely=116.35806274414062/1824,relwidth=1230/2736,relheight=1142/1824)
        self.posProjectionPlot.draw()

    def plot(self):
        self.axes.clear()
        self.axes.set_xlim(-50.2, 50.2) #TODO update to gloabl max
        self.axes.set_ylim(-50.2, 50.2)
        plt.plot(self.x, self.y, 'ro', linewidth = 5)
        cc = plt.Circle(self.center, self.radius, fill=False)
        plt.grid(True)
        self.axes.set_aspect( 1 )
        self.axes.add_artist( cc )
        self.posProjectionPlot.draw()

    def defineCircle(self, p1, p2, p3):
        """
        Returns the center and radius of the circle passing the given 3 points.
        In case the 3 points form a line, returns (None, infinity).
        """
        temp = p2[0] * p2[0] + p2[1] * p2[1]
        bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
        cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

        if abs(det) < 1.0e-6:
            return (None, np.inf)

        # Center of circle
        cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

        radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
        return ((cx, cy), radius)

    def inCircle(self, x, y):
        if pow((x - self.center[0]), 2) + pow((y - self.center[1]), 2) <= pow(self.radius, 2):
            return True

        return False

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

class pidTuningWindow(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
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
        self.buildUX()
        self.buildUI()
        self.canvas.pack(expand = 1, fill ="both")
        self.updateDisplay()

    def buildUX(self):
        # container
        self.canvas.create_rectangle(40.0 ,40.0 , 721.0739135742188 , 913.536132813 , width=1, fill="#424242", outline="black")
        #air pressure widget
        self.canvas.create_text(232.0 , 60.0 ,anchor="nw",text="PID Tuning",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(81.57742309570312 ,115.77041625976562 ,anchor="nw",text="Current Values:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(186.5096893310547 ,201.29144287109375 ,anchor="nw",text="Channel 1",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,285.79437255859375 ,anchor="nw",text="Channel 2",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,371.31536865234375 ,anchor="nw",text="Channel 3",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,638.647216796875 ,anchor="nw",text="Channel 1",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,558.1863403320312 ,anchor="nw",text="Channel 2",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,730.7073364257812 ,anchor="nw",text="Channel 3 ",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(81.57742309570312 ,476.1624145507812 ,anchor="nw",text="Desired values:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))

    def buildUI(self):
        #Pressure text
        self.kpText = ttk.Label(self, text='0.0')
        self.kpText.place(relx=402/2736,rely=199/1824,relwidth=169/2736,relheight=47/1824)

        self.kiText = ttk.Label(self, text='0.0')
        self.kiText.place(relx=402/2736,rely=284/1824,relwidth=169/2736,relheight=47/1824)

        self.kdText = ttk.Label(self, text='0.0')
        self.kdText.place(relx=402/2736,rely=369/1824,relwidth=169/2736,relheight=47/1824)

        #Pressure Entry Boxes
        self.kpTextEntry = ttk.Entry(self)
        self.kpTextEntry.place(relx=402/2736,rely=568/1824,relwidth=169/2736,relheight=47/1824)
        self.kpTextEntry.bind("<Return>", self.handleSetKpCommand)

        self.kiTextEntry = ttk.Entry(self)
        self.kiTextEntry.place(relx=402/2736,rely=652/1824,relwidth=169/2736,relheight=47/1824)
        self.kiTextEntry.bind("<Return>", self.handleSetKiCommand)

        self.kdTextEntry = ttk.Entry(self)
        self.kdTextEntry.place(relx=402/2736,rely=738/1824,relwidth=169/2736,relheight=47/1824)
        self.kdTextEntry.bind("<Return>", self.handleSetKdCommand)

    def handleSetKpCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("EM_Sensor", "setKp", float(self.kpTextEntry.get()))
        commandsFromGUI.put(newCmd)
        self.updateDisplay()

    def handleSetKiCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("EM_Sensor", "setKi", float(self.kiTextEntry.get()))
        commandsFromGUI.put(newCmd)
        self.updateDisplay()

    def handleSetKdCommand(self, *args):
        '''
        Handle setting the gain from the GUI
        '''
        newCmd = command("EM_Sensor", "setKd", float(self.kdTextEntry.get()))
        commandsFromGUI.put(newCmd)
        self.updateDisplay()

    def updateDisplay(self):
        global k_p, k_i, k_d

        self.kpText.configure(text = k_p)
        self.kiText.configure(text = k_i)
        self.kdText.configure(text = k_d)

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
        self.updateDisplay()

    def buildUX(self):
        """
        UX Rectangle: making pretty but unnecessary rectangles
        """
        #data recording container
        self.canvas.create_rectangle(1464.5130615234375 , 1278.018798828125 , 2694.99951171875 , 1639.447021484375 , width=1, fill="#424242", outline="black")
        #channel pressure container
        self.canvas.create_rectangle(40.0 ,116.35806274414062 , 721.0739135742188 , 989.8941955566406 , width=1, fill="#424242", outline="black")
        #postion container
        self.canvas.create_rectangle(750.7725219726562 , 767.9468383789062 , 1431.846435546875 , 1639.44677734375 , width=1, fill="#424242", outline="black")
        #controller type container
        self.canvas.create_rectangle(40.0 , 1019.8941955566406 , 721.0739135742188 , 1280.8941955566406 , width=1, fill="#424242", outline="black")

        """
        UX Text: defining text that remains constant
        """
        #data loggin widget
        self.canvas.create_text(1953.540283203125 ,1315.688720703125 ,anchor="nw",text="Data Recording",fill="#ffffff", font=("TkDefaultFont", 32 * -1))

        #position widget
        self.canvas.create_text(967.5676879882812 ,787.2908935546875 ,anchor="nw",text="Positioning",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(793.3386840820312 ,845.3230590820312 ,anchor="nw",text="Current Position:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(899.2613525390625 ,930.8441772460938 ,anchor="nw",text="x coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625 ,1016.3651733398438 ,anchor="nw",text="y coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625 ,1100.867919921875 ,anchor="nw",text="z coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(793.3386840820312 ,1204.715087890625 ,anchor="nw",text="Desired Position:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(899.2613525390625 ,1290.236083984375 ,anchor="nw",text="x coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625 ,1374.739013671875 ,anchor="nw",text="y coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(899.2613525390625 ,1460.260009765625 ,anchor="nw",text="z coordinate",fill="#ffffff",font=("TkDefaultFont", 20 * -1))

        #air pressure widget
        self.canvas.create_text(232.0 ,140.0 ,anchor="nw",text="Channel Pressures",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(81.57742309570312 ,195.77041625976562 ,anchor="nw",text="Current Pressures:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))
        self.canvas.create_text(186.5096893310547 ,281.29144287109375 ,anchor="nw",text="Channel 1",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,365.79437255859375 ,anchor="nw",text="Channel 2",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,451.31536865234375 ,anchor="nw",text="Channel 3",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,638.647216796875 ,anchor="nw",text="Channel 1",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,725.1863403320312 ,anchor="nw",text="Channel 2",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(186.5096893310547 ,810.7073364257812 ,anchor="nw",text="Channel 3 ",fill="#ffffff",font=("TkDefaultFont", 20 * -1))
        self.canvas.create_text(81.57742309570312 ,556.1624145507812 ,anchor="nw",text="Desired Pressures:",fill="#ffffff",font=("TkDefaultFont", 24 * -1))

        #controller type widget
        self.canvas.create_text(252.0 ,1043.53613281 ,anchor="nw",text="Controller Type",fill="#ffffff",font=("TkDefaultFont", 32 * -1))

        #Graph titles
        self.canvas.create_text(1936 ,40.0 ,anchor="nw",text="Position Projection ",fill="#ffffff",font=("TkDefaultFont", 32 * -1))
        self.canvas.create_text(922 ,40.0 ,anchor="nw",text="Curvature Visualization",fill="#ffffff",font=("TkDefaultFont", 32 * -1))

        #medtronic logo
        self.img = Image.open(relative_to_assets("medtronic1.png"))  # PIL solution
        self.img = self.img.resize((690, 121)) #The (250, 250) is (height, width)
        self.img = ImageTk.PhotoImage(self.img) # convert to PhotoImage
        self.canvas.create_image(40.0 , 1310.0 , image=self.img, anchor='nw')

    def buildUI(self):
        #position text
        self.xPosText = ttk.Label(self, text='0.0')
        self.xPosText.place(relx=1113/2736,rely=928/1824,relwidth=169/2736,relheight=47/1824)

        self.yPosText = ttk.Label(self,  text='0.0')
        self.yPosText.place(relx=1113/2736,rely=1014/1824,relwidth=169/2736,relheight=47/1824)

        self.zPosText = ttk.Label(self,  text='0.0')
        self.zPosText.place(relx=1113/2736,rely=1099/1824,relwidth=169/2736,relheight=47/1824)

        #position entry boxes
        self.xPosEntry = ttk.Entry(self,state='disabled')
        self.xPosEntry.place(relx=1113/2736,rely=1288/1824,relwidth=169/2736,relheight=47/1824)
        self.xPosEntry.bind("<Return>", self.handleSetXPositionCommand)

        self.yPosEntry = ttk.Entry(self,state='disabled')
        self.yPosEntry.place(relx=1113/2736,rely=1373/1824,relwidth=169/2736,relheight=47/1824)
        self.yPosEntry.bind("<Return>", self.handleSetYPositionCommand)

        self.zPosEntry = ttk.Entry(self, state='disabled')
        self.zPosEntry.place(relx=1113/2736,rely=1460/1824,relwidth=169/2736,relheight=47/1824)
        self.zPosEntry.bind("<Return>", self.handleSetZPositionCommand)

        #Pressure text
        self.channel0Text = ttk.Label(self, text='0.0')
        self.channel0Text.place(relx=402/2736,rely=279/1824,relwidth=169/2736,relheight=47/1824)

        self.channel1Text = ttk.Label(self, text='0.0')
        self.channel1Text.place(relx=402/2736,rely=364/1824,relwidth=169/2736,relheight=47/1824)

        self.channel2Text = ttk.Label(self, text='0.0')
        self.channel2Text.place(relx=402/2736,rely=449/1824,relwidth=169/2736,relheight=47/1824)

        #Pressure Entry Boxes
        self.channel0Entry = ttk.Entry(self,state='disabled')
        self.channel0Entry.place(relx=402/2736,rely=648/1824,relwidth=169/2736,relheight=47/1824)
        self.channel0Entry.bind("<Return>", self.setChannel0)

        self.channel1Entry = ttk.Entry(self,state='disabled')
        self.channel1Entry.place(relx=402/2736,rely=732/1824,relwidth=169/2736,relheight=47/1824)
        self.channel1Entry.bind("<Return>", self.setChannel1)

        self.channel2Entry = ttk.Entry(self,state='disabled')
        self.channel2Entry.place(relx=402/2736,rely=818/1824,relwidth=169/2736,relheight=47/1824)
        self.channel2Entry.bind("<Return>", self.setChannel2)

        #Data Logging Buttons
        self.stop = ttk.Button(self, text ="Stop Logging",command=lambda: self.handleLoggingCommand("stop"))
        self.stop.place(relx=1921/2736,rely=1439/1824,relwidth=314/2736,relheight=97/1824)

        self.clear = ttk.Button(self, text ="Clear Log File",command=lambda: self.handleLoggingCommand("clear"))
        self.clear.place(relx=2281/2736, rely=1439/1824,relwidth=314/2736,relheight=97/1824)

        self.start = ttk.Button(self, text ="Start Logging",command=lambda: lambda: self.handleLoggingCommand("start"))
        self.start.place(relx=1565/2736,rely=1439/1824,relwidth=314/2736,relheight=97/1824)

        #controller type buttons
        self.open = ttk.Button(self, text ="Open Controller", command=lambda: self.startOpenControl())
        self.open.place(relx=55/2736,rely=1113/1824,relwidth=314/2736,relheight=97/1824)

        self.pid = ttk.Button(self, text ="PI Controller", command=lambda: self.startPidControl())
        self.pid.place(relx=392/2736,rely=1113/1824,relwidth=314/2736,relheight=97/1824)

        #tye of controller
        self.controllerTypeText = ttk.Label(self, text='No Controller Type Selected')
        self.controllerTypeText.place(relx=362.0/2736,rely=1230/1824, anchor='n')

        #position projection
        self.projectionWidget = projectPostition(self.canvas)

    def updateDisplay(self, *args):
        global r_des, r_act, y_des, y_act, int_sum, P_act

        #update pressure
        self.channel0Text.configure(text = str(round(P_act[0],3)))
        self.channel1Text.configure(text = str(round(P_act[1],3)))
        self.channel2Text.configure(text = str(round(P_act[2],3)))

        #update postitoin
        self.xPosText.configure(text = str(round(r_act[1],3)))
        self.zPosText.configure(text = str(round(r_act[0],3)))
        self.yPosText.configure(text = str(round(z_act,3)))

        #update projection plot
        self.projectionWidget.updatePosition(round(r_act[1],3), round(r_act[0],3))
        self.projectionWidget.plot()
        # call again after 100 ms
        self.parent.after(100, self.updateDisplay)

    def handleSetXPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        newCmd = command("EM_Sensor", "setXPosition", float(self.xPosEntry.get()))
        commandsFromGUI.put(newCmd)

    def handleSetYPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        newCmd = command("EM_Sensor", "setYPosition", float(self.yPosEntry.get()))
        commandsFromGUI.put(newCmd)

    def handleSetZPositionCommand(self, *args):
        '''
        Handle setting the position from the GUI
        '''
        print('error z entry box not disabled')

    def setChannel0(self, *args):
        '''
        Handle setting the channel in pressure 0
        '''
        newCmd = command("Arduino", "setPressure", 0, float(self.channel0Entry.get())) # arduino.channel0
        commandsFromGUI.put(newCmd)

    def setChannel1(self, *args):
        '''
        Handle setting the channel in pressure 1
        '''
        newCmd = command("Arduino", "setPressure", 1, float(self.channel1Entry.get())) # arduino.channel1
        commandsFromGUI.put(newCmd)

    def setChannel2(self, *args):
        '''
        Handle setting the channel in pressure 2
        '''
        newCmd = command("Arduino", "setPressure", 2, float(self.channel2Entry.get())) # arduino.channel2
        commandsFromGUI.put(newCmd)

    def handleLoggingCommand(self, status):
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

    def startOpenControl(self):
        global cThread

        #check if their is an active controller
        if cThread:
            self.stopPidControl() #close controller

        #start new controller
        cThread = openControllerThread('Thread 1')
        cThread.start()
        #activate gui
        self.controllerTypeText.configure(text = 'Open Controller Type Selected')
        self.channel0Entry.configure(state="normal")
        self.channel1Entry.configure(state="normal")
        self.channel2Entry.configure(state="normal")

    def stopOpenControl(self):
        global cThread

        #disable gui
        self.controllerTypeText.configure(text = 'No Controller Type Selected')
        self.channel0Entry.configure(state="disable")
        self.channel1Entry.configure(state="disable")
        self.channel2Entry.configure(state="disable")

        cThread.raise_exception()
        cThread.join()
        time.sleep(0.1)

    def startPidControl(self):
        global cThread

        #check if their is an active controller
        if cThread:
            self.stopOpenControl() #close controller

        #start new controller
        cThread = pidControllerThread('Thread 1')
        cThread.start()
        #activate gui
        self.controllerTypeText.configure(text ='PID Controller Type Selected')
        self.xPosEntry.configure(state="normal")
        self.yPosEntry.configure(state="normal")
        self.zPosEntry.configure(state="normal")

    def stopPidControl(self):
        global cThread

        #disable gui
        self.controllerTypeText.configure(text = 'No Controller Type Selected')
        self.xPosEntry.configure(state="disable")
        self.yPosEntry.configure(state="disable")
        self.zPosEntry.configure(state="disable")

        cThread.raise_exception()
        cThread.join()
        time.sleep(0.1)

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
        self.pidFrame = pidTuningWindow(self.navbar)
        self.shalomFrame = shalomWindow(self.navbar)

        self.navbar.pack(expand = 1, fill ="both")
        self.controlFrame.pack(expand = 1, fill ="both")
        self.pidFrame.pack(expand = 1, fill ="both")
        self.shalomFrame.pack(expand = 1, fill ="both")

        self.navbar.add(self.controlFrame, text='Controls')
        self.navbar.add(self.pidFrame, text='PID Tuning')
        self.navbar.add(self.shalomFrame, text='Shalom')
    """
    def setup(self):
        self.controlWindow.startOpenControl()
        newCmd = command("Arduino", "setPressure", 0, float(max_pressure[0]) # arduino.channel0
        commandsFromGUI.put(newCmd)

    """


class openControllerThread(threading.Thread):
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

                self.three_channel_main()
                time.sleep(.07)

        finally:
            print('Controller thread teminated')

    def three_channel_main(self):
        '''
        main function used in thread to perform 3 channel algorithm
        '''
        global P_des, P_act, r_act, csv_logger
        try:
            for channel in range(3):
                if P_des[channel] < 9.0:
                    # lower limit of the pressure we are sending into the controller
                    P_des[channel] = 9.0
                elif P_des[channel] > 17.0:
                    # higher limit of the pressure we are sending into the controller
                    P_des[channel] = 17.0

            # get the actual pressure from the pressure sensor
            P_act[0] = arduino.getActualPressure(arduino.channel0)
            P_act[1] = arduino.getActualPressure(arduino.channel1)
            P_act[2] = arduino.getActualPressure(arduino.channel2)

            # get actual position from EM sensor
            # position = ndi.getPositionInRange()
            r_act[0] = 0 # position.deltaX          # x dim
            r_act[1] = 0 # position.deltaY          # y dim

            # perform 3 channel control algorithm
            # self.three_channel_algorithm()

            # send the desired pressure into Arduino
            arduino.sendDesiredPressure(arduino.channel0, float(P_des[0]))
            arduino.sendDesiredPressure(arduino.channel1, float(P_des[1]))
            arduino.sendDesiredPressure(arduino.channel2, float(P_des[2]))

            # Log all control variables if needed / TODO: find out how to re-implement time_diff variable
            # TODO: figure out if logging works with vectors/matrices
            # logging.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (r_des, r_act, P_des, P_act, k_p, k_i))
            # if logging.getLogger().getEffectiveLevel() == logging.INFO:
            #    csv_logger.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (r_des, r_act, P_des, P_act, k_p, k_i))
        except:
            print("An exception occurred")


    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        global P_des

        if (newCmd.id == "Arduino"):
            if (newCmd.field1 == "setPressure"):
                P_des[newCmd.field2] = newCmd.field3
                # print("Setting channel " + str(newCmd.field2) + " to " + str(newCmd.field3))


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

class pidControllerThread(threading.Thread):
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
        try:
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
        except:
            print("An exception occurred")

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
    #function to calibrate the correct DPI of current computer
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
    # Designate main thread to GUI
    root = tk.Tk()
    style = ThemedStyle(root)
    style.set_theme("black")
    App(root)
    # root.resizable(True, True)
    root.mainloop()

    if cThread:
        # Kill controller once GUI is exited
        cThread.raise_exception()
        cThread.join()


if __name__ == '__main__':
    main()
