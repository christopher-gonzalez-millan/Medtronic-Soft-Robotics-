# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 18:39:27 2022

@author: Christopher
"""
import threading
from tkinter import *
from tkinter import ttk
from ttkthemes import ThemedStyle

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
            with open('data'):
                pass
            
            
# Designate main thread to GUI
root = Tk()
style = ThemedStyle(root)
style.set_theme("equilux")
GUI(root)
root.mainloop()