'''
 * @file    one_channel_prop_control.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Basic 1D proportional controller
'''
import NDISensor
import arduino_control
import threading
from queue import Queue
import ctypes
import time
from tkinter import *
from tkinter import ttk
from ttkthemes import ThemedStyle
import logging

# logging.basicConfig(level = logging.DEBUG)
logging.basicConfig(filename = 'data.log', level = logging.WARNING, 
    format = '%(asctime)s,%(message)s')

# Init EM Nav and Arduino
ndi = NDISensor.NDISensor()
arduino = arduino_control.arduino()

# Parameters for controller
z_des = 40.0     # stores the desired z position input by user
z_act = 0.0     # actual z_position from EM sensor
k_p = .015       # proportional controller gain
P_act = 0.0     # actual pressure read from the pressure sensor
P_des = 12.0    # desired pressure we're sending to the Arduino
dT = 0.125      # time between cycles (seconds) # TODO: find a way to clock the cycles to get this value
int_sum = 0.0    # sum of the integral term # TODO: figure out if this should be a global value
epsi_z_prev = 0.0 # error in z for the previous time step # TODO: figure out if this should be a global value
k_i = 0.01         # integral gain # TODO: figure out how to pass in integral gain and what is best gain value

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
        if (status == "start"):
            logging.getLogger().setLevel(logging.INFO)
        elif (status == "stop"):
            logging.getLogger().setLevel(logging.WARNING)
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
                
                self.one_D_main() 
                time.sleep(.07)

        finally:
            print('Controller thread teminated')
    
    def one_D_main(self):
        '''
        main function used in thread to perform 1D algorithm
        '''
        global P_act, z_act

        # get the actual pressure from the pressure sensor
        P_act = arduino.getActualPressure()

        # get actual position from EM sensor
        position = ndi.getPositionInRange()
        z_act = position.deltaX

        # perform 1D proportional control
        self.one_D_algorithm()

        # send the desired pressure into Arduino
        self.sendDesiredPressure()

        # Log all control variables if needed
        logging.info('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (z_des, z_act, P_des, P_act, k_p, k_i))

    def one_D_algorithm(self):
        '''
        Proportional feedback loop algorithm (includes our method and Shalom's del P)
        '''
        global z_des, z_act, P_des, P_act, k_p, dT, int_sum, epsi_z_prev, k_i

        # Calculate the error between current and desired positions
        epsi_z = z_des - z_act

        # Calculate the integral sum
        int_sum = int_sum + 0.5*(epsi_z + epsi_z_prev)*dT

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
    
    def sendDesiredPressure(self):
        '''
        convert P_des and send this pressure into the Arduino
        '''
        global P_des
        if P_des < 9.0:
            # lower limit of the pressure we are sending into the controller
            P_des = 9.0
        elif P_des > 13.25:
            # higher limit of the pressure we are sending into the controller
            P_des = 13.25

        arduino.sendDesiredPressure(P_des)

          
    def handleGUICommand(self, newCmd):
        '''
        Function to handle commands from the GUI.
        Takes place on controller thread
        '''
        global z_des, k_p, k_i

        if (newCmd.id == "EM_Sensor"):
            if (newCmd.field1 == "setPosition"):
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