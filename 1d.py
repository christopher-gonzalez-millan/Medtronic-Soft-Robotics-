import NDISensor
import serial as pys
import time
import serial.tools.list_ports
import threading
c = threading.Condition() #pretty threading
zDes = None
gainValue = 1.0

"""Setup Variables

These varibale will hopefully never need to be changed
"""
ndi = NDISensor.NDISensor()
ser = pys.Serial()
ser.baudrate = 115200
#TODO find a way to automate finding the COM port.
ser.port = 'COM3'
# Open and check serial
ser.open()
if (not ser.is_open):
    print("Serial Port Could not be Opened")
    quit()

"""Helper Functions

These varibale will hopefully never need to be changed
"""
def floatToCommand(float):
    lessThanTen = False

    if float < 10.0:
        lessThanTen = True

    temp = str(float)
    temp = temp.replace('.', '')

    if lessThanTen:
        temp = '0' + temp

    if len(temp) == 3:
        temp = temp + '0'

    return temp

def one_D_feedback(z_des, z_act, gainValue):
    # define the proportional gain
    k_p = gainValue

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

"""Feedback Thread

These varibale will hopefully never need to be changed
"""
class Feedback(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name

    def runFeedback():
        #locking variables.
        global zDes
        global gainValue

        while True:
            c.acquire()

            try:
                position = ndi.getPosition()
                print("Current Position: {}".format(position.deltaX))
                pressure =  one_D_feedback(zDes, position.deltaX, gainValue)
                pressure = round(pressure,2)
                pressure = floatToCommand(pressure)
                bytesSent = ser.write(pressure.encode('utf-8'))
                time.sleep(0.1)

            except:
                print("There was an issue with NDISensor's parser")
                    #ndi.cleanup()

            c.release()

"""Input Thread

These varibale will hopefully never need to be changed
"""
class Input(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.name = name

    def runInput():
        global zDes
        global gainValue

        while True:
            c.acquire()
            userInput = input('Desire Position: ')


            if userInput =="quit" or userInput == "q":
                print("Exiting")
                ser.close()
                return False

            if 'g' in userInput:
                gainValue =  float(userInput.replace('g', ''))
                print("recivie gain of {}".format(gainValue))


            elif userInput:
                zDes = userInput
                print("recivie positon of {}".format(userInput))

            c.release()

def main():
    '''Starting point for script
    '''

    a = Feedback("feedback")
    b = Input("input")

    b.start()
    a.start()

    a.join()
    b.join()



if __name__ == "__main__":
    main()
