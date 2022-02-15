import NDISensor
import serial as pys
import time
import serial.tools.list_ports
import threading


class globalVars():
    pass

G = globalVars() #technically not a gloabl var
G.lock = threading.Lock() #not really necessary in this case, but useful none the less
G.value = 0
G.kill = False
G.zdes = None

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

def floatToCommand(float):
    temp = str(float)
    temp = temp.replace('.', '')

    if len(temp) == 2:
        temp = '0' + temp

    if len(temp) == 3:
        temp = temp + '0'

    return temp

def one_D_feedback(z_des, z_act):
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

def runFeedback():
    while not G.kill:
        try:
            position = ndi.getPosition()
            pressure =  one_D_feedback(G.zdes, position.deltaZ)
            pressure = round(pressure,2)
            pressure = floatToCommand(pressure)
            bytesSent = ser.write(pressure.encode('utf-8'))
            #time.sleep(stabilizationTime)

        except:
            print("There was an issue with NDISensor's parser")


t1 = threading.Thread(target=runFeedback)
t1.start()

def askInput():
    userInput = input('Desire Position')

    if userInput =="quit" or userInput == "q":
        print("Exiting")
        ser.write(b'L')
        ser.close()
        G.kill = True
        return False

    if 'g' in userInput:
        gainValue = float(userInput.replace('g', ''))

    elif userInput:
        G.zdes = userInput

    return True

while askInput():
    pass

ser.close()
