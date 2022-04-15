'''
 * @file    three_channel_feedback_test.py
 * @author  CU Boulder Medtronic Team 7
 * @brief   Test to verify the three channel feedback controller is working
'''

import scipy.optimize as sp_opt
import numpy as np
from math import sqrt

P_des = np.array([12.25, 12.25, 12.25])     # desired pressure we're sending to the Arduino (c0, c1, c2)
P_act = np.array([0.0, 0.0, 0.0])           # actual pressure read from the pressure sensor (c0, c1, c2)
r_des = np.array([0.0, 0.0])                # desired position of robot in form (x, y)
r_act = np.array([0.0, 0.0])                # actual position of the robot using EM sensor (x, y)
k_p = np.array([.012, .012, .012])          # proportional controller gain for c0, c1, c2
err_r = np.array([0.0, 0.0])                # error between measured position and actual position
epsi = np.array([0.0, 0.0, 0.0])            # stores the solution to the force vector algorithm
epsi_prev = np.array([0.0, 0.0, 0.0])       # modified error in r (after force vector solution) for the previous time step # TODO: figure out if this should be a global value

def three_channel_main():
    '''
    main function used in thread to perform 3 channel algorithm
    '''
    global P_act, r_act, r_des

    # get the actual pressure from the pressure sensor
    P_act[0] = float(input("P_act[0] = "))
    P_act[1] = float(input("P_act[1] = "))
    P_act[2] = float(input("P_act[2] = "))

    # get the desired position from the user
    r_des[0] = float(input("r_des[0] = "))
    r_des[1] = float(input("r_des[1] = "))

    # get actual position from EM sensor
    r_act[0] = float(input("r_act[0] = "))          # x dim
    r_act[1] = float(input("r_act[1] = "))          # Z dim

    # perform 3 channel control algorithm
    three_channel_algorithm()

    # send the desired pressure into Arduino
    sendDesiredPressure()

def three_channel_algorithm():
    '''
    Proportional/PI feedback loop algorithm (vector based solution -- includes dot product and bounded least squares solution)
    '''
    global r_des, r_act, err_r, P_des, P_act, k_p, epsi, epsi_prev

    # Calculate the error between current and desired positions
    err_r = r_des - r_act
    print("err_r = ", err_r)

    # perform force vector calculations
    forceVectorCalc()

    # < ------- Feedback controller --------- >
    del_P_des = k_p*epsi                                # should be element-wise operations / TODO: check the matrix math here
    P_des = P_act + del_P_des
    print("del_P_des: ", del_P_des)
    print("P_des: ", P_des)

    # Update the error value for next iteration of epsi_prev
    epsi_prev = epsi

def forceVectorCalc():
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
    print("epsi: ", epsi)

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

def sendDesiredPressure():
    '''
    convert P_des and send this pressure into the Arduino
    '''
    global P_des
    for i in range(len(P_des)):
        # TODO: check the range limits for the pressure being sent for the smaller robot
        if P_des[i] < 9.0:
            # lower limit of the pressure we are sending into the controller
            P_des[i] = 9.0
        elif P_des[i] > 15.5:
            # higher limit of the pressure we are sending into the controller
            P_des[i] = 15.5

    # send each channel pressure
    print("P_des[0] sent = ", P_des[0])
    print("P_des[1] sent = ", P_des[1])
    print("P_des[2] sent = ", P_des[2])

def main():
    three_channel_main()

if __name__ == "__main__":
    main() 
