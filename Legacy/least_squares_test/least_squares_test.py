#{
# Filename: matrix_math_test.py
# Author: William Wang (Team 7 Medtronic)
# Description: This script tests the linear algebra capabilities for the 3 channel control algorithm
# NOTE: 1) If we use the bounded version of the least squares and the bounds are not large enough to give
#       solution, then the lsq_linear() function will bin the solution to the bounds (not ideal for accurate
#       representations of the solution). ----> use unbounded version
#       2) The execution time for the unbounded solution is much faster than the bounded solution
#       ----> use the unbounded version
# }

import numpy as np
import scipy.optimize as sp_opt
from math import sqrt
import time

# creation of test arrays
A = np.array([[1, 1], [2, 8]])
B = np.array([[5, 11, -21], [12, 19, 0]])
# print(A)
# print(B)

# ---- test matrix operatons of the test arrays ---- #
# matrix multiplication
C = A.dot(B)
# print(C)

# transpose of matrix
A_trans = A.transpose()
# print(A_trans)

# inverse of matrix
A_inv = np.linalg.inv(A)
# print(A_inv)

# pseudoinverse of matrix
B_pinv = np.linalg.pinv(B)
# print(B_pinv)

# ----- Attempt to use the least squares function to find solution -----#
# array that contains C1, C2, C3 unit vectors
A = np.array([[sqrt(3)/2, -sqrt(3)/2, 0], [1/2, 1/2, -1]])
# print(A)

# measured error from the sensor
b = [-71.298659660359900, 86.354318375361270]

# bounds on the solution (NOTE: if the bounds are too small to solve for the soluton
# the lsq_linear function returns a combination of the max of the bounds)
lb = np.array([-15, -15, -15])
# print(lb)
ub = -1*lb
# print(ub)

# perform least squares optimization (unbounded)
t_start = time.time()
res = sp_opt.lsq_linear(A, b)
t_stop = time.time()
print("x unbounded solution: ") 
print(res.x)
print("Execution time (unbounded): ")
print(t_stop - t_start)

# calculate for error vector to see if it matches "measured" error
e = A.dot(res.x)
print("Recalculated e vector solution (unbounded): ")
print(e)
print("")

# perform least squares optimization (bounded)
t_start = time.time()
res2 = sp_opt.lsq_linear(A, b, bounds = (lb, ub))
t_stop = time.time()
print("x bounded solution: ")
print(res2.x)
print("execution time (bounded): ")
print(t_stop - t_start)

# calculate for error vector to see if it matches "measured" error
e2 = A.dot(res2.x)
print("Recalculated e vector solution (bounded): ")
print(e2)