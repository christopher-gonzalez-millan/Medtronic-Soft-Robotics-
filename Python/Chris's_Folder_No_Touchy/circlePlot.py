# -*- coding: utf-8 -*-
"""
Created on Tue Apr 12 11:24:24 2022

@author: Christopher
"""
import numpy as np 
import matplotlib.pyplot as plt

def defineCircle(p1, p2, p3):
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

def inCircle(cx, cy, radius, x, y):
    if pow((x - cx), 2) + pow((y - cy), 2) <= pow(radius, 2):
        return True
    return False
        
p = (0,0)
center, radius = defineCircle((0,1), (1,0), (0,-1))
print(inCircle(center[0], center[1], radius, p[0], p[1]))
if center is not None:
    plt.figure(figsize=(4, 4))
    plt.xlim([-1.2, 1.2])
    plt.ylim([-1.2, 1.2])
    plt.plot(p[0], p[1],'ro') 
    circle = plt.Circle(center, radius, fill=False)
    plt.gca().add_patch(circle)