# -*- coding: utf-8 -*-
"""
Created on Thu Dec  2 10:20:19 2021

@author: Christopher
"""

import numpy as np
import csv

from pyquaternion import Quaternion
from matplotlib import pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D


def readQuaternion(filename = 'random_movement.csv'): 
#def readCSV(filename = 'y90_test1.csv'):     
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
    
        for row in reader:
            yield Quaternion(np.array(row[5:9])), np.array(list(map(float, row[9:12])))
            
            
quaternion_generator = readQuaternion()


# Set up figure & 3D axis for animation
fig = plt.figure()
ax = fig.add_axes([0, 0, 1, 1], projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
#ax.axis('off')

# use a different color for each axis
colors = ['r', 'g', 'b']

# set up lines and points
lines = sum([ax.plot([], [], [], c=c)
             for c in colors], [])

startpoints = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
endpoints = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])

# prepare the axes limits
"""
ax.set_xlim([-13, -3])
ax.set_ylim([-17, 0])
ax.set_zlim([15, 35])
"""
ax.set_xlim([-38, 53])
ax.set_ylim([-98, 88])
ax.set_zlim([45, 189])


# set point-of-view: specified by (altitude degrees, azimuth degrees)
ax.view_init(30, 0)

# initialization function: plot the background of each frame
def init():
    for line in lines:
        line.set_data(np.array([]), np.array([]))
        line.set_3d_properties(np.array([]))

    return lines

# animation function.  This will be called sequentially with the frame number
def animate(i):
    # we'll step two time-steps per frame.  This leads to nice results.
    #i = (2 * i) % x_t.shape[1]
    
    q,p = next(quaternion_generator)
    print("i:{} q:{}".format(i,q))


    for line, start, end in zip(lines, startpoints, endpoints):
        #end *= 5
        start = q.rotate(start)
        end = q.rotate(end)
        
        start = np.add(start, p)
        end = np.add(end , p)
        
        #print("line:{} start:{} end:{}".format(line, start, end))

        line.set_data(np.array([start[0], end[0]]), np.array([start[1], end[1]]))
        line.set_3d_properties(np.array([start[2], end[2]]))

        #pt.set_data(x[-1:], y[-1:])
        #pt.set_3d_properties(z[-1:])

    #ax.view_init(30, 0.6 * i)
    fig.canvas.draw()
    return lines

    
# instantiate the animator.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=1215, interval=25, blit=False)

# Save as mp4. This requires mplayer or ffmpeg to be installed
#anim.save('lorentz_attractor.mp4', fps=15, extra_args=['-vcodec', 'libx264'])

plt.show()