# -*- coding: utf-8 -*-
"""
Created on Tue Feb  8 10:30:54 2022

@author: chgo7806
"""

import NDISensor

test = NDISensor.NDISensor()

try:
    while True:
        pos = test.getPosition()
        if pos:
            pos.getZ()
except:
  test.cleanup()