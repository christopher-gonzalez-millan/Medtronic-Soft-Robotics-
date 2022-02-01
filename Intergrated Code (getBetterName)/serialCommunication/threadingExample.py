# -*- coding: utf-8 -*-
"""
Created on Wed Jan 26 16:29:29 2022

@author: Christopher
"""
import threading

def printer():
    while True:
        print("hello")
    
t1 = threading.Thread(target=printer)
t1.start()
    

def askinput():
    userInput = input()

    if userInput =="h":
        print(1)
    elif userInput =="l":
        print(2)
    elif userInput =="quit" or userInput == "q":
        print("Exiting")
        
    return True

while askinput():
    pass
