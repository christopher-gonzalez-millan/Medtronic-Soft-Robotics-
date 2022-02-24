# This is a test script for communicating between Python and Arduino
# The code follow this website https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0
# This is the Python side of the code

# Importing Libraries
import serial
import time

arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value
