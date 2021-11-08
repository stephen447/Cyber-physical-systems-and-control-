from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import micropython
import math

# Write your code here :-)
a_prev = 0
b_prev = 0
throttle = 0

uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
micropython.kbd_intr(-1)  # enabling or disabling keyboard interrupt

#print("Current B: ", b_curr)

threshold = 500
while True:
    a_prev = pin2.read_analog() #Read in signal A
    b_prev = pin1.read_analog() #Read in signal B

    #print((a_prev,b_prev,0))
    sleep(10)

    a_curr = pin2.read_analog() #Read in signal A
    b_curr = pin1.read_analog() #Read in signal B

    a_diff = abs(a_curr - a_prev)
    b_diff = abs(b_curr - b_prev)

    counter = 1
    if (a_diff > threshold) or (b_diff > threshold): #If there was a level change
        #print("transition")
        
        
        # if a is high and b is low
        if a_curr > threshold and b_curr < threshold and (a_curr-a_prev>threshold): # If A rose before B, increase throttle
            throttle += 5
            print("increase")
            sleep(20)

        # if b is high and a is low
        if b_curr > threshold and a_curr < threshold and (b_curr-b_prev>threshold):  #If B rose before A, decrease throttle
            throttle -= 5
            print("*")
            sleep(20)

    if throttle > 100: #Limit max throttle to 100
        throttle = 100
    if throttle < 0: #Limit min throttle to 0
        throttle = 0
    print("Throttle", throttle)
    #print((a_curr,0,0))
    #print((0,0,throttle))
    #display.scroll(throttle)
