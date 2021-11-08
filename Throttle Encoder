from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import micropython
import math
# Write your code here :-)
a_prev = 0
b_prev = 0
throttle = 0

while True:
    a_curr = pin2.read_analog() #Read in signal A
    print("Current A: ", a_curr)
    b_curr = pin1.read_analog() #Read in signal B
    print("Current B: ", b_curr)
    if (abs(a_curr - a_prev) > 900)|((b_curr - b_prev) > 900): #If there was a level change
        if abs(a_curr - a_prev) > 900: # If A rose before B, increase throttle
                throttle = throttle + 5
        if abs(b_curr - b_prev) > 900: #If B rose before A, decrease throttle
                throttle = throttle - 5
        sleep(3000) # Sleep to allow values to stabilise
                
     
    if throttle > 100: #Limit max throttle to 100
        throttle = 100
    if throttle < 0: #Limit min throttle to 0
        throttle = 0
    a_prev = a_curr #Save current value as the previous value for next loop
    b_prev = b_curr #Save current value as the previous value for next loop
    #print("Throttle
    #display.scroll(throttle)
    sleep(200) #Sleep for controlling rate of signals being read in

