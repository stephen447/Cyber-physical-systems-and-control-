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

encoder_pinA = pin4
encoder_pinB = pin10
display.off()
pitch_pin = pin0
roll_pin = pin1

def throttle_encoder():
    global throttle
    threshold = 500
    a_prev = encoder_pinA.read_analog() #Read in signal A
    b_prev = encoder_pinB.read_analog() #Read in signal B

    sleep(10)

    a_curr = encoder_pinA.read_analog() #Read in signal A
    b_curr = encoder_pinB.read_analog() #Read in signal B

    a_diff = abs(a_curr - a_prev)
    b_diff = abs(b_curr - b_prev)

    if (a_diff > threshold) or (b_diff > threshold): #If there was a level change
        # if a is high and b is low
        if a_curr > threshold and b_curr < threshold and (a_curr-a_prev>threshold): # If A rose before B, increase throttle
            throttle += 5
            #print("increase")
            #sleep(10)

        # if b is high and a is low
        if b_curr > threshold and a_curr < threshold and (b_curr-b_prev>threshold):  #If B rose before A, decrease throttle
            throttle -= 5
            #print("*")
            #sleep(10)

    if throttle > 100: #Limit max throttle to 100
        throttle = 100
    if throttle < 0: #Limit min throttle to 0
        throttle = 0
    #print("Throttle", throttle)

def mapping(value, fromLow, fromHigh, toLow, toHigh):
    a = (toLow - toHigh) / (fromLow - fromHigh)
    b = toHigh - a * fromHigh
    exact = a * value + b
    rest = exact - math.floor(exact)
    if rest > 0.5:
        return math.ceil(exact)
    else:
        return math.floor(exact)

while True:
    throttle_encoder()
    print((mapping(throttle,0,100,0,1023),0,0))
    sleep(20)
