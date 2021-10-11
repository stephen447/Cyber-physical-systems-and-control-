from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio   # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython
import math

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel = 77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
# channel can be 0-83
micropython.kbd_intr(-1)

#initialize UART communication
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0

# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
display.set_pixel(0, 0, 9)

# INITIALISE COMMAND OUTPUT STRING
command = ""

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
    #print("YAW IS THIS ", accelerometer.get_z())

    # Arming
    # ARM THE DRONE USING BOTH BUTTONS
    # Use is_pressed function
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b is pressed - arm / disarm depending on current state
        if arm == 0:
            arm = 1
            print("arming")
            command = str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
            radio.send(command)
            display.clear()
            display.set_pixel(1, 1, 9)
            sleep(100) # to prevent switch bouncing effect
        else:
            throttle = 0
            display.clear()
            display.set_pixel(0, 0, 9)
            arm = 0
            sleep(100) # to prevent switch bouncing effect


    # Increase or decrease throttle
    # If button a was pressed decrease throttle by 5
    throttle=mapping(int(pin2.read_analog()), 512,1023,-20,20)


    # Map throttle
    if throttle > 100: throttle = 100
    if throttle < 0 : throttle = 0


        # Get (x,y,z) coordinates
    # Using accelerometer class to get rotation in x-axis
    # roll = accelerometer.get_x()
    # Using accelerometer class to get rotation in y-axis
    # pitch = accelerometer.get_y()
    # Using accelerometer class to get rotation in z-axis
    # yaw = accelerometer.get_z()

    #roll=mapping(accelerometer.get_x(),-1024,1024,-20,20)
    roll=-mapping(int(pin0.read_analog()),0,1023,-20,20)
    if roll>20: roll=20
    if roll<-20: roll=-20
    print("roll ", roll)

    pitch=-mapping(int(pin1.read_analog()),0,1023,-20,20)
    #pitch=mapping(accelerometer.get_y(),-1024,1024,-20,20)
    if pitch>20: pitch=20
    if pitch<-20: pitch=-20
    print("pitch ", pitch)

    '''yaw=mapping(accelerometer.get_z(),-1024,1024,-90,90)
    if yaw>90: yaw=90
    if yaw<-90: yaw=-90
    yaw=mapping(accelerometer.get_z(),-1024,1024,0,1023)
    if yaw>1023: yaw=1023
    if yaw<0: yaw=0
    print("yaw ", yaw)'''

    yaw = 0


    #failsafe
    # USE ACCLEREROMETER CLASS FOR DEALING WITH ROLL, PITCH AND YAW (X, Y AND Z AXES)
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        arm = 0
        throttle = 0
        display.clear()
        display.set_pixel(0, 0, 9)



    # do you need to include this at the end of command "\n" ???
    command = str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
    print(command)
    radio.send(command)  # Send command via radio
    #display.scroll(throttle)

    # print(command)
    # display.scroll(command)

    sleep(100)
    # sleep() IS YOUR FRIEND, FIND GOOD
    # VALUE FOR LENGTH OF SLEEP NEEDED TO FUNCTION WITHOUT COMMANDS GETTING MISSED BY
    # THE DRONE
