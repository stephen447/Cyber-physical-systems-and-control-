'''****************************************************************************
*** Write what this file does here
****************************************************************************'''

from microbit import *
import radio
import micropython
import math
import utime


radio.on()
radio.config(channel = 78, address = 0x55555555, group = 9)
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
micropython.kbd_intr(-1)


count = 0
count_interval = 200


# Battery parameters
battery:float = 0
total_battery:float = 0
avg_battery:float = 0
true_battery:float = 0
battery_msg = ""


# initialise party commands
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0


# Pins for rotary encoder and joystick
# Pins connected to display require display to be off
'''
encoder_pinB = pin6
encoder_pinA = pin7
pitch_pin = pin1
roll_pin = pin2
display.off()
'''

'''****************************************************************************
Display battery level received from drone
****************************************************************************'''
def display_battery_level(b)->none:

    battery_percent = ((b-300)/(1023-300))

    if battery_percent >= 0.6 and battery_percent < 0.8:
        display.set_pixel(4,0,0)
        display.set_pixel(4,1,9)
        display.set_pixel(4,2,9)
        display.set_pixel(4,3,9)
        display.set_pixel(4,4,9)

    elif battery_percent >= 0.4 and battery_percent < 0.6:
        display.set_pixel(4,0,0)
        display.set_pixel(4,1,0)
        display.set_pixel(4,2,9)
        display.set_pixel(4,3,9)
        display.set_pixel(4,4,9)

    elif battery_percent >= 0.2 and battery_percent < 0.4:
        display.set_pixel(4,0,0)
        display.set_pixel(4,1,0)
        display.set_pixel(4,2,0)
        display.set_pixel(4,3,9)
        display.set_pixel(4,4,9)

    elif battery_percent < 0.2:
        display.show(Image.SKULL)

    else:
        display.set_pixel(4,0,9)
        display.set_pixel(4,1,9)
        display.set_pixel(4,2,9)
        display.set_pixel(4,3,9)
        display.set_pixel(4,4,9)



'''****************************************************************************
Mapping function allows conversion from one range to another
****************************************************************************'''
def mapping(value, fromLow, fromHigh, toLow, toHigh):
    a = (toLow - toHigh) / (fromLow - fromHigh)
    b = toHigh - a * fromHigh
    exact = a * value + b
    rest = exact - math.floor(exact)
    if rest > 0.5:
        return math.ceil(exact)
    else:
        return math.floor(exact)




'''****************************************************************************
Main loop
****************************************************************************'''
display.set_pixel(0, 0, 9)
while True:
    # receive battery message
    #battery_msg = radio.receive()
    #print(battery_msg)

    if battery_msg :
        battery = float(battery_msg)
        display_battery_level(battery)
        total_battery = total_battery + battery
        #print("Battery level:", (battery / 1023) * 3.3, "V")


    # Failsafe - disarm for emergency stop
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        arm = 0
        throttle = 0
        # first string is blank, just to be on the same side, data from second string
        command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)


    # Arm drone when both buttons are pressed
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b is pressed - arm / disarm depending on current state
        if arm == 0:
            arm = 1
            throttle = 0
            # Throttle needs to be 0 for arming
            command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
            radio.send(command)
            display.set_pixel(0, 0, 0)
            display.set_pixel(1, 1, 9)
            sleep(500) #to prevent switch bouncing effect
        else:
            throttle = 0
            arm = 0
            command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(0)+","+str(yaw)
            radio.send(command)
            display.set_pixel(1, 1, 0)
            display.set_pixel(0, 0, 9)
            sleep(500) # to prevent switch bouncing effect

    if arm == 1:
        # Throttle
        # If button b was pressed decrease throttle by 5
        if button_b.was_pressed():
            throttle += 5
        if button_a.was_pressed():
            throttle -= 5

        if throttle > 100 or throttle == 100 : throttle = 100
        if throttle < 0 or throttle == 0 : throttle = 0

        #throttle = mapping(throttle, 0,100, 0, 1023)

        print(throttle)

        # Roll
        #roll=mapping(accelerometer.get_x(),-1024,1024,-90,90)
        roll=-mapping(int(pin0.read_analog()),0,1023,-90,90)
        if roll>90: roll=90
        if roll<-90: roll=-90
        print("roll ", roll)


        # Pitch
        #pitch=-mapping(accelerometer.get_y(),-1024,1024,-90,90)
        pitch=-mapping(int(pin1.read_analog()),0,1023,-90,90)
        if pitch>90: pitch=90
        if pitch<-90: pitch=-90
        print("pitch ", pitch)
        #throttle = int(  throttle_pid_control())

        command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(0)
        radio.send(command)  # Send command via radio

    sleep(10)




    """
    Information
    1) No print statements in the Drone code
    2) To prevent Switch bouncing add sleep(500) inside both button pressed function
    3) In general keep sleep(10) in both sides
    6) Don't use display.scroll function in transmitter
    7) Throttle should be 0 for arming
    8) Wait few secs to arm, disarm, etc.
    4) Throttle = 40-45 is the point of lift-off, half of joystick action used at this point.
    6) Think of approaches to save battery - things like going to sleep (coz radio is contantly working)
        decreasing throttle to low value while testing, etc.
    7) More convenient to map roll and pitch between -90 and 90.
    8) Some of the pins are connected to LEDs. To access the pins display must be off.
    display.off()
    """
    '''
    # Calculate dt
    t2 = utime.ticks_ms()
    dt = t2 - t1
    t1 = t2
    # reset timer after 2 mins to prevent overflow
    if (t2 >= 120000):
        t2 = 0
        t1 = 0
    '''
