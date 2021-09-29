from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio   # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel = 77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
# channel can be 0-83

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

while True:

    # CHECK OUT is_pressed() AND was_pressed() FUNCTIONS
    # FIGURE OUT WHICH FUNCTION IS BETTER SUITED FOR THE

    # ARM THE DRONE USING BOTH BUTTONS
    # need is_pressed for arming, because we press a and then b after some time,
    # this will result in arming.
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b was pressed - arm / disarm depending on current state
        throttle = 5
        if arm == 0:
            arm = 1
            # without arm drone cannot do anything, as soon as both buttons are pressed,
            # send arm command
            command = str(pitch) + "|" + str(arm) + "|" + str(roll) + "|" + str(throttle) + "|" + str(yaw)
            radio.send(command)
            display.clear()
            display.set_pixel(1, 1, 9)
            sleep(50) # to prevent switch bouncing effect
        else:
            display.clear()
            display.set_pixel(0, 0, 9)
            arm = 0
            sleep(50) # to prevent switch bouncing effect

    # Decrease THROTTLE WITH A BUTTON
    # If button a was pressed decrease throttle by 5
    if arm == 1 and button_a.was_pressed():
        throttle = throttle - 5

    # Increase THROTTLE WITH B BUTTON
    # If button b was pressed decrease throttle by 5
    if arm == 1 and button_b.was_pressed():
        throttle = throttle + 5
        #display.scroll(throttle)

    # USE ACCLEREROMETER CLASS FOR DEALING WITH ROLL, PITCH AND YAW (X, Y AND Z AXES)
    #if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
    #    arm = 0
    #    throttle = 0

    # Using accelerometer class to get rotation in x-axis
    roll = accelerometer.get_x()
    # Using accelerometer class to get rotation in y-axis
    yaw = accelerometer.get_z()
    # Using accelerometer class to get rotation in z-axis
    pitch = accelerometer.get_y()

    # Will probably need to normalise roll, pitch and yaw values

    # UPDATE COMMAND STRING TO BE SENT OUT WITH CONCATENATED PARTY COMMANDS

    # do you need to include this at the end of command "\n" ???
    command = str(pitch) + "|" + str(arm) + "|" + str(roll) + "|" + str(throttle) + "|" + str(yaw)
    # display.scroll(str(arm))
    radio.send(command)  # Send command via radio
    #uart.write(command)
    print(command)
    # display.scroll(command)

    sleep(100)
    # sleep() IS YOUR FRIEND, FIND GOOD
    # VALUE FOR LENGTH OF SLEEP NEEDED TO FUNCTION WITHOUT COMMANDS GETTING MISSED BY
    # THE DRONE
