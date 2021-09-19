from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import microbit as m
import radio   # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config()  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
radio_group = 7  # Radio group
radio.set_group(radio_group)  # Set radio group


# INITIALISE COMMANDS (PARTY)
Pitch = 0
Arm = 0
Roll = 0
Throttle = 0
Yaw = 0


# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP

while True:

    # INITIALISE COMMAND OUTPUT STRING
    command = ""
# ARM THE DRONE USING BOTH BUTTONS
# INCREASE THROTTLE WITH B BUTTON
# CHECK OUT is_pressed() AND was_pressed() FUNCTIONS
# FIGURE OUT WHICH FUNCTION IS BETTER SUITED FOR THE CONTROLLER - "WAS" better as using
# "IS" could be caught out by sleep timer
    if button_a.was_pressed() and button_b.was_pressed():
        # if button a and b was pressed - Arm / disarm depending on current state
        throttle = 0
        if Arm == 0:
            Arm = 1
        else:
            Arm = 0
        break
    elif button_a.was_pressed():  # If button a was pressed increase throttle by 5
        Throttle -= 5
    elif button_b.was_pressed():  # If button b was pressed decrease throttle by 5
        Throttle += 5
# USE ACCLEREROMETER CLASS FOR DEALING WITH ROLL, PITCH AND YAW (X, Y AND Z AXES)
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        Arm = 0
        Throttle = 0

Roll = m.accelerometer.get_x()  # Using accelerometer class to get rotation in x-axis
Yaw = m.accelerometer.get_y()  # Using accelerometer class to get rotation in y-axis
Pitch = m.accelerometer.get_z()  # Using accelerometer class to get rotation in z-axis

# Will probably need to normalise Roll, Pitch and Yaw values

# UPDATE COMMAND STRING TO BE SENT OUT WITH CONCATENATED PARTY COMMANDS
command = Pitch, Arm, Roll, Throttle, Yaw

radio.send(command)  # Send command via radio

sleep(10)
# Picked 10, as most common in online examples. sleep() IS YOUR FRIEND, FIND GOOD
# VALUE FOR LENGTH OF SLEEP NEEDED TO FUNCTION WITHOUT COMMANDS GETTING MISSED BY
# THE DRONE

