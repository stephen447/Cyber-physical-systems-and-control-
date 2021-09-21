from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio   # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel = 77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0

# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
display.show(Image.HAPPY)

while True:

    # INITIALISE COMMAND OUTPUT STRING
    command = ""

    # CHECK OUT is_pressed() AND was_pressed() FUNCTIONS
    # FIGURE OUT WHICH FUNCTION IS BETTER SUITED FOR THE
    # CONTROLLER - "WAS" better as using
    # "IS" could be caught out by sleep timer

    # ARM THE DRONE USING BOTH BUTTONS
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b was pressed - arm / disarm depending on current state
        throttle = 0
        if arm == 0:
            arm = 1
        else:
            arm = 0

    # Decrease THROTTLE WITH A BUTTON
    # If button a was pressed decrease throttle by 5
    if arm == 1 and button_a.is_pressed():
        throttle -= 5

    # Increase THROTTLE WITH B BUTTON
    # If button b was pressed decrease throttle by 5
    if arm == 1 and button_b.is_pressed():
        throttle += 5

    # USE ACCLEREROMETER CLASS FOR DEALING WITH ROLL, PITCH AND YAW (X, Y AND Z AXES)
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        arm = 0
        throttle = 0

    # Using accelerometer class to get rotation in x-axis
    roll = accelerometer.get_x()
    # Using accelerometer class to get rotation in y-axis
    yaw = accelerometer.get_z()
    # Using accelerometer class to get rotation in z-axis
    pitch = accelerometer.get_y()

    # Will probably need to normalise roll, pitch and yaw values

    # UPDATE COMMAND STRING TO BE SENT OUT WITH CONCATENATED PARTY COMMANDS

    command = str(pitch) + "|" + str(arm) + "|" + str(roll) + "|" + str(throttle) + "|" + str(yaw)
    # display.scroll(str(arm))
    radio.send(command)  # Send command via radio

    sleep(100)
    # Picked 10, as most common in online examples. sleep() IS YOUR FRIEND, FIND GOOD
    # VALUE FOR LENGTH OF SLEEP NEEDED TO FUNCTION WITHOUT COMMANDS GETTING MISSED BY
    # THE DRONE
