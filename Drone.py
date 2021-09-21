# 21.09.2021

# Aurthor - Keshav Sapkota and Stephen Bryne
# Drone.py


# Write your code here :-)
from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio  # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel=77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0


# Scaling, scale by 3.5 for pitch, roll, throttle and 5 for yaw, arm and flight mode
# values and formulas are obtained from CPS_Lab2 lecture slide
scale1 = 3.5
scale2 = 5
offset1 = 512
offset2 = 521  # for roll

# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP

while True:

    # INITIALISE COMMAND OUTPUT STRING
    incoming = ""

    incoming = radio.receive()

    # need to consider if everything should be inside incoming or can it be outside??
    if incoming:

        parsed_incoming = incoming.split("|", 5)
        pitch = int(parsed_incoming[0])
        arm = int(parsed_incoming[1])
        roll = int(parsed_incoming[2])
        throttle = int(parsed_incoming[3])
        yaw = int(parsed_incoming[4])


    if arm == 1:
        display.clear()
        display.set_pixel(1, 1, 9)
        # command need to be scaled and offset before going into buffer
        pitch = scale1 * pitch + offset1
        roll = scale1 * roll + offset2
        throttle = scale1 * throttle + offset1
        yaw = scale2 * yaw + offset1

        if arm == 0:
            arm = 0
        else:
            arm = scale2 * arm + offset1

        throttle = (throttle * offset1) / 50

        # initialize UART communication
        uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin0, rx=pin1)


    if arm == 0:
        # pixel (0,0) lights up.
        display.clear()
        display.set_pixel(0, 0, 9)


    sleep(500)
