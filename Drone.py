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
flight = 0
buzzer = 0


# Scaling, scale by 3.5 for pitch, roll, throttle and 5 for yaw, arm and flight mode
# values and formulas are obtained from CPS_Lab2 lecture slide
scale1 = 3.5
scale2 = 5
offset1 = 512
offset2 = 521  # for roll


# channel ID, Each need
roll_id = 0
pitch_id = 1 << 2
throttle_id = 2 << 2
yaw_id = 3 << 2
arm_id = 4 << 2
flight_mode_id = 5 << 2
buzzer_id = 6 << 2

# buffer
buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

def load_buffer():
    #mask_lsb = 0b00111111
    #mask_msb = 0b11000000
    mask_lsb = 63
    mask_msb = 192

    #Given
    buffer[0] = 0
    buffer[1] = 0x01

    #Roll
    buffer[2] = (roll_id << 2) + ((mask_msb & roll) >> 6)
    buffer[3] = mask_lsb & roll

    #Pitch
    buffer[4] = (pitch_id << 2) + ((mask_msb & pitch) >> 6)
    buffer[5] = mask_lsb & pitch

    #Throttle
    buffer[6] = (throttle_id << 2) + ((mask_msb & throttle) >> 6)
    buffer[7] = mask_lsb & throttle

    #Yaw
    buffer[8] = (yaw_id << 2) + ((mask_msb & yaw) >> 6)
    buffer[9] = mask_lsb & yaw

    #Arm
    buffer[10] = (arm_id << 2) + ((mask_msb & arm) >> 6)
    buffer[11] = mask_lsb & arm

    #Flightmode
    buffer[12] = (flight_mode_id << 2) + ((mask_msb & flight) >> 6)
    buffer[13] = mask_lsb & flight

    #Buzzer
    buffer[14] = (buzzer_id << 2) + ((mask_msb & buzzer) >> 6)
    buffer[15] = mask_lsb & buzzer


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


        load_buffer()

        buffer = bytearray(buffer)
        # initialize UART communication
        uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin0, rx=pin1)




    if arm == 0:
        # pixel (0,0) lights up.
        display.clear()
        display.set_pixel(0, 0, 9)


    sleep(500)
