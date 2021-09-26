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
pitch_id = 1
throttle_id = 2
yaw_id = 3
arm_id = 4
flight_mode_id = 5
buzzer_id = 6

# buffer
# buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
buffer = bytearray(16)

def load_buffer():
    # mask_lsb = 0b00111111
    # mask_msb = 0b11000000
    mask_lsb = 255
    mask_msb = 768

    # Given
    buffer[0] = 0
    buffer[1] = 0x01

    # Roll
    buffer[2] = (roll_id << 2) | ((mask_msb & scaled_roll) >> 8)
    buffer[3] = mask_lsb & scaled_roll

    # Pitch
    buffer[4] = (pitch_id << 2) | ((mask_msb & scaled_pitch) >> 8)
    buffer[5] = mask_lsb & scaled_pitch

    # Throttle
    buffer[6] = (throttle_id << 2) | ((mask_msb & scaled_throttle) >> 8)
    buffer[7] = mask_lsb & scaled_throttle

    # Yaw
    buffer[8] = (yaw_id << 2) | ((mask_msb & scaled_yaw) >> 8)
    buffer[9] = mask_lsb & scaled_yaw

    # Arm
    buffer[10] = (arm_id << 2) | ((mask_msb & scaled_arm) >> 8)
    buffer[11] = mask_lsb & scaled_arm

    # Flightmode
    buffer[12] = (flight_mode_id << 2) | ((mask_msb & flight) >> 8)
    buffer[13] = mask_lsb & flight

    # Buzzer
    buffer[14] = (buzzer_id << 2) | ((mask_msb & buzzer) >> 8)
    buffer[15] = mask_lsb & buzzer

def display_throttle():
    if throttle > 0:
        display.set_pixel(5, 5, 9)
    if throttle > 20:
        display.set_pixel(4, 4, 9)
    if throttle > 50:
        display.set_pixel(3, 3, 9)

# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
while True:

    # INITIALISE COMMAND OUTPUT STRING
    incoming = ""

    incoming = radio.receive()

    # need to consider if everything should be inside incoming or can it be outside??
    if incoming:
        # display.scroll(incoming)
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
        scaled_pitch = round(scale1 * pitch + offset1)
        scaled_roll = round(scale1 * roll + offset2)
        scaled_throttle = round(scale1 * throttle + offset1)
        scaled_yaw = round(scale2 * yaw + offset1)
        scaled_arm = round(scale2 * arm + offset1)
        scaled_throttle = round((throttle * offset1) / 50)  # round to nearest decimal

        load_buffer()

        # buffer = bytearray(buffer)
        # initialize UART communication
        uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin2)
        uart.write(buffer)

    # display_throttle()

    if arm == 0:
        # pixel (0,0) lights up.
        display.clear()
        display.set_pixel(0, 0, 9)

    sleep(150)
