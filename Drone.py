# 21.09.2021

# Aurthor - Keshav Sapkota and Stephen Bryne
# Drone.py

# Write your code here :-)
from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio  # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel=77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
micropython.kbd_intr(-1) # enabling or disabling keyboard interrupt

#initialize UART communication
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin0)
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0
flight = 0
buzzer = 0

scaled_roll = 0
scaled_pitch = 0
scaled_buzzer = 0
scaled_flight_mode = 0
scaled_throttle = 0
scaled_arm = 0
scaled_yaw = 0

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

incoming = ""

# buffer
#buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#buffer = bytearray(16)

#buffer = []
#buffer.extend(range(0,16,1))
#buffer = bytearray(buffer)
buffer = bytearray(16)


# NEED TO MAKE SURE BUFFER HAS THE VALUES WE EXPECT AND NO UNUSUAL CHARACTERS
# USE REPL TO READ FROM SERIAL, AND SET UART PINS TO NONE WHILE READING FROM UART
def load_buffer():
    print("inside load_buffer function")
    # mask_lsb = 0b0011111111
    mask_lsb = 255 # retrieve last 8-bits
    mask_msb = 3 # right shift by 8, and use mask_msb

    # Given
    buffer[0] = 0
    buffer[1] = 0x01

    # Roll
    # not sure why buffer[3] is written as t.
    buffer[2] = (roll_id << 2) | ((scaled_roll >> 8) & mask_msb)
    temp = mask_lsb & scaled_roll
    buffer[3] = temp
    print(temp)
    print(mask_lsb & scaled_roll)
    print(buffer[3])
    # WHEN ROLL WAS SET TO 0 IN TRANSMITTER, EVEN THOUGH TEMP WAS 9,
    # BUFFER[3] CAME OUT AS 't', COULDN'T UNDERSTAND WHAT WAS GOING ON
    # THAT'S WHY REMOVED ROLL, PITCH, YAW = 0 IN TRANSMITTER.

    # Pitch
    buffer[4] = (pitch_id << 2) | ((scaled_pitch >> 8) & mask_msb)
    buffer[5] = mask_lsb & scaled_pitch

    # Throttle
    buffer[6] = (throttle_id << 2) | ((scaled_throttle >> 8) & mask_msb)
    #print(int((throttle_id << 2) | ((mask_msb & scaled_throttle) >> 8)))
    buffer[7] = mask_lsb & scaled_throttle
    #print(int(mask_lsb & scaled_throttle))
    #buffer[6] = 10
    #buffer[7] = 10

    # Yaw
    buffer[8] = (yaw_id << 2) | ((scaled_yaw >> 8) & mask_msb)
    buffer[9] = mask_lsb & scaled_yaw

    # Arm
    buffer[10] = (arm_id << 2) |  ((scaled_arm >> 8) & mask_msb)
    buffer[11] = mask_lsb & scaled_arm

    # Flightmode
    buffer[12] = (flight_mode_id << 2) | ((scaled_flight_mode >> 8) & mask_msb)
    buffer[13] = mask_lsb & scaled_flight_mode

    # Buzzer
    buffer[14] = (buzzer_id << 2) | ((scaled_buzzer >> 8) & mask_msb)
    buffer[15] = mask_lsb & scaled_buzzer

    print(buffer)


def display_throttle():
    if scaled_throttle > 0:
        display.set_pixel(5, 5, 9)
    if scaled_throttle > 20:
        display.set_pixel(5, 4, 9)
    if scaled_throttle > 50:
        display.set_pixel(5, 3, 9)


# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
while True:

    incoming = radio.receive()

    # need to consider if everything should be inside incoming or can it be outside??
    if incoming:
        #display.scroll(incoming)
        print("incoming")

        parsed_incoming = incoming.split("|", 5)
        print(parsed_incoming)

        pitch = int(parsed_incoming[0])
        arm = int(parsed_incoming[1])
        roll = int(parsed_incoming[2])
        throttle = int(parsed_incoming[3])
        yaw = int(parsed_incoming[4])
        #display.scroll(throttle)

        if arm == 1:
            print("arm == 1")
            display.clear()
            display.set_pixel(1, 1, 9)

            # command need to be scaled and offset before going into buffer
            # NEED TO NORMALISE THESE VALUES BETWEEN 0-1023. I TRIED ALL THEM TO BE 1023 JUST FOR
            # TESTING, WE SHOULD DETERMINE MAX, MIN VALUES FROM ACCELEROMETER, AND NORMALISE BASED ON THAT.

            # TOM MENTIONED THAT SCALED_FLIGHT_MODE SHOULD BE 1023 (NOT SURE WHY)
            scaled_pitch = int((scale1 * pitch) + offset1)
            if scaled_pitch > 1023:
                scaled_pitch = 1023
            scaled_roll = int((scale1 * roll) + offset2)
            if scaled_roll > 1023:
                scaled_roll = 1023

            scaled_throttle = int((scale1 * throttle) + offset1)
            scaled_throttle = int((scaled_throttle * offset1) / 50)  # round to nearest decimal
            if scaled_throttle > 1023:
                scaled_throttle = 1023
            scaled_yaw = int((scale2 * yaw) + offset1)
            if scaled_yaw > 1023:
                scaled_yaw = 1023
            scaled_arm = int(180 * scale2 * arm)
            scaled_flight_mode = int(flight * scale2 + offset1)
            #scaled_flight_mode = 1023
            scaled_buzzer = 0

            print(scaled_pitch)
            print(scaled_arm)
            print(scaled_roll)
            print(scaled_throttle)
            print(scaled_yaw)
            print(scaled_flight_mode)

            #if uart.any():
            #    display.show(Image.HAPPY)

            #display_throttle()

        elif arm == 0:
            # pixel (0,0) lights up.
            print("arm == 0")
            scaled_throttle = 0
            scaled_arm = 0
            display.clear()
            display.set_pixel(0, 0, 9)

        print("loading buffer")
        load_buffer()
        #uart.write(bytes(buffer))
        print("writing in buffer")
        uart.write(buffer)

    print("sleep")
    sleep(150)
