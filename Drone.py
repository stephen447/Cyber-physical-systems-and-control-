# Write your code here :-)
# 21.09.2021

# Aurthor - Keshav Sapkota and Stephen Bryne
# Drone.py

# Write your code here :-)
from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio  # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel=1)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
micropython.kbd_intr(-1)  # enabling or disabling keyboard interrupt

#initialize UART communication
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin2)
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0
flight_mode = 1
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

buffer = bytearray(16)

# NEED TO MAKE SURE BUFFER HAS THE VALUES WE EXPECT AND NO UNUSUAL CHARACTERS
# USE REPL TO READ FROM SERIAL, AND SET UART PINS TO NONE WHILE READING FROM UART
def load_buffer():
    print("inside load_buffer function")

    # Given
    buffer[0] = 0
    buffer[1] = 0x01

    # Roll
    buffer[2] = (roll_id << 2) | ((scaled_roll >> 8) & 3)
    print("buf2 ", buffer[2])
    buffer[3] = 255 & scaled_roll

    # Pitch
    buffer[4] = (pitch_id << 2) | ((scaled_pitch >> 8) & 3)
    buffer[5] = 255 & scaled_pitch

    # Throttle
    buffer[6] = (throttle_id << 2) | ((scaled_throttle >> 8) & 3)
    #print(int((throttle_id << 2) | ((3 & scaled_throttle) >> 8)))
    buffer[7] = 255 & scaled_throttle

    # Yaw
    buffer[8] = (yaw_id << 2) | ((scaled_yaw >> 8) & 3)
    buffer[9] = 255 & scaled_yaw

    # Arm
    buffer[10] = (arm_id << 2) | ((scaled_arm >> 8) & 3)
    buffer[11] = 255 & scaled_arm

    # Flightmode
    buffer[12] = (flight_mode_id << 2) | ((scaled_flight_mode >> 8) & 3)
    buffer[13] = 255 & scaled_flight_mode

    # Buzzer
    buffer[14] = (buzzer_id << 2) | ((scaled_buzzer >> 8) & 3)
    buffer[15] = 255 & scaled_buzzer

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

        parsed_incoming = incoming.split(",")
        print("Parsed incoming", parsed_incoming)

        pitch = int(parsed_incoming[0])
        arm = int(parsed_incoming[1])
        roll = int(parsed_incoming[2])
        throttle = int(parsed_incoming[3])
        yaw = int(parsed_incoming[4])
        #display.scroll(throttle)


        # scaling and offsetting
        scaled_pitch = int((scale1 * pitch) + offset1)
        scaled_roll = int((scale1 * roll) + offset2)
        scaled_yaw = int((scale2 * yaw) + offset1)
        scaled_flight_mode = int(180 * scale2)
        scaled_throttle = int (2 * scale1 * throttle + offset1 / 2.5)
        #scaled_throttle = int((scale1 * throttle) + offset1)
        #scaled_throttle = int((scaled_throttle * offset1) / 50)  # round to nearest decimal
        scaled_buzzer = 0
        print (scaled_pitch, scaled_arm, scaled_roll, scaled_throttle, scaled_yaw, scaled_flight_mode, scaled_buzzer)

        if arm == 1:
            scaled_arm = int(180 * scale2)
            display.clear()
            display.set_pixel(1, 1, 9)
        elif arm == 0:
            scaled_arm = 0
            display.clear()
            display.set_pixel(0, 0, 9)

        # load buffer
        print("loading buffer")
        load_buffer()
        print("writing to buffer")
        uart.write(buffer)

    print("sleep")
    sleep(1500)

    '''
        print("PARTY")
        print(scaled_pitch)
        print(scaled_arm)
        print(scaled_roll)
        print(scaled_throttle)
        print(scaled_yaw)
        print(scaled_flight_mode) '''

