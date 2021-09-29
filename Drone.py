# 21.09.2021

# Aurthor - Keshav Sapkota and Stephen Bryne
# Drone.py


# Write your code here :-)
from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio  # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel=77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
micropython.kbd_intr(-1)

#initialize UART communication
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin2)
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)

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

incoming = ""

# buffer
#buffer = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#buffer = bytearray(16)

#buffer = []
#buffer.extend(range(0,16,1))
buffer = bytearray(16)

def load_buffer():
    # mask_lsb = 0b0011111111
    # mask_msb = 0b1100000000
    mask_lsb = 255
    mask_msb = 3

    # Given
    buffer[0] = 0
    buffer[1] = 0x01

    # Roll
    buffer[2] = (roll_id << 2) | (((scaled_roll) >> 8) & mask_msb)
    buffer[3] = mask_lsb & scaled_roll

    # Pitch
    buffer[4] = (pitch_id << 2) | (((scaled_pitch) >> 8) & mask_msb)
    buffer[5] = mask_lsb & scaled_pitch

    # Throttle
    buffer[6] = (throttle_id << 2) | (((scaled_throttle) >> 8) & mask_msb)
    #print(int((throttle_id << 2) | ((mask_msb & scaled_throttle) >> 8)))
    buffer[7] = mask_lsb & scaled_throttle
    print(buffer[6])
    print(buffer[7])
    #print(int(mask_lsb & scaled_throttle))
    #buffer[6] = 10
    #buffer[7] = 10

    # Yaw
    buffer[8] = (yaw_id << 2) | (((scaled_yaw) >> 8) & mask_msb)
    buffer[9] = mask_lsb & scaled_yaw

    # Arm
    buffer[10] = (arm_id << 2) |  (((scaled_arm) >> 8) & mask_msb)
    buffer[11] = mask_lsb & scaled_arm

    # Flightmode
    buffer[12] = (flight_mode_id << 2) | ((mask_msb & flight) >> 8)
    buffer[13] = mask_lsb & flight

    # Buzzer
    buffer[14] = (buzzer_id << 2) | ((mask_msb & buzzer) >> 8)
    buffer[15] = mask_lsb & buzzer

    #print(buffer)


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

        parsed_incoming = incoming.split("|", 5)
        #print(parsed_incoming)

        pitch = int(parsed_incoming[0])
        arm = int(parsed_incoming[1])
        roll = int(parsed_incoming[2])
        throttle = int(parsed_incoming[3])
        yaw = int(parsed_incoming[4])
        #display.scroll(throttle)

        if arm == 1:
            display.clear()
            display.set_pixel(1, 1, 9)

            # command need to be scaled and offset before going into buffer
            scaled_pitch = int(scale1 * pitch + offset1)
            scaled_roll = int(scale1 * roll + offset2)
            scaled_throttle = int(scale1 * throttle + offset1)
            scaled_yaw = int(scale2 * yaw + offset1)
            #scaled_arm = int(180 * scale2 * arm )
            scaled_arm = 900
            scaled_throttle = int((scaled_throttle * offset1) / 1000)  # round to nearest decimal
            print(scaled_throttle)

            load_buffer()

            #uart.write(bytes(buffer))
            uart.write(buffer)
            #print(uart.write(buffer))

            #if uart.any():
            #    display.show(Image.HAPPY)

            #display_throttle()

        elif arm == 0:
            # pixel (0,0) lights up.
            display.clear()
            display.set_pixel(0, 0, 9)

    sleep(150)
