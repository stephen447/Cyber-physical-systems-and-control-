# Write your code here :-)
# Write your code here :-)
# 21.09.2021

# Aurthor - Keshav Sapkota and Stephen Bryne
# Drone.py

# Write your code here :-)
from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio  # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython
import utime

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel=77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER


#initialize UART communication
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin8)
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
micropython.kbd_intr(-1)  # enabling or disabling keyboard interrupt

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0
flight_mode = 1
buzzer = 0
battery = 0

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


# Variables for throttle PID control
t2 = 0
t1 = 0
dt = 0
throttle_target = 700
throttle = 0
throttle_new_error = 0
throttle_old_error = 0
throttle_error_change = 0
throttle_error_slope = 0
throttle_error_area = 0
throttle_p_corr = 0
throttle_d_corr = 0
throttle_i_corr = 0
throttle_kp = 0.1
throttle_ki = 0.0005
throttle_kd = 20
throttle_pid_corr = 0

"""
def throttle_pid_control(throttle):
    t1 = t2
    t2 = utime.tick_ms()
    dt = t2 - t1
    throttle_old_error = throttle_new_error
    throttle_new_error = throttle_target - throttle

    #Proportional
    throttle_p_corr = throttle_kp * throttle_new_error

    #Integral
    throttle_error_area = throttle_error_area + dt * throttle_new_error
    throttle_i_corr = throttle_ki * throttle_error_area

    #Differential
    throttle_error_change = throttle_new_error - throttle_old_error
    throttle_error_slope = throttle_error_change / dt
    throttle_d_corr = throttle_kd * throttle_error_slope

    return throttle_p_corr + throttle_i_corr + throttle_d_corr
"""

def display_battery_level(b)->none:

    battery_percent = ((b-300)/(1023-300))
    #print(battery_percent)
    #display.scroll(battery_percent)

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

# NEED TO MAKE SURE BUFFER HAS THE VALUES WE EXPECT AND NO UNUSUAL CHARACTERS
# USE REPL TO READ FROM SERIAL, AND SET UART PINS TO NONE WHILE READING FROM UART
def flight_control(pitch, arm, roll, throttle, yaw):
    buf = bytearray(16)

    #print("inside load_buf function")

    if arm == 1:
        scaled_arm = int(180 * scale2)
        display.set_pixel(1, 1, 9)
        display.set_pixel(0, 0, 0)
    elif arm == 0:
        scaled_arm = 0
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)
    """
    if throttle > 99:
        throttle = 99
    if throttle < 0:
        throttle = 0
    """
    if yaw > 90:
        yaw = 90
    if yaw < -90:
        yaw = -90

    if pitch > 90:
        pitch = 90
    if pitch < -90:
        pitch = -90

    if roll > 90:
        roll = 90
    if roll < -90:
        roll = -90


    # scaling and offsetting
    scaled_pitch = int((scale1 * pitch) + offset1)
    scaled_roll = int((scale1 * roll) + offset1 + 9.5)
    scaled_yaw = int((scale2 * yaw) + offset1)
    scaled_flight_mode = int(45 * scale2)
    #scaled_throttle = int((throttle * offset1) / 50)  # round to nearest decimal
    scaled_throttle = throttle
    scaled_buzzer = 0
    #print (scaled_pitch, scaled_arm, scaled_roll, scaled_throttle, scaled_yaw, scaled_flight_mode, scaled_buzzer)


    # Given
    buf[0] = 0
    buf[1] = 0x01

    # Roll
    buf[2] = (roll_id << 2) | ((scaled_roll >> 8) & 3)
    #print("buf2 ", buf[2])
    buf[3] = scaled_roll & 255

    # Pitch
    buf[4] = (pitch_id << 2) | ((scaled_pitch >> 8) & 3)
    buf[5] = scaled_pitch & 255

    # Throttle
    buf[6] = (throttle_id << 2) | ((scaled_throttle >> 8) & 3)
    #print(int((throttle_id << 2) | ((3 & scaled_throttle) >> 8)))
    buf[7] = scaled_throttle & 255

    # Yaw
    buf[8] = (yaw_id << 2) | ((scaled_yaw >> 8) & 3)
    buf[9] = scaled_yaw & 255

    # Arm
    buf[10] = (arm_id << 2) | ((scaled_arm >> 8) & 3)
    buf[11] = scaled_arm & 255

    # Flightmode
    buf[12] = (flight_mode_id << 2) | ((scaled_flight_mode >> 8) & 3)
    buf[13] = scaled_flight_mode & 255

    # Buzzer
    buf[14] = (buzzer_id << 2) | ((scaled_buzzer >> 8) & 3)
    buf[15] = scaled_buzzer & 255

    uart.write(buf)


# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
while True:
    battery = pin0.read_analog()
    display_battery_level(battery)
    radio.send(str(battery))  #battery is not used when connected via usb
    #print(str(battery))

    incoming = radio.receive()
    #print(incoming)
    # need to consider if everything should be inside incoming or can it be outside??
    if incoming:
        #display.scroll(incoming)
        #print("incoming")

        parsed_incoming = incoming.split(",")
        #print("Parsed incoming", parsed_incoming)


        pitch = int(parsed_incoming[1])
        arm = int(parsed_incoming[2])
        roll = int(parsed_incoming[3])
        throttle = int(parsed_incoming[4])
        yaw = int(parsed_incoming[5])


        if arm == 1:
            #print("throttle_pid_corr", throttle_pid_corr)
            #print((throttle_i_corr,throttle_d_corr,throttle_pid_corr))
            flight_control(pitch, arm, roll, throttle_pid_corr, 0)
            #flight_control(0, arm, 0, throttle, 0)

            """
            t1 = t2
            t2 = utime.ticks_ms()
            dt = t2 - t1
            """
            # need to figure out how to find dt properly. Following value is an estimate
            # 100 for sleep and 5 for rest of program execution
            dt = 105

            throttle_old_error = throttle_new_error
            throttle_new_error = throttle_target - throttle_pid_corr

            #Proportional
            throttle_p_corr = throttle_kp * throttle_new_error
            #print("throttle_p_corr", throttle_p_corr)

            #Integral
            #print("dt", dt)
            #print("ki", throttle_ki)
            # following value is too high at the start
            throttle_error_area = (dt * throttle_new_error)
            #print("err_area", throttle_error_area)
            throttle_i_corr = throttle_ki * throttle_error_area
            #print("throttle_i_corr", throttle_i_corr)

            #Differential
            throttle_error_change = throttle_new_error - throttle_old_error
            throttle_error_slope = throttle_error_change / dt
            throttle_d_corr = throttle_kd * throttle_error_slope
            #print("throttle_d_corr", throttle_d_corr)

            throttle_pid_corr = int(throttle_pid_corr + throttle_p_corr + throttle_i_corr + throttle_d_corr)

        else:
            #print("disarmed")
            throttle_pid_corr = 0
            throttle_error_data = 0
            flight_control(0, 0, 0, 0, 0)

    sleep(100)

# Troubleshoot
# No print lines in the drone side
