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
import math

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel=77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER


# initialize UART communication
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin8)
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
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

# Need to find a way of calculating these values
t2 = 0
t1 = 0
dt = 15


# Variables for throt0tle PID control
throttle_target = 0
throttle_current = 0
throttle_new_error = 0
throttle_old_error = 0
throttle_error_area = 0
throttle_kp = 0.01 # these values seem reasonable
throttle_ki = 0.0001
throttle_kd = 10 # usually controllers use PI system so stick with PI for now
throttle_pid_corr = 0


# Variables for pitch PID control
pitch_target = 0
pitch_current = 0
pitch_new_error = 0
pitch_old_error = 0
pitch_error_area = 0
pitch_kp = 0.1
pitch_ki = 0.00001
pitch_kd = 0
pitch_pid_corr = 0


# Variables for roll PID control
roll_target = 0
roll_current = 0
roll_new_error = 0
roll_old_error = 0
roll_error_area = 0


roll_kd = 0
roll_pid_corr = 0

roll_calibration = 6 # initially on a flat surface, drone roll is reading -6


"""************************************************************
Function: mapping
This function maps a value from a range of fromLow and
fromHigh to a range of toLow and toHigh.
************************************************************"""
def mapping(value, fromLow, fromHigh, toLow, toHigh):
    a = (toLow - toHigh) / (fromLow - fromHigh)
    b = toHigh - a * fromHigh
    exact = a * value + b
    rest = exact - math.floor(exact)
    if rest > 0.5:
        return math.ceil(exact)
    else:
        return math.floor(exact)


"""************************************************************

************************************************************"""
def throttle_pid_control(throttle):
    global throttle_new_error, throttle_pid_corr, throttle_error_area
    """
    t1 = t2
    t2 = utime.tick_ms()
    dt = t2 - t1
    """
    throttle_target = throttle

    throttle_old_error = throttle_new_error
    # new error = target - current, however its not possible to measure
    # current for throttle, so use throttle_pid_corr
    throttle_new_error = throttle_target - throttle_pid_corr

    # Proportional
    throttle_p_corr = throttle_kp * throttle_new_error

    # Integral
    # following value is too high at the start
    # throttle_error_area = throttle_error_area + (dt * throttle_new_error)
    throttle_error_area = throttle_error_area + (dt * throttle_new_error)
    # print("err_area", throttle_error_area)
    throttle_i_corr = throttle_ki * throttle_error_area
    # print("throttle_i_corr", throttle_i_corr)

    # Differential
    throttle_error_change = throttle_new_error - throttle_old_error
    throttle_error_slope = throttle_error_change / dt
    throttle_d_corr = throttle_kd * throttle_error_slope
    # print("throttle_d_corr", throttle_d_corr)

    throttle_pid_corr = int(throttle_pid_corr + throttle_p_corr + throttle_i_corr + throttle_d_corr)
    #print(throttle_p_corr)
    #print(throttle_i_corr)
    #print(throttle_d_corr)
    print((0,throttle_target,throttle_pid_corr))
    return throttle_pid_corr


"""************************************************************

************************************************************"""
def pitch_pid_control(pitch):
    #print("inside pitch pid")
    global pitch_new_error, pitch_pid_corr, pitch_error_area, pitch_current

    # to make the drone hover our pitch target is 0
    # and current pitch = drone's accelerometer
    # try changing pitch_target to pitch input parameter,
    # this might improve control with transmitter

    pitch_target = 0
    pitch_current = mapping(accelerometer.get_y(),-1023,1023,-90,90)

    pitch_old_error = pitch_new_error

    # pitch_current should match similar mapping
    pitch_new_error = pitch_target - pitch_current

    # Proportional
    pitch_p_corr = pitch_kp * pitch_new_error

    # Integral
    # following value is too high at the start
    pitch_error_area = pitch_error_area + (dt * pitch_new_error)
    # print("err_area", pitch_error_area)
    pitch_i_corr = pitch_ki * pitch_error_area
    # print("pitch_i_corr", pitch_i_corr)

    # Differential
    pitch_error_change = pitch_new_error - pitch_old_error
    pitch_error_slope = pitch_error_change / dt
    pitch_d_corr = pitch_kd * pitch_error_slope
    # print("pitch_d_corr", pitch_d_corr)

    pitch_pid_corr = int(pitch_pid_corr + pitch_p_corr + pitch_i_corr + pitch_d_corr)
    print(pitch_pid_corr)
    print((0,0,pitch_pid_corr))
    return pitch_pid_corr


"""************************************************************

************************************************************"""
def roll_pid_control(roll):
    roll_kp = 0.2       # (0.2 - 0.3)
    roll_ki = 0.002 # somewhere around 0.002
    roll_kd = 50
    #print("inside roll pid")
    global roll_new_error, roll_pid_corr, roll_error_area, roll_current

    # to make the drone hover our roll target is 0
    # and current roll = drone's accelerometer
    # try changing roll_target to roll input parameter,
    # this might improve control with transmitter

    roll_target = 0
    roll_current = mapping(accelerometer.get_x(),-1024,1024,-90,90) + roll_calibration
    #print("roll_curr", roll_current)
    roll_old_error = roll_new_error

    # roll_current should match similar mapping
    roll_new_error = roll_target - roll_current

    # Proportional
    roll_p_corr = roll_kp * roll_new_error
    #print("roll_p_corr", roll_p_corr)
    # Integral
    # following value is too high at the start
    roll_error_area = roll_error_area + (dt * roll_new_error)
    #print("err_area", roll_error_area)
    roll_i_corr = roll_ki * roll_error_area
    #print("roll_i_corr", roll_i_corr)

    # Differential
    roll_error_change = roll_new_error - roll_old_error
    roll_error_slope = roll_error_change / dt
    roll_d_corr = roll_kd * roll_error_slope
    #print("roll_d_corr", roll_d_corr)

    roll_pid_corr = int( roll_pid_corr + roll_p_corr + roll_i_corr + roll_d_corr)
    #print("roll_pid_corr", roll_pid_corr)
    #print((0,roll_current,roll_pid_corr))
    return roll_pid_corr


"""************************************************************

************************************************************"""
def display_battery_level(b)->none:

    battery_percent = ((b-300)/(1023-300))
    # print(battery_percent)
    # display.scroll(battery_percent)

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

"""************************************************************

************************************************************"""
# NEED TO MAKE SURE BUFFER HAS THE VALUES WE EXPECT AND NO UNUSUAL CHARACTERS
# USE REPL TO READ FROM SERIAL, AND SET UART PINS TO NONE WHILE READING FROM UART
def flight_control(pitch, arm, roll, throttle, yaw):
    buf = bytearray(16)

    # print("inside load_buf function")

    if arm == 1:
        scaled_arm = int(180 * scale2)
        display.set_pixel(1, 1, 9)
        display.set_pixel(0, 0, 0)
    elif arm == 0:
        scaled_arm = 0
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)


    # PID CONTROL
    throttle = throttle_pid_control(throttle)
    #pitch = pitch_pid_control(pitch)
    #roll = roll_pid_control(roll)
    #print((0,0, roll))


    # Filter throttle, pitch and roll
    if throttle > 1023:
        throttle = 1023
    if throttle < 0:
        throttle = 0

    if pitch > 90:
        pitch = 90
    if pitch < -90:
        pitch = -90

    if roll > 90:
        roll = 90
    if roll < -90:
        roll = -90

    # Scaling and offsetting
    scaled_pitch = int((scale1 * pitch) + offset1)
    scaled_roll = int((scale1 * roll) + offset1 + 9.5)
    scaled_yaw = int((scale2 * yaw) + offset1)
    scaled_flight_mode = int(45 * scale2)
    # scaled_throttle = int((throttle * offset1) / 50)  # round to nearest decimal
    scaled_throttle = throttle
    scaled_buzzer = 0


    # Given
    buf[0] = 0
    buf[1] = 0x01

    # Roll
    buf[2] = (roll_id << 2) | ((scaled_roll >> 8) & 3)
    # print("buf2 ", buf[2])
    buf[3] = scaled_roll & 255

    # Pitch
    buf[4] = (pitch_id << 2) | ((scaled_pitch >> 8) & 3)
    buf[5] = scaled_pitch & 255

    # Throttle
    buf[6] = (throttle_id << 2) | ((scaled_throttle >> 8) & 3)
    # print(int((throttle_id << 2) | ((3 & scaled_throttle) >> 8)))
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

    #uart.write(buf)


"""************************************************************

************************************************************"""
# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
while True:
    battery = pin0.read_analog()
    display_battery_level(battery)
    radio.send(str(battery))  # battery is not used when connected via usb
    # print(str(battery))

    incoming = radio.receive()
    # print(incoming)
    # need to consider if everything should be inside incoming or can it be outside??

    # add a while loop that calculates calibration values for roll and pitch


    if incoming:
        # display.scroll(incoming)
        # print("incoming")

        parsed_incoming = incoming.split(",")
        # print("Parsed incoming", parsed_incoming)

        # check comment at the bottom *****
        pitch = int(parsed_incoming[1])
        arm = int(parsed_incoming[2])
        roll = int(parsed_incoming[3])
        throttle = int(parsed_incoming[4])
        yaw = int(parsed_incoming[5])

        #arm = 1
        if arm == 1:
            # print("throttle_pid_corr", throttle_pid_corr)
            # print((throttle_i_corr,throttle_d_corr,throttle_pid_corr))
            flight_control(pitch, arm, roll, throttle, 0)
            # flight_control(0, arm, 0, throttle, 0)

        else:
            throttle = 0
            throttle_error_data = 0
            flight_control(0, 0, 0, 0, 0)

    sleep(10)

# Troubleshoot
# No print lines in the drone side
# use a gesture command to move between controller mode and hover mode.
# if gesture == controller mode -> send the parsed data to flight control
# elif gesture == hover mode -> send pitch = 0, roll = 0, throttle = throttle,
# arm = 1
