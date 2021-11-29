# Write your code here :-)
# Keshav Sapkota and Stephen Bryne
# Drone.py


'''****************************************************************************
*** Import libraries and set global constants
****************************************************************************'''

from microbit import *
import radio
import micropython
import utime
import math


radio.on()
radio.config(channel=78, address = 0x55555555, group= 9)
# initialize UART communication
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=pin1, rx=pin8)
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
micropython.kbd_intr(-1)  # enabling or disabling keyboard interrupt

rad_to_deg = 180/3.1416

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0
flight_mode = 1
buzzer = 0
battery = 0


dt = 1


# Scaling, scale by 3.5 for pitch, roll, throttle and 5 for
# yaw, arm and flight mode values and formulas are obtained
# from CPS_Lab2 lecture slide
scale1 = 3.5
scale2 = 5
offset1 = 512
offset2 = 521  # for roll


# channel ID
roll_id = 0
pitch_id = 1
throttle_id = 2
yaw_id = 3
arm_id = 4
flight_mode_id = 5
buzzer_id = 6


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
Display battery level
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
Roll PID control
************************************************************"""
roll_target = 0; roll_current = 0; roll_new_error = 0
roll_old_error = 0; roll_error_area = 0; roll_target = 0
roll_pid_corr=0; roll_max = 2; roll_min = -2; roll_i_corr = 0

roll_kp = 1#0.4   #0.2 - 0.3
roll_ki = 0.005     #0.001 #0.000001 #0.00001# somewhere around 0.002
roll_kd = 1     #1#11 #10 #4 #10

def roll_pid_control():
    global roll_new_error, roll_pid_corr, roll_error_area, roll_i_corr


    get_curr_ang()
    roll_current = roll_ang + 4

    #print("roll_curr", roll_current)
    roll_old_error = roll_new_error
    roll_new_error = roll_target - roll_current

    #print(roll_new_error)

    # Proportional
    roll_p_corr = roll_kp * roll_new_error

    # Integral
    if roll_new_error < 3 and roll_new_error > -3:
        #roll_error_area += dt * roll_new_error
        roll_i_corr += roll_ki * roll_new_error

    # Differential
    roll_error_change = roll_new_error - roll_old_error
    roll_error_slope = roll_error_change / dt
    roll_d_corr = roll_kd * roll_error_slope

    if roll_d_corr > 20 or roll_d_corr < -20:
        roll_d_corr = 0

    roll_pid_corr =   roll_p_corr + roll_i_corr + roll_d_corr

    #print(roll_pid_corr)
    #print("p_corr",roll_p_corr)
    #print("i_corr",roll_i_corr)
    #print("d_corr", roll_d_corr)
    #print((roll_new_error, roll_current, roll_pid_corr))
    return roll_pid_corr



"""************************************************************
Pitch pid control
************************************************************"""
pitch_current = 0; pitch_new_error = 0; pitch_old_error = 0
pitch_error_area = 0; pitch_target = 0; pitch_target = 0
pitch_pid_corr = 0; pitch_max = 2; pitch_min = -2; pitch_i_corr = 0

pitch_kp = 1      #0.08#0.1
pitch_ki = 0.005#0.05#       0.001 #0.000001
pitch_kd = 1#1#2        #1#11

def pitch_pid_control():
    global pitch_new_error, pitch_pid_corr, pitch_error_area, pitch_i_corr

    #pitch_current = -mapping(accelerometer.get_y(),-1024,1024,-90,90)+ 2
    #print("pitch_curr", pitch_current)

    pitch_current = pitch_ang + 5
    #print("pitch_curr", pitch_current)
    pitch_old_error = pitch_new_error
    pitch_new_error = pitch_target - pitch_current

    # Proportional
    pitch_p_corr = pitch_kp * pitch_new_error

    # Integral
    if pitch_new_error < 3 and pitch_new_error > -3:
        #pitch_error_area += dt * pitch_new_error
        pitch_i_corr += pitch_ki * pitch_new_error

    # Differential
    pitch_error_change = pitch_new_error - pitch_old_error
    pitch_error_slope = pitch_error_change / dt
    pitch_d_corr = pitch_kd * pitch_error_slope

    if pitch_d_corr > 20 or pitch_d_corr < -20:
        pitch_d_corr = 0

    pitch_pid_corr = pitch_p_corr + pitch_i_corr + pitch_d_corr
    #print(pitch_pid_corr)
    #print("p_corr",pitch_p_corr)
    #print("i_corr",pitch_i_corr)
    #print("d_corr", pitch_d_corr)
    #print((pitch_target, pitch_current, pitch_pid_corr))
    return pitch_pid_corr



pitch_ang = 0; roll_ang = 0
def get_curr_ang():
    global pitch_ang, roll_ang
    # Get accelerometer values and convert to "g" units
    x = accelerometer.get_x() / 1000
    y = accelerometer.get_y() / 1000
    z = accelerometer.get_z() / 1000

    # Formula to calculate roll and pitch from accelerometer data
    # correct this formula

    # use Euler formula to convert this to degrees
    if pow(x,2)+pow(z,2) != 0 and pow(y,2)+pow(z,2) != 0:
        # pitch
        acc_ang0 = math.atan(-1* y/math.sqrt(pow(x,2)+pow(z,2))) * rad_to_deg
        # roll
        acc_ang1 = math.atan(x/math.sqrt(pow(y,2)+pow(z,2))) * rad_to_deg
    else:
        acc_ang0 = 0
        acc_ang1 = 0

    # Total angle
    pitch_ang = 0.98 * pitch_ang + 0.02 * acc_ang0
    roll_ang = 0.98 * roll_ang + 0.02 * acc_ang1

    #print((0,0,roll_ang))


def flight_control(pitch, arm, roll, throttle, yaw):
    buf = bytearray(16)
    global roll_new_error, roll_pid_corr, roll_error_area
    global pitch_new_error, pitch_pid_corr, pitch_error_area

    if arm == 1:
        scaled_arm = int(180 * scale2)
        roll = roll_pid_control()
        pitch = pitch_pid_control()
        display.set_pixel(1, 1, 9)
        display.set_pixel(0, 0, 0)


    else:
        scaled_arm = 0
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)
        roll = 0
        pitch = 0


    # Filter throttle, pitch and roll

    if throttle > 1023:
        throttle = 1023
    if throttle < 0:
        throttle = 0

    if pitch >90:
        pitch = 90
    if pitch < -90:
        pitch = -90

    if roll > 90:
        roll = 90
    if roll < -90:
        roll = -90


    # Scaling and offsetting
    scaled_pitch = int((scale1 * pitch) + offset1)
    scaled_roll = int((scale1 * roll) + offset1)
    scaled_yaw = int((scale2 * yaw) + offset1)
    scaled_flight_mode = int(45 * scale2)
    #scaled_throttle = int((throttle * offset1) / 50)  # round to nearest decimal
    scaled_throttle = throttle
    scaled_buzzer = 0

    # Given
    buf[0] = 0
    buf[1] = 0x01

    # Roll
    buf[2] = (roll_id << 2) | ((scaled_roll >> 8) & 3)
    buf[3] = scaled_roll & 255

    # Pitch
    buf[4] = (pitch_id << 2) | ((scaled_pitch >> 8) & 3)
    buf[5] = scaled_pitch & 255

    # Throttle
    buf[6] = (throttle_id << 2) | ((scaled_throttle >> 8) & 3)
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



"""************************************************************
Main loop
************************************************************"""
while True:
    battery = pin0.read_analog()
    display_battery_level(battery)
    #radio.send(str(battery))

    incoming = radio.receive()

    if incoming:
        if len(incoming) > 3:
            #print(incoming)
            # display.scroll(incoming)
            # print("incoming"
            parsed_incoming = incoming.split(",")
            #print(parsed_incoming)
            # print("Parsed incoming", parsed_incoming)

            if int(parsed_incoming[0]) == 0:
                # check comment at the bottom *****
                pitch = int(parsed_incoming[1])
                arm = int(parsed_incoming[2])
                roll = int(parsed_incoming[3])
                throttle = int(parsed_incoming[4])
                yaw = int(parsed_incoming[5])


    if arm == 1:
        flight_control(pitch, arm, roll, throttle, 0)


    else:
        throttle = 0
        throttle_error_data = 0
        flight_control(0, 0, 0, 0, 0)
        roll_p_corr = 0; roll_i_corr = 0; roll_d_corr = 0; roll_error_area = 0; roll_new_error = 0
        pitch_p_corr = 0; pitch_i_corr = 0; pitch_d_corr = 0; pitch_error_area = 0; pitch_new_error = 0
        pitch_ang = 0; roll_ang = 0

    sleep(10)

