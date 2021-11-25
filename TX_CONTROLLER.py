from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio   # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython
import math
import utime

# drone

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel = 78, address = 0x55555555, group = 9)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
# channel can be 0-83
micropython.kbd_intr(-1)

count = 0
count_interval = 200

battery:float = 0
total_battery:float = 0
avg_battery:float = 0
true_battery:float = 0

# INITIALISE COMMANDS (PARTY)
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0

# NO NEED TO MAKE FUNCTIONS FOR THIS CONTROLLER
# JUST USE WHILE LOOP
display.set_pixel(0, 0, 9)

# INITIALISE COMMAND OUTPUT STRING
command = ""
battery_msg = ""


# Pins
# use analog pins for now
# pin4 and pin10 are connected to leds. To use them as analog pins
# first turn off the display
encoder_pinB = pin6
encoder_pinA = pin7
pitch_pin = pin1
roll_pin = pin2

display.off()


# PID
# Need to find a way of calculating these values
t2 = 0
t1 = 0
dt = 20


# Variables for throt0tle PID control
throttle_target = 700
throttle_current = 0
throttle_new_error = 0
throttle_old_error = 0
throttle_error_area = 0
throttle_pid_corr = 0


#roll_calibration = 6 # initially on a flat surface, drone roll is reading -6

"""************************************************************
************************************************************"""
throttle_kp = 0.006 # these values seem reasonable
throttle_ki = 0#0.000001 #0.000001 #0.00001
# kd not so relevant for throttle
throttle_kd = 5#5 #usually controllers use PI system so stick with PI for now
def throttle_pid_control():
    # Height of the drone can be obatined using geometric properties of encoder

    global throttle_new_error, throttle_pid_corr, throttle_error_area, t1

    # throttle_current = Pid value seems to make logical sense. when we obtain
    # a pid value, this is the value that will be written to uart, so this is
    # current throttle value, gives more stability as well, use throttle target
    # for height property

    '''
    # Calculate dt
    t2 = utime.ticks_ms()
    dt = t2 - t1
    t1 = t2
    # reset timer after 2 mins to prevent overflow
    if (t2 >= 120000):
        t2 = 0
        t1 = 0
    '''

    #throttle_current = throttle_pid_corr

    throttle_current = throttle_encoder()
    #print((0,0,throttle_current))

    throttle_old_error = throttle_new_error
    throttle_new_error = throttle_target - throttle_current

    # Proportional
    throttle_p_corr = throttle_kp * throttle_new_error

    # Integral
    # following value is too high at the start
    throttle_error_area = throttle_error_area + (dt * throttle_new_error)
    # print("err_area", throttle_error_area)
    throttle_i_corr = throttle_ki * throttle_error_area

    # Differential
    throttle_error_change = throttle_new_error - throttle_old_error
    throttle_error_slope = throttle_error_change / dt
    throttle_d_corr = throttle_kd * throttle_error_slope

    throttle_pid_corr = throttle_pid_corr + throttle_p_corr + throttle_i_corr + throttle_d_corr
    #print("throttle_p_corr", throttle_p_corr)
    #print("throttle_i_corr", throttle_i_corr)
    #print("throttle_d_corr", throttle_d_corr)
    print((throttle_target,throttle_current,throttle_pid_corr))
    #print("throttle", throttle_pid_corr)
    return throttle_pid_corr


def display_battery_level(b)->none:

    battery_percent = ((b-300)/(1023-300))

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



def mapping(value, fromLow, fromHigh, toLow, toHigh):
    a = (toLow - toHigh) / (fromLow - fromHigh)
    b = toHigh - a * fromHigh
    exact = a * value + b
    rest = exact - math.floor(exact)
    if rest > 0.5:
        return math.ceil(exact)
    else:
        return math.floor(exact)



# Function is fine
# However, the sensor doesn't work well when rotary speed is high
throttle_encoder_val = 0
a_prev = 0
def throttle_encoder():
    global throttle_encoder_val, a_prev
    a_curr = encoder_pinA.read_digital() #Read in signal A
    b_curr = encoder_pinB.read_digital()

    #print((a_curr, 0.50, 0.50))

    if (a_curr != a_prev):
        if (b_curr != a_curr):
            throttle_encoder_val += 1
            #print("increase")
        else:
            throttle_encoder_val -= 1
            #print("decrease")
    a_prev = a_curr

    if throttle_encoder_val > 100:
        throttle_encoder_val = 100
    elif throttle_encoder_val < 0:
        throttle_encoder_val = 0

    temp = mapping(throttle_encoder_val, 0, 100, 0, 1023)
    #print((throttle_encoder_val, 0, 0))
    return temp


while True:
    battery_msg = radio.receive()
    #print(battery_msg)
    #print(type(battery))

    if battery_msg:
        battery = float(battery_msg)
        #print(battery)

        display_battery_level(battery)

        total_battery = total_battery + battery

        #print("Battery level:", (battery / 1023) * 3.3, "V")

    """
    if count % count_interval == 0:
        avg_battery = total_battery / count_interval
        true_battery = (avg_battery / 1023) * 3.3
        print("Battery level:", true_battery, "V")
        total_battery = 0
        if avg_battery < 300:
            print("LOW BATTERY RUNNING EMERGENCY PROTOCOLS")
            #emergency_safety_function() #run function when battery is low
            #break
    """

    # Failsafe
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        arm = 0
        throttle = 0
        # first string is blank, just to be on the same side, data from second string
        command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)

    # Arming
    # ARM THE DRONE USING BOTH BUTTONS
    # Use is_pressed function
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b is pressed - arm / disarm depending on current state
        if arm == 0:
            arm = 1
            #print("arming")
            # Throttle needs to be 0 for arming
            command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(0)+","+str(yaw)
            radio.send(command)
            display.set_pixel(0, 0, 0)
            display.set_pixel(1, 1, 9)
            sleep(500) # to prevent switch bouncing effect
        else:
            #print("Dis-armed")
            throttle = 0
            arm = 0
            command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(0)+","+str(yaw)
            radio.send(command)
            display.set_pixel(1, 1, 0)
            display.set_pixel(0, 0, 9)
            throttle_new_error = 0
            throttle_pid_corr = 0
            throttle_error_area = 0
            t1 = 0
            t2 = 0
            sleep(500) # to prevent switch bouncing effect


    # Throttle
    #print("Analogue pin 2 value:", int(pin2.read_analog()))
    # Increase or decrease throttle
    #throttle=mapping(int(pin2.read_analog()), 0,1023,1023,0)

    #throttle=mapping(throttle, 0,100,0,1023)
    if arm == 1:

        # PID CONTROL
        throttle = int(throttle_pid_control())
        #pitch = int(pitch_pid_control()) #mapping(int(pitch_pid_control()),0,1023, -15,15)
        # -15 to 15 seems more stable
        #roll = int(roll_pid_control())# mapping(,0,1023, -15,15)
        #print((0,0, roll))
        roll = 0
        pitch = 0


        command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(0)
        radio.send(command)  # Send command via radio
        #print(command)

    sleep(10)


    """
    Information
    1) No print statements in the Drone code
    2) To prevent Switch bouncing add sleep(500) inside both button pressed function
    3) In general keep sleep(10) in both sides
    6) Don't use display.scroll function in transmitter
    7) Throttle should be 0 for arming
    8) Wait few secs to arm, disarm, etc.
    4) Throttle = 40-45 is the point of lift-off, half of joystick action used at this point.
    6) Think of approaches to save battery - things like going to sleep (coz radio is contantly working)
        decreasing throttle to low value while testing, etc.
    7) More convenient to map roll and pitch between -90 and 90.
    8) Some of the pins are connected to LEDs. To access the pins display must be off.
    display.off()
    """
