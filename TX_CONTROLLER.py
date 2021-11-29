'''****************************************************************************
*** Write what this file does here
****************************************************************************'''

from microbit import *
import radio
import micropython
import math
import utime


radio.on()
radio.config(channel = 78, address = 0x55555555, group = 9)
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
micropython.kbd_intr(-1)


count = 0
count_interval = 200


# Battery parameters
battery:float = 0
total_battery:float = 0
avg_battery:float = 0
true_battery:float = 0
battery_msg = ""


# initialise party commands
pitch = 0
arm = 0
roll = 0
throttle = 0
yaw = 0


# Pins for rotary encoder and joystick
# Pins connected to display require display to be off
encoder_pinB = pin6
encoder_pinA = pin7
pitch_pin = pin1
roll_pin = pin2
display.off()


# PID
# Need to find a way of calculating these values
t2 = 0
t1 = 0
dt = 0.02


'''****************************************************************************
Throttle PID control
****************************************************************************'''
throttle_target = 600; throttle_current = 0; throttle_new_error = 0
throttle_old_error = 0; throttle_error_area = 0; throttle_pid_corr = 0
max_throttle = 1000; throttle_i_corr = 0

throttle_kp = 0.01
throttle_ki = 0.002#0.1
throttle_kd = 0     #kd not so relevant for throttle

def throttle_pid_control():
    global throttle_new_error, throttle_pid_corr, throttle_error_area, t1, throttle_i_corr
    throttle_current = throttle_pid_corr
    #throttle_current = throttle_encoder()
    #print((0,0,throttle_current))

    throttle_old_error = throttle_new_error
    throttle_new_error = throttle_target - throttle_current

    # Proportional
    throttle_p_corr = throttle_kp * throttle_new_error

    # Integral
    #if (throttle_new_error > throttle_target - 100 ) and (throttle_new_error < throttle_target + 100):
    if throttle_new_error > -100 and throttle_new_error < 100:
        throttle_i_corr += throttle_ki * throttle_new_error
    else:
        throttle_i_corr = 0

    # Differential
    throttle_error_change = throttle_new_error - throttle_old_error
    throttle_error_slope = throttle_error_change / dt
    throttle_d_corr = throttle_kd * throttle_error_slope

    throttle_pid_corr = throttle_pid_corr + throttle_p_corr + throttle_i_corr + throttle_d_corr

    if throttle_pid_corr > max_throttle:
        throttle_pid_corr = max_throttle
    #print("throttle_p_corr", throttle_p_corr)
    #print("throttle_i_corr", throttle_i_corr)
    #print("throttle_d_corr", throttle_d_corr)
    print((throttle_d_corr,throttle_current,throttle_pid_corr))
    #print("throttle", throttle_pid_corr)
    return throttle_pid_corr



'''****************************************************************************
Display battery level received from drone
****************************************************************************'''
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



'''****************************************************************************
Mapping function allows conversion from one range to another
****************************************************************************'''
def mapping(value, fromLow, fromHigh, toLow, toHigh):
    a = (toLow - toHigh) / (fromLow - fromHigh)
    b = toHigh - a * fromHigh
    exact = a * value + b
    rest = exact - math.floor(exact)
    if rest > 0.5:
        return math.ceil(exact)
    else:
        return math.floor(exact)



'''****************************************************************************
Rotary encoder function - Reads throttle value from the encoder and returns
a number between 0 and 1023.
****************************************************************************'''
throttle_encoder_val = 0
a_prev = 0
def throttle_encoder():
    global throttle_encoder_val, a_prev
    a_curr = encoder_pinA.read_digital()
    b_curr = encoder_pinB.read_digital()
    #print((a_curr, 0.50, 0.50))

    if (a_curr != a_prev):
        if (b_curr != a_curr):
            throttle_encoder_val += 1
        else:
            throttle_encoder_val -= 1
    a_prev = a_curr

    if throttle_encoder_val > 100:
        throttle_encoder_val = 100
    elif throttle_encoder_val < 0:
        throttle_encoder_val = 0

    temp = mapping(throttle_encoder_val, 0, 100, 0, 1023)
    #print((throttle_encoder_val, 0, 0))
    return temp


'''****************************************************************************
Main loop
****************************************************************************'''
display.set_pixel(0, 0, 9)
while True:
    # receive battery message
    #battery_msg = radio.receive()
    #print(battery_msg)

    if battery_msg :
        battery = float(battery_msg)
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

    # Failsafe - disarm for emergency stop
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        arm = 0
        throttle = 0
        # first string is blank, just to be on the same side, data from second string
        command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)


    # Arm drone when both buttons are pressed
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b is pressed - arm / disarm depending on current state
        if arm == 0:
            arm = 1
            # Throttle needs to be 0 for arming
            command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(0)+","+str(yaw)
            radio.send(command)
            display.set_pixel(0, 0, 0)
            display.set_pixel(1, 1, 9)
            sleep(500) #to prevent switch bouncing effect
        else:
            throttle = 0
            arm = 0
            command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(0)+","+str(yaw)
            radio.send(command)
            display.set_pixel(1, 1, 0)
            display.set_pixel(0, 0, 9)
            throttle_new_error = 0
            throttle_pid_corr = 0
            throttle_error_area = 0
            throttle_p_corr = 0; throttle_i_corr = 0; throttle_d_corr = 0
            throttle_encoder_val = 0
            t1 = 0
            t2 = 0
            sleep(500) # to prevent switch bouncing effect

    if arm == 1:
        throttle = int(  throttle_pid_control())
        command = "1"+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(0)
        radio.send(command)  # Send command via radio

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
