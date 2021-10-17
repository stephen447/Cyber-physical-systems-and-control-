from microbit import *  # NEEDS TO BE INCLUDED IN ALL CODE WRITTEN FOR MICROBIT
import radio   # WORTH CHECKING OUT RADIO CLASS IN BBC MICRO DOCS
import micropython
import math

radio.on()  # TURNS ON USE OF ANTENNA ON MICROBIT
radio.config(channel = 77)  # A FEW PARAMETERS CAN BE SET BY THE PROGRAMMER
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)
# channel can be 0-83
micropython.kbd_intr(-1)
count = 0
count_interval = 200
total = 0
battery:float = 0
total_battery:float = 0
avg_battery:float = 0
true_battery:float = 0

#initialize UART communication
#uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=None, rx=None)

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


while True:
    battery_msg = radio.receive()
    print(battery_msg)
    #print(type(battery))

    if battery_msg:
        battery = float(battery_msg)
        print(battery)

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
    # Arming
    # ARM THE DRONE USING BOTH BUTTONS
    # Use is_pressed function
    if button_a.is_pressed() and button_b.is_pressed():
        # if button a and b is pressed - arm / disarm depending on current state
        if arm == 0:
            arm = 1
            #print("arming")
            command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
            radio.send(command)
            display.set_pixel(0, 0, 0)
            display.set_pixel(1, 1, 9)
            sleep(500) # to prevent switch bouncing effect
        else:
            #print("Dis-armed")
            throttle = 0
            arm = 0
            command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
            radio.send(command)
            display.set_pixel(1, 1, 0)
            display.set_pixel(0, 0, 9)
            sleep(500) # to prevent switch bouncing effect


    # Throttle
    #print("Analogue pin 2 value:", int(pin2.read_analog()))
    # Increase or decrease throttle
    throttle=mapping(int(pin2.read_analog()), 0,1023,90,0)
    # currently the way we have throttle joystick its upside-down
    # try 99, 0 in the above line or

    # Map throttle
    if throttle > 90: throttle = 90
    if throttle < 0 : throttle = 0
    print("throttle is:", throttle)
    #display.scroll(throttle)


    # Roll
    #roll=mapping(accelerometer.get_x(),-1024,1024,-20,20)
    roll=-mapping(int(pin0.read_analog()),0,1023,-20,20)
    if roll>20: roll=20
    if roll<-20: roll=-20
    #print("roll ", roll)


    # Pitch
    pitch=-mapping(int(pin1.read_analog()),0,1023,-20,20)
    if pitch>20: pitch=20
    if pitch<-20: pitch=-20
    #print("pitch ", pitch)

    yaw = 0

    # Failsafe
    if accelerometer.was_gesture('shake'):  # Killswitch - using the predefined gestures
        arm = 0
        throttle = 0
        command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
        display.set_pixel(0, 0, 9)
        display.set_pixel(1, 1, 0)


    command = ""+","+str(pitch)+","+str(arm)+","+str(roll)+","+str(throttle)+","+str(yaw)
    #print(command)
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

    """


