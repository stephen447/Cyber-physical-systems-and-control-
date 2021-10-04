# Cyber-physical-systems-and-control-

04/10/2021
To do:

1) Fix arming/disarming. (Try increasing sleep value to 100 inside if button_a.is_pressed() and button_b.is_pressed() loop...
make sure the fix is consistent. 

2) Determine the value of throttle that just about lifts the drone. (will be useful for 5th point)

3) Determine (max, min) value from the joystick. 

4) Add joystick/throttle functionality. Make sure its not too sensitive...
use mapping function to map throttle between (0, 100)
if that's still sensitive (like other groups were saying) we can map it to (0,50) and increase the scaling of throttle proportionally. 
Not fully sure on this, sth we can think about. 
 
5) If we can make joystick/throttle responsive, and easy to use, we can improve joystick for pitch and roll as well. 
 
6) autopilot() -> function will make the drone hover at a point using the accelerometer in the microbit.

7) Have fun practising how to the fly the drone :) 

