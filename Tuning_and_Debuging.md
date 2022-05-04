# Tuning and Debugging the Gcode-Mechaduino

I fried the Mechaduinos before I could debug and tune them. Here's what I learned on how to *not* fry them, what to do to debug, and how to tune the device parameters once they are functional.

To not fry: always turn off the motor +12V before unplugging the Mechaduino. Having 12V present and the 5V USB not present seems to have caused all my problems.

Now if I hadn't fried it, I would be doing calibration testing. 

## Initial Testing

Un-comment the line `//    SerialUSB.println(precalculated_v[i]);` in ProcessCmd.cpp and upload the code to the Mechaduino.

### Rotation Calibration

First, with everything unplugged, manually jog the carriage to the middle of its travel.

To perform calibration testing, connect the Mechaduino to your computer and send the command "`M111 S1`". This is the "Turn on debugging" command. You will first see the entrie calibration table being spat out and then a message every `DEBUG_DEFAULT` milliseconds indicating various global variables. If this works, turn on the 12V supply. 

Next, run calibration with the command "`G28 A`". `G28` is the normal home command and the `A` modifier is a custom command to instruct a full calibration. This is to match the precedent of 3D printers using the `G28` command to do automatic bed levelling.

When the command is run, the Mechaduino will try to step the motor back and forth to determine if it is wired correctly. Check the Serial port for the error message "`Appears to be backwards`"; if you see this, turn off 12V, unplug the Mechaduino USB, unplug the motor form the Mechaduino, and plug the motor back in with the connector rotated 180 degrees. 

If you do not see this error message, the motor will start stepping through all 200 steps and record the magnetic field at each position. When it is done, it will print "`Done reading`". It next will compute the intermediate values for the steps such that each of the 2^14 values for the magnetic encoder have a "degree" value associated with it. When this is done, it will print "`The calibration table has been written to non-volatile Flash memory!`" to the Serial monitor.

If this fails, check the `calibrate()` function. I'm not sure how it would fail but that's where it would go wrong.

Next, we have to validate the outputs of the calibration. Send "`M111 S1`" again and lok at the new calibration table being printed out. The maximum output value minus the minimum value output should be `VAL_PER_STEP` and the values output should be within some small increment of each other EXCEPT for when it wraps from 0 to `VAL_PER_STEP` or vice versa. 

If the calibration succeeded, we can move on to homing.

### Homing 

Once calibrated and with everything plugged in properly, run "`M111 S1`" and "`G28`". This is the homing instruction; when run without any parameters it will check to see if the rotation calibration occured and will perform that if necessary and then it will perform homing calibration. This again matches the precedent set by 3D printers.

Homing calibration works by putting the device in closed loop mode and running at constant (low) velocity until the carriage hits and end stop. When we hit the end stop, we detect a spike in "effort" and stop, recording the carriage postion. We then go the opposite direction and repeat this operation. 

This operation occurs in closed loop mode and thus depends on our closed loop parameters. Look at the debugging output; we should see the third column (position) increase or decrease linearly with time. The rate of change should also match our set target velocity. If there is any difference or if the velocity is unstable (oscillating, accelerating then decellerating, etc.) then we have to tune the PID parameters. Refer to the Mechaduino maual for guidance on PID tuning. They are under `\\ Parameters for velocity mode` in TimeControlInt.h.

Next, check the fourth and sixth column. The sixth column should be a low pass filtered version of the fourth column. Ensure the sixth column is actual LPF'ed and tacks the actual value without being too noisy. If it is too noisy, make the filter more agressive by increasing `fLPF`. However, we want to make sure we see a rapid rise in filtered effort when we hit the endstops.

If we hit the endstops but fail to detect it and we see our filtered effort is responding properly, use the debug printouts from the filtered effort to set `UNLOADED_EFFORT_LIM`, `UNLOADED_EFFORT_NOM`, and `EFFORT_SLOPE_THRESH`. `UNLOADED_EFFORT_LIM` is the maximum effort we can expend before throwing an error and giving up, `UNLOADED_EFFORT_NOM` is the nominal effort when unloaded, and `EFFORT_SLOPE_THRESH` is the change in filtered effort between timestamps that indicates we are hitting an endstop. These values are all too large; have your hand on the power supply to turn it off when the Mechaduino inevitably runs itself into an endstop and fails to detect it.

Return home by sending "`G28 X`".

### Rapid Move PID

We tuned the velocity PID when doing the homing tests. Next, we tune the positioning PID. Run the command "`G90`" to go to absolute positioning mode, "`G21`" to go to units millimeters, and "`G1 X30`" to move 30 millimeters. It should move rapidly from its current position to 30 mm. If it doesn't move, there's probably something wrong with the timer control. If it moves the wrong amount, there's probably something wrong with the unit conversion. If it moves unusually slowly or starts oscillating, tune the PID controller. 

If this worked, go to units inches and relative positioning mode (`G20` and `G21`) and try moving the carriage around.

### Linear Move Test

If everything else worked, all we have left is the linear move test. Return home, go to units mm, and change mode to absolute positioning and run "`G1 F10`". Nothing should happen. Next, run "`G1 F100 X30`". We should see the carriage accelerate from 10 mm/minute to 100 mm/minute as it moves from its current position to 30mm away. Check the serial monitor to verify the speed is increasing properly.

Try other (positive) feedrates and moving the carriage around its travel. 

If all this works, comment out whatever lines aren't useful, reupload the code, and recalibrate (reuploading wipes calibration data).