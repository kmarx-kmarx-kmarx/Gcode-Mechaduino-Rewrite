# Tuning and Debugging the Gcode-Mechaduino

I fried the Mechaduinos before I could debug and tune them. Here's what I learned on how to *not* fry them, what to do to debug, and how to tune the device parameters once they are functional.

To not fry: always turn off the motor +12V before unplugging the Mechaduino. Having 12V present and the 5V USB not present seems to have caused all my problems.

Now if I hadn't fried it, I would be doing calibration testing. 

## Initial Testing

To perform calibration testing, connect the Mechaduino to your computer and send the command "M111 S1". This is the "Turn on debugging" command. You will first see the entrie calibration table being spat out and then a message every `DEBUG_DEFAULT` milliseconds indicating various global variables. If this works, turn on the 12V supply, 