# Poseidon Prime Firmware and Hardware

# Files
- `Tuning_and_debugging.md`: Guide for tuning and debugging the system
- Supporting Docs
	- `Sine Lookup.ipynb`: Jupyter notebook for generating a sine lookup table
	- `Velocity Lookup.ipynb`: Jupyter notebook for velocity lookup table proof-of-concept
- HARDWARE
	- `poseidon-prime v123.f3z`: Fusion 360 design file for poseidon prime
- Gcode-Mechaduino-Rewrite
	- `pseudocode.txt`: Basic overview of code operation
	- `Gcode-Mechaduino-Rewrite.ino`: Arduino sketch for the project
	- `MagneticEncoder.cpp`, `MagneticEncoder.h`: Files for communicating with the magnetic encoder
	- `MotorCtrl.cpp`, `MotorCtrl.h`: Files for interacting with the motor driver
	- `SerialCommand.cpp`, `SerialCommand.h`: Files for sending and recieving serial communications
	- `ProcessCmd.cpp`, `ProcessCmd.h`: Files for parsing the commands
	- `TimeControlInt.cpp`, `TimeControlInt.h`: Files for initializing and operating the control loops
	- `fastPWM.c`, `fastPWM.h`: Implement a higher frequency PWM