## Poseidon Prime Firmware and Hardware
Poseidon prime is an open source and low cost syringe pump that uses off-the-shelf parts, 3D printed components, and free software which altogether costs a tenth of other syringe pumps. The design is open-source so anybody can download, modify, and make their own poseidon prime. The syringe pump is highly customizable and can be mounted to rails for extra stability, 3D printed out of polypropylene for heat resistance, and adjusted to fit a variety of syringe sizes. Poseidon prime currently can perform basic fluid dispensing tasks, but going forward it will be able to perform more complex operations and undergo rigorous testing to verify its precision and reliability.

This project builds on [poseidon](https://github.com/pachterlab/poseidon), a project from the Pachter lab. Poseidon prime improves the firmware by adding feedback control and G-code support and builds on the hardware with autoclave-safe polypropylene and a metal shell.

To control poseidon prime, we are using the [Mechaduino](https://github.com/jcchurch13/Mechaduino-Hardware) development board. The Mechaduino repo has the PCB files, BOM, and more information on the project. We found some parts utilized by the Mechaduino project were no longer in stock so we included a new BOM with links to products in stock. 

### The Pump:
(photo goes here)

## What's Included in this Repo
- `Tuning_and_debugging.md`: Guide for tuning and debugging the system
- Supporting Docs
	- `Sine Lookup.ipynb`: Jupyter notebook for generating a sine lookup table
	- `Velocity Lookup.ipynb`: Jupyter notebook for velocity lookup table proof-of-concept
- HARDWARE
	- `poseidon-prime v123.f3z`: Fusion 360 design file for poseidon prime
	- `p_prime.stl`: STL files for poseidon prime
	- `p_prime_bom.csv`: Bill of Materials for poseidon prime
	- `mecha_bom.csv`: Bill of Materials for Mechaduino.
- Gcode-Mechaduino-Rewrite
	- `pseudocode.txt`: Basic overview of code operation
	- `Gcode-Mechaduino-Rewrite.ino`: Arduino sketch for the project
	- `MagneticEncoder.cpp`, `MagneticEncoder.h`: Files for communicating with the magnetic encoder
	- `MotorCtrl.cpp`, `MotorCtrl.h`: Files for interacting with the motor driver
	- `SerialCommand.cpp`, `SerialCommand.h`: Files for sending and recieving serial communications
	- `ProcessCmd.cpp`, `ProcessCmd.h`: Files for parsing the commands
	- `TimeControlInt.cpp`, `TimeControlInt.h`: Files for initializing and operating the control loops
	- `fastPWM.c`, `fastPWM.h`: Implement a higher frequency PWM
- SOFTWARE
	- `gcode_sender.ipynb`: 

Poseidon prime uses the Mechaduino microcontroller; see [the Mechaduino project GitHub](https://github.com/jcchurch13/Mechaduino-Hardware) for the BOM.

## Getting Started
### 3D Printing Components and Purchasing Components

### Build Videos
Also see our protocols.io:

## 