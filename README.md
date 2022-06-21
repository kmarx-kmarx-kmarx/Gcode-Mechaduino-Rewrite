## Poseidon Prime Firmware and Hardware
Poseidon prime is an open source and low cost syringe pump that uses off-the-shelf parts, 3D printed components, and free software which altogether costs a tenth of other syringe pumps. The design is open-source so anybody can download, modify, and make their own poseidon prime. The syringe pump is highly customizable and can be mounted to rails for extra stability, 3D printed out of polypropylene for heat resistance, and adjusted to fit a variety of syringe sizes. Poseidon prime currently can perform basic fluid dispensing tasks, but going forward it will be able to perform more complex operations and undergo rigorous testing to verify its precision and reliability.

This project builds on [poseidon](https://github.com/pachterlab/poseidon), a project from the Pachter lab. Poseidon prime improves the firmware by adding feedback control and G-code support and builds on the hardware with autoclave-safe polypropylene and a metal shell.

To control poseidon prime, we are using the [Mechaduino](https://github.com/jcchurch13/Mechaduino-Hardware) development board. The Mechaduino repo has the PCB files, BOM, and more information on the project. We found some parts utilized by the Mechaduino project were no longer in stock so we included a new BOM with links to products in stock. 

### The Pump:
![poseidon prime](/MEDIA/PoseidonPrimeWhole.jpg)



## What's Included in this Repo
- `Tuning_and_debugging.md`: Guide for tuning and debugging the system
- Supporting Docs
	- `Sine Lookup.ipynb`: Jupyter notebook for generating a sine lookup table
	- `Velocity Lookup.ipynb`: Jupyter notebook for velocity lookup table proof-of-concept
- HARDWARE
	- `poseidon-prime v123.f3z`: Fusion 360 design file for poseidon prime
	- `p_prime_bom.csv`: Bill of Materials for poseidon prime
	- `mecha_bom.csv`: Bill of Materials for Mechaduino.
	- Design Files
		- `p_prime.stl`: STL file for the entire project
		- `back_block.stl`
		- `front_block.stl`
		- `front_clamp.stl`
		- `front_ear_holder.stl`
		- `small_syringe_press.stl`
		- `carriage_block.stl`
		- `carriage_block_cap.stl`
		- `enclosure.dxf`: DXF for the sheet metal enclosure
- Gcode-Mechaduino-Rewrite
	- `pseudocode.txt`: Basic overview of code operation
	- `Gcode-Mechaduino-Rewrite.ino`: Arduino sketch for the project
	- `MagneticEncoder.cpp`, `MagneticEncoder.h`: Files for communicating with the magnetic encoder
	- `MotorCtrl.cpp`, `MotorCtrl.h`: Files for interacting with the motor driver
	- `SerialCommand.cpp`, `SerialCommand.h`: Files for sending and recieving serial communications
	- `ProcessCmd.cpp`, `ProcessCmd.h`: Files for parsing the commands
	- `TimeControlInt.cpp`, `TimeControlInt.h`: Files for initializing and operating the control loops
	- `fastPWM.c`, `fastPWM.h`: Implement a higher frequency PWM
- MEDIA
	- `1CarriageBlockAssembly.mp4`: Video of assembling the carriage block. Perform this step first when building poseidon prime
	- `1CarriageBlockParts.jpg`: Still image of the parts necessary for assembling the carriage block
	- `2FrontBlockAssembly.mp4`: Video of assembling the front block, step 2
	- `2FrontBlockParts.jpg`: The parts of the front block
	- `3BackBlockAssembly.mp4`
	- `3BackBlockParts.jpg`
	- `4EnclosureAssembly.mp4`
	- `4EnclosureParts.jpg`
	- `PoseidonPrimeWhole.jpg`: View of the whole poseidon prime when mounted on rails
	- `PoseidonPrimeTip.jpg`: Close up on syringe end

Poseidon prime uses the Mechaduino microcontroller; see [the Mechaduino project GitHub](https://github.com/jcchurch13/Mechaduino-Hardware) for the BOM.

## Getting Started
### 3D Printing Components and Purchasing Components
See the BOMs for components, estimated prices, and where to purchase. 

The poseidon prime body could be printed in PLA, but PLA will deform and become brittle in an autoclave. It is advised to print usig polypropylene with a relatively high number of walls to ensure no water can enter the blocks. 

### Build Videos
See the buld videos in the MEDIA folder. Follow the numbers 1 to 4 to see assembly in the proper order.
