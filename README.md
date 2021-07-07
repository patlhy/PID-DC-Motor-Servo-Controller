# Arduino PID-DC-Motor-Servo-Controller
Use Arduino Nano or STM32 Blue Pill as a controller to replace stepper motors with DC motors. This Arduino code accept STEP and DIRECTION input from driver sockets meant for A4988 or DRV8825 driver.

I used Miguel Sanchez's code in https://github.com/misan/dcservo as a starting point for this code. I've added features I needed to build my CNC machine which uses Arduino Nano as the controller (https://www.thingiverse.com/thing:4544582) and DC motor servo extruder using STM32 blue pill as the controller  (https://www.thingiverse.com/thing:4562946).

Features added are:
- Motor driver changed to TB6612
- Added support for STM32 blue pill board 
- Added a step response output printout for PID parameter tuning
- Added dual motor control
- Added LED output as an indicator when PID output is at maximum
- Motor stuck detection (PID max output over a defined period and not moving much)
- Motor stuck output alarm to send kill signal to terminate CNC job or 3D print job. This can happen when the 3D print nozzle is stuck or when the milling bit is stuck in the workpiece
- Motor runaway detection (PID max output over a defined period and moving much more than the setpoint)
- Modified Brett Beauregard Arduino PID library: reduce sampling time from millisecond to microseconds, average differential term over 2 samples to smooth out motor response, added maximum output for definable sample period at setpoint change to for motor to overcome intitial inertia.

I've added a PID visualization and tuning Software written in Python. I use it to change the PID values and see how the DC motor would response to a step response. The software uses the USB COM port to send command to the Arduino Nano or STM32 board and read the motor step response from the controller memory. Once an acceptable PID value is found, this value can be saved permanently to Arduino Nano or STM32 board using the save button.

Detailed information and instruction can be found in https://hackaday.io/project/178310-stepper-to-dc-motor-conversion

