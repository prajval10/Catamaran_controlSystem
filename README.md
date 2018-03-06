# Catamaran_controlSystem
Embedded control system for a robotic autonomous catamaran

## Tools used:
1. Microchip dsPIC30F4011
2. MPLAB X IDE
3. XC 16 Compiler
4. HTerm serial communication software

## Functionality:
A microcontroller board is connected to two outboard motors. The outboard motors are composed by a DC
motor and a propeller installed at the end of its shaft. Together, the two outboard motors allow the catamaran
to move and rotate in the water.

The microcontroller receives desired reference values for the rotation speed of the motors from a control PC, in
terms of motor RPMs (rounds per minute). These reference signals are sent through a serial interface. The
microcontroller sends some feedback messages back to the control PC with a period between 1 and 5 seconds.

## How to run:
Project.c is the main file. The functions, variables, macros used are listed in function.c and functions.h. Compile in MPLAB X IDE and run on the dsPIC30F4011
