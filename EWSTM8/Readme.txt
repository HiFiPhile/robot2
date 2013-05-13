Readme.txt for BSP for STM8 128 eval

This project was built for the IAR Workbench for STM8 V6.
It has been tested with the following compiler versions:
- V1.20
- V1.20A
- V1.30A

Supported hardware:
===================
The sample project for STM8 128 is prepared
to run on an ST STM8 128 Eval board,
but may be used on other target hardware as well.

Using different target hardware may require modifications.

Configurations
==============
- Debug:
  This configuration is prepared for download into
  the internal Flash using ST-Link and CSpy.
  It uses an embOS Debug + Profiling library

- Debug_Simulator:
  This configuration is prepared for debugging
  using the CSpy simulator.
  The macro file Start_STM8_Sim.mac enables the
  embOS timer interrupt simulation.
  It uses an embOS Debug + Profiling library

- Release:
  This configuration is prepared for download into
  the internal Flash using ST-Link and CSpy.
  It uses an embOS Release library
