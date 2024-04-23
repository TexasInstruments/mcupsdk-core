# EPWM Deadband {#EXAMPLES_DRIVERS_EPWM_DEADBAND}

[TOC]

# Introduction

\imageStyle{am263_epwm_deadband_block_diagram.png,width:50%}
\image html am263_epwm_deadband_block_diagram.png "Diagram featuring the DeadBand working in EPWM"
## Example Description
The Example sets up the following EPMWs to showcase the Deadband features, as follows,
1. CONFIG_EPWM0 : Deadband disabled (Reference)
2. CONFIG_EPWM1 : Deadband Active High
3. CONFIG_EPWM2 : Deadband Active Low
4. CONFIG_EPWM3 : Deadband Active High Complimentary
5. CONFIG_EPWM4 : Deadband Active Low Complimentary
6. CONFIG_EPWM5 : Deadband Output Swap i.e., (switch A and B outputs)
6. CONFIG_EPWM6 : Deadband On Rising and Falling edges.

The sequence of operations to obtain the deadband occur in the following manner,
1. Input (EPWMx_A or EPWMx_B) is selected for operation
2. Rising edge delay operation (if selected)
3. Falling edge delay operation (if selected)
4. Inversion of the RED / FED output (if selected)
5. Output Swap (if selected)

Note :
     - All the EPWM in the example are synced with CONFIG_EPWM0 as its Counter reaches 0 and all EPMWs start counting up from 0 when sync pulse reaches.


\imageStyle{am263_epwm_deadband_example_block_diagram.png,width:50%}
\image html am263_epwm_deadband_example_block_diagram.png "EPWM DeadBand Example Block Diagram"

# External Connections

## AM263X-CC or AM263PX-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
Probe the following
- CONFIG_EPWM0 output on HSEC PIN 49  (EPWM0_A)
- CONFIG_EPWM0 output on HSEC PIN 51  (EPWM0_B)
- CONFIG_EPWM1 output on HSEC PIN 53  (EPWM1_A)
- CONFIG_EPWM1 output on HSEC PIN 55  (EPWM1_B)
- CONFIG_EPWM2 output on HSEC PIN 50  (EPWM2_A)
- CONFIG_EPWM2 output on HSEC PIN 52  (EPWM2_B)
- CONFIG_EPWM3 output on HSEC PIN 54  (EPWM3_A)
- CONFIG_EPWM3 output on HSEC PIN 56  (EPWM3_B)
- CONFIG_EPWM4 output on HSEC PIN 153 (EPWM13_A)
- CONFIG_EPWM4 output on HSEC PIN 154 (EPWM13_B)
- CONFIG_EPWM5 output on HSEC PIN 159 (EPWM15_A)
- CONFIG_EPWM5 output on HSEC PIN 160 (EPWM15_B)
- CONFIG_EPWM6 output on HSEC PIN 57  (EPWM4_A)
- CONFIG_EPWM6 output on HSEC PIN 59  (EPWM4_B)
Early Access:AM263Px-CC E1 EPWM 15_A/15_B doesn't have a pinout so insted used EPWM 5_A/5_B ie: CONFIG_EPWM5 output on HSEC PIN 61 (EPWM5_A) and CONFIG_EPWM5 output on HSEC PIN 63 (EPWM5_B)
## AM263X-LP or AM263PX-CC
Probe the following
- CONFIG_EPWM0 output on  PIN J2/4 11  (EPWM0_A)
- CONFIG_EPWM0 output on  PIN J6/8 59  (EPWM0_B)
- CONFIG_EPWM1 output on  PIN J2/4 37  (EPWM1_A)
- CONFIG_EPWM1 output on  PIN J2/4 38  (EPWM1_B)
- CONFIG_EPWM2 output on  PIN J2/4 39  (EPWM2_A)
- CONFIG_EPWM2 output on  PIN J2/4 40  (EPWM2_B)
- CONFIG_EPWM3 output on  PIN J6/8 77  (EPWM3_A)
- CONFIG_EPWM3 output on  PIN J6/8 78  (EPWM3_B)
- CONFIG_EPWM4 output on  PIN J6/8 79  (EPWM13_A)
- CONFIG_EPWM4 output on  PIN J6/8 80  (EPWM13_B)
- CONFIG_EPWM5 output on  PIN J2/4 34  (EPWM15_A)
- CONFIG_EPWM5 output on  PIN J5/7 45  (EPWM15_B)
- CONFIG_EPWM6 output on  PIN J2/4 35  (EPWM14_A)
- CONFIG_EPWM6 output on  PIN J2/4 36  (EPWM14_B)
# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_DEADBAND_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_deadband/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM DeadBand Test Started ...
EPWM DeadBand Example runs for 5 Secs
	Rising Edge timestamp of Reference waveform  : 			502
	Rising Edge timestamp of Deadband Active High output B  : 	502 	(Same as Reference Rising Edge Timestamp)
	Rising Edge timestamp of Deadband Active High output A  : 	902 	(Reference Rising Edge Timestamp + Rising Edge Delay)
	Falling Edge timestamp of Reference waveform : 			3502
	Falling Edge timestamp of Deadband Active High output A : 	3502 	(Same as Reference Falling Edge Timestamp)
	Falling Edge timestamp of Deadband Active High output B : 	3702 	(Reference Falling Edge Timestamp + Falling Edge Delay)
	Observed Rising Edege Delay : 	400
	Observed Falling Edege Delay : 	200
EPWM DeadBand Test Passed!!
All tests have passed!!

\endcode

\imageStyle{am263_epwm_deadband_output.png,width:50%}
\image html am263_epwm_deadband_output.png "EPWM DeadBand Example Sample Capture"
