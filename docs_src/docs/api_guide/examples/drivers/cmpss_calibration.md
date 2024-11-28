# CMPSS Calibration {#EXAMPLES_DRIVERS_CMPSS_CALIBRATION}

[TOC]

# Introduction

The CMPSS has two sources of offset errors: comparator offset error and compdac offset error. In the data sheet the comparator offset error is referred to as Input referred offset error and compdac offset error is referred to as Static offset error. If both inputs of the comparator are driven from a pin, only the comparator offset error applies. However if the inverting input of the comparator is driven from the compdac, then only the compdac offset error applies. This is because the compdac offset error includes comparator offset error. Due to the offset errors, the CMPSS must be calibrated to make sure trips happen at the expected levels. 

This example shows the calibration of CMPSS when the negative input of the comparator is driven from the compdac. CMPSSA1 COMPH comparator is enabled for calibration. CMPIN1P is used to give positive input coming from the DAC and Compdac is configured to provide the negative input. DAC is used to provide different voltage values to CMPIN1P. Compdac is configured to provide signal of 4095 and will decrement till CMPSS latch is set for that specific voltage value at positive input. 

Output will be the measured voltages at which CMPSS latch is set when compared with different DAC value including the compdac offset error. Please refer the datasheet for the allowed compdac offset error.

# External Connections
 - Connect DAC output to CMPIN1P

## AM263PX-CC or AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC Pin 9 to HSEC Pin 15

## AM263PX-LP or AM263X-LP
When using AM263x-LP
- Connect J1/J3 Pin 30 to J1/3 Pin 66

\cond SOC_AM261X
## AM261x-LP
When using AM261x-LP 
- Connect J1/J3 Pin 30 to J1/3 Pin 63

\endcond 

# Supported Combinations {#EXAMPLES_DRIVERS_CMPSS_CALIBRATION_COMBOS}

\cond SOC_AM263X || SOC_AM263PX || SOC_m261x

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/cmpss/cmpss_calibration/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- When a low input is provided, Trip signal output is low and EPWM gives a PWM waveform
- When a high input(higher than VDD/2) is provided, Trip signal output turns high and EPWM gets tripped

# See Also

\ref DRIVERS_CMPSS_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
\cond SOC_AM261X
CMPSS Calibration Test Started ...
DAC value	  Measured calibration voltage
4000:		    4001 
3000:		    3011 
2000:		    2016 
1000:		    1018 
CMPSS Calibration Test Passed!!!All tests have Passed!!!
\endcond

\cond SOC_AM263X
CMPSS Calibration Test Started ...
DAC value	  Measured calibration voltage
4000:		    3973 
3000:		    2985 
2000:		    1989 
1000:		    994 
CMPSS Calibration Test Passed!!!All tests have Passed!!!
\endcond

\cond SOC_AM26PX
CMPSS Calibration Test Started ...
DAC value	  Measured calibration voltage
4000:		    3998 
3000:		    3004 
2000:		    1998 
1000:		    994 
CMPSS Calibration Test Passed!!!All tests have Passed!!!
\endcond
\endcode

