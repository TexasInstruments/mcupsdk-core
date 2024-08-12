# ADC Safety Checker {#EXAMPLES_DRIVERS_ADC_SAFETY_CHECKER}

[TOC]

# Safety Checker module
 The safety checker module provides the ability to compare ADC conversion results for safety critical application. It is divided into two parts: Safety checker tiles and Safety checker aggregator. Safety checker tile captures conversion results from multiple ADc modules and compares the absolute value of the difference to the configured tolerance. While Safety checker aggregator takes the computed data and genreate a trip event signal if the result is out of range. The trip event can be sent to INT XBAR and an ISR can be triggered.

\imageStyle{ADC_safety_checker_tile.png,width:50%}
\image html ADC_safety_checker_tile.png "ADC Safety Checker Tile Diagram"

# Example Description
This example compares the absolute value of the two ADC conversion results with the set tolerance value and generates an out-of-tolerance flag if the difference exceeds the tolerance value. Channel 0 and 1 of ADC1 are used to compare the two ADC conversion. If the difference between two conversion results exceeds the value configured as tolerance then the ADC safety checker aggregator generates an interrupt signal from out-of-tolerance.

## Safety Checker Configurations
1. Safety checker tile 1 is configured to compare two ADC channels ADC1_AIN0 and ADC1_AIN1
2. The tolerance value is set to 100 in digital representation
    - If the difference between two ADC channel is > 100, it will generate an interrupt
3. Safety checker aggregator is configured to generate an interrupt when oot event occures

# Watch Variables
- gAdc1Result0[], gAdc1Result1[] - the digital representation of the Analog signal on ADC1_AIN0 and ADC1_AIN1.
- count - the number of times the OOT flag is generated.

# External Connections
ADC1_AIN0, ADC1_AIN1 pins should be connected to signals to be converted
 - on AM263Px CC E1, with HSEC Dock
     - Feed Analog input to ADC1_AIN0 - HSEC PIN 12
     - Feed Analog input to ADC1_AIN1 - HSEC PIN 14
 - on AM263Px LP
     - Feed Analog input to ADC1_AIN0 - J1/3 Pin 24
     - Feed Analog input to ADC1_AIN1 - J1/3 Pin 29
 - on AM261x LP
     - Feed Analog input to ADC1_AIN0 - J1/3 Pin 24
     - Feed Analog input to ADC1_AIN1 - J5/7 Pin 42

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SAFETY_CHECKER_COMBOS}

\cond SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_safety_checker/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section.
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using the watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is running,

\code
ADC Safety Checker Test Started ...
ADC 1 Channel 0		ADC 1 Channel 1	    Safety Check OOT Flag
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
	4095			    0			        1
ADC OOT Flag Count 10
ADC Safety Checker Test Passed!!
All tests have passed!!
\endcode
