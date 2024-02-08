# ADC Open Shorts Detection {#EXAMPLES_DRIVERS_ADC_OPEN_SHORTS_DETECTION}

[TOC]

# Example Description
This example demonstrates the Open Shorts Detection circuit on the ADC channels for detecting the pin faults in the system. the example enables the OSD circuit along with the mandatory ADC configurations (see Assumptions below) and diagnoses ADC0-Channel 0 input pin.
The open short detection circuit connects a resistor between an analog input pad and either supply or ground. These resistors can be used to estimate the input impedance of the circuit driving the analog input pad thereby detecting if the pad is shorted or open.

## Assumptions for the OSD Usage
1. ADC is in Single Ended Mode
2. Sampling time is increased well above the minimum value
3. Atleast 1 uS delay after configuring the OSD circuit to ADC samples
4. OSD is not implemented on the CAL 1,2 inputs.
5. Divider Resistence tolerances vary widely and are not used for accuracy checks

## OSD Circuit configurations

config   | function      | Impedance | Voltage on 5K | Voltage on 7K
---------|---------------|-----------|---------------|-----------
0        | Zero Scale    | 5K // 7K  | VSSA          | VSSA
1        | Zero Scale    | 5K        | VSSA          | OPEN
2        | Zero Scale    | 7K        | OPEN          | VSSA
3        | Full Scale    | 5K // 7K  | VDD           | VDD
4        | Full Scale    | 5K        | VDD           | OPEN
5        | Full Scale    | 7K        | OPEN          | VDD
6        | 5/12 Scale    | 5K // 7K  | VSSA          | VDD
7        | 5/12 Scale    | 5K // 7K  | VDD           | VSSA

## Qualification process
In the example, ADCA A0 channel is configured and following algorithm is used to check the A0 pin status:
1. Configure full scale OSDETECT mode & capture ADC results(resultHi)
2. Configure zero scale OSDETECT mode & capture ADC results(resultLo)
3. Disable OSDETECT mode and capture ADC results(resultNormal)
4. Determine the state of the ADC pin
 1. If the pin is open, resultLo would be equal to Vreflo and resultHi would be equal to Vrefhi
 2. If the pin is shorted to Vrefhi, resultLo should be approximately equal to Vrefhi and resultHi should be equal to Vrefhi
 3. If the pin is shorted to Vreflo, resultLo should be equal to Vreflo and resultHi should be approximately equal to Vreflo
 4. If the pin is connected to a valid signal, resultLo should be greater than osdLoLimit but less than resultNormal while resultHi should be less than osdHiLimit but greater than resultNormal

Input  |  Full-Scale output     |  Zero-scale Output     | Pin Status
-------|------------------------|------------------------|------------------
Unknown| VREFHI                 | VREFLO                 | Open
VREFHI | VREFHI                 | approx. VREFHI         | Shorted to VREFHI
VREFLO | approx. VREFLO         | VREFLO                 | Shorted to VREFLO
Vn     | Vn < resultHi < VREFHI | VREFLO < resultLo < Vn | Good

5. osDetectStatusVal of value greater than 4 would mean that there is
no pin fault.
 1. If osDetectStatusVal == 1, means pin A0 is OPEN
 2. If osDetectStatusVal == 2, means pin A0 is shorted to VREFLO
 3. If osDetectStatusVal == 4, means pin A0 is shorted to VREFHI
 4. If osDetectStatusVal == 8, means pin A0 is in GOOD/VALID state
 5. Any value of osDetectStatusVal > 4, means pin A0 is in VALID state

## Configurations
1. The ADC0 SOC0 is configured sample on the Channel 2.
2. The SOC0 Sample and hold window is set to 256 for OSD detection usecase
3. ADC0 INT1 is configured for the EOC0.
4. SOC0 is triggered by Software.
5. OSD circuit is enabeld for ADC0, Channel 2 (see below execution)
## External Connections
ADC0-SOC0 Samples on Channel 2.
- on AM263x CC E2, AM263Px CC E2, with HSEC Dock
    - Feed Analog input to ADC0_AIN2 - HSEC PIN 15
- on AM263x LP E2, AM263Px LP
    - Feed Analog Input to the ADC0_AIN2 - J7 Pin 66

## Watch Variables
osDetectStatusVal - OS detection status of voltage on ADC Channel.
adcResult         - a digital representation of the voltage on ADC Channel.
 */

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_OPEN_SHORTS_DETECTION_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_open_shorts_detection/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,
## An Open Pin sample result
\code
ADC Open Shorts Detection Test Started ...
config	|	Result	|	Mean Result	|Result without OSD on
0	    |	43	    |	2056		|4095
1	    |	11	    |	2056		|4095
2	    |	12	    |	2056		|4095
3	    |	4095	|	2056		|4095
4	    |	4095	|	2056		|4095
5	    |	4095	|	2056		|4095
status : APP_ADC_OSDETECT_STATUS_OPEN
ADC Open Shorts Detection Test Passed
All tests have passed!!
\endcode

## A Pin Shorted to VREFLO sample result
\code
ADC Open Shorts Detection Test Started ...
config	|	Result	|	Mean Result	|Result without OSD on
0	    |	0	    |	1		    |0
1	    |	0	    |	1		    |0
2	    |	0	    |	1		    |0
3	    |	6	    |	1		    |0
4	    |	2	    |	1		    |0
5	    |	1	    |	1		    |0
status : APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFLO
ADC Open Shorts Detection Test Passed
All tests have passed!!
\endcode


## A Pin Shorted to VREFHI sample result
\code
ADC Open Shorts Detection Test Started ...
config	|	Result	|	Mean Result	|Result without OSD on
0	    |	4095	|	4092		|4095
1	    |	4095	|	4092		|4095
2	    |	4095	|	4092		|4095
3	    |	4095	|	4092		|4095
4	    |	4095	|	4092		|4095
5	    |	4095	|	4092		|4095
status : APP_ADC_OSDETECT_STATUS_SHORTED_TO_VREFHI
ADC Open Shorts Detection Test Passed
All tests have passed!!
\endcode
