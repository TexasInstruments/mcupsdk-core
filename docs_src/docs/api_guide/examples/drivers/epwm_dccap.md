# EPWM Digital counter compare capture {#EXAMPLES_DRIVERS_EPWM_DCCAP}

[TOC]

# Introduction:
The EPWM Edge Detection in DCCAP is a feature to monitor the trip input to be present in a given window. A typical usecase would when the EPWMs are used for controlling a motor, then the expected high current pushes in the initial switching time is expected and if they are not present, then, there is some issue with system and the EPWMs need to to be tripped. 


\imageStyle{am263_epwm_dccap_flow.png, width:60%}
\image html am263_epwm_dccap_example_flow.png "Flow Chart"


# Example Description
This example showcases the configurations needed to use the Edge Detection feature in the DCCAP to detect occurrence of a trip event in a configured time window. The window is configured by MIN and MAX values configured in MINMAX register set. Purpose of this window is to detect the occurrence of such edge. If no such edge occurs, this module will generate a trip event as well as interrupt configurable by user.
 
EPMW0_A to detect the trip within a MINMAX window and trip its EPWMxA output
EPMW1_A waveform is used as a trip input. 

# Configurations 
1. EPWM0
  - DCCAP is enabled, trip inputs are configured for Trip1. 
  - waveform A/B are configured (in the example) to match the min-max window for observing.
  - IN Trip zone, CAPEVT is configured for CBC source and Trip action on output A to low Action when trip occured.
  - XCMP is enbaled for this feature usage.

2. EPWM1
  - XCMP is enabled (not a necessary requirement)
  - generates a waveform, that goes high on counter = 100, goes low on counter = 1000, for a period of 2000 

3. PWMXbar 0 to take GPIO45 via Inputxbar (a common pin that has EPWM 1A output) 
 
\note
The EPWM1A is routed internally via GPIO. if wish to use a different/external trip input, then, 
     1. remove EPWM 1A configuration (either pinmux or all configurations from syscfg)
     2. Add GPIO45 as a input pin. 
     3. connect external trip input to GPIO45   

# External Connections
1. AM263Px-cc with HSEC dock connected
     - EPWM0A can be observed on HSEC PIN 49
     - EPWM0B can be observed on HSEC PIN 51
     - EPWM1A (Trip Input) can be observed on HSEC PIN 53.


# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_DCCAP_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_dccap

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

EPWM Capture Logic Test Started ...
minValue : 10	maxValue : 60
INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : 100
CAPEVT Status = 1
DCCAP Status = 1

minValue : 10	maxValue : 160
INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : 100
CAPEVT Status = 0
DCCAP Status = 1

minValue : 60	maxValue : 110
INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : 100
CAPEVT Status = 0
DCCAP Status = 1

minValue : 60	maxValue : 210
INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : 100
CAPEVT Status = 0
DCCAP Status = 1

minValue : 110	maxValue : 160
INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : 100
CAPEVT Status = 1
DCCAP Status = 1

minValue : 160	maxValue : 210
INPUT_WAVEFORM_RISING_EDGE_COUNTER_VALUE : 100
CAPEVT Status = 1
DCCAP Status = 1

EPWM Capture Logic Test Passed!!
All tests have passed!!

\endcode


\cond SOC_AM263PX

\imageStyle{am263p_epwm_dccap_output.png, width:60%}
    \image html am263p_epwm_dccap_output.png "EPWM DCCAP Edge Monitoring Sample output"

\imageStyle{am263p_epwm_dccap_minWindow_greater_than_edge.png, width:60%}
    \image html am263p_epwm_dccap_minWindow_greater_than_edge.png "when edge is before minimum value of the window"

\imageStyle{am263p_epwm_dccap_maxWindow_lesser_than_edge.png, width:60%}
    \image html am263p_epwm_dccap_maxWindow_lesser_than_edge.png "when edge is after the maximum value of the window"

\imageStyle{am263p_epwm_dccap_edge_in_range.png, width:60%}
    \image html am263p_epwm_dccap_edge_in_range.png "when edge is in the minimum and maximum value range of the window"


\endcond