# CMPSS Diode Emulation {#EXAMPLES_DRIVERS_CMPSS_DIODE_EMULATION}

[TOC]

# Introduction

This example showcases the Diode Emulation features using CMPSS where the trip inputs are sourced from CMPSS. At CMPSS, there are 2 shadow values for the CMPSS DACs (high and low comparator's), namely "Shadow" and "Shadow 2". when the DE mode is enabled, and DEACTIVE signal high, the "shadow 2" value is used for updating the active register (see image below).

# EPWM Configuration 
EPWM0, EPWM1 and EPWM2 are configured to generate a 25KHz signal with 50% duty cycle.

# DAC Configuration
DAC0 is used as positive input to CMPSSA1 High and Low: (CMPSSA0 in case of LP)
Reference voltage source = external

# CMPSS Configuration
CMPSSA1 (CMPSSA0 in case of LP) configuration for TRIPH/TRIPL signal inputs to enter DE mode:
  - Positive input = DAC_OUT for both CMPSSA1 High and Low. A ramp wave is generated in the example for showcasing.
  - Negative input = 1.65v from internal DAC for CMPSSA1 High (CMPSSA0 in case of LP)
                     3.3v from internal DAC for CMPSSA1 Low (CMPSSA0 in case of LP)
This means only CMPSSTRIPH1 is triggered when DAC_OUT > 1.65v and CMPSSTRIPL1 is always low

Diode Emulation Configuration:
  - For EPWM0 and EPWM1, TripH = CMPSSTRIPH1 and TRIPL = CMPSSTRIPL1
    Since a TRIPH_OR_TRIPL triggers DE entry, only TripH is used for demo
  - EPWM0_A is set to active high and EPWM0_B set to low during DE
  - EPWM1_A is configured same as EPWM0 with re-entry delay
    - Re-entry delay = 10 EPWMSYNCPER cycles
  - EPWMSYNCPER is configured at Counter equals TB Period for both the epwms.
  - EPWM2 is confiugred same as EPWM0 with DE mode disabled for the purpose of comparison

\imageStyle{am263px_basic_diode_emulation_config.png,width:50%}
\image html am263px_basic_diode_emulation_config.png "Basic diode emulation configuration with trip inputs from CMPSS"

OutputXbar 13 and 14 is used to route the CTRIPOUTH and CTRIPOUTL signal.
OutputXbar 7 and 8 is used to route DEActive0 from EPWM0 and DEActive1 from EPWM1.

# Various DE configuration

## EPWM in CBC mode

EPWM0 is configured in CBC mode where on occurrence of every PWMSYNC it evaluates if TRIPH_OR_TRIPL condition is present and if not then DEACTIVE flag is cleared.

Channel A:

\imageStyle{am263px_cmpss_diode_emulation.png,width:50%}
\image html am263px_cmpss_diode_emulation.png "EPWM in CBC mode with DEACTIVE for channel A"

Initially when TRIPH_OR_TRIPL was 0 as marked by the blue marker, EPWM0A was following the normal action configured in the AQ. We can compare with EPWM2A.
Once TRIPH_OR_TRIPL gets High as marked by the red marker, it enters into the DEACTIVE mode and now EPWM0A is driven by TRIPH as per the configuration.On occurrence of the PWMSYNC, since TripHorTripL goes low, DEACTIVE flag will get cleared and EPWM0A starts following the normal AQ actions (take EPWM2A as reference)

## EPWM in CBC mode with Re-entry delay

EPWM1 is configured in CBC mode but with Re-entry delay = 10. This means once EPWM1 exits the DEACTIVE mode, for next 10 PWMSYNCs it won’t enter into DEACTIVE mode even if TRIPH_OR_TRIPL is High. EPWM1A are also shown for comparison.

\imageStyle{am263px_cmpss_diode_emulation_reentry.png,width:50%}
\image html am263px_cmpss_diode_emulation_reentry.png "EPWM in CBC mode with Re-entry = 10 for channel A

The green marker marks the exit of DEACTIVE because that’s the point where PWMSYNC comes and TRIPH_OR_TRIPL is low. At this point, the Re-entry counter will get loaded with the Re-entry value i.e. 10 in our case. It will decrement by one on every PWMSYNC. The trip signals are blocked as long as counter value is greater than 0. The counter reaches 0 on occurrence of PWMSYNC. If we see carefully, the EPWM1A now is driven by TripH from this point.

# External Connections
 - Connect DAC output to CMPIN1P and CMPIN1N for AM263Px CC (CMPIN0P and CMPIN0N in case of AM263PX LP)
 - Probe EPWM2A for reference waveform
 - Probe EPWM0A and 1A for the diode emulation waveforms on non-reentry delay and re-entry delay DE EPWMs respectively

## AM263PX-CC
When using AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC Pin 9 (DAC_OUT) to HSEC Pin 15 and 17 (CMPSSA1H/L)
- Probe the following on the HSEC Pin
  - EPWM0A    : 49
  - EPWM1A    : 53
  - EPWM2A    : 50
  - DEActive0 : 87
  - DEActive1 : 85
  - CTRIPH    : 68
  - CTRIPL    : 70

## AM263PX-LP
- Connect J1/J3 Pin 30 (DAC_OUT) to J1/3 Pin 23 and J1/3 28 (CMPSSA0H/L)
- Probe the following on boosterpack
  - EPWM0A    : J2/4 Pin 11
  - EPWM1A    : J2/4 Pin 37
  - EPWM2A    : J2/4 Pin 39
  - DEActive0 : J5/7 Pin 49
  - DEActive1 : J5/7 Pin 50
  - CTRIPH    : J6/8 Pin 72
  - CTRIPL    : J6/8 Pin 71 

## AM261X-LP
- Connect J1/J3 Pin 30 (DAC_OUT) to J1/3 Pin 23 and J1/3 2 (CMPSSA0H/L)
- Probe the following on boosterpack
  - EPWM0A    : J5/7 Pin 70
  - EPWM1A    : J5/7 Pin 69
  - EPWM2A    : J2/4 Pin 40
  - DEActive0 : J5/7 Pin 49
  - DEActive1 : J5/7 Pin 50
  - CTRIPH    : J6/8 Pin 3
  - CTRIPL    : J6/8 Pin 4 


# Supported Combinations {#EXAMPLES_DRIVERS_CMPSS_DIODE_EMULATION_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/cmpss/cmpss_diode_emulation/

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
CMPSS Diode Emulation Logic Test Started ...
CMPSS Diode Emulation Logic Test Passed!!
All tests have passed!!
\endcode

# References

\imageStyle{am263_epwm_de_reference_1.PNG,width:60%}
 \image html am263_epwm_de_reference_1.PNG "Figure 6a. Diode Emulation Logic"

\n \n

\imageStyle{am263_epwm_de_reference_2.PNG,width:60%}
 \image html am263_epwm_de_reference_2.PNG "Figure 6b. DE Active flag functionality"
 