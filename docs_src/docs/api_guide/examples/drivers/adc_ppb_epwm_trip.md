# ADC PPB EPWM Trip {#EXAMPLES_DRIVERS_ADC_PPB_EPWM_TRIP}

[TOC]

# Introduction

This example demonstrates EPWM tripping through ADC limit detection PPB block. ADC0INT1 is
configured to periodically trigger the ADC0 channel 2 post initial software forced trigger. The limit
detection post-processing block(PPB) is configured and if the ADC results are outside of the defined
range, the post-processing block will generate an ADCxEVTy event. This event is configured as
EPWM trip source through configuring EPWM XBAR and corresponding EPWM’s trip zone and
digital compare sub-modules. The example showcases one-shot, cycle-by-cycle and direct tripping of
PWMs through ADCAEVT1 source via Digital compare submodule.
DAC is configured to generate a ramp waveform and its output is connected as ADC input. PWM tripping is
validated by reading different trip flags.

\note
The ADC EVT routing the Outputxbar in AM263x is missing. Hence in order to bring the ADC EVT to GPIO, we can follow other routes such as routing.
1. ADC EVT is typically paired usage with Tripping the EPWMs. We can route the ADC EVT to PWMxbar, to DCEVT(A/B)(1/2) to Trip Zone TripOut to Outputxbar. Note that it can be a OST or CBC, but then the ADCEVT is now latched instead of its actual status. So, forward the DCEVT(A/B)(1/2) as is to the tripzone. The example showcases this routing.

\imageStyle{am263_adc_ppb_epwm_trip.png,width:50%}
\image html am263_adc_ppb_epwm_trip.png "Module Block diagram"

The example does the below
- Configures SOC0 of ADC0 to read from Channel 2 and ADC0INT1 to periodically trigger conversions after the initial software forced trigger.
- ADC PPB is configured to generate ADC EVT1 if ADC results are outside the range.
- ADC Event is configured as EPWM trip source through EPWM XBAR.
- Trip Zone and Digital Compare submodule are configured to enable different trip combinations.
- EPWM0 (EPWM2 in case of AM261x-LP) is configured for One Shot trip.
- EPWM1 (EPWM3 in case of AM261x-LP) is configured for Cycle by Cycle trip.
- EPWM2 (EPWM4 in case of AM261x-LP) is configured for Direct trip via Digital compare submodule.
- The ramp waveform on DAC output is routed to ADC input.
- The watch variables storing ADC conversion outputs can be used to view the results.
- ADC EVT is routed via OutputXbar to GPIO

Watch  Variables
- adc0Result2 - Digital representation of the voltage on pin ADC0_AIN2

# External Connections
- ADC0_AIN2 pin should be connected to DAC output pin.
- EPWM0A, EPWM0B, EPWM1A, EPWM1B, EPWM2A, EPWM2B can be connected to an oscilloscope to validate tripping.
- In case of AM261x-LP, these are EPWM 2A/B, 3A/B, 4A/B respectively

## AM263PX-CC E2
When using AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)

- Connect HSEC Pin 15 (ADC input) to HSEC Pin 9 (DAC out)
- Capture and analyze waveforms at HSEC Pin 49, HSEC Pin 51, HSEC Pin 53, HSEC Pin 55, HSEC Pin 50, HSEC Pin 52
- Observe ADC EVT via Output Xbar on HSEC Pin 85


## AM263X-CC E2 or AM263X-CC E1

When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)

- Connect HSEC Pin 15 (ADC input) to HSEC Pin 9 (DAC out)
- Capture and analyze waveforms at HSEC Pin 49, HSEC Pin 51, HSEC Pin 53, HSEC Pin 55, HSEC Pin 50, HSEC Pin 52
- Observe ADC EVT via PWMXbar - DCEVT - Output Xbar on HSEC Pin 85


## AM263PX-LP 

When using AM263Px-LP
- Connect booster pack header J5/J7 pin 66 (ADC input) to J1/J3 pin 30 (DAC out)
- Capture and analyze waveforms at booster pack header J2/J4 pin 11, J6/J8 pin 59, J2/J4 pin 37, J2/J4 pin 38, J2/J4 pin 39, J2/J4 pin 40.
- Observe ADC EVT via Output Xbar on J5.50

## AM263X-LP

When using AM263x-LP
- Connect booster pack header J5/J7 pin 66 (ADC input) to J1/J3 pin 30 (DAC out)
- Capture and analyze waveforms at booster pack header J2/J4 pin 11, J6/J8 pin 59, J2/J4 pin 37, J2/J4 pin 38, J2/J4 pin 39, J2/J4 pin 40.
- Observe ADC EVT via PWMXbar - DCEVT - Output Xbar on J5.58

## AM261X-LP

When using AM261x-LP
- Connect booster pack header J7 pin 63 (ADC input) to J3 pin 30 (DAC out) | note for these pins refer to IO Expander configurations on Schematics and syscfg
- Capture and analyze waveforms at booster pack header 
  - EPWM 2A / 2B J4 pin 40/39
  - EPWM 3A / 3B J4 pin 38/37
  - EPWM 4A / 4B J4 pin 36/35
  - Observe ADC EVT via Output Xbar on J6.51

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_PPB_EPWM_TRIP_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_ppb_epwm_trip/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using the watch variables, view the ADC conversion results.
- Observe 6 EPWM waveforms. After ADC input voltage exceeds the PPB limit, PWMs get tripped.

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ADC PPB ePWM Trip Test Started ...
ADC PPB ePWM Trip Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_adc_ppb_epwm_trip_sample_output.png,width:50%}
\image html am263_adc_ppb_epwm_trip_sample_output.png "Sample Output Capture"
