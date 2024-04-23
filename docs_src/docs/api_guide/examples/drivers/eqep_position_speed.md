# EQEP Position Speed {#EXAMPLES_DRIVERS_EQEP_POSITION_SPEED}

[TOC]

# Introduction

Position and Speed Measurement Using eQEP

This example provides position and speed measurement using the
capture unit and speed measurement using unit time out of the eQEP module.
ePWM and a GPIO are configured to generate simulated eQEP signals. The
ePWM module will interrupt once every period and call the position/speed
calculation function. This example uses the IQMath library to simplify
high-precision calculations.

\imageStyle{am263_eqep_position_speed_measurement_fig1a.png,width:50%}
\image html am263_eqep_position_speed_measurement_fig1a.png "Figure 1a. Block diagram"

The configuration for this example is as follows
- Maximum speed is configured to 6000rpm (baseRPM)
- Minimum speed is assumed at 10rpm for capture pre-scalar selection
- Pole pair is configured to 2 (polePairs)
- Encoder resolution is configured to 4000 counts/revolution (mechScaler)
- Which means: 4000 / 4 = 1000 line/revolution quadrature encoder
  (simulated by ePWM)
- ePWM (simulating QEP encoder signals) is configured for a 5kHz frequency
  or 300 rpm (= 4 * 5000 cnts/sec * 60 sec/min) / 4000 cnts/rev)

SPEEDRPM_FR: High Speed Measurement is obtained by counting the QEP
input pulses for 10ms (unit timer set to 100Hz).

SPEEDRPM_FR = (Position Delta / 10ms) * 60 rpm

SPEEDRPM_PR: Low Speed Measurement is obtained by measuring time period
of QEP edges. Time measurement is averaged over 64 edges for better results
and the capture unit performs the time measurement using pre-scaled SYSCLK.

Note that the pre-scaler for capture unit clock is selected such that the
capture timer does not overflow at the required minimum frequency.

This example waits for 10 iterations of unit time out event and verifies the
measured speed:  295 < posSpeed.speedRPMFR < 305

 - posSpeed.speedRPMFR - Speed meas. in rpm using QEP position counter
 - posSpeed.speedRPMPR - Speed meas. in rpm using capture unit
 - posSpeed.thetaMech  - Motor mechanical angle (Q15)
 - posSpeed.thetaElec  - Motor electrical angle (Q15)

# External connections

- Connect ePWM0A to eQEP0A (EPWM_A simulates eQEP Phase A signal)
- Connect ePWM0B to eQEP0B (EPWM_B simulates eQEP Phase B signal)
- Connect GPIO48 to eQEP0I (GPIO simulates eQEP Index Signal)

## AM263X-CC or AM263PX-CC

When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC Pin 49 (ePWM0A) to HSEC Pin 102 (eQEP0A)
- Connect HSEC Pin 51 (ePWM0B) to HSEC Pin 100 (eQEP0B)
- Connect HSEC Pin 52 (GPIO48) to HSEC Pin 106 (eQEP0I)

## AM263X-LP or AM263PX-LP
- Connect J2 Pin 11 (ePWM0A) to J25 Pin 1 (eQEP2A)
- Connect J6 Pin 59 (ePWM0B) to J25 Pin 2 (eQEP2B)
- Connect J2 Pin 40 (GPIO48) to J25 Pin 3 (eQEP2I)


# Supported Combinations {#EXAMPLES_DRIVERS_EQEP_POSITION_SPEED_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/eqep/eqep_position_speed/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- View the UART console logs for results

# See Also

\ref DRIVERS_EQEP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EQEP Position Speed Test Started ...
Please ensure EPWM to EQEP loopback is connected...
Please wait few seconds ...
Expected speed = 300 RPM, Measured speed = 299 RPM
Electrical angle (Q15) = 19608
Mechanical angle (Q15) = 26188
Rotation direction = CW, forward
EQEP Position Speed Test Passed!!
All tests have passed!!
\endcode
