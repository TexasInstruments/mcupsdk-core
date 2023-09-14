# Resolver Angle and Velocity resolution example {#EXAMPLES_DRIVERS_RESOLVER_ANGLE_SPEED}

[TOC]

# Introduction
A Resolver to Digital Converter (RDC) resolves the angle and velocity of a motor from the physical resolver connected to it. this is done so by sending out an excitation sine wave of higher frequency, 20KHz in case of this example, and modulating it across the sine and cosine windings of the resolver, based on the motor shaft position.

\imageStyle{am263p_resovler_modulator_diagram.png,width:100%}
\image html am263p_resovler_modulator_diagram.png "External Resolver Modulation based on the shaft position"

## Resolver to Digital Converter in AM26x Family
RDC in AM26x family, reads the Sine and Cosine inputs via its ADC_R0, ADC_R1 in various configurable parameters. these are then fed to the Hardware loops for various signal error corrections viz., DC offset correction / Band Pass filter, Phase Correction, Gain Correction. the DC offset, Phase-Gain corrections can be configured for Auto mode as the IP supports estimation for the same.

\imageStyle{am263p_resolver_ip_block_diagram.png,width:100%}
\image html am263p_resolver_ip_block_diagram.png "Resolver to Digital Converter Block Diagram"

# Example Description
This example configures the RDC in Single Ended mode, and to take inputs parallely on sine and cosine modulated waveforms from one motor. The Results of the resolver are read in an ISR triggered at every 5 uS, or at 20KHz frequency, equal to that of the Excitation Sine Signal.

### Input Configurations
The *Sequencer mode 0* is selected. The sine signal is sample by ADC_R0 on channel AIN0, while the cosine is sampled by ADC_R1 on Channel AIN0. Both signals are sampled simultaneously by the ADC_Rs.
ADC burst count is disabled, with ADC SoC width is set for 0, i.e., minimum offered from the IP, of 100nS-120nS.
#### Note
The sequencer mode selection automatically takes care of the ADC_R configurations, which the user/ application need not configure any further.

### Excitation Configurations
The *Excitation frequency* is set at *20KHz*, with sync in enabled. The *PWM XBar 2*, which is connected to resolver to digital converter in case of AM263Px, is configured for the EPWM0 syncout signal. The ADC SoC start Delay is set at 20, i.e., 20*1250nS +30nS or, 25.030 uS.
### Tuning Parameters
Manual Tuning parameters set as following.
1. Phase offset correction value : 0
2. Sin Gain Correction Value     : 18000
3. Cos Gain Correction Value     : 18000
4. Ideal Sample Override Value   : 0

### Resolver Core Configurations
- DC offset Correction enabled, for calcoef1,2 being 8,9 and Manual correction offset correction values being 0.
- Ideal Sample Mode is set to 0 (auto detect), with detection threshold of 2500 and bottom sampling disabled.
- Phase Gain Estimation is disabled, and Gain Correction with Manual values of 18000 on both sin and cos, is enabled.
- track2 kvelfilt is set to 8.
#### Note
In Sequencer Mode 0, the Resolver Core 0 takes in the values form the ADC_R0, ADC_R1 on the sin and cos samples and computes for the angle and the velocity. the Resovler Core 1 is not used in this scenario, and hence need not configure the parameters for the Resolver Core 1.

### Interrupt Configurations and ISR details
- EPWM1 is set to generate interrupt from its Event Trigger Submodule when the counter hits compare A value of 20 while incrementing. the interrupt is generated at every period, i.e., the interrupt generation frequency is 20KHz.
- EPMM1 INT is configured at INTxBar0, which further is used for ISR App_epwm1ISR for reading the HW results from RDC and computing SW track2 loop (optional for more accurate results).
#### Note
No Interrupt from the RDC (Resolver to Digital Converter) is enable. Although interrupts may be used for ESM, refer integration details from TRM, the example does not configure them.

## SW Track2 details
- The example showcases an additional method for computing the angle and velocity which may be optionally used. For quicker accesses, the loop needs to be placed at TCM and should be strictly timed to the excitation signal frequency.

# External Connections
## AM263PX-CC
Connect the Single ended inputs from the external resolver to the following.
- Sine Modulated signal     : ADC_R0_AIN0 or HSEC pin
- Cosine Modulated signal   : ADC_R1_AIN0 or HSEC pin
Connect the Excitation Frequency generated from RES0_PWMOUT0 or HSEC pin

# Watch variables:
- raw_atan_angle        : Holds the results of Angle, in degrees, from Arc Tan loop of the RDC.
- raw_track2_angle      : Holds the results of Angle, in degrees, from the raw Track2 Loop of RDC.
- sw_track2_angle       : Holds the results of Angle, in degrees, from the SW track2 loop.
- raw_track2_velocity   : Holds the results of Velocity, in RPS, from the raw Track2 Loop of RDC.
- sw_track2_velocity    : Holds the results of Velocity, in RPS, from the SW Track2 Loop

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOFTWARE_INTERLEAVED_AVERAGING_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/resolver/resolver_angle_speed/

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

\ref DRIVERS_RESOLVER_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
TODO
\endcode
