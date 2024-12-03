# Resolver Diagnostics example {#EXAMPLES_DRIVERS_RESOLVER_DIAGNOSITCS}

[TOC]

# Introduction
## Resolver Diagnostics
AM263Px Resolver to Digital Converter offers a wide range of the Diagnostics at HW level to help improve safety. These are crucial to help support the safety aspect of the application without actaully consuming the CPU. The following are the diagnostics that are present in the IP. These are present at each RDC core within the Resolver IP.

1. *Monitor Sin or Cos offset drift (DOS)*
    - Performed over the estimated Offset from the DC Offset block of the RDC
2. *Monitor Sin or Cos gain drift (DOS)*
    - Performed over the Gain Estimation from the PG estimation and correction block of the RDC
3. *Monitor phase drift (phase drift between sin and cos channels) (DOS)*
    - Performed over the Phase Estimation from the PG estimation and correction block of the RDC
4. *Monitor excitation frequency degradation or loss (DOS)*
    - Performed over the input samples from the ADC.
5. *Monitor rotational signal integrity (DOS)*
    - Performed over the demodulated samples after PG correction in RDC
6. *Monitor signal integrity by checking Sin2+Cos2=Constant (DOS)*
    - Performed over the demodulated samples after PG correction in RDC
7. *Monitor Sin or Cos saturation or very high amplitude (DOS)*
    - Performed over the samples collected from the ADC before the DC Offset block of the RDC
8. *Monitor weak Sin or Cos signal below a threshold (LOS)*
    - Performed over the Demodulated Samples at PG block of the RDC
9. *Monitor track2 loop locking to incoming angle (LOT)*
    - Performed in the Track2 loop of the RDC.
10. *Monitor ADC health through calibration channels (DOS)*
    - Performed by software trigger on any channel, preferably on the Cal Channels with known voltages.

It is important from the Software (Application) to verify / check the diagnostics periodically by injecting fault. 

# Example Description
The example intends to demonstrate a way to use the diagnostic checks. Also, the example intends to showcase the usage of the static configuration reads and verification of the static configurations with the initialised values.
## Input Configurations
1. Sequencer Mode 4 is used in the application to avail both RDC for dual motor (or single motor redundant sensing). Please note that, Sequencer modes 0,1,2 activate only RDC0 and are used for the single motor sensing, where as, Sequencer modes 3,4,5 activate two RDC cores RDC0 and RDC1 for dual motor sensing or redundant sensing of single motor.
2. Differential Mode of Sampling is selected.
## Excitation Configurations
1. Amplitude control is 210 or (84%)
2. Sync in is enabled and corresponding PWMxBar is connected with the ISR_SYNC_PWM_EPWM3 Syncout.
3. Over Sampling Ratio is set to be 20.
## Resolver Core Configurations
Both the Cores RDC0 and RDC1 are configured in exact same manner for the redundant sensing application.
1. Band Pass Filter is enabled. Auto DC Offset Correction is disabled, with the manual offset correction values set to be 0.
2. Ideal Sample Mode is set to Mode 3 (manual mode) and Override value is 7. Bottom Sampling is not enabled.
3. Phase Gain estimation is enabled, but the Auto Correction is disabled. Train limit for the estimation is set to 256 Rotations.
4. Manual Gain correction is set to 2.66 (43697) for both Sine and Cosine channels. Manual Phase Correction is set to 0 deg (0).
5. Track2 Configurations include the Kvelfilt Value at 8.
## Interrupt Configurations and ISR details
1. No interrupts from the RDCs are enabled.
2. ISR_SYNC_PWM_EPWM3 is configured for interrupt, via INTxBar0 at 100 uS for the Resolver Angle, velocity reads.
3. ISR_SYNC_PWMDIAGNOSTICS_10MS_ISR_EPWM7_EPWM3 is configured for interrupt, via INTxBar1 at 10 mS for the Resolver Diagnostics Checks.

## SW Track2 details
The example showcases an additional method for computing the angle and velocity which may be optionally used. For quicker accesses, the loop needs to be placed at TCM and should be strictly timed to the excitation signal frequency.

# Watch variables
1. testLog is an array of 2 structs, defined for the example purpose. these contain the test status of the diagnostics performed. These can be monitored. The following are the values and their meaning.
    - TEST_PASS     : 0
    - TEST_FAIL     : -1
    - TEST_NOT_RUN  : 1
2. raw_atan_angle : array, with RDC Core number as index, to read the results from the Atan Angle
3. raw_track2_angle : array, with RDC Core number as index, to read the results from the track2 Angle
4. raw_track2_velocity  : array, with RDC Core number as index, to read the results from the track2 Velocity
5. sw_track2_angle  : array, with RDC Core number as index, to read the results from the sw_track2 Angle
6. sw_track2_velocity : array, with RDC Core number as index, to read the results from the sw_track2 Angle

# External Connections
## DMRD (Dual Motor Resolver Dock) 
\imageStyle{am263p_dmrd.png,width:50%}
\image html am263p_dmrd.png "Dual Motor Resolver Dock"
\imageStyle{am263p_dmrd_topView.png,width:50%}
\image html am263p_dmrd_topView.png "Dual Motor Resolver Dock Top View"
DMRD is used for the interfacing from the external Resovler equiment to the AM263Px-CC E2. 
DMRD Supports the different Sequencer Modes of the Resolver and route the Single ended as well as Differential ended signals to the Control Card. the following table showcase the configurations for each of the combinations supported.

1. For the example, Differential mode of sampling and Sequencer mode 4 are used. and the connections are done corresponding to the below table. 
2. The USB power supply to DMRD powers up the Control Card. For this configuration, pins 1 and 2 of J3 have a jumper populated. (same as above diagram's J3 configuration)
3. A 12V power Supply is connected to the J7 header to power up the excitation signal amplification circuitry.
4. J11 and J15 headers are the Resolver interface test pins. 
    1. On J11, from the **left** (as shown in above picture), are exc-, exc+, cos-. cos+, sin+, sin- respectively. 
    2. On J15, from the **right** (as shown in above picture), are exc-, exc+, cos-. cos+, sin+, sin- respectively. 
    3. For Single motor sensing, the sin and cos signals from J11 can be shorted with the J15, and the resolver can be connected to either of the sides.
    4. For Dual motor sensing, the resolvers from each motor can be connected to J11 and J15.
    \note 
    If only 1 motor and 1 RDC is needed, please use the J11 header for resolver connections, and change the sequencer modes to one of 0,1,2. also, either modify the applcation to use 1 RDC core (instead of 2) or just ignore the 2nd core results.
5. Additionally, J28 J27 pair as well as J30 J29 pair can be used for motor control, and are compatible with the 3PHGANINV BoosterPacks. The corresponding EPWM pins are selected in the syscfg of this example for reference.
    1. A 24V power supply may be used for the 3PHGANINV Driver as per its requirement [visit BOOSTXL-3PHGANINV for more info on 3PHGANINV usage](https://www.ti.com/tool/BOOSTXL-3PHGANINV) .
    2. J31 header shows the GND and Enable pin pairs which can be populated with the Jumpers to enable the BoosterPack.
    3. This example has a placeholder for the motor control code, but is not validating the motor control. User can place their motor control code here.

<table>
<tr>
    <th> Sampling Mode \n and \n J21 Header Connections
    <th> Sequencer Modes
    <th> SW4
    <th> SW3
    <th> SW2
    <th> SW1
    <th> Selection Header
    <th> Signal routed
    <th> Resolver Input
</tr>
<tr>
    <td rowspan=16> **Single Ended**\n<img src="am263p_dmrd_singleEnded.png" alt="" width="50%" height="50%">
    <td rowspan=8> Modes 0,1,2,4,5
    <td rowspan=8> x
    <td rowspan=8> x
    <td rowspan=8> High (towards 1 position)
    <td rowspan=8> Low  (away from 1 position)
    <td rowspan=8> Op Amp
    <td> OpAmp_R0_sin
    <td> ADC_R0_AIN0
</tr>
<tr>
    <td> -
    <td> ADC_R0_AIN1
</tr>
<tr>
    <td> OpAmp_R0_cos
    <td> ADC_R0_AIN2
</tr>
<tr>
    <td> -
    <td> ADC_R0_AIN3
</tr>
<tr>
    <td> OpAmp_R1_sin
    <td> ADC_R1_AIN0
</tr>
<tr>
    <td> -
    <td> ADC_R1_AIN1
</tr>
<tr>
    <td> OpAmp_R1_cos
    <td> ADC_R1_AIN2
</tr>
<tr>
    <td> -
    <td> ADC_R1_AIN3
</tr>
<tr>
    <td rowspan=8> Mode 3
    <td rowspan=8> x
    <td rowspan=8> x
    <td rowspan=8> Low  (away from 1 position)
    <td rowspan=8> High (towards 1 position)
    <td rowspan=8> Op Amp
    <td> OpAmp_R0_sin
    <td> ADC_R0_AIN0
</tr>
<tr>
    <td> -
    <td> ADC_R0_AIN1
</tr>
<tr>
    <td> OpAmp_R1_sin
    <td> ADC_R0_AIN2
</tr>
<tr>
    <td> -
    <td> ADC_R0_AIN3
</tr>
<tr>
    <td> OpAmp_R0_cos
    <td> ADC_R1_AIN0
</tr>
<tr>
    <td> -
    <td> ADC_R1_AIN1
</tr>
<tr>
    <td> OpAmp_R1_cos
    <td> ADC_R1_AIN2
</tr>
<tr>
    <td> -
    <td> ADC_R1_AIN3
</tr>
<tr>
    <td rowspan=16> **Differential Ended**\n<img src="am263p_dmrd_differentialEnded.png" alt="" width="50%" height="50%">
    <td rowspan=8> Modes 0,1,2,4,5
    <td rowspan=8> High (towards 1 position)
    <td rowspan=8> Low  (away from 1 position)
    <td rowspan=8> High (towards 1 position)
    <td rowspan=8> Low  (away from 1 position)
    <td rowspan=8> RDN (Resistor Divider Network)
    <td> RDN_R0_sin+
    <td> ADC_R0_AIN0
</tr>
<tr>
    <td> RDN_R0_sin-
    <td> ADC_R0_AIN1
</tr>
<tr>
    <td> RDN_R0_cos+
    <td> ADC_R0_AIN2
</tr>
<tr>
    <td> RDN_R0_cos-
    <td> ADC_R0_AIN3
</tr>
<tr>
    <td> RDN_R1_sin+
    <td> ADC_R1_AIN0
</tr>
<tr>
    <td> RDN_R1_sin-
    <td> ADC_R1_AIN1
</tr>
<tr>
    <td> RDN_R1_cos+
    <td> ADC_R1_AIN2
</tr>
<tr>
    <td> RDN_R1_cos-
    <td> ADC_R1_AIN3
</tr>
<tr>
    <td rowspan=8> Mode 3
    <td rowspan=8> Low  (away from 1 position)
    <td rowspan=8> High (towards 1 position)
    <td rowspan=8> Low  (away from 1 position)
    <td rowspan=8> High (towards 1 position)
    <td rowspan=8> RDN (Resistor Divider Network)
    <td> RDN_R0_sin+
    <td> ADC_R0_AIN0
</tr>
<tr>
    <td> RDN_R0_sin-
    <td> ADC_R0_AIN1
</tr>
<tr>
    <td> RDN_R1_sin+
    <td> ADC_R0_AIN2
</tr>
<tr>
    <td> RDN_R1_sin-
    <td> ADC_R0_AIN3
</tr>
<tr>
    <td> RDN_R0_cos+
    <td> ADC_R1_AIN0
</tr>
<tr>
    <td> RDN_R0_cos-
    <td> ADC_R1_AIN1
</tr>
<tr>
    <td> RDN_R1_cos+
    <td> ADC_R1_AIN2
</tr>
<tr>
    <td> RDN_R1_cos-
    <td> ADC_R1_AIN3
</tr>
</table>



# Supported Combinations {#EXAMPLES_DRIVERS_RESOLVER_DIAGNOSITCS_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/resolver/resolver_diagnostics/

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
the following is a sample CCS Console output for the example run.,

\code
[Cortex_R5_0] Detected CC version is E2. Calling TCA6424 Drivers for io expander configurations
Setting RESOLVER ADC Mux Select Lines, Index : 14, State : 0
Setting RESOLVER ADC Mux Select Lines, Index : 15, State : 0
Setting RESOLVER ADC Mux Select Lines, Index : 5, State : 0
Setting RESOLVER ADC Mux Select Lines, Index : 6, State : 1
Setting RESOLVER ADC Mux Select Lines, Index : 8, State : 1
Setting RESOLVER ADC Mux Select Lines, Index : 2, State : 1
setting up the EPWM ISR...
Setting up the EPWM ISR Complete!!
Setting up Diagnostics ISR...
Setting up the Diagnostics ISR Complete!!
Setting up Diagnostics...
Diagnostics setup complete!!
Forcing the Sync Pulse from the EPMW0!!
Atan angles from each core || Test Status From Each Core
 core 0 | core 1 || OffsetDrift ExcFreqDeg SigIntegritySinSq HighAmp WeakAmp GainDrift PhaseDrift RotSigIntegrity || OffsetDrift ExcFreqDeg SigIntegritySinSq HighAmp WeakAmp GainDrift PhaseDrift RotSigIntegrity
23.455811	|	23.527222	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.406372	|	23.499756	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.428345	|	23.538208	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.455811	|	23.516235	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.406372	|	23.560181	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.417358	|	23.538208	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.417358	|	23.549194	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.417358	|	23.527222	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.455811	|	23.516235	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.428345	|	23.549194	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.406372	|	23.538208	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.428345	|	23.538208	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.417358	|	23.549194	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.455811	|	23.516235	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
23.428345	|	23.516235	||	0	0	0	0	0	1	1	1	||	0	0	0	0	0	1	1	1	
All tests have passed!!
\endcode
