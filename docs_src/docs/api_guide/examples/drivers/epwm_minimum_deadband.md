# EPWM Minimum Deadband {#EXAMPLES_DRIVERS_EPWM_MINIMUM_DEADBAND}

[TOC]

# Introduction
In this example ePWM uses Minimum Deadband to do various operations on the signal coming to the MDL block.

# Various MDL configurations

We will be using some terminologies like EPWMxA_DE.sclk and EPWMxA_DE. The reference diagrams at the end can be referred for understanding.
Note that DEPWMA(B).sclk and EPWMxA(B)_DE.sclk are used interchangeably. Same for DEPWMA(B) and EPWMxA(B)_DE.

The screenshots attached in the documentation are as per the results obtained by executing the example on CC. Please note that there are some instance differences between CC and LP.

The mapping is such that:

EPWM0A(B) on CC <--> EPWM0A(B) on LP    \n
EPWM1A(B) on CC <--> EPWM1A(B) on LP    \n
EPWM2A(B) on CC <--> EPWM2A(B) on LP    \n
EPWM3A(B) on CC <--> EPWM3A(B) on LP    \n
EPWM4A(B) on CC <--> EPWM9A(B) on LP    \n
EPWM6A(B) on CC <--> EPWM12A(B) on LP    \n

## Add falling edge delay on same channel

We can introduce a falling edge delay on the same signal.

\imageStyle{am263_epwm_mdl_fed.PNG,width:60%}
 \image html am263_epwm_mdl_fed.PNG "Figure 1.a EPWM showing falling edge delay on same channel"

In our example, EPWM3A is used as the reference for comparing with EPWM2A. EPWM3A has all configurations same as EPWM2A but with MDL disabled.

1.	Enable MINBCFG[ENABLEA] = 1
2.	Select MINDBCFG[SELA] = 0 i.e. EPWMxA_DE.sclk
3.	MINDBCFG[INVERTA] = 0 i.e. no inversion of the reference signal going inside the Delay logic block
4.	MINDBCFG[DELAYA] = 2000 i.e. 2000 * SYSCLK (5ns) = 10 Us
5.	Select Block A as the blocking signal. MINDBCFG[SELBLOCKA] = 0
6.	MINDBCFG[POLSELA] = 1 i.e. OR EPWMxA_DE with the selected block signal (BLOCKA)

The first waveform shows the signal before entering into the MINDB logic block.
The second one shows the BLOCK signal which is falling edge stretched version of the reference signal.
The third shows OR-ed result of BLOCK and the original signal that comes out of the MDL block. The shift in the falling edge (marked in red) is 10 Us.

Note that there's no way to tap the first 2 waveforms from inside the block. We have configured one ePWM instance with same configuration other than the MDL enabled just for reference.

## Add rising edge delay on same channel

We can introduce a rising edge delay on the same signal.

\imageStyle{am263_epwm_mdl_red.PNG,width:60%}
 \image html am263_epwm_mdl_red.PNG "Figure 2.a EPWM showing rising edge delay on same channel"

In our example, EPWM3B is used as the reference for comparing with EPWM2B. EPWM3B has all configurations same as EPWM2B but with MDL disabled.

1.	Enable MINBCFG[ENABLEB] = 1
2.	Select MINDBCFG[SELB] = 0 i.e. EPWMxB_DE.sclk
3.	MINDBCFG[INVERTB] = 1 i.e. inversion of the reference signal going inside the Delay logic block
4.	MINDBCFG[DELAYB] = 3000 i.e. 3000 * SYSCLK (5ns) = 15 Us
5.	Select Block B as the blocking signal. MINDBCFG[SELBLOCKB] = 0
6.	MINDBCFG[POLSELB] = 0 i.e. AND EPWMxA_DE with the inverted selected block signal (BLOCKB)

The first waveform shows the signal before entering into the MINDB logic block.
The second one shows the BLOCK signal which is the falling edge stretched and inverted version of the reference signal.
The third shows inverted BLOCK signal AND-ed with the original signal to get a shift in the rising edge (marked in green). The value of it is 15 Us.


## Add rising edge delay wrt falling edge of other channel of same instance

We can delay the occurrence of rising edge of one signal wrt the falling edge of another signal.

\imageStyle{am263_epwm_mdl_red_wrt_falling.PNG,width:60%}
 \image html am263_epwm_mdl_red_wrt_falling.PNG "Figure 3.a EPWM showing rising edge delay wrt other channel of same instance"

In our example, EPWM1A/B are used as the references for comparing with EPWM0A/B. EPWM1A/B have all configurations same as EPWM0A/B but with MDL disabled.

In this case, the upper 2 signals show channel A and B output before entering the MDL logic. They are complementary to each other.
With help of MDL, we can delay the rising of B by some amount wrt falling edge of A (marked in blue).
Similarly, we can delay the rising of A by some amount wrt falling edge of B (marked in orange).
In order to achieve this, we can do the following configuration:
For output channel A:
1.	Enable MINBCFG[ENABLEA] = 1
2.	Select MINDBCFG[SELA] = 0 i.e. EPWMxA_DE.sclk
3.	MINDBCFG[INVERTA] = 0 i.e. no inversion of the reference signal going inside the Delay logic block
4.	MINDBCFG[DELAYA] = 2000 i.e. 2000 * SYSCLK (5ns) = 10 Us
5.	Select Block B as the blocking signal. MINDBCFG[SELBLOCKA] = 1
6.	MINDBCFG[POLSELA] = 0 i.e. AND EPWMxA_DE with the inverted selected block signal (BLOCKB)
For output channel B:
1.	Enable MINBCFG[ENABLEB] = 1
2.	Select MINDBCFG[SELB] = 0 i.e. EPWMxB_DE.sclk
3.	MINDBCFG[INVERTB] = 0 i.e. no inversion of the reference signal going inside the Delay logic block
4.	MINDBCFG[DELAYB] = 2000 i.e. 2000 * SYSCLK (5ns) = 10 Us
5.	Select Block A as the blocking signal. MINDBCFG[SELBLOCKB] = 1
6.	MINDBCFG[POLSELB] = 0 i.e. AND EPWMxA_DE with the inverted selected block signal (BLOCKA)

For better understanding on how the block signal A is applied on output channel B, the below diagram can be referred.

\imageStyle{am263_epwm_mdl_red_wrt_falling_block.PNG,width:60%}
 \image html am263_epwm_mdl_red_wrt_falling_block.PNG "Figure 3.b EPWM showing how block signal is applied"

The second waveform shows the block signal: which is the falling edge stretched of reference signal i.e. EPWM0A before entering MDL and then inverting it.
This is then AND-ed with EPWM0B to get the forth waveform i.e. EPWM0B after MDL.

## Add rising edge wrt rising edge of other channel  of same instance

Delay the rising edge wrt the rising edge of the other channel.

\imageStyle{am263_epwm_mdl_red_wrt_rising.PNG,width:60%}
 \image html am263_epwm_mdl_red_wrt_rising.PNG "Figure 4.a EPWM showing rising edge delay wrt other channel of same instance"

In our example, EPWM3A/B are used as the references for comparing with EPWM4A/B. EPWM3A/B have all configurations same as EPWM4A/B but with MDL disabled.

This is similar to Case 2 where the Block signal is applied on itself but here EPWMxB can use the BLOCKA as its blocking signal.

## Add delay wrt other channel of other instance

Till now all cases were based on channel A and B of the same epwm instance. This case shows that we can even add delay wrt to signal of some other output channel belonging to some other instance.

\imageStyle{am263_epwm_mdl_delay_wrt_instance_connection.PNG,width:60%}
 \image html am263_epwm_mdl_delay_wrt_instance_connection.PNG "Figure 5.a MDLXBar connection between EPWM3 and EPWM6"

\imageStyle{am263_epwm_mdl_delay_wrt_instance.PNG,width:60%}
 \image html am263_epwm_mdl_delay_wrt_instance.PNG "Figure 5.b EPWM showing delay wrt channel of other instance"

The second waveform is generated by EPWM6B.

The EPWM6_A before entering into MDL is shown in the second waveform.
Inside EPWM6_A, we have taken the reference signal for block signal as EPWM3_A (the first waveform). Then added delay of 4000 i.e. 20 Us wrt falling edge of EPWM3_A. The rising edge of EPWM6_A occurs only after this delay. Till then it is pulled low by the BLOCK signal.
We select EPWM3A_sclk as the output for MDLXBAR_1.
1.	Enable MINBCFG[ENABLEA] = 1
2.	Select MINDBCFG[SELA] = 1 i.e. Output from MDLXBar_1 (which routes the EPWM3A_sclk)
3.	MINDBCFG[INVERTA] = 0 i.e. no inversion of the reference signal going inside the Delay logic block
4.	MINDBCFG[DELAYA] = 4000 i.e. 4000 * SYSCLK (5ns) = 20 Us
5.	Select Block A as the blocking signal. MINDBCFG[SELBLOCKA] = 0
6.	MINDBCFG[POLSELA] = 0 i.e. AND EPWMxA_DE with the inverted selected block signal (BLOCKB)

# External Connections
- EPWM0_A/B pin can be connected to an oscilloscope to view the waveform.
- EPWM1_A, pin can be connected to an oscilloscope to view the waveform.
- EPWM2_B, pin can be connected to an oscilloscope to view the waveform.
- EPWM3_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM4_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM5_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM6_A/B, pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC or AM263PX-CC
When using AM263x-CC or AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC pin 49, 51 to scope for EPWM0A(B)
- Connect HSEC pin 53, 55 to scope for EPWM1A(B)
- Connect HSEC pin 50, 52 to scope for EPWM2A(B)
- Connect HSEC pin 54, 56 to scope for EPWM3A(B)
- Connect HSEC pin 57, 59 to scope for EPWM4A(B)
- Connect HSEC pin 58, 60 to scope for EPWM6A(B)

## AM263X-LP or AM263PX-LP
- Connect J2/J4 pin 11, J6/J8 pin 59 to scope for EPWM0A(B)
- Connect J2/J4 pin 37, 38 to scope for EPWM1A(B)
- Connect J2/J4 pin 39, 40 to scope for EPWM2A(B)
- Connect J6/J8 pin 77, 78 to scope for EPWM3A(B)
- Connect J6/J8 pin 75, 76 to scope for EPWM9A(B)
- Connect J6/J8 pin 53, 57 to scope for EPWM12A(B)

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_MINIMUM_DEADBAND_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_minimum_deadband/

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
EPWM Minimum Deadband Test Started ...
App will wait for 60 seconds
EPWM Minimum Deadband Test Passed!!
All tests have passed!!
\endcode

# References

\imageStyle{am263_epwm_mdl_icl_blockdiagram.PNG,width:60%}
 \image html am263_epwm_mdl_icl_blockdiagram.PNG "Figure 6.a Block diagram for MDL"

\n\n
\imageStyle{am263_epwm_mdl_reference1.PNG,width:60%}
 \image html am263_epwm_mdl_reference1.PNG "Figure 6.b Understanding PWMxA(B).sclk"
\n\n
PWMxA.sclk and PWMxB.sclk are tapped from the EPWM modules. These two signals are PWM signals which are tapped before the signals pass through the high resolution delay lines.
\n
\imageStyle{am263_epwm_mdl_reference2.PNG,width:60%}
 \image html am263_epwm_mdl_reference2.PNG "Figure 6.c Understanding the difference b/w DEPWMxA(B).sclk and DEPWMxA(B)"

In our case, DEPWMA(B), DEPWMA(B).sclk are driven by PWMA(B) and PWMA(B).sclk respectively.

