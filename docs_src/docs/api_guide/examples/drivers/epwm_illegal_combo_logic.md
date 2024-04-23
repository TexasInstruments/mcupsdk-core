# EPWM Illegal Combo Logic {#EXAMPLES_DRIVERS_EPWM_ILLEGAL_COMBO_LOGIC}

[TOC]

# Introduction

The purpose of the example is to show some of the operations that can be done using ICL.
We can route a signal of EPWM instance through some other EPWM instance without even configuring the latter.
We use ICLXBar along with the ICL block functionalities provided inside each instance to get the desired results.

# Various ICL configurations

The screenshots attached in the documentation are as per the results obtained by executing the example on CC. Please note that there are some instance differences between CC and LP.

The mapping is such that:

\cond SOC_AM263X
EPWM0A(B) on CC <--> EPWM0A(B) on LP   \n
EPWM1A    on CC <--> EPWM1A on LP      \n
EPWM2B    on CC <--> EPWM2B on LP      \n
EPWM3A(B) on CC <--> EPWM3A(B) on LP   \n
EPWM4A(B) on CC <--> EPWM9A(B) on LP   \n
EPWM5A(B) on CC <--> EPWM11A(B) on LP  \n
EPWM6A(B) on CC <--> EPWM12A(B) on LP  \n
\endcond
\cond SOC_AM263PX
EPWM0A(B) on CC <--> EPWM0A(B) on LP   \n
EPWM1A    on CC <--> EPWM1A on LP      \n
EPWM2B    on CC <--> EPWM2B on LP      \n
EPWM3A(B) on CC <--> EPWM3A(B) on LP   \n
EPWM4A(B) on CC <--> EPWM9A(B) on LP   \n
EPWM5A(B) on CC <--> EPWM13A(B) on LP  \n
EPWM6A(B) on CC <--> EPWM12A(B) on LP  \n
\endcond

## XOR and XNOR

Here, we configure EPWM3 with some TBPRD, CMPA and CMPB values and define some actions on each event.
Since no MDL functions are performed, so the signal coming out of AQ submodule is what comes out of MDL blocks A and B respectively.
They are then routed through separate ICLXBARs.

\imageStyle{am263_epwm_icl_xor_xnor_connection.PNG,width:60%}
 \image html am263_epwm_icl_xor_xnor_connection.PNG "Figure 1.a Xbar connectivity for ICL"

In EPWM4, we can define the combo logic. As IN3 gets its input from ICLXBARs, to output the same signal we can define the truth table as

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
0                  | X                 | X                     | 0
1                  | X                 | X                     | 1

Inside EPWM3, for ICL Block A we need to put logic such that Output = IN1 xor IN2; where IN1 gets EPWM3A_MDL and IN2 gets EPWMxB_MDL.
Similarly for ICL Block B, output = IN1 xnor IN2; where IN1 gets EPWMxB_MDL and IN2 gets EPWM3A_MDL.

For ICL block A,

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
X                  | 0                 | 0                     | 0
X                  | 0                 | 1                     | 1
X                  | 1                 | 0                     | 1
X                  | 1                 | 1                     | 0

For ICL block B,

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
X                  | 0                 | 0                     | 1
X                  | 0                 | 1                     | 0
X                  | 1                 | 0                     | 0
X                  | 1                 | 1                     | 1

\imageStyle{am263_epwm_icl_xor_xnor.PNG,width:60%}
 \image html am263_epwm_icl_xor_xnor.PNG "Figure 1.b EPWM showing XOR and XNOR operation on channels A and B"

## NAND and NOR

Here, we configure EPWM5A with some TBPRD, CMPA and CMPB values and define some actions on each event.
Since no MDL functions are performed, so the signal coming out of AQ submodule is what comes out of MDL blocks A and B respectively.

They are then routed through separate ICLXBARs.

\imageStyle{am263_epwm_icl_nand_nor_connection.PNG,width:60%}
 \image html am263_epwm_icl_nand_nor_connection.PNG "Figure 2.a Xbar connectivity for ICL"

In EPWM6, we can define the combo logic. As IN3 gets its input from ICLXBARs, to output the same signal we can define the truth table as

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
0                  | X                 | X                     | 0
1                  | X                 | X                     | 1

Inside EPWM5, for ICL Block A we need to put logic such that Output = IN1 nand IN2; where IN1 gets EPWM5A_MDL and IN2 gets EPWMxB_MDL.
Similarly for ICL Block B, output = IN1 nor IN2; where IN1 gets EPWMxB_MDL and IN2 gets EPWM5A_MDL.

For ICL block A,

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
X                  | 0                 | 0                     | 1
X                  | 0                 | 1                     | 1
X                  | 1                 | 0                     | 1
X                  | 1                 | 1                     | 0

For ICL block B,

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
X                  | 0                 | 0                     | 1
X                  | 0                 | 1                     | 0
X                  | 1                 | 0                     | 0
X                  | 1                 | 1                     | 0

\imageStyle{am263_epwm_icl_nand_nor.PNG,width:60%}
 \image html am263_epwm_icl_nand_nor.PNG "Figure 1.b EPWM showing NAND and NOR operation on channels A and B"

## Both NOT HIGH at the same time

EPWM0 is configured such that when TripIN gets HIGH, then it pulls EPWM0A to high.
Here the EPWM0A_MDL has been routed through EPWM2 in the same process as mentioned in the above cases.
EPWM2B shows how EPWM0B was before ICL was applied to it.
The ICL for block B inside EPWM0 was configured such that EPWM0A and EPWM0B cannot be high at the same time. So whenever EPWM0A_MDL gets pulled because of the trip, the EPWM0B should be pulled down.
Since ICL Block A is disabled, EPWM0A_MDL will go as the final output.

\imageStyle{am263_epwm_icl_both_not_high_connection.PNG,width:60%}
 \image html am263_epwm_icl_both_not_high_connection.PNG "Figure 3.a Connection between EPWMs"

For ICL Block B, the truth table will be like:

IN3                | IN2               | IN1                   | Force
-------------------|-------------------|-----------------------|-----------------------
X                  | 0                 | 0                     | 0 (IN1)
X                  | 0                 | 1                     | 1 (IN1)
X                  | 1                 | 0                     | 0 (IN1)
X                  | 1                 | 1                     | 0 (both cannot be high, so pull this low)

\imageStyle{am263_epwm_icl_both_not_high.PNG,width:60%}
 \image html am263_epwm_icl_both_not_high.PNG "Figure 3.b EPWM showing how channel B gets pulled to low when channel A is high"

# External Connections
- EPWM0_A/B pin can be connected to an oscilloscope to view the waveform.
- EPWM1_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM2_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM3_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM4_A/B, pin can be connected to an oscilloscope to view the waveform.
- EPWM6_A/B, pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC or AM263Px-CC
When using AM263x-CC or AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC pin 49, 51 to scope for EPWM0A(B)
- Connect HSEC pin 53 to scope for EPWM1A
- Connect HSEC pin 52 to scope for EPWM2B
- Connect HSEC pin 54, 56 to scope for EPWM3A(B)
- Connect HSEC pin 57, 59 to scope for EPWM4A(B)
- Connect HSEC pin 61, 63 to scope for EPWM5A(B)
- Connect HSEC pin 58, 60 to scope for EPWM6A(B)

\cond SOC_AM263X
## AM263X-LP
- Connect J2/J4 pin 11, J6/J8 pin 59 to scope for EPWM0A(B)
- Connect J2/J4 pin 37 to scope for EPWM1A
- Connect J2/J4 pin 40 to scope for EPWM2B
- Connect J6/J8 pin 77, 78 to scope for EPWM3A(B)
- Connect J6/J8 pin 75, 76 to scope for EPWM9A(B)
- Connect J6/J8 pin 51, 52 to scope for EPWM11A(B)
- Connect J6/J8 pin 53, 57 to scope for EPWM12A(B)
\endcond

\cond SOC_AM263PX
## AM263PX-LP
- Connect J2/J4 pin 11, J6/J8 pin 59 to scope for EPWM0A(B)
- Connect J2/J4 pin 37 to scope for EPWM1A
- Connect J2/J4 pin 40 to scope for EPWM2B
- Connect J6/J8 pin 77, 78 to scope for EPWM3A(B)
- Connect J6/J8 pin 75, 76 to scope for EPWM9A(B)
- Connect J6/J8 pin 79, 80 to scope for EPWM13A(B)
- Connect J6/J8 pin 53, 57 to scope for EPWM12A(B)
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_ILLEGAL_COMBO_LOGIC_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_illegal_combo_logic/

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
EPWM Illegal Combo Logic Test Started ...
App will wait for 60 seconds
EPWM Illegal Combo Logic Test Passed!!
All tests have passed!!
\endcode

# References

\imageStyle{am263_epwm_mdl_icl_blockdiagram.PNG,width:60%}
 \image html am263_epwm_mdl_icl_blockdiagram.PNG"Figure 4a. ICL taking input from MDL"

\imageStyle{am263_epwm_icl_blockdiagram.PNG,width:60%}
 \image html am263_epwm_icl_blockdiagram.PNG "Figure 4b. Illegal Combo Logic Block Diagram"
