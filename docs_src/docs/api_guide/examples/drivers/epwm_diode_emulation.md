# EPWM Diode Emulation {#EXAMPLES_DRIVERS_EPWM_DIODE_EMULATION}

[TOC]

# Introduction
This example demonstrates the functionalities of epwm working in DE mode, the trip inputs are sourced from other epwm instances only and not from cmpss.

# EPWM Basic Configuration

All total 7 epwm instances have been used out of which EPWM0A and EPWM1A are made to be TripH and TripL sources respectively.

The screenshots attached in the documentation are as per the results obtained by executing the example on CC. Please note that there are some instance differences between CC and LP.

The mapping is such that:
\cond SOC_AM263X
EPWM0A(B) on CC <--> EPWM0A(B) on LP  \n
EPWM1A    on CC <--> EPWM1A on LP     \n
EPWM2A(B) on CC <--> EPWM2A(B) on LP  \n
EPWM3A(B) on CC <--> EPWM3A(B) on LP  \n
EPWM4A(B) on CC <--> EPWM9A(B) on LP  \n
EPWM5A(B) on CC <--> EPWM11A(B) on LP \n
EPWM6A(B) on CC <--> EPWM12A(B) on LP \n
\endcond
\cond SOC_AM263PX
EPWM0A(B) on CC <--> EPWM0A(B) on LP  \n
EPWM1A    on CC <--> EPWM1A on LP     \n
EPWM2A(B) on CC <--> EPWM2A(B) on LP  \n
EPWM3A(B) on CC <--> EPWM3A(B) on LP  \n
EPWM4A(B) on CC <--> EPWM9A(B) on LP  \n
EPWM5A(B) on CC <--> EPWM13A(B) on LP \n
EPWM6A(B) on CC <--> EPWM12A(B) on LP \n
\endcond

\imageStyle{am263_epwm_de_basic_config.PNG,width:40%}
 \image html am263_epwm_de_basic_config.PNG "Figure 1.a Basic waveform configuration"
The above diagram shows the configuration for EPWM0A (TripH) and EPWM1A (TripL) and the ones marked as Channel A and B are consistent across the remaining 5 epwm instances.

The table below summarizes the configurations of the EPWMs for the given submodules mentioned in the column.

\imageStyle{am263_epwm_de_config_table_part1.PNG,width:60%}
\image html am263_epwm_de_config_table_part1.PNG
\imageStyle{am263_epwm_de_config_table_part2.PNG,width:60%}
 \image html am263_epwm_de_config_table_part2.PNG "Table 1.a Basic Configuration of EPWMs"

All EPWM instances with DE mode enabled are configured such that on entering into the DE mode, Channel A will be driver by TripH and Channel B will be driven by TripL.
DE mode for EPWM4 is disabled and has been used here for the purpose of comparison.
Note that EPWM0B has been used to show the TripHorTripL waveform by using ICL for the sake for easy understanding and is not necessary for this example.

\imageStyle{am263_epwm_de_xbar_config.PNG,width:60%}
 \image html am263_epwm_de_xbar_config.PNG "Figure 1.c XBar Configuration b/w Trip sources and EPWM"

OutputXbar13 is used to route the PWMSYNCOUT from EPWM4.
OutputXbar14 is used to route the DE_Trip from EPWM6.

In the below section, we will be discussing about various cases which can be observed by varying the configurations inside DE.

# Various DE configurations

## EPWM in CBC mode

EPWM2 is configured in CBC mode where on occurrence of every PWMSYNC it evaluates if TRIPHorTRIPL condition is present and if not then DEACTIVE flag is cleared.

Channel A:

\imageStyle{am263_epwm_de_cbc_A_deentry.PNG,width:60%}
 \image html am263_epwm_de_cbc_A_deentry.PNG "Figure 2.a EPWM in CBC mode entering DEACTIVE zone for channel A"

Initially when TripHorTripL was 0, EPWM2A was following the normal action configured in the AQ. We can compare this with EPWM4A.
Once TripHorTripL gets High as marked by the red single marker, it enters into the DEACTIVE mode and now EPWM2A is driven by TripH as per the configuration.

\imageStyle{am263_epwm_de_cbc_A_deexit.PNG,width:60%}
 \image html am263_epwm_de_cbc_A_deexit.PNG "Figure 2.b EPWM in CBC mode exiting DEACTIVE zone for channel A"

The region bounded by the green pair of markers denote the zone where EPWM2 is out of DEACTIVE mode. How?
Observe the PWMSYNCOUT coming before the instance marked by first green marker, EPWM2A was still in the DEACTIVE mode as TripHorTripL was high. On occurrence of this PWMSYNC (marked by first green), since TripHorTripL goes low, DEACTIVE flag will get cleared and EPWM2A starts following the normal AQ actions (take EPWM4A as reference).
Recall that we have configured EPWM0 and EPWM1 such that they alternate their AQ actions after a period of 5 and 10 TBPRDs respectively. This is the reason why we see that TripHorTripL remains LOW for min(5, 10) i.e. 5 PWMSYNCs. After that again EPWM2 enters into DEACTIVE mode.

Channel B:

\imageStyle{am263_epwm_de_cbc_B_deentry.PNG,width:60%}
 \image html am263_epwm_de_cbc_B_deentry.PNG "Figure 2.c EPWM in CBC mode entering DEACTIVE zone for channel B"

We take the same instant for explaining the behaviour for EPWM2B.
At red marker, EPWM2 enters into DEACTIVE mode. Till that point EPWM2B was driven by its AQ actions (take EPWM4B as reference). After this red point, TripL drives EPWM2B and not by AQ anymore. The effect can be observed better at the violet marker point where EPWM2B goes low as TripL also goes low.
For the next 5 PWMSYNCs, TripHorTripL is High so EPWM2B is driven by TripL during this entire period.

\imageStyle{am263_epwm_de_cbc_B_deexit.PNG,width:60%}
 \image html am263_epwm_de_cbc_B_deexit.PNG "Figure 2.d EPWM in CBC mode exiting DEACTIVE zone for channel B"

This shows the period where EPWM2 exits the DE mode and EPWM2B generates signals as per its AQ actions until the second green marker. At this point it again enters the DEACTIVE mode and since TripL is low, EPWM2B is immediately pulled down to Low.
The little green High signal that we observe at the beginning of this Exit zone is due to the AQ action. As PWMSYNC clears the DEACTIVE flag, EPWM2B resumes its normal behaviour (take EPWM4B as reference) just when its about to go low with its TBCTR reaching TBPRD.


## EPWM in OST mode

EPWM3 works in DE OST mode where if once entered into the DEACTIVE mode, it remains there until the flag is software cleared. Here we don’t software clear it to see the difference.
EPWM2A(B) are also shown for comparison along with EPWM4A(B).

Channel A:

\imageStyle{am263_epwm_de_ost_A.PNG,width:60%}
 \image html am263_epwm_de_ost_A.PNG "Figure 3.a EPWM in OST mode for channel A"

It will be redundant to explain the behaviour of EPWM in DEACTIVE mode. But what’s important here to notice is that, even after exiting from DEACTIVE mode (marked by green pointer) unlike EPWM2A, EPWM3A remains low i.e. it’s still being driven by TripH.

Channel B:

\imageStyle{am263_epwm_de_ost_B.PNG,width:60%}
 \image html am263_epwm_de_ost_B.PNG "Figure 3.b EPWM in OST mode for channel B"

For EPWM3B also, it doesn’t recover even after exiting the DEACTIVE mode and remains low in this period as TripL is also low.

## EPWM in CBC mode with Re-entry delay

EPWM5 works in DE CBC mode but with Re-entry delay = 8. This means once EPWM5 exits the DEACTIVE mode, for next 8 PWMSYNCs it won’t enter into DEACTIVE mode even if TripHorTripL is High.
EPWM2A(B) are also shown for comparison.

Channel A:

\imageStyle{am263_epwm_de_cbc_reentry_A.PNG,width:60%}
 \image html am263_epwm_de_cbc_reentry_A.PNG "Figure 4.a EPWM in CBC mode with Re-entry = 8 for channel A"

The red marker marks the exit of DEACTIVE because that’s the point where PWMSYNC comes and TripHorTripL is low.
At this point, the Re-entry counter will get loaded with the Re-entry value i.e. 8 in our case. It will decrement by one on every PWMSYNC. The trip signals are blocked as long as counter value is greater than 0.
The counter reaches 0 on occurrence of PWMSYNC which comes at the dotted blue marker. If we see carefully, the EPWM5A now is driven by TripH from this point.

Channel B:

\imageStyle{am263_epwm_de_cbc_reentry_B.PNG,width:60%}
 \image html am263_epwm_de_cbc_reentry_B.PNG "Figure 4.b EPWM in CBC mode with Re-entry = 8 for channel B"

Here also we can see that till the counter reaches 0, EPWM5B continues its normal action by AQ. On the counter reaching 0, TripL starts driving it.

## EPWM in OST mode with DE Monitor Mode Enabled

EPWM6 works in DEL OST mode and here the DE mode monitor is enabled. The DEMONTHRES.THRESHOLD = 200, DEMONSTEP.INCSTEP = 20, DEMONSTEP.DECSTEP = 0. So on every PWMSYNC, if TripHorTripL is High, DEMONCNT.CNT is increased in steps of 20 (INCSTEP) and decremented in step of 0 (DECSTEP) if TripHorTripL is Low.

Once the counter reaches the threshold, DE_TRIP pulse is generated and the counter is cleared.

\imageStyle{am263_epwm_de_ost_monitor_connection.PNG,width:60%}
 \image html am263_epwm_de_ost_monitor_connection.PNG "Figure 5.a Connection for EPWM in OST mode with DE Monitor Mode enabled"

In this example we take the DE_TRIP pulse as the TRIPIN to the same instance and call TZ interrupt on the occurence of trip event. Inside the ISR we software clear the DEACTIVE flag after putting some delay.

Channel A:

\imageStyle{am263_epwm_de_ost_monitor_A.PNG,width:60%}
 \image html am263_epwm_de_ost_monitor_A.PNG "Figure 5.b EPWM in OST mode with DE Monitor Mode enabled for Channel A"

The 5th waveform shows the DE trip events that is generated every time the DEMONCNT.CNT reaches the threshold.
Initially when no trip is there, the counter will be set to its reset value i.e. 0. When PWMSYNC comes and TripHorTripL ==1, then counter starts counting in the steps of 20. Again when TripHorTripL is gone, the counter will hold the value i.e. 100 in this case. Once it reaches 200 which is the threshold the DE_trip comes.
On getting this DE_trip as the TRIPIN source to EPWM6, the DEACTIVE flag is cleared after some delay. In the ISR we have written some dummy code to introduce the delay. The reason behind doing this is that we want to push the clearing of the DEACTIVE flag to the time where TripHorTripL is low so that the clearing will take effect. Otherwise if TripHorTripL is high and the clearing also happens that the same time, then it won’t take any effect.
We can see at the green single marker point that it has resumed its AQ action (take EPWM4A as reference).
EPWM3A which works in OST has been added for comparison.


Channel B:

\imageStyle{am263_epwm_de_ost_monitor_B.PNG,width:60%}
 \image html am263_epwm_de_ost_monitor_B.PNG "Figure 5.c EPWM in OST mode with DE Monitor Mode enabled for Channel B"

Similarly for channel B, the flag gets cleared at the point marked by the green single pointer after which it resumes it normal AQ actions.

# External Connections
- EPWM0_A(B) pin can be connected to an oscilloscope to view the waveform.
- EPWM1_A, pin can be connected to an oscilloscope to view the waveform.
- EPWM2_A(B), pin can be connected to an oscilloscope to view the waveform.
- EPWM3_A(B), pin can be connected to an oscilloscope to view the waveform.
- EPWM4_A(B), pin can be connected to an oscilloscope to view the waveform.
- EPWM5_A(B), pin can be connected to an oscilloscope to view the waveform.
- EPWM6_A(B), pin can be connected to an oscilloscope to view the waveform.
- Output Xbar 13 and 14 can be connected to an oscilloscope to view the waveform.

## AM263PX-CC
When using AM263PX-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect HSEC pin 49, 51 to scope for EPWM0A(B)
- Connect HSEC pin 53 to scope for EPWM1A
- Connect HSEC pin 50, 52 to scope for EPWM2A(B)
- Connect HSEC pin 54, 56 to scope for EPWM3A(B)
- Connect HSEC pin 57, 59 to scope for EPWM4A(B)
- Connect HSEC pin 61, 63 to scope for EPWM5A(B)
- Connect HSEC pin 58, 60 to scope for EPWM6A(B)
- Connect HSEC pin 68 to scope for OutputXbar 13
- Connect HSEC pin 70 to scope for OutputXbar 14

## AM263PX-LP
- Connect J2/J4 pin 11, J6/J8 pin 59 to scope for EPWM0A(B)
- Connect J2/J4 pin 37 to scope for EPWM1A
- Connect J2/J4 pin 39, 40 to scope for EPWM2A(B)
- Connect J6/J8 pin 77, 78 to scope for EPWM3A(B)
- Connect J6/J8 pin 75, 76 to scope for EPWM9A(B)
- Connect J6/J8 pin 51, 52 to scope for EPWM11A(B)
- Connect J6/J8 pin 53, 57 to scope for EPWM12A(B)

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_DIODE_EMULATION_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_diode_emulation/

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
EPWM Diode Emulation Logic Test Started ...
App will wait for 60 seconds
EPWM Diode Emulation Logic Test Passed!!
All tests have passed!!
\endcode

# References

\imageStyle{am263_epwm_de_reference_1.PNG,width:60%}
 \image html am263_epwm_de_reference_1.PNG "Figure 6a. Diode Emulation Logic"

\n \n

\imageStyle{am263_epwm_de_reference_2.PNG,width:60%}
 \image html am263_epwm_de_reference_2.PNG "Figure 6b. DE Active flag functionality"
