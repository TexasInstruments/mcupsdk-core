# ECAP Signal Monitoring Example {#EXAMPLES_DRIVERS_ECAP_SIGNAL_MONITORING}

[TOC]

# Introduction
## ECAP Signal Monitoring

Each ECAP instance is equipped with 2 signal monitoring units. These signal monitoring units can be configured in one of the 4 following modes.
1. High Pulse Monitoring Mode                   -> Rising to Falling
2. Low Pulse Monitoring Mode                    -> Falling to Rising
3. Period Monitoring, Rise edge to Rise edge    -> Rising to Rising
4. Period Monitoring, Fall edge to Fall edge    -> Falling to Falling.
5. Edge Monitoring ( Posedge )                  -> Rising Edge : Sync in is mandatory for this mode
5. Edge Monitoring ( Negedge )                  -> Rising Edge : Sync in is mandatory for this mode

For each mode, the CAPEVTs and Stop/Wrap are configured respectively. The Input signal is captured and monitored between Min and Max registers against the Counter value captured with the CAPEVT for the respective mode. Also, these Min and Max registers are 32bit values contain Active-Shadow set. The Shadow to Active loading can be selected to be triggered by either
- a Sync event or
- a Global Load Strobe event.

Debug Mode is present, for when enabled, the events occuring in the min-max window ranges, the minimum and maximum values within the noted min-max window are captured and stored per monitoring unit.

\imageStyle{am263p_ecap_signal_monitoring.png,width:100%}
\image html am263p_ecap_signal_monitoring.png "ECAP Signal Monitoring Summary"

\note
1. The ECAP should be in Absolute mode where no CAPEVT reset the counters for the signal monitoring to be working.

2. For Edge Monitoring, there needs to be a Sync pulse from an EPWM to restart the monitoring.
3. For Pulse or Peroid Monitoring there should not be a Sync in enabled.

4. The error1 and error2 are used for min window violation error and max window violation error in the pulse/ period monitoring. once these are set, in order to restart the monitoring, one needs to re-enable the monitoring unit and start timestamping on ecap.

5. On the Edge monitoring modes, the error1 is set for min or max window violation. the error2 is set if there is never an edge between two sync events. For every sync pulse, the edge monitoring restarts.


## Example Description
This example configures 3 ECAP moudule, with a total of 6 signal monitoring modules, each with one configuration that are supported. All ECAPs are enabled with Interrupts for the Errors from the Signal Monitoring Modules.  An EPWM is configured to provide the input, routed to ECAP via GPI and Inputxbar. This input is varied in a way to cross min and max values from the siganl monitoring modules and the application checks for the interrupt flags are well as the debug range values.

# External Connections
## AM263PX-CC
No external connection is required.

## AM263PX-LP
No external connection is required.

# Supported Combinations {#EXAMPLES_DRIVERS_ECAP_SIGNAL_MONITORING_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_capture_pwm/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_ECAP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ECAP Signal Monitoring Test Test Started ...
Currently the Input waveform is having all the properties in expected state

Pulse Width is modified to exceed bounds
----------------------------------------
	HIGH PULSE Min Bound : 9950	HIGH PULSE Max Bound : 10050	Configured High Pulse Width : 10100
	Observed Min Width : 10000	Observed Max Width : 10050

	LOW PULSE Min Bound  : 9950	LOW PULSE Max Bound  : 10050	Configured low Pulse Width  : 9899
	Observed Min Width : 9949	Observed Max Width : 10000

Edge Position is modified to exceed bounds
------------------------------------------
	Posedge Min Bound : 4899	Posedge Max Bound : 5099	Configured Posedge Position : 4849
	Observed Min Value : 4900	Observed Max Value : 5004

	Negedge Min Bound : 14899	Negedge Max Bound : 15099	Configured Negedge Position : 15149
	Observed Min Value : 15004	Observed Max Value : 15100

Period is modified to exceed bounds
-----------------------------------
	Period Min Bound : 19949	Period Max Bound : 20049	Configured Period : 20149
	Observed Min Value on High Period Monitor : 19999	Observed Max Value on High Period Monitor : 20050
	Observed Min Value on Low Period Monitor : 19999	Observed Max Value on Low Period Monitor : 20050
ECAP Signal Monitoring Test Passed!!
All tests have passed!!
\endcode
