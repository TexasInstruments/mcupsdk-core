# DAC Synced by EPWM {#EXAMPLES_DRIVERS_DAC_EPWM_SYNC}

[TOC]

# Introduction
The DAC Shadow value syncs to DAC Active Value from EPWM Event. The EPWM Peripheral can send out a EPWMSYNCPER signal, on certain events,(check syscfg, under timebase submodule for more details on events) which may be used by the DAC peripheral to sync its shadow to active register transfers. 
The example showcases this in the EPWM ISR, where the event for sync signal and the event for the interrupt are same. now, if there is a new shadow value set, it wouldn't reflect in the active until the next ISR iteration. and similarly, the last set shadow value would appear in the  active register of the DAC.

Note that, the events for EPWMSYNCPER signal and the Interrupt signal from  EPWM are not dependant, in other words, the EPWMSYNCPER may be configured without the need to configure the INT signal from the EPWM. 
 
## Configurations
- DAC loadmode is set to the EPWM SYNC PER signal.
- EPWM SYNCPER signal is generated at its counter equals to period event
- EPWM INT is generated at its counter equals to period event. routed via INT Xbar 0

## External connections
The status Pin, EPWM output A, DAC output may be probed to view the  waveforms
### On AM263x-CC or AM263Px-CC, with HSEC dock
  - Probe the HSEC Pin 9 for DAC output.
  - Probe the HSEC Pin 49 for EPWM A output.
  - Probe the HSEC Pin 51 for STATUS (GPIO 44) output.
### On AM263x-LP or AM263Px-LP,
  - Probe the J3 Pin 30
  - Probe the J4 Pin 11 for EPWM A output.
  - Probe the J8 Pin 59 for STATUS (GPIO 44) output.

# Supported Combinations {#EXAMPLES_DRIVERS_DAC_EPWM_SYNC_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/dac/dac_epwm_sync/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Use an Oscilloscope to view the ramp wave on DAC output pin.

# See Also

\ref DRIVERS_DAC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM synced DAC Test Started ...
EPWM synced DAC Test Passed!!
All tests have passed!!
\endcode


\imageStyle{dac_epwm_sync.png,width:50%}
\image html dac_epwm_sync.png "Sample Output"