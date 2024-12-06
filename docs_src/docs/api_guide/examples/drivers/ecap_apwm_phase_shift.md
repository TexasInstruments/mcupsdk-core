# ECAP APWM Phase Shift {#EXAMPLES_DRIVERS_ECAP_APWM_PHASE_SHIFT}

[TOC]

# Introduction
ECAP APWM Phase Shift Feature.

This example showcases the phase shift feature of the ECAP.

Configurations,
1. EPWM and 2 ECAPs (in APWM mode) are configured for period of 5000 SYSCLKs
2. ECAPs are configured for Duty of 50% Active High PWM.
3. ECAP 0 has phase shift value of 0, where are the ECAP1 has phase shift value of 2500.
4. Output Xbars are used for routing the ECAP APWM output

# External Connections
Observe ECAP outputs on the Outputxbar instances
## AM263x
- CC E2, outputxbar 7,8 showcase ecap0,1 apwm outs respectively on HSEC pins 85 and 87
- LP, outputxbar 7,8 showcase ecap0,1 apwm outs respectively on J5.49, J5.50
## AM263Px
- CC E2, outputxbar 7,8 showcase ecap0,1 apwm outs respectively on HSEC pins 85 and 87
- LP, outputxbar 7,8 showcase ecap0,1 apwm outs respectively on J5.49, J5.50
## AM261x
- LP, outputxbar 1,4, showcase ecap 0,1 apwm outs respectively on J4.17, J4.12


# Supported Combinations {#EXAMPLES_DRIVERS_ECAP_APWM_PHASE_SHIFT_COMBOS}

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_apwm_phase_shift/

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
ECAP APWM Phase Shift Test Started ...
ECAP APWM Phase Shift Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_ecap_apwm_phase_shift.png,width:50%}
\image html am263_ecap_apwm_phase_shift.png "Sample Output"