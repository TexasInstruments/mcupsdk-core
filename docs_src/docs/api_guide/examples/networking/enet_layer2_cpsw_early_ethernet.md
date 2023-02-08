# Enet L2 CPSW Early Ethernet Example {#EXAMPLES_ENET_CPSW_EARLY_ETHERNET}

[TOC]

# Introduction

The Enet CPSW early ethernet example is dedicated to demonstrate the early ethernet functionality of CPSW3G showing Strapped and forced mode configuration of PHY.


\cond SOC_AM263X

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack

\endcond

This example does:
    - Demonstrates the strapped PHY functionality(Auto-Negotiation disabled).
    - Target-side application running on a Cortex R5F core.
    - The example enables both the ports of CPSW in MAC mode, creates a packet and sent it from port 1 to port 2.
    - when the packet is received on port 2, the application prints out the profiling nodes and halts in a infinite loop.



# Supported Combinations

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_cpsw_early_ethernet

\endcond

# Modifications needed before running the example:

## EVM Board modification

- On AM263x-cc(E1), we need to disable Auto-negotiation in the PHY  and force it to strapped mode with external starp resistors connected on PHY.
- To do that, modify below registers mentioned in below section for RGMII1 and RGMII2
- For other variants of AM263x, refer the schematics of board to modify the strap resistors for that specific EVM.

### Modifications for CPSW RGMII1:
    
- Remove Strapping resistors from R270, R115, R278 and add the same to R101, R104, R284. All resistors are of same value.

### Modifications for CPSW RGMII2:
    
- Remove Strapping resistors from R248, R240, R74 and add the same to R64, R60, R63. All resistors are of same value.

## SW modifications

- We need to disable Enet Logs and recompiler the enet-cpsw library.
- To do so, remove the ENABLE_ENET_LOG define from mcu_plus_sdk/source/networking/enet/makefile.cpsw.am263x.r5f.ti-arm-clang makefile.
- Recompile the enet-cpsw library.

# Tips to minimize link up time for Ethernet applications

- Auto-Negotiation on Am263x-cc phy takes around 2-3 seconds, so to achieve quick link up, we cannot operate in Auto-Neg mode and have to use strapped and forced modes.
- Forced mode needs to be enabled on both sides of nodes.
- Typically Enet examples wait for 1 second after checking for link status.This is mainly considering auto-negotiation timing, but if forced and strapped mode, this delay can be significantly low, around 3ms(tested).
- HW strapped of PHY is needed to disble auto-negotiation when the phy is powered on.


# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)


## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.



## Sample output for L2 CPSW Boot example

\code

=====================================================
     Boot Time Profiling logs in Microseconds(us)    
=====================================================
Application start time: 14 
Enet-lld initialisation done time : 3024
Both ports linked up time: 8039 
Time for First packet sent out from Port 1: 8057 
Time for First packet received on Port 2: 8102 
Application Terminating...

\endcode

# See Also

\ref NETWORKING
