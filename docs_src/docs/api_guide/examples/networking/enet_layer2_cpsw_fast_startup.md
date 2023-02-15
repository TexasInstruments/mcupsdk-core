# Enet Layer 2 CPSW Fast Startup Example {#EXAMPLES_ENET_LAYER2_CPSW_FAST_STARTUP}

[TOC]

# Introduction

The Enet CPSW fast startup example is dedicated to demonstrate the fast link up functionality of CPSW3G showing Strapped and forced mode configuration of PHY.


On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack

This example does:
    - Demonstrates the strapped PHY functionality(Auto-Negotiation disabled).
    - Target-side application running on a Cortex R5F core.
    - The example enables both the ports of CPSW in MAC mode, creates a packet and sent it from port 1 to port 2.
    - when the packet is received on port 2, the application prints out the profiling nodes and halts in a infinite loop.



# Supported Combinations

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_cpsw_fast_startup

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_cpsw_fast_startup

\endcond

# Modifications needed before running the example:

\cond SOC_AM263X
## EVM Board modification

- On AM263x-cc(PROC110E1), we need to disable Auto-negotiation in the PHY  and force it to strapped mode with external starp resistors connected on PHY.
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

\endcond

\cond SOC_AM243X
## EVM Board modification

- On AM243x-lp(PROC109E3), we need to disable Auto-negotiation in the PHY  and force it to strapped mode with external starp resistors connected on PHY.
- To do that, modify below registers mentioned in below section for RGMII1 and RGMII2
- For other variants of AM243x, refer the schematics of board to modify the strap resistors for that specific EVM.

### Modifications for CPSW RGMII1:
    
- Remove Strapping resistors from R138, R137, R139 and add the same to R265, R266, R264. All resistors are of same value.

### Modifications for CPSW RGMII2:
    
- Remove Strapping resistors from  R75, R74, R76 and add the same to R226, R227, R225. All resistors are of same value.

## SW modifications

- We need to disable Enet Logs and recompiler the enet-cpsw library.
- To do so, remove the ENABLE_ENET_LOG define from mcu_plus_sdk/source/networking/enet/makefile.cpsw.am243x.r5f.ti-arm-clang makefile.
- Recompile the enet-cpsw library.

\endcond

# Media Dependent Interface(MDI configuration)

- With forced strapped mode, we don't have Auto-MDIX option in Phy, it can only be configured in manual MDI or MDIX mode, here in current configuration, we are setting both ports to manual MDIX mode.
- Ports that are connected by MDI-to-MDI and MDIX-to-MDIX connections must use crossover twisted-pair cabling.
- If straight cable is used, we need to put a switch in between the two ports.
- Refer the crossover cable wiring shceme given below:

  \imageStyle{crossover_cable.png,width:30%}
  \image html crossover_cable.png Crossover cable wiring shceme
# Tips to minimize link up time for Ethernet applications

- Auto-Negotiation on @VAR_SOC_NAME phy takes around 2-3 seconds, so to achieve quick link up, we cannot operate in Auto-Neg mode and have to use strapped and forced modes.
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
     Fast Srtartup Profiling logs in Microseconds(us)    
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
