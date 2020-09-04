# Enet ICSSG Time Aware Shaper (EST) Example {#EXAMPLES_ENET_ICSSG_TAS}

[TOC]

# Introduction

This example demonstrates Time-Aware Shaper functionality on ICSSG Switch and Dual-MAC peripherals.


\cond SOC_AM64X || SOC_AM243X

On @VAR_SOC_NAME, we can do ethernet based communication using ICSSG HW Mechanism

  - PRU firmware based Ethernet Switch and Dual MAC implementation
  - This HW can be used with industrial communication protocols as well (see \ref EXAMPLES_INDUSTRIAL_COMMS)

\endcond

This example do below:
- Target-side application running on a Cortex R5F core.
    - One TX channel and one RX flow are opened for each available MAC port.
    - The application has a menu to start the TAS test and to Print or Reset the Statistics. This menu along with application logs are implemented via UART.
- Host-side functionality
	- Wireshark can be used to receive and verify packet contents.

Below shows the configuration used for the Time Aware Shaper which is used to configure each MAC port in AM64xx device.

\code

                    |<------------- Cycle time = 250 us --------------->|

                    +------------+------------+------------+------------+
                    |            |            |            |            |
                    +------------+------------+------------+------------+

                    |<---------->|<---------->|<---------->|<---------->|
      Window index       0             1            2            3
      Duration         62.5 us       62.5 us      62.5 us      62.5 us
      Gate mask      0b00000011    0b00001100   0b00110000   0b11000000

\endcode

- The default cycle time is set to be 250us and each window is equally divided in the cycle as shown above.
- The startTasTest test case configures each of the MAC ports with the above-mentioned configuration and transmits 2 packets each with traffic class values varying from 0 to 7.
- As per the TAS configuration the packets with traffic class of 0 and 1 will only be transmitted within the time duration of window index 0, traffic class 2 and 3 will be transmitted in window index 1 and so on.
- The startTasTest test case verifies if all the packets transmitted have their tx timestamp within their corresponding windows. If this condition is satisfied then test is marked as passed.
- The test is repeated for all ports with link up.


# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Icssg Instance | ICSSG1
 Example folder | examples/networking/enet_icssg_tas

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Icssg Instance | ICSSG1
 Example folder | examples/networking/enet_icssg_tas

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

\cond SOC_AM64X

### AM64X-EVM

#### For ICSSG based example

- Connect two ethernet cables to the EVM from two host PC's as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

\endcond

\cond SOC_AM243X

### AM243X-EVM

#### For ICSSG based example

- Connect two ethernet cables to the EVM from two host PC's as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as both CPSW/ICSS ports.

#### For ICSSG based examples

- Connect two ethernet cables to the AM243X-LP from two host PC's as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for ICSSG based ethernet

\endcond


## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Wait until the interface MAC ID is assigned and printed on the console and select the test case

## Sample output for ICSSG TAS example

\note "EnetMod_ioctl: icssgx-x.tas: Failed to do IOCTL cmd 0x01000b0x: 1" This is not a ioctl failure but
      ioctl is returned with status as inprogress as it is Asynchronous ioctl.

\code

==========================
   ENET ICSSG TAS TEST
==========================

Init all peripheral clocks
----------------------------------------------
Enabling clocks!

Init all configs
----------------------------------------------

Init  configs EnetType:2, InstId :1
----------------------------------------------
icssg1: Open port 1
EnetPhy_bindDriver:
icssg1: Open port 2
EnetPhy_bindDriver:
PHY 3 is alive
PHY 15 is alive

Open all peripherals
----------------------------------------------
icssg1: Register async IOCTL callback
icssg1: Register TX timestamp callback

Attach core id 1 on all peripherals
----------------------------------------------
icssg1: Attach core

Create RX tasks
----------------------------------------------
icssg1: Create RX task
icssg1: Waiting for link up...

Enet Multiport Menu:
 'T'  -  Start the TAS test with default config
 's'  -  Print statistics
 'r'  -  Reset statistics
 'x'  -  Stop the test

Icssg_handleLinkUp:
Icssg_handleLinkUp:
icssg1: Port 1 link is up
icssg1: Set port state to 'Forward'
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
icssg1: Port 2 link is up
icssg1: Set port state to 'Forward'
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
icssg1: Open DMA
initQs() txFreePktInfoQ initialized with 8 pkts
icssg1: Set MAC addr: 70:ff:76:1d:92:c1
icssg1: MAC port addr: 70:ff:76:1d:92:c1

Invalid option, try again...

Enet Multiport Menu:
 'T'  -  Start the TAS test with default config
 's'  -  Print statistics
 'r'  -  Reset statistics
 'x'  -  Stop the test

T
Success: IOCTL command sent for making MAC entry in FDB
icssg1: Async IOCTL completed
Success: IOCTL command sent for making MAC entry in FDB
icssg1: Async IOCTL completed
EnetMod_ioctl:
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
EnetMod_ioctl:
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
EnetMod_ioctl:
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
Transmitting 16 packets
Packet 0 is within window timestamp : 1100 window_start : 0 window_end : 62500
Packet 1 is within window timestamp : 3304 window_start : 0 window_end : 62500
Packet 2 is within window timestamp : 64640 window_start : 62500 window_end : 125000
Packet 3 is within window timestamp : 63448 window_start : 62500 window_end : 125000
Packet 4 is within window timestamp : 128280 window_start : 125000 window_end : 187500
Packet 5 is within window timestamp : 125844 window_start : 125000 window_end : 187500
Packet 6 is within window timestamp : 190516 window_start : 187500 window_end : 250000
Packet 7 is within window timestamp : 188096 window_start : 187500 window_end : 250000
Packet 8 is within window timestamp : 57388 window_start : 0 window_end : 62500
Packet 9 is within window timestamp : 1208 window_start : 0 window_end : 62500
Packet 10 is within window timestamp : 70952 window_start : 62500 window_end : 125000
Packet 11 is within window timestamp : 75696 window_start : 62500 window_end : 125000
Packet 12 is within window timestamp : 129472 window_start : 125000 window_end : 187500
Packet 13 is within window timestamp : 127040 window_start : 125000 window_end : 187500
Packet 14 is within window timestamp : 191720 window_start : 187500 window_end : 250000
Packet 15 is within window timestamp : 189288 window_start : 187500 window_end : 250000
Enet TAS Test PASSED
EnetMod_ioctl:
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
EnetMod_ioctl:
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
EnetMod_ioctl:
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
Transmitting 16 packets
Packet 0 is within window timestamp : 30448 window_start : 0 window_end : 62500
Packet 1 is within window timestamp : 35912 window_start : 0 window_end : 62500
Packet 2 is within window timestamp : 64304 window_start : 62500 window_end : 125000
Packet 3 is within window timestamp : 63108 window_start : 62500 window_end : 125000
Packet 4 is within window timestamp : 128216 window_start : 125000 window_end : 187500
Packet 5 is within window timestamp : 125788 window_start : 125000 window_end : 187500
Packet 6 is within window timestamp : 190784 window_start : 187500 window_end : 250000
Packet 7 is within window timestamp : 188356 window_start : 187500 window_end : 250000
Packet 8 is within window timestamp : 2336 window_start : 0 window_end : 62500
Packet 9 is within window timestamp : 1144 window_start : 0 window_end : 62500
Packet 10 is within window timestamp : 100280 window_start : 62500 window_end : 125000
Packet 11 is within window timestamp : 105036 window_start : 62500 window_end : 125000
Packet 12 is within window timestamp : 129416 window_start : 125000 window_end : 187500
Packet 13 is within window timestamp : 126984 window_start : 125000 window_end : 187500
Packet 14 is within window timestamp : 191984 window_start : 187500 window_end : 250000
Packet 15 is within window timestamp : 189552 window_start : 187500 window_end : 250000
Enet TAS Test PASSED

\endcode

# See Also

\ref ENET_MOD_TAS   \ref NETWORKING