# Enet CPSW TimeSync PTP Demo {#EXAMPLES_ENET_LAYER2_MULTI_CHANNEL_PTP}

[TOC]

# Terms and Abbreviations

Acronym | Description
--------|------------
PTP |  Precision Time Protocol
P2P | Peer to Peer
E2E | End to End
TC  | Transparent Clock
OC  | Ordinary Clock
BC  | Boundary Clock
Broadcaster | PTP broadcaster
Listener | PTP Listener
BMCA | Best Master Clock Algorithm
RCF | Synchronization/(Frequency Compensation) Factor
IEC 61558 | PTP profile for network control systems


# Introduction

This is extended documentation for ENET CPSW Layer2 Multichannel example that gives details on TimeSync (PTP) demo. The demo is setup with EVM as PTP listener
The broadcaster capability added to the TimeSync driver is currently limitted to sending Sync and Announce frames. The BMCA/Management part of PTP broadcaster is not implemented.
*** note
Please note that the stack code provided here is only a Reference stack. This stack is not tested for any performance benchmarking.
***

The supplied  demo TimeSync (PTP) does the task of

- Running BMCA
- Calculating RCF and Sync packet interval.
- Performing clock synchronization
- Calculating peer and line delay
- Sending Delay Request and Delay Response frames (includes Pdelay Req and Pdelay Res)

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V0
 PTP Stack      | examples/networking/enet_layer2_multi_channel/ptp_stack

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V0
 PTP Stack      | examples/networking/enet_layer2_multi_channel/ptp_stack

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V1
 PTP Stack      | examples/networking/enet_layer2_multi_channel/ptp_stack

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V1
 PTP Stack      | examples/networking/enet_layer2_multi_channel/ptp_stack

\endcond


# Steps to Run the Demo

## Build the example

- When using CCS projects to build, import the CCS project \ref EXAMPLES_ENET_LAYER2_MULTI_CHANNEL for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

- \note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
- Connect EVM and PC usaing Cat5/6 LAN cable.


## PC setup

Need a linux machine for 'ptp4l' tool to run PTP broadcaster on the Linux PC. Below steps have been tried with a Linux Ubuntu 18.04 host PC running bash shell and with HW timestamp supported NIC.

 - Install 'ptp4l', if not installed by doing below
    \code
        $ sudo apt install linuxptp
    \endcode
 - create ptp_config.cfg file in $HOME directory and copy below content to it.
    \code
    [global]
        tx_timestamp_timeout    400
        logMinPdelayReqInterval  3
        logSyncInterval          -4
        logAnnounceInterval      0
    \endcode
 - Notedown the network interface available on your PC.
    \code
    $ ifconfig -s -a
    \endcode
 - Invoke ptp4l as broadcaster as per below cmd
    \code
    $ sudo ptp4l -i eno1 -m -2 -P -l 6 -q -f $HOME/ptp_config.cfg
    \endcode
    - replace `eno1` with correct network interface connected to PC

 - Wait until PC assumes as PTP broadcaster clock role
    \code
    $ sudo ptp4l -i eno1 -m -2 -P -l 6 -q -f $HOME/ptp_config.cfg
    ptp4l[331856.581]: selected /dev/ptp0 as PTP clock
    ptp4l[331856.618]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
    ptp4l[331856.618]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
    ptp4l[331860.608]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
    ptp4l[331860.608]: selected local clock c01803.fffe.bdb1ff as best broadcaster
    ptp4l[331860.608]: assuming the grand broadcaster role
    \endcode


## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example \ref EXAMPLES_ENET_LAYER2_MULTI_CHANNEL executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the bellow section.


## Sample UART Output

\code


==========================
      L2 Multi-channel Test
==========================

Init all peripheral clocks
----------------------------------------------
Enabling clocks!

Create RX tasks
----------------------------------------------
cpsw-3g: Create RX task

Open all peripherals
----------------------------------------------
cpsw-3g: Open enet

Init all configs
----------------------------------------------
cpsw-3g: init config
Mdio_open: MDIO Manual_Mode enabled
cpsw-3g: Open port 1
EnetPhy_bindDriver: PHY 3: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
PHY 3 is alive
PHY 15 is alive

Attach core id 1 on all peripherals
----------------------------------------------
cpsw-3g: Attach core
cpsw-3g: Open DMA
initQs() freePktInfoQ initialized with 16 pkts
initQs() freePktInfoQ initialized with 16 pkts
cpsw-3g: Waiting for link up...
Cpsw_handleLinkUp: Port 1: Link up: 1-Gbps Full-Duplex
MAC Port 1: link up
cpsw-3g: MAC port addr: f4:84:4c:fb:c0:42
TimeSync PTP enabled

Enet Multi-channel Menu:
 'c'  -  GetCurrentTime
 't'  -  Toggle Printing timestamps
 's'  -  Print statistics
 'r'  -  Reset statistics
 'm'  -  Show allocated MAC addresses
 'p'  -  Enable Policer for rate limiting
 'x'  -  Stop the test

Doing first adjustment
CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP: Setting PPM to 1024, as 775 is less than min value (1024)
CpswCpts_ioctl_handler_ENET_TIMESYNC_IOCTL_ADJUST_TIMESTAMP: Setting PPM to 1024, as 885 is less than min value (1024)
Avg=684
248ns<---->1354ns
Avg=91
-166ns<---->176ns
Avg=64
-155ns<---->-20ns
Avg=62
-6ns<---->167ns
Avg=102
-83ns<---->228ns
Avg=62
-107ns<---->105ns
Avg=70
-142ns<---->62ns
Avg=48
-95ns<---->56ns
Avg=91
-75ns<---->204ns
Avg=78
-61ns<---->204ns
Avg=79
-22ns<---->184ns
Avg=193
134ns<---->243ns
Avg=146
-232ns<---->165ns
Avg=206
-297ns<---->-155ns
Avg=94
-99ns<---->223ns
Avg=87
-70ns<---->230ns
Avg=89
28ns<---->169ns
Avg=173
103ns<---->230ns
Avg=192
147ns<---->236ns
Avg=167
117ns<---->193ns
Avg=184
117ns<---->242ns
Avg=172
96ns<---->217ns
Avg=171
139ns<---->219ns
Avg=175
109ns<---->236ns
Avg=181
141ns<---->227ns
Avg=167
108ns<---->247ns
Avg=32
-82ns<---->50ns
Avg=69
-108ns<---->113ns
Avg=61
-139ns<---->96ns
Avg=105
-43ns<---->173ns
Avg=159
121ns<---->235ns
Avg=67
-44ns<---->238ns
Avg=51
-113ns<---->33ns
Avg=78
-7ns<---->145ns
Avg=89
-35ns<---->243ns
Avg=106
-50ns<---->173ns
Avg=161
94ns<---->237ns
Avg=183
138ns<---->238ns
Avg=176
123ns<---->240ns
Avg=194
103ns<---->246ns
Avg=170
99ns<---->243ns
Avg=183
119ns<---->226ns
Avg=176
98ns<---->248ns
Avg=176
122ns<---->251ns
Avg=178
117ns<---->233ns
Avg=81
-95ns<---->154ns
Avg=77
-10ns<---->178ns
Avg=167
75ns<---->214ns
Avg=178
102ns<---->256ns
Avg=102
-78ns<---->245ns
Avg=84
-18ns<---->180ns
Avg=82
-97ns<---->210ns
Avg=94
-54ns<---->224ns
Avg=181
118ns<---->235ns
Avg=79
-92ns<---->193ns
Avg=62
-156ns<---->27ns
Avg=73
-30ns<---->166ns
Avg=186
133ns<---->242ns
Avg=181
126ns<---->257ns
Avg=184
153ns<---->239ns
Avg=75
-69ns<---->199ns
Avg=82
-2ns<---->142ns
Avg=147
-250ns<---->206ns
Avg=55
-167ns<---->55ns
Avg=180
100ns<---->271ns
Avg=71
-32ns<---->178ns
Avg=73
3ns<---->147ns
Avg=179
117ns<---->213ns
Avg=76
-66ns<---->159ns
Avg=95
-56ns<---->198ns
Avg=163
120ns<---->228ns
Avg=110
-104ns<---->231ns
Avg=72
-126ns<---->38ns
Avg=44
-85ns<---->17ns
Avg=136
-318ns<---->18ns
Avg=269
-390ns<---->-71ns
Avg=135
-225ns<---->139ns
Avg=252
-342ns<---->-109ns
Avg=442
-595ns<---->-220ns
Avg=221
-276ns<---->-177ns
Avg=44
-83ns<---->107ns
Avg=244
-327ns<---->-175ns
Avg=199
-398ns<---->-9ns
Avg=347
-439ns<---->-208ns
Avg=91
-133ns<---->-35ns
Avg=27
-57ns<---->18ns
Avg=288
-482ns<---->33ns
Avg=258
-324ns<---->-169ns
Avg=46
-119ns<---->64ns
Avg=168
-284ns<---->-6ns
Avg=207
-289ns<---->-90ns
Avg=63
-122ns<---->89ns
Avg=153
-290ns<---->71ns
Avg=151
-197ns<---->-88ns
Avg=57
-99ns<---->25ns
Avg=81
-65ns<---->159ns
Avg=84
-49ns<---->200ns
Avg=78
-118ns<---->-10ns
Avg=105
-49ns<---->189ns
Avg=84
-92ns<---->164ns
Avg=67
-150ns<---->38ns
Avg=66
-126ns<---->62ns
Avg=70
-143ns<---->34ns
Avg=72
41ns<---->153ns
Avg=80
-61ns<---->203ns
Avg=83
-166ns<---->47ns
Avg=374
-490ns<---->-276ns
Avg=322
-461ns<---->-216ns
Avg=114
-189ns<---->-30ns


\endcode

# See Also

\ref NETWORKING
\ref EXAMPLES_ENET_LAYER2_MULTI_CHANNEL