# Datasheet {#DATASHEET_AM273X_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM273x

## Generic Setup details

SOC Details             | Values
------------------------|------------------------------
Core                    | R5F
Core Operating Speed    | 400 MHz
Cache Status            | Enabled

Optimization Details    | Values
------------------------|------------------------------
Build Profile           | Release
R5F Compiler flags      | -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -Wall -Werror -g -mthumb -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -Os
R5F Linker flags        | -Wl,--diag_suppress=10063 -Wl,--ram_model -Wl,--reread_libs
Code Placement          | MSS L2 RAM
Data Placement          | MSS L2 RAM

## Performance Numbers

### SBL QSPI performance

- Software/Application used        : sbl_qspi and ipc_rpmsg_echo
- Size of sbl_qspi appimage        : 163 KB
- Size of ipc_rpmsg_echo appimage     : 53 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
SBL : System Init                       |   72
SBL : Drivers_open                      |   16
SBL : LoadHsmRtFw                       |   12974
SBL : Board_driversOpen                 |   2531
SBL : CPU Load                          |   3352
SBL : Total time taken                  |   18947

- Please note that the total time taken provided at the end is not including the ROM boot time.

### MCAN performance

- Internal loopback mode of operation
- Memory Mode               : TX Buffer Mode, RX FIFO Mode
- MCAN CLK                  : 80MHz
- CPU                       : R5F

Theoretical Rate Calculation

Frame Type             | Arbitration BitRate(Mbps) | Data BitRate(Mbps) | Arb Phase bits | Data Phase bits | Theoretical Throughput (Msg/Sec) | Actual Throughput (Msg/Sec)
-----------------------|---------------------------|--------------------|----------------|-----------------|----------------------------------|----------------------------
CAN FD STANDARD FORMAT | 1                         | 5                  |  27            | 538             | 7430                             | 6579
CAN FD EXTENDED FORMAT | 1                         | 5                  |  46            | 538             | 6510                             | 5714


### MIBSPI performance

- Internal loopback operation
- Software/Application used : test_mibspi_performance
- MIBSPI Clock              : 40MHz
- CPU                       : R5F

Theoretical Rate Calculation
Data Width 	| Data Length | Transfer Time (micro sec)
------------|-------------|--------------------------
16	        | 1024	      | 204.80

Actual Rate Calculation
Data Width 	| Data Length | Transfer Time (micro sec)
------------|-------------|--------------------------
16	        | 1024	      | 246.90

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0	| r5f0-1	|  1.63
 r5f0-0	| c66ss0	|  3.32

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 0.878
 r5f0-0	| c66ss0	| 4	| 1.299
 r5f0-0	| r5f0-1	| 32	| 1.130
 r5f0-0	| r5f0-1	| 64	| 1.370
 r5f0-0	| r5f0-1	| 112	| 1.729

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed

Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.040001 		| 680			| 276.511993		|
cos  		|0.0000002870	| 64			| 64.106003 		| 495			| 276.656006		|
sincos sin  	|0.0000001790	| 78			| 78.073997 		| 672			| 275.087982		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 73			| 73.033997 		| 562			| 428.747986		|
acos 		|0.0000004770	| 74			| 74.029999 		| 772			| 383.888000		|
atan 		|0.0000005360	| 85			| 85.019997 		| 755			| 372.580017		|
atan2 		|0.0000007150	| 117			| 105.606003 		| 577			| 477.250000		|


### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance
