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

- Software/Application used        : sbl_qspi and hello_world
- Size of sbl_qspi appimage        : 65 KB
- Size of hello_world appimage     : 41 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
SBL : System Init                       |   73
SBL : Drivers_open                      |   16
SBL : LoadHsmRtFw                       |   4
SBL : Board_driversOpen                 |   2700
SBL : CPU Load                          |   1818
SBL : Total time taken                  |   4613

### MCAN performance

- Internal loopback mode of operation
- Memory Mode               : TX Buffer Mode, RX FIFO Mode
- MCAN CLK                  : 80MHz
- CPU                       : R5F

Theoretical Rate Calculation

Frame Type             | Arbitration BitRate(Mbps) | Data BitRate(Mbps) | Arb Phase bits | Data Phase bits | Theoretical Throughput (Msg/Sec) | Actual Throughput (Msg/Sec)
-----------------------|---------------------------|--------------------|----------------|-----------------|----------------------------------|----------------------------
CAN FD STANDARD FORMAT | 1                         | 5                  |  27            | 538             | 7430                             | 6578
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
16	        | 1024	      | 246.80

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0	| r5f0-1	|  1.56
 r5f0-0	| c66ss0	|  3.26

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 0.832
 r5f0-0	| c66ss0	| 4	| 1.247
 r5f0-0	| r5f0-1	| 32	| 1.097
 r5f0-0	| r5f0-1	| 64	| 1.332
 r5f0-0	| r5f0-1	| 112	| 1.691

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 62			| 43.063999 		| 687			| 288.306000		|
cos  		|0.0000002870	| 73			| 53.976002 		| 488			| 288.447998		|
sincos sin  	|0.0000001790	| 88			| 68.973999 		| 643			| 286.645996		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 83			| 63.921997 		| 592			| 444.145996		|
acos 		|0.0000004770	| 84			| 64.926003 		| 728			| 396.635986		|
atan 		|0.0000005360	| 89			| 72.288002 		| 512			| 385.382019		|
atan2 		|0.0000007150	| 513			| 103.234001 		| 594			| 490.760010		|


### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance
