#  Datasheet {#DATASHEET_AM263X_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM263x

## Generic Setup details

SOC Details             | Values
------------------------|------------------------------
Core                    | R5F
Core Operating Speed    | 400 MHz
Cache Status            | Enabled
Device Type             | HSFS

Optimization Details    | Values
------------------------|------------------------------
Build Profile           | Release
R5F Compiler flags      | -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -Wall -Werror -g -mthumb -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -Os
R5F Linker flags        | -Wl,--diag_suppress=10063 -Wl,--ram_model -Wl,--reread_libs
Code Placement          | MSRAM
Data Placement          | MSRAM

## Performance Numbers

### SBL QSPI performance

- Software/Application used        : sbl_qspi and ipc_notify_echo
- Size of sbl_qspi appimage        : 63 KB
- Size of ipc_notify_echo appimage : 99 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   788
SBL : Drivers_open                      |   33
SBL : LoadHsmRtFw                       |   177
SBL : Board_driversOpen                 |   239
SBL : CPU Load                          |   5927
SBL : Total time taken                  |   7166

- Please note that the total time taken provided at the end is not including the ROM boot time.

### MCAN performance

- Internal loopback mode of operation
- Software/Application used : test_mcan
- Memory Mode               : TX Buffer Mode, RX FIFO Mode
- MCAN CLK                  : 80MHz
- CPU                       : R5F

#### Theoretical Rate Calculation

Frame Type              | Arbitration BitRate(Mbps) | Data BitRate(Mbps)  | Arb Phase bits  | Data Phase bits | Throughput (Msg/Sec)
------------------------|---------------------------|---------------------|-----------------|-----------------|---------------------
CAN FD STANDARD FORMAT  | 1                         | 5                   |  27             | 538             | 7430
CAN FD EXTENDED FORMAT  | 1                         | 5                   |  46             | 538             | 6510

#### Actual Numbers

Frame Type             | Arbitration BitRate(Mbps) | Data BitRate(Mbps) | Theoretical Throughput (Msg/Sec) | Actual Throughput (Msg/Sec)
-----------------------|---------------------------|--------------------|----------------------------------|----------------------------
CAN FD STANDARD FORMAT | 1                         | 5                  |  7430                            | 6578
CAN FD EXTENDED FORMAT | 1                         | 5                  |  6510                            | 5713

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0 | r5f0-1        |  1.81
 r5f0-0 | r5f1-0        |  1.78
 r5f0-0 | r5f1-1        |  1.83

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0 | r5f0-1        | 4     |  1.06
 r5f0-0 | r5f1-0        | 4     |  1.06
 r5f0-0 | r5f1-1        | 4     |  1.06
 r5f0-0 | r5f0-1        | 32    |  1.30
 r5f0-0 | r5f0-1        | 64    |  1.58
 r5f0-0 | r5f0-1        | 112   |  1.98

### MCSPI performance

### MCSPI driver performance in different modes
- measurement is provided for Master mode in Loopback configuration with TX and RX.
- Transfer time captured with driver API in different operation mode.
- Throughput comparison is added for transfer of 400 Bytes with different word width and corresponding number of words.
- Time
- CPU                       : R5F
- Transfer Length           : 400 Bytes
- SPI Clock speed           : 50MHz

Number of Words | Word Width (Bits)     | Polled mode Throughput / Transfer time  | Interrupt mode (Mbps) Throughput / Transfer time | Dma mode (Mbps) Throughput / Transfer time
----------------|-----------------------|-------------------------------|-------------------------------|-------------------------------
 400            | 08                    | 20.70 Mbps / 154.62 us        | 22.97 Mbps / 139.31 us        |  0.91 Mbps / 3524.02 us
 200            | 16                    | 32.08 Mbps / 99.76 us         | 29.98 Mbps / 106.74 us        |  0.95 Mbps / 3368.14 us
 100            | 32                    | 38.10 Mbps / 83.99 us         | 35.26 Mbps / 90.75 us         |  0.97 Mbps / 3290.38 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### MATHLIB performance

- Calculated for the 500 samples taken between 0 and 2 * Pi
- trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed

Function        | Err           | Max Cycles Mathlib (mcusdk)   | avg cycles Mathlib (mcusdk)   | max cycles mathlib (clang)    | avg cycles mathlib (clang)    |
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin             |0.0000007150   | 549                   | 44.771999             | 549                   | 288.019989            |
cos             |0.0000002870   | 75                    | 53.669998             | 533                   | 288.928009            |
sincos sin      |0.0000001790   | 90                    | 68.585999             | 775                   | 287.963989            |
asin            |0.0000003430   | 93                    | 64.907997             | 617                   | 444.674011            |
acos            |0.0000004770   | 86                    | 64.584000             | 543                   | 396.074005            |
atan            |0.0000005360   | 93                    | 72.323997             | 853                   | 386.246002            |
atan2           |0.0000007150   | 112                   | 102.505997            | 826                   | 490.612000            |

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

