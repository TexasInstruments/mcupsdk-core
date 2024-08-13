#  Datasheet {#DATASHEET_AM65X_IDK}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM65x

## Generic Setup details

SOC Details             | Values
------------------------|------------------------------
Core                    | R5F
Core Operating Speed    | 400 MHz
Cache Status            | Enabled
Device Type             | GP

Optimization Details    | Values
------------------------|------------------------------
Build Profile           | Release
R5F Compiler flags      | -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mthumb -Wall -Werror -g -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -Wno-extra  -Os
R5F Linker flags        | -Wl,--diag_suppress=10063 -Wl,--ram_model -Wl,--reread_libs
Code Placement          | MSRAM
Data Placement          | MSRAM

## Performance Numbers

### SBL OSPI performance

- Software/Application used : sbl_ospi and i2c_led_blink
- Size of sbl_ospi tiimage : 59 KB
- Size of i2c_led_blink appimage : 54 KB
- Cores present in the appimage : r5f0-0
- Boot Media Clock : 133.333 MHz

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
SBL : System Init                       |   0.35
SBL : Drivers_open                      |   0.10
SBL : Board_driversOpen                 |   14.19
SBL : SYSFW init                        |   199.69
SBL : Sciclient Get Version             |   82.31
SBL : CPU Load                          |   54.96
SBL : Total time taken                  |   351.62

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory  directly, this would be slower that mem to mem copy.
- The time taken for Sciclient Get Version can be avoided if the version check is disabled.

### SBL SD performance

- Software/Application used : sbl_sd and i2c_led_blink
- Size of sbl_ospi tiimage : 79 KB
- Size of i2c_led_blink appimage : 54 KB
- Cores present in the appimage : r5f0-0

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
SBL : System Init                       |   0.5
SBL : Drivers_open                      |   17.35
SBL : Board_driversOpen                 |   0.00
SBL : SYSFW init                        |   121.70
SBL : Sciclient Get Version             |   14.88
SBL : CPU Load                          |   23.13
SBL : Total time taken                  |   177.63

- The MMCSD driver and SD card initialization is done as part of Drivers_open, so Board_driversOpen happens instantaneously.
- The time taken for Sciclient Get Version can be avoided if the version check is disabled

### MCSPI performance

### MCSPI driver performance in different modes
- measurement is provided for Master mode in Loopback configuration with TX and RX.
- Transfer time captured with driver API in different operation mode.
- Throughput comparison is added for transfer of 400 Bytes with different word width and corresponding number of words.
- Time
- CPU : R5F
- Transfer Length : 400 Bytes
- SPI Clock speed : 12.5MHz

Number of Words  | Word Width (Bits) | Polled mode Throughput / Transfer time | Interrupt mode (Mbps) Throughput / Transfer time   |  Dma mode (Mbps) Throughput / Transfer time
-----------------|-------------------|----------------------------------------|------------------------------------|--------------------------------
 400 |  08  | 5.40 Mbps / 593.00 us   | 7.42 Mbps / 431.33 us   |  0.90 Mbps / 3548.39 us
 200 |  16  | 10.68 Mbps / 299.65 us  | 12.86 Mbps / 248.78 us  |  0.94 Mbps / 3388.54 us
 100 |  32  | 20.73 Mbps / 154.39 us  | 19.98 Mbps / 160.15 us  |  0.97 Mbps / 3308.77 us

### OSPI NOR Flash Performance

- Flash protocol: FLASH_CFG_PROTO_8D_8D_8D
- PHY : enabled
- DMA : enabled

Data size(MiB) | Write speed(MiBps) | Read speed(MiBps)
---------------|--------------------|-----------------
 1	       | 2.26		    | 122.69
 5	       | 2.26		    | 123.00
 10	       | 2.26		    | 123.03

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
r5f0-0    | r5f0-1    |  3.79

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0 | r5f0-1        | 4     |  2.912
 r5f0-0 | r5f0-1        | 32    |  4.018
 r5f0-0 | r5f0-1        | 64    |  5.270
 r5f0-0 | r5f0-1        | 112   |  7.150

