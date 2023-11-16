#  Datasheet {#DATASHEET_AM263PX_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM263Px

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

### SBL OSPI performance

- Software/Application used        : sbl_ospi and hello_world
- Size of sbl_ospi appimage        : 217 KB
- Size of hello_world              : 24.5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   256
SBL : Drivers_open                      |   105
SBL : LoadHsmRtFw                       |   13445
SBL : Board_driversOpen                 |   33941
SBL : CPU Load                          |   78
SBL : Total time taken                  |   47832

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_sd appimage        : 225 KB
- Size of hello_world : 24.5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   377
SBL : Drivers_open                      |   149221
SBL : LoadHsmRtFw                       |   12748
SBL : Board_driversOpen                 |   2734
SBL : File read from SD card            |   9702
SBL : CPU Load                          |   3291
SBL : Total time taken                  |   178076

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
 r5f0-0	| r5f0-1	|  1.84
 r5f0-0	| r5f1-0	|  1.85
 r5f0-0	| r5f1-1	|  1.94

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0 | r5f0-1        | 4     | 1.018
 r5f0-0 | r5f1-0        | 4     | 1.018
 r5f0-0 | r5f1-1        | 4     | 1.020
 r5f0-0 | r5f0-1        | 32    | 1.328
 r5f0-0 | r5f0-1        | 64    | 1.620
 r5f0-0 | r5f0-1        | 112   | 2.045

### DTHE

### AES CBC

- Software/Application used : test_dthe_aes_cbc
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption |  32.00 KB | 99.171315 |
| 128 | Decryption |  32.00 KB | 98.863972 |
| 128 | Encryption |  16.00 KB | 99.078172 |
| 128 | Decryption |  16.00 KB | 98.835736 |
| 128 | Encryption |  8.00 KB | 99.466137 |
| 128 | Decryption |  8.00 KB | 99.625280 |
| 128 | Encryption |  4.00 KB | 101.219371 |
| 128 | Decryption |  4.00 KB | 102.919421 |
| 128 | Encryption |  2.00 KB | 101.095240 |
| 128 | Decryption |  2.00 KB | 101.724486 |
| 128 | Encryption |  1024.00 B | 99.209785 |
| 128 | Decryption |  1024.00 B | 97.964065 |
| 128 | Encryption |  512.00 B | 94.410511 |
| 128 | Decryption |  512.00 B | 93.896498 |
| 256 | Encryption |  32.00 KB | 90.842036 |
| 256 | Decryption |  32.00 KB | 90.807894 |
| 256 | Encryption |  16.00 KB | 90.700519 |
| 256 | Decryption |  16.00 KB | 90.603652 |
| 256 | Encryption |  8.00 KB | 90.847152 |
| 256 | Decryption |  8.00 KB | 91.612294 |
| 256 | Encryption |  4.00 KB | 92.825881 |
| 256 | Decryption |  4.00 KB | 93.868256 |
| 256 | Encryption |  2.00 KB | 93.110748 |
| 256 | Decryption |  2.00 KB | 92.604211 |
| 256 | Encryption |  1024.00 B | 91.212248 |
| 256 | Decryption |  1024.00 B | 91.153889 |
| 256 | Encryption |  512.00 B | 87.135032 |
| 256 | Decryption |  512.00 B | 87.065576 |


### AES ECB

- Software/Application used : test_dthe_aes_ecb
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption |  32.00 KB | 99.041113 |
| 128 | Decryption |  32.00 KB | 98.903510 |
| 128 | Encryption |  16.00 KB | 98.759590 |
| 128 | Decryption |  16.00 KB | 98.768893 |
| 128 | Encryption |  8.00 KB | 99.335726 |
| 128 | Decryption |  8.00 KB | 99.777337 |
| 128 | Encryption |  4.00 KB | 101.902430 |
| 128 | Decryption |  4.00 KB | 102.304090 |
| 128 | Encryption |  2.00 KB | 101.469336 |
| 128 | Decryption |  2.00 KB | 102.041261 |
| 128 | Encryption |  1024.00 B | 99.896348 |
| 128 | Decryption |  1024.00 B | 99.795949 |
| 128 | Encryption |  512.00 B | 95.283513 |
| 128 | Decryption |  512.00 B | 95.062373 |
| 256 | Encryption |  32.00 KB | 90.890384 |
| 256 | Decryption |  32.00 KB | 90.939808 |
| 256 | Encryption |  16.00 KB | 91.174972 |
| 256 | Decryption |  16.00 KB | 90.871086 |
| 256 | Encryption |  8.00 KB | 92.620570 |
| 256 | Decryption |  8.00 KB | 91.292613 |
| 256 | Encryption |  4.00 KB | 93.220014 |
| 256 | Decryption |  4.00 KB | 93.840030 |
| 256 | Encryption |  2.00 KB | 93.199465 |
| 256 | Decryption |  2.00 KB | 93.264455 |
| 256 | Encryption |  1024.00 B | 91.633110 |
| 256 | Decryption |  1024.00 B | 90.647044 |
| 256 | Encryption |  512.00 B | 87.582188 |
| 256 | Decryption |  512.00 B | 87.690002 |

### SHA

- Software/Application used : test_dthe_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 284.580312 |
| 512 |  16.00 KB | 283.258055 |
| 512 |  8.00 KB | 279.966679 |
| 512 |  4.00 KB | 273.751044 |
| 512 |  2.00 KB | 262.616710 |
| 512 |  1024.00 B | 242.348939 |
| 512 |  512.00 B | 208.873024 |
| 256 |  32.00 KB | 275.411972 |
| 256 |  16.00 KB | 274.090222 |
| 256 |  8.00 KB | 271.792639 |
| 256 |  4.00 KB | 267.215755 |
| 256 |  2.00 KB | 258.422713 |
| 256 |  1024.00 B | 243.031966 |
| 256 |  512.00 B | 216.834304 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 282.354208 |
| 512 |  16.00 KB | 279.244961 |
| 512 |  8.00 KB | 272.252744 |
| 512 |  4.00 KB | 259.522820 |
| 512 |  2.00 KB | 236.805781 |
| 512 |  1024.00 B | 201.984836 |
| 512 |  512.00 B | 156.530047 |
| 256 |  32.00 KB | 273.671738 |
| 256 |  16.00 KB | 271.290561 |
| 256 |  8.00 KB | 266.374019 |
| 256 |  4.00 KB | 255.028699 |
| 256 |  2.00 KB | 240.032231 |
| 256 |  1024.00 B | 212.049440 |
| 256 |  512.00 B | 172.336173 |

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 514			| 275.335999		|
cos  		|0.0000002870	| 64			| 64.036003 		| 508			| 276.654022		|
sincos sin  	|0.0000001790	| 79			| 79.033997 		| 479			| 274.308014		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 73			| 73.073997 		| 574			| 428.777985		|
acos 		|0.0000004770	| 74			| 74.070000 		| 825			| 383.791992		|
atan 		|0.0000005360	| 85			| 85.019997 		| 700			| 372.238007		|
atan2 		|0.0000007150	| 119			| 106.589996 		| 715			| 477.832001		|


### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

