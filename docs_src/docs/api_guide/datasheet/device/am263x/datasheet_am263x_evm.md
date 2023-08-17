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

- Software/Application used        : sbl_qspi and hello_world
- Size of sbl_qspi appimage        : 185 KB
- Size of hello_world              : 24.4 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   290
SBL : Drivers_open                      |   75
SBL : LoadHsmRtFw                       |   14517
SBL : Board_driversOpen                 |   2973
SBL : CPU Load                          |   130
SBL : Total time taken                  |   17988

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_qspi appimage        : 227 KB
- Size of hello_world : 24.4 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   375
SBL : Drivers_open                      |   80988
SBL : LoadHsmRtFw                       |   43
SBL : Board_driversOpen                 |   389
SBL : File read from SD card            |   7250
SBL : CPU Load                          |   2284
SBL : Total time taken                  |   91332

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
 r5f0-0	| r5f0-1	|  1.70
 r5f0-0	| r5f1-0	|  1.74
 r5f0-0	| r5f1-1	|  1.79

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 0.943
 r5f0-0	| r5f1-0	| 4	| 0.939
 r5f0-0	| r5f1-1	| 4	| 0.942
 r5f0-0	| r5f0-1	| 32	| 1.219
 r5f0-0	| r5f0-1	| 64	| 1.478
 r5f0-0	| r5f0-1	| 112	| 1.856

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
 400		| 08			| 20.76 Mbps / 154.14 us 	| 23.20 Mbps / 137.92 us 	|  0.91 Mbps / 3524.49 us
 200		| 16			| 32.24 Mbps / 99.25 us 	| 30.39 Mbps / 105.29 us 	|  0.95 Mbps / 3368.58 us
 100		| 32			| 38.31 Mbps / 83.52 us 	| 35.95 Mbps / 89.01 us 	|  0.97 Mbps / 3290.81 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### DTHE

### AES CBC

- Software/Application used : test_dthe_aes_cbc
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption |  32.00 KB | 107.892286 |
| 128 | Decryption |  32.00 KB | 107.712963 |
| 128 | Encryption |  16.00 KB | 107.546036 |
| 128 | Decryption |  16.00 KB | 107.621978 |
| 128 | Encryption |  8.00 KB | 107.275592 |
| 128 | Decryption |  8.00 KB | 107.343240 |
| 128 | Encryption |  4.00 KB | 110.840317 |
| 128 | Decryption |  4.00 KB | 112.583532 |
| 128 | Encryption |  2.00 KB | 111.634245 |
| 128 | Decryption |  2.00 KB | 111.767515 |
| 128 | Encryption |  1024.00 B | 108.791501 |
| 128 | Decryption |  1024.00 B | 108.331129 |
| 128 | Encryption |  512.00 B | 103.617506 |
| 128 | Decryption |  512.00 B | 102.875801 |
| 256 | Encryption |  32.00 KB | 98.877210 |
| 256 | Decryption |  32.00 KB | 98.973245 |
| 256 | Encryption |  16.00 KB | 98.914239 |
| 256 | Decryption |  16.00 KB | 99.024091 |
| 256 | Encryption |  8.00 KB | 100.135605 |
| 256 | Decryption |  8.00 KB | 99.666567 |
| 256 | Encryption |  4.00 KB | 102.334442 |
| 256 | Decryption |  4.00 KB | 102.804794 |
| 256 | Encryption |  2.00 KB | 102.443218 |
| 256 | Decryption |  2.00 KB | 102.340834 |
| 256 | Encryption |  1024.00 B | 99.987794 |
| 256 | Decryption |  1024.00 B | 99.686654 |
| 256 | Encryption |  512.00 B | 94.985217 |
| 256 | Decryption |  512.00 B | 94.524895 |


### AES ECB

- Software/Application used : test_dthe_aes_ecb
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption |  32.00 KB | 107.910052 |
| 128 | Decryption |  32.00 KB | 107.698691 |
| 128 | Encryption |  16.00 KB | 107.535889 |
| 128 | Decryption |  16.00 KB | 107.573177 |
| 128 | Encryption |  8.00 KB | 107.892841 |
| 128 | Decryption |  8.00 KB | 109.188451 |
| 128 | Encryption |  4.00 KB | 111.208966 |
| 128 | Decryption |  4.00 KB | 112.725865 |
| 128 | Encryption |  2.00 KB | 112.144287 |
| 128 | Decryption |  2.00 KB | 111.782766 |
| 128 | Encryption |  1024.00 B | 109.077594 |
| 128 | Decryption |  1024.00 B | 109.204826 |
| 128 | Encryption |  512.00 B | 104.263714 |
| 128 | Decryption |  512.00 B | 103.906646 |
| 256 | Encryption |  32.00 KB | 98.994642 |
| 256 | Decryption |  32.00 KB | 99.029328 |
| 256 | Encryption |  16.00 KB | 99.247534 |
| 256 | Decryption |  16.00 KB | 99.087160 |
| 256 | Encryption |  8.00 KB | 99.689687 |
| 256 | Decryption |  8.00 KB | 99.677178 |
| 256 | Encryption |  4.00 KB | 102.178879 |
| 256 | Decryption |  4.00 KB | 102.921038 |
| 256 | Encryption |  2.00 KB | 102.683985 |
| 256 | Decryption |  2.00 KB | 102.004732 |
| 256 | Encryption |  1024.00 B | 100.543095 |
| 256 | Decryption |  1024.00 B | 100.073296 |
| 256 | Encryption |  512.00 B | 96.138951 |
| 256 | Decryption |  512.00 B | 95.422248 |

### SHA

- Software/Application used : test_dthe_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 310.771137 |
| 512 |  16.00 KB | 309.403898 |
| 512 |  8.00 KB | 305.757208 |
| 512 |  4.00 KB | 299.019026 |
| 512 |  2.00 KB | 285.311276 |
| 512 |  1024.00 B | 262.964449 |
| 512 |  512.00 B | 225.519615 |
| 256 |  32.00 KB | 299.906760 |
| 256 |  16.00 KB | 298.918441 |
| 256 |  8.00 KB | 296.325100 |
| 256 |  4.00 KB | 291.277584 |
| 256 |  2.00 KB | 281.234176 |
| 256 |  1024.00 B | 262.648285 |
| 256 |  512.00 B | 232.727273 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 308.598033 |
| 512 |  16.00 KB | 304.153710 |
| 512 |  8.00 KB | 297.033562 |
| 512 |  4.00 KB | 282.811893 |
| 512 |  2.00 KB | 257.782323 |
| 512 |  1024.00 B | 218.818030 |
| 512 |  512.00 B | 167.903259 |
| 256 |  32.00 KB | 298.540574 |
| 256 |  16.00 KB | 295.059345 |
| 256 |  8.00 KB | 289.767540 |
| 256 |  4.00 KB | 279.548702 |
| 256 |  2.00 KB | 260.135752 |
| 256 |  1024.00 B | 228.842796 |
| 256 |  512.00 B | 184.069206 |

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 64			| 43.299999 		| 533			| 287.783997		|
cos  		|0.0000002870	| 75			| 54.026001 		| 533			| 288.701996		|
sincos sin  	|0.0000001790	| 90			| 68.720001 		| 506			| 286.729980		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 85			| 63.760002 		| 617			| 444.753998		|
acos 		|0.0000004770	| 86			| 64.748001 		| 835			| 396.686005		|
atan 		|0.0000005360	| 72			| 72.281998 		| 524			| 385.391998		|
atan2 		|0.0000007150	| 112			| 102.442001 		| 595			| 490.354004		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

