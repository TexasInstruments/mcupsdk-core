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

- Software/Application used        : sbl_qspi and ipc_rpmsg_echo
- Size of sbl_qspi appimage        : 185 KB
- Size of ipc_rpmsg_echo              : 74 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   324
SBL : Drivers_open                      |   79
SBL : LoadHsmRtFw                       |   15073
SBL : Board_driversOpen                 |   3273
SBL : CPU Load                          |   5897
SBL : Total time taken                  |   24651

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_qspi appimage        : 227 KB
- Size of hello_world : 24.4 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   530
SBL : Drivers_open                      |   137487
SBL : LoadHsmRtFw                       |   15048
SBL : Board_driversOpen                 |   3056
SBL : File read from SD card            |   10727
SBL : CPU Load                          |   44
SBL : Total time taken                  |   166895

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
CAN FD EXTENDED FORMAT | 1                         | 5                  |  6510                            | 5714

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0	| r5f0-1	|  1.80
 r5f0-0	| r5f1-0	|  1.79
 r5f0-0	| r5f1-1	|  1.85

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 0.964
 r5f0-0	| r5f1-0	| 4	| 0.972
 r5f0-0	| r5f1-1	| 4	| 0.970
 r5f0-0	| r5f0-1	| 32	| 1.245
 r5f0-0	| r5f0-1	| 64	| 1.502
 r5f0-0	| r5f0-1	| 112	| 1.886

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
 400		| 08			| 13.07 Mbps / 244.88 us 	| 23.12 Mbps / 138.43 us 	|  0.91 Mbps / 3525.14 us
 200		| 16			| 27.54 Mbps / 116.19 us 	| 30.22 Mbps / 105.89 us 	|  0.95 Mbps / 3369.22 us
 100		| 32			| 37.73 Mbps / 84.81 us 	| 35.63 Mbps / 89.82 us 	|  0.97 Mbps / 3291.46 us

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
| 128 | Encryption |  32.00 KB | 107.935822 |
| 128 | Decryption |  32.00 KB | 107.633136 |
| 128 | Encryption |  16.00 KB | 107.951267 |
| 128 | Decryption |  16.00 KB | 107.456544 |
| 128 | Encryption |  8.00 KB | 108.609853 |
| 128 | Decryption |  8.00 KB | 108.578363 |
| 128 | Encryption |  4.00 KB | 111.253332 |
| 128 | Decryption |  4.00 KB | 112.667727 |
| 128 | Encryption |  2.00 KB | 111.347843 |
| 128 | Decryption |  2.00 KB | 111.626639 |
| 128 | Encryption |  1024.00 B | 108.521278 |
| 128 | Decryption |  1024.00 B | 108.694066 |
| 128 | Encryption |  512.00 B | 103.024587 |
| 128 | Decryption |  512.00 B | 102.470448 |
| 256 | Encryption |  32.00 KB | 98.956712 |
| 256 | Decryption |  32.00 KB | 98.984083 |
| 256 | Encryption |  16.00 KB | 99.144310 |
| 256 | Decryption |  16.00 KB | 98.669633 |
| 256 | Encryption |  8.00 KB | 99.559823 |
| 256 | Decryption |  8.00 KB | 99.982074 |
| 256 | Encryption |  4.00 KB | 102.107239 |
| 256 | Decryption |  4.00 KB | 102.939629 |
| 256 | Encryption |  2.00 KB | 101.645599 |
| 256 | Decryption |  2.00 KB | 102.123946 |
| 256 | Encryption |  1024.00 B | 99.741272 |
| 256 | Decryption |  1024.00 B | 99.875034 |
| 256 | Encryption |  512.00 B | 92.727376 |
| 256 | Decryption |  512.00 B | 94.770939 |

### AES CMAC
- Software/Application used : test_athe_aes_cmac
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Size | Performance (Mbps) |
|------------|------|--------------------|
| 256 |  32.00 KB | 23.777482 |
| 256 |  16.00 KB | 23.753631 |
| 256 |  8.00 KB | 23.749940 |
| 256 |  4.00 KB | 23.743250 |
| 256 |  2.00 KB | 23.639748 |
| 256 |  1024.00 B | 23.368158 |
| 256 |  512.00 B | 22.834843 |
| 128 |  32.00 KB | 24.428380 |
| 128 |  16.00 KB | 24.411740 |
| 128 |  8.00 KB | 24.426866 |
| 128 |  4.00 KB | 24.412967 |
| 128 |  2.00 KB | 24.283295 |
| 128 |  1024.00 B | 24.049202 |
| 128 |  512.00 B | 23.398695 |


### AES ECB

- Software/Application used : test_dthe_aes_ecb
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption |  32.00 KB | 107.720598 |
| 128 | Decryption |  32.00 KB | 107.762001 |
| 128 | Encryption |  16.00 KB | 107.603203 |
| 128 | Decryption |  16.00 KB | 107.666843 |
| 128 | Encryption |  8.00 KB | 107.352032 |
| 128 | Decryption |  8.00 KB | 107.608062 |
| 128 | Encryption |  4.00 KB | 111.342168 |
| 128 | Decryption |  4.00 KB | 112.831637 |
| 128 | Encryption |  2.00 KB | 112.111674 |
| 128 | Decryption |  2.00 KB | 112.123182 |
| 128 | Encryption |  1024.00 B | 109.357896 |
| 128 | Decryption |  1024.00 B | 108.968774 |
| 128 | Encryption |  512.00 B | 103.506223 |
| 128 | Decryption |  512.00 B | 103.316938 |
| 256 | Encryption |  32.00 KB | 98.909294 |
| 256 | Decryption |  32.00 KB | 98.957553 |
| 256 | Encryption |  16.00 KB | 99.199086 |
| 256 | Decryption |  16.00 KB | 98.901364 |
| 256 | Encryption |  8.00 KB | 99.189890 |
| 256 | Decryption |  8.00 KB | 99.188013 |
| 256 | Encryption |  4.00 KB | 102.193218 |
| 256 | Decryption |  4.00 KB | 102.789476 |
| 256 | Encryption |  2.00 KB | 101.816148 |
| 256 | Decryption |  2.00 KB | 102.454429 |
| 256 | Encryption |  1024.00 B | 100.293830 |
| 256 | Decryption |  1024.00 B | 100.109984 |
| 256 | Encryption |  512.00 B | 96.009376 |
| 256 | Decryption |  512.00 B | 95.062373 |

### SHA

- Software/Application used : test_dthe_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 311.797800 |
| 512 |  16.00 KB | 309.991131 |
| 512 |  8.00 KB | 304.436289 |
| 512 |  4.00 KB | 298.794082 |
| 512 |  2.00 KB | 286.358472 |
| 512 |  1024.00 B | 263.281376 |
| 512 |  512.00 B | 225.364512 |
| 256 |  32.00 KB | 300.963818 |
| 256 |  16.00 KB | 299.798719 |
| 256 |  8.00 KB | 295.346898 |
| 256 |  4.00 KB | 292.056418 |
| 256 |  2.00 KB | 282.008692 |
| 256 |  1024.00 B | 264.364663 |
| 256 |  512.00 B | 234.761427 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 308.962656 |
| 512 |  16.00 KB | 305.514894 |
| 512 |  8.00 KB | 297.593316 |
| 512 |  4.00 KB | 282.756984 |
| 512 |  2.00 KB | 257.397588 |
| 512 |  1024.00 B | 218.293252 |
| 512 |  512.00 B | 167.115463 |
| 256 |  32.00 KB | 299.226377 |
| 256 |  16.00 KB | 296.539632 |
| 256 |  8.00 KB | 290.689732 |
| 256 |  4.00 KB | 280.044441 |
| 256 |  2.00 KB | 259.919093 |
| 256 |  1024.00 B | 228.332520 |
| 256 |  512.00 B | 183.945212 |

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.051998 		| 517			| 275.503998		|
cos  		|0.0000002870	| 64			| 64.040001 		| 512			| 276.832001		|
sincos sin  	|0.0000001790	| 79			| 79.038002 		| 483			| 274.227997		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 73			| 73.082001 		| 578			| 428.697998		|
acos 		|0.0000004770	| 74			| 74.033997 		| 810			| 383.730011		|
atan 		|0.0000005360	| 85			| 85.071999 		| 493			| 371.148010		|
atan2 		|0.0000007150	| 119			| 106.711998 		| 729			| 478.171997		|
### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

