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

### SBL QSPI MULTICORE ELF performance

- Software/Application used           : sbl_qspi_multicore_elf and ipc_notify_echo
- Size of sbl_qspi mcelf image        : 252 KB
- Size of ipc_notify_echo             : 96 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   499
SBL : Drivers_open                      |   92
SBL : LoadHsmRtFw                       |   9402
SBL : Board_driversOpen                 |   111
SBL : CPU Load                          |   11812
SBL : SBL End                           |   17
SBL : Total time taken                  |   21935

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a> . 

### SBL SD MULTICORE ELF performance

- Software/Application used           : sbl_sd_multicore_elf and hello_world
- Size of sbl_qspi mcelf image        : 311 KB
- Size of hello_world                 : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   518
SBL : Drivers_open                      |   139667
SBL : LoadHsmRtFw                       |   9615
SBL : Board_driversOpen                 |   2950
SBL : File read from SD card            |   7886
SBL : CPU Load                          |   2779
SBL : SBL End                           |   2
SBL : Total time taken                  |   163419

- Please note that the total time taken provided at the end is not including the ROM boot time.

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    49   
    1024      |      TCMA     |     TCMA           |    46   
    1024      |      TCMB     |     TCMB           |    47   
    1024      |      OCRAM    |     TCMA           |    45   
    1024      |      TCMA     |     OCRAM          |    46   

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
 r5f0-0	| r5f0-1	|  1.90
 r5f0-0	| r5f1-0	|  1.85
 r5f0-0	| r5f1-1	|  1.92

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 11.832
 r5f0-0	| r5f1-0	| 4	| 11.820
 r5f0-0	| r5f1-1	| 4	| 11.787
 r5f0-0	| r5f0-1	| 32	| 14.599
 r5f0-0	| r5f0-1	| 64	| 17.242
 r5f0-0	| r5f0-1	| 112	| 21.055

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
 400		| 08			|  8.61 Mbps / 371.59 us 	| 22.20 Mbps / 144.16 us 	|  0.91 Mbps / 3529.38 us
 200		| 16			| 17.42 Mbps / 183.68 us 	| 28.54 Mbps / 112.14 us 	|  0.95 Mbps / 3373.52 us
 100		| 32			| 31.53 Mbps / 101.48 us 	| 33.48 Mbps / 95.57 us 	|  0.97 Mbps / 3295.78 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### DPL Low latency interrupt performance

Interrupt handler                         |   Measured execution time (ns)
------------------------------------------|-------------------------------
Non Re-entrant, Without FPU context save  |	            310
Non Re-entrant, With FPU context save	  |             426
Re-entrant, Without FPU context save	  |             376
Re-entrant, With FPU context save	      |             496

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 71
- End tick         : 10236520
- Total ticks      : 10236449
- Total time (secs): 10.236449
- Iterations/Sec   : 1465.351901
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.351901 
CoreMark/MHz :3.663380 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146129138
- USER cycle count:                          146129131

BENCHMARK Using clock 400000000
- Usertime in sec:                           0.365323
- Microseconds for one run through Dhrystone:   0.7 
- Dhrystones per Second:                     1368652.5 

Normalized MIPS/MHz:                         1.9474

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 204.012242 |
| 256 |  16.00 KB | 204.669683 |
| 256 |  8.00 KB | 202.678212 |
| 256 |  4.00 KB | 189.951162 |
| 256 |  2.00 KB | 167.967809 |
| 256 |  1024.00 B | 136.658604 |
| 256 |  512.00 B | 99.526182 |
| 128 |  32.00 KB | 205.179903 |
| 128 |  16.00 KB | 203.179327 |
| 128 |  8.00 KB | 203.040841 |
| 128 |  4.00 KB | 190.578109 |
| 128 |  2.00 KB | 167.260477 |
| 128 |  1024.00 B | 137.599731 |
| 128 |  512.00 B | 100.238605 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.895670 |
| 256 | Decryption |  32.00 KB | 95.857624 |
| 256 | Encryption |  16.00 KB | 95.925410 |
| 256 | Decryption |  16.00 KB | 95.620471 |
| 256 | Encryption |  8.00 KB | 95.268969 |
| 256 | Decryption |  8.00 KB | 96.288324 |
| 256 | Encryption |  4.00 KB | 97.341295 |
| 256 | Decryption |  4.00 KB | 98.011680 |
| 256 | Encryption |  2.00 KB | 96.478624 |
| 256 | Decryption |  2.00 KB | 96.936708 |
| 256 | Encryption |  1024.00 B | 92.864025 |
| 256 | Decryption |  1024.00 B | 93.247204 |
| 256 | Encryption |  512.00 B | 84.917591 |
| 256 | Decryption |  512.00 B | 85.306675 |
| 128 | Encryption |  32.00 KB | 104.112364 |
| 128 | Decryption |  32.00 KB | 104.021372 |
| 128 | Encryption |  16.00 KB | 103.532385 |
| 128 | Decryption |  16.00 KB | 103.739496 |
| 128 | Encryption |  8.00 KB | 103.599488 |
| 128 | Decryption |  8.00 KB | 103.449446 |
| 128 | Encryption |  4.00 KB | 106.372342 |
| 128 | Decryption |  4.00 KB | 106.364573 |
| 128 | Encryption |  2.00 KB | 105.194222 |
| 128 | Decryption |  2.00 KB | 105.476961 |
| 128 | Encryption |  1024.00 B | 100.740923 |
| 128 | Decryption |  1024.00 B | 100.275415 |
| 128 | Encryption |  512.00 B | 91.540954 |
| 128 | Decryption |  512.00 B | 91.597249 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 70.865052 |
| 256 | Decryption |  512.00 B | 256 | 77.014196 |
| 256 | Encryption |  1024.00 B | 256 | 84.227843 |
| 256 | Decryption |  1024.00 B | 256 | 85.701582 |
| 256 | Encryption |  1024.00 B | 512 | 87.509681 |
| 256 | Decryption |  1024.00 B | 512 | 88.230701 |
| 256 | Encryption |  2.00 KB | 256 | 89.947845 |
| 256 | Decryption |  2.00 KB | 256 | 90.064041 |
| 256 | Encryption |  2.00 KB | 512 | 92.455279 |
| 256 | Decryption |  2.00 KB | 512 | 92.911421 |
| 256 | Encryption |  2.00 KB | 1024 | 94.779163 |
| 256 | Decryption |  2.00 KB | 1024 | 94.732582 |
| 256 | Encryption |  4.00 KB | 256 | 92.456583 |
| 256 | Decryption |  4.00 KB | 256 | 92.761500 |
| 256 | Encryption |  4.00 KB | 512 | 95.395857 |
| 256 | Decryption |  4.00 KB | 512 | 95.636693 |
| 256 | Encryption |  4.00 KB | 1024 | 97.248850 |
| 256 | Decryption |  4.00 KB | 1024 | 96.779243 |
| 256 | Encryption |  4.00 KB | 2048 | 97.894556 |
| 256 | Decryption |  4.00 KB | 2048 | 97.589159 |
| 256 | Encryption |  8.00 KB | 256 | 90.742119 |
| 256 | Decryption |  8.00 KB | 256 | 90.526214 |
| 256 | Encryption |  8.00 KB | 512 | 93.428992 |
| 256 | Decryption |  8.00 KB | 512 | 93.789333 |
| 256 | Encryption |  8.00 KB | 1024 | 94.792529 |
| 256 | Decryption |  8.00 KB | 1024 | 95.058926 |
| 256 | Encryption |  8.00 KB | 2048 | 95.777509 |
| 256 | Decryption |  8.00 KB | 2048 | 96.009376 |
| 256 | Encryption |  8.00 KB | 4096 | 96.113924 |
| 256 | Decryption |  8.00 KB | 4096 | 95.922953 |
| 256 | Encryption |  16.00 KB | 256 | 90.201483 |
| 256 | Decryption |  16.00 KB | 256 | 90.010850 |
| 256 | Encryption |  16.00 KB | 512 | 92.815528 |
| 256 | Decryption |  16.00 KB | 512 | 92.727376 |
| 256 | Encryption |  16.00 KB | 1024 | 94.196954 |
| 256 | Decryption |  16.00 KB | 1024 | 94.059056 |
| 256 | Encryption |  16.00 KB | 2048 | 94.937573 |
| 256 | Decryption |  16.00 KB | 2048 | 94.768884 |
| 256 | Encryption |  16.00 KB | 4096 | 95.278491 |
| 256 | Decryption |  16.00 KB | 4096 | 95.017924 |
| 256 | Encryption |  16.00 KB | 8192 | 95.502726 |
| 256 | Decryption |  16.00 KB | 8192 | 95.169705 |
| 256 | Encryption |  32.00 KB | 256 | 90.179839 |
| 256 | Decryption |  32.00 KB | 256 | 90.285595 |
| 256 | Encryption |  32.00 KB | 512 | 92.943787 |
| 256 | Decryption |  32.00 KB | 512 | 92.966448 |
| 256 | Encryption |  32.00 KB | 1024 | 94.458562 |
| 256 | Decryption |  32.00 KB | 1024 | 94.273174 |
| 256 | Encryption |  32.00 KB | 2048 | 95.124551 |
| 256 | Decryption |  32.00 KB | 2048 | 94.981775 |
| 256 | Encryption |  32.00 KB | 4096 | 95.408443 |
| 256 | Decryption |  32.00 KB | 4096 | 95.449696 |
| 256 | Encryption |  32.00 KB | 8192 | 95.713426 |
| 256 | Decryption |  32.00 KB | 8192 | 95.559646 |
| 256 | Encryption |  32.00 KB | 16384 | 95.713077 |
| 256 | Decryption |  32.00 KB | 16384 | 95.717183 |
| 128 | Encryption |  512.00 B | 256 | 79.000916 |
| 128 | Decryption |  512.00 B | 256 | 82.302708 |
| 128 | Encryption |  1024.00 B | 256 | 91.390322 |
| 128 | Decryption |  1024.00 B | 256 | 92.672304 |
| 128 | Encryption |  1024.00 B | 512 | 95.796059 |
| 128 | Decryption |  1024.00 B | 512 | 95.692550 |
| 128 | Encryption |  2.00 KB | 256 | 97.318167 |
| 128 | Decryption |  2.00 KB | 256 | 97.795950 |
| 128 | Encryption |  2.00 KB | 512 | 101.025111 |
| 128 | Decryption |  2.00 KB | 512 | 101.000200 |
| 128 | Encryption |  2.00 KB | 1024 | 102.840285 |
| 128 | Decryption |  2.00 KB | 1024 | 101.743437 |
| 128 | Encryption |  4.00 KB | 256 | 99.133999 |
| 128 | Decryption |  4.00 KB | 256 | 99.835476 |
| 128 | Encryption |  4.00 KB | 512 | 103.885234 |
| 128 | Decryption |  4.00 KB | 512 | 103.914884 |
| 128 | Encryption |  4.00 KB | 1024 | 105.643588 |
| 128 | Decryption |  4.00 KB | 1024 | 105.580616 |
| 128 | Encryption |  4.00 KB | 2048 | 106.123440 |
| 128 | Decryption |  4.00 KB | 2048 | 106.374932 |
| 128 | Encryption |  8.00 KB | 256 | 97.583347 |
| 128 | Decryption |  8.00 KB | 256 | 97.957475 |
| 128 | Encryption |  8.00 KB | 512 | 101.281551 |
| 128 | Decryption |  8.00 KB | 512 | 101.698440 |
| 128 | Encryption |  8.00 KB | 1024 | 102.927099 |
| 128 | Decryption |  8.00 KB | 1024 | 102.963888 |
| 128 | Encryption |  8.00 KB | 2048 | 103.763107 |
| 128 | Decryption |  8.00 KB | 2048 | 103.772554 |
| 128 | Encryption |  8.00 KB | 4096 | 104.096860 |
| 128 | Decryption |  8.00 KB | 4096 | 104.149384 |
| 128 | Encryption |  16.00 KB | 256 | 97.318348 |
| 128 | Decryption |  16.00 KB | 256 | 97.312929 |
| 128 | Encryption |  16.00 KB | 512 | 100.473153 |
| 128 | Decryption |  16.00 KB | 512 | 100.372553 |
| 128 | Encryption |  16.00 KB | 1024 | 102.227292 |
| 128 | Decryption |  16.00 KB | 1024 | 101.941463 |
| 128 | Encryption |  16.00 KB | 2048 | 102.915179 |
| 128 | Decryption |  16.00 KB | 2048 | 102.622884 |
| 128 | Encryption |  16.00 KB | 4096 | 103.393751 |
| 128 | Decryption |  16.00 KB | 4096 | 103.112929 |
| 128 | Encryption |  16.00 KB | 8192 | 103.615253 |
| 128 | Decryption |  16.00 KB | 8192 | 103.241255 |
| 128 | Encryption |  32.00 KB | 256 | 97.422962 |
| 128 | Decryption |  32.00 KB | 256 | 97.615233 |
| 128 | Encryption |  32.00 KB | 512 | 100.707349 |
| 128 | Decryption |  32.00 KB | 512 | 100.630901 |
| 128 | Encryption |  32.00 KB | 1024 | 102.357917 |
| 128 | Decryption |  32.00 KB | 1024 | 102.166334 |
| 128 | Encryption |  32.00 KB | 2048 | 103.164566 |
| 128 | Decryption |  32.00 KB | 2048 | 103.000703 |
| 128 | Encryption |  32.00 KB | 4096 | 103.575644 |
| 128 | Decryption |  32.00 KB | 4096 | 103.349931 |
| 128 | Encryption |  32.00 KB | 8192 | 103.852824 |
| 128 | Decryption |  32.00 KB | 8192 | 103.607472 |
| 128 | Encryption |  32.00 KB | 16384 | 103.976400 |
| 128 | Decryption |  32.00 KB | 16384 | 103.840791 |


### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.716484 |
| 256 | Decryption |  32.00 KB | 95.744889 |
| 256 | Encryption |  16.00 KB | 95.757567 |
| 256 | Decryption |  16.00 KB | 95.499073 |
| 256 | Encryption |  8.00 KB | 96.097364 |
| 256 | Decryption |  8.00 KB | 96.066374 |
| 256 | Encryption |  4.00 KB | 97.358648 |
| 256 | Decryption |  4.00 KB | 97.832448 |
| 256 | Encryption |  2.00 KB | 96.268876 |
| 256 | Decryption |  2.00 KB | 96.605198 |
| 256 | Encryption |  1024.00 B | 91.640798 |
| 256 | Decryption |  1024.00 B | 92.213311 |
| 256 | Encryption |  512.00 B | 83.775630 |
| 256 | Decryption |  512.00 B | 84.610618 |
| 128 | Encryption |  32.00 KB | 104.064214 |
| 128 | Decryption |  32.00 KB | 104.053681 |
| 128 | Encryption |  16.00 KB | 103.655200 |
| 128 | Decryption |  16.00 KB | 103.768036 |
| 128 | Encryption |  8.00 KB | 103.246948 |
| 128 | Decryption |  8.00 KB | 103.837500 |
| 128 | Encryption |  4.00 KB | 105.200976 |
| 128 | Decryption |  4.00 KB | 106.544411 |
| 128 | Encryption |  2.00 KB | 104.393259 |
| 128 | Decryption |  2.00 KB | 104.564819 |
| 128 | Encryption |  1024.00 B | 99.200775 |
| 128 | Decryption |  1024.00 B | 99.716990 |
| 128 | Encryption |  512.00 B | 89.878765 |
| 128 | Decryption |  512.00 B | 90.609446 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 69.826117 |
| 256 | Decryption |  512.00 B | 256 | 76.034899 |
| 256 | Encryption |  1024.00 B | 256 | 84.022667 |
| 256 | Decryption |  1024.00 B | 256 | 84.566945 |
| 256 | Encryption |  1024.00 B | 512 | 86.557308 |
| 256 | Decryption |  1024.00 B | 512 | 87.793377 |
| 256 | Encryption |  2.00 KB | 256 | 89.721264 |
| 256 | Decryption |  2.00 KB | 256 | 90.145805 |
| 256 | Encryption |  2.00 KB | 512 | 92.151073 |
| 256 | Decryption |  2.00 KB | 512 | 92.677546 |
| 256 | Encryption |  2.00 KB | 1024 | 94.200170 |
| 256 | Decryption |  2.00 KB | 1024 | 94.236742 |
| 256 | Encryption |  4.00 KB | 256 | 92.172458 |
| 256 | Decryption |  4.00 KB | 256 | 92.604865 |
| 256 | Encryption |  4.00 KB | 512 | 95.112730 |
| 256 | Decryption |  4.00 KB | 512 | 95.372257 |
| 256 | Encryption |  4.00 KB | 1024 | 96.757096 |
| 256 | Decryption |  4.00 KB | 1024 | 96.814270 |
| 256 | Encryption |  4.00 KB | 2048 | 97.627684 |
| 256 | Decryption |  4.00 KB | 2048 | 97.322503 |
| 256 | Encryption |  8.00 KB | 256 | 90.653314 |
| 256 | Decryption |  8.00 KB | 256 | 90.551230 |
| 256 | Encryption |  8.00 KB | 512 | 93.588430 |
| 256 | Decryption |  8.00 KB | 512 | 93.582750 |
| 256 | Encryption |  8.00 KB | 1024 | 94.890664 |
| 256 | Decryption |  8.00 KB | 1024 | 94.666898 |
| 256 | Encryption |  8.00 KB | 2048 | 95.839137 |
| 256 | Decryption |  8.00 KB | 2048 | 95.869280 |
| 256 | Encryption |  8.00 KB | 4096 | 96.031534 |
| 256 | Decryption |  8.00 KB | 4096 | 95.773660 |
| 256 | Encryption |  16.00 KB | 256 | 89.985514 |
| 256 | Decryption |  16.00 KB | 256 | 90.042077 |
| 256 | Encryption |  16.00 KB | 512 | 92.825059 |
| 256 | Decryption |  16.00 KB | 512 | 92.674270 |
| 256 | Encryption |  16.00 KB | 1024 | 94.184432 |
| 256 | Decryption |  16.00 KB | 1024 | 93.961117 |
| 256 | Encryption |  16.00 KB | 2048 | 94.936885 |
| 256 | Decryption |  16.00 KB | 2048 | 94.570931 |
| 256 | Encryption |  16.00 KB | 4096 | 95.264987 |
| 256 | Decryption |  16.00 KB | 4096 | 95.420164 |
| 256 | Encryption |  16.00 KB | 8192 | 95.516297 |
| 256 | Decryption |  16.00 KB | 8192 | 95.036181 |
| 256 | Encryption |  32.00 KB | 256 | 90.147432 |
| 256 | Decryption |  32.00 KB | 256 | 90.191086 |
| 256 | Encryption |  32.00 KB | 512 | 92.992583 |
| 256 | Decryption |  32.00 KB | 512 | 92.847909 |
| 256 | Encryption |  32.00 KB | 1024 | 94.403456 |
| 256 | Decryption |  32.00 KB | 1024 | 94.210495 |
| 256 | Encryption |  32.00 KB | 2048 | 95.048500 |
| 256 | Decryption |  32.00 KB | 2048 | 94.978678 |
| 256 | Encryption |  32.00 KB | 4096 | 95.410179 |
| 256 | Decryption |  32.00 KB | 4096 | 95.263776 |
| 256 | Encryption |  32.00 KB | 8192 | 95.578548 |
| 256 | Decryption |  32.00 KB | 8192 | 95.550590 |
| 256 | Encryption |  32.00 KB | 16384 | 95.734923 |
| 256 | Decryption |  32.00 KB | 16384 | 95.693773 |
| 128 | Encryption |  512.00 B | 256 | 78.149296 |
| 128 | Decryption |  512.00 B | 256 | 80.836787 |
| 128 | Encryption |  1024.00 B | 256 | 90.979260 |
| 128 | Decryption |  1024.00 B | 256 | 91.874614 |
| 128 | Encryption |  1024.00 B | 512 | 95.089959 |
| 128 | Decryption |  1024.00 B | 512 | 93.794367 |
| 128 | Encryption |  2.00 KB | 256 | 96.833582 |
| 128 | Decryption |  2.00 KB | 256 | 97.299384 |
| 128 | Encryption |  2.00 KB | 512 | 100.584759 |
| 128 | Decryption |  2.00 KB | 512 | 99.357186 |
| 128 | Encryption |  2.00 KB | 1024 | 102.364812 |
| 128 | Decryption |  2.00 KB | 1024 | 102.361614 |
| 128 | Encryption |  4.00 KB | 256 | 97.264005 |
| 128 | Decryption |  4.00 KB | 256 | 99.132500 |
| 128 | Encryption |  4.00 KB | 512 | 103.516850 |
| 128 | Decryption |  4.00 KB | 512 | 103.126721 |
| 128 | Encryption |  4.00 KB | 1024 | 105.284634 |
| 128 | Decryption |  4.00 KB | 1024 | 105.160462 |
| 128 | Encryption |  4.00 KB | 2048 | 106.281776 |
| 128 | Decryption |  4.00 KB | 2048 | 106.198247 |
| 128 | Encryption |  8.00 KB | 256 | 97.422690 |
| 128 | Decryption |  8.00 KB | 256 | 97.923811 |
| 128 | Encryption |  8.00 KB | 512 | 101.121368 |
| 128 | Decryption |  8.00 KB | 512 | 101.202177 |
| 128 | Encryption |  8.00 KB | 1024 | 103.085762 |
| 128 | Decryption |  8.00 KB | 1024 | 102.544203 |
| 128 | Encryption |  8.00 KB | 2048 | 103.883176 |
| 128 | Decryption |  8.00 KB | 2048 | 103.690460 |
| 128 | Encryption |  8.00 KB | 4096 | 104.398663 |
| 128 | Decryption |  8.00 KB | 4096 | 103.894292 |
| 128 | Encryption |  16.00 KB | 256 | 97.201252 |
| 128 | Decryption |  16.00 KB | 256 | 97.158741 |
| 128 | Encryption |  16.00 KB | 512 | 100.438507 |
| 128 | Decryption |  16.00 KB | 512 | 100.044270 |
| 128 | Encryption |  16.00 KB | 1024 | 102.091134 |
| 128 | Decryption |  16.00 KB | 1024 | 101.677139 |
| 128 | Encryption |  16.00 KB | 2048 | 102.863892 |
| 128 | Decryption |  16.00 KB | 2048 | 102.520141 |
| 128 | Encryption |  16.00 KB | 4096 | 103.349727 |
| 128 | Decryption |  16.00 KB | 4096 | 102.974404 |
| 128 | Encryption |  16.00 KB | 8192 | 103.661963 |
| 128 | Decryption |  16.00 KB | 8192 | 103.244101 |
| 128 | Encryption |  32.00 KB | 256 | 97.291530 |
| 128 | Decryption |  32.00 KB | 256 | 97.483917 |
| 128 | Encryption |  32.00 KB | 512 | 100.608018 |
| 128 | Decryption |  32.00 KB | 512 | 100.610431 |
| 128 | Encryption |  32.00 KB | 1024 | 102.232774 |
| 128 | Decryption |  32.00 KB | 1024 | 102.140459 |
| 128 | Encryption |  32.00 KB | 2048 | 103.101878 |
| 128 | Decryption |  32.00 KB | 2048 | 103.064282 |
| 128 | Encryption |  32.00 KB | 4096 | 103.484260 |
| 128 | Decryption |  32.00 KB | 4096 | 103.414349 |
| 128 | Encryption |  32.00 KB | 8192 | 103.783544 |
| 128 | Decryption |  32.00 KB | 8192 | 103.489265 |
| 128 | Encryption |  32.00 KB | 16384 | 103.772451 |
| 128 | Decryption |  32.00 KB | 16384 | 103.818172 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 310.742587 |
| 512 |  16.00 KB | 309.976469 |
| 512 |  8.00 KB | 306.242991 |
| 512 |  4.00 KB | 299.141866 |
| 512 |  2.00 KB | 286.183406 |
| 512 |  1024.00 B | 263.048888 |
| 512 |  512.00 B | 225.768224 |
| 256 |  32.00 KB | 300.894727 |
| 256 |  16.00 KB | 300.104177 |
| 256 |  8.00 KB | 297.394126 |
| 256 |  4.00 KB | 292.206171 |
| 256 |  2.00 KB | 282.263761 |
| 256 |  1024.00 B | 264.535400 |
| 256 |  512.00 B | 235.030842 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 69.867804 |
| 256 | Decryption |  512.00 B | 256 | 76.428605 |
| 256 | Encryption |  1024.00 B | 256 | 84.597511 |
| 256 | Decryption |  1024.00 B | 256 | 84.860413 |
| 256 | Encryption |  1024.00 B | 512 | 87.554107 |
| 256 | Decryption |  1024.00 B | 512 | 87.989044 |
| 256 | Encryption |  2.00 KB | 256 | 88.541821 |
| 256 | Decryption |  2.00 KB | 256 | 90.230202 |
| 256 | Encryption |  2.00 KB | 512 | 92.848238 |
| 256 | Decryption |  2.00 KB | 512 | 92.875870 |
| 256 | Encryption |  2.00 KB | 1024 | 93.907262 |
| 256 | Decryption |  2.00 KB | 1024 | 94.059562 |
| 256 | Encryption |  4.00 KB | 256 | 91.482174 |
| 256 | Decryption |  4.00 KB | 256 | 92.228883 |
| 256 | Encryption |  4.00 KB | 512 | 95.370869 |
| 256 | Decryption |  4.00 KB | 512 | 95.015513 |
| 256 | Encryption |  4.00 KB | 1024 | 96.909473 |
| 256 | Decryption |  4.00 KB | 1024 | 96.700702 |
| 256 | Encryption |  4.00 KB | 2048 | 97.669151 |
| 256 | Decryption |  4.00 KB | 2048 | 97.781359 |
| 256 | Encryption |  8.00 KB | 256 | 90.380770 |
| 256 | Decryption |  8.00 KB | 256 | 90.745574 |
| 256 | Encryption |  8.00 KB | 512 | 93.436651 |
| 256 | Decryption |  8.00 KB | 512 | 93.350141 |
| 256 | Encryption |  8.00 KB | 1024 | 94.663479 |
| 256 | Decryption |  8.00 KB | 1024 | 94.970764 |
| 256 | Encryption |  8.00 KB | 2048 | 95.379197 |
| 256 | Decryption |  8.00 KB | 2048 | 95.691153 |
| 256 | Encryption |  8.00 KB | 4096 | 95.818469 |
| 256 | Decryption |  8.00 KB | 4096 | 95.904704 |
| 256 | Encryption |  16.00 KB | 256 | 89.921311 |
| 256 | Decryption |  16.00 KB | 256 | 89.967293 |
| 256 | Encryption |  16.00 KB | 512 | 92.845278 |
| 256 | Decryption |  16.00 KB | 512 | 92.602248 |
| 256 | Encryption |  16.00 KB | 1024 | 94.071714 |
| 256 | Decryption |  16.00 KB | 1024 | 93.979307 |
| 256 | Encryption |  16.00 KB | 2048 | 94.838826 |
| 256 | Decryption |  16.00 KB | 2048 | 94.777792 |
| 256 | Encryption |  16.00 KB | 4096 | 95.268623 |
| 256 | Decryption |  16.00 KB | 4096 | 95.048931 |
| 256 | Encryption |  16.00 KB | 8192 | 95.451781 |
| 256 | Decryption |  16.00 KB | 8192 | 95.060994 |
| 256 | Encryption |  32.00 KB | 256 | 90.141155 |
| 256 | Decryption |  32.00 KB | 256 | 90.259250 |
| 256 | Encryption |  32.00 KB | 512 | 92.974361 |
| 256 | Decryption |  32.00 KB | 512 | 92.902695 |
| 256 | Encryption |  32.00 KB | 1024 | 94.443163 |
| 256 | Decryption |  32.00 KB | 1024 | 94.247753 |
| 256 | Encryption |  32.00 KB | 2048 | 95.075819 |
| 256 | Decryption |  32.00 KB | 2048 | 94.977301 |
| 256 | Encryption |  32.00 KB | 4096 | 95.404884 |
| 256 | Decryption |  32.00 KB | 4096 | 95.274163 |
| 256 | Encryption |  32.00 KB | 8192 | 95.628843 |
| 256 | Decryption |  32.00 KB | 8192 | 95.582294 |
| 256 | Encryption |  32.00 KB | 16384 | 95.673167 |
| 256 | Decryption |  32.00 KB | 16384 | 95.651523 |
| 128 | Encryption |  512.00 B | 256 | 77.239299 |
| 128 | Decryption |  512.00 B | 256 | 80.566483 |
| 128 | Encryption |  1024.00 B | 256 | 90.724846 |
| 128 | Decryption |  1024.00 B | 256 | 91.753703 |
| 128 | Encryption |  1024.00 B | 512 | 94.716152 |
| 128 | Decryption |  1024.00 B | 512 | 95.007248 |
| 128 | Encryption |  2.00 KB | 256 | 95.483420 |
| 128 | Decryption |  2.00 KB | 256 | 97.445505 |
| 128 | Encryption |  2.00 KB | 512 | 99.538275 |
| 128 | Decryption |  2.00 KB | 512 | 100.376781 |
| 128 | Encryption |  2.00 KB | 1024 | 102.224302 |
| 128 | Decryption |  2.00 KB | 1024 | 102.342432 |
| 128 | Encryption |  4.00 KB | 256 | 99.817989 |
| 128 | Decryption |  4.00 KB | 256 | 100.135223 |
| 128 | Encryption |  4.00 KB | 512 | 103.274607 |
| 128 | Decryption |  4.00 KB | 512 | 103.493146 |
| 128 | Encryption |  4.00 KB | 1024 | 105.233072 |
| 128 | Decryption |  4.00 KB | 1024 | 104.824056 |
| 128 | Encryption |  4.00 KB | 2048 | 106.278329 |
| 128 | Decryption |  4.00 KB | 2048 | 106.246454 |
| 128 | Encryption |  8.00 KB | 256 | 97.475217 |
| 128 | Decryption |  8.00 KB | 256 | 97.820765 |
| 128 | Encryption |  8.00 KB | 512 | 101.474049 |
| 128 | Decryption |  8.00 KB | 512 | 101.142826 |
| 128 | Encryption |  8.00 KB | 1024 | 102.692030 |
| 128 | Decryption |  8.00 KB | 1024 | 102.812858 |
| 128 | Encryption |  8.00 KB | 2048 | 103.604401 |
| 128 | Decryption |  8.00 KB | 2048 | 103.724103 |
| 128 | Encryption |  8.00 KB | 4096 | 104.098100 |
| 128 | Decryption |  8.00 KB | 4096 | 104.246300 |
| 128 | Encryption |  16.00 KB | 256 | 97.104756 |
| 128 | Decryption |  16.00 KB | 256 | 97.264005 |
| 128 | Encryption |  16.00 KB | 512 | 100.304959 |
| 128 | Decryption |  16.00 KB | 512 | 100.234389 |
| 128 | Encryption |  16.00 KB | 1024 | 101.941661 |
| 128 | Decryption |  16.00 KB | 1024 | 101.777804 |
| 128 | Encryption |  16.00 KB | 2048 | 102.830603 |
| 128 | Decryption |  16.00 KB | 2048 | 102.518938 |
| 128 | Encryption |  16.00 KB | 4096 | 103.176646 |
| 128 | Decryption |  16.00 KB | 4096 | 103.164465 |
| 128 | Encryption |  16.00 KB | 8192 | 103.425774 |
| 128 | Decryption |  16.00 KB | 8192 | 103.284779 |
| 128 | Encryption |  32.00 KB | 256 | 97.359190 |
| 128 | Decryption |  32.00 KB | 256 | 97.539594 |
| 128 | Encryption |  32.00 KB | 512 | 100.634378 |
| 128 | Decryption |  32.00 KB | 512 | 100.607921 |
| 128 | Encryption |  32.00 KB | 1024 | 102.263781 |
| 128 | Decryption |  32.00 KB | 1024 | 102.218124 |
| 128 | Encryption |  32.00 KB | 2048 | 103.160608 |
| 128 | Decryption |  32.00 KB | 2048 | 102.895385 |
| 128 | Encryption |  32.00 KB | 4096 | 103.528501 |
| 128 | Decryption |  32.00 KB | 4096 | 103.276031 |
| 128 | Encryption |  32.00 KB | 8192 | 103.780565 |
| 128 | Decryption |  32.00 KB | 8192 | 103.617915 |
| 128 | Encryption |  32.00 KB | 16384 | 103.879883 |
| 128 | Decryption |  32.00 KB | 16384 | 103.807689 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.857536 |
| 256 | Decryption |  32.00 KB | 95.736060 |
| 256 | Encryption |  16.00 KB | 95.511251 |
| 256 | Decryption |  16.00 KB | 95.387353 |
| 256 | Encryption |  8.00 KB | 95.842291 |
| 256 | Decryption |  8.00 KB | 95.401065 |
| 256 | Encryption |  4.00 KB | 96.981177 |
| 256 | Decryption |  4.00 KB | 97.805437 |
| 256 | Encryption |  2.00 KB | 95.975631 |
| 256 | Decryption |  2.00 KB | 96.301412 |
| 256 | Encryption |  1024.00 B | 91.684387 |
| 256 | Decryption |  1024.00 B | 91.982933 |
| 256 | Encryption |  512.00 B | 84.098142 |
| 256 | Decryption |  512.00 B | 84.397054 |
| 128 | Encryption |  32.00 KB | 103.931158 |
| 128 | Decryption |  32.00 KB | 103.923020 |
| 128 | Encryption |  16.00 KB | 103.411086 |
| 128 | Decryption |  16.00 KB | 103.722256 |
| 128 | Encryption |  8.00 KB | 103.780771 |
| 128 | Decryption |  8.00 KB | 104.060497 |
| 128 | Encryption |  4.00 KB | 105.781709 |
| 128 | Decryption |  4.00 KB | 106.480361 |
| 128 | Encryption |  2.00 KB | 104.320143 |
| 128 | Decryption |  2.00 KB | 104.889486 |
| 128 | Encryption |  1024.00 B | 98.847662 |
| 128 | Decryption |  1024.00 B | 99.339114 |
| 128 | Encryption |  512.00 B | 89.854119 |
| 128 | Decryption |  512.00 B | 91.017166 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 308.784329 |
| 512 |  16.00 KB | 304.857598 |
| 512 |  8.00 KB | 295.640014 |
| 512 |  4.00 KB | 282.397552 |
| 512 |  2.00 KB | 256.480902 |
| 512 |  1024.00 B | 217.035369 |
| 512 |  512.00 B | 165.645536 |
| 256 |  32.00 KB | 299.569177 |
| 256 |  16.00 KB | 296.325100 |
| 256 |  8.00 KB | 289.042274 |
| 256 |  4.00 KB | 278.983440 |
| 256 |  2.00 KB | 258.933228 |
| 256 |  1024.00 B | 227.066731 |
| 256 |  512.00 B | 181.379387 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 248.978041 |
| 256 |  16.00 KB | 246.235928 |
| 256 |  8.00 KB | 247.651438 |
| 256 |  4.00 KB | 229.507967 |
| 256 |  2.00 KB | 199.531131 |
| 256 |  1024.00 B | 157.281367 |
| 256 |  512.00 B | 111.010231 |
| 128 |  32.00 KB | 250.275916 |
| 128 |  16.00 KB | 252.681093 |
| 128 |  8.00 KB | 248.546046 |
| 128 |  4.00 KB | 230.521114 |
| 128 |  2.00 KB | 200.471078 |
| 128 |  1024.00 B | 158.590650 |
| 128 |  512.00 B | 112.188442 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      203.124225      |
| 256        |      32.00 KB     |        1024        |      225.010032      |
| 256        |      32.00 KB     |        2048        |      235.979746      |
| 256        |      32.00 KB     |        4096        |      243.747181      |
| 256        |      32.00 KB     |        8192        |      247.522827      |
| 256        |      32.00 KB     |        16384        |      246.235928      |
| 256        |      16.00 KB     |        512        |      208.398124      |
| 256        |      16.00 KB     |        1024        |      228.649181      |
| 256        |      16.00 KB     |        2048        |      242.415051      |
| 256        |      16.00 KB     |        4096        |      249.191049      |
| 256        |      16.00 KB     |        8192        |      252.854139      |
| 256        |      8.00 KB     |        512        |      202.418421      |
| 256        |      8.00 KB     |        1024        |      224.188831      |
| 256        |      8.00 KB     |        2048        |      236.572842      |
| 256        |      8.00 KB     |        4096        |      241.919528      |
| 256        |      4.00 KB     |        512        |      189.585744      |
| 256        |      4.00 KB     |        1024        |      207.077856      |
| 256        |      4.00 KB     |        2048        |      219.404084      |
| 256        |      2.00 KB     |        512        |      167.328806      |
| 256        |      2.00 KB     |        1024        |      183.440631      |
| 256        |      1024.00 B     |        512        |      136.124958      |
| 128        |      32.00 KB     |        512        |      205.124915      |
| 128        |      32.00 KB     |        1024        |      225.526405      |
| 128        |      32.00 KB     |        2048        |      236.095575      |
| 128        |      32.00 KB     |        4096        |      243.047739      |
| 128        |      32.00 KB     |        8192        |      246.482502      |
| 128        |      32.00 KB     |        16384        |      248.892940      |
| 128        |      16.00 KB     |        512        |      202.615551      |
| 128        |      16.00 KB     |        1024        |      226.331558      |
| 128        |      16.00 KB     |        2048        |      242.158647      |
| 128        |      16.00 KB     |        4096        |      249.384255      |
| 128        |      16.00 KB     |        8192        |      253.100712      |
| 128        |      8.00 KB     |        512        |      202.243516      |
| 128        |      8.00 KB     |        1024        |      224.225265      |
| 128        |      8.00 KB     |        2048        |      237.099210      |
| 128        |      8.00 KB     |        4096        |      242.353420      |
| 128        |      4.00 KB     |        512        |      190.315227      |
| 128        |      4.00 KB     |        1024        |      209.473886      |
| 128        |      4.00 KB     |        2048        |      218.580839      |
| 128        |      2.00 KB     |        512        |      168.109994      |
| 128        |      2.00 KB     |        1024        |      184.400675      |
| 128        |      1024.00 B     |        512        |      136.721325      |

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 493			| 276.440002		|
cos  		|0.0000002870	| 65			| 65.082001 		| 497			| 277.664001		|
sincos sin  	|0.0000001790	| 79			| 78.958000 		| 477			| 275.514008		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.008003 		| 580			| 428.794006		|
acos 		|0.0000004770	| 76			| 76.000000 		| 818			| 384.067993		|
atan 		|0.0000005360	| 80			| 80.019997 		| 719			| 371.600006		|
atan2 		|0.0000007150	| 117			| 104.680000 		| 733			| 480.196014		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      921 |         8.686211 |
|        2 |     1828 |         8.752735 |
|        4 |     3656 |         8.752735 |
|        8 |     7301 |         8.765922 |
|       16 |    14597 |         8.768925 |
|       32 |    29186 |         8.771329 |
|       64 |    58366 |         8.772230 |
|      128 |   116724 |         8.772832 |
|      256 |   233443 |         8.773020 |
|      512 |   466878 |         8.773170 |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |       73 |       109.589041 |
|        2 |      128 |       125.000000 |
|        4 |      253 |       126.482213 |
|        8 |      494 |       129.554656 |
|       16 |      973 |       131.551901 |
|       32 |     1926 |       132.917965 |
|       64 |     3840 |       133.333333 |
|      128 |     7664 |       133.611691 |
|      256 |    15316 |       133.716375 |
|      512 |    30623 |       133.755674 |