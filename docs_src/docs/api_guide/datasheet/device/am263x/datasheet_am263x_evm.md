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

- Software/Application used           : sbl_qspi and ipc_notify_echo
- Size of sbl_qspi mcelf image        : 252 KB
- Size of ipc_notify_echo             : 97 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   556
SBL : Drivers_open                      |   96
SBL : LoadHsmRtFw                       |   8862
SBL : Board_driversOpen                 |   112
SBL : CPU Load                          |   10654
SBL : SBL End                           |   17
SBL : Total time taken                  |   20300

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a> . 

### SBL SD MULTICORE ELF performance

- Software/Application used           : sbl_sd_multicore_elf and hello_world
- Size of sbl_qspi mcelf image        : 311 KB
- Size of hello_world                 : 29 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   515
SBL : Drivers_open                      |   153865
SBL : LoadHsmRtFw                       |   9613
SBL : Board_driversOpen                 |   2952
SBL : File read from SD card            |   7872
SBL : CPU Load                          |   2784
SBL : SBL End                           |   2
SBL : Total time taken                  |   177606

- Please note that the total time taken provided at the end is not including the ROM boot time.

### EDMA performance

EDMA Memory Copy Benchmark Numbers

Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48   
    1024      |      TCMA     |     TCMA           |    45   
    1024      |      TCMB     |     TCMB           |    47   
    1024      |      OCRAM    |     TCMA           |    45   
    1024      |      TCMA     |     OCRAM          |    45   

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
 r5f0-0	| r5f0-1	|  1.84
 r5f0-0	| r5f1-0	|  1.85
 r5f0-0	| r5f1-1	|  1.91

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 11.909
 r5f0-0	| r5f1-0	| 4	| 11.983
 r5f0-0	| r5f1-1	| 4	| 11.861
 r5f0-0	| r5f0-1	| 32	| 14.585
 r5f0-0	| r5f0-1	| 64	| 17.251
 r5f0-0	| r5f0-1	| 112	| 21.103

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
 400		| 08			|  8.61 Mbps / 371.61 us 	| 22.20 Mbps / 144.17 us 	|  0.91 Mbps / 3529.54 us
 200		| 16			| 17.42 Mbps / 183.67 us 	| 28.57 Mbps / 112.01 us 	|  0.95 Mbps / 3373.64 us
 100		| 32			| 31.57 Mbps / 101.36 us 	| 33.47 Mbps / 95.60 us 	|  0.97 Mbps / 3295.88 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 71
- End tick         : 10234814
- Total ticks      : 10234743
- Total time (secs): 10.234743
- Iterations/Sec   : 1465.596156
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.596156 
CoreMark/MHz :3.663990 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146129366
- USER cycle count:                          146129359

BENCHMARK Using clock 400000000
- Usertime in sec:                           0.365323
- Microseconds for one run through Dhrystone:   0.7 
- Dhrystones per Second:                     1368650.4 

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
| 256 |  32.00 KB | 205.431541 |
| 256 |  16.00 KB | 204.812001 |
| 256 |  8.00 KB | 202.160853 |
| 256 |  4.00 KB | 189.849363 |
| 256 |  2.00 KB | 168.075503 |
| 256 |  1024.00 B | 136.744147 |
| 256 |  512.00 B | 99.508047 |
| 128 |  32.00 KB | 204.733222 |
| 128 |  16.00 KB | 206.254254 |
| 128 |  8.00 KB | 202.476268 |
| 128 |  4.00 KB | 190.414760 |
| 128 |  2.00 KB | 166.707367 |
| 128 |  1024.00 B | 137.058725 |
| 128 |  512.00 B | 100.018314 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.833269 |
| 256 | Decryption |  32.00 KB | 95.834845 |
| 256 | Encryption |  16.00 KB | 95.881903 |
| 256 | Decryption |  16.00 KB | 95.515253 |
| 256 | Encryption |  8.00 KB | 96.724965 |
| 256 | Decryption |  8.00 KB | 96.519831 |
| 256 | Encryption |  4.00 KB | 97.707010 |
| 256 | Decryption |  4.00 KB | 98.076952 |
| 256 | Encryption |  2.00 KB | 96.610894 |
| 256 | Decryption |  2.00 KB | 96.823568 |
| 256 | Encryption |  1024.00 B | 92.544058 |
| 256 | Decryption |  1024.00 B | 93.125302 |
| 256 | Encryption |  512.00 B | 85.337778 |
| 256 | Decryption |  512.00 B | 86.113739 |
| 128 | Encryption |  32.00 KB | 103.937545 |
| 128 | Decryption |  32.00 KB | 104.009610 |
| 128 | Encryption |  16.00 KB | 103.672007 |
| 128 | Decryption |  16.00 KB | 103.704612 |
| 128 | Encryption |  8.00 KB | 104.208175 |
| 128 | Decryption |  8.00 KB | 104.591936 |
| 128 | Encryption |  4.00 KB | 106.372342 |
| 128 | Decryption |  4.00 KB | 106.612874 |
| 128 | Encryption |  2.00 KB | 105.081213 |
| 128 | Decryption |  2.00 KB | 105.458290 |
| 128 | Encryption |  1024.00 B | 99.975592 |
| 128 | Decryption |  1024.00 B | 100.284621 |
| 128 | Encryption |  512.00 B | 92.460497 |
| 128 | Decryption |  512.00 B | 92.044944 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 70.657237 |
| 256 | Decryption |  512.00 B | 256 | 77.035923 |
| 256 | Encryption |  1024.00 B | 256 | 84.091667 |
| 256 | Decryption |  1024.00 B | 256 | 84.860413 |
| 256 | Encryption |  1024.00 B | 512 | 87.292877 |
| 256 | Decryption |  1024.00 B | 512 | 88.114446 |
| 256 | Encryption |  2.00 KB | 256 | 88.920246 |
| 256 | Decryption |  2.00 KB | 256 | 89.973778 |
| 256 | Encryption |  2.00 KB | 512 | 92.191259 |
| 256 | Decryption |  2.00 KB | 512 | 92.832455 |
| 256 | Encryption |  2.00 KB | 1024 | 94.655959 |
| 256 | Decryption |  2.00 KB | 1024 | 94.593112 |
| 256 | Encryption |  4.00 KB | 256 | 91.628626 |
| 256 | Decryption |  4.00 KB | 256 | 92.341292 |
| 256 | Encryption |  4.00 KB | 512 | 95.230207 |
| 256 | Decryption |  4.00 KB | 512 | 95.437534 |
| 256 | Encryption |  4.00 KB | 1024 | 97.045083 |
| 256 | Decryption |  4.00 KB | 1024 | 96.770669 |
| 256 | Encryption |  4.00 KB | 2048 | 97.898943 |
| 256 | Decryption |  4.00 KB | 2048 | 97.560104 |
| 256 | Encryption |  8.00 KB | 256 | 90.201948 |
| 256 | Decryption |  8.00 KB | 256 | 90.485905 |
| 256 | Encryption |  8.00 KB | 512 | 93.456305 |
| 256 | Decryption |  8.00 KB | 512 | 93.453306 |
| 256 | Encryption |  8.00 KB | 1024 | 94.959411 |
| 256 | Decryption |  8.00 KB | 1024 | 94.880017 |
| 256 | Encryption |  8.00 KB | 2048 | 95.637042 |
| 256 | Decryption |  8.00 KB | 2048 | 95.777859 |
| 256 | Encryption |  8.00 KB | 4096 | 95.928218 |
| 256 | Decryption |  8.00 KB | 4096 | 95.841590 |
| 256 | Encryption |  16.00 KB | 256 | 90.012241 |
| 256 | Decryption |  16.00 KB | 256 | 89.912983 |
| 256 | Encryption |  16.00 KB | 512 | 92.624988 |
| 256 | Decryption |  16.00 KB | 512 | 92.632025 |
| 256 | Encryption |  16.00 KB | 1024 | 94.098221 |
| 256 | Decryption |  16.00 KB | 1024 | 93.960444 |
| 256 | Encryption |  16.00 KB | 2048 | 94.960271 |
| 256 | Decryption |  16.00 KB | 2048 | 94.751757 |
| 256 | Encryption |  16.00 KB | 4096 | 95.224499 |
| 256 | Decryption |  16.00 KB | 4096 | 95.084096 |
| 256 | Encryption |  16.00 KB | 8192 | 95.561214 |
| 256 | Decryption |  16.00 KB | 8192 | 95.165559 |
| 256 | Encryption |  32.00 KB | 256 | 90.018346 |
| 256 | Decryption |  32.00 KB | 256 | 90.172782 |
| 256 | Encryption |  32.00 KB | 512 | 92.867726 |
| 256 | Decryption |  32.00 KB | 512 | 92.777259 |
| 256 | Encryption |  32.00 KB | 1024 | 94.290383 |
| 256 | Decryption |  32.00 KB | 1024 | 94.193738 |
| 256 | Encryption |  32.00 KB | 2048 | 95.086079 |
| 256 | Decryption |  32.00 KB | 2048 | 94.940668 |
| 256 | Encryption |  32.00 KB | 4096 | 95.377896 |
| 256 | Decryption |  32.00 KB | 4096 | 95.425895 |
| 256 | Encryption |  32.00 KB | 8192 | 95.605040 |
| 256 | Decryption |  32.00 KB | 8192 | 95.516993 |
| 256 | Encryption |  32.00 KB | 16384 | 95.678492 |
| 256 | Decryption |  32.00 KB | 16384 | 95.610009 |
| 128 | Encryption |  512.00 B | 256 | 78.508793 |
| 128 | Decryption |  512.00 B | 256 | 81.642416 |
| 128 | Encryption |  1024.00 B | 256 | 91.405618 |
| 128 | Decryption |  1024.00 B | 256 | 92.039773 |
| 128 | Encryption |  1024.00 B | 512 | 95.128607 |
| 128 | Decryption |  1024.00 B | 512 | 95.522388 |
| 128 | Encryption |  2.00 KB | 256 | 95.684167 |
| 128 | Decryption |  2.00 KB | 256 | 97.358648 |
| 128 | Encryption |  2.00 KB | 512 | 100.877382 |
| 128 | Decryption |  2.00 KB | 512 | 100.838578 |
| 128 | Encryption |  2.00 KB | 1024 | 101.754495 |
| 128 | Decryption |  2.00 KB | 1024 | 102.433611 |
| 128 | Encryption |  4.00 KB | 256 | 99.892541 |
| 128 | Decryption |  4.00 KB | 256 | 99.811148 |
| 128 | Encryption |  4.00 KB | 512 | 103.583932 |
| 128 | Decryption |  4.00 KB | 512 | 103.357673 |
| 128 | Encryption |  4.00 KB | 1024 | 105.457442 |
| 128 | Decryption |  4.00 KB | 1024 | 105.469322 |
| 128 | Encryption |  4.00 KB | 2048 | 106.182761 |
| 128 | Decryption |  4.00 KB | 2048 | 106.444906 |
| 128 | Encryption |  8.00 KB | 256 | 97.782453 |
| 128 | Decryption |  8.00 KB | 256 | 97.887976 |
| 128 | Encryption |  8.00 KB | 512 | 101.278421 |
| 128 | Decryption |  8.00 KB | 512 | 101.232661 |
| 128 | Encryption |  8.00 KB | 1024 | 103.324675 |
| 128 | Decryption |  8.00 KB | 1024 | 102.657044 |
| 128 | Encryption |  8.00 KB | 2048 | 103.459245 |
| 128 | Decryption |  8.00 KB | 2048 | 104.022920 |
| 128 | Encryption |  8.00 KB | 4096 | 104.189537 |
| 128 | Decryption |  8.00 KB | 4096 | 104.130354 |
| 128 | Encryption |  16.00 KB | 256 | 96.857732 |
| 128 | Decryption |  16.00 KB | 256 | 96.918072 |
| 128 | Encryption |  16.00 KB | 512 | 100.425809 |
| 128 | Decryption |  16.00 KB | 512 | 100.172722 |
| 128 | Encryption |  16.00 KB | 1024 | 101.947806 |
| 128 | Decryption |  16.00 KB | 1024 | 101.792031 |
| 128 | Encryption |  16.00 KB | 2048 | 102.924473 |
| 128 | Decryption |  16.00 KB | 2048 | 102.600793 |
| 128 | Encryption |  16.00 KB | 4096 | 103.321621 |
| 128 | Decryption |  16.00 KB | 4096 | 103.024991 |
| 128 | Encryption |  16.00 KB | 8192 | 103.383150 |
| 128 | Decryption |  16.00 KB | 8192 | 103.376423 |
| 128 | Encryption |  32.00 KB | 256 | 97.047328 |
| 128 | Decryption |  32.00 KB | 256 | 97.201973 |
| 128 | Encryption |  32.00 KB | 512 | 100.514085 |
| 128 | Decryption |  32.00 KB | 512 | 100.373129 |
| 128 | Encryption |  32.00 KB | 1024 | 102.158073 |
| 128 | Decryption |  32.00 KB | 1024 | 102.095309 |
| 128 | Encryption |  32.00 KB | 2048 | 103.085458 |
| 128 | Decryption |  32.00 KB | 2048 | 103.026004 |
| 128 | Encryption |  32.00 KB | 4096 | 103.528194 |
| 128 | Decryption |  32.00 KB | 4096 | 103.372754 |
| 128 | Encryption |  32.00 KB | 8192 | 103.678465 |
| 128 | Decryption |  32.00 KB | 8192 | 103.674877 |
| 128 | Encryption |  32.00 KB | 16384 | 103.916841 |
| 128 | Decryption |  32.00 KB | 16384 | 103.763107 |


### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.856046 |
| 256 | Decryption |  32.00 KB | 95.873663 |
| 256 | Encryption |  16.00 KB | 95.498725 |
| 256 | Decryption |  16.00 KB | 95.586128 |
| 256 | Encryption |  8.00 KB | 95.212913 |
| 256 | Decryption |  8.00 KB | 95.996016 |
| 256 | Encryption |  4.00 KB | 97.104756 |
| 256 | Decryption |  4.00 KB | 97.790113 |
| 256 | Encryption |  2.00 KB | 96.169988 |
| 256 | Decryption |  2.00 KB | 96.544003 |
| 256 | Encryption |  1024.00 B | 91.530726 |
| 256 | Decryption |  1024.00 B | 91.941639 |
| 256 | Encryption |  512.00 B | 84.384013 |
| 256 | Decryption |  512.00 B | 84.553853 |
| 128 | Encryption |  32.00 KB | 103.982690 |
| 128 | Decryption |  32.00 KB | 103.996303 |
| 128 | Encryption |  16.00 KB | 103.891410 |
| 128 | Decryption |  16.00 KB | 103.577793 |
| 128 | Encryption |  8.00 KB | 103.865477 |
| 128 | Decryption |  8.00 KB | 104.272838 |
| 128 | Encryption |  4.00 KB | 105.500732 |
| 128 | Decryption |  4.00 KB | 106.965243 |
| 128 | Encryption |  2.00 KB | 104.165938 |
| 128 | Decryption |  2.00 KB | 105.005448 |
| 128 | Encryption |  1024.00 B | 99.465760 |
| 128 | Decryption |  1024.00 B | 99.723059 |
| 128 | Encryption |  512.00 B | 90.051665 |
| 128 | Decryption |  512.00 B | 90.916153 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 69.520940 |
| 256 | Decryption |  512.00 B | 256 | 75.764162 |
| 256 | Encryption |  1024.00 B | 256 | 83.421589 |
| 256 | Decryption |  1024.00 B | 256 | 83.709286 |
| 256 | Encryption |  1024.00 B | 512 | 86.465947 |
| 256 | Decryption |  1024.00 B | 512 | 87.448961 |
| 256 | Encryption |  2.00 KB | 256 | 89.672158 |
| 256 | Decryption |  2.00 KB | 256 | 89.896025 |
| 256 | Encryption |  2.00 KB | 512 | 91.998428 |
| 256 | Decryption |  2.00 KB | 512 | 92.670994 |
| 256 | Encryption |  2.00 KB | 1024 | 94.243518 |
| 256 | Decryption |  2.00 KB | 1024 | 94.021778 |
| 256 | Encryption |  4.00 KB | 256 | 91.203997 |
| 256 | Decryption |  4.00 KB | 256 | 91.817335 |
| 256 | Encryption |  4.00 KB | 512 | 95.074095 |
| 256 | Decryption |  4.00 KB | 512 | 95.376421 |
| 256 | Encryption |  4.00 KB | 1024 | 96.958937 |
| 256 | Decryption |  4.00 KB | 1024 | 96.432487 |
| 256 | Encryption |  4.00 KB | 2048 | 97.646594 |
| 256 | Decryption |  4.00 KB | 2048 | 97.192603 |
| 256 | Encryption |  8.00 KB | 256 | 90.223680 |
| 256 | Decryption |  8.00 KB | 256 | 90.537470 |
| 256 | Encryption |  8.00 KB | 512 | 93.230291 |
| 256 | Decryption |  8.00 KB | 512 | 93.594779 |
| 256 | Encryption |  8.00 KB | 1024 | 94.722655 |
| 256 | Decryption |  8.00 KB | 1024 | 94.899939 |
| 256 | Encryption |  8.00 KB | 2048 | 95.502552 |
| 256 | Decryption |  8.00 KB | 2048 | 95.828276 |
| 256 | Encryption |  8.00 KB | 4096 | 95.963334 |
| 256 | Decryption |  8.00 KB | 4096 | 95.707224 |
| 256 | Encryption |  16.00 KB | 256 | 89.975167 |
| 256 | Decryption |  16.00 KB | 256 | 89.885236 |
| 256 | Encryption |  16.00 KB | 512 | 92.734756 |
| 256 | Decryption |  16.00 KB | 512 | 92.728360 |
| 256 | Encryption |  16.00 KB | 1024 | 94.111058 |
| 256 | Decryption |  16.00 KB | 1024 | 93.960107 |
| 256 | Encryption |  16.00 KB | 2048 | 94.772139 |
| 256 | Decryption |  16.00 KB | 2048 | 94.676129 |
| 256 | Encryption |  16.00 KB | 4096 | 95.149497 |
| 256 | Decryption |  16.00 KB | 4096 | 95.027913 |
| 256 | Encryption |  16.00 KB | 8192 | 96.353268 |
| 256 | Decryption |  16.00 KB | 8192 | 95.190094 |
| 256 | Encryption |  32.00 KB | 256 | 90.087177 |
| 256 | Decryption |  32.00 KB | 256 | 90.233307 |
| 256 | Encryption |  32.00 KB | 512 | 92.895370 |
| 256 | Decryption |  32.00 KB | 512 | 92.808299 |
| 256 | Encryption |  32.00 KB | 1024 | 94.335002 |
| 256 | Decryption |  32.00 KB | 1024 | 94.268089 |
| 256 | Encryption |  32.00 KB | 2048 | 95.105052 |
| 256 | Decryption |  32.00 KB | 2048 | 94.961475 |
| 256 | Encryption |  32.00 KB | 4096 | 95.363843 |
| 256 | Decryption |  32.00 KB | 4096 | 95.347628 |
| 256 | Encryption |  32.00 KB | 8192 | 95.660861 |
| 256 | Decryption |  32.00 KB | 8192 | 95.471075 |
| 256 | Encryption |  32.00 KB | 16384 | 95.670636 |
| 256 | Decryption |  32.00 KB | 16384 | 95.606522 |
| 128 | Encryption |  512.00 B | 256 | 77.778305 |
| 128 | Decryption |  512.00 B | 256 | 80.824824 |
| 128 | Encryption |  1024.00 B | 256 | 89.613302 |
| 128 | Decryption |  1024.00 B | 256 | 91.278309 |
| 128 | Encryption |  1024.00 B | 512 | 94.369726 |
| 128 | Decryption |  1024.00 B | 512 | 94.694255 |
| 128 | Encryption |  2.00 KB | 256 | 95.607393 |
| 128 | Decryption |  2.00 KB | 256 | 96.290093 |
| 128 | Encryption |  2.00 KB | 512 | 100.344506 |
| 128 | Decryption |  2.00 KB | 512 | 100.418307 |
| 128 | Encryption |  2.00 KB | 1024 | 101.381433 |
| 128 | Decryption |  2.00 KB | 1024 | 101.879460 |
| 128 | Encryption |  4.00 KB | 256 | 96.776385 |
| 128 | Decryption |  4.00 KB | 256 | 98.785828 |
| 128 | Encryption |  4.00 KB | 512 | 103.444956 |
| 128 | Decryption |  4.00 KB | 512 | 103.184362 |
| 128 | Encryption |  4.00 KB | 1024 | 105.316781 |
| 128 | Decryption |  4.00 KB | 1024 | 105.248282 |
| 128 | Encryption |  4.00 KB | 2048 | 105.666583 |
| 128 | Decryption |  4.00 KB | 2048 | 106.181040 |
| 128 | Encryption |  8.00 KB | 256 | 97.607691 |
| 128 | Decryption |  8.00 KB | 256 | 97.828066 |
| 128 | Encryption |  8.00 KB | 512 | 100.967912 |
| 128 | Decryption |  8.00 KB | 512 | 101.486620 |
| 128 | Encryption |  8.00 KB | 1024 | 102.990991 |
| 128 | Decryption |  8.00 KB | 1024 | 102.999489 |
| 128 | Encryption |  8.00 KB | 2048 | 103.647413 |
| 128 | Decryption |  8.00 KB | 2048 | 103.857247 |
| 128 | Encryption |  8.00 KB | 4096 | 104.134904 |
| 128 | Decryption |  8.00 KB | 4096 | 104.126217 |
| 128 | Encryption |  16.00 KB | 256 | 97.156761 |
| 128 | Decryption |  16.00 KB | 256 | 97.050113 |
| 128 | Encryption |  16.00 KB | 512 | 100.502813 |
| 128 | Decryption |  16.00 KB | 512 | 100.031290 |
| 128 | Encryption |  16.00 KB | 1024 | 102.121758 |
| 128 | Decryption |  16.00 KB | 1024 | 101.795984 |
| 128 | Encryption |  16.00 KB | 2048 | 102.825561 |
| 128 | Decryption |  16.00 KB | 2048 | 102.611235 |
| 128 | Encryption |  16.00 KB | 4096 | 103.207721 |
| 128 | Decryption |  16.00 KB | 4096 | 103.029850 |
| 128 | Encryption |  16.00 KB | 8192 | 103.460674 |
| 128 | Decryption |  16.00 KB | 8192 | 103.276234 |
| 128 | Encryption |  32.00 KB | 256 | 97.181883 |
| 128 | Decryption |  32.00 KB | 256 | 97.281511 |
| 128 | Encryption |  32.00 KB | 512 | 100.556690 |
| 128 | Decryption |  32.00 KB | 512 | 100.519770 |
| 128 | Encryption |  32.00 KB | 1024 | 102.111714 |
| 128 | Decryption |  32.00 KB | 1024 | 102.153395 |
| 128 | Encryption |  32.00 KB | 2048 | 103.033191 |
| 128 | Decryption |  32.00 KB | 2048 | 103.028433 |
| 128 | Encryption |  32.00 KB | 4096 | 103.619656 |
| 128 | Decryption |  32.00 KB | 4096 | 103.195939 |
| 128 | Encryption |  32.00 KB | 8192 | 103.727899 |
| 128 | Decryption |  32.00 KB | 8192 | 103.599078 |
| 128 | Encryption |  32.00 KB | 16384 | 103.800804 |
| 128 | Decryption |  32.00 KB | 16384 | 103.815500 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 311.154765 |
| 512 |  16.00 KB | 309.807954 |
| 512 |  8.00 KB | 306.425557 |
| 512 |  4.00 KB | 299.456249 |
| 512 |  2.00 KB | 286.358472 |
| 512 |  1024.00 B | 263.260223 |
| 512 |  512.00 B | 226.048565 |
| 256 |  32.00 KB | 300.956908 |
| 256 |  16.00 KB | 299.997711 |
| 256 |  8.00 KB | 297.421119 |
| 256 |  4.00 KB | 292.427826 |
| 256 |  2.00 KB | 282.434063 |
| 256 |  1024.00 B | 264.813318 |
| 256 |  512.00 B | 235.436126 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 68.982359 |
| 256 | Decryption |  512.00 B | 256 | 74.898286 |
| 256 | Encryption |  1024.00 B | 256 | 83.249917 |
| 256 | Decryption |  1024.00 B | 256 | 84.171590 |
| 256 | Encryption |  1024.00 B | 512 | 87.225490 |
| 256 | Decryption |  1024.00 B | 512 | 87.821612 |
| 256 | Encryption |  2.00 KB | 256 | 89.726177 |
| 256 | Decryption |  2.00 KB | 256 | 90.017032 |
| 256 | Encryption |  2.00 KB | 512 | 92.783826 |
| 256 | Decryption |  2.00 KB | 512 | 92.182181 |
| 256 | Encryption |  2.00 KB | 1024 | 94.223193 |
| 256 | Decryption |  2.00 KB | 1024 | 94.269275 |
| 256 | Encryption |  4.00 KB | 256 | 90.706011 |
| 256 | Decryption |  4.00 KB | 256 | 91.832131 |
| 256 | Encryption |  4.00 KB | 512 | 95.043072 |
| 256 | Decryption |  4.00 KB | 512 | 95.352830 |
| 256 | Encryption |  4.00 KB | 1024 | 96.969697 |
| 256 | Decryption |  4.00 KB | 1024 | 96.579572 |
| 256 | Encryption |  4.00 KB | 2048 | 97.737611 |
| 256 | Decryption |  4.00 KB | 2048 | 97.641502 |
| 256 | Encryption |  8.00 KB | 256 | 89.732934 |
| 256 | Decryption |  8.00 KB | 256 | 90.286139 |
| 256 | Encryption |  8.00 KB | 512 | 93.242561 |
| 256 | Decryption |  8.00 KB | 512 | 93.504641 |
| 256 | Encryption |  8.00 KB | 1024 | 94.770939 |
| 256 | Decryption |  8.00 KB | 1024 | 94.386036 |
| 256 | Encryption |  8.00 KB | 2048 | 95.630413 |
| 256 | Decryption |  8.00 KB | 2048 | 95.639484 |
| 256 | Encryption |  8.00 KB | 4096 | 96.169988 |
| 256 | Decryption |  8.00 KB | 4096 | 95.951041 |
| 256 | Encryption |  16.00 KB | 256 | 89.848267 |
| 256 | Decryption |  16.00 KB | 256 | 89.942599 |
| 256 | Encryption |  16.00 KB | 512 | 92.614680 |
| 256 | Decryption |  16.00 KB | 512 | 92.496057 |
| 256 | Encryption |  16.00 KB | 1024 | 94.153141 |
| 256 | Decryption |  16.00 KB | 1024 | 94.008291 |
| 256 | Encryption |  16.00 KB | 2048 | 94.871776 |
| 256 | Decryption |  16.00 KB | 2048 | 94.604719 |
| 256 | Encryption |  16.00 KB | 4096 | 95.284898 |
| 256 | Decryption |  16.00 KB | 4096 | 94.850494 |
| 256 | Encryption |  16.00 KB | 8192 | 95.344853 |
| 256 | Decryption |  16.00 KB | 8192 | 95.235742 |
| 256 | Encryption |  32.00 KB | 256 | 90.148905 |
| 256 | Decryption |  32.00 KB | 256 | 90.166734 |
| 256 | Encryption |  32.00 KB | 512 | 92.976999 |
| 256 | Decryption |  32.00 KB | 512 | 92.827689 |
| 256 | Encryption |  32.00 KB | 1024 | 94.295216 |
| 256 | Decryption |  32.00 KB | 1024 | 94.215743 |
| 256 | Encryption |  32.00 KB | 2048 | 95.113075 |
| 256 | Decryption |  32.00 KB | 2048 | 94.933447 |
| 256 | Encryption |  32.00 KB | 4096 | 95.415128 |
| 256 | Decryption |  32.00 KB | 4096 | 95.264641 |
| 256 | Encryption |  32.00 KB | 8192 | 95.608091 |
| 256 | Decryption |  32.00 KB | 8192 | 95.429369 |
| 256 | Encryption |  32.00 KB | 16384 | 95.721203 |
| 256 | Decryption |  32.00 KB | 16384 | 95.635123 |
| 128 | Encryption |  512.00 B | 256 | 75.858876 |
| 128 | Decryption |  512.00 B | 256 | 79.980474 |
| 128 | Encryption |  1024.00 B | 256 | 90.294847 |
| 128 | Decryption |  1024.00 B | 256 | 91.505166 |
| 128 | Encryption |  1024.00 B | 512 | 94.743538 |
| 128 | Decryption |  1024.00 B | 512 | 94.902688 |
| 128 | Encryption |  2.00 KB | 256 | 96.331138 |
| 128 | Decryption |  2.00 KB | 256 | 97.261839 |
| 128 | Encryption |  2.00 KB | 512 | 100.499923 |
| 128 | Decryption |  2.00 KB | 512 | 100.577041 |
| 128 | Encryption |  2.00 KB | 1024 | 101.243608 |
| 128 | Decryption |  2.00 KB | 1024 | 101.890547 |
| 128 | Encryption |  4.00 KB | 256 | 98.229836 |
| 128 | Decryption |  4.00 KB | 256 | 98.724071 |
| 128 | Encryption |  4.00 KB | 512 | 103.429446 |
| 128 | Decryption |  4.00 KB | 512 | 103.162435 |
| 128 | Encryption |  4.00 KB | 1024 | 105.213643 |
| 128 | Decryption |  4.00 KB | 1024 | 105.271950 |
| 128 | Encryption |  4.00 KB | 2048 | 105.783416 |
| 128 | Decryption |  4.00 KB | 2048 | 106.212016 |
| 128 | Encryption |  8.00 KB | 256 | 97.645867 |
| 128 | Decryption |  8.00 KB | 256 | 97.773335 |
| 128 | Encryption |  8.00 KB | 512 | 100.993975 |
| 128 | Decryption |  8.00 KB | 512 | 101.157658 |
| 128 | Encryption |  8.00 KB | 1024 | 103.056988 |
| 128 | Decryption |  8.00 KB | 1024 | 103.046455 |
| 128 | Encryption |  8.00 KB | 2048 | 103.929716 |
| 128 | Decryption |  8.00 KB | 2048 | 104.115465 |
| 128 | Encryption |  8.00 KB | 4096 | 104.280304 |
| 128 | Decryption |  8.00 KB | 4096 | 104.049345 |
| 128 | Encryption |  16.00 KB | 256 | 97.291078 |
| 128 | Decryption |  16.00 KB | 256 | 97.060174 |
| 128 | Encryption |  16.00 KB | 512 | 100.515337 |
| 128 | Decryption |  16.00 KB | 512 | 99.972160 |
| 128 | Encryption |  16.00 KB | 1024 | 102.022596 |
| 128 | Decryption |  16.00 KB | 1024 | 101.694889 |
| 128 | Encryption |  16.00 KB | 2048 | 102.854004 |
| 128 | Decryption |  16.00 KB | 2048 | 102.544203 |
| 128 | Encryption |  16.00 KB | 4096 | 103.432711 |
| 128 | Decryption |  16.00 KB | 4096 | 102.847951 |
| 128 | Encryption |  16.00 KB | 8192 | 103.467003 |
| 128 | Decryption |  16.00 KB | 8192 | 103.356043 |
| 128 | Encryption |  32.00 KB | 256 | 97.238389 |
| 128 | Decryption |  32.00 KB | 256 | 97.290717 |
| 128 | Encryption |  32.00 KB | 512 | 100.593058 |
| 128 | Decryption |  32.00 KB | 512 | 100.424655 |
| 128 | Encryption |  32.00 KB | 1024 | 102.214138 |
| 128 | Decryption |  32.00 KB | 1024 | 102.143444 |
| 128 | Encryption |  32.00 KB | 2048 | 103.074413 |
| 128 | Decryption |  32.00 KB | 2048 | 102.920634 |
| 128 | Encryption |  32.00 KB | 4096 | 103.503362 |
| 128 | Decryption |  32.00 KB | 4096 | 103.354821 |
| 128 | Encryption |  32.00 KB | 8192 | 103.693126 |
| 128 | Decryption |  32.00 KB | 8192 | 103.714049 |
| 128 | Encryption |  32.00 KB | 16384 | 103.750376 |
| 128 | Decryption |  32.00 KB | 16384 | 103.773581 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.602338 |
| 256 | Decryption |  32.00 KB | 95.784421 |
| 256 | Encryption |  16.00 KB | 95.564872 |
| 256 | Decryption |  16.00 KB | 95.612101 |
| 256 | Encryption |  8.00 KB | 95.850701 |
| 256 | Decryption |  8.00 KB | 95.359420 |
| 256 | Encryption |  4.00 KB | 97.322503 |
| 256 | Decryption |  4.00 KB | 97.771877 |
| 256 | Encryption |  2.00 KB | 96.030478 |
| 256 | Decryption |  2.00 KB | 96.706410 |
| 256 | Encryption |  1024.00 B | 91.643361 |
| 256 | Decryption |  1024.00 B | 91.877191 |
| 256 | Encryption |  512.00 B | 84.501521 |
| 256 | Decryption |  512.00 B | 84.479736 |
| 128 | Encryption |  32.00 KB | 103.762491 |
| 128 | Decryption |  32.00 KB | 104.057295 |
| 128 | Encryption |  16.00 KB | 103.477214 |
| 128 | Decryption |  16.00 KB | 103.794125 |
| 128 | Encryption |  8.00 KB | 103.454754 |
| 128 | Decryption |  8.00 KB | 103.200205 |
| 128 | Encryption |  4.00 KB | 105.509225 |
| 128 | Decryption |  4.00 KB | 106.223206 |
| 128 | Encryption |  2.00 KB | 104.361673 |
| 128 | Decryption |  2.00 KB | 104.785508 |
| 128 | Encryption |  1024.00 B | 99.065816 |
| 128 | Decryption |  1024.00 B | 99.659367 |
| 128 | Encryption |  512.00 B | 90.324715 |
| 128 | Decryption |  512.00 B | 90.384509 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 308.918965 |
| 512 |  16.00 KB | 305.027868 |
| 512 |  8.00 KB | 297.053758 |
| 512 |  4.00 KB | 282.543652 |
| 512 |  2.00 KB | 257.610063 |
| 512 |  1024.00 B | 218.672005 |
| 512 |  512.00 B | 167.474190 |
| 256 |  32.00 KB | 299.266515 |
| 256 |  16.00 KB | 296.694018 |
| 256 |  8.00 KB | 291.064132 |
| 256 |  4.00 KB | 277.818521 |
| 256 |  2.00 KB | 260.342430 |
| 256 |  1024.00 B | 228.603321 |
| 256 |  512.00 B | 184.670875 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 249.687110 |
| 256 |  16.00 KB | 253.779430 |
| 256 |  8.00 KB | 248.012261 |
| 256 |  4.00 KB | 229.251058 |
| 256 |  2.00 KB | 199.743980 |
| 256 |  1024.00 B | 156.657264 |
| 256 |  512.00 B | 109.856511 |
| 128 |  32.00 KB | 249.567068 |
| 128 |  16.00 KB | 246.865495 |
| 128 |  8.00 KB | 240.342529 |
| 128 |  4.00 KB | 229.934741 |
| 128 |  2.00 KB | 199.920686 |
| 128 |  1024.00 B | 156.799694 |
| 128 |  512.00 B | 110.501113 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      199.611276      |
| 256        |      32.00 KB     |        1024        |      221.723759      |
| 256        |      32.00 KB     |        2048        |      235.134141      |
| 256        |      32.00 KB     |        4096        |      242.300778      |
| 256        |      32.00 KB     |        8192        |      246.615254      |
| 256        |      32.00 KB     |        16384        |      247.889135      |
| 256        |      16.00 KB     |        512        |      199.406673      |
| 256        |      16.00 KB     |        1024        |      224.349999      |
| 256        |      16.00 KB     |        2048        |      236.457610      |
| 256        |      16.00 KB     |        4096        |      247.483101      |
| 256        |      16.00 KB     |        8192        |      251.931939      |
| 256        |      8.00 KB     |        512        |      198.481166      |
| 256        |      8.00 KB     |        1024        |      221.491458      |
| 256        |      8.00 KB     |        2048        |      234.006999      |
| 256        |      8.00 KB     |        4096        |      242.125097      |
| 256        |      4.00 KB     |        512        |      185.035857      |
| 256        |      4.00 KB     |        1024        |      206.406097      |
| 256        |      4.00 KB     |        2048        |      217.966541      |
| 256        |      2.00 KB     |        512        |      164.816538      |
| 256        |      2.00 KB     |        1024        |      181.258989      |
| 256        |      1024.00 B     |        512        |      133.779701      |
| 128        |      32.00 KB     |        512        |      200.871626      |
| 128        |      32.00 KB     |        1024        |      223.011123      |
| 128        |      32.00 KB     |        2048        |      233.914079      |
| 128        |      32.00 KB     |        4096        |      243.875301      |
| 128        |      32.00 KB     |        8192        |      247.334827      |
| 128        |      32.00 KB     |        16384        |      248.410619      |
| 128        |      16.00 KB     |        512        |      199.427911      |
| 128        |      16.00 KB     |        1024        |      223.055716      |
| 128        |      16.00 KB     |        2048        |      241.046413      |
| 128        |      16.00 KB     |        4096        |      249.643121      |
| 128        |      16.00 KB     |        8192        |      252.496123      |
| 128        |      8.00 KB     |        512        |      198.816855      |
| 128        |      8.00 KB     |        1024        |      222.016701      |
| 128        |      8.00 KB     |        2048        |      234.193058      |
| 128        |      8.00 KB     |        4096        |      242.499144      |
| 128        |      4.00 KB     |        512        |      185.544011      |
| 128        |      4.00 KB     |        1024        |      207.051687      |
| 128        |      4.00 KB     |        2048        |      219.004495      |
| 128        |      2.00 KB     |        512        |      165.219584      |
| 128        |      2.00 KB     |        1024        |      182.201340      |
| 128        |      1024.00 B     |        512        |      134.947698      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48   
    1024      |      TCMA     |     TCMA           |    45   
    1024      |      TCMB     |     TCMB           |    47   
    1024      |      OCRAM    |     TCMA           |    45   
    1024      |      TCMA     |     OCRAM          |    45

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 493			| 276.447998		|
cos  		|0.0000002870	| 65			| 65.120003 		| 497			| 277.664001		|
sincos sin  	|0.0000001790	| 79			| 78.958000 		| 477			| 275.350006		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.008003 		| 580			| 428.787994		|
acos 		|0.0000004770	| 76			| 76.000000 		| 854			| 384.052002		|
atan 		|0.0000005360	| 80			| 80.061996 		| 756			| 371.660004		|
atan2 		|0.0000007150	| 117			| 104.690002 		| 759			| 480.178009		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      916 |         8.733624 |
|        2 |     1825 |         8.767123 |
|        4 |     3647 |         8.774335 |
|        8 |     7287 |         8.782764 |
|       16 |    14566 |         8.787588 |
|       32 |    29134 |         8.786984 |
|       64 |    58264 |         8.787588 |
|      128 |   116528 |         8.787588 |
|      256 |   233053 |         8.787701 |
|      512 |   466105 |         8.787720 |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |       68 |       117.647059 |
|        2 |      124 |       129.032258 |
|        4 |      236 |       135.593220 |
|        8 |      463 |       138.228942 |
|       16 |      916 |       139.737991 |
|       32 |     1825 |       140.273973 |
|       64 |     3637 |       140.775364 |
|      128 |     7264 |       140.969163 |
|      256 |    14511 |       141.134312 |
|      512 |    29022 |       141.134312 |