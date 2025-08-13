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
- Size of sbl_qspi appimage        : 228 KB
- Size of ipc_notify_echo          : 97 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   646
SBL : Drivers_open                      |   102
SBL : LoadHsmRtFw                       |   9071
SBL : Board_driversOpen                 |   115
SBL : CPU Load                          |   10374
SBL : SBL End                           |   1
SBL : Total time taken                  |   20173

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI MULTICORE ELF performance

- Software/Application used        : sbl_qspi and ipc_notify_echo
- Size of sbl_qspi appimage        : 253 KB
- Size of ipc_notify_echo          : 97 KB

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

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_qspi appimage        : 780 KB
- Size of hello_world              : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   598
SBL : Drivers_open                      |   135261
SBL : LoadHsmRtFw                       |   9063
SBL : Board_driversOpen                 |   2948
SBL : File read from SD card            |   8966
SBL : CPU Load                          |   40
SBL : SBL End                           |   2979
SBL : Total time taken                  |   159858

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD MULTICORE ELF performance

- Software/Application used        : sbl_sd_multicore_elf and hello_world
- Size of sbl_qspi appimage        : 313 KB
- Size of hello_world              : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   726
SBL : Drivers_open                      |   139505
SBL : LoadHsmRtFw                       |   9072
SBL : Board_driversOpen                 |   2982
SBL : File read from SD card            |   8522
SBL : CPU Load                          |   4375
SBL : SBL End                           |   16
SBL : Total time taken                  |   165200

- Please note that the total time taken provided at the end is not including the ROM boot time.

### EDMA performance

EDMA Memory Copy Benchmark Numbers

Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    53   
    1024      |      TCMA     |     TCMA           |    51   
    1024      |      TCMB     |     TCMB           |    50   
    1024      |      OCRAM    |     TCMA           |    50   
    1024      |      TCMA     |     OCRAM          |    51   

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
 r5f0-0	| r5f0-1	|  1.68
 r5f0-0	| r5f1-0	|  1.83
 r5f0-0	| r5f1-1	|  1.89

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 11.890
 r5f0-0	| r5f1-0	| 4	| 11.999
 r5f0-0	| r5f1-1	| 4	| 11.901
 r5f0-0	| r5f0-1	| 32	| 14.550
 r5f0-0	| r5f0-1	| 64	| 17.133
 r5f0-0	| r5f0-1	| 112	| 20.923

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
 400		| 08			|  8.57 Mbps / 373.38 us 	| 22.32 Mbps / 143.38 us 	|  0.91 Mbps / 3528.99 us
 200		| 16			| 17.35 Mbps / 184.49 us 	| 28.66 Mbps / 111.64 us 	|  0.95 Mbps / 3373.16 us
 100		| 32			| 31.45 Mbps / 101.75 us 	| 33.64 Mbps / 95.13 us 	|  0.97 Mbps / 3295.35 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 72
- End tick         : 10234555
- Total ticks      : 10234483
- Total time (secs): 10.234483
- Iterations/Sec   : 1465.633389
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.633389 
CoreMark/MHz :3.664083 / STACK

### DHRYSTONE

- BEGIN cycle count:                                7
- END Cycle count:                                  146628813
- USER cycle count:                                 146628806

BENCHMARK Using clock 400000000
- Usertime in sec:                                  0.366572
- Microseconds for one run through Dhrystone:       0.7 
- Dhrystones per Second:                            1363988.5 

Normalized MIPS/MHz:                                1.9408

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 208.380730 |
| 256 |  16.00 KB | 209.138813 |
| 256 |  8.00 KB | 207.456415 |
| 256 |  4.00 KB | 197.314386 |
| 256 |  2.00 KB | 177.287237 |
| 256 |  1024.00 B | 147.060408 |
| 256 |  512.00 B | 110.226050 |
| 128 |  32.00 KB | 208.690945 |
| 128 |  16.00 KB | 208.274772 |
| 128 |  8.00 KB | 208.681808 |
| 128 |  4.00 KB | 197.868422 |
| 128 |  2.00 KB | 178.504113 |
| 128 |  1024.00 B | 146.181299 |
| 128 |  512.00 B | 112.249931 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.985735 |
| 256 | Decryption |  32.00 KB | 95.850964 |
| 256 | Encryption |  16.00 KB | 95.717708 |
| 256 | Decryption |  16.00 KB | 95.692725 |
| 256 | Encryption |  8.00 KB | 95.761065 |
| 256 | Decryption |  8.00 KB | 95.556163 |
| 256 | Encryption |  4.00 KB | 97.949423 |
| 256 | Decryption |  4.00 KB | 98.719610 |
| 256 | Encryption |  2.00 KB | 97.047238 |
| 256 | Decryption |  2.00 KB | 97.801788 |
| 256 | Encryption |  1024.00 B | 94.337124 |
| 256 | Decryption |  1024.00 B | 94.519442 |
| 256 | Encryption |  512.00 B | 88.509535 |
| 256 | Decryption |  512.00 B | 89.087053 |
| 128 | Encryption |  32.00 KB | 104.206103 |
| 128 | Decryption |  32.00 KB | 103.984236 |
| 128 | Encryption |  16.00 KB | 103.734775 |
| 128 | Decryption |  16.00 KB | 103.848401 |
| 128 | Encryption |  8.00 KB | 106.670139 |
| 128 | Decryption |  8.00 KB | 105.053420 |
| 128 | Encryption |  4.00 KB | 106.484686 |
| 128 | Decryption |  4.00 KB | 107.213729 |
| 128 | Encryption |  2.00 KB | 106.194805 |
| 128 | Decryption |  2.00 KB | 106.541813 |
| 128 | Encryption |  1024.00 B | 102.448023 |
| 128 | Decryption |  1024.00 B | 102.460836 |
| 128 | Encryption |  512.00 B | 94.880704 |
| 128 | Decryption |  512.00 B | 95.222597 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 73.751969 |
| 256 | Decryption |  512.00 B | 256 | 79.092445 |
| 256 | Encryption |  1024.00 B | 256 | 85.960126 |
| 256 | Decryption |  1024.00 B | 256 | 86.584754 |
| 256 | Encryption |  1024.00 B | 512 | 88.569343 |
| 256 | Decryption |  1024.00 B | 512 | 89.320177 |
| 256 | Encryption |  2.00 KB | 256 | 90.410694 |
| 256 | Decryption |  2.00 KB | 256 | 91.126005 |
| 256 | Encryption |  2.00 KB | 512 | 93.123979 |
| 256 | Decryption |  2.00 KB | 512 | 93.605473 |
| 256 | Encryption |  2.00 KB | 1024 | 95.468120 |
| 256 | Decryption |  2.00 KB | 1024 | 95.391692 |
| 256 | Encryption |  4.00 KB | 256 | 92.661822 |
| 256 | Decryption |  4.00 KB | 256 | 93.053899 |
| 256 | Encryption |  4.00 KB | 512 | 95.761765 |
| 256 | Decryption |  4.00 KB | 512 | 96.084684 |
| 256 | Encryption |  4.00 KB | 1024 | 97.621140 |
| 256 | Decryption |  4.00 KB | 1024 | 97.207740 |
| 256 | Encryption |  4.00 KB | 2048 | 98.477073 |
| 256 | Decryption |  4.00 KB | 2048 | 98.082089 |
| 256 | Encryption |  8.00 KB | 256 | 91.104153 |
| 256 | Decryption |  8.00 KB | 256 | 90.629181 |
| 256 | Encryption |  8.00 KB | 512 | 93.749754 |
| 256 | Decryption |  8.00 KB | 512 | 93.777590 |
| 256 | Encryption |  8.00 KB | 1024 | 95.200119 |
| 256 | Decryption |  8.00 KB | 1024 | 95.075130 |
| 256 | Encryption |  8.00 KB | 2048 | 96.079402 |
| 256 | Decryption |  8.00 KB | 2048 | 95.950338 |
| 256 | Encryption |  8.00 KB | 4096 | 96.302473 |
| 256 | Decryption |  8.00 KB | 4096 | 96.001641 |
| 256 | Encryption |  16.00 KB | 256 | 90.154485 |
| 256 | Decryption |  16.00 KB | 256 | 90.049500 |
| 256 | Encryption |  16.00 KB | 512 | 92.940903 |
| 256 | Decryption |  16.00 KB | 512 | 92.633335 |
| 256 | Encryption |  16.00 KB | 1024 | 94.118154 |
| 256 | Decryption |  16.00 KB | 1024 | 94.328637 |
| 256 | Encryption |  16.00 KB | 2048 | 95.102206 |
| 256 | Decryption |  16.00 KB | 2048 | 94.838483 |
| 256 | Encryption |  16.00 KB | 4096 | 95.418948 |
| 256 | Decryption |  16.00 KB | 4096 | 95.036181 |
| 256 | Encryption |  16.00 KB | 8192 | 95.531613 |
| 256 | Decryption |  16.00 KB | 8192 | 95.320238 |
| 256 | Encryption |  32.00 KB | 256 | 90.208777 |
| 256 | Decryption |  32.00 KB | 256 | 90.304956 |
| 256 | Encryption |  32.00 KB | 512 | 93.070500 |
| 256 | Decryption |  32.00 KB | 512 | 92.961832 |
| 256 | Encryption |  32.00 KB | 1024 | 94.421817 |
| 256 | Decryption |  32.00 KB | 1024 | 94.295216 |
| 256 | Encryption |  32.00 KB | 2048 | 95.167891 |
| 256 | Decryption |  32.00 KB | 2048 | 95.043761 |
| 256 | Encryption |  32.00 KB | 4096 | 95.505944 |
| 256 | Decryption |  32.00 KB | 4096 | 95.482550 |
| 256 | Encryption |  32.00 KB | 8192 | 95.728106 |
| 256 | Decryption |  32.00 KB | 8192 | 95.627971 |
| 256 | Encryption |  32.00 KB | 16384 | 95.907248 |
| 256 | Decryption |  32.00 KB | 16384 | 95.641927 |
| 128 | Encryption |  512.00 B | 256 | 81.124975 |
| 128 | Decryption |  512.00 B | 256 | 81.764647 |
| 128 | Encryption |  1024.00 B | 256 | 91.686953 |
| 128 | Decryption |  1024.00 B | 256 | 93.297648 |
| 128 | Encryption |  1024.00 B | 512 | 97.448403 |
| 128 | Decryption |  1024.00 B | 512 | 97.101879 |
| 128 | Encryption |  2.00 KB | 256 | 96.959654 |
| 128 | Decryption |  2.00 KB | 256 | 97.708467 |
| 128 | Encryption |  2.00 KB | 512 | 101.694494 |
| 128 | Decryption |  2.00 KB | 512 | 101.843046 |
| 128 | Encryption |  2.00 KB | 1024 | 103.609315 |
| 128 | Decryption |  2.00 KB | 1024 | 102.640564 |
| 128 | Encryption |  4.00 KB | 256 | 99.074053 |
| 128 | Decryption |  4.00 KB | 256 | 99.425017 |
| 128 | Encryption |  4.00 KB | 512 | 104.234693 |
| 128 | Decryption |  4.00 KB | 512 | 104.350041 |
| 128 | Encryption |  4.00 KB | 1024 | 105.666583 |
| 128 | Decryption |  4.00 KB | 1024 | 106.163840 |
| 128 | Encryption |  4.00 KB | 2048 | 107.113846 |
| 128 | Decryption |  4.00 KB | 2048 | 106.678821 |
| 128 | Encryption |  8.00 KB | 256 | 97.659690 |
| 128 | Decryption |  8.00 KB | 256 | 97.917958 |
| 128 | Encryption |  8.00 KB | 512 | 101.536149 |
| 128 | Decryption |  8.00 KB | 512 | 101.901637 |
| 128 | Encryption |  8.00 KB | 1024 | 103.203861 |
| 128 | Decryption |  8.00 KB | 1024 | 103.312866 |
| 128 | Encryption |  8.00 KB | 2048 | 104.499376 |
| 128 | Decryption |  8.00 KB | 2048 | 103.818584 |
| 128 | Encryption |  8.00 KB | 4096 | 104.726482 |
| 128 | Decryption |  8.00 KB | 4096 | 104.333014 |
| 128 | Encryption |  16.00 KB | 256 | 97.304801 |
| 128 | Decryption |  16.00 KB | 256 | 97.428845 |
| 128 | Encryption |  16.00 KB | 512 | 100.674185 |
| 128 | Decryption |  16.00 KB | 512 | 100.435621 |
| 128 | Encryption |  16.00 KB | 1024 | 102.261986 |
| 128 | Decryption |  16.00 KB | 1024 | 101.931355 |
| 128 | Encryption |  16.00 KB | 2048 | 103.163044 |
| 128 | Decryption |  16.00 KB | 2048 | 102.803988 |
| 128 | Encryption |  16.00 KB | 4096 | 103.459041 |
| 128 | Decryption |  16.00 KB | 4096 | 103.194923 |
| 128 | Encryption |  16.00 KB | 8192 | 103.706663 |
| 128 | Decryption |  16.00 KB | 8192 | 103.520938 |
| 128 | Encryption |  32.00 KB | 256 | 97.357382 |
| 128 | Decryption |  32.00 KB | 256 | 97.487723 |
| 128 | Encryption |  32.00 KB | 512 | 100.780136 |
| 128 | Decryption |  32.00 KB | 512 | 100.638821 |
| 128 | Encryption |  32.00 KB | 1024 | 102.475556 |
| 128 | Decryption |  32.00 KB | 1024 | 102.177684 |
| 128 | Encryption |  32.00 KB | 2048 | 103.230176 |
| 128 | Decryption |  32.00 KB | 2048 | 103.027826 |
| 128 | Encryption |  32.00 KB | 4096 | 103.600818 |
| 128 | Decryption |  32.00 KB | 4096 | 103.423937 |
| 128 | Encryption |  32.00 KB | 8192 | 103.776560 |
| 128 | Decryption |  32.00 KB | 8192 | 103.943109 |
| 128 | Encryption |  32.00 KB | 16384 | 104.118877 |
| 128 | Decryption |  32.00 KB | 16384 | 103.938266 |


### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.980902 |
| 256 | Decryption |  32.00 KB | 95.816192 |
| 256 | Encryption |  16.00 KB | 95.887515 |
| 256 | Decryption |  16.00 KB | 95.664264 |
| 256 | Encryption |  8.00 KB | 95.730990 |
| 256 | Decryption |  8.00 KB | 96.918072 |
| 256 | Encryption |  4.00 KB | 97.736153 |
| 256 | Decryption |  4.00 KB | 98.384675 |
| 256 | Encryption |  2.00 KB | 97.090370 |
| 256 | Decryption |  2.00 KB | 97.648777 |
| 256 | Encryption |  1024.00 B | 94.239452 |
| 256 | Decryption |  1024.00 B | 94.174450 |
| 256 | Encryption |  512.00 B | 87.831028 |
| 256 | Decryption |  512.00 B | 87.977232 |
| 128 | Encryption |  32.00 KB | 104.002492 |
| 128 | Decryption |  32.00 KB | 104.131388 |
| 128 | Encryption |  16.00 KB | 104.017967 |
| 128 | Decryption |  16.00 KB | 103.922711 |
| 128 | Encryption |  8.00 KB | 103.914884 |
| 128 | Decryption |  8.00 KB | 104.724391 |
| 128 | Encryption |  4.00 KB | 105.981856 |
| 128 | Decryption |  4.00 KB | 107.554199 |
| 128 | Encryption |  2.00 KB | 105.400624 |
| 128 | Decryption |  2.00 KB | 106.098528 |
| 128 | Encryption |  1024.00 B | 100.948860 |
| 128 | Decryption |  1024.00 B | 102.125538 |
| 128 | Encryption |  512.00 B | 93.708534 |
| 128 | Decryption |  512.00 B | 94.814815 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 71.602133 |
| 256 | Decryption |  512.00 B | 256 | 77.017816 |
| 256 | Encryption |  1024.00 B | 256 | 84.444903 |
| 256 | Decryption |  1024.00 B | 256 | 86.054940 |
| 256 | Encryption |  1024.00 B | 512 | 87.894638 |
| 256 | Decryption |  1024.00 B | 512 | 88.782920 |
| 256 | Encryption |  2.00 KB | 256 | 90.098710 |
| 256 | Decryption |  2.00 KB | 256 | 90.588154 |
| 256 | Encryption |  2.00 KB | 512 | 92.914056 |
| 256 | Decryption |  2.00 KB | 512 | 93.439982 |
| 256 | Encryption |  2.00 KB | 1024 | 95.080302 |
| 256 | Decryption |  2.00 KB | 1024 | 94.987970 |
| 256 | Encryption |  4.00 KB | 256 | 91.480259 |
| 256 | Decryption |  4.00 KB | 256 | 92.474848 |
| 256 | Encryption |  4.00 KB | 512 | 95.472987 |
| 256 | Decryption |  4.00 KB | 512 | 95.773660 |
| 256 | Encryption |  4.00 KB | 1024 | 97.449128 |
| 256 | Decryption |  4.00 KB | 1024 | 97.132821 |
| 256 | Encryption |  4.00 KB | 2048 | 98.154805 |
| 256 | Decryption |  4.00 KB | 2048 | 97.770418 |
| 256 | Encryption |  8.00 KB | 256 | 90.193258 |
| 256 | Decryption |  8.00 KB | 256 | 90.021360 |
| 256 | Encryption |  8.00 KB | 512 | 93.359118 |
| 256 | Decryption |  8.00 KB | 512 | 93.675048 |
| 256 | Encryption |  8.00 KB | 1024 | 95.057202 |
| 256 | Decryption |  8.00 KB | 1024 | 95.044795 |
| 256 | Encryption |  8.00 KB | 2048 | 95.722950 |
| 256 | Decryption |  8.00 KB | 2048 | 95.891373 |
| 256 | Encryption |  8.00 KB | 4096 | 96.139303 |
| 256 | Decryption |  8.00 KB | 4096 | 96.030478 |
| 256 | Encryption |  16.00 KB | 256 | 90.228493 |
| 256 | Decryption |  16.00 KB | 256 | 90.152160 |
| 256 | Encryption |  16.00 KB | 512 | 92.851362 |
| 256 | Decryption |  16.00 KB | 512 | 92.726064 |
| 256 | Encryption |  16.00 KB | 1024 | 94.209141 |
| 256 | Decryption |  16.00 KB | 1024 | 94.117309 |
| 256 | Encryption |  16.00 KB | 2048 | 94.988486 |
| 256 | Decryption |  16.00 KB | 2048 | 94.783275 |
| 256 | Encryption |  16.00 KB | 4096 | 95.351616 |
| 256 | Decryption |  16.00 KB | 4096 | 95.166941 |
| 256 | Encryption |  16.00 KB | 8192 | 95.577590 |
| 256 | Decryption |  16.00 KB | 8192 | 96.165578 |
| 256 | Encryption |  32.00 KB | 256 | 90.167897 |
| 256 | Decryption |  32.00 KB | 256 | 90.258783 |
| 256 | Encryption |  32.00 KB | 512 | 93.010565 |
| 256 | Decryption |  32.00 KB | 512 | 92.983925 |
| 256 | Encryption |  32.00 KB | 1024 | 94.455499 |
| 256 | Decryption |  32.00 KB | 1024 | 94.249617 |
| 256 | Encryption |  32.00 KB | 2048 | 95.179466 |
| 256 | Decryption |  32.00 KB | 2048 | 94.947717 |
| 256 | Encryption |  32.00 KB | 4096 | 95.464034 |
| 256 | Decryption |  32.00 KB | 4096 | 95.349535 |
| 256 | Encryption |  32.00 KB | 8192 | 95.687922 |
| 256 | Decryption |  32.00 KB | 8192 | 95.535617 |
| 256 | Encryption |  32.00 KB | 16384 | 95.819957 |
| 256 | Decryption |  32.00 KB | 16384 | 95.678056 |
| 128 | Encryption |  512.00 B | 256 | 78.486228 |
| 128 | Decryption |  512.00 B | 256 | 82.406197 |
| 128 | Encryption |  1024.00 B | 256 | 92.680167 |
| 128 | Decryption |  1024.00 B | 256 | 92.622534 |
| 128 | Encryption |  1024.00 B | 512 | 96.712119 |
| 128 | Decryption |  1024.00 B | 512 | 96.697849 |
| 128 | Encryption |  2.00 KB | 256 | 96.746383 |
| 128 | Decryption |  2.00 KB | 256 | 98.027074 |
| 128 | Encryption |  2.00 KB | 512 | 101.441065 |
| 128 | Decryption |  2.00 KB | 512 | 101.362617 |
| 128 | Encryption |  2.00 KB | 1024 | 103.138082 |
| 128 | Decryption |  2.00 KB | 1024 | 101.863624 |
| 128 | Encryption |  4.00 KB | 256 | 99.086037 |
| 128 | Decryption |  4.00 KB | 256 | 100.351420 |
| 128 | Encryption |  4.00 KB | 512 | 104.082394 |
| 128 | Decryption |  4.00 KB | 512 | 104.144419 |
| 128 | Encryption |  4.00 KB | 1024 | 105.420970 |
| 128 | Decryption |  4.00 KB | 1024 | 105.794516 |
| 128 | Encryption |  4.00 KB | 2048 | 106.923359 |
| 128 | Decryption |  4.00 KB | 2048 | 106.772675 |
| 128 | Encryption |  8.00 KB | 256 | 97.787924 |
| 128 | Decryption |  8.00 KB | 256 | 98.074750 |
| 128 | Encryption |  8.00 KB | 512 | 101.421834 |
| 128 | Decryption |  8.00 KB | 512 | 101.430075 |
| 128 | Encryption |  8.00 KB | 1024 | 102.977236 |
| 128 | Decryption |  8.00 KB | 1024 | 103.404967 |
| 128 | Encryption |  8.00 KB | 2048 | 103.965575 |
| 128 | Decryption |  8.00 KB | 2048 | 104.200719 |
| 128 | Encryption |  8.00 KB | 4096 | 104.234279 |
| 128 | Decryption |  8.00 KB | 4096 | 104.331768 |
| 128 | Encryption |  16.00 KB | 256 | 97.310761 |
| 128 | Decryption |  16.00 KB | 256 | 97.269779 |
| 128 | Encryption |  16.00 KB | 512 | 100.618927 |
| 128 | Decryption |  16.00 KB | 512 | 100.313403 |
| 128 | Encryption |  16.00 KB | 1024 | 102.180671 |
| 128 | Decryption |  16.00 KB | 1024 | 101.962676 |
| 128 | Encryption |  16.00 KB | 2048 | 103.287018 |
| 128 | Decryption |  16.00 KB | 2048 | 102.702089 |
| 128 | Encryption |  16.00 KB | 4096 | 103.409658 |
| 128 | Decryption |  16.00 KB | 4096 | 103.144575 |
| 128 | Encryption |  16.00 KB | 8192 | 103.352987 |
| 128 | Decryption |  16.00 KB | 8192 | 103.440874 |
| 128 | Encryption |  32.00 KB | 256 | 97.488358 |
| 128 | Decryption |  32.00 KB | 256 | 97.453113 |
| 128 | Encryption |  32.00 KB | 512 | 100.656500 |
| 128 | Decryption |  32.00 KB | 512 | 100.696228 |
| 128 | Encryption |  32.00 KB | 1024 | 102.304789 |
| 128 | Decryption |  32.00 KB | 1024 | 102.234667 |
| 128 | Encryption |  32.00 KB | 2048 | 103.181418 |
| 128 | Decryption |  32.00 KB | 2048 | 103.044430 |
| 128 | Encryption |  32.00 KB | 4096 | 103.607779 |
| 128 | Decryption |  32.00 KB | 4096 | 103.362156 |
| 128 | Encryption |  32.00 KB | 8192 | 103.811492 |
| 128 | Decryption |  32.00 KB | 8192 | 103.809128 |
| 128 | Encryption |  32.00 KB | 16384 | 104.015594 |
| 128 | Decryption |  32.00 KB | 16384 | 103.800701 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 311.305339 |
| 512 |  16.00 KB | 309.197707 |
| 512 |  8.00 KB | 306.583241 |
| 512 |  4.00 KB | 299.278473 |
| 512 |  2.00 KB | 286.521226 |
| 512 |  1024.00 B | 263.514274 |
| 512 |  512.00 B | 226.454734 |
| 256 |  32.00 KB | 300.782523 |
| 256 |  16.00 KB | 300.042349 |
| 256 |  8.00 KB | 297.094157 |
| 256 |  4.00 KB | 291.763868 |
| 256 |  2.00 KB | 282.409722 |
| 256 |  1024.00 B | 264.749131 |
| 256 |  512.00 B | 235.300876 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 72.239859 |
| 256 | Decryption |  512.00 B | 256 | 77.301250 |
| 256 | Encryption |  1024.00 B | 256 | 84.774791 |
| 256 | Decryption |  1024.00 B | 256 | 86.392997 |
| 256 | Encryption |  1024.00 B | 512 | 88.814202 |
| 256 | Decryption |  1024.00 B | 512 | 88.763680 |
| 256 | Encryption |  2.00 KB | 256 | 89.998489 |
| 256 | Decryption |  2.00 KB | 256 | 90.761284 |
| 256 | Encryption |  2.00 KB | 512 | 93.688439 |
| 256 | Decryption |  2.00 KB | 512 | 93.449308 |
| 256 | Encryption |  2.00 KB | 1024 | 94.531712 |
| 256 | Decryption |  2.00 KB | 1024 | 94.787388 |
| 256 | Encryption |  4.00 KB | 256 | 90.637642 |
| 256 | Decryption |  4.00 KB | 256 | 92.362114 |
| 256 | Encryption |  4.00 KB | 512 | 95.758266 |
| 256 | Decryption |  4.00 KB | 512 | 95.484811 |
| 256 | Encryption |  4.00 KB | 1024 | 97.381794 |
| 256 | Decryption |  4.00 KB | 1024 | 97.183234 |
| 256 | Encryption |  4.00 KB | 2048 | 98.071081 |
| 256 | Decryption |  4.00 KB | 2048 | 98.083557 |
| 256 | Encryption |  8.00 KB | 256 | 90.695341 |
| 256 | Decryption |  8.00 KB | 256 | 90.871401 |
| 256 | Encryption |  8.00 KB | 512 | 93.848765 |
| 256 | Decryption |  8.00 KB | 512 | 93.683752 |
| 256 | Encryption |  8.00 KB | 1024 | 95.083061 |
| 256 | Decryption |  8.00 KB | 1024 | 95.174888 |
| 256 | Encryption |  8.00 KB | 2048 | 95.796059 |
| 256 | Decryption |  8.00 KB | 2048 | 95.881553 |
| 256 | Encryption |  8.00 KB | 4096 | 96.235655 |
| 256 | Decryption |  8.00 KB | 4096 | 95.980551 |
| 256 | Encryption |  16.00 KB | 256 | 90.126279 |
| 256 | Decryption |  16.00 KB | 256 | 89.975631 |
| 256 | Encryption |  16.00 KB | 512 | 92.885084 |
| 256 | Decryption |  16.00 KB | 512 | 92.847909 |
| 256 | Encryption |  16.00 KB | 1024 | 94.244534 |
| 256 | Decryption |  16.00 KB | 1024 | 94.027511 |
| 256 | Encryption |  16.00 KB | 2048 | 95.038593 |
| 256 | Decryption |  16.00 KB | 2048 | 94.805899 |
| 256 | Encryption |  16.00 KB | 4096 | 95.453867 |
| 256 | Decryption |  16.00 KB | 4096 | 94.883452 |
| 256 | Encryption |  16.00 KB | 8192 | 95.543973 |
| 256 | Decryption |  16.00 KB | 8192 | 95.211184 |
| 256 | Encryption |  32.00 KB | 256 | 90.158671 |
| 256 | Decryption |  32.00 KB | 256 | 90.379056 |
| 256 | Encryption |  32.00 KB | 512 | 93.017248 |
| 256 | Decryption |  32.00 KB | 512 | 92.960184 |
| 256 | Encryption |  32.00 KB | 1024 | 94.442993 |
| 256 | Decryption |  32.00 KB | 1024 | 94.319134 |
| 256 | Encryption |  32.00 KB | 2048 | 95.097721 |
| 256 | Decryption |  32.00 KB | 2048 | 95.042728 |
| 256 | Encryption |  32.00 KB | 4096 | 95.478203 |
| 256 | Decryption |  32.00 KB | 4096 | 95.315213 |
| 256 | Encryption |  32.00 KB | 8192 | 95.649953 |
| 256 | Decryption |  32.00 KB | 8192 | 95.644544 |
| 256 | Encryption |  32.00 KB | 16384 | 95.830554 |
| 256 | Decryption |  32.00 KB | 16384 | 95.691328 |
| 128 | Encryption |  512.00 B | 256 | 79.634490 |
| 128 | Decryption |  512.00 B | 256 | 83.561993 |
| 128 | Encryption |  1024.00 B | 256 | 90.895978 |
| 128 | Decryption |  1024.00 B | 256 | 92.343244 |
| 128 | Encryption |  1024.00 B | 512 | 94.642290 |
| 128 | Decryption |  1024.00 B | 512 | 96.003750 |
| 128 | Encryption |  2.00 KB | 256 | 96.321228 |
| 128 | Decryption |  2.00 KB | 256 | 97.899674 |
| 128 | Encryption |  2.00 KB | 512 | 101.393982 |
| 128 | Decryption |  2.00 KB | 512 | 100.177316 |
| 128 | Encryption |  2.00 KB | 1024 | 102.912957 |
| 128 | Decryption |  2.00 KB | 1024 | 103.121853 |
| 128 | Encryption |  4.00 KB | 256 | 99.002213 |
| 128 | Decryption |  4.00 KB | 256 | 100.711509 |
| 128 | Encryption |  4.00 KB | 512 | 103.816117 |
| 128 | Decryption |  4.00 KB | 512 | 104.137800 |
| 128 | Encryption |  4.00 KB | 1024 | 105.930464 |
| 128 | Decryption |  4.00 KB | 1024 | 105.499883 |
| 128 | Encryption |  4.00 KB | 2048 | 106.760499 |
| 128 | Decryption |  4.00 KB | 2048 | 106.834465 |
| 128 | Encryption |  8.00 KB | 256 | 97.334066 |
| 128 | Decryption |  8.00 KB | 256 | 98.605981 |
| 128 | Encryption |  8.00 KB | 512 | 101.473657 |
| 128 | Decryption |  8.00 KB | 512 | 101.583365 |
| 128 | Encryption |  8.00 KB | 1024 | 102.967933 |
| 128 | Decryption |  8.00 KB | 1024 | 103.351969 |
| 128 | Encryption |  8.00 KB | 2048 | 103.965988 |
| 128 | Decryption |  8.00 KB | 2048 | 104.117946 |
| 128 | Encryption |  8.00 KB | 4096 | 104.572327 |
| 128 | Decryption |  8.00 KB | 4096 | 104.035305 |
| 128 | Encryption |  16.00 KB | 256 | 97.378900 |
| 128 | Decryption |  16.00 KB | 256 | 97.411829 |
| 128 | Encryption |  16.00 KB | 512 | 100.521504 |
| 128 | Decryption |  16.00 KB | 512 | 100.397540 |
| 128 | Encryption |  16.00 KB | 1024 | 102.121161 |
| 128 | Decryption |  16.00 KB | 1024 | 101.896884 |
| 128 | Encryption |  16.00 KB | 2048 | 102.951757 |
| 128 | Decryption |  16.00 KB | 2048 | 102.793103 |
| 128 | Encryption |  16.00 KB | 4096 | 103.470679 |
| 128 | Decryption |  16.00 KB | 4096 | 103.025801 |
| 128 | Encryption |  16.00 KB | 8192 | 103.658479 |
| 128 | Decryption |  16.00 KB | 8192 | 103.293326 |
| 128 | Encryption |  32.00 KB | 256 | 97.477210 |
| 128 | Decryption |  32.00 KB | 256 | 97.516554 |
| 128 | Encryption |  32.00 KB | 512 | 100.749441 |
| 128 | Decryption |  32.00 KB | 512 | 100.630901 |
| 128 | Encryption |  32.00 KB | 1024 | 102.300098 |
| 128 | Decryption |  32.00 KB | 1024 | 102.193617 |
| 128 | Encryption |  32.00 KB | 2048 | 103.208737 |
| 128 | Decryption |  32.00 KB | 2048 | 102.966113 |
| 128 | Encryption |  32.00 KB | 4096 | 103.645364 |
| 128 | Decryption |  32.00 KB | 4096 | 103.424141 |
| 128 | Encryption |  32.00 KB | 8192 | 103.711177 |
| 128 | Decryption |  32.00 KB | 8192 | 103.720820 |
| 128 | Encryption |  32.00 KB | 16384 | 103.975678 |
| 128 | Decryption |  32.00 KB | 16384 | 103.856013 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.769111 |
| 256 | Decryption |  32.00 KB | 95.936731 |
| 256 | Encryption |  16.00 KB | 95.846846 |
| 256 | Decryption |  16.00 KB | 95.582817 |
| 256 | Encryption |  8.00 KB | 95.691153 |
| 256 | Decryption |  8.00 KB | 96.532982 |
| 256 | Encryption |  4.00 KB | 97.892363 |
| 256 | Decryption |  4.00 KB | 98.224684 |
| 256 | Encryption |  2.00 KB | 96.969697 |
| 256 | Decryption |  2.00 KB | 97.516554 |
| 256 | Encryption |  1024.00 B | 93.366765 |
| 256 | Decryption |  1024.00 B | 93.593442 |
| 256 | Encryption |  512.00 B | 86.628245 |
| 256 | Decryption |  512.00 B | 87.390655 |
| 128 | Encryption |  32.00 KB | 104.132215 |
| 128 | Decryption |  32.00 KB | 104.030867 |
| 128 | Encryption |  16.00 KB | 104.016316 |
| 128 | Decryption |  16.00 KB | 103.740727 |
| 128 | Encryption |  8.00 KB | 103.600307 |
| 128 | Decryption |  8.00 KB | 103.765571 |
| 128 | Encryption |  4.00 KB | 106.291256 |
| 128 | Decryption |  4.00 KB | 107.500390 |
| 128 | Encryption |  2.00 KB | 105.378592 |
| 128 | Decryption |  2.00 KB | 105.928752 |
| 128 | Encryption |  1024.00 B | 99.972542 |
| 128 | Decryption |  1024.00 B | 102.131904 |
| 128 | Encryption |  512.00 B | 93.499971 |
| 128 | Decryption |  512.00 B | 94.063612 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 309.012734 |
| 512 |  16.00 KB | 305.198328 |
| 512 |  8.00 KB | 297.235640 |
| 512 |  4.00 KB | 282.580200 |
| 512 |  2.00 KB | 257.508841 |
| 512 |  1024.00 B | 218.191504 |
| 512 |  512.00 B | 166.979209 |
| 256 |  32.00 KB | 299.355369 |
| 256 |  16.00 KB | 296.707451 |
| 256 |  8.00 KB | 291.057669 |
| 256 |  4.00 KB | 280.200094 |
| 256 |  2.00 KB | 260.311408 |
| 256 |  1024.00 B | 228.507671 |
| 256 |  512.00 B | 183.635956 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 253.977973 |
| 256 |  16.00 KB | 255.204977 |
| 256 |  8.00 KB | 251.379912 |
| 256 |  4.00 KB | 237.759396 |
| 256 |  2.00 KB | 210.179276 |
| 256 |  1024.00 B | 170.329556 |
| 256 |  512.00 B | 122.947621 |
| 128 |  32.00 KB | 253.375926 |
| 128 |  16.00 KB | 250.149339 |
| 128 |  8.00 KB | 254.158345 |
| 128 |  4.00 KB | 237.097066 |
| 128 |  2.00 KB | 211.816419 |
| 128 |  1024.00 B | 171.273259 |
| 128 |  512.00 B | 125.336597 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      207.915484      |
| 256        |      32.00 KB     |        1024        |      229.282138      |
| 256        |      32.00 KB     |        2048        |      240.633749      |
| 256        |      32.00 KB     |        4096        |      247.459739      |
| 256        |      32.00 KB     |        8192        |      249.340372      |
| 256        |      32.00 KB     |        16384        |      252.867554      |
| 256        |      16.00 KB     |        512        |      210.143051      |
| 256        |      16.00 KB     |        1024        |      234.074907      |
| 256        |      16.00 KB     |        2048        |      246.360890      |
| 256        |      16.00 KB     |        4096        |      254.179292      |
| 256        |      16.00 KB     |        8192        |      257.739236      |
| 256        |      8.00 KB     |        512        |      208.698421      |
| 256        |      8.00 KB     |        1024        |      230.128520      |
| 256        |      8.00 KB     |        2048        |      242.485685      |
| 256        |      8.00 KB     |        4096        |      247.522827      |
| 256        |      4.00 KB     |        512        |      196.344898      |
| 256        |      4.00 KB     |        1024        |      216.708827      |
| 256        |      4.00 KB     |        2048        |      227.243884      |
| 256        |      2.00 KB     |        512        |      176.646900      |
| 256        |      2.00 KB     |        1024        |      192.554723      |
| 256        |      1024.00 B     |        512        |      144.206311      |
| 128        |      32.00 KB     |        512        |      210.483042      |
| 128        |      32.00 KB     |        1024        |      229.486871      |
| 128        |      32.00 KB     |        2048        |      239.910312      |
| 128        |      32.00 KB     |        4096        |      245.861226      |
| 128        |      32.00 KB     |        8192        |      249.646687      |
| 128        |      32.00 KB     |        16384        |      251.259441      |
| 128        |      16.00 KB     |        512        |      209.193054      |
| 128        |      16.00 KB     |        1024        |      229.000725      |
| 128        |      16.00 KB     |        2048        |      240.972184      |
| 128        |      16.00 KB     |        4096        |      252.878531      |
| 128        |      16.00 KB     |        8192        |      257.439297      |
| 128        |      8.00 KB     |        512        |      209.011250      |
| 128        |      8.00 KB     |        1024        |      229.522033      |
| 128        |      8.00 KB     |        2048        |      242.739411      |
| 128        |      8.00 KB     |        4096        |      249.169732      |
| 128        |      4.00 KB     |        512        |      198.137622      |
| 128        |      4.00 KB     |        1024        |      217.067718      |
| 128        |      4.00 KB     |        2048        |      228.074266      |
| 128        |      2.00 KB     |        512        |      177.561028      |
| 128        |      2.00 KB     |        1024        |      193.864813      |
| 128        |      1024.00 B     |        512        |      148.567283      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    53   
    1024      |      TCMA     |     TCMA           |    51   
    1024      |      TCMB     |     TCMB           |    50   
    1024      |      OCRAM    |     TCMA           |    50   
    1024      |      TCMA     |     OCRAM          |    51   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.166000 		| -77			| 275.826019		|
cos  		|0.0000002870	| 65			| 65.094002 		| 497			| 277.687988		|
sincos sin  	|0.0000001790	| 79			| 78.998001 		| 611			| 276.056000		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 417			| 74.776001 		| 618			| 428.783997		|
acos 		|0.0000004770	| 76			| 76.036003 		| 556			| 383.334015		|
atan 		|0.0000005360	| 80			| 80.057999 		| 493			| 370.946014		|
atan2 		|0.0000007150	| 430			| 105.363998 		| 594			| 479.402008		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size (KB) | Time (us) | Throughput (Mbps) |
|-----------|-----------|-------------------|
| 1         | 997       | 8.024072          |
| 2         | 1985      | 8.060453          |
| 4         | 3960      | 8.080808          |
| 8         | 7911      | 8.090001          |
| 16        | 15815     | 8.093582          |
| 32        | 31622     | 8.095630          |
| 64        | 63233     | 8.097038          |
| 128       | 126459    | 8.097486          |
| 256       | 252912    | 8.097678          |
| 512       | 505815    | 8.097822          |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size (KB) | Time (us) | Throughput (Mbps) |
|-----------|-----------|-------------------|
| 1         | 75        | 106.666667        |
| 2         | 128       | 125.000000        |
| 4         | 241       | 132.780083        |
| 8         | 467       | 137.044968        |
| 16        | 921       | 138.979370        |
| 32        | 1831      | 139.814309        |
| 64        | 3642      | 140.582098        |
| 128       | 7270      | 140.852820        |
| 256       | 14519     | 141.056547        |
| 512       | 29028     | 141.105140        |