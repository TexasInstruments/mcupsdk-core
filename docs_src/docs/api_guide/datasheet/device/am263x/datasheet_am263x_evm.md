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
- Size of ipc_notify_echo             : 97 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   550
SBL : Drivers_open                      |   95
SBL : LoadHsmRtFw                       |   9387
SBL : Board_driversOpen                 |   112
SBL : CPU Load                          |   11921
SBL : SBL End                           |   17
SBL : Total time taken                  |   22084

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
SBL : System Init                       |   572
SBL : Drivers_open                      |   136635
SBL : LoadHsmRtFw                       |   9597
SBL : Board_driversOpen                 |   2976
SBL : File read from SD card            |   7809
SBL : CPU Load                          |   4346
SBL : SBL End                           |   1
SBL : Total time taken                  |   161940

- Please note that the total time taken provided at the end is not including the ROM boot time.

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
 r5f0-0	| r5f1-1	|  1.91

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 11.914
 r5f0-0	| r5f1-0	| 4	| 12.026
 r5f0-0	| r5f1-1	| 4	| 11.838
 r5f0-0	| r5f0-1	| 32	| 14.578
 r5f0-0	| r5f0-1	| 64	| 17.280
 r5f0-0	| r5f0-1	| 112	| 21.096

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
- END Cycle count:                           146129238
- USER cycle count:                          146129231

BENCHMARK Using clock 400000000
- Usertime in sec:                           0.365323
- Microseconds for one run through Dhrystone:   0.7 
- Dhrystones per Second:                     1368651.6 

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
| 256 |  32.00 KB | 205.085598 |
| 256 |  16.00 KB | 202.697802 |
| 256 |  8.00 KB | 200.653680 |
| 256 |  4.00 KB | 189.995216 |
| 256 |  2.00 KB | 167.985031 |
| 256 |  1024.00 B | 136.595940 |
| 256 |  512.00 B | 99.080793 |
| 128 |  32.00 KB | 205.874811 |
| 128 |  16.00 KB | 204.067430 |
| 128 |  8.00 KB | 202.596760 |
| 128 |  4.00 KB | 190.066849 |
| 128 |  2.00 KB | 166.639544 |
| 128 |  1024.00 B | 137.213685 |
| 128 |  512.00 B | 99.932906 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.769024 |
| 256 | Decryption |  32.00 KB | 95.849212 |
| 256 | Encryption |  16.00 KB | 95.502552 |
| 256 | Decryption |  16.00 KB | 95.521170 |
| 256 | Encryption |  8.00 KB | 95.712116 |
| 256 | Decryption |  8.00 KB | 95.533876 |
| 256 | Encryption |  4.00 KB | 97.618232 |
| 256 | Decryption |  4.00 KB | 98.380983 |
| 256 | Encryption |  2.00 KB | 96.840736 |
| 256 | Decryption |  2.00 KB | 97.004144 |
| 256 | Encryption |  1024.00 B | 92.345846 |
| 256 | Decryption |  1024.00 B | 93.059184 |
| 256 | Encryption |  512.00 B | 85.209070 |
| 256 | Decryption |  512.00 B | 85.829535 |
| 128 | Encryption |  32.00 KB | 103.870621 |
| 128 | Decryption |  32.00 KB | 104.002595 |
| 128 | Encryption |  16.00 KB | 103.622421 |
| 128 | Decryption |  16.00 KB | 103.625083 |
| 128 | Encryption |  8.00 KB | 104.518125 |
| 128 | Decryption |  8.00 KB | 104.645379 |
| 128 | Encryption |  4.00 KB | 106.292980 |
| 128 | Decryption |  4.00 KB | 106.554805 |
| 128 | Encryption |  2.00 KB | 105.093008 |
| 128 | Decryption |  2.00 KB | 105.177339 |
| 128 | Encryption |  1024.00 B | 100.201823 |
| 128 | Decryption |  1024.00 B | 100.358335 |
| 128 | Encryption |  512.00 B | 91.828270 |
| 128 | Decryption |  512.00 B | 92.122575 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 71.815552 |
| 256 | Decryption |  512.00 B | 256 | 76.474981 |
| 256 | Encryption |  1024.00 B | 256 | 84.886793 |
| 256 | Decryption |  1024.00 B | 256 | 85.100636 |
| 256 | Encryption |  1024.00 B | 512 | 87.181397 |
| 256 | Decryption |  1024.00 B | 512 | 88.313928 |
| 256 | Encryption |  2.00 KB | 256 | 88.622042 |
| 256 | Decryption |  2.00 KB | 256 | 89.592476 |
| 256 | Encryption |  2.00 KB | 512 | 92.429200 |
| 256 | Decryption |  2.00 KB | 512 | 92.774632 |
| 256 | Encryption |  2.00 KB | 1024 | 94.500360 |
| 256 | Decryption |  2.00 KB | 1024 | 94.511263 |
| 256 | Encryption |  4.00 KB | 256 | 91.352742 |
| 256 | Decryption |  4.00 KB | 256 | 92.196446 |
| 256 | Encryption |  4.00 KB | 512 | 95.144489 |
| 256 | Decryption |  4.00 KB | 512 | 95.328557 |
| 256 | Encryption |  4.00 KB | 1024 | 97.127783 |
| 256 | Decryption |  4.00 KB | 1024 | 96.682157 |
| 256 | Encryption |  4.00 KB | 2048 | 97.752918 |
| 256 | Decryption |  4.00 KB | 2048 | 97.480292 |
| 256 | Encryption |  8.00 KB | 256 | 90.117604 |
| 256 | Decryption |  8.00 KB | 256 | 90.760655 |
| 256 | Encryption |  8.00 KB | 512 | 93.501639 |
| 256 | Decryption |  8.00 KB | 512 | 93.364770 |
| 256 | Encryption |  8.00 KB | 1024 | 94.916089 |
| 256 | Decryption |  8.00 KB | 1024 | 94.625208 |
| 256 | Encryption |  8.00 KB | 2048 | 95.657981 |
| 256 | Decryption |  8.00 KB | 2048 | 95.654141 |
| 256 | Encryption |  8.00 KB | 4096 | 96.050534 |
| 256 | Decryption |  8.00 KB | 4096 | 95.789058 |
| 256 | Encryption |  16.00 KB | 256 | 90.044087 |
| 256 | Decryption |  16.00 KB | 256 | 90.040530 |
| 256 | Encryption |  16.00 KB | 512 | 92.827031 |
| 256 | Decryption |  16.00 KB | 512 | 92.653634 |
| 256 | Encryption |  16.00 KB | 1024 | 94.231153 |
| 256 | Decryption |  16.00 KB | 1024 | 93.914663 |
| 256 | Encryption |  16.00 KB | 2048 | 94.945482 |
| 256 | Decryption |  16.00 KB | 2048 | 94.645023 |
| 256 | Encryption |  16.00 KB | 4096 | 95.201329 |
| 256 | Decryption |  16.00 KB | 4096 | 95.090304 |
| 256 | Encryption |  16.00 KB | 8192 | 95.483941 |
| 256 | Decryption |  16.00 KB | 8192 | 95.203922 |
| 256 | Encryption |  32.00 KB | 256 | 89.964746 |
| 256 | Decryption |  32.00 KB | 256 | 90.133872 |
| 256 | Encryption |  32.00 KB | 512 | 92.999511 |
| 256 | Decryption |  32.00 KB | 512 | 92.889857 |
| 256 | Encryption |  32.00 KB | 1024 | 94.387820 |
| 256 | Decryption |  32.00 KB | 1024 | 94.205755 |
| 256 | Encryption |  32.00 KB | 2048 | 95.079699 |
| 256 | Decryption |  32.00 KB | 2048 | 94.943246 |
| 256 | Encryption |  32.00 KB | 4096 | 95.473943 |
| 256 | Decryption |  32.00 KB | 4096 | 95.321451 |
| 256 | Encryption |  32.00 KB | 8192 | 95.696655 |
| 256 | Decryption |  32.00 KB | 8192 | 95.479942 |
| 256 | Encryption |  32.00 KB | 16384 | 95.666272 |
| 256 | Decryption |  32.00 KB | 16384 | 95.630238 |
| 128 | Encryption |  512.00 B | 256 | 78.750300 |
| 128 | Decryption |  512.00 B | 256 | 81.997898 |
| 128 | Encryption |  1024.00 B | 256 | 91.202093 |
| 128 | Decryption |  1024.00 B | 256 | 92.551899 |
| 128 | Encryption |  1024.00 B | 512 | 95.653443 |
| 128 | Decryption |  1024.00 B | 512 | 95.636693 |
| 128 | Encryption |  2.00 KB | 256 | 96.185514 |
| 128 | Decryption |  2.00 KB | 256 | 97.689533 |
| 128 | Encryption |  2.00 KB | 512 | 100.969079 |
| 128 | Decryption |  2.00 KB | 512 | 100.944195 |
| 128 | Encryption |  2.00 KB | 1024 | 101.805077 |
| 128 | Decryption |  2.00 KB | 1024 | 102.472051 |
| 128 | Encryption |  4.00 KB | 256 | 98.927491 |
| 128 | Decryption |  4.00 KB | 256 | 99.757213 |
| 128 | Encryption |  4.00 KB | 512 | 103.767625 |
| 128 | Decryption |  4.00 KB | 512 | 103.361749 |
| 128 | Encryption |  4.00 KB | 1024 | 105.450654 |
| 128 | Decryption |  4.00 KB | 1024 | 105.478658 |
| 128 | Encryption |  4.00 KB | 2048 | 106.131174 |
| 128 | Decryption |  4.00 KB | 2048 | 106.417251 |
| 128 | Encryption |  8.00 KB | 256 | 97.662965 |
| 128 | Decryption |  8.00 KB | 256 | 97.790843 |
| 128 | Encryption |  8.00 KB | 512 | 101.264336 |
| 128 | Decryption |  8.00 KB | 512 | 101.562113 |
| 128 | Encryption |  8.00 KB | 1024 | 103.138488 |
| 128 | Decryption |  8.00 KB | 1024 | 102.962271 |
| 128 | Encryption |  8.00 KB | 2048 | 104.046454 |
| 128 | Decryption |  8.00 KB | 2048 | 103.994034 |
| 128 | Encryption |  8.00 KB | 4096 | 104.163041 |
| 128 | Decryption |  8.00 KB | 4096 | 104.148557 |
| 128 | Encryption |  16.00 KB | 256 | 97.336054 |
| 128 | Decryption |  16.00 KB | 256 | 97.178911 |
| 128 | Encryption |  16.00 KB | 512 | 100.410038 |
| 128 | Decryption |  16.00 KB | 512 | 100.228257 |
| 128 | Encryption |  16.00 KB | 1024 | 101.923230 |
| 128 | Decryption |  16.00 KB | 1024 | 101.891339 |
| 128 | Encryption |  16.00 KB | 2048 | 102.848153 |
| 128 | Decryption |  16.00 KB | 2048 | 102.746770 |
| 128 | Encryption |  16.00 KB | 4096 | 103.202439 |
| 128 | Decryption |  16.00 KB | 4096 | 103.140111 |
| 128 | Encryption |  16.00 KB | 8192 | 103.563675 |
| 128 | Decryption |  16.00 KB | 8192 | 103.441690 |
| 128 | Encryption |  32.00 KB | 256 | 97.168915 |
| 128 | Decryption |  32.00 KB | 256 | 97.403686 |
| 128 | Encryption |  32.00 KB | 512 | 100.531816 |
| 128 | Decryption |  32.00 KB | 512 | 100.537889 |
| 128 | Encryption |  32.00 KB | 1024 | 102.246530 |
| 128 | Decryption |  32.00 KB | 1024 | 102.194214 |
| 128 | Encryption |  32.00 KB | 2048 | 103.158883 |
| 128 | Decryption |  32.00 KB | 2048 | 102.948320 |
| 128 | Encryption |  32.00 KB | 4096 | 103.574212 |
| 128 | Decryption |  32.00 KB | 4096 | 103.309812 |
| 128 | Encryption |  32.00 KB | 8192 | 103.771630 |
| 128 | Decryption |  32.00 KB | 8192 | 103.664218 |
| 128 | Encryption |  32.00 KB | 16384 | 103.805736 |
| 128 | Decryption |  32.00 KB | 16384 | 103.838117 |


### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.826525 |
| 256 | Decryption |  32.00 KB | 95.838612 |
| 256 | Encryption |  16.00 KB | 95.268623 |
| 256 | Decryption |  16.00 KB | 95.597282 |
| 256 | Encryption |  8.00 KB | 95.812165 |
| 256 | Decryption |  8.00 KB | 96.429294 |
| 256 | Encryption |  4.00 KB | 97.664784 |
| 256 | Decryption |  4.00 KB | 97.924542 |
| 256 | Encryption |  2.00 KB | 96.096660 |
| 256 | Decryption |  2.00 KB | 96.424683 |
| 256 | Encryption |  1024.00 B | 91.825697 |
| 256 | Decryption |  1024.00 B | 92.314627 |
| 256 | Encryption |  512.00 B | 83.685770 |
| 256 | Decryption |  512.00 B | 84.072250 |
| 128 | Encryption |  32.00 KB | 104.003523 |
| 128 | Decryption |  32.00 KB | 104.138420 |
| 128 | Encryption |  16.00 KB | 103.603173 |
| 128 | Decryption |  16.00 KB | 103.986196 |
| 128 | Encryption |  8.00 KB | 102.946097 |
| 128 | Decryption |  8.00 KB | 103.216865 |
| 128 | Encryption |  4.00 KB | 104.941553 |
| 128 | Decryption |  4.00 KB | 106.836207 |
| 128 | Encryption |  2.00 KB | 104.546470 |
| 128 | Decryption |  2.00 KB | 104.745313 |
| 128 | Encryption |  1024.00 B | 99.152748 |
| 128 | Decryption |  1024.00 B | 99.474819 |
| 128 | Encryption |  512.00 B | 90.130927 |
| 128 | Decryption |  512.00 B | 90.820399 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 69.963276 |
| 256 | Decryption |  512.00 B | 256 | 75.231885 |
| 256 | Encryption |  1024.00 B | 256 | 83.334605 |
| 256 | Decryption |  1024.00 B | 256 | 84.761634 |
| 256 | Encryption |  1024.00 B | 512 | 86.534450 |
| 256 | Decryption |  1024.00 B | 512 | 87.586871 |
| 256 | Encryption |  2.00 KB | 256 | 88.787731 |
| 256 | Decryption |  2.00 KB | 256 | 89.318959 |
| 256 | Encryption |  2.00 KB | 512 | 91.851437 |
| 256 | Decryption |  2.00 KB | 512 | 92.371878 |
| 256 | Encryption |  2.00 KB | 1024 | 94.327619 |
| 256 | Decryption |  2.00 KB | 1024 | 94.169038 |
| 256 | Encryption |  4.00 KB | 256 | 91.104469 |
| 256 | Decryption |  4.00 KB | 256 | 91.789686 |
| 256 | Encryption |  4.00 KB | 512 | 95.018957 |
| 256 | Decryption |  4.00 KB | 512 | 95.477160 |
| 256 | Encryption |  4.00 KB | 1024 | 97.023532 |
| 256 | Decryption |  4.00 KB | 1024 | 96.472944 |
| 256 | Encryption |  4.00 KB | 2048 | 97.785006 |
| 256 | Decryption |  4.00 KB | 2048 | 97.374560 |
| 256 | Encryption |  8.00 KB | 256 | 90.463422 |
| 256 | Decryption |  8.00 KB | 256 | 90.755313 |
| 256 | Encryption |  8.00 KB | 512 | 93.395040 |
| 256 | Decryption |  8.00 KB | 512 | 93.422999 |
| 256 | Encryption |  8.00 KB | 1024 | 94.712388 |
| 256 | Decryption |  8.00 KB | 1024 | 94.686046 |
| 256 | Encryption |  8.00 KB | 2048 | 95.616809 |
| 256 | Decryption |  8.00 KB | 2048 | 95.649953 |
| 256 | Encryption |  8.00 KB | 4096 | 96.181632 |
| 256 | Decryption |  8.00 KB | 4096 | 95.779609 |
| 256 | Encryption |  16.00 KB | 256 | 90.102272 |
| 256 | Decryption |  16.00 KB | 256 | 90.028780 |
| 256 | Encryption |  16.00 KB | 512 | 92.802385 |
| 256 | Decryption |  16.00 KB | 512 | 92.664115 |
| 256 | Encryption |  16.00 KB | 1024 | 94.115789 |
| 256 | Decryption |  16.00 KB | 1024 | 94.014023 |
| 256 | Encryption |  16.00 KB | 2048 | 94.879846 |
| 256 | Decryption |  16.00 KB | 2048 | 94.672710 |
| 256 | Encryption |  16.00 KB | 4096 | 95.101861 |
| 256 | Decryption |  16.00 KB | 4096 | 95.191131 |
| 256 | Encryption |  16.00 KB | 8192 | 95.423116 |
| 256 | Decryption |  16.00 KB | 8192 | 95.051688 |
| 256 | Encryption |  32.00 KB | 256 | 90.181235 |
| 256 | Decryption |  32.00 KB | 256 | 90.207070 |
| 256 | Encryption |  32.00 KB | 512 | 92.957629 |
| 256 | Decryption |  32.00 KB | 512 | 92.834428 |
| 256 | Encryption |  32.00 KB | 1024 | 94.318710 |
| 256 | Decryption |  32.00 KB | 1024 | 94.260970 |
| 256 | Encryption |  32.00 KB | 2048 | 95.063838 |
| 256 | Decryption |  32.00 KB | 2048 | 94.952876 |
| 256 | Encryption |  32.00 KB | 4096 | 95.413305 |
| 256 | Decryption |  32.00 KB | 4096 | 95.326304 |
| 256 | Encryption |  32.00 KB | 8192 | 95.531526 |
| 256 | Decryption |  32.00 KB | 8192 | 95.505857 |
| 256 | Encryption |  32.00 KB | 16384 | 95.703118 |
| 256 | Decryption |  32.00 KB | 16384 | 95.697266 |
| 128 | Encryption |  512.00 B | 256 | 77.535375 |
| 128 | Decryption |  512.00 B | 256 | 81.294036 |
| 128 | Encryption |  1024.00 B | 256 | 89.876299 |
| 128 | Decryption |  1024.00 B | 256 | 91.359746 |
| 128 | Encryption |  1024.00 B | 512 | 94.650491 |
| 128 | Decryption |  1024.00 B | 512 | 94.858731 |
| 128 | Encryption |  2.00 KB | 256 | 96.922372 |
| 128 | Decryption |  2.00 KB | 256 | 97.260396 |
| 128 | Encryption |  2.00 KB | 512 | 100.553893 |
| 128 | Decryption |  2.00 KB | 512 | 100.535383 |
| 128 | Encryption |  2.00 KB | 1024 | 101.386139 |
| 128 | Decryption |  2.00 KB | 1024 | 101.879460 |
| 128 | Encryption |  4.00 KB | 256 | 98.465237 |
| 128 | Decryption |  4.00 KB | 256 | 99.593487 |
| 128 | Encryption |  4.00 KB | 512 | 103.489061 |
| 128 | Decryption |  4.00 KB | 512 | 103.258335 |
| 128 | Encryption |  4.00 KB | 1024 | 105.335401 |
| 128 | Decryption |  4.00 KB | 1024 | 105.493939 |
| 128 | Encryption |  4.00 KB | 2048 | 105.897086 |
| 128 | Decryption |  4.00 KB | 2048 | 106.341273 |
| 128 | Encryption |  8.00 KB | 256 | 97.315638 |
| 128 | Decryption |  8.00 KB | 256 | 97.344909 |
| 128 | Encryption |  8.00 KB | 512 | 100.957802 |
| 128 | Decryption |  8.00 KB | 512 | 101.239698 |
| 128 | Encryption |  8.00 KB | 1024 | 103.080898 |
| 128 | Decryption |  8.00 KB | 1024 | 103.041595 |
| 128 | Encryption |  8.00 KB | 2048 | 103.472313 |
| 128 | Decryption |  8.00 KB | 2048 | 103.862185 |
| 128 | Encryption |  8.00 KB | 4096 | 104.183740 |
| 128 | Decryption |  8.00 KB | 4096 | 104.181669 |
| 128 | Encryption |  16.00 KB | 256 | 97.100620 |
| 128 | Decryption |  16.00 KB | 256 | 97.149380 |
| 128 | Encryption |  16.00 KB | 512 | 100.478544 |
| 128 | Decryption |  16.00 KB | 512 | 100.193780 |
| 128 | Encryption |  16.00 KB | 1024 | 102.129715 |
| 128 | Decryption |  16.00 KB | 1024 | 101.667281 |
| 128 | Encryption |  16.00 KB | 2048 | 102.908311 |
| 128 | Decryption |  16.00 KB | 2048 | 102.520542 |
| 128 | Encryption |  16.00 KB | 4096 | 103.299228 |
| 128 | Decryption |  16.00 KB | 4096 | 103.140922 |
| 128 | Encryption |  16.00 KB | 8192 | 103.521960 |
| 128 | Decryption |  16.00 KB | 8192 | 103.242068 |
| 128 | Encryption |  32.00 KB | 256 | 97.382247 |
| 128 | Decryption |  32.00 KB | 256 | 97.429026 |
| 128 | Encryption |  32.00 KB | 512 | 100.588137 |
| 128 | Decryption |  32.00 KB | 512 | 100.570867 |
| 128 | Encryption |  32.00 KB | 1024 | 102.262884 |
| 128 | Decryption |  32.00 KB | 1024 | 102.156779 |
| 128 | Encryption |  32.00 KB | 2048 | 103.176545 |
| 128 | Decryption |  32.00 KB | 2048 | 102.870351 |
| 128 | Encryption |  32.00 KB | 4096 | 103.633379 |
| 128 | Decryption |  32.00 KB | 4096 | 103.205283 |
| 128 | Encryption |  32.00 KB | 8192 | 103.769165 |
| 128 | Decryption |  32.00 KB | 8192 | 103.631843 |
| 128 | Encryption |  32.00 KB | 16384 | 103.860230 |
| 128 | Decryption |  32.00 KB | 16384 | 103.844699 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 311.151072 |
| 512 |  16.00 KB | 309.928826 |
| 512 |  8.00 KB | 306.293085 |
| 512 |  4.00 KB | 299.497304 |
| 512 |  2.00 KB | 286.358472 |
| 512 |  1024.00 B | 263.239075 |
| 512 |  512.00 B | 226.079757 |
| 256 |  32.00 KB | 300.886093 |
| 256 |  16.00 KB | 299.774721 |
| 256 |  8.00 KB | 297.161513 |
| 256 |  4.00 KB | 292.193143 |
| 256 |  2.00 KB | 282.446235 |
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
| 256 | Encryption |  512.00 B | 256 | 68.469221 |
| 256 | Decryption |  512.00 B | 256 | 76.084332 |
| 256 | Encryption |  1024.00 B | 256 | 83.459834 |
| 256 | Decryption |  1024.00 B | 256 | 84.113253 |
| 256 | Encryption |  1024.00 B | 512 | 87.826320 |
| 256 | Decryption |  1024.00 B | 512 | 87.838091 |
| 256 | Encryption |  2.00 KB | 256 | 88.515512 |
| 256 | Decryption |  2.00 KB | 256 | 89.925629 |
| 256 | Encryption |  2.00 KB | 512 | 92.875870 |
| 256 | Decryption |  2.00 KB | 512 | 91.955829 |
| 256 | Encryption |  2.00 KB | 1024 | 94.243518 |
| 256 | Decryption |  2.00 KB | 1024 | 94.376521 |
| 256 | Encryption |  4.00 KB | 256 | 90.829840 |
| 256 | Decryption |  4.00 KB | 256 | 91.761411 |
| 256 | Encryption |  4.00 KB | 512 | 95.019646 |
| 256 | Decryption |  4.00 KB | 512 | 95.400023 |
| 256 | Encryption |  4.00 KB | 1024 | 96.943877 |
| 256 | Decryption |  4.00 KB | 1024 | 96.632262 |
| 256 | Encryption |  4.00 KB | 2048 | 97.666239 |
| 256 | Decryption |  4.00 KB | 2048 | 97.755105 |
| 256 | Encryption |  8.00 KB | 256 | 89.792563 |
| 256 | Decryption |  8.00 KB | 256 | 90.712289 |
| 256 | Encryption |  8.00 KB | 512 | 93.230623 |
| 256 | Decryption |  8.00 KB | 512 | 93.609484 |
| 256 | Encryption |  8.00 KB | 1024 | 95.038248 |
| 256 | Decryption |  8.00 KB | 1024 | 94.848434 |
| 256 | Encryption |  8.00 KB | 2048 | 95.498725 |
| 256 | Decryption |  8.00 KB | 2048 | 95.718407 |
| 256 | Encryption |  8.00 KB | 4096 | 96.015706 |
| 256 | Decryption |  8.00 KB | 4096 | 96.002344 |
| 256 | Encryption |  16.00 KB | 256 | 90.076420 |
| 256 | Decryption |  16.00 KB | 256 | 90.000343 |
| 256 | Encryption |  16.00 KB | 512 | 92.694915 |
| 256 | Decryption |  16.00 KB | 512 | 92.590146 |
| 256 | Encryption |  16.00 KB | 1024 | 94.164302 |
| 256 | Decryption |  16.00 KB | 1024 | 94.055175 |
| 256 | Encryption |  16.00 KB | 2048 | 94.887916 |
| 256 | Decryption |  16.00 KB | 2048 | 94.554557 |
| 256 | Encryption |  16.00 KB | 4096 | 95.240241 |
| 256 | Decryption |  16.00 KB | 4096 | 94.990035 |
| 256 | Encryption |  16.00 KB | 8192 | 95.325438 |
| 256 | Decryption |  16.00 KB | 8192 | 95.213259 |
| 256 | Encryption |  32.00 KB | 256 | 90.168827 |
| 256 | Decryption |  32.00 KB | 256 | 90.201172 |
| 256 | Encryption |  32.00 KB | 512 | 92.946670 |
| 256 | Decryption |  32.00 KB | 512 | 92.888869 |
| 256 | Encryption |  32.00 KB | 1024 | 94.343744 |
| 256 | Decryption |  32.00 KB | 1024 | 94.252667 |
| 256 | Encryption |  32.00 KB | 2048 | 95.011208 |
| 256 | Decryption |  32.00 KB | 2048 | 95.041694 |
| 256 | Encryption |  32.00 KB | 4096 | 95.482029 |
| 256 | Decryption |  32.00 KB | 4096 | 95.250795 |
| 256 | Encryption |  32.00 KB | 8192 | 95.529002 |
| 256 | Decryption |  32.00 KB | 8192 | 95.591531 |
| 256 | Encryption |  32.00 KB | 16384 | 95.693424 |
| 256 | Decryption |  32.00 KB | 16384 | 95.584037 |
| 128 | Encryption |  512.00 B | 256 | 77.181082 |
| 128 | Decryption |  512.00 B | 256 | 80.598190 |
| 128 | Encryption |  1024.00 B | 256 | 88.756467 |
| 128 | Decryption |  1024.00 B | 256 | 91.072818 |
| 128 | Encryption |  1024.00 B | 512 | 94.718890 |
| 128 | Decryption |  1024.00 B | 512 | 94.982463 |
| 128 | Encryption |  2.00 KB | 256 | 95.794659 |
| 128 | Decryption |  2.00 KB | 256 | 96.962523 |
| 128 | Encryption |  2.00 KB | 512 | 100.492218 |
| 128 | Decryption |  2.00 KB | 512 | 100.662008 |
| 128 | Encryption |  2.00 KB | 1024 | 102.243440 |
| 128 | Decryption |  2.00 KB | 1024 | 102.324855 |
| 128 | Encryption |  4.00 KB | 256 | 99.963392 |
| 128 | Decryption |  4.00 KB | 256 | 99.871990 |
| 128 | Encryption |  4.00 KB | 512 | 103.450671 |
| 128 | Decryption |  4.00 KB | 512 | 103.548744 |
| 128 | Encryption |  4.00 KB | 1024 | 104.939873 |
| 128 | Decryption |  4.00 KB | 1024 | 105.354875 |
| 128 | Encryption |  4.00 KB | 2048 | 106.249899 |
| 128 | Decryption |  4.00 KB | 2048 | 105.985283 |
| 128 | Encryption |  8.00 KB | 256 | 97.754376 |
| 128 | Decryption |  8.00 KB | 256 | 98.186790 |
| 128 | Encryption |  8.00 KB | 512 | 101.081986 |
| 128 | Decryption |  8.00 KB | 512 | 101.097969 |
| 128 | Encryption |  8.00 KB | 1024 | 102.913765 |
| 128 | Decryption |  8.00 KB | 1024 | 102.712149 |
| 128 | Encryption |  8.00 KB | 2048 | 103.312866 |
| 128 | Decryption |  8.00 KB | 2048 | 103.843259 |
| 128 | Encryption |  8.00 KB | 4096 | 104.141109 |
| 128 | Decryption |  8.00 KB | 4096 | 104.197405 |
| 128 | Encryption |  16.00 KB | 256 | 97.093067 |
| 128 | Decryption |  16.00 KB | 256 | 97.110512 |
| 128 | Encryption |  16.00 KB | 512 | 100.419846 |
| 128 | Decryption |  16.00 KB | 512 | 100.031290 |
| 128 | Encryption |  16.00 KB | 1024 | 102.038084 |
| 128 | Decryption |  16.00 KB | 1024 | 101.828013 |
| 128 | Encryption |  16.00 KB | 2048 | 102.915785 |
| 128 | Decryption |  16.00 KB | 2048 | 102.425206 |
| 128 | Encryption |  16.00 KB | 4096 | 103.398033 |
| 128 | Decryption |  16.00 KB | 4096 | 102.961866 |
| 128 | Encryption |  16.00 KB | 8192 | 103.534839 |
| 128 | Decryption |  16.00 KB | 8192 | 103.103602 |
| 128 | Encryption |  32.00 KB | 256 | 97.273388 |
| 128 | Decryption |  32.00 KB | 256 | 97.447316 |
| 128 | Encryption |  32.00 KB | 512 | 100.593830 |
| 128 | Decryption |  32.00 KB | 512 | 100.506859 |
| 128 | Encryption |  32.00 KB | 1024 | 102.320062 |
| 128 | Decryption |  32.00 KB | 1024 | 102.053775 |
| 128 | Encryption |  32.00 KB | 2048 | 103.091640 |
| 128 | Decryption |  32.00 KB | 2048 | 102.898818 |
| 128 | Encryption |  32.00 KB | 4096 | 103.460878 |
| 128 | Decryption |  32.00 KB | 4096 | 103.357673 |
| 128 | Encryption |  32.00 KB | 8192 | 103.813855 |
| 128 | Decryption |  32.00 KB | 8192 | 103.584341 |
| 128 | Encryption |  32.00 KB | 16384 | 103.769063 |
| 128 | Decryption |  32.00 KB | 16384 | 103.807895 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.947353 |
| 256 | Decryption |  32.00 KB | 95.844744 |
| 256 | Encryption |  16.00 KB | 95.586128 |
| 256 | Decryption |  16.00 KB | 95.482376 |
| 256 | Encryption |  8.00 KB | 95.334798 |
| 256 | Decryption |  8.00 KB | 95.761415 |
| 256 | Encryption |  4.00 KB | 96.963958 |
| 256 | Decryption |  4.00 KB | 97.870434 |
| 256 | Encryption |  2.00 KB | 95.941910 |
| 256 | Decryption |  2.00 KB | 96.535470 |
| 256 | Encryption |  1024.00 B | 91.352105 |
| 256 | Decryption |  1024.00 B | 91.530726 |
| 256 | Encryption |  512.00 B | 83.357924 |
| 256 | Decryption |  512.00 B | 84.457962 |
| 128 | Encryption |  32.00 KB | 103.925801 |
| 128 | Decryption |  32.00 KB | 104.013737 |
| 128 | Encryption |  16.00 KB | 103.040380 |
| 128 | Decryption |  16.00 KB | 103.857041 |
| 128 | Encryption |  8.00 KB | 103.449038 |
| 128 | Decryption |  8.00 KB | 104.566904 |
| 128 | Encryption |  4.00 KB | 105.277024 |
| 128 | Decryption |  4.00 KB | 106.130315 |
| 128 | Encryption |  2.00 KB | 104.433184 |
| 128 | Decryption |  2.00 KB | 104.686751 |
| 128 | Encryption |  1024.00 B | 98.796997 |
| 128 | Decryption |  1024.00 B | 99.321048 |
| 128 | Encryption |  512.00 B | 89.305571 |
| 128 | Decryption |  512.00 B | 90.484343 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 308.935348 |
| 512 |  16.00 KB | 305.194775 |
| 512 |  8.00 KB | 297.384005 |
| 512 |  4.00 KB | 282.811893 |
| 512 |  2.00 KB | 257.427920 |
| 512 |  1024.00 B | 218.555326 |
| 512 |  512.00 B | 167.354443 |
| 256 |  32.00 KB | 299.299829 |
| 256 |  16.00 KB | 296.606737 |
| 256 |  8.00 KB | 291.089988 |
| 256 |  4.00 KB | 277.636094 |
| 256 |  2.00 KB | 260.684169 |
| 256 |  1024.00 B | 228.874764 |
| 256 |  512.00 B | 184.110574 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 250.061050 |
| 256 |  16.00 KB | 246.737698 |
| 256 |  8.00 KB | 247.651438 |
| 256 |  4.00 KB | 228.707032 |
| 256 |  2.00 KB | 198.401550 |
| 256 |  1024.00 B | 157.379569 |
| 256 |  512.00 B | 110.048361 |
| 128 |  32.00 KB | 250.851302 |
| 128 |  16.00 KB | 252.514365 |
| 128 |  8.00 KB | 247.539188 |
| 128 |  4.00 KB | 230.261933 |
| 128 |  2.00 KB | 200.030522 |
| 128 |  1024.00 B | 157.659738 |
| 128 |  512.00 B | 111.836177 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      203.884907      |
| 256        |      32.00 KB     |        1024        |      224.781880      |
| 256        |      32.00 KB     |        2048        |      235.757968      |
| 256        |      32.00 KB     |        4096        |      242.514848      |
| 256        |      32.00 KB     |        8192        |      247.130804      |
| 256        |      32.00 KB     |        16384        |      247.410693      |
| 256        |      16.00 KB     |        512        |      200.666736      |
| 256        |      16.00 KB     |        1024        |      220.061617      |
| 256        |      16.00 KB     |        2048        |      240.746455      |
| 256        |      16.00 KB     |        4096        |      249.085683      |
| 256        |      16.00 KB     |        8192        |      252.341783      |
| 256        |      8.00 KB     |        512        |      198.101687      |
| 256        |      8.00 KB     |        1024        |      221.418496      |
| 256        |      8.00 KB     |        2048        |      233.752430      |
| 256        |      8.00 KB     |        4096        |      241.723222      |
| 256        |      4.00 KB     |        512        |      186.229434      |
| 256        |      4.00 KB     |        1024        |      205.146184      |
| 256        |      4.00 KB     |        2048        |      217.669722      |
| 256        |      2.00 KB     |        512        |      164.899479      |
| 256        |      2.00 KB     |        1024        |      181.098707      |
| 256        |      1024.00 B     |        512        |      133.828875      |
| 128        |      32.00 KB     |        512        |      202.006627      |
| 128        |      32.00 KB     |        1024        |      223.061410      |
| 128        |      32.00 KB     |        2048        |      235.510156      |
| 128        |      32.00 KB     |        4096        |      243.065768      |
| 128        |      32.00 KB     |        8192        |      246.808551      |
| 128        |      32.00 KB     |        16384        |      247.688877      |
| 128        |      16.00 KB     |        512        |      195.364505      |
| 128        |      16.00 KB     |        1024        |      225.297689      |
| 128        |      16.00 KB     |        2048        |      238.473155      |
| 128        |      16.00 KB     |        4096        |      248.996960      |
| 128        |      16.00 KB     |        8192        |      251.457074      |
| 128        |      8.00 KB     |        512        |      198.209533      |
| 128        |      8.00 KB     |        1024        |      222.026103      |
| 128        |      8.00 KB     |        2048        |      234.475850      |
| 128        |      8.00 KB     |        4096        |      241.865958      |
| 128        |      4.00 KB     |        512        |      186.994607      |
| 128        |      4.00 KB     |        1024        |      205.574115      |
| 128        |      4.00 KB     |        2048        |      218.872840      |
| 128        |      2.00 KB     |        512        |      165.024048      |
| 128        |      2.00 KB     |        1024        |      182.292565      |
| 128        |      1024.00 B     |        512        |      134.825543      |

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 493			| 276.447998		|
cos  		|0.0000002870	| 65			| 65.120003 		| 497			| 277.664001		|
sincos sin  	|0.0000001790	| 79			| 78.961998 		| 477			| 275.350006		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.008003 		| 580			| 428.794006		|
acos 		|0.0000004770	| 76			| 76.000000 		| 826			| 383.996002		|
atan 		|0.0000005360	| 80			| 80.019997 		| 719			| 371.585999		|
atan2 		|0.0000007150	| 117			| 104.643997 		| 733			| 480.093994		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      921 |         8.686211 |
|        2 |     1830 |         8.743169 |
|        4 |     3654 |         8.757526 |
|        8 |     7302 |         8.764722 |
|       16 |    14594 |         8.770728 |
|       32 |    29186 |         8.771329 |
|       64 |    58370 |         8.771629 |
|      128 |   116727 |         8.772606 |
|      256 |   233452 |         8.772681 |
|      512 |   466877 |         8.773189 |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |       73 |       109.589041 |
|        2 |      128 |       125.000000 |
|        4 |      254 |       125.984252 |
|        8 |      494 |       129.554656 |
|       16 |      970 |       131.958763 |
|       32 |     1926 |       132.917965 |
|       64 |     3841 |       133.298620 |
|      128 |     7663 |       133.629127 |
|      256 |    15319 |       133.690189 |
|      512 |    30628 |       133.733838 |