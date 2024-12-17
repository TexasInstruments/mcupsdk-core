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

- Software/Application used        : sbl_ospi and ipc_notify_echo
- Size of sbl_ospi appimage        : 224 KB
- Size of ipc_notify_echo          : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   602
SBL : Drivers_open                      |   136
SBL : LoadHsmRtFw                       |   8388
SBL : Board_driversOpen                 |   2723
SBL : CPU Load                          |   5993
SBL : SBL End                           |   18
SBL : Total time taken                  |   17563

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI MULTICORE ELF performance

- Software/Application used           : sbl_ospi_multicore_elf and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 290 KB
- Size of ipc_notify_echo             : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   602
SBL : Drivers_open                      |   112
SBL : LoadHsmRtFw                       |   8410
SBL : Board_driversOpen                 |   2719
SBL : CPU Load                          |   5757
SBL : SBL End                           |   18
SBL : Total time taken                  |   17637

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 290 KB
- Size of ipc_notify_echo             : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   614
SBL : Drivers_open                      |   136
SBL : LoadHsmRtFw                       |   8388
SBL : Board_driversOpen                 |   2719
SBL : CPU Load                          |   5779
SBL : SBL End                           |   18
SBL : Total time taken                  |   17656

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a>. 

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_sd appimage          : 275 KB
- Size of hello_world              : 97 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   584
SBL : Drivers_open                      |   175352
SBL : LoadHsmRtFw                       |   8615
SBL : Board_driversOpen                 |   2823
SBL : File read from SD card            |   16442
SBL : CPU Load                          |   7693
SBL : SBL End                           |   5334
SBL : Total time taken                  |   216845

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD MULTICORE ELF performance

- Software/Application used        : sbl_sd_multicore_elf and hello_world
- Size of sbl_sd appimage          : 295 KB
- Size of hello_world              : 97 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   776
SBL : Drivers_open                      |   185172
SBL : LoadHsmRtFw                       |   8596
SBL : Board_driversOpen                 |   2859
SBL : File read from SD card            |   17182
SBL : CPU Load                          |   12967
SBL : SBL End                           |   15
SBL : Total time taken                  |   227499

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
 r5f0-0	| r5f0-1	|  1.74
 r5f0-0	| r5f1-0	|  1.87
 r5f0-0	| r5f1-1	|  1.93

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 10.512
 r5f0-0	| r5f1-0	| 4	| 10.647
 r5f0-0	| r5f1-1	| 4	| 10.611
 r5f0-0	| r5f0-1	| 32	| 13.702
 r5f0-0	| r5f0-1	| 64	| 16.674
 r5f0-0	| r5f0-1	| 112	| 20.892

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
 400		| 08			|  8.79 Mbps / 363.96 us 	| 22.89 Mbps / 139.82 us 	|  0.91 Mbps / 3526.71 us
 200		| 16			| 17.78 Mbps / 179.99 us 	| 29.83 Mbps / 107.26 us 	|  0.95 Mbps / 3370.77 us
 100		| 32			| 32.58 Mbps / 98.21 us 	| 35.15 Mbps / 91.04 us 	|  0.97 Mbps / 3293.03 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 71
- End tick         : 10245480
- Total ticks      : 10245409
- Total time (secs): 10.245409
- Iterations/Sec   : 1464.070395
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1464.070395 
CoreMark/MHz :3.660176 / STACK

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 193.123924 |
| 256 |  16.00 KB | 194.365748 |
| 256 |  8.00 KB | 191.823444 |
| 256 |  4.00 KB | 181.940839 |
| 256 |  2.00 KB | 163.967074 |
| 256 |  1024.00 B | 136.949889 |
| 256 |  512.00 B | 102.521745 |
| 128 |  32.00 KB | 194.017981 |
| 128 |  16.00 KB | 193.914288 |
| 128 |  8.00 KB | 191.826251 |
| 128 |  4.00 KB | 181.068685 |
| 128 |  2.00 KB | 163.983486 |
| 128 |  1024.00 B | 137.351721 |
| 128 |  512.00 B | 103.676517 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 87.926109 |
| 256 | Decryption |  32.00 KB | 88.026568 |
| 256 | Encryption |  16.00 KB | 87.871510 |
| 256 | Decryption |  16.00 KB | 87.937391 |
| 256 | Encryption |  8.00 KB | 87.963358 |
| 256 | Decryption |  8.00 KB | 87.989635 |
| 256 | Encryption |  4.00 KB | 89.078577 |
| 256 | Decryption |  4.00 KB | 90.291115 |
| 256 | Encryption |  2.00 KB | 88.611258 |
| 256 | Decryption |  2.00 KB | 89.224109 |
| 256 | Encryption |  1024.00 B | 86.488769 |
| 256 | Decryption |  1024.00 B | 86.486486 |
| 256 | Encryption |  512.00 B | 81.512438 |
| 256 | Decryption |  512.00 B | 81.165164 |
| 128 | Encryption |  32.00 KB | 95.554073 |
| 128 | Decryption |  32.00 KB | 95.496725 |
| 128 | Encryption |  16.00 KB | 95.507945 |
| 128 | Decryption |  16.00 KB | 95.340692 |
| 128 | Encryption |  8.00 KB | 96.289385 |
| 128 | Decryption |  8.00 KB | 96.403052 |
| 128 | Encryption |  4.00 KB | 97.456373 |
| 128 | Decryption |  4.00 KB | 98.478553 |
| 128 | Encryption |  2.00 KB | 96.873660 |
| 128 | Decryption |  2.00 KB | 97.028560 |
| 128 | Encryption |  1024.00 B | 93.425329 |
| 128 | Decryption |  1024.00 B | 93.896498 |
| 128 | Encryption |  512.00 B | 88.104969 |
| 128 | Decryption |  512.00 B | 88.019770 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 69.028860 |
| 256 | Decryption |  512.00 B | 256 | 72.495575 |
| 256 | Encryption |  1024.00 B | 256 | 78.531371 |
| 256 | Decryption |  1024.00 B | 256 | 79.518540 |
| 256 | Encryption |  1024.00 B | 512 | 82.187108 |
| 256 | Decryption |  1024.00 B | 512 | 82.094451 |
| 256 | Encryption |  2.00 KB | 256 | 82.433146 |
| 256 | Decryption |  2.00 KB | 256 | 83.261552 |
| 256 | Encryption |  2.00 KB | 512 | 85.816049 |
| 256 | Decryption |  2.00 KB | 512 | 85.729610 |
| 256 | Encryption |  2.00 KB | 1024 | 86.519598 |
| 256 | Decryption |  2.00 KB | 1024 | 86.990456 |
| 256 | Encryption |  4.00 KB | 256 | 83.925290 |
| 256 | Decryption |  4.00 KB | 256 | 84.997439 |
| 256 | Encryption |  4.00 KB | 512 | 87.422713 |
| 256 | Decryption |  4.00 KB | 512 | 87.640164 |
| 256 | Encryption |  4.00 KB | 1024 | 88.763079 |
| 256 | Decryption |  4.00 KB | 1024 | 88.900344 |
| 256 | Encryption |  4.00 KB | 2048 | 89.611464 |
| 256 | Decryption |  4.00 KB | 2048 | 89.367679 |
| 256 | Encryption |  8.00 KB | 256 | 83.575846 |
| 256 | Decryption |  8.00 KB | 256 | 83.697259 |
| 256 | Encryption |  8.00 KB | 512 | 86.114871 |
| 256 | Decryption |  8.00 KB | 512 | 86.067089 |
| 256 | Encryption |  8.00 KB | 1024 | 87.279217 |
| 256 | Decryption |  8.00 KB | 1024 | 87.242618 |
| 256 | Encryption |  8.00 KB | 2048 | 88.229216 |
| 256 | Decryption |  8.00 KB | 2048 | 87.767217 |
| 256 | Encryption |  8.00 KB | 4096 | 88.325533 |
| 256 | Decryption |  8.00 KB | 4096 | 88.198344 |
| 256 | Encryption |  16.00 KB | 256 | 83.053814 |
| 256 | Decryption |  16.00 KB | 256 | 83.137839 |
| 256 | Encryption |  16.00 KB | 512 | 85.459770 |
| 256 | Decryption |  16.00 KB | 512 | 85.219319 |
| 256 | Encryption |  16.00 KB | 1024 | 86.600915 |
| 256 | Decryption |  16.00 KB | 1024 | 86.455253 |
| 256 | Encryption |  16.00 KB | 2048 | 87.188211 |
| 256 | Decryption |  16.00 KB | 2048 | 87.115196 |
| 256 | Encryption |  16.00 KB | 4096 | 87.585846 |
| 256 | Decryption |  16.00 KB | 4096 | 87.283140 |
| 256 | Encryption |  16.00 KB | 8192 | 87.715089 |
| 256 | Decryption |  16.00 KB | 8192 | 87.601359 |
| 256 | Encryption |  32.00 KB | 256 | 83.211072 |
| 256 | Decryption |  32.00 KB | 256 | 83.219591 |
| 256 | Encryption |  32.00 KB | 512 | 85.572963 |
| 256 | Decryption |  32.00 KB | 512 | 85.459701 |
| 256 | Encryption |  32.00 KB | 1024 | 86.758049 |
| 256 | Decryption |  32.00 KB | 1024 | 86.653230 |
| 256 | Encryption |  32.00 KB | 2048 | 87.360876 |
| 256 | Decryption |  32.00 KB | 2048 | 87.238916 |
| 256 | Encryption |  32.00 KB | 4096 | 87.672626 |
| 256 | Decryption |  32.00 KB | 4096 | 87.603774 |
| 256 | Encryption |  32.00 KB | 8192 | 87.838018 |
| 256 | Decryption |  32.00 KB | 8192 | 87.682449 |
| 256 | Encryption |  32.00 KB | 16384 | 87.971106 |
| 256 | Decryption |  32.00 KB | 16384 | 87.829778 |
| 128 | Encryption |  512.00 B | 256 | 76.471412 |
| 128 | Decryption |  512.00 B | 256 | 78.478709 |
| 128 | Encryption |  1024.00 B | 256 | 84.573493 |
| 128 | Decryption |  1024.00 B | 256 | 85.939836 |
| 128 | Encryption |  1024.00 B | 512 | 88.775704 |
| 128 | Decryption |  1024.00 B | 512 | 88.416395 |
| 128 | Encryption |  2.00 KB | 256 | 88.515512 |
| 128 | Decryption |  2.00 KB | 256 | 90.139605 |
| 128 | Encryption |  2.00 KB | 512 | 92.958865 |
| 128 | Decryption |  2.00 KB | 512 | 92.933819 |
| 128 | Encryption |  2.00 KB | 1024 | 94.289619 |
| 128 | Decryption |  2.00 KB | 1024 | 93.566717 |
| 128 | Encryption |  4.00 KB | 256 | 92.022326 |
| 128 | Decryption |  4.00 KB | 256 | 92.273684 |
| 128 | Encryption |  4.00 KB | 512 | 95.043761 |
| 128 | Decryption |  4.00 KB | 512 | 94.729158 |
| 128 | Encryption |  4.00 KB | 1024 | 96.540447 |
| 128 | Decryption |  4.00 KB | 1024 | 96.367259 |
| 128 | Encryption |  4.00 KB | 2048 | 97.269057 |
| 128 | Decryption |  4.00 KB | 2048 | 97.292162 |
| 128 | Encryption |  8.00 KB | 256 | 89.749831 |
| 128 | Decryption |  8.00 KB | 256 | 90.259094 |
| 128 | Encryption |  8.00 KB | 512 | 93.464968 |
| 128 | Decryption |  8.00 KB | 512 | 93.280053 |
| 128 | Encryption |  8.00 KB | 1024 | 94.747990 |
| 128 | Decryption |  8.00 KB | 1024 | 94.605060 |
| 128 | Encryption |  8.00 KB | 2048 | 95.402454 |
| 128 | Decryption |  8.00 KB | 2048 | 95.735186 |
| 128 | Encryption |  8.00 KB | 4096 | 95.822322 |
| 128 | Decryption |  8.00 KB | 4096 | 96.010431 |
| 128 | Encryption |  16.00 KB | 256 | 89.894021 |
| 128 | Decryption |  16.00 KB | 256 | 89.915759 |
| 128 | Encryption |  16.00 KB | 512 | 92.638736 |
| 128 | Decryption |  16.00 KB | 512 | 92.458377 |
| 128 | Encryption |  16.00 KB | 1024 | 93.971053 |
| 128 | Decryption |  16.00 KB | 1024 | 93.766689 |
| 128 | Encryption |  16.00 KB | 2048 | 94.643144 |
| 128 | Decryption |  16.00 KB | 2048 | 94.470221 |
| 128 | Encryption |  16.00 KB | 4096 | 95.031185 |
| 128 | Decryption |  16.00 KB | 4096 | 94.654763 |
| 128 | Encryption |  16.00 KB | 8192 | 95.114628 |
| 128 | Decryption |  16.00 KB | 8192 | 94.927603 |
| 128 | Encryption |  32.00 KB | 256 | 90.032413 |
| 128 | Decryption |  32.00 KB | 256 | 90.042309 |
| 128 | Encryption |  32.00 KB | 512 | 92.727540 |
| 128 | Decryption |  32.00 KB | 512 | 92.673860 |
| 128 | Encryption |  32.00 KB | 1024 | 94.168107 |
| 128 | Decryption |  32.00 KB | 1024 | 93.962212 |
| 128 | Encryption |  32.00 KB | 2048 | 94.795186 |
| 128 | Decryption |  32.00 KB | 2048 | 94.608731 |
| 128 | Encryption |  32.00 KB | 4096 | 95.171000 |
| 128 | Decryption |  32.00 KB | 4096 | 95.015944 |
| 128 | Encryption |  32.00 KB | 8192 | 95.349795 |
| 128 | Decryption |  32.00 KB | 8192 | 95.242230 |
| 128 | Encryption |  32.00 KB | 16384 | 95.420251 |
| 128 | Decryption |  32.00 KB | 16384 | 95.360721 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 87.978487 |
| 256 | Decryption |  32.00 KB | 88.006768 |
| 256 | Encryption |  16.00 KB | 87.995690 |
| 256 | Decryption |  16.00 KB | 87.726097 |
| 256 | Encryption |  8.00 KB | 88.263973 |
| 256 | Decryption |  8.00 KB | 88.327617 |
| 256 | Encryption |  4.00 KB | 89.310439 |
| 256 | Decryption |  4.00 KB | 90.028161 |
| 256 | Encryption |  2.00 KB | 88.255653 |
| 256 | Decryption |  2.00 KB | 88.724024 |
| 256 | Encryption |  1024.00 B | 85.807060 |
| 256 | Decryption |  1024.00 B | 85.558370 |
| 256 | Encryption |  512.00 B | 80.207568 |
| 256 | Decryption |  512.00 B | 80.246853 |
| 128 | Encryption |  32.00 KB | 95.340952 |
| 128 | Decryption |  32.00 KB | 95.507945 |
| 128 | Encryption |  16.00 KB | 95.573060 |
| 128 | Decryption |  16.00 KB | 95.410440 |
| 128 | Encryption |  8.00 KB | 96.234242 |
| 128 | Decryption |  8.00 KB | 96.037515 |
| 128 | Encryption |  4.00 KB | 97.143619 |
| 128 | Decryption |  4.00 KB | 97.720122 |
| 128 | Encryption |  2.00 KB | 96.370802 |
| 128 | Decryption |  2.00 KB | 96.515567 |
| 128 | Encryption |  1024.00 B | 92.627770 |
| 128 | Decryption |  1024.00 B | 91.702348 |
| 128 | Encryption |  512.00 B | 85.906040 |
| 128 | Decryption |  512.00 B | 86.463666 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 68.110580 |
| 256 | Decryption |  512.00 B | 256 | 71.689857 |
| 256 | Encryption |  1024.00 B | 256 | 78.714358 |
| 256 | Decryption |  1024.00 B | 256 | 79.084810 |
| 256 | Encryption |  1024.00 B | 512 | 81.512438 |
| 256 | Decryption |  1024.00 B | 512 | 81.557071 |
| 256 | Encryption |  2.00 KB | 256 | 81.975333 |
| 256 | Decryption |  2.00 KB | 256 | 82.413451 |
| 256 | Encryption |  2.00 KB | 512 | 84.508059 |
| 256 | Decryption |  2.00 KB | 512 | 85.534920 |
| 256 | Encryption |  2.00 KB | 1024 | 86.130715 |
| 256 | Decryption |  2.00 KB | 1024 | 86.715359 |
| 256 | Encryption |  4.00 KB | 256 | 83.634507 |
| 256 | Decryption |  4.00 KB | 256 | 84.610618 |
| 256 | Encryption |  4.00 KB | 512 | 87.221427 |
| 256 | Decryption |  4.00 KB | 512 | 87.347559 |
| 256 | Encryption |  4.00 KB | 1024 | 88.526871 |
| 256 | Decryption |  4.00 KB | 1024 | 88.716217 |
| 256 | Encryption |  4.00 KB | 2048 | 89.399985 |
| 256 | Decryption |  4.00 KB | 2048 | 89.148858 |
| 256 | Encryption |  8.00 KB | 256 | 83.583041 |
| 256 | Decryption |  8.00 KB | 256 | 83.744318 |
| 256 | Encryption |  8.00 KB | 512 | 86.094790 |
| 256 | Decryption |  8.00 KB | 512 | 85.928286 |
| 256 | Encryption |  8.00 KB | 1024 | 87.123737 |
| 256 | Decryption |  8.00 KB | 1024 | 87.213593 |
| 256 | Encryption |  8.00 KB | 2048 | 87.934441 |
| 256 | Decryption |  8.00 KB | 2048 | 87.770743 |
| 256 | Encryption |  8.00 KB | 4096 | 88.219121 |
| 256 | Decryption |  8.00 KB | 4096 | 87.972213 |
| 256 | Encryption |  16.00 KB | 256 | 83.072633 |
| 256 | Decryption |  16.00 KB | 256 | 83.076450 |
| 256 | Encryption |  16.00 KB | 512 | 85.386978 |
| 256 | Decryption |  16.00 KB | 512 | 85.334861 |
| 256 | Encryption |  16.00 KB | 1024 | 86.626098 |
| 256 | Decryption |  16.00 KB | 1024 | 86.392001 |
| 256 | Encryption |  16.00 KB | 2048 | 87.198072 |
| 256 | Decryption |  16.00 KB | 2048 | 87.083942 |
| 256 | Encryption |  16.00 KB | 4096 | 87.548990 |
| 256 | Decryption |  16.00 KB | 4096 | 87.284593 |
| 256 | Encryption |  16.00 KB | 8192 | 87.614534 |
| 256 | Decryption |  16.00 KB | 8192 | 87.572095 |
| 256 | Encryption |  32.00 KB | 256 | 83.148322 |
| 256 | Decryption |  32.00 KB | 256 | 83.202422 |
| 256 | Encryption |  32.00 KB | 512 | 85.562489 |
| 256 | Decryption |  32.00 KB | 512 | 85.453293 |
| 256 | Encryption |  32.00 KB | 1024 | 86.712993 |
| 256 | Decryption |  32.00 KB | 1024 | 86.687186 |
| 256 | Encryption |  32.00 KB | 2048 | 87.392913 |
| 256 | Decryption |  32.00 KB | 2048 | 87.221500 |
| 256 | Encryption |  32.00 KB | 4096 | 87.651373 |
| 256 | Decryption |  32.00 KB | 4096 | 87.582554 |
| 256 | Encryption |  32.00 KB | 8192 | 87.784337 |
| 256 | Decryption |  32.00 KB | 8192 | 87.680250 |
| 256 | Encryption |  32.00 KB | 16384 | 87.856564 |
| 256 | Decryption |  32.00 KB | 16384 | 87.885724 |
| 128 | Encryption |  512.00 B | 256 | 74.543883 |
| 128 | Decryption |  512.00 B | 256 | 76.403656 |
| 128 | Encryption |  1024.00 B | 256 | 83.786341 |
| 128 | Decryption |  1024.00 B | 256 | 84.577859 |
| 128 | Encryption |  1024.00 B | 512 | 87.998496 |
| 128 | Decryption |  1024.00 B | 512 | 88.076551 |
| 128 | Encryption |  2.00 KB | 256 | 87.845156 |
| 128 | Decryption |  2.00 KB | 256 | 88.921453 |
| 128 | Encryption |  2.00 KB | 512 | 92.332836 |
| 128 | Decryption |  2.00 KB | 512 | 92.575432 |
| 128 | Encryption |  2.00 KB | 1024 | 93.362775 |
| 128 | Decryption |  2.00 KB | 1024 | 93.842717 |
| 128 | Encryption |  4.00 KB | 256 | 91.899737 |
| 128 | Decryption |  4.00 KB | 256 | 92.075981 |
| 128 | Encryption |  4.00 KB | 512 | 94.952876 |
| 128 | Decryption |  4.00 KB | 512 | 94.706571 |
| 128 | Encryption |  4.00 KB | 1024 | 96.293630 |
| 128 | Decryption |  4.00 KB | 1024 | 96.185514 |
| 128 | Encryption |  4.00 KB | 2048 | 97.082460 |
| 128 | Decryption |  4.00 KB | 2048 | 97.099001 |
| 128 | Encryption |  8.00 KB | 256 | 89.936736 |
| 128 | Decryption |  8.00 KB | 256 | 90.454994 |
| 128 | Encryption |  8.00 KB | 512 | 93.171971 |
| 128 | Decryption |  8.00 KB | 512 | 93.317243 |
| 128 | Encryption |  8.00 KB | 1024 | 94.639215 |
| 128 | Decryption |  8.00 KB | 1024 | 94.366669 |
| 128 | Encryption |  8.00 KB | 2048 | 95.341386 |
| 128 | Decryption |  8.00 KB | 2048 | 95.601466 |
| 128 | Encryption |  8.00 KB | 4096 | 95.741130 |
| 128 | Decryption |  8.00 KB | 4096 | 95.801661 |
| 128 | Encryption |  16.00 KB | 256 | 89.706066 |
| 128 | Decryption |  16.00 KB | 256 | 89.915296 |
| 128 | Encryption |  16.00 KB | 512 | 92.504543 |
| 128 | Decryption |  16.00 KB | 512 | 92.330396 |
| 128 | Encryption |  16.00 KB | 1024 | 93.977286 |
| 128 | Decryption |  16.00 KB | 1024 | 93.627873 |
| 128 | Encryption |  16.00 KB | 2048 | 94.676813 |
| 128 | Decryption |  16.00 KB | 2048 | 94.289959 |
| 128 | Encryption |  16.00 KB | 4096 | 94.993477 |
| 128 | Decryption |  16.00 KB | 4096 | 94.667240 |
| 128 | Encryption |  16.00 KB | 8192 | 95.148633 |
| 128 | Decryption |  16.00 KB | 8192 | 94.824761 |
| 128 | Encryption |  32.00 KB | 256 | 90.070849 |
| 128 | Decryption |  32.00 KB | 256 | 90.028084 |
| 128 | Encryption |  32.00 KB | 512 | 92.752885 |
| 128 | Decryption |  32.00 KB | 512 | 92.619670 |
| 128 | Encryption |  32.00 KB | 1024 | 94.079057 |
| 128 | Decryption |  32.00 KB | 1024 | 93.999948 |
| 128 | Encryption |  32.00 KB | 2048 | 94.786445 |
| 128 | Decryption |  32.00 KB | 2048 | 94.599342 |
| 128 | Encryption |  32.00 KB | 4096 | 95.105915 |
| 128 | Decryption |  32.00 KB | 4096 | 95.006990 |
| 128 | Encryption |  32.00 KB | 8192 | 95.264728 |
| 128 | Decryption |  32.00 KB | 8192 | 95.321711 |
| 128 | Encryption |  32.00 KB | 16384 | 95.398721 |
| 128 | Decryption |  32.00 KB | 16384 | 95.345980 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 284.574133 |
| 512 |  16.00 KB | 283.079116 |
| 512 |  8.00 KB | 279.987610 |
| 512 |  4.00 KB | 273.819671 |
| 512 |  2.00 KB | 262.060141 |
| 512 |  1024.00 B | 241.331566 |
| 512 |  512.00 B | 207.866024 |
| 256 |  32.00 KB | 275.003147 |
| 256 |  16.00 KB | 273.442650 |
| 256 |  8.00 KB | 270.642164 |
| 256 |  4.00 KB | 267.155844 |
| 256 |  2.00 KB | 258.076711 |
| 256 |  1024.00 B | 241.973121 |
| 256 |  512.00 B | 215.126050 |


### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 66.707382 |
| 256 | Decryption |  512.00 B | 256 | 70.748769 |
| 256 | Encryption |  1024.00 B | 256 | 77.830032 |
| 256 | Decryption |  1024.00 B | 256 | 78.167939 |
| 256 | Encryption |  1024.00 B | 512 | 81.118950 |
| 256 | Decryption |  1024.00 B | 512 | 81.471905 |
| 256 | Encryption |  2.00 KB | 256 | 81.613948 |
| 256 | Decryption |  2.00 KB | 256 | 82.607709 |
| 256 | Encryption |  2.00 KB | 512 | 85.366680 |
| 256 | Decryption |  2.00 KB | 512 | 85.380026 |
| 256 | Encryption |  2.00 KB | 1024 | 86.577891 |
| 256 | Decryption |  2.00 KB | 1024 | 86.429457 |
| 256 | Encryption |  4.00 KB | 256 | 84.403575 |
| 256 | Decryption |  4.00 KB | 256 | 84.855469 |
| 256 | Encryption |  4.00 KB | 512 | 87.263237 |
| 256 | Decryption |  4.00 KB | 512 | 87.137349 |
| 256 | Encryption |  4.00 KB | 1024 | 88.562761 |
| 256 | Decryption |  4.00 KB | 1024 | 88.437274 |
| 256 | Encryption |  4.00 KB | 2048 | 89.303745 |
| 256 | Decryption |  4.00 KB | 2048 | 89.056788 |
| 256 | Encryption |  8.00 KB | 256 | 83.239344 |
| 256 | Decryption |  8.00 KB | 256 | 83.536962 |
| 256 | Encryption |  8.00 KB | 512 | 86.009764 |
| 256 | Decryption |  8.00 KB | 512 | 85.893092 |
| 256 | Encryption |  8.00 KB | 1024 | 87.059214 |
| 256 | Decryption |  8.00 KB | 1024 | 87.073673 |
| 256 | Encryption |  8.00 KB | 2048 | 87.841623 |
| 256 | Decryption |  8.00 KB | 2048 | 87.678857 |
| 256 | Encryption |  8.00 KB | 4096 | 88.016815 |
| 256 | Decryption |  8.00 KB | 4096 | 87.997905 |
| 256 | Encryption |  16.00 KB | 256 | 82.869372 |
| 256 | Decryption |  16.00 KB | 256 | 83.056314 |
| 256 | Encryption |  16.00 KB | 512 | 85.449603 |
| 256 | Decryption |  16.00 KB | 512 | 85.213779 |
| 256 | Encryption |  16.00 KB | 1024 | 86.603347 |
| 256 | Decryption |  16.00 KB | 1024 | 86.421479 |
| 256 | Encryption |  16.00 KB | 2048 | 87.199667 |
| 256 | Decryption |  16.00 KB | 2048 | 87.061527 |
| 256 | Encryption |  16.00 KB | 4096 | 87.495953 |
| 256 | Decryption |  16.00 KB | 4096 | 87.331554 |
| 256 | Encryption |  16.00 KB | 8192 | 87.660972 |
| 256 | Decryption |  16.00 KB | 8192 | 87.491865 |
| 256 | Encryption |  32.00 KB | 256 | 83.129271 |
| 256 | Decryption |  32.00 KB | 256 | 83.018306 |
| 256 | Encryption |  32.00 KB | 512 | 85.539385 |
| 256 | Decryption |  32.00 KB | 512 | 85.432894 |
| 256 | Encryption |  32.00 KB | 1024 | 86.747785 |
| 256 | Decryption |  32.00 KB | 1024 | 86.566883 |
| 256 | Encryption |  32.00 KB | 2048 | 87.308431 |
| 256 | Decryption |  32.00 KB | 2048 | 87.195897 |
| 256 | Encryption |  32.00 KB | 4096 | 87.627274 |
| 256 | Decryption |  32.00 KB | 4096 | 87.495661 |
| 256 | Encryption |  32.00 KB | 8192 | 87.807051 |
| 256 | Decryption |  32.00 KB | 8192 | 87.708926 |
| 256 | Encryption |  32.00 KB | 16384 | 87.857668 |
| 256 | Decryption |  32.00 KB | 16384 | 87.881084 |
| 128 | Encryption |  512.00 B | 256 | 73.985098 |
| 128 | Decryption |  512.00 B | 256 | 74.550667 |
| 128 | Encryption |  1024.00 B | 256 | 82.933867 |
| 128 | Decryption |  1024.00 B | 256 | 84.370977 |
| 128 | Encryption |  1024.00 B | 512 | 87.582188 |
| 128 | Decryption |  1024.00 B | 512 | 87.751058 |
| 128 | Encryption |  2.00 KB | 256 | 89.167052 |
| 128 | Decryption |  2.00 KB | 256 | 89.609626 |
| 128 | Encryption |  2.00 KB | 512 | 91.648487 |
| 128 | Decryption |  2.00 KB | 512 | 92.173106 |
| 128 | Encryption |  2.00 KB | 1024 | 93.794367 |
| 128 | Decryption |  2.00 KB | 1024 | 93.876323 |
| 128 | Encryption |  4.00 KB | 256 | 90.796492 |
| 128 | Decryption |  4.00 KB | 256 | 91.882343 |
| 128 | Encryption |  4.00 KB | 512 | 94.707940 |
| 128 | Decryption |  4.00 KB | 512 | 94.862850 |
| 128 | Encryption |  4.00 KB | 1024 | 96.131900 |
| 128 | Decryption |  4.00 KB | 1024 | 96.355216 |
| 128 | Encryption |  4.00 KB | 2048 | 97.034306 |
| 128 | Decryption |  4.00 KB | 2048 | 96.792107 |
| 128 | Encryption |  8.00 KB | 256 | 89.984896 |
| 128 | Decryption |  8.00 KB | 256 | 90.413813 |
| 128 | Encryption |  8.00 KB | 512 | 92.975350 |
| 128 | Decryption |  8.00 KB | 512 | 93.332859 |
| 128 | Encryption |  8.00 KB | 1024 | 94.337803 |
| 128 | Decryption |  8.00 KB | 1024 | 94.861477 |
| 128 | Encryption |  8.00 KB | 2048 | 95.106519 |
| 128 | Decryption |  8.00 KB | 2048 | 95.088580 |
| 128 | Encryption |  8.00 KB | 4096 | 95.412523 |
| 128 | Decryption |  8.00 KB | 4096 | 95.755468 |
| 128 | Encryption |  16.00 KB | 256 | 89.720803 |
| 128 | Decryption |  16.00 KB | 256 | 89.583903 |
| 128 | Encryption |  16.00 KB | 512 | 92.611408 |
| 128 | Decryption |  16.00 KB | 512 | 92.199365 |
| 128 | Encryption |  16.00 KB | 1024 | 93.860525 |
| 128 | Decryption |  16.00 KB | 1024 | 93.672035 |
| 128 | Encryption |  16.00 KB | 2048 | 94.511604 |
| 128 | Decryption |  16.00 KB | 2048 | 94.372104 |
| 128 | Encryption |  16.00 KB | 4096 | 94.902688 |
| 128 | Decryption |  16.00 KB | 4096 | 94.800071 |
| 128 | Encryption |  16.00 KB | 8192 | 95.146216 |
| 128 | Decryption |  16.00 KB | 8192 | 94.801099 |
| 128 | Encryption |  32.00 KB | 256 | 89.891555 |
| 128 | Decryption |  32.00 KB | 256 | 89.856275 |
| 128 | Encryption |  32.00 KB | 512 | 92.674761 |
| 128 | Decryption |  32.00 KB | 512 | 92.568567 |
| 128 | Encryption |  32.00 KB | 1024 | 94.039823 |
| 128 | Decryption |  32.00 KB | 1024 | 93.934855 |
| 128 | Encryption |  32.00 KB | 2048 | 94.709993 |
| 128 | Decryption |  32.00 KB | 2048 | 94.708966 |
| 128 | Encryption |  32.00 KB | 4096 | 95.171951 |
| 128 | Decryption |  32.00 KB | 4096 | 94.939464 |
| 128 | Encryption |  32.00 KB | 8192 | 95.321798 |
| 128 | Decryption |  32.00 KB | 8192 | 95.196835 |
| 128 | Encryption |  32.00 KB | 16384 | 95.361155 |
| 128 | Decryption |  32.00 KB | 16384 | 95.275201 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 87.990890 |
| 256 | Decryption |  32.00 KB | 87.954135 |
| 256 | Encryption |  16.00 KB | 87.779119 |
| 256 | Decryption |  16.00 KB | 87.777209 |
| 256 | Encryption |  8.00 KB | 87.923234 |
| 256 | Decryption |  8.00 KB | 88.355899 |
| 256 | Encryption |  4.00 KB | 88.840088 |
| 256 | Decryption |  4.00 KB | 89.968220 |
| 256 | Encryption |  2.00 KB | 88.377048 |
| 256 | Decryption |  2.00 KB | 88.359175 |
| 256 | Encryption |  1024.00 B | 85.580715 |
| 256 | Decryption |  1024.00 B | 85.856522 |
| 256 | Encryption |  512.00 B | 80.066461 |
| 256 | Decryption |  512.00 B | 80.207568 |
| 128 | Encryption |  32.00 KB | 95.512034 |
| 128 | Decryption |  32.00 KB | 95.449522 |
| 128 | Encryption |  16.00 KB | 95.292172 |
| 128 | Decryption |  16.00 KB | 95.285418 |
| 128 | Encryption |  8.00 KB | 95.723999 |
| 128 | Decryption |  8.00 KB | 95.908564 |
| 128 | Encryption |  4.00 KB | 96.922372 |
| 128 | Decryption |  4.00 KB | 97.847054 |
| 128 | Encryption |  2.00 KB | 95.666010 |
| 128 | Decryption |  2.00 KB | 96.518409 |
| 128 | Encryption |  1024.00 B | 92.189962 |
| 128 | Decryption |  1024.00 B | 92.695898 |
| 128 | Encryption |  512.00 B | 85.538269 |
| 128 | Decryption |  512.00 B | 86.113739 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 282.718103 |
| 512 |  16.00 KB | 279.166156 |
| 512 |  8.00 KB | 272.052139 |
| 512 |  4.00 KB | 259.302050 |
| 512 |  2.00 KB | 237.002748 |
| 512 |  1024.00 B | 201.922603 |
| 512 |  512.00 B | 155.652670 |
| 256 |  32.00 KB | 273.636029 |
| 256 |  16.00 KB | 270.727413 |
| 256 |  8.00 KB | 266.328013 |
| 256 |  4.00 KB | 256.797477 |
| 256 |  2.00 KB | 239.707388 |
| 256 |  1024.00 B | 211.652241 |
| 256 |  512.00 B | 171.524288 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 230.432461 |
| 256 |  16.00 KB | 234.962377 |
| 256 |  8.00 KB | 229.351345 |
| 256 |  4.00 KB | 215.966123 |
| 256 |  2.00 KB | 190.939020 |
| 256 |  1024.00 B | 155.941560 |
| 256 |  512.00 B | 113.063281 |
| 128 |  32.00 KB | 230.521620 |
| 128 |  16.00 KB | 228.443949 |
| 128 |  8.00 KB | 230.320602 |
| 128 |  4.00 KB | 216.433289 |
| 128 |  2.00 KB | 192.193319 |
| 128 |  1024.00 B | 155.549226 |
| 128 |  512.00 B | 113.533366 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      193.399978      |
| 256        |      32.00 KB     |        1024        |      210.626793      |
| 256        |      32.00 KB     |        2048        |      220.879790      |
| 256        |      32.00 KB     |        4096        |      224.196021      |
| 256        |      32.00 KB     |        8192        |      229.014229      |
| 256        |      32.00 KB     |        16384        |      230.840829      |
| 256        |      16.00 KB     |        512        |      194.331888      |
| 256        |      16.00 KB     |        1024        |      211.083018      |
| 256        |      16.00 KB     |        2048        |      224.151450      |
| 256        |      16.00 KB     |        4096        |      229.590380      |
| 256        |      16.00 KB     |        8192        |      234.213983      |
| 256        |      8.00 KB     |        512        |      191.351572      |
| 256        |      8.00 KB     |        1024        |      210.248390      |
| 256        |      8.00 KB     |        2048        |      220.646932      |
| 256        |      8.00 KB     |        4096        |      225.172867      |
| 256        |      4.00 KB     |        512        |      181.133744      |
| 256        |      4.00 KB     |        1024        |      197.999940      |
| 256        |      4.00 KB     |        2048        |      207.195700      |
| 256        |      2.00 KB     |        512        |      163.840000      |
| 256        |      2.00 KB     |        1024        |      177.354406      |
| 256        |      1024.00 B     |        512        |      134.317101      |
| 128        |      32.00 KB     |        512        |      193.837935      |
| 128        |      32.00 KB     |        1024        |      210.691121      |
| 128        |      32.00 KB     |        2048        |      219.683561      |
| 128        |      32.00 KB     |        4096        |      224.732259      |
| 128        |      32.00 KB     |        8192        |      229.709581      |
| 128        |      32.00 KB     |        16384        |      230.877933      |
| 128        |      16.00 KB     |        512        |      191.983595      |
| 128        |      16.00 KB     |        1024        |      212.667881      |
| 128        |      16.00 KB     |        2048        |      224.177328      |
| 128        |      16.00 KB     |        4096        |      230.102260      |
| 128        |      16.00 KB     |        8192        |      233.097696      |
| 128        |      8.00 KB     |        512        |      192.718932      |
| 128        |      8.00 KB     |        1024        |      210.687735      |
| 128        |      8.00 KB     |        2048        |      219.952677      |
| 128        |      8.00 KB     |        4096        |      226.036870      |
| 128        |      4.00 KB     |        512        |      182.554075      |
| 128        |      4.00 KB     |        1024        |      198.675216      |
| 128        |      4.00 KB     |        2048        |      207.661840      |
| 128        |      2.00 KB     |        512        |      164.489734      |
| 128        |      2.00 KB     |        1024        |      177.835667      |
| 128        |      1024.00 B     |        512        |      137.669103      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    52   
    1024      |      TCMA     |     TCMA           |    50   
    1024      |      TCMB     |     TCMB           |    50   
    1024      |      OCRAM    |     TCMA           |    50   
    1024      |      TCMA     |     OCRAM          |    50   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 53			| 53.042000 		| 628			| 276.915985		|
cos  		|0.0000002870	| 65			| 65.073997 		| 497			| 277.488007		|
sincos sin  	|0.0000001790	| 79			| 78.961998 		| 600			| 275.812012		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 426			| 74.708000 		| 602			| 428.773987		|
acos 		|0.0000004770	| 433			| 76.713997 		| 552			| 383.252014		|
atan 		|0.0000005360	| 80			| 80.054001 		| 500			| 370.944000		|
atan2 		|0.0000007150	| 117			| 104.653999 		| 747			| 480.036011		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

