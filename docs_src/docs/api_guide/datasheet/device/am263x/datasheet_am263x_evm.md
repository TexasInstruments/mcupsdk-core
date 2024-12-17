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
- Size of ipc_notify_echo           : 83 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   160
SBL : Drivers_open                      |   37
SBL : LoadHsmRtFw                       |   9024
SBL : Board_driversOpen                 |   26
SBL : CPU Load                          |   7931
SBL : SBL End                           |   2904
SBL : Total time taken                  |   20085

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI MULTICORE ELF performance

- Software/Application used        : sbl_qspi and ipc_notify_echo
- Size of sbl_qspi appimage        : 253 KB
- Size of ipc_notify_echo          : 83 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   160
SBL : Drivers_open                      |   38
SBL : LoadHsmRtFw                       |   8840
SBL : Board_driversOpen                 |   26
SBL : CPU Load                          |   9305
SBL : SBL End                           |   5
SBL : Total time taken                  |   18216

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a> . 

### SBL SD performance

- Software/Application used        : sbl_sd and ipc_notify_echo
- Size of sbl_qspi appimage        : 762 KB
- Size of ipc_notify_echo          : 133 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   701
SBL : Drivers_open                      |   328388
SBL : LoadHsmRtFw                       |   9009
SBL : Board_driversOpen                 |   2987
SBL : File read from SD card            |   22938
SBL : CPU Load                          |   7769
SBL : SBL End                           |   4459
SBL : Total time taken                  |   376254

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD MULTICORE ELF performance

- Software/Application used        : sbl_sd_multicore_elf and ipc_notify_echo
- Size of sbl_qspi appimage        : 306 KB
- Size of ipc_notify_echo          : 133 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   700
SBL : Drivers_open                      |   334324
SBL : LoadHsmRtFw                       |   9019
SBL : Board_driversOpen                 |   2985
SBL : File read from SD card            |   24106
SBL : CPU Load                          |   19094
SBL : SBL End                           |   16
SBL : Total time taken                  |   390247

- Please note that the total time taken provided at the end is not including the ROM boot time.

### EDMA performance

EDMA Memory Copy Benchmark Numbers

Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48   
    1024      |      TCMA     |     TCMA           |    46   
    1024      |      TCMB     |     TCMB           |    46   
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
 r5f0-0	| r5f0-1	|  1.68
 r5f0-0	| r5f1-0	|  1.80
 r5f0-0	| r5f1-1	|  1.86

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 10.235
 r5f0-0	| r5f1-0	| 4	| 10.257
 r5f0-0	| r5f1-1	| 4	| 10.213
 r5f0-0	| r5f0-1	| 32	| 13.057
 r5f0-0	| r5f0-1	| 64	| 15.756
 r5f0-0	| r5f0-1	| 112	| 19.480

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
 400		| 08			|  8.74 Mbps / 365.96 us 	| 22.88 Mbps / 139.87 us 	|  0.91 Mbps / 3526.66 us
 200		| 16			| 17.76 Mbps / 180.16 us 	| 29.74 Mbps / 107.61 us 	|  0.95 Mbps / 3370.77 us
 100		| 32			| 32.42 Mbps / 98.71 us 	| 35.05 Mbps / 91.31 us 	|  0.97 Mbps / 3293.00 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 72
- End tick         : 10236140
- Total ticks      : 10236068
- Total time (secs): 10.236068
- Iterations/Sec   : 1465.406443
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.406443 
CoreMark/MHz :3.663516 / STACK

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 209.754214 |
| 256 |  16.00 KB | 210.531219 |
| 256 |  8.00 KB | 208.378245 |
| 256 |  4.00 KB | 197.154117 |
| 256 |  2.00 KB | 175.478619 |
| 256 |  1024.00 B | 147.424304 |
| 256 |  512.00 B | 110.174164 |
| 128 |  32.00 KB | 210.293929 |
| 128 |  16.00 KB | 210.417152 |
| 128 |  8.00 KB | 208.512500 |
| 128 |  4.00 KB | 197.534437 |
| 128 |  2.00 KB | 178.372935 |
| 128 |  1024.00 B | 147.743361 |
| 128 |  512.00 B | 111.289227 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.361935 |
| 256 | Decryption |  32.00 KB | 95.568617 |
| 256 | Encryption |  16.00 KB | 95.575673 |
| 256 | Decryption |  16.00 KB | 95.409919 |
| 256 | Encryption |  8.00 KB | 95.064441 |
| 256 | Decryption |  8.00 KB | 96.195750 |
| 256 | Encryption |  4.00 KB | 98.050539 |
| 256 | Decryption |  4.00 KB | 98.561492 |
| 256 | Encryption |  2.00 KB | 97.547035 |
| 256 | Decryption |  2.00 KB | 97.605147 |
| 256 | Encryption |  1024.00 B | 93.915336 |
| 256 | Decryption |  1024.00 B | 94.511263 |
| 256 | Encryption |  512.00 B | 88.071816 |
| 256 | Decryption |  512.00 B | 88.922659 |
| 128 | Encryption |  32.00 KB | 103.540769 |
| 128 | Decryption |  32.00 KB | 103.745141 |
| 128 | Encryption |  16.00 KB | 103.444547 |
| 128 | Decryption |  16.00 KB | 103.405171 |
| 128 | Encryption |  8.00 KB | 103.672417 |
| 128 | Decryption |  8.00 KB | 104.385361 |
| 128 | Encryption |  4.00 KB | 106.349902 |
| 128 | Decryption |  4.00 KB | 106.668403 |
| 128 | Encryption |  2.00 KB | 105.851760 |
| 128 | Decryption |  2.00 KB | 106.038444 |
| 128 | Encryption |  1024.00 B | 102.237060 |
| 128 | Decryption |  1024.00 B | 102.084177 |
| 128 | Encryption |  512.00 B | 96.065670 |
| 128 | Decryption |  512.00 B | 95.488985 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 74.334195 |
| 256 | Decryption |  512.00 B | 256 | 78.697344 |
| 256 | Encryption |  1024.00 B | 256 | 85.766634 |
| 256 | Decryption |  1024.00 B | 256 | 86.283803 |
| 256 | Encryption |  1024.00 B | 512 | 87.701737 |
| 256 | Decryption |  1024.00 B | 512 | 88.876834 |
| 256 | Encryption |  2.00 KB | 256 | 88.999946 |
| 256 | Decryption |  2.00 KB | 256 | 90.108621 |
| 256 | Encryption |  2.00 KB | 512 | 92.689343 |
| 256 | Decryption |  2.00 KB | 512 | 92.782513 |
| 256 | Encryption |  2.00 KB | 1024 | 94.664163 |
| 256 | Decryption |  2.00 KB | 1024 | 94.642290 |
| 256 | Encryption |  4.00 KB | 256 | 91.612615 |
| 256 | Decryption |  4.00 KB | 256 | 91.919716 |
| 256 | Encryption |  4.00 KB | 512 | 95.304990 |
| 256 | Decryption |  4.00 KB | 512 | 95.461862 |
| 256 | Encryption |  4.00 KB | 1024 | 97.078146 |
| 256 | Decryption |  4.00 KB | 1024 | 96.835728 |
| 256 | Encryption |  4.00 KB | 2048 | 97.973584 |
| 256 | Decryption |  4.00 KB | 2048 | 97.504222 |
| 256 | Encryption |  8.00 KB | 256 | 89.612996 |
| 256 | Decryption |  8.00 KB | 256 | 90.302934 |
| 256 | Encryption |  8.00 KB | 512 | 93.512979 |
| 256 | Decryption |  8.00 KB | 512 | 93.485300 |
| 256 | Encryption |  8.00 KB | 1024 | 94.768884 |
| 256 | Decryption |  8.00 KB | 1024 | 94.694939 |
| 256 | Encryption |  8.00 KB | 2048 | 95.531091 |
| 256 | Decryption |  8.00 KB | 2048 | 95.661821 |
| 256 | Encryption |  8.00 KB | 4096 | 96.168930 |
| 256 | Decryption |  8.00 KB | 4096 | 95.847898 |
| 256 | Encryption |  16.00 KB | 256 | 89.889860 |
| 256 | Decryption |  16.00 KB | 256 | 89.709596 |
| 256 | Encryption |  16.00 KB | 512 | 92.579519 |
| 256 | Decryption |  16.00 KB | 512 | 92.464410 |
| 256 | Encryption |  16.00 KB | 1024 | 93.905412 |
| 256 | Decryption |  16.00 KB | 1024 | 93.804100 |
| 256 | Encryption |  16.00 KB | 2048 | 94.675958 |
| 256 | Decryption |  16.00 KB | 2048 | 94.367008 |
| 256 | Encryption |  16.00 KB | 4096 | 95.185082 |
| 256 | Decryption |  16.00 KB | 4096 | 94.806242 |
| 256 | Encryption |  16.00 KB | 8192 | 95.309148 |
| 256 | Decryption |  16.00 KB | 8192 | 94.990035 |
| 256 | Encryption |  32.00 KB | 256 | 89.906585 |
| 256 | Decryption |  32.00 KB | 256 | 89.872294 |
| 256 | Encryption |  32.00 KB | 512 | 92.576086 |
| 256 | Decryption |  32.00 KB | 512 | 92.546345 |
| 256 | Encryption |  32.00 KB | 1024 | 94.066566 |
| 256 | Decryption |  32.00 KB | 1024 | 93.944617 |
| 256 | Encryption |  32.00 KB | 2048 | 94.770168 |
| 256 | Decryption |  32.00 KB | 2048 | 94.707426 |
| 256 | Encryption |  32.00 KB | 4096 | 95.120409 |
| 256 | Decryption |  32.00 KB | 4096 | 95.043934 |
| 256 | Encryption |  32.00 KB | 8192 | 95.345460 |
| 256 | Decryption |  32.00 KB | 8192 | 95.236607 |
| 256 | Encryption |  32.00 KB | 16384 | 95.392386 |
| 256 | Decryption |  32.00 KB | 16384 | 95.431106 |
| 128 | Encryption |  512.00 B | 256 | 81.475956 |
| 128 | Decryption |  512.00 B | 256 | 83.464086 |
| 128 | Encryption |  1024.00 B | 256 | 92.887717 |
| 128 | Decryption |  1024.00 B | 256 | 93.382730 |
| 128 | Encryption |  1024.00 B | 512 | 96.331138 |
| 128 | Decryption |  1024.00 B | 512 | 96.399153 |
| 128 | Encryption |  2.00 KB | 256 | 97.670606 |
| 128 | Decryption |  2.00 KB | 256 | 98.041738 |
| 128 | Encryption |  2.00 KB | 512 | 101.348509 |
| 128 | Decryption |  2.00 KB | 512 | 101.204522 |
| 128 | Encryption |  2.00 KB | 1024 | 102.862883 |
| 128 | Decryption |  2.00 KB | 1024 | 102.958227 |
| 128 | Encryption |  4.00 KB | 256 | 98.551858 |
| 128 | Decryption |  4.00 KB | 256 | 99.457458 |
| 128 | Encryption |  4.00 KB | 512 | 103.718358 |
| 128 | Decryption |  4.00 KB | 512 | 103.161623 |
| 128 | Encryption |  4.00 KB | 1024 | 105.504129 |
| 128 | Decryption |  4.00 KB | 1024 | 105.447261 |
| 128 | Encryption |  4.00 KB | 2048 | 106.066761 |
| 128 | Decryption |  4.00 KB | 2048 | 106.521898 |
| 128 | Encryption |  8.00 KB | 256 | 98.450075 |
| 128 | Decryption |  8.00 KB | 256 | 97.756563 |
| 128 | Encryption |  8.00 KB | 512 | 101.053930 |
| 128 | Decryption |  8.00 KB | 512 | 101.228752 |
| 128 | Encryption |  8.00 KB | 1024 | 102.881453 |
| 128 | Decryption |  8.00 KB | 1024 | 102.517735 |
| 128 | Encryption |  8.00 KB | 2048 | 103.777073 |
| 128 | Decryption |  8.00 KB | 2048 | 104.069998 |
| 128 | Encryption |  8.00 KB | 4096 | 104.468143 |
| 128 | Decryption |  8.00 KB | 4096 | 104.239253 |
| 128 | Encryption |  16.00 KB | 256 | 97.079044 |
| 128 | Decryption |  16.00 KB | 256 | 96.790320 |
| 128 | Encryption |  16.00 KB | 512 | 100.282511 |
| 128 | Decryption |  16.00 KB | 512 | 99.844602 |
| 128 | Encryption |  16.00 KB | 1024 | 101.874906 |
| 128 | Decryption |  16.00 KB | 1024 | 101.390452 |
| 128 | Encryption |  16.00 KB | 2048 | 102.651416 |
| 128 | Decryption |  16.00 KB | 2048 | 102.384402 |
| 128 | Encryption |  16.00 KB | 4096 | 103.143763 |
| 128 | Decryption |  16.00 KB | 4096 | 102.842302 |
| 128 | Encryption |  16.00 KB | 8192 | 103.312255 |
| 128 | Decryption |  16.00 KB | 8192 | 103.016084 |
| 128 | Encryption |  32.00 KB | 256 | 96.849502 |
| 128 | Decryption |  32.00 KB | 256 | 97.075270 |
| 128 | Encryption |  32.00 KB | 512 | 100.288650 |
| 128 | Decryption |  32.00 KB | 512 | 100.169851 |
| 128 | Encryption |  32.00 KB | 1024 | 101.878965 |
| 128 | Decryption |  32.00 KB | 1024 | 101.777211 |
| 128 | Encryption |  32.00 KB | 2048 | 102.758148 |
| 128 | Decryption |  32.00 KB | 2048 | 102.673328 |
| 128 | Encryption |  32.00 KB | 4096 | 103.184972 |
| 128 | Decryption |  32.00 KB | 4096 | 103.130170 |
| 128 | Encryption |  32.00 KB | 8192 | 103.444241 |
| 128 | Decryption |  32.00 KB | 8192 | 103.258640 |
| 128 | Encryption |  32.00 KB | 16384 | 103.518280 |
| 128 | Decryption |  32.00 KB | 16384 | 103.431691 |


### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.599549 |
| 256 | Decryption |  32.00 KB | 95.450391 |
| 256 | Encryption |  16.00 KB | 95.882254 |
| 256 | Decryption |  16.00 KB | 95.327864 |
| 256 | Encryption |  8.00 KB | 95.420164 |
| 256 | Decryption |  8.00 KB | 95.804462 |
| 256 | Encryption |  4.00 KB | 97.487542 |
| 256 | Decryption |  4.00 KB | 98.315306 |
| 256 | Encryption |  2.00 KB | 96.659341 |
| 256 | Decryption |  2.00 KB | 96.962523 |
| 256 | Encryption |  1024.00 B | 92.719504 |
| 256 | Decryption |  1024.00 B | 93.425329 |
| 256 | Encryption |  512.00 B | 86.977757 |
| 256 | Decryption |  512.00 B | 87.765160 |
| 128 | Encryption |  32.00 KB | 103.651101 |
| 128 | Decryption |  32.00 KB | 103.647003 |
| 128 | Encryption |  16.00 KB | 103.348709 |
| 128 | Decryption |  16.00 KB | 103.527888 |
| 128 | Encryption |  8.00 KB | 103.458020 |
| 128 | Decryption |  8.00 KB | 104.235522 |
| 128 | Encryption |  4.00 KB | 105.380286 |
| 128 | Decryption |  4.00 KB | 107.056104 |
| 128 | Encryption |  2.00 KB | 104.949956 |
| 128 | Decryption |  2.00 KB | 105.783416 |
| 128 | Encryption |  1024.00 B | 101.082765 |
| 128 | Decryption |  1024.00 B | 99.935954 |
| 128 | Encryption |  512.00 B | 93.564045 |
| 128 | Decryption |  512.00 B | 92.585895 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 72.791896 |
| 256 | Decryption |  512.00 B | 256 | 77.304898 |
| 256 | Encryption |  1024.00 B | 256 | 85.253408 |
| 256 | Decryption |  1024.00 B | 256 | 85.504788 |
| 256 | Encryption |  1024.00 B | 512 | 87.044760 |
| 256 | Decryption |  1024.00 B | 512 | 87.748708 |
| 256 | Encryption |  2.00 KB | 256 | 89.786412 |
| 256 | Decryption |  2.00 KB | 256 | 90.077658 |
| 256 | Encryption |  2.00 KB | 512 | 92.446150 |
| 256 | Decryption |  2.00 KB | 512 | 92.814049 |
| 256 | Encryption |  2.00 KB | 1024 | 94.406431 |
| 256 | Decryption |  2.00 KB | 1024 | 94.311330 |
| 256 | Encryption |  4.00 KB | 256 | 91.070920 |
| 256 | Decryption |  4.00 KB | 256 | 91.592129 |
| 256 | Encryption |  4.00 KB | 512 | 95.109969 |
| 256 | Decryption |  4.00 KB | 512 | 95.298061 |
| 256 | Encryption |  4.00 KB | 1024 | 96.872944 |
| 256 | Decryption |  4.00 KB | 1024 | 96.457325 |
| 256 | Encryption |  4.00 KB | 2048 | 97.655325 |
| 256 | Decryption |  4.00 KB | 2048 | 97.281330 |
| 256 | Encryption |  8.00 KB | 256 | 90.085397 |
| 256 | Decryption |  8.00 KB | 256 | 90.334675 |
| 256 | Encryption |  8.00 KB | 512 | 93.386389 |
| 256 | Decryption |  8.00 KB | 512 | 93.433654 |
| 256 | Encryption |  8.00 KB | 1024 | 94.869373 |
| 256 | Decryption |  8.00 KB | 1024 | 94.560356 |
| 256 | Encryption |  8.00 KB | 2048 | 95.565917 |
| 256 | Decryption |  8.00 KB | 2048 | 95.778909 |
| 256 | Encryption |  8.00 KB | 4096 | 96.171752 |
| 256 | Decryption |  8.00 KB | 4096 | 95.749173 |
| 256 | Encryption |  16.00 KB | 256 | 89.738771 |
| 256 | Decryption |  16.00 KB | 256 | 89.696704 |
| 256 | Encryption |  16.00 KB | 512 | 92.527725 |
| 256 | Decryption |  16.00 KB | 512 | 92.407696 |
| 256 | Encryption |  16.00 KB | 1024 | 93.886410 |
| 256 | Decryption |  16.00 KB | 1024 | 93.788998 |
| 256 | Encryption |  16.00 KB | 2048 | 94.704005 |
| 256 | Decryption |  16.00 KB | 2048 | 94.488439 |
| 256 | Encryption |  16.00 KB | 4096 | 95.119460 |
| 256 | Decryption |  16.00 KB | 4096 | 94.741483 |
| 256 | Encryption |  16.00 KB | 8192 | 95.257718 |
| 256 | Decryption |  16.00 KB | 8192 | 95.010348 |
| 256 | Encryption |  32.00 KB | 256 | 89.762892 |
| 256 | Decryption |  32.00 KB | 256 | 89.996481 |
| 256 | Encryption |  32.00 KB | 512 | 92.675498 |
| 256 | Decryption |  32.00 KB | 512 | 92.589492 |
| 256 | Encryption |  32.00 KB | 1024 | 94.150774 |
| 256 | Decryption |  32.00 KB | 1024 | 93.925768 |
| 256 | Encryption |  32.00 KB | 2048 | 94.767685 |
| 256 | Decryption |  32.00 KB | 2048 | 94.666898 |
| 256 | Encryption |  32.00 KB | 4096 | 95.198131 |
| 256 | Decryption |  32.00 KB | 4096 | 95.009831 |
| 256 | Encryption |  32.00 KB | 8192 | 95.309755 |
| 256 | Decryption |  32.00 KB | 8192 | 95.200465 |
| 256 | Encryption |  32.00 KB | 16384 | 95.373211 |
| 256 | Decryption |  32.00 KB | 16384 | 95.375033 |
| 128 | Encryption |  512.00 B | 256 | 80.136953 |
| 128 | Decryption |  512.00 B | 256 | 83.020015 |
| 128 | Encryption |  1024.00 B | 256 | 90.757513 |
| 128 | Decryption |  1024.00 B | 256 | 92.070806 |
| 128 | Encryption |  1024.00 B | 512 | 95.603209 |
| 128 | Decryption |  1024.00 B | 512 | 95.866124 |
| 128 | Encryption |  2.00 KB | 256 | 95.808663 |
| 128 | Decryption |  2.00 KB | 256 | 96.573879 |
| 128 | Encryption |  2.00 KB | 512 | 100.953525 |
| 128 | Decryption |  2.00 KB | 512 | 99.966442 |
| 128 | Encryption |  2.00 KB | 1024 | 102.451226 |
| 128 | Decryption |  2.00 KB | 1024 | 102.472051 |
| 128 | Encryption |  4.00 KB | 256 | 98.326369 |
| 128 | Decryption |  4.00 KB | 256 | 99.323305 |
| 128 | Encryption |  4.00 KB | 512 | 103.723282 |
| 128 | Decryption |  4.00 KB | 512 | 103.272166 |
| 128 | Encryption |  4.00 KB | 1024 | 105.492241 |
| 128 | Decryption |  4.00 KB | 1024 | 105.321859 |
| 128 | Encryption |  4.00 KB | 2048 | 106.003283 |
| 128 | Decryption |  4.00 KB | 2048 | 106.293842 |
| 128 | Encryption |  8.00 KB | 256 | 97.583710 |
| 128 | Decryption |  8.00 KB | 256 | 97.783547 |
| 128 | Encryption |  8.00 KB | 512 | 101.135412 |
| 128 | Decryption |  8.00 KB | 512 | 100.951581 |
| 128 | Encryption |  8.00 KB | 1024 | 103.032685 |
| 128 | Decryption |  8.00 KB | 1024 | 102.724626 |
| 128 | Encryption |  8.00 KB | 2048 | 103.591300 |
| 128 | Decryption |  8.00 KB | 2048 | 103.651101 |
| 128 | Encryption |  8.00 KB | 4096 | 104.485631 |
| 128 | Decryption |  8.00 KB | 4096 | 104.022920 |
| 128 | Encryption |  16.00 KB | 256 | 97.054963 |
| 128 | Decryption |  16.00 KB | 256 | 96.978665 |
| 128 | Encryption |  16.00 KB | 512 | 100.251255 |
| 128 | Decryption |  16.00 KB | 512 | 99.878079 |
| 128 | Encryption |  16.00 KB | 1024 | 101.721131 |
| 128 | Decryption |  16.00 KB | 1024 | 101.487799 |
| 128 | Encryption |  16.00 KB | 2048 | 102.744958 |
| 128 | Decryption |  16.00 KB | 2048 | 102.328450 |
| 128 | Encryption |  16.00 KB | 4096 | 102.947108 |
| 128 | Decryption |  16.00 KB | 4096 | 102.883876 |
| 128 | Encryption |  16.00 KB | 8192 | 103.291698 |
| 128 | Decryption |  16.00 KB | 8192 | 102.945693 |
| 128 | Encryption |  32.00 KB | 256 | 97.281330 |
| 128 | Decryption |  32.00 KB | 256 | 97.259133 |
| 128 | Encryption |  32.00 KB | 512 | 100.357663 |
| 128 | Decryption |  32.00 KB | 512 | 100.261991 |
| 128 | Encryption |  32.00 KB | 1024 | 101.981220 |
| 128 | Decryption |  32.00 KB | 1024 | 101.831375 |
| 128 | Encryption |  32.00 KB | 2048 | 102.830301 |
| 128 | Decryption |  32.00 KB | 2048 | 102.604006 |
| 128 | Encryption |  32.00 KB | 4096 | 103.320603 |
| 128 | Decryption |  32.00 KB | 4096 | 102.955800 |
| 128 | Encryption |  32.00 KB | 8192 | 103.384577 |
| 128 | Decryption |  32.00 KB | 8192 | 103.301060 |
| 128 | Encryption |  32.00 KB | 16384 | 103.521244 |
| 128 | Decryption |  32.00 KB | 16384 | 103.383353 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 311.418134 |
| 512 |  16.00 KB | 310.049794 |
| 512 |  8.00 KB | 306.436303 |
| 512 |  4.00 KB | 299.593143 |
| 512 |  2.00 KB | 286.533753 |
| 512 |  1024.00 B | 263.535467 |
| 512 |  512.00 B | 226.454734 |
| 256 |  32.00 KB | 300.906816 |
| 256 |  16.00 KB | 299.135039 |
| 256 |  8.00 KB | 297.208680 |
| 256 |  4.00 KB | 292.297400 |
| 256 |  2.00 KB | 278.308137 |
| 256 |  1024.00 B | 264.279377 |
| 256 |  512.00 B | 235.300876 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 70.699922 |
| 256 | Decryption |  512.00 B | 256 | 76.585799 |
| 256 | Encryption |  1024.00 B | 256 | 84.162942 |
| 256 | Decryption |  1024.00 B | 256 | 85.591892 |
| 256 | Encryption |  1024.00 B | 512 | 88.069449 |
| 256 | Decryption |  1024.00 B | 512 | 88.672404 |
| 256 | Encryption |  2.00 KB | 256 | 89.798715 |
| 256 | Decryption |  2.00 KB | 256 | 89.721264 |
| 256 | Encryption |  2.00 KB | 512 | 93.073722 |
| 256 | Decryption |  2.00 KB | 512 | 93.053899 |
| 256 | Encryption |  2.00 KB | 1024 | 94.008291 |
| 256 | Decryption |  2.00 KB | 1024 | 94.497635 |
| 256 | Encryption |  4.00 KB | 256 | 91.544790 |
| 256 | Decryption |  4.00 KB | 256 | 92.248357 |
| 256 | Encryption |  4.00 KB | 512 | 95.258583 |
| 256 | Decryption |  4.00 KB | 512 | 95.092029 |
| 256 | Encryption |  4.00 KB | 1024 | 96.885117 |
| 256 | Decryption |  4.00 KB | 1024 | 96.645087 |
| 256 | Encryption |  4.00 KB | 2048 | 97.644412 |
| 256 | Decryption |  4.00 KB | 2048 | 97.731780 |
| 256 | Encryption |  8.00 KB | 256 | 90.261891 |
| 256 | Decryption |  8.00 KB | 256 | 90.556235 |
| 256 | Encryption |  8.00 KB | 512 | 93.267110 |
| 256 | Decryption |  8.00 KB | 512 | 93.117363 |
| 256 | Encryption |  8.00 KB | 1024 | 94.619744 |
| 256 | Decryption |  8.00 KB | 1024 | 94.640240 |
| 256 | Encryption |  8.00 KB | 2048 | 95.491072 |
| 256 | Decryption |  8.00 KB | 2048 | 95.798860 |
| 256 | Encryption |  8.00 KB | 4096 | 95.918039 |
| 256 | Decryption |  8.00 KB | 4096 | 95.476465 |
| 256 | Encryption |  16.00 KB | 256 | 89.779032 |
| 256 | Decryption |  16.00 KB | 256 | 89.778110 |
| 256 | Encryption |  16.00 KB | 512 | 92.513031 |
| 256 | Decryption |  16.00 KB | 512 | 92.426919 |
| 256 | Encryption |  16.00 KB | 1024 | 93.925431 |
| 256 | Decryption |  16.00 KB | 1024 | 93.711883 |
| 256 | Encryption |  16.00 KB | 2048 | 94.649808 |
| 256 | Decryption |  16.00 KB | 2048 | 94.364970 |
| 256 | Encryption |  16.00 KB | 4096 | 95.037215 |
| 256 | Decryption |  16.00 KB | 4096 | 94.746277 |
| 256 | Encryption |  16.00 KB | 8192 | 95.211875 |
| 256 | Decryption |  16.00 KB | 8192 | 94.832651 |
| 256 | Encryption |  32.00 KB | 256 | 89.878765 |
| 256 | Decryption |  32.00 KB | 256 | 89.954481 |
| 256 | Encryption |  32.00 KB | 512 | 92.729672 |
| 256 | Decryption |  32.00 KB | 512 | 92.527236 |
| 256 | Encryption |  32.00 KB | 1024 | 94.031727 |
| 256 | Decryption |  32.00 KB | 1024 | 93.986889 |
| 256 | Encryption |  32.00 KB | 2048 | 94.817215 |
| 256 | Decryption |  32.00 KB | 2048 | 94.631015 |
| 256 | Encryption |  32.00 KB | 4096 | 95.052894 |
| 256 | Decryption |  32.00 KB | 4096 | 95.119978 |
| 256 | Encryption |  32.00 KB | 8192 | 95.280049 |
| 256 | Decryption |  32.00 KB | 8192 | 95.197872 |
| 256 | Encryption |  32.00 KB | 16384 | 95.386138 |
| 256 | Decryption |  32.00 KB | 16384 | 95.405058 |
| 128 | Encryption |  512.00 B | 256 | 79.906360 |
| 128 | Decryption |  512.00 B | 256 | 82.818582 |
| 128 | Encryption |  1024.00 B | 256 | 89.288536 |
| 128 | Decryption |  1024.00 B | 256 | 92.291227 |
| 128 | Encryption |  1024.00 B | 512 | 95.555815 |
| 128 | Decryption |  1024.00 B | 512 | 96.124850 |
| 128 | Encryption |  2.00 KB | 256 | 95.909616 |
| 128 | Decryption |  2.00 KB | 256 | 96.665044 |
| 128 | Encryption |  2.00 KB | 512 | 100.586303 |
| 128 | Decryption |  2.00 KB | 512 | 100.810657 |
| 128 | Encryption |  2.00 KB | 1024 | 102.605210 |
| 128 | Decryption |  2.00 KB | 1024 | 102.606817 |
| 128 | Encryption |  4.00 KB | 256 | 98.970069 |
| 128 | Decryption |  4.00 KB | 256 | 100.150525 |
| 128 | Encryption |  4.00 KB | 512 | 103.129967 |
| 128 | Decryption |  4.00 KB | 512 | 103.480890 |
| 128 | Encryption |  4.00 KB | 1024 | 105.393844 |
| 128 | Decryption |  4.00 KB | 1024 | 104.925592 |
| 128 | Encryption |  4.00 KB | 2048 | 106.289533 |
| 128 | Decryption |  4.00 KB | 2048 | 105.857744 |
| 128 | Encryption |  8.00 KB | 256 | 96.797468 |
| 128 | Decryption |  8.00 KB | 256 | 97.680069 |
| 128 | Encryption |  8.00 KB | 512 | 101.334405 |
| 128 | Decryption |  8.00 KB | 512 | 101.213900 |
| 128 | Encryption |  8.00 KB | 1024 | 102.946097 |
| 128 | Decryption |  8.00 KB | 1024 | 103.052127 |
| 128 | Encryption |  8.00 KB | 2048 | 103.845727 |
| 128 | Decryption |  8.00 KB | 2048 | 103.713433 |
| 128 | Encryption |  8.00 KB | 4096 | 104.165110 |
| 128 | Decryption |  8.00 KB | 4096 | 103.926419 |
| 128 | Encryption |  16.00 KB | 256 | 97.199810 |
| 128 | Decryption |  16.00 KB | 256 | 96.967545 |
| 128 | Encryption |  16.00 KB | 512 | 100.110366 |
| 128 | Decryption |  16.00 KB | 512 | 99.967204 |
| 128 | Encryption |  16.00 KB | 1024 | 101.874709 |
| 128 | Decryption |  16.00 KB | 1024 | 101.584546 |
| 128 | Encryption |  16.00 KB | 2048 | 102.651014 |
| 128 | Decryption |  16.00 KB | 2048 | 102.269367 |
| 128 | Encryption |  16.00 KB | 4096 | 103.052532 |
| 128 | Decryption |  16.00 KB | 4096 | 102.854811 |
| 128 | Encryption |  16.00 KB | 8192 | 103.286000 |
| 128 | Decryption |  16.00 KB | 8192 | 102.936799 |
| 128 | Encryption |  32.00 KB | 256 | 96.924343 |
| 128 | Decryption |  32.00 KB | 256 | 97.129942 |
| 128 | Encryption |  32.00 KB | 512 | 100.252597 |
| 128 | Decryption |  32.00 KB | 512 | 100.238605 |
| 128 | Encryption |  32.00 KB | 1024 | 101.901439 |
| 128 | Decryption |  32.00 KB | 1024 | 101.832562 |
| 128 | Encryption |  32.00 KB | 2048 | 102.697260 |
| 128 | Decryption |  32.00 KB | 2048 | 102.692030 |
| 128 | Encryption |  32.00 KB | 4096 | 103.162435 |
| 128 | Decryption |  32.00 KB | 4096 | 103.043620 |
| 128 | Encryption |  32.00 KB | 8192 | 103.437200 |
| 128 | Decryption |  32.00 KB | 8192 | 103.209144 |
| 128 | Encryption |  32.00 KB | 16384 | 103.481707 |
| 128 | Decryption |  32.00 KB | 16384 | 103.511740 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.424853 |
| 256 | Decryption |  32.00 KB | 95.497246 |
| 256 | Encryption |  16.00 KB | 95.329251 |
| 256 | Decryption |  16.00 KB | 95.234532 |
| 256 | Encryption |  8.00 KB | 95.989689 |
| 256 | Decryption |  8.00 KB | 96.145650 |
| 256 | Encryption |  4.00 KB | 97.218555 |
| 256 | Decryption |  4.00 KB | 98.165832 |
| 256 | Encryption |  2.00 KB | 96.692142 |
| 256 | Decryption |  2.00 KB | 97.116268 |
| 256 | Encryption |  1024.00 B | 93.396038 |
| 256 | Decryption |  1024.00 B | 93.380069 |
| 256 | Encryption |  512.00 B | 86.802649 |
| 256 | Decryption |  512.00 B | 87.732262 |
| 128 | Encryption |  32.00 KB | 103.591095 |
| 128 | Decryption |  32.00 KB | 103.664525 |
| 128 | Encryption |  16.00 KB | 103.354413 |
| 128 | Decryption |  16.00 KB | 103.470066 |
| 128 | Encryption |  8.00 KB | 103.300653 |
| 128 | Decryption |  8.00 KB | 104.187052 |
| 128 | Encryption |  4.00 KB | 106.206852 |
| 128 | Decryption |  4.00 KB | 106.978339 |
| 128 | Encryption |  2.00 KB | 105.290555 |
| 128 | Decryption |  2.00 KB | 105.780002 |
| 128 | Encryption |  1024.00 B | 100.740923 |
| 128 | Decryption |  1024.00 B | 101.335972 |
| 128 | Encryption |  512.00 B | 93.993460 |
| 128 | Decryption |  512.00 B | 94.328977 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 308.764326 |
| 512 |  16.00 KB | 305.098870 |
| 512 |  8.00 KB | 295.886947 |
| 512 |  4.00 KB | 282.081522 |
| 512 |  2.00 KB | 256.631554 |
| 512 |  1024.00 B | 216.633611 |
| 512 |  512.00 B | 165.294592 |
| 256 |  32.00 KB | 299.203324 |
| 256 |  16.00 KB | 296.325100 |
| 256 |  8.00 KB | 289.425221 |
| 256 |  4.00 KB | 279.703805 |
| 256 |  2.00 KB | 259.117508 |
| 256 |  1024.00 B | 227.350309 |
| 256 |  512.00 B | 182.145636 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 253.668921 |
| 256 |  16.00 KB | 255.042346 |
| 256 |  8.00 KB | 252.219175 |
| 256 |  4.00 KB | 237.854317 |
| 256 |  2.00 KB | 210.401952 |
| 256 |  1024.00 B | 170.037881 |
| 256 |  512.00 B | 122.524678 |
| 128 |  32.00 KB | 253.744425 |
| 128 |  16.00 KB | 256.995103 |
| 128 |  8.00 KB | 254.089367 |
| 128 |  4.00 KB | 238.912180 |
| 128 |  2.00 KB | 211.045632 |
| 128 |  1024.00 B | 171.605132 |
| 128 |  512.00 B | 124.300129 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      207.902292      |
| 256        |      32.00 KB     |        1024        |      227.897798      |
| 256        |      32.00 KB     |        2048        |      239.841170      |
| 256        |      32.00 KB     |        4096        |      245.942535      |
| 256        |      32.00 KB     |        8192        |      251.253420      |
| 256        |      32.00 KB     |        16384        |      252.813293      |
| 256        |      16.00 KB     |        512        |      206.666456      |
| 256        |      16.00 KB     |        1024        |      230.938443      |
| 256        |      16.00 KB     |        2048        |      246.131890      |
| 256        |      16.00 KB     |        4096        |      254.521093      |
| 256        |      16.00 KB     |        8192        |      256.793703      |
| 256        |      8.00 KB     |        512        |      207.976516      |
| 256        |      8.00 KB     |        1024        |      230.033609      |
| 256        |      8.00 KB     |        2048        |      241.224971      |
| 256        |      8.00 KB     |        4096        |      248.637985      |
| 256        |      4.00 KB     |        512        |      195.975001      |
| 256        |      4.00 KB     |        1024        |      216.080054      |
| 256        |      4.00 KB     |        2048        |      227.508158      |
| 256        |      2.00 KB     |        512        |      176.461402      |
| 256        |      2.00 KB     |        1024        |      192.906131      |
| 256        |      1024.00 B     |        512        |      142.985557      |
| 128        |      32.00 KB     |        512        |      208.717946      |
| 128        |      32.00 KB     |        1024        |      230.170953      |
| 128        |      32.00 KB     |        2048        |      241.150632      |
| 128        |      32.00 KB     |        4096        |      245.676317      |
| 128        |      32.00 KB     |        8192        |      251.467929      |
| 128        |      32.00 KB     |        16384        |      253.997045      |
| 128        |      16.00 KB     |        512        |      210.212142      |
| 128        |      16.00 KB     |        1024        |      235.254420      |
| 128        |      16.00 KB     |        2048        |      247.321993      |
| 128        |      16.00 KB     |        4096        |      253.466572      |
| 128        |      16.00 KB     |        8192        |      258.099580      |
| 128        |      8.00 KB     |        512        |      208.868031      |
| 128        |      8.00 KB     |        1024        |      229.660779      |
| 128        |      8.00 KB     |        2048        |      242.454287      |
| 128        |      8.00 KB     |        4096        |      249.136579      |
| 128        |      4.00 KB     |        512        |      197.686379      |
| 128        |      4.00 KB     |        1024        |      216.526250      |
| 128        |      4.00 KB     |        2048        |      227.891854      |
| 128        |      2.00 KB     |        512        |      177.512934      |
| 128        |      2.00 KB     |        1024        |      193.509906      |
| 128        |      1024.00 B     |        512        |      147.510579      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48   
    1024      |      TCMA     |     TCMA           |    46   
    1024      |      TCMB     |     TCMB           |    46   
    1024      |      OCRAM    |     TCMA           |    45   
    1024      |      TCMA     |     OCRAM          |    45   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 670			| 277.253998		|
cos  		|0.0000002870	| 462			| 65.834000 		| 497			| 277.526001		|
sincos sin  	|0.0000001790	| 79			| 78.958000 		| 467			| 276.196014		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 403			| 74.706001 		| 608			| 428.809998		|
acos 		|0.0000004770	| 76			| 76.000000 		| 767			| 384.022003		|
atan 		|0.0000005360	| 80			| 80.019997 		| 494			| 370.888000		|
atan2 		|0.0000007150	| 117			| 104.615997 		| 885			| 480.858002		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

