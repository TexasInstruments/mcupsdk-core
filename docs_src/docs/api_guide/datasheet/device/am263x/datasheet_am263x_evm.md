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
- Size of ipc_rpmsg_echo              : 83 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   615
SBL : Drivers_open                      |   98
SBL : LoadHsmRtFw                       |   8898
SBL : Board_driversOpen                 |   3150
SBL : CPU Load                          |   6049
SBL : Total time taken                  |   18814

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_qspi appimage        : 227 KB
- Size of hello_world : 24.4 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   910
SBL : Drivers_open                      |   255029
SBL : LoadHsmRtFw                       |   8937
SBL : Board_driversOpen                 |   2983
SBL : File read from SD card            |   11878
SBL : CPU Load                          |   46
SBL : Total time taken                  |   283940

- Please note that the total time taken provided at the end is not including the ROM boot time.

### EDMA performance

EDMA Memory Copy Benchmark Numbers

Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48
    1024      |      TCMA     |     TCMA           |    47
    1024      |      TCMB     |     TCMB           |    46
    1024      |      OCRAM    |     TCMA           |    46
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

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 74
- End tick         : 9947475
- Total ticks      : 9947401
- Total time (secs): 9.947401
- Iterations/Sec   : 1507.931569
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1507.931569
CoreMark/MHz :3.769829 / STACK

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Size | Performance (Mbps) |
|------------|------|--------------------|
| 256 |  32.00 KB | 217.964276 |
| 256 |  16.00 KB | 219.874271 |
| 256 |  8.00 KB | 219.724071 |
| 256 |  4.00 KB | 207.586196 |
| 256 |  2.00 KB | 186.414837 |
| 256 |  1024.00 B | 154.580621 |
| 256 |  512.00 B | 115.461593 |
| 128 |  32.00 KB | 218.660605 |
| 128 |  16.00 KB | 218.787896 |
| 128 |  8.00 KB | 217.597450 |
| 128 |  4.00 KB | 208.097037 |
| 128 |  2.00 KB | 185.717524 |
| 128 |  1024.00 B | 156.082690 |
| 128 |  512.00 B | 118.006338 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 75.558015 |
| 256 | Decryption |  512.00 B | 256 | 81.427364 |
| 256 | Encryption |  1024.00 B | 256 | 88.713214 |
| 256 | Decryption |  1024.00 B | 256 | 89.740921 |
| 256 | Encryption |  1024.00 B | 512 | 91.546069 |
| 256 | Decryption |  1024.00 B | 512 | 92.483983 |
| 256 | Encryption |  2.00 KB | 256 | 93.022200 |
| 256 | Decryption |  2.00 KB | 256 | 93.689778 |
| 256 | Encryption |  2.00 KB | 512 | 96.102296 |
| 256 | Decryption |  2.00 KB | 512 | 96.742099 |
| 256 | Encryption |  2.00 KB | 1024 | 98.365478 |
| 256 | Decryption |  2.00 KB | 1024 | 98.170978 |
| 256 | Encryption |  4.00 KB | 256 | 95.492463 |
| 256 | Decryption |  4.00 KB | 256 | 95.998125 |
| 256 | Encryption |  4.00 KB | 512 | 98.473374 |
| 256 | Decryption |  4.00 KB | 512 | 98.736714 |
| 256 | Encryption |  4.00 KB | 1024 | 100.449090 |
| 256 | Decryption |  4.00 KB | 1024 | 100.158178 |
| 256 | Encryption |  4.00 KB | 2048 | 101.233443 |
| 256 | Decryption |  4.00 KB | 2048 | 101.246736 |
| 256 | Encryption |  8.00 KB | 256 | 93.437650 |
| 256 | Decryption |  8.00 KB | 256 | 93.267442 |
| 256 | Encryption |  8.00 KB | 512 | 96.473299 |
| 256 | Decryption |  8.00 KB | 512 | 96.022740 |
| 256 | Encryption |  8.00 KB | 1024 | 97.667331 |
| 256 | Decryption |  8.00 KB | 1024 | 97.592066 |
| 256 | Encryption |  8.00 KB | 2048 | 98.511490 |
| 256 | Decryption |  8.00 KB | 2048 | 98.226156 |
| 256 | Encryption |  8.00 KB | 4096 | 98.635291 |
| 256 | Decryption |  8.00 KB | 4096 | 98.719610 |
| 256 | Encryption |  16.00 KB | 256 | 92.675744 |
| 256 | Decryption |  16.00 KB | 256 | 92.412908 |
| 256 | Encryption |  16.00 KB | 512 | 95.360981 |
| 256 | Decryption |  16.00 KB | 512 | 94.997953 |
| 256 | Encryption |  16.00 KB | 1024 | 96.557338 |
| 256 | Decryption |  16.00 KB | 1024 | 96.434970 |
| 256 | Encryption |  16.00 KB | 2048 | 97.250113 |
| 256 | Decryption |  16.00 KB | 2048 | 97.106555 |
| 256 | Encryption |  16.00 KB | 4096 | 97.611144 |
| 256 | Decryption |  16.00 KB | 4096 | 97.404590 |
| 256 | Encryption |  16.00 KB | 8192 | 97.863309 |
| 256 | Decryption |  16.00 KB | 8192 | 97.567366 |
| 256 | Encryption |  32.00 KB | 256 | 92.663378 |
| 256 | Decryption |  32.00 KB | 256 | 92.624497 |
| 256 | Encryption |  32.00 KB | 512 | 95.393341 |
| 256 | Decryption |  32.00 KB | 512 | 95.272085 |
| 256 | Encryption |  32.00 KB | 1024 | 96.734156 |
| 256 | Decryption |  32.00 KB | 1024 | 96.600036 |
| 256 | Encryption |  32.00 KB | 2048 | 97.401786 |
| 256 | Decryption |  32.00 KB | 2048 | 97.329910 |
| 256 | Encryption |  32.00 KB | 4096 | 97.700182 |
| 256 | Decryption |  32.00 KB | 4096 | 97.535784 |
| 256 | Encryption |  32.00 KB | 8192 | 97.929389 |
| 256 | Decryption |  32.00 KB | 8192 | 97.812006 |
| 256 | Encryption |  32.00 KB | 16384 | 98.045314 |
| 256 | Decryption |  32.00 KB | 16384 | 97.827519 |
| 128 | Encryption |  512.00 B | 256 | 84.206198 |
| 128 | Decryption |  512.00 B | 256 | 86.536735 |
| 128 | Encryption |  1024.00 B | 256 | 94.584921 |
| 128 | Decryption |  1024.00 B | 256 | 95.720504 |
| 128 | Encryption |  1024.00 B | 512 | 100.561608 |
| 128 | Decryption |  1024.00 B | 512 | 100.419846 |
| 128 | Encryption |  2.00 KB | 256 | 101.587302 |
| 128 | Decryption |  2.00 KB | 256 | 101.666098 |
| 128 | Encryption |  2.00 KB | 512 | 105.309166 |
| 128 | Decryption |  2.00 KB | 512 | 104.458152 |
| 128 | Encryption |  2.00 KB | 1024 | 106.939934 |
| 128 | Decryption |  2.00 KB | 1024 | 106.894583 |
| 128 | Encryption |  4.00 KB | 256 | 102.083382 |
| 128 | Decryption |  4.00 KB | 256 | 103.492329 |
| 128 | Encryption |  4.00 KB | 512 | 107.753142 |
| 128 | Decryption |  4.00 KB | 512 | 107.145368 |
| 128 | Encryption |  4.00 KB | 1024 | 109.503162 |
| 128 | Decryption |  4.00 KB | 1024 | 109.482125 |
| 128 | Encryption |  4.00 KB | 2048 | 109.968034 |
| 128 | Decryption |  4.00 KB | 2048 | 110.329966 |
| 128 | Encryption |  8.00 KB | 256 | 101.185380 |
| 128 | Decryption |  8.00 KB | 256 | 100.792439 |
| 128 | Encryption |  8.00 KB | 512 | 104.426111 |
| 128 | Decryption |  8.00 KB | 512 | 104.752010 |
| 128 | Encryption |  8.00 KB | 1024 | 106.116997 |
| 128 | Decryption |  8.00 KB | 1024 | 106.074487 |
| 128 | Encryption |  8.00 KB | 2048 | 107.123914 |
| 128 | Decryption |  8.00 KB | 2048 | 106.817052 |
| 128 | Encryption |  8.00 KB | 4096 | 107.312480 |
| 128 | Decryption |  8.00 KB | 4096 | 106.908533 |
| 128 | Encryption |  16.00 KB | 256 | 100.281360 |
| 128 | Decryption |  16.00 KB | 256 | 100.008393 |
| 128 | Encryption |  16.00 KB | 512 | 103.282134 |
| 128 | Decryption |  16.00 KB | 512 | 103.150257 |
| 128 | Encryption |  16.00 KB | 1024 | 105.017857 |
| 128 | Decryption |  16.00 KB | 1024 | 104.824895 |
| 128 | Encryption |  16.00 KB | 2048 | 105.781922 |
| 128 | Decryption |  16.00 KB | 2048 | 105.275755 |
| 128 | Encryption |  16.00 KB | 4096 | 106.257005 |
| 128 | Decryption |  16.00 KB | 4096 | 105.742026 |
| 128 | Encryption |  16.00 KB | 8192 | 106.295782 |
| 128 | Decryption |  16.00 KB | 8192 | 106.079208 |
| 128 | Encryption |  32.00 KB | 256 | 100.100332 |
| 128 | Decryption |  32.00 KB | 256 | 100.297091 |
| 128 | Encryption |  32.00 KB | 512 | 103.518383 |
| 128 | Decryption |  32.00 KB | 512 | 103.313477 |
| 128 | Encryption |  32.00 KB | 1024 | 105.018594 |
| 128 | Decryption |  32.00 KB | 1024 | 104.927692 |
| 128 | Encryption |  32.00 KB | 2048 | 105.869821 |
| 128 | Decryption |  32.00 KB | 2048 | 105.741173 |
| 128 | Encryption |  32.00 KB | 4096 | 106.227833 |
| 128 | Decryption |  32.00 KB | 4096 | 106.153522 |
| 128 | Encryption |  32.00 KB | 8192 | 106.523738 |
| 128 | Decryption |  32.00 KB | 8192 | 106.313672 |
| 128 | Encryption |  32.00 KB | 16384 | 106.660699 |
| 128 | Decryption |  32.00 KB | 16384 | 106.512809 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 256 | Encryption |  32.00 KB | 98.288581 |
| 256 | Decryption |  32.00 KB | 98.210700 |
| 256 | Encryption |  16.00 KB | 98.728161 |
| 256 | Decryption |  16.00 KB | 98.008016 |
| 256 | Encryption |  8.00 KB | 98.620077 |
| 256 | Decryption |  8.00 KB | 99.337608 |
| 256 | Encryption |  4.00 KB | 100.295364 |
| 256 | Decryption |  4.00 KB | 101.261598 |
| 256 | Encryption |  2.00 KB | 99.495962 |
| 256 | Decryption |  2.00 KB | 100.335288 |
| 256 | Encryption |  1024.00 B | 96.903741 |
| 256 | Decryption |  1024.00 B | 96.694995 |
| 256 | Encryption |  512.00 B | 90.264999 |
| 256 | Decryption |  512.00 B | 89.947845 |
| 128 | Encryption |  32.00 KB | 106.632606 |
| 128 | Decryption |  32.00 KB | 106.595317 |
| 128 | Encryption |  16.00 KB | 107.085842 |
| 128 | Decryption |  16.00 KB | 106.547875 |
| 128 | Encryption |  8.00 KB | 106.969608 |
| 128 | Decryption |  8.00 KB | 106.906789 |
| 128 | Encryption |  4.00 KB | 109.215745 |
| 128 | Decryption |  4.00 KB | 110.937884 |
| 128 | Encryption |  2.00 KB | 109.012276 |
| 128 | Decryption |  2.00 KB | 108.618404 |
| 128 | Encryption |  1024.00 B | 103.877001 |
| 128 | Decryption |  1024.00 B | 104.924752 |
| 128 | Encryption |  512.00 B | 97.119146 |
| 128 | Decryption |  512.00 B | 96.700702 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 74.530319 |
| 256 | Decryption |  512.00 B | 256 | 80.364938 |
| 256 | Encryption |  1024.00 B | 256 | 88.983028 |
| 256 | Decryption |  1024.00 B | 256 | 89.271509 |
| 256 | Encryption |  1024.00 B | 512 | 91.057634 |
| 256 | Decryption |  1024.00 B | 512 | 91.915849 |
| 256 | Encryption |  2.00 KB | 256 | 91.819264 |
| 256 | Decryption |  2.00 KB | 256 | 92.874554 |
| 256 | Encryption |  2.00 KB | 512 | 95.925059 |
| 256 | Decryption |  2.00 KB | 512 | 96.356632 |
| 256 | Encryption |  2.00 KB | 1024 | 97.917227 |
| 256 | Decryption |  2.00 KB | 1024 | 97.797409 |
| 256 | Encryption |  4.00 KB | 256 | 94.110889 |
| 256 | Decryption |  4.00 KB | 256 | 95.064441 |
| 256 | Encryption |  4.00 KB | 512 | 98.349979 |
| 256 | Decryption |  4.00 KB | 512 | 98.714405 |
| 256 | Encryption |  4.00 KB | 1024 | 100.284621 |
| 256 | Decryption |  4.00 KB | 1024 | 99.863620 |
| 256 | Encryption |  4.00 KB | 2048 | 101.077309 |
| 256 | Decryption |  4.00 KB | 2048 | 100.951193 |
| 256 | Encryption |  8.00 KB | 256 | 92.872250 |
| 256 | Decryption |  8.00 KB | 256 | 93.097191 |
| 256 | Encryption |  8.00 KB | 512 | 96.220466 |
| 256 | Decryption |  8.00 KB | 512 | 96.002695 |
| 256 | Encryption |  8.00 KB | 1024 | 97.406762 |
| 256 | Decryption |  8.00 KB | 1024 | 97.416535 |
| 256 | Encryption |  8.00 KB | 2048 | 98.407943 |
| 256 | Decryption |  8.00 KB | 2048 | 98.123575 |
| 256 | Encryption |  8.00 KB | 4096 | 98.434549 |
| 256 | Decryption |  8.00 KB | 4096 | 98.679104 |
| 256 | Encryption |  16.00 KB | 256 | 92.538994 |
| 256 | Decryption |  16.00 KB | 256 | 92.445661 |
| 256 | Encryption |  16.00 KB | 512 | 95.336358 |
| 256 | Decryption |  16.00 KB | 512 | 94.902516 |
| 256 | Encryption |  16.00 KB | 1024 | 96.574768 |
| 256 | Decryption |  16.00 KB | 1024 | 96.253500 |
| 256 | Encryption |  16.00 KB | 2048 | 97.263463 |
| 256 | Decryption |  16.00 KB | 2048 | 96.840915 |
| 256 | Encryption |  16.00 KB | 4096 | 97.679887 |
| 256 | Decryption |  16.00 KB | 4096 | 97.048136 |
| 256 | Encryption |  16.00 KB | 8192 | 97.919239 |
| 256 | Decryption |  16.00 KB | 8192 | 97.322322 |
| 256 | Encryption |  32.00 KB | 256 | 92.542669 |
| 256 | Decryption |  32.00 KB | 256 | 92.635053 |
| 256 | Encryption |  32.00 KB | 512 | 95.349622 |
| 256 | Decryption |  32.00 KB | 512 | 95.213864 |
| 256 | Encryption |  32.00 KB | 1024 | 96.709175 |
| 256 | Decryption |  32.00 KB | 1024 | 96.535915 |
| 256 | Encryption |  32.00 KB | 2048 | 97.424772 |
| 256 | Decryption |  32.00 KB | 2048 | 97.208190 |
| 256 | Encryption |  32.00 KB | 4096 | 97.722945 |
| 256 | Decryption |  32.00 KB | 4096 | 97.621958 |
| 256 | Encryption |  32.00 KB | 8192 | 97.955005 |
| 256 | Decryption |  32.00 KB | 8192 | 97.716206 |
| 256 | Encryption |  32.00 KB | 16384 | 98.066770 |
| 256 | Decryption |  32.00 KB | 16384 | 97.914575 |
| 128 | Encryption |  512.00 B | 256 | 83.408848 |
| 128 | Decryption |  512.00 B | 256 | 85.883525 |
| 128 | Encryption |  1024.00 B | 256 | 96.169988 |
| 128 | Decryption |  1024.00 B | 256 | 96.254737 |
| 128 | Encryption |  1024.00 B | 512 | 98.621561 |
| 128 | Decryption |  1024.00 B | 512 | 99.498983 |
| 128 | Encryption |  2.00 KB | 256 | 99.025400 |
| 128 | Decryption |  2.00 KB | 256 | 99.817229 |
| 128 | Encryption |  2.00 KB | 512 | 104.101407 |
| 128 | Decryption |  2.00 KB | 512 | 104.167594 |
| 128 | Encryption |  2.00 KB | 1024 | 106.572999 |
| 128 | Decryption |  2.00 KB | 1024 | 106.417251 |
| 128 | Encryption |  4.00 KB | 256 | 103.010822 |
| 128 | Decryption |  4.00 KB | 256 | 103.813650 |
| 128 | Encryption |  4.00 KB | 512 | 107.565673 |
| 128 | Decryption |  4.00 KB | 512 | 107.017644 |
| 128 | Encryption |  4.00 KB | 1024 | 109.201186 |
| 128 | Decryption |  4.00 KB | 1024 | 109.176627 |
| 128 | Encryption |  4.00 KB | 2048 | 109.907175 |
| 128 | Decryption |  4.00 KB | 2048 | 110.125104 |
| 128 | Encryption |  8.00 KB | 256 | 100.955469 |
| 128 | Decryption |  8.00 KB | 256 | 100.992807 |
| 128 | Encryption |  8.00 KB | 512 | 104.541884 |
| 128 | Decryption |  8.00 KB | 512 | 104.480217 |
| 128 | Encryption |  8.00 KB | 1024 | 106.272728 |
| 128 | Decryption |  8.00 KB | 1024 | 105.973715 |
| 128 | Encryption |  8.00 KB | 2048 | 106.738329 |
| 128 | Decryption |  8.00 KB | 2048 | 106.573432 |
| 128 | Encryption |  8.00 KB | 4096 | 107.149309 |
| 128 | Decryption |  8.00 KB | 4096 | 107.103780 |
| 128 | Encryption |  16.00 KB | 256 | 100.109793 |
| 128 | Decryption |  16.00 KB | 256 | 100.063937 |
| 128 | Encryption |  16.00 KB | 512 | 103.361545 |
| 128 | Decryption |  16.00 KB | 512 | 103.078263 |
| 128 | Encryption |  16.00 KB | 1024 | 104.852357 |
| 128 | Decryption |  16.00 KB | 1024 | 104.595484 |
| 128 | Encryption |  16.00 KB | 2048 | 105.644653 |
| 128 | Decryption |  16.00 KB | 2048 | 105.559146 |
| 128 | Encryption |  16.00 KB | 4096 | 106.123655 |
| 128 | Decryption |  16.00 KB | 4096 | 105.659982 |
| 128 | Encryption |  16.00 KB | 8192 | 106.303110 |
| 128 | Decryption |  16.00 KB | 8192 | 105.913986 |
| 128 | Encryption |  32.00 KB | 256 | 100.196556 |
| 128 | Decryption |  32.00 KB | 256 | 100.289129 |
| 128 | Encryption |  32.00 KB | 512 | 103.435466 |
| 128 | Decryption |  32.00 KB | 512 | 103.217982 |
| 128 | Encryption |  32.00 KB | 1024 | 104.977800 |
| 128 | Decryption |  32.00 KB | 1024 | 104.979481 |
| 128 | Encryption |  32.00 KB | 2048 | 105.807860 |
| 128 | Decryption |  32.00 KB | 2048 | 105.699710 |
| 128 | Encryption |  32.00 KB | 4096 | 106.221376 |
| 128 | Decryption |  32.00 KB | 4096 | 106.065795 |
| 128 | Encryption |  32.00 KB | 8192 | 106.411528 |
| 128 | Decryption |  32.00 KB | 8192 | 106.326501 |
| 128 | Encryption |  32.00 KB | 16384 | 106.563360 |
| 128 | Decryption |  32.00 KB | 16384 | 106.477441 |

### SHA

- Software/Application used : test_dthe_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 310.793243 |
| 512 |  16.00 KB | 310.147002 |
| 512 |  8.00 KB | 306.447050 |
| 512 |  4.00 KB | 299.613688 |
| 512 |  2.00 KB | 285.921208 |
| 512 |  1024.00 B | 263.853772 |
| 512 |  512.00 B | 226.267090 |
| 256 |  32.00 KB | 300.904226 |
| 256 |  16.00 KB | 299.855302 |
| 256 |  8.00 KB | 297.515634 |
| 256 |  4.00 KB | 292.349556 |
| 256 |  2.00 KB | 282.130096 |
| 256 |  1024.00 B | 264.535400 |
| 256 |  512.00 B | 234.190966 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 74.717256 |
| 256 | Decryption |  512.00 B | 256 | 79.395232 |
| 256 | Encryption |  1024.00 B | 256 | 88.650813 |
| 256 | Decryption |  1024.00 B | 256 | 89.266645 |
| 256 | Encryption |  1024.00 B | 512 | 91.856586 |
| 256 | Decryption |  1024.00 B | 512 | 91.694650 |
| 256 | Encryption |  2.00 KB | 256 | 92.935137 |
| 256 | Decryption |  2.00 KB | 256 | 93.645600 |
| 256 | Encryption |  2.00 KB | 512 | 96.480045 |
| 256 | Decryption |  2.00 KB | 512 | 96.521252 |
| 256 | Encryption |  2.00 KB | 1024 | 97.760938 |
| 256 | Decryption |  2.00 KB | 1024 | 97.874819 |
| 256 | Encryption |  4.00 KB | 256 | 94.169038 |
| 256 | Decryption |  4.00 KB | 256 | 95.331331 |
| 256 | Encryption |  4.00 KB | 512 | 98.585214 |
| 256 | Decryption |  4.00 KB | 512 | 98.332271 |
| 256 | Encryption |  4.00 KB | 1024 | 100.234006 |
| 256 | Decryption |  4.00 KB | 1024 | 100.199525 |
| 256 | Encryption |  4.00 KB | 2048 | 100.792052 |
| 256 | Decryption |  4.00 KB | 2048 | 100.994753 |
| 256 | Encryption |  8.00 KB | 256 | 92.809449 |
| 256 | Decryption |  8.00 KB | 256 | 93.518984 |
| 256 | Encryption |  8.00 KB | 512 | 95.867527 |
| 256 | Decryption |  8.00 KB | 512 | 95.896635 |
| 256 | Encryption |  8.00 KB | 1024 | 97.350693 |
| 256 | Decryption |  8.00 KB | 1024 | 97.586616 |
| 256 | Encryption |  8.00 KB | 2048 | 98.262975 |
| 256 | Decryption |  8.00 KB | 2048 | 98.111823 |
| 256 | Encryption |  8.00 KB | 4096 | 98.568904 |
| 256 | Decryption |  8.00 KB | 4096 | 98.596709 |
| 256 | Encryption |  16.00 KB | 256 | 92.449084 |
| 256 | Decryption |  16.00 KB | 256 | 92.533441 |
| 256 | Encryption |  16.00 KB | 512 | 95.272431 |
| 256 | Decryption |  16.00 KB | 512 | 94.941355 |
| 256 | Encryption |  16.00 KB | 1024 | 96.623714 |
| 256 | Decryption |  16.00 KB | 1024 | 96.418476 |
| 256 | Encryption |  16.00 KB | 2048 | 97.352501 |
| 256 | Decryption |  16.00 KB | 2048 | 96.956965 |
| 256 | Encryption |  16.00 KB | 4096 | 97.580804 |
| 256 | Decryption |  16.00 KB | 4096 | 97.341837 |
| 256 | Encryption |  16.00 KB | 8192 | 97.717208 |
| 256 | Decryption |  16.00 KB | 8192 | 97.540683 |
| 256 | Encryption |  32.00 KB | 256 | 92.522827 |
| 256 | Decryption |  32.00 KB | 256 | 92.591863 |
| 256 | Encryption |  32.00 KB | 512 | 95.339305 |
| 256 | Decryption |  32.00 KB | 512 | 95.256593 |
| 256 | Encryption |  32.00 KB | 1024 | 96.731389 |
| 256 | Decryption |  32.00 KB | 1024 | 96.609559 |
| 256 | Encryption |  32.00 KB | 2048 | 97.332620 |
| 256 | Decryption |  32.00 KB | 2048 | 97.326387 |
| 256 | Encryption |  32.00 KB | 4096 | 97.756746 |
| 256 | Decryption |  32.00 KB | 4096 | 97.555838 |
| 256 | Encryption |  32.00 KB | 8192 | 97.899492 |
| 256 | Decryption |  32.00 KB | 8192 | 97.876189 |
| 256 | Encryption |  32.00 KB | 16384 | 98.058242 |
| 256 | Decryption |  32.00 KB | 16384 | 97.863401 |
| 128 | Encryption |  512.00 B | 256 | 82.948562 |
| 128 | Decryption |  512.00 B | 256 | 85.721760 |
| 128 | Encryption |  1024.00 B | 256 | 95.995313 |
| 128 | Decryption |  1024.00 B | 256 | 96.487147 |
| 128 | Encryption |  1024.00 B | 512 | 99.571546 |
| 128 | Decryption |  1024.00 B | 512 | 99.650275 |
| 128 | Encryption |  2.00 KB | 256 | 101.132681 |
| 128 | Decryption |  2.00 KB | 256 | 100.768805 |
| 128 | Encryption |  2.00 KB | 512 | 104.765406 |
| 128 | Decryption |  2.00 KB | 512 | 104.785508 |
| 128 | Encryption |  2.00 KB | 1024 | 106.450093 |
| 128 | Decryption |  2.00 KB | 1024 | 106.467387 |
| 128 | Encryption |  4.00 KB | 256 | 102.136679 |
| 128 | Decryption |  4.00 KB | 256 | 103.466187 |
| 128 | Encryption |  4.00 KB | 512 | 107.183920 |
| 128 | Decryption |  4.00 KB | 512 | 107.348075 |
| 128 | Encryption |  4.00 KB | 1024 | 108.919876 |
| 128 | Decryption |  4.00 KB | 1024 | 109.063072 |
| 128 | Encryption |  4.00 KB | 2048 | 110.042818 |
| 128 | Decryption |  4.00 KB | 2048 | 109.775544 |
| 128 | Encryption |  8.00 KB | 256 | 100.519963 |
| 128 | Decryption |  8.00 KB | 256 | 100.904174 |
| 128 | Encryption |  8.00 KB | 512 | 104.364997 |
| 128 | Decryption |  8.00 KB | 512 | 104.196577 |
| 128 | Encryption |  8.00 KB | 1024 | 105.615497 |
| 128 | Decryption |  8.00 KB | 1024 | 105.869714 |
| 128 | Encryption |  8.00 KB | 2048 | 107.204083 |
| 128 | Decryption |  8.00 KB | 2048 | 106.513675 |
| 128 | Encryption |  8.00 KB | 4096 | 107.261107 |
| 128 | Decryption |  8.00 KB | 4096 | 107.580241 |
| 128 | Encryption |  16.00 KB | 256 | 99.848786 |
| 128 | Decryption |  16.00 KB | 256 | 100.048279 |
| 128 | Encryption |  16.00 KB | 512 | 103.129561 |
| 128 | Decryption |  16.00 KB | 512 | 103.061243 |
| 128 | Encryption |  16.00 KB | 1024 | 104.772525 |
| 128 | Decryption |  16.00 KB | 1024 | 104.606336 |
| 128 | Encryption |  16.00 KB | 2048 | 105.598054 |
| 128 | Decryption |  16.00 KB | 2048 | 105.284000 |
| 128 | Encryption |  16.00 KB | 4096 | 105.906498 |
| 128 | Decryption |  16.00 KB | 4096 | 105.883612 |
| 128 | Encryption |  16.00 KB | 8192 | 106.158251 |
| 128 | Decryption |  16.00 KB | 8192 | 105.983784 |
| 128 | Encryption |  32.00 KB | 256 | 100.141726 |
| 128 | Decryption |  32.00 KB | 256 | 100.252501 |
| 128 | Encryption |  32.00 KB | 512 | 103.361953 |
| 128 | Decryption |  32.00 KB | 512 | 103.330886 |
| 128 | Encryption |  32.00 KB | 1024 | 104.993041 |
| 128 | Decryption |  32.00 KB | 1024 | 104.944599 |
| 128 | Encryption |  32.00 KB | 2048 | 105.735202 |
| 128 | Decryption |  32.00 KB | 2048 | 105.744479 |
| 128 | Encryption |  32.00 KB | 4096 | 106.258512 |
| 128 | Decryption |  32.00 KB | 4096 | 106.154704 |
| 128 | Encryption |  32.00 KB | 8192 | 106.516921 |
| 128 | Decryption |  32.00 KB | 8192 | 106.287701 |
| 128 | Encryption |  32.00 KB | 16384 | 106.545061 |
| 128 | Decryption |  32.00 KB | 16384 | 106.492148 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 256 | Encryption |  32.00 KB | 98.107601 |
| 256 | Decryption |  32.00 KB | 98.026707 |
| 256 | Encryption |  16.00 KB | 98.155172 |
| 256 | Decryption |  16.00 KB | 97.819305 |
| 256 | Encryption |  8.00 KB | 97.753647 |
| 256 | Decryption |  8.00 KB | 98.797742 |
| 256 | Encryption |  4.00 KB | 100.347578 |
| 256 | Decryption |  4.00 KB | 101.749755 |
| 256 | Encryption |  2.00 KB | 99.932906 |
| 256 | Decryption |  2.00 KB | 99.792910 |
| 256 | Encryption |  1024.00 B | 96.192573 |
| 256 | Decryption |  1024.00 B | 96.774956 |
| 256 | Encryption |  512.00 B | 89.799945 |
| 256 | Decryption |  512.00 B | 89.716351 |
| 128 | Encryption |  32.00 KB | 106.673177 |
| 128 | Decryption |  32.00 KB | 106.635426 |
| 128 | Encryption |  16.00 KB | 106.424164 |
| 128 | Decryption |  16.00 KB | 106.329843 |
| 128 | Encryption |  8.00 KB | 107.404372 |
| 128 | Decryption |  8.00 KB | 106.664063 |
| 128 | Encryption |  4.00 KB | 109.253980 |
| 128 | Decryption |  4.00 KB | 110.684006 |
| 128 | Encryption |  2.00 KB | 108.914445 |
| 128 | Decryption |  2.00 KB | 109.235770 |
| 128 | Encryption |  1024.00 B | 104.583174 |
| 128 | Decryption |  1024.00 B | 104.944914 |
| 128 | Encryption |  512.00 B | 96.632262 |
| 128 | Decryption |  512.00 B | 97.552843 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 308.705240 |
| 512 |  16.00 KB | 305.562970 |
| 512 |  8.00 KB | 297.823222 |
| 512 |  4.00 KB | 283.392792 |
| 512 |  2.00 KB | 258.636884 |
| 512 |  1024.00 B | 219.845689 |
| 512 |  512.00 B | 168.994327 |
| 256 |  32.00 KB | 299.246871 |
| 256 |  16.00 KB | 296.705772 |
| 256 |  8.00 KB | 291.222574 |
| 256 |  4.00 KB | 280.662084 |
| 256 |  2.00 KB | 261.214078 |
| 256 |  1024.00 B | 229.902477 |
| 256 |  512.00 B | 185.486245 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Size | Performance (Mbps) |
|------------|------|--------------------|
| 256 |  32.00 KB | 251.652603 |
| 256 |  16.00 KB | 254.628636 |
| 256 |  8.00 KB | 255.028699 |
| 256 |  4.00 KB | 239.549675 |
| 256 |  2.00 KB | 212.303606 |
| 256 |  1024.00 B | 173.421540 |
| 256 |  512.00 B | 126.409999 |
| 128 |  32.00 KB | 254.919579 |
| 128 |  16.00 KB | 259.180274 |
| 128 |  8.00 KB | 253.222955 |
| 128 |  4.00 KB | 240.521149 |
| 128 |  2.00 KB | 213.368061 |
| 128 |  1024.00 B | 174.939939 |
| 128 |  512.00 B | 128.512040 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) |
|------------|-------------------|--------------------|--------------------|
| 256        |      32.00 KB     |        512        |      217.244909      |
| 256        |      32.00 KB     |        1024        |      233.949046      |
| 256        |      32.00 KB     |        2048        |      244.686819      |
| 256        |      32.00 KB     |        4096        |      248.698135      |
| 256        |      32.00 KB     |        8192        |      249.598553      |
| 256        |      32.00 KB     |        16384        |      252.631782      |
| 256        |      16.00 KB     |        512        |      221.246571      |
| 256        |      16.00 KB     |        1024        |      239.750138      |
| 256        |      16.00 KB     |        2048        |      250.763116      |
| 256        |      16.00 KB     |        4096        |      255.464871      |
| 256        |      16.00 KB     |        8192        |      259.518966      |
| 256        |      8.00 KB     |        512        |      218.400553      |
| 256        |      8.00 KB     |        1024        |      236.929918      |
| 256        |      8.00 KB     |        2048        |      246.531180      |
| 256        |      8.00 KB     |        4096        |      251.382323      |
| 256        |      4.00 KB     |        512        |      207.711202      |
| 256        |      4.00 KB     |        1024        |      222.729744      |
| 256        |      4.00 KB     |        2048        |      231.310333      |
| 256        |      2.00 KB     |        512        |      186.224142      |
| 256        |      2.00 KB     |        1024        |      198.732450      |
| 256        |      1024.00 B     |        512        |      154.880182      |
| 128        |      32.00 KB     |        512        |      218.376447      |
| 128        |      32.00 KB     |        1024        |      236.450678      |
| 128        |      32.00 KB     |        2048        |      244.931443      |
| 128        |      32.00 KB     |        4096        |      249.572414      |
| 128        |      32.00 KB     |        8192        |      251.344361      |
| 128        |      32.00 KB     |        16384        |      253.831648      |
| 128        |      16.00 KB     |        512        |      215.754603      |
| 128        |      16.00 KB     |        1024        |      237.474012      |
| 128        |      16.00 KB     |        2048        |      250.071785      |
| 128        |      16.00 KB     |        4096        |      255.635520      |
| 128        |      16.00 KB     |        8192        |      259.135438      |
| 128        |      8.00 KB     |        512        |      219.659631      |
| 128        |      8.00 KB     |        1024        |      237.204336      |
| 128        |      8.00 KB     |        2048        |      245.632578      |
| 128        |      8.00 KB     |        4096        |      251.314843      |
| 128        |      4.00 KB     |        512        |      206.917673      |
| 128        |      4.00 KB     |        1024        |      223.401681      |
| 128        |      4.00 KB     |        2048        |      232.014586      |
| 128        |      2.00 KB     |        512        |      187.283171      |
| 128        |      2.00 KB     |        1024        |      200.000000      |
| 128        |      1024.00 B     |        512        |      156.373181      |

### AES ECB

- Software/Application used : test_dthe_aes_ecb
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 256 | Encryption |  32.00 KB | 98.253952 |
| 256 | Decryption |  32.00 KB | 98.078144 |
| 256 | Encryption |  16.00 KB | 97.878291 |
| 256 | Decryption |  16.00 KB | 97.877925 |
| 256 | Encryption |  8.00 KB | 99.123878 |
| 256 | Decryption |  8.00 KB | 98.775406 |
| 256 | Encryption |  4.00 KB | 101.000979 |
| 256 | Decryption |  4.00 KB | 101.399472 |
| 256 | Encryption |  2.00 KB | 100.547722 |
| 256 | Decryption |  2.00 KB | 100.734729 |
| 256 | Encryption |  1024.00 B | 97.759480 |
| 256 | Decryption |  1024.00 B | 97.506398 |
| 256 | Encryption |  512.00 B | 91.704914 |
| 256 | Decryption |  512.00 B | 91.321554 |
| 128 | Encryption |  32.00 KB | 106.651369 |
| 128 | Decryption |  32.00 KB | 106.647898 |
| 128 | Encryption |  16.00 KB | 106.839908 |
| 128 | Decryption |  16.00 KB | 106.450742 |
| 128 | Encryption |  8.00 KB | 106.422003 |
| 128 | Decryption |  8.00 KB | 106.698360 |
| 128 | Encryption |  4.00 KB | 109.990182 |
| 128 | Decryption |  4.00 KB | 110.140836 |
| 128 | Encryption |  2.00 KB | 109.747970 |
| 128 | Decryption |  2.00 KB | 109.933908 |
| 128 | Encryption |  1024.00 B | 106.227510 |
| 128 | Decryption |  1024.00 B | 106.083072 |
| 128 | Encryption |  512.00 B | 99.098772 |
| 128 | Decryption |  512.00 B | 99.182759 |


### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 517			| 275.247986		|
cos  		|0.0000002870	| 64			| 64.071999 		| 513			| 276.701996		|
sincos sin  	|0.0000001790	| 79			| 79.038002 		| 467			| 274.231995		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 73			| 73.038002 		| 580			| 428.742004		|
acos 		|0.0000004770	| 74			| 74.033997 		| 807			| 383.768005		|
atan 		|0.0000005360	| 85			| 85.150002 		| 492			| 371.135986		|
atan2 		|0.0000007150	| 119			| 106.744003 		| 893			| 478.372009		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

