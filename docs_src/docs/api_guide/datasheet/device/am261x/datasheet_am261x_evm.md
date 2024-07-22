#  Datasheet {#DATASHEET_AM261X_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM261x

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

- Software/Application used        : sbl_ospi and ipc_rpmsg_echo
- Size of sbl_ospi appimage        : 217 KB
- Size of hello_world              : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   660
SBL : Drivers_open                      |   117
SBL : LoadHsmRtFw                       |   8496
SBL : Board_driversOpen                 |   33713
SBL : CPU Load                          |   2255
SBL : Total time taken                  |   46373

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_sd appimage        : 225 KB
- Size of hello_world : 24.5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   2301
SBL : Drivers_open                      |   260499
SBL : LoadHsmRtFw                       |   7165
SBL : Board_driversOpen                 |   2824
SBL : File read from SD card            |   11112
SBL : CPU Load                          |   3805
SBL : Total time taken                  |   287440

- Please note that the total time taken provided at the end is not including the ROM boot time.

### EDMA performance

EDMA Memory Copy Benchmark Numbers

Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    53
    1024      |      TCMA     |     TCMA           |    50
    1024      |      TCMB     |     TCMB           |    50
    1024      |      OCRAM    |     TCMA           |    50
    1024      |      TCMA     |     OCRAM          |    49

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
 r5f0-0	| r5f0-1	|  1.86
 r5f0-0	| r5f1-0	|  1.85
 r5f0-0	| r5f1-1	|  1.92

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 10.258
 r5f0-0	| r5f1-0	| 4	| 10.192
 r5f0-0	| r5f1-1	| 4	| 10.379
 r5f0-0	| r5f0-1	| 32	| 13.318
 r5f0-0	| r5f0-1	| 64	| 16.236
 r5f0-0	| r5f0-1	| 112	| 20.501

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 73
- End tick         : 9953469
- Total ticks      : 9953396
- Total time (secs): 9.953396
- Iterations/Sec   : 1507.023332
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1507.023332
CoreMark/MHz :3.767558 / STACK

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Size | Performance (Mbps) |
|------------|------|--------------------|
| 256 |  32.00 KB | 202.203346 |
| 256 |  16.00 KB | 202.550581 |
| 256 |  8.00 KB | 200.718207 |
| 256 |  4.00 KB | 190.727860 |
| 256 |  2.00 KB | 171.085470 |
| 256 |  1024.00 B | 142.655638 |
| 256 |  512.00 B | 106.959133 |
| 128 |  32.00 KB | 202.525152 |
| 128 |  16.00 KB | 201.774176 |
| 128 |  8.00 KB | 201.593405 |
| 128 |  4.00 KB | 190.922333 |
| 128 |  2.00 KB | 170.179174 |
| 128 |  1024.00 B | 143.354624 |
| 128 |  512.00 B | 108.180918 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 71.114198 |
| 256 | Decryption |  512.00 B | 256 | 76.031370 |
| 256 | Encryption |  1024.00 B | 256 | 81.275889 |
| 256 | Decryption |  1024.00 B | 256 | 82.764195 |
| 256 | Encryption |  1024.00 B | 512 | 85.315559 |
| 256 | Decryption |  1024.00 B | 512 | 85.111688 |
| 256 | Encryption |  2.00 KB | 256 | 86.537877 |
| 256 | Decryption |  2.00 KB | 256 | 86.935067 |
| 256 | Encryption |  2.00 KB | 512 | 89.014452 |
| 256 | Decryption |  2.00 KB | 512 | 88.766084 |
| 256 | Encryption |  2.00 KB | 1024 | 89.972543 |
| 256 | Decryption |  2.00 KB | 1024 | 89.424992 |
| 256 | Encryption |  4.00 KB | 256 | 87.253943 |
| 256 | Decryption |  4.00 KB | 256 | 88.345477 |
| 256 | Encryption |  4.00 KB | 512 | 90.539971 |
| 256 | Decryption |  4.00 KB | 512 | 90.800895 |
| 256 | Encryption |  4.00 KB | 1024 | 91.907470 |
| 256 | Decryption |  4.00 KB | 1024 | 91.638235 |
| 256 | Encryption |  4.00 KB | 2048 | 92.393365 |
| 256 | Decryption |  4.00 KB | 2048 | 92.384899 |
| 256 | Encryption |  8.00 KB | 256 | 87.107525 |
| 256 | Decryption |  8.00 KB | 256 | 87.151544 |
| 256 | Encryption |  8.00 KB | 512 | 89.052250 |
| 256 | Decryption |  8.00 KB | 512 | 88.909389 |
| 256 | Encryption |  8.00 KB | 1024 | 89.891401 |
| 256 | Decryption |  8.00 KB | 1024 | 89.794408 |
| 256 | Encryption |  8.00 KB | 2048 | 90.275257 |
| 256 | Decryption |  8.00 KB | 2048 | 90.296402 |
| 256 | Encryption |  8.00 KB | 4096 | 90.595355 |
| 256 | Decryption |  8.00 KB | 4096 | 90.346506 |
| 256 | Encryption |  16.00 KB | 256 | 86.414214 |
| 256 | Decryption |  16.00 KB | 256 | 86.256406 |
| 256 | Encryption |  16.00 KB | 512 | 88.231740 |
| 256 | Decryption |  16.00 KB | 512 | 88.129109 |
| 256 | Encryption |  16.00 KB | 1024 | 89.183282 |
| 256 | Decryption |  16.00 KB | 1024 | 88.837528 |
| 256 | Encryption |  16.00 KB | 2048 | 89.515227 |
| 256 | Decryption |  16.00 KB | 2048 | 89.312417 |
| 256 | Encryption |  16.00 KB | 4096 | 89.732781 |
| 256 | Decryption |  16.00 KB | 4096 | 89.501169 |
| 256 | Encryption |  16.00 KB | 8192 | 90.001734 |
| 256 | Decryption |  16.00 KB | 8192 | 89.681208 |
| 256 | Encryption |  32.00 KB | 256 | 86.434230 |
| 256 | Decryption |  32.00 KB | 256 | 86.511817 |
| 256 | Encryption |  32.00 KB | 512 | 88.373398 |
| 256 | Decryption |  32.00 KB | 512 | 88.215484 |
| 256 | Encryption |  32.00 KB | 1024 | 89.202324 |
| 256 | Decryption |  32.00 KB | 1024 | 89.161669 |
| 256 | Encryption |  32.00 KB | 2048 | 89.632529 |
| 256 | Decryption |  32.00 KB | 2048 | 89.576020 |
| 256 | Encryption |  32.00 KB | 4096 | 89.901651 |
| 256 | Decryption |  32.00 KB | 4096 | 89.832180 |
| 256 | Encryption |  32.00 KB | 8192 | 90.028393 |
| 256 | Decryption |  32.00 KB | 8192 | 89.981885 |
| 256 | Encryption |  32.00 KB | 16384 | 90.124963 |
| 256 | Decryption |  32.00 KB | 16384 | 89.939590 |
| 128 | Encryption |  512.00 B | 256 | 78.553963 |
| 128 | Decryption |  512.00 B | 256 | 79.157407 |
| 128 | Encryption |  1024.00 B | 256 | 89.468942 |
| 128 | Decryption |  1024.00 B | 256 | 89.364023 |
| 128 | Encryption |  1024.00 B | 512 | 91.319009 |
| 128 | Decryption |  1024.00 B | 512 | 91.838565 |
| 128 | Encryption |  2.00 KB | 256 | 93.481300 |
| 128 | Decryption |  2.00 KB | 256 | 92.879819 |
| 128 | Encryption |  2.00 KB | 512 | 96.319812 |
| 128 | Decryption |  2.00 KB | 512 | 96.198221 |
| 128 | Encryption |  2.00 KB | 1024 | 97.693902 |
| 128 | Decryption |  2.00 KB | 1024 | 97.593519 |
| 128 | Encryption |  4.00 KB | 256 | 95.481333 |
| 128 | Decryption |  4.00 KB | 256 | 95.059615 |
| 128 | Encryption |  4.00 KB | 512 | 98.398709 |
| 128 | Decryption |  4.00 KB | 512 | 98.107784 |
| 128 | Encryption |  4.00 KB | 1024 | 99.851448 |
| 128 | Decryption |  4.00 KB | 1024 | 99.881884 |
| 128 | Encryption |  4.00 KB | 2048 | 100.260841 |
| 128 | Decryption |  4.00 KB | 2048 | 100.624141 |
| 128 | Encryption |  8.00 KB | 256 | 93.870608 |
| 128 | Decryption |  8.00 KB | 256 | 93.363773 |
| 128 | Encryption |  8.00 KB | 512 | 96.030830 |
| 128 | Decryption |  8.00 KB | 512 | 96.030830 |
| 128 | Encryption |  8.00 KB | 1024 | 97.477754 |
| 128 | Decryption |  8.00 KB | 1024 | 97.326477 |
| 128 | Encryption |  8.00 KB | 2048 | 98.245667 |
| 128 | Decryption |  8.00 KB | 2048 | 98.204445 |
| 128 | Encryption |  8.00 KB | 4096 | 98.626385 |
| 128 | Decryption |  8.00 KB | 4096 | 98.571498 |
| 128 | Encryption |  16.00 KB | 256 | 92.460007 |
| 128 | Decryption |  16.00 KB | 256 | 92.416003 |
| 128 | Encryption |  16.00 KB | 512 | 95.310361 |
| 128 | Decryption |  16.00 KB | 512 | 95.041522 |
| 128 | Encryption |  16.00 KB | 1024 | 96.624604 |
| 128 | Decryption |  16.00 KB | 1024 | 96.272942 |
| 128 | Encryption |  16.00 KB | 2048 | 97.210443 |
| 128 | Decryption |  16.00 KB | 2048 | 97.155681 |
| 128 | Encryption |  16.00 KB | 4096 | 97.534514 |
| 128 | Decryption |  16.00 KB | 4096 | 97.429389 |
| 128 | Encryption |  16.00 KB | 8192 | 97.688987 |
| 128 | Decryption |  16.00 KB | 8192 | 97.435545 |
| 128 | Encryption |  32.00 KB | 256 | 92.681560 |
| 128 | Decryption |  32.00 KB | 256 | 92.827771 |
| 128 | Encryption |  32.00 KB | 512 | 95.355171 |
| 128 | Decryption |  32.00 KB | 512 | 95.312874 |
| 128 | Encryption |  32.00 KB | 1024 | 96.710424 |
| 128 | Decryption |  32.00 KB | 1024 | 96.658539 |
| 128 | Encryption |  32.00 KB | 2048 | 97.413006 |
| 128 | Decryption |  32.00 KB | 2048 | 97.270591 |
| 128 | Encryption |  32.00 KB | 4096 | 97.725860 |
| 128 | Decryption |  32.00 KB | 4096 | 97.607419 |
| 128 | Encryption |  32.00 KB | 8192 | 97.941464 |
| 128 | Decryption |  32.00 KB | 8192 | 97.731872 |
| 128 | Encryption |  32.00 KB | 16384 | 98.078328 |
| 128 | Decryption |  32.00 KB | 16384 | 97.886971 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 256 | Encryption |  32.00 KB | 90.033108 |
| 256 | Decryption |  32.00 KB | 90.143170 |
| 256 | Encryption |  16.00 KB | 90.035582 |
| 256 | Decryption |  16.00 KB | 89.827639 |
| 256 | Encryption |  8.00 KB | 91.003263 |
| 256 | Decryption |  8.00 KB | 90.459676 |
| 256 | Encryption |  4.00 KB | 91.743427 |
| 256 | Decryption |  4.00 KB | 92.494425 |
| 256 | Encryption |  2.00 KB | 91.349558 |
| 256 | Decryption |  2.00 KB | 91.452813 |
| 256 | Encryption |  1024.00 B | 88.454582 |
| 256 | Decryption |  1024.00 B | 88.211699 |
| 256 | Encryption |  512.00 B | 82.195354 |
| 256 | Decryption |  512.00 B | 82.174742 |
| 128 | Encryption |  32.00 KB | 98.038072 |
| 128 | Decryption |  32.00 KB | 98.051823 |
| 128 | Encryption |  16.00 KB | 98.237567 |
| 128 | Decryption |  16.00 KB | 97.841941 |
| 128 | Encryption |  8.00 KB | 98.084291 |
| 128 | Decryption |  8.00 KB | 98.560751 |
| 128 | Encryption |  4.00 KB | 99.975592 |
| 128 | Decryption |  4.00 KB | 100.965968 |
| 128 | Encryption |  2.00 KB | 99.411443 |
| 128 | Decryption |  2.00 KB | 99.753417 |
| 128 | Encryption |  1024.00 B | 94.858731 |
| 128 | Decryption |  1024.00 B | 95.706525 |
| 128 | Encryption |  512.00 B | 89.174332 |
| 128 | Decryption |  512.00 B | 88.961286 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 70.733497 |
| 256 | Decryption |  512.00 B | 256 | 75.432781 |
| 256 | Encryption |  1024.00 B | 256 | 80.808878 |
| 256 | Decryption |  1024.00 B | 256 | 82.082112 |
| 256 | Encryption |  1024.00 B | 512 | 84.501521 |
| 256 | Decryption |  1024.00 B | 512 | 84.169428 |
| 256 | Encryption |  2.00 KB | 256 | 85.193563 |
| 256 | Decryption |  2.00 KB | 256 | 86.614506 |
| 256 | Encryption |  2.00 KB | 512 | 88.660407 |
| 256 | Decryption |  2.00 KB | 512 | 88.537037 |
| 256 | Encryption |  2.00 KB | 1024 | 89.574107 |
| 256 | Decryption |  2.00 KB | 1024 | 88.998737 |
| 256 | Encryption |  4.00 KB | 256 | 87.141983 |
| 256 | Decryption |  4.00 KB | 256 | 87.935326 |
| 256 | Encryption |  4.00 KB | 512 | 90.421297 |
| 256 | Decryption |  4.00 KB | 512 | 90.647671 |
| 256 | Encryption |  4.00 KB | 1024 | 91.717107 |
| 256 | Decryption |  4.00 KB | 1024 | 91.431123 |
| 256 | Encryption |  4.00 KB | 2048 | 92.273034 |
| 256 | Decryption |  4.00 KB | 2048 | 92.196446 |
| 256 | Encryption |  8.00 KB | 256 | 86.868520 |
| 256 | Decryption |  8.00 KB | 256 | 86.750082 |
| 256 | Encryption |  8.00 KB | 512 | 88.904263 |
| 256 | Decryption |  8.00 KB | 512 | 89.026846 |
| 256 | Encryption |  8.00 KB | 1024 | 89.557277 |
| 256 | Decryption |  8.00 KB | 1024 | 89.648238 |
| 256 | Encryption |  8.00 KB | 2048 | 89.982116 |
| 256 | Decryption |  8.00 KB | 2048 | 90.089112 |
| 256 | Encryption |  8.00 KB | 4096 | 90.310090 |
| 256 | Decryption |  8.00 KB | 4096 | 90.406017 |
| 256 | Encryption |  16.00 KB | 256 | 86.322018 |
| 256 | Decryption |  16.00 KB | 256 | 86.301273 |
| 256 | Encryption |  16.00 KB | 512 | 88.123480 |
| 256 | Decryption |  16.00 KB | 512 | 88.084690 |
| 256 | Encryption |  16.00 KB | 1024 | 89.013998 |
| 256 | Decryption |  16.00 KB | 1024 | 88.894465 |
| 256 | Encryption |  16.00 KB | 2048 | 89.542440 |
| 256 | Decryption |  16.00 KB | 2048 | 89.233979 |
| 256 | Encryption |  16.00 KB | 4096 | 89.781338 |
| 256 | Decryption |  16.00 KB | 4096 | 89.491391 |
| 256 | Encryption |  16.00 KB | 8192 | 89.793024 |
| 256 | Decryption |  16.00 KB | 8192 | 89.759819 |
| 256 | Encryption |  32.00 KB | 256 | 86.413858 |
| 256 | Decryption |  32.00 KB | 256 | 86.550806 |
| 256 | Encryption |  32.00 KB | 512 | 88.311474 |
| 256 | Decryption |  32.00 KB | 512 | 88.292884 |
| 256 | Encryption |  32.00 KB | 1024 | 89.226918 |
| 256 | Decryption |  32.00 KB | 1024 | 89.097121 |
| 256 | Encryption |  32.00 KB | 2048 | 89.686961 |
| 256 | Decryption |  32.00 KB | 2048 | 89.482761 |
| 256 | Encryption |  32.00 KB | 4096 | 89.864823 |
| 256 | Decryption |  32.00 KB | 4096 | 89.809098 |
| 256 | Encryption |  32.00 KB | 8192 | 90.014018 |
| 256 | Decryption |  32.00 KB | 8192 | 89.923547 |
| 256 | Encryption |  32.00 KB | 16384 | 90.161229 |
| 256 | Decryption |  32.00 KB | 16384 | 89.963048 |
| 128 | Encryption |  512.00 B | 256 | 77.454735 |
| 128 | Decryption |  512.00 B | 256 | 79.065727 |
| 128 | Encryption |  1024.00 B | 256 | 88.732433 |
| 128 | Decryption |  1024.00 B | 256 | 88.581315 |
| 128 | Encryption |  1024.00 B | 512 | 91.230024 |
| 128 | Decryption |  1024.00 B | 512 | 91.055103 |
| 128 | Encryption |  2.00 KB | 256 | 91.834704 |
| 128 | Decryption |  2.00 KB | 256 | 91.789686 |
| 128 | Encryption |  2.00 KB | 512 | 95.814266 |
| 128 | Decryption |  2.00 KB | 512 | 95.688359 |
| 128 | Encryption |  2.00 KB | 1024 | 97.219997 |
| 128 | Decryption |  2.00 KB | 1024 | 97.123464 |
| 128 | Encryption |  4.00 KB | 256 | 93.644931 |
| 128 | Decryption |  4.00 KB | 256 | 94.783275 |
| 128 | Encryption |  4.00 KB | 512 | 98.071815 |
| 128 | Decryption |  4.00 KB | 512 | 97.841941 |
| 128 | Encryption |  4.00 KB | 1024 | 99.723059 |
| 128 | Decryption |  4.00 KB | 1024 | 99.593487 |
| 128 | Encryption |  4.00 KB | 2048 | 100.328376 |
| 128 | Decryption |  4.00 KB | 2048 | 100.319161 |
| 128 | Encryption |  8.00 KB | 256 | 93.126956 |
| 128 | Decryption |  8.00 KB | 256 | 93.143501 |
| 128 | Encryption |  8.00 KB | 512 | 95.885410 |
| 128 | Decryption |  8.00 KB | 512 | 96.197868 |
| 128 | Encryption |  8.00 KB | 1024 | 97.656416 |
| 128 | Decryption |  8.00 KB | 1024 | 97.372028 |
| 128 | Encryption |  8.00 KB | 2048 | 98.014245 |
| 128 | Decryption |  8.00 KB | 2048 | 97.969556 |
| 128 | Encryption |  8.00 KB | 4096 | 98.593742 |
| 128 | Decryption |  8.00 KB | 4096 | 98.349610 |
| 128 | Encryption |  16.00 KB | 256 | 92.467182 |
| 128 | Decryption |  16.00 KB | 256 | 92.407859 |
| 128 | Encryption |  16.00 KB | 512 | 95.261179 |
| 128 | Decryption |  16.00 KB | 512 | 94.920729 |
| 128 | Encryption |  16.00 KB | 1024 | 96.616413 |
| 128 | Decryption |  16.00 KB | 1024 | 96.297698 |
| 128 | Encryption |  16.00 KB | 2048 | 97.139120 |
| 128 | Decryption |  16.00 KB | 2048 | 97.034127 |
| 128 | Encryption |  16.00 KB | 4096 | 97.518549 |
| 128 | Decryption |  16.00 KB | 4096 | 97.252277 |
| 128 | Encryption |  16.00 KB | 8192 | 97.646412 |
| 128 | Decryption |  16.00 KB | 8192 | 97.608600 |
| 128 | Encryption |  32.00 KB | 256 | 92.632925 |
| 128 | Decryption |  32.00 KB | 256 | 92.775371 |
| 128 | Encryption |  32.00 KB | 512 | 95.435710 |
| 128 | Decryption |  32.00 KB | 512 | 95.289141 |
| 128 | Encryption |  32.00 KB | 1024 | 96.640723 |
| 128 | Decryption |  32.00 KB | 1024 | 96.508016 |
| 128 | Encryption |  32.00 KB | 2048 | 97.348976 |
| 128 | Decryption |  32.00 KB | 2048 | 97.202063 |
| 128 | Encryption |  32.00 KB | 4096 | 97.697269 |
| 128 | Decryption |  32.00 KB | 4096 | 97.545492 |
| 128 | Encryption |  32.00 KB | 8192 | 97.899766 |
| 128 | Decryption |  32.00 KB | 8192 | 97.738795 |
| 128 | Encryption |  32.00 KB | 16384 | 97.955188 |
| 128 | Decryption |  32.00 KB | 16384 | 97.810272 |

### SHA

- Software/Application used : test_dthe_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 284.386586 |
| 512 |  16.00 KB | 282.846985 |
| 512 |  8.00 KB | 280.062392 |
| 512 |  4.00 KB | 273.945575 |
| 512 |  2.00 KB | 262.543065 |
| 512 |  1024.00 B | 241.937389 |
| 512 |  512.00 B | 209.166348 |
| 256 |  32.00 KB | 275.065909 |
| 256 |  16.00 KB | 274.276627 |
| 256 |  8.00 KB | 270.902270 |
| 256 |  4.00 KB | 267.674148 |
| 256 |  2.00 KB | 258.994625 |
| 256 |  1024.00 B | 243.700729 |
| 256 |  512.00 B | 217.930301 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 70.287430 |
| 256 | Decryption |  512.00 B | 256 | 73.718785 |
| 256 | Encryption |  1024.00 B | 256 | 82.476718 |
| 256 | Decryption |  1024.00 B | 256 | 82.726584 |
| 256 | Encryption |  1024.00 B | 512 | 83.805627 |
| 256 | Decryption |  1024.00 B | 512 | 84.336233 |
| 256 | Encryption |  2.00 KB | 256 | 85.473563 |
| 256 | Decryption |  2.00 KB | 256 | 86.388442 |
| 256 | Encryption |  2.00 KB | 512 | 88.643618 |
| 256 | Decryption |  2.00 KB | 512 | 88.570540 |
| 256 | Encryption |  2.00 KB | 1024 | 89.473828 |
| 256 | Decryption |  2.00 KB | 1024 | 89.587577 |
| 256 | Encryption |  4.00 KB | 256 | 88.387775 |
| 256 | Decryption |  4.00 KB | 256 | 88.353219 |
| 256 | Encryption |  4.00 KB | 512 | 90.754371 |
| 256 | Decryption |  4.00 KB | 512 | 90.438767 |
| 256 | Encryption |  4.00 KB | 1024 | 91.664510 |
| 256 | Decryption |  4.00 KB | 1024 | 91.729944 |
| 256 | Encryption |  4.00 KB | 2048 | 92.151073 |
| 256 | Decryption |  4.00 KB | 2048 | 92.246409 |
| 256 | Encryption |  8.00 KB | 256 | 87.334755 |
| 256 | Decryption |  8.00 KB | 256 | 87.031467 |
| 256 | Encryption |  8.00 KB | 512 | 88.700607 |
| 256 | Decryption |  8.00 KB | 512 | 88.903660 |
| 256 | Encryption |  8.00 KB | 1024 | 89.661117 |
| 256 | Decryption |  8.00 KB | 1024 | 89.748602 |
| 256 | Encryption |  8.00 KB | 2048 | 90.174953 |
| 256 | Decryption |  8.00 KB | 2048 | 90.262824 |
| 256 | Encryption |  8.00 KB | 4096 | 90.436271 |
| 256 | Decryption |  8.00 KB | 4096 | 90.346506 |
| 256 | Encryption |  16.00 KB | 256 | 86.347751 |
| 256 | Decryption |  16.00 KB | 256 | 86.258109 |
| 256 | Encryption |  16.00 KB | 512 | 88.195673 |
| 256 | Decryption |  16.00 KB | 512 | 87.870037 |
| 256 | Encryption |  16.00 KB | 1024 | 89.079485 |
| 256 | Decryption |  16.00 KB | 1024 | 88.856198 |
| 256 | Encryption |  16.00 KB | 2048 | 89.547793 |
| 256 | Decryption |  16.00 KB | 2048 | 89.175394 |
| 256 | Encryption |  16.00 KB | 4096 | 89.725870 |
| 256 | Decryption |  16.00 KB | 4096 | 89.489558 |
| 256 | Encryption |  16.00 KB | 8192 | 89.749524 |
| 256 | Decryption |  16.00 KB | 8192 | 89.738003 |
| 256 | Encryption |  32.00 KB | 256 | 86.388157 |
| 256 | Decryption |  32.00 KB | 256 | 86.487128 |
| 256 | Encryption |  32.00 KB | 512 | 88.314523 |
| 256 | Decryption |  32.00 KB | 512 | 88.211625 |
| 256 | Encryption |  32.00 KB | 1024 | 89.183888 |
| 256 | Decryption |  32.00 KB | 1024 | 89.053687 |
| 256 | Encryption |  32.00 KB | 2048 | 89.691180 |
| 256 | Decryption |  32.00 KB | 2048 | 89.497425 |
| 256 | Encryption |  32.00 KB | 4096 | 89.850115 |
| 256 | Decryption |  32.00 KB | 4096 | 89.865747 |
| 256 | Encryption |  32.00 KB | 8192 | 90.000807 |
| 256 | Decryption |  32.00 KB | 8192 | 89.910593 |
| 256 | Encryption |  32.00 KB | 16384 | 90.037592 |
| 256 | Decryption |  32.00 KB | 16384 | 90.013091 |
| 128 | Encryption |  512.00 B | 256 | 77.130214 |
| 128 | Decryption |  512.00 B | 256 | 79.333721 |
| 128 | Encryption |  1024.00 B | 256 | 87.179078 |
| 128 | Decryption |  1024.00 B | 256 | 87.556446 |
| 128 | Encryption |  1024.00 B | 512 | 91.098137 |
| 128 | Decryption |  1024.00 B | 512 | 91.118403 |
| 128 | Encryption |  2.00 KB | 256 | 92.990522 |
| 128 | Decryption |  2.00 KB | 256 | 93.040688 |
| 128 | Encryption |  2.00 KB | 512 | 95.700935 |
| 128 | Decryption |  2.00 KB | 512 | 95.703730 |
| 128 | Encryption |  2.00 KB | 1024 | 96.616591 |
| 128 | Decryption |  2.00 KB | 1024 | 97.162342 |
| 128 | Encryption |  4.00 KB | 256 | 94.055513 |
| 128 | Decryption |  4.00 KB | 256 | 94.985217 |
| 128 | Encryption |  4.00 KB | 512 | 97.574630 |
| 128 | Decryption |  4.00 KB | 512 | 98.090163 |
| 128 | Encryption |  4.00 KB | 1024 | 99.674525 |
| 128 | Decryption |  4.00 KB | 1024 | 99.406166 |
| 128 | Encryption |  4.00 KB | 2048 | 100.419077 |
| 128 | Decryption |  4.00 KB | 2048 | 100.157413 |
| 128 | Encryption |  8.00 KB | 256 | 92.834428 |
| 128 | Decryption |  8.00 KB | 256 | 92.890021 |
| 128 | Encryption |  8.00 KB | 512 | 95.918741 |
| 128 | Decryption |  8.00 KB | 512 | 95.799910 |
| 128 | Encryption |  8.00 KB | 1024 | 97.243799 |
| 128 | Decryption |  8.00 KB | 1024 | 97.310219 |
| 128 | Encryption |  8.00 KB | 2048 | 98.070347 |
| 128 | Decryption |  8.00 KB | 2048 | 98.019743 |
| 128 | Encryption |  8.00 KB | 4096 | 98.129820 |
| 128 | Decryption |  8.00 KB | 4096 | 98.348134 |
| 128 | Encryption |  16.00 KB | 256 | 92.593744 |
| 128 | Decryption |  16.00 KB | 256 | 92.500463 |
| 128 | Encryption |  16.00 KB | 512 | 95.245777 |
| 128 | Decryption |  16.00 KB | 512 | 95.027396 |
| 128 | Encryption |  16.00 KB | 1024 | 96.564274 |
| 128 | Decryption |  16.00 KB | 1024 | 96.311850 |
| 128 | Encryption |  16.00 KB | 2048 | 97.154780 |
| 128 | Decryption |  16.00 KB | 2048 | 96.911443 |
| 128 | Encryption |  16.00 KB | 4096 | 97.570998 |
| 128 | Decryption |  16.00 KB | 4096 | 97.326297 |
| 128 | Encryption |  16.00 KB | 8192 | 97.821130 |
| 128 | Decryption |  16.00 KB | 8192 | 97.336054 |
| 128 | Encryption |  32.00 KB | 256 | 92.568404 |
| 128 | Decryption |  32.00 KB | 256 | 92.633744 |
| 128 | Encryption |  32.00 KB | 512 | 95.310881 |
| 128 | Decryption |  32.00 KB | 512 | 95.290873 |
| 128 | Encryption |  32.00 KB | 1024 | 96.726214 |
| 128 | Decryption |  32.00 KB | 1024 | 96.551203 |
| 128 | Encryption |  32.00 KB | 2048 | 97.365337 |
| 128 | Decryption |  32.00 KB | 2048 | 97.215580 |
| 128 | Encryption |  32.00 KB | 4096 | 97.661327 |
| 128 | Decryption |  32.00 KB | 4096 | 97.606328 |
| 128 | Encryption |  32.00 KB | 8192 | 97.940091 |
| 128 | Decryption |  32.00 KB | 8192 | 97.784368 |
| 128 | Encryption |  32.00 KB | 16384 | 98.028357 |
| 128 | Decryption |  32.00 KB | 16384 | 97.896110 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 256 | Encryption |  32.00 KB | 89.990148 |
| 256 | Decryption |  32.00 KB | 90.082301 |
| 256 | Encryption |  16.00 KB | 89.888319 |
| 256 | Decryption |  16.00 KB | 90.005287 |
| 256 | Encryption |  8.00 KB | 89.959266 |
| 256 | Decryption |  8.00 KB | 89.788257 |
| 256 | Encryption |  4.00 KB | 91.517944 |
| 256 | Decryption |  4.00 KB | 92.502258 |
| 256 | Encryption |  2.00 KB | 90.869511 |
| 256 | Decryption |  2.00 KB | 91.363567 |
| 256 | Encryption |  1024.00 B | 87.088715 |
| 256 | Decryption |  1024.00 B | 88.204576 |
| 256 | Encryption |  512.00 B | 82.092394 |
| 256 | Decryption |  512.00 B | 81.593625 |
| 128 | Encryption |  32.00 KB | 97.903331 |
| 128 | Decryption |  32.00 KB | 98.052556 |
| 128 | Encryption |  16.00 KB | 97.933322 |
| 128 | Decryption |  16.00 KB | 97.577535 |
| 128 | Encryption |  8.00 KB | 97.354670 |
| 128 | Decryption |  8.00 KB | 97.479204 |
| 128 | Encryption |  4.00 KB | 99.826352 |
| 128 | Decryption |  4.00 KB | 100.970634 |
| 128 | Encryption |  2.00 KB | 99.122754 |
| 128 | Decryption |  2.00 KB | 99.511069 |
| 128 | Encryption |  1024.00 B | 95.586476 |
| 128 | Decryption |  1024.00 B | 95.712116 |
| 128 | Encryption |  512.00 B | 88.859963 |
| 128 | Decryption |  512.00 B | 88.884067 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 282.317718 |
| 512 |  16.00 KB | 279.241987 |
| 512 |  8.00 KB | 271.300388 |
| 512 |  4.00 KB | 259.759409 |
| 512 |  2.00 KB | 237.423468 |
| 512 |  1024.00 B | 198.847017 |
| 512 |  512.00 B | 156.410501 |
| 256 |  32.00 KB | 273.598902 |
| 256 |  16.00 KB | 271.308812 |
| 256 |  8.00 KB | 265.121312 |
| 256 |  4.00 KB | 256.878001 |
| 256 |  2.00 KB | 239.689854 |
| 256 |  1024.00 B | 212.544594 |
| 256 |  512.00 B | 172.790551 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Size | Performance (Mbps) |
|------------|------|--------------------|
| 256 |  32.00 KB | 232.089562 |
| 256 |  16.00 KB | 233.660754 |
| 256 |  8.00 KB | 231.242998 |
| 256 |  4.00 KB | 217.416979 |
| 256 |  2.00 KB | 193.538480 |
| 256 |  1024.00 B | 158.675125 |
| 256 |  512.00 B | 116.338848 |
| 128 |  32.00 KB | 232.361114 |
| 128 |  16.00 KB | 236.080692 |
| 128 |  8.00 KB | 231.210365 |
| 128 |  4.00 KB | 217.713109 |
| 128 |  2.00 KB | 194.025520 |
| 128 |  1024.00 B | 159.230283 |
| 128 |  512.00 B | 117.490140 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) |
|------------|-------------------|--------------------|--------------------|
| 256        |      32.00 KB     |        512        |      201.572091      |
| 256        |      32.00 KB     |        1024        |      215.635694      |
| 256        |      32.00 KB     |        2048        |      222.994524      |
| 256        |      32.00 KB     |        4096        |      227.309402      |
| 256        |      32.00 KB     |        8192        |      230.026039      |
| 256        |      32.00 KB     |        16384        |      230.314532      |
| 256        |      16.00 KB     |        512        |      205.675729      |
| 256        |      16.00 KB     |        1024        |      220.770504      |
| 256        |      16.00 KB     |        2048        |      228.346443      |
| 256        |      16.00 KB     |        4096        |      232.575512      |
| 256        |      16.00 KB     |        8192        |      235.387503      |
| 256        |      8.00 KB     |        512        |      201.293097      |
| 256        |      8.00 KB     |        1024        |      214.518703      |
| 256        |      8.00 KB     |        2048        |      223.529311      |
| 256        |      8.00 KB     |        4096        |      227.822535      |
| 256        |      4.00 KB     |        512        |      190.522705      |
| 256        |      4.00 KB     |        1024        |      203.527950      |
| 256        |      4.00 KB     |        2048        |      210.408707      |
| 256        |      2.00 KB     |        512        |      171.677058      |
| 256        |      2.00 KB     |        1024        |      179.890752      |
| 256        |      1024.00 B     |        512        |      142.469565      |
| 128        |      32.00 KB     |        512        |      202.838959      |
| 128        |      32.00 KB     |        1024        |      216.096976      |
| 128        |      32.00 KB     |        2048        |      223.004957      |
| 128        |      32.00 KB     |        4096        |      227.119356      |
| 128        |      32.00 KB     |        8192        |      230.228060      |
| 128        |      32.00 KB     |        16384        |      232.165101      |
| 128        |      16.00 KB     |        512        |      201.241330      |
| 128        |      16.00 KB     |        1024        |      217.979229      |
| 128        |      16.00 KB     |        2048        |      227.484467      |
| 128        |      16.00 KB     |        4096        |      230.730097      |
| 128        |      16.00 KB     |        8192        |      232.857511      |
| 128        |      8.00 KB     |        512        |      201.483394      |
| 128        |      8.00 KB     |        1024        |      216.031975      |
| 128        |      8.00 KB     |        2048        |      224.096839      |
| 128        |      8.00 KB     |        4096        |      227.301026      |
| 128        |      4.00 KB     |        512        |      190.919552      |
| 128        |      4.00 KB     |        1024        |      203.651280      |
| 128        |      4.00 KB     |        2048        |      209.731979      |
| 128        |      2.00 KB     |        512        |      171.623108      |
| 128        |      2.00 KB     |        1024        |      182.795939      |
| 128        |      1024.00 B     |        512        |      143.191750      |

### AES ECB

- Software/Application used : test_dthe_aes_ecb
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 256 | Encryption |  32.00 KB | 90.237578 |
| 256 | Decryption |  32.00 KB | 90.112338 |
| 256 | Encryption |  16.00 KB | 90.255365 |
| 256 | Decryption |  16.00 KB | 90.101652 |
| 256 | Encryption |  8.00 KB | 91.075033 |
| 256 | Decryption |  8.00 KB | 90.589093 |
| 256 | Encryption |  4.00 KB | 91.970670 |
| 256 | Decryption |  4.00 KB | 92.963480 |
| 256 | Encryption |  2.00 KB | 91.515388 |
| 256 | Decryption |  2.00 KB | 91.834704 |
| 256 | Encryption |  1024.00 B | 89.108857 |
| 256 | Decryption |  1024.00 B | 88.925073 |
| 256 | Encryption |  512.00 B | 83.472590 |
| 256 | Decryption |  512.00 B | 83.383378 |
| 128 | Encryption |  32.00 KB | 98.172541 |
| 128 | Decryption |  32.00 KB | 98.132115 |
| 128 | Encryption |  16.00 KB | 97.643503 |
| 128 | Decryption |  16.00 KB | 97.735971 |
| 128 | Encryption |  8.00 KB | 97.752553 |
| 128 | Decryption |  8.00 KB | 98.959234 |
| 128 | Encryption |  4.00 KB | 100.196461 |
| 128 | Decryption |  4.00 KB | 101.097579 |
| 128 | Encryption |  2.00 KB | 99.934430 |
| 128 | Decryption |  2.00 KB | 99.482369 |
| 128 | Encryption |  1024.00 B | 96.348133 |
| 128 | Decryption |  1024.00 B | 96.464423 |
| 128 | Encryption |  512.00 B | 90.245112 |
| 128 | Decryption |  512.00 B | 90.106143 |

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.084000 		| 533			| 275.360016		|
cos  		|0.0000002870	| 64			| 64.110001 		| 503			| 276.773987		|
sincos sin  	|0.0000001790	| 79			| 79.073997 		| 467			| 274.328003		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 73			| 73.033997 		| 576			| 428.686005		|
acos 		|0.0000004770	| 74			| 74.029999 		| 788			| 383.786011		|
atan 		|0.0000005360	| 85			| 85.056000 		| 716			| 371.657990		|
atan2 		|0.0000007150	| 119			| 106.739998 		| 743			| 477.947998		|


### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

