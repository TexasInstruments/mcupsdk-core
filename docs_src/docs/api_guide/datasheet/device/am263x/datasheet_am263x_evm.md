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
SBL : System Init                       |   712
SBL : Drivers_open                      |   98
SBL : LoadHsmRtFw                       |   9015
SBL : Board_driversOpen                 |   118
SBL : CPU Load                          |   7783
SBL : SBL End                           |   2944
SBL : Total time taken                  |   20673

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI MULTICORE ELF performance

- Software/Application used        : sbl_qspi and ipc_notify_echo
- Size of sbl_qspi appimage        : 253 KB
- Size of ipc_notify_echo          : 83 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   693
SBL : Drivers_open                      |   98
SBL : LoadHsmRtFw                       |   8815
SBL : Board_driversOpen                 |   110
SBL : CPU Load                          |   11056
SBL : SBL End                           |   17
SBL : Total time taken                  |   20788

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL QSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a> . 

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_qspi appimage        : 780 KB
- Size of hello_world              : 31.1 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   778
SBL : Drivers_open                      |   146593
SBL : LoadHsmRtFw                       |   9008
SBL : Board_driversOpen                 |   2988
SBL : File read from SD card            |   8062
SBL : CPU Load                          |   44
SBL : SBL End                           |   2919
SBL : Total time taken                  |   170394

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD MULTICORE ELF performance

- Software/Application used        : sbl_sd_multicore_elf and hello_world
- Size of sbl_qspi appimage        : 313 KB
- Size of hello_world              : 31.3 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   771
SBL : Drivers_open                      |   149706
SBL : LoadHsmRtFw                       |   9013
SBL : Board_driversOpen                 |   2981
SBL : File read from SD card            |   8377
SBL : CPU Load                          |   4363
SBL : SBL End                           |   15
SBL : Total time taken                  |   175229

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
 r5f0-0	| r5f0-1	|  3.58
 r5f0-0	| r5f1-0	|  1.80
 r5f0-0	| r5f1-1	|  1.86

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 9.721
 r5f0-0	| r5f1-0	| 4	| 9.794
 r5f0-0	| r5f1-1	| 4	| 9.824
 r5f0-0	| r5f0-1	| 32	| 12.553
 r5f0-0	| r5f0-1	| 64	| 15.096
 r5f0-0	| r5f0-1	| 112	| 18.910

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
 400		| 08			| 10.80 Mbps / 296.18 us 	| 22.87 Mbps / 139.91 us 	|  0.91 Mbps / 3526.30 us
 200		| 16			| 22.51 Mbps / 142.14 us 	| 29.94 Mbps / 106.88 us 	|  0.95 Mbps / 3370.35 us
 100		| 32			| 37.22 Mbps / 85.97 us 	| 35.31 Mbps / 90.62 us 	|  0.97 Mbps / 3292.82 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 72
- End tick         : 10234487
- Total ticks      : 10234415
- Total time (secs): 10.234415
- Iterations/Sec   : 1465.643127
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.643127 
CoreMark/MHz :3.664108 / STACK

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 217.334964 |
| 256 |  16.00 KB | 217.828891 |
| 256 |  8.00 KB | 216.454735 |
| 256 |  4.00 KB | 205.094824 |
| 256 |  2.00 KB | 183.976194 |
| 256 |  1024.00 B | 153.393877 |
| 256 |  512.00 B | 114.669653 |
| 128 |  32.00 KB | 217.517103 |
| 128 |  16.00 KB | 217.861476 |
| 128 |  8.00 KB | 217.186413 |
| 128 |  4.00 KB | 205.596687 |
| 128 |  2.00 KB | 183.000112 |
| 128 |  1024.00 B | 153.724901 |
| 128 |  512.00 B | 116.050432 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.745413 |
| 256 | Decryption |  32.00 KB | 95.935853 |
| 256 | Encryption |  16.00 KB | 96.100535 |
| 256 | Decryption |  16.00 KB | 95.892952 |
| 256 | Encryption |  8.00 KB | 95.726096 |
| 256 | Decryption |  8.00 KB | 95.711766 |
| 256 | Encryption |  4.00 KB | 97.962600 |
| 256 | Decryption |  4.00 KB | 98.356621 |
| 256 | Encryption |  2.00 KB | 97.825146 |
| 256 | Decryption |  2.00 KB | 97.964065 |
| 256 | Encryption |  1024.00 B | 95.275201 |
| 256 | Decryption |  1024.00 B | 94.954939 |
| 256 | Encryption |  512.00 B | 89.427433 |
| 256 | Decryption |  512.00 B | 88.908183 |
| 128 | Encryption |  32.00 KB | 103.889557 |
| 128 | Decryption |  32.00 KB | 104.098824 |
| 128 | Encryption |  16.00 KB | 103.726155 |
| 128 | Decryption |  16.00 KB | 103.836883 |
| 128 | Encryption |  8.00 KB | 104.935672 |
| 128 | Decryption |  8.00 KB | 104.969708 |
| 128 | Encryption |  4.00 KB | 106.500260 |
| 128 | Decryption |  4.00 KB | 107.109469 |
| 128 | Encryption |  2.00 KB | 106.217180 |
| 128 | Decryption |  2.00 KB | 106.429349 |
| 128 | Encryption |  1024.00 B | 102.782221 |
| 128 | Decryption |  1024.00 B | 103.125098 |
| 128 | Encryption |  512.00 B | 96.592383 |
| 128 | Decryption |  512.00 B | 96.655065 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 74.035246 |
| 256 | Decryption |  512.00 B | 256 | 79.080992 |
| 256 | Encryption |  1024.00 B | 256 | 86.653444 |
| 256 | Decryption |  1024.00 B | 256 | 87.176759 |
| 256 | Encryption |  1024.00 B | 512 | 89.281238 |
| 256 | Decryption |  1024.00 B | 512 | 89.942907 |
| 256 | Encryption |  2.00 KB | 256 | 90.227717 |
| 256 | Decryption |  2.00 KB | 256 | 91.265597 |
| 256 | Encryption |  2.00 KB | 512 | 93.354796 |
| 256 | Decryption |  2.00 KB | 512 | 94.096026 |
| 256 | Encryption |  2.00 KB | 1024 | 95.582294 |
| 256 | Decryption |  2.00 KB | 1024 | 95.519604 |
| 256 | Encryption |  4.00 KB | 256 | 92.307476 |
| 256 | Decryption |  4.00 KB | 256 | 93.079010 |
| 256 | Encryption |  4.00 KB | 512 | 95.650651 |
| 256 | Decryption |  4.00 KB | 512 | 96.186220 |
| 256 | Encryption |  4.00 KB | 1024 | 97.701184 |
| 256 | Decryption |  4.00 KB | 1024 | 97.289273 |
| 256 | Encryption |  4.00 KB | 2048 | 98.384675 |
| 256 | Decryption |  4.00 KB | 2048 | 98.087227 |
| 256 | Encryption |  8.00 KB | 256 | 90.549979 |
| 256 | Decryption |  8.00 KB | 256 | 91.158961 |
| 256 | Encryption |  8.00 KB | 512 | 94.053150 |
| 256 | Decryption |  8.00 KB | 512 | 93.779268 |
| 256 | Encryption |  8.00 KB | 1024 | 95.313653 |
| 256 | Decryption |  8.00 KB | 1024 | 95.151396 |
| 256 | Encryption |  8.00 KB | 2048 | 95.789758 |
| 256 | Decryption |  8.00 KB | 2048 | 96.205282 |
| 256 | Encryption |  8.00 KB | 4096 | 96.271704 |
| 256 | Decryption |  8.00 KB | 4096 | 96.353091 |
| 256 | Encryption |  16.00 KB | 256 | 90.294692 |
| 256 | Decryption |  16.00 KB | 256 | 90.217159 |
| 256 | Encryption |  16.00 KB | 512 | 92.965788 |
| 256 | Decryption |  16.00 KB | 512 | 92.938102 |
| 256 | Encryption |  16.00 KB | 1024 | 94.323207 |
| 256 | Decryption |  16.00 KB | 1024 | 94.136236 |
| 256 | Encryption |  16.00 KB | 2048 | 95.106519 |
| 256 | Decryption |  16.00 KB | 2048 | 94.760319 |
| 256 | Encryption |  16.00 KB | 4096 | 95.394469 |
| 256 | Decryption |  16.00 KB | 4096 | 95.230553 |
| 256 | Encryption |  16.00 KB | 8192 | 95.631459 |
| 256 | Decryption |  16.00 KB | 8192 | 95.391345 |
| 256 | Encryption |  32.00 KB | 256 | 90.363946 |
| 256 | Decryption |  32.00 KB | 256 | 90.379212 |
| 256 | Encryption |  32.00 KB | 512 | 93.069592 |
| 256 | Decryption |  32.00 KB | 512 | 92.995305 |
| 256 | Encryption |  32.00 KB | 1024 | 94.454819 |
| 256 | Decryption |  32.00 KB | 1024 | 94.278259 |
| 256 | Encryption |  32.00 KB | 2048 | 95.209714 |
| 256 | Decryption |  32.00 KB | 2048 | 95.075475 |
| 256 | Encryption |  32.00 KB | 4096 | 95.561824 |
| 256 | Decryption |  32.00 KB | 4096 | 95.346761 |
| 256 | Encryption |  32.00 KB | 8192 | 95.776547 |
| 256 | Decryption |  32.00 KB | 8192 | 95.629715 |
| 256 | Encryption |  32.00 KB | 16384 | 95.921110 |
| 256 | Decryption |  32.00 KB | 16384 | 95.588393 |
| 128 | Encryption |  512.00 B | 256 | 82.593134 |
| 128 | Decryption |  512.00 B | 256 | 84.838442 |
| 128 | Encryption |  1024.00 B | 256 | 93.053899 |
| 128 | Decryption |  1024.00 B | 256 | 94.574001 |
| 128 | Encryption |  1024.00 B | 512 | 97.911375 |
| 128 | Decryption |  1024.00 B | 512 | 97.712837 |
| 128 | Encryption |  2.00 KB | 256 | 98.079887 |
| 128 | Decryption |  2.00 KB | 256 | 98.649767 |
| 128 | Encryption |  2.00 KB | 512 | 102.162154 |
| 128 | Decryption |  2.00 KB | 512 | 102.096900 |
| 128 | Encryption |  2.00 KB | 1024 | 103.850664 |
| 128 | Decryption |  2.00 KB | 1024 | 103.230684 |
| 128 | Encryption |  4.00 KB | 256 | 100.576269 |
| 128 | Decryption |  4.00 KB | 256 | 100.592479 |
| 128 | Encryption |  4.00 KB | 512 | 104.307690 |
| 128 | Decryption |  4.00 KB | 512 | 104.368321 |
| 128 | Encryption |  4.00 KB | 1024 | 105.866294 |
| 128 | Decryption |  4.00 KB | 1024 | 106.144066 |
| 128 | Encryption |  4.00 KB | 2048 | 107.144492 |
| 128 | Decryption |  4.00 KB | 2048 | 106.752673 |
| 128 | Encryption |  8.00 KB | 256 | 98.642714 |
| 128 | Decryption |  8.00 KB | 256 | 98.788062 |
| 128 | Encryption |  8.00 KB | 512 | 102.181268 |
| 128 | Decryption |  8.00 KB | 512 | 101.961883 |
| 128 | Encryption |  8.00 KB | 1024 | 103.805017 |
| 128 | Decryption |  8.00 KB | 1024 | 103.624059 |
| 128 | Encryption |  8.00 KB | 2048 | 104.451909 |
| 128 | Decryption |  8.00 KB | 2048 | 103.777484 |
| 128 | Encryption |  8.00 KB | 4096 | 104.603622 |
| 128 | Decryption |  8.00 KB | 4096 | 104.838310 |
| 128 | Encryption |  16.00 KB | 256 | 97.116808 |
| 128 | Decryption |  16.00 KB | 256 | 97.458909 |
| 128 | Encryption |  16.00 KB | 512 | 100.710154 |
| 128 | Decryption |  16.00 KB | 512 | 100.491062 |
| 128 | Encryption |  16.00 KB | 1024 | 102.393600 |
| 128 | Decryption |  16.00 KB | 1024 | 101.887379 |
| 128 | Encryption |  16.00 KB | 2048 | 102.993014 |
| 128 | Decryption |  16.00 KB | 2048 | 102.832217 |
| 128 | Encryption |  16.00 KB | 4096 | 103.453529 |
| 128 | Decryption |  16.00 KB | 4096 | 103.322639 |
| 128 | Encryption |  16.00 KB | 8192 | 103.826396 |
| 128 | Decryption |  16.00 KB | 8192 | 103.310830 |
| 128 | Encryption |  32.00 KB | 256 | 97.412372 |
| 128 | Decryption |  32.00 KB | 256 | 97.612234 |
| 128 | Encryption |  32.00 KB | 512 | 100.812111 |
| 128 | Decryption |  32.00 KB | 512 | 100.670899 |
| 128 | Encryption |  32.00 KB | 1024 | 102.394600 |
| 128 | Decryption |  32.00 KB | 1024 | 102.242443 |
| 128 | Encryption |  32.00 KB | 2048 | 103.264741 |
| 128 | Decryption |  32.00 KB | 2048 | 103.084951 |
| 128 | Encryption |  32.00 KB | 4096 | 103.668830 |
| 128 | Decryption |  32.00 KB | 4096 | 103.468433 |
| 128 | Encryption |  32.00 KB | 8192 | 103.799571 |
| 128 | Decryption |  32.00 KB | 8192 | 103.860745 |
| 128 | Encryption |  32.00 KB | 16384 | 103.971039 |
| 128 | Decryption |  32.00 KB | 16384 | 103.829686 |


### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.914003 |
| 256 | Decryption |  32.00 KB | 95.881903 |
| 256 | Encryption |  16.00 KB | 96.186573 |
| 256 | Decryption |  16.00 KB | 95.679278 |
| 256 | Encryption |  8.00 KB | 96.117096 |
| 256 | Decryption |  8.00 KB | 95.874539 |
| 256 | Encryption |  4.00 KB | 97.475942 |
| 256 | Decryption |  4.00 KB | 98.341862 |
| 256 | Encryption |  2.00 KB | 97.321057 |
| 256 | Decryption |  2.00 KB | 97.526712 |
| 256 | Encryption |  1024.00 B | 94.231322 |
| 256 | Decryption |  1024.00 B | 94.144688 |
| 256 | Encryption |  512.00 B | 88.418780 |
| 256 | Decryption |  512.00 B | 87.996133 |
| 128 | Encryption |  32.00 KB | 103.981452 |
| 128 | Decryption |  32.00 KB | 104.117326 |
| 128 | Encryption |  16.00 KB | 103.788167 |
| 128 | Decryption |  16.00 KB | 103.927243 |
| 128 | Encryption |  8.00 KB | 104.529795 |
| 128 | Decryption |  8.00 KB | 104.815674 |
| 128 | Encryption |  4.00 KB | 105.928752 |
| 128 | Decryption |  4.00 KB | 107.377976 |
| 128 | Encryption |  2.00 KB | 105.689588 |
| 128 | Decryption |  2.00 KB | 106.189643 |
| 128 | Encryption |  1024.00 B | 102.316868 |
| 128 | Decryption |  1024.00 B | 102.249821 |
| 128 | Encryption |  512.00 B | 95.189403 |
| 128 | Decryption |  512.00 B | 95.029291 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 73.032005 |
| 256 | Decryption |  512.00 B | 256 | 78.704905 |
| 256 | Encryption |  1024.00 B | 256 | 85.549435 |
| 256 | Decryption |  1024.00 B | 256 | 86.333816 |
| 256 | Encryption |  1024.00 B | 512 | 88.569343 |
| 256 | Decryption |  1024.00 B | 512 | 89.191323 |
| 256 | Encryption |  2.00 KB | 256 | 89.429874 |
| 256 | Decryption |  2.00 KB | 256 | 90.268729 |
| 256 | Encryption |  2.00 KB | 512 | 93.211395 |
| 256 | Decryption |  2.00 KB | 512 | 93.645600 |
| 256 | Encryption |  2.00 KB | 1024 | 95.287669 |
| 256 | Decryption |  2.00 KB | 1024 | 95.072026 |
| 256 | Encryption |  4.00 KB | 256 | 92.479415 |
| 256 | Decryption |  4.00 KB | 256 | 92.971393 |
| 256 | Encryption |  4.00 KB | 512 | 95.398635 |
| 256 | Decryption |  4.00 KB | 512 | 95.763164 |
| 256 | Encryption |  4.00 KB | 1024 | 97.436088 |
| 256 | Decryption |  4.00 KB | 1024 | 96.971132 |
| 256 | Encryption |  4.00 KB | 2048 | 98.173920 |
| 256 | Decryption |  4.00 KB | 2048 | 97.932591 |
| 256 | Encryption |  8.00 KB | 256 | 90.487779 |
| 256 | Decryption |  8.00 KB | 256 | 90.983049 |
| 256 | Encryption |  8.00 KB | 512 | 93.785978 |
| 256 | Decryption |  8.00 KB | 512 | 93.870272 |
| 256 | Encryption |  8.00 KB | 1024 | 95.270354 |
| 256 | Decryption |  8.00 KB | 1024 | 95.163141 |
| 256 | Encryption |  8.00 KB | 2048 | 95.938048 |
| 256 | Decryption |  8.00 KB | 2048 | 95.796409 |
| 256 | Encryption |  8.00 KB | 4096 | 96.249789 |
| 256 | Decryption |  8.00 KB | 4096 | 96.181985 |
| 256 | Encryption |  16.00 KB | 256 | 90.415528 |
| 256 | Decryption |  16.00 KB | 256 | 90.118224 |
| 256 | Encryption |  16.00 KB | 512 | 92.964140 |
| 256 | Decryption |  16.00 KB | 512 | 92.762157 |
| 256 | Encryption |  16.00 KB | 1024 | 94.253853 |
| 256 | Decryption |  16.00 KB | 1024 | 94.049101 |
| 256 | Encryption |  16.00 KB | 2048 | 94.981775 |
| 256 | Decryption |  16.00 KB | 2048 | 94.722655 |
| 256 | Encryption |  16.00 KB | 4096 | 95.421032 |
| 256 | Decryption |  16.00 KB | 4096 | 95.067544 |
| 256 | Encryption |  16.00 KB | 8192 | 95.558776 |
| 256 | Decryption |  16.00 KB | 8192 | 95.366012 |
| 256 | Encryption |  32.00 KB | 256 | 90.274869 |
| 256 | Decryption |  32.00 KB | 256 | 90.402276 |
| 256 | Encryption |  32.00 KB | 512 | 93.062323 |
| 256 | Decryption |  32.00 KB | 512 | 93.004378 |
| 256 | Encryption |  32.00 KB | 1024 | 94.454563 |
| 256 | Decryption |  32.00 KB | 1024 | 94.259276 |
| 256 | Encryption |  32.00 KB | 2048 | 95.141986 |
| 256 | Decryption |  32.00 KB | 2048 | 95.102551 |
| 256 | Encryption |  32.00 KB | 4096 | 95.426416 |
| 256 | Decryption |  32.00 KB | 4096 | 95.416691 |
| 256 | Encryption |  32.00 KB | 8192 | 95.688446 |
| 256 | Decryption |  32.00 KB | 8192 | 95.567136 |
| 256 | Encryption |  32.00 KB | 16384 | 95.776634 |
| 256 | Decryption |  32.00 KB | 16384 | 95.640007 |
| 128 | Encryption |  512.00 B | 256 | 81.285969 |
| 128 | Decryption |  512.00 B | 256 | 82.634791 |
| 128 | Encryption |  1024.00 B | 256 | 92.304225 |
| 128 | Decryption |  1024.00 B | 256 | 92.877186 |
| 128 | Encryption |  1024.00 B | 512 | 96.892279 |
| 128 | Decryption |  1024.00 B | 512 | 96.663618 |
| 128 | Encryption |  2.00 KB | 256 | 96.756382 |
| 128 | Decryption |  2.00 KB | 256 | 97.800328 |
| 128 | Encryption |  2.00 KB | 512 | 101.554244 |
| 128 | Decryption |  2.00 KB | 512 | 101.686605 |
| 128 | Encryption |  2.00 KB | 1024 | 103.432711 |
| 128 | Decryption |  2.00 KB | 1024 | 102.663074 |
| 128 | Encryption |  4.00 KB | 256 | 99.043359 |
| 128 | Decryption |  4.00 KB | 256 | 100.317626 |
| 128 | Encryption |  4.00 KB | 512 | 104.003110 |
| 128 | Decryption |  4.00 KB | 512 | 103.973410 |
| 128 | Encryption |  4.00 KB | 1024 | 105.411644 |
| 128 | Decryption |  4.00 KB | 1024 | 105.718572 |
| 128 | Encryption |  4.00 KB | 2048 | 106.620680 |
| 128 | Decryption |  4.00 KB | 2048 | 106.306774 |
| 128 | Encryption |  8.00 KB | 256 | 97.896018 |
| 128 | Decryption |  8.00 KB | 256 | 98.341124 |
| 128 | Encryption |  8.00 KB | 512 | 101.312082 |
| 128 | Decryption |  8.00 KB | 512 | 101.561326 |
| 128 | Encryption |  8.00 KB | 1024 | 103.151881 |
| 128 | Decryption |  8.00 KB | 1024 | 103.316123 |
| 128 | Encryption |  8.00 KB | 2048 | 104.493544 |
| 128 | Decryption |  8.00 KB | 2048 | 104.115465 |
| 128 | Encryption |  8.00 KB | 4096 | 104.455655 |
| 128 | Decryption |  8.00 KB | 4096 | 104.503959 |
| 128 | Encryption |  16.00 KB | 256 | 97.425224 |
| 128 | Decryption |  16.00 KB | 256 | 97.532518 |
| 128 | Encryption |  16.00 KB | 512 | 100.694294 |
| 128 | Decryption |  16.00 KB | 512 | 100.391003 |
| 128 | Encryption |  16.00 KB | 1024 | 102.284929 |
| 128 | Decryption |  16.00 KB | 1024 | 101.984890 |
| 128 | Encryption |  16.00 KB | 2048 | 102.972179 |
| 128 | Decryption |  16.00 KB | 2048 | 102.845530 |
| 128 | Encryption |  16.00 KB | 4096 | 103.334654 |
| 128 | Decryption |  16.00 KB | 4096 | 103.236376 |
| 128 | Encryption |  16.00 KB | 8192 | 103.572268 |
| 128 | Decryption |  16.00 KB | 8192 | 103.458837 |
| 128 | Encryption |  32.00 KB | 256 | 97.495881 |
| 128 | Decryption |  32.00 KB | 256 | 97.615506 |
| 128 | Encryption |  32.00 KB | 512 | 100.835087 |
| 128 | Decryption |  32.00 KB | 512 | 100.655244 |
| 128 | Encryption |  32.00 KB | 1024 | 102.512022 |
| 128 | Decryption |  32.00 KB | 1024 | 102.188836 |
| 128 | Encryption |  32.00 KB | 2048 | 103.243288 |
| 128 | Decryption |  32.00 KB | 2048 | 103.002322 |
| 128 | Encryption |  32.00 KB | 4096 | 103.590481 |
| 128 | Decryption |  32.00 KB | 4096 | 103.446180 |
| 128 | Encryption |  32.00 KB | 8192 | 103.795153 |
| 128 | Decryption |  32.00 KB | 8192 | 103.661553 |
| 128 | Encryption |  32.00 KB | 16384 | 103.952177 |
| 128 | Decryption |  32.00 KB | 16384 | 103.854161 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 311.063383 |
| 512 |  16.00 KB | 309.974636 |
| 512 |  8.00 KB | 306.629860 |
| 512 |  4.00 KB | 299.586295 |
| 512 |  2.00 KB | 286.533753 |
| 512 |  1024.00 B | 263.514274 |
| 512 |  512.00 B | 226.454734 |
| 256 |  32.00 KB | 300.827395 |
| 256 |  16.00 KB | 299.958235 |
| 256 |  8.00 KB | 297.556158 |
| 256 |  4.00 KB | 292.382163 |
| 256 |  2.00 KB | 282.348886 |
| 256 |  1024.00 B | 264.663597 |
| 256 |  512.00 B | 235.165782 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 72.960456 |
| 256 | Decryption |  512.00 B | 256 | 78.629361 |
| 256 | Encryption |  1024.00 B | 256 | 85.242319 |
| 256 | Decryption |  1024.00 B | 256 | 86.390720 |
| 256 | Encryption |  1024.00 B | 512 | 89.461614 |
| 256 | Decryption |  1024.00 B | 512 | 89.736006 |
| 256 | Encryption |  2.00 KB | 256 | 90.606940 |
| 256 | Decryption |  2.00 KB | 256 | 90.981786 |
| 256 | Encryption |  2.00 KB | 512 | 93.766185 |
| 256 | Decryption |  2.00 KB | 512 | 93.348147 |
| 256 | Encryption |  2.00 KB | 1024 | 95.047207 |
| 256 | Decryption |  2.00 KB | 1024 | 95.380585 |
| 256 | Encryption |  4.00 KB | 256 | 91.936479 |
| 256 | Decryption |  4.00 KB | 256 | 92.967437 |
| 256 | Encryption |  4.00 KB | 512 | 95.575324 |
| 256 | Decryption |  4.00 KB | 512 | 95.922251 |
| 256 | Encryption |  4.00 KB | 1024 | 97.318167 |
| 256 | Decryption |  4.00 KB | 1024 | 97.099001 |
| 256 | Encryption |  4.00 KB | 2048 | 98.090163 |
| 256 | Decryption |  4.00 KB | 2048 | 98.174655 |
| 256 | Encryption |  8.00 KB | 256 | 90.522150 |
| 256 | Decryption |  8.00 KB | 256 | 90.827322 |
| 256 | Encryption |  8.00 KB | 512 | 93.850108 |
| 256 | Decryption |  8.00 KB | 512 | 93.345488 |
| 256 | Encryption |  8.00 KB | 1024 | 95.129988 |
| 256 | Decryption |  8.00 KB | 1024 | 94.864223 |
| 256 | Encryption |  8.00 KB | 2048 | 95.757917 |
| 256 | Decryption |  8.00 KB | 2048 | 95.842992 |
| 256 | Encryption |  8.00 KB | 4096 | 96.350966 |
| 256 | Decryption |  8.00 KB | 4096 | 96.193632 |
| 256 | Encryption |  16.00 KB | 256 | 90.170921 |
| 256 | Decryption |  16.00 KB | 256 | 90.116055 |
| 256 | Encryption |  16.00 KB | 512 | 92.952602 |
| 256 | Decryption |  16.00 KB | 512 | 92.817335 |
| 256 | Encryption |  16.00 KB | 1024 | 94.445886 |
| 256 | Decryption |  16.00 KB | 1024 | 94.095688 |
| 256 | Encryption |  16.00 KB | 2048 | 95.004150 |
| 256 | Decryption |  16.00 KB | 2048 | 94.950468 |
| 256 | Encryption |  16.00 KB | 4096 | 95.406273 |
| 256 | Decryption |  16.00 KB | 4096 | 95.089959 |
| 256 | Encryption |  16.00 KB | 8192 | 95.603558 |
| 256 | Decryption |  16.00 KB | 8192 | 95.268969 |
| 256 | Encryption |  32.00 KB | 256 | 90.299668 |
| 256 | Decryption |  32.00 KB | 256 | 90.355069 |
| 256 | Encryption |  32.00 KB | 512 | 93.141019 |
| 256 | Decryption |  32.00 KB | 512 | 92.961832 |
| 256 | Encryption |  32.00 KB | 1024 | 94.501382 |
| 256 | Decryption |  32.00 KB | 1024 | 94.337124 |
| 256 | Encryption |  32.00 KB | 2048 | 95.139310 |
| 256 | Decryption |  32.00 KB | 2048 | 94.977645 |
| 256 | Encryption |  32.00 KB | 4096 | 95.549197 |
| 256 | Decryption |  32.00 KB | 4096 | 95.323878 |
| 256 | Encryption |  32.00 KB | 8192 | 95.674913 |
| 256 | Decryption |  32.00 KB | 8192 | 95.621343 |
| 256 | Encryption |  32.00 KB | 16384 | 95.747074 |
| 256 | Decryption |  32.00 KB | 16384 | 95.679278 |
| 128 | Encryption |  512.00 B | 256 | 81.044717 |
| 128 | Decryption |  512.00 B | 256 | 84.184565 |
| 128 | Encryption |  1024.00 B | 256 | 93.556031 |
| 128 | Decryption |  1024.00 B | 256 | 93.966506 |
| 128 | Encryption |  1024.00 B | 512 | 96.846461 |
| 128 | Decryption |  1024.00 B | 512 | 96.918072 |
| 128 | Encryption |  2.00 KB | 256 | 97.574630 |
| 128 | Decryption |  2.00 KB | 256 | 97.835369 |
| 128 | Encryption |  2.00 KB | 512 | 101.510200 |
| 128 | Decryption |  2.00 KB | 512 | 101.836716 |
| 128 | Encryption |  2.00 KB | 1024 | 103.289256 |
| 128 | Decryption |  2.00 KB | 1024 | 103.378869 |
| 128 | Encryption |  4.00 KB | 256 | 99.331585 |
| 128 | Decryption |  4.00 KB | 256 | 100.264676 |
| 128 | Encryption |  4.00 KB | 512 | 103.924771 |
| 128 | Decryption |  4.00 KB | 512 | 104.231378 |
| 128 | Encryption |  4.00 KB | 1024 | 105.420970 |
| 128 | Decryption |  4.00 KB | 1024 | 105.658917 |
| 128 | Encryption |  4.00 KB | 2048 | 106.737026 |
| 128 | Decryption |  4.00 KB | 2048 | 106.370616 |
| 128 | Encryption |  8.00 KB | 256 | 97.652414 |
| 128 | Decryption |  8.00 KB | 256 | 98.074750 |
| 128 | Encryption |  8.00 KB | 512 | 101.682661 |
| 128 | Decryption |  8.00 KB | 512 | 101.712645 |
| 128 | Encryption |  8.00 KB | 1024 | 103.117797 |
| 128 | Decryption |  8.00 KB | 1024 | 102.990587 |
| 128 | Encryption |  8.00 KB | 2048 | 104.300220 |
| 128 | Decryption |  8.00 KB | 2048 | 103.889351 |
| 128 | Encryption |  8.00 KB | 4096 | 104.322634 |
| 128 | Decryption |  8.00 KB | 4096 | 104.423199 |
| 128 | Encryption |  16.00 KB | 256 | 97.276637 |
| 128 | Decryption |  16.00 KB | 256 | 97.445143 |
| 128 | Encryption |  16.00 KB | 512 | 100.788952 |
| 128 | Decryption |  16.00 KB | 512 | 100.549265 |
| 128 | Encryption |  16.00 KB | 1024 | 102.240250 |
| 128 | Decryption |  16.00 KB | 1024 | 102.060330 |
| 128 | Encryption |  16.00 KB | 2048 | 103.020943 |
| 128 | Decryption |  16.00 KB | 2048 | 102.936799 |
| 128 | Encryption |  16.00 KB | 4096 | 103.545881 |
| 128 | Decryption |  16.00 KB | 4096 | 103.173397 |
| 128 | Encryption |  16.00 KB | 8192 | 103.526252 |
| 128 | Decryption |  16.00 KB | 8192 | 103.527888 |
| 128 | Encryption |  32.00 KB | 256 | 97.541409 |
| 128 | Decryption |  32.00 KB | 256 | 97.592429 |
| 128 | Encryption |  32.00 KB | 512 | 100.784495 |
| 128 | Decryption |  32.00 KB | 512 | 100.628583 |
| 128 | Encryption |  32.00 KB | 1024 | 102.357618 |
| 128 | Decryption |  32.00 KB | 1024 | 102.244835 |
| 128 | Encryption |  32.00 KB | 2048 | 103.164465 |
| 128 | Decryption |  32.00 KB | 2048 | 103.132401 |
| 128 | Encryption |  32.00 KB | 4096 | 103.553243 |
| 128 | Decryption |  32.00 KB | 4096 | 103.524821 |
| 128 | Encryption |  32.00 KB | 8192 | 103.860950 |
| 128 | Decryption |  32.00 KB | 8192 | 103.591300 |
| 128 | Encryption |  32.00 KB | 16384 | 104.033860 |
| 128 | Decryption |  32.00 KB | 16384 | 103.759000 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 95.892952 |
| 256 | Decryption |  32.00 KB | 95.859551 |
| 256 | Encryption |  16.00 KB | 95.898038 |
| 256 | Decryption |  16.00 KB | 95.726271 |
| 256 | Encryption |  8.00 KB | 96.473654 |
| 256 | Decryption |  8.00 KB | 96.475429 |
| 256 | Encryption |  4.00 KB | 97.866049 |
| 256 | Decryption |  4.00 KB | 98.571869 |
| 256 | Encryption |  2.00 KB | 97.310942 |
| 256 | Decryption |  2.00 KB | 97.386136 |
| 256 | Encryption |  1024.00 B | 94.193400 |
| 256 | Decryption |  1024.00 B | 94.128461 |
| 256 | Encryption |  512.00 B | 87.807492 |
| 256 | Decryption |  512.00 B | 87.925298 |
| 128 | Encryption |  32.00 KB | 104.050997 |
| 128 | Decryption |  32.00 KB | 104.062459 |
| 128 | Encryption |  16.00 KB | 103.772554 |
| 128 | Decryption |  16.00 KB | 103.909735 |
| 128 | Encryption |  8.00 KB | 103.818584 |
| 128 | Decryption |  8.00 KB | 104.293165 |
| 128 | Encryption |  4.00 KB | 106.107117 |
| 128 | Decryption |  4.00 KB | 107.028130 |
| 128 | Encryption |  2.00 KB | 105.764638 |
| 128 | Decryption |  2.00 KB | 105.739041 |
| 128 | Encryption |  1024.00 B | 101.621957 |
| 128 | Decryption |  1024.00 B | 101.868374 |
| 128 | Encryption |  512.00 B | 94.787388 |
| 128 | Decryption |  512.00 B | 94.601305 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 308.949912 |
| 512 |  16.00 KB | 304.776078 |
| 512 |  8.00 KB | 297.546026 |
| 512 |  4.00 KB | 283.068417 |
| 512 |  2.00 KB | 258.056387 |
| 512 |  1024.00 B | 218.978883 |
| 512 |  512.00 B | 167.525562 |
| 256 |  32.00 KB | 299.351951 |
| 256 |  16.00 KB | 296.835121 |
| 256 |  8.00 KB | 290.773565 |
| 256 |  4.00 KB | 280.331936 |
| 256 |  2.00 KB | 260.704909 |
| 256 |  1024.00 B | 229.339306 |
| 256 |  512.00 B | 184.421432 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 252.043362 |
| 256 |  16.00 KB | 252.996897 |
| 256 |  8.00 KB | 250.393054 |
| 256 |  4.00 KB | 239.121392 |
| 256 |  2.00 KB | 211.406452 |
| 256 |  1024.00 B | 171.794065 |
| 256 |  512.00 B | 124.631066 |
| 128 |  32.00 KB | 252.447492 |
| 128 |  16.00 KB | 253.709430 |
| 128 |  8.00 KB | 255.795164 |
| 128 |  4.00 KB | 238.520891 |
| 128 |  2.00 KB | 212.737778 |
| 128 |  1024.00 B | 172.699484 |
| 128 |  512.00 B | 126.166641 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      213.695492      |
| 256        |      32.00 KB     |        1024        |      232.483726      |
| 256        |      32.00 KB     |        2048        |      241.990990      |
| 256        |      32.00 KB     |        4096        |      248.976859      |
| 256        |      32.00 KB     |        8192        |      251.941019      |
| 256        |      32.00 KB     |        16384        |      252.674395      |
| 256        |      16.00 KB     |        512        |      212.904510      |
| 256        |      16.00 KB     |        1024        |      232.616788      |
| 256        |      16.00 KB     |        2048        |      244.296891      |
| 256        |      16.00 KB     |        4096        |      253.089715      |
| 256        |      16.00 KB     |        8192        |      256.236468      |
| 256        |      8.00 KB     |        512        |      216.476184      |
| 256        |      8.00 KB     |        1024        |      233.600371      |
| 256        |      8.00 KB     |        2048        |      244.147861      |
| 256        |      8.00 KB     |        4096        |      248.289449      |
| 256        |      4.00 KB     |        512        |      200.848925      |
| 256        |      4.00 KB     |        1024        |      219.624665      |
| 256        |      4.00 KB     |        2048        |      229.612501      |
| 256        |      2.00 KB     |        512        |      184.364363      |
| 256        |      2.00 KB     |        1024        |      197.558255      |
| 256        |      1024.00 B     |        512        |      150.539808      |
| 128        |      32.00 KB     |        512        |      214.053639      |
| 128        |      32.00 KB     |        1024        |      232.725207      |
| 128        |      32.00 KB     |        2048        |      243.615800      |
| 128        |      32.00 KB     |        4096        |      250.617476      |
| 128        |      32.00 KB     |        8192        |      252.260436      |
| 128        |      32.00 KB     |        16384        |      252.754791      |
| 128        |      16.00 KB     |        512        |      215.377031      |
| 128        |      16.00 KB     |        1024        |      237.474012      |
| 128        |      16.00 KB     |        2048        |      248.792548      |
| 128        |      16.00 KB     |        4096        |      256.453301      |
| 128        |      16.00 KB     |        8192        |      259.178992      |
| 128        |      8.00 KB     |        512        |      216.798439      |
| 128        |      8.00 KB     |        1024        |      235.599055      |
| 128        |      8.00 KB     |        2048        |      244.891401      |
| 128        |      8.00 KB     |        4096        |      250.927539      |
| 128        |      4.00 KB     |        512        |      205.442006      |
| 128        |      4.00 KB     |        1024        |      221.746265      |
| 128        |      4.00 KB     |        2048        |      231.420601      |
| 128        |      2.00 KB     |        512        |      184.905341      |
| 128        |      2.00 KB     |        1024        |      197.219380      |
| 128        |      1024.00 B     |        512        |      149.019965      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    49   
    1024      |      TCMA     |     TCMA           |    45   
    1024      |      TCMB     |     TCMB           |    46   
    1024      |      OCRAM    |     TCMA           |    46   
    1024      |      TCMA     |     OCRAM          |    45   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.048000 		| 655			| 277.201996		|
cos  		|0.0000002870	| 442			| 65.793999 		| 511			| 277.641998		|
sincos sin  	|0.0000001790	| 79			| 78.994003 		| 481			| 276.058014		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| -260			| 73.379997 		| 583			| 428.884003		|
acos 		|0.0000004770	| 76			| 76.000000 		| 656			| 383.851990		|
atan 		|0.0000005360	| 80			| 80.019997 		| 511			| 370.962006		|
atan2 		|0.0000007150	| 117			| 104.662003 		| 891			| 480.712006		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

