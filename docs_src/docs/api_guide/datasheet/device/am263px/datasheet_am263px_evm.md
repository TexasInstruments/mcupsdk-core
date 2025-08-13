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
- Size of ipc_notify_echo          : 98 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   632
SBL : Drivers_open                      |   130
SBL : LoadHsmRtFw                       |   8469
SBL : Board_driversOpen                 |   1939
SBL : CPU Load                          |   5679
SBL : SBL End                           |   18
SBL : Total time taken                  |   16869

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI MULTICORE ELF performance

- Software/Application used           : sbl_ospi_multicore_elf and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 290 KB
- Size of ipc_notify_echo             : 98 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   619
SBL : Drivers_open                      |   139
SBL : LoadHsmRtFw                       |   8468
SBL : Board_driversOpen                 |   1941
SBL : CPU Load                          |   5717
SBL : SBL End                           |   19
SBL : Total time taken                  |   16907

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 290 KB
- Size of ipc_notify_echo             : 98 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   624
SBL : Drivers_open                      |   136
SBL : LoadHsmRtFw                       |   8446
SBL : Board_driversOpen                 |   1898
SBL : CPU Load                          |   4782
SBL : SBL End                           |   19
SBL : Total time taken                  |   15909

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a>. 

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_sd appimage          : 282 KB
- Size of hello_world              : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   605
SBL : Drivers_open                      |   140536
SBL : LoadHsmRtFw                       |   8652
SBL : Board_driversOpen                 |   2824
SBL : File read from SD card            |   8623
SBL : CPU Load                          |   39
SBL : SBL End                           |   3949
SBL : Total time taken                  |   165231

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD MULTICORE ELF performance

- Software/Application used        : sbl_sd_multicore_elf and hello_world
- Size of sbl_sd appimage          : 302 KB
- Size of hello_world              : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   695
SBL : Drivers_open                      |   14524
SBL : LoadHsmRtFw                       |   8654
SBL : Board_driversOpen                 |   2836
SBL : File read from SD card            |   8573
SBL : CPU Load                          |   4165
SBL : SBL End                           |   15
SBL : Total time taken                  |   170186

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
 r5f0-0	| r5f1-0	|  1.89
 r5f0-0	| r5f1-1	|  1.95

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 12.216
 r5f0-0	| r5f1-0	| 4	| 12.451
 r5f0-0	| r5f1-1	| 4	| 12.275
 r5f0-0	| r5f0-1	| 32	| 15.254
 r5f0-0	| r5f0-1	| 64	| 18.146
 r5f0-0	| r5f0-1	| 112	| 22.459

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
 400		| 08			|  8.66 Mbps / 369.43 us 	| 22.19 Mbps / 144.20 us 	|  0.92 Mbps / 3475.06 us
 200		| 16			| 17.45 Mbps / 183.42 us 	| 28.72 Mbps / 111.43 us 	|  0.96 Mbps / 3346.79 us
 100		| 32			| 31.64 Mbps / 101.12 us 	| 33.57 Mbps / 95.32 us 	|  0.97 Mbps / 3282.82 us
- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 71
- End tick         : 10244352
- Total ticks      : 10244281
- Total time (secs): 10.244281
- Iterations/Sec   : 1464.231604
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1464.231604 
CoreMark/MHz :3.660579 / STACK

### DHRYSTONE

- BEGIN cycle count:                                33
- END Cycle count:                                  146623221
- USER cycle count:                                 146623188

BENCHMARK Using clock 400000000
- Usertime in sec:                                  0.366558
- Microseconds for one run through Dhrystone:       0.7 
- Dhrystones per Second:                            1364040.8 

Normalized MIPS/MHz:                                1.9409

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 193.472060 |
| 256 |  16.00 KB | 190.721616 |
| 256 |  8.00 KB | 187.986920 |
| 256 |  4.00 KB | 181.447181 |
| 256 |  2.00 KB | 163.525214 |
| 256 |  1024.00 B | 136.567475 |
| 256 |  512.00 B | 102.540994 |
| 128 |  32.00 KB | 194.494092 |
| 128 |  16.00 KB | 194.400341 |
| 128 |  8.00 KB | 192.105996 |
| 128 |  4.00 KB | 181.304119 |
| 128 |  2.00 KB | 164.650906 |
| 128 |  1024.00 B | 137.294172 |
| 128 |  512.00 B | 103.742164 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.165121 |
| 256 | Decryption |  32.00 KB | 88.274079 |
| 256 | Encryption |  16.00 KB | 88.364834 |
| 256 | Decryption |  16.00 KB | 88.128220 |
| 256 | Encryption |  8.00 KB | 88.081286 |
| 256 | Decryption |  8.00 KB | 88.680503 |
| 256 | Encryption |  4.00 KB | 89.607788 |
| 256 | Decryption |  4.00 KB | 90.421921 |
| 256 | Encryption |  2.00 KB | 89.424992 |
| 256 | Decryption |  2.00 KB | 88.972155 |
| 256 | Encryption |  1024.00 B | 86.561881 |
| 256 | Decryption |  1024.00 B | 86.641988 |
| 256 | Encryption |  512.00 B | 81.512438 |
| 256 | Decryption |  512.00 B | 81.516493 |
| 128 | Encryption |  32.00 KB | 96.109343 |
| 128 | Decryption |  32.00 KB | 95.811990 |
| 128 | Encryption |  16.00 KB | 95.407488 |
| 128 | Decryption |  16.00 KB | 95.832656 |
| 128 | Encryption |  8.00 KB | 95.879448 |
| 128 | Decryption |  8.00 KB | 95.681024 |
| 128 | Encryption |  4.00 KB | 97.675701 |
| 128 | Decryption |  4.00 KB | 98.616367 |
| 128 | Encryption |  2.00 KB | 97.104756 |
| 128 | Decryption |  2.00 KB | 97.228651 |
| 128 | Encryption |  1024.00 B | 93.791682 |
| 128 | Decryption |  1024.00 B | 93.899189 |
| 128 | Encryption |  512.00 B | 88.019770 |
| 128 | Decryption |  512.00 B | 88.147630 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 69.274026 |
| 256 | Decryption |  512.00 B | 256 | 73.457676 |
| 256 | Encryption |  1024.00 B | 256 | 79.412549 |
| 256 | Decryption |  1024.00 B | 256 | 80.262578 |
| 256 | Encryption |  1024.00 B | 512 | 82.414487 |
| 256 | Decryption |  1024.00 B | 512 | 82.333727 |
| 256 | Encryption |  2.00 KB | 256 | 82.969565 |
| 256 | Decryption |  2.00 KB | 256 | 83.906486 |
| 256 | Encryption |  2.00 KB | 512 | 86.414642 |
| 256 | Decryption |  2.00 KB | 512 | 86.381610 |
| 256 | Encryption |  2.00 KB | 1024 | 86.822198 |
| 256 | Decryption |  2.00 KB | 1024 | 87.256847 |
| 256 | Encryption |  4.00 KB | 256 | 84.058231 |
| 256 | Decryption |  4.00 KB | 256 | 85.893092 |
| 256 | Encryption |  4.00 KB | 512 | 88.060574 |
| 256 | Decryption |  4.00 KB | 512 | 88.526871 |
| 256 | Encryption |  4.00 KB | 1024 | 89.311047 |
| 256 | Decryption |  4.00 KB | 1024 | 89.561254 |
| 256 | Encryption |  4.00 KB | 2048 | 90.020123 |
| 256 | Decryption |  4.00 KB | 2048 | 89.742765 |
| 256 | Encryption |  8.00 KB | 256 | 84.489266 |
| 256 | Decryption |  8.00 KB | 256 | 85.067773 |
| 256 | Encryption |  8.00 KB | 512 | 87.039269 |
| 256 | Decryption |  8.00 KB | 512 | 86.914602 |
| 256 | Encryption |  8.00 KB | 1024 | 87.779560 |
| 256 | Decryption |  8.00 KB | 1024 | 87.887566 |
| 256 | Encryption |  8.00 KB | 2048 | 88.460851 |
| 256 | Decryption |  8.00 KB | 2048 | 88.165418 |
| 256 | Encryption |  8.00 KB | 4096 | 88.624139 |
| 256 | Decryption |  8.00 KB | 4096 | 88.458761 |
| 256 | Encryption |  16.00 KB | 256 | 84.199572 |
| 256 | Decryption |  16.00 KB | 256 | 84.579633 |
| 256 | Encryption |  16.00 KB | 512 | 86.228033 |
| 256 | Decryption |  16.00 KB | 512 | 86.116992 |
| 256 | Encryption |  16.00 KB | 1024 | 87.208515 |
| 256 | Decryption |  16.00 KB | 1024 | 87.034501 |
| 256 | Encryption |  16.00 KB | 2048 | 87.605750 |
| 256 | Decryption |  16.00 KB | 2048 | 87.466613 |
| 256 | Encryption |  16.00 KB | 4096 | 87.955832 |
| 256 | Decryption |  16.00 KB | 4096 | 87.716850 |
| 256 | Encryption |  16.00 KB | 8192 | 88.055989 |
| 256 | Decryption |  16.00 KB | 8192 | 87.805581 |
| 256 | Encryption |  32.00 KB | 256 | 84.075418 |
| 256 | Decryption |  32.00 KB | 256 | 83.929791 |
| 256 | Encryption |  32.00 KB | 512 | 86.261586 |
| 256 | Decryption |  32.00 KB | 512 | 86.414428 |
| 256 | Encryption |  32.00 KB | 1024 | 87.219178 |
| 256 | Decryption |  32.00 KB | 1024 | 87.362914 |
| 256 | Encryption |  32.00 KB | 2048 | 87.780368 |
| 256 | Decryption |  32.00 KB | 2048 | 87.704304 |
| 256 | Encryption |  32.00 KB | 4096 | 87.956496 |
| 256 | Decryption |  32.00 KB | 4096 | 88.010092 |
| 256 | Encryption |  32.00 KB | 8192 | 88.163416 |
| 256 | Decryption |  32.00 KB | 8192 | 88.032554 |
| 256 | Encryption |  32.00 KB | 16384 | 88.257361 |
| 256 | Decryption |  32.00 KB | 16384 | 88.110373 |
| 128 | Encryption |  512.00 B | 256 | 76.087865 |
| 128 | Decryption |  512.00 B | 256 | 78.324888 |
| 128 | Encryption |  1024.00 B | 256 | 84.479736 |
| 128 | Decryption |  1024.00 B | 256 | 85.609782 |
| 128 | Encryption |  1024.00 B | 512 | 89.067681 |
| 128 | Decryption |  1024.00 B | 512 | 89.116127 |
| 128 | Encryption |  2.00 KB | 256 | 89.422552 |
| 128 | Decryption |  2.00 KB | 256 | 90.414436 |
| 128 | Encryption |  2.00 KB | 512 | 92.719504 |
| 128 | Decryption |  2.00 KB | 512 | 93.298976 |
| 128 | Encryption |  2.00 KB | 1024 | 94.713414 |
| 128 | Decryption |  2.00 KB | 1024 | 94.786017 |
| 128 | Encryption |  4.00 KB | 256 | 91.972606 |
| 128 | Decryption |  4.00 KB | 256 | 92.195149 |
| 128 | Encryption |  4.00 KB | 512 | 95.447263 |
| 128 | Decryption |  4.00 KB | 512 | 95.527261 |
| 128 | Encryption |  4.00 KB | 1024 | 96.794966 |
| 128 | Decryption |  4.00 KB | 1024 | 97.049394 |
| 128 | Encryption |  4.00 KB | 2048 | 97.836829 |
| 128 | Decryption |  4.00 KB | 2048 | 97.459996 |
| 128 | Encryption |  8.00 KB | 256 | 90.705069 |
| 128 | Decryption |  8.00 KB | 256 | 90.753742 |
| 128 | Encryption |  8.00 KB | 512 | 93.372419 |
| 128 | Decryption |  8.00 KB | 512 | 93.449975 |
| 128 | Encryption |  8.00 KB | 1024 | 95.080647 |
| 128 | Decryption |  8.00 KB | 1024 | 94.933275 |
| 128 | Encryption |  8.00 KB | 2048 | 95.792209 |
| 128 | Decryption |  8.00 KB | 2048 | 95.533528 |
| 128 | Encryption |  8.00 KB | 4096 | 96.052646 |
| 128 | Decryption |  8.00 KB | 4096 | 96.209166 |
| 128 | Encryption |  16.00 KB | 256 | 89.995554 |
| 128 | Decryption |  16.00 KB | 256 | 90.070075 |
| 128 | Encryption |  16.00 KB | 512 | 92.929372 |
| 128 | Decryption |  16.00 KB | 512 | 92.610263 |
| 128 | Encryption |  16.00 KB | 1024 | 94.399121 |
| 128 | Decryption |  16.00 KB | 1024 | 93.825755 |
| 128 | Encryption |  16.00 KB | 2048 | 94.970592 |
| 128 | Decryption |  16.00 KB | 2048 | 94.718719 |
| 128 | Encryption |  16.00 KB | 4096 | 95.307935 |
| 128 | Decryption |  16.00 KB | 4096 | 95.184391 |
| 128 | Encryption |  16.00 KB | 8192 | 95.387006 |
| 128 | Decryption |  16.00 KB | 8192 | 95.413044 |
| 128 | Encryption |  32.00 KB | 256 | 90.231832 |
| 128 | Decryption |  32.00 KB | 256 | 90.329461 |
| 128 | Encryption |  32.00 KB | 512 | 93.057946 |
| 128 | Decryption |  32.00 KB | 512 | 92.822183 |
| 128 | Encryption |  32.00 KB | 1024 | 94.415696 |
| 128 | Decryption |  32.00 KB | 1024 | 94.259530 |
| 128 | Encryption |  32.00 KB | 2048 | 95.126967 |
| 128 | Decryption |  32.00 KB | 2048 | 94.973430 |
| 128 | Encryption |  32.00 KB | 4096 | 95.459341 |
| 128 | Decryption |  32.00 KB | 4096 | 95.428761 |
| 128 | Encryption |  32.00 KB | 8192 | 95.745763 |
| 128 | Decryption |  32.00 KB | 8192 | 95.583427 |
| 128 | Encryption |  32.00 KB | 16384 | 95.845357 |
| 128 | Decryption |  32.00 KB | 16384 | 95.606609 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.283443 |
| 256 | Decryption |  32.00 KB | 88.243026 |
| 256 | Encryption |  16.00 KB | 88.427877 |
| 256 | Decryption |  16.00 KB | 88.127479 |
| 256 | Encryption |  8.00 KB | 88.906676 |
| 256 | Decryption |  8.00 KB | 89.083421 |
| 256 | Encryption |  4.00 KB | 89.410962 |
| 256 | Decryption |  4.00 KB | 90.284896 |
| 256 | Encryption |  2.00 KB | 88.557375 |
| 256 | Decryption |  2.00 KB | 89.190109 |
| 256 | Encryption |  1024.00 B | 85.890278 |
| 256 | Decryption |  1024.00 B | 85.919555 |
| 256 | Encryption |  512.00 B | 80.003906 |
| 256 | Decryption |  512.00 B | 80.042992 |
| 128 | Encryption |  32.00 KB | 95.772785 |
| 128 | Decryption |  32.00 KB | 95.913301 |
| 128 | Encryption |  16.00 KB | 95.541710 |
| 128 | Decryption |  16.00 KB | 95.578809 |
| 128 | Encryption |  8.00 KB | 96.194338 |
| 128 | Decryption |  8.00 KB | 96.546136 |
| 128 | Encryption |  4.00 KB | 97.527438 |
| 128 | Decryption |  4.00 KB | 97.889438 |
| 128 | Encryption |  2.00 KB | 96.784960 |
| 128 | Decryption |  2.00 KB | 96.984047 |
| 128 | Encryption |  1024.00 B | 93.098844 |
| 128 | Decryption |  1024.00 B | 93.098844 |
| 128 | Encryption |  512.00 B | 86.372503 |
| 128 | Decryption |  512.00 B | 86.399831 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 68.150243 |
| 256 | Decryption |  512.00 B | 256 | 71.843894 |
| 256 | Encryption |  1024.00 B | 256 | 79.153582 |
| 256 | Decryption |  1024.00 B | 256 | 80.185978 |
| 256 | Encryption |  1024.00 B | 512 | 81.936387 |
| 256 | Decryption |  1024.00 B | 512 | 81.883153 |
| 256 | Encryption |  2.00 KB | 256 | 82.818582 |
| 256 | Decryption |  2.00 KB | 256 | 84.296096 |
| 256 | Encryption |  2.00 KB | 512 | 86.112608 |
| 256 | Decryption |  2.00 KB | 512 | 86.112608 |
| 256 | Encryption |  2.00 KB | 1024 | 86.502468 |
| 256 | Decryption |  2.00 KB | 1024 | 87.045916 |
| 256 | Encryption |  4.00 KB | 256 | 84.789598 |
| 256 | Decryption |  4.00 KB | 256 | 86.203790 |
| 256 | Encryption |  4.00 KB | 512 | 87.943586 |
| 256 | Decryption |  4.00 KB | 512 | 88.329998 |
| 256 | Encryption |  4.00 KB | 1024 | 89.040454 |
| 256 | Decryption |  4.00 KB | 1024 | 89.303745 |
| 256 | Encryption |  4.00 KB | 2048 | 89.907741 |
| 256 | Decryption |  4.00 KB | 2048 | 89.562478 |
| 256 | Encryption |  8.00 KB | 256 | 84.636023 |
| 256 | Decryption |  8.00 KB | 256 | 84.982834 |
| 256 | Encryption |  8.00 KB | 512 | 86.680091 |
| 256 | Decryption |  8.00 KB | 512 | 86.847798 |
| 256 | Encryption |  8.00 KB | 1024 | 87.713769 |
| 256 | Decryption |  8.00 KB | 1024 | 87.875486 |
| 256 | Encryption |  8.00 KB | 2048 | 88.227434 |
| 256 | Decryption |  8.00 KB | 2048 | 88.073888 |
| 256 | Encryption |  8.00 KB | 4096 | 88.460851 |
| 256 | Decryption |  8.00 KB | 4096 | 88.491309 |
| 256 | Encryption |  16.00 KB | 256 | 83.599168 |
| 256 | Decryption |  16.00 KB | 256 | 84.435519 |
| 256 | Encryption |  16.00 KB | 512 | 86.258960 |
| 256 | Decryption |  16.00 KB | 512 | 86.036158 |
| 256 | Encryption |  16.00 KB | 1024 | 87.052709 |
| 256 | Decryption |  16.00 KB | 1024 | 87.028722 |
| 256 | Encryption |  16.00 KB | 2048 | 87.545773 |
| 256 | Decryption |  16.00 KB | 2048 | 87.422568 |
| 256 | Encryption |  16.00 KB | 4096 | 87.832058 |
| 256 | Decryption |  16.00 KB | 4096 | 87.647050 |
| 256 | Encryption |  16.00 KB | 8192 | 88.019918 |
| 256 | Decryption |  16.00 KB | 8192 | 87.749149 |
| 256 | Encryption |  32.00 KB | 256 | 84.171252 |
| 256 | Decryption |  32.00 KB | 256 | 84.705966 |
| 256 | Encryption |  32.00 KB | 512 | 86.227253 |
| 256 | Decryption |  32.00 KB | 512 | 86.407877 |
| 256 | Encryption |  32.00 KB | 1024 | 87.264617 |
| 256 | Decryption |  32.00 KB | 1024 | 87.203656 |
| 256 | Encryption |  32.00 KB | 2048 | 87.732483 |
| 256 | Decryption |  32.00 KB | 2048 | 87.660605 |
| 256 | Encryption |  32.00 KB | 4096 | 87.954872 |
| 256 | Decryption |  32.00 KB | 4096 | 87.990300 |
| 256 | Encryption |  32.00 KB | 8192 | 88.123406 |
| 256 | Decryption |  32.00 KB | 8192 | 88.038615 |
| 256 | Encryption |  32.00 KB | 16384 | 88.276531 |
| 256 | Decryption |  32.00 KB | 16384 | 88.068932 |
| 128 | Encryption |  512.00 B | 256 | 75.035493 |
| 128 | Decryption |  512.00 B | 256 | 76.307578 |
| 128 | Encryption |  1024.00 B | 256 | 84.416622 |
| 128 | Decryption |  1024.00 B | 256 | 85.605309 |
| 128 | Encryption |  1024.00 B | 512 | 88.195080 |
| 128 | Decryption |  1024.00 B | 512 | 87.276601 |
| 128 | Encryption |  2.00 KB | 256 | 89.949080 |
| 128 | Decryption |  2.00 KB | 256 | 90.026925 |
| 128 | Encryption |  2.00 KB | 512 | 92.270436 |
| 128 | Decryption |  2.00 KB | 512 | 92.940409 |
| 128 | Encryption |  2.00 KB | 1024 | 94.368367 |
| 128 | Decryption |  2.00 KB | 1024 | 94.371085 |
| 128 | Encryption |  4.00 KB | 256 | 91.008318 |
| 128 | Decryption |  4.00 KB | 256 | 91.478343 |
| 128 | Encryption |  4.00 KB | 512 | 94.814129 |
| 128 | Decryption |  4.00 KB | 512 | 94.965259 |
| 128 | Encryption |  4.00 KB | 1024 | 96.576014 |
| 128 | Decryption |  4.00 KB | 1024 | 96.812840 |
| 128 | Encryption |  4.00 KB | 2048 | 97.384689 |
| 128 | Decryption |  4.00 KB | 2048 | 97.598606 |
| 128 | Encryption |  8.00 KB | 256 | 90.663974 |
| 128 | Decryption |  8.00 KB | 256 | 90.392301 |
| 128 | Encryption |  8.00 KB | 512 | 93.391713 |
| 128 | Decryption |  8.00 KB | 512 | 93.584087 |
| 128 | Encryption |  8.00 KB | 1024 | 94.913340 |
| 128 | Decryption |  8.00 KB | 1024 | 95.216025 |
| 128 | Encryption |  8.00 KB | 2048 | 95.761415 |
| 128 | Decryption |  8.00 KB | 2048 | 95.642624 |
| 128 | Encryption |  8.00 KB | 4096 | 95.945071 |
| 128 | Decryption |  8.00 KB | 4096 | 95.975983 |
| 128 | Encryption |  16.00 KB | 256 | 89.945839 |
| 128 | Decryption |  16.00 KB | 256 | 90.064505 |
| 128 | Encryption |  16.00 KB | 512 | 92.870770 |
| 128 | Decryption |  16.00 KB | 512 | 92.623352 |
| 128 | Encryption |  16.00 KB | 1024 | 94.275208 |
| 128 | Decryption |  16.00 KB | 1024 | 93.985541 |
| 128 | Encryption |  16.00 KB | 2048 | 94.928634 |
| 128 | Decryption |  16.00 KB | 2048 | 94.779677 |
| 128 | Encryption |  16.00 KB | 4096 | 95.271219 |
| 128 | Decryption |  16.00 KB | 4096 | 95.154332 |
| 128 | Encryption |  16.00 KB | 8192 | 95.445700 |
| 128 | Decryption |  16.00 KB | 8192 | 95.311401 |
| 128 | Encryption |  32.00 KB | 256 | 90.293370 |
| 128 | Decryption |  32.00 KB | 256 | 90.210873 |
| 128 | Encryption |  32.00 KB | 512 | 93.023272 |
| 128 | Decryption |  32.00 KB | 512 | 92.966036 |
| 128 | Encryption |  32.00 KB | 1024 | 94.484948 |
| 128 | Decryption |  32.00 KB | 1024 | 94.286822 |
| 128 | Encryption |  32.00 KB | 2048 | 95.079009 |
| 128 | Decryption |  32.00 KB | 2048 | 94.998727 |
| 128 | Encryption |  32.00 KB | 4096 | 95.475508 |
| 128 | Decryption |  32.00 KB | 4096 | 95.327951 |
| 128 | Encryption |  32.00 KB | 8192 | 95.747337 |
| 128 | Decryption |  32.00 KB | 8192 | 95.478986 |
| 128 | Encryption |  32.00 KB | 16384 | 95.733525 |
| 128 | Decryption |  32.00 KB | 16384 | 95.741742 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 284.859401 |
| 512 |  16.00 KB | 282.981330 |
| 512 |  8.00 KB | 279.936782 |
| 512 |  4.00 KB | 273.499708 |
| 512 |  2.00 KB | 262.049662 |
| 512 |  1024.00 B | 241.313793 |
| 512 |  512.00 B | 207.839655 |
| 256 |  32.00 KB | 275.044985 |
| 256 |  16.00 KB | 273.511122 |
| 256 |  8.00 KB | 270.837896 |
| 256 |  4.00 KB | 267.150398 |
| 256 |  2.00 KB | 258.066549 |
| 256 |  1024.00 B | 241.812412 |
| 256 |  512.00 B | 215.097808 |


### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 66.669379 |
| 256 | Decryption |  512.00 B | 256 | 71.461595 |
| 256 | Encryption |  1024.00 B | 256 | 79.230137 |
| 256 | Decryption |  1024.00 B | 256 | 79.094354 |
| 256 | Encryption |  1024.00 B | 512 | 81.465828 |
| 256 | Decryption |  1024.00 B | 512 | 82.051282 |
| 256 | Encryption |  2.00 KB | 256 | 83.096859 |
| 256 | Decryption |  2.00 KB | 256 | 83.885006 |
| 256 | Encryption |  2.00 KB | 512 | 85.840778 |
| 256 | Decryption |  2.00 KB | 512 | 86.043642 |
| 256 | Encryption |  2.00 KB | 1024 | 86.991611 |
| 256 | Decryption |  2.00 KB | 1024 | 86.632826 |
| 256 | Encryption |  4.00 KB | 256 | 83.723188 |
| 256 | Decryption |  4.00 KB | 256 | 85.915050 |
| 256 | Encryption |  4.00 KB | 512 | 88.082470 |
| 256 | Decryption |  4.00 KB | 512 | 88.042237 |
| 256 | Encryption |  4.00 KB | 1024 | 89.239903 |
| 256 | Decryption |  4.00 KB | 1024 | 89.105222 |
| 256 | Encryption |  4.00 KB | 2048 | 89.785182 |
| 256 | Decryption |  4.00 KB | 2048 | 89.776572 |
| 256 | Encryption |  8.00 KB | 256 | 84.646681 |
| 256 | Decryption |  8.00 KB | 256 | 84.489538 |
| 256 | Encryption |  8.00 KB | 512 | 86.627673 |
| 256 | Decryption |  8.00 KB | 512 | 86.598483 |
| 256 | Encryption |  8.00 KB | 1024 | 87.695869 |
| 256 | Decryption |  8.00 KB | 1024 | 87.835737 |
| 256 | Encryption |  8.00 KB | 2048 | 88.087205 |
| 256 | Decryption |  8.00 KB | 2048 | 88.107930 |
| 256 | Encryption |  8.00 KB | 4096 | 88.463537 |
| 256 | Decryption |  8.00 KB | 4096 | 88.449210 |
| 256 | Encryption |  16.00 KB | 256 | 84.180915 |
| 256 | Decryption |  16.00 KB | 256 | 84.059040 |
| 256 | Encryption |  16.00 KB | 512 | 86.086874 |
| 256 | Decryption |  16.00 KB | 512 | 85.856240 |
| 256 | Encryption |  16.00 KB | 1024 | 87.130108 |
| 256 | Decryption |  16.00 KB | 1024 | 86.932472 |
| 256 | Encryption |  16.00 KB | 2048 | 87.533934 |
| 256 | Decryption |  16.00 KB | 2048 | 87.410470 |
| 256 | Encryption |  16.00 KB | 4096 | 87.911734 |
| 256 | Decryption |  16.00 KB | 4096 | 87.608239 |
| 256 | Encryption |  16.00 KB | 8192 | 88.021543 |
| 256 | Decryption |  16.00 KB | 8192 | 87.750470 |
| 256 | Encryption |  32.00 KB | 256 | 84.140859 |
| 256 | Decryption |  32.00 KB | 256 | 83.728269 |
| 256 | Encryption |  32.00 KB | 512 | 86.276562 |
| 256 | Decryption |  32.00 KB | 512 | 85.705225 |
| 256 | Encryption |  32.00 KB | 1024 | 87.187486 |
| 256 | Decryption |  32.00 KB | 1024 | 87.148791 |
| 256 | Encryption |  32.00 KB | 2048 | 87.618341 |
| 256 | Decryption |  32.00 KB | 2048 | 87.707165 |
| 256 | Encryption |  32.00 KB | 4096 | 87.973985 |
| 256 | Decryption |  32.00 KB | 4096 | 87.856932 |
| 256 | Encryption |  32.00 KB | 8192 | 88.146000 |
| 256 | Decryption |  32.00 KB | 8192 | 88.005882 |
| 256 | Encryption |  32.00 KB | 16384 | 88.294891 |
| 256 | Decryption |  32.00 KB | 16384 | 88.084542 |
| 128 | Encryption |  512.00 B | 256 | 74.165950 |
| 128 | Decryption |  512.00 B | 256 | 77.159273 |
| 128 | Encryption |  1024.00 B | 256 | 84.792341 |
| 128 | Decryption |  1024.00 B | 256 | 85.486943 |
| 128 | Encryption |  1024.00 B | 512 | 88.076551 |
| 128 | Decryption |  1024.00 B | 512 | 88.330593 |
| 128 | Encryption |  2.00 KB | 256 | 88.774501 |
| 128 | Decryption |  2.00 KB | 256 | 89.453066 |
| 128 | Encryption |  2.00 KB | 512 | 92.598977 |
| 128 | Decryption |  2.00 KB | 512 | 92.889034 |
| 128 | Encryption |  2.00 KB | 1024 | 93.577405 |
| 128 | Decryption |  2.00 KB | 1024 | 94.062262 |
| 128 | Encryption |  4.00 KB | 256 | 89.906507 |
| 128 | Decryption |  4.00 KB | 256 | 91.378854 |
| 128 | Encryption |  4.00 KB | 512 | 95.181799 |
| 128 | Decryption |  4.00 KB | 512 | 94.851180 |
| 128 | Encryption |  4.00 KB | 1024 | 96.737101 |
| 128 | Decryption |  4.00 KB | 1024 | 96.519120 |
| 128 | Encryption |  4.00 KB | 2048 | 97.546309 |
| 128 | Decryption |  4.00 KB | 2048 | 97.611689 |
| 128 | Encryption |  8.00 KB | 256 | 90.089112 |
| 128 | Decryption |  8.00 KB | 256 | 90.057543 |
| 128 | Encryption |  8.00 KB | 512 | 93.430657 |
| 128 | Decryption |  8.00 KB | 512 | 93.549688 |
| 128 | Encryption |  8.00 KB | 1024 | 94.681258 |
| 128 | Decryption |  8.00 KB | 1024 | 94.720944 |
| 128 | Encryption |  8.00 KB | 2048 | 95.649255 |
| 128 | Decryption |  8.00 KB | 2048 | 95.490028 |
| 128 | Encryption |  8.00 KB | 4096 | 96.094546 |
| 128 | Decryption |  8.00 KB | 4096 | 95.743577 |
| 128 | Encryption |  16.00 KB | 256 | 90.015332 |
| 128 | Decryption |  16.00 KB | 256 | 90.008378 |
| 128 | Encryption |  16.00 KB | 512 | 92.791052 |
| 128 | Decryption |  16.00 KB | 512 | 92.700815 |
| 128 | Encryption |  16.00 KB | 1024 | 94.329995 |
| 128 | Decryption |  16.00 KB | 1024 | 93.964822 |
| 128 | Encryption |  16.00 KB | 2048 | 95.036181 |
| 128 | Decryption |  16.00 KB | 2048 | 94.558991 |
| 128 | Encryption |  16.00 KB | 4096 | 95.073406 |
| 128 | Decryption |  16.00 KB | 4096 | 95.267238 |
| 128 | Encryption |  16.00 KB | 8192 | 95.345547 |
| 128 | Decryption |  16.00 KB | 8192 | 95.317466 |
| 128 | Encryption |  32.00 KB | 256 | 90.129378 |
| 128 | Decryption |  32.00 KB | 256 | 90.134491 |
| 128 | Encryption |  32.00 KB | 512 | 92.936867 |
| 128 | Decryption |  32.00 KB | 512 | 92.976999 |
| 128 | Encryption |  32.00 KB | 1024 | 94.393343 |
| 128 | Decryption |  32.00 KB | 1024 | 94.262665 |
| 128 | Encryption |  32.00 KB | 2048 | 95.135167 |
| 128 | Decryption |  32.00 KB | 2048 | 95.010692 |
| 128 | Encryption |  32.00 KB | 4096 | 95.443093 |
| 128 | Decryption |  32.00 KB | 4096 | 95.386312 |
| 128 | Encryption |  32.00 KB | 8192 | 95.697616 |
| 128 | Decryption |  32.00 KB | 8192 | 95.499595 |
| 128 | Encryption |  32.00 KB | 16384 | 95.769986 |
| 128 | Decryption |  32.00 KB | 16384 | 95.677358 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.228548 |
| 256 | Decryption |  32.00 KB | 88.297270 |
| 256 | Encryption |  16.00 KB | 88.040611 |
| 256 | Decryption |  16.00 KB | 88.111780 |
| 256 | Encryption |  8.00 KB | 87.705552 |
| 256 | Decryption |  8.00 KB | 87.833971 |
| 256 | Encryption |  4.00 KB | 89.440248 |
| 256 | Decryption |  4.00 KB | 90.317246 |
| 256 | Encryption |  2.00 KB | 88.921453 |
| 256 | Decryption |  2.00 KB | 88.703608 |
| 256 | Encryption |  1024.00 B | 85.591892 |
| 256 | Decryption |  1024.00 B | 85.816049 |
| 256 | Encryption |  512.00 B | 79.291487 |
| 256 | Decryption |  512.00 B | 80.451755 |
| 128 | Encryption |  32.00 KB | 95.863408 |
| 128 | Decryption |  32.00 KB | 95.831604 |
| 128 | Encryption |  16.00 KB | 95.388568 |
| 128 | Decryption |  16.00 KB | 95.574105 |
| 128 | Encryption |  8.00 KB | 95.676834 |
| 128 | Decryption |  8.00 KB | 96.533337 |
| 128 | Encryption |  4.00 KB | 97.750002 |
| 128 | Decryption |  4.00 KB | 98.549635 |
| 128 | Encryption |  2.00 KB | 96.050182 |
| 128 | Decryption |  2.00 KB | 96.956785 |
| 128 | Encryption |  1024.00 B | 92.598977 |
| 128 | Decryption |  1024.00 B | 92.919325 |
| 128 | Encryption |  512.00 B | 85.901536 |
| 128 | Decryption |  512.00 B | 86.045901 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 282.809605 |
| 512 |  16.00 KB | 279.115626 |
| 512 |  8.00 KB | 272.213165 |
| 512 |  4.00 KB | 259.199494 |
| 512 |  2.00 KB | 236.549359 |
| 512 |  1024.00 B | 201.364223 |
| 512 |  512.00 B | 155.697045 |
| 256 |  32.00 KB | 273.554648 |
| 256 |  16.00 KB | 270.590482 |
| 256 |  8.00 KB | 265.330621 |
| 256 |  4.00 KB | 256.958576 |
| 256 |  2.00 KB | 239.689854 |
| 256 |  1024.00 B | 211.665913 |
| 256 |  512.00 B | 171.542247 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 232.115250 |
| 256 |  16.00 KB | 229.724176 |
| 256 |  8.00 KB | 228.296727 |
| 256 |  4.00 KB | 215.454919 |
| 256 |  2.00 KB | 190.994667 |
| 256 |  1024.00 B | 155.416430 |
| 256 |  512.00 B | 112.853010 |
| 128 |  32.00 KB | 231.625080 |
| 128 |  16.00 KB | 232.900956 |
| 128 |  8.00 KB | 229.582337 |
| 128 |  4.00 KB | 216.315416 |
| 128 |  2.00 KB | 192.091919 |
| 128 |  1024.00 B | 156.897295 |
| 128 |  512.00 B | 114.693735 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      192.138733      |
| 256        |      32.00 KB     |        1024        |      210.767774      |
| 256        |      32.00 KB     |        2048        |      219.960521      |
| 256        |      32.00 KB     |        4096        |      226.525670      |
| 256        |      32.00 KB     |        8192        |      228.776891      |
| 256        |      32.00 KB     |        16384        |      227.898293      |
| 256        |      16.00 KB     |        512        |      191.266407      |
| 256        |      16.00 KB     |        1024        |      210.941190      |
| 256        |      16.00 KB     |        2048        |      221.625346      |
| 256        |      16.00 KB     |        4096        |      227.408490      |
| 256        |      16.00 KB     |        8192        |      232.268468      |
| 256        |      8.00 KB     |        512        |      191.737858      |
| 256        |      8.00 KB     |        1024        |      210.521920      |
| 256        |      8.00 KB     |        2048        |      220.587518      |
| 256        |      8.00 KB     |        4096        |      225.039489      |
| 256        |      4.00 KB     |        512        |      180.866301      |
| 256        |      4.00 KB     |        1024        |      198.203538      |
| 256        |      4.00 KB     |        2048        |      207.143309      |
| 256        |      2.00 KB     |        512        |      163.496657      |
| 256        |      2.00 KB     |        1024        |      177.186579      |
| 256        |      1024.00 B     |        512        |      134.355652      |
| 128        |      32.00 KB     |        512        |      193.746247      |
| 128        |      32.00 KB     |        1024        |      210.752100      |
| 128        |      32.00 KB     |        2048        |      220.308977      |
| 128        |      32.00 KB     |        4096        |      225.543384      |
| 128        |      32.00 KB     |        8192        |      229.680398      |
| 128        |      32.00 KB     |        16384        |      231.816082      |
| 128        |      16.00 KB     |        512        |      192.649534      |
| 128        |      16.00 KB     |        1024        |      213.135600      |
| 128        |      16.00 KB     |        2048        |      225.422650      |
| 128        |      16.00 KB     |        4096        |      231.302169      |
| 128        |      16.00 KB     |        8192        |      233.729504      |
| 128        |      8.00 KB     |        512        |      191.359953      |
| 128        |      8.00 KB     |        1024        |      210.073165      |
| 128        |      8.00 KB     |        2048        |      219.838315      |
| 128        |      8.00 KB     |        4096        |      226.231942      |
| 128        |      4.00 KB     |        512        |      182.620205      |
| 128        |      4.00 KB     |        1024        |      198.521750      |
| 128        |      4.00 KB     |        2048        |      207.714494      |
| 128        |      2.00 KB     |        512        |      164.370094      |
| 128        |      2.00 KB     |        1024        |      177.854972      |
| 128        |      1024.00 B     |        512        |      137.674888      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48   
    1024      |      TCMA     |     TCMA           |    45   
    1024      |      TCMB     |     TCMB           |    45   
    1024      |      OCRAM    |     TCMA           |    45   
    1024      |      TCMA     |     OCRAM          |    45   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 53			| 53.042000 		| 651			| 277.002014		|
cos  		|0.0000002870	| 65			| 65.036003 		| 497			| 277.473999		|
sincos sin  	|0.0000001790	| 79			| 79.001999 		| 638			| 275.950012		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 402			| 74.699997 		| 602			| 428.753998		|
acos 		|0.0000004770	| 403			| 76.694000 		| 551			| 383.257996		|
atan 		|0.0000005360	| 80			| 80.129997 		| 501			| 370.954010		|
atan2 		|0.0000007150	| 117			| 104.612000 		| 591			| 479.394012		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### XIP Benchmark

Comparing data from \ref EXAMPLES_OPTIFLASH_XIP_BENCHMARK and \ref EXAMPLES_OPTIFLASH_OCRAM_BENCHMARK, execution time of a code which throws ~3 Million I-Cache Miss per seconds is 2.2 times slower when it runs from OCRAM. 

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size (KB) | Time (us) | Throughput (Mbps) |
|-----------|-----------|-------------------|
| 1         | 545       | 14.678899         |
| 2         | 1084      | 14.760148         |
| 4         | 2163      | 14.794267         |
| 8         | 4325      | 14.797688         |
| 16        | 8646      | 14.804534         |
| 32        | 17285     | 14.810529         |
| 64        | 34573     | 14.809244         |
| 128       | 69151     | 14.808173         |
| 256       | 138283    | 14.810208         |
| 512       | 276590    | 14.808923         |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size (KB) | Time (us) | Throughput (Mbps) |
|-----------|-----------|-------------------|
| 1         | 545       | 14.678899         |
| 2         | 31        | 516.129032        |
| 4         | 45        | 711.111111        |
| 8         | 77        | 831.168831        |
| 16        | 142       | 901.408451        |
| 32        | 275       | 930.909091        |
| 64        | 531       | 964.218456        |
| 128       | 1049      | 976.167779        |
| 256       | 2086      | 981.783317        |
| 512       | 4161      | 984.378755        |