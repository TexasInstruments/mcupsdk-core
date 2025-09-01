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

### SBL OSPI MULTICORE ELF performance

- Software/Application used           : sbl_ospi_multicore_elf and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 299 KB
- Size of ipc_notify_echo             : 98 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   527
SBL : Drivers_open                      |   133
SBL : LoadHsmRtFw                       |   10369
SBL : Board_driversOpen                 |   2674
SBL : CPU Load                          |   6025
SBL : SBL End                           |   18
SBL : Total time taken                  |   19749

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 299 KB
- Size of hello_world                 : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   520
SBL : Drivers_open                      |   132
SBL : LoadHsmRtFw                       |   10331
SBL : Board_driversOpen                 |   1973
SBL : CPU Load                          |   4609
SBL : SBL End                           |   4
SBL : Total time taken                  |   17572

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a>. 

### SBL SD MULTICORE ELF performance

- Software/Application used           : sbl_sd_multicore_elf and hello_world
- Size of sbl_sd mcelf image          : 311 KB
- Size of hello_world                 : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   600
SBL : Drivers_open                      |   15801
SBL : LoadHsmRtFw                       |   10537
SBL : Board_driversOpen                 |   2853
SBL : File read from SD card            |   7753
SBL : CPU Load                          |   3898
SBL : SBL End                           |   2
SBL : Total time taken                  |   183662

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
 r5f0-0	| r5f0-1	|  1.96
 r5f0-0	| r5f1-0	|  1.90
 r5f0-0	| r5f1-1	|  2.00

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 11.972
 r5f0-0	| r5f1-0	| 4	| 12.131
 r5f0-0	| r5f1-1	| 4	| 12.062
 r5f0-0	| r5f0-1	| 32	| 15.000
 r5f0-0	| r5f0-1	| 64	| 17.930
 r5f0-0	| r5f0-1	| 112	| 22.351

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
 400		| 08			|  8.62 Mbps / 371.35 us 	| 22.19 Mbps / 144.18 us 	|  0.92 Mbps / 3474.66 us
 200		| 16			| 17.44 Mbps / 183.46 us 	| 28.71 Mbps / 111.46 us 	|  0.96 Mbps / 3346.35 us
 100		| 32			| 31.66 Mbps / 101.08 us 	| 33.69 Mbps / 94.99 us 	|  0.97 Mbps / 3282.43 us
- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 70
- End tick         : 10237982
- Total ticks      : 10237912
- Total time (secs): 10.237912
- Iterations/Sec   : 1465.142502
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.142502 
CoreMark/MHz :3.662856 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146628815
- USER cycle count:                          146628808

BENCHMARK Using clock 400000000
- Usertime in sec:                           0.366572
- Microseconds for one run through Dhrystone:   0.7 
- Dhrystones per Second:                     1363988.3 

Normalized MIPS/MHz:                         1.9408

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 189.954947 |
| 256 |  16.00 KB | 185.956636 |
| 256 |  8.00 KB | 186.420139 |
| 256 |  4.00 KB | 175.523267 |
| 256 |  2.00 KB | 155.830321 |
| 256 |  1024.00 B | 127.541647 |
| 256 |  512.00 B | 93.016918 |
| 128 |  32.00 KB | 188.829182 |
| 128 |  16.00 KB | 187.443869 |
| 128 |  8.00 KB | 186.603266 |
| 128 |  4.00 KB | 175.888352 |
| 128 |  2.00 KB | 154.785073 |
| 128 |  1024.00 B | 127.343386 |
| 128 |  512.00 B | 93.249858 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.395227 |
| 256 | Decryption |  32.00 KB | 88.268803 |
| 256 | Encryption |  16.00 KB | 88.004257 |
| 256 | Decryption |  16.00 KB | 87.944914 |
| 256 | Encryption |  8.00 KB | 88.335653 |
| 256 | Decryption |  8.00 KB | 88.869000 |
| 256 | Encryption |  4.00 KB | 89.250233 |
| 256 | Decryption |  4.00 KB | 90.262513 |
| 256 | Encryption |  2.00 KB | 88.644817 |
| 256 | Decryption |  2.00 KB | 88.248522 |
| 256 | Encryption |  1024.00 B | 85.131589 |
| 256 | Decryption |  1024.00 B | 85.175847 |
| 256 | Encryption |  512.00 B | 78.067375 |
| 256 | Decryption |  512.00 B | 78.516318 |
| 128 | Encryption |  32.00 KB | 95.753719 |
| 128 | Decryption |  32.00 KB | 95.858149 |
| 128 | Encryption |  16.00 KB | 95.846321 |
| 128 | Decryption |  16.00 KB | 95.485507 |
| 128 | Encryption |  8.00 KB | 95.280396 |
| 128 | Decryption |  8.00 KB | 96.140714 |
| 128 | Encryption |  4.00 KB | 97.020659 |
| 128 | Decryption |  4.00 KB | 97.707010 |
| 128 | Encryption |  2.00 KB | 95.537706 |
| 128 | Decryption |  2.00 KB | 96.249082 |
| 128 | Encryption |  1024.00 B | 91.385225 |
| 128 | Decryption |  1024.00 B | 92.104450 |
| 128 | Encryption |  512.00 B | 83.647317 |
| 128 | Decryption |  512.00 B | 84.362288 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 65.567472 |
| 256 | Decryption |  512.00 B | 256 | 71.284372 |
| 256 | Encryption |  1024.00 B | 256 | 76.767014 |
| 256 | Decryption |  1024.00 B | 256 | 78.287462 |
| 256 | Encryption |  1024.00 B | 512 | 81.407135 |
| 256 | Decryption |  1024.00 B | 512 | 81.177228 |
| 256 | Encryption |  2.00 KB | 256 | 82.262417 |
| 256 | Decryption |  2.00 KB | 256 | 82.971666 |
| 256 | Encryption |  2.00 KB | 512 | 85.422315 |
| 256 | Decryption |  2.00 KB | 512 | 85.276704 |
| 256 | Encryption |  2.00 KB | 1024 | 86.793452 |
| 256 | Decryption |  2.00 KB | 1024 | 86.210026 |
| 256 | Encryption |  4.00 KB | 256 | 83.733885 |
| 256 | Decryption |  4.00 KB | 256 | 84.318872 |
| 256 | Encryption |  4.00 KB | 512 | 87.381916 |
| 256 | Decryption |  4.00 KB | 512 | 87.640750 |
| 256 | Encryption |  4.00 KB | 1024 | 88.940158 |
| 256 | Decryption |  4.00 KB | 1024 | 88.719820 |
| 256 | Encryption |  4.00 KB | 2048 | 89.566150 |
| 256 | Decryption |  4.00 KB | 2048 | 89.203463 |
| 256 | Encryption |  8.00 KB | 256 | 83.662267 |
| 256 | Decryption |  8.00 KB | 256 | 83.403806 |
| 256 | Encryption |  8.00 KB | 512 | 86.322160 |
| 256 | Decryption |  8.00 KB | 512 | 86.021901 |
| 256 | Encryption |  8.00 KB | 1024 | 87.256556 |
| 256 | Decryption |  8.00 KB | 1024 | 87.112736 |
| 256 | Encryption |  8.00 KB | 2048 | 87.905543 |
| 256 | Decryption |  8.00 KB | 2048 | 87.960996 |
| 256 | Encryption |  8.00 KB | 4096 | 88.412518 |
| 256 | Decryption |  8.00 KB | 4096 | 88.451598 |
| 256 | Encryption |  16.00 KB | 256 | 83.217544 |
| 256 | Decryption |  16.00 KB | 256 | 83.251107 |
| 256 | Encryption |  16.00 KB | 512 | 85.633692 |
| 256 | Decryption |  16.00 KB | 512 | 85.398244 |
| 256 | Encryption |  16.00 KB | 1024 | 86.803655 |
| 256 | Decryption |  16.00 KB | 1024 | 86.610643 |
| 256 | Encryption |  16.00 KB | 2048 | 87.414405 |
| 256 | Decryption |  16.00 KB | 2048 | 87.281977 |
| 256 | Encryption |  16.00 KB | 4096 | 87.732703 |
| 256 | Decryption |  16.00 KB | 4096 | 87.424609 |
| 256 | Encryption |  16.00 KB | 8192 | 87.933999 |
| 256 | Decryption |  16.00 KB | 8192 | 87.765600 |
| 256 | Encryption |  32.00 KB | 256 | 83.279472 |
| 256 | Decryption |  32.00 KB | 256 | 83.460963 |
| 256 | Encryption |  32.00 KB | 512 | 85.795897 |
| 256 | Decryption |  32.00 KB | 512 | 85.713281 |
| 256 | Encryption |  32.00 KB | 1024 | 86.912513 |
| 256 | Decryption |  32.00 KB | 1024 | 86.927211 |
| 256 | Encryption |  32.00 KB | 2048 | 87.562369 |
| 256 | Decryption |  32.00 KB | 2048 | 87.507709 |
| 256 | Encryption |  32.00 KB | 4096 | 87.880863 |
| 256 | Decryption |  32.00 KB | 4096 | 87.839342 |
| 256 | Encryption |  32.00 KB | 8192 | 88.092312 |
| 256 | Decryption |  32.00 KB | 8192 | 87.969778 |
| 256 | Encryption |  32.00 KB | 16384 | 88.192780 |
| 256 | Decryption |  32.00 KB | 16384 | 87.992072 |
| 128 | Encryption |  512.00 B | 256 | 72.653097 |
| 128 | Decryption |  512.00 B | 256 | 73.116744 |
| 128 | Encryption |  1024.00 B | 256 | 83.547079 |
| 128 | Decryption |  1024.00 B | 256 | 84.926394 |
| 128 | Encryption |  1024.00 B | 512 | 87.269628 |
| 128 | Decryption |  1024.00 B | 512 | 87.530719 |
| 128 | Encryption |  2.00 KB | 256 | 88.416395 |
| 128 | Decryption |  2.00 KB | 256 | 89.050738 |
| 128 | Encryption |  2.00 KB | 512 | 92.429200 |
| 128 | Decryption |  2.00 KB | 512 | 92.373180 |
| 128 | Encryption |  2.00 KB | 1024 | 93.896498 |
| 128 | Decryption |  2.00 KB | 1024 | 93.884392 |
| 128 | Encryption |  4.00 KB | 256 | 91.987452 |
| 128 | Decryption |  4.00 KB | 256 | 91.817335 |
| 128 | Encryption |  4.00 KB | 512 | 95.052032 |
| 128 | Decryption |  4.00 KB | 512 | 94.716152 |
| 128 | Encryption |  4.00 KB | 1024 | 96.506328 |
| 128 | Decryption |  4.00 KB | 1024 | 96.514856 |
| 128 | Encryption |  4.00 KB | 2048 | 96.944594 |
| 128 | Decryption |  4.00 KB | 2048 | 97.300106 |
| 128 | Encryption |  8.00 KB | 256 | 90.369865 |
| 128 | Decryption |  8.00 KB | 256 | 90.404147 |
| 128 | Encryption |  8.00 KB | 512 | 93.234270 |
| 128 | Decryption |  8.00 KB | 512 | 93.333856 |
| 128 | Encryption |  8.00 KB | 1024 | 94.985561 |
| 128 | Decryption |  8.00 KB | 1024 | 94.624184 |
| 128 | Encryption |  8.00 KB | 2048 | 95.433017 |
| 128 | Decryption |  8.00 KB | 2048 | 95.572188 |
| 128 | Encryption |  8.00 KB | 4096 | 96.067430 |
| 128 | Decryption |  8.00 KB | 4096 | 95.923304 |
| 128 | Encryption |  16.00 KB | 256 | 89.884158 |
| 128 | Decryption |  16.00 KB | 256 | 89.827793 |
| 128 | Encryption |  16.00 KB | 512 | 92.664933 |
| 128 | Decryption |  16.00 KB | 512 | 92.610590 |
| 128 | Encryption |  16.00 KB | 1024 | 94.008460 |
| 128 | Decryption |  16.00 KB | 1024 | 94.016214 |
| 128 | Encryption |  16.00 KB | 2048 | 94.913511 |
| 128 | Decryption |  16.00 KB | 2048 | 94.564961 |
| 128 | Encryption |  16.00 KB | 4096 | 95.243182 |
| 128 | Decryption |  16.00 KB | 4096 | 95.007765 |
| 128 | Encryption |  16.00 KB | 8192 | 95.404710 |
| 128 | Decryption |  16.00 KB | 8192 | 95.343293 |
| 128 | Encryption |  32.00 KB | 256 | 89.999185 |
| 128 | Decryption |  32.00 KB | 256 | 90.115513 |
| 128 | Encryption |  32.00 KB | 512 | 92.952026 |
| 128 | Decryption |  32.00 KB | 512 | 92.812160 |
| 128 | Encryption |  32.00 KB | 1024 | 94.266139 |
| 128 | Decryption |  32.00 KB | 1024 | 94.233609 |
| 128 | Encryption |  32.00 KB | 2048 | 94.975151 |
| 128 | Decryption |  32.00 KB | 2048 | 94.970334 |
| 128 | Encryption |  32.00 KB | 4096 | 95.437273 |
| 128 | Decryption |  32.00 KB | 4096 | 95.333498 |
| 128 | Encryption |  32.00 KB | 8192 | 95.569314 |
| 128 | Decryption |  32.00 KB | 8192 | 95.482463 |
| 128 | Encryption |  32.00 KB | 16384 | 95.624046 |
| 128 | Decryption |  32.00 KB | 16384 | 95.649691 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.252681 |
| 256 | Decryption |  32.00 KB | 88.088389 |
| 256 | Encryption |  16.00 KB | 87.755905 |
| 256 | Decryption |  16.00 KB | 87.988454 |
| 256 | Encryption |  8.00 KB | 87.636355 |
| 256 | Decryption |  8.00 KB | 87.501794 |
| 256 | Encryption |  4.00 KB | 88.899138 |
| 256 | Decryption |  4.00 KB | 89.527609 |
| 256 | Encryption |  2.00 KB | 88.201015 |
| 256 | Decryption |  2.00 KB | 87.568145 |
| 256 | Encryption |  1024.00 B | 84.556034 |
| 256 | Decryption |  1024.00 B | 84.682776 |
| 256 | Encryption |  512.00 B | 77.627215 |
| 256 | Decryption |  512.00 B | 77.634572 |
| 128 | Encryption |  32.00 KB | 95.725135 |
| 128 | Decryption |  32.00 KB | 95.718581 |
| 128 | Encryption |  16.00 KB | 95.504466 |
| 128 | Decryption |  16.00 KB | 95.497681 |
| 128 | Encryption |  8.00 KB | 95.563130 |
| 128 | Decryption |  8.00 KB | 94.786702 |
| 128 | Encryption |  4.00 KB | 96.731389 |
| 128 | Decryption |  4.00 KB | 97.152260 |
| 128 | Encryption |  2.00 KB | 95.518211 |
| 128 | Decryption |  2.00 KB | 96.026257 |
| 128 | Encryption |  1024.00 B | 90.838023 |
| 128 | Decryption |  1024.00 B | 91.405618 |
| 128 | Encryption |  512.00 B | 82.914980 |
| 128 | Decryption |  512.00 B | 83.515139 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 64.756334 |
| 256 | Decryption |  512.00 B | 256 | 69.918491 |
| 256 | Encryption |  1024.00 B | 256 | 76.277381 |
| 256 | Decryption |  1024.00 B | 256 | 78.244466 |
| 256 | Encryption |  1024.00 B | 512 | 80.703396 |
| 256 | Decryption |  1024.00 B | 512 | 80.465585 |
| 256 | Encryption |  2.00 KB | 256 | 81.916928 |
| 256 | Decryption |  2.00 KB | 256 | 82.555679 |
| 256 | Encryption |  2.00 KB | 512 | 85.013426 |
| 256 | Decryption |  2.00 KB | 512 | 85.069706 |
| 256 | Encryption |  2.00 KB | 1024 | 86.449979 |
| 256 | Decryption |  2.00 KB | 1024 | 85.692617 |
| 256 | Encryption |  4.00 KB | 256 | 84.450888 |
| 256 | Decryption |  4.00 KB | 256 | 84.732592 |
| 256 | Encryption |  4.00 KB | 512 | 87.103184 |
| 256 | Decryption |  4.00 KB | 512 | 87.436127 |
| 256 | Encryption |  4.00 KB | 1024 | 88.754664 |
| 256 | Decryption |  4.00 KB | 1024 | 88.353814 |
| 256 | Encryption |  4.00 KB | 2048 | 89.384134 |
| 256 | Decryption |  4.00 KB | 2048 | 89.061024 |
| 256 | Encryption |  8.00 KB | 256 | 83.547878 |
| 256 | Decryption |  8.00 KB | 256 | 83.576113 |
| 256 | Encryption |  8.00 KB | 512 | 86.056917 |
| 256 | Decryption |  8.00 KB | 512 | 85.918147 |
| 256 | Encryption |  8.00 KB | 1024 | 87.225200 |
| 256 | Decryption |  8.00 KB | 1024 | 87.213593 |
| 256 | Encryption |  8.00 KB | 2048 | 88.021248 |
| 256 | Decryption |  8.00 KB | 2048 | 87.886682 |
| 256 | Encryption |  8.00 KB | 4096 | 88.341011 |
| 256 | Decryption |  8.00 KB | 4096 | 87.932671 |
| 256 | Encryption |  16.00 KB | 256 | 83.171075 |
| 256 | Decryption |  16.00 KB | 256 | 83.191399 |
| 256 | Encryption |  16.00 KB | 512 | 85.631175 |
| 256 | Decryption |  16.00 KB | 512 | 85.396018 |
| 256 | Encryption |  16.00 KB | 1024 | 86.749077 |
| 256 | Decryption |  16.00 KB | 1024 | 86.569170 |
| 256 | Encryption |  16.00 KB | 2048 | 87.323554 |
| 256 | Decryption |  16.00 KB | 2048 | 87.212287 |
| 256 | Encryption |  16.00 KB | 4096 | 87.759430 |
| 256 | Decryption |  16.00 KB | 4096 | 87.348869 |
| 256 | Encryption |  16.00 KB | 8192 | 87.887124 |
| 256 | Decryption |  16.00 KB | 8192 | 87.602530 |
| 256 | Encryption |  32.00 KB | 256 | 83.447945 |
| 256 | Decryption |  32.00 KB | 256 | 83.400290 |
| 256 | Encryption |  32.00 KB | 512 | 85.781578 |
| 256 | Decryption |  32.00 KB | 512 | 85.708938 |
| 256 | Encryption |  32.00 KB | 1024 | 86.952585 |
| 256 | Decryption |  32.00 KB | 1024 | 86.786484 |
| 256 | Encryption |  32.00 KB | 2048 | 87.551987 |
| 256 | Decryption |  32.00 KB | 2048 | 87.484784 |
| 256 | Encryption |  32.00 KB | 4096 | 87.868344 |
| 256 | Decryption |  32.00 KB | 4096 | 87.786688 |
| 256 | Encryption |  32.00 KB | 8192 | 88.044824 |
| 256 | Decryption |  32.00 KB | 8192 | 87.940562 |
| 256 | Encryption |  32.00 KB | 16384 | 88.158079 |
| 256 | Decryption |  32.00 KB | 16384 | 88.048965 |
| 128 | Encryption |  512.00 B | 256 | 71.705545 |
| 128 | Decryption |  512.00 B | 256 | 74.283642 |
| 128 | Encryption |  1024.00 B | 256 | 82.927570 |
| 128 | Decryption |  1024.00 B | 256 | 82.534885 |
| 128 | Encryption |  1024.00 B | 512 | 86.402109 |
| 128 | Decryption |  1024.00 B | 512 | 86.892419 |
| 128 | Encryption |  2.00 KB | 256 | 88.075367 |
| 128 | Decryption |  2.00 KB | 256 | 88.545410 |
| 128 | Encryption |  2.00 KB | 512 | 91.917138 |
| 128 | Decryption |  2.00 KB | 512 | 92.087625 |
| 128 | Encryption |  2.00 KB | 1024 | 93.544013 |
| 128 | Decryption |  2.00 KB | 1024 | 93.493302 |
| 128 | Encryption |  4.00 KB | 256 | 90.253811 |
| 128 | Decryption |  4.00 KB | 256 | 91.309467 |
| 128 | Encryption |  4.00 KB | 512 | 94.789444 |
| 128 | Decryption |  4.00 KB | 512 | 94.545350 |
| 128 | Encryption |  4.00 KB | 1024 | 96.324767 |
| 128 | Decryption |  4.00 KB | 1024 | 96.326183 |
| 128 | Encryption |  4.00 KB | 2048 | 96.687149 |
| 128 | Decryption |  4.00 KB | 2048 | 96.987635 |
| 128 | Encryption |  8.00 KB | 256 | 90.170921 |
| 128 | Decryption |  8.00 KB | 256 | 90.365503 |
| 128 | Encryption |  8.00 KB | 512 | 92.928219 |
| 128 | Decryption |  8.00 KB | 512 | 92.994480 |
| 128 | Encryption |  8.00 KB | 1024 | 94.970420 |
| 128 | Decryption |  8.00 KB | 1024 | 94.359196 |
| 128 | Encryption |  8.00 KB | 2048 | 95.202194 |
| 128 | Decryption |  8.00 KB | 2048 | 95.234705 |
| 128 | Encryption |  8.00 KB | 4096 | 95.637739 |
| 128 | Decryption |  8.00 KB | 4096 | 95.777859 |
| 128 | Encryption |  16.00 KB | 256 | 89.996172 |
| 128 | Decryption |  16.00 KB | 256 | 89.923624 |
| 128 | Encryption |  16.00 KB | 512 | 92.701962 |
| 128 | Decryption |  16.00 KB | 512 | 92.458051 |
| 128 | Encryption |  16.00 KB | 1024 | 93.979139 |
| 128 | Decryption |  16.00 KB | 1024 | 93.966674 |
| 128 | Encryption |  16.00 KB | 2048 | 94.840370 |
| 128 | Decryption |  16.00 KB | 2048 | 94.530860 |
| 128 | Encryption |  16.00 KB | 4096 | 95.122566 |
| 128 | Decryption |  16.00 KB | 4096 | 94.963711 |
| 128 | Encryption |  16.00 KB | 8192 | 95.330117 |
| 128 | Decryption |  16.00 KB | 8192 | 95.142935 |
| 128 | Encryption |  32.00 KB | 256 | 90.044010 |
| 128 | Decryption |  32.00 KB | 256 | 90.168207 |
| 128 | Encryption |  32.00 KB | 512 | 92.914303 |
| 128 | Decryption |  32.00 KB | 512 | 92.795732 |
| 128 | Encryption |  32.00 KB | 1024 | 94.249363 |
| 128 | Decryption |  32.00 KB | 1024 | 94.269699 |
| 128 | Encryption |  32.00 KB | 2048 | 95.031444 |
| 128 | Decryption |  32.00 KB | 2048 | 94.903804 |
| 128 | Encryption |  32.00 KB | 4096 | 95.407401 |
| 128 | Decryption |  32.00 KB | 4096 | 95.264901 |
| 128 | Encryption |  32.00 KB | 8192 | 95.548239 |
| 128 | Decryption |  32.00 KB | 8192 | 95.474639 |
| 128 | Encryption |  32.00 KB | 16384 | 95.605912 |
| 128 | Decryption |  32.00 KB | 16384 | 95.603297 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 284.314103 |
| 512 |  16.00 KB | 283.164734 |
| 512 |  8.00 KB | 279.841155 |
| 512 |  4.00 KB | 273.573918 |
| 512 |  2.00 KB | 261.610315 |
| 512 |  1024.00 B | 240.852628 |
| 512 |  512.00 B | 207.786937 |
| 256 |  32.00 KB | 275.044264 |
| 256 |  16.00 KB | 274.073029 |
| 256 |  8.00 KB | 270.310067 |
| 256 |  4.00 KB | 266.867556 |
| 256 |  2.00 KB | 257.904057 |
| 256 |  1024.00 B | 241.776728 |
| 256 |  512.00 B | 215.154301 |


### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 65.324349 |
| 256 | Decryption |  512.00 B | 256 | 70.356851 |
| 256 | Encryption |  1024.00 B | 256 | 77.061286 |
| 256 | Decryption |  1024.00 B | 256 | 78.020905 |
| 256 | Encryption |  1024.00 B | 512 | 79.746897 |
| 256 | Decryption |  1024.00 B | 512 | 80.211495 |
| 256 | Encryption |  2.00 KB | 256 | 82.294440 |
| 256 | Decryption |  2.00 KB | 256 | 82.220103 |
| 256 | Encryption |  2.00 KB | 512 | 84.955018 |
| 256 | Decryption |  2.00 KB | 512 | 85.038798 |
| 256 | Encryption |  2.00 KB | 1024 | 86.252007 |
| 256 | Decryption |  2.00 KB | 1024 | 86.263360 |
| 256 | Encryption |  4.00 KB | 256 | 84.077643 |
| 256 | Decryption |  4.00 KB | 256 | 84.574039 |
| 256 | Encryption |  4.00 KB | 512 | 87.234779 |
| 256 | Decryption |  4.00 KB | 512 | 87.264980 |
| 256 | Encryption |  4.00 KB | 1024 | 88.714415 |
| 256 | Decryption |  4.00 KB | 1024 | 88.539429 |
| 256 | Encryption |  4.00 KB | 2048 | 89.329917 |
| 256 | Decryption |  4.00 KB | 2048 | 89.376210 |
| 256 | Encryption |  8.00 KB | 256 | 83.462491 |
| 256 | Decryption |  8.00 KB | 256 | 83.881248 |
| 256 | Encryption |  8.00 KB | 512 | 85.960126 |
| 256 | Decryption |  8.00 KB | 512 | 85.844713 |
| 256 | Encryption |  8.00 KB | 1024 | 87.358620 |
| 256 | Decryption |  8.00 KB | 1024 | 87.248715 |
| 256 | Encryption |  8.00 KB | 2048 | 87.914093 |
| 256 | Decryption |  8.00 KB | 2048 | 88.193300 |
| 256 | Encryption |  8.00 KB | 4096 | 88.374665 |
| 256 | Decryption |  8.00 KB | 4096 | 88.152966 |
| 256 | Encryption |  16.00 KB | 256 | 83.113721 |
| 256 | Decryption |  16.00 KB | 256 | 83.084218 |
| 256 | Encryption |  16.00 KB | 512 | 85.484713 |
| 256 | Decryption |  16.00 KB | 512 | 85.419531 |
| 256 | Encryption |  16.00 KB | 1024 | 86.848662 |
| 256 | Decryption |  16.00 KB | 1024 | 86.577033 |
| 256 | Encryption |  16.00 KB | 2048 | 87.358911 |
| 256 | Decryption |  16.00 KB | 2048 | 87.184297 |
| 256 | Encryption |  16.00 KB | 4096 | 87.660679 |
| 256 | Decryption |  16.00 KB | 4096 | 87.381042 |
| 256 | Encryption |  16.00 KB | 8192 | 87.731382 |
| 256 | Decryption |  16.00 KB | 8192 | 87.787497 |
| 256 | Encryption |  32.00 KB | 256 | 83.334075 |
| 256 | Decryption |  32.00 KB | 256 | 83.370782 |
| 256 | Encryption |  32.00 KB | 512 | 85.802847 |
| 256 | Decryption |  32.00 KB | 512 | 85.651739 |
| 256 | Encryption |  32.00 KB | 1024 | 86.944005 |
| 256 | Decryption |  32.00 KB | 1024 | 86.901348 |
| 256 | Encryption |  32.00 KB | 2048 | 87.555423 |
| 256 | Decryption |  32.00 KB | 2048 | 87.442835 |
| 256 | Encryption |  32.00 KB | 4096 | 87.859582 |
| 256 | Decryption |  32.00 KB | 4096 | 87.844273 |
| 256 | Encryption |  32.00 KB | 8192 | 88.082766 |
| 256 | Decryption |  32.00 KB | 8192 | 87.940046 |
| 256 | Encryption |  32.00 KB | 16384 | 88.137554 |
| 256 | Decryption |  32.00 KB | 16384 | 87.998939 |
| 128 | Encryption |  512.00 B | 256 | 71.802963 |
| 128 | Decryption |  512.00 B | 256 | 74.266806 |
| 128 | Encryption |  1024.00 B | 256 | 83.657995 |
| 128 | Decryption |  1024.00 B | 256 | 84.251665 |
| 128 | Encryption |  1024.00 B | 512 | 86.511603 |
| 128 | Decryption |  1024.00 B | 512 | 86.871686 |
| 128 | Encryption |  2.00 KB | 256 | 87.629031 |
| 128 | Decryption |  2.00 KB | 256 | 88.546607 |
| 128 | Encryption |  2.00 KB | 512 | 91.848862 |
| 128 | Decryption |  2.00 KB | 512 | 92.039773 |
| 128 | Encryption |  2.00 KB | 1024 | 92.604211 |
| 128 | Decryption |  2.00 KB | 1024 | 93.330865 |
| 128 | Encryption |  4.00 KB | 256 | 90.049191 |
| 128 | Decryption |  4.00 KB | 256 | 91.042454 |
| 128 | Encryption |  4.00 KB | 512 | 94.411871 |
| 128 | Decryption |  4.00 KB | 512 | 94.742168 |
| 128 | Encryption |  4.00 KB | 1024 | 96.271704 |
| 128 | Decryption |  4.00 KB | 1024 | 96.036107 |
| 128 | Encryption |  4.00 KB | 2048 | 97.091809 |
| 128 | Decryption |  4.00 KB | 2048 | 96.787819 |
| 128 | Encryption |  8.00 KB | 256 | 90.012705 |
| 128 | Decryption |  8.00 KB | 256 | 90.608193 |
| 128 | Encryption |  8.00 KB | 512 | 93.416673 |
| 128 | Decryption |  8.00 KB | 512 | 93.506642 |
| 128 | Encryption |  8.00 KB | 1024 | 94.783275 |
| 128 | Decryption |  8.00 KB | 1024 | 94.790815 |
| 128 | Encryption |  8.00 KB | 2048 | 95.699537 |
| 128 | Decryption |  8.00 KB | 2048 | 95.014824 |
| 128 | Encryption |  8.00 KB | 4096 | 95.562782 |
| 128 | Decryption |  8.00 KB | 4096 | 95.624831 |
| 128 | Encryption |  16.00 KB | 256 | 89.783029 |
| 128 | Decryption |  16.00 KB | 256 | 89.843956 |
| 128 | Encryption |  16.00 KB | 512 | 92.794008 |
| 128 | Decryption |  16.00 KB | 512 | 92.344545 |
| 128 | Encryption |  16.00 KB | 1024 | 94.113592 |
| 128 | Decryption |  16.00 KB | 1024 | 93.869600 |
| 128 | Encryption |  16.00 KB | 2048 | 94.683652 |
| 128 | Decryption |  16.00 KB | 2048 | 94.676813 |
| 128 | Encryption |  16.00 KB | 4096 | 95.048241 |
| 128 | Decryption |  16.00 KB | 4096 | 94.923994 |
| 128 | Encryption |  16.00 KB | 8192 | 95.275548 |
| 128 | Decryption |  16.00 KB | 8192 | 95.007937 |
| 128 | Encryption |  32.00 KB | 256 | 90.017187 |
| 128 | Decryption |  32.00 KB | 256 | 90.088802 |
| 128 | Encryption |  32.00 KB | 512 | 92.921137 |
| 128 | Decryption |  32.00 KB | 512 | 92.805670 |
| 128 | Encryption |  32.00 KB | 1024 | 94.362762 |
| 128 | Decryption |  32.00 KB | 1024 | 94.208125 |
| 128 | Encryption |  32.00 KB | 2048 | 94.965087 |
| 128 | Decryption |  32.00 KB | 2048 | 94.902516 |
| 128 | Encryption |  32.00 KB | 4096 | 95.367399 |
| 128 | Decryption |  32.00 KB | 4096 | 95.280223 |
| 128 | Encryption |  32.00 KB | 8192 | 95.625878 |
| 128 | Decryption |  32.00 KB | 8192 | 95.479073 |
| 128 | Encryption |  32.00 KB | 16384 | 95.695694 |
| 128 | Decryption |  32.00 KB | 16384 | 95.515253 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.195302 |
| 256 | Decryption |  32.00 KB | 88.128442 |
| 256 | Encryption |  16.00 KB | 87.997167 |
| 256 | Decryption |  16.00 KB | 87.850602 |
| 256 | Encryption |  8.00 KB | 87.823083 |
| 256 | Decryption |  8.00 KB | 87.763984 |
| 256 | Encryption |  4.00 KB | 89.379258 |
| 256 | Decryption |  4.00 KB | 90.052903 |
| 256 | Encryption |  2.00 KB | 88.139332 |
| 256 | Decryption |  2.00 KB | 88.422359 |
| 256 | Encryption |  1024.00 B | 84.260331 |
| 256 | Decryption |  1024.00 B | 84.523318 |
| 256 | Encryption |  512.00 B | 76.618032 |
| 256 | Decryption |  512.00 B | 77.017816 |
| 128 | Encryption |  32.00 KB | 95.825474 |
| 128 | Decryption |  32.00 KB | 95.873750 |
| 128 | Encryption |  16.00 KB | 95.213777 |
| 128 | Decryption |  16.00 KB | 95.410440 |
| 128 | Encryption |  8.00 KB | 95.401759 |
| 128 | Decryption |  8.00 KB | 96.087502 |
| 128 | Encryption |  4.00 KB | 96.480755 |
| 128 | Decryption |  4.00 KB | 97.840481 |
| 128 | Encryption |  2.00 KB | 95.547456 |
| 128 | Decryption |  2.00 KB | 95.147943 |
| 128 | Encryption |  1024.00 B | 90.424416 |
| 128 | Decryption |  1024.00 B | 90.994418 |
| 128 | Encryption |  512.00 B | 82.211852 |
| 128 | Decryption |  512.00 B | 83.154850 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 282.164260 |
| 512 |  16.00 KB | 279.082939 |
| 512 |  8.00 KB | 270.737199 |
| 512 |  4.00 KB | 258.784971 |
| 512 |  2.00 KB | 236.677501 |
| 512 |  1024.00 B | 196.450839 |
| 512 |  512.00 B | 154.814325 |
| 256 |  32.00 KB | 273.586053 |
| 256 |  16.00 KB | 271.287754 |
| 256 |  8.00 KB | 264.853451 |
| 256 |  4.00 KB | 256.511018 |
| 256 |  2.00 KB | 238.877346 |
| 256 |  1024.00 B | 210.848723 |
| 256 |  512.00 B | 170.329556 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 227.757710 |
| 256 |  16.00 KB | 228.913737 |
| 256 |  8.00 KB | 224.885046 |
| 256 |  4.00 KB | 208.338499 |
| 256 |  2.00 KB | 181.938314 |
| 256 |  1024.00 B | 144.365142 |
| 256 |  512.00 B | 101.953951 |
| 128 |  32.00 KB | 229.911046 |
| 128 |  16.00 KB | 230.351973 |
| 128 |  8.00 KB | 225.424589 |
| 128 |  4.00 KB | 209.574366 |
| 128 |  2.00 KB | 182.388957 |
| 128 |  1024.00 B | 145.029654 |
| 128 |  512.00 B | 102.849969 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      188.553128      |
| 256        |      32.00 KB     |        1024        |      206.878890      |
| 256        |      32.00 KB     |        2048        |      216.678824      |
| 256        |      32.00 KB     |        4096        |      223.090359      |
| 256        |      32.00 KB     |        8192        |      226.380910      |
| 256        |      32.00 KB     |        16384        |      226.519798      |
| 256        |      16.00 KB     |        512        |      186.830017      |
| 256        |      16.00 KB     |        1024        |      208.385699      |
| 256        |      16.00 KB     |        2048        |      218.844518      |
| 256        |      16.00 KB     |        4096        |      225.799338      |
| 256        |      16.00 KB     |        8192        |      229.388473      |
| 256        |      8.00 KB     |        512        |      185.948062      |
| 256        |      8.00 KB     |        1024        |      205.210421      |
| 256        |      8.00 KB     |        2048        |      215.603770      |
| 256        |      8.00 KB     |        4096        |      220.085635      |
| 256        |      4.00 KB     |        512        |      175.662056      |
| 256        |      4.00 KB     |        1024        |      191.614525      |
| 256        |      4.00 KB     |        2048        |      199.446118      |
| 256        |      2.00 KB     |        512        |      156.097561      |
| 256        |      2.00 KB     |        1024        |      168.707203      |
| 256        |      1024.00 B     |        512        |      127.432527      |
| 128        |      32.00 KB     |        512        |      189.076038      |
| 128        |      32.00 KB     |        1024        |      207.137171      |
| 128        |      32.00 KB     |        2048        |      216.813232      |
| 128        |      32.00 KB     |        4096        |      223.813193      |
| 128        |      32.00 KB     |        8192        |      224.675439      |
| 128        |      32.00 KB     |        16384        |      229.128827      |
| 128        |      16.00 KB     |        512        |      187.873749      |
| 128        |      16.00 KB     |        1024        |      209.363469      |
| 128        |      16.00 KB     |        2048        |      220.269640      |
| 128        |      16.00 KB     |        4096        |      227.231081      |
| 128        |      16.00 KB     |        8192        |      230.289241      |
| 128        |      8.00 KB     |        512        |      187.202925      |
| 128        |      8.00 KB     |        1024        |      205.425907      |
| 128        |      8.00 KB     |        2048        |      215.713768      |
| 128        |      8.00 KB     |        4096        |      221.140187      |
| 128        |      4.00 KB     |        512        |      176.025355      |
| 128        |      4.00 KB     |        1024        |      191.799584      |
| 128        |      4.00 KB     |        2048        |      201.043009      |
| 128        |      2.00 KB     |        512        |      154.777762      |
| 128        |      2.00 KB     |        1024        |      169.229975      |
| 128        |      1024.00 B     |        512        |      127.840200      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    53   
    1024      |      TCMA     |     TCMA           |    50   
    1024      |      TCMB     |     TCMB           |    51   
    1024      |      OCRAM    |     TCMA           |    49   
    1024      |      TCMA     |     OCRAM          |    50 

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.084000 		| 485			| 276.317993		|
cos  		|0.0000002870	| 65			| 65.073997 		| 488			| 277.615997		|
sincos sin  	|0.0000001790	| 79			| 78.961998 		| 477			| 275.466003		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.003998 		| 576			| 428.760010		|
acos 		|0.0000004770	| 76			| 76.031998 		| 820			| 383.955994		|
atan 		|0.0000005360	| 80			| 80.015999 		| 652			| 371.529999		|
atan2 		|0.0000007150	| 117			| 104.627998 		| 573			| 479.410004		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### XIP Benchmark

Comparing data from \ref EXAMPLES_OPTIFLASH_XIP_BENCHMARK and \ref EXAMPLES_OPTIFLASH_OCRAM_BENCHMARK, execution time of a code which throws ~3 Million I-Cache Miss per seconds is 2.2 times slower when it runs from OCRAM. 

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      552 |        14.492754 |
|        2 |     1096 |        14.598540 |
|        4 |     2195 |        14.578588 |
|        8 |     4385 |        14.595211 |
|       16 |     8771 |        14.593547 |
|       32 |    17532 |        14.601871 |
|       64 |    35068 |        14.600205 |
|      128 |    70135 |        14.600413 |
|      256 |   140252 |        14.602287 |
|      512 |   280518 |        14.601559 |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      553 |        14.466546 |
|        2 |       31 |       516.129032 |
|        4 |       44 |       727.272727 |
|        8 |       77 |       831.168831 |
|       16 |      142 |       901.408451 |
|       32 |      273 |       937.728938 |
|       64 |      532 |       962.406015 |
|      128 |     1049 |       976.167779 |
|      256 |     2086 |       981.783317 |
|      512 |     4161 |       984.378755 |