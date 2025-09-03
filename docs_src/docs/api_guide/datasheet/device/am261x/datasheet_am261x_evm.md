#  Datasheet {#DATASHEET_AM261X_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM261x

## Generic Setup details

SOC Details             | Values
------------------------|------------------------------
Core                    | R5F
Core Operating Speed    | 500 MHz (AM261x-LP)
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
- Size of ipc_notify_echo          : 58 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   523
SBL : Drivers_open                      |   97
SBL : LoadHsmRtFw                       |   6249
SBL : Board_driversOpen                 |   4424
SBL : CPU Load                          |   4305
SBL : SBL End                           |   3
SBL : Total time taken                  |   15602

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI MULTICORE ELF performance

- Software/Application used           : sbl_ospi_multicore_elf and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 356 KB
- Size of ipc_notify_echo             : 58 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   515
SBL : Drivers_open                      |   96
SBL : LoadHsmRtFw                       |   6250
SBL : Board_driversOpen                 |   4439
SBL : CPU Load                          |   4383
SBL : SBL End                           |   9
SBL : Total time taken                  |   15695

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 356 KB
- Size of hello_world                 : 5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   508
SBL : Drivers_open                      |   91
SBL : LoadHsmRtFw                       |   6256
SBL : Board_driversOpen                 |   4415
SBL : CPU Load                          |   3983
SBL : SBL End                           |   9
SBL : Total time taken                  |   15265

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a>. 

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
 r5f0-0	| r5f0-1	|  1.40

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 9.768
 r5f0-0	| r5f0-1	| 32	| 11.877
 r5f0-0	| r5f0-1	| 64	| 14.035
 r5f0-0	| r5f0-1	| 112	| 17.366

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
 400		| 08			| 10.75 Mbps / 297.64 us 	| 24.21 Mbps / 132.20 us 	|  0.92 Mbps / 3463.49 us
 200		| 16			| 21.66 Mbps / 147.76 us 	| 30.55 Mbps / 104.74 us 	|  0.96 Mbps / 3339.31 us
 100		| 32			| 37.74 Mbps / 84.78 us 	| 35.68 Mbps / 89.69 us 	|  0.98 Mbps / 3277.23 us
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
- Begin tick       : 57
- End tick         : 8186031
- Total ticks      : 8185974
- Total time (secs): 8.185974
- Iterations/Sec   : 1832.402595
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1832.402595 
CoreMark/MHz :4.581006 / STACK

### DHRYSTONE

- BEGIN cycle count:                            30
- END Cycle count:                              146600012
- USER cycle count:                             146599982

BENCHMARK Using clock 500000000
- Usertime in sec:                              0.293200
- Microseconds for one run through Dhrystone:   0.6 
- Dhrystones per Second:                        1705320.9 

Normalized MIPS/MHz:                            1.9412

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 251.503877 |
| 256 |  16.00 KB | 254.284572 |
| 256 |  8.00 KB | 250.525624 |
| 256 |  4.00 KB | 236.773271 |
| 256 |  2.00 KB | 213.250033 |
| 256 |  1024.00 B | 177.508126 |
| 256 |  512.00 B | 133.437581 |
| 128 |  32.00 KB | 253.189693 |
| 128 |  16.00 KB | 253.065039 |
| 128 |  8.00 KB | 249.448090 |
| 128 |  4.00 KB | 237.442393 |
| 128 |  2.00 KB | 213.667188 |
| 128 |  1024.00 B | 178.498279 |
| 128 |  512.00 B | 134.674821 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 115.348500 |
| 256 | Decryption |  32.00 KB | 115.107004 |
| 256 | Encryption |  16.00 KB | 114.878095 |
| 256 | Decryption |  16.00 KB | 115.018217 |
| 256 | Encryption |  8.00 KB | 114.466165 |
| 256 | Decryption |  8.00 KB | 114.693333 |
| 256 | Encryption |  4.00 KB | 117.175878 |
| 256 | Decryption |  4.00 KB | 118.046299 |
| 256 | Encryption |  2.00 KB | 116.373555 |
| 256 | Decryption |  2.00 KB | 116.554030 |
| 256 | Encryption |  1024.00 B | 112.871669 |
| 256 | Decryption |  1024.00 B | 112.834357 |
| 256 | Encryption |  512.00 B | 106.191019 |
| 256 | Decryption |  512.00 B | 106.240598 |
| 128 | Encryption |  32.00 KB | 125.274545 |
| 128 | Decryption |  32.00 KB | 124.952930 |
| 128 | Encryption |  16.00 KB | 124.937327 |
| 128 | Decryption |  16.00 KB | 124.683943 |
| 128 | Encryption |  8.00 KB | 126.533008 |
| 128 | Decryption |  8.00 KB | 126.124878 |
| 128 | Encryption |  4.00 KB | 126.413901 |
| 128 | Decryption |  4.00 KB | 128.791869 |
| 128 | Encryption |  2.00 KB | 126.964446 |
| 128 | Decryption |  2.00 KB | 127.444422 |
| 128 | Encryption |  1024.00 B | 121.897506 |
| 128 | Decryption |  1024.00 B | 122.722915 |
| 128 | Encryption |  512.00 B | 114.298471 |
| 128 | Decryption |  512.00 B | 114.869034 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 88.146682 |
| 256 | Decryption |  512.00 B | 256 | 93.558703 |
| 256 | Encryption |  1024.00 B | 256 | 101.163279 |
| 256 | Decryption |  1024.00 B | 256 | 102.961138 |
| 256 | Encryption |  1024.00 B | 512 | 106.975894 |
| 256 | Decryption |  1024.00 B | 512 | 107.329088 |
| 256 | Encryption |  2.00 KB | 256 | 107.414935 |
| 256 | Decryption |  2.00 KB | 256 | 107.921536 |
| 256 | Encryption |  2.00 KB | 512 | 112.018159 |
| 256 | Decryption |  2.00 KB | 512 | 112.145439 |
| 256 | Encryption |  2.00 KB | 1024 | 114.048643 |
| 256 | Decryption |  2.00 KB | 1024 | 113.894836 |
| 256 | Encryption |  4.00 KB | 256 | 111.010231 |
| 256 | Decryption |  4.00 KB | 256 | 111.596987 |
| 256 | Encryption |  4.00 KB | 512 | 114.279337 |
| 256 | Decryption |  4.00 KB | 512 | 114.799815 |
| 256 | Encryption |  4.00 KB | 1024 | 116.510930 |
| 256 | Decryption |  4.00 KB | 1024 | 116.126106 |
| 256 | Encryption |  4.00 KB | 2048 | 117.227859 |
| 256 | Decryption |  4.00 KB | 2048 | 117.407630 |
| 256 | Encryption |  8.00 KB | 256 | 108.981458 |
| 256 | Decryption |  8.00 KB | 256 | 109.225938 |
| 256 | Encryption |  8.00 KB | 512 | 112.551823 |
| 256 | Decryption |  8.00 KB | 512 | 112.830083 |
| 256 | Encryption |  8.00 KB | 1024 | 114.336359 |
| 256 | Decryption |  8.00 KB | 1024 | 114.240292 |
| 256 | Encryption |  8.00 KB | 2048 | 115.181958 |
| 256 | Decryption |  8.00 KB | 2048 | 114.756797 |
| 256 | Encryption |  8.00 KB | 4096 | 115.388814 |
| 256 | Decryption |  8.00 KB | 4096 | 115.620071 |
| 256 | Encryption |  16.00 KB | 256 | 108.604184 |
| 256 | Decryption |  16.00 KB | 256 | 108.613543 |
| 256 | Encryption |  16.00 KB | 512 | 111.818050 |
| 256 | Decryption |  16.00 KB | 512 | 111.553486 |
| 256 | Encryption |  16.00 KB | 1024 | 113.397228 |
| 256 | Decryption |  16.00 KB | 1024 | 113.032665 |
| 256 | Encryption |  16.00 KB | 2048 | 114.178393 |
| 256 | Decryption |  16.00 KB | 2048 | 113.969508 |
| 256 | Encryption |  16.00 KB | 4096 | 114.516769 |
| 256 | Decryption |  16.00 KB | 4096 | 114.315417 |
| 256 | Encryption |  16.00 KB | 8192 | 114.654206 |
| 256 | Decryption |  16.00 KB | 8192 | 114.740121 |
| 256 | Encryption |  32.00 KB | 256 | 108.732387 |
| 256 | Decryption |  32.00 KB | 256 | 108.683160 |
| 256 | Encryption |  32.00 KB | 512 | 111.819862 |
| 256 | Decryption |  32.00 KB | 512 | 111.929102 |
| 256 | Encryption |  32.00 KB | 1024 | 113.482742 |
| 256 | Decryption |  32.00 KB | 1024 | 113.273064 |
| 256 | Encryption |  32.00 KB | 2048 | 114.321898 |
| 256 | Decryption |  32.00 KB | 2048 | 114.092622 |
| 256 | Encryption |  32.00 KB | 4096 | 114.709293 |
| 256 | Decryption |  32.00 KB | 4096 | 114.565715 |
| 256 | Encryption |  32.00 KB | 8192 | 114.919491 |
| 256 | Decryption |  32.00 KB | 8192 | 114.826064 |
| 256 | Encryption |  32.00 KB | 16384 | 115.074463 |
| 256 | Decryption |  32.00 KB | 16384 | 114.980381 |
| 128 | Encryption |  512.00 B | 256 | 98.381131 |
| 128 | Decryption |  512.00 B | 256 | 101.200771 |
| 128 | Encryption |  1024.00 B | 256 | 108.907206 |
| 128 | Decryption |  1024.00 B | 256 | 111.204626 |
| 128 | Encryption |  1024.00 B | 512 | 116.122813 |
| 128 | Decryption |  1024.00 B | 512 | 116.096483 |
| 128 | Encryption |  2.00 KB | 256 | 117.212763 |
| 128 | Decryption |  2.00 KB | 256 | 117.097157 |
| 128 | Encryption |  2.00 KB | 512 | 121.409728 |
| 128 | Decryption |  2.00 KB | 512 | 121.799637 |
| 128 | Encryption |  2.00 KB | 1024 | 123.690171 |
| 128 | Decryption |  2.00 KB | 1024 | 123.578217 |
| 128 | Encryption |  4.00 KB | 256 | 119.004903 |
| 128 | Decryption |  4.00 KB | 256 | 120.719128 |
| 128 | Encryption |  4.00 KB | 512 | 124.223791 |
| 128 | Decryption |  4.00 KB | 512 | 124.482400 |
| 128 | Encryption |  4.00 KB | 1024 | 126.623954 |
| 128 | Decryption |  4.00 KB | 1024 | 126.747379 |
| 128 | Encryption |  4.00 KB | 2048 | 127.283039 |
| 128 | Decryption |  4.00 KB | 2048 | 127.608203 |
| 128 | Encryption |  8.00 KB | 256 | 117.919283 |
| 128 | Decryption |  8.00 KB | 256 | 117.898918 |
| 128 | Encryption |  8.00 KB | 512 | 122.553548 |
| 128 | Decryption |  8.00 KB | 512 | 122.377792 |
| 128 | Encryption |  8.00 KB | 1024 | 124.205427 |
| 128 | Decryption |  8.00 KB | 1024 | 124.128265 |
| 128 | Encryption |  8.00 KB | 2048 | 124.739713 |
| 128 | Decryption |  8.00 KB | 2048 | 125.148472 |
| 128 | Encryption |  8.00 KB | 4096 | 125.437834 |
| 128 | Decryption |  8.00 KB | 4096 | 125.347625 |
| 128 | Encryption |  16.00 KB | 256 | 117.096111 |
| 128 | Decryption |  16.00 KB | 256 | 117.226391 |
| 128 | Encryption |  16.00 KB | 512 | 120.950165 |
| 128 | Decryption |  16.00 KB | 512 | 120.773187 |
| 128 | Encryption |  16.00 KB | 1024 | 122.849671 |
| 128 | Decryption |  16.00 KB | 1024 | 122.648271 |
| 128 | Encryption |  16.00 KB | 2048 | 123.872763 |
| 128 | Decryption |  16.00 KB | 2048 | 123.656330 |
| 128 | Encryption |  16.00 KB | 4096 | 124.450960 |
| 128 | Decryption |  16.00 KB | 4096 | 123.790401 |
| 128 | Encryption |  16.00 KB | 8192 | 124.633674 |
| 128 | Decryption |  16.00 KB | 8192 | 124.403240 |
| 128 | Encryption |  32.00 KB | 256 | 117.376404 |
| 128 | Decryption |  32.00 KB | 256 | 117.508784 |
| 128 | Encryption |  32.00 KB | 512 | 121.208422 |
| 128 | Decryption |  32.00 KB | 512 | 121.109866 |
| 128 | Encryption |  32.00 KB | 1024 | 123.105013 |
| 128 | Decryption |  32.00 KB | 1024 | 122.817785 |
| 128 | Encryption |  32.00 KB | 2048 | 124.077386 |
| 128 | Decryption |  32.00 KB | 2048 | 123.818350 |
| 128 | Encryption |  32.00 KB | 4096 | 124.464787 |
| 128 | Decryption |  32.00 KB | 4096 | 124.462187 |
| 128 | Encryption |  32.00 KB | 8192 | 124.728436 |
| 128 | Decryption |  32.00 KB | 8192 | 124.754317 |
| 128 | Encryption |  32.00 KB | 16384 | 124.920062 |
| 128 | Decryption |  32.00 KB | 16384 | 124.881737 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 115.226815 |
| 256 | Decryption |  32.00 KB | 115.072544 |
| 256 | Encryption |  16.00 KB | 115.309837 |
| 256 | Decryption |  16.00 KB | 114.849306 |
| 256 | Encryption |  8.00 KB | 115.269477 |
| 256 | Decryption |  8.00 KB | 115.348601 |
| 256 | Encryption |  4.00 KB | 116.940866 |
| 256 | Decryption |  4.00 KB | 117.249671 |
| 256 | Encryption |  2.00 KB | 116.241451 |
| 256 | Decryption |  2.00 KB | 116.514244 |
| 256 | Encryption |  1024.00 B | 111.885056 |
| 256 | Decryption |  1024.00 B | 111.998250 |
| 256 | Encryption |  512.00 B | 104.730248 |
| 256 | Decryption |  512.00 B | 105.025641 |
| 128 | Encryption |  32.00 KB | 125.042811 |
| 128 | Decryption |  32.00 KB | 125.053190 |
| 128 | Encryption |  16.00 KB | 125.105231 |
| 128 | Decryption |  16.00 KB | 124.982121 |
| 128 | Encryption |  8.00 KB | 125.929541 |
| 128 | Decryption |  8.00 KB | 126.219127 |
| 128 | Encryption |  4.00 KB | 127.061926 |
| 128 | Decryption |  4.00 KB | 128.186271 |
| 128 | Encryption |  2.00 KB | 126.040465 |
| 128 | Decryption |  2.00 KB | 125.601791 |
| 128 | Encryption |  1024.00 B | 121.194189 |
| 128 | Decryption |  1024.00 B | 121.409728 |
| 128 | Encryption |  512.00 B | 112.552209 |
| 128 | Decryption |  512.00 B | 112.676056 |

### AES CBC STREAM

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 88.116341 |
| 256 | Decryption |  512.00 B | 256 | 93.107838 |
| 256 | Encryption |  1024.00 B | 256 | 100.926473 |
| 256 | Decryption |  1024.00 B | 256 | 103.101087 |
| 256 | Encryption |  1024.00 B | 512 | 106.597267 |
| 256 | Decryption |  1024.00 B | 512 | 106.505799 |
| 256 | Encryption |  2.00 KB | 256 | 106.850316 |
| 256 | Decryption |  2.00 KB | 256 | 107.594105 |
| 256 | Encryption |  2.00 KB | 512 | 111.381528 |
| 256 | Decryption |  2.00 KB | 512 | 111.258998 |
| 256 | Encryption |  2.00 KB | 1024 | 113.391930 |
| 256 | Decryption |  2.00 KB | 1024 | 113.490898 |
| 256 | Encryption |  4.00 KB | 256 | 109.881561 |
| 256 | Decryption |  4.00 KB | 256 | 111.191796 |
| 256 | Encryption |  4.00 KB | 512 | 114.049437 |
| 256 | Decryption |  4.00 KB | 512 | 114.485361 |
| 256 | Encryption |  4.00 KB | 1024 | 116.056186 |
| 256 | Decryption |  4.00 KB | 1024 | 115.683339 |
| 256 | Encryption |  4.00 KB | 2048 | 117.024392 |
| 256 | Decryption |  4.00 KB | 2048 | 117.130642 |
| 256 | Encryption |  8.00 KB | 256 | 108.760808 |
| 256 | Decryption |  8.00 KB | 256 | 109.324694 |
| 256 | Encryption |  8.00 KB | 512 | 112.452170 |
| 256 | Decryption |  8.00 KB | 512 | 112.905504 |
| 256 | Encryption |  8.00 KB | 1024 | 114.350723 |
| 256 | Decryption |  8.00 KB | 1024 | 114.081599 |
| 256 | Encryption |  8.00 KB | 2048 | 115.084062 |
| 256 | Decryption |  8.00 KB | 2048 | 114.834816 |
| 256 | Encryption |  8.00 KB | 4096 | 115.367689 |
| 256 | Decryption |  8.00 KB | 4096 | 115.395316 |
| 256 | Encryption |  16.00 KB | 256 | 108.581512 |
| 256 | Decryption |  16.00 KB | 256 | 108.668293 |
| 256 | Encryption |  16.00 KB | 512 | 111.727310 |
| 256 | Decryption |  16.00 KB | 512 | 111.479102 |
| 256 | Encryption |  16.00 KB | 1024 | 113.382318 |
| 256 | Decryption |  16.00 KB | 1024 | 113.083765 |
| 256 | Encryption |  16.00 KB | 2048 | 113.958806 |
| 256 | Decryption |  16.00 KB | 2048 | 113.868912 |
| 256 | Encryption |  16.00 KB | 4096 | 114.505564 |
| 256 | Decryption |  16.00 KB | 4096 | 114.218192 |
| 256 | Encryption |  16.00 KB | 8192 | 114.607288 |
| 256 | Decryption |  16.00 KB | 8192 | 114.512767 |
| 256 | Encryption |  32.00 KB | 256 | 108.504838 |
| 256 | Decryption |  32.00 KB | 256 | 108.737980 |
| 256 | Encryption |  32.00 KB | 512 | 111.862234 |
| 256 | Decryption |  32.00 KB | 512 | 111.751220 |
| 256 | Encryption |  32.00 KB | 1024 | 113.444829 |
| 256 | Decryption |  32.00 KB | 1024 | 113.323990 |
| 256 | Encryption |  32.00 KB | 2048 | 114.264294 |
| 256 | Decryption |  32.00 KB | 2048 | 114.102951 |
| 256 | Encryption |  32.00 KB | 4096 | 114.629238 |
| 256 | Decryption |  32.00 KB | 4096 | 114.580938 |
| 256 | Encryption |  32.00 KB | 8192 | 114.782423 |
| 256 | Decryption |  32.00 KB | 8192 | 114.800519 |
| 256 | Encryption |  32.00 KB | 16384 | 114.977153 |
| 256 | Decryption |  32.00 KB | 16384 | 114.983810 |
| 128 | Encryption |  512.00 B | 256 | 96.937568 |
| 128 | Decryption |  512.00 B | 256 | 99.543113 |
| 128 | Encryption |  1024.00 B | 256 | 109.777015 |
| 128 | Decryption |  1024.00 B | 256 | 111.401218 |
| 128 | Encryption |  1024.00 B | 512 | 115.082041 |
| 128 | Decryption |  1024.00 B | 512 | 115.137034 |
| 128 | Encryption |  2.00 KB | 256 | 115.805991 |
| 128 | Decryption |  2.00 KB | 256 | 116.469518 |
| 128 | Encryption |  2.00 KB | 512 | 120.646235 |
| 128 | Decryption |  2.00 KB | 512 | 121.142215 |
| 128 | Encryption |  2.00 KB | 1024 | 123.336344 |
| 128 | Decryption |  2.00 KB | 1024 | 123.241714 |
| 128 | Encryption |  4.00 KB | 256 | 118.940109 |
| 128 | Decryption |  4.00 KB | 256 | 120.448447 |
| 128 | Encryption |  4.00 KB | 512 | 123.742485 |
| 128 | Decryption |  4.00 KB | 512 | 124.321822 |
| 128 | Encryption |  4.00 KB | 1024 | 126.394396 |
| 128 | Decryption |  4.00 KB | 1024 | 126.379772 |
| 128 | Encryption |  4.00 KB | 2048 | 127.093466 |
| 128 | Decryption |  4.00 KB | 2048 | 127.387941 |
| 128 | Encryption |  8.00 KB | 256 | 118.056932 |
| 128 | Decryption |  8.00 KB | 256 | 117.493510 |
| 128 | Encryption |  8.00 KB | 512 | 121.846287 |
| 128 | Decryption |  8.00 KB | 512 | 121.734931 |
| 128 | Encryption |  8.00 KB | 1024 | 123.401836 |
| 128 | Decryption |  8.00 KB | 1024 | 123.992992 |
| 128 | Encryption |  8.00 KB | 2048 | 124.717874 |
| 128 | Decryption |  8.00 KB | 2048 | 124.699838 |
| 128 | Encryption |  8.00 KB | 4096 | 125.456084 |
| 128 | Decryption |  8.00 KB | 4096 | 125.247969 |
| 128 | Encryption |  16.00 KB | 256 | 117.259531 |
| 128 | Decryption |  16.00 KB | 256 | 117.275478 |
| 128 | Encryption |  16.00 KB | 512 | 120.995272 |
| 128 | Decryption |  16.00 KB | 512 | 120.866292 |
| 128 | Encryption |  16.00 KB | 1024 | 122.790286 |
| 128 | Decryption |  16.00 KB | 1024 | 122.551485 |
| 128 | Encryption |  16.00 KB | 2048 | 123.777308 |
| 128 | Decryption |  16.00 KB | 2048 | 123.534191 |
| 128 | Encryption |  16.00 KB | 4096 | 124.285750 |
| 128 | Decryption |  16.00 KB | 4096 | 123.792739 |
| 128 | Encryption |  16.00 KB | 8192 | 124.541776 |
| 128 | Decryption |  16.00 KB | 8192 | 124.236743 |
| 128 | Encryption |  32.00 KB | 256 | 117.517844 |
| 128 | Decryption |  32.00 KB | 256 | 117.561693 |
| 128 | Encryption |  32.00 KB | 512 | 121.211112 |
| 128 | Decryption |  32.00 KB | 512 | 121.060200 |
| 128 | Encryption |  32.00 KB | 1024 | 123.040298 |
| 128 | Decryption |  32.00 KB | 1024 | 122.922831 |
| 128 | Encryption |  32.00 KB | 2048 | 123.948318 |
| 128 | Decryption |  32.00 KB | 2048 | 123.890209 |
| 128 | Encryption |  32.00 KB | 4096 | 124.494105 |
| 128 | Decryption |  32.00 KB | 4096 | 124.461241 |
| 128 | Encryption |  32.00 KB | 8192 | 124.859848 |
| 128 | Decryption |  32.00 KB | 8192 | 124.605119 |
| 128 | Encryption |  32.00 KB | 16384 | 124.854853 |
| 128 | Decryption |  32.00 KB | 16384 | 124.880071 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 372.695038 |
| 512 |  16.00 KB | 369.150350 |
| 512 |  8.00 KB | 367.041534 |
| 512 |  4.00 KB | 358.410080 |
| 512 |  2.00 KB | 343.163539 |
| 512 |  1024.00 B | 315.805705 |
| 512 |  512.00 B | 271.690103 |
| 256 |  32.00 KB | 360.351246 |
| 256 |  16.00 KB | 358.999080 |
| 256 |  8.00 KB | 356.073284 |
| 256 |  4.00 KB | 349.980775 |
| 256 |  2.00 KB | 332.791680 |
| 256 |  1024.00 B | 316.880706 |
| 256 |  512.00 B | 281.628163 |


### AES CTR STREAM

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 87.137812 |
| 256 | Decryption |  512.00 B | 256 | 93.901880 |
| 256 | Encryption |  1024.00 B | 256 | 101.870275 |
| 256 | Decryption |  1024.00 B | 256 | 103.244020 |
| 256 | Encryption |  1024.00 B | 512 | 106.326091 |
| 256 | Decryption |  1024.00 B | 512 | 105.141566 |
| 256 | Encryption |  2.00 KB | 256 | 107.153602 |
| 256 | Decryption |  2.00 KB | 256 | 108.160921 |
| 256 | Encryption |  2.00 KB | 512 | 111.697413 |
| 256 | Decryption |  2.00 KB | 512 | 111.197079 |
| 256 | Encryption |  2.00 KB | 1024 | 113.464175 |
| 256 | Decryption |  2.00 KB | 1024 | 113.420189 |
| 256 | Encryption |  4.00 KB | 256 | 109.512860 |
| 256 | Decryption |  4.00 KB | 256 | 110.782047 |
| 256 | Encryption |  4.00 KB | 512 | 114.417403 |
| 256 | Decryption |  4.00 KB | 512 | 114.116959 |
| 256 | Encryption |  4.00 KB | 1024 | 116.122813 |
| 256 | Decryption |  4.00 KB | 1024 | 116.212593 |
| 256 | Encryption |  4.00 KB | 2048 | 116.542423 |
| 256 | Decryption |  4.00 KB | 2048 | 116.982614 |
| 256 | Encryption |  8.00 KB | 256 | 109.061620 |
| 256 | Decryption |  8.00 KB | 256 | 109.088488 |
| 256 | Encryption |  8.00 KB | 512 | 112.202271 |
| 256 | Decryption |  8.00 KB | 512 | 112.421691 |
| 256 | Encryption |  8.00 KB | 1024 | 113.861593 |
| 256 | Decryption |  8.00 KB | 1024 | 114.276946 |
| 256 | Encryption |  8.00 KB | 2048 | 114.858565 |
| 256 | Decryption |  8.00 KB | 2048 | 114.692932 |
| 256 | Encryption |  8.00 KB | 4096 | 115.320185 |
| 256 | Decryption |  8.00 KB | 4096 | 115.423362 |
| 256 | Encryption |  16.00 KB | 256 | 108.451958 |
| 256 | Decryption |  16.00 KB | 256 | 108.325041 |
| 256 | Encryption |  16.00 KB | 512 | 111.675335 |
| 256 | Decryption |  16.00 KB | 512 | 111.508123 |
| 256 | Encryption |  16.00 KB | 1024 | 113.277861 |
| 256 | Decryption |  16.00 KB | 1024 | 112.958826 |
| 256 | Encryption |  16.00 KB | 2048 | 114.129678 |
| 256 | Decryption |  16.00 KB | 2048 | 113.724469 |
| 256 | Encryption |  16.00 KB | 4096 | 114.533780 |
| 256 | Decryption |  16.00 KB | 4096 | 114.168249 |
| 256 | Encryption |  16.00 KB | 8192 | 114.615105 |
| 256 | Decryption |  16.00 KB | 8192 | 114.466564 |
| 256 | Encryption |  32.00 KB | 256 | 108.413386 |
| 256 | Decryption |  32.00 KB | 256 | 108.709030 |
| 256 | Encryption |  32.00 KB | 512 | 111.846675 |
| 256 | Decryption |  32.00 KB | 512 | 111.678760 |
| 256 | Encryption |  32.00 KB | 1024 | 113.385064 |
| 256 | Decryption |  32.00 KB | 1024 | 113.333887 |
| 256 | Encryption |  32.00 KB | 2048 | 114.150551 |
| 256 | Decryption |  32.00 KB | 2048 | 114.171630 |
| 256 | Encryption |  32.00 KB | 4096 | 114.656011 |
| 256 | Decryption |  32.00 KB | 4096 | 114.591457 |
| 256 | Encryption |  32.00 KB | 8192 | 114.964951 |
| 256 | Decryption |  32.00 KB | 8192 | 114.718831 |
| 256 | Encryption |  32.00 KB | 16384 | 115.010749 |
| 256 | Decryption |  32.00 KB | 16384 | 114.969791 |
| 128 | Encryption |  512.00 B | 256 | 98.423683 |
| 128 | Decryption |  512.00 B | 256 | 100.500540 |
| 128 | Encryption |  1024.00 B | 256 | 110.630942 |
| 128 | Decryption |  1024.00 B | 256 | 111.507364 |
| 128 | Encryption |  1024.00 B | 512 | 114.547794 |
| 128 | Decryption |  1024.00 B | 512 | 115.318562 |
| 128 | Encryption |  2.00 KB | 256 | 114.862591 |
| 128 | Decryption |  2.00 KB | 256 | 116.891641 |
| 128 | Encryption |  2.00 KB | 512 | 121.181639 |
| 128 | Decryption |  2.00 KB | 512 | 121.267745 |
| 128 | Encryption |  2.00 KB | 1024 | 123.038104 |
| 128 | Decryption |  2.00 KB | 1024 | 123.416244 |
| 128 | Encryption |  4.00 KB | 256 | 120.429855 |
| 128 | Decryption |  4.00 KB | 256 | 120.490079 |
| 128 | Encryption |  4.00 KB | 512 | 124.198366 |
| 128 | Decryption |  4.00 KB | 512 | 124.434183 |
| 128 | Encryption |  4.00 KB | 1024 | 126.313517 |
| 128 | Decryption |  4.00 KB | 1024 | 125.777280 |
| 128 | Encryption |  4.00 KB | 2048 | 127.406762 |
| 128 | Decryption |  4.00 KB | 2048 | 127.457311 |
| 128 | Encryption |  8.00 KB | 256 | 118.165492 |
| 128 | Decryption |  8.00 KB | 256 | 118.139079 |
| 128 | Encryption |  8.00 KB | 512 | 122.020972 |
| 128 | Decryption |  8.00 KB | 512 | 122.756016 |
| 128 | Encryption |  8.00 KB | 1024 | 123.469257 |
| 128 | Decryption |  8.00 KB | 1024 | 124.143784 |
| 128 | Encryption |  8.00 KB | 2048 | 124.753485 |
| 128 | Decryption |  8.00 KB | 2048 | 124.314276 |
| 128 | Encryption |  8.00 KB | 4096 | 125.259939 |
| 128 | Decryption |  8.00 KB | 4096 | 125.117412 |
| 128 | Encryption |  16.00 KB | 256 | 117.046128 |
| 128 | Decryption |  16.00 KB | 256 | 117.046128 |
| 128 | Encryption |  16.00 KB | 512 | 120.854033 |
| 128 | Decryption |  16.00 KB | 512 | 120.679115 |
| 128 | Encryption |  16.00 KB | 1024 | 122.789596 |
| 128 | Decryption |  16.00 KB | 1024 | 122.693965 |
| 128 | Encryption |  16.00 KB | 2048 | 123.894541 |
| 128 | Decryption |  16.00 KB | 2048 | 123.214836 |
| 128 | Encryption |  16.00 KB | 4096 | 124.152250 |
| 128 | Decryption |  16.00 KB | 4096 | 123.931559 |
| 128 | Encryption |  16.00 KB | 8192 | 124.409144 |
| 128 | Decryption |  16.00 KB | 8192 | 124.448833 |
| 128 | Encryption |  32.00 KB | 256 | 117.469080 |
| 128 | Decryption |  32.00 KB | 256 | 117.446555 |
| 128 | Encryption |  32.00 KB | 512 | 121.191499 |
| 128 | Decryption |  32.00 KB | 512 | 120.900184 |
| 128 | Encryption |  32.00 KB | 1024 | 123.090331 |
| 128 | Decryption |  32.00 KB | 1024 | 122.893672 |
| 128 | Encryption |  32.00 KB | 2048 | 123.984547 |
| 128 | Decryption |  32.00 KB | 2048 | 123.852162 |
| 128 | Encryption |  32.00 KB | 4096 | 124.483109 |
| 128 | Decryption |  32.00 KB | 4096 | 124.340692 |
| 128 | Encryption |  32.00 KB | 8192 | 124.703635 |
| 128 | Decryption |  32.00 KB | 8192 | 124.665206 |
| 128 | Encryption |  32.00 KB | 16384 | 124.887092 |
| 128 | Decryption |  32.00 KB | 16384 | 124.725232 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 115.315823 |
| 256 | Decryption |  32.00 KB | 115.102152 |
| 256 | Encryption |  16.00 KB | 114.951037 |
| 256 | Decryption |  16.00 KB | 114.871248 |
| 256 | Encryption |  8.00 KB | 115.214357 |
| 256 | Decryption |  8.00 KB | 115.625375 |
| 256 | Encryption |  4.00 KB | 116.954222 |
| 256 | Decryption |  4.00 KB | 117.784072 |
| 256 | Encryption |  2.00 KB | 115.183982 |
| 256 | Decryption |  2.00 KB | 116.383474 |
| 256 | Encryption |  1024.00 B | 111.483084 |
| 256 | Decryption |  1024.00 B | 112.536748 |
| 256 | Encryption |  512.00 B | 104.123240 |
| 256 | Decryption |  512.00 B | 104.585844 |
| 128 | Encryption |  32.00 KB | 124.957933 |
| 128 | Decryption |  32.00 KB | 124.951977 |
| 128 | Encryption |  16.00 KB | 124.765123 |
| 128 | Decryption |  16.00 KB | 124.586761 |
| 128 | Encryption |  8.00 KB | 124.258650 |
| 128 | Decryption |  8.00 KB | 124.719298 |
| 128 | Encryption |  4.00 KB | 127.275128 |
| 128 | Decryption |  4.00 KB | 128.376102 |
| 128 | Encryption |  2.00 KB | 126.083142 |
| 128 | Decryption |  2.00 KB | 126.470497 |
| 128 | Encryption |  1024.00 B | 120.922268 |
| 128 | Decryption |  1024.00 B | 121.730861 |
| 128 | Encryption |  512.00 B | 112.225327 |
| 128 | Decryption |  512.00 B | 112.225327 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 369.935932 |
| 512 |  16.00 KB | 365.359528 |
| 512 |  8.00 KB | 356.328839 |
| 512 |  4.00 KB | 339.346741 |
| 512 |  2.00 KB | 309.470742 |
| 512 |  1024.00 B | 263.425301 |
| 512 |  512.00 B | 202.571711 |
| 256 |  32.00 KB | 358.300348 |
| 256 |  16.00 KB | 355.210597 |
| 256 |  8.00 KB | 348.736723 |
| 256 |  4.00 KB | 336.116525 |
| 256 |  2.00 KB | 312.922572 |
| 256 |  1024.00 B | 275.713516 |
| 256 |  512.00 B | 222.657099 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 302.914470 |
| 256 |  16.00 KB | 305.250214 |
| 256 |  8.00 KB | 301.772805 |
| 256 |  4.00 KB | 283.558325 |
| 256 |  2.00 KB | 250.711553 |
| 256 |  1024.00 B | 203.831799 |
| 256 |  512.00 B | 147.806005 |
| 128 |  32.00 KB | 301.945905 |
| 128 |  16.00 KB | 303.288058 |
| 128 |  8.00 KB | 303.062253 |
| 128 |  4.00 KB | 284.498776 |
| 128 |  2.00 KB | 252.613402 |
| 128 |  1024.00 B | 205.777443 |
| 128 |  512.00 B | 150.223722 |

### AES CMAC STREAM

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      251.442120      |
| 256        |      32.00 KB     |        1024        |      272.977949      |
| 256        |      32.00 KB     |        2048        |      288.323801      |
| 256        |      32.00 KB     |        4096        |      295.693370      |
| 256        |      32.00 KB     |        8192        |      298.727346      |
| 256        |      32.00 KB     |        16384        |      298.186385      |
| 256        |      16.00 KB     |        512        |      250.631972      |
| 256        |      16.00 KB     |        1024        |      279.590444      |
| 256        |      16.00 KB     |        2048        |      295.565347      |
| 256        |      16.00 KB     |        4096        |      301.764468      |
| 256        |      16.00 KB     |        8192        |      306.321718      |
| 256        |      8.00 KB     |        512        |      250.120221      |
| 256        |      8.00 KB     |        1024        |      274.899329      |
| 256        |      8.00 KB     |        2048        |      288.855000      |
| 256        |      8.00 KB     |        4096        |      295.183273      |
| 256        |      4.00 KB     |        512        |      236.742479      |
| 256        |      4.00 KB     |        1024        |      258.614430      |
| 256        |      4.00 KB     |        2048        |      269.194749      |
| 256        |      2.00 KB     |        512        |      212.630104      |
| 256        |      2.00 KB     |        1024        |      230.650111      |
| 256        |      1024.00 B     |        512        |      177.070725      |
| 128        |      32.00 KB     |        512        |      253.280206      |
| 128        |      32.00 KB     |        1024        |      275.953834      |
| 128        |      32.00 KB     |        2048        |      288.131122      |
| 128        |      32.00 KB     |        4096        |      295.006550      |
| 128        |      32.00 KB     |        8192        |      301.632523      |
| 128        |      32.00 KB     |        16384        |      302.377085      |
| 128        |      16.00 KB     |        512        |      249.809411      |
| 128        |      16.00 KB     |        1024        |      277.192211      |
| 128        |      16.00 KB     |        2048        |      292.027788      |
| 128        |      16.00 KB     |        4096        |      300.881949      |
| 128        |      16.00 KB     |        8192        |      307.520494      |
| 128        |      8.00 KB     |        512        |      250.211895      |
| 128        |      8.00 KB     |        1024        |      273.735036      |
| 128        |      8.00 KB     |        2048        |      288.661610      |
| 128        |      8.00 KB     |        4096        |      295.828180      |
| 128        |      4.00 KB     |        512        |      236.728796      |
| 128        |      4.00 KB     |        1024        |      259.540292      |
| 128        |      4.00 KB     |        2048        |      271.748686      |
| 128        |      2.00 KB     |        512        |      213.695057      |
| 128        |      2.00 KB     |        1024        |      232.140327      |
| 128        |      1024.00 B     |        512        |      178.607247      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    38   
    1024      |      TCMA     |     TCMA           |    36   
    1024      |      TCMB     |     TCMB           |    37   
    1024      |      OCRAM    |     TCMA           |    36   
    1024      |      TCMA     |     OCRAM          |    36     

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 

Function        | Err           | Max Cycles Mathlib (mcusdk)   | avg cycles Mathlib (mcusdk)   | max cycles mathlib (clang)    | avg cycles mathlib (clang)    |
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin             |0.0000007150   | 52                    | 52.043999             | 504                   | 276.377991            |
cos             |0.0000002870   | 65                    | 65.106003             | 504                   | 277.562012            |
sincos sin      |0.0000001790   | -271                  | 78.262001             | 467                   | 275.291992            |
sincos cos      |0.0000001900   |                       |                       |                       |                       |
asin            |0.0000003430   | 418                   | 74.723999             | 602                   | 428.768005            |
acos            |0.0000004770   | 76                    | 76.033997             | 533                   | 383.220001            |
atan            |0.0000005360   | 80                    | 80.054001             | 765                   | 371.664001            |
atan2           |0.0000007150   | 117                   | 104.758003            | 845                   | 479.917999            |

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### XIP Benchmark

Comparing data from \ref EXAMPLES_OPTIFLASH_XIP_BENCHMARK and \ref EXAMPLES_OPTIFLASH_OCRAM_BENCHMARK, execution time of a code which throws ~3 Million I-Cache Miss per seconds is 2.2 times slower when it runs from OCRAM. 

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
| 1        | 460      | 17.391304        |
| 2        | 917      | 17.448201        |
| 4        | 1830     | 17.486339        |
| 8        | 3657     | 17.500684        |
| 16       | 7306     | 17.519847        |
| 32       | 14620    | 17.510260        |
| 64       | 29241    | 17.509661        |
| 128      | 58481    | 17.509960        |
| 256      | 116959   | 17.510410        |
| 512      | 233919   | 17.510335        |

#### DMA read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
| 1        | 461      | 17.353579        |
| 2        | 24       | 666.666667       |
| 4        | 37       | 864.864865       |
| 8        | 62       | 1032.258065      |
| 16       | 113      | 1132.743363      |
| 32       | 219      | 1168.949772      |
| 64       | 426      | 1201.877934      |
| 128      | 841      | 1217.598098      |
| 256      | 1667     | 1228.554289      |
| 512      | 3327     | 1231.139164      |
