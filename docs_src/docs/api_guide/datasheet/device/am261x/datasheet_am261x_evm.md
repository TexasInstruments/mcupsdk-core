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

### SBL OSPI MULTICORE ELF performance

- Software/Application used           : sbl_ospi_multicore_elf and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 349 KB
- Size of ipc_notify_echo             : 58 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   429
SBL : Drivers_open                      |   90
SBL : LoadHsmRtFw                       |   6294
SBL : Board_driversOpen                 |   6102
SBL : CPU Load                          |   4387
SBL : SBL End                           |   8
SBL : Total time taken                  |   17312

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 356 KB
- Size of hello_world                 : 5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   432
SBL : Drivers_open                      |   96
SBL : LoadHsmRtFw                       |   6294
SBL : Board_driversOpen                 |   6133
SBL : CPU Load                          |   3988
SBL : SBL End                           |   4
SBL : Total time taken                  |   16951

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
 r5f0-0	| r5f0-1	|  1.46

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 9.719
 r5f0-0	| r5f0-1	| 32	| 11.744
 r5f0-0	| r5f0-1	| 64	| 13.926
 r5f0-0	| r5f0-1	| 112	| 17.270

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
 400		| 08			| 10.76 Mbps / 297.50 us 	| 24.18 Mbps / 132.33 us 	|  0.92 Mbps / 3463.92 us
 200		| 16			| 21.68 Mbps / 147.62 us 	| 30.49 Mbps / 104.97 us 	|  0.96 Mbps / 3339.69 us
 100		| 32			| 37.79 Mbps / 84.68 us 	| 35.45 Mbps / 90.26 us 	|  0.98 Mbps / 3277.71 us
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
- Begin tick       : 56
- End tick         : 8186198
- Total ticks      : 8186142
- Total time (secs): 8.186142
- Iterations/Sec   : 1832.364990
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1832.364990 
CoreMark/MHz :4.580912 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146103481
- USER cycle count:                          146103474

BENCHMARK Using clock 500000000
- Usertime in sec:                           0.292207
- Microseconds for one run through Dhrystone:   0.6 
- Dhrystones per Second:                     1711116.1 

Normalized MIPS/MHz:                         1.9478

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 247.078162 |
| 256 |  16.00 KB | 244.355870 |
| 256 |  8.00 KB | 241.974908 |
| 256 |  4.00 KB | 227.552395 |
| 256 |  2.00 KB | 200.946844 |
| 256 |  1024.00 B | 163.951487 |
| 256 |  512.00 B | 118.545960 |
| 128 |  32.00 KB | 247.620091 |
| 128 |  16.00 KB | 244.399611 |
| 128 |  8.00 KB | 243.722480 |
| 128 |  4.00 KB | 228.227559 |
| 128 |  2.00 KB | 202.837547 |
| 128 |  1024.00 B | 161.986870 |
| 128 |  512.00 B | 120.961550 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 114.944787 |
| 256 | Decryption |  32.00 KB | 115.028210 |
| 256 | Encryption |  16.00 KB | 114.763428 |
| 256 | Decryption |  16.00 KB | 114.778503 |
| 256 | Encryption |  8.00 KB | 114.825159 |
| 256 | Decryption |  8.00 KB | 114.862994 |
| 256 | Encryption |  4.00 KB | 116.659428 |
| 256 | Decryption |  4.00 KB | 117.454764 |
| 256 | Encryption |  2.00 KB | 115.019025 |
| 256 | Decryption |  2.00 KB | 116.126106 |
| 256 | Encryption |  1024.00 B | 110.711679 |
| 256 | Decryption |  1024.00 B | 111.262020 |
| 256 | Encryption |  512.00 B | 102.806084 |
| 256 | Decryption |  512.00 B | 102.759659 |
| 128 | Encryption |  32.00 KB | 124.898635 |
| 128 | Decryption |  32.00 KB | 124.934827 |
| 128 | Encryption |  16.00 KB | 124.816925 |
| 128 | Decryption |  16.00 KB | 124.714788 |
| 128 | Encryption |  8.00 KB | 124.147076 |
| 128 | Decryption |  8.00 KB | 125.478663 |
| 128 | Encryption |  4.00 KB | 126.892663 |
| 128 | Decryption |  4.00 KB | 128.310753 |
| 128 | Encryption |  2.00 KB | 125.438314 |
| 128 | Decryption |  2.00 KB | 125.858440 |
| 128 | Encryption |  1024.00 B | 120.219542 |
| 128 | Decryption |  1024.00 B | 120.318421 |
| 128 | Encryption |  512.00 B | 110.006983 |
| 128 | Decryption |  512.00 B | 110.517511 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 84.768212 |
| 256 | Decryption |  512.00 B | 256 | 91.900381 |
| 256 | Encryption |  1024.00 B | 256 | 101.499195 |
| 256 | Decryption |  1024.00 B | 256 | 102.661788 |
| 256 | Encryption |  1024.00 B | 512 | 105.910948 |
| 256 | Decryption |  1024.00 B | 512 | 105.938341 |
| 256 | Encryption |  2.00 KB | 256 | 105.941081 |
| 256 | Decryption |  2.00 KB | 256 | 107.937177 |
| 256 | Encryption |  2.00 KB | 512 | 111.461848 |
| 256 | Decryption |  2.00 KB | 512 | 111.410309 |
| 256 | Encryption |  2.00 KB | 1024 | 113.127296 |
| 256 | Decryption |  2.00 KB | 1024 | 113.083570 |
| 256 | Encryption |  4.00 KB | 256 | 109.145899 |
| 256 | Decryption |  4.00 KB | 256 | 110.935818 |
| 256 | Encryption |  4.00 KB | 512 | 114.270570 |
| 256 | Decryption |  4.00 KB | 512 | 114.018483 |
| 256 | Encryption |  4.00 KB | 1024 | 116.074275 |
| 256 | Decryption |  4.00 KB | 1024 | 116.024134 |
| 256 | Encryption |  4.00 KB | 2048 | 116.564810 |
| 256 | Decryption |  4.00 KB | 2048 | 116.839125 |
| 256 | Encryption |  8.00 KB | 256 | 107.941799 |
| 256 | Decryption |  8.00 KB | 256 | 108.920962 |
| 256 | Encryption |  8.00 KB | 512 | 112.485368 |
| 256 | Decryption |  8.00 KB | 512 | 112.221868 |
| 256 | Encryption |  8.00 KB | 1024 | 113.920574 |
| 256 | Decryption |  8.00 KB | 1024 | 113.777383 |
| 256 | Encryption |  8.00 KB | 2048 | 114.725458 |
| 256 | Decryption |  8.00 KB | 2048 | 114.918584 |
| 256 | Encryption |  8.00 KB | 4096 | 115.482345 |
| 256 | Decryption |  8.00 KB | 4096 | 114.957287 |
| 256 | Encryption |  16.00 KB | 256 | 108.302664 |
| 256 | Decryption |  16.00 KB | 256 | 108.401371 |
| 256 | Encryption |  16.00 KB | 512 | 111.626069 |
| 256 | Decryption |  16.00 KB | 512 | 111.350681 |
| 256 | Encryption |  16.00 KB | 1024 | 113.176332 |
| 256 | Decryption |  16.00 KB | 1024 | 113.015512 |
| 256 | Encryption |  16.00 KB | 2048 | 113.941767 |
| 256 | Decryption |  16.00 KB | 2048 | 113.715984 |
| 256 | Encryption |  16.00 KB | 4096 | 114.437582 |
| 256 | Decryption |  16.00 KB | 4096 | 114.211424 |
| 256 | Encryption |  16.00 KB | 8192 | 114.574829 |
| 256 | Decryption |  16.00 KB | 8192 | 114.463965 |
| 256 | Encryption |  32.00 KB | 256 | 108.439307 |
| 256 | Decryption |  32.00 KB | 256 | 108.711194 |
| 256 | Encryption |  32.00 KB | 512 | 111.841999 |
| 256 | Decryption |  32.00 KB | 512 | 111.665820 |
| 256 | Encryption |  32.00 KB | 1024 | 113.435011 |
| 256 | Decryption |  32.00 KB | 1024 | 113.308315 |
| 256 | Encryption |  32.00 KB | 2048 | 114.239197 |
| 256 | Decryption |  32.00 KB | 2048 | 114.135342 |
| 256 | Encryption |  32.00 KB | 4096 | 114.663534 |
| 256 | Decryption |  32.00 KB | 4096 | 114.470463 |
| 256 | Encryption |  32.00 KB | 8192 | 114.827070 |
| 256 | Decryption |  32.00 KB | 8192 | 114.759108 |
| 256 | Encryption |  32.00 KB | 16384 | 115.001163 |
| 256 | Decryption |  32.00 KB | 16384 | 114.803234 |
| 128 | Encryption |  512.00 B | 256 | 94.915883 |
| 128 | Decryption |  512.00 B | 256 | 98.178332 |
| 128 | Encryption |  1024.00 B | 256 | 107.464253 |
| 128 | Decryption |  1024.00 B | 256 | 109.319953 |
| 128 | Encryption |  1024.00 B | 512 | 114.403821 |
| 128 | Decryption |  1024.00 B | 512 | 114.365489 |
| 128 | Encryption |  2.00 KB | 256 | 116.633683 |
| 128 | Decryption |  2.00 KB | 256 | 117.008513 |
| 128 | Encryption |  2.00 KB | 512 | 120.820613 |
| 128 | Decryption |  2.00 KB | 512 | 120.770739 |
| 128 | Encryption |  2.00 KB | 1024 | 122.456911 |
| 128 | Decryption |  2.00 KB | 1024 | 122.339869 |
| 128 | Encryption |  4.00 KB | 256 | 118.462818 |
| 128 | Decryption |  4.00 KB | 256 | 119.865093 |
| 128 | Encryption |  4.00 KB | 512 | 123.562373 |
| 128 | Decryption |  4.00 KB | 512 | 124.018803 |
| 128 | Encryption |  4.00 KB | 1024 | 126.119053 |
| 128 | Decryption |  4.00 KB | 1024 | 125.699117 |
| 128 | Encryption |  4.00 KB | 2048 | 127.152647 |
| 128 | Decryption |  4.00 KB | 2048 | 127.196081 |
| 128 | Encryption |  8.00 KB | 256 | 117.774335 |
| 128 | Decryption |  8.00 KB | 256 | 117.723984 |
| 128 | Encryption |  8.00 KB | 512 | 121.634614 |
| 128 | Decryption |  8.00 KB | 512 | 121.699665 |
| 128 | Encryption |  8.00 KB | 1024 | 123.511607 |
| 128 | Decryption |  8.00 KB | 1024 | 123.457627 |
| 128 | Encryption |  8.00 KB | 2048 | 124.722621 |
| 128 | Decryption |  8.00 KB | 2048 | 124.648608 |
| 128 | Encryption |  8.00 KB | 4096 | 125.177157 |
| 128 | Decryption |  8.00 KB | 4096 | 124.877573 |
| 128 | Encryption |  16.00 KB | 256 | 116.938570 |
| 128 | Decryption |  16.00 KB | 256 | 117.248832 |
| 128 | Encryption |  16.00 KB | 512 | 120.819053 |
| 128 | Decryption |  16.00 KB | 512 | 120.616259 |
| 128 | Encryption |  16.00 KB | 1024 | 122.743371 |
| 128 | Decryption |  16.00 KB | 1024 | 122.283942 |
| 128 | Encryption |  16.00 KB | 2048 | 123.535588 |
| 128 | Decryption |  16.00 KB | 2048 | 123.505323 |
| 128 | Encryption |  16.00 KB | 4096 | 124.195777 |
| 128 | Decryption |  16.00 KB | 4096 | 123.953359 |
| 128 | Encryption |  16.00 KB | 8192 | 124.408435 |
| 128 | Decryption |  16.00 KB | 8192 | 124.128030 |
| 128 | Encryption |  32.00 KB | 256 | 117.286917 |
| 128 | Decryption |  32.00 KB | 256 | 117.361585 |
| 128 | Encryption |  32.00 KB | 512 | 121.110761 |
| 128 | Decryption |  32.00 KB | 512 | 121.037618 |
| 128 | Encryption |  32.00 KB | 1024 | 123.027826 |
| 128 | Decryption |  32.00 KB | 1024 | 122.859690 |
| 128 | Encryption |  32.00 KB | 2048 | 123.920194 |
| 128 | Decryption |  32.00 KB | 2048 | 123.890443 |
| 128 | Encryption |  32.00 KB | 4096 | 124.365114 |
| 128 | Decryption |  32.00 KB | 4096 | 124.292703 |
| 128 | Encryption |  32.00 KB | 8192 | 124.778069 |
| 128 | Decryption |  32.00 KB | 8192 | 124.405011 |
| 128 | Encryption |  32.00 KB | 16384 | 124.832022 |
| 128 | Decryption |  32.00 KB | 16384 | 124.788523 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 114.977355 |
| 256 | Decryption |  32.00 KB | 115.074160 |
| 256 | Encryption |  16.00 KB | 114.476362 |
| 256 | Decryption |  16.00 KB | 114.936521 |
| 256 | Encryption |  8.00 KB | 114.370679 |
| 256 | Decryption |  8.00 KB | 115.030734 |
| 256 | Encryption |  4.00 KB | 116.103887 |
| 256 | Decryption |  4.00 KB | 116.720097 |
| 256 | Encryption |  2.00 KB | 114.525374 |
| 256 | Decryption |  2.00 KB | 115.500663 |
| 256 | Encryption |  1024.00 B | 110.347800 |
| 256 | Decryption |  1024.00 B | 110.332938 |
| 256 | Encryption |  512.00 B | 100.782442 |
| 256 | Decryption |  512.00 B | 101.185771 |
| 128 | Encryption |  32.00 KB | 124.907086 |
| 128 | Decryption |  32.00 KB | 124.932207 |
| 128 | Encryption |  16.00 KB | 124.251347 |
| 128 | Decryption |  16.00 KB | 124.534203 |
| 128 | Encryption |  8.00 KB | 124.739713 |
| 128 | Decryption |  8.00 KB | 123.924060 |
| 128 | Encryption |  4.00 KB | 126.817035 |
| 128 | Decryption |  4.00 KB | 128.192288 |
| 128 | Encryption |  2.00 KB | 125.085890 |
| 128 | Decryption |  2.00 KB | 125.519038 |
| 128 | Encryption |  1024.00 B | 118.890050 |
| 128 | Decryption |  1024.00 B | 119.497039 |
| 128 | Encryption |  512.00 B | 108.722196 |
| 128 | Decryption |  512.00 B | 109.185904 |

### AES CBC STREAM

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 84.387490 |
| 256 | Decryption |  512.00 B | 256 | 90.503336 |
| 256 | Encryption |  1024.00 B | 256 | 99.593941 |
| 256 | Decryption |  1024.00 B | 256 | 100.730394 |
| 256 | Encryption |  1024.00 B | 512 | 104.840155 |
| 256 | Decryption |  1024.00 B | 512 | 104.751675 |
| 256 | Encryption |  2.00 KB | 256 | 106.834987 |
| 256 | Decryption |  2.00 KB | 256 | 107.548904 |
| 256 | Encryption |  2.00 KB | 512 | 110.927556 |
| 256 | Decryption |  2.00 KB | 512 | 110.903528 |
| 256 | Encryption |  2.00 KB | 1024 | 112.663659 |
| 256 | Decryption |  2.00 KB | 1024 | 112.530564 |
| 256 | Encryption |  4.00 KB | 256 | 109.084856 |
| 256 | Decryption |  4.00 KB | 256 | 110.180833 |
| 256 | Encryption |  4.00 KB | 512 | 113.640462 |
| 256 | Decryption |  4.00 KB | 512 | 113.649133 |
| 256 | Encryption |  4.00 KB | 1024 | 115.788804 |
| 256 | Decryption |  4.00 KB | 1024 | 115.728282 |
| 256 | Encryption |  4.00 KB | 2048 | 116.357852 |
| 256 | Decryption |  4.00 KB | 2048 | 116.577251 |
| 256 | Encryption |  8.00 KB | 256 | 108.166991 |
| 256 | Decryption |  8.00 KB | 256 | 108.701638 |
| 256 | Encryption |  8.00 KB | 512 | 112.359628 |
| 256 | Decryption |  8.00 KB | 512 | 111.912186 |
| 256 | Encryption |  8.00 KB | 1024 | 113.958608 |
| 256 | Decryption |  8.00 KB | 1024 | 113.953852 |
| 256 | Encryption |  8.00 KB | 2048 | 114.529777 |
| 256 | Decryption |  8.00 KB | 2048 | 114.608290 |
| 256 | Encryption |  8.00 KB | 4096 | 115.169003 |
| 256 | Decryption |  8.00 KB | 4096 | 114.932289 |
| 256 | Encryption |  16.00 KB | 256 | 108.125409 |
| 256 | Decryption |  16.00 KB | 256 | 108.398323 |
| 256 | Encryption |  16.00 KB | 512 | 111.646226 |
| 256 | Decryption |  16.00 KB | 512 | 111.218025 |
| 256 | Encryption |  16.00 KB | 1024 | 113.148195 |
| 256 | Decryption |  16.00 KB | 1024 | 112.944226 |
| 256 | Encryption |  16.00 KB | 2048 | 113.978626 |
| 256 | Decryption |  16.00 KB | 2048 | 113.608745 |
| 256 | Encryption |  16.00 KB | 4096 | 114.338553 |
| 256 | Decryption |  16.00 KB | 4096 | 114.209832 |
| 256 | Encryption |  16.00 KB | 8192 | 114.498362 |
| 256 | Decryption |  16.00 KB | 8192 | 114.232128 |
| 256 | Encryption |  32.00 KB | 256 | 108.506096 |
| 256 | Decryption |  32.00 KB | 256 | 108.692714 |
| 256 | Encryption |  32.00 KB | 512 | 111.739407 |
| 256 | Decryption |  32.00 KB | 512 | 111.754459 |
| 256 | Encryption |  32.00 KB | 1024 | 113.376531 |
| 256 | Decryption |  32.00 KB | 1024 | 113.246639 |
| 256 | Encryption |  32.00 KB | 2048 | 114.190231 |
| 256 | Decryption |  32.00 KB | 2048 | 114.055094 |
| 256 | Encryption |  32.00 KB | 4096 | 114.606186 |
| 256 | Decryption |  32.00 KB | 4096 | 114.441479 |
| 256 | Encryption |  32.00 KB | 8192 | 114.771367 |
| 256 | Decryption |  32.00 KB | 8192 | 114.676776 |
| 256 | Encryption |  32.00 KB | 16384 | 114.851822 |
| 256 | Decryption |  32.00 KB | 16384 | 114.857659 |
| 128 | Encryption |  512.00 B | 256 | 93.622857 |
| 128 | Decryption |  512.00 B | 256 | 96.804689 |
| 128 | Encryption |  1024.00 B | 256 | 107.769621 |
| 128 | Decryption |  1024.00 B | 256 | 109.800558 |
| 128 | Encryption |  1024.00 B | 512 | 113.258676 |
| 128 | Decryption |  1024.00 B | 512 | 113.459461 |
| 128 | Encryption |  2.00 KB | 256 | 116.012632 |
| 128 | Decryption |  2.00 KB | 256 | 116.196933 |
| 128 | Encryption |  2.00 KB | 512 | 120.083848 |
| 128 | Decryption |  2.00 KB | 512 | 120.178977 |
| 128 | Encryption |  2.00 KB | 1024 | 122.037332 |
| 128 | Decryption |  2.00 KB | 1024 | 122.139226 |
| 128 | Encryption |  4.00 KB | 256 | 117.006006 |
| 128 | Decryption |  4.00 KB | 256 | 118.636090 |
| 128 | Encryption |  4.00 KB | 512 | 123.259331 |
| 128 | Decryption |  4.00 KB | 512 | 123.777074 |
| 128 | Encryption |  4.00 KB | 1024 | 125.789833 |
| 128 | Decryption |  4.00 KB | 1024 | 125.367287 |
| 128 | Encryption |  4.00 KB | 2048 | 126.867116 |
| 128 | Decryption |  4.00 KB | 2048 | 126.752282 |
| 128 | Encryption |  8.00 KB | 256 | 117.511628 |
| 128 | Decryption |  8.00 KB | 256 | 117.299617 |
| 128 | Encryption |  8.00 KB | 512 | 121.173125 |
| 128 | Decryption |  8.00 KB | 512 | 121.529954 |
| 128 | Encryption |  8.00 KB | 1024 | 123.334952 |
| 128 | Decryption |  8.00 KB | 1024 | 123.482750 |
| 128 | Encryption |  8.00 KB | 2048 | 124.644815 |
| 128 | Decryption |  8.00 KB | 2048 | 124.262891 |
| 128 | Encryption |  8.00 KB | 4096 | 125.140825 |
| 128 | Decryption |  8.00 KB | 4096 | 124.701262 |
| 128 | Encryption |  16.00 KB | 256 | 116.988670 |
| 128 | Decryption |  16.00 KB | 256 | 116.987835 |
| 128 | Encryption |  16.00 KB | 512 | 120.789214 |
| 128 | Decryption |  16.00 KB | 512 | 120.491630 |
| 128 | Encryption |  16.00 KB | 1024 | 122.585871 |
| 128 | Decryption |  16.00 KB | 1024 | 122.316351 |
| 128 | Encryption |  16.00 KB | 2048 | 123.675465 |
| 128 | Decryption |  16.00 KB | 2048 | 123.243337 |
| 128 | Encryption |  16.00 KB | 4096 | 124.072100 |
| 128 | Decryption |  16.00 KB | 4096 | 123.936481 |
| 128 | Encryption |  16.00 KB | 8192 | 124.377741 |
| 128 | Decryption |  16.00 KB | 8192 | 124.041102 |
| 128 | Encryption |  32.00 KB | 256 | 117.205740 |
| 128 | Decryption |  32.00 KB | 256 | 117.212658 |
| 128 | Encryption |  32.00 KB | 512 | 121.049020 |
| 128 | Decryption |  32.00 KB | 512 | 120.972156 |
| 128 | Encryption |  32.00 KB | 1024 | 122.885722 |
| 128 | Decryption |  32.00 KB | 1024 | 122.848635 |
| 128 | Encryption |  32.00 KB | 2048 | 123.888335 |
| 128 | Decryption |  32.00 KB | 2048 | 123.811098 |
| 128 | Encryption |  32.00 KB | 4096 | 124.400997 |
| 128 | Decryption |  32.00 KB | 4096 | 124.252407 |
| 128 | Encryption |  32.00 KB | 8192 | 124.682875 |
| 128 | Decryption |  32.00 KB | 8192 | 124.423670 |
| 128 | Encryption |  32.00 KB | 16384 | 124.756217 |
| 128 | Decryption |  32.00 KB | 16384 | 124.640192 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 371.999035 |
| 512 |  16.00 KB | 370.869622 |
| 512 |  8.00 KB | 366.569341 |
| 512 |  4.00 KB | 358.504190 |
| 512 |  2.00 KB | 342.489234 |
| 512 |  1024.00 B | 314.665438 |
| 512 |  512.00 B | 270.006592 |
| 256 |  32.00 KB | 360.220521 |
| 256 |  16.00 KB | 359.091532 |
| 256 |  8.00 KB | 353.961653 |
| 256 |  4.00 KB | 349.764106 |
| 256 |  2.00 KB | 338.051417 |
| 256 |  1024.00 B | 317.052403 |
| 256 |  512.00 B | 281.899518 |


### AES CTR STREAM

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 83.728536 |
| 256 | Decryption |  512.00 B | 256 | 90.937347 |
| 256 | Encryption |  1024.00 B | 256 | 99.460930 |
| 256 | Decryption |  1024.00 B | 256 | 101.290865 |
| 256 | Encryption |  1024.00 B | 512 | 103.801318 |
| 256 | Decryption |  1024.00 B | 512 | 104.444501 |
| 256 | Encryption |  2.00 KB | 256 | 106.953547 |
| 256 | Decryption |  2.00 KB | 256 | 107.616721 |
| 256 | Encryption |  2.00 KB | 512 | 110.743109 |
| 256 | Decryption |  2.00 KB | 512 | 110.632436 |
| 256 | Encryption |  2.00 KB | 1024 | 112.550663 |
| 256 | Decryption |  2.00 KB | 1024 | 112.544478 |
| 256 | Encryption |  4.00 KB | 256 | 110.072759 |
| 256 | Decryption |  4.00 KB | 256 | 110.049839 |
| 256 | Encryption |  4.00 KB | 512 | 113.844187 |
| 256 | Decryption |  4.00 KB | 512 | 113.989133 |
| 256 | Encryption |  4.00 KB | 1024 | 115.312880 |
| 256 | Decryption |  4.00 KB | 1024 | 115.671905 |
| 256 | Encryption |  4.00 KB | 2048 | 116.646969 |
| 256 | Decryption |  4.00 KB | 2048 | 116.280225 |
| 256 | Encryption |  8.00 KB | 256 | 108.448010 |
| 256 | Decryption |  8.00 KB | 256 | 108.632087 |
| 256 | Encryption |  8.00 KB | 512 | 111.767515 |
| 256 | Decryption |  8.00 KB | 512 | 112.013564 |
| 256 | Encryption |  8.00 KB | 1024 | 114.143990 |
| 256 | Decryption |  8.00 KB | 1024 | 113.769877 |
| 256 | Encryption |  8.00 KB | 2048 | 114.540586 |
| 256 | Decryption |  8.00 KB | 2048 | 114.754787 |
| 256 | Encryption |  8.00 KB | 4096 | 114.840853 |
| 256 | Decryption |  8.00 KB | 4096 | 114.791772 |
| 256 | Encryption |  16.00 KB | 256 | 108.303201 |
| 256 | Decryption |  16.00 KB | 256 | 108.112388 |
| 256 | Encryption |  16.00 KB | 512 | 111.494464 |
| 256 | Decryption |  16.00 KB | 512 | 111.378878 |
| 256 | Encryption |  16.00 KB | 1024 | 113.130030 |
| 256 | Decryption |  16.00 KB | 1024 | 112.874780 |
| 256 | Encryption |  16.00 KB | 2048 | 113.983581 |
| 256 | Decryption |  16.00 KB | 2048 | 113.584133 |
| 256 | Encryption |  16.00 KB | 4096 | 114.215604 |
| 256 | Decryption |  16.00 KB | 4096 | 114.192519 |
| 256 | Encryption |  16.00 KB | 8192 | 114.514568 |
| 256 | Decryption |  16.00 KB | 8192 | 114.329976 |
| 256 | Encryption |  32.00 KB | 256 | 108.389538 |
| 256 | Decryption |  32.00 KB | 256 | 108.462009 |
| 256 | Encryption |  32.00 KB | 512 | 111.766943 |
| 256 | Decryption |  32.00 KB | 512 | 111.673051 |
| 256 | Encryption |  32.00 KB | 1024 | 113.361136 |
| 256 | Decryption |  32.00 KB | 1024 | 113.169296 |
| 256 | Encryption |  32.00 KB | 2048 | 114.222572 |
| 256 | Decryption |  32.00 KB | 2048 | 114.069487 |
| 256 | Encryption |  32.00 KB | 4096 | 114.543589 |
| 256 | Decryption |  32.00 KB | 4096 | 114.499762 |
| 256 | Encryption |  32.00 KB | 8192 | 114.780312 |
| 256 | Decryption |  32.00 KB | 8192 | 114.630341 |
| 256 | Encryption |  32.00 KB | 16384 | 115.007923 |
| 256 | Decryption |  32.00 KB | 16384 | 114.780111 |
| 128 | Encryption |  512.00 B | 256 | 92.493903 |
| 128 | Decryption |  512.00 B | 256 | 96.105115 |
| 128 | Encryption |  1024.00 B | 256 | 108.376991 |
| 128 | Decryption |  1024.00 B | 256 | 109.641844 |
| 128 | Encryption |  1024.00 B | 512 | 113.142920 |
| 128 | Decryption |  1024.00 B | 512 | 113.399779 |
| 128 | Encryption |  2.00 KB | 256 | 114.355910 |
| 128 | Decryption |  2.00 KB | 256 | 116.393396 |
| 128 | Encryption |  2.00 KB | 512 | 119.347319 |
| 128 | Decryption |  2.00 KB | 512 | 120.022270 |
| 128 | Encryption |  2.00 KB | 1024 | 121.895692 |
| 128 | Decryption |  2.00 KB | 1024 | 121.973735 |
| 128 | Encryption |  4.00 KB | 256 | 119.161563 |
| 128 | Decryption |  4.00 KB | 256 | 119.270864 |
| 128 | Encryption |  4.00 KB | 512 | 123.656563 |
| 128 | Decryption |  4.00 KB | 512 | 123.792973 |
| 128 | Encryption |  4.00 KB | 1024 | 125.415270 |
| 128 | Decryption |  4.00 KB | 1024 | 125.624904 |
| 128 | Encryption |  4.00 KB | 2048 | 126.932970 |
| 128 | Decryption |  4.00 KB | 2048 | 126.940837 |
| 128 | Encryption |  8.00 KB | 256 | 117.315996 |
| 128 | Decryption |  8.00 KB | 256 | 117.705800 |
| 128 | Encryption |  8.00 KB | 512 | 121.060759 |
| 128 | Decryption |  8.00 KB | 512 | 121.415126 |
| 128 | Encryption |  8.00 KB | 1024 | 123.369314 |
| 128 | Decryption |  8.00 KB | 1024 | 123.075074 |
| 128 | Encryption |  8.00 KB | 2048 | 124.781989 |
| 128 | Decryption |  8.00 KB | 2048 | 124.312390 |
| 128 | Encryption |  8.00 KB | 4096 | 125.151818 |
| 128 | Decryption |  8.00 KB | 4096 | 124.885664 |
| 128 | Encryption |  16.00 KB | 256 | 116.962571 |
| 128 | Decryption |  16.00 KB | 256 | 116.845791 |
| 128 | Encryption |  16.00 KB | 512 | 120.598281 |
| 128 | Decryption |  16.00 KB | 512 | 120.646679 |
| 128 | Encryption |  16.00 KB | 1024 | 122.521930 |
| 128 | Decryption |  16.00 KB | 1024 | 122.392419 |
| 128 | Encryption |  16.00 KB | 2048 | 123.661230 |
| 128 | Decryption |  16.00 KB | 2048 | 123.302001 |
| 128 | Encryption |  16.00 KB | 4096 | 124.056364 |
| 128 | Decryption |  16.00 KB | 4096 | 123.715388 |
| 128 | Encryption |  16.00 KB | 8192 | 124.448360 |
| 128 | Decryption |  16.00 KB | 8192 | 124.068107 |
| 128 | Encryption |  32.00 KB | 256 | 117.144880 |
| 128 | Decryption |  32.00 KB | 256 | 117.254496 |
| 128 | Encryption |  32.00 KB | 512 | 121.118259 |
| 128 | Decryption |  32.00 KB | 512 | 120.812149 |
| 128 | Encryption |  32.00 KB | 1024 | 122.959847 |
| 128 | Decryption |  32.00 KB | 1024 | 122.727971 |
| 128 | Encryption |  32.00 KB | 2048 | 123.836950 |
| 128 | Decryption |  32.00 KB | 2048 | 123.694840 |
| 128 | Encryption |  32.00 KB | 4096 | 124.381754 |
| 128 | Decryption |  32.00 KB | 4096 | 124.232739 |
| 128 | Encryption |  32.00 KB | 8192 | 124.689399 |
| 128 | Decryption |  32.00 KB | 8192 | 124.541066 |
| 128 | Encryption |  32.00 KB | 16384 | 124.748499 |
| 128 | Decryption |  32.00 KB | 16384 | 124.762985 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 115.177201 |
| 256 | Decryption |  32.00 KB | 115.096290 |
| 256 | Encryption |  16.00 KB | 114.915360 |
| 256 | Decryption |  16.00 KB | 114.781920 |
| 256 | Encryption |  8.00 KB | 114.397031 |
| 256 | Decryption |  8.00 KB | 115.218003 |
| 256 | Encryption |  4.00 KB | 115.819908 |
| 256 | Decryption |  4.00 KB | 117.229536 |
| 256 | Encryption |  2.00 KB | 114.534981 |
| 256 | Decryption |  2.00 KB | 115.313692 |
| 256 | Encryption |  1024.00 B | 110.172683 |
| 256 | Decryption |  1024.00 B | 110.574198 |
| 256 | Encryption |  512.00 B | 98.841699 |
| 256 | Decryption |  512.00 B | 101.240793 |
| 128 | Encryption |  32.00 KB | 124.681689 |
| 128 | Decryption |  32.00 KB | 124.943639 |
| 128 | Encryption |  16.00 KB | 125.011445 |
| 128 | Decryption |  16.00 KB | 124.784128 |
| 128 | Encryption |  8.00 KB | 124.495642 |
| 128 | Decryption |  8.00 KB | 124.459705 |
| 128 | Encryption |  4.00 KB | 126.047252 |
| 128 | Decryption |  4.00 KB | 127.813273 |
| 128 | Encryption |  2.00 KB | 124.543907 |
| 128 | Decryption |  2.00 KB | 125.336597 |
| 128 | Encryption |  1024.00 B | 119.028246 |
| 128 | Decryption |  1024.00 B | 119.423873 |
| 128 | Encryption |  512.00 B | 107.971320 |
| 128 | Decryption |  512.00 B | 109.215017 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 369.802335 |
| 512 |  16.00 KB | 365.461399 |
| 512 |  8.00 KB | 354.585985 |
| 512 |  4.00 KB | 338.722349 |
| 512 |  2.00 KB | 308.294445 |
| 512 |  1024.00 B | 261.558110 |
| 512 |  512.00 B | 201.080020 |
| 256 |  32.00 KB | 358.775032 |
| 256 |  16.00 KB | 355.193271 |
| 256 |  8.00 KB | 348.529005 |
| 256 |  4.00 KB | 335.696431 |
| 256 |  2.00 KB | 312.469009 |
| 256 |  1024.00 B | 274.622863 |
| 256 |  512.00 B | 220.903894 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 295.778112 |
| 256 |  16.00 KB | 294.383728 |
| 256 |  8.00 KB | 293.081705 |
| 256 |  4.00 KB | 273.778491 |
| 256 |  2.00 KB | 237.697307 |
| 256 |  1024.00 B | 187.734898 |
| 256 |  512.00 B | 132.625308 |
| 128 |  32.00 KB | 298.218950 |
| 128 |  16.00 KB | 303.871656 |
| 128 |  8.00 KB | 293.499095 |
| 128 |  4.00 KB | 273.879175 |
| 128 |  2.00 KB | 239.140589 |
| 128 |  1024.00 B | 189.946207 |
| 128 |  512.00 B | 133.961277 |

### AES CMAC STREAM

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      246.379645      |
| 256        |      32.00 KB     |        1024        |      269.872056      |
| 256        |      32.00 KB     |        2048        |      283.106325      |
| 256        |      32.00 KB     |        4096        |      292.345644      |
| 256        |      32.00 KB     |        8192        |      294.472353      |
| 256        |      32.00 KB     |        16384        |      296.262810      |
| 256        |      16.00 KB     |        512        |      243.912969      |
| 256        |      16.00 KB     |        1024        |      270.684642      |
| 256        |      16.00 KB     |        2048        |      285.658244      |
| 256        |      16.00 KB     |        4096        |      292.487861      |
| 256        |      16.00 KB     |        8192        |      300.323530      |
| 256        |      8.00 KB     |        512        |      241.335121      |
| 256        |      8.00 KB     |        1024        |      267.195055      |
| 256        |      8.00 KB     |        2048        |      281.606380      |
| 256        |      8.00 KB     |        4096        |      289.280859      |
| 256        |      4.00 KB     |        512        |      227.429206      |
| 256        |      4.00 KB     |        1024        |      249.334206      |
| 256        |      4.00 KB     |        2048        |      261.508013      |
| 256        |      2.00 KB     |        512        |      200.882786      |
| 256        |      2.00 KB     |        1024        |      218.622401      |
| 256        |      1024.00 B     |        512        |      162.617119      |
| 128        |      32.00 KB     |        512        |      245.611865      |
| 128        |      32.00 KB     |        1024        |      269.372891      |
| 128        |      32.00 KB     |        2048        |      283.994540      |
| 128        |      32.00 KB     |        4096        |      293.762873      |
| 128        |      32.00 KB     |        8192        |      293.457040      |
| 128        |      32.00 KB     |        16384        |      298.773649      |
| 128        |      16.00 KB     |        512        |      247.545733      |
| 128        |      16.00 KB     |        1024        |      275.849296      |
| 128        |      16.00 KB     |        2048        |      289.210646      |
| 128        |      16.00 KB     |        4096        |      298.099579      |
| 128        |      16.00 KB     |        8192        |      301.310787      |
| 128        |      8.00 KB     |        512        |      242.817657      |
| 128        |      8.00 KB     |        1024        |      266.990410      |
| 128        |      8.00 KB     |        2048        |      281.892243      |
| 128        |      8.00 KB     |        4096        |      289.365159      |
| 128        |      4.00 KB     |        512        |      228.431209      |
| 128        |      4.00 KB     |        1024        |      249.969486      |
| 128        |      4.00 KB     |        2048        |      262.677761      |
| 128        |      2.00 KB     |        512        |      201.917626      |
| 128        |      2.00 KB     |        1024        |      218.086947      |
| 128        |      1024.00 B     |        512        |      164.240747      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    42   
    1024      |      TCMA     |     TCMA           |    40   
    1024      |      TCMB     |     TCMB           |    40   
    1024      |      OCRAM    |     TCMA           |    41   
    1024      |      TCMA     |     OCRAM          |    40       

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 

Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.122002 		| 504			| 276.343994		|
cos  		|0.0000002870	| 66			| 66.092003 		| 504			| 277.593994		|
sincos sin  	|0.0000001790	| 453			| 79.783997 		| 467			| 275.239990		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.033997 		| 594			| 428.753998		|
acos 		|0.0000004770	| 76			| 76.003998 		| 528			| 383.145996		|
atan 		|0.0000005360	| 80			| 80.092003 		| 494			| 370.838013		|
atan2 		|0.0000007150	| 117			| 104.574005 		| 881			| 479.950012		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### XIP Benchmark

Comparing data from \ref EXAMPLES_OPTIFLASH_XIP_BENCHMARK and \ref EXAMPLES_OPTIFLASH_OCRAM_BENCHMARK, execution time of a code which throws ~3 Million I-Cache Miss per seconds is 2.2 times slower when it runs from OCRAM. 

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      460 |        17.391304 |
|        2 |      917 |        17.448201 |
|        4 |     1830 |        17.486339 |
|        8 |     3658 |        17.495899 |
|       16 |     7307 |        17.517449 |
|       32 |    14621 |        17.509062 |
|       64 |    29241 |        17.509661 |
|      128 |    58476 |        17.511458 |
|      256 |   116965 |        17.509511 |
|      512 |   233919 |        17.510335 |

#### DMA read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      461 |        17.353579 |
|        2 |       25 |       640.000000 |
|        4 |       36 |       888.888889 |
|        8 |       62 |      1032.258065 |
|       16 |      113 |      1132.743363 |
|       32 |      219 |      1168.949772 |
|       64 |      426 |      1201.877934 |
|      128 |      839 |      1220.500596 |
|      256 |     1670 |      1226.347305 |
|      512 |     3329 |      1230.399519 |
