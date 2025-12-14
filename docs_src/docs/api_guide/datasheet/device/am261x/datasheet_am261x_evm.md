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
SBL : System Init                       |   394
SBL : Drivers_open                      |   86
SBL : LoadHsmRtFw                       |   6282
SBL : Board_driversOpen                 |   4770
SBL : CPU Load                          |   4359
SBL : SBL End                           |   8
SBL : Total time taken                  |   15902

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 356 KB
- Size of hello_world                 : 5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   429
SBL : Drivers_open                      |   94
SBL : LoadHsmRtFw                       |   6282
SBL : Board_driversOpen                 |   4821
SBL : CPU Load                          |   4411
SBL : SBL End                           |   4
SBL : Total time taken                  |   16044

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
 r5f0-0	| r5f0-1	| 4	| 9.706
 r5f0-0	| r5f0-1	| 32	| 11.737
 r5f0-0	| r5f0-1	| 64	| 13.923
 r5f0-0	| r5f0-1	| 112	| 17.260

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
 400		| 08			| 10.76 Mbps / 297.48 us 	| 24.18 Mbps / 132.34 us 	|  0.92 Mbps / 3463.94 us
 200		| 16			| 21.68 Mbps / 147.63 us 	| 30.49 Mbps / 104.95 us 	|  0.96 Mbps / 3339.67 us
 100		| 32			| 37.79 Mbps / 84.67 us 	| 35.47 Mbps / 90.22 us 	|  0.98 Mbps / 3277.71 us
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
- End tick         : 8186156
- Total ticks      : 8186100
- Total time (secs): 8.186100
- Iterations/Sec   : 1832.374391
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1832.374391 
CoreMark/MHz :4.580936 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146103303
- USER cycle count:                          146103296

BENCHMARK Using clock 500000000
- Usertime in sec:                           0.292207
- Microseconds for one run through Dhrystone:   0.6 
- Dhrystones per Second:                     1711118.2 

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
| 128 |  32.00 KB | 247.618220 |
| 128 |  16.00 KB | 244.399611 |
| 128 |  8.00 KB | 243.722480 |
| 128 |  4.00 KB | 228.227559 |
| 128 |  2.00 KB | 202.837547 |
| 128 |  1024.00 B | 161.986870 |
| 128 |  512.00 B | 121.004431 |

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
| 256 | Encryption |  16.00 KB | 114.770262 |
| 256 | Decryption |  16.00 KB | 114.776493 |
| 256 | Encryption |  8.00 KB | 114.827573 |
| 256 | Decryption |  8.00 KB | 114.847696 |
| 256 | Encryption |  4.00 KB | 116.659428 |
| 256 | Decryption |  4.00 KB | 117.454764 |
| 256 | Encryption |  2.00 KB | 115.019025 |
| 256 | Decryption |  2.00 KB | 116.126106 |
| 256 | Encryption |  1024.00 B | 110.711679 |
| 256 | Decryption |  1024.00 B | 111.262020 |
| 256 | Encryption |  512.00 B | 102.806084 |
| 256 | Decryption |  512.00 B | 102.759659 |
| 128 | Encryption |  32.00 KB | 124.898397 |
| 128 | Decryption |  32.00 KB | 124.935303 |
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
| 256 | Decryption |  4.00 KB | 512 | 114.059759 |
| 256 | Encryption |  4.00 KB | 1024 | 116.074275 |
| 256 | Decryption |  4.00 KB | 1024 | 116.024134 |
| 256 | Encryption |  4.00 KB | 2048 | 116.554859 |
| 256 | Decryption |  4.00 KB | 2048 | 116.849125 |
| 256 | Encryption |  8.00 KB | 256 | 107.943222 |
| 256 | Decryption |  8.00 KB | 256 | 108.918065 |
| 256 | Encryption |  8.00 KB | 512 | 112.485368 |
| 256 | Decryption |  8.00 KB | 512 | 112.221868 |
| 256 | Encryption |  8.00 KB | 1024 | 113.922950 |
| 256 | Decryption |  8.00 KB | 1024 | 113.777383 |
| 256 | Encryption |  8.00 KB | 2048 | 114.726261 |
| 256 | Decryption |  8.00 KB | 2048 | 114.918584 |
| 256 | Encryption |  8.00 KB | 4096 | 115.482345 |
| 256 | Decryption |  8.00 KB | 4096 | 114.959707 |
| 256 | Encryption |  16.00 KB | 256 | 108.302664 |
| 256 | Decryption |  16.00 KB | 256 | 108.401371 |
| 256 | Encryption |  16.00 KB | 512 | 111.626069 |
| 256 | Decryption |  16.00 KB | 512 | 111.350303 |
| 256 | Encryption |  16.00 KB | 1024 | 113.176332 |
| 256 | Decryption |  16.00 KB | 1024 | 113.015512 |
| 256 | Encryption |  16.00 KB | 2048 | 113.941767 |
| 256 | Decryption |  16.00 KB | 2048 | 113.715984 |
| 256 | Encryption |  16.00 KB | 4096 | 114.437582 |
| 256 | Decryption |  16.00 KB | 4096 | 114.211424 |
| 256 | Encryption |  16.00 KB | 8192 | 114.574829 |
| 256 | Decryption |  16.00 KB | 8192 | 114.463965 |
| 256 | Encryption |  32.00 KB | 256 | 108.439486 |
| 256 | Decryption |  32.00 KB | 256 | 108.711194 |
| 256 | Encryption |  32.00 KB | 512 | 111.837036 |
| 256 | Decryption |  32.00 KB | 512 | 111.666772 |
| 256 | Encryption |  32.00 KB | 1024 | 113.423232 |
| 256 | Decryption |  32.00 KB | 1024 | 113.283245 |
| 256 | Encryption |  32.00 KB | 2048 | 114.233024 |
| 256 | Decryption |  32.00 KB | 2048 | 114.085273 |
| 256 | Encryption |  32.00 KB | 4096 | 114.659521 |
| 256 | Decryption |  32.00 KB | 4096 | 114.538885 |
| 256 | Encryption |  32.00 KB | 8192 | 114.809368 |
| 256 | Decryption |  32.00 KB | 8192 | 114.790666 |
| 256 | Encryption |  32.00 KB | 16384 | 114.893501 |
| 256 | Decryption |  32.00 KB | 16384 | 114.874772 |
| 128 | Encryption |  512.00 B | 256 | 94.801648 |
| 128 | Decryption |  512.00 B | 256 | 97.952937 |
| 128 | Encryption |  1024.00 B | 256 | 109.119003 |
| 128 | Decryption |  1024.00 B | 256 | 110.672791 |
| 128 | Encryption |  1024.00 B | 512 | 114.336757 |
| 128 | Decryption |  1024.00 B | 512 | 114.237903 |
| 128 | Encryption |  2.00 KB | 256 | 114.770865 |
| 128 | Decryption |  2.00 KB | 256 | 115.958441 |
| 128 | Encryption |  2.00 KB | 512 | 120.713791 |
| 128 | Decryption |  2.00 KB | 512 | 120.777861 |
| 128 | Encryption |  2.00 KB | 1024 | 122.455081 |
| 128 | Decryption |  2.00 KB | 1024 | 122.352660 |
| 128 | Encryption |  4.00 KB | 256 | 118.313968 |
| 128 | Decryption |  4.00 KB | 256 | 119.137302 |
| 128 | Encryption |  4.00 KB | 512 | 123.592200 |
| 128 | Decryption |  4.00 KB | 512 | 124.108989 |
| 128 | Encryption |  4.00 KB | 1024 | 126.276523 |
| 128 | Decryption |  4.00 KB | 1024 | 125.633574 |
| 128 | Encryption |  4.00 KB | 2048 | 127.181271 |
| 128 | Decryption |  4.00 KB | 2048 | 127.182258 |
| 128 | Encryption |  8.00 KB | 256 | 117.751481 |
| 128 | Decryption |  8.00 KB | 256 | 117.313476 |
| 128 | Encryption |  8.00 KB | 512 | 121.753024 |
| 128 | Decryption |  8.00 KB | 512 | 121.709610 |
| 128 | Encryption |  8.00 KB | 1024 | 123.601990 |
| 128 | Decryption |  8.00 KB | 1024 | 123.741083 |
| 128 | Encryption |  8.00 KB | 2048 | 124.773912 |
| 128 | Decryption |  8.00 KB | 2048 | 124.412923 |
| 128 | Encryption |  8.00 KB | 4096 | 125.531059 |
| 128 | Decryption |  8.00 KB | 4096 | 125.011922 |
| 128 | Encryption |  16.00 KB | 256 | 116.922715 |
| 128 | Decryption |  16.00 KB | 256 | 117.319986 |
| 128 | Encryption |  16.00 KB | 512 | 121.065456 |
| 128 | Decryption |  16.00 KB | 512 | 120.516890 |
| 128 | Encryption |  16.00 KB | 1024 | 122.804092 |
| 128 | Decryption |  16.00 KB | 1024 | 122.468124 |
| 128 | Encryption |  16.00 KB | 2048 | 123.718891 |
| 128 | Decryption |  16.00 KB | 2048 | 123.358864 |
| 128 | Encryption |  16.00 KB | 4096 | 124.256529 |
| 128 | Decryption |  16.00 KB | 4096 | 123.784322 |
| 128 | Encryption |  16.00 KB | 8192 | 124.459941 |
| 128 | Decryption |  16.00 KB | 8192 | 124.232033 |
| 128 | Encryption |  32.00 KB | 256 | 117.329963 |
| 128 | Decryption |  32.00 KB | 256 | 117.464764 |
| 128 | Encryption |  32.00 KB | 512 | 121.112328 |
| 128 | Decryption |  32.00 KB | 512 | 121.024207 |
| 128 | Encryption |  32.00 KB | 1024 | 123.023438 |
| 128 | Decryption |  32.00 KB | 1024 | 122.829295 |
| 128 | Encryption |  32.00 KB | 2048 | 123.955820 |
| 128 | Decryption |  32.00 KB | 2048 | 123.830749 |
| 128 | Encryption |  32.00 KB | 4096 | 124.462896 |
| 128 | Decryption |  32.00 KB | 4096 | 124.277383 |
| 128 | Encryption |  32.00 KB | 8192 | 124.642563 |
| 128 | Decryption |  32.00 KB | 8192 | 124.663664 |
| 128 | Encryption |  32.00 KB | 16384 | 124.829644 |
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
| 256 | Decryption |  16.00 KB | 114.935715 |
| 256 | Encryption |  8.00 KB | 114.370679 |
| 256 | Decryption |  8.00 KB | 115.030734 |
| 256 | Encryption |  4.00 KB | 116.103887 |
| 256 | Decryption |  4.00 KB | 116.720097 |
| 256 | Encryption |  2.00 KB | 114.522172 |
| 256 | Decryption |  2.00 KB | 115.500663 |
| 256 | Encryption |  1024.00 B | 110.347800 |
| 256 | Decryption |  1024.00 B | 110.332938 |
| 256 | Encryption |  512.00 B | 100.782442 |
| 256 | Decryption |  512.00 B | 101.185771 |
| 128 | Encryption |  32.00 KB | 124.907086 |
| 128 | Decryption |  32.00 KB | 124.932207 |
| 128 | Encryption |  16.00 KB | 124.227794 |
| 128 | Decryption |  16.00 KB | 124.638888 |
| 128 | Encryption |  8.00 KB | 123.975166 |
| 128 | Decryption |  8.00 KB | 124.308617 |
| 128 | Encryption |  4.00 KB | 126.840598 |
| 128 | Decryption |  4.00 KB | 128.094069 |
| 128 | Encryption |  2.00 KB | 124.883760 |
| 128 | Decryption |  2.00 KB | 125.503654 |
| 128 | Encryption |  1024.00 B | 118.972929 |
| 128 | Decryption |  1024.00 B | 119.559824 |
| 128 | Encryption |  512.00 B | 108.895624 |
| 128 | Decryption |  512.00 B | 109.454332 |

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
| 256 | Encryption |  2.00 KB | 256 | 106.768152 |
| 256 | Decryption |  2.00 KB | 256 | 107.564438 |
| 256 | Encryption |  2.00 KB | 512 | 110.984664 |
| 256 | Decryption |  2.00 KB | 512 | 110.900525 |
| 256 | Encryption |  2.00 KB | 1024 | 112.660561 |
| 256 | Decryption |  2.00 KB | 1024 | 112.527473 |
| 256 | Encryption |  4.00 KB | 256 | 109.151716 |
| 256 | Decryption |  4.00 KB | 256 | 110.232724 |
| 256 | Encryption |  4.00 KB | 512 | 113.631004 |
| 256 | Decryption |  4.00 KB | 512 | 113.657017 |
| 256 | Encryption |  4.00 KB | 1024 | 115.793714 |
| 256 | Decryption |  4.00 KB | 1024 | 115.733186 |
| 256 | Encryption |  4.00 KB | 2048 | 116.394223 |
| 256 | Decryption |  4.00 KB | 2048 | 116.579739 |
| 256 | Encryption |  8.00 KB | 256 | 108.154852 |
| 256 | Decryption |  8.00 KB | 256 | 108.708850 |
| 256 | Encryption |  8.00 KB | 512 | 112.351923 |
| 256 | Decryption |  8.00 KB | 512 | 111.916773 |
| 256 | Encryption |  8.00 KB | 1024 | 113.966535 |
| 256 | Decryption |  8.00 KB | 1024 | 113.953060 |
| 256 | Encryption |  8.00 KB | 2048 | 114.534581 |
| 256 | Decryption |  8.00 KB | 2048 | 114.611497 |
| 256 | Encryption |  8.00 KB | 4096 | 115.156052 |
| 256 | Decryption |  8.00 KB | 4096 | 114.941158 |
| 256 | Encryption |  16.00 KB | 256 | 108.172168 |
| 256 | Decryption |  16.00 KB | 256 | 108.264377 |
| 256 | Encryption |  16.00 KB | 512 | 111.619225 |
| 256 | Decryption |  16.00 KB | 512 | 111.350303 |
| 256 | Encryption |  16.00 KB | 1024 | 113.173205 |
| 256 | Decryption |  16.00 KB | 1024 | 112.911146 |
| 256 | Encryption |  16.00 KB | 2048 | 113.949691 |
| 256 | Decryption |  16.00 KB | 2048 | 113.631595 |
| 256 | Encryption |  16.00 KB | 4096 | 114.425194 |
| 256 | Decryption |  16.00 KB | 4096 | 114.181974 |
| 256 | Encryption |  16.00 KB | 8192 | 114.543589 |
| 256 | Decryption |  16.00 KB | 8192 | 114.229739 |
| 256 | Encryption |  32.00 KB | 256 | 108.424954 |
| 256 | Decryption |  32.00 KB | 256 | 108.754401 |
| 256 | Encryption |  32.00 KB | 512 | 111.731977 |
| 256 | Decryption |  32.00 KB | 512 | 111.714454 |
| 256 | Encryption |  32.00 KB | 1024 | 113.386731 |
| 256 | Decryption |  32.00 KB | 1024 | 113.266016 |
| 256 | Encryption |  32.00 KB | 2048 | 114.200976 |
| 256 | Decryption |  32.00 KB | 2048 | 114.014020 |
| 256 | Encryption |  32.00 KB | 4096 | 114.546092 |
| 256 | Decryption |  32.00 KB | 4096 | 114.537283 |
| 256 | Encryption |  32.00 KB | 8192 | 114.760916 |
| 256 | Decryption |  32.00 KB | 8192 | 114.651097 |
| 256 | Encryption |  32.00 KB | 16384 | 114.867926 |
| 256 | Decryption |  32.00 KB | 16384 | 114.918483 |
| 128 | Encryption |  512.00 B | 256 | 93.682814 |
| 128 | Decryption |  512.00 B | 256 | 96.118646 |
| 128 | Encryption |  1024.00 B | 256 | 107.769621 |
| 128 | Decryption |  1024.00 B | 256 | 109.509932 |
| 128 | Encryption |  1024.00 B | 512 | 113.277469 |
| 128 | Decryption |  1024.00 B | 512 | 113.277469 |
| 128 | Encryption |  2.00 KB | 256 | 114.562211 |
| 128 | Decryption |  2.00 KB | 256 | 115.372157 |
| 128 | Encryption |  2.00 KB | 512 | 120.129632 |
| 128 | Decryption |  2.00 KB | 512 | 120.203665 |
| 128 | Encryption |  2.00 KB | 1024 | 121.950130 |
| 128 | Decryption |  2.00 KB | 1024 | 121.964655 |
| 128 | Encryption |  4.00 KB | 256 | 119.476996 |
| 128 | Decryption |  4.00 KB | 256 | 119.688215 |
| 128 | Encryption |  4.00 KB | 512 | 123.198159 |
| 128 | Decryption |  4.00 KB | 512 | 123.823847 |
| 128 | Encryption |  4.00 KB | 1024 | 125.911637 |
| 128 | Decryption |  4.00 KB | 1024 | 125.429672 |
| 128 | Encryption |  4.00 KB | 2048 | 126.847472 |
| 128 | Decryption |  4.00 KB | 2048 | 126.914288 |
| 128 | Encryption |  8.00 KB | 256 | 117.927771 |
| 128 | Decryption |  8.00 KB | 256 | 117.931166 |
| 128 | Encryption |  8.00 KB | 512 | 121.330159 |
| 128 | Decryption |  8.00 KB | 512 | 121.669841 |
| 128 | Encryption |  8.00 KB | 1024 | 123.373029 |
| 128 | Decryption |  8.00 KB | 1024 | 123.386966 |
| 128 | Encryption |  8.00 KB | 2048 | 124.720722 |
| 128 | Decryption |  8.00 KB | 2048 | 124.498007 |
| 128 | Encryption |  8.00 KB | 4096 | 125.124579 |
| 128 | Decryption |  8.00 KB | 4096 | 124.729267 |
| 128 | Encryption |  16.00 KB | 256 | 116.726126 |
| 128 | Decryption |  16.00 KB | 256 | 117.040903 |
| 128 | Encryption |  16.00 KB | 512 | 120.698006 |
| 128 | Decryption |  16.00 KB | 512 | 120.473467 |
| 128 | Encryption |  16.00 KB | 1024 | 122.676281 |
| 128 | Decryption |  16.00 KB | 1024 | 122.300829 |
| 128 | Encryption |  16.00 KB | 2048 | 123.560277 |
| 128 | Decryption |  16.00 KB | 2048 | 123.326829 |
| 128 | Encryption |  16.00 KB | 4096 | 124.082906 |
| 128 | Decryption |  16.00 KB | 4096 | 123.917265 |
| 128 | Encryption |  16.00 KB | 8192 | 124.385767 |
| 128 | Decryption |  16.00 KB | 8192 | 124.044389 |
| 128 | Encryption |  32.00 KB | 256 | 117.321141 |
| 128 | Decryption |  32.00 KB | 256 | 117.286917 |
| 128 | Encryption |  32.00 KB | 512 | 121.086147 |
| 128 | Decryption |  32.00 KB | 512 | 120.832085 |
| 128 | Encryption |  32.00 KB | 1024 | 122.953964 |
| 128 | Decryption |  32.00 KB | 1024 | 122.758661 |
| 128 | Encryption |  32.00 KB | 2048 | 123.849704 |
| 128 | Decryption |  32.00 KB | 2048 | 123.749845 |
| 128 | Encryption |  32.00 KB | 4096 | 124.420599 |
| 128 | Decryption |  32.00 KB | 4096 | 124.114513 |
| 128 | Encryption |  32.00 KB | 8192 | 124.572197 |
| 128 | Decryption |  32.00 KB | 8192 | 124.615544 |
| 128 | Encryption |  32.00 KB | 16384 | 124.927920 |
| 128 | Decryption |  32.00 KB | 16384 | 124.496470 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 371.999035 |
| 512 |  16.00 KB | 370.844438 |
| 512 |  8.00 KB | 366.667786 |
| 512 |  4.00 KB | 358.504190 |
| 512 |  2.00 KB | 342.231692 |
| 512 |  1024.00 B | 314.665438 |
| 512 |  512.00 B | 270.006592 |
| 256 |  32.00 KB | 360.220521 |
| 256 |  16.00 KB | 359.091532 |
| 256 |  8.00 KB | 353.900487 |
| 256 |  4.00 KB | 349.771573 |
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
| 256 | Encryption |  1024.00 B | 512 | 104.001625 |
| 256 | Decryption |  1024.00 B | 512 | 104.444501 |
| 256 | Encryption |  2.00 KB | 256 | 106.953547 |
| 256 | Decryption |  2.00 KB | 256 | 107.616721 |
| 256 | Encryption |  2.00 KB | 512 | 110.782047 |
| 256 | Decryption |  2.00 KB | 512 | 110.630942 |
| 256 | Encryption |  2.00 KB | 1024 | 112.550663 |
| 256 | Decryption |  2.00 KB | 1024 | 112.544478 |
| 256 | Encryption |  4.00 KB | 256 | 110.072759 |
| 256 | Decryption |  4.00 KB | 256 | 110.051318 |
| 256 | Encryption |  4.00 KB | 512 | 113.844187 |
| 256 | Decryption |  4.00 KB | 512 | 113.989133 |
| 256 | Encryption |  4.00 KB | 1024 | 115.312880 |
| 256 | Decryption |  4.00 KB | 1024 | 115.671905 |
| 256 | Encryption |  4.00 KB | 2048 | 116.646969 |
| 256 | Decryption |  4.00 KB | 2048 | 116.280225 |
| 256 | Encryption |  8.00 KB | 256 | 108.448728 |
| 256 | Decryption |  8.00 KB | 256 | 108.632087 |
| 256 | Encryption |  8.00 KB | 512 | 111.767515 |
| 256 | Decryption |  8.00 KB | 512 | 112.013564 |
| 256 | Encryption |  8.00 KB | 1024 | 114.143990 |
| 256 | Decryption |  8.00 KB | 1024 | 113.761978 |
| 256 | Encryption |  8.00 KB | 2048 | 114.514968 |
| 256 | Decryption |  8.00 KB | 2048 | 114.753180 |
| 256 | Encryption |  8.00 KB | 4096 | 114.844878 |
| 256 | Decryption |  8.00 KB | 4096 | 114.773277 |
| 256 | Encryption |  16.00 KB | 256 | 108.153781 |
| 256 | Decryption |  16.00 KB | 256 | 108.117382 |
| 256 | Encryption |  16.00 KB | 512 | 111.505087 |
| 256 | Decryption |  16.00 KB | 512 | 111.195192 |
| 256 | Encryption |  16.00 KB | 1024 | 113.127686 |
| 256 | Decryption |  16.00 KB | 1024 | 112.921457 |
| 256 | Encryption |  16.00 KB | 2048 | 114.047849 |
| 256 | Decryption |  16.00 KB | 2048 | 113.503478 |
| 256 | Encryption |  16.00 KB | 4096 | 114.328779 |
| 256 | Decryption |  16.00 KB | 4096 | 114.100270 |
| 256 | Encryption |  16.00 KB | 8192 | 114.350723 |
| 256 | Decryption |  16.00 KB | 8192 | 114.528176 |
| 256 | Encryption |  32.00 KB | 256 | 108.453753 |
| 256 | Decryption |  32.00 KB | 256 | 108.520380 |
| 256 | Encryption |  32.00 KB | 512 | 111.740073 |
| 256 | Decryption |  32.00 KB | 512 | 111.641472 |
| 256 | Encryption |  32.00 KB | 1024 | 113.338003 |
| 256 | Decryption |  32.00 KB | 1024 | 113.231475 |
| 256 | Encryption |  32.00 KB | 2048 | 114.211225 |
| 256 | Decryption |  32.00 KB | 2048 | 114.006186 |
| 256 | Encryption |  32.00 KB | 4096 | 114.603881 |
| 256 | Decryption |  32.00 KB | 4096 | 114.419001 |
| 256 | Encryption |  32.00 KB | 8192 | 114.755390 |
| 256 | Decryption |  32.00 KB | 8192 | 114.686108 |
| 256 | Encryption |  32.00 KB | 16384 | 114.906696 |
| 256 | Decryption |  32.00 KB | 16384 | 114.917375 |
| 128 | Encryption |  512.00 B | 256 | 92.252252 |
| 128 | Decryption |  512.00 B | 256 | 97.181361 |
| 128 | Encryption |  1024.00 B | 256 | 106.666667 |
| 128 | Decryption |  1024.00 B | 256 | 108.494689 |
| 128 | Encryption |  1024.00 B | 512 | 112.996221 |
| 128 | Decryption |  1024.00 B | 512 | 113.418619 |
| 128 | Encryption |  2.00 KB | 256 | 114.965757 |
| 128 | Decryption |  2.00 KB | 256 | 116.088257 |
| 128 | Encryption |  2.00 KB | 512 | 119.073229 |
| 128 | Decryption |  2.00 KB | 512 | 120.064488 |
| 128 | Encryption |  2.00 KB | 1024 | 121.846740 |
| 128 | Decryption |  2.00 KB | 1024 | 122.104636 |
| 128 | Encryption |  4.00 KB | 256 | 118.448258 |
| 128 | Decryption |  4.00 KB | 256 | 118.959108 |
| 128 | Encryption |  4.00 KB | 512 | 123.655630 |
| 128 | Decryption |  4.00 KB | 512 | 123.770529 |
| 128 | Encryption |  4.00 KB | 1024 | 125.315507 |
| 128 | Decryption |  4.00 KB | 1024 | 125.738669 |
| 128 | Encryption |  4.00 KB | 2048 | 126.838634 |
| 128 | Decryption |  4.00 KB | 2048 | 126.848454 |
| 128 | Encryption |  8.00 KB | 256 | 117.191803 |
| 128 | Decryption |  8.00 KB | 256 | 117.175040 |
| 128 | Encryption |  8.00 KB | 512 | 121.258770 |
| 128 | Decryption |  8.00 KB | 512 | 121.630550 |
| 128 | Encryption |  8.00 KB | 1024 | 123.544670 |
| 128 | Decryption |  8.00 KB | 1024 | 123.009467 |
| 128 | Encryption |  8.00 KB | 2048 | 124.670897 |
| 128 | Decryption |  8.00 KB | 2048 | 124.318049 |
| 128 | Encryption |  8.00 KB | 4096 | 125.002861 |
| 128 | Decryption |  8.00 KB | 4096 | 124.843793 |
| 128 | Encryption |  16.00 KB | 256 | 116.846625 |
| 128 | Decryption |  16.00 KB | 256 | 116.962989 |
| 128 | Encryption |  16.00 KB | 512 | 120.704008 |
| 128 | Decryption |  16.00 KB | 512 | 120.522874 |
| 128 | Encryption |  16.00 KB | 1024 | 122.602612 |
| 128 | Decryption |  16.00 KB | 1024 | 122.213934 |
| 128 | Encryption |  16.00 KB | 2048 | 123.564237 |
| 128 | Decryption |  16.00 KB | 2048 | 123.353988 |
| 128 | Encryption |  16.00 KB | 4096 | 124.065758 |
| 128 | Decryption |  16.00 KB | 4096 | 123.717724 |
| 128 | Encryption |  16.00 KB | 8192 | 124.452614 |
| 128 | Decryption |  16.00 KB | 8192 | 123.868783 |
| 128 | Encryption |  32.00 KB | 256 | 117.171269 |
| 128 | Decryption |  32.00 KB | 256 | 117.298148 |
| 128 | Encryption |  32.00 KB | 512 | 121.024542 |
| 128 | Decryption |  32.00 KB | 512 | 120.969364 |
| 128 | Encryption |  32.00 KB | 1024 | 122.898742 |
| 128 | Decryption |  32.00 KB | 1024 | 122.790056 |
| 128 | Encryption |  32.00 KB | 2048 | 123.785958 |
| 128 | Decryption |  32.00 KB | 2048 | 123.795779 |
| 128 | Encryption |  32.00 KB | 4096 | 124.342108 |
| 128 | Decryption |  32.00 KB | 4096 | 124.193423 |
| 128 | Encryption |  32.00 KB | 8192 | 124.734015 |
| 128 | Decryption |  32.00 KB | 8192 | 124.516694 |
| 128 | Encryption |  32.00 KB | 16384 | 124.834994 |
| 128 | Decryption |  32.00 KB | 16384 | 124.705296 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 115.170724 |
| 256 | Decryption |  32.00 KB | 115.100535 |
| 256 | Encryption |  16.00 KB | 114.908912 |
| 256 | Decryption |  16.00 KB | 114.777900 |
| 256 | Encryption |  8.00 KB | 115.250017 |
| 256 | Decryption |  8.00 KB | 115.081233 |
| 256 | Encryption |  4.00 KB | 116.252173 |
| 256 | Decryption |  4.00 KB | 117.412679 |
| 256 | Encryption |  2.00 KB | 114.515768 |
| 256 | Decryption |  2.00 KB | 115.252044 |
| 256 | Encryption |  1024.00 B | 109.850618 |
| 256 | Decryption |  1024.00 B | 110.514529 |
| 256 | Encryption |  512.00 B | 98.442607 |
| 256 | Decryption |  512.00 B | 101.230784 |
| 128 | Encryption |  32.00 KB | 125.013472 |
| 128 | Decryption |  32.00 KB | 124.835351 |
| 128 | Encryption |  16.00 KB | 124.628933 |
| 128 | Decryption |  16.00 KB | 124.547457 |
| 128 | Encryption |  8.00 KB | 123.686902 |
| 128 | Decryption |  8.00 KB | 124.229442 |
| 128 | Encryption |  4.00 KB | 126.866134 |
| 128 | Decryption |  4.00 KB | 127.875122 |
| 128 | Encryption |  2.00 KB | 124.750636 |
| 128 | Decryption |  2.00 KB | 125.213989 |
| 128 | Encryption |  1024.00 B | 118.067566 |
| 128 | Decryption |  1024.00 B | 119.007496 |
| 128 | Encryption |  512.00 B | 108.549319 |
| 128 | Decryption |  512.00 B | 108.907206 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 369.804422 |
| 512 |  16.00 KB | 365.538860 |
| 512 |  8.00 KB | 354.662741 |
| 512 |  4.00 KB | 338.750362 |
| 512 |  2.00 KB | 308.294445 |
| 512 |  1024.00 B | 261.141218 |
| 512 |  512.00 B | 200.371784 |
| 256 |  32.00 KB | 358.664094 |
| 256 |  16.00 KB | 355.212523 |
| 256 |  8.00 KB | 348.432649 |
| 256 |  4.00 KB | 335.840935 |
| 256 |  2.00 KB | 312.421342 |
| 256 |  1024.00 B | 274.770242 |
| 256 |  512.00 B | 221.429344 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 295.771438 |
| 256 |  16.00 KB | 294.333488 |
| 256 |  8.00 KB | 293.170858 |
| 256 |  4.00 KB | 273.239718 |
| 256 |  2.00 KB | 237.318578 |
| 256 |  1024.00 B | 188.321839 |
| 256 |  512.00 B | 132.857606 |
| 128 |  32.00 KB | 298.142297 |
| 128 |  16.00 KB | 303.882928 |
| 128 |  8.00 KB | 293.493838 |
| 128 |  4.00 KB | 274.025757 |
| 128 |  2.00 KB | 239.175499 |
| 128 |  1024.00 B | 188.834079 |
| 128 |  512.00 B | 134.330316 |

### AES CMAC STREAM

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      246.382424      |
| 256        |      32.00 KB     |        1024        |      269.854276      |
| 256        |      32.00 KB     |        2048        |      283.102656      |
| 256        |      32.00 KB     |        4096        |      292.353469      |
| 256        |      32.00 KB     |        8192        |      294.485585      |
| 256        |      32.00 KB     |        16384        |      296.270846      |
| 256        |      16.00 KB     |        512        |      243.912969      |
| 256        |      16.00 KB     |        1024        |      270.684642      |
| 256        |      16.00 KB     |        2048        |      285.658244      |
| 256        |      16.00 KB     |        4096        |      292.487861      |
| 256        |      16.00 KB     |        8192        |      300.323530      |
| 256        |      8.00 KB     |        512        |      241.331566      |
| 256        |      8.00 KB     |        1024        |      267.195055      |
| 256        |      8.00 KB     |        2048        |      281.606380      |
| 256        |      8.00 KB     |        4096        |      289.280859      |
| 256        |      4.00 KB     |        512        |      227.498681      |
| 256        |      4.00 KB     |        1024        |      249.478477      |
| 256        |      4.00 KB     |        2048        |      261.624936      |
| 256        |      2.00 KB     |        512        |      200.695771      |
| 256        |      2.00 KB     |        1024        |      218.249634      |
| 256        |      1024.00 B     |        512        |      162.675245      |
| 128        |      32.00 KB     |        512        |      245.593457      |
| 128        |      32.00 KB     |        1024        |      269.383963      |
| 128        |      32.00 KB     |        2048        |      284.024079      |
| 128        |      32.00 KB     |        4096        |      293.731274      |
| 128        |      32.00 KB     |        8192        |      293.418938      |
| 128        |      32.00 KB     |        16384        |      298.791358      |
| 128        |      16.00 KB     |        512        |      247.613074      |
| 128        |      16.00 KB     |        1024        |      275.907363      |
| 128        |      16.00 KB     |        2048        |      289.256600      |
| 128        |      16.00 KB     |        4096        |      298.118563      |
| 128        |      16.00 KB     |        8192        |      301.332953      |
| 128        |      8.00 KB     |        512        |      243.073431      |
| 128        |      8.00 KB     |        1024        |      266.983884      |
| 128        |      8.00 KB     |        2048        |      281.899518      |
| 128        |      8.00 KB     |        4096        |      289.513443      |
| 128        |      4.00 KB     |        512        |      228.218022      |
| 128        |      4.00 KB     |        1024        |      249.664757      |
| 128        |      4.00 KB     |        2048        |      262.353883      |
| 128        |      2.00 KB     |        512        |      201.421160      |
| 128        |      2.00 KB     |        1024        |      217.837579      |
| 128        |      1024.00 B     |        512        |      164.590533      |

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
sincos sin  	|0.0000001790	| 451			| 79.779999 		| 467			| 275.239990		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.033997 		| 594			| 428.753998		|
acos 		|0.0000004770	| 76			| 76.003998 		| 528			| 383.145996		|
atan 		|0.0000005360	| 80			| 80.092003 		| 494			| 370.838013		|
atan2 		|0.0000007150	| 117			| 104.574005 		| 773			| 480.040009		|

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
|        2 |      916 |        17.467249 |
|        4 |     1830 |        17.486339 |
|        8 |     3658 |        17.495899 |
|       16 |     7304 |        17.524644 |
|       32 |    14618 |        17.512656 |
|       64 |    29241 |        17.509661 |
|      128 |    58482 |        17.509661 |
|      256 |   116955 |        17.511009 |
|      512 |   233923 |        17.510035 |

#### DMA read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      461 |        17.353579 |
|        2 |       25 |       640.000000 |
|        4 |       37 |       864.864865 |
|        8 |       62 |      1032.258065 |
|       16 |      115 |      1113.043478 |
|       32 |      219 |      1168.949772 |
|       64 |      425 |      1204.705882 |
|      128 |      841 |      1217.598098 |
|      256 |     1668 |      1227.817746 |
|      512 |     3328 |      1230.769231 |
