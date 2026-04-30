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
SBL : System Init                       |   434
SBL : Drivers_open                      |   94
SBL : LoadHsmRtFw                       |   6298
SBL : Board_driversOpen                 |   4865
SBL : CPU Load                          |   4181
SBL : SBL End                           |   9
SBL : Total time taken                  |   15884

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 356 KB
- Size of hello_world                 : 5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   434
SBL : Drivers_open                      |   94
SBL : LoadHsmRtFw                       |   6298
SBL : Board_driversOpen                 |   4861
SBL : CPU Load                          |   3519
SBL : SBL End                           |   5
SBL : Total time taken                  |   15214

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
 r5f0-0	| r5f0-1	| 4	| 9.793
 r5f0-0	| r5f0-1	| 32	| 11.903
 r5f0-0	| r5f0-1	| 64	| 14.051
 r5f0-0	| r5f0-1	| 112	| 17.435

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
- End tick         : 8186143
- Total ticks      : 8186087
- Total time (secs): 8.186087
- Iterations/Sec   : 1832.377301
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1832.377301 
CoreMark/MHz :4.580943 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146103112
- USER cycle count:                          146103105

BENCHMARK Using clock 500000000
- Usertime in sec:                           0.292206
- Microseconds for one run through Dhrystone:   0.6 
- Dhrystones per Second:                     1711120.4 

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
| 256 |  32.00 KB | 245.631197 |
| 256 |  16.00 KB | 246.040178 |
| 256 |  8.00 KB | 243.649991 |
| 256 |  4.00 KB | 228.412101 |
| 256 |  2.00 KB | 202.136847 |
| 256 |  1024.00 B | 164.703044 |
| 256 |  512.00 B | 119.920365 |
| 128 |  32.00 KB | 246.922697 |
| 128 |  16.00 KB | 246.565962 |
| 128 |  8.00 KB | 243.143773 |
| 128 |  4.00 KB | 228.718206 |
| 128 |  2.00 KB | 203.194761 |
| 128 |  1024.00 B | 163.330409 |
| 128 |  512.00 B | 121.528602 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 114.912741 |
| 256 | Decryption |  32.00 KB | 115.094066 |
| 256 | Encryption |  16.00 KB | 114.731282 |
| 256 | Decryption |  16.00 KB | 114.793380 |
| 256 | Encryption |  8.00 KB | 115.281643 |
| 256 | Decryption |  8.00 KB | 115.156861 |
| 256 | Encryption |  4.00 KB | 116.817466 |
| 256 | Decryption |  4.00 KB | 117.699458 |
| 256 | Encryption |  2.00 KB | 115.044869 |
| 256 | Decryption |  2.00 KB | 115.955158 |
| 256 | Encryption |  1024.00 B | 110.574198 |
| 256 | Decryption |  1024.00 B | 111.189533 |
| 256 | Encryption |  512.00 B | 102.548696 |
| 256 | Decryption |  512.00 B | 102.976669 |
| 128 | Encryption |  32.00 KB | 125.134613 |
| 128 | Decryption |  32.00 KB | 124.869840 |
| 128 | Encryption |  16.00 KB | 125.176440 |
| 128 | Decryption |  16.00 KB | 124.775575 |
| 128 | Encryption |  8.00 KB | 124.535860 |
| 128 | Decryption |  8.00 KB | 125.371124 |
| 128 | Encryption |  4.00 KB | 126.530076 |
| 128 | Decryption |  4.00 KB | 127.898081 |
| 128 | Encryption |  2.00 KB | 125.463289 |
| 128 | Decryption |  2.00 KB | 125.918411 |
| 128 | Encryption |  1024.00 B | 119.776589 |
| 128 | Decryption |  1024.00 B | 120.598281 |
| 128 | Encryption |  512.00 B | 109.448482 |
| 128 | Decryption |  512.00 B | 110.321051 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 85.209070 |
| 256 | Decryption |  512.00 B | 256 | 91.135635 |
| 256 | Encryption |  1024.00 B | 256 | 100.318393 |
| 256 | Decryption |  1024.00 B | 256 | 102.073365 |
| 256 | Encryption |  1024.00 B | 512 | 105.730511 |
| 256 | Decryption |  1024.00 B | 512 | 105.719595 |
| 256 | Encryption |  2.00 KB | 256 | 107.184446 |
| 256 | Decryption |  2.00 KB | 256 | 107.833458 |
| 256 | Encryption |  2.00 KB | 512 | 111.401218 |
| 256 | Decryption |  2.00 KB | 512 | 111.237847 |
| 256 | Encryption |  2.00 KB | 1024 | 113.188256 |
| 256 | Decryption |  2.00 KB | 1024 | 112.965057 |
| 256 | Encryption |  4.00 KB | 256 | 109.196820 |
| 256 | Decryption |  4.00 KB | 256 | 110.624219 |
| 256 | Encryption |  4.00 KB | 512 | 114.278540 |
| 256 | Decryption |  4.00 KB | 512 | 113.821251 |
| 256 | Encryption |  4.00 KB | 1024 | 115.955979 |
| 256 | Decryption |  4.00 KB | 1024 | 116.003597 |
| 256 | Encryption |  4.00 KB | 2048 | 116.664412 |
| 256 | Decryption |  4.00 KB | 2048 | 116.864127 |
| 256 | Encryption |  8.00 KB | 256 | 108.654060 |
| 256 | Decryption |  8.00 KB | 256 | 108.862702 |
| 256 | Encryption |  8.00 KB | 512 | 112.298787 |
| 256 | Decryption |  8.00 KB | 512 | 112.376197 |
| 256 | Encryption |  8.00 KB | 1024 | 113.832718 |
| 256 | Decryption |  8.00 KB | 1024 | 113.915030 |
| 256 | Encryption |  8.00 KB | 2048 | 114.665239 |
| 256 | Decryption |  8.00 KB | 2048 | 114.979070 |
| 256 | Encryption |  8.00 KB | 4096 | 115.461186 |
| 256 | Decryption |  8.00 KB | 4096 | 115.050928 |
| 256 | Encryption |  16.00 KB | 256 | 108.186454 |
| 256 | Decryption |  16.00 KB | 256 | 108.361222 |
| 256 | Encryption |  16.00 KB | 512 | 111.648509 |
| 256 | Decryption |  16.00 KB | 512 | 111.436262 |
| 256 | Encryption |  16.00 KB | 1024 | 113.222085 |
| 256 | Decryption |  16.00 KB | 1024 | 112.981610 |
| 256 | Encryption |  16.00 KB | 2048 | 113.929486 |
| 256 | Decryption |  16.00 KB | 2048 | 113.734535 |
| 256 | Encryption |  16.00 KB | 4096 | 114.410811 |
| 256 | Decryption |  16.00 KB | 4096 | 114.139417 |
| 256 | Encryption |  16.00 KB | 8192 | 114.645381 |
| 256 | Decryption |  16.00 KB | 8192 | 114.380260 |
| 256 | Encryption |  32.00 KB | 256 | 108.528018 |
| 256 | Decryption |  32.00 KB | 256 | 108.702629 |
| 256 | Encryption |  32.00 KB | 512 | 111.906931 |
| 256 | Decryption |  32.00 KB | 512 | 111.718835 |
| 256 | Encryption |  32.00 KB | 1024 | 113.393009 |
| 256 | Decryption |  32.00 KB | 1024 | 113.314389 |
| 256 | Encryption |  32.00 KB | 2048 | 114.220282 |
| 256 | Decryption |  32.00 KB | 2048 | 113.994585 |
| 256 | Encryption |  32.00 KB | 4096 | 114.624025 |
| 256 | Decryption |  32.00 KB | 4096 | 114.472863 |
| 256 | Encryption |  32.00 KB | 8192 | 114.875577 |
| 256 | Decryption |  32.00 KB | 8192 | 114.725157 |
| 256 | Encryption |  32.00 KB | 16384 | 114.911431 |
| 256 | Decryption |  32.00 KB | 16384 | 114.983205 |
| 128 | Encryption |  512.00 B | 256 | 94.338754 |
| 128 | Decryption |  512.00 B | 256 | 97.761230 |
| 128 | Encryption |  1024.00 B | 256 | 108.270995 |
| 128 | Decryption |  1024.00 B | 256 | 109.906622 |
| 128 | Encryption |  1024.00 B | 512 | 114.028006 |
| 128 | Decryption |  1024.00 B | 512 | 114.116959 |
| 128 | Encryption |  2.00 KB | 256 | 115.580513 |
| 128 | Decryption |  2.00 KB | 256 | 117.026900 |
| 128 | Encryption |  2.00 KB | 512 | 120.849130 |
| 128 | Decryption |  2.00 KB | 512 | 120.881229 |
| 128 | Encryption |  2.00 KB | 1024 | 121.154756 |
| 128 | Decryption |  2.00 KB | 1024 | 122.266832 |
| 128 | Encryption |  4.00 KB | 256 | 119.935289 |
| 128 | Decryption |  4.00 KB | 256 | 120.019632 |
| 128 | Encryption |  4.00 KB | 512 | 123.711652 |
| 128 | Decryption |  4.00 KB | 512 | 124.148487 |
| 128 | Encryption |  4.00 KB | 1024 | 126.205515 |
| 128 | Decryption |  4.00 KB | 1024 | 125.755075 |
| 128 | Encryption |  4.00 KB | 2048 | 127.152647 |
| 128 | Decryption |  4.00 KB | 2048 | 127.077694 |
| 128 | Encryption |  8.00 KB | 256 | 117.768409 |
| 128 | Decryption |  8.00 KB | 256 | 117.769256 |
| 128 | Encryption |  8.00 KB | 512 | 121.973281 |
| 128 | Decryption |  8.00 KB | 512 | 121.850818 |
| 128 | Encryption |  8.00 KB | 1024 | 123.912345 |
| 128 | Decryption |  8.00 KB | 1024 | 123.964847 |
| 128 | Encryption |  8.00 KB | 2048 | 124.639125 |
| 128 | Decryption |  8.00 KB | 2048 | 124.402531 |
| 128 | Encryption |  8.00 KB | 4096 | 125.500289 |
| 128 | Decryption |  8.00 KB | 4096 | 125.143693 |
| 128 | Encryption |  16.00 KB | 256 | 116.952344 |
| 128 | Decryption |  16.00 KB | 256 | 117.231424 |
| 128 | Encryption |  16.00 KB | 512 | 120.951504 |
| 128 | Decryption |  16.00 KB | 512 | 120.658896 |
| 128 | Encryption |  16.00 KB | 1024 | 122.655617 |
| 128 | Decryption |  16.00 KB | 1024 | 122.549194 |
| 128 | Encryption |  16.00 KB | 2048 | 123.762816 |
| 128 | Decryption |  16.00 KB | 2048 | 123.447163 |
| 128 | Encryption |  16.00 KB | 4096 | 124.139316 |
| 128 | Decryption |  16.00 KB | 4096 | 123.776841 |
| 128 | Encryption |  16.00 KB | 8192 | 124.440562 |
| 128 | Decryption |  16.00 KB | 8192 | 124.160953 |
| 128 | Encryption |  32.00 KB | 256 | 117.440136 |
| 128 | Decryption |  32.00 KB | 256 | 117.513841 |
| 128 | Encryption |  32.00 KB | 512 | 121.095096 |
| 128 | Decryption |  32.00 KB | 512 | 121.153300 |
| 128 | Encryption |  32.00 KB | 1024 | 123.071954 |
| 128 | Decryption |  32.00 KB | 1024 | 122.788331 |
| 128 | Encryption |  32.00 KB | 2048 | 123.914805 |
| 128 | Decryption |  32.00 KB | 2048 | 123.789115 |
| 128 | Encryption |  32.00 KB | 4096 | 124.436191 |
| 128 | Decryption |  32.00 KB | 4096 | 124.255469 |
| 128 | Encryption |  32.00 KB | 8192 | 124.763342 |
| 128 | Decryption |  32.00 KB | 8192 | 124.509479 |
| 128 | Encryption |  32.00 KB | 16384 | 124.865082 |
| 128 | Decryption |  32.00 KB | 16384 | 124.726063 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 115.162426 |
| 256 | Decryption |  32.00 KB | 115.056180 |
| 256 | Encryption |  16.00 KB | 114.750970 |
| 256 | Decryption |  16.00 KB | 114.718630 |
| 256 | Encryption |  8.00 KB | 115.080828 |
| 256 | Decryption |  8.00 KB | 114.680488 |
| 256 | Encryption |  4.00 KB | 116.225783 |
| 256 | Decryption |  4.00 KB | 116.974262 |
| 256 | Encryption |  2.00 KB | 114.198090 |
| 256 | Decryption |  2.00 KB | 115.224485 |
| 256 | Encryption |  1024.00 B | 110.131211 |
| 256 | Decryption |  1024.00 B | 110.499622 |
| 256 | Encryption |  512.00 B | 100.653659 |
| 256 | Decryption |  512.00 B | 101.045984 |
| 128 | Encryption |  32.00 KB | 124.846290 |
| 128 | Decryption |  32.00 KB | 124.877216 |
| 128 | Encryption |  16.00 KB | 124.729979 |
| 128 | Decryption |  16.00 KB | 124.619454 |
| 128 | Encryption |  8.00 KB | 123.949842 |
| 128 | Decryption |  8.00 KB | 124.932326 |
| 128 | Encryption |  4.00 KB | 126.424631 |
| 128 | Decryption |  4.00 KB | 127.762442 |
| 128 | Encryption |  2.00 KB | 124.929468 |
| 128 | Decryption |  2.00 KB | 125.311673 |
| 128 | Encryption |  1024.00 B | 118.084585 |
| 128 | Decryption |  1024.00 B | 119.097465 |
| 128 | Encryption |  512.00 B | 107.687454 |
| 128 | Decryption |  512.00 B | 108.005485 |

### AES CBC STREAM

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 84.870084 |
| 256 | Decryption |  512.00 B | 256 | 90.140845 |
| 256 | Encryption |  1024.00 B | 256 | 101.178272 |
| 256 | Decryption |  1024.00 B | 256 | 101.786735 |
| 256 | Encryption |  1024.00 B | 512 | 105.171263 |
| 256 | Decryption |  1024.00 B | 512 | 105.157762 |
| 256 | Encryption |  2.00 KB | 256 | 105.629626 |
| 256 | Decryption |  2.00 KB | 256 | 106.462890 |
| 256 | Encryption |  2.00 KB | 512 | 111.011735 |
| 256 | Decryption |  2.00 KB | 512 | 111.097549 |
| 256 | Encryption |  2.00 KB | 1024 | 112.655913 |
| 256 | Decryption |  2.00 KB | 1024 | 112.584693 |
| 256 | Encryption |  4.00 KB | 256 | 109.401709 |
| 256 | Decryption |  4.00 KB | 256 | 110.357463 |
| 256 | Encryption |  4.00 KB | 512 | 113.959004 |
| 256 | Decryption |  4.00 KB | 512 | 113.805439 |
| 256 | Encryption |  4.00 KB | 1024 | 115.721742 |
| 256 | Decryption |  4.00 KB | 1024 | 115.851028 |
| 256 | Encryption |  4.00 KB | 2048 | 116.457100 |
| 256 | Decryption |  4.00 KB | 2048 | 116.583058 |
| 256 | Encryption |  8.00 KB | 256 | 108.556871 |
| 256 | Decryption |  8.00 KB | 256 | 108.691542 |
| 256 | Encryption |  8.00 KB | 512 | 112.473785 |
| 256 | Decryption |  8.00 KB | 512 | 111.838086 |
| 256 | Encryption |  8.00 KB | 1024 | 113.760003 |
| 256 | Decryption |  8.00 KB | 1024 | 114.036339 |
| 256 | Encryption |  8.00 KB | 2048 | 114.813089 |
| 256 | Decryption |  8.00 KB | 2048 | 114.700560 |
| 256 | Encryption |  8.00 KB | 4096 | 115.090529 |
| 256 | Decryption |  8.00 KB | 4096 | 114.817916 |
| 256 | Encryption |  16.00 KB | 256 | 108.172883 |
| 256 | Decryption |  16.00 KB | 256 | 108.203959 |
| 256 | Encryption |  16.00 KB | 512 | 111.468863 |
| 256 | Decryption |  16.00 KB | 512 | 111.378310 |
| 256 | Encryption |  16.00 KB | 1024 | 113.020579 |
| 256 | Decryption |  16.00 KB | 1024 | 112.954348 |
| 256 | Encryption |  16.00 KB | 2048 | 113.891076 |
| 256 | Decryption |  16.00 KB | 2048 | 113.681662 |
| 256 | Encryption |  16.00 KB | 4096 | 114.372475 |
| 256 | Decryption |  16.00 KB | 4096 | 114.077032 |
| 256 | Encryption |  16.00 KB | 8192 | 114.455170 |
| 256 | Decryption |  16.00 KB | 8192 | 114.491961 |
| 256 | Encryption |  32.00 KB | 256 | 108.745197 |
| 256 | Decryption |  32.00 KB | 256 | 108.637759 |
| 256 | Encryption |  32.00 KB | 512 | 111.742836 |
| 256 | Decryption |  32.00 KB | 512 | 111.748552 |
| 256 | Encryption |  32.00 KB | 1024 | 113.443847 |
| 256 | Decryption |  32.00 KB | 1024 | 113.287749 |
| 256 | Encryption |  32.00 KB | 2048 | 114.204757 |
| 256 | Decryption |  32.00 KB | 2048 | 114.061446 |
| 256 | Encryption |  32.00 KB | 4096 | 114.604983 |
| 256 | Decryption |  32.00 KB | 4096 | 114.449073 |
| 256 | Encryption |  32.00 KB | 8192 | 114.871752 |
| 256 | Decryption |  32.00 KB | 8192 | 114.638662 |
| 256 | Encryption |  32.00 KB | 16384 | 114.838941 |
| 256 | Decryption |  32.00 KB | 16384 | 114.918685 |
| 128 | Encryption |  512.00 B | 256 | 93.554429 |
| 128 | Decryption |  512.00 B | 256 | 96.055532 |
| 128 | Encryption |  1024.00 B | 256 | 107.340339 |
| 128 | Decryption |  1024.00 B | 256 | 109.530431 |
| 128 | Encryption |  1024.00 B | 512 | 112.986870 |
| 128 | Decryption |  1024.00 B | 512 | 113.221107 |
| 128 | Encryption |  2.00 KB | 256 | 113.561695 |
| 128 | Decryption |  2.00 KB | 256 | 115.247179 |
| 128 | Encryption |  2.00 KB | 512 | 120.163112 |
| 128 | Decryption |  2.00 KB | 512 | 120.203665 |
| 128 | Encryption |  2.00 KB | 1024 | 121.897506 |
| 128 | Decryption |  2.00 KB | 1024 | 122.026425 |
| 128 | Encryption |  4.00 KB | 256 | 119.627042 |
| 128 | Decryption |  4.00 KB | 256 | 119.834409 |
| 128 | Encryption |  4.00 KB | 512 | 123.193528 |
| 128 | Decryption |  4.00 KB | 512 | 123.694840 |
| 128 | Encryption |  4.00 KB | 1024 | 125.859407 |
| 128 | Decryption |  4.00 KB | 1024 | 125.327969 |
| 128 | Encryption |  4.00 KB | 2048 | 126.939854 |
| 128 | Decryption |  4.00 KB | 2048 | 126.916254 |
| 128 | Encryption |  8.00 KB | 256 | 117.266874 |
| 128 | Decryption |  8.00 KB | 256 | 117.705800 |
| 128 | Encryption |  8.00 KB | 512 | 121.356221 |
| 128 | Decryption |  8.00 KB | 512 | 121.466885 |
| 128 | Encryption |  8.00 KB | 1024 | 123.321955 |
| 128 | Decryption |  8.00 KB | 1024 | 123.538615 |
| 128 | Encryption |  8.00 KB | 2048 | 124.701736 |
| 128 | Decryption |  8.00 KB | 2048 | 124.330785 |
| 128 | Encryption |  8.00 KB | 4096 | 125.149428 |
| 128 | Decryption |  8.00 KB | 4096 | 124.850928 |
| 128 | Encryption |  16.00 KB | 256 | 116.863919 |
| 128 | Decryption |  16.00 KB | 256 | 116.978229 |
| 128 | Encryption |  16.00 KB | 512 | 120.844896 |
| 128 | Decryption |  16.00 KB | 512 | 120.652232 |
| 128 | Encryption |  16.00 KB | 1024 | 122.650108 |
| 128 | Decryption |  16.00 KB | 1024 | 122.258849 |
| 128 | Encryption |  16.00 KB | 2048 | 123.590102 |
| 128 | Decryption |  16.00 KB | 2048 | 123.282286 |
| 128 | Encryption |  16.00 KB | 4096 | 123.941169 |
| 128 | Decryption |  16.00 KB | 4096 | 123.846077 |
| 128 | Encryption |  16.00 KB | 8192 | 124.341872 |
| 128 | Decryption |  16.00 KB | 8192 | 123.805836 |
| 128 | Encryption |  32.00 KB | 256 | 117.152104 |
| 128 | Decryption |  32.00 KB | 256 | 117.464554 |
| 128 | Encryption |  32.00 KB | 512 | 121.086818 |
| 128 | Decryption |  32.00 KB | 512 | 120.997617 |
| 128 | Encryption |  32.00 KB | 1024 | 122.908768 |
| 128 | Decryption |  32.00 KB | 1024 | 122.763950 |
| 128 | Encryption |  32.00 KB | 2048 | 123.932614 |
| 128 | Decryption |  32.00 KB | 2048 | 123.711768 |
| 128 | Encryption |  32.00 KB | 4096 | 124.360630 |
| 128 | Decryption |  32.00 KB | 4096 | 124.286574 |
| 128 | Encryption |  32.00 KB | 8192 | 124.686434 |
| 128 | Decryption |  32.00 KB | 8192 | 124.367710 |
| 128 | Encryption |  32.00 KB | 16384 | 124.741969 |
| 128 | Decryption |  32.00 KB | 16384 | 124.747668 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 372.418653 |
| 512 |  16.00 KB | 371.016593 |
| 512 |  8.00 KB | 366.840190 |
| 512 |  4.00 KB | 358.316020 |
| 512 |  2.00 KB | 342.374723 |
| 512 |  1024.00 B | 314.134520 |
| 512 |  512.00 B | 269.793176 |
| 256 |  32.00 KB | 360.199732 |
| 256 |  16.00 KB | 358.922400 |
| 256 |  8.00 KB | 354.172071 |
| 256 |  4.00 KB | 349.607375 |
| 256 |  2.00 KB | 337.911975 |
| 256 |  1024.00 B | 316.366726 |
| 256 |  512.00 B | 280.894253 |


### AES CTR STREAM

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 84.908789 |
| 256 | Decryption |  512.00 B | 256 | 90.535343 |
| 256 | Encryption |  1024.00 B | 256 | 101.021062 |
| 256 | Decryption |  1024.00 B | 256 | 101.834817 |
| 256 | Encryption |  1024.00 B | 512 | 103.911918 |
| 256 | Decryption |  1024.00 B | 512 | 104.979881 |
| 256 | Encryption |  2.00 KB | 256 | 107.489634 |
| 256 | Decryption |  2.00 KB | 256 | 107.642174 |
| 256 | Encryption |  2.00 KB | 512 | 109.947925 |
| 256 | Decryption |  2.00 KB | 512 | 111.082485 |
| 256 | Encryption |  2.00 KB | 1024 | 112.542932 |
| 256 | Decryption |  2.00 KB | 1024 | 112.516654 |
| 256 | Encryption |  4.00 KB | 256 | 110.135653 |
| 256 | Decryption |  4.00 KB | 256 | 110.175646 |
| 256 | Encryption |  4.00 KB | 512 | 114.093118 |
| 256 | Decryption |  4.00 KB | 512 | 114.124113 |
| 256 | Encryption |  4.00 KB | 1024 | 115.459152 |
| 256 | Decryption |  4.00 KB | 1024 | 115.675172 |
| 256 | Encryption |  4.00 KB | 2048 | 116.674381 |
| 256 | Decryption |  4.00 KB | 2048 | 116.433927 |
| 256 | Encryption |  8.00 KB | 256 | 109.018804 |
| 256 | Decryption |  8.00 KB | 256 | 109.361546 |
| 256 | Encryption |  8.00 KB | 512 | 112.246855 |
| 256 | Decryption |  8.00 KB | 512 | 112.122415 |
| 256 | Encryption |  8.00 KB | 1024 | 113.776988 |
| 256 | Decryption |  8.00 KB | 1024 | 113.709868 |
| 256 | Encryption |  8.00 KB | 2048 | 114.603881 |
| 256 | Decryption |  8.00 KB | 2048 | 114.703370 |
| 256 | Encryption |  8.00 KB | 4096 | 114.844073 |
| 256 | Decryption |  8.00 KB | 4096 | 114.953658 |
| 256 | Encryption |  16.00 KB | 256 | 108.293537 |
| 256 | Decryption |  16.00 KB | 256 | 108.321460 |
| 256 | Encryption |  16.00 KB | 512 | 111.610671 |
| 256 | Decryption |  16.00 KB | 512 | 111.272221 |
| 256 | Encryption |  16.00 KB | 1024 | 113.216413 |
| 256 | Decryption |  16.00 KB | 1024 | 112.889168 |
| 256 | Encryption |  16.00 KB | 2048 | 113.905922 |
| 256 | Decryption |  16.00 KB | 2048 | 113.600671 |
| 256 | Encryption |  16.00 KB | 4096 | 114.358704 |
| 256 | Decryption |  16.00 KB | 4096 | 114.212419 |
| 256 | Encryption |  16.00 KB | 8192 | 114.489361 |
| 256 | Decryption |  16.00 KB | 8192 | 114.336757 |
| 256 | Encryption |  32.00 KB | 256 | 108.450881 |
| 256 | Decryption |  32.00 KB | 256 | 108.603464 |
| 256 | Encryption |  32.00 KB | 512 | 111.795923 |
| 256 | Decryption |  32.00 KB | 512 | 111.654215 |
| 256 | Encryption |  32.00 KB | 1024 | 113.427452 |
| 256 | Decryption |  32.00 KB | 1024 | 113.231084 |
| 256 | Encryption |  32.00 KB | 2048 | 114.202070 |
| 256 | Decryption |  32.00 KB | 2048 | 114.008368 |
| 256 | Encryption |  32.00 KB | 4096 | 114.629739 |
| 256 | Decryption |  32.00 KB | 4096 | 114.482561 |
| 256 | Encryption |  32.00 KB | 8192 | 114.831999 |
| 256 | Decryption |  32.00 KB | 8192 | 114.799614 |
| 256 | Encryption |  32.00 KB | 16384 | 114.964144 |
| 256 | Decryption |  32.00 KB | 16384 | 114.890379 |
| 128 | Encryption |  512.00 B | 256 | 93.661392 |
| 128 | Decryption |  512.00 B | 256 | 96.494534 |
| 128 | Encryption |  1024.00 B | 256 | 107.616721 |
| 128 | Decryption |  1024.00 B | 256 | 109.116096 |
| 128 | Encryption |  1024.00 B | 512 | 113.208590 |
| 128 | Decryption |  1024.00 B | 512 | 113.315075 |
| 128 | Encryption |  2.00 KB | 256 | 113.695664 |
| 128 | Decryption |  2.00 KB | 256 | 115.237452 |
| 128 | Encryption |  2.00 KB | 512 | 119.114782 |
| 128 | Decryption |  2.00 KB | 512 | 119.792352 |
| 128 | Encryption |  2.00 KB | 1024 | 121.962839 |
| 128 | Decryption |  2.00 KB | 1024 | 122.040968 |
| 128 | Encryption |  4.00 KB | 256 | 119.577276 |
| 128 | Decryption |  4.00 KB | 256 | 119.040353 |
| 128 | Encryption |  4.00 KB | 512 | 123.664963 |
| 128 | Decryption |  4.00 KB | 512 | 123.644432 |
| 128 | Encryption |  4.00 KB | 1024 | 125.748319 |
| 128 | Decryption |  4.00 KB | 1024 | 125.397032 |
| 128 | Encryption |  4.00 KB | 2048 | 126.793481 |
| 128 | Decryption |  4.00 KB | 2048 | 126.874976 |
| 128 | Encryption |  8.00 KB | 256 | 117.453080 |
| 128 | Decryption |  8.00 KB | 256 | 117.838289 |
| 128 | Encryption |  8.00 KB | 512 | 121.480844 |
| 128 | Decryption |  8.00 KB | 512 | 121.393536 |
| 128 | Encryption |  8.00 KB | 1024 | 123.738747 |
| 128 | Decryption |  8.00 KB | 1024 | 123.403230 |
| 128 | Encryption |  8.00 KB | 2048 | 124.910419 |
| 128 | Decryption |  8.00 KB | 2048 | 124.392614 |
| 128 | Encryption |  8.00 KB | 4096 | 125.259939 |
| 128 | Decryption |  8.00 KB | 4096 | 125.033388 |
| 128 | Encryption |  16.00 KB | 256 | 116.939614 |
| 128 | Decryption |  16.00 KB | 256 | 116.906654 |
| 128 | Encryption |  16.00 KB | 512 | 120.841554 |
| 128 | Decryption |  16.00 KB | 512 | 120.485206 |
| 128 | Encryption |  16.00 KB | 1024 | 122.664111 |
| 128 | Decryption |  16.00 KB | 1024 | 122.232625 |
| 128 | Encryption |  16.00 KB | 2048 | 123.728468 |
| 128 | Decryption |  16.00 KB | 2048 | 123.321955 |
| 128 | Encryption |  16.00 KB | 4096 | 124.161659 |
| 128 | Decryption |  16.00 KB | 4096 | 123.631370 |
| 128 | Encryption |  16.00 KB | 8192 | 124.501318 |
| 128 | Decryption |  16.00 KB | 8192 | 124.061061 |
| 128 | Encryption |  32.00 KB | 256 | 117.186983 |
| 128 | Decryption |  32.00 KB | 256 | 117.251978 |
| 128 | Encryption |  32.00 KB | 512 | 121.027224 |
| 128 | Decryption |  32.00 KB | 512 | 120.970481 |
| 128 | Encryption |  32.00 KB | 1024 | 122.920641 |
| 128 | Decryption |  32.00 KB | 1024 | 122.768434 |
| 128 | Encryption |  32.00 KB | 2048 | 124.002377 |
| 128 | Decryption |  32.00 KB | 2048 | 123.741317 |
| 128 | Encryption |  32.00 KB | 4096 | 124.388364 |
| 128 | Decryption |  32.00 KB | 4096 | 124.271609 |
| 128 | Encryption |  32.00 KB | 8192 | 124.679673 |
| 128 | Decryption |  32.00 KB | 8192 | 124.516694 |
| 128 | Encryption |  32.00 KB | 16384 | 124.778663 |
| 128 | Decryption |  32.00 KB | 16384 | 124.615189 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 114.984617 |
| 256 | Decryption |  32.00 KB | 115.032652 |
| 256 | Encryption |  16.00 KB | 115.132382 |
| 256 | Decryption |  16.00 KB | 114.815704 |
| 256 | Encryption |  8.00 KB | 114.518170 |
| 256 | Decryption |  8.00 KB | 114.859370 |
| 256 | Encryption |  4.00 KB | 116.179630 |
| 256 | Decryption |  4.00 KB | 117.373986 |
| 256 | Encryption |  2.00 KB | 114.343141 |
| 256 | Decryption |  2.00 KB | 115.273126 |
| 256 | Encryption |  1024.00 B | 110.069062 |
| 256 | Decryption |  1024.00 B | 110.267593 |
| 256 | Encryption |  512.00 B | 98.433144 |
| 256 | Decryption |  512.00 B | 101.541970 |
| 128 | Encryption |  32.00 KB | 124.957933 |
| 128 | Decryption |  32.00 KB | 124.913157 |
| 128 | Encryption |  16.00 KB | 124.976878 |
| 128 | Decryption |  16.00 KB | 124.387184 |
| 128 | Encryption |  8.00 KB | 124.736389 |
| 128 | Decryption |  8.00 KB | 124.154132 |
| 128 | Encryption |  4.00 KB | 125.883582 |
| 128 | Decryption |  4.00 KB | 127.312710 |
| 128 | Encryption |  2.00 KB | 124.935184 |
| 128 | Decryption |  2.00 KB | 125.325093 |
| 128 | Encryption |  1024.00 B | 118.265288 |
| 128 | Decryption |  1024.00 B | 119.021329 |
| 128 | Encryption |  512.00 B | 108.342591 |
| 128 | Decryption |  512.00 B | 109.162625 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 369.585446 |
| 512 |  16.00 KB | 365.296396 |
| 512 |  8.00 KB | 354.513096 |
| 512 |  4.00 KB | 338.624339 |
| 512 |  2.00 KB | 308.957194 |
| 512 |  1024.00 B | 260.692464 |
| 512 |  512.00 B | 199.941423 |
| 256 |  32.00 KB | 358.789763 |
| 256 |  16.00 KB | 355.114360 |
| 256 |  8.00 KB | 348.110612 |
| 256 |  4.00 KB | 335.284247 |
| 256 |  2.00 KB | 311.258027 |
| 256 |  1024.00 B | 272.648605 |
| 256 |  512.00 B | 219.813245 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 299.412469 |
| 256 |  16.00 KB | 301.119734 |
| 256 |  8.00 KB | 294.739872 |
| 256 |  4.00 KB | 274.117450 |
| 256 |  2.00 KB | 238.070328 |
| 256 |  1024.00 B | 188.807965 |
| 256 |  512.00 B | 133.177266 |
| 128 |  32.00 KB | 299.165763 |
| 128 |  16.00 KB | 290.529937 |
| 128 |  8.00 KB | 294.702761 |
| 128 |  4.00 KB | 274.655089 |
| 128 |  2.00 KB | 239.357195 |
| 128 |  1024.00 B | 189.963825 |
| 128 |  512.00 B | 135.163675 |

### AES CMAC STREAM

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 500MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      245.776744      |
| 256        |      32.00 KB     |        1024        |      270.111200      |
| 256        |      32.00 KB     |        2048        |      283.305811      |
| 256        |      32.00 KB     |        4096        |      293.235135      |
| 256        |      32.00 KB     |        8192        |      293.933495      |
| 256        |      32.00 KB     |        16384        |      299.665062      |
| 256        |      16.00 KB     |        512        |      243.041880      |
| 256        |      16.00 KB     |        1024        |      269.217971      |
| 256        |      16.00 KB     |        2048        |      287.511735      |
| 256        |      16.00 KB     |        4096        |      296.321751      |
| 256        |      16.00 KB     |        8192        |      300.822562      |
| 256        |      8.00 KB     |        512        |      237.752496      |
| 256        |      8.00 KB     |        1024        |      265.113269      |
| 256        |      8.00 KB     |        2048        |      280.264801      |
| 256        |      8.00 KB     |        4096        |      288.303507      |
| 256        |      4.00 KB     |        512        |      224.293948      |
| 256        |      4.00 KB     |        1024        |      246.925488      |
| 256        |      4.00 KB     |        2048        |      259.857256      |
| 256        |      2.00 KB     |        512        |      198.574684      |
| 256        |      2.00 KB     |        1024        |      216.845783      |
| 256        |      1024.00 B     |        512        |      161.405998      |
| 128        |      32.00 KB     |        512        |      241.993671      |
| 128        |      32.00 KB     |        1024        |      267.892983      |
| 128        |      32.00 KB     |        2048        |      282.530253      |
| 128        |      32.00 KB     |        4096        |      288.746822      |
| 128        |      32.00 KB     |        8192        |      294.425388      |
| 128        |      32.00 KB     |        16384        |      298.008080      |
| 128        |      16.00 KB     |        512        |      241.745514      |
| 128        |      16.00 KB     |        1024        |      271.078756      |
| 128        |      16.00 KB     |        2048        |      288.195742      |
| 128        |      16.00 KB     |        4096        |      296.804873      |
| 128        |      16.00 KB     |        8192        |      300.691440      |
| 128        |      8.00 KB     |        512        |      238.992335      |
| 128        |      8.00 KB     |        1024        |      264.375328      |
| 128        |      8.00 KB     |        2048        |      280.216867      |
| 128        |      8.00 KB     |        4096        |      288.521819      |
| 128        |      4.00 KB     |        512        |      224.506016      |
| 128        |      4.00 KB     |        1024        |      247.675772      |
| 128        |      4.00 KB     |        2048        |      260.908338      |
| 128        |      2.00 KB     |        512        |      198.521750      |
| 128        |      2.00 KB     |        1024        |      217.883930      |
| 128        |      1024.00 B     |        512        |      162.102264      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    39   
    1024      |      TCMA     |     TCMA           |    37   
    1024      |      TCMB     |     TCMB           |    36   
    1024      |      OCRAM    |     TCMA           |    37   
    1024      |      TCMA     |     OCRAM          |    36   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 

Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.122002 		| 504			| 276.381989		|
cos  		|0.0000002870	| 66			| 66.092003 		| 504			| 277.592010		|
sincos sin  	|0.0000001790	| 422			| 79.758003 		| 467			| 275.152008		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.033997 		| 593			| 428.773987		|
acos 		|0.0000004770	| 76			| 76.003998 		| 528			| 383.144012		|
atan 		|0.0000005360	| 80			| 80.017998 		| 495			| 370.929993		|
atan2 		|0.0000007150	| 117			| 104.574005 		| 878			| 479.947998		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### XIP Benchmark

Comparing data from \ref EXAMPLES_OPTIFLASH_XIP_BENCHMARK and \ref EXAMPLES_OPTIFLASH_OCRAM_BENCHMARK, execution time of a code which throws ~3 Million I-Cache Miss per seconds is 2.2 times slower when it runs from OCRAM. 

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      119 |        67.226891 |
|        2 |      234 |        68.376068 |
|        4 |      466 |        68.669528 |
|        8 |      927 |        69.039914 |
|       16 |     1845 |        69.376694 |
|       32 |     3704 |        69.114471 |
|       64 |     7403 |        69.161151 |
|      128 |    14805 |        69.165822 |
|      256 |    29599 |        69.191527 |
|      512 |    59212 |        69.175167 |

#### DMA read

CPU with operating speed  : R5F with 500MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      120 |        66.666667 |
|        2 |       25 |       640.000000 |
|        4 |       36 |       888.888889 |
|        8 |       62 |      1032.258065 |
|       16 |      114 |      1122.807018 |
|       32 |      218 |      1174.311927 |
|       64 |      426 |      1201.877934 |
|      128 |      840 |      1219.047619 |
|      256 |     1670 |      1226.347305 |
|      512 |     3328 |      1230.769231 |
