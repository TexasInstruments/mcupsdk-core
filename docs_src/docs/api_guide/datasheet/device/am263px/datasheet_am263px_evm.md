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
- Size of ipc_notify_echo          : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   653
SBL : Drivers_open                      |   116
SBL : LoadHsmRtFw                       |   8387
SBL : Board_driversOpen                 |   3398
SBL : CPU Load                          |   5946
SBL : SBL End                           |   16
SBL : Total time taken                  |   18519

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI MULTICORE ELF performance

- Software/Application used           : sbl_ospi_multicore_elf and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 290 KB
- Size of ipc_notify_echo             : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   642
SBL : Drivers_open                      |   117
SBL : LoadHsmRtFw                       |   8393
SBL : Board_driversOpen                 |   3445
SBL : CPU Load                          |   5953
SBL : SBL End                           |   17
SBL : Total time taken                  |   18566

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and ipc_notify_echo
- Size of sbl_ospi mcelf image        : 290 KB
- Size of ipc_notify_echo             : 132 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   577
SBL : Drivers_open                      |   111
SBL : LoadHsmRtFw                       |   8387
SBL : Board_driversOpen                 |   3405
SBL : CPU Load                          |   5926
SBL : SBL End                           |   18
SBL : Total time taken                  |   18424

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI Fastboot performance

Please check out the secure boot times using the boot time calculator tool <a href="../boottime_calculator/index.html">here</a>. 

### SBL SD performance

- Software/Application used        : sbl_sd and hello_world
- Size of sbl_sd appimage          : 282 KB
- Size of hello_world              : 24.5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   654
SBL : Drivers_open                      |   23877
SBL : LoadHsmRtFw                       |   8615
SBL : Board_driversOpen                 |   2825
SBL : File read from SD card            |   18208
SBL : CPU Load                          |   7400
SBL : SBL End                           |   5043
SBL : Total time taken                  |   66624

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL SD MULTICORE ELF performance

- Software/Application used        : sbl_sd_multicore_elf and hello_world
- Size of sbl_sd appimage          : 302 KB
- Size of hello_world              : 24.5 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   776
SBL : Drivers_open                      |   24294
SBL : LoadHsmRtFw                       |   8596
SBL : Board_driversOpen                 |   2852
SBL : File read from SD card            |   94
SBL : CPU Load                          |   10928
SBL : SBL End                           |   15
SBL : Total time taken                  |   66458

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
 r5f0-0	| r5f0-1	|  3.63
 r5f0-0	| r5f1-0	|  1.87
 r5f0-0	| r5f1-1	|  1.93

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 10.211
 r5f0-0	| r5f1-0	| 4	| 10.300
 r5f0-0	| r5f1-1	| 4	| 10.280
 r5f0-0	| r5f0-1	| 32	| 13.337
 r5f0-0	| r5f0-1	| 64	| 16.136
 r5f0-0	| r5f0-1	| 112	| 20.461

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
 400		| 08			| 10.85 Mbps / 294.94 us 	| 22.92 Mbps / 139.62 us 	|  0.91 Mbps / 3526.69 us
 200		| 16			| 22.53 Mbps / 142.02 us 	| 30.00 Mbps / 106.67 us 	|  0.95 Mbps / 3370.81 us
 100		| 32			| 37.34 Mbps / 85.71 us 	| 35.34 Mbps / 90.56 us 	|  0.97 Mbps / 3293.08 us

- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### ARM R5F

### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 72
- End tick         : 10239729
- Total ticks      : 10239657
- Total time (secs): 10.239657
- Iterations/Sec   : 1464.892818
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1464.892818 
CoreMark/MHz :3.662232 / STACK

### DTHE

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 200.405943 |
| 256 |  16.00 KB | 202.025309 |
| 256 |  8.00 KB | 198.920953 |
| 256 |  4.00 KB | 189.180763 |
| 256 |  2.00 KB | 170.227798 |
| 256 |  1024.00 B | 142.308695 |
| 256 |  512.00 B | 106.113990 |
| 128 |  32.00 KB | 200.786236 |
| 128 |  16.00 KB | 201.977055 |
| 128 |  8.00 KB | 198.987392 |
| 128 |  4.00 KB | 188.197456 |
| 128 |  2.00 KB | 170.640004 |
| 128 |  1024.00 B | 142.506741 |
| 128 |  512.00 B | 107.647832 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.285673 |
| 256 | Decryption |  32.00 KB | 88.319805 |
| 256 | Encryption |  16.00 KB | 88.303814 |
| 256 | Decryption |  16.00 KB | 88.023169 |
| 256 | Encryption |  8.00 KB | 87.940046 |
| 256 | Decryption |  8.00 KB | 88.622342 |
| 256 | Encryption |  4.00 KB | 89.724949 |
| 256 | Decryption |  4.00 KB | 90.578764 |
| 256 | Encryption |  2.00 KB | 89.510490 |
| 256 | Decryption |  2.00 KB | 88.872013 |
| 256 | Encryption |  1024.00 B | 86.729130 |
| 256 | Decryption |  1024.00 B | 86.548163 |
| 256 | Encryption |  512.00 B | 81.870877 |
| 256 | Decryption |  512.00 B | 81.480008 |
| 128 | Encryption |  32.00 KB | 95.935941 |
| 128 | Decryption |  32.00 KB | 95.885498 |
| 128 | Encryption |  16.00 KB | 95.933835 |
| 128 | Decryption |  16.00 KB | 95.613496 |
| 128 | Encryption |  8.00 KB | 96.410498 |
| 128 | Decryption |  8.00 KB | 96.786032 |
| 128 | Encryption |  4.00 KB | 97.845593 |
| 128 | Decryption |  4.00 KB | 98.643086 |
| 128 | Encryption |  2.00 KB | 97.120586 |
| 128 | Decryption |  2.00 KB | 97.462895 |
| 128 | Encryption |  1024.00 B | 93.961117 |
| 128 | Decryption |  1024.00 B | 94.006943 |
| 128 | Encryption |  512.00 B | 88.185586 |
| 128 | Decryption |  512.00 B | 88.171349 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 68.414899 |
| 256 | Decryption |  512.00 B | 256 | 73.159187 |
| 256 | Encryption |  1024.00 B | 256 | 78.439258 |
| 256 | Decryption |  1024.00 B | 256 | 79.929749 |
| 256 | Encryption |  1024.00 B | 512 | 82.766285 |
| 256 | Decryption |  1024.00 B | 512 | 82.837424 |
| 256 | Encryption |  2.00 KB | 256 | 82.967464 |
| 256 | Decryption |  2.00 KB | 256 | 83.890375 |
| 256 | Encryption |  2.00 KB | 512 | 86.233848 |
| 256 | Decryption |  2.00 KB | 512 | 86.058330 |
| 256 | Encryption |  2.00 KB | 1024 | 86.922383 |
| 256 | Decryption |  2.00 KB | 1024 | 87.263818 |
| 256 | Encryption |  4.00 KB | 256 | 84.067397 |
| 256 | Decryption |  4.00 KB | 256 | 84.909339 |
| 256 | Encryption |  4.00 KB | 512 | 87.781030 |
| 256 | Decryption |  4.00 KB | 512 | 87.994951 |
| 256 | Encryption |  4.00 KB | 1024 | 89.096743 |
| 256 | Decryption |  4.00 KB | 1024 | 89.220465 |
| 256 | Encryption |  4.00 KB | 2048 | 89.921928 |
| 256 | Decryption |  4.00 KB | 2048 | 89.637203 |
| 256 | Encryption |  8.00 KB | 256 | 83.494923 |
| 256 | Decryption |  8.00 KB | 256 | 83.847979 |
| 256 | Encryption |  8.00 KB | 512 | 86.515886 |
| 256 | Decryption |  8.00 KB | 512 | 86.396129 |
| 256 | Encryption |  8.00 KB | 1024 | 87.750177 |
| 256 | Decryption |  8.00 KB | 1024 | 87.619366 |
| 256 | Encryption |  8.00 KB | 2048 | 88.527170 |
| 256 | Decryption |  8.00 KB | 2048 | 88.174611 |
| 256 | Encryption |  8.00 KB | 4096 | 88.536140 |
| 256 | Decryption |  8.00 KB | 4096 | 88.378538 |
| 256 | Encryption |  16.00 KB | 256 | 83.269222 |
| 256 | Decryption |  16.00 KB | 256 | 83.546281 |
| 256 | Encryption |  16.00 KB | 512 | 85.833891 |
| 256 | Decryption |  16.00 KB | 512 | 85.630056 |
| 256 | Encryption |  16.00 KB | 1024 | 86.974438 |
| 256 | Decryption |  16.00 KB | 1024 | 86.783252 |
| 256 | Encryption |  16.00 KB | 2048 | 87.427816 |
| 256 | Decryption |  16.00 KB | 2048 | 87.416737 |
| 256 | Encryption |  16.00 KB | 4096 | 87.802640 |
| 256 | Decryption |  16.00 KB | 4096 | 87.703644 |
| 256 | Encryption |  16.00 KB | 8192 | 87.995099 |
| 256 | Decryption |  16.00 KB | 8192 | 87.816464 |
| 256 | Encryption |  32.00 KB | 256 | 83.512878 |
| 256 | Decryption |  32.00 KB | 256 | 83.628771 |
| 256 | Encryption |  32.00 KB | 512 | 85.910334 |
| 256 | Decryption |  32.00 KB | 512 | 85.811062 |
| 256 | Encryption |  32.00 KB | 1024 | 87.066805 |
| 256 | Decryption |  32.00 KB | 1024 | 87.039197 |
| 256 | Encryption |  32.00 KB | 2048 | 87.677904 |
| 256 | Decryption |  32.00 KB | 2048 | 87.565001 |
| 256 | Encryption |  32.00 KB | 4096 | 87.980997 |
| 256 | Decryption |  32.00 KB | 4096 | 87.862159 |
| 256 | Encryption |  32.00 KB | 8192 | 88.151706 |
| 256 | Decryption |  32.00 KB | 8192 | 87.992367 |
| 256 | Encryption |  32.00 KB | 16384 | 88.236566 |
| 256 | Decryption |  32.00 KB | 16384 | 88.077217 |
| 128 | Encryption |  512.00 B | 256 | 76.492833 |
| 128 | Decryption |  512.00 B | 256 | 78.561496 |
| 128 | Encryption |  1024.00 B | 256 | 85.793580 |
| 128 | Decryption |  1024.00 B | 256 | 85.645583 |
| 128 | Encryption |  1024.00 B | 512 | 89.564314 |
| 128 | Decryption |  1024.00 B | 512 | 89.422552 |
| 128 | Encryption |  2.00 KB | 256 | 88.583710 |
| 128 | Decryption |  2.00 KB | 256 | 89.278806 |
| 128 | Encryption |  2.00 KB | 512 | 93.385391 |
| 128 | Decryption |  2.00 KB | 512 | 93.346817 |
| 128 | Encryption |  2.00 KB | 1024 | 94.886199 |
| 128 | Decryption |  2.00 KB | 1024 | 94.701097 |
| 128 | Encryption |  4.00 KB | 256 | 91.297382 |
| 128 | Decryption |  4.00 KB | 256 | 92.076627 |
| 128 | Encryption |  4.00 KB | 512 | 95.458386 |
| 128 | Decryption |  4.00 KB | 512 | 95.191477 |
| 128 | Encryption |  4.00 KB | 1024 | 97.017069 |
| 128 | Decryption |  4.00 KB | 1024 | 97.009170 |
| 128 | Encryption |  4.00 KB | 2048 | 97.410743 |
| 128 | Decryption |  4.00 KB | 2048 | 97.624776 |
| 128 | Encryption |  8.00 KB | 256 | 90.376407 |
| 128 | Decryption |  8.00 KB | 256 | 90.914262 |
| 128 | Encryption |  8.00 KB | 512 | 93.942597 |
| 128 | Decryption |  8.00 KB | 512 | 93.822229 |
| 128 | Encryption |  8.00 KB | 1024 | 95.194588 |
| 128 | Decryption |  8.00 KB | 1024 | 95.025157 |
| 128 | Encryption |  8.00 KB | 2048 | 95.987931 |
| 128 | Decryption |  8.00 KB | 2048 | 95.824423 |
| 128 | Encryption |  8.00 KB | 4096 | 96.146356 |
| 128 | Decryption |  8.00 KB | 4096 | 96.154467 |
| 128 | Encryption |  16.00 KB | 256 | 90.291737 |
| 128 | Decryption |  16.00 KB | 256 | 90.121322 |
| 128 | Encryption |  16.00 KB | 512 | 92.961997 |
| 128 | Decryption |  16.00 KB | 512 | 92.719340 |
| 128 | Encryption |  16.00 KB | 1024 | 94.285719 |
| 128 | Decryption |  16.00 KB | 1024 | 94.108018 |
| 128 | Encryption |  16.00 KB | 2048 | 94.962679 |
| 128 | Decryption |  16.00 KB | 2048 | 94.820817 |
| 128 | Encryption |  16.00 KB | 4096 | 95.392907 |
| 128 | Decryption |  16.00 KB | 4096 | 95.096686 |
| 128 | Encryption |  16.00 KB | 8192 | 95.635123 |
| 128 | Decryption |  16.00 KB | 8192 | 95.302738 |
| 128 | Encryption |  32.00 KB | 256 | 90.300446 |
| 128 | Decryption |  32.00 KB | 256 | 90.190931 |
| 128 | Encryption |  32.00 KB | 512 | 93.120588 |
| 128 | Decryption |  32.00 KB | 512 | 92.999016 |
| 128 | Encryption |  32.00 KB | 1024 | 94.407875 |
| 128 | Decryption |  32.00 KB | 1024 | 94.411616 |
| 128 | Encryption |  32.00 KB | 2048 | 95.165472 |
| 128 | Decryption |  32.00 KB | 2048 | 95.054186 |
| 128 | Encryption |  32.00 KB | 4096 | 95.513252 |
| 128 | Decryption |  32.00 KB | 4096 | 95.408617 |
| 128 | Encryption |  32.00 KB | 8192 | 95.697354 |
| 128 | Decryption |  32.00 KB | 8192 | 95.599810 |
| 128 | Encryption |  32.00 KB | 16384 | 95.826788 |
| 128 | Decryption |  32.00 KB | 16384 | 95.646201 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.395599 |
| 256 | Decryption |  32.00 KB | 88.269769 |
| 256 | Encryption |  16.00 KB | 88.070041 |
| 256 | Decryption |  16.00 KB | 88.039872 |
| 256 | Encryption |  8.00 KB | 89.085842 |
| 256 | Decryption |  8.00 KB | 88.498479 |
| 256 | Encryption |  4.00 KB | 89.449404 |
| 256 | Decryption |  4.00 KB | 90.044242 |
| 256 | Encryption |  2.00 KB | 88.341309 |
| 256 | Decryption |  2.00 KB | 88.976987 |
| 256 | Encryption |  1024.00 B | 85.962381 |
| 256 | Decryption |  1024.00 B | 86.181684 |
| 256 | Encryption |  512.00 B | 80.495234 |
| 256 | Decryption |  512.00 B | 80.388597 |
| 128 | Encryption |  32.00 KB | 95.698489 |
| 128 | Decryption |  32.00 KB | 95.875240 |
| 128 | Encryption |  16.00 KB | 95.436839 |
| 128 | Decryption |  16.00 KB | 95.471944 |
| 128 | Encryption |  8.00 KB | 95.998828 |
| 128 | Decryption |  8.00 KB | 95.297368 |
| 128 | Encryption |  4.00 KB | 97.050113 |
| 128 | Decryption |  4.00 KB | 97.836829 |
| 128 | Encryption |  2.00 KB | 96.866501 |
| 128 | Decryption |  2.00 KB | 97.101879 |
| 128 | Encryption |  1024.00 B | 93.133242 |
| 128 | Decryption |  1024.00 B | 93.133242 |
| 128 | Encryption |  512.00 B | 86.738313 |
| 128 | Decryption |  512.00 B | 86.917772 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 67.299240 |
| 256 | Decryption |  512.00 B | 256 | 71.916425 |
| 256 | Encryption |  1024.00 B | 256 | 79.545565 |
| 256 | Decryption |  1024.00 B | 256 | 79.941449 |
| 256 | Encryption |  1024.00 B | 512 | 82.176803 |
| 256 | Decryption |  1024.00 B | 512 | 82.207727 |
| 256 | Encryption |  2.00 KB | 256 | 83.340963 |
| 256 | Decryption |  2.00 KB | 256 | 83.308122 |
| 256 | Encryption |  2.00 KB | 512 | 85.675813 |
| 256 | Decryption |  2.00 KB | 512 | 85.750922 |
| 256 | Encryption |  2.00 KB | 1024 | 86.682098 |
| 256 | Decryption |  2.00 KB | 1024 | 86.852114 |
| 256 | Encryption |  4.00 KB | 256 | 84.214855 |
| 256 | Decryption |  4.00 KB | 256 | 85.301123 |
| 256 | Encryption |  4.00 KB | 512 | 87.500918 |
| 256 | Decryption |  4.00 KB | 512 | 87.725803 |
| 256 | Encryption |  4.00 KB | 1024 | 88.914214 |
| 256 | Decryption |  4.00 KB | 1024 | 89.033801 |
| 256 | Encryption |  4.00 KB | 2048 | 89.820253 |
| 256 | Decryption |  4.00 KB | 2048 | 89.439638 |
| 256 | Encryption |  8.00 KB | 256 | 83.510350 |
| 256 | Decryption |  8.00 KB | 256 | 83.842080 |
| 256 | Encryption |  8.00 KB | 512 | 86.229877 |
| 256 | Decryption |  8.00 KB | 512 | 86.293176 |
| 256 | Encryption |  8.00 KB | 1024 | 87.679737 |
| 256 | Decryption |  8.00 KB | 1024 | 87.632840 |
| 256 | Encryption |  8.00 KB | 2048 | 88.218230 |
| 256 | Decryption |  8.00 KB | 2048 | 88.207247 |
| 256 | Encryption |  8.00 KB | 4096 | 88.445331 |
| 256 | Decryption |  8.00 KB | 4096 | 88.173425 |
| 256 | Encryption |  16.00 KB | 256 | 83.386826 |
| 256 | Decryption |  16.00 KB | 256 | 83.456645 |
| 256 | Encryption |  16.00 KB | 512 | 85.847384 |
| 256 | Decryption |  16.00 KB | 512 | 85.512737 |
| 256 | Encryption |  16.00 KB | 1024 | 86.872694 |
| 256 | Decryption |  16.00 KB | 1024 | 86.764007 |
| 256 | Encryption |  16.00 KB | 2048 | 87.501356 |
| 256 | Decryption |  16.00 KB | 2048 | 87.384246 |
| 256 | Encryption |  16.00 KB | 4096 | 87.783381 |
| 256 | Decryption |  16.00 KB | 4096 | 87.624491 |
| 256 | Encryption |  16.00 KB | 8192 | 87.953471 |
| 256 | Decryption |  16.00 KB | 8192 | 87.771478 |
| 256 | Encryption |  32.00 KB | 256 | 83.502503 |
| 256 | Decryption |  32.00 KB | 256 | 83.577179 |
| 256 | Encryption |  32.00 KB | 512 | 85.924554 |
| 256 | Decryption |  32.00 KB | 512 | 85.769299 |
| 256 | Encryption |  32.00 KB | 1024 | 87.078807 |
| 256 | Decryption |  32.00 KB | 1024 | 86.941554 |
| 256 | Encryption |  32.00 KB | 2048 | 87.620464 |
| 256 | Decryption |  32.00 KB | 2048 | 87.553522 |
| 256 | Encryption |  32.00 KB | 4096 | 87.959594 |
| 256 | Decryption |  32.00 KB | 4096 | 87.841329 |
| 256 | Encryption |  32.00 KB | 8192 | 88.121184 |
| 256 | Decryption |  32.00 KB | 8192 | 87.987420 |
| 256 | Encryption |  32.00 KB | 16384 | 88.148890 |
| 256 | Decryption |  32.00 KB | 16384 | 88.126442 |
| 128 | Encryption |  512.00 B | 256 | 75.266446 |
| 128 | Decryption |  512.00 B | 256 | 76.650292 |
| 128 | Encryption |  1024.00 B | 256 | 84.979253 |
| 128 | Decryption |  1024.00 B | 256 | 85.284472 |
| 128 | Encryption |  1024.00 B | 512 | 88.742045 |
| 128 | Decryption |  1024.00 B | 512 | 88.631630 |
| 128 | Encryption |  2.00 KB | 256 | 89.002363 |
| 128 | Decryption |  2.00 KB | 256 | 89.311656 |
| 128 | Encryption |  2.00 KB | 512 | 92.889034 |
| 128 | Decryption |  2.00 KB | 512 | 92.911421 |
| 128 | Encryption |  2.00 KB | 1024 | 94.469030 |
| 128 | Decryption |  2.00 KB | 1024 | 94.377880 |
| 128 | Encryption |  4.00 KB | 256 | 91.080412 |
| 128 | Decryption |  4.00 KB | 256 | 91.406255 |
| 128 | Encryption |  4.00 KB | 512 | 94.932932 |
| 128 | Decryption |  4.00 KB | 512 | 95.129298 |
| 128 | Encryption |  4.00 KB | 1024 | 96.828575 |
| 128 | Decryption |  4.00 KB | 1024 | 96.864354 |
| 128 | Encryption |  4.00 KB | 2048 | 97.264726 |
| 128 | Decryption |  4.00 KB | 2048 | 97.498419 |
| 128 | Encryption |  8.00 KB | 256 | 91.408168 |
| 128 | Decryption |  8.00 KB | 256 | 90.933814 |
| 128 | Encryption |  8.00 KB | 512 | 93.920047 |
| 128 | Decryption |  8.00 KB | 512 | 93.330865 |
| 128 | Encryption |  8.00 KB | 1024 | 94.810700 |
| 128 | Decryption |  8.00 KB | 1024 | 94.981431 |
| 128 | Encryption |  8.00 KB | 2048 | 95.669501 |
| 128 | Decryption |  8.00 KB | 2048 | 95.424680 |
| 128 | Encryption |  8.00 KB | 4096 | 96.048071 |
| 128 | Decryption |  8.00 KB | 4096 | 95.938750 |
| 128 | Encryption |  16.00 KB | 256 | 90.375473 |
| 128 | Decryption |  16.00 KB | 256 | 90.110479 |
| 128 | Encryption |  16.00 KB | 512 | 92.912574 |
| 128 | Decryption |  16.00 KB | 512 | 92.674434 |
| 128 | Encryption |  16.00 KB | 1024 | 94.303357 |
| 128 | Decryption |  16.00 KB | 1024 | 94.075934 |
| 128 | Encryption |  16.00 KB | 2048 | 94.830592 |
| 128 | Decryption |  16.00 KB | 2048 | 94.718377 |
| 128 | Encryption |  16.00 KB | 4096 | 95.447437 |
| 128 | Decryption |  16.00 KB | 4096 | 95.168150 |
| 128 | Encryption |  16.00 KB | 8192 | 95.442746 |
| 128 | Decryption |  16.00 KB | 8192 | 95.385270 |
| 128 | Encryption |  32.00 KB | 256 | 90.208311 |
| 128 | Decryption |  32.00 KB | 256 | 90.340279 |
| 128 | Encryption |  32.00 KB | 512 | 93.097769 |
| 128 | Decryption |  32.00 KB | 512 | 92.949966 |
| 128 | Encryption |  32.00 KB | 1024 | 94.416546 |
| 128 | Decryption |  32.00 KB | 1024 | 94.303272 |
| 128 | Encryption |  32.00 KB | 2048 | 95.154418 |
| 128 | Decryption |  32.00 KB | 2048 | 94.989003 |
| 128 | Encryption |  32.00 KB | 4096 | 95.441443 |
| 128 | Decryption |  32.00 KB | 4096 | 95.404276 |
| 128 | Encryption |  32.00 KB | 8192 | 95.662781 |
| 128 | Decryption |  32.00 KB | 8192 | 95.525869 |
| 128 | Encryption |  32.00 KB | 16384 | 95.781271 |
| 128 | Decryption |  32.00 KB | 16384 | 95.689669 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 284.609664 |
| 512 |  16.00 KB | 283.227451 |
| 512 |  8.00 KB | 279.984620 |
| 512 |  4.00 KB | 273.476882 |
| 512 |  2.00 KB | 261.850727 |
| 512 |  1024.00 B | 240.958894 |
| 512 |  512.00 B | 207.313678 |
| 256 |  32.00 KB | 275.014687 |
| 256 |  16.00 KB | 274.103119 |
| 256 |  8.00 KB | 270.482991 |
| 256 |  4.00 KB | 267.155844 |
| 256 |  2.00 KB | 254.914621 |
| 256 |  1024.00 B | 242.134043 |
| 256 |  512.00 B | 215.097808 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 67.535037 |
| 256 | Decryption |  512.00 B | 256 | 72.824251 |
| 256 | Encryption |  1024.00 B | 256 | 79.299163 |
| 256 | Decryption |  1024.00 B | 256 | 80.168322 |
| 256 | Encryption |  1024.00 B | 512 | 82.174742 |
| 256 | Decryption |  1024.00 B | 512 | 82.373052 |
| 256 | Encryption |  2.00 KB | 256 | 83.032637 |
| 256 | Decryption |  2.00 KB | 256 | 84.134850 |
| 256 | Encryption |  2.00 KB | 512 | 86.215697 |
| 256 | Decryption |  2.00 KB | 512 | 86.267902 |
| 256 | Encryption |  2.00 KB | 1024 | 87.237101 |
| 256 | Decryption |  2.00 KB | 1024 | 86.845209 |
| 256 | Encryption |  4.00 KB | 256 | 85.991143 |
| 256 | Decryption |  4.00 KB | 256 | 86.332679 |
| 256 | Encryption |  4.00 KB | 512 | 88.307978 |
| 256 | Decryption |  4.00 KB | 512 | 88.088389 |
| 256 | Encryption |  4.00 KB | 1024 | 89.349403 |
| 256 | Decryption |  4.00 KB | 1024 | 89.165839 |
| 256 | Encryption |  4.00 KB | 2048 | 89.828255 |
| 256 | Decryption |  4.00 KB | 2048 | 89.868905 |
| 256 | Encryption |  8.00 KB | 256 | 84.736975 |
| 256 | Decryption |  8.00 KB | 256 | 85.163118 |
| 256 | Encryption |  8.00 KB | 512 | 86.946024 |
| 256 | Decryption |  8.00 KB | 512 | 87.048807 |
| 256 | Encryption |  8.00 KB | 1024 | 88.008541 |
| 256 | Decryption |  8.00 KB | 1024 | 87.813375 |
| 256 | Encryption |  8.00 KB | 2048 | 88.361558 |
| 256 | Decryption |  8.00 KB | 2048 | 88.119185 |
| 256 | Encryption |  8.00 KB | 4096 | 88.411325 |
| 256 | Decryption |  8.00 KB | 4096 | 88.406853 |
| 256 | Encryption |  16.00 KB | 256 | 84.248145 |
| 256 | Decryption |  16.00 KB | 256 | 84.578951 |
| 256 | Encryption |  16.00 KB | 512 | 86.288632 |
| 256 | Decryption |  16.00 KB | 512 | 86.215414 |
| 256 | Encryption |  16.00 KB | 1024 | 87.327481 |
| 256 | Decryption |  16.00 KB | 1024 | 86.973139 |
| 256 | Encryption |  16.00 KB | 2048 | 87.609703 |
| 256 | Decryption |  16.00 KB | 2048 | 87.546797 |
| 256 | Encryption |  16.00 KB | 4096 | 87.796317 |
| 256 | Decryption |  16.00 KB | 4096 | 87.717584 |
| 256 | Encryption |  16.00 KB | 8192 | 88.029820 |
| 256 | Decryption |  16.00 KB | 8192 | 87.737254 |
| 256 | Encryption |  32.00 KB | 256 | 84.402624 |
| 256 | Decryption |  32.00 KB | 256 | 84.337997 |
| 256 | Encryption |  32.00 KB | 512 | 86.284371 |
| 256 | Decryption |  32.00 KB | 512 | 86.412790 |
| 256 | Encryption |  32.00 KB | 1024 | 87.355272 |
| 256 | Decryption |  32.00 KB | 1024 | 87.301743 |
| 256 | Encryption |  32.00 KB | 2048 | 87.791613 |
| 256 | Decryption |  32.00 KB | 2048 | 87.718611 |
| 256 | Encryption |  32.00 KB | 4096 | 88.054066 |
| 256 | Decryption |  32.00 KB | 4096 | 87.897290 |
| 256 | Encryption |  32.00 KB | 8192 | 88.077883 |
| 256 | Decryption |  32.00 KB | 8192 | 88.155930 |
| 256 | Encryption |  32.00 KB | 16384 | 88.164676 |
| 256 | Decryption |  32.00 KB | 16384 | 88.199531 |
| 128 | Encryption |  512.00 B | 256 | 75.121504 |
| 128 | Decryption |  512.00 B | 256 | 77.796771 |
| 128 | Encryption |  1024.00 B | 256 | 84.867007 |
| 128 | Decryption |  1024.00 B | 256 | 85.213502 |
| 128 | Encryption |  1024.00 B | 512 | 88.442645 |
| 128 | Decryption |  1024.00 B | 512 | 88.643618 |
| 128 | Encryption |  2.00 KB | 256 | 88.876834 |
| 128 | Decryption |  2.00 KB | 256 | 90.230202 |
| 128 | Encryption |  2.00 KB | 512 | 92.911421 |
| 128 | Decryption |  2.00 KB | 512 | 93.060506 |
| 128 | Encryption |  2.00 KB | 1024 | 94.405071 |
| 128 | Decryption |  2.00 KB | 1024 | 94.399631 |
| 128 | Encryption |  4.00 KB | 256 | 92.193852 |
| 128 | Decryption |  4.00 KB | 256 | 91.776832 |
| 128 | Encryption |  4.00 KB | 512 | 95.165904 |
| 128 | Decryption |  4.00 KB | 512 | 94.950812 |
| 128 | Encryption |  4.00 KB | 1024 | 96.742099 |
| 128 | Decryption |  4.00 KB | 1024 | 96.783531 |
| 128 | Encryption |  4.00 KB | 2048 | 97.190441 |
| 128 | Decryption |  4.00 KB | 2048 | 97.563009 |
| 128 | Encryption |  8.00 KB | 256 | 90.408823 |
| 128 | Decryption |  8.00 KB | 256 | 90.585650 |
| 128 | Encryption |  8.00 KB | 512 | 93.644262 |
| 128 | Decryption |  8.00 KB | 512 | 93.362110 |
| 128 | Encryption |  8.00 KB | 1024 | 94.906811 |
| 128 | Decryption |  8.00 KB | 1024 | 94.913683 |
| 128 | Encryption |  8.00 KB | 2048 | 95.500117 |
| 128 | Decryption |  8.00 KB | 2048 | 95.748473 |
| 128 | Encryption |  8.00 KB | 4096 | 96.058630 |
| 128 | Decryption |  8.00 KB | 4096 | 95.911020 |
| 128 | Encryption |  16.00 KB | 256 | 89.802406 |
| 128 | Decryption |  16.00 KB | 256 | 90.051356 |
| 128 | Encryption |  16.00 KB | 512 | 92.831469 |
| 128 | Decryption |  16.00 KB | 512 | 92.696226 |
| 128 | Encryption |  16.00 KB | 1024 | 94.313366 |
| 128 | Decryption |  16.00 KB | 1024 | 94.018743 |
| 128 | Encryption |  16.00 KB | 2048 | 95.027568 |
| 128 | Decryption |  16.00 KB | 2048 | 94.641265 |
| 128 | Encryption |  16.00 KB | 4096 | 95.338612 |
| 128 | Decryption |  16.00 KB | 4096 | 94.954595 |
| 128 | Encryption |  16.00 KB | 8192 | 95.442398 |
| 128 | Decryption |  16.00 KB | 8192 | 95.370522 |
| 128 | Encryption |  32.00 KB | 256 | 90.230667 |
| 128 | Decryption |  32.00 KB | 256 | 90.315690 |
| 128 | Encryption |  32.00 KB | 512 | 93.100166 |
| 128 | Decryption |  32.00 KB | 512 | 92.938761 |
| 128 | Encryption |  32.00 KB | 1024 | 94.447077 |
| 128 | Decryption |  32.00 KB | 1024 | 94.274021 |
| 128 | Encryption |  32.00 KB | 2048 | 95.079095 |
| 128 | Decryption |  32.00 KB | 2048 | 95.032563 |
| 128 | Encryption |  32.00 KB | 4096 | 95.471423 |
| 128 | Decryption |  32.00 KB | 4096 | 95.408183 |
| 128 | Encryption |  32.00 KB | 8192 | 95.653443 |
| 128 | Decryption |  32.00 KB | 8192 | 95.607917 |
| 128 | Encryption |  32.00 KB | 16384 | 95.683993 |
| 128 | Decryption |  32.00 KB | 16384 | 95.656235 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.395972 |
| 256 | Decryption |  32.00 KB | 88.235527 |
| 256 | Encryption |  16.00 KB | 87.930312 |
| 256 | Decryption |  16.00 KB | 88.040611 |
| 256 | Encryption |  8.00 KB | 88.008541 |
| 256 | Decryption |  8.00 KB | 87.818376 |
| 256 | Encryption |  4.00 KB | 89.501932 |
| 256 | Decryption |  4.00 KB | 90.448128 |
| 256 | Encryption |  2.00 KB | 88.954041 |
| 256 | Decryption |  2.00 KB | 88.696405 |
| 256 | Encryption |  1024.00 B | 86.217966 |
| 256 | Decryption |  1024.00 B | 86.281531 |
| 256 | Encryption |  512.00 B | 80.329476 |
| 256 | Decryption |  512.00 B | 80.396487 |
| 128 | Encryption |  32.00 KB | 95.748124 |
| 128 | Decryption |  32.00 KB | 95.837648 |
| 128 | Encryption |  16.00 KB | 95.360808 |
| 128 | Decryption |  16.00 KB | 95.757742 |
| 128 | Encryption |  8.00 KB | 96.050886 |
| 128 | Decryption |  8.00 KB | 95.918390 |
| 128 | Encryption |  4.00 KB | 97.163783 |
| 128 | Decryption |  4.00 KB | 98.315306 |
| 128 | Encryption |  2.00 KB | 96.136130 |
| 128 | Decryption |  2.00 KB | 97.050113 |
| 128 | Encryption |  1024.00 B | 92.953591 |
| 128 | Decryption |  1024.00 B | 93.425329 |
| 128 | Encryption |  512.00 B | 86.614506 |
| 128 | Decryption |  512.00 B | 86.894723 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 282.667802 |
| 512 |  16.00 KB | 279.151292 |
| 512 |  8.00 KB | 272.244262 |
| 512 |  4.00 KB | 259.015098 |
| 512 |  2.00 KB | 236.600599 |
| 512 |  1024.00 B | 201.711296 |
| 512 |  512.00 B | 155.667458 |
| 256 |  32.00 KB | 273.570350 |
| 256 |  16.00 KB | 270.649150 |
| 256 |  8.00 KB | 265.185680 |
| 256 |  4.00 KB | 256.862899 |
| 256 |  2.00 KB | 239.646031 |
| 256 |  1024.00 B | 211.542931 |
| 256 |  512.00 B | 171.237458 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 232.647754 |
| 256 |  16.00 KB | 234.220261 |
| 256 |  8.00 KB | 229.612501 |
| 256 |  4.00 KB | 217.042557 |
| 256 |  2.00 KB | 193.270223 |
| 256 |  1024.00 B | 157.803997 |
| 256 |  512.00 B | 116.042213 |
| 128 |  32.00 KB | 231.088074 |
| 128 |  16.00 KB | 235.095130 |
| 128 |  8.00 KB | 230.193186 |
| 128 |  4.00 KB | 217.919431 |
| 128 |  2.00 KB | 193.572779 |
| 128 |  1024.00 B | 158.998496 |
| 128 |  512.00 B | 116.778332 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      200.009156      |
| 256        |      32.00 KB     |        1024        |      214.434035      |
| 256        |      32.00 KB     |        2048        |      223.123589      |
| 256        |      32.00 KB     |        4096        |      227.101648      |
| 256        |      32.00 KB     |        8192        |      229.341814      |
| 256        |      32.00 KB     |        16384        |      229.850570      |
| 256        |      16.00 KB     |        512        |      200.214616      |
| 256        |      16.00 KB     |        1024        |      217.978322      |
| 256        |      16.00 KB     |        2048        |      227.130671      |
| 256        |      16.00 KB     |        4096        |      231.811469      |
| 256        |      16.00 KB     |        8192        |      234.305046      |
| 256        |      8.00 KB     |        512        |      198.756558      |
| 256        |      8.00 KB     |        1024        |      214.676810      |
| 256        |      8.00 KB     |        2048        |      223.219060      |
| 256        |      8.00 KB     |        4096        |      226.628974      |
| 256        |      4.00 KB     |        512        |      188.717712      |
| 256        |      4.00 KB     |        1024        |      201.497333      |
| 256        |      4.00 KB     |        2048        |      210.078215      |
| 256        |      2.00 KB     |        512        |      168.603036      |
| 256        |      2.00 KB     |        1024        |      181.253976      |
| 256        |      1024.00 B     |        512        |      141.975737      |
| 128        |      32.00 KB     |        512        |      199.818204      |
| 128        |      32.00 KB     |        1024        |      214.599045      |
| 128        |      32.00 KB     |        2048        |      222.629964      |
| 128        |      32.00 KB     |        4096        |      227.404052      |
| 128        |      32.00 KB     |        8192        |      228.012273      |
| 128        |      32.00 KB     |        16384        |      231.357796      |
| 128        |      16.00 KB     |        512        |      199.192271      |
| 128        |      16.00 KB     |        1024        |      215.587812      |
| 128        |      16.00 KB     |        2048        |      225.635111      |
| 128        |      16.00 KB     |        4096        |      233.238725      |
| 128        |      16.00 KB     |        8192        |      234.928686      |
| 128        |      8.00 KB     |        512        |      198.834951      |
| 128        |      8.00 KB     |        1024        |      215.058986      |
| 128        |      8.00 KB     |        2048        |      223.689532      |
| 128        |      8.00 KB     |        4096        |      227.143464      |
| 128        |      4.00 KB     |        512        |      188.338075      |
| 128        |      4.00 KB     |        1024        |      203.014110      |
| 128        |      4.00 KB     |        2048        |      210.638640      |
| 128        |      2.00 KB     |        512        |      170.568945      |
| 128        |      2.00 KB     |        1024        |      182.449889      |
| 128        |      1024.00 B     |        512        |      142.618384      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    52   
    1024      |      TCMA     |     TCMA           |    50   
    1024      |      TCMB     |     TCMB           |    50   
    1024      |      OCRAM    |     TCMA           |    50   
    1024      |      TCMA     |     OCRAM          |    49   

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.043999 		| 624			| 276.950012		|
cos  		|0.0000002870	| 65			| 65.073997 		| 485			| 277.545990		|
sincos sin  	|0.0000001790	| 79			| 78.961998 		| 607			| 275.884003		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 413			| 74.681999 		| 563			| 428.765991		|
acos 		|0.0000004770	| -290			| 75.267998 		| 527			| 383.097992		|
atan 		|0.0000005360	| 80			| 80.760002 		| 486			| 370.846008		|
atan2 		|0.0000007150	| 117			| 104.648003 		| 854			| 479.937988		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

