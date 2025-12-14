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
- Size of ipc_notify_echo             : 97 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   480
SBL : Drivers_open                      |   128
SBL : LoadHsmRtFw                       |   10330
SBL : Board_driversOpen                 |   1908
SBL : CPU Load                          |   5647
SBL : SBL End                           |   19
SBL : Total time taken                  |   18515

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 299 KB
- Size of hello_world                 : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   535
SBL : Drivers_open                      |   104
SBL : LoadHsmRtFw                       |   7866
SBL : Board_driversOpen                 |   2630
SBL : CPU Load                          |   4767
SBL : SBL End                           |   6
SBL : Total time taken                  |   15911

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
SBL : System Init                       |   612
SBL : Drivers_open                      |   145131
SBL : LoadHsmRtFw                       |   10535
SBL : Board_driversOpen                 |   2846
SBL : File read from SD card            |   7861
SBL : CPU Load                          |   3896
SBL : SBL End                           |   2
SBL : Total time taken                  |   170885

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
 r5f0-0	| r5f0-1	|  1.95
 r5f0-0	| r5f1-0	|  1.90
 r5f0-0	| r5f1-1	|  2.00

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 12.002
 r5f0-0	| r5f1-0	| 4	| 12.140
 r5f0-0	| r5f1-1	| 4	| 12.056
 r5f0-0	| r5f0-1	| 32	| 15.008
 r5f0-0	| r5f0-1	| 64	| 17.929
 r5f0-0	| r5f0-1	| 112	| 22.360

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
 400		| 08			|  8.62 Mbps / 371.33 us 	| 22.17 Mbps / 144.37 us 	|  0.92 Mbps / 3475.20 us
 200		| 16			| 17.44 Mbps / 183.44 us 	| 28.65 Mbps / 111.68 us 	|  0.96 Mbps / 3346.91 us
 100		| 32			| 31.65 Mbps / 101.10 us 	| 33.68 Mbps / 95.00 us 	|  0.97 Mbps / 3282.92 us
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
- Begin tick       : 69
- End tick         : 10237856
- Total ticks      : 10237787
- Total time (secs): 10.237787
- Iterations/Sec   : 1465.160391
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1465.160391 
CoreMark/MHz :3.662901 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146628827
- USER cycle count:                          146628820

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
| 256 |  32.00 KB | 189.900936 |
| 256 |  16.00 KB | 188.399662 |
| 256 |  8.00 KB | 186.470530 |
| 256 |  4.00 KB | 175.052086 |
| 256 |  2.00 KB | 155.291218 |
| 256 |  1024.00 B | 127.373086 |
| 256 |  512.00 B | 92.943045 |
| 128 |  32.00 KB | 189.267497 |
| 128 |  16.00 KB | 187.777527 |
| 128 |  8.00 KB | 186.761468 |
| 128 |  4.00 KB | 175.605573 |
| 128 |  2.00 KB | 154.398530 |
| 128 |  1024.00 B | 127.720611 |
| 128 |  512.00 B | 93.783629 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.268580 |
| 256 | Decryption |  32.00 KB | 88.139554 |
| 256 | Encryption |  16.00 KB | 87.943438 |
| 256 | Decryption |  16.00 KB | 88.151335 |
| 256 | Encryption |  8.00 KB | 87.965719 |
| 256 | Decryption |  8.00 KB | 88.762778 |
| 256 | Encryption |  4.00 KB | 89.105828 |
| 256 | Decryption |  4.00 KB | 90.156346 |
| 256 | Encryption |  2.00 KB | 88.873218 |
| 256 | Decryption |  2.00 KB | 88.247334 |
| 256 | Encryption |  1024.00 B | 84.977049 |
| 256 | Decryption |  1024.00 B | 85.275595 |
| 256 | Encryption |  512.00 B | 78.501270 |
| 256 | Decryption |  512.00 B | 78.587874 |
| 128 | Encryption |  32.00 KB | 95.844568 |
| 128 | Decryption |  32.00 KB | 95.821884 |
| 128 | Encryption |  16.00 KB | 95.309322 |
| 128 | Decryption |  16.00 KB | 95.671247 |
| 128 | Encryption |  8.00 KB | 96.147061 |
| 128 | Decryption |  8.00 KB | 96.518409 |
| 128 | Encryption |  4.00 KB | 97.181793 |
| 128 | Decryption |  4.00 KB | 97.859473 |
| 128 | Encryption |  2.00 KB | 95.468120 |
| 128 | Decryption |  2.00 KB | 96.074120 |
| 128 | Encryption |  1024.00 B | 91.995845 |
| 128 | Decryption |  1024.00 B | 92.055287 |
| 128 | Encryption |  512.00 B | 83.596102 |
| 128 | Decryption |  512.00 B | 84.750672 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 66.791684 |
| 256 | Decryption |  512.00 B | 256 | 70.390101 |
| 256 | Encryption |  1024.00 B | 256 | 76.288036 |
| 256 | Decryption |  1024.00 B | 256 | 78.454282 |
| 256 | Encryption |  1024.00 B | 512 | 81.245661 |
| 256 | Decryption |  1024.00 B | 512 | 81.044717 |
| 256 | Encryption |  2.00 KB | 256 | 82.419669 |
| 256 | Decryption |  2.00 KB | 256 | 82.969565 |
| 256 | Encryption |  2.00 KB | 512 | 85.462417 |
| 256 | Decryption |  2.00 KB | 512 | 85.315559 |
| 256 | Encryption |  2.00 KB | 1024 | 86.703887 |
| 256 | Decryption |  2.00 KB | 1024 | 86.188485 |
| 256 | Encryption |  4.00 KB | 256 | 83.901115 |
| 256 | Decryption |  4.00 KB | 256 | 84.510784 |
| 256 | Encryption |  4.00 KB | 512 | 87.335919 |
| 256 | Decryption |  4.00 KB | 512 | 87.596822 |
| 256 | Encryption |  4.00 KB | 1024 | 88.859963 |
| 256 | Decryption |  4.00 KB | 1024 | 88.718619 |
| 256 | Encryption |  4.00 KB | 2048 | 89.545958 |
| 256 | Decryption |  4.00 KB | 2048 | 89.329308 |
| 256 | Encryption |  8.00 KB | 256 | 83.594236 |
| 256 | Decryption |  8.00 KB | 256 | 83.713830 |
| 256 | Encryption |  8.00 KB | 512 | 86.159307 |
| 256 | Decryption |  8.00 KB | 512 | 86.087439 |
| 256 | Encryption |  8.00 KB | 1024 | 87.503547 |
| 256 | Decryption |  8.00 KB | 1024 | 87.243198 |
| 256 | Encryption |  8.00 KB | 2048 | 87.938571 |
| 256 | Decryption |  8.00 KB | 2048 | 87.867239 |
| 256 | Encryption |  8.00 KB | 4096 | 88.336844 |
| 256 | Decryption |  8.00 KB | 4096 | 88.217340 |
| 256 | Encryption |  16.00 KB | 256 | 83.208563 |
| 256 | Decryption |  16.00 KB | 256 | 83.302959 |
| 256 | Encryption |  16.00 KB | 512 | 85.699481 |
| 256 | Decryption |  16.00 KB | 512 | 85.469801 |
| 256 | Encryption |  16.00 KB | 1024 | 86.788136 |
| 256 | Decryption |  16.00 KB | 1024 | 86.718945 |
| 256 | Encryption |  16.00 KB | 2048 | 87.388470 |
| 256 | Decryption |  16.00 KB | 2048 | 87.242763 |
| 256 | Encryption |  16.00 KB | 4096 | 87.759430 |
| 256 | Decryption |  16.00 KB | 4096 | 87.586139 |
| 256 | Encryption |  16.00 KB | 8192 | 87.727124 |
| 256 | Decryption |  16.00 KB | 8192 | 87.805581 |
| 256 | Encryption |  32.00 KB | 256 | 83.358786 |
| 256 | Decryption |  32.00 KB | 256 | 83.481828 |
| 256 | Encryption |  32.00 KB | 512 | 85.800390 |
| 256 | Decryption |  32.00 KB | 512 | 85.758847 |
| 256 | Encryption |  32.00 KB | 1024 | 87.058275 |
| 256 | Decryption |  32.00 KB | 1024 | 86.872622 |
| 256 | Encryption |  32.00 KB | 2048 | 87.553303 |
| 256 | Decryption |  32.00 KB | 2048 | 87.503766 |
| 256 | Encryption |  32.00 KB | 4096 | 87.870994 |
| 256 | Decryption |  32.00 KB | 4096 | 87.809331 |
| 256 | Encryption |  32.00 KB | 8192 | 88.142666 |
| 256 | Decryption |  32.00 KB | 8192 | 87.929500 |
| 256 | Encryption |  32.00 KB | 16384 | 88.099565 |
| 256 | Decryption |  32.00 KB | 16384 | 88.065973 |
| 128 | Encryption |  512.00 B | 256 | 72.892290 |
| 128 | Decryption |  512.00 B | 256 | 75.363385 |
| 128 | Encryption |  1024.00 B | 256 | 82.238675 |
| 128 | Decryption |  1024.00 B | 256 | 84.977049 |
| 128 | Encryption |  1024.00 B | 512 | 87.507344 |
| 128 | Decryption |  1024.00 B | 512 | 87.547089 |
| 128 | Encryption |  2.00 KB | 256 | 87.904070 |
| 128 | Decryption |  2.00 KB | 256 | 89.271509 |
| 128 | Encryption |  2.00 KB | 512 | 92.431807 |
| 128 | Decryption |  2.00 KB | 512 | 92.618607 |
| 128 | Encryption |  2.00 KB | 1024 | 93.926104 |
| 128 | Decryption |  2.00 KB | 1024 | 93.248531 |
| 128 | Encryption |  4.00 KB | 256 | 91.780045 |
| 128 | Decryption |  4.00 KB | 256 | 92.177643 |
| 128 | Encryption |  4.00 KB | 512 | 95.070647 |
| 128 | Decryption |  4.00 KB | 512 | 94.830592 |
| 128 | Encryption |  4.00 KB | 1024 | 96.568187 |
| 128 | Decryption |  4.00 KB | 1024 | 96.329014 |
| 128 | Encryption |  4.00 KB | 2048 | 97.440434 |
| 128 | Decryption |  4.00 KB | 2048 | 97.381794 |
| 128 | Encryption |  8.00 KB | 256 | 90.497150 |
| 128 | Decryption |  8.00 KB | 256 | 90.424104 |
| 128 | Encryption |  8.00 KB | 512 | 93.281381 |
| 128 | Decryption |  8.00 KB | 512 | 93.296652 |
| 128 | Encryption |  8.00 KB | 1024 | 94.535462 |
| 128 | Decryption |  8.00 KB | 1024 | 94.907842 |
| 128 | Encryption |  8.00 KB | 2048 | 95.402454 |
| 128 | Decryption |  8.00 KB | 2048 | 95.270008 |
| 128 | Encryption |  8.00 KB | 4096 | 95.792209 |
| 128 | Decryption |  8.00 KB | 4096 | 95.725048 |
| 128 | Encryption |  16.00 KB | 256 | 89.879535 |
| 128 | Decryption |  16.00 KB | 256 | 90.050892 |
| 128 | Encryption |  16.00 KB | 512 | 92.700979 |
| 128 | Decryption |  16.00 KB | 512 | 92.702290 |
| 128 | Encryption |  16.00 KB | 1024 | 94.182402 |
| 128 | Decryption |  16.00 KB | 1024 | 94.035438 |
| 128 | Encryption |  16.00 KB | 2048 | 94.869545 |
| 128 | Decryption |  16.00 KB | 2048 | 94.722826 |
| 128 | Encryption |  16.00 KB | 4096 | 95.297541 |
| 128 | Decryption |  16.00 KB | 4096 | 94.848949 |
| 128 | Encryption |  16.00 KB | 8192 | 95.527783 |
| 128 | Decryption |  16.00 KB | 8192 | 95.023607 |
| 128 | Encryption |  32.00 KB | 256 | 90.130230 |
| 128 | Decryption |  32.00 KB | 256 | 90.233696 |
| 128 | Encryption |  32.00 KB | 512 | 92.949719 |
| 128 | Decryption |  32.00 KB | 512 | 92.919243 |
| 128 | Encryption |  32.00 KB | 1024 | 94.347395 |
| 128 | Decryption |  32.00 KB | 1024 | 94.221416 |
| 128 | Encryption |  32.00 KB | 2048 | 95.029549 |
| 128 | Decryption |  32.00 KB | 2048 | 94.890922 |
| 128 | Encryption |  32.00 KB | 4096 | 95.490811 |
| 128 | Decryption |  32.00 KB | 4096 | 95.244306 |
| 128 | Encryption |  32.00 KB | 8192 | 95.643061 |
| 128 | Decryption |  32.00 KB | 8192 | 95.550677 |
| 128 | Encryption |  32.00 KB | 16384 | 95.643061 |
| 128 | Decryption |  32.00 KB | 16384 | 95.669414 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.116371 |
| 256 | Decryption |  32.00 KB | 88.122295 |
| 256 | Encryption |  16.00 KB | 87.875339 |
| 256 | Decryption |  16.00 KB | 88.081878 |
| 256 | Encryption |  8.00 KB | 88.169866 |
| 256 | Decryption |  8.00 KB | 88.345477 |
| 256 | Encryption |  4.00 KB | 89.161593 |
| 256 | Decryption |  4.00 KB | 89.560030 |
| 256 | Encryption |  2.00 KB | 88.375856 |
| 256 | Decryption |  2.00 KB | 87.596236 |
| 256 | Encryption |  1024.00 B | 84.639029 |
| 256 | Decryption |  1024.00 B | 84.571311 |
| 256 | Encryption |  512.00 B | 77.557396 |
| 256 | Decryption |  512.00 B | 77.594127 |
| 128 | Encryption |  32.00 KB | 95.690018 |
| 128 | Decryption |  32.00 KB | 95.857010 |
| 128 | Encryption |  16.00 KB | 95.188711 |
| 128 | Decryption |  16.00 KB | 95.645067 |
| 128 | Encryption |  8.00 KB | 96.319458 |
| 128 | Decryption |  8.00 KB | 96.157289 |
| 128 | Encryption |  4.00 KB | 96.429649 |
| 128 | Decryption |  4.00 KB | 97.227209 |
| 128 | Encryption |  2.00 KB | 95.688359 |
| 128 | Decryption |  2.00 KB | 95.926463 |
| 128 | Encryption |  1024.00 B | 91.230024 |
| 128 | Decryption |  1024.00 B | 91.339373 |
| 128 | Encryption |  512.00 B | 82.944363 |
| 128 | Decryption |  512.00 B | 83.515139 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 64.440511 |
| 256 | Decryption |  512.00 B | 256 | 69.772592 |
| 256 | Encryption |  1024.00 B | 256 | 76.501763 |
| 256 | Decryption |  1024.00 B | 256 | 77.754313 |
| 256 | Encryption |  1024.00 B | 512 | 80.412270 |
| 256 | Decryption |  1024.00 B | 512 | 80.645796 |
| 256 | Encryption |  2.00 KB | 256 | 81.250697 |
| 256 | Decryption |  2.00 KB | 256 | 82.326487 |
| 256 | Encryption |  2.00 KB | 512 | 85.163671 |
| 256 | Decryption |  2.00 KB | 512 | 85.070810 |
| 256 | Encryption |  2.00 KB | 1024 | 86.391859 |
| 256 | Decryption |  2.00 KB | 1024 | 85.938709 |
| 256 | Encryption |  4.00 KB | 256 | 84.534766 |
| 256 | Decryption |  4.00 KB | 256 | 85.048178 |
| 256 | Encryption |  4.00 KB | 512 | 87.228973 |
| 256 | Decryption |  4.00 KB | 512 | 87.462382 |
| 256 | Encryption |  4.00 KB | 1024 | 88.751058 |
| 256 | Decryption |  4.00 KB | 1024 | 88.566351 |
| 256 | Encryption |  4.00 KB | 2048 | 89.408523 |
| 256 | Decryption |  4.00 KB | 2048 | 89.053763 |
| 256 | Encryption |  8.00 KB | 256 | 83.228508 |
| 256 | Decryption |  8.00 KB | 256 | 83.616634 |
| 256 | Encryption |  8.00 KB | 512 | 86.175735 |
| 256 | Decryption |  8.00 KB | 512 | 85.919274 |
| 256 | Encryption |  8.00 KB | 1024 | 87.285610 |
| 256 | Decryption |  8.00 KB | 1024 | 87.154441 |
| 256 | Encryption |  8.00 KB | 2048 | 87.857816 |
| 256 | Decryption |  8.00 KB | 2048 | 87.850160 |
| 256 | Encryption |  8.00 KB | 4096 | 88.353517 |
| 256 | Decryption |  8.00 KB | 4096 | 88.029524 |
| 256 | Encryption |  16.00 KB | 256 | 83.071975 |
| 256 | Decryption |  16.00 KB | 256 | 83.265915 |
| 256 | Encryption |  16.00 KB | 512 | 85.666714 |
| 256 | Decryption |  16.00 KB | 512 | 85.325973 |
| 256 | Encryption |  16.00 KB | 1024 | 86.858589 |
| 256 | Decryption |  16.00 KB | 1024 | 86.575460 |
| 256 | Encryption |  16.00 KB | 2048 | 87.331991 |
| 256 | Decryption |  16.00 KB | 2048 | 87.190531 |
| 256 | Encryption |  16.00 KB | 4096 | 87.669034 |
| 256 | Decryption |  16.00 KB | 4096 | 87.481500 |
| 256 | Encryption |  16.00 KB | 8192 | 87.967342 |
| 256 | Decryption |  16.00 KB | 8192 | 87.469824 |
| 256 | Encryption |  32.00 KB | 256 | 83.347257 |
| 256 | Decryption |  32.00 KB | 256 | 83.433470 |
| 256 | Encryption |  32.00 KB | 512 | 85.817383 |
| 256 | Decryption |  32.00 KB | 512 | 85.712441 |
| 256 | Encryption |  32.00 KB | 1024 | 86.959075 |
| 256 | Decryption |  32.00 KB | 1024 | 86.897531 |
| 256 | Encryption |  32.00 KB | 2048 | 87.570120 |
| 256 | Decryption |  32.00 KB | 2048 | 87.501575 |
| 256 | Encryption |  32.00 KB | 4096 | 87.831837 |
| 256 | Decryption |  32.00 KB | 4096 | 87.788893 |
| 256 | Encryption |  32.00 KB | 8192 | 88.003592 |
| 256 | Decryption |  32.00 KB | 8192 | 88.010831 |
| 256 | Encryption |  32.00 KB | 16384 | 88.097641 |
| 256 | Decryption |  32.00 KB | 16384 | 88.076625 |
| 128 | Encryption |  512.00 B | 256 | 71.505259 |
| 128 | Decryption |  512.00 B | 256 | 72.511618 |
| 128 | Encryption |  1024.00 B | 256 | 83.536430 |
| 128 | Decryption |  1024.00 B | 256 | 82.766285 |
| 128 | Encryption |  1024.00 B | 512 | 86.402109 |
| 128 | Decryption |  1024.00 B | 512 | 86.584754 |
| 128 | Encryption |  2.00 KB | 256 | 89.039849 |
| 128 | Decryption |  2.00 KB | 256 | 88.673603 |
| 128 | Encryption |  2.00 KB | 512 | 91.958410 |
| 128 | Decryption |  2.00 KB | 512 | 92.051408 |
| 128 | Encryption |  2.00 KB | 1024 | 93.509310 |
| 128 | Decryption |  2.00 KB | 1024 | 93.466634 |
| 128 | Encryption |  4.00 KB | 256 | 91.747923 |
| 128 | Decryption |  4.00 KB | 256 | 91.602371 |
| 128 | Encryption |  4.00 KB | 512 | 94.719575 |
| 128 | Decryption |  4.00 KB | 512 | 94.647757 |
| 128 | Encryption |  4.00 KB | 1024 | 96.397026 |
| 128 | Decryption |  4.00 KB | 1024 | 96.343176 |
| 128 | Encryption |  4.00 KB | 2048 | 96.858627 |
| 128 | Decryption |  4.00 KB | 2048 | 97.081741 |
| 128 | Encryption |  8.00 KB | 256 | 90.050737 |
| 128 | Decryption |  8.00 KB | 256 | 90.189535 |
| 128 | Encryption |  8.00 KB | 512 | 93.120340 |
| 128 | Decryption |  8.00 KB | 512 | 93.153430 |
| 128 | Encryption |  8.00 KB | 1024 | 94.730185 |
| 128 | Decryption |  8.00 KB | 1024 | 94.558650 |
| 128 | Encryption |  8.00 KB | 2048 | 95.249238 |
| 128 | Decryption |  8.00 KB | 2048 | 95.204268 |
| 128 | Encryption |  8.00 KB | 4096 | 95.949987 |
| 128 | Decryption |  8.00 KB | 4096 | 95.485854 |
| 128 | Encryption |  16.00 KB | 256 | 90.006369 |
| 128 | Decryption |  16.00 KB | 256 | 89.919306 |
| 128 | Encryption |  16.00 KB | 512 | 92.768230 |
| 128 | Decryption |  16.00 KB | 512 | 92.513194 |
| 128 | Encryption |  16.00 KB | 1024 | 93.927955 |
| 128 | Decryption |  16.00 KB | 1024 | 93.950173 |
| 128 | Encryption |  16.00 KB | 2048 | 94.796814 |
| 128 | Decryption |  16.00 KB | 2048 | 94.533928 |
| 128 | Encryption |  16.00 KB | 4096 | 95.157095 |
| 128 | Decryption |  16.00 KB | 4096 | 94.999846 |
| 128 | Encryption |  16.00 KB | 8192 | 95.364450 |
| 128 | Decryption |  16.00 KB | 8192 | 95.070820 |
| 128 | Encryption |  32.00 KB | 256 | 90.082997 |
| 128 | Decryption |  32.00 KB | 256 | 90.144332 |
| 128 | Encryption |  32.00 KB | 512 | 92.903601 |
| 128 | Decryption |  32.00 KB | 512 | 92.919901 |
| 128 | Encryption |  32.00 KB | 1024 | 94.268343 |
| 128 | Decryption |  32.00 KB | 1024 | 94.226326 |
| 128 | Encryption |  32.00 KB | 2048 | 95.005613 |
| 128 | Decryption |  32.00 KB | 2048 | 94.895903 |
| 128 | Encryption |  32.00 KB | 4096 | 95.392299 |
| 128 | Decryption |  32.00 KB | 4096 | 95.236521 |
| 128 | Encryption |  32.00 KB | 8192 | 95.625616 |
| 128 | Decryption |  32.00 KB | 8192 | 95.488376 |
| 128 | Encryption |  32.00 KB | 16384 | 95.703468 |
| 128 | Decryption |  32.00 KB | 16384 | 95.616373 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 284.311019 |
| 512 |  16.00 KB | 283.131092 |
| 512 |  8.00 KB | 280.137213 |
| 512 |  4.00 KB | 273.676738 |
| 512 |  2.00 KB | 261.861190 |
| 512 |  1024.00 B | 241.278256 |
| 512 |  512.00 B | 207.786937 |
| 256 |  32.00 KB | 275.013966 |
| 256 |  16.00 KB | 274.138950 |
| 256 |  8.00 KB | 270.499737 |
| 256 |  4.00 KB | 267.063306 |
| 256 |  2.00 KB | 257.914207 |
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
| 256 | Encryption |  512.00 B | 256 | 65.119237 |
| 256 | Decryption |  512.00 B | 256 | 69.713216 |
| 256 | Encryption |  1024.00 B | 256 | 77.199265 |
| 256 | Decryption |  1024.00 B | 256 | 77.896639 |
| 256 | Encryption |  1024.00 B | 512 | 79.503106 |
| 256 | Decryption |  1024.00 B | 512 | 80.683525 |
| 256 | Encryption |  2.00 KB | 256 | 82.206696 |
| 256 | Decryption |  2.00 KB | 256 | 82.265515 |
| 256 | Encryption |  2.00 KB | 512 | 84.738618 |
| 256 | Decryption |  2.00 KB | 512 | 85.130483 |
| 256 | Encryption |  2.00 KB | 1024 | 86.311076 |
| 256 | Decryption |  2.00 KB | 1024 | 86.357707 |
| 256 | Encryption |  4.00 KB | 256 | 84.518958 |
| 256 | Decryption |  4.00 KB | 256 | 84.663084 |
| 256 | Encryption |  4.00 KB | 512 | 87.320791 |
| 256 | Decryption |  4.00 KB | 512 | 87.190676 |
| 256 | Encryption |  4.00 KB | 1024 | 88.628634 |
| 256 | Decryption |  4.00 KB | 1024 | 88.453388 |
| 256 | Encryption |  4.00 KB | 2048 | 89.318959 |
| 256 | Decryption |  4.00 KB | 2048 | 89.430484 |
| 256 | Encryption |  8.00 KB | 256 | 83.960234 |
| 256 | Decryption |  8.00 KB | 256 | 83.826530 |
| 256 | Encryption |  8.00 KB | 512 | 86.022183 |
| 256 | Decryption |  8.00 KB | 512 | 85.957307 |
| 256 | Encryption |  8.00 KB | 1024 | 87.176179 |
| 256 | Decryption |  8.00 KB | 1024 | 87.212722 |
| 256 | Encryption |  8.00 KB | 2048 | 87.846922 |
| 256 | Decryption |  8.00 KB | 2048 | 88.002336 |
| 256 | Encryption |  8.00 KB | 4096 | 88.323450 |
| 256 | Decryption |  8.00 KB | 4096 | 87.999087 |
| 256 | Encryption |  16.00 KB | 256 | 83.161972 |
| 256 | Decryption |  16.00 KB | 256 | 83.201036 |
| 256 | Encryption |  16.00 KB | 512 | 85.526548 |
| 256 | Decryption |  16.00 KB | 512 | 85.459074 |
| 256 | Encryption |  16.00 KB | 1024 | 86.841181 |
| 256 | Decryption |  16.00 KB | 1024 | 86.522739 |
| 256 | Encryption |  16.00 KB | 2048 | 87.323700 |
| 256 | Decryption |  16.00 KB | 2048 | 87.245521 |
| 256 | Encryption |  16.00 KB | 4096 | 87.632254 |
| 256 | Decryption |  16.00 KB | 4096 | 87.416008 |
| 256 | Encryption |  16.00 KB | 8192 | 87.930607 |
| 256 | Decryption |  16.00 KB | 8192 | 87.723748 |
| 256 | Encryption |  32.00 KB | 256 | 83.416810 |
| 256 | Decryption |  32.00 KB | 256 | 83.429022 |
| 256 | Encryption |  32.00 KB | 512 | 85.754850 |
| 256 | Decryption |  32.00 KB | 512 | 85.676514 |
| 256 | Encryption |  32.00 KB | 1024 | 86.979488 |
| 256 | Decryption |  32.00 KB | 1024 | 86.873269 |
| 256 | Encryption |  32.00 KB | 2048 | 87.528381 |
| 256 | Decryption |  32.00 KB | 2048 | 87.464862 |
| 256 | Encryption |  32.00 KB | 4096 | 87.844861 |
| 256 | Decryption |  32.00 KB | 4096 | 87.718024 |
| 256 | Encryption |  32.00 KB | 8192 | 88.118888 |
| 256 | Decryption |  32.00 KB | 8192 | 87.898617 |
| 256 | Encryption |  32.00 KB | 16384 | 88.130590 |
| 256 | Decryption |  32.00 KB | 16384 | 88.105857 |
| 128 | Encryption |  512.00 B | 256 | 72.090465 |
| 128 | Decryption |  512.00 B | 256 | 74.540491 |
| 128 | Encryption |  1024.00 B | 256 | 82.455964 |
| 128 | Decryption |  1024.00 B | 256 | 84.210526 |
| 128 | Encryption |  1024.00 B | 512 | 86.637407 |
| 128 | Decryption |  1024.00 B | 512 | 86.947754 |
| 128 | Encryption |  2.00 KB | 256 | 87.005470 |
| 128 | Decryption |  2.00 KB | 256 | 88.423552 |
| 128 | Encryption |  2.00 KB | 512 | 91.967443 |
| 128 | Decryption |  2.00 KB | 512 | 92.121280 |
| 128 | Encryption |  2.00 KB | 1024 | 92.741810 |
| 128 | Decryption |  2.00 KB | 1024 | 93.538672 |
| 128 | Encryption |  4.00 KB | 256 | 90.465607 |
| 128 | Decryption |  4.00 KB | 256 | 91.153889 |
| 128 | Encryption |  4.00 KB | 512 | 94.534439 |
| 128 | Decryption |  4.00 KB | 512 | 94.872463 |
| 128 | Encryption |  4.00 KB | 1024 | 96.326891 |
| 128 | Decryption |  4.00 KB | 1024 | 96.034700 |
| 128 | Encryption |  4.00 KB | 2048 | 97.048675 |
| 128 | Decryption |  4.00 KB | 2048 | 96.862922 |
| 128 | Encryption |  8.00 KB | 256 | 89.984896 |
| 128 | Decryption |  8.00 KB | 256 | 90.619469 |
| 128 | Encryption |  8.00 KB | 512 | 93.342164 |
| 128 | Decryption |  8.00 KB | 512 | 93.236923 |
| 128 | Encryption |  8.00 KB | 1024 | 94.694939 |
| 128 | Decryption |  8.00 KB | 1024 | 94.791501 |
| 128 | Encryption |  8.00 KB | 2048 | 95.721902 |
| 128 | Decryption |  8.00 KB | 2048 | 95.143799 |
| 128 | Encryption |  8.00 KB | 4096 | 95.758266 |
| 128 | Decryption |  8.00 KB | 4096 | 95.735885 |
| 128 | Encryption |  16.00 KB | 256 | 89.865516 |
| 128 | Decryption |  16.00 KB | 256 | 89.931799 |
| 128 | Encryption |  16.00 KB | 512 | 92.700159 |
| 128 | Decryption |  16.00 KB | 512 | 92.398250 |
| 128 | Encryption |  16.00 KB | 1024 | 94.142321 |
| 128 | Decryption |  16.00 KB | 1024 | 93.780945 |
| 128 | Encryption |  16.00 KB | 2048 | 94.704860 |
| 128 | Decryption |  16.00 KB | 2048 | 94.556604 |
| 128 | Encryption |  16.00 KB | 4096 | 95.113075 |
| 128 | Decryption |  16.00 KB | 4096 | 94.963195 |
| 128 | Encryption |  16.00 KB | 8192 | 95.227094 |
| 128 | Decryption |  16.00 KB | 8192 | 95.309148 |
| 128 | Encryption |  32.00 KB | 256 | 90.056460 |
| 128 | Decryption |  32.00 KB | 256 | 90.155260 |
| 128 | Encryption |  32.00 KB | 512 | 92.955569 |
| 128 | Decryption |  32.00 KB | 512 | 92.831469 |
| 128 | Encryption |  32.00 KB | 1024 | 94.299456 |
| 128 | Decryption |  32.00 KB | 1024 | 94.159736 |
| 128 | Encryption |  32.00 KB | 2048 | 95.039971 |
| 128 | Decryption |  32.00 KB | 2048 | 94.894614 |
| 128 | Encryption |  32.00 KB | 4096 | 95.443788 |
| 128 | Decryption |  32.00 KB | 4096 | 95.211875 |
| 128 | Encryption |  32.00 KB | 8192 | 95.494550 |
| 128 | Decryption |  32.00 KB | 8192 | 95.497507 |
| 128 | Encryption |  32.00 KB | 16384 | 95.623610 |
| 128 | Decryption |  32.00 KB | 16384 | 95.587174 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.115704 |
| 256 | Decryption |  32.00 KB | 88.239388 |
| 256 | Encryption |  16.00 KB | 88.010905 |
| 256 | Decryption |  16.00 KB | 88.189443 |
| 256 | Encryption |  8.00 KB | 88.181729 |
| 256 | Decryption |  8.00 KB | 87.670060 |
| 256 | Encryption |  4.00 KB | 89.428653 |
| 256 | Decryption |  4.00 KB | 90.153245 |
| 256 | Encryption |  2.00 KB | 87.965424 |
| 256 | Decryption |  2.00 KB | 88.383007 |
| 256 | Encryption |  1024.00 B | 84.143492 |
| 256 | Decryption |  1024.00 B | 84.390533 |
| 256 | Encryption |  512.00 B | 77.159273 |
| 256 | Decryption |  512.00 B | 77.502365 |
| 128 | Encryption |  32.00 KB | 95.661995 |
| 128 | Decryption |  32.00 KB | 95.840451 |
| 128 | Encryption |  16.00 KB | 95.258064 |
| 128 | Decryption |  16.00 KB | 95.468467 |
| 128 | Encryption |  8.00 KB | 95.327517 |
| 128 | Decryption |  8.00 KB | 95.466729 |
| 128 | Encryption |  4.00 KB | 96.510592 |
| 128 | Decryption |  4.00 KB | 97.953815 |
| 128 | Encryption |  2.00 KB | 95.488985 |
| 128 | Decryption |  2.00 KB | 95.204614 |
| 128 | Encryption |  1024.00 B | 90.805298 |
| 128 | Decryption |  1024.00 B | 91.382676 |
| 128 | Encryption |  512.00 B | 81.719787 |
| 128 | Decryption |  512.00 B | 83.459834 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 282.152112 |
| 512 |  16.00 KB | 279.082939 |
| 512 |  8.00 KB | 270.720423 |
| 512 |  4.00 KB | 259.091897 |
| 512 |  2.00 KB | 236.660407 |
| 512 |  1024.00 B | 196.180327 |
| 512 |  512.00 B | 154.814325 |
| 256 |  32.00 KB | 273.588908 |
| 256 |  16.00 KB | 271.186721 |
| 256 |  8.00 KB | 265.003387 |
| 256 |  4.00 KB | 256.722031 |
| 256 |  2.00 KB | 239.296016 |
| 256 |  1024.00 B | 211.270148 |
| 256 |  512.00 B | 171.022965 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 226.783860 |
| 256 |  16.00 KB | 228.013760 |
| 256 |  8.00 KB | 224.190748 |
| 256 |  4.00 KB | 208.836416 |
| 256 |  2.00 KB | 181.902964 |
| 256 |  1024.00 B | 144.416042 |
| 256 |  512.00 B | 102.721003 |
| 128 |  32.00 KB | 227.453873 |
| 128 |  16.00 KB | 229.567258 |
| 128 |  8.00 KB | 224.823328 |
| 128 |  4.00 KB | 209.473886 |
| 128 |  2.00 KB | 182.622750 |
| 128 |  1024.00 B | 144.734982 |
| 128 |  512.00 B | 102.798344 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      189.768277      |
| 256        |      32.00 KB     |        1024        |      207.316138      |
| 256        |      32.00 KB     |        2048        |      217.026834      |
| 256        |      32.00 KB     |        4096        |      224.022629      |
| 256        |      32.00 KB     |        8192        |      225.498760      |
| 256        |      32.00 KB     |        16384        |      226.782389      |
| 256        |      16.00 KB     |        512        |      189.195100      |
| 256        |      16.00 KB     |        1024        |      209.290758      |
| 256        |      16.00 KB     |        2048        |      220.902591      |
| 256        |      16.00 KB     |        4096        |      226.501694      |
| 256        |      16.00 KB     |        8192        |      229.774515      |
| 256        |      8.00 KB     |        512        |      185.917730      |
| 256        |      8.00 KB     |        1024        |      205.022642      |
| 256        |      8.00 KB     |        2048        |      215.453148      |
| 256        |      8.00 KB     |        4096        |      219.993286      |
| 256        |      4.00 KB     |        512        |      175.492716      |
| 256        |      4.00 KB     |        1024        |      191.332020      |
| 256        |      4.00 KB     |        2048        |      199.209678      |
| 256        |      2.00 KB     |        512        |      155.571381      |
| 256        |      2.00 KB     |        1024        |      168.637744      |
| 256        |      1024.00 B     |        512        |      127.195094      |
| 128        |      32.00 KB     |        512        |      189.552501      |
| 128        |      32.00 KB     |        1024        |      206.752438      |
| 128        |      32.00 KB     |        2048        |      217.190461      |
| 128        |      32.00 KB     |        4096        |      223.512635      |
| 128        |      32.00 KB     |        8192        |      225.908794      |
| 128        |      32.00 KB     |        16384        |      228.512153      |
| 128        |      16.00 KB     |        512        |      186.266485      |
| 128        |      16.00 KB     |        1024        |      207.761412      |
| 128        |      16.00 KB     |        2048        |      219.323316      |
| 128        |      16.00 KB     |        4096        |      225.317054      |
| 128        |      16.00 KB     |        8192        |      228.782880      |
| 128        |      8.00 KB     |        512        |      186.732201      |
| 128        |      8.00 KB     |        1024        |      204.447009      |
| 128        |      8.00 KB     |        2048        |      215.669401      |
| 128        |      8.00 KB     |        4096        |      221.018996      |
| 128        |      4.00 KB     |        512        |      176.048998      |
| 128        |      4.00 KB     |        1024        |      191.768716      |
| 128        |      4.00 KB     |        2048        |      201.150995      |
| 128        |      2.00 KB     |        512        |      154.946094      |
| 128        |      2.00 KB     |        1024        |      169.225605      |
| 128        |      1024.00 B     |        512        |      127.790344      |

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
acos 		|0.0000004770	| 76			| 76.031998 		| 812			| 383.940002		|
atan 		|0.0000005360	| 80			| 80.015999 		| 652			| 371.477997		|
atan2 		|0.0000007150	| 117			| 104.669998 		| 562			| 479.351990		|

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
|        2 |     1100 |        14.545455 |
|        4 |     2195 |        14.578588 |
|        8 |     4387 |        14.588557 |
|       16 |     8770 |        14.595211 |
|       32 |    17536 |        14.598540 |
|       64 |    35063 |        14.602287 |
|      128 |    70129 |        14.601663 |
|      256 |   140258 |        14.601663 |
|      512 |   280522 |        14.601350 |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      554 |        14.440433 |
|        2 |       30 |       533.333333 |
|        4 |       45 |       711.111111 |
|        8 |       78 |       820.512821 |
|       16 |      141 |       907.801418 |
|       32 |      273 |       937.728938 |
|       64 |      532 |       962.406015 |
|      128 |     1050 |       975.238095 |
|      256 |     2084 |       982.725528 |
|      512 |     4161 |       984.378755 |