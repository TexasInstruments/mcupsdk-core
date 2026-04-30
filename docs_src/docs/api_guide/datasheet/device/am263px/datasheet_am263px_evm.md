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
SBL : System Init                       |   538
SBL : Drivers_open                      |   109
SBL : LoadHsmRtFw                       |   10374
SBL : Board_driversOpen                 |   307
SBL : CPU Load                          |   9154
SBL : SBL End                           |   18
SBL : Total time taken                  |   20503

- Please note that the total time taken provided at the end is not including the ROM boot time.

### SBL OSPI SWAP performance

- Software/Application used           : sbl_ospi_swap and hello_world
- Size of sbl_ospi mcelf image        : 299 KB
- Size of hello_world                 : 30 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   22400
SBL : System Init                       |   558
SBL : Drivers_open                      |   112
SBL : LoadHsmRtFw                       |   7943
SBL : Board_driversOpen                 |   2597
SBL : CPU Load                          |   4731
SBL : SBL End                           |   7
SBL : Total time taken                  |   15951

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
SBL : System Init                       |   629
SBL : Drivers_open                      |   146053
SBL : LoadHsmRtFw                       |   10487
SBL : Board_driversOpen                 |   2809
SBL : File read from SD card            |   7952
SBL : CPU Load                          |   3864
SBL : SBL End                           |   2
SBL : Total time taken                  |   171798

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
 r5f0-0	| r5f1-0	|  1.91
 r5f0-0	| r5f1-1	|  2.01

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 12.202
 r5f0-0	| r5f1-0	| 4	| 12.587
 r5f0-0	| r5f1-1	| 4	| 12.201
 r5f0-0	| r5f0-1	| 32	| 15.236
 r5f0-0	| r5f0-1	| 64	| 18.185
 r5f0-0	| r5f0-1	| 112	| 22.507

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
- Begin tick       : 70
- End tick         : 10246868
- Total ticks      : 10246798
- Total time (secs): 10.246798
- Iterations/Sec   : 1463.871933
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1463.871933 
CoreMark/MHz :3.659680 / STACK

### DHRYSTONE

- BEGIN cycle count:                         7
- END Cycle count:                           146628881
- USER cycle count:                          146628874

BENCHMARK Using clock 400000000
- Usertime in sec:                           0.366572
- Microseconds for one run through Dhrystone:   0.7 
- Dhrystones per Second:                     1363987.8 

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
| 256 |  32.00 KB | 189.809155 |
| 256 |  16.00 KB | 189.069560 |
| 256 |  8.00 KB | 186.002157 |
| 256 |  4.00 KB | 175.742136 |
| 256 |  2.00 KB | 155.700744 |
| 256 |  1024.00 B | 127.427571 |
| 256 |  512.00 B | 93.377408 |
| 128 |  32.00 KB | 190.534129 |
| 128 |  16.00 KB | 188.885646 |
| 128 |  8.00 KB | 186.905280 |
| 128 |  4.00 KB | 176.041905 |
| 128 |  2.00 KB | 154.737563 |
| 128 |  1024.00 B | 127.387941 |
| 128 |  512.00 B | 93.409350 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.193448 |
| 256 | Decryption |  32.00 KB | 88.182397 |
| 256 | Encryption |  16.00 KB | 88.195228 |
| 256 | Decryption |  16.00 KB | 87.903775 |
| 256 | Encryption |  8.00 KB | 87.581311 |
| 256 | Decryption |  8.00 KB | 88.270809 |
| 256 | Encryption |  4.00 KB | 89.418282 |
| 256 | Decryption |  4.00 KB | 90.183019 |
| 256 | Encryption |  2.00 KB | 88.541821 |
| 256 | Decryption |  2.00 KB | 88.234265 |
| 256 | Encryption |  1024.00 B | 85.393376 |
| 256 | Decryption |  1024.00 B | 85.324445 |
| 256 | Encryption |  512.00 B | 78.780593 |
| 256 | Decryption |  512.00 B | 78.955231 |
| 128 | Encryption |  32.00 KB | 95.764388 |
| 128 | Decryption |  32.00 KB | 95.874802 |
| 128 | Encryption |  16.00 KB | 95.718581 |
| 128 | Decryption |  16.00 KB | 95.656934 |
| 128 | Encryption |  8.00 KB | 95.044106 |
| 128 | Decryption |  8.00 KB | 95.166250 |
| 128 | Encryption |  4.00 KB | 97.167385 |
| 128 | Decryption |  4.00 KB | 97.706282 |
| 128 | Encryption |  2.00 KB | 95.268276 |
| 128 | Decryption |  2.00 KB | 96.203870 |
| 128 | Encryption |  1024.00 B | 91.944218 |
| 128 | Decryption |  1024.00 B | 92.057873 |
| 128 | Encryption |  512.00 B | 83.749936 |
| 128 | Decryption |  512.00 B | 84.388360 |

### AES ECB STREAM

- Software/Application used : test_dthe_aes_ecb_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 66.428803 |
| 256 | Decryption |  512.00 B | 256 | 70.963271 |
| 256 | Encryption |  1024.00 B | 256 | 76.954510 |
| 256 | Decryption |  1024.00 B | 256 | 78.752193 |
| 256 | Encryption |  1024.00 B | 512 | 81.455702 |
| 256 | Decryption |  1024.00 B | 512 | 81.179239 |
| 256 | Encryption |  2.00 KB | 256 | 82.078000 |
| 256 | Decryption |  2.00 KB | 256 | 82.729717 |
| 256 | Encryption |  2.00 KB | 512 | 85.529338 |
| 256 | Decryption |  2.00 KB | 512 | 85.503673 |
| 256 | Encryption |  2.00 KB | 1024 | 86.708476 |
| 256 | Decryption |  2.00 KB | 1024 | 86.028958 |
| 256 | Encryption |  4.00 KB | 256 | 83.814737 |
| 256 | Decryption |  4.00 KB | 256 | 84.504790 |
| 256 | Encryption |  4.00 KB | 512 | 87.485149 |
| 256 | Decryption |  4.00 KB | 512 | 87.638406 |
| 256 | Encryption |  4.00 KB | 1024 | 88.940158 |
| 256 | Decryption |  4.00 KB | 1024 | 88.689804 |
| 256 | Encryption |  4.00 KB | 2048 | 89.575332 |
| 256 | Decryption |  4.00 KB | 2048 | 89.311656 |
| 256 | Encryption |  8.00 KB | 256 | 83.981483 |
| 256 | Decryption |  8.00 KB | 256 | 84.077912 |
| 256 | Encryption |  8.00 KB | 512 | 86.373356 |
| 256 | Decryption |  8.00 KB | 512 | 86.085460 |
| 256 | Encryption |  8.00 KB | 1024 | 87.532180 |
| 256 | Decryption |  8.00 KB | 1024 | 87.350178 |
| 256 | Encryption |  8.00 KB | 2048 | 88.136665 |
| 256 | Decryption |  8.00 KB | 2048 | 88.102304 |
| 256 | Encryption |  8.00 KB | 4096 | 88.607963 |
| 256 | Decryption |  8.00 KB | 4096 | 88.130146 |
| 256 | Encryption |  16.00 KB | 256 | 83.245291 |
| 256 | Decryption |  16.00 KB | 256 | 83.312490 |
| 256 | Encryption |  16.00 KB | 512 | 85.722461 |
| 256 | Decryption |  16.00 KB | 512 | 85.530315 |
| 256 | Encryption |  16.00 KB | 1024 | 86.892851 |
| 256 | Decryption |  16.00 KB | 1024 | 86.594335 |
| 256 | Encryption |  16.00 KB | 2048 | 87.434814 |
| 256 | Decryption |  16.00 KB | 2048 | 87.303488 |
| 256 | Encryption |  16.00 KB | 4096 | 87.812346 |
| 256 | Decryption |  16.00 KB | 4096 | 87.478872 |
| 256 | Encryption |  16.00 KB | 8192 | 87.916304 |
| 256 | Decryption |  16.00 KB | 8192 | 87.556592 |
| 256 | Encryption |  32.00 KB | 256 | 83.439711 |
| 256 | Decryption |  32.00 KB | 256 | 83.531439 |
| 256 | Encryption |  32.00 KB | 512 | 85.829465 |
| 256 | Decryption |  32.00 KB | 512 | 85.819631 |
| 256 | Encryption |  32.00 KB | 1024 | 87.042087 |
| 256 | Decryption |  32.00 KB | 1024 | 86.927932 |
| 256 | Encryption |  32.00 KB | 2048 | 87.612557 |
| 256 | Decryption |  32.00 KB | 2048 | 87.495150 |
| 256 | Encryption |  32.00 KB | 4096 | 87.895301 |
| 256 | Decryption |  32.00 KB | 4096 | 87.790657 |
| 256 | Encryption |  32.00 KB | 8192 | 88.055619 |
| 256 | Decryption |  32.00 KB | 8192 | 88.019548 |
| 256 | Encryption |  32.00 KB | 16384 | 88.117778 |
| 256 | Decryption |  32.00 KB | 16384 | 88.073814 |
| 128 | Encryption |  512.00 B | 256 | 73.467558 |
| 128 | Decryption |  512.00 B | 256 | 75.666189 |
| 128 | Encryption |  1024.00 B | 256 | 83.867830 |
| 128 | Decryption |  1024.00 B | 256 | 85.074123 |
| 128 | Encryption |  1024.00 B | 512 | 86.674073 |
| 128 | Decryption |  1024.00 B | 512 | 87.666542 |
| 128 | Encryption |  2.00 KB | 256 | 89.576556 |
| 128 | Decryption |  2.00 KB | 256 | 89.357931 |
| 128 | Encryption |  2.00 KB | 512 | 92.528379 |
| 128 | Decryption |  2.00 KB | 512 | 92.605519 |
| 128 | Encryption |  2.00 KB | 1024 | 94.002897 |
| 128 | Decryption |  2.00 KB | 1024 | 94.006943 |
| 128 | Encryption |  4.00 KB | 256 | 92.130993 |
| 128 | Decryption |  4.00 KB | 256 | 91.881055 |
| 128 | Encryption |  4.00 KB | 512 | 95.138964 |
| 128 | Decryption |  4.00 KB | 512 | 94.851180 |
| 128 | Encryption |  4.00 KB | 1024 | 96.593095 |
| 128 | Decryption |  4.00 KB | 1024 | 96.594519 |
| 128 | Encryption |  4.00 KB | 2048 | 97.015632 |
| 128 | Decryption |  4.00 KB | 2048 | 97.308775 |
| 128 | Encryption |  8.00 KB | 256 | 90.739606 |
| 128 | Decryption |  8.00 KB | 256 | 90.666796 |
| 128 | Encryption |  8.00 KB | 512 | 93.375080 |
| 128 | Decryption |  8.00 KB | 512 | 93.713558 |
| 128 | Encryption |  8.00 KB | 1024 | 94.927775 |
| 128 | Decryption |  8.00 KB | 1024 | 94.854955 |
| 128 | Encryption |  8.00 KB | 2048 | 95.417733 |
| 128 | Decryption |  8.00 KB | 2048 | 95.482376 |
| 128 | Encryption |  8.00 KB | 4096 | 96.213403 |
| 128 | Decryption |  8.00 KB | 4096 | 95.684517 |
| 128 | Encryption |  16.00 KB | 256 | 90.194965 |
| 128 | Decryption |  16.00 KB | 256 | 90.094840 |
| 128 | Encryption |  16.00 KB | 512 | 92.854651 |
| 128 | Decryption |  16.00 KB | 512 | 92.649868 |
| 128 | Encryption |  16.00 KB | 1024 | 94.195939 |
| 128 | Decryption |  16.00 KB | 1024 | 94.054331 |
| 128 | Encryption |  16.00 KB | 2048 | 94.798871 |
| 128 | Decryption |  16.00 KB | 2048 | 94.729330 |
| 128 | Encryption |  16.00 KB | 4096 | 95.270873 |
| 128 | Decryption |  16.00 KB | 4096 | 94.980915 |
| 128 | Encryption |  16.00 KB | 8192 | 95.559124 |
| 128 | Decryption |  16.00 KB | 8192 | 95.216544 |
| 128 | Encryption |  32.00 KB | 256 | 90.245655 |
| 128 | Decryption |  32.00 KB | 256 | 90.368229 |
| 128 | Encryption |  32.00 KB | 512 | 93.099918 |
| 128 | Decryption |  32.00 KB | 512 | 92.943787 |
| 128 | Encryption |  32.00 KB | 1024 | 94.377286 |
| 128 | Decryption |  32.00 KB | 1024 | 94.296996 |
| 128 | Encryption |  32.00 KB | 2048 | 95.047811 |
| 128 | Decryption |  32.00 KB | 2048 | 94.937315 |
| 128 | Encryption |  32.00 KB | 4096 | 95.366098 |
| 128 | Decryption |  32.00 KB | 4096 | 95.390737 |
| 128 | Encryption |  32.00 KB | 8192 | 95.613758 |
| 128 | Decryption |  32.00 KB | 8192 | 95.492724 |
| 128 | Encryption |  32.00 KB | 16384 | 95.763776 |
| 128 | Decryption |  32.00 KB | 16384 | 95.621605 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.127479 |
| 256 | Decryption |  32.00 KB | 88.148667 |
| 256 | Encryption |  16.00 KB | 88.178912 |
| 256 | Decryption |  16.00 KB | 88.044898 |
| 256 | Encryption |  8.00 KB | 87.640457 |
| 256 | Decryption |  8.00 KB | 87.647196 |
| 256 | Encryption |  4.00 KB | 88.891903 |
| 256 | Decryption |  4.00 KB | 89.705299 |
| 256 | Encryption |  2.00 KB | 88.136961 |
| 256 | Decryption |  2.00 KB | 87.543581 |
| 256 | Encryption |  1024.00 B | 84.599695 |
| 256 | Decryption |  1024.00 B | 84.436199 |
| 256 | Encryption |  512.00 B | 77.275729 |
| 256 | Decryption |  512.00 B | 77.656650 |
| 128 | Encryption |  32.00 KB | 95.681112 |
| 128 | Decryption |  32.00 KB | 95.740081 |
| 128 | Encryption |  16.00 KB | 95.487072 |
| 128 | Decryption |  16.00 KB | 95.546063 |
| 128 | Encryption |  8.00 KB | 96.009728 |
| 128 | Decryption |  8.00 KB | 95.714213 |
| 128 | Encryption |  4.00 KB | 96.571744 |
| 128 | Decryption |  4.00 KB | 97.194044 |
| 128 | Encryption |  2.00 KB | 95.548849 |
| 128 | Decryption |  2.00 KB | 95.932079 |
| 128 | Encryption |  1024.00 B | 90.921199 |
| 128 | Decryption |  1024.00 B | 91.247807 |
| 128 | Encryption |  512.00 B | 82.265515 |
| 128 | Decryption |  512.00 B | 82.898199 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 65.625250 |
| 256 | Decryption |  512.00 B | 256 | 70.314579 |
| 256 | Encryption |  1024.00 B | 256 | 76.334242 |
| 256 | Decryption |  1024.00 B | 256 | 78.214584 |
| 256 | Encryption |  1024.00 B | 512 | 80.794931 |
| 256 | Decryption |  1024.00 B | 512 | 80.580352 |
| 256 | Encryption |  2.00 KB | 256 | 81.302104 |
| 256 | Decryption |  2.00 KB | 256 | 82.163409 |
| 256 | Encryption |  2.00 KB | 512 | 85.210178 |
| 256 | Decryption |  2.00 KB | 512 | 85.126060 |
| 256 | Encryption |  2.00 KB | 1024 | 86.446558 |
| 256 | Decryption |  2.00 KB | 1024 | 85.825039 |
| 256 | Encryption |  4.00 KB | 256 | 84.609525 |
| 256 | Decryption |  4.00 KB | 256 | 85.067497 |
| 256 | Encryption |  4.00 KB | 512 | 87.294039 |
| 256 | Decryption |  4.00 KB | 512 | 87.559956 |
| 256 | Encryption |  4.00 KB | 1024 | 88.814804 |
| 256 | Decryption |  4.00 KB | 1024 | 88.516110 |
| 256 | Encryption |  4.00 KB | 2048 | 89.415232 |
| 256 | Decryption |  4.00 KB | 2048 | 89.120975 |
| 256 | Encryption |  8.00 KB | 256 | 83.486148 |
| 256 | Decryption |  8.00 KB | 256 | 83.802144 |
| 256 | Encryption |  8.00 KB | 512 | 86.417205 |
| 256 | Decryption |  8.00 KB | 512 | 86.081785 |
| 256 | Encryption |  8.00 KB | 1024 | 87.403767 |
| 256 | Decryption |  8.00 KB | 1024 | 87.329809 |
| 256 | Encryption |  8.00 KB | 2048 | 87.787203 |
| 256 | Decryption |  8.00 KB | 2048 | 87.826614 |
| 256 | Encryption |  8.00 KB | 4096 | 88.360664 |
| 256 | Decryption |  8.00 KB | 4096 | 87.952143 |
| 256 | Encryption |  16.00 KB | 256 | 83.288402 |
| 256 | Decryption |  16.00 KB | 256 | 83.277554 |
| 256 | Encryption |  16.00 KB | 512 | 85.608803 |
| 256 | Decryption |  16.00 KB | 512 | 85.392124 |
| 256 | Encryption |  16.00 KB | 1024 | 86.830970 |
| 256 | Decryption |  16.00 KB | 1024 | 86.580893 |
| 256 | Encryption |  16.00 KB | 2048 | 87.339847 |
| 256 | Decryption |  16.00 KB | 2048 | 87.308140 |
| 256 | Encryption |  16.00 KB | 4096 | 87.728886 |
| 256 | Decryption |  16.00 KB | 4096 | 87.388761 |
| 256 | Encryption |  16.00 KB | 8192 | 87.912471 |
| 256 | Decryption |  16.00 KB | 8192 | 87.648808 |
| 256 | Encryption |  32.00 KB | 256 | 83.428691 |
| 256 | Decryption |  32.00 KB | 256 | 83.531572 |
| 256 | Encryption |  32.00 KB | 512 | 85.873045 |
| 256 | Decryption |  32.00 KB | 512 | 85.707677 |
| 256 | Encryption |  32.00 KB | 1024 | 86.984178 |
| 256 | Decryption |  32.00 KB | 1024 | 86.923176 |
| 256 | Encryption |  32.00 KB | 2048 | 87.560321 |
| 256 | Decryption |  32.00 KB | 2048 | 87.470480 |
| 256 | Encryption |  32.00 KB | 4096 | 87.881157 |
| 256 | Decryption |  32.00 KB | 4096 | 87.817126 |
| 256 | Encryption |  32.00 KB | 8192 | 88.085652 |
| 256 | Decryption |  32.00 KB | 8192 | 87.946758 |
| 256 | Encryption |  32.00 KB | 16384 | 88.111558 |
| 256 | Decryption |  32.00 KB | 16384 | 88.091720 |
| 128 | Encryption |  512.00 B | 256 | 72.396271 |
| 128 | Decryption |  512.00 B | 256 | 73.221309 |
| 128 | Encryption |  1024.00 B | 256 | 82.885617 |
| 128 | Decryption |  1024.00 B | 256 | 83.672948 |
| 128 | Encryption |  1024.00 B | 512 | 85.746435 |
| 128 | Decryption |  1024.00 B | 512 | 86.791153 |
| 128 | Encryption |  2.00 KB | 256 | 88.319879 |
| 128 | Decryption |  2.00 KB | 256 | 87.981957 |
| 128 | Encryption |  2.00 KB | 512 | 91.994554 |
| 128 | Decryption |  2.00 KB | 512 | 92.157552 |
| 128 | Encryption |  2.00 KB | 1024 | 93.572061 |
| 128 | Decryption |  2.00 KB | 1024 | 93.578741 |
| 128 | Encryption |  4.00 KB | 256 | 90.672754 |
| 128 | Decryption |  4.00 KB | 256 | 91.058899 |
| 128 | Encryption |  4.00 KB | 512 | 94.788759 |
| 128 | Decryption |  4.00 KB | 512 | 94.621110 |
| 128 | Encryption |  4.00 KB | 1024 | 96.423974 |
| 128 | Decryption |  4.00 KB | 1024 | 96.278776 |
| 128 | Encryption |  4.00 KB | 2048 | 96.921655 |
| 128 | Decryption |  4.00 KB | 2048 | 97.109792 |
| 128 | Encryption |  8.00 KB | 256 | 90.023833 |
| 128 | Decryption |  8.00 KB | 256 | 90.331562 |
| 128 | Encryption |  8.00 KB | 512 | 93.211064 |
| 128 | Decryption |  8.00 KB | 512 | 93.360448 |
| 128 | Encryption |  8.00 KB | 1024 | 94.716494 |
| 128 | Decryption |  8.00 KB | 1024 | 94.627258 |
| 128 | Encryption |  8.00 KB | 2048 | 95.312960 |
| 128 | Decryption |  8.00 KB | 2048 | 95.477508 |
| 128 | Encryption |  8.00 KB | 4096 | 95.916986 |
| 128 | Decryption |  8.00 KB | 4096 | 95.660773 |
| 128 | Encryption |  16.00 KB | 256 | 89.959112 |
| 128 | Decryption |  16.00 KB | 256 | 89.903887 |
| 128 | Encryption |  16.00 KB | 512 | 92.718684 |
| 128 | Decryption |  16.00 KB | 512 | 92.640700 |
| 128 | Encryption |  16.00 KB | 1024 | 94.060068 |
| 128 | Decryption |  16.00 KB | 1024 | 93.902048 |
| 128 | Encryption |  16.00 KB | 2048 | 94.791330 |
| 128 | Decryption |  16.00 KB | 2048 | 94.653738 |
| 128 | Encryption |  16.00 KB | 4096 | 95.232110 |
| 128 | Decryption |  16.00 KB | 4096 | 95.061511 |
| 128 | Encryption |  16.00 KB | 8192 | 95.418948 |
| 128 | Decryption |  16.00 KB | 8192 | 95.138447 |
| 128 | Encryption |  32.00 KB | 256 | 90.144022 |
| 128 | Decryption |  32.00 KB | 256 | 90.345494 |
| 128 | Encryption |  32.00 KB | 512 | 93.045064 |
| 128 | Decryption |  32.00 KB | 512 | 92.861969 |
| 128 | Encryption |  32.00 KB | 1024 | 94.316165 |
| 128 | Decryption |  32.00 KB | 1024 | 94.253091 |
| 128 | Encryption |  32.00 KB | 2048 | 95.042469 |
| 128 | Decryption |  32.00 KB | 2048 | 94.878215 |
| 128 | Encryption |  32.00 KB | 4096 | 95.465251 |
| 128 | Decryption |  32.00 KB | 4096 | 95.169964 |
| 128 | Encryption |  32.00 KB | 8192 | 95.593186 |
| 128 | Decryption |  32.00 KB | 8192 | 95.457430 |
| 128 | Encryption |  32.00 KB | 16384 | 95.624395 |
| 128 | Decryption |  32.00 KB | 16384 | 95.663653 |

### SHA

- Software/Application used : test_dthe_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 284.278646 |
| 512 |  16.00 KB | 282.959949 |
| 512 |  8.00 KB | 279.823232 |
| 512 |  4.00 KB | 273.516830 |
| 512 |  2.00 KB | 261.829804 |
| 512 |  1024.00 B | 241.207214 |
| 512 |  512.00 B | 207.628944 |
| 256 |  32.00 KB | 275.016851 |
| 256 |  16.00 KB | 273.984228 |
| 256 |  8.00 KB | 270.273837 |
| 256 |  4.00 KB | 267.014342 |
| 256 |  2.00 KB | 257.843176 |
| 256 |  1024.00 B | 241.651917 |
| 256 |  512.00 B | 214.984910 |


### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Stream Size | Performance (Mbps) | 
|------------|------------|------|-------------|--------------------| 
| 256 | Encryption |  512.00 B | 256 | 65.509796 |
| 256 | Decryption |  512.00 B | 256 | 70.041040 |
| 256 | Encryption |  1024.00 B | 256 | 77.548219 |
| 256 | Decryption |  1024.00 B | 256 | 78.489988 |
| 256 | Encryption |  1024.00 B | 512 | 79.925850 |
| 256 | Decryption |  1024.00 B | 512 | 80.916634 |
| 256 | Encryption |  2.00 KB | 256 | 81.585499 |
| 256 | Decryption |  2.00 KB | 256 | 82.337865 |
| 256 | Encryption |  2.00 KB | 512 | 84.856018 |
| 256 | Decryption |  2.00 KB | 512 | 85.084064 |
| 256 | Encryption |  2.00 KB | 1024 | 86.392997 |
| 256 | Decryption |  2.00 KB | 1024 | 86.419200 |
| 256 | Encryption |  4.00 KB | 256 | 84.703024 |
| 256 | Decryption |  4.00 KB | 256 | 84.795632 |
| 256 | Encryption |  4.00 KB | 512 | 87.459464 |
| 256 | Decryption |  4.00 KB | 512 | 87.326609 |
| 256 | Encryption |  4.00 KB | 1024 | 88.748053 |
| 256 | Decryption |  4.00 KB | 1024 | 88.522088 |
| 256 | Encryption |  4.00 KB | 2048 | 89.433535 |
| 256 | Decryption |  4.00 KB | 2048 | 89.454898 |
| 256 | Encryption |  8.00 KB | 256 | 83.215034 |
| 256 | Decryption |  8.00 KB | 256 | 83.772953 |
| 256 | Encryption |  8.00 KB | 512 | 86.026700 |
| 256 | Decryption |  8.00 KB | 512 | 86.033193 |
| 256 | Encryption |  8.00 KB | 1024 | 87.246392 |
| 256 | Decryption |  8.00 KB | 1024 | 87.299853 |
| 256 | Encryption |  8.00 KB | 2048 | 87.862822 |
| 256 | Decryption |  8.00 KB | 2048 | 87.915272 |
| 256 | Encryption |  8.00 KB | 4096 | 88.378836 |
| 256 | Decryption |  8.00 KB | 4096 | 88.204576 |
| 256 | Encryption |  16.00 KB | 256 | 83.235511 |
| 256 | Decryption |  16.00 KB | 256 | 83.139553 |
| 256 | Encryption |  16.00 KB | 512 | 85.659576 |
| 256 | Decryption |  16.00 KB | 512 | 85.392541 |
| 256 | Encryption |  16.00 KB | 1024 | 86.895875 |
| 256 | Decryption |  16.00 KB | 1024 | 86.513031 |
| 256 | Encryption |  16.00 KB | 2048 | 87.417466 |
| 256 | Decryption |  16.00 KB | 2048 | 87.164874 |
| 256 | Encryption |  16.00 KB | 4096 | 87.722868 |
| 256 | Decryption |  16.00 KB | 4096 | 87.372014 |
| 256 | Encryption |  16.00 KB | 8192 | 87.742834 |
| 256 | Decryption |  16.00 KB | 8192 | 87.787203 |
| 256 | Encryption |  32.00 KB | 256 | 83.368130 |
| 256 | Decryption |  32.00 KB | 256 | 83.489870 |
| 256 | Encryption |  32.00 KB | 512 | 85.833470 |
| 256 | Decryption |  32.00 KB | 512 | 85.768317 |
| 256 | Encryption |  32.00 KB | 1024 | 87.005037 |
| 256 | Decryption |  32.00 KB | 1024 | 86.919213 |
| 256 | Encryption |  32.00 KB | 2048 | 87.573850 |
| 256 | Decryption |  32.00 KB | 2048 | 87.496099 |
| 256 | Encryption |  32.00 KB | 4096 | 87.928689 |
| 256 | Decryption |  32.00 KB | 4096 | 87.725950 |
| 256 | Encryption |  32.00 KB | 8192 | 88.075811 |
| 256 | Decryption |  32.00 KB | 8192 | 87.946831 |
| 256 | Encryption |  32.00 KB | 16384 | 88.145259 |
| 256 | Decryption |  32.00 KB | 16384 | 88.038393 |
| 128 | Encryption |  512.00 B | 256 | 71.768365 |
| 128 | Decryption |  512.00 B | 256 | 74.632169 |
| 128 | Encryption |  1024.00 B | 256 | 83.730676 |
| 128 | Decryption |  1024.00 B | 256 | 84.217019 |
| 128 | Encryption |  1024.00 B | 512 | 86.890115 |
| 128 | Decryption |  1024.00 B | 512 | 87.019333 |
| 128 | Encryption |  2.00 KB | 256 | 88.313928 |
| 128 | Decryption |  2.00 KB | 256 | 89.399375 |
| 128 | Encryption |  2.00 KB | 512 | 92.068219 |
| 128 | Decryption |  2.00 KB | 512 | 92.114806 |
| 128 | Encryption |  2.00 KB | 1024 | 92.821936 |
| 128 | Decryption |  2.00 KB | 1024 | 93.210070 |
| 128 | Encryption |  4.00 KB | 256 | 90.155726 |
| 128 | Decryption |  4.00 KB | 256 | 91.204631 |
| 128 | Encryption |  4.00 KB | 512 | 94.520123 |
| 128 | Decryption |  4.00 KB | 512 | 94.809328 |
| 128 | Encryption |  4.00 KB | 1024 | 96.331138 |
| 128 | Decryption |  4.00 KB | 1024 | 95.965793 |
| 128 | Encryption |  4.00 KB | 2048 | 97.093247 |
| 128 | Decryption |  4.00 KB | 2048 | 96.710691 |
| 128 | Encryption |  8.00 KB | 256 | 90.444071 |
| 128 | Decryption |  8.00 KB | 256 | 90.394171 |
| 128 | Encryption |  8.00 KB | 512 | 93.639913 |
| 128 | Decryption |  8.00 KB | 512 | 93.367431 |
| 128 | Encryption |  8.00 KB | 1024 | 94.640240 |
| 128 | Decryption |  8.00 KB | 1024 | 94.638190 |
| 128 | Encryption |  8.00 KB | 2048 | 95.640182 |
| 128 | Decryption |  8.00 KB | 2048 | 95.073406 |
| 128 | Encryption |  8.00 KB | 4096 | 95.777859 |
| 128 | Decryption |  8.00 KB | 4096 | 95.840188 |
| 128 | Encryption |  16.00 KB | 256 | 89.751829 |
| 128 | Decryption |  16.00 KB | 256 | 90.000961 |
| 128 | Encryption |  16.00 KB | 512 | 92.795650 |
| 128 | Decryption |  16.00 KB | 512 | 92.432133 |
| 128 | Encryption |  16.00 KB | 1024 | 94.149760 |
| 128 | Decryption |  16.00 KB | 1024 | 93.833648 |
| 128 | Encryption |  16.00 KB | 2048 | 94.818244 |
| 128 | Decryption |  16.00 KB | 2048 | 94.648782 |
| 128 | Encryption |  16.00 KB | 4096 | 95.123774 |
| 128 | Decryption |  16.00 KB | 4096 | 94.959411 |
| 128 | Encryption |  16.00 KB | 8192 | 95.253737 |
| 128 | Decryption |  16.00 KB | 8192 | 95.022746 |
| 128 | Encryption |  32.00 KB | 256 | 90.092208 |
| 128 | Decryption |  32.00 KB | 256 | 90.223059 |
| 128 | Encryption |  32.00 KB | 512 | 92.983595 |
| 128 | Decryption |  32.00 KB | 512 | 92.825223 |
| 128 | Encryption |  32.00 KB | 1024 | 94.288093 |
| 128 | Decryption |  32.00 KB | 1024 | 94.233863 |
| 128 | Encryption |  32.00 KB | 2048 | 95.010003 |
| 128 | Decryption |  32.00 KB | 2048 | 94.964743 |
| 128 | Encryption |  32.00 KB | 4096 | 95.348495 |
| 128 | Decryption |  32.00 KB | 4096 | 95.265680 |
| 128 | Encryption |  32.00 KB | 8192 | 95.593622 |
| 128 | Decryption |  32.00 KB | 8192 | 95.452477 |
| 128 | Encryption |  32.00 KB | 16384 | 95.776634 |
| 128 | Decryption |  32.00 KB | 16384 | 95.552680 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | operation  | Size | Performance (Mbps) | 
|-------------|------------|------|-------------| 
| 256 | Encryption |  32.00 KB | 88.327319 |
| 256 | Decryption |  32.00 KB | 88.124665 |
| 256 | Encryption |  16.00 KB | 88.117556 |
| 256 | Decryption |  16.00 KB | 88.029967 |
| 256 | Encryption |  8.00 KB | 87.608824 |
| 256 | Decryption |  8.00 KB | 88.572336 |
| 256 | Encryption |  4.00 KB | 88.925676 |
| 256 | Decryption |  4.00 KB | 90.051665 |
| 256 | Encryption |  2.00 KB | 88.113261 |
| 256 | Decryption |  2.00 KB | 88.235453 |
| 256 | Encryption |  1024.00 B | 84.191054 |
| 256 | Decryption |  1024.00 B | 84.658709 |
| 256 | Encryption |  512.00 B | 76.909355 |
| 256 | Decryption |  512.00 B | 77.465721 |
| 128 | Encryption |  32.00 KB | 95.707748 |
| 128 | Decryption |  32.00 KB | 95.845883 |
| 128 | Encryption |  16.00 KB | 95.561562 |
| 128 | Decryption |  16.00 KB | 95.634250 |
| 128 | Encryption |  8.00 KB | 95.056513 |
| 128 | Decryption |  8.00 KB | 95.869630 |
| 128 | Encryption |  4.00 KB | 96.595942 |
| 128 | Decryption |  4.00 KB | 97.641502 |
| 128 | Encryption |  2.00 KB | 95.088580 |
| 128 | Decryption |  2.00 KB | 94.979710 |
| 128 | Encryption |  1024.00 B | 90.667109 |
| 128 | Decryption |  1024.00 B | 90.923722 |
| 128 | Encryption |  512.00 B | 82.030742 |
| 128 | Decryption |  512.00 B | 83.192851 |

### HMAC SHA

- Software/Application used : test_dthe_hmac_sha 
- Code Placement            : OCMC 
- Data Placement            : OCMC 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| SHA | Size | Performance (Mbps) | 
|-----|------|-------------| 
| 512 |  32.00 KB | 282.058759 |
| 512 |  16.00 KB | 278.715000 |
| 512 |  8.00 KB | 270.849090 |
| 512 |  4.00 KB | 259.076534 |
| 512 |  2.00 KB | 236.370194 |
| 512 |  1024.00 B | 196.027758 |
| 512 |  512.00 B | 154.449472 |
| 256 |  32.00 KB | 273.494001 |
| 256 |  16.00 KB | 271.136233 |
| 256 |  8.00 KB | 264.797269 |
| 256 |  4.00 KB | 256.415674 |
| 256 |  2.00 KB | 239.103944 |
| 256 |  1024.00 B | 210.998068 |
| 256 |  512.00 B | 170.435868 |

### AES CMAC

- Software/Application used : test_athe_aes_cmac 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Size | Performance (Mbps) | 
|------------|------|--------------------| 
| 256 |  32.00 KB | 227.974102 |
| 256 |  16.00 KB | 228.215240 |
| 256 |  8.00 KB | 224.411458 |
| 256 |  4.00 KB | 209.460496 |
| 256 |  2.00 KB | 182.206406 |
| 256 |  1024.00 B | 144.639153 |
| 256 |  512.00 B | 102.927503 |
| 128 |  32.00 KB | 229.057253 |
| 128 |  16.00 KB | 227.248809 |
| 128 |  8.00 KB | 225.455609 |
| 128 |  4.00 KB | 209.973888 |
| 128 |  2.00 KB | 182.908178 |
| 128 |  1024.00 B | 145.557925 |
| 128 |  512.00 B | 103.369085 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream 
- Code Placement            : OCRAM 
- Data Placement            : OCRAM 
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ 
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) | 
|------------|-------------------|--------------------|--------------------| 
| 256        |      32.00 KB     |        512        |      190.338718      |
| 256        |      32.00 KB     |        1024        |      207.988067      |
| 256        |      32.00 KB     |        2048        |      217.078503      |
| 256        |      32.00 KB     |        4096        |      224.376403      |
| 256        |      32.00 KB     |        8192        |      226.081707      |
| 256        |      32.00 KB     |        16384        |      226.351589      |
| 256        |      16.00 KB     |        512        |      188.563639      |
| 256        |      16.00 KB     |        1024        |      208.250781      |
| 256        |      16.00 KB     |        2048        |      219.829097      |
| 256        |      16.00 KB     |        4096        |      225.249292      |
| 256        |      16.00 KB     |        8192        |      228.395185      |
| 256        |      8.00 KB     |        512        |      186.081377      |
| 256        |      8.00 KB     |        1024        |      205.345449      |
| 256        |      8.00 KB     |        2048        |      215.605543      |
| 256        |      8.00 KB     |        4096        |      220.161419      |
| 256        |      4.00 KB     |        512        |      175.765703      |
| 256        |      4.00 KB     |        1024        |      191.732249      |
| 256        |      4.00 KB     |        2048        |      199.197568      |
| 256        |      2.00 KB     |        512        |      155.793277      |
| 256        |      2.00 KB     |        1024        |      168.807150      |
| 256        |      1024.00 B     |        512        |      126.576020      |
| 128        |      32.00 KB     |        512        |      189.732566      |
| 128        |      32.00 KB     |        1024        |      207.563185      |
| 128        |      32.00 KB     |        2048        |      217.325505      |
| 128        |      32.00 KB     |        4096        |      223.378361      |
| 128        |      32.00 KB     |        8192        |      226.160702      |
| 128        |      32.00 KB     |        16384        |      228.722497      |
| 128        |      16.00 KB     |        512        |      188.162334      |
| 128        |      16.00 KB     |        1024        |      209.119627      |
| 128        |      16.00 KB     |        2048        |      220.899799      |
| 128        |      16.00 KB     |        4096        |      226.862365      |
| 128        |      16.00 KB     |        8192        |      230.295310      |
| 128        |      8.00 KB     |        512        |      186.899949      |
| 128        |      8.00 KB     |        1024        |      204.510809      |
| 128        |      8.00 KB     |        2048        |      215.743949      |
| 128        |      8.00 KB     |        4096        |      221.373620      |
| 128        |      4.00 KB     |        512        |      176.089205      |
| 128        |      4.00 KB     |        1024        |      191.824847      |
| 128        |      4.00 KB     |        2048        |      201.147908      |
| 128        |      2.00 KB     |        512        |      154.690082      |
| 128        |      2.00 KB     |        1024        |      169.225605      |
| 128        |      1024.00 B     |        512        |      127.690749      |

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48   
    1024      |      TCMA     |     TCMA           |    45   
    1024      |      TCMB     |     TCMB           |    46   
    1024      |      OCRAM    |     TCMA           |    45   
    1024      |      TCMA     |     OCRAM          |    44     

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi 
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed 
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.146000 		| 504			| 276.373993		|
cos  		|0.0000002870	| 65			| 65.073997 		| 504			| 277.738007		|
sincos sin  	|0.0000001790	| 79			| 78.961998 		| 467			| 275.261993		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.003998 		| 581			| 428.809998		|
acos 		|0.0000004770	| 76			| 76.031998 		| 824			| 384.041992		|
atan 		|0.0000005360	| 80			| 80.054001 		| 507			| 371.420013		|
atan2 		|0.0000007150	| 117			| 104.667999 		| 722			| 479.970001		|

### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance

### XIP Benchmark

Comparing data from \ref EXAMPLES_OPTIFLASH_XIP_BENCHMARK and \ref EXAMPLES_OPTIFLASH_OCRAM_BENCHMARK, execution time of a code which throws ~3 Million I-Cache Miss per seconds is 2.2 times slower when it runs from OCRAM. 

### Flash performance Benchmark

#### CPU Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      143 |        55.944056 |
|        2 |      280 |        57.142857 |
|        4 |      558 |        57.347670 |
|        8 |     1105 |        57.918552 |
|       16 |     2216 |        57.761733 |
|       32 |     4439 |        57.670647 |
|       64 |     8874 |        57.696642 |
|      128 |    17746 |        57.703144 |
|      256 |    35480 |        57.722661 |
|      512 |    70963 |        57.720220 |

#### DMA Read

CPU with operating speed  : R5F with 400MHZ 

| Size(KB) | Time(us) | Throughput(Mbps) |
|----------|----------|------------------|
|        1 |      144 |        55.555556 |
|        2 |       31 |       516.129032 |
|        4 |       45 |       711.111111 |
|        8 |       77 |       831.168831 |
|       16 |      142 |       901.408451 |
|       32 |      273 |       937.728938 |
|       64 |      531 |       964.218456 |
|      128 |     1051 |       974.310181 |
|      256 |     2088 |       980.842912 |
|      512 |     3160 |      1296.202532 |