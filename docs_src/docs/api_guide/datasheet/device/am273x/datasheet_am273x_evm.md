# Datasheet {#DATASHEET_AM273X_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for AM273x

## Generic Setup details

SOC Details             | Values
------------------------|------------------------------
Core                    | R5F
Core Operating Speed    | 400 MHz
Cache Status            | Enabled

Optimization Details    | Values
------------------------|------------------------------
Build Profile           | Release
R5F Compiler flags      | -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -Wall -Werror -g -mthumb -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -Os
R5F Linker flags        | -Wl,--diag_suppress=10063 -Wl,--ram_model -Wl,--reread_libs
Code Placement          | MSS L2 RAM
Data Placement          | MSS L2 RAM

## Performance Numbers

### SBL QSPI performance

- Software/Application used        		    : sbl_qspi and test_soc_r5f_system
- Size of sbl_qspi appimage        		    : 167.3 KB
- Size of test_soc_r5f_system appimage      : 72.4 KB

Boot time breakdown                     |   Time (us)
----------------------------------------|--------------
SBL : System Init                       |   501
SBL : Drivers_open                      |   20
SBL : LoadHsmRtFw                       |   7868
SBL : Board_driversOpen                 |   2540
SBL : CPU Load                          |   1648
SBL : Total time taken                  |   12581

- Please note that the total time taken provided at the end is not including the ROM boot time.

### MCAN performance

- Internal loopback mode of operation
- Memory Mode               : TX Buffer Mode, RX FIFO Mode
- MCAN CLK                  : 80MHz
- CPU                       : R5F

Theoretical Rate Calculation

Frame Type             | Arbitration BitRate(Mbps) | Data BitRate(Mbps) | Arb Phase bits | Data Phase bits | Theoretical Throughput (Msg/Sec) | Actual Throughput (Msg/Sec)
-----------------------|---------------------------|--------------------|----------------|-----------------|----------------------------------|----------------------------
CAN FD STANDARD FORMAT | 1                         | 5                  |  27            | 538             | 7430                             | 6579
CAN FD EXTENDED FORMAT | 1                         | 5                  |  46            | 538             | 6510                             | 5714

### EDMA

### EDMA MEMORY COPY BENCHMARK

EDMA Memory Copy Benchmark Numbers Print Start
Size in Bytes | Source Memory | Destination Memory | Transfer time(us)
--------------|---------------|--------------------|------------------
    1024      |      OCRAM    |     OCRAM          |    48
    1024      |      TCMA     |     TCMA           |    46
    1024      |      TCMB     |     TCMB           |    46
    1024      |      OCRAM    |     TCMA           |    45
    1024      |      TCMA     |     OCRAM          |    46


### MIBSPI Performance

- Internal loopback operation
- Software/Application used : test_mibspi_performance


- CPU: R5F
- MIBSPI Clock: 40MHz

Data Width  | Data Length | Transfer Time (micro sec)
------------|-------------|------------------------------
 16		    | 1024        | 246.90


- CPU: C66
- MIBSPI Clock: 40MHz

Data Width  | Data Length | Transfer Time (micro sec)
------------|-------------|------------------------------
 16		    | 1024        | 249.40
----------------------------------------------------------

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0	| r5f0-1	|  1.63
 r5f0-0	| c66ss0	|  3.32

#### IPC RPMSG

- 1000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 4	| 8.931
 r5f0-0	| c66ss0	| 4	| 12.997
 r5f0-0	| r5f0-1	| 32	| 11.276
 r5f0-0	| r5f0-1	| 64	| 13.715
 r5f0-0	| r5f0-1	| 112	| 17.232

### MATHLIB

#### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.040001 		| 680			| 276.511993		|
cos  		|0.0000002870	| 64			| 64.106003 		| 495			| 276.656006		|
sincos sin  	|0.0000001790	| 78			| 78.073997 		| 672			| 275.087982		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 73			| 73.033997 		| 562			| 428.747986		|
acos 		|0.0000004770	| 74			| 74.029999 		| 772			| 383.888000		|
atan 		|0.0000005360	| 85			| 85.019997 		| 755			| 372.580017		|
atan2 		|0.0000007150	| 117			| 105.606003 		| 577			| 477.250000		|


### COREMARK

2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 80
- End tick         : 9947432
- Total ticks      : 9947352
- Total time (secs): 9.947352
- Iterations/Sec   : 1507.938997
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1507.938997
CoreMark/MHz :3.769847 / STACK

### DHRYSTONE

status 0

Dhrystone Benchmark, Version 2.1 (Language: C)

- BEGIN cycle count:                         7
- END Cycle count:                           144622362
- USER cycle count:                          144622355
- Usertime in sec:                           0.723112
- Microseconds for one run through Dhrystone:   1.4
- Dhrystones per Second:                     691456.0

Normalized MIPS/MHz:                         1.9677

### DTHE

### SHA

- Software/Application used : test_dthe_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 329.857057 |
| 512 |  16.00 KB | 328.647456 |
| 512 |  8.00 KB | 325.055179 |
| 512 |  4.00 KB | 317.434792 |
| 512 |  2.00 KB | 303.815308 |
| 512 |  1024.00 B | 279.566590 |
| 512 |  512.00 B | 240.340326 |
| 256 |  32.00 KB | 318.828523 |
| 256 |  16.00 KB | 316.845852 |
| 256 |  8.00 KB | 314.982277 |
| 256 |  4.00 KB | 309.475126 |
| 256 |  2.00 KB | 293.554311 |
| 256 |  1024.00 B | 279.853104 |
| 256 |  512.00 B | 248.392965 |

### AES CTR

- Software/Application used : test_dthe_aes_ctr_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 78.795749 |
| 256 | Decryption |  512.00 B | 256 | 85.955616 |
| 256 | Encryption |  1024.00 B | 256 | 92.317228 |
| 256 | Decryption |  1024.00 B | 256 | 93.740703 |
| 256 | Encryption |  1024.00 B | 512 | 96.615167 |
| 256 | Decryption |  1024.00 B | 512 | 96.843599 |
| 256 | Encryption |  2.00 KB | 256 | 96.392063 |
| 256 | Decryption |  2.00 KB | 256 | 97.316722 |
| 256 | Encryption |  2.00 KB | 512 | 100.992418 |
| 256 | Decryption |  2.00 KB | 512 | 101.167027 |
| 256 | Encryption |  2.00 KB | 1024 | 102.790282 |
| 256 | Decryption |  2.00 KB | 1024 | 102.795119 |
| 256 | Encryption |  4.00 KB | 256 | 100.031290 |
| 256 | Decryption |  4.00 KB | 256 | 99.685138 |
| 256 | Encryption |  4.00 KB | 512 | 103.277862 |
| 256 | Decryption |  4.00 KB | 512 | 103.612591 |
| 256 | Encryption |  4.00 KB | 1024 | 105.185780 |
| 256 | Decryption |  4.00 KB | 1024 | 105.098064 |
| 256 | Encryption |  4.00 KB | 2048 | 105.759517 |
| 256 | Decryption |  4.00 KB | 2048 | 106.081355 |
| 256 | Encryption |  8.00 KB | 256 | 97.812006 |
| 256 | Decryption |  8.00 KB | 256 | 98.128718 |
| 256 | Encryption |  8.00 KB | 512 | 101.387707 |
| 256 | Decryption |  8.00 KB | 512 | 101.082375 |
| 256 | Encryption |  8.00 KB | 1024 | 102.440816 |
| 256 | Decryption |  8.00 KB | 1024 | 102.739119 |
| 256 | Encryption |  8.00 KB | 2048 | 103.388654 |
| 256 | Decryption |  8.00 KB | 2048 | 103.449446 |
| 256 | Encryption |  8.00 KB | 4096 | 103.746270 |
| 256 | Decryption |  8.00 KB | 4096 | 103.750787 |
| 256 | Encryption |  16.00 KB | 256 | 97.265087 |
| 256 | Decryption |  16.00 KB | 256 | 97.106735 |
| 256 | Encryption |  16.00 KB | 512 | 100.164301 |
| 256 | Decryption |  16.00 KB | 512 | 99.832625 |
| 256 | Encryption |  16.00 KB | 1024 | 101.663338 |
| 256 | Decryption |  16.00 KB | 1024 | 101.444402 |
| 256 | Encryption |  16.00 KB | 2048 | 102.428008 |
| 256 | Decryption |  16.00 KB | 2048 | 102.132103 |
| 256 | Encryption |  16.00 KB | 4096 | 102.747173 |
| 256 | Decryption |  16.00 KB | 4096 | 102.478260 |
| 256 | Encryption |  16.00 KB | 8192 | 102.860461 |
| 256 | Decryption |  16.00 KB | 8192 | 102.769932 |
| 256 | Encryption |  32.00 KB | 256 | 97.213688 |
| 256 | Decryption |  32.00 KB | 256 | 97.415902 |
| 256 | Encryption |  32.00 KB | 512 | 100.322233 |
| 256 | Decryption |  32.00 KB | 512 | 100.173009 |
| 256 | Encryption |  32.00 KB | 1024 | 101.671815 |
| 256 | Decryption |  32.00 KB | 1024 | 101.707614 |
| 256 | Encryption |  32.00 KB | 2048 | 102.516332 |
| 256 | Decryption |  32.00 KB | 2048 | 102.366511 |
| 256 | Encryption |  32.00 KB | 4096 | 102.926089 |
| 256 | Decryption |  32.00 KB | 4096 | 102.753516 |
| 256 | Encryption |  32.00 KB | 8192 | 103.073197 |
| 256 | Decryption |  32.00 KB | 8192 | 102.968236 |
| 256 | Encryption |  32.00 KB | 16384 | 103.157969 |
| 256 | Decryption |  32.00 KB | 16384 | 103.075325 |
| 128 | Encryption |  512.00 B | 256 | 87.372014 |
| 128 | Decryption |  512.00 B | 256 | 91.260514 |
| 128 | Encryption |  1024.00 B | 256 | 99.710921 |
| 128 | Decryption |  1024.00 B | 256 | 101.376729 |
| 128 | Encryption |  1024.00 B | 512 | 103.141328 |
| 128 | Decryption |  1024.00 B | 512 | 104.526460 |
| 128 | Encryption |  2.00 KB | 256 | 105.332862 |
| 128 | Decryption |  2.00 KB | 256 | 106.196526 |
| 128 | Encryption |  2.00 KB | 512 | 109.705715 |
| 128 | Decryption |  2.00 KB | 512 | 109.762674 |
| 128 | Encryption |  2.00 KB | 1024 | 111.752268 |
| 128 | Decryption |  2.00 KB | 1024 | 111.727501 |
| 128 | Encryption |  4.00 KB | 256 | 105.636777 |
| 128 | Decryption |  4.00 KB | 256 | 107.690285 |
| 128 | Encryption |  4.00 KB | 512 | 112.788917 |
| 128 | Decryption |  4.00 KB | 512 | 112.101126 |
| 128 | Encryption |  4.00 KB | 1024 | 114.497362 |
| 128 | Decryption |  4.00 KB | 1024 | 114.660625 |
| 128 | Encryption |  4.00 KB | 2048 | 114.925033 |
| 128 | Decryption |  4.00 KB | 2048 | 115.438204 |
| 128 | Encryption |  8.00 KB | 256 | 105.986997 |
| 128 | Decryption |  8.00 KB | 256 | 106.450093 |
| 128 | Encryption |  8.00 KB | 512 | 109.606636 |
| 128 | Decryption |  8.00 KB | 512 | 109.276296 |
| 128 | Encryption |  8.00 KB | 1024 | 111.686563 |
| 128 | Decryption |  8.00 KB | 1024 | 111.025746 |
| 128 | Encryption |  8.00 KB | 2048 | 112.046504 |
| 128 | Decryption |  8.00 KB | 2048 | 112.212933 |
| 128 | Encryption |  8.00 KB | 4096 | 112.700665 |
| 128 | Decryption |  8.00 KB | 4096 | 112.479673 |
| 128 | Encryption |  16.00 KB | 256 | 105.102278 |
| 128 | Decryption |  16.00 KB | 256 | 105.214699 |
| 128 | Encryption |  16.00 KB | 512 | 108.625155 |
| 128 | Decryption |  16.00 KB | 512 | 108.275422 |
| 128 | Encryption |  16.00 KB | 1024 | 110.253401 |
| 128 | Decryption |  16.00 KB | 1024 | 110.034734 |
| 128 | Encryption |  16.00 KB | 2048 | 111.212269 |
| 128 | Decryption |  16.00 KB | 2048 | 110.925444 |
| 128 | Encryption |  16.00 KB | 4096 | 111.603353 |
| 128 | Decryption |  16.00 KB | 4096 | 111.397763 |
| 128 | Encryption |  16.00 KB | 8192 | 111.944588 |
| 128 | Decryption |  16.00 KB | 8192 | 111.447964 |
| 128 | Encryption |  32.00 KB | 256 | 105.315512 |
| 128 | Decryption |  32.00 KB | 256 | 105.380180 |
| 128 | Encryption |  32.00 KB | 512 | 108.711645 |
| 128 | Decryption |  32.00 KB | 512 | 108.649579 |
| 128 | Encryption |  32.00 KB | 1024 | 110.464095 |
| 128 | Decryption |  32.00 KB | 1024 | 110.386763 |
| 128 | Encryption |  32.00 KB | 2048 | 111.410545 |
| 128 | Decryption |  32.00 KB | 2048 | 111.210736 |
| 128 | Encryption |  32.00 KB | 4096 | 111.806127 |
| 128 | Decryption |  32.00 KB | 4096 | 111.754769 |
| 128 | Encryption |  32.00 KB | 8192 | 112.051532 |
| 128 | Decryption |  32.00 KB | 8192 | 111.922005 |
| 128 | Encryption |  32.00 KB | 16384 | 112.097771 |
| 128 | Decryption |  32.00 KB | 16384 | 112.210171 |

### AES ECB

- Software/Application used : test_dthe_aes_ecb_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 78.441136 |
| 256 | Decryption |  512.00 B | 256 | 84.566945 |
| 256 | Encryption |  1024.00 B | 256 | 93.612159 |
| 256 | Decryption |  1024.00 B | 256 | 94.293689 |
| 256 | Encryption |  1024.00 B | 512 | 96.362300 |
| 256 | Decryption |  1024.00 B | 512 | 97.191161 |
| 256 | Encryption |  2.00 KB | 256 | 97.728866 |
| 256 | Decryption |  2.00 KB | 256 | 98.483733 |
| 256 | Encryption |  2.00 KB | 512 | 101.577854 |
| 256 | Decryption |  2.00 KB | 512 | 101.584152 |
| 256 | Encryption |  2.00 KB | 1024 | 103.240442 |
| 256 | Decryption |  2.00 KB | 1024 | 103.157563 |
| 256 | Encryption |  4.00 KB | 256 | 99.988557 |
| 256 | Decryption |  4.00 KB | 256 | 100.560065 |
| 256 | Encryption |  4.00 KB | 512 | 103.870416 |
| 256 | Decryption |  4.00 KB | 512 | 103.459653 |
| 256 | Encryption |  4.00 KB | 1024 | 105.420122 |
| 256 | Decryption |  4.00 KB | 1024 | 105.437082 |
| 256 | Encryption |  4.00 KB | 2048 | 106.442313 |
| 256 | Decryption |  4.00 KB | 2048 | 106.357669 |
| 256 | Encryption |  8.00 KB | 256 | 98.100809 |
| 256 | Decryption |  8.00 KB | 256 | 98.490393 |
| 256 | Encryption |  8.00 KB | 512 | 101.308167 |
| 256 | Decryption |  8.00 KB | 512 | 101.355171 |
| 256 | Encryption |  8.00 KB | 1024 | 102.898818 |
| 256 | Decryption |  8.00 KB | 1024 | 102.738314 |
| 256 | Encryption |  8.00 KB | 2048 | 103.573700 |
| 256 | Decryption |  8.00 KB | 2048 | 103.438017 |
| 256 | Encryption |  8.00 KB | 4096 | 103.684308 |
| 256 | Decryption |  8.00 KB | 4096 | 103.917356 |
| 256 | Encryption |  16.00 KB | 256 | 97.185216 |
| 256 | Decryption |  16.00 KB | 256 | 97.230995 |
| 256 | Encryption |  16.00 KB | 512 | 100.345466 |
| 256 | Decryption |  16.00 KB | 512 | 99.955579 |
| 256 | Encryption |  16.00 KB | 1024 | 101.752323 |
| 256 | Decryption |  16.00 KB | 1024 | 101.316977 |
| 256 | Encryption |  16.00 KB | 2048 | 102.466643 |
| 256 | Decryption |  16.00 KB | 2048 | 102.143245 |
| 256 | Encryption |  16.00 KB | 4096 | 102.804995 |
| 256 | Decryption |  16.00 KB | 4096 | 102.438214 |
| 256 | Encryption |  16.00 KB | 8192 | 103.106643 |
| 256 | Decryption |  16.00 KB | 8192 | 102.657647 |
| 256 | Encryption |  32.00 KB | 256 | 97.297488 |
| 256 | Decryption |  32.00 KB | 256 | 97.453747 |
| 256 | Encryption |  32.00 KB | 512 | 100.365347 |
| 256 | Decryption |  32.00 KB | 512 | 100.259403 |
| 256 | Encryption |  32.00 KB | 1024 | 101.792921 |
| 256 | Decryption |  32.00 KB | 1024 | 101.693015 |
| 256 | Encryption |  32.00 KB | 2048 | 102.528862 |
| 256 | Decryption |  32.00 KB | 2048 | 102.418903 |
| 256 | Encryption |  32.00 KB | 4096 | 102.889226 |
| 256 | Decryption |  32.00 KB | 4096 | 102.812757 |
| 256 | Encryption |  32.00 KB | 8192 | 103.157868 |
| 256 | Decryption |  32.00 KB | 8192 | 102.949230 |
| 256 | Encryption |  32.00 KB | 16384 | 103.214731 |
| 256 | Decryption |  32.00 KB | 16384 | 103.015781 |
| 128 | Encryption |  512.00 B | 256 | 88.941968 |
| 128 | Decryption |  512.00 B | 256 | 91.510277 |
| 128 | Encryption |  1024.00 B | 256 | 102.004732 |
| 128 | Decryption |  1024.00 B | 256 | 102.026964 |
| 128 | Encryption |  1024.00 B | 512 | 105.723688 |
| 128 | Decryption |  1024.00 B | 512 | 105.434538 |
| 128 | Encryption |  2.00 KB | 256 | 105.185780 |
| 128 | Decryption |  2.00 KB | 256 | 105.288863 |
| 128 | Encryption |  2.00 KB | 512 | 110.437801 |
| 128 | Decryption |  2.00 KB | 512 | 109.379798 |
| 128 | Encryption |  2.00 KB | 1024 | 112.146206 |
| 128 | Decryption |  2.00 KB | 1024 | 111.839995 |
| 128 | Encryption |  4.00 KB | 256 | 107.754914 |
| 128 | Decryption |  4.00 KB | 256 | 109.038575 |
| 128 | Encryption |  4.00 KB | 512 | 113.258676 |
| 128 | Decryption |  4.00 KB | 512 | 112.992129 |
| 128 | Encryption |  4.00 KB | 1024 | 115.068301 |
| 128 | Decryption |  4.00 KB | 1024 | 114.959304 |
| 128 | Encryption |  4.00 KB | 2048 | 116.112115 |
| 128 | Decryption |  4.00 KB | 2048 | 115.979578 |
| 128 | Encryption |  8.00 KB | 256 | 106.352491 |
| 128 | Decryption |  8.00 KB | 256 | 105.974572 |
| 128 | Encryption |  8.00 KB | 512 | 109.695156 |
| 128 | Decryption |  8.00 KB | 512 | 109.833037 |
| 128 | Encryption |  8.00 KB | 1024 | 111.550164 |
| 128 | Decryption |  8.00 KB | 1024 | 111.329401 |
| 128 | Encryption |  8.00 KB | 2048 | 112.689522 |
| 128 | Decryption |  8.00 KB | 2048 | 112.569029 |
| 128 | Encryption |  8.00 KB | 4096 | 112.945880 |
| 128 | Decryption |  8.00 KB | 4096 | 113.097426 |
| 128 | Encryption |  16.00 KB | 256 | 105.439839 |
| 128 | Decryption |  16.00 KB | 256 | 105.140849 |
| 128 | Encryption |  16.00 KB | 512 | 108.800757 |
| 128 | Decryption |  16.00 KB | 512 | 108.398099 |
| 128 | Encryption |  16.00 KB | 1024 | 110.506004 |
| 128 | Decryption |  16.00 KB | 1024 | 110.162358 |
| 128 | Encryption |  16.00 KB | 2048 | 111.226661 |
| 128 | Decryption |  16.00 KB | 2048 | 111.001535 |
| 128 | Encryption |  16.00 KB | 4096 | 111.824489 |
| 128 | Decryption |  16.00 KB | 4096 | 111.327746 |
| 128 | Encryption |  16.00 KB | 8192 | 111.900864 |
| 128 | Decryption |  16.00 KB | 8192 | 111.767753 |
| 128 | Encryption |  32.00 KB | 256 | 105.392043 |
| 128 | Decryption |  32.00 KB | 256 | 105.480462 |
| 128 | Encryption |  32.00 KB | 512 | 108.942621 |
| 128 | Decryption |  32.00 KB | 512 | 108.773557 |
| 128 | Encryption |  32.00 KB | 1024 | 110.685291 |
| 128 | Decryption |  32.00 KB | 1024 | 110.389784 |
| 128 | Encryption |  32.00 KB | 2048 | 111.518725 |
| 128 | Decryption |  32.00 KB | 2048 | 111.248256 |
| 128 | Encryption |  32.00 KB | 4096 | 111.900028 |
| 128 | Decryption |  32.00 KB | 4096 | 111.654810 |
| 128 | Encryption |  32.00 KB | 8192 | 112.061472 |
| 128 | Decryption |  32.00 KB | 8192 | 112.078720 |
| 128 | Encryption |  32.00 KB | 16384 | 112.416846 |
| 128 | Decryption |  32.00 KB | 16384 | 111.962159 |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Input Data Size   | Stream Size        | Performance (Mbps) |
|------------|-------------------|--------------------|--------------------|
| 256        |      32.00 KB     |        512        |      226.263672      |
| 256        |      32.00 KB     |        1024        |      246.168293      |
| 256        |      32.00 KB     |        2048        |      254.809933      |
| 256        |      32.00 KB     |        4096        |      261.844842      |
| 256        |      32.00 KB     |        8192        |      264.039141      |
| 256        |      32.00 KB     |        16384        |      265.934228      |
| 256        |      16.00 KB     |        512        |      223.764954      |
| 256        |      16.00 KB     |        1024        |      248.461240      |
| 256        |      16.00 KB     |        2048        |      261.138616      |
| 256        |      16.00 KB     |        4096        |      268.469133      |
| 256        |      16.00 KB     |        8192        |      272.408346      |
| 256        |      8.00 KB     |        512        |      226.364782      |
| 256        |      8.00 KB     |        1024        |      246.510316      |
| 256        |      8.00 KB     |        2048        |      258.593511      |
| 256        |      8.00 KB     |        4096        |      264.471348      |
| 256        |      4.00 KB     |        512        |      214.766508      |
| 256        |      4.00 KB     |        1024        |      232.479603      |
| 256        |      4.00 KB     |        2048        |      242.550750      |
| 256        |      2.00 KB     |        512        |      192.650949      |
| 256        |      2.00 KB     |        1024        |      207.339914      |
| 256        |      1024.00 B     |        512        |      159.563693      |
| 128        |      32.00 KB     |        512        |      227.305953      |
| 128        |      32.00 KB     |        1024        |      245.921194      |
| 128        |      32.00 KB     |        2048        |      256.630926      |
| 128        |      32.00 KB     |        4096        |      261.348846      |
| 128        |      32.00 KB     |        8192        |      264.461343      |
| 128        |      32.00 KB     |        16384        |      269.887754      |
| 128        |      16.00 KB     |        512        |      222.212427      |
| 128        |      16.00 KB     |        1024        |      245.858343      |
| 128        |      16.00 KB     |        2048        |      261.727853      |
| 128        |      16.00 KB     |        4096        |      270.382556      |
| 128        |      16.00 KB     |        8192        |      272.924518      |
| 128        |      8.00 KB     |        512        |      226.321787      |
| 128        |      8.00 KB     |        1024        |      245.761536      |
| 128        |      8.00 KB     |        2048        |      258.836075      |
| 128        |      8.00 KB     |        4096        |      264.845423      |
| 128        |      4.00 KB     |        512        |      215.437212      |
| 128        |      4.00 KB     |        1024        |      232.946487      |
| 128        |      4.00 KB     |        2048        |      243.185276      |
| 128        |      2.00 KB     |        512        |      193.755913      |
| 128        |      2.00 KB     |        1024        |      208.394810      |
| 128        |      1024.00 B     |        512        |      158.559954      |

### AES CMAC

- Software/Application used : test_dthe_aes_cmac_stream_unaligned
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | Size | Performance (Mbps) |
|------------|------|--------------------|
| 256 |  32.00 KB | 229.757397 |
| 256 |  16.00 KB | 232.206745 |
| 256 |  8.00 KB | 229.425613 |
| 256 |  4.00 KB | 217.172018 |
| 256 |  2.00 KB | 195.105686 |
| 256 |  1024.00 B | 161.881237 |
| 256 |  512.00 B | 120.603607 |
| 128 |  32.00 KB | 228.874265 |
| 128 |  16.00 KB | 231.103353 |
| 128 |  8.00 KB | 229.014729 |
| 128 |  4.00 KB | 218.187871 |
| 128 |  2.00 KB | 196.297849 |
| 128 |  1024.00 B | 163.106023 |
| 128 |  512.00 B | 122.031878 |

### AES CBC

- Software/Application used : test_dthe_aes_cbc_stream
- Code Placement            : OCRAM
- Data Placement            : OCRAM
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 400MHZ
| Key Length | operation  | Size | Stream Size | Performance (Mbps) |
|------------|------------|------|-------------|--------------------|
| 256 | Encryption |  512.00 B | 256 | 77.638250 |
| 256 | Decryption |  512.00 B | 256 | 83.814201 |
| 256 | Encryption |  1024.00 B | 256 | 92.265240 |
| 256 | Decryption |  1024.00 B | 256 | 93.961117 |
| 256 | Encryption |  1024.00 B | 512 | 95.810064 |
| 256 | Decryption |  1024.00 B | 512 | 96.620865 |
| 256 | Encryption |  2.00 KB | 256 | 95.265507 |
| 256 | Decryption |  2.00 KB | 256 | 97.011324 |
| 256 | Encryption |  2.00 KB | 512 | 100.686752 |
| 256 | Decryption |  2.00 KB | 512 | 100.801354 |
| 256 | Encryption |  2.00 KB | 1024 | 102.828990 |
| 256 | Decryption |  2.00 KB | 1024 | 102.764493 |
| 256 | Encryption |  4.00 KB | 256 | 99.604082 |
| 256 | Decryption |  4.00 KB | 256 | 100.258540 |
| 256 | Encryption |  4.00 KB | 512 | 103.631433 |
| 256 | Decryption |  4.00 KB | 512 | 103.197361 |
| 256 | Encryption |  4.00 KB | 1024 | 105.316781 |
| 256 | Decryption |  4.00 KB | 1024 | 105.135157 |
| 256 | Encryption |  4.00 KB | 2048 | 106.130315 |
| 256 | Decryption |  4.00 KB | 2048 | 106.051314 |
| 256 | Encryption |  8.00 KB | 256 | 97.991530 |
| 256 | Decryption |  8.00 KB | 256 | 97.735060 |
| 256 | Encryption |  8.00 KB | 512 | 101.373984 |
| 256 | Decryption |  8.00 KB | 512 | 100.953525 |
| 256 | Encryption |  8.00 KB | 1024 | 102.526155 |
| 256 | Decryption |  8.00 KB | 1024 | 102.532571 |
| 256 | Encryption |  8.00 KB | 2048 | 103.698254 |
| 256 | Decryption |  8.00 KB | 2048 | 103.240442 |
| 256 | Encryption |  8.00 KB | 4096 | 103.753250 |
| 256 | Decryption |  8.00 KB | 4096 | 103.888116 |
| 256 | Encryption |  16.00 KB | 256 | 97.329187 |
| 256 | Decryption |  16.00 KB | 256 | 97.159281 |
| 256 | Encryption |  16.00 KB | 512 | 100.239180 |
| 256 | Decryption |  16.00 KB | 512 | 99.823501 |
| 256 | Encryption |  16.00 KB | 1024 | 101.618608 |
| 256 | Decryption |  16.00 KB | 1024 | 101.250842 |
| 256 | Encryption |  16.00 KB | 2048 | 102.437414 |
| 256 | Decryption |  16.00 KB | 2048 | 102.072054 |
| 256 | Encryption |  16.00 KB | 4096 | 102.829595 |
| 256 | Decryption |  16.00 KB | 4096 | 102.369009 |
| 256 | Encryption |  16.00 KB | 8192 | 103.026409 |
| 256 | Decryption |  16.00 KB | 8192 | 102.632326 |
| 256 | Encryption |  32.00 KB | 256 | 97.460993 |
| 256 | Decryption |  32.00 KB | 256 | 97.447407 |
| 256 | Encryption |  32.00 KB | 512 | 100.346138 |
| 256 | Decryption |  32.00 KB | 512 | 100.156552 |
| 256 | Encryption |  32.00 KB | 1024 | 101.726756 |
| 256 | Decryption |  32.00 KB | 1024 | 101.675266 |
| 256 | Encryption |  32.00 KB | 2048 | 102.493585 |
| 256 | Decryption |  32.00 KB | 2048 | 102.367510 |
| 256 | Encryption |  32.00 KB | 4096 | 102.849061 |
| 256 | Decryption |  32.00 KB | 4096 | 102.755933 |
| 256 | Encryption |  32.00 KB | 8192 | 103.023878 |
| 256 | Decryption |  32.00 KB | 8192 | 102.992913 |
| 256 | Encryption |  32.00 KB | 16384 | 103.267182 |
| 256 | Decryption |  32.00 KB | 16384 | 103.047772 |
| 128 | Encryption |  512.00 B | 256 | 87.629031 |
| 128 | Decryption |  512.00 B | 256 | 90.344637 |
| 128 | Encryption |  1024.00 B | 256 | 101.445776 |
| 128 | Decryption |  1024.00 B | 256 | 101.270204 |
| 128 | Encryption |  1024.00 B | 512 | 104.894523 |
| 128 | Decryption |  1024.00 B | 512 | 104.556477 |
| 128 | Encryption |  2.00 KB | 256 | 104.829086 |
| 128 | Decryption |  2.00 KB | 256 | 106.301601 |
| 128 | Encryption |  2.00 KB | 512 | 109.228487 |
| 128 | Decryption |  2.00 KB | 512 | 109.683682 |
| 128 | Encryption |  2.00 KB | 1024 | 111.761797 |
| 128 | Decryption |  2.00 KB | 1024 | 111.626639 |
| 128 | Encryption |  4.00 KB | 256 | 109.034947 |
| 128 | Decryption |  4.00 KB | 256 | 108.723819 |
| 128 | Encryption |  4.00 KB | 512 | 112.846209 |
| 128 | Decryption |  4.00 KB | 512 | 112.819983 |
| 128 | Encryption |  4.00 KB | 1024 | 114.380460 |
| 128 | Decryption |  4.00 KB | 1024 | 114.690724 |
| 128 | Encryption |  4.00 KB | 2048 | 115.809470 |
| 128 | Decryption |  4.00 KB | 2048 | 115.716430 |
| 128 | Encryption |  8.00 KB | 256 | 105.456169 |
| 128 | Decryption |  8.00 KB | 256 | 106.243439 |
| 128 | Encryption |  8.00 KB | 512 | 109.569528 |
| 128 | Decryption |  8.00 KB | 512 | 109.738322 |
| 128 | Encryption |  8.00 KB | 1024 | 111.367238 |
| 128 | Decryption |  8.00 KB | 1024 | 111.284975 |
| 128 | Encryption |  8.00 KB | 2048 | 112.597556 |
| 128 | Decryption |  8.00 KB | 2048 | 112.091540 |
| 128 | Encryption |  8.00 KB | 4096 | 112.947827 |
| 128 | Decryption |  8.00 KB | 4096 | 112.895780 |
| 128 | Encryption |  16.00 KB | 256 | 105.249338 |
| 128 | Decryption |  16.00 KB | 256 | 105.324609 |
| 128 | Encryption |  16.00 KB | 512 | 108.696995 |
| 128 | Decryption |  16.00 KB | 512 | 108.447875 |
| 128 | Encryption |  16.00 KB | 1024 | 110.380372 |
| 128 | Decryption |  16.00 KB | 1024 | 110.115621 |
| 128 | Encryption |  16.00 KB | 2048 | 111.306002 |
| 128 | Decryption |  16.00 KB | 2048 | 110.936006 |
| 128 | Encryption |  16.00 KB | 4096 | 111.660636 |
| 128 | Decryption |  16.00 KB | 4096 | 111.353755 |
| 128 | Encryption |  16.00 KB | 8192 | 112.006529 |
| 128 | Decryption |  16.00 KB | 8192 | 111.601215 |
| 128 | Encryption |  32.00 KB | 256 | 105.242894 |
| 128 | Decryption |  32.00 KB | 256 | 105.510605 |
| 128 | Encryption |  32.00 KB | 512 | 108.870230 |
| 128 | Decryption |  32.00 KB | 512 | 108.679533 |
| 128 | Encryption |  32.00 KB | 1024 | 110.544565 |
| 128 | Decryption |  32.00 KB | 1024 | 110.378164 |
| 128 | Encryption |  32.00 KB | 2048 | 111.501649 |
| 128 | Decryption |  32.00 KB | 2048 | 111.210736 |
| 128 | Encryption |  32.00 KB | 4096 | 111.817215 |
| 128 | Decryption |  32.00 KB | 4096 | 111.728096 |
| 128 | Encryption |  32.00 KB | 8192 | 112.122343 |
| 128 | Decryption |  32.00 KB | 8192 | 111.925111 |
| 128 | Encryption |  32.00 KB | 16384 | 112.172240 |
| 128 | Decryption |  32.00 KB | 16384 | 112.129537 |


### Ethernet Performance

For Ethernet performance refer \ref enetlld_performance