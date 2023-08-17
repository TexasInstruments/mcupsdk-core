# Datasheet {#DATASHEET_AM243X_EVM}

[TOC]

## Introduction

This datasheet provides the performance numbers of various device drivers in MCU PLUS SDK for @VAR_SOC_NAME

## Generic Setup details

SOC Details             | Values
------------------------|------------------------------
Core                    | R5F
Core Operating Speed    | 800 MHz
Cache Status            | Enabled

Optimization Details    | Values
------------------------|------------------------------
Build Profile           | Release
R5F Compiler flags      | -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -Wall -Werror -g -mthumb -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function
R5F Linker flags        | -Wl,--diag_suppress=10063 -Wl,--ram_model -Wl,--reread_libs
Code Placement          | MSRAM
Data Placement          | MSRAM

## Performance Numbers

\cond SOC_AM64X
### SBL OSPI performance

### AM64X-EVM

- Software/Application used        : sbl_ospi and ipc_rpmsg_echo
- Size of sbl_ospi appimage        : 256 KB
- Size of ipc_rpmsg_echo appimage  : 146 KB
- Cores present in the appimage    : m4f0-0, r5f0-0, r5f0-1, r5f1-0, r5f1-1
- Boot Media Clock                 : 166.667 MHz (Octal DDR mode)

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   12.17
SBL : System Init                       |   0.57
SBL : Drivers_open                      |   0.27
SBL : Board_driversOpen                 |   22.15
SBL : Sciclient Get Version             |   9.01
SBL : CPU Load                          |   246.53
SBL : Total time taken                  |   302.7

- The time taken for Board_driversOpen (around 20 ms) is mostly for the PHY tuning of OSPI. If this needs to be further reduced, one can pre-train the PHY, note down delay values and set it directly instead of the tuning procedure.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory directly, this would be slower that mem to mem copy.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled
\endcond

\cond SOC_AM243X

### AM243X-EVM

- Software/Application used        : sbl_ospi and ipc_rpmsg_echo
- Size of sbl_ospi appimage        : 256 KB
- Size of ipc_rpmsg_echo appimage  : 146 KB
- Cores present in the appimage    : m4f0-0, r5f0-0, r5f0-1, r5f1-0, r5f1-1
- Boot Media Clock                 : 166.667 MHz (Octal DDR mode)

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   12.17
SBL : System Init                       |   0.57
SBL : Drivers_open                      |   0.27
SBL : Board_driversOpen                 |   22.15
SBL : Sciclient Get Version             |   9.01
SBL : CPU Load                          |   246.53
SBL : Total time taken                  |   302.7

- The time taken for Board_driversOpen (around 20 ms) is mostly for the PHY tuning of OSPI. If this needs to be further reduced, one can pre-train the PHY, note down delay values and set it directly instead of the tuning procedure.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory directly, this would be slower that mem to mem copy.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled

\endcond

### SBL SD performance

- Software/Application used        : sbl_sd and gpio_input_interrupt
- Size of sbl_sd appimage          : 300 KB
- Size of gpio_input_interrupt appimage  : 34 KB
- Cores present in the appimage    : r5f0-0
- SD card read speed               : 12.5 MBps mode

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   12.26
SBL : System Init                       |   29.46
SBL : Drivers_open                      |   78.35
SBL : Board_driversOpen                 |   0.00
SBL : Sciclient Get Version             |   10.02
SBL : File read from SD Card            |   6.48
SBL : CPU Load                          |   57.09
----------------------------------------|--------------
SBL : Total time taken                  |   205.66

- The MMCSD driver and SD card initialization is done as part of Drivers_open, so Board_driversOpen happens instantaneously.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- The CPU load is faster in this case because SD bootloader is at its core a memory bootloader. Appimage file from SD card is read into the memory (this takes 15 ms in this case). The CPU load now is mostly memcpy in the OCRAM. This would be quite fast. The only caveat here is that the buffer size to receive
appimage is allocated in OCRAM, so it's limited. This is not the case in OSPI.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled


### XIP Benchmark

- Software/Application used : xip_benchmark
- FIR operation

Caching status                          | Cycles taken
----------------------------------------|-------------
Code/Data fully cached                  |    26750
Code/Data not cached                    |    53539
Code/Data not cached 1 of 10 iterations |    29552

- MEMCPY operation

Caching status                          | Cycles taken
----------------------------------------|-------------
Code/Data fully cached                  |    1564
Code/Data not cached                    |    3472
Code/Data not cached 1 of 10 iterations |    1759

### MCAN performance

- Internal loopback mode of operation
- Memory Mode               : TX Buffer Mode, RX FIFO Mode
- MCAN CLK                  : 80MHz
- CPU                       : R5F


Frame Type              | Arbitration BitRate(Mbps) | Data BitRate(Mbps)  | Arb Phase bits  | Data Phase bits | Throughput (Msg/Sec)
------------------------|---------------------------|---------------------|-----------------|-----------------|----------------------------
CAN FD STANDARD FORMAT  | 1                         | 5                   |  27             | 538             | 6666
CAN FD EXTENDED FORMAT  | 1                         | 5                   |  46             | 538             | 5780

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
 400		| 08			| 20.69 Mbps / 154.63 us 	| 23.81 Mbps / 134.37 us 	|  0.91 Mbps / 3499.32 us
 200		| 16			| 32.77 Mbps / 97.66 us 	| 31.60 Mbps / 101.27 us 	|  0.95 Mbps / 3355.45 us
 100		| 32			| 39.14 Mbps / 81.76 us 	| 37.72 Mbps / 84.83 us 	|  0.97 Mbps / 3283.49 us


- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### CPSW Performance

#### TCP Test
- Software/Application used : enet_lwip_cpsw
- iperf test type           : TCP iperf
- iperf command used        : iperf -c 192.168.0.158 -i 10 -t 100 -r

#### TCP Performance(am64x/243x emvs):

 TCP direction           | B/W (Mb/s)                     | CPU load(%)
-------------------------|--------------------------------|--------------
 TCP RX                  | 93.4                           | 45.37
 TCP TX                  | 93.3                           | 57.42
 TCP Biderectional       | TX = 87 +  RX = 87             | 93.94

#### TCP Performance(am243x-lp):

 TCP direction           | B/W (Mb/s)                     | CPU load(%)
-------------------------|--------------------------------|--------------
 TCP RX                  | 93.35                          | 31.01
 TCP TX                  | 93.30                          | 34.92
 TCP Biderectional       | TX = 93.20 +  RX = 93.2        | 63.28

#### UDP Test
- Software/Application used : enet_lwip_cpsw
- iperf test type           : UDP iperf
- iperf command used        : iperf -c 192.168.0.158 -u -b 60M -l 256 -i 10 -t 100 -r

#### UDP TX Performance(am64x/243x emvs):

  - For Packet Size: 1500 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 100                            | 0                     | -
 50 Mbps                 | 100                            | 0.03                  | -
 100 Mbps                | 100                            | 0.07                  | -

#### UDP TX Performance(am243x-lp):

  - For Packet Size: 1500 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 100                            | 0                     | -
 50 Mbps                 | 100                            | 0                     | -
 100 Mbps                | 100                            | 0                     | -

#### UDP RX Performance(am64x/243x emvs):
- For Packet Size: 256 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 52.26                          | 0.08                  | -
 50 Mbps                 | 60.34                          | 41                    | -
 100 Mbps                | -                              | -                     | -

- For Packet Size: 512 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 30.83                          | 0                     | -
 50 Mbps                 | 56.74                          | 0.09                  | -
 100 Mbps                | 65.61                          | 41                    | -

- For Packet Size: 1500 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 16.84                          | 0                     | -
 50 Mbps                 | 28.93                          | 0                     | -
 100 Mbps                | 53.16                          | 0.06                  | -
 Max(158 Mbps)           | 80.83                          | 1.1                   | -

#### UDP RX Performance(am243x-lp):
- For Packet Size: 256 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 42                             | 0                     | -
 50 Mbps                 | 49                             | 41                    | -
 100 Mbps                | -                              | -                     | -

- For Packet Size: 512 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 24                             | 0                     | -
 50 Mbps                 | 45                             | 0.05                  | -
 100 Mbps                | 61                             | 41                    | -

- For Packet Size: 1500 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 12                             | 0                     | -
 50 Mbps                 | 21                             | 0                     | -
 100 Mbps                | 38                             | 0.03                  | -

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0	| m4f0-0	|  1.43
 r5f0-0	| r5f0-1	|  0.75
 r5f0-0	| r5f1-0	|  0.80
 r5f0-0	| r5f1-1	|  0.86
 r5f0-0	| a530-0	|  0.00
 r5f0-0	| a530-1	|  0.00

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (ns)
------------|-------------|--------------|------------------------------
 r5f0-0    | r5f0-1    | 4      | 0.636
 r5f0-0    | r5f1-0    | 4      | 0.634
 r5f0-0    | r5f1-1    | 4      | 0.633
 r5f0-0    | m4f0-0    | 4      | 1.174
 r5f0-0    | r5f0-1    | 32     | 0.841
 r5f0-0    | r5f0-1    | 64     | 1.068
 r5f0-0    | r5f0-1    | 112    | 1.454
 r5f0-0    | m4f0-0    | 32     | 1.703
 r5f0-0    | m4f0-0    | 64     | 2.310
 r5f0-0    | m4f0-0    | 112    | 3.212


### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 42			| 42.798000 		| 642			| 293.339996		|
cos  		|0.0000002870	| 53			| 53.736000 		| 1220			| 290.665985		|
sincos sin  	|0.0000001790	| 68			| 68.675995 		| 531			| 284.369995		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 63			| 63.538002 		| 1183			| 447.066010		|
acos 		|0.0000004770	| 64			| 64.534004 		| 1146			| 398.294006		|
atan 		|0.0000005360	| 72			| 72.758003 		| 740			| 390.471985		|
atan2 		|0.0000007150	| 112			| 103.040001 		| 1190			| 491.321991		|

### SA2UL

### AES CBC

- Software/Application used : test_sa2ul_aes
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 800MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption |  32.00 KB | 1501.150297 |
| 128 | Decryption |  32.00 KB | 1865.179612 |
| 128 | Encryption |  16.00 KB | 1391.773404 |
| 128 | Decryption |  16.00 KB | 1665.357982 |
| 128 | Encryption |  8.00 KB | 1180.376883 |
| 128 | Decryption |  8.00 KB | 1381.705099 |
| 128 | Encryption |  4.00 KB | 907.858009 |
| 128 | Decryption |  4.00 KB | 1011.826463 |
| 128 | Encryption |  2.00 KB | 654.378432 |
| 128 | Decryption |  2.00 KB | 711.728931 |
| 128 | Encryption |  1024.00 B | 407.917341 |
| 128 | Decryption |  1024.00 B | 430.336857 |
| 128 | Encryption |  512.00 B | 234.542982 |
| 128 | Decryption |  512.00 B | 236.865693 |
| 256 | Encryption |  32.00 KB | 1334.668966 |
| 256 | Decryption |  32.00 KB | 1521.682219 |
| 256 | Encryption |  16.00 KB | 1220.423888 |
| 256 | Decryption |  16.00 KB | 1375.686810 |
| 256 | Encryption |  8.00 KB | 1044.107221 |
| 256 | Decryption |  8.00 KB | 1156.475130 |
| 256 | Encryption |  4.00 KB | 828.782801 |
| 256 | Decryption |  4.00 KB | 888.894917 |
| 256 | Encryption |  2.00 KB | 615.708380 |
| 256 | Decryption |  2.00 KB | 635.500606 |
| 256 | Encryption |  1024.00 B | 393.019490 |
| 256 | Decryption |  1024.00 B | 399.853569 |
| 256 | Encryption |  512.00 B | 231.004582 |
| 256 | Decryption |  512.00 B | 228.014752 |

### PKA RSA SIGN VERIFY

- Software/Application used : test_sa2ul_pka
- Supported keys            : 4K and 2K
- CPU with operating speed  : R5F with 800MHZ
- OS used                   : nortos
| ECDSA            | Sign/sec  | Verify/sec  | Sign and verify/sec |
|------------------|-----------|-------------|---------------------|
| 2048 | 632 | 8849 | 589 |
| 4096 | 97 | 2793 | 94 |

### PKA ECDSA

- Software/Application used : test_sa2ul_pka
- Supported Curves          : p-256 and p-384
- CPU with operating speed  : R5F with 800MHZ
- OS used                   : nortos
| ECDSA            | Sign/sec  | Verify/sec  | Sign and verify/sec |
|------------------|-----------|-------------|---------------------|
| 256 | 1136 | 693 | 430 |
| 384 | 586 | 349 | 218 |

### RSA ENCRYPT DECRYPT

- Software/Application used : test_sa2ul_rsa
- Code Placement            : OCMC
- Supported keys            : 4K and 2K
- CPU with operating speed  : R5F with 800MHZ
- Software/Application used : test_sa2ul_rsa
| Key Length | operation  | Size | Performance(Mbps) |
|-------------|------------|------|-------------|
| 2048 | Encryption |  256.00 B | 0.554973 |
| 2048 | Decryption |  256.00 B | 1.294659 |
| 4096 | Encryption |  512.00 B | 0.597194 |
| 4096 | Decryption |  512.00 B | 0.400659 |

### SHA

- Software/Application used : test_sa2ul_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 800MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 |  32.00 KB | 2038.900610 |
| 512 |  16.00 KB | 1824.975199 |
| 512 |  8.00 KB | 1428.811250 |
| 512 |  4.00 KB | 981.298196 |
| 512 |  2.00 KB | 604.380504 |
| 512 |  1024.00 B | 346.220086 |
| 512 |  512.00 B | 182.612572 |
| 256 |  32.00 KB | 1592.443088 |
| 256 |  16.00 KB | 1415.063225 |
| 256 |  8.00 KB | 1163.223287 |
| 256 |  4.00 KB | 846.827756 |
| 256 |  2.00 KB | 550.029375 |
| 256 |  1024.00 B | 325.160010 |
| 256 |  512.00 B | 172.754112 |

### USB

### NET Driver ( RNDIS )

- Measures TCP only TX bandwidth at every 1 sec interval
using standard **lwip-perf** application.
- Software/Applicaiton used    : rndis_nortos
- Input media                  : USB 2.0
- Linux Host application used  : iperf
- CPU with operating speed     : R5F with 800MHz
- lwip-webserver ip address    : 192.168.7.1
- RNDIS Host ip address        : 192.168.7.2
- TCP window size              : 85.0 KByte

| ID  | Interval     | Transfer    | Bandwidth      |
|-----|--------------|-------------|----------------|
|  0  | 0.0- 1.0 sec | 4.25 MBytes | 35.7 Mbits/sec |
|  1  | 1.0- 2.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  2  | 2.0- 3.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  3  | 3.0- 4.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  4  | 4.0- 5.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  5  | 5.0- 6.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  6  | 6.0- 7.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  7  | 7.0- 8.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
|  8  | 8.0- 9.0 sec | 4.25 MBytes | 35.7 Mbits/sec |
|  9  | 9.0-10.0 sec | 4.12 MBytes | 34.6 Mbits/sec |
| 10  | 0.0-10.0 sec | 41.5 MBytes | 34.8 Mbits/sec |

