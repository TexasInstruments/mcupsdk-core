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

- Software/Application used        : sbl_ospi and ipc_notify_echo
- Size of sbl_ospi appimage        : 256 KB
- Size of ipc_notify_echo appimage  : 77 KB
- Cores present in the appimage    : m4f0-0, r5f0-0, r5f0-1, r5f1-0, r5f1-1
- Boot Media Clock                 : 166.667 MHz (Octal DDR mode)

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   12.159
SBL : System Init                       |   0.553
SBL : Drivers_open                      |   0.280
SBL : Board_driversOpen                 |   22.23
SBL : Sciclient Get Version             |   9.9
SBL : CPU Load                          |   38.25
SBL : Total time taken                  |   95.372

- The time taken for Board_driversOpen (around 20 ms) is mostly for the PHY tuning of OSPI. If this needs to be further reduced, one can pre-train the PHY, note down delay values and set it directly instead of the tuning procedure.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory directly, this would be slower that mem to mem copy.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled
\endcond

\cond SOC_AM243X

### AM243X-EVM

- Software/Application used        : sbl_ospi and ipc_notify_echo
- Size of sbl_ospi appimage        : 256 KB
- Size of ipc_notify_echo appimage  : 77 KB
- Cores present in the appimage    : m4f0-0, r5f0-0, r5f0-1, r5f1-0, r5f1-1
- Boot Media Clock                 : 166.667 MHz (Octal DDR mode)

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   12.159
SBL : System Init                       |   0.553
SBL : Drivers_open                      |   0.280
SBL : Board_driversOpen                 |   22.23
SBL : Sciclient Get Version             |   9.9
SBL : CPU Load                          |   38.25
SBL : Total time taken                  |   95.372

- The time taken for Board_driversOpen (around 20 ms) is mostly for the PHY tuning of OSPI. If this needs to be further reduced, one can pre-train the PHY, note down delay values and set it directly instead of the tuning procedure.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory directly, this would be slower that mem to mem copy.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled

\endcond

### SBL SD performance

- Software/Application used        : sbl_sd and adc_singleshot
- Size of sbl_sd appimage          : 300 KB
- Size of adc_singleshot appimage : 34 KB
- Cores present in the appimage    : r5f0-0
- SD card read speed               : 12.5 MBps mode

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   12.161
SBL : System Init                       |   0.131
SBL : Drivers_open                      |   13.958
SBL : Board_driversOpen                 |   0.00
SBL : Sciclient Get Version             |   9.892
SBL : File read from SD Card            |   6.48
SBL : CPU Load                          |   57.09
----------------------------------------|--------------
SBL : Total time taken                  |   111.712

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
Code/Data fully cached                  |    26710
Code/Data not cached                    |    178717
Code/Data not cached 1 of 10 iterations |    42210

- MEMCPY operation

Caching status                          | Cycles taken
----------------------------------------|-------------
Code/Data fully cached                  |    1564
Code/Data not cached                    |    6284
Code/Data not cached 1 of 10 iterations |    2043

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
400        | 08            | 12.91 Mbps / 247.78 us     | 23.74 Mbps / 134.81 us     |  0.91 Mbps / 3500.67 us
200        | 16            | 26.22 Mbps / 122.05 us     | 31.49 Mbps / 101.60 us     |  0.95 Mbps / 3356.72 us
100        | 32            | 38.84 Mbps / 82.40 us     | 37.58 Mbps / 85.15 us     |  0.97 Mbps / 3284.85 us


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
r5f0-0    | m4f0-0    |  1.76
r5f0-0    | r5f0-1    |  0.77
r5f0-0    | r5f1-0    |  0.83
r5f0-0    | r5f1-1    |  0.89
r5f0-0    | a530-0    |  1.03
r5f0-0    | a530-1    |  0.00

#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
r5f0-0    | r5f0-1    | 32    | 7.870
r5f0-0    | r5f0-1    | 64    | 10.416
r5f0-0    | r5f0-1    | 112    | 14.273
r5f0-0    | m4f0-0    | 32    | 17.348
r5f0-0    | m4f0-0    | 64    | 23.326
r5f0-0    | m4f0-0    | 112    | 32.296
r5f0-0    | a530-0    | 32    | 7.239
r5f0-0    | a530-0    | 64    | 8.545
r5f0-0    | a530-0    | 112    | 10.751

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150    | 52            | 52.250000         | 659            | 277.497986        |
cos          |0.0000002870    | 64            | 63.896000         | 593            | 278.550018        |
sincos sin      |0.0000001790    | 79            | 79.094002         | 467            | 274.223999        |
sincos cos    |0.0000001900    |            |            |            |            |
asin         |0.0000003430    | 73            | 73.096001         | 1271            | 431.056000        |
acos         |0.0000004770    | 74            | 74.194000         | 1075            | 385.683990        |
atan         |0.0000005360    | 85            | 85.085999         | 623            | 371.787994        |
atan2         |0.0000007150    | 120            | 107.704002         | 660            | 478.279999        |

\cond SOC_AM243X
- Function	| Err		| Max Cycles Mathlib (fastrts) 	| avg cycles Mathlib (fastrts) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000004170	| 51			| 51.048000 		| 517			| 275.503998		|
cos  		|0.0000003870	| 54			| 54.015999 		| 512			| 276.832001		|
sincos sin  	|0.0000004170	| 58			| 58.084000 		| 483			| 274.227997		|
sincos cos	|0.0000003750	|			|			|			|			|
asin 		|0.0000003430	| 75			| 75.038002 		| 578			| 428.697998		|
acos 		|0.0000004770	| 76			| 76.033997 		| 810			| 383.730011		|
atan 		|0.0000001190	| 70			| 70.113998 		| 493			| 371.148010		|
atan2 		|0.0000002380	| 109			| 109.122002 		| 729			| 478.171997		|
\endcond

### SA2UL

### PKA RSA SIGN VERIFY

- Software/Application used : test_sa2ul_pka
- Supported keys            : 4K and 2K
- CPU with operating speed  : R5F with 800MHZ
- OS used                   : nortos
| ECDSA            | Sign/sec  | Verify/sec  | Sign and verify/sec |
|------------------|-----------|-------------|---------------------|
| 2048 | 632 | 8849 | 589 |
| 4096 | 97 | 2801 | 94 |

### SHA

- Software/Application used : test_sa2ul_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 800MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 | 32.00 KB | 2042.136834 |
| 512 | 16.00 KB | 1799.234716 |
| 512 | 8.00 KB | 1401.427388 |
| 512 | 4.00 KB | 954.153017 |
| 512 | 2.00 KB | 579.451813 |
| 512 | 1024.00 B | 331.040057 |
| 512 | 512.00 B | 176.086840 |
| 256 | 32.00 KB | 1586.191978 |
| 256 | 16.00 KB | 1406.767018 |
| 256 | 8.00 KB | 1142.388983 |
| 256 | 4.00 KB | 829.254713 |
| 256 | 2.00 KB | 537.951980 |
| 256 | 1024.00 B | 316.537867 |
| 256 | 512.00 B | 172.299926 |

### PKA ECDSA

- Software/Application used : test_sa2ul_pka
- Supported Curves          : p-256 and p-384
- CPU with operating speed  : R5F with 800MHZ
- OS used                   : nortos
| ECDSA            | Sign/sec  | Verify/sec  | Sign and verify/sec |
|------------------|-----------|-------------|---------------------|
| 256 | 1136 | 693 | 430 |
| 384 | 586 | 349 | 218 |

### AES

- Software/Application used : test_sa2ul_aes
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 800MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | Encryption | 32.00 KB | 1402.083250 |
| 128 | Decryption | 32.00 KB | 1820.570873 |
| 128 | Encryption | 16.00 KB | 1273.409113 |
| 128 | Decryption | 16.00 KB | 1618.172840 |
| 128 | Encryption | 8.00 KB | 1070.259457 |
| 128 | Decryption | 8.00 KB | 1338.015517 |
| 128 | Encryption | 4.00 KB | 839.989746 |
| 128 | Decryption | 4.00 KB | 981.445152 |
| 128 | Encryption | 2.00 KB | 590.574029 |
| 128 | Decryption | 2.00 KB | 674.204002 |
| 128 | Encryption | 1024.00 B | 371.856559 |
| 128 | Decryption | 1024.00 B | 394.154087 |
| 128 | Encryption | 512.00 B | 217.698645 |
| 128 | Decryption | 512.00 B | 217.915808 |
| 256 | Encryption | 32.00 KB | 1324.377644 |
| 256 | Decryption | 32.00 KB | 1507.473565 |
| 256 | Encryption | 16.00 KB | 1206.341318 |
| 256 | Decryption | 16.00 KB | 1361.150631 |
| 256 | Encryption | 8.00 KB | 1037.885777 |
| 256 | Decryption | 8.00 KB | 1137.456881 |
| 256 | Encryption | 4.00 KB | 822.928897 |
| 256 | Decryption | 4.00 KB | 874.804779 |
| 256 | Encryption | 2.00 KB | 585.875201 |
| 256 | Decryption | 2.00 KB | 615.910906 |
| 256 | Encryption | 1024.00 B | 374.555638 |
| 256 | Decryption | 1024.00 B | 382.200968 |
| 256 | Encryption | 512.00 B | 211.161232 |
| 256 | Decryption | 512.00 B | 208.288838 |

### RSA ENCRYPT DECRYPT

- Software/Application used : test_sa2ul_rsa
- Code Placement            : OCMC
- Supported keys            : 4K and 2K
- CPU with operating speed  : R5F with 800MHZ
- Software/Application used : test_sa2ul_rsa
| Key Length | operation  | Size | Performance(Mbps) |
|-------------|------------|------|-------------|
| 2048 | Encryption | 256.00 B | 0.554899 |
| 2048 | Decryption | 256.00 B | 1.295114 |
| 4096 | Encryption | 512.00 B | 0.597249 |
| 4096 | Decryption | 512.00 B | 0.400690 |

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

