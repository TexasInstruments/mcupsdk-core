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
- Size of sbl_ospi appimage        : 325 KB
- Size of ipc_notify_echo appimage  : 185 KB
- Cores present in the appimage    : m4f0-0, r5f0-0, r5f0-1, r5f1-0, r5f1-1, a530-0
- Boot Media Clock                 : 166.667 MHz (Octal DDR mode)

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   11.11
SBL : System Init                       |   14.65
SBL : Drivers_open                      |   1.638
SBL : Board_driversOpen                 |   126.34
SBL : Sciclient Get Version             |   9.84
SBL : CPU Load                          |   34.25
SBL : Total time taken                  |   194.186

- The time taken for Board_driversOpen (around 20 ms) is mostly for the PHY tuning of OSPI. If this needs to be further reduced, one can pre-train the PHY, note down delay values and set it directly instead of the tuning procedure.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory directly, this would be slower that mem to mem copy.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled
\endcond

\cond SOC_AM243X

### AM243X-EVM

- Software/Application used        : sbl_ospi and ipc_notify_echo
- Size of sbl_ospi appimage        : 321 KB
- Size of ipc_notify_echo appimage  : 81 KB
- Cores present in the appimage    : m4f0-0, r5f0-0, r5f0-1, r5f1-0, r5f1-1
- Boot Media Clock                 : 166.667 MHz (Octal DDR mode)

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   11.095
SBL : System Init                       |   14.674
SBL : Drivers_open                      |   1.636
SBL : Board_driversOpen                 |   114.865
SBL : Sciclient Get Version             |   9.84
SBL : CPU Load                          |   24.66
SBL : Total time taken                  |   176.787

- The time taken for Board_driversOpen (around 20 ms) is mostly for the PHY tuning of OSPI. If this needs to be further reduced, one can pre-train the PHY, note down delay values and set it directly instead of the tuning procedure.

- Most of the time taken for the CPU load is in loading the M4F. Other CPUs take `~1.5ms` each. This needs more investigation. Possible reason is the M4F IRAM and DRAM initialization time.

- Here the CPU load or section copy takes place from the OSPI memory to the respective memory directly, this would be slower that mem to mem copy.

- The time taken for Sciclient Get Version can be avoided if the version check is disabled

\endcond

### SBL SD performance

- Software/Application used        : sbl_sd and adc_singleshot
- Size of sbl_sd appimage          : 339 KB
- Size of adc_singleshot appimage : 54 KB
- Cores present in the appimage    : r5f0-0
- SD card read speed               : 12.5 MBps mode

Boot time breakdown                     |   Time (ms)
----------------------------------------|--------------
ROM : init + SBL load from flash        |   12.00
SBL : SYSFW Load                        |   11.10
SBL : System Init                       |   14.234
SBL : Drivers_open                      |   76.724
SBL : Board_driversOpen                 |   0.00
SBL : Sciclient Get Version             |   9.844
SBL : File read from SD Card            |   5.580
SBL : CPU Load                          |   12.857
----------------------------------------|--------------
SBL : Total time taken                  |   130.351

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
Code/Data fully cached                  |    24004
Code/Data not cached                    |    47857
Code/Data not cached 1 of 10 iterations |    26389

- MEMCPY operation

Caching status                          | Cycles taken
----------------------------------------|-------------
Code/Data fully cached                  |    1564
Code/Data not cached                    |    3535
Code/Data not cached 1 of 10 iterations |    1772

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
 400		| 08			| 10.57 Mbps / 302.81 us 	| 23.57 Mbps / 135.76 us 	|  0.91 Mbps / 3502.12 us
 200		| 16			| 21.55 Mbps / 148.51 us 	| 31.18 Mbps / 102.62 us 	|  0.95 Mbps / 3358.21 us
 100		| 32			| 38.56 Mbps / 82.98 us 	| 37.16 Mbps / 86.12 us 	|  0.97 Mbps / 3286.31 us


- Theoretically for 400 Bytes at 50MHz time required for clocks is 64us.
- Additionally hardware adds 160ns + 1bit time delay between each word transfer as measured in the scope

### CPSW Performance

For CPSW performance refer \ref enetlld_performance


### ICSSG Performance

#### TCP Test
- Software/Application used : enet_lwip_icssg
- iperf test type           : TCP iperf
- iperf command used        : iperf -c 192.168.0.158 -i 10 -t 100 -r
- Packet allocation         : 32 buffers per each Rx flow, 16 buffers for Tx channel

#### TCP Performance(am64x/243x emvs):

 TCP direction           | B/W (Mb/s)                     | CPU load(%)
-------------------------|--------------------------------|--------------
 TCP RX                  | 93.2                           | 52.1
 TCP TX                  | 92.9                           | 87.57

#### TCP Performance(am243x-lp):

 TCP direction           | B/W (Mb/s)                     | CPU load(%)
-------------------------|--------------------------------|--------------
 TCP RX                  | 93.20                          | 36.82
 TCP TX                  | 93.30                          | 52.54

#### UDP Test
- Software/Application used : enet_lwip_icssg
- iperf test type           : UDP iperf
- iperf command used        : iperf -c 192.168.0.158 -u -b 60M -l 256 -i 10 -t 100 -r
- Packet allocation         : 32 buffers per each Rx flow, 16 buffers for Tx channel

#### UDP TX Performance(am64x/243x emvs):

  - For Packet Size: 1500 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 100                            | 0                     | -
 50 Mbps                 | 100                            | 0.03                  | -
 100 Mbps                | 100                            | 0.07                  | -
 Max(220 Mbps)           | 100                            | 0.07                  | -

#### UDP TX Performance(am243x-lp):

  - For Packet Size: 1500 B

 Tx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 100                            | 0                     | -
 50 Mbps                 | 100                            | 0                     | -
 100 Mbps                | 100                            | 0                     | -

#### UDP RX Performance(am64x/243x emvs):
- For Packet Size: 256 B

 Rx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 44.35                          | 0.11                  | -
 50 Mbps                 | 60.34                          | 41                    | -
 100 Mbps                | -                              | -                     | -

- For Packet Size: 512 B

 Rx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 21.83                          | 0                     | -
 50 Mbps                 | 56.74                          | 0.09                  | -
 100 Mbps                | 65.61                          | 41                    | -

- For Packet Size: 1500 B

 Rx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 16.84                          | 0                     | -
 50 Mbps                 | 28.93                          | 0                     | -
 90 Mbps                 | 39                             | 0.7                  | -
 

#### UDP RX Performance(am243x-lp):
- For Packet Size: 256 B

 Rx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 37.35                          | 0.12                  | -
 50 Mbps                 | 48                             | 36                    | -
 100 Mbps                | -                              | -                     | -

- For Packet Size: 512 B

 Rx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 21.83                          | 0                     | -
 50 Mbps                 | 40                             | 0.12                  | -
 100 Mbps                | 50                             | 36                    | -

- For Packet Size: 1500 B

 Rx Side BW/Packet Size  | CPU load(%)                    | Packet loss(%)        | Latency
-------------------------|--------------------------------|-----------------------|-----------
 25 Mbps                 | 15                             | 0                     | -
 50 Mbps                 | 26.5                           | 0.08                  | -
 89 Mbps                 | 33                             | 0.1                   | -

### IPC performance

#### IPC NOTIFY

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Average Message Latency (us)
------------|-------------|------------------------------
 r5f0-0	| m4f0-0	|  1.72
 r5f0-0	| r5f0-1	|  0.80
 r5f0-0	| r5f1-0	|  0.86
 r5f0-0	| r5f1-1	|  0.92
 r5f0-0	| a530-0	|  1.03
 r5f0-0	| a530-1	|  0.00


#### IPC RPMSG

- 10000 messages are sent and average one way message latency is measured

Local Core  | Remote Core | Message Size | Average Message Latency (us)
------------|-------------|--------------|------------------------------
 r5f0-0	| r5f0-1	| 32	| 8.609
 r5f0-0	| r5f0-1	| 64	| 11.141
 r5f0-0	| r5f0-1	| 112	| 14.970
 r5f0-0	| m4f0-0	| 32	| 17.766
 r5f0-0	| m4f0-0	| 64	| 23.682
 r5f0-0	| m4f0-0	| 112	| 32.675
 r5f0-0	| a530-0	| 32	| 7.873
 r5f0-0	| a530-0	| 64	| 9.296
 r5f0-0	| a530-0	| 112	| 11.473

### MATHLIB

### MATHLIB BENCHMARK

- Calculated for the 500 samples taken between 0 and 2 * Pi
- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and the compiler mathlib version
- The max error for each operation between the optimized Mathlib mcusdk functions and the compiler mathlib version is printed
Function	| Err		| Max Cycles Mathlib (mcusdk) 	| avg cycles Mathlib (mcusdk) 	| max cycles mathlib (clang) 	| avg cycles mathlib (clang) 	|
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin 		|0.0000007150	| 52			| 52.203999 		| 765			| 278.556000		|
cos  		|0.0000002870	| 66			| 66.015999 		| 971			| 280.813995		|
sincos sin  	|0.0000001790	| 79			| 79.103996 		| 467			| 275.118011		|
sincos cos	|0.0000001900	|			|			|			|			|
asin 		|0.0000003430	| 74			| 74.000000 		| 1225			| 431.264008		|
acos 		|0.0000004770	| 76			| 76.096001 		| 565			| 384.160004		|
atan 		|0.0000005360	| 80			| 80.001999 		| 652			| 372.014008		|
atan2 		|0.0000007150	| 117			| 104.702003 		| 608			| 480.313995		|

### SA2UL

### PKA RSA SIGN VERIFY

- Software/Application used : test_sa2ul_pka
- Supported keys            : 4K and 2K
- CPU with operating speed  : R5F with 800MHZ
- OS used                   : nortos
| ECDSA            | Sign/sec  | Verify/sec  | Sign and verify/sec |
|------------------|-----------|-------------|---------------------|
| 2048 | 631 | 8771 | 589 |
| 4096 | 97 | 2770 | 94 |

### SHA

- Software/Application used : test_sa2ul_sha
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 800MHZ
| SHA | Size | Performance (Mbps) |
|-----|------|-------------|
| 512 | 32.00 KB | 2036.504885 |
| 512 | 16.00 KB | 1803.412218 |
| 512 | 8.00 KB | 1394.679719 |
| 512 | 4.00 KB | 946.299906 |
| 512 | 2.00 KB | 570.995426 |
| 512 | 1024.00 B | 328.303777 |
| 512 | 512.00 B | 173.983222 |
| 256 | 32.00 KB | 1575.656852 |
| 256 | 16.00 KB | 1400.136198 |
| 256 | 8.00 KB | 1139.211681 |
| 256 | 4.00 KB | 821.278862 |
| 256 | 2.00 KB | 529.284445 |
| 256 | 1024.00 B | 311.187085 |
| 256 | 512.00 B | 168.942050 |

### PKA ECDSA

- Software/Application used : test_sa2ul_pka
- Supported Curves          : p-256 and p-384
- CPU with operating speed  : R5F with 800MHZ
- OS used                   : nortos
| ECDSA            | Sign/sec  | Verify/sec  | Sign and verify/sec |
|------------------|-----------|-------------|---------------------|
| 256 | 1135 | 693 | 430 |
| 384 | 585 | 349 | 218 |

### AES

- Software/Application used : test_sa2ul_aes
- Code Placement            : OCMC
- Data Placement            : OCMC
- Input Data sizes          : 512B, 1KB, 2KB, 4KB, 8KB, 16KB and 32KB
- CPU with operating speed  : R5F with 800MHZ
| Key Length | operation  | Size | Performance (Mbps) |
|-------------|------------|------|-------------|
| 128 | GHASH generation | 0.03 MB | 11004.050792 |
| 128 | Encryption | 0.03 MB | 959.154066 |
| 128 | Decryption | 0.03 MB | 1222.080942 |
| 128 | GHASH generation | 0.02 MB | 7113.812754 |
| 128 | Encryption | 0.02 MB | 884.836927 |
| 128 | Decryption | 0.02 MB | 1131.564975 |
| 128 | GHASH generation | 0.01 MB | 3634.326910 |
| 128 | Encryption | 0.01 MB | 780.097607 |
| 128 | Decryption | 0.01 MB | 974.947932 |
| 128 | GHASH generation | 0.00 MB | 1908.861866 |
| 128 | Encryption | 0.00 MB | 645.468200 |
| 128 | Decryption | 0.00 MB | 782.753061 |
| 128 | GHASH generation | 0.00 MB | 976.909890 |
| 128 | Encryption | 0.00 MB | 478.557085 |
| 128 | Decryption | 0.00 MB | 555.884473 |
| 128 | GHASH generation | 1024.00 B | 488.709918 |
| 128 | Encryption | 1024.00 B | 310.995112 |
| 128 | Decryption | 1024.00 B | 352.211533 |
| 128 | GHASH generation | 512.00 B | 248.204817 |
| 128 | Encryption | 512.00 B | 188.192051 |
| 128 | Decryption | 512.00 B | 201.376598 |
| 256 | GHASH generation | 0.03 MB | 14435.242291 |
| 256 | Encryption | 0.03 MB | 865.894011 |
| 256 | Decryption | 0.03 MB | 1061.951276 |
| 256 | GHASH generation | 0.02 MB | 7234.552229 |
| 256 | Encryption | 0.02 MB | 794.959933 |
| 256 | Decryption | 0.02 MB | 993.082547 |
| 256 | GHASH generation | 0.01 MB | 3655.103179 |
| 256 | Encryption | 0.01 MB | 708.372853 |
| 256 | Decryption | 0.01 MB | 870.303111 |
| 256 | GHASH generation | 0.00 MB | 1863.538779 |
| 256 | Encryption | 0.00 MB | 593.126230 |
| 256 | Decryption | 0.00 MB | 713.258782 |
| 256 | GHASH generation | 0.00 MB | 988.625735 |
| 256 | Encryption | 0.00 MB | 448.784496 |
| 256 | Decryption | 0.00 MB | 522.032818 |
| 256 | GHASH generation | 1024.00 B | 502.885206 |
| 256 | Encryption | 1024.00 B | 298.882656 |
| 256 | Decryption | 1024.00 B | 336.237238 |
| 256 | GHASH generation | 512.00 B | 245.361288 |
| 256 | Encryption | 512.00 B | 177.826016 |
| 256 | Decryption | 512.00 B | 195.117304 |

### RSA ENCRYPT DECRYPT

- Software/Application used : test_sa2ul_rsa
- Code Placement            : OCMC
- Supported keys            : 4K and 2K
- CPU with operating speed  : R5F with 800MHZ
- Software/Application used : test_sa2ul_rsa
| Key Length | operation  | Size | Performance(Mbps) |
|-------------|------------|------|-------------|
| 2048 | Encryption | 256.00 B | 0.263871 |
| 2048 | Decryption | 256.00 B | 1.294870 |
| 4096 | Encryption | 512.00 B | 0.512152 |
| 4096 | Decryption | 512.00 B | 0.400639 |

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
