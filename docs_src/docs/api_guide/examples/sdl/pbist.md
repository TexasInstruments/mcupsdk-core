# SDL PBIST {#EXAMPLES_SDL_PBIST}

[TOC]

# Introduction

This example demonstrates the usage of the PBIST module. The example shows how to setup and use the PBIST controller.
The example configures the algorithm and memory group. Example also prints the time taken for test execution.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Configure wrong combination of algorithm and memory group.
 UC-2     | Configure correct combination of algorithm and memory group.

# Supported Combinations {#EXAMPLES_SDL_PBIST_COMBOS}

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/pbist/pbist_mcu/

\endcond

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^				| m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/pbist/pbist_mpu/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_PBIST_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond SOC_AM263X || SOC_AM263PX
\code
[Cortex_R5_0]
PBIST Application

 Starting PBIST failure insertion test on TOP PBIST
 PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 47 micro secs

 Starting PBIST test on TOP PBIST
 PBIST complete for R5 STC
 PBIST complete for R51 STC
 PBIST complete for PBISTROM
 PBIST complete for CPSW
 PBIST complete for ICSSM
 PBIST complete for MBOX
 PBIST complete for MCAN
 PBIST complete for TPCC
 PBIST complete for MSS_L2_1
 PBIST complete for MSS_L2_2
 PBIST complete for MSS_L2_3
 PBIST complete for VIM1 R5SS0
 PBIST complete for VIM0 R5SS1
 PBIST complete for VIM1 R5SS1
 PBIST complete for R5SS1 RAM
 PBIST complete for MSS CR5B ATCM0
 PBIST complete for MSS CR5B ATCM1
 PBIST complete for MSS CR5B BTCM0
 PBIST complete for MSS CR5B BTCM1

 All tests have passed.

 [Cortex_R5_2]
  PBIST Application

  Starting PBIST failure insertion test on TOP PBIST
  PBIST failure Insertion test complete for TOP BIST
 PBIST Failure Insertion Test completed in 0 micro secs

  Starting PBIST test on TOP PBIST
  PBIST complete for  VIM0 R5SS0
  PBIST complete for MSS_L2_0
  PBIST complete for R5SS0 RAM
  PBIST complete for CR5A ROM0
  PBIST complete for TRACE
  PBIST complete for MMCH0
  PBIST complete for MSS CR5A ATCM0
  PBIST complete for MSS CR5A ATCM1
  PBIST complete for MSS CR5A BTCM0
  PBIST complete for MSS CR5A BTCM1

  All tests have passed.
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code
[Cortex_R5_0]
PBIST Application

Starting PBIST failure insertion test on TOP PBIST
PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 19 micro secs

Starting PBIST test on TOP PBIST
PBIST complete for ADCBUF
PBIST complete for TPCC
PBIST complete for MAILBOX
PBIST complete for COREB VIM
PBIST complete for MCAN
PBIST complete for SPIA
PBIST complete for SPIB
PBIST complete for CORE B R5FSS RAM
PBIST complete for MSS_L2_1
PBIST complete for CPSW
PBIST complete for GPADC
PBIST complete for RETRAM
PBIST complete for STCROM
PBIST complete for CORE B ATCM
PBIST complete for CORE B BTCM

All tests have passed.

Starting PBIST failure insertion test on TOP PBIST
PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 14 micro secs

Starting PBIST test on DSP PBIST
PBIST complete for DSS C66 STCROM
PBIST complete for HWA STCROM
PBIST complete for DSS PBISTROM
PBIST complete for C66 L1D
PBIST complete for C66 L1P
PBIST complete for PBIST C66 L2 TAG
PBIST complete for DSS HWA
PBIST complete for DSS HWA MBOX
PBIST complete for PBIST DSS L3 BANKA SUB0
PBIST complete for PBIST DSS L3 BANKB SUB0
PBIST complete for PBIST DSS L3 BANKC SUB0
PBIST complete for DSS MBOX RAM
PBIST complete for DSS TPCC RAM
PBIST complete for DSS L2 BANK0
PBIST complete for DSS L2 BANK1
PBIST complete for DSS L2 PARITY
PBIST complete for HWA RAM
PBIST complete for DSS CBUF
PBIST complete for PBIST DSS L3 BANKA SUB1
PBIST complete for PBIST DSS L3 BANKB SUB1
PBIST complete for PBIST DSS L3 BANKB SUB2
PBIST complete for PBIST DSS L3 BANKC SUB1

All tests have passed.

[Cortex_R5_1]
 PBIST Application

 Starting PBIST failure insertion test on TOP PBIST
 PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 19 micro secs

 Starting PBIST test on TOP PBIST
 PBIST complete for MSS_TCMAROM_0
 PBIST complete for MSS_TCMAROM_1
 PBIST complete for PBISTROM
 PBIST complete for CORE A VIM
 PBIST complete for MSS_L2_0
 PBIST complete for CORE A ATCM
 PBIST complete for CORE A BTCM
 PBIST complete for CORE A R5SS RAM
 PBIST complete for MEM_TOP_AURORA
 PBIST complete for MEM_TOP_MDO
 PBIST complete for DBGSS_TRACE

All tests have passed.
\endcode
\endcond

\cond SOC_AM64X
\code
[BLAZAR_Cortex_M4F_0]
PBIST Test Application

 Starting PBIST failure insertion test on Pulsar Instance 0, index 0...
  Delta Cores prep time in micro secs 1113
  Delta PBIST execution time in micro secs 6
  Delta Cores restore time in micro secs 1078
 PBIST complete Pulsar Instance 0, test index 0

 Starting PBIST failure insertion test on Pulsar Instance 1, index 1...
  Delta Cores prep time in micro secs 1107
  Delta PBIST execution time in micro secs 6
  Delta Cores restore time in micro secs 1062
 PBIST complete Pulsar Instance 1, test index 1

 Starting PBIST failure insertion test on MPU PBIST, index 2...
  Delta Cores prep time in micro secs 809
  Delta PBIST execution time in micro secs 7
  Delta Cores restore time in micro secs 1604
 PBIST complete MPU PBIST, test index 2

 Starting PBIST failure insertion test on Infra PBIST, index 3...
  Delta Cores prep time in micro secs 4225
  Delta PBIST execution time in micro secs 8
  Delta Cores restore time in micro secs 4200
 PBIST complete Infra PBIST, test index 3

 Starting PBIST test on Pulsar Instance 0, index 0...
  Delta Cores prep time in micro secs 1181
  Delta PBIST execution time in micro secs 3554
  Delta Cores restore time in micro secs 1120
 PBIST complete Pulsar Instance 0, test index 0

 Starting PBIST test on Pulsar Instance 1, index 1...
  Delta Cores prep time in micro secs 1188
  Delta PBIST execution time in micro secs 3554
  Delta Cores restore time in micro secs 1104
 PBIST complete Pulsar Instance 1, test index 1

 Starting PBIST test on MPU PBIST, index 2...
  Delta Cores prep time in micro secs 850
  Delta PBIST execution time in micro secs 29979
  Delta Cores restore time in micro secs 1711
 PBIST complete MPU PBIST, test index 2

 Starting PBIST test on Infra PBIST, index 3...
  Delta Cores prep time in micro secs 9839
  Delta PBIST execution time in micro secs 92768
  Delta Cores restore time in micro secs 7905
 PBIST complete Infra PBIST, test index 3

 PBIST Functionality Test Passed.

All tests have passed.

[MAIN_Cortex_R5_0_0]
PBIST Test Application

 Starting PBIST failure insertion test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 1106
  Delta PBIST execution time in micro secs 18
  Delta Cores restore time in micro secs 1223
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST failure insertion test on MPU PBIST, index 1...
  Delta Cores prep time in micro secs 810
  Delta PBIST execution time in micro secs 17
  Delta Cores restore time in micro secs 1606
 PBIST complete MPU PBIST, test index 1

 Starting PBIST failure insertion test on Infra PBIST, index 2...
  Delta Cores prep time in micro secs 4237
  Delta PBIST execution time in micro secs 24
  Delta Cores restore time in micro secs 4184
 PBIST complete Infra PBIST, test index 2

 Starting PBIST test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 1386
  Delta PBIST execution time in micro secs 3560
  Delta Cores restore time in micro secs 1264
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST test on MPU PBIST, index 1...
  Delta Cores prep time in micro secs 864
  Delta PBIST execution time in micro secs 29983
  Delta Cores restore time in micro secs 1720
 PBIST complete MPU PBIST, test index 1

 Starting PBIST test on Infra PBIST, index 2...
  Delta Cores prep time in micro secs 9725
  Delta PBIST execution time in micro secs 92779
  Delta Cores restore time in micro secs 7854
 PBIST complete Infra PBIST, test index 2

 PBIST Functionality Test Passed.

All tests have passed.

\endcode
\endcond

\cond SOC_AM62X
\code
[MAIN_Cortex_R5_0_0]
PBIST Test Application

 Starting PBIST failure insertion test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 889
  Delta PBIST execution time in micro secs 18
  Delta Cores restore time in micro secs 994
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST failure insertion test on Infra PBIST, index 1...
  Delta Cores prep time in micro secs 3493
  Delta PBIST execution time in micro secs 25
  Delta Cores restore time in micro secs 2792
 PBIST complete Infra PBIST, test index 1

 Starting PBIST test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 1153
  Delta PBIST execution time in micro secs 3560
  Delta Cores restore time in micro secs 1029
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST test on Infra PBIST, index 1...
  Delta Cores prep time in micro secs 8968
  Delta PBIST execution time in micro secs 92779
  Delta Cores restore time in micro secs 8453
 PBIST complete Infra PBIST, test index 1

 PBIST Functionality Test Passed.

All tests have passed.

\endcode
\endcond

\cond SOC_AM243X
\code
[BLAZAR_Cortex_M4F_0]
PBIST Test Application

 Starting PBIST failure insertion test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 798
  Delta PBIST execution time in micro secs 6
  Delta Cores restore time in micro secs 754
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST failure insertion test on Infra PBIST, index 1...
  Delta Cores prep time in micro secs 3204
  Delta PBIST execution time in micro secs 8
  Delta Cores restore time in micro secs 3650
 PBIST complete Infra PBIST, test index 1

 Starting PBIST test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 868
  Delta PBIST execution time in micro secs 3553
  Delta Cores restore time in micro secs 789
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST test on Infra PBIST, index 1...
  Delta Cores prep time in micro secs 8791
  Delta PBIST execution time in micro secs 92767
  Delta Cores restore time in micro secs 7364
 PBIST complete Infra PBIST, test index 1

 PBIST Functionality Test Passed.

All tests have passed.


[MAIN_Cortex_R5_0_0]
PBIST Test Application

 Starting PBIST failure insertion test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 798
  Delta PBIST execution time in micro secs 18
  Delta Cores restore time in micro secs 908
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST failure insertion test on Infra PBIST, index 1...
  Delta Cores prep time in micro secs 3212
  Delta PBIST execution time in micro secs 24
  Delta Cores restore time in micro secs 2645
 PBIST complete Infra PBIST, test index 1

 Starting PBIST test on Pulsar Instance 1, index 0...
  Delta Cores prep time in micro secs 1066
  Delta PBIST execution time in micro secs 3560
  Delta Cores restore time in micro secs 944
 PBIST complete Pulsar Instance 1, test index 0

 Starting PBIST test on Infra PBIST, index 1...
  Delta Cores prep time in micro secs 9682
  Delta PBIST execution time in micro secs 92780
  Delta Cores restore time in micro secs 7302
 PBIST complete Infra PBIST, test index 1

 PBIST Functionality Test Passed.

All tests have passed.

\endcode
\endcond