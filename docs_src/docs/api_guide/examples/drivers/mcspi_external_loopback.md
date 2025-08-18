# MCSPI External Loopback {#EXAMPLES_DRIVERS_MCSPI_EXTERNAL_LOOPBACK}

[TOC]

# Introduction

This example demonstrates the McSPI RX and TX operation configured
in blocking, interrupt mode of operation with External loopback connection
of 2 McSPI instances controlled by different cores - one in controller and 
other in peripheral mode

This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE 
and then receives the same in RX mode.

When transfer is completed, TX and RX buffer data are compared.
If data is matched, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_MCSPI_EXTERNAL_LOOPBACK_COMBOS}


\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_external_loopback

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

- Make below external loopback connections between two SPI instances:
  
  \cond SOC_AM263PX
  - AM263px CC and AM263x HSEC Board
    - MCU_SPI0_CS0 (C11) (HSEC Pin 12)   ->   MCU_SPI1_CS0 (C9)  (HSEC Pin 16)
    - MCU_SPI0_CLK (A11) (HSEC Pin 11)   ->   MCU_SPI1_CLK (A10) (HSEC Pin 15)
    - MCU_SPI0_D0  (C10) (HSEC Pin 9)    ->   MCU_SPI1_D0 (B10)  (HSEC Pin 14) 
    - MCU_SPI0_D1  (B11) (HSEC Pin 10)   ->   MCU_SPI1_D1 (D9)   (HSEC Pin 13) 
  
  - AM263px CC and C2000 HSEC Board
    - MCU_SPI0_CS0 (C11) (HSEC Pin 73)   ->   MCU_SPI1_CS0 (C9)  (HSEC Pin 81)
    - MCU_SPI0_CLK (A11) (HSEC Pin 71)   ->   MCU_SPI1_CLK (A10) (HSEC Pin 79)
    - MCU_SPI0_D0  (C10) (HSEC Pin 67)   ->   MCU_SPI1_D0 (B10)  (HSEC Pin 77) 
    - MCU_SPI0_D1  (B11) (HSEC Pin 69)   ->   MCU_SPI1_D1 (D9)   (HSEC Pin 75)
  
  - AM263px LP
    - MCU_SPI0_CS0(Pin 8) (C11)   ->   MCU_SPI1_CS0(Pin 58) (C9)
    - MCU_SPI0_CLK(Pin 7) (A11)   ->   MCU_SPI1_CLK(Pin 47) (A10)
    - MCU_SPI0_D0(Pin 15) (C10)   ->   MCU_SPI1_D1(Pin 54)  (D9)
    - MCU_SPI0_D1(Pin 14) (B11)   ->   MCU_SPI1_D0(Pin 55)  (B10)
  \endcond
  
  \cond SOC_AM261X
  - AM261x LP
    - MCU_SPI0_CS0 (J2/J4 Pin 19) (B13)  ->  MCU_SPI2_CS1  (J5/J7 Pin 59)  A18  
    - MCU_SPI0_CLK (J6/J8 Pin 7) (T2)    ->  MCU_SPI2_CLK  (J6/J8 Pin 47)  D17 
    - MCU_SPI0_D0  (J6/J8 Pin 15) (T1)   ->  MCU_SPI2_D1   (J6/J8 Pin 54)  B18
    - MCU_SPI0_D1  (J6/J8 Pin 14) (U1)   ->  MCU_SPI2_D0   (J6/J8 Pin 55)  A16
  
  - AM261x SOM and AM263x HSEC Board
    - MCU_SPI0_CS0 (C11) -> HSEC_SPI1_CS0 (J20-16)  ->  MCU_SPI3_CS0 (D7) -> HSEC_SPI1_CS0 (J20-12)
    - MCU_SPI0_CLK (A11) -> HSEC_SPI1_CLK (J20-15)  ->  MCU_SPI3_CLK (C8) -> HSEC_SPI1_CLK (J20-11)
    - MCU_SPI0_D0  (C10) -> HSEC_SPI1_CS0 (J20-14)  ->  MCU_SPI3_D0 (C7) -> HSEC_SPI1_CS0  (J20-10)
    - MCU_SPI0_D1  (B11) -> HSEC_SPI1_CS0 (J20-13)  ->  MCU_SPI3_D1 (B7) -> HSEC_SPI1_CS0  (J20-9)
 
  - AM261x SOM and C2000 HSEC Board
    - MCU_SPI0_CS0 (C11) -> HSEC_SPI1_CS0 (81)  ->  MCU_SPI3_CS0 (D7) -> HSEC_SPI1_CS0 (73)
    - MCU_SPI0_CLK (A11) -> HSEC_SPI1_CLK (79)  ->  MCU_SPI3_CLK (C8) -> HSEC_SPI1_CLK (71)
    - MCU_SPI0_D0  (C10) -> HSEC_SPI1_CS0 (77)  ->  MCU_SPI3_D0 (C7) -> HSEC_SPI1_CS0  (69)
    - MCU_SPI0_D1  (B11) -> HSEC_SPI1_CS0 (75)  ->  MCU_SPI3_D1 (B7) -> HSEC_SPI1_CS0  (67)
  \endcond

  \cond SOC_AM263X
  - AM263x CC and AM263x HSEC Board
    - MCU_SPI0_CS0 (C11) (HSEC Pin 12)   ->   MCU_SPI1_CS0 (C9)  (HSEC Pin 16)
    - MCU_SPI0_CLK (A11) (HSEC Pin 11)   ->   MCU_SPI1_CLK (A10) (HSEC Pin 15)
    - MCU_SPI0_D0  (C10) (HSEC Pin 9)    ->   MCU_SPI1_D0 (B10)  (HSEC Pin 14) 
    - MCU_SPI0_D1  (B11) (HSEC Pin 10)   ->   MCU_SPI1_D1 (D9)   (HSEC Pin 13) 
  
  - AM263x CC and C2000 HSEC Board
    - MCU_SPI0_CS0 (C11) (HSEC Pin 73)   ->   MCU_SPI1_CS0 (C9)  (HSEC Pin 81)
    - MCU_SPI0_CLK (A11) (HSEC Pin 71)   ->   MCU_SPI1_CLK (A10) (HSEC Pin 79)
    - MCU_SPI0_D0  (C10) (HSEC Pin 67)   ->   MCU_SPI1_D0 (B10)  (HSEC Pin 77) 
    - MCU_SPI0_D1  (B11) (HSEC Pin 69)   ->   MCU_SPI1_D1 (D9)   (HSEC Pin 75)
  - AM263x LP
    - MCU_SPI0_CS0(Pin 18)   ->   MCU_SPI1_CS0(Pin 58)
    - MCU_SPI0_CLK(Pin 7)    ->   MCU_SPI1_CLK(Pin 47)
    - MCU_SPI0_D0(Pin 55)    ->   MCU_SPI1_D1(Pin 14)
    - MCU_SPI0_D1(Pin 54)    ->   MCU_SPI1_D0(Pin 15)
  \endcond

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCSPI_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MCSPI] External Loopback example started in Peripheral mode...
[r5f0-1]     0.000059s : [MCSPI] External Loopback example started in Controller mode...
Successfully received and sent data in Peripheral mode!! 
[r5f0-1]     2.348816s : Successfully received and sent data in Controller mode!! 
[r5f0-1]     2.348837s : All tests have passed!!
\endcode

