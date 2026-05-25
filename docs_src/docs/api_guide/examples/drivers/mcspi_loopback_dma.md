# MCSPI Loopback DMA {#EXAMPLES_DRIVERS_MCSPI_LOOPBACK_DMA}

[TOC]

# Introduction

This example demonstrates the McSPI RX and TX operation configured
in blocking, DMA mode of operation.

This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
and then receives the same in RX mode. Internal pad level loopback mode
is enabled to receive data.
To enable internal pad level loopback mode, D0 pin is configured to both
TX Enable as well as RX input pin in the SYSCFG.

When transfer is completed, TX and RX buffer data are compared.
If data is matched, test result is passed otherwise failed.

## Important DMA Limitation

**Maximum McSPI DMA transfer: 4,095 WORDS per transaction**

The 12-bit PDMA transfer counter limits transfers to 4,095 words maximum. The actual byte limit depends on data width:

| Data Width | Max Words | Max Bytes |
|------------|-----------|-----------|
| 8-bit  | 4,095 | **4,095 bytes** |
| 16-bit | 4,095 | **8,190 bytes** |
| 32-bit | 4,095 | **16,380 bytes** |

Transfer size validation: `transferBytes = count << bufWidthShift`, where:
- count = number of words to transfer
- bufWidthShift = log2(dataSize in bytes)

**CRITICAL:** Attempting a transfer count of 4,096 or higher causes register overflow (0x1000 wraps to 0x000), transfer fails, and system may hang.

For transfers larger than these limits, refer to \ref DRIVERS_MCSPI_HLD_PAGE for detailed workaround guidance (split into multiple transactions).

# Supported Combinations {#EXAMPLES_DRIVERS_MCSPI_LOOPBACK_DMA_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback_dma

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback_dma

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback_dma

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCSPI_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MCSPI] Loopback example DMA mode started ...
All tests have passed!!
\endcode

