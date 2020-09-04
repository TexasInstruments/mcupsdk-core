# FSI Loopback Dma {#EXAMPLES_DRIVERS_FSI_LOOPBACK_DMA}

[TOC]

# Introduction

This example performs FSI TX to FSI RX internal loopback in DMA mode.
The application configures an instance of FSI TX and FSI RX module with below configuration

- Single lane
- TX clock at 50 MHz
- 16 words per frame (transfer)
- Register both FSI TX DMA and FSI RX DMA edma interrupts
- Set up two DMA channels to be triggered by the same FSI transmitter and DMA trigger.
- Configure one channel to fill the transmit buffer.
- Configure the other channel to set the frame tag and user data fields
- Similar configuration for RX to receive data and frame tag.
- 4 DMA Channels required to transmit and receive.

With above configuration, the application transfers 100 frames of data from FSI TX,
waits for data to be received by FSI RX. EDMA completion interrupt is configured
once all the data, tag and user data is transmitted/received.

Once the transfer completes, it compares the source and destination buffers for any data mismatch.

# Supported Combinations {#EXAMPLES_DRIVERS_FSI_LOOPBACK_DMA_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/fsi/fsi_loopback_dma/

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_FSI_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[FSI] Loopback Dma application started ...
[FSI] 100 frames successfully received!!!
All tests have passed!!
\endcode
