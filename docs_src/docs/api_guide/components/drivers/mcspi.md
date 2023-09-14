# MCSPI {#DRIVERS_MCSPI_PAGE}

[TOC]

The Multi Channel Serial Peripheral Interface (MCSPI) driver is a generic,
full-duplex driver that transmits and receives data on the SPI bus.
The SPI protocol defines the format of a data transfer over the SPI bus,
but it leaves flow control, data formatting, and handshaking mechanisms
to higher-level software layers.

## Features Supported

- Controller and Peripheral mode of operation
- Per transfer selection of different channels/chip select
- Blocking and non-blocking (callback) transfers
- For low latency transfers, refer \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_32BIT and \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_8BIT example.
  This example uses polling mode of operation.
- DMA mode of operation

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- MCSPI module configuration parameters like Transmit/Receive mode,MISO, MOSI pin selection and others.
- MCSPI channel configurations.
- In the advanced configurations option you can configure initial delay for first transfer, transfer mode and timeout.
- MCSPI instances and pin configurations.
- Operation Mode selection - Polling, Interrupt or DMA
- Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
    - Set MCSPI instance parameter configuration.
    - Driver ISR registration if Interrupt Mode is enabled.
\cond SOC_AM263X || SOC_AM263PX
    - EDMA Trigger Crossbar configuration for McSPI RX and TX in DMA mode.
\endcond
\cond SOC_AM64X || SOC_AM243X
- In case of DMA mode, please configure UDMA instance to `PKTDMA_0`.
\endcond

## Features NOT Supported
\cond SOC_AM64X || SOC_AM243X
- For MCU domain instances, DMA mode is not supported.
\endcond
- Default TX data feature is not supported in DMA mode.
- In DMA mode, FIFO is not enabled.

\cond SOC_AM64X || SOC_AM243X
## Constraint

- Due to the design constraint maximum DMA PKTDMA_0 TX/RX channels each can be used is 3 per R5F core.
  So in case of MCSPI instance with DMA mode enabled can use atmost 3 CS in multi-controller mode.

\endcond
## Usage Overview

### API Sequence

To use the MCSPI driver to send data over the SPI bus, the application
calls the following APIs:

- #MCSPI_init(): Initialize the MCSPI driver.
- #MCSPI_OpenParams_init():  Initialize a #MCSPI_OpenParams structure with default
  values.  Then change the parameters from non-default values as
  needed.
- #MCSPI_open():  Open an instance of the MCSPI driver, passing the
  initialized parameters, or NULL, and an index to the configuration to
  open (detailed later).
- #MCSPI_chConfig():  Configure the required channels
- #MCSPI_dmaChConfig(): Configure the required DMA channels(in DMA mode only)
- #MCSPI_transfer():  Transmit/receive data.  This function takes a
  #MCSPI_Transaction argument that describes the transfer that is requested.
 -#MCSPI_dmaClose(): De-initialize the DMA channels(in DMA mode only)
- #MCSPI_close():  De-initialize the MCSPI instance.
- #MCSPI_deinit(): De-Initialize the MCSPI driver.

### Initializing the MCSPI Driver

#MCSPI_init() must be called before any other MCSPI APIs.  This function
iterates through the elements of the MCSPI_config[] array, calling
the element's device implementation MCSPI initialization function.
Please note that initializing MCSPI driver is taken care by the
SysConfig generated code.

### Opening the MCSPI Driver

After initializing the MCSPI driver by calling #MCSPI_init(), the application
can open a MCSPI instance by calling #MCSPI_open().
Please note that opening MCSPI driver is taken care by the
SysConfig generated code.
This function takes an index into the MCSPI_config[] array, and the MCSPI parameters data
structure. The MCSPI instance is specified by the index of the SPI in
MCSPI_config[]. Calling #MCSPI_open() a second time with the same index
previously passed to #MCSPI_open() will result in an error.  You can,
though, re-use the index if the instance is closed via #MCSPI_close().
In DMA mode, #MCSPI_dmaChConfig() needs to be called after #MCSPI_open() to acquire and initialize
DMA channels. This is also taken care by the SysConfig generated code.

If no #MCSPI_OpenParams structure is passed to MCSPI_open(), default values are
used. If the open call is successful, it returns a non-NULL value.

### MCSPI Transfer Mode

The MCSPI driver supports three transfer modes of operation: Interrupt, Polling and DMA Mode.
In Interrupt and DMA mode, it again supports two modes: blocking and callback.
The transfer mode is determined by the #MCSPI_OpenParams.transferMode parameter.
The MCSPI driver defaults to blocking mode, if the application does not set it.
Once a MCSPI driver is opened, the only way to change the operation mode
is to close and re-open the MCSPI instance with the new transfer mode.

In blocking mode, a task's code execution is blocked until a MCSPI
transaction has completed or a timeout has occurred. This ensures
that only one MCSPI transfer operates at a given time. Other tasks requesting
MCSPI transfers while a transfer is currently taking place will receive
a error return value. If a timeout occurs the transfer is canceled, the
task is unblocked & will receive a error return value. The transaction
count field will have the amount of frames which were transferred
successfully before the timeout. In blocking mode, transfers cannot be
performed in software or hardware ISR context.

In callback mode, a MCSPI transaction functions asynchronously, which
means that it does not block code execution. After a MCSPI transaction
has been completed, the MCSPI driver calls a user-provided hook function.
Callback mode is supported in the execution context of tasks and
hardware interrupt routines.

In multichannel mode connected to multiple external devices,
the MCSPI exchanges data with one MCSPI device at a time and FIFO is enabled
per each channel at a time.

### MCSPI Frame Formats and Data Size

The MCSPI driver can configure the device's MCSPI peripheral to transfer
data in several MCSPI format options: MCSPI (with various polarity and phase
settings). The frame format is set with #MCSPI_ChConfig.frameFormat.

The smallest single unit of data transmitted onto the MCSPI bus is called
a MCSPI frame and is of size #MCSPI_Transaction.dataSize. A series of MCSPI frames
transmitted/received on a MCSPI bus is referred to as a MCSPI transaction.

### MCSPI Transactions

A MCSPI transaction consists of a series of MCSPI frames
transmitted/received on a MCSPI bus.  A MCSPI transaction is performed
using #MCSPI_transfer(). #MCSPI_transfer() accepts a pointer to a
#MCSPI_Transaction structure that dictates the quantity of data to be
sent and received.
The #MCSPI_Transaction.txBuf and #MCSPI_Transaction.rxBuf are both pointers
to data buffers.
If txBuf is NULL, the driver sends MCSPI frames with all data set to the default
value specified in the hardware attributes.
If rxBuf is NULL, the driver discards all MCSPI frames received. #MCSPI_transfer()
of a MCSPI transaction is performed atomically.

@warning The use of NULL as a sentinel txBuf or rxBuf value to determine
whether the MCSPI transaction includes a tx or rx component implies
that it is not possible to perform a transmit or receive transfer
directly from/to a buffer with a base address of 0x00000000. To support
this rare use-case, the application will have to manually copy the
contents of location 0x00000000 to/from a temporary buffer before/after
the tx/rx MCSPI transaction.

#MCSPI_Transaction.dataSize determines the element types
of txBuf and rxBuf. If the dataSize is from 4 to 8 bits, the driver
assumes the data buffers are of type uint8_t (unsigned char). If the
dataSize is from 9 to 16 bits, the driver assumes the data buffers are
of type uint16_t (unsigned short).  If the dataSize is greater than
16 bits, the driver assumes the data buffers are uint32_t (unsigned long).

#MCSPI_Transaction.csDisable can be set to TRUE/FALSE to disable CS(chip select).
If it is set to TRUE, CS is de-asseted automatically at the end of the
transfer. If user wants to chain more transfers under one CS pulse,
user needs to set it to FALSE for each transfer and for the last
transfer, user needs to set to TRUE to de-assert CS.
Generally this is useful when SPI needs to communicate with memory device where
usually command/address is sent first and then the data will be sent.

The optional #MCSPI_Transaction.args variable can only be used when the
MCSPI driver has been opened in callback mode. This variable is used to
pass a user-defined value into the user-defined callback function.

#MCSPI_transfer() always performs full-duplex MCSPI transactions. This means
the MCSPI simultaneously receives data as it transmits data. The application
is responsible for formatting the data to be transmitted as well as
determining whether the data received is meaningful.
Specifics about MCSPI frame formatting and data sizes are provided in
device-specific data sheets and technical reference manuals.

In case of MCSPI operating in #MCSPI_MS_MODE_PERIPHERAL mode if Rx overflow or
Tx underflow occurs, driver cancels the current transfer and return status
MCSPI_TRANSFER_CANCELLED to the application. Application need to check the
status and reinitiate transfers again.

## Important Usage Guidelines

- The MCSPI protocol does not account for a built-in handshaking mechanism
  and neither does this driver. Therefore, when operating in
  #MCSPI_MS_MODE_PERIPHERAL mode, the application must provide such a mechanism to
  ensure that the MCSPI peripheral is ready for the MCSPI controller. The MCSPI peripheral
  must call #MCSPI_transfer() *before* the controller starts transmitting.
  Some example application mechanisms could include:
    - Timed delays on the MCSPI controller to guarantee the MCSPI peripheral is ready
      for a MCSPI transaction.
    - A form of GPIO flow control from the peripheral to the MCSPI controller to notify
      the controller when ready.

\cond !SOC_AM62X
- In case of DMA mode, as R5F core is not Cache Coherent, Cache Writeback is required if R5F writes to the buffers.
  And before reading the buffers, application needs to invalidate those. Please refer \ref EXAMPLES_DRIVERS_MCSPI_LOOPBACK_DMA.
\endcond
## Example Usage

Include the below file to access the APIs
\snippet Mcspi_sample.c include

Instance Open Example
\snippet Mcspi_sample.c open

Instance Close Example
\snippet Mcspi_sample.c close

Blocking Transfer Example
\snippet Mcspi_sample.c transfer_blocking

Chain Transfer Example Blocking Mode
\snippet Mcspi_sample.c chain_transfer_blocking

Non-Blocking Transfer Example
\snippet Mcspi_sample.c transfer_nonblocking

## API

\ref DRV_MCSPI_MODULE
