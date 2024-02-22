# MCSPI Low Level Driver{#DRIVERS_MCSPI_LLD_PAGE}

## Features Supported

- Controller and Peripheral mode of operation
- Per transfer selection of different channels/chip select
- Non-blocking (Callback) transfers

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

- #MCSPI_lld_init(): Initialize the MCSPI driver.
- #MCSPI_lld_readWrite() / #MCSPI_lld_readWriteIntr() / #MCSPI_lld_readWriteDma():
  Read and Write data in polling, interrupt, and Dma mode respectively.
- #MCSPI_lld_read() / #MCSPI_lld_readIntr() / #MCSPI_lld_readDma():
  Read data in polling, interrupt, and Dma mode respectively.
- #MCSPI_lld_write() / #MCSPI_lld_writeIntr() / #MCSPI_lld_writeDma():
  Write data in polling, interrupt, and Dma mode respectively.
- #MCSPI_lld_deInit():  De-initialize the MCSPI instance.

### Initializing the MCSPI Driver

#MCSPI_lld_init() must be called before any other MCSPI APIs.  This function
iterates till the channel count. This function uses mcspi handle to initialize
each instance. Calling #MCSPI_lld_init() a second time with the same handle
previously passed to #MCSPI_lld_init() will result in an error.  You can,
though, re-use the handle if the instance is closed via #MCSPI_lld_deInit().
In DMA mode, #MCSPI_lld_initDma() needs to be called instead of
#MCSPI_lld_init() to acquire and initialize mcspi instance.
Please note that initializing MCSPI driver is taken care by the
SysConfig generated code.

### MCSPI Transfer Mode

The MCSPI driver supports three transfer modes of operation: Interrupt, Polling and DMA Mode.
Interrupt and DMA mode, it supports only callback mode.
Once a MCSPI driver is opened, the only way to change the operation mode
is to close and re-open the MCSPI instance with the new transfer mode.

In callback mode, a MCSPI transaction functions asynchronously, which
means that it does not block code execution. After a MCSPI transaction
has been completed, the MCSPI driver calls a user-provided hook function.
Callback mode is supported in the execution context of tasks and
hardware interrupt routines.

In multichannel mode connected to multiple external devices,
the MCSPI exchanges data with one MCSPI device at a time and FIFO is enabled
per each channel at a time.

NOTE: The size of txBuf and RxBuf must be greater than the data size bits, if data size
is not a multiple of 8 bits. Mask the data size bits in txBuf and rxBuf as the
remaining bits will be discarded. For example, consider datasize = 18 bits then
txBuf and rxBuf size should be uint32_t and the mask bits should be 0x3FFFF.

## Important Usage Guidelines

- The MCSPI protocol does not account for a built-in handshaking mechanism
  and neither does this driver. Therefore, when operating in
  #MCSPI_MS_MODE_PERIPHERAL mode, the application must provide such a mechanism to
  ensure that the MCSPI peripheral is ready for the MCSPI controller. The MCSPI peripheral
  must call #MCSPI_lld_readWrite() / #MCSPI_lld_readWriteIntr() / #MCSPI_lld_readWriteDma()
  *before* the controller starts transmitting.
  Some example application mechanisms could include:
    - Timed delays on the MCSPI controller to guarantee the MCSPI peripheral is ready
      for a MCSPI transaction.
    - A form of GPIO flow control from the peripheral to the MCSPI controller to notify
      the controller when ready.

\cond !SOC_AM62X
- In case of DMA mode, as R5F core is not Cache Coherent, Cache Writeback is required if R5F writes to the buffers.
  And before reading the buffers, application needs to invalidate those. Please refer \ref EXAMPLES_DRIVERS_MCSPI_LOOPBACK_DMA_LLD.
\endcond
## Example Usage

Include the below file to access the APIs
\snippet Mcspi_lld_sample.c include

Instance Open Example
\snippet Mcspi_lld_sample.c open

Instance Close Example
\snippet Mcspi_lld_sample.c close

Non-Blocking Transfer Example
\snippet Mcspi_lld_sample.c transfer_nonblocking

Non-Blocking Example transfer callback
\snippet Mcspi_lld_sample.c transfer_callback

Non-Blocking Example ISR CALL callback
\snippet Mcspi_lld_sample.c isr_call

## API

\ref DRV_MCSPI_LLD_MODULE
