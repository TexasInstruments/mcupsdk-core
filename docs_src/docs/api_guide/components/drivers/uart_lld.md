
# UART Low Level Driver{#DRIVERS_UART_LLD_PAGE}

## Features Supported

- Write and Read mode of operation
- Interrupt, Polled Mode
\cond !SOC_AM62X
- DMA mode of operation
\endcond
- Non-blocking (Callback) transfers

## Features NOT Supported

\cond !SOC_AM62X
- #UART_READ_RETURN_MODE_PARTIAL is not supported in DMA mode of operation
\endcond
\cond SOC_AM62X
- DMA mode is not supported.
\endcond
- MODEM control functions
- IrDA(Infrared Data Association) and CIR(Consumer Infrared) features

## Usage Overview

### API Sequence

To use the UART driver to send data or receive, the application
calls the following APIs:

- #UART_lld_init() : Initialize the UART driver.
- #UART_lld_read() / #UART_lld_readIntr() / #UART_lld_readDma():
  Read data in polling, interrupt, and Dma mode respectively.
- #UART_lld_write() / #UART_lld_writeIntr() / #UART_lld_writeDma():
  Write data in polling, interrupt, and Dma mode respectively..
- #UART_lld_deInit():  De-initialize the UART instance.

### Initializing the UART Driver

#UART_lld_init() must be called before any other UART APIs.
This function uses uart handle to initialize
each instance. Calling #UART_lld_init() a second time with the same handle
previously passed to #UART_lld_init() will result in an error.  You can,
though, re-use the handle if the instance is closed via #UART_lld_deInit().
In DMA mode, #UART_lld_initDma() needs to be called instead of
#UART_lld_init() to acquire and initialize uart instance.
Please note that initializing UART driver is taken care by the
SysConfig generated code.

### UART Write Mode

The UART driver supports two transfer modes of operation: interrupt and polling mode.
In polling mode a task's code execution is blocked until a UART
transaction has completed or a timeout has occurred.

In interrupt mode, again there are two modes blocking and callback.
The UART driver defaults to blocking mode, if the application does not set it.
Once a UART driver is opened, the only way to change the operation mode
is to close and re-open the UART instance with the new write mode.

In callback mode, a UART transaction functions asynchronously, which
means that it does not block code execution. After a UART transaction
has been completed, the UART driver calls a user-provided callback function.
Callback mode is supported in the execution context of tasks and
hardware interrupt routines.

### UART Read Mode

The UART driver supports two read modes of operation: interrupt and polling mode.
In polling mode a task's code execution is blocked until a UART
transaction has completed or a timeout has occurred.

In interrupt mode, again there are two modes blocking and callback.
The UART driver defaults to blocking mode, if the application does not set it.
Once a UART driver is opened, the only way to change the operation mode
is to close and re-open the UART instance with the new read mode.

In callback mode, a UART transaction functions asynchronously, which
means that it does not block code execution. After a UART transaction
has been completed, the UART driver calls a user-provided callback function.
Callback mode is supported in the execution context of tasks and
hardware interrupt routines.

\cond !SOC_AM62X && !SOC_AM65X
## Important Usage Guidelines

- In case of DMA mode, as R5F core is not Cache Coherent, Cache Writeback is required if R5F writes to the buffers.
  And before reading the buffers, application needs to invalidate those. Please refer \ref EXAMPLES_DRIVERS_UART_ECHO_DMA_LLD.
\endcond
## Example Usage

Include the below file to access the APIs
\snippet Uart_lld_sample.c include

Instance Open Example
\snippet Uart_lld_sample.c open

Instance Close Example
\snippet Uart_lld_sample.c close

Write Non-Blocking Transfer Example
\snippet Uart_lld_sample.c write_transfer_nonblocking

Read Non-Blocking Transfer Example
\snippet Uart_lld_sample.c read_transfer_nonblocking

Non-Blocking Example write callback
\snippet Uart_lld_sample.c write_callback

Non-Blocking Example read callback
\snippet Uart_lld_sample.c read_callback

Non-Blocking Example ISR CALL callback
\snippet Uart_lld_sample.c isr_call

## API

\ref DRV_UART_MODULE
