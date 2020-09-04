# UART {#DRIVERS_UART_PAGE}

[TOC]
UART is used to translate the data between the chip and a serial port.
The UART driver provides API to perform read and write to any of the UART peripherals on the board, with the multiple modes of operation.

## Features Supported

- UART 16x mode (<= 115.2 Kbaud/s)
- Write and Read mode of operation
- Interrupt, Polled Mode
- Blocking and Non-blocking (callback) transfers
- Write and Read Cancel mode of operation

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- UART module configuration parmaters like baudrate, parity type, datalength and others.
- UART instances and pin configurations.
- Interrupt mode option is used to select one of the following.
    - Polling Mode.
    - Interrupt Mode in which driver manages the interrupt service routine.
    - User Managed Interrupt in which user manages the interrupt service routine.
      This mode can be used in low latency applications.
- Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
    - Set UART instance parameter configuration.
    - Driver ISR registration if Interrupt Mode is enabled.
    - Skip driver ISR registration if "User Managed Interrupt" mode is configured.

## Features NOT Supported

- DMA mode of operation

## Usage Overview

### API Sequence

To use the UART driver to send data or receive, the application
calls the following APIs:

- #UART_init() : Initialize the UART driver.
- #UART_Params_init():  Initialize a #UART_Params structure with default
  values.  Then change the parameters from non-default values as
  needed.
- #UART_open() :  Open an instance of the UART driver, passing the
  initialized parameters, or NULL, and an index to the configuration to
  open (detailed later).
- #UART_write():  Transmit data.  This function takes a
  #UART_Transaction argument that describes the transfer that is requested.
- #UART_read() :  Receive data.  This function takes a
  #UART_Transaction argument that describes the receive that is requested.
- #UART_close():  De-initialize the UART instance.
- #UART_deinit(): De-Initialize the UART driver.

### Initializing the UART Driver

#UART_init() must be called before any other UART APIs.  This function
iterates through the elements of the UART_config[] array, calling
the element's device implementation UART initialization function.
Please note that initializing UART driver is taken care by the
SysConfig generated code.

### Opening the UART Driver

After initializing the UART driver by calling #UART_init(), the application
can open a UART instance by calling #UART_open().
Please note that opening UART driver is taken care by the
SysConfig generated code.
This function takes an index into the UART_config[] array, and the UART parameters data
structure. The UART instance is specified by the index of the UART in
UART_config[]. Calling #UART_open() second time with the same index
previously passed to #UART_open() will result in an error.  You can,
though, re-use the index if the instance is closed via #UART_close().

If no #UART_Params_init structure is passed to UART_open(), default values are
used. If the open call is successful, it returns a non-NULL value.

### UART Write Mode

The UART driver supports two transfer modes of operation: interrupt and polling mode.
In polling mode a task's code execution is blocked until a UART
transaction has completed or a timeout has occurred.

In interrupt mode, again there are two modes blocking and callback.
The transfer mode is determined by the #UART_Params.writeMode parameter.
The UART driver defaults to blocking mode, if the application does not set it.
Once a UART driver is opened, the only way to change the operation mode
is to close and re-open the UART instance with the new write mode.

In blocking mode, a task's code execution is blocked until a UART
transaction has completed or a timeout has occurred. This ensures
that only one UART transfer operates at a given time. Other tasks requesting
UART transfers while a transfer is currently taking place will receive
an error as return value. If a timeout occurs the transfer is cancelled, the
task is unblocked & will receive an error as return value. The transaction
count field will have the number of bytes transferred
successfully before the timeout.

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
The read mode is determined by the #UART_Params.readMode parameter.
The UART driver defaults to blocking mode, if the application does not set it.
Once a UART driver is opened, the only way to change the operation mode
is to close and re-open the UART instance with the new read mode.

In blocking mode, a task's code execution is blocked until a UART
transaction has completed or a timeout has occurred. This ensures
that only one UART read completes at a given time. Other tasks requesting
UART read while a read is currently taking place will receive
an error as return value. If a timeout occurs the read is cancelled, the
task is unblocked & will receive an error as return value. The transaction
count field will have the number of bytes read
successfully before the timeout.

In callback mode, a UART transaction functions asynchronously, which
means that it does not block code execution. After a UART transaction
has been completed, the UART driver calls a user-provided callback function.
Callback mode is supported in the execution context of tasks and
hardware interrupt routines.

## Important Usage Guidelines

- None

## Example Usage

Include the below file to access the APIs
\snippet Uart_sample.c include

Instance Open Example
\snippet Uart_sample.c open

Instance Close Example
\snippet Uart_sample.c close

Write Transfer Example
\snippet Uart_sample.c write_transfer_blocking

Read Transfer Example
\snippet Uart_sample.c read_transfer_blocking

Write Non-Blocking Transfer Example
\snippet Uart_sample.c write_transfer_nonblocking

Read Non-Blocking Transfer Example
\snippet Uart_sample.c read_transfer_nonblocking

## API

\ref DRV_UART_MODULE
