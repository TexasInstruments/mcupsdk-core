# MIBSPI {#DRIVERS_MIBSPI_PAGE}

[TOC]

The Multi Buffered Serial Peripheral Interface (MibSPI/MibSPIP) driver is a generic,
full-duplex driver that transmits and receives data on the SPI bus.
The SPI protocol defines the format of a data transfer over the SPI bus,
but it leaves flow control, data formatting, and handshaking mechanisms
to higher-level software layers.

## Features Supported

- Controller and Peripheral mode of operation
- Per transfer selection of different channels/chip select
- Blocking and non-blocking (callback) transfers
- icount enable/disable
- EDMA

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- MIBSPI instances and pin configurations
- Init/Deinit API integration with SYSCFG
- MIBSPI module configuration parameters like MOSI, MISO pin selection and others.
- MIBSPI channel configurations.
- In the advanced configurations option you can configure MIBSPI advanced features such as number of parallel mode pin,
  feature bit map, transmit dummy value, ecc enable, cs hold and compatibility mode.
- Based on above parameters, the SysConfig generated code does MIBSPI instance parameter configuration as part of
  Drivers_open and Drivers_close functions.

## Features NOT Supported

- EDMA support in compatibility mode

## Usage Overview

### API Sequence

To use the MIBSPI driver to send data over the SPI bus, the application
calls the following APIs:

- #MIBSPI_init(): Initialize the MIBSPI driver.
- #MIBSPI_Params_init():  Initialize a #MIBSPI_OpenParams structure with default
  values.  Then change the parameters from non-default values as needed.
- #MIBSPI_open():  Initialize the driver.
- #MIBSPI_transfer():  Transmit/receive data.  This function takes a
  #MIBSPI_Transaction argument that describes the transfer that is requested.
- #MIBSPI_close():  De-initialize the MIBSPI instance.
- #MIBSPI_deinit(): De-Initialize the MIBSPI driver.

### Initializing the MIBSPI Driver

#MIBSPI_init() must be called before any other MIBSPI APIs.  This function
Create driver lock Construct thread safe handles for SPI driver level
Semaphore to provide exclusive access to the SPI APIs

### Opening the MIBSPI Driver

After initializing the MIBSPI driver by calling #MIBSPI_init(), the application
can open a MIBSPI instance by calling #MIBSPI_open().
Please note that opening MIBSPI driver is taken care by the
SysConfig generated code.
This function iterates through the elements of the MIBSPI_config[] array, calling
the element's device implementation MIBSPI initialization function.
Please note that initializing MIBSPI driver is taken care by the
SysConfig generated code.

If no #MIBSPI_OpenParams structure is passed to MIBSPI_open(), default values are
used. If the open call is successful, it returns a non-NULL value.

### MIBSPI Transfer Mode

The MIBSPI driver supports two transfer modes of operation: blocking and callback.
The transfer mode is determined by the #MIBSPI_OpenParams.transferMode parameter.
The MIBSPI driver defaults to blocking mode, if the application does not set it.
Once a MIBSPI driver is opened, the only way to change the operation mode
is to close and re-open the MIBSPI instance with the new transfer mode.

In blocking mode, a task's code execution is blocked until a MIBSPI
transaction has completed or a timeout has occurred. This ensures
that only one MIBSPI transfer operates at a given time. Other tasks requesting
MIBSPI transfers while a transfer is currently taking place will receive
a error return value. If a timeout occurs the transfer is cancelled, the
task is unblocked & will receive a error return value. The transaction
count field will have the amount of frames which were transferred
successfully before the timeout. In blocking mode, transfers cannot be
performed in software or hardware ISR context.

In callback mode, a MIBSPI transaction functions asynchronously, which
means that it does not block code execution. After a MIBSPI transaction
has been completed, the MIBSPI driver calls a user-provided hook function.
Callback mode is supported in the execution context of tasks and
hardware interrupt routines.

### MIBSPI Frame Formats and Data Size

The MIBSPI driver can configure the device's MIBSPI peripheral to transfer
data in several MIBSPI format options: MIBSPI (with various polarity and phase
settings). The frame format is set with #MIBSPI_OpenParams.frameFormat.

The smallest single unit of data transmitted onto the MIBSPI bus is called
a MIBSPI frame and is of size #MIBSPI_OpenParams.dataSize. A series of MIBSPI frames
transmitted/received on a MIBSPI bus is referred to as a MIBSPI transaction.

### MIBSPI Transactions

A MIBSPI transaction consists of a series of MIBSPI frames
transmitted/received on a MIBSPI bus.  A MIBSPI transaction is performed
using #MIBSPI_transfer(). #MIBSPI_transfer() accepts a pointer to a
#MIBSPI_Transaction structure that dictates the quantity of data to be
sent and received.
The #MIBSPI_Transaction.txBuf and #MIBSPI_Transaction.rxBuf are both pointers
to data buffers.
The optional #MIBSPI_Transaction.arg variable can only be used when the
MIBSPI driver has been opened in callback mode. This variable is used to
pass a user-defined value into the user-defined callback function.

#MIBSPI_transfer() always performs full-duplex MIBSPI transactions. This means
the MIBSPI simultaneously receives data as it transmits data. The application
is responsible for formatting the data to be transmitted as well as
determining whether the data received is meaningful.
Specifics about MIBSPI frame formatting and data sizes are provided in
device-specific data sheets and technical reference manuals.

## Important Usage Guidelines

- The MIBSPI protocol does not account for a built-in handshaking mechanism
  and neither does this driver. Therefore, when operating in
  PERIPHERAL mode, the application must provide such a mechanism to
  ensure that the MIBSPI peripheral is ready for the MIBSPI controller. The MIBSPI peripheral
  must call #MIBSPI_transfer() *before* the controller starts transmitting.
  Some example application mechanisms could include:
    - Timed delays on the MIBSPI controller to guarantee the MIBSPI peripheral is ready
      for a MIBSPI transaction.
    - A form of GPIO flow control from the peripheral to the MIBSPI controller to notify
      the controller when ready.

## Example Usage

Include the below file to access the APIs
\snippet Mibspi_sample.c include

Instance Open Example
\snippet Mibspi_sample.c open

Instance Close Example
\snippet Mibspi_sample.c close

Blocking Transfer Example
\snippet Mibspi_sample.c transfer_blocking

Non-Blocking Transfer Example
\snippet Mibspi_sample.c transfer_nonblocking

## API

\ref DRV_MIBSPI_MODULE
