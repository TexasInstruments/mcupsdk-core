
# MCSPI High Level Driver{#DRIVERS_MCSPI_HLD_PAGE}

## Features Supported

- Controller and Peripheral mode of operation
- Per transfer selection of different channels/chip select
- Blocking and non-blocking (Callback) transfers
- For low latency transfers, refer \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_32BIT and \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_8BIT example.
  This example uses polling mode of operation.
- DMA mode of operation
- Multi Word Acess. To use this feature, following requirement must be satisfied.
    - It is only supported in interrupt mode.
    - The channel selected must have the FIFO enabled.
    - Transmit and receive registers must write and read 32-bits respectively.
    - FIFO Trigger level must be 32 bit aligned i.e, It must be in power of 2.
    - Data Size must be 8 bits or 1 byte.
    - Total frame i.e spiTransaction.count must be in multiple of data size.

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

## DMA Transfer Size Limitation (12-bit Counter)

**Maximum McSPI DMA transfer: 4,095 WORDS per transaction**

When using MCSPI in DMA mode, the transfer is limited by the 12-bit transfer counter in the PDMA hardware. The counter tracks the number of **words** (not bytes) to transfer.

### 12-bit Counter Limitation

**Counter field:** 12-bit register (0x000 to 0xFFF = 0 to 4,095 decimal)

**CRITICAL - Register Overflow Risk:** Attempting a transfer count of 4,096 or higher causes register overflow:
```
4,096 decimal = 0x1000 (requires 13 bits)
              ↓ register overflow ↓
Wraps to 0x000 → Transfer FAILS → System may HANG
```

### Understanding Words vs. Bytes

The transaction->count field specifies the number of **WORDS** to transfer.
A word size depends on #MCSPI_Transaction.dataSize:

| Data Width | Word = | bufWidthShift |
|------------|--------|---------------|
| 1-8 bits   | 1 byte | 0 |
| 9-16 bits  | 2 bytes| 1 |
| 17-32 bits | 4 bytes| 2 |

### Byte Conversion Formula

```c
Total bytes = transaction->count << bufWidthShift
```

### Maximum Transfer Sizes by Data Width

| Data Width | Max Words | Calculation | Max Bytes |
|------------|-----------|-------------|-----------|
| 8-bit  | 4,095 | count << 0 | **4,095 bytes** |
| 16-bit | 4,095 | count << 1 | **8,190 bytes** |
| 32-bit | 4,095 | count << 2 | **16,380 bytes** |

### Real-World Examples

**Example 1: 8-bit data, transfer 1,000 bytes**
```c
transaction.dataSize = 8;
transaction.count = 1000;  // 1000 words = 1000 bytes
// Total bytes: 1000 << 0 = 1000 bytes (< 4095)  PASS
```

**Example 2: 16-bit data, transfer 5,000 bytes**
```c
transaction.dataSize = 16;
transaction.count = 2500;  // 2500 words = 5000 bytes
// Total bytes: 2500 << 1 = 5000 bytes (< 8190)  PASS
```

**Example 3: 8-bit data, transfer 5,000 bytes (FAILS)**
```c
transaction.dataSize = 8;
transaction.count = 5000;  // 5000 words = 5000 bytes
// Total bytes: 5000 << 0 = 5000 bytes (> 4095) FAIL
// API returns: MCSPI_TRANSFER_INVALID_PARAM
```

### API Validation Behavior

When calling #MCSPI_transfer() in DMA mode, the driver validates:

```c
uint32_t transferBytes = transaction->count << bufWidthShift;

if (transferBytes > 4095)  // 12-bit limit (0xFFF max)
{
    return MCSPI_TRANSFER_INVALID_PARAM;
}
else
{
    // Proceed with transfer
}
```

### Workaround for Large Transfers (> 4,095 words)

To transfer data exceeding the limit, split into multiple transactions:

```c
// Example: Transfer 10,000 bytes with 8-bit data width
uint32_t totalWords = 10000;  // For 8-bit: words = bytes
uint32_t wordIndex = 0;
uint32_t maxWordsPerTransfer = 4095;  // 12-bit limit

MCSPI_Transaction transaction;
MCSPI_Transaction_init(&transaction);
transaction.dataSize = 8;
transaction.csDisable = FALSE;  // Keep CS asserted between transfers

while (wordIndex < totalWords)
{
    uint32_t wordsRemaining = totalWords - wordIndex;
    uint32_t wordsToTransfer = (wordsRemaining > maxWordsPerTransfer) ?
                               maxWordsPerTransfer : wordsRemaining;

    // Last chunk: deassert CS
    if ((wordIndex + wordsToTransfer) >= totalWords)
    {
        transaction.csDisable = TRUE;
    }

    // Configure chunk
    transaction.count = wordsToTransfer;
    transaction.txBuf = &txBuffer[wordIndex];
    transaction.rxBuf = &rxBuffer[wordIndex];

    // Transfer
    int32_t status = MCSPI_transfer(handle, &transaction);
    if (status != MCSPI_TRANSFER_COMPLETED)
    {
        DebugP_log("Transfer failed at word offset %u\r\n", wordIndex);
        break;
    }

    wordIndex += wordsToTransfer;
}
```

### For 16-bit or 32-bit Data

When using wider data widths, the word count calculation differs:

```c
// 16-bit data: 10,000 bytes transfer
uint32_t totalBytes = 10000;
uint32_t totalWords = totalBytes / 2;  // 5000 words
uint32_t maxWordsPerTransfer = 4095;   // Hardware limit

// Split into:
//   - 4095 words (8190 bytes)
//   - 905 words  (1810 bytes)
```

\endcond
## Usage Overview

### API Sequence

To use the MCSPI driver to send data over the SPI bus, the application
calls the following APIs:

- #MCSPI_init(): Initialize the MCSPI driver.
- #MCSPI_OpenParams_init():  Initialize a #MCSPI_OpenParams structure with default
  values.  Then change the parameters from non-default values as needed.
- #MCSPI_open():  Open an instance of the MCSPI driver, passing the
  initialized parameters, or NULL, and an index to the configuration to
  open (detailed later).
- #MCSPI_transfer():  Transmit/receive data.  This function takes a
  #MCSPI_Transaction as an argument that describes the transfer that is requested.
- #MCSPI_close():  Close the MCSPI instance.
- #MCSPI_deinit(): De-initialize the MCSPI driver.

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

NOTE: The size of txBuf and RxBuf must be greater than the data size bits, if data size
is not a multiple of 8 bits. Mask the data size bits in txBuf and rxBuf as the
remaining bits will be discarded. For example, consider datasize = 18 bits then
txBuf and rxBuf size should be uint32_t and the mask bits should be 0x3FFFF.

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
