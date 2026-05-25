# MCSPI Low Level Driver{#DRIVERS_MCSPI_LLD_PAGE}

## Features Supported

- Controller and Peripheral mode of operation
- Per transfer selection of different channels/chip select
- Non-blocking (Callback) transfers
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

The McSPI DMA transfer counter is a **12-bit field** that tracks the number of **words** (not bytes) to transfer. A word is defined by the configured data width.

### Transfer Counter Architecture

**Counter field:** 12-bit register (0x000 to 0xFFF = 0 to 4,095 decimal)

**Important:** Attempting a transfer count of 4,096 or higher causes register overflow:
- 4,096 decimal = 0x1000 (requires 13 bits)
- Overflow wraps to 0x000
- Transfer fails and system may hang

### Word Definition and Byte Conversion

The **word** size depends on the SPIDAT0 WL[3:0] field (word length in bits):

| Data Width | Word Size | bufWidthShift | Formula |
|------------|-----------|---------------|---------|
| 1-8 bits   | 1 byte    | 0             | bytes = count × 1 |
| 9-16 bits  | 2 bytes   | 1             | bytes = count × 2 |
| 17-32 bits | 4 bytes   | 2             | bytes = count × 4 |

### Maximum Transfer Sizes

**Since counter max = 4,095 words:**

| Data Width | Max Words | Max Bytes |
|------------|-----------|-----------|
| 8-bit      | 4,095     | **4,095 bytes** |
| 16-bit     | 4,095     | **8,190 bytes** |
| 32-bit     | 4,095     | **16,380 bytes** |

### Calculation Method

```
Total bytes = transaction->count × (1 << bufWidthShift)
           OR
Total bytes = transaction->count << bufWidthShift
```

**Example:**
- Data width: 8-bit (bufWidthShift = 0)
- Count: 4,095 words
- Total bytes: 4,095 << 0 = 4,095 bytes

- Data width: 16-bit (bufWidthShift = 1)
- Count: 4,095 words
- Total bytes: 4,095 << 1 = 8,190 bytes

- Data width: 8-bit (bufWidthShift = 0)
- Count: 4,096 words
- Total bytes: 4,096 << 0 = 4,096 bytes  **OVERFLOW**

### Workaround for Large Transfers

For transfers exceeding 4,095 words:

1. **Split into multiple transactions** (each ≤ 4,095 words)
2. **Use transaction chaining** if available
3. **Loop-based approach** for sequential transfers

### Implementation Detail:

The DMA transfer size validation is performed in all DMA API functions:
- `#MCSPI_lld_writeDma()`
- `#MCSPI_lld_readDma()`
- `#MCSPI_lld_readWriteDma()`

The driver validates that the transaction count does not exceed 4,095 words.

**Validation formula used in driver:**
```c
uint32_t transferBytes = count << bufWidthShift;
if (transferBytes > MCSPI_DMA_MAX_TRANSFER_BYTES)  // 4095
{
    return MCSPI_INVALID_PARAM;  // Reject transfer
}
```

If the limit is exceeded, the API returns `MCSPI_INVALID_PARAM` status.

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

\cond !SOC_AM62X && !SOC_AM65X
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
