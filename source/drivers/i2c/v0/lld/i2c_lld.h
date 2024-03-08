/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \defgroup DRV_I2C_LLD_MODULE APIs for I2C LLD
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the I2C LLD module. The APIs
 *  can be used by other drivers to get access to I2C and also by
 *  application to initiate transaction operations.
 *
 *  The I2C LLD header file should be included in an application as follows:
 *  \code
 *  #include <drivers/i2c/v0/lld/i2c_lld.h>
 *  \endcode
 *
 *  @{
 */

/**
 *  \file v0/lld/i2c_lld.h
 *  \brief I2C LLD Driver API/interface file.
 */

#ifndef I2C_LLD_H_
#define I2C_LLD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include <drivers/i2c/v0/cslr_i2c.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define I2C_MAX_NUM_OWN_TARGET_ADDR         (4U)

/**
 * \anchor   I2C_StatusCode
 * @name Return status
 */
/**@{*/

/** \brief Return status when the API execution was successful */
#define I2C_STS_SUCCESS                     ((int32_t) 0)
/** \brief Return status when the API execution was not successful
 *  due to a generic Error */
#define I2C_STS_ERR                         ((int32_t)-1)
/** \brief Return status when the API execution was not successful
 *  due to a time out */
#define I2C_STS_ERR_TIMEOUT                 ((int32_t)-2)
/** \brief Return status when the API execution failed due invalid parameters */
#define I2C_STS_ERR_INVALID_PARAM           ((int32_t)-3)
/** \brief Return status when the API execution failed due to driver busy */
#define I2C_STS_ERR_BUS_BUSY                ((int32_t)-4)
/** \brief Return status when the API execution failed due to no
 *  Acknowledgement from target */
#define I2C_STS_ERR_NO_ACK                  ((int32_t)-5)
/** \brief Return status when the API execution failed due to loss in
 *  arbitration during transmission */
#define I2C_STS_ERR_ARBITRATION_LOST        ((int32_t)-6)

#define I2C_STS_ERR_ACCESS_ERROR            ((int32_t)-7)
/*! \cond PRIVATE */
#define I2C_STS_RESTART                     ((int32_t)-8)
/*! \endcond */
/**@}*/

/**
 *  \anchor I2cTimeoutValues
 *  @name Timeout values
 */
/** @{ */

/** \brief Value to use when needing a timeout of zero or NO timeout,
 *  return immediately on resource not available. */
#define I2C_NO_WAIT                         ((uint32_t)0)
/** \brief Value to use when needing a timeout of infinity or
 *  wait forver until resource is available */
#define I2C_WAIT_FOREVER                    ((uint32_t)-1)

/** @} */

/** @name I2C Driver states
 */
/** @{ */

/** \brief I2C driver is in Reset State prior to driver init
 *  and post driver deinit */
#define I2C_STATE_RESET                     ((uint8_t) 0U)
/** \brief I2C driver accepts runtime APIs only Ready State,
 *  otherwise return error */
#define I2C_STATE_IDLE                      ((uint8_t) 1U)
/** \brief I2C driver is busy performing operation with peripherals,
 *  return error when APIs are invoked */
#define I2C_STATE_BUSY                      ((uint8_t) 2U)
/** \brief I2C driver ran into error, returns error for all APIs
 *  other than deinit in this state */
#define I2C_STATE_ERROR                     ((uint8_t) 3U)

/** @} */

/**
 *  \anchor I2cBitRates
 *  \name MACROS used to select one of the standardized bus bit rates for
 *  I2C communication. Deafults to I2C_400KHZ
 *  @{
 */

/** \brief I2C bus frequency : 100KHz */
#define I2C_100KHZ                          ((uint8_t) 0U)
/** \brief I2C bus frequency : 400KHz */
#define I2C_400KHZ                          ((uint8_t) 1U)
/** \brief I2C bus frequency : 1.0MHz */
#define I2C_1P0MHZ                          ((uint8_t) 2U)
/** \brief I2C bus frequency : 3.4MHz */
#define I2C_3P4MHZ                          ((uint8_t) 3U)

/** @} */

/**
 * \anchor   I2CLLD_MemoryAddrSize
 * \name MACROS for the possible values of Memory Write/Read API parameter.
 * @{
 */

/** \brief I2C Target Internal Memory address 8 bits */
#define I2C_MEM_ADDR_SIZE_8_BITS            ((uint8_t) 1U)
/** \brief I2C Target Internal Memory address 16 bits */
#define I2C_MEM_ADDR_SIZE_16_BITS           ((uint8_t) 2U)

/** @} */

/**
 *  \anchor I2C_State
 *  \name MACROS used to define the state of the I2C Driver State Machine
 *  @{
 */

/** \brief  I2C is in Write state */
#define I2C_WRITE_STATE                     ((uint8_t) 10U)
/** \brief  I2C is in Write state */
#define I2C_READ_STATE                      ((uint8_t) 11U)
/** \brief  I2C is trasferring in target mode */
#define I2C_TARGET_XFER_STATE               ((uint8_t) 12U)
/** \brief  I2C is restarting trasfer in target mode */
#define I2C_TARGET_RESTART_STATE            ((uint8_t) 13U)

/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  I2C Transaction
 *
 *  This structure defines the nature of the I2C transaction.
 *
 *  I2C Controller Mode:
 *  This structure specifies the buffer and buffer's size that is to be
 *  written to and read from the I2C target peripheral.
 *
 */
typedef struct {

/** [IN] Data buffer from which data to be written to the target */
    uint8_t                                 *writeBuf;
/** [IN] Number of bytes to be written to the target */
    uint32_t                                writeCount;
/** [IN] Data buffer to which data to be read from the target */
    uint8_t                                 *readBuf;
/** [IN] Number of bytes to be read from the target */
    uint32_t                                readCount;

} I2CLLD_Transaction;

/**
 *  \brief  I2C Message
 *
 *  This structure defines the complete I2C transfer.
 *
 *  I2C Controller Mode:
 *  This structure specifies the transaction objects used to initiate
 *  transfer to the I2C target peripheral.
 */
typedef struct {

/** [IN] Array of transaction objects */
    I2CLLD_Transaction                      *txn;
/** [IN] Number of transaction objects */
    uint32_t                                txnCount;
/** [IN] Controller mode: input field from user to set the address of I2C target
 *  target mode: output field from driver to report the address of a
 *  target channel when multi-target channels are supported, if only one
 *  channel is supported, this field is ignored */
    uint32_t                                targetAddress;
/** [IN] Not used by driver. Used for passing argument to callback function */
    void                                    *arg;
/** [IN] Timeout value for i2c transaction in micro Seconds */
    uint32_t                                timeout;
/** [IN] I2C controller or target mode */
    bool                                    controllerMode;
/** [IN] Expand target address: true: 10-bit address mode,
 *  false: 7-bit address mode */
    bool                                    expandSA;

} I2CLLD_Message;

/**
 *  \brief Data structure used with #I2C_lld_write(), #I2C_lld_writeIntr(),
 *  #I2C_lld_read(), #I2C_lld_readIntr()
 */
typedef struct {

    uint32_t                                deviceAddress;
    /**< [IN] Target device address */
    uint8_t                                 *buffer;
    /**< [IN] Pointer to Read or Write buffer */
    uint32_t                                size;
    /**< [IN] Size of Read or Write buffer */
    bool                                    expandSA;
    /**< [IN] Expand target address: 10-bit address mode,
     *   false: 7-bit address mode */

} I2C_ExtendedParams;

/**
 *  \brief Data structure used with #I2C_lld_mem_write(),
 *  #I2C_lld_mem_writeIntr(), #I2C_lld_mem_read(), #I2C_lld_mem_readIntr()
 */
typedef struct {

    I2C_ExtendedParams                      extendedParams;
    /**< [IN] Extended Parameters */
    uint32_t                                memAddr;
    /**< [IN] Memory address to write to or read from */
    uint8_t                                 memAddrSize;
    /**< [IN] Memory address size \ref I2CLLD_MemoryAddrSize */

} I2C_Memory_ExtendedParams;


/**
 *  \brief  I2C Target Transaction
 *
 *  This structure defines the complete I2C transfer.
 *
 *  This structure specifies the buffer and buffer's size that is to be
 *  read from or written to the I2C controller. In restart condition,
 *  readBuf/writeBuf and readCount/writeCount are used repeatedly for
 *  every start in one transfer. When each restart happens, driver will
 *  call back to application with the restart transfer status, and
 *  application should save the data transferred in the previous start,
 *  and provide the new data to the current start. When all the starts
 *  complete (stop condition), driver will call back to application with
 *  transfer success status, and readBuf/writeBuf and readCount/writeCount
 *  will only record the data transferred in the last start condition.
 */
typedef struct {

/** Buffer containing data to be written to controller */
    uint8_t                                 *writeBuf;
/** Number of bytes to be written to the controller */
    uint32_t                                writeCount;
/** Buffer to which data from controller is to be read into */
    uint8_t                                 *readBuf;
/** Number of bytes to be read to the controller */
    uint32_t                                readCount;
/** Timeout value for i2c transaction in Micro Seconds */
    uint32_t                                timeout;
/** Expand target address: true: 10-bit address mode,
 *  false: 7-bit address mode */
    bool                                    expandSA;

} I2CLLD_targetTransaction;

/* ========================================================================== */
/*                      Function pointers Declarations                        */
/* ========================================================================== */

/**
 *  \brief  The definition of a get System Tick function used by
 *  the I2C driver to keep track of time
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*I2C_Clock_getTicks) (void);

/**
 *  \brief  The definition of a micro seconds to ticks function used by
 *  the I2C driver to get ticks from microseconds
 *
 *  \param usecs                        Micro Seconds
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*I2C_Clock_usecToTicks) (uint64_t usecs);

/**
 *  \brief  The definition of a sleep function used by
 *  the I2C driver for delay
 *
 *  \param usec                         Micro Seconds
 *
 */
typedef void (*I2C_Clock_uSleep) (uint32_t usec);

/**
 *  \brief  The definition of a transfer completion callback function used by
 *  the I2C driver when used in Controller Callback Mode
 *
 *  \param args                         Void Pointer
 *  \param msg                          Pointer to I2CLLD_Message Object
 *  \param transferStatus               Transfer Status
 */
typedef void (*I2C_lld_transferCompleteCallback) (void *args,
                                                  const I2CLLD_Message *msg,
                                                  int32_t transferStatus);
/**
 *  \brief  The definition of a transfer completion callback function used by
 *  the I2C driver when used in Target Mode
 *
 *  \param args                         Void Pointer
 *  \param targetTxn                    Pointer to I2CLLD_targetTransaction
 *                                      Object
 *  \param transferStatus               Transfer Status
 */
typedef void (*I2C_lld_targetTransferCompleteCallback) (void *args,
                                    const I2CLLD_targetTransaction * targetTxn,
                                    int32_t transferStatus);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  I2C Driver Object
 */
typedef struct {

/**
 * @name Initialization Parameters
 */

/** Peripheral base address - CPU view */
    uint32_t                                baseAddr;
/** I2C Peripheral interrupt vector */
    uint32_t                                intrNum;
/** I2C bit rate */
    uint8_t                                 bitRate;
/** I2C input functional clock */
    uint32_t                                funcClk;
/** I2C own target addresses */
    uint32_t                                ownTargetAddr[I2C_MAX_NUM_OWN_TARGET_ADDR];
/** Clock_getTicks Function Pointer used by driver */
    I2C_Clock_getTicks                      Clock_getTicks;
/** Clock_usecToTicks Function Pointer used by driver */
    I2C_Clock_usecToTicks                   Clock_usecToTicks;
/** Clock_uSleep Function Pointer used by driver */
    I2C_Clock_uSleep                        Clock_uSleep;
/** Callback Function Pointer in Controller Mode */
    I2C_lld_transferCompleteCallback        transferCompleteCallback;
/** Callback Function Pointer in Target Mode */
    I2C_lld_targetTransferCompleteCallback  targetTransferCompleteCallback;

/**
 * @name User Parameters
 */

/** Stores the I2C state */
    uint8_t                                 state;
/** Stores the error status of i2c Controller */
    uint32_t                                intStatusErr;

/**
 * @name Transfer Parameters
 */

/** I2C transaction variables */
    I2CLLD_Message                          *currentMsg;
/** Pointer to current I2C target transaction */
    I2CLLD_targetTransaction                *currentTargetTransaction;
/** Internal inc. writeBuf index */
    uint8_t                                 *writeBufIdx;
/** Internal dec. writeCounter */
    uint32_t                                writeCountIdx;
/** Internal inc. readBuf index */
    uint8_t                                 *readBufIdx;
/** Internal dec. readCounter */
    uint32_t                                readCountIdx;
/** Current msg Start ticks */
    uint32_t                                startTicks;
/** Stores current transaction count */
    uint32_t                                currentTxnCount;
/** i2cMessage object used by read/write APIs */
    I2CLLD_Message                          i2cMsg;
/** i2cTransaction array for read and write */
    I2CLLD_Transaction                      i2ctxn;
/** Pointer to data array for memory read and write */
    uint8_t                                 *dataArray;
/** Data buffer for storing address during memory read and write */
    uint8_t                                 addBuff[2];
/** Stores memory address size for memory read and write Operation */
    uint8_t                                 memAddrSize;
/** Memory transaction in progress: true: Memory transaction in progress,
 *  false: No memory transaction in progress */
    bool                                    memTxnActive;
/** I2CLLD_targetTransaction object used by transfer API */
    I2CLLD_targetTransaction                i2cTargetTransaction;
/** Pointer to be used by application to store miscellaneous data */
    void*                                   args;

}I2CLLD_Object, *I2CLLD_Handle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief This API Initializes the I2C instance
 *
 *  \param  handle              [IN] Handle to the I2C instance used
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_init(I2CLLD_Handle handle);

/**
 *  \brief This API De-Initializes the I2C instance
 *
 *  \param  handle              [IN] Handle to the I2C instance used
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_deInit(I2CLLD_Handle handle);

/**
 *  \brief API to set default values of I2CLLD_Transaction in transaction
 *
 *  \param transaction          [IN] Pointer to the structure to be initialized
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_Transaction_init(I2CLLD_Transaction *transaction);

/**
 *  \brief API to set default values of I2CLLD_Message in msg
 *
 *  \param msg                  [IN] Pointer to the structure to be initialized
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_Message_init(I2CLLD_Message *msg);

/**
 *  \brief API to initiate the write transaction in polled mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param extendedParams       [IN] Pointer to structure containing transfer
 *                                   parameters
 *  \param timeout              [IN] Timeout for read operation in
 *                                   Micro Seconds \ref I2cTimeoutValues
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_write(I2CLLD_Handle handle,
                      I2C_ExtendedParams *extendedParams,
                      uint32_t timeout);

/**
 *  \brief API to initiate the write transaction in Interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param extendedParams       [IN] Pointer to structure containing
 *                                   transfer parameters
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_writeIntr(I2CLLD_Handle handle,
                          I2C_ExtendedParams *extendedParams);

/**
 *  \brief API to initiate the read transaction in polled mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param extendedParams       [IN] Pointer to structure containing transfer
 *                                   parameters
 *  \param timeout              [IN] Timeout for read operation in
 *                                   Micro Seconds \ref I2cTimeoutValues
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_read(I2CLLD_Handle handle,
                     I2C_ExtendedParams *extendedParams,
                     uint32_t timeout);

/**
 *  \brief API to initiate the read transaction in Interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param extendedParams       [IN] Pointer to structure containing
 *                                   transfer parameters
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_readIntr(I2CLLD_Handle handle,
                         I2C_ExtendedParams *extendedParams);

/**
 *  \brief Function to initiate a transfer from I2C in interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param mem_extendedParams   [IN] Pointer to structure containing transfer
 *                                   parameters
 *  \param timeout              [IN] Timeout for read operation in
 *                                   Micro Seconds \ref I2cTimeoutValues
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_mem_write(I2CLLD_Handle handle,
                          I2C_Memory_ExtendedParams *mem_extendedParams,
                          uint32_t timeout);

/**
 *  \brief Function to initiate a transfer from I2C in interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param mem_extendedParams   [IN] Pointer to structure containing transfer
 *                                   parameters
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_mem_writeIntr(I2CLLD_Handle handle,
                              I2C_Memory_ExtendedParams *mem_extendedParams);

/**
 *  \brief Function to initiate a transfer from I2C in interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param mem_extendedParams   [IN] Pointer to structure containing transfer
 *                                   parameters
 *  \param timeout              [IN] Timeout for read operation in
 *                                   Micro Seconds \ref I2cTimeoutValues
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_mem_read(I2CLLD_Handle handle,
                         I2C_Memory_ExtendedParams *mem_extendedParams,
                         uint32_t timeout);

/**
 *  \brief Function to initiate a transfer from I2C in interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param mem_extendedParams   [IN] Pointer to structure containing transfer
 *                                   parameters
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_mem_readIntr(I2CLLD_Handle handle,
                             I2C_Memory_ExtendedParams *mem_extendedParams);

/**
 *  \brief Function to initiate a transfer from I2C in polled mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param msg                  [IN] Pointer to the I2CLLD_Message structure
 *                                   that contains values for this specific
 *                                   transfer
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_transferPoll(I2CLLD_Handle handle, I2CLLD_Message *msg);

/**
 *  \brief Function to initiate a transfer from I2C in interrupt mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param msg                  [IN] Pointer to the I2CLLD_Message structure
 *                                   that contains values for this specific
 *                                   transfer
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_transferIntr(I2CLLD_Handle handle, I2CLLD_Message *msg);

/**
 *  \brief Function to initiate a transfer from I2C in target mode
 *
 *  \param handle               [IN] Handle to the I2C instance used
 *  \param txn                  [IN] Poiter to the I2CLLD_targetTransaction
 *                                   structure that contains values for this
 *                                   specific transfer
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_targetTransferIntr(I2CLLD_Handle handle,
                                   I2CLLD_targetTransaction *txn);

/**
 *  \brief Function to probe I2C
 *
 *  \param handle               [IN] handle to the I2C instance used
 *  \param targetAddr           [IN] address of the target to probe
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_probe(I2CLLD_Handle handle, uint32_t targetAddr);

/**
 *  \brief Function to set the bus frequency
 *
 *  \param handle               [IN] handle to the I2C instance used
 *  \param busFrequency         [IN] frequency value to be set \ref I2cBitRates
 *
 *  \return \ref I2C_StatusCode
 */
int32_t I2C_lld_setBusFrequency(I2CLLD_Handle handle, uint32_t busFrequency);

/**
 *  \brief Function to recover the bus in case of lockup
 *
 *  \param handle         [IN] handle to the I2C
 *  \param i2cDelay       [IN] the length of delay for sending
 *                             clock pulses to target
 *
 *  \return \ref I2C_StatusCode
 */
int32_t     I2C_lld_recoverBus(I2CLLD_Handle handle, uint32_t i2cDelay);

/* ========================================================================== */
/*                        ISR Function Declarations                           */
/* ========================================================================== */

/**
 *  \brief  This is the I2C Controller ISR and can be used as IRQ handler in Controller mode.
 *
 *  \param  args                [IN] Argument to the ISR
 *
 */
void I2C_lld_controllerIsr(void *args);

/**
 *  \brief  This is the I2C Target ISR and can be used as IRQ handler in Target mode.
 *
 *  \param  args                [IN] Argument to the ISR
 *
 */
void I2C_lld_targetIsr(void *args);

#ifdef __cplusplus
}
#endif

#endif /* I2C_LLD_H_ */

/** @} */
