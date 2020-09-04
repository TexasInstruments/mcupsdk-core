/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PRU_IPC_H_
#define PRU_IPC_H_

/**
 *  \defgroup DRV_PRU_IPC_MODULE APIs for PRU IPC: R5F-PRU communication
 *
 *  This module contains APIs which are used by the R5F cores to transfer data
 *  to and from PRU cores.
 *
 *  @{
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/pruicss.h>
#include <kernel/dpl/SystemP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief   A handle that is returned from a #PRU_IPC_open() call. This handle
 *          is required for calling other PRU_IPC APIs.
 */
typedef struct PRU_IPC_Config_s         *PRU_IPC_Handle;

/**
 * \brief   A Callback function required to register callback on data receive if
 *          receive interrupt is enabled.
 */
typedef void (*PRU_IPC_CallbackFxn)(void *args);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \anchor   Config_Mem_Struct
 * \name PRU_IPC Pru Config Memory (DRAM) Offsets
 * @{
 */
typedef struct {
    uint16_t blockId;
    uint8_t  dataSize;
    uint8_t  noOfBuffers;
    uint16_t blockSize;
    uint16_t noOfBlocks;
    uint32_t bufferAddrs;
} Config_Mem_Struct;
/** @} */

typedef struct PRU_IPC_Attrs_s {
    uint16_t  dataSize;          /**< Size of 1 data packet in bytes */
    uint16_t  blockSize;         /**< Size of each Block in terms of data packets */
    uint16_t  noOfBlocks;        /**< Total Blocks per Buffer */
    uint16_t  noOfBuffers;       /**< Total Buffers to reserve for shared memory */
    uint32_t  *bufferAddrs;      /**< Buffers' base addresses */
    Config_Mem_Struct *config;   /**< PRU Mem Address where Configurables' Info for IPC will be stored */
    uint32_t  enableRxInt;       /**< Enable interrupt on Receiving Data */
    uint32_t  pruEvtoutNum;      /**< Event number for the interrupt triggered by this instance */
    uint32_t  sysEventNum;       /**< System event number, used to clear interrupt */
    uint32_t  r5fIntrNum;        /**< Interrupt number on R5F side */
    uint32_t  enableTxInt;       /**< Enable interrupt to PRU core */
    uint32_t  txEventNum;        /**< Event number for the interrupt sent to PRU */
    uint32_t  blockSizeBytes;    /**< blockSize * dataSize */
} PRU_IPC_Attrs;

typedef struct PRU_IPC_Params_s {
/** #PRUICSS_Handle of ICSS with which the IPC will take place */
    PRUICSS_Handle          pruicssHandle;
/** PRU_IPC_CallbackFxn     transferCallbackFxn; Callback function pointer */
    PRU_IPC_CallbackFxn     transferCallbackFxn;
} PRU_IPC_Params;

typedef struct PRU_IPC_Object_s {
/** PRU_IPC open parameters as provided by user */
    PRU_IPC_Params  pruIpcParams;
} PRU_IPC_Object;

/*!
 * \brief   This struct is used to store configurables for PRU_IPC
 */
typedef struct PRU_IPC_Config_s {
/** Pointer to a driver specific data object */
    PRU_IPC_Object       *object;
/** Pointer to a driver specific attributes structure */
    PRU_IPC_Attrs  const *attrs;
} PRU_IPC_Config;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief      This function initializes the PRU_IPC module
 */
void PRU_IPC_init(void);

/**
 *  \brief      This function deinitializes the PRU_IPC module
 */
void PRU_IPC_deinit(void);

/**
 *  \brief      Function to set default values of PRU_IPC_Params in params
 *
 *  \param[in]  params pointer to the structure to be initialized
 */
void PRU_IPC_Params_init(PRU_IPC_Params *params);

/**
 *  \brief      To initialize and configure PRU_IPC instance
 *
 *  \param[in]  idx  Identifier for instance to use (generated by sysconfig)
 *  \param[in]  params pointer to the #PRU_IPC_Params structure
 *
 *  \return     #PRU_IPC_Handle in case of success, NULL otherwise
 */
PRU_IPC_Handle PRU_IPC_open(uint32_t idx , PRU_IPC_Params *params);

/**
 *  \brief      To stop/end PRU_IPC instance
 *
 *  \param[in]  handle  #PRU_IPC_Handle returned from #PRU_IPC_open()
 */
void PRU_IPC_close(PRU_IPC_Handle handle);

/**
 *  \brief      Reads the id of last written block by PRU from the PRU-Config memory
 *
 *  \param[in]  handle  #PRU_IPC_Handle returned from #PRU_IPC_open()
 *
 *  \return     Returns the id of last written block by PRU
 */
uint16_t PRU_IPC_getBlockId(PRU_IPC_Handle handle);

/**
 *  \brief      Reads the data from the configured shared memory
 *
 *  \param[in]  handle  #PRU_IPC_Handle returned from #PRU_IPC_open()
 *  \param[in,out]  container  To store the data read from the configured shared memory,
 *                  type: 2D Array - int32_t container[BUFFERS][BLOCKSIZE]
 *
 *  \return     #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRU_IPC_getData(PRU_IPC_Handle handle, void *container);

/**
 *  \brief      Send/Write data to the configured shared memory (generates interrupt to PRU  if it is enabled)
 *
 *  \param[in]  handle  #PRU_IPC_Handle returned from #PRU_IPC_open()
 *  \param[in]  container  To write the data to the configured shared memory
 *                  type: 2D Array - int32_t container[BUFFERS][BLOCKSIZE]
 *
 *  \return     #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRU_IPC_sendData(PRU_IPC_Handle handle, void *container);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef PRU_IPC_H_ */
