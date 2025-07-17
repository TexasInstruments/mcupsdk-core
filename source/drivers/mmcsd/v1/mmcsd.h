/*
 *  Copyright (C) 2022-2025 Texas Instruments Incorporated
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

/**
 *  \defgroup DRV_MMCSD_HLD_MODULE APIs for MMCSD HLD
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MMCSD module. The APIs
 *  can be used by other drivers to get access to MMCSD and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v1/mmcsd.h
 *
 *  \brief MMCSD Driver API/interface file.
 */

#ifndef MMCSD_H_
#define MMCSD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_mmcsd.h>
#include <drivers/mmcsd/v1/lld/mmcsd_lld.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor MMCSD_TransferMode
 *  \name MACROS used to select the transfer mode
 *
 *  MMCSD_MODE_BLOCKING block task execution while a Transfer is in progress
 *  MMCSD_MODE_CALLBACK does not block task execution; but calls a callback
 *  function when the MMCSD transfer has completed
 *
 *  @{
 */
#define MMCSD_MODE_BLOCKING                       ((uint32_t) 0U)
#define MMCSD_MODE_CALLBACK                       ((uint32_t) 1U)

/** @} */

struct MMCSD_Config_s;

typedef struct MMCSD_Config_s* MMCSD_Handle;

/* ========================================================================== */
/*                      Function Pointers Declarations                        */
/* ========================================================================== */

/**
 *  \brief  MMCSD callback function
 *
 *  User definable callback function prototype. The MMCSD driver will call the
 *  defined function and pass in the MMCSD driver's handle, the pointer to the
 *  MMCSD transaction that just completed, and the status of Transaction.
 *
 *
 *  \param  mmcsdHandle          MMCSD_Handle
 *  \param  transferStatus  Results of the MMCSD transaction
 */

typedef void (*MMCSD_txnCallbackFxn) (MMCSD_Handle mmcsdHandle,
                                      int32_t transferStatus);

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 *  \brief MMCSD instance attributes - used during initialization
 */
typedef struct
{
    /* Pointer to eMMC/SD device data structure. Memory for this structure
    has to be allocated in application */
	void                    *deviceData;
    /*  Pointer to a 512 byte dataBuffer used for temporary data transactions
     *  internal to driver like ECSD read, tuning etc.
     *  To be allocated by application */
    uint8_t                 *dataBuf;
    /**< EDMA instance used for MMCSD transfer */
    uint32_t                edmaInst;
    /** Blocking or Callback mode */
    uint8_t                 transferMode;
    /** MMCSD Callback function pointer */
    MMCSD_txnCallbackFxn    txnCallbackFxn;

} MMCSD_Params;

/**
 *  \brief MMCSD instance attributes - used during init time
 */
typedef struct
{
    uint32_t                baseAddr;
	/**< MMCSD subsystem registers base address */
    uint32_t                inputClk;
    /**< MMC input functional clock */
    bool                    enableDma;
    /**< DMA Enable */
    bool                    intrEnable;
    /**< Module interrupt enable */
    uint32_t                intrPriority;
    /**< Priority of interrupt for MMCSD ISR */
    uint32_t                intrNum;
    /**< Module interrupt vector */
    uint32_t                eventId;
    /**< Module interrupt event ID */
    uint32_t                cardType;
    /**< Type of card */
    uint32_t                busWidth;
    /**< Supported bus width */
    bool                    autoAssignMaxSpeed;
    /**< Auto Assign Maximum Speed */
    uint32_t                uaBusSpeed;
    /**< User Assigned Bus Speed Mode */

} MMCSD_Attrs;

/**
 *  \brief MMCSD driver object
 */
typedef struct
{
    uint32_t                cardType;
    /**< Type of card */
    bool                    isOpen;
    /**< Flag to indicate if the instance is already open */
    SemaphoreP_Object       mutex;
    /**< Grants Exclusive Access to MMCSD */
    SemaphoreP_Object       xferCompleteSemObj;
    /**< Transfer complete semaphore */
    HwiP_Object             hwiObj;
    /**< Interrupt object */
    MMCSDLLD_Object         mmcsdLldObject;
    /**< MMCSD driver object for LLD */
    MMCSDLLD_Handle         mmcsdLldHandle;
    /**< MMCSD driver handle for LLD */
    MMCSDLLD_InitObject     mmcsdLldInitObject;
    /**< MMCSD driver Init Object for LLD */

    uint32_t                transferMode;
    /**< Transfer Mode, Blocking or Callback */
    MMCSD_txnCallbackFxn    txnCallbackFxn;
    /**< Transaction complete Callback function pointer */

} MMCSD_Object;

/**
 *  \brief  MMCSD Global configuration
 *
 *  The MMCSD_Config structure contains a set of pointers used to characterize
 *  the MMCSD driver implementation.
 *
 */
typedef struct MMCSD_Config_s
{
    MMCSD_Attrs             *attrs;
    /**< Pointer to driver specific hardware attributes */
    MMCSD_Object            *object;
    /**< Pointer to driver specific data object */
} MMCSD_Config;

/* ========================================================================== */
/*                                Externs                                     */
/* ========================================================================== */

/** \brief Externally defined driver configuration array */
extern MMCSD_Config         gMmcsdConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t             gMmcsdConfigNum;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function initializes the MMCSD module
 */
void MMCSD_init(void);

/**
 *  \brief  This function de-initializes the MMCSD module
 */
void MMCSD_deinit(void);

/**
 *  \brief  Initialize data structure with defaults
 *
 *  \param  mmcsdParams [out] Initialized parameters
 */
void MMCSD_Params_init(MMCSD_Params *mmcsdParams);

/**
 *  \brief  This function opens a given MMCSD peripheral
 *
 *  \pre    MMCSD controller has been initialized using #MMCSD_init()
 *
 *  \param  index       Index of config to use in the *MMCSD_Config* array
 *  \param  openParams  Pointer to parameters to open the driver with
 *
 *  \return A #MMCSD_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_close()
 */
MMCSD_Handle MMCSD_open(uint32_t index, const MMCSD_Params *openParams);

/**
 *  \brief  Function to close a MMCSD peripheral specified by the MMCSD handle
 *
 *  \pre    #MMCSD_open() has to be called first
 *
 *  \param  handle      #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \sa     #MMCSD_open()
 */
void MMCSD_close(MMCSD_Handle handle);

/**
 *  \brief  This function returns the handle of an open MMCSD Instance from the instance index
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  index Index of config to use in the *MMCSD_Config* array
 *
 *  \return An #MMCSD_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
MMCSD_Handle MMCSD_getHandle(uint32_t index);

/**
 *  \brief  Function to perform block reads from the MMC/SD media
 *
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *  \param  buf        Pointer to a buffer to which the data is to be read into
 *  \param  startBlk   Block to start reading data from
 *  \param  numBlks    Number of blocks to read
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #MMCSD_open
 */
int32_t MMCSD_read(MMCSD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the MMC/SD media
 *
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *  \param  buf        Pointer to a buffer to which the data is to be read into
 *  \param  startBlk   Block to start reading data from
 *  \param  numBlks    Number of blocks to read
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #MMCSD_open
 */
int32_t MMCSD_write(MMCSD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  This function returns the block size of the MMC/SD media connected to the MMCSD controller
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \return Block size of the media
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
uint32_t MMCSD_getBlockSize(MMCSD_Handle handle);

/**
 *  \brief  This function returns the block count of User Data Area of the MMC/SD media connected to the MMCSD controller
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \return Block size of the media
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
uint32_t MMCSD_getBlockCount(MMCSD_Handle handle);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* MMCSD_H_ */
