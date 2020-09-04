/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 *  \file cbuff.c
 *
 *  \brief The file implements the CBUFF Driver.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/cbuff.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/hw_include/hw_types.h>


CBUFF_Session      gCbuffSession[CBUFF_MAX_NUM_SESSION] = {{0u}};

/* This is the size of the Chirp Parameters (CP) in CBUFF Units */
#define CP_SIZE_CBUFF_UNITS                                 (2U)

/* Definition for field S_CHIRP_DONE in Register STAT_CBUFF_REG0 */
#define STAT_CBUFF_REG0_S_CHIRP_DONE_BIT_START              (11U)
#define STAT_CBUFF_REG0_S_CHIRP_DONE_BIT_END                (11U)

/* Definition for field S_FRAME_DONE in Register STAT_CBUFF_REG0 */
#define STAT_CBUFF_REG0_S_FRAME_DONE_BIT_START              (12U)
#define STAT_CBUFF_REG0_S_FRAME_DONE_BIT_END                (12U)

/* Definition for field S_CHIRP_ERR in Register STAT_CBUFF_REG1 */
#define STAT_CBUFF_REG1_S_CHIRP_ERR_BIT_START               (16U)
#define STAT_CBUFF_REG1_S_CHIRP_ERR_BIT_END                 (16U)

/* Definition for field S_FRAME_ERR in Register STAT_CBUFF_REG1 */
#define STAT_CBUFF_REG1_S_FRAME_ERR_BIT_START               (17U)
#define STAT_CBUFF_REG1_S_FRAME_ERR_BIT_END                 (17U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* CBUFF Register Read/Write API: */
static void CBUFF_reset          (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_releaseReset   (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_enableErrorInt (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_disableErrorInt(CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_disableFrameDoneInt (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_enableFrameDoneInt (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_disableChirpDoneInt (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_enableChirpDoneInt (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_setECC         (CSL_CbuffRegs* ptrCBUFFReg, uint8_t ecc);
static void CBUFF_getStatus      (CSL_CbuffRegs* ptrCBUFFReg, uint8_t* isFrameDone, uint8_t* isChirpDone);
static void CBUFF_clearStatus    (CSL_CbuffRegs* ptrCBUFFReg, uint8_t isFrameDone, uint8_t isChirpDone);
static void CBUFF_setFrameStartSWTrigger (CSL_CbuffRegs* ptrCBUFFReg, uint32_t enableSW);
static void CBUFF_setChirpAvailableSWTrigger (CSL_CbuffRegs* ptrCBUFFReg, uint32_t enableSW);
static void CBUFF_triggerFrameStart (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_triggerChirpAvailable (CSL_CbuffRegs* ptrCBUFFReg);
static void CBUFF_clearLLEntry (CSL_CbuffRegs* ptrCBUFFReg, uint8_t listIndex);
static void CBUFF_setLLEntry (CSL_CbuffRegs* ptrCBUFFReg, uint8_t listIndex, CBUFF_LinkListParams* ptrLLParams);
static void CBUFF_setNumberChirps (CSL_CbuffRegs* ptrCBUFFReg, uint32_t numChirps);

/* CBUFF Driver Internal API: */
static CBUFF_InterfaceFxn* CBUFF_isIfSupported (CBUFF_Interface interface);
static int32_t CBUFF_validateBufferCfg (CBUFF_BufferCfg* ptrUserBuffer, int32_t* errCode);

/* CBUFF Session Management API: */
static int32_t CBUFF_validateHWSessionCfg (CBUFF_Session* ptrSession, int32_t* errCode);
static int32_t CBUFF_validateSWSessionCfg (CBUFF_Session* ptrSession, int32_t* errCode);
static CBUFF_Session* CBUFF_allocateSession (CBUFF_Object* ptrDriverMCB);
static void CBUFF_freeSession (CBUFF_Session* ptrSession);

/* CBUFF Frame Done Management API: */
static void CBUFF_registerFrameDone (CBUFF_Object* ptrDriverMCB);
static void CBUFF_deregisterFrameDone (CBUFF_Object* ptrDriverMCB);

/**************************************************************************
 * CBUFF ISR:
 **************************************************************************/
static void CBUFF_ISR (void* arg);
static void CBUFF_ErrorISR (void* arg);

/* LVDS High Speed Interface Exported API: */
extern int32_t CBUFF_initLVDS(CBUFF_Object* ptrDriverMCB, int32_t* errCode);
extern int32_t CBUFF_deinitLVDS(CBUFF_Object* ptrDriverMCB, int32_t* errCode);
extern int32_t CBUFF_openLVDS(CBUFF_Session* ptrSession, int32_t* errCode);
extern int32_t CBUFF_closeLVDS(CBUFF_Session* ptrSession, int32_t* errCode);
extern void    CBUFF_initLVDSLinkListParams(CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLinkListParams);
extern void    CBUFF_finalizeLVDSLinkListParams(CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLinkListParams);


/* CBUFF Transfer Exported API: */
extern int32_t CBUFF_setupTransfer (CBUFF_Session* ptrSession, int32_t* errCode);
extern int32_t CBUFF_shutdownTransfer (CBUFF_Session* ptrSession, int32_t* errCode);

/*********************************************************************************
 * CBUFF Interleaved Transfer API for different data formats:
 *********************************************************************************/
extern int32_t CBUFF_setupInterleaved_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupInterleaved_CP_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupInterleaved_ADC_CP
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupInterleaved_CP_ADC_CQ
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupInterleaved_ADC_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupInterleaved_CP_ADC_CQ_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);

/*********************************************************************************
 * CBUFF Non-Interleaved Transfer API for different data formats:
 *********************************************************************************/
extern int32_t CBUFF_setupNonInterleaved_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_CP_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_ADC_CP
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_CP_ADC_CQ
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_ADC_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_CP_ADC_CQ_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);

/*********************************************************************************
 * CBUFF Non-Interleaved Multi Chirp Transfer API for different data formats:
 *********************************************************************************/
extern int32_t CBUFF_setupNonInterleaved_MultiChirp_CP_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_MultiChirp_ADC_CP
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_MultiChirp_CP_ADC_CQ
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);
extern int32_t CBUFF_setupNonInterleaved_MultiChirp_CP_ADC_CQ_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
);

/* CBUFF Transfer API for user: This is agnostic of interleaved or non-interleaved mode */
extern int32_t CBUFF_setup_USER (CBUFF_Object* ptrDriverMCB, CBUFF_Session* ptrSession, int32_t* errCode);


/**
 * @brief   This is the CBUFF supported high speed interfaces for AWR294X
 */
CBUFF_InterfaceFxn gCBUFFInterfaceFxn[] =
{
    {
        CBUFF_Interface_LVDS,               /* LVDS Interface                       */
        &CBUFF_initLVDS,                    /* Initialize the LVDS Interface        */
        &CBUFF_deinitLVDS,                  /* Deinitialize the LVDS Interface      */
        &CBUFF_openLVDS,                    /* Open the LVDS Interface              */
        &CBUFF_closeLVDS,                   /* Close the LVDS Interface             */
        &CBUFF_initLVDSLinkListParams,      /* LVDS Init Linked List Parameters     */
        &CBUFF_finalizeLVDSLinkListParams   /* LVDS Finalize Linked List Parameters */
    },
    {
        CBUFF_Interface_MDO,                /* MDO/Aurora Interface                 */
        NULL,                               /* NULL                                 */
        NULL,                               /* NULL                                 */
        NULL,                               /* NULL                                 */
		NULL                                /* NULL                                 */
    }
};

/**
 * @brief   On the AWR2944 Interleaved mode is not supported
 */
CBUFF_setupTransferFxn gInterleavedTransferFxn[] =
{
    CBUFF_setupInterleaved_ADC,                        /* Data Format: CBUFF_DataFmt_ADC_DATA          */
    CBUFF_setupInterleaved_CP_ADC,                     /* Data Format: CBUFF_DataFmt_CP_ADC            */
    CBUFF_setupInterleaved_ADC_CP,                     /* Data Format: CBUFF_DataFmt_ADC_CP            */
    CBUFF_setupInterleaved_CP_ADC_CQ,                  /* Data Format: CBUFF_DataFmt_CP_ADC_CQ         */
    CBUFF_setupInterleaved_ADC_USER,                   /* Data Format: CBUFF_DataFmt_ADC_USER          */
    CBUFF_setupInterleaved_CP_ADC_CQ_USER              /* Data Format: CBUFF_DataFmt_CP_ADC_CQ_USER    */
};

/**
 * @brief   This is the function table for non-interleaved transfer functions.
 * The default driver library supports some data formats. The full
 * set is provided as reference under the \#ifdef ENABLE_ALL..
 * Modify this table to enable the formats of choice and rebuild the driver.
 */
CBUFF_setupTransferFxn gNonInterleavedTransferFxn[] =
{
    CBUFF_setupNonInterleaved_ADC,                      /* Data Format: CBUFF_DataFmt_ADC_DATA          */
    CBUFF_setupNonInterleaved_CP_ADC,                   /* Data Format: CBUFF_DataFmt_CP_ADC            */
    CBUFF_setupNonInterleaved_ADC_CP,                   /* Data Format: CBUFF_DataFmt_ADC_CP            */
    CBUFF_setupNonInterleaved_CP_ADC_CQ,                /* Data Format: CBUFF_DataFmt_CP_ADC_CQ         */
    CBUFF_setupNonInterleaved_ADC_USER,                 /* Data Format: CBUFF_DataFmt_ADC_USER          */
    CBUFF_setupNonInterleaved_CP_ADC_CQ_USER            /* Data Format: CBUFF_DataFmt_CP_ADC_CQ_USER    */
};

/**
 * @brief   This is the function table for multi-chirp non-interleaved transfer functions.
 * The default driver library does not support any data format. The full
 * set is provided as reference under the \#ifdef ENABLE_ALL..
 * Modify this table to enable the formats of choice and rebuild the driver.
 * *NOTE*: There are some transfer functions which are the same between single and multiple
 * chirp.
 */
CBUFF_setupTransferFxn gMultipleChirpNonInterleavedTransferFxn[] =
{
#ifdef ENABLE_ALL_MULTI_CHIRP_NON_INTERLEAVED
    CBUFF_setupNonInterleaved_ADC,                      /* Data Format: CBUFF_DataFmt_ADC_DATA          */
    CBUFF_setupNonInterleaved_MultiChirp_CP_ADC,        /* Data Format: CBUFF_DataFmt_CP_ADC            */
    CBUFF_setupNonInterleaved_MultiChirp_ADC_CP,        /* Data Format: CBUFF_DataFmt_ADC_CP            */
    CBUFF_setupNonInterleaved_MultiChirp_CP_ADC_CQ,     /* Data Format: CBUFF_DataFmt_CP_ADC_CQ         */
    CBUFF_setupNonInterleaved_ADC_USER,                 /* Data Format: CBUFF_DataFmt_ADC_USER          */
    CBUFF_setupNonInterleaved_MultiChirp_CP_ADC_CQ_USER /* Data Format: CBUFF_DataFmt_CP_ADC_CQ_USER    */
#else
    NULL,                                               /* Data Format: CBUFF_DataFmt_ADC_DATA          */
    NULL,                                               /* Data Format: CBUFF_DataFmt_CP_ADC            */
    NULL,                                               /* Data Format: CBUFF_DataFmt_ADC_CP            */
    NULL,                                               /* Data Format: CBUFF_DataFmt_CP_ADC_CQ         */
    NULL,                                               /* Data Format: CBUFF_DataFmt_ADC_USER          */
    NULL                                                /* Data Format: CBUFF_DataFmt_CP_ADC_CQ_USER    */
#endif
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @b Description
 *  @n
 *      The function is used to reset the CBUFF module.
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_reset (CSL_CbuffRegs* ptrCBUFFReg)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 27U, 27U, 1U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to release the reset of the CBUFF module.
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_releaseReset (CSL_CbuffRegs* ptrCBUFFReg)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 27U, 27U, 0U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the ECC
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  ecc
 *      Flag to enable/disable ECC
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_setECC (CSL_CbuffRegs* ptrCBUFFReg, uint8_t ecc)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 1U, 1U, (uint32_t)ecc);
}

/**
 *  @b Description
 *  @n
 *      The function is used to disable the frame done interrupts
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_disableFrameDoneInt (CSL_CbuffRegs* ptrCBUFFReg)
{
    /* Disable Frame Done:
     *  - Interrupts are disabled by writing a 1 to the corresponding bits */
    ptrCBUFFReg->CFG_MASK_REG0 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG0,
                                            STAT_CBUFF_REG0_S_FRAME_DONE_BIT_START,
                                            STAT_CBUFF_REG0_S_FRAME_DONE_BIT_END,
                                            1U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the chirp done interrupt
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_enableFrameDoneInt (CSL_CbuffRegs* ptrCBUFFReg)
{
    /* Enable Frame Done:
     *  - Interrupts are enabled by writing a 0 to the corresponding bits */
    ptrCBUFFReg->CFG_MASK_REG0 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG0,
                                            STAT_CBUFF_REG0_S_FRAME_DONE_BIT_START,
                                            STAT_CBUFF_REG0_S_FRAME_DONE_BIT_END,
                                            0U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to disable the chirp done interrupts
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_disableChirpDoneInt (CSL_CbuffRegs* ptrCBUFFReg)
{
    /* Disable Chirp Done:
     *  - Interrupts are disabled by writing a 1 to the corresponding bits */
    ptrCBUFFReg->CFG_MASK_REG0 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG0,
                                            STAT_CBUFF_REG0_S_CHIRP_DONE_BIT_START,
                                            STAT_CBUFF_REG0_S_CHIRP_DONE_BIT_END,
                                            1U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the chirp done interrupt
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_enableChirpDoneInt (CSL_CbuffRegs* ptrCBUFFReg)
{
    /* Enable Chirp Done:
     *  - Interrupts are enabled by writing a 0 to the corresponding bits */
    ptrCBUFFReg->CFG_MASK_REG0 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG0,
                                            STAT_CBUFF_REG0_S_CHIRP_DONE_BIT_START,
                                            STAT_CBUFF_REG0_S_CHIRP_DONE_BIT_END,
                                            0U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to enable the error interrupts associated with the
 *      CBUFF Module
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_enableErrorInt (CSL_CbuffRegs* ptrCBUFFReg)
{
    /* Enable Frame Start Error & Chirp Error:
     *  - Interrupts are enabled by writing a 0 to the corresponding bits */
    ptrCBUFFReg->CFG_MASK_REG1 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG1,
                                            STAT_CBUFF_REG1_S_FRAME_ERR_BIT_START,
                                            STAT_CBUFF_REG1_S_FRAME_ERR_BIT_END,
                                            0U);
    ptrCBUFFReg->CFG_MASK_REG1 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG1,
                                            STAT_CBUFF_REG1_S_CHIRP_ERR_BIT_START,
                                            STAT_CBUFF_REG1_S_CHIRP_ERR_BIT_END,
                                            0U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to disable the error interrupts associated with the
 *      CBUFF Module
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_disableErrorInt(CSL_CbuffRegs* ptrCBUFFReg)
{
    /* Enable Frame Start Error & Chirp Error:
     *  - Interrupts are disabled by writing a 1 to the corresponding bits */
    ptrCBUFFReg->CFG_MASK_REG1 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG1,
                                            STAT_CBUFF_REG1_S_FRAME_ERR_BIT_START,
                                            STAT_CBUFF_REG1_S_FRAME_ERR_BIT_END,
                                            1U);
    ptrCBUFFReg->CFG_MASK_REG1 = CSL_FINSR (ptrCBUFFReg->CFG_MASK_REG1,
                                            STAT_CBUFF_REG1_S_CHIRP_ERR_BIT_START,
                                            STAT_CBUFF_REG1_S_CHIRP_ERR_BIT_END,
                                            1U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the CBUFF status.
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[out]  isFrameDone
 *      Set to 1 to indicate that the frame has been done
 *  @param[out]  isChirpDone
 *      Set to 1 to indicate that the chirp has been done
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_getStatus (CSL_CbuffRegs* ptrCBUFFReg, uint8_t* isFrameDone, uint8_t* isChirpDone)
{
    *isFrameDone = (uint8_t)CSL_FEXTR (ptrCBUFFReg->STAT_CBUFF_REG0, 12U, 12U);
    *isChirpDone = (uint8_t)CSL_FEXTR (ptrCBUFFReg->STAT_CBUFF_REG0, 11U, 11U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to clear the CBUFF status.
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  isFrameDone
 *      Set to 1 to clear the Frame Done Status
 *  @param[out]  isChirpDone
 *      Set to 1 to clear the Chirp Done Status
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_clearStatus (CSL_CbuffRegs* ptrCBUFFReg, uint8_t isFrameDone, uint8_t isChirpDone)
{
    ptrCBUFFReg->CLR_CBUFF_REG0 = CSL_FINSR (ptrCBUFFReg->CLR_CBUFF_REG0, 12U, 12U, (uint32_t)isFrameDone);
    ptrCBUFFReg->CLR_CBUFF_REG0 = CSL_FINSR (ptrCBUFFReg->CLR_CBUFF_REG0, 11U, 11U, (uint32_t)isChirpDone);
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the software triggerring capability of the frame start
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  enableSW
 *      Set to 1 to enable the triggering be done by the SW else set to 0
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_setFrameStartSWTrigger (CSL_CbuffRegs* ptrCBUFFReg, uint32_t enableSW)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 2U, 2U, enableSW);
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the software triggerring capability of the chirp available
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  enableSW
 *      Set to 1 to enable the triggering be done by the SW else set to 0
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_setChirpAvailableSWTrigger (CSL_CbuffRegs* ptrCBUFFReg, uint32_t enableSW)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 3U, 3U, enableSW);
}

/**
 *  @b Description
 *  @n
 *      The function is used to trigger the frame start. This function should only be
 *      invoked if the software has assumed the responsibility for triggering the frame
 *      start
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_triggerFrameStart (CSL_CbuffRegs* ptrCBUFFReg)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 25U, 25U, 1U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to trigger the chirp available. This function should only be
 *      invoked if the software has assumed the responsibility for triggering the chirp
 *      available
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_triggerChirpAvailable (CSL_CbuffRegs* ptrCBUFFReg)
{
    ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 24U, 24U, 1U);
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the number of chirps
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  numChirps
 *      Number of chirps to be configured
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_setNumberChirps (CSL_CbuffRegs* ptrCBUFFReg, uint32_t numChirps)
{
    ptrCBUFFReg->CFG_CHIRPS_PER_FRAME = numChirps;
}

/**
 *  @b Description
 *  @n
 *      The function is used to clear the linked list entry in the CBUFF IP
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  listIndex
 *      Linked List index to be cleared.
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_clearLLEntry (CSL_CbuffRegs* ptrCBUFFReg, uint8_t listIndex)
{
    /* Reset the LL to the default values. */
    ptrCBUFFReg->LL_ENTRY[listIndex].CFG_DATA_LL           = 0U;
    ptrCBUFFReg->LL_ENTRY[listIndex].CFG_DATA_LL_LPHDR_VAL = 0U;
    ptrCBUFFReg->LL_ENTRY[listIndex].CFG_DATA_LL_THRESHOLD = CSL_FMKR (14U, 8U, 0x3FU);
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the linked list entry in the CBUFF IP
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  listIndex
 *      Linked List index to be configured.
 *  @param[in]  ptrLLParams
 *      Pointer to the Linked list params used to set the LL entry
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_setLLEntry (CSL_CbuffRegs* ptrCBUFFReg, uint8_t listIndex, CBUFF_LinkListParams* ptrLLParams)
{
    /* Sanity Check: This should never occur and the transfer size should always be within the IP Limits */
    DebugP_assert (ptrLLParams->transferSize <= CBUFF_MAX_TRANSFER_SIZE_CBUFF_UNITS);

    /* Setup the linked list data:
     *  - CRC Enable as per the specified configuration
     *  - LP Header as per the specified configuration
     *  - Wait for Packet sent is enabled
     *  - Size as per the specified configuration
     *  - Data Formating of the incoming data sources [128bit(0) or 96bit (1)]
     *  - Data Format map applicable only for LVDS.
     *  - LVDS/CSI2 Output format i.e. 16bit, 14bit or 12bit
     *  - Number of the virtual channel
     *  - Horizontal Sync Start as per the specified configuration
     *  - Horizontal Sync End as per the specified configuration
     *  - Linked List entry is valid */
    ptrCBUFFReg->LL_ENTRY[listIndex].CFG_DATA_LL =
             CSL_FMKR (28U, 28U, ptrLLParams->crcEnable)        |
             CSL_FMKR (27U, 27U, ptrLLParams->lpHdrEn)          |
             CSL_FMKR (22U, 9U,  ptrLLParams->transferSize)     |
             CSL_FMKR (8U,  8U,  ptrLLParams->align96)          |
             CSL_FMKR (7U,  7U,  ptrLLParams->dataFmtMap)       |
             CSL_FMKR (6U,  5U,  ptrLLParams->outputDataFmt)    |
             CSL_FMKR (4U,  3U,  ptrLLParams->vcNum)            |
             CSL_FMKR (2U,  2U,  ptrLLParams->hsyncStart)       |
             CSL_FMKR (1U,  1U,  ptrLLParams->hsyncEnd)         |
             CSL_FMKR (0U,  0U,  1U);

    /* Set the Long Packet Header & Thresholds: */
    ptrCBUFFReg->LL_ENTRY[listIndex].CFG_DATA_LL_LPHDR_VAL = ptrLLParams->lpHeaderValue;
    ptrCBUFFReg->LL_ENTRY[listIndex].CFG_DATA_LL_THRESHOLD = ptrLLParams->threshold;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the VSYNC parameters for a specific
 *      virtual channel.
 *
 *  @param[in]  ptrCBUFFReg
 *      Pointer to the CBUFF Register space
 *  @param[in]  vc
 *      Virtual channel [Valid from 0 to 3]
 *  @param[in]  vcStart
 *      Flag to enable/disable the VSYNC Start
 *  @param[in]  vcEnd
 *      Flag to enable/disable the VSYNC End
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CBUFF_setVC (CSL_CbuffRegs* ptrCBUFFReg, uint8_t vc, uint8_t vcStart, uint8_t vcEnd)
{
    switch (vc)
    {
        case 0U:
        {
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 10U, 10U, (uint32_t)vcStart);
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 11U, 11U, (uint32_t)vcEnd);
            break;
        }
        case 1U:
        {
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 12U, 12U, (uint32_t)vcStart);
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 13U, 13U, (uint32_t)vcEnd);
            break;
        }
        case 2U:
        {
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 14U, 14U, (uint32_t)vcStart);
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 15U, 15U, (uint32_t)vcEnd);
            break;
        }
        case 3U:
        {
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 16U, 16U, (uint32_t)vcStart);
            ptrCBUFFReg->CONFIG_REG_0 = CSL_FINSR (ptrCBUFFReg->CONFIG_REG_0, 17U, 17U, (uint32_t)vcEnd);
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This is the function which is used to validate the buffers
 *      configuration. User buffers passed by the applications for
 *      either the header *OR* data buffers need to be placed into
 *      the EDMA address space.
 *
 *  @param[in]  ptrUserBuffer
 *      Pointer to the user buffer which needs to be validated
 *  @param[out] errCode
 *      Error code populated on the error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CBUFF_validateBufferCfg (CBUFF_BufferCfg* ptrUserBuffer, int32_t* errCode)
{
    int32_t     retVal = -1;

    /* Sanity Check: Ensure that the size does not exceed the MAXIMUM CBUFF IP Size */
    if (ptrUserBuffer->size > CBUFF_MAX_TRANSFER_SIZE_CBUFF_UNITS)
    {
        /* Error: Invalid size exceeds the max which the IP block can support */
        *errCode = CBUFF_ELIMIT;
        return retVal;
    }

    /* Is the address specified? */
    if (ptrUserBuffer->size != 0U)
    {
        /* YES: Translate to the EDMA space */
        /* Update the configuration with the EDMA Address: This is what we will be using to
         * setup the EDMA Transfers */
        ptrUserBuffer->address = (uint32_t) AddrTranslateP_getLocalAddr (ptrUserBuffer->address);
    }

    /* User buffers have been validated: */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered ISR for the CBUFF to handle the CBUFF
 *      completion status
 *
 *  @param[in]  arg
 *      Pointer to the CBUFF Driver MCB
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_ISR (void* arg)
{
    CBUFF_Object*           ptrDriverMCB;
    uint8_t                 isFrameDone;
    uint8_t                 isChirpDone;
    CBUFF_Session*          ptrSession;

    /* Get the pointer to the driver MCB: */
    ptrDriverMCB = (CBUFF_Object*)arg;

    /* Sanity Check: */
    DebugP_assert (ptrDriverMCB != NULL);

    /* Get the CBUFF Status: */
    CBUFF_getStatus (ptrDriverMCB->ptrCBUFFReg, &isFrameDone, &isChirpDone);

    /* Clear the CBUFF Status: */
    CBUFF_clearStatus (ptrDriverMCB->ptrCBUFFReg, isFrameDone, isChirpDone);

    /* Get the currently active session: */
    ptrSession = ptrDriverMCB->ptrCurrentActiveSession;
    if (ptrSession == NULL)
    {
        /* Record the number of the interrupts. */
        ptrDriverMCB->totalNumNonActiveSessionInterrupts++;
        return;
    }

    /* Was the frame processed? */
    if (isFrameDone == 1U)
    {
        /* YES: The frame was processed. Increment the statistics */
        ptrDriverMCB->totalNumFrameDone++;

        /* Increment the statistics for the session */
        ptrSession->numFrameDone++;

        /* Do we need to invoke the callback function? */
        if (ptrSession->sessionCfg.frameDoneCallbackFxn != NULL)
        {
            /* YES: Notify the callback function that the frame done was received for the
             * active session. */
            ptrSession->sessionCfg.frameDoneCallbackFxn ((CBUFF_SessionHandle)ptrSession);
        }
    }

    /* Are we operating in debug mode? */
    if (ptrDriverMCB->initCfg.enableDebugMode == true)
    {
        /* Was the chirp processed? */
        if (isChirpDone == 1U)
        {
            /* YES: The chirp was handled. Increment the statistics */
            ptrDriverMCB->totalNumChirpDone++;

            /* Increment the statistics for the session */
            ptrSession->numChirpDone++;
        }
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is the registered ISR for the CBUFF to handle the
 *      CBUFF errors
 *
 *  @param[in]  arg
 *      Pointer to the CBUFF Driver MCB
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_ErrorISR (void* arg)
{
    CBUFF_Object*           ptrDriverMCB;
    uint8_t                 isFrameError = 0;
    uint8_t                 isChirpError = 0;

    /* Get the pointer to the driver MCB: */
    ptrDriverMCB = (CBUFF_Object*)arg;

    /* Sanity Check: */
    DebugP_assert (ptrDriverMCB != NULL);

    /* Get the CBUFF Error Status: */
    isFrameError = (uint8_t)HW_GET_FIELD(ptrDriverMCB->ptrCBUFFReg->STAT_CBUFF_REG1, CSL_CBUFF_STAT_CBUFF_REG1_S_FRAME_ERR);
    isChirpError = (uint8_t)HW_GET_FIELD(ptrDriverMCB->ptrCBUFFReg->STAT_CBUFF_REG1, CSL_CBUFF_STAT_CBUFF_REG1_S_CHIRP_ERR);

    /* Clear the CBUFF Error Status: */
    HW_SET_FIELD32(ptrDriverMCB->ptrCBUFFReg->CLR_CBUFF_REG1, CSL_CBUFF_STAT_CBUFF_REG1_S_FRAME_ERR, (uint32_t)isFrameError);
    HW_SET_FIELD32(ptrDriverMCB->ptrCBUFFReg->CLR_CBUFF_REG1, CSL_CBUFF_STAT_CBUFF_REG1_S_CHIRP_ERR, (uint32_t)isChirpError);

    /* Increment the error counter */
    ptrDriverMCB->totalNumErrorInterrupts++;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the high speed interface with the platform
 *      supplied interface table.
 *
 *  @param[in]  interface
 *      Interface to be verified
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - Supported interface function block
 *  @retval
 *      Error       - NULL
 */
static CBUFF_InterfaceFxn* CBUFF_isIfSupported (CBUFF_Interface interface)
{
    uint8_t             ifIndex = 0U;
    CBUFF_InterfaceFxn* ptrInterfaceFxn;
    int32_t             done = 0;

    /* Initialize the interface function to the platform supplied definition */
    while (done == 0)
    {
        /* Get the interface block: */
        ptrInterfaceFxn = &gCBUFFInterfaceFxn[ifIndex];

        /* Is this a valid interface or the end of the table? */
        if (ptrInterfaceFxn->initFxn != NULL)
        {
            /* YES: Do we have a match on the interface? */
            if (interface == ptrInterfaceFxn->interface)
            {
                /* YES: We have found a match. */
                done = 1;
            }
            else
            {
                /* NO: Skip to the next entry in the platform table */
                ifIndex++;
            }
        }
        else
        {
            /* NO: We have reached the end of the platform table and have not found a match. */
            ptrInterfaceFxn = NULL;
            done = 1;
        }
    }
    return ptrInterfaceFxn;
}

/**
 *  @b Description
 *  @n
 *      The function is used to free an allocated session
 *
 *  @param[in]  ptrSession
 *      Pointer to the session to be freed up
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_freeSession (CBUFF_Session* ptrSession)
{
    CBUFF_Object*       ptrDriverMCB;
    uint8_t             dmaNum;

    /* Keep a local copy of the session information */
    ptrDriverMCB = ptrSession->ptrDriverMCB;
    dmaNum       = ptrSession->dmaNum;

    /* Reset the contents of the session memory: */
    memset ((void*)ptrSession, 0, sizeof(CBUFF_Session));

    /* Update session back: */
    ptrSession->ptrDriverMCB = ptrDriverMCB;
    ptrSession->dmaNum       = dmaNum;
}

/**
 *  @b Description
 *  @n
 *      The function is used to allocate a session block
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the driver MCB
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - Allocated session handle
 *  @retval
 *      Error   - NULL
 */
static CBUFF_Session* CBUFF_allocateSession (CBUFF_Object* ptrDriverMCB)
{
    uint8_t         sessionIndex;
    CBUFF_Session*  ptrSession;
    uintptr_t       key;

    /* Disable the interrupts: The session table is a critical resource which needs
     * to be protected against concurrent access from multiple threads. */
    key = HwiP_disable();

    /* Cycle through all the sessions: */
    for (sessionIndex = 0U; sessionIndex < ptrDriverMCB->initCfg.maxSessions; sessionIndex++)
    {
        /* Get the session: */
        ptrSession = &ptrDriverMCB->ptrSessionTable[sessionIndex];

        /* Is the session valid and in use? */
        if (ptrSession->isValid == false)
        {
            /* NO: Session is free and can be used. */
            ptrSession->isValid = true;
            break;
        }
    }

    /* Were we able to allocate a session? */
    if (sessionIndex == ptrDriverMCB->initCfg.maxSessions)
    {
        /* NO: Reached the end of the table and unable to find a free slot */
        ptrSession = NULL;
    }

    /* Restore the interrupts: Session allocation has been completed. */
    HwiP_restore (key);

    /* Return the session: */
    return ptrSession;
}

/**
 *  @b Description
 *  @n
 *      The function is used to register the frame done interrupts.
 *      If applications are executing only one session we dont want
 *      to overwhelm the processor with frame done interrupts. Frame
 *      done interrupts should only be used if there are multiple
 *      sessions which are getting activated and the application has
 *      registered for callback functions.
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the driver MCB
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_registerFrameDone (CBUFF_Object* ptrDriverMCB)
{
    uint8_t         sessionIndex;
    CBUFF_Session*  ptrSession;

    /* Is the debug mode enabled? */
    if (ptrDriverMCB->initCfg.enableDebugMode == true)
    {
        /* YES: Frame done interrupts are already registered. There is no need
         * to perform any other actions. */
        return;
    }

    /* Cycle through all the sessions: */
    for (sessionIndex = 0U; sessionIndex < ptrDriverMCB->initCfg.maxSessions; sessionIndex++)
    {
        /* Get the session: */
        ptrSession = &ptrDriverMCB->ptrSessionTable[sessionIndex];

        /* Is the session valid and does it need the frame done callback? */
        if ((ptrSession->isValid == true) && (ptrSession->sessionCfg.frameDoneCallbackFxn != NULL))
        {
            /* YES: Frame Done interrupt notifications are required. */
            CBUFF_enableFrameDoneInt (ptrDriverMCB->ptrCBUFFReg);
            break;
        }
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deregister the frame done interrupts.
 *      If there are no active sessions which require this functionality
 *      then we can deregister this interrupt and not take a performance
 *      hit for these interrupts.
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the driver MCB
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_deregisterFrameDone (CBUFF_Object* ptrDriverMCB)
{
    uint8_t         sessionIndex;
    CBUFF_Session*  ptrSession;
    uint8_t         isFrameDone;
    uint8_t         isChirpDone;

    /* Is the debug mode enabled? */
    if (ptrDriverMCB->initCfg.enableDebugMode == true)
    {
        /* YES: Executing in debug mode. There is no need to perform any deregisteration
         * We need to keep the interrupts always registered. */
        return;
    }

    /* Cycle through all the sessions: */
    for (sessionIndex = 0U; sessionIndex < ptrDriverMCB->initCfg.maxSessions; sessionIndex++)
    {
        /* Get the session: */
        ptrSession = &ptrDriverMCB->ptrSessionTable[sessionIndex];

        /* Is the session valid and does it need the frame done callback? */
        if ((ptrSession->isValid == true) && (ptrSession->sessionCfg.frameDoneCallbackFxn != NULL))
        {
            break;
        }
    }
    if (sessionIndex == ptrDriverMCB->initCfg.maxSessions)
    {
        /* Control comes here indicates that there were no either:-
         * (a) No more active sessions  *OR*
         * (b) None of the sessions needed the frame done functionality
         *
         * So we can deregister the ISR and avoid the performance penalty. */
        CBUFF_disableFrameDoneInt (ptrDriverMCB->ptrCBUFFReg);

        /* We have disabled the interrupts but if there are any pending interrupts
         * then these need to be acknowledged here; else we will not get any more
         * interrupts. */
        CBUFF_getStatus (ptrDriverMCB->ptrCBUFFReg, &isFrameDone, &isChirpDone);
        CBUFF_clearStatus (ptrDriverMCB->ptrCBUFFReg, isFrameDone, isChirpDone);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the HW Session configuration.
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t CBUFF_validateHWSessionCfg (CBUFF_Session* ptrSession, int32_t* errCode)
{
    uint32_t            index;
    ADCBufMMWave_CQType cqType;
    int32_t             retVal = SystemP_FAILURE;

    /************************************************************************
     * Sanity Check: Continuous mode is *NOT* supported for all data formats
     ************************************************************************/
    if (ptrSession->sessionCfg.u.hwCfg.opMode == CBUFF_OperationalMode_CONTINUOUS)
    {
        /* Sanity Check: Curently only the ADC Data mode is supported with continuous mode */
        if (ptrSession->sessionCfg.u.hwCfg.dataFormat != CBUFF_DataFmt_ADC_DATA)
        {
            *errCode = CBUFF_ENOTSUP;
            DebugP_assert(0);
        }

        /* Sanity Check: Ensure that the chirp mode is correctly configured */
        if (ptrSession->sessionCfg.u.hwCfg.chirpMode != 0U)
        {
            *errCode = CBUFF_EINVAL;
            DebugP_assert(0);
        }
    }
    else
    {
        /* Sanity Check: Validate the range. (Single or Multiple Chirp) */
        if ((ptrSession->sessionCfg.u.hwCfg.chirpMode < gCbuffObjectPtr[0]->hwAttrs->chirpModeStartIndex) ||
            (ptrSession->sessionCfg.u.hwCfg.chirpMode > gCbuffObjectPtr[0]->hwAttrs->chirpModeEndIndex))
        {
            /* Error: The chirp mode is not within the supported range */
            *errCode = CBUFF_EINVAL;
            DebugP_assert(0);
        }
    }

    /************************************************************************
     * Sanity Check: Validate the user buffers
     ************************************************************************/
    if ((ptrSession->sessionCfg.u.hwCfg.dataFormat == CBUFF_DataFmt_ADC_USER) ||
        (ptrSession->sessionCfg.u.hwCfg.dataFormat == CBUFF_DataFmt_CP_ADC_CQ_USER))
    {
        /* User data will be streamed out: */
        for (index = 0U; index < CBUFF_MAX_USER_BUFFER; index++)
        {
            /* Validate the user buffers to make sure these can be streamed out via the CBUFF */
            if (CBUFF_validateBufferCfg (&ptrSession->sessionCfg.u.hwCfg.userBufferInfo[index], errCode) < 0)
            {
                /* Error: Invalid user buffer configuration error code is already setup */
                DebugP_assert(0);
            }
        }
    }

    /* Cycle through the configuration and process it */
    ptrSession->numActiveADCChannels = 0U;
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Get the ADCBUF Channel Address from the ADCBUFFF registers */
    	ptrSession->rxChannelAddress[index] = ADCBuf_getChanBufAddr (ptrSession->sessionCfg.u.hwCfg.adcBufHandle, index, errCode);

        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /* YES: Increment the number of active ADC channels */
            ptrSession->numActiveADCChannels++;

            /* Translate the address to EDMA:  */
            ptrSession->rxChannelAddress[index] = (uint32_t) AddrTranslateP_getLocalAddr(ptrSession->rxChannelAddress[index]);

            /* Track the cumulative Chirp Parameter Transfer Size */
            ptrSession->cpTotalTransferSize = ptrSession->cpTotalTransferSize + CP_SIZE_CBUFF_UNITS;
        }
    }

    /************************************************************************
     * Sanity Check: ADC channels should be configured and be enabled
     ************************************************************************/
    if (ptrSession->numActiveADCChannels == 0U)
    {
        /* Error: No ADC channel has been enabled. We cannot calculate the ADC channel address
         * and setup the EDMA chains before the ADC is configured. */
        *errCode = CBUFF_EINVAL;
        DebugP_assert(0);
    }

    /************************************************************************
     * Sanity Check: Chirp Quality
     ************************************************************************/
    if ((ptrSession->sessionCfg.u.hwCfg.dataFormat == CBUFF_DataFmt_CP_ADC_CQ) ||
        (ptrSession->sessionCfg.u.hwCfg.dataFormat == CBUFF_DataFmt_CP_ADC_CQ_USER))
    {
        /* Initialize the CQ */
        ptrSession->numCQ = 0U;

        /* Cycle through all the Chirp Quality: */
        for (cqType = ADCBufMMWave_CQType_CQ0; cqType < ADCBufMMWave_CQType_MAX_CQ; cqType++)
        {
            /* Do we have a valid CQ to be streamed out? */
            if (ptrSession->sessionCfg.u.hwCfg.cqSize[cqType] != 0U)
            {
                /* YES: Get the CQ Base address from the ADC Driver */
            	ptrSession->cqBuffer[ptrSession->numCQ].address = ADCBUF_MMWave_getCQBufAddr(ptrSession->sessionCfg.u.hwCfg.adcBufHandle,
                                                                                             cqType, errCode);

                if (ptrSession->cqBuffer[ptrSession->numCQ].address == 0U)
                {
                    /* Error: Unable to get the base address; error code is already setup */
                    DebugP_assert(0);
                }

                /* Translate the address to EDMA: */
                ptrSession->cqBuffer[ptrSession->numCQ].address = (uint32_t) AddrTranslateP_getLocalAddr(ptrSession->cqBuffer[ptrSession->numCQ].address);

                /* Store the CQn Size: */
                ptrSession->cqBuffer[ptrSession->numCQ].size = ptrSession->sessionCfg.u.hwCfg.cqSize[cqType];

                /* Increment the detected number of CQ: */
                ptrSession->numCQ++;
            }
        }

        /* Sanity Check: Do we have at least 1 CQ to be streamed out. */
        if (ptrSession->numCQ == 0U)
        {
            /* Error: There is no CQ. */
            *errCode = CBUFF_EINVAL;
            DebugP_assert(0);
        }
    }

    /* HW Session configuration has been validated */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to validate the SW Session configuration.
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t CBUFF_validateSWSessionCfg (CBUFF_Session* ptrSession, int32_t* errCode)
{
    uint32_t    index;
    int32_t     retVal = SystemP_FAILURE;

    /************************************************************************
     * Sanity Check: Validate the user buffers
     ************************************************************************/
    for (index = 0U; index < CBUFF_MAX_USER_BUFFER; index++)
    {
        /* Validate the user buffers to make sure these can be streamed out via the CBUFF */
        if (CBUFF_validateBufferCfg (&ptrSession->sessionCfg.u.swCfg.userBufferInfo[index], errCode) < 0)
        {
            /* Error: Invalid user buffer configuration error code is already setup */
            DebugP_assert(0);
        }
    }

    /* SW Session configuration has been validated */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deactivate the session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session to be activated
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CBUFF_deactivateSession (CBUFF_SessionHandle sessionHandle, int32_t* errCode)
{
    int32_t             retVal = SystemP_FAILURE;
    CBUFF_Object*       ptrDriverMCB;
    CBUFF_Session*      ptrSession;
    uint8_t             index;

    /* Get the pointer to the session: */
    ptrSession = (CBUFF_Session*)sessionHandle;
    DebugP_assert (ptrSession != NULL);

    /* Sanity Check: Ensure that there exists a back pointer to the CBUFF Driver */
    ptrDriverMCB = ptrSession->ptrDriverMCB;
    DebugP_assert (ptrDriverMCB != NULL);

    if ((ptrSession == NULL) || (ptrDriverMCB->ptrCurrentActiveSession != ptrSession))
    {
        *errCode = CBUFF_EINVAL;
        DebugP_assert(0);
    }
    else
    {
        /* Clear out all the linked list entries in the CBUFF IP */
        for (index = 0U; index < CBUFF_MAX_LINKED_LIST_SIZE; index++)
        {
            CBUFF_clearLLEntry (ptrDriverMCB->ptrCBUFFReg, index);
        }

        /* Default State: The triggers are always owned by the software. Frame Starts from
        * the BSS can cause the CBUFF IP to falsely kickstart the state machine especially
        * if there is no active session */
        CBUFF_setFrameStartSWTrigger (ptrDriverMCB->ptrCBUFFReg, 1U);
        CBUFF_setChirpAvailableSWTrigger (ptrDriverMCB->ptrCBUFFReg, 1U);

        /* Close the high speed interface (HSI) for the session: */
        DebugP_assert (ptrDriverMCB->interfaceFxn->closeFxn (ptrSession, errCode) == 0);

        /* The session is no longer active: */
        ptrDriverMCB->ptrCurrentActiveSession = NULL;

        /* Session has been deactivated successfully */
        retVal = 0;
    }

    return retVal;
}

int32_t CBUFF_activateSession (CBUFF_SessionHandle sessionHandle, int32_t* errCode)
{
    int32_t             retVal = SystemP_FAILURE;
    CBUFF_Object*       ptrDriverMCB;
    CBUFF_Session*      ptrSession;
    uint8_t             index;

    /* Get the pointer to the session: */
    ptrSession = (CBUFF_Session*)sessionHandle;
    if (ptrSession == NULL)
    {
        *errCode = CBUFF_EINVAL;
        DebugP_assert(0);
    }
    else
    {
        /* Sanity Check: Ensure that there exists a back pointer to the CBUFF Driver */
        ptrDriverMCB = ptrSession->ptrDriverMCB;
        DebugP_assert (ptrDriverMCB != NULL);

        /* Is there a current session which is already active? */
        if (ptrDriverMCB->ptrCurrentActiveSession != NULL)
        {
            /* YES: Let the application deactivate it explicitly. */
            *errCode = CBUFF_EINUSE;
            DebugP_assert(0);
        }

        /* The Linked List entries are already configured in the session. */
        for (index = 0U; index < ptrSession->linkListIndex; index++)
        {
            /* Configure the CBUFF IP Linked List Entry. */
            CBUFF_setLLEntry (ptrDriverMCB->ptrCBUFFReg, index, &ptrSession->linkedListTracker[index]);
        }

        /* Process the session configuration on the basis of the execution mode: */
        if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
        {
            /* Hardware Triggered Execution Mode: Are we operating in chirp *or* continuous mode? */
            if (ptrSession->sessionCfg.u.hwCfg.opMode == CBUFF_OperationalMode_CONTINUOUS)
            {
                /* The Frame start needs to be triggered by the driver. */
                CBUFF_setFrameStartSWTrigger (ptrDriverMCB->ptrCBUFFReg, 1U);
                CBUFF_setChirpAvailableSWTrigger (ptrDriverMCB->ptrCBUFFReg, 0U);
            }
            else
            {
                /* HW Trigger Chirp Mode: The triggers are owned by the HW */
                CBUFF_setFrameStartSWTrigger (ptrDriverMCB->ptrCBUFFReg, 0U);
                CBUFF_setChirpAvailableSWTrigger (ptrDriverMCB->ptrCBUFFReg, 0U);
            }
        }
        else
        {
            /* Software Triggered Execution Mode: The chirp available & frame start needs to be triggered
            * by the driver. */
            CBUFF_setFrameStartSWTrigger (ptrDriverMCB->ptrCBUFFReg, 1U);
            CBUFF_setChirpAvailableSWTrigger (ptrDriverMCB->ptrCBUFFReg, 1U);
        }

        /* Setup the number of chirps per frame: */
        CBUFF_setNumberChirps (ptrDriverMCB->ptrCBUFFReg, ptrSession->numChirpsPerFrame);

        /* The session is now active and can be used. */
        ptrDriverMCB->ptrCurrentActiveSession = ptrSession;

        /* Open the high speed interface (HSI) for the session: */
        DebugP_assert (ptrDriverMCB->interfaceFxn->openFxn (ptrSession, errCode) == 0);

        /* We need to determine if we need to do the manual trigger sending of the packet.
        * This has to be done in the following cases:-
        *  (1) Hardware Triggerred Mode: Handle the continuous operational mode
        *  (2) Software Triggerred Mode */
        if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
        {
            /* Are we operationg in continuous mode? */
            if (ptrSession->sessionCfg.u.hwCfg.opMode == CBUFF_OperationalMode_CONTINUOUS)
            {
                /* YES: We need to trigger the frame start manually for the first time only. */
                CBUFF_triggerFrameStart (ptrDriverMCB->ptrCBUFFReg);
            }
        }
        else
        {
            /* In Software Triggered mode: We need to trigger the frame start & chirp available */
            CBUFF_triggerFrameStart (ptrDriverMCB->ptrCBUFFReg);
            CBUFF_triggerChirpAvailable (ptrDriverMCB->ptrCBUFFReg);
        }

        /* Session has been activated: */
        retVal = 0;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get/set information from the CBUFF Driver
 *
 *  @param[in]  cBuffHandle
 *      Handle to the driver
 *  @param[in]  cmd
 *      CBUFF command
 *  @param[in]  arg
 *      Command specified pointer to the argument
 *  @param[in]  argLen
 *      Length of the argument
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CBUFF_control
(
    CBUFF_Handle    cBuffHandle,
    CBUFF_Command   cmd,
    void*           arg,
    uint32_t        argLen,
    int32_t*        errCode
)
{
    CBUFF_Object*       ptrDriverMCB;
    int32_t             retVal = SystemP_FAILURE;

    /* Sanity Check: Validate the arguments */
    ptrDriverMCB = (CBUFF_Object*)cBuffHandle;

    /* Process the command: */
    switch (cmd)
    {
        case CBUFF_Command_GET_CBUFF_STATS:
        {
            CBUFF_Stats*    ptrStats;
            uint32_t        stats;

            /* Sanity Check: Validate the command specific arguments */
            if ((arg == NULL) || (argLen != sizeof (CBUFF_Stats)))
            {
                *errCode = CBUFF_EINVAL;
                return retVal;
            }

            /* Get the pointer to the statistics: */
            ptrStats = (CBUFF_Stats*)arg;

            /* Read the statistics register */
            stats = ptrDriverMCB->ptrCBUFFReg->STAT_CBUFF_REG1;

            /* Populate the statistics: */
            ptrStats->frameStartError = CSL_FEXTR(stats, 17U, 17U);
            ptrStats->chirpError      = CSL_FEXTR(stats, 16U, 16U);

            /* Copy over the CBUFF statistics: */
            ptrStats->numFrameDone       = ptrDriverMCB->totalNumFrameDone;
            ptrStats->numChirpDone       = ptrDriverMCB->totalNumChirpDone;
            ptrStats->numErrorInterrupts = ptrDriverMCB->totalNumErrorInterrupts;

            /* Command executed successfully */
            retVal = 0;
            break;
        }
        case CBUFF_Command_CLEAR_CBUFF_STATS:
        {
            /* Reset the statistics: */
            ptrDriverMCB->totalNumFrameDone       = 0U;
            ptrDriverMCB->totalNumChirpDone       = 0U;
            ptrDriverMCB->totalNumErrorInterrupts = 0U;

            /* Command executed successfully */
            retVal = 0;
            break;
        }
        case CBUFF_Command_GET_ACTIVE_SESSION:
        {
            CBUFF_SessionHandle* ptrActiveSessionHandle;

            /* Sanity Check: Validate the command specific arguments */
            if ((arg == NULL) || (argLen != sizeof (CBUFF_SessionHandle)))
            {
                *errCode = CBUFF_EINVAL;
                DebugP_assert(0);
            }

            /* Get the pointer to the argument which is to be populated */
            ptrActiveSessionHandle = (CBUFF_SessionHandle*)arg;

            /* Return the active session handle back */
            *ptrActiveSessionHandle = (CBUFF_SessionHandle)ptrDriverMCB->ptrCurrentActiveSession;

            /* Command executed successfully */
            retVal = 0;
            break;
        }
        default:
        {
            /* Error: This is not a valid command. */
            *errCode = CBUFF_EINVAL;
            break;
        }
    }

    return retVal;
}

void CBUFF_init(void)
{
    return;
}

CBUFF_Handle CBUFF_open (CBUFF_InitCfg* ptrInitCfg, int32_t* errCode)
{
    CBUFF_Handle            retHandle = NULL;
    int32_t                 status = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;
    CBUFF_Session*          ptrSession;
    HwiP_Params             hwiPrms;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if (ptrInitCfg == NULL)
    {
        /* Error: Invalid configuration */
        *errCode = CBUFF_EINVAL;
        DebugP_assert(0);
    }
    else
    {
        /*
        * check if driver is already init
        */
        if (gCbuffObjectPtr[0] == NULL)
        {
            /* Allocate memory for the driver and initialize it */
            gCbuffObjectPtr[0] = &gCbuffObject[0];

            /* Initialize the allocated memory: */
            memset ((void *)gCbuffObjectPtr[0], 0U, sizeof(CBUFF_Object));

            gCbuffObjectPtr[0]->hwAttrs = &gCbuffAttrs[0];

            /* Allocate memory for the sessions: */
            gCbuffObjectPtr[0]->ptrSessionTable = (CBUFF_Session*)&gCbuffSession;

            /* Initialize the allocated memory: */
            memset ((void *)gCbuffObjectPtr[0]->ptrSessionTable, 0, (sizeof(CBUFF_Session) * ptrInitCfg->maxSessions));

            /* Populate the driver block: Copy over the initialization configuration */
            memcpy ((void *)&gCbuffObjectPtr[0]->initCfg, (void*)ptrInitCfg, sizeof(CBUFF_InitCfg));

            /* Sanity Check: Ensure that the high speed interface is supported or not? */
            gCbuffObjectPtr[0]->interfaceFxn = CBUFF_isIfSupported (gCbuffObjectPtr[0]->initCfg.interface);
            if (gCbuffObjectPtr[0]->interfaceFxn == NULL)
            {
                /* NO: High speed interface is not supported. This seems to be an invalid configuration */
                *errCode = CBUFF_EINVAL;
            }
            else
            {
                /* Initialize and configure the driver MCB: */
                gCbuffObjectPtr[0]->ptrCBUFFReg = (CSL_CbuffRegs*)gCbuffObjectPtr[0]->hwAttrs->baseAddr;

                /* Reset the CBUFF Module: */
                CBUFF_reset (gCbuffObjectPtr[0]->ptrCBUFFReg);

                /* Register CBUFF interrupt */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum   = gCbuffObjectPtr[0]->hwAttrs->intrNum;
                hwiPrms.callback = &CBUFF_ISR;
                hwiPrms.args     = gCbuffObjectPtr[0];
                status = HwiP_construct(&gCbuffObjectPtr[0]->hwiISRHandle, &hwiPrms);
                DebugP_assert(status == CBUFF_STATUS_SUCCESS);

                /* Register CBUFF Error interrupt */
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum   = gCbuffObjectPtr[0]->hwAttrs->errorIntrNum;
                hwiPrms.callback = &CBUFF_ErrorISR;
                hwiPrms.args     = gCbuffObjectPtr[0];
                status = HwiP_construct(&gCbuffObjectPtr[0]->hwiErrorISRHandle, &hwiPrms);
                DebugP_assert(status == CBUFF_STATUS_SUCCESS);

                /* Enable all error interrupts: */
                CBUFF_enableErrorInt (gCbuffObjectPtr[0]->ptrCBUFFReg);

                /* Register the CBUFF & Frame Start Interrupt Handlers only if the debug mode is enabled.
                * We do not want to overwhelm the system with a large number of interrupts. */
                if (gCbuffObjectPtr[0]->initCfg.enableDebugMode == true)
                {
                    /* Debug Mode: Enable the Chirp & Frame Done Interrupt: */
                    CBUFF_enableChirpDoneInt (gCbuffObjectPtr[0]->ptrCBUFFReg);
                    CBUFF_enableFrameDoneInt (gCbuffObjectPtr[0]->ptrCBUFFReg);
                }
                else
                {
                    /* Normal Mode: Disable the chirp & frame done interrupts. */
                    CBUFF_disableChirpDoneInt (gCbuffObjectPtr[0]->ptrCBUFFReg);
                    CBUFF_disableFrameDoneInt (gCbuffObjectPtr[0]->ptrCBUFFReg);
                }

                /* Cycle through and initialize the session table: */
                for (index = 0U; index < gCbuffObjectPtr[0]->initCfg.maxSessions; index++)
                {
                    /* Get the session: */
                    ptrSession = &gCbuffObjectPtr[0]->ptrSessionTable[index];

                    /* Initialize the session: */
                    ptrSession->ptrDriverMCB = gCbuffObjectPtr[0];
                    ptrSession->isValid      = false;
                    ptrSession->dmaNum       = index;
                }

                /* Initialize the high speed interface (HSI) on which the CBUFF is operating: */
                if (gCbuffObjectPtr[0]->interfaceFxn->initFxn (gCbuffObjectPtr[0], errCode) < 0)
                {
                    /* Error: Unable to initialize the HSI. Error code is already setup */
                    DebugP_assert(0);
                }

                /* Reset the linked list entries in the CBUFF IP: */
                for (index = 0U; index < CBUFF_MAX_LINKED_LIST_SIZE; index++)
                {
                    CBUFF_clearLLEntry (gCbuffObjectPtr[0]->ptrCBUFFReg, index);
                }

                /* Setup the ECC: */
                CBUFF_setECC (gCbuffObjectPtr[0]->ptrCBUFFReg, gCbuffObjectPtr[0]->initCfg.enableECC);

                /* Default State: The triggers are always owned by the software */
                CBUFF_setFrameStartSWTrigger (gCbuffObjectPtr[0]->ptrCBUFFReg, 1U);
                CBUFF_setChirpAvailableSWTrigger (gCbuffObjectPtr[0]->ptrCBUFFReg, 1U);

                /* Release the reset of the CBUFF Module.  */
                CBUFF_releaseReset (gCbuffObjectPtr[0]->ptrCBUFFReg);

                /* Configure Pad registers for LVDS. */
                HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_LVDS_PAD_CTRL0 , 0x0);
                HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_LVDS_PAD_CTRL1 , 0x02000000);

                /* NO. CBUFF Initialization was successful */
                retHandle = (CBUFF_Handle)gCbuffObjectPtr[0];
            }
        }
    }

    return retHandle;
}

CBUFF_SessionHandle CBUFF_createSession (CBUFF_Handle cbuffHandle, CBUFF_SessionCfg* ptrSessionCfg, int32_t* errCode)
{
    CBUFF_Object*           ptrDriverMCB;
    CBUFF_SessionHandle     sessionHandle;
    CBUFF_Session*          ptrSession = NULL;
    int32_t                 tmpErrCode;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the driver MCB: */
    ptrDriverMCB = (CBUFF_Object*)cbuffHandle;

    /* Sanity Check: Validate the arguments */
    if ((ptrDriverMCB == NULL) || (ptrSessionCfg == NULL))
    {
        /* Error: Invalid configuration */
        *errCode = CBUFF_EINVAL;
        CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
        sessionHandle = NULL;
        DebugP_assert(0);
    }

    /* Sanity Check: Validate the arguments */
    if ((ptrSessionCfg->edmaHandle             == NULL) ||
        (ptrSessionCfg->allocateEDMAChannelFxn == NULL) ||
        (ptrSessionCfg->freeEDMAChannelFxn     == NULL))
    {
        *errCode = CBUFF_EINVAL;
        CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
        sessionHandle = NULL;
        DebugP_assert(0);
    }

    /* Allocate a session: */
    ptrSession = CBUFF_allocateSession (ptrDriverMCB);
    if (ptrSession == NULL)
    {
        /* Error: Out of sessions. Limit reached. */
        *errCode = CBUFF_ELIMIT;
        CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
        sessionHandle = NULL;
        DebugP_assert(0);
    }

    /* Copy over the session configuration: */
    memcpy ((void *)&ptrSession->sessionCfg, (void*)ptrSessionCfg, sizeof (CBUFF_SessionCfg));

    /* Do we need to add a header to the stream? */
    if (CBUFF_validateBufferCfg (&ptrSession->sessionCfg.header, errCode) < 0)
    {
        /* Error: Invalid header configuration error code is already setup */
        CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
        sessionHandle = NULL;
        DebugP_assert(0);
    }

    /* Process the session configuration on the basis of the execution mode: */
    if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
    {
        /* Sanity Check: Validate the HW Session configuration */
        if (CBUFF_validateHWSessionCfg (ptrSession, errCode) < 0)
        {
            /* Error: Session configuration was invalid. Error code is setup */
            CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
            sessionHandle = NULL;
            DebugP_assert(0);
        }

        /* Determine the ADC Transfer Size: For Complex Samples we send I/Q */
        ptrSession->adcTransferSize = 0U;
        if (ptrSession->sessionCfg.dataType == CBUFF_DataType_REAL)
        {
            /* Real: */
            ptrSession->adcTransferSize = ptrSession->sessionCfg.u.hwCfg.numADCSamples;
        }
        else
        {
            /* Complex: */
            ptrSession->adcTransferSize = ptrSession->sessionCfg.u.hwCfg.numADCSamples * 2;
        }

        /* Setup the number of chirps per frame: */
        if (ptrSession->sessionCfg.u.hwCfg.opMode == CBUFF_OperationalMode_CHIRP)
        {
            /* Chirp Mode: Use the provided configuration */
            ptrSession->numChirpsPerFrame = (ptrSession->sessionCfg.u.hwCfg.numChirpsPerFrame / ptrSession->sessionCfg.u.hwCfg.chirpMode);
            ptrSession->adcTransferSize   = ptrSession->adcTransferSize * ptrSession->sessionCfg.u.hwCfg.chirpMode;
        }
        else
        {
            /* Continuous Mode: There are infinite chirps per frame. */
            ptrSession->numChirpsPerFrame = 0xFFFFFFFFU;
        }
    }
    else
    {
        /* Sanity Check: Validate the SW Session configuration */
        if (CBUFF_validateSWSessionCfg (ptrSession, errCode) < 0)
        {
            /* Error: Session configuration was invalid. Error code is setup */
            CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
            sessionHandle = NULL;
            DebugP_assert(0);
        }

        /* There is only 1 chirp per frame. */
        ptrSession->numChirpsPerFrame = 1U;
    }

    /* Setup the transfer: */
    if (CBUFF_setupTransfer (ptrSession, errCode) < 0)
    {
        /*Error: Unable to setup the transfer. Error code is already setup*/
        CBUFF_close ((CBUFF_SessionHandle)ptrSession, &tmpErrCode);
        sessionHandle = NULL;
        DebugP_assert(0);
    }

    /* Register for the Frame Done Interrupts */
    CBUFF_registerFrameDone (ptrDriverMCB);

    /* NO: Set the session handle */
    sessionHandle = (CBUFF_SessionHandle)ptrSession;

    return sessionHandle;
}

int32_t CBUFF_close(CBUFF_SessionHandle sessionHandle, int32_t* errCode)
{
    int32_t             retVal = SystemP_FAILURE;
    CBUFF_Object*       ptrDriverMCB;
    CBUFF_Session*      ptrSession;

    *errCode = CBUFF_STATUS_SUCCESS;

    /* Get the pointer to the session: */
    ptrSession = (CBUFF_Session*)sessionHandle;
    if (ptrSession == NULL)
    {
        *errCode = CBUFF_EINVAL;
        DebugP_assert(0);
    }
    else
    {
        /* Sanity Check: Ensure that there exists a back pointer to the CBUFF Driver */
        ptrDriverMCB = ptrSession->ptrDriverMCB;
        DebugP_assert (ptrDriverMCB != NULL);

        /* Is the session currently active? */
        if (ptrDriverMCB->ptrCurrentActiveSession == ptrSession)
        {
            /* YES: Application needs to deactivate the session before it
            * can be deleted. This needs to be done explicitly by the application. */
            *errCode = CBUFF_EINUSE;
            DebugP_assert(0);
        }

        /* Shutdown the transfer: */
        retVal = CBUFF_shutdownTransfer (ptrSession, errCode);
        DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

        /* Free the session: */
        CBUFF_freeSession (ptrSession);

        /* Deregister for the Frame Done Interrupts */
        CBUFF_deregisterFrameDone (ptrDriverMCB);

        /* Session has been successfully deleted */
        retVal = 0;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize and shutdown the CBUFF driver.
 *
 *  @param[in]  cBuffHandle
 *      Handle to the driver
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t CBUFF_deinit (CBUFF_Handle cBuffHandle, int32_t* errCode)
{
    CBUFF_Object*       ptrDriverMCB;
    CBUFF_Session*      ptrSession;
    int32_t             retVal = SystemP_FAILURE;
    uint8_t             sessionIndex;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    ptrDriverMCB = (CBUFF_Object*)cBuffHandle;
    if ((ptrDriverMCB == NULL) || (ptrDriverMCB->ptrCurrentActiveSession != NULL))
    {
        *errCode = CBUFF_EINVAL;
        DebugP_assert(0);
    }
    else
    {
        /* Disable all the interrupts: The driver is going down. */
        CBUFF_disableErrorInt (ptrDriverMCB->ptrCBUFFReg);
        CBUFF_disableFrameDoneInt (ptrDriverMCB->ptrCBUFFReg);
        CBUFF_disableChirpDoneInt (ptrDriverMCB->ptrCBUFFReg);

        /* Was the session table created? */
        if (ptrDriverMCB->ptrSessionTable != NULL)
        {
            /* YES: Cycle through and delete all the valid sessions */
            for (sessionIndex = 0U; sessionIndex < ptrDriverMCB->initCfg.maxSessions; sessionIndex++)
            {
                /* Get the session: */
                ptrSession = &ptrDriverMCB->ptrSessionTable[sessionIndex];

                /* Was the session valid? */
                if (ptrSession->isValid == true)
                {
                    /* YES: Delete the session */
                    retVal = CBUFF_close ((CBUFF_SessionHandle)ptrSession, errCode);
                    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);
                }
            }
        }

        /* Deregister the CBUFF Error ISR Handler only if had been registered */
        HwiP_destruct(&ptrDriverMCB->hwiErrorISRHandle);

        /* Deregister the CBUFF Interrupt handlers only if it had been registered */
        HwiP_destruct(&ptrDriverMCB->hwiISRHandle);

        /* Deinitialize the high speed interface: */
        retVal = ptrDriverMCB->interfaceFxn->deinitFxn (ptrDriverMCB, errCode);
        DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

        /* CBUFF has been deinitialized successfully. */
        retVal = 0;
    }

    return retVal;
}

