/*
 *  Copyright (C) 2021-2022 Texas Instruments Incorporated
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
 *  \defgroup SECURITY_SA2UL_MODULE APIs for SA2UL
 *  \ingroup  SECURITY_MODULE
 *
 *  This module contains APIs to program and use the SA2UL.
 *
 *  @{
 */

/**
 *  \file sa2ul.h
 *
 *  \brief This file contains the prototype of SA2UL driver APIs
 */

#ifndef SA2UL_H_
#define SA2UL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <drivers/udma.h>
#include <security/crypto/pka/hw_include/cslr_cp_ace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor SA2UL_InputSizes
 *  \name SA2UL macros
 *  @{
 */
/** \brief Max hash size in bytes */
#define SA2UL_MAX_HASH_SIZE_BYTES       (64U)
/** \brief Max key size in bytes */
#define SA2UL_MAX_KEY_SIZE_BYTES        (32U)
/** \brief Max Initialization vector (IV) size in bytes */
#define SA2UL_MAX_IV_SIZE_BYTES         (16U)
/** \brief Max number of rings */
#define SA2UL_NUMBER_OF_RINGS           (3U)
/** \brief Cache line size for alignment of descriptor and buffers */
#define SA2UL_CACHELINE_ALIGNMENT       (16U)
/** \brief Secure context memsize */
#define SA2UL_SECCTX_SIZE               (256U)
/** \brief Max SA2UL instances */
#define SA2UL_INSTANCES_MAX             (4U)
/** \brief Max SA2UL ring n elements */
#define SA2UL_RING_N_ELEMS              (8U)
/** \brief Max SA2UL sw ring number */
#define SA2UL_SW_RING_NUM               (304U)
/** \brief Max Aes input length for encryption and decryption (64KB-1)*/
#define SA2UL_MAX_INPUT_LENGTH_ENC      (0xFFFFU)
/** \brief Max input length for authentication (4MB-1) */
#define SA2UL_MAX_INPUT_LENGTH_AUTH     (0x3FFFFFU)
/** @} */

/**
 *  \anchor SA2UL_AlgoTypes
 *  \name SA2UL Algo Types
 *  @{
 */
/** \brief Hash Algo NULL */
#define SA2UL_HASH_ALG_NULL             (0x10U)
/** \brief Hash Algo SHA-1 */
#define SA2UL_HASH_ALG_SHA1             (0x12U)
/** \brief Hash Algo SHA-256 */
#define SA2UL_HASH_ALG_SHA2_256         (0x14U)
/** \brief Hash Algo SHA-512 */
#define SA2UL_HASH_ALG_SHA2_512         (0x16U)
/** @} */

/**
 *  \anchor SA2UL_Operations_Sequence
 *  \name SA2UL Operations Sequence
 *  @{
 */
/** \brief SA2UL operations encryption */
#define SA2UL_OP_ENC                    (0x01U)
/** \brief SA2UL operations authentication */
#define SA2UL_OP_AUTH                   (0x02U)
/** \brief SA2UL operation encryption then authentication */
#define SA2UL_OP_ENC_THEN_AUTH          (0x03U)
/** \brief SA2UL operations authentication then encryption */
#define SA2UL_OP_AUTH_THEN_ENC          (0x04U)
/** @} */

/**
 *  \anchor SA2UL_Engine_Operation_Type
 *  \name SA2UL engine operation type
 *  @{
 */
/** \brief SA2UL default engine Id */
#define SA2UL_ENGINE_CODE_DEFAULT_ENGINE_ID             (0U)
/** \brief SA2UL Encryption engine module p1 */
#define SA2UL_ENGINE_CODE_ENCRYPTION_MODULE_P1          (2U)
/** \brief SA2UL Encryption engine module p2 */
#define SA2UL_ENGINE_CODE_ENCRYPTION_MODULE_P2          (3U)
/** \brief SA2UL Authentication engine module p1 */
#define SA2UL_ENGINE_CODE_AUTHENTICATION_MODULE_P1      (4U)
/** \brief SA2UL Authentication engine module p2 */
#define SA2UL_ENGINE_CODE_AUTHENTICATION_MODULE_P2      (5U)
/** \brief SA2UL Default Engress Port */
#define SA2UL_ENGINE_CODE_DEFAULT_EGRESS_PORT           (20U)
/** @} */

/**
 *  \anchor SA2UL_Encryption_Algos
 *  \name SA2UL Encryption Algos
 *  @{
 */
/** \brief SA2UL AES Encryption Algorithm */
#define SA2UL_ENC_ALG_AES               (0x0U)
/** \brief SA2UL TDES Encryption Algorithm */
#define SA2UL_ENC_ALG_TDES              (0x1U)
/** \brief SA2UL Encryption Maximun number of Algorithms */
#define SA2UL_ENC_ALG_MAX               (0x2U)
/** @} */

/**
 *  \anchor SA2UL_Encryption_Direction
 *  \name SA2UL Encryption direction
 *  @{
 */
/** \brief Encryption direction */
#define SA2UL_ENC_DIR_ENCRYPT           (0x0U)
/** \brief Decryption direction */
#define SA2UL_ENC_DIR_DECRYPT           (0x1U)
/** @} */

/**
 *  \anchor SA2UL_Encryption_Modes
 *  \name SA2UL Encryption modes
 *  @{
 */
/** \brief ECB mode */
#define SA2UL_ENC_MODE_ECB              (0x0U)
/** \brief CBC mode */
#define SA2UL_ENC_MODE_CBC              (0x1U)
/** \brief Maximun Encryption modes */
#define SA2UL_ENC_MODE_MAX              (0x2U)
/** @} */

/**
 *  \anchor SA2UL_Encryption_Key_Sizes
 *  \name SA2UL Encryption key sizes
 *  @{
 */
/** \brief Encryption 128 bit key size */
#define SA2UL_ENC_KEYSIZE_128           (0x0U)
/** \brief Encryption 192 bit key size */
#define SA2UL_ENC_KEYSIZE_192           (0x1U)
/** \brief Encryption 256 bit key size */
#define SA2UL_ENC_KEYSIZE_256           (0x2U)
/** \brief Max key size */
#define SA2UL_ENC_KEYSIZE_MAX           (0x3U)
/** @} */

#define SA2UL_ENC_KEYSIZE_BITS(k)       (128u + (64u * (k)))
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \anchor SA2UL_Handle
 *  \brief Handle to the SA2UL driver returned by #SA2UL_open() */
typedef void *SA2UL_Handle;

/**
 *  \brief Parameters passed to #SA2UL_contextAlloc()
 */
typedef struct
{
    uint8_t                 opType;
    /**< SA operation type, refer \ref SA2UL_Engine_Operation_Type*/
    uint8_t                 hashAlg;
    /**< Hash algorithm, refer \ref SA2UL_AlgoTypes*/
    uint8_t                 encAlg;
    /**< Encryption algorithm to run, refer \ref SA2UL_Encryption_Algos*/
    uint8_t                 encDirection;
    /**< Encrypt or decrypt, refer \ref SA2UL_Encryption_Direction*/
    uint8_t                 encMode;
    /**< Encryption mode to run, refer \ref SA2UL_Encryption_Modes*/
    uint8_t                 key[SA2UL_MAX_KEY_SIZE_BYTES];
    /**< Key input for encryption */
    uint8_t                 encKeySize;
    /**< Encryption key size, refer \ref SA2UL_Encryption_Key_Sizes*/
    uint32_t                inputLen;
    /**< Overall data length, must be sum of all packet lengths */
    uint8_t                 iv[SA2UL_MAX_IV_SIZE_BYTES];
    /**< IV input for encryption, refer \ref SA2UL_InputSizes */
} SA2UL_ContextParams;

/**
 * \anchor SA2UL_Params 
 *  \brief Parameters passed to #SA2UL_open()
 */
typedef struct
{
    uint32_t                reserved;
     /**< tbd for future */
} SA2UL_Params;

/**
 * \brief Object for a SA2UL crypto operation
 */
typedef struct
{
    SA2UL_Handle            handle;
    /**< Instance handle to which this object belongs, refer \ref SA2UL_Handle */
    SA2UL_Params            prms;
    /**< Open parameter as provided by user, refer \ref SA2UL_Params*/
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    CSL_Cp_aceRegs          *pSaRegs;
    /**< Pointer to register overlay */
    uint32_t                psilInst;
    /**< SA2UL psil Instance */
    uint32_t                ringaccChnls[SA2UL_NUMBER_OF_RINGS];
    /**< SA2UL psil Instance */
    uint32_t                udmapTxChnum;
    /**< UDMA tx channel allocated */
    uint32_t                udmapRxChnum[2];
    /**< UDMA Rx channel allocated */
    uint32_t                udmapRxFlownum;
    /**< UDMA Rx flow  allocated */
    /*
     * UDMA objects and druver handles
     */
    Udma_ChObject           udmaTxChObj;
    /**< UDMA tx ch object */
    Udma_ChObject           udmaRxChObj[2];
     /**< UDMA rx ch objects */
    Udma_FlowObject         udmaFlowObj;
     /**< UDMA rx flow object */
    Udma_RingObject         udmaRingObj[2];
    /**< UDMA ring object */
    Udma_EventObject        udmaEvtObj;
    /**< UDMA event object */
    Udma_EventObject        udmaEvtObj0;
    /**< UDMA event object0 */
    Udma_DrvHandle          drvHandle;
    /**< UDMA driver handle */
    Udma_ChHandle           rxChHandle[2];
    /**< UDMA rx channel handles */
    Udma_ChHandle           txChHandle;
    /**< UDMA tx channel handle */
    Udma_FlowHandle         rxFlowHandle;
    /**< UDMA rx flow  handle */
    Udma_RingHandle         rxFreeRingHandle;
    /**< UDMA rx free ring handle */
    Udma_RingHandle         rxRingHandle;
    /**< UDMA rx  ring handle */
    Udma_RingHandle         txComplRingHandle;
    /**< UDMA tx completion ring handle */
    Udma_RingHandle         txRingHandle;
    /**< UDMA tx  ring handle */
    Udma_EventHandle        rxEvtHandle;
    /**< UDMA rx event handle */
    /*
     * sa2ul storage Queue params
     */
    uint32_t                storageQueueHead;
    uint32_t                storageQueueTail;
    uint32_t                storageQueueFree;
    uint64_t                storageQueue[SA2UL_RING_N_ELEMS];
    /*
     * memory for SA2UL security context
     *
     * Caller must allocate SA2UL_SECCTX_SIZE bytes of 128-byte aligned
     * memory for this.
     */
    uint8_t                 sa2ulContextId[SA2UL_INSTANCES_MAX];
    /**< secure contextId array */
    uint8_t                 contextId;
    /**< secure context Id */
} SA2UL_Object;

/** \brief SA2UL instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                saBaseAddr;
    /**< Peripheral SA base address */
    uint32_t                txPsilThreadId;
    /**< Tx ch psil thread number  */
    uint32_t                rxPsil0ThreadId;
    /**< Rx ch0 psil thread number  */
    uint32_t                rxPsil1ThreadId;
    /**< Rx ch1 psil thread number  */
    uint32_t                rxRingMemAddr;
    /**< Mem address for RX ring elements  */
    uint32_t                txRingMemAddr;
    /**< Mem address for TX ring elements  */
    uint32_t                numDesc;
    /**< Descriptor count  */
    uint32_t                descSize;
    /**< Descriptor size  */
    uint32_t                descMemAddr;
    /**< Descriptor mem Address  */
    uint32_t                descMemSize;
    /**< Descriptor mem size in Bytes  */
    uint32_t                udmaSaTxGroupNum;
    /**< SA2UL Tx group num  */
    uint32_t                udmaSaRxGroupNum;
    /**< SA2UL Rx group num  */
    uint32_t                ringCnt;
    /**< Num rings  */
    uint32_t                udmaHandle;
    /**< Handle for UDMA object */
    uint32_t                contextIdStart;
    /**< secure context start index */
    uint32_t                contextIdMaxNum;
    /**< secure context max index*/
    uint32_t                txRingNumInt;
    /**< tx ring num internal always 0*/
    uint32_t                rxRingNumInt;
    /**< rx ring num internal always 1*/
    uint32_t                swRingNumInt;
    /**< sw ring num internal always 2*/
    uint32_t                privId;
    /**< core privileged Id*/
    uint32_t                priv;
    /**< core privileged Id*/
    uint32_t                secure;
    /**< secure transaction*/
} SA2UL_Attrs;

/** \brief SA2UL driver context */
typedef struct
{
    SA2UL_Attrs             *attrs;
    /**< Driver params passed during open */
    SA2UL_Object            *object;
    /**< Pointer to driver functions */
} SA2UL_Config;

/** \brief MEDdata for Encryption engine */
typedef struct
{
    uint8_t                 sopOffset;
    uint8_t                 middleOffset;
    uint8_t                 eopOffset;
    uint8_t                 nMCInstrs;
    const uint8_t          *mcInstrs;
}SA2UL_MCEData;

/** \brief SA2UL context RAM structure for authentication engine. */
typedef struct 
{
    uint32_t                authCtx1;
    uint32_t                reserved0;
    uint32_t                authenticationLengthHi;
    uint32_t                authenticationLengthLo;
    uint32_t                reserved1[4];
    uint32_t                authenticationKeyValueL[8];
    uint32_t                oPadL[8];
    uint32_t                authenticationKeyValueH[8];
    uint32_t                oPadH[8];
}SA2UL_SecCtxAuth;

/**  \brief SA2UL SC control structure */
typedef struct
{
    uint32_t                scctl1;
    uint32_t                scctl2;
    uint32_t                scptrH;
    uint32_t                scptrL;
}SA2UL_Scctl;

/** \brief SA2UL context RAM structure for Encryption engine */
typedef struct 
{
    uint32_t                encrCtl;
    uint32_t                modeCtrlInstrs[6];
    uint32_t                hwCtrlWord;
    uint32_t                encKeyValue[8];
    uint32_t                encAux1[8];
    uint32_t                encAux2[4];
    uint32_t                encAux3[4];
    uint32_t                encAux4[4];
    uint8_t                 preCryptoData[15];
}SA2UL_SecCtxEnc;

/**
 * \brief SA2UL context RAM structure
 */
typedef struct 
{
    SA2UL_Scctl      scctl;
    uint32_t         unused[12];
    union
    {
        SA2UL_SecCtxAuth  auth;
        SA2UL_SecCtxEnc   enc;
    } u;
}SA2UL_SecCtx;

/**
 * \brief SA2UL context object structure
 */
typedef struct
{
    SA2UL_SecCtx            secCtx;
    uint8_t                 res[32];
    uint8_t                 secCtxId;
    SA2UL_Handle            handle;
    SA2UL_ContextParams     ctxPrms;
    /**< SA2UL secure parameter as provided by user */
    uint32_t                totalLengthInBytes;
    uint8_t                 computedHash[SA2UL_MAX_HASH_SIZE_BYTES];
    uint32_t                computationStatus;
    uint32_t                txBytesCnt;
    uint32_t                rxBytesCnt;
    uint32_t                sa2ulErrCnt;
} SA2UL_ContextObject;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Externally defined driver configuration array */
extern SA2UL_Config         gSa2ulConfig[];
/** \brief Externally defined driver configuration Num */
extern uint32_t             gSa2ulConfigNum;
/** \brief Externally defined driver engine configuration mask */
extern uint32_t             gSa2ulEngineEnableMask;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/**
 *  \brief  This function initializes the SA2UL module
 */
void SA2UL_init(void);

/**
 *  \brief  This function de-initializes the SA2UL module
 */
void SA2UL_deinit(void);

/**
 *  \brief  This function enable particular SA2UL engine based on user input mask.
 *
 *  \param  hwEnableMask Mask to enable user specific engine.
 */
int32_t SA2UL_engineEnable(int32_t hwEnableMask);

/**
 *  \brief  This function disable particular SA2UL engine based on user input mask.
 *
 *  \param  hwDisableMask Mask to disable user specific engine.
 */
int32_t SA2UL_engineDisable(int32_t hwDisableMask);

/**
 *  \brief  This function opens a given SA2UL module
 *
 *  \param  index    Index of config to use in the *SA2UL_Config* array
 *
 *  \param  params   Pointer to open parameters. If NULL is passed, then
 *                   default values will be used
 *
 *  \return A #SA2UL_Handle on success or a NULL on an error or if it has been
 *          opened already
 */
SA2UL_Handle SA2UL_open(uint32_t index, const SA2UL_Params *params);

/**
 *  \brief  Function to close a SA2UL module specified by the SA2UL handle
 *
 *  \param  handle  #SA2UL_Handle returned from #SA2UL_open()
 */
void SA2UL_close(SA2UL_Handle handle);

/**
 *  \brief  Function to configure secure context
 *
 *  \param  handle  #SA2UL_Handle returned from #SA2UL_open()
 *
 *  \param  ctxObj  Pointer to #SA2UL_ContextObject structure
 *
 *  \param  ctxPrms Pointer to secure context parameters.If NULL is passed, then
 *                  default values will be used
 *
 *  \return A #SystemP_SUCCESS on success or a NULL on an error or if the instance
 *            index has  NOT been opened yet
 */
int32_t SA2UL_contextAlloc(SA2UL_Handle handle, SA2UL_ContextObject *ctxObj, const SA2UL_ContextParams *ctxPrms);

/**
 *  \brief  Function to free secure context configuration
 *
 *  \param  pCtxObj  Pointer to *SA2UL_ContextObject* structure
 *
 *  \return A #SystemP_SUCCESS on success or a NULL on an error or if the instance
 *            index has  NOT been opened yet
 */
int32_t SA2UL_contextFree(SA2UL_ContextObject *pCtxObj);

/**
 *  \brief  Function to transfer and recieve data buffer
 *
 *  \param  ctxObj  Pointer to *SA2UL_ContextObject* structure
 *
 *  \param  input   Pointer to input buffer to be processed
 *
 *  \param  ilen    Length of the input buffer
 *
 *  \param  output  Pointer to output buffer after the process is complete
 *
 *  \return A #SystemP_SUCCESS on success or a NULL on an error or if the instance
 *            index has  NOT been opened yet */
int32_t SA2UL_contextProcess(SA2UL_ContextObject *ctxObj, const uint8_t *input, uint32_t ilen, uint8_t *output);

/**
 *  \brief  Function to initialize default SA2UL parameters
 *
 *  \param  prms  Pointer to open parameters.
 */
static inline void SA2UL_Params_init(SA2UL_Params *prms);

static inline void SA2UL_ContextParams_init(SA2UL_ContextParams *ctxPrms);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void SA2UL_Params_init(SA2UL_Params *prms)
{
    if(prms != NULL)
    {
        prms->reserved                        = 0;
    }
}

static inline void SA2UL_ContextParams_init(SA2UL_ContextParams *ctxPrms)
{
    if(ctxPrms != NULL)
    {
        memset(ctxPrms, 0, sizeof(*ctxPrms));
    }
}

#ifdef __cplusplus
}
#endif

#endif /* SA2UL_H_ */

/** @} */
