/*
 * SDL ECC
 *
 * Software Diagnostics Library module for ECC
 *
 *  Copyright (c) Texas Instruments Incorporated 2022-2024
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
 *
 */

#include <string.h>
#include <stdint.h>
#include <sdl/sdl_common.h>
#include <sdl/sdl_ecc.h>
#include <sdl/include/sdl_types.h>
#include <sdl/esm/soc/am263px/sdl_esm_core.h>
#include <sdl/sdl_exception.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/ecc/soc/am263px/sdl_ecc_soc.h>
#include <sdl/ecc/sdl_ecc_core.h>
#include <sdl/ecc/sdl_ecc_priv.h>
#include <sdl/dpl/sdl_dpl.h>


/* Local defines */
#define SDL_ECC_INVALID_ERROR_SOURCE      (0xffffffffu)

#define SDL_ECC_BITS_PER_WORD             (32U)

#define ECC_AGGR_LINE_SIZE                (4U)

#define SDL_ECC_INVALID_SELF_TEST_RAM_ID  (0xffffffffu)
#define SDL_ECC_INVALID_CHECKER_TYPE      (0xffffffffu)

#define SDL_ESM_MAX_EVENT_MAP_WORDS       (32U)

/* Event BitMap for ECC ESM callback for MAIN */
uint32_t eventBitMapMAIN[SDL_ESM_MAX_EVENT_MAP_WORDS] =
{
     0xffffffffu, 0xffffffffu, 0x0007e01fu, 0x00007ff8u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
};

/* ========================================================================== */
/*                           enums                                            */
/* ========================================================================== */

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the values for ecc self test flag
 * ----------------------------------------------------------------------------
 */
typedef enum
{
    SDL_ECC_ERROR_FLAG_NONE=0,
    /**< Flag set during ECC initialization or end of the ECC self test */
    SDL_ECC_ERROR_FLAG_STARTING=1,
    /**< Flag set at the start of the ECC self test */
    SDL_ECC_ERROR_FLAG_TRIGGERED=2,
    /**< Flag set when ECC error happens during the ECC self test */

} SDL_ECC_ErrorFlag;

/** ---------------------------------------------------------------------------
 * @brief This structure defines the elements of ECC software instance
 * ----------------------------------------------------------------------------
 */

/**
 * Design: PROC_SDL-1295
 */
typedef struct SDL_ECC_Instance_s
{
    SDL_ECC_InitConfig_t eccInitConfig;
    /**< ecc Initial configuration */
    SDL_ECC_ErrorFlag eccErrorFlag;
    /**< Ecc error triggered flag */
    SDL_ECC_InjectErrorType eccSelfTestErrorType;
    /**< Ecc self type error type in progress */
    uint32_t eccSelfTestRamId;
    /**< Ram id used in self test in progress */
    uint32_t *eccSelfTestAddr;
    /**< Address used in self test in progress */
    uint32_t eccSelfTestcheckerType;
    /**< Group checker (for Interconnect RAM ID's only) */
}  SDL_ECC_Instance_t;

/* Global objects */
static SDL_ECC_Instance_t SDL_ECC_instance[SDL_ECC_Base_Address_TOTAL_ENTRIES];
/* Local functions */
static int32_t SDL_ECC_getRamId(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType,
                           uint32_t *ramIdP, uint32_t *pRAMIdType);
static int32_t SDL_ECC_getAggregatorType(SDL_ECC_MemType eccMemType,
                           SDL_ECC_MemSubType memSubType, uint32_t *pIinjectOnly);
static int32_t SDL_ECC_getAggrBaseAddr(SDL_ECC_MemType eccMemType, SDL_ecc_aggrRegs **pEccAggr);
static int32_t SDL_ECC_memoryRefresh(uint32_t *memAddr, size_t size);
static uint32_t SDL_ECC_triggerAccessForEcc(const uint32_t *memoryAccessAddr);

static int32_t SDL_ECC_getMemConfig(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType,
                               SDL_MemConfig_t *memConfig);


static uint32_t SDL_ECC_getDetectErrorSource (SDL_ECC_InjectErrorType injectErorType);

static int32_t SDL_ECC_getBitLocation(uint32_t bitMask,
                            uint32_t startBitLocation,
                            uint32_t *bitLocation);
static int32_t SDL_ECC_handleEccAggrEvent (SDL_ECC_MemType eccMemType, uint32_t errorSrc,
                                       uint32_t errorAddr);
static int32_t SDL_ECC_ESMCallBackFunction_MAIN (SDL_ESM_Inst instance, SDL_ESM_IntType intrType,
                                             uint32_t grpChannel, uint32_t index, uint32_t intSrc,
                                             void *arg);
static int32_t SDL_ECC_checkMemoryType(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType);

static int32_t SDL_ECC_searchMemEntryTable(SDL_ECC_MemSubType memSubType,
                                              const SDL_MemConfig_t memEntryTable[],
                                              uint32_t tableSize,
                                              SDL_MemConfig_t *pMemConfig);
static int32_t SDL_ECC_tcmparityerrForce_Regs(SDL_ECC_MemType eccMemType,
							  SDL_ECC_MemSubType memSubType,
							  uint32_t RegValue);

static int32_t SDL_ECC_enableParityerr(uint32_t setmask, uint32_t paramregs,
											   uint32_t paramval);
/** ============================================================================*
 *
 * \brief   Get the Error Source corresponding to the inject error
 *          type
 *
 *   @n  This function is called in ECC callback function to convert the injected
 *       error type to the source of the ECC error
 *
 * \param1  errorType: error Type
 *
 * \return  error Source or SDL_ECC_INVALID_ERROR_SOURCE in case of error
 */
static uint32_t SDL_ECC_getDetectErrorSource (SDL_ECC_InjectErrorType injectErorType)
{
    uint32_t errorSource;

    switch(injectErorType)
    {
        case SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE:
        case SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE:
        case SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT:
        case SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT:
            errorSource = SDL_ESM_ECC_PARAM_MCU_CPU0_SEC_ERROR;
            break;


        case SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE:
        case SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE:
        case SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT:
        case SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT:
            errorSource = SDL_ESM_ECC_PARAM_MCU_CPU0_DED_ERROR;
            break;

        default:
            errorSource = ((uint32_t)SDL_ECC_INVALID_ERROR_SOURCE);
            break;
    }
    return errorSource;
}


static int32_t SDL_ECC_mapRatEccAggrBaseAddress(SDL_ecc_aggrRegs * const EccAggrAddr, int32_t desiredMapIdx, SDL_ecc_aggrRegs **ppEccAggr)
{
    int32_t mappedIdx = -1;
    void *localAddr = (void *)(-1);
    uint32_t size = SDL_ECC_AGG_TOP_U_SIZE;
    localAddr = SDL_DPL_addrTranslate((uint64_t) EccAggrAddr,size);
    if (localAddr != (void *)(-1))
    {
        mappedIdx = desiredMapIdx;
        *ppEccAggr = (SDL_ecc_aggrRegs *)localAddr;
    }

    return (mappedIdx);
}



/** ============================================================================*
 *
 * \brief   Map an ECC Aggregator using the RAT
 *
 *   @n  This function is called prior to SDL_ECC_init() for an ECC aggregator
 *   where the base address of the ECC Aggegator is greater than 32-bits, so
 *   needs a mapping by the RAT.
 *
 * \param1  errorType: error Type
 *
 * \return  error Source or SDL_ECC_INVALID_ERROR_SOURCE in case of error
 */
/* Returns index >= 0 if successful, otherwise returns -1 if failure */
static int32_t SDL_ECC_mapEccAggrReg(SDL_ECC_MemType eccMemType, SDL_ecc_aggrRegs **ppEccAggr)
{
    int32_t retVal = SDL_PASS;
    SDL_ecc_aggrRegs *eccAggrRegs;
    int32_t mapIdx;

    eccAggrRegs = *ppEccAggr;
    mapIdx = SDL_ECC_mapRatEccAggrBaseAddress(SDL_ECC_aggrBaseAddressTable[eccMemType],
                                  (int32_t)(eccMemType),
                                  &eccAggrRegs);
    if (mapIdx < 0)
    {
        retVal = SDL_EFAIL;
    }
    else
    {
        /* Mapping was successful, so fill the array with the local address */
        SDL_ECC_aggrTransBaseAddressTable[mapIdx] = eccAggrRegs;
        *ppEccAggr = eccAggrRegs;
    }

    return retVal;
}
/** ============================================================================*
 *
 * \brief   Ecc call back function registered with exception handler
 *
 * \param1  eccMemType: ECC Memory Type
 * \param2  errorSrc: Source of ECC Error
 * \param3  errorAddr: ECC error address
 *
 * \return  None
 */
static int32_t SDL_ECC_handleEccAggrEvent (SDL_ECC_MemType eccMemType, uint32_t errorSrc,
                                           uint32_t errorAddr)
{
    SDL_ecc_aggrRegs *eccAggrRegs;
    SDL_Ecc_AggrEccRamErrorStatusInfo eccErrorStatusWrap;
    uint32_t bitErrCnt;
    uint32_t ramId=0U;
    uint32_t ramIdType=0U;
    SDL_ECC_MemSubType memSubType;
    uint32_t i;
    bool eventFound1;
    int32_t sdlResult;
    uint32_t selfTestErrorSrc;
    int32_t handledResult = 0;

    (void)SDL_ECC_getAggrBaseAddr(eccMemType, &eccAggrRegs);
    eccErrorStatusWrap.singleBitErrorCount = 0x0u;
    eccErrorStatusWrap.doubleBitErrorCount = 0x0u;

    /* Check which Ram Id triggered the error */
    for (i = ((uint32_t)0U);
         i < SDL_ECC_instance[eccMemType].eccInitConfig.numRams;
         i++)
    {
        /* Get corresponding ram Id */
        memSubType = SDL_ECC_instance[eccMemType].eccInitConfig.pMemSubTypeList[i];
        (void)SDL_ECC_getRamId(eccMemType, memSubType, &ramId, &ramIdType);

        /* Check if this event is triggered, by reading the ECC aggregator status
         * register */
        sdlResult = SDL_ecc_aggrIsEccRamIntrPending(eccAggrRegs,
                                                    ramId,
                                                    errorSrc,
                                                    &eventFound1);

        if((sdlResult == SDL_PASS)  && (eventFound1))
        {

            /* Read the locations of the bit errors */
            /* Get the error status information for Wrapper type */
            (void)SDL_ecc_aggrGetEccRamErrorStatus(eccAggrRegs,
                                                   ramId,
                                                   &eccErrorStatusWrap);
            /* Get the total number of interrupts pending */
            if (errorSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
            {
                bitErrCnt = eccErrorStatusWrap.singleBitErrorCount;
            }
            else
            {
                bitErrCnt = eccErrorStatusWrap.doubleBitErrorCount;
            }

            /* Check If it matches self test set flag.
             */
            selfTestErrorSrc = SDL_ECC_getDetectErrorSource(SDL_ECC_instance[eccMemType].eccSelfTestErrorType);

            if ((errorSrc == selfTestErrorSrc)
                && (ramId == SDL_ECC_instance[eccMemType].eccSelfTestRamId))
            {
                /* Clear the event(s) */
                sdlResult = SDL_ecc_aggrClrEccRamNIntrPending(eccAggrRegs,
                                                              ramId,
                                                              errorSrc,
                                                              bitErrCnt);

                if (sdlResult == SDL_PASS)
                {
                    handledResult = ((int32_t)1);
                    SDL_ECC_instance[eccMemType].eccErrorFlag = SDL_ECC_ERROR_FLAG_TRIGGERED;
                }
            }
        } /* if((sdlResult == SDL_PASS)  && (eventFound1)) { */
    } /* RAM ID for loop */

    if (handledResult == (int32_t)1U)
    {
        /* Ack Ecc aggregator error */
        sdlResult = SDL_ecc_aggrAckIntr(eccAggrRegs, errorSrc);
        if (sdlResult == SDL_PASS)
        {
            handledResult = ((int32_t)1U);
        }
    }
    return handledResult;
}

/** ============================================================================
 *
 * \brief   Handle any event that needs to be handled locally before
 *          reporting to application
 *
 * \param1  pInstance: Pointer to ESM instance
 * \param2  intSrc: Source interrupt number
 *
 * \return  true: if event handled; false if not handled
 */
static bool SDL_ECC_getIntSrcErrInfo(SDL_ESM_Inst instance, uint32_t intSrc, uint32_t *errorSrc, uint32_t *errorAddr, uint32_t *eccMemType)
{
    bool handledFlag = ((bool)false);
    uint32_t i = 0;

    for (i = ((uint32_t)0); i < (uint32_t)SDL_ECC_MEMTYPE_MAX; i++)
    {
        if (SDL_ECC_aggrTable[i].esmInst == instance)
        {
            if (SDL_ECC_aggrTable[i].esmIntSEC == intSrc)
            {
                *errorSrc = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;
                *eccMemType = i;
                handledFlag = ((bool)true);
            }
            else if (SDL_ECC_aggrTable[i].esmIntDED == intSrc)
            {
                *errorSrc = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
                *eccMemType = i;
                handledFlag = ((bool)true);
            }
            else
            {
                ; /* For Misra Compliance */
            }
            if (handledFlag == (bool)true)
            {
                break;
            }
        }
    }

    return handledFlag;
}

/** ============================================================================
 *
 * \brief   Get the ECC error information from the ESM error information
 *
 * \param1  instance: ESM instance
 * \param2  intSrc: Source interrupt number
 * \param3  eccMemType: Pointer to the ecc mem type
 * \param4  intrSrcType: Pointer to the interrupt source type
 *
 * \return SDL_PASS : Success; SDL_EFAIL for failures
 */
int32_t SDL_ECC_getESMErrorInfo(SDL_ESM_Inst instance, uint32_t intSrc,
                                SDL_ECC_MemType *eccMemType, SDL_Ecc_AggrIntrSrc *intrSrcType)
{
    int32_t retVal = SDL_PASS;
    bool status;
    uint32_t errorAddr;

    if ((eccMemType == NULL) || (intrSrcType == NULL))
    {
        retVal = SDL_EFAIL;
    }
    else
    {
        status = SDL_ECC_getIntSrcErrInfo(instance, intSrc, intrSrcType, &errorAddr, eccMemType);
        if (status != (bool)true)
        {
            retVal = SDL_EFAIL;
        }
    }

    return retVal;
}
/** ============================================================================*
 *
 * \brief   Ecc call back function registered with Main Domain ESM handler
 *
 * \param1  errorSrc: Error source
 *
 * \return  None
 */
static int32_t SDL_ECC_ESMCallBackFunction_MAIN (SDL_ESM_Inst instance, SDL_ESM_IntType intrType,
                                             uint32_t grpChannel, uint32_t index, uint32_t intSrc,
                                             void *arg)
{
    bool status;
    int32_t infoStatus;
    uint32_t errorAddr;
    SDL_ECC_MemType eccMemType;
    SDL_Ecc_AggrIntrSrc intrSrcType;
    int32_t handledResult = 0;

    status = SDL_ECC_getIntSrcErrInfo(instance, intSrc, &intrSrcType, &errorAddr, &eccMemType);

    if (status == (bool)true)
    {
        infoStatus = SDL_ECC_getESMErrorInfo(instance, intSrc, &eccMemType, &intrSrcType);

        if (infoStatus == SDL_PASS)
        {
            /* Handle ECC Aggregator event */
            handledResult = SDL_ECC_handleEccAggrEvent(eccMemType, intrSrcType, errorAddr);
        }
    }

    return handledResult;
}

/** ============================================================================
 * \brief   Retrieves the error information for a specified ECC aggregator
 *          and interrupt source.
 *
 * \param   eccMemType ECC Mem Type
 * \param   intrSrc: interrupt source type
 * \param   pErrorInfo: pointer to the error information structure
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
/*
 * Design: PROC_SDL-1291,PROC_SDL-1292
 */
int32_t SDL_ECC_getErrorInfo(SDL_ECC_MemType eccMemType, SDL_Ecc_AggrIntrSrc intrSrc, SDL_ECC_ErrorInfo_t *pErrorInfo)
{
    int32_t retVal = SDL_PASS;
    SDL_ecc_aggrRegs *eccAggrRegs;
    SDL_Ecc_AggrEccRamErrorStatusInfo eccErrorStatusWrap;
    uint32_t i;
    uint32_t ramId=0U;
    uint32_t ramIdType=0U;
    int32_t sdlResult;
    bool eventFound1 = (bool)false;
    SDL_MemConfig_t memConfig;

    eccErrorStatusWrap.singleBitErrorCount = 0x0u;
    eccErrorStatusWrap.doubleBitErrorCount = 0x0u;
    eccErrorStatusWrap.eccRow = 0x0u;
    eccErrorStatusWrap.eccBit1 = 0x0u;

    if ((pErrorInfo == NULL) || (SDL_ECC_getAggrBaseAddr(eccMemType, &eccAggrRegs) != SDL_PASS))
    {
        retVal = SDL_EFAIL;
    }
    else
    {
        pErrorInfo->eccMemType = eccMemType;
        pErrorInfo->intrSrc = intrSrc;

        /* Check which Ram Id triggered the error */
        for (i = ((uint32_t)0U);
             i < SDL_ECC_instance[eccMemType].eccInitConfig.numRams;
             i++)
        {
            /* Get corresponding ram Id */
            pErrorInfo->memSubType = SDL_ECC_instance[eccMemType].eccInitConfig.pMemSubTypeList[i];
            retVal = SDL_ECC_getRamId(eccMemType, pErrorInfo->memSubType, &ramId, &ramIdType);
            if (retVal != SDL_PASS)
            {
                continue;
            }

            /* Check if this event is triggered, by reading the ECC aggregator status
             * register */
            sdlResult = SDL_ecc_aggrIsEccRamIntrPending(eccAggrRegs,
                                                        ramId,
                                                        intrSrc,
                                                        &eventFound1);


            if((sdlResult == SDL_PASS)  && (eventFound1))
            {
                /* Read the locations of the bit errors */
                /* Get the error status information for Wrapper type */
                (void)SDL_ecc_aggrGetEccRamErrorStatus(eccAggrRegs,
                                                   ramId,
                                                   &eccErrorStatusWrap);
                /* Get the total number of interrupts pending */
                if (intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
                {
                    pErrorInfo->injectBitErrCnt = ((uint32_t)0U);
                    pErrorInfo->bitErrCnt = eccErrorStatusWrap.singleBitErrorCount;
                }
                else
                {
                    pErrorInfo->injectBitErrCnt = 0U;
                    pErrorInfo->bitErrCnt = eccErrorStatusWrap.doubleBitErrorCount;
                }


                if ((uint32_t)ramIdType == (uint32_t)SDL_ECC_RAM_ID_TYPE_WRAPPER)
                {
                    retVal = SDL_ECC_getMemConfig(eccMemType, ramId, &memConfig);
                    if (retVal == SDL_PASS)
                    {
                        if (intrSrc == SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT)
                        {
                            pErrorInfo->bitErrorOffset = (((uint64_t)eccErrorStatusWrap.eccRow * (uint64_t)memConfig.rowSize) +
                                                             ((uint64_t)eccErrorStatusWrap.eccBit1));
                        }
                        else
                        {
                            /* In case of DED error, eccBit1 is not valid, so calculate
                             * the bit offset of the start of the row with the DED error */
                            pErrorInfo->bitErrorOffset = (uint64_t)eccErrorStatusWrap.eccRow * (uint64_t)memConfig.rowSize;
                        }
                    }
                    /* Bit error group not used for Wrapper type */
                    pErrorInfo->bitErrorGroup  = ((uint32_t)0U);
                }
                break;
            }
        }
    }

    if (eventFound1 == (bool)false)
    {
        retVal = SDL_EFAIL;
    }

    return retVal;
}

/** ============================================================================
 * \brief   Acknowledges the ECC interrupt
 *
 * \param   eccMemType ECC Mem Type
 * \param   intrSrc: interrupt source type
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
int32_t SDL_ECC_ackIntr(SDL_ECC_MemType eccMemType, SDL_Ecc_AggrIntrSrc errorSrc)
{
    int32_t retVal = SDL_PASS;
    SDL_ecc_aggrRegs *eccAggrRegs;

    (void)SDL_ECC_getAggrBaseAddr(eccMemType, &eccAggrRegs);

    retVal = SDL_ecc_aggrAckIntr(eccAggrRegs, errorSrc);

    return retVal;
}

/** ============================================================================
 * \brief   Clears the pending interrupt
 *
 * \param   eccMemType ECC Mem Type
 * \param   memSubType: ECC mem sub-type
 * \param   intrSrc: interrupt source type
 * \param   subType: EDC sub type (if EDC type)
 * \param   numEvents: number of events to clear
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
/*
 * Design: PROC_SDL-1293,PROC_SDL-1294
 */
int32_t SDL_ECC_clearNIntrPending(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType,
                                  SDL_Ecc_AggrIntrSrc intrSrc,
                                  SDL_Ecc_AggrEDCErrorSubType subType, uint32_t numEvents)
{
    int32_t retVal = SDL_PASS;
    SDL_ecc_aggrRegs *eccAggrRegs;
    uint32_t ramId, ramIdType;

    (void)SDL_ECC_getAggrBaseAddr(eccMemType, &eccAggrRegs);

    /* Get the associated RAM ID */
    retVal = SDL_ECC_getRamId(eccMemType, memSubType, &ramId, &ramIdType);

    /* Clear the event(s) */
    (void)SDL_ecc_aggrClrEccRamNIntrPending(eccAggrRegs,
                                            ramId,
                                            intrSrc,
                                            numEvents);
    return retVal;
}

/** ============================================================================*
 *
 * \brief   Initializes an ESM module for usage with ECC module
 *
 * \param   esmInstType: Instance of ESM
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */

/**
 * Design: PROC_SDL-1282,PROC_SDL-1283
 */
int32_t SDL_ECC_initEsm (const SDL_ESM_Inst esmInstType)
{
    if (esmInstType == SDL_ESM_INST_MAIN_ESM0)
    {
        (void)SDL_ESM_registerECCCallback(esmInstType, eventBitMapMAIN,
                                          &SDL_ECC_ESMCallBackFunction_MAIN,
                                          NULL);

    }
    else
    { /* Nothing */

    }
    return SDL_PASS;
}

/** ============================================================================*
 *
 * \brief   Initializes ECC module for ECC detection
 *
 * \param1 eccAggrInstNumber Instance number of ECC aggregator
 * \param2 pECCInitConfig     Pointer to Ecc init configuration
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 *          NOTE: On failure the ECC peripheral registers may be initialized
 *          partially.
 */
 /**
 * Design: PROC_SDL-1284,PROC_SDL-1283
 */
int32_t SDL_ECC_init (SDL_ECC_MemType eccMemType,
                         const SDL_ECC_InitConfig_t *pECCInitConfig)
{
    SDL_ecc_aggrEnableCtrl     memParityCtrl;
    SDL_ecc_aggrRegs *eccAggrRegs = NULL;
    uint32_t         numMemRegions;
    int32_t retVal = SDL_PASS;
    uint32_t ramId, ramIdType = 0U;
    uint32_t i;
    int32_t sdlResult;
    SDL_ECC_MemSubType memSubType;
    uint32_t injectOnlyFlag;

    if (pECCInitConfig == NULL)
    {
        retVal = SDL_EBADARGS;
    }

    if (retVal == SDL_PASS)
    {
        retVal = SDL_ECC_mapEccAggrReg(eccMemType, &eccAggrRegs);
        eccAggrRegs = (SDL_ECC_aggrTransBaseAddressTable[eccMemType]);

        /* Disable all interrupts to start clean */
        (void)SDL_ecc_aggrDisableAllIntrs(eccAggrRegs);

        /* Get the number of RAMs */
        sdlResult = SDL_ecc_aggrGetNumRams(eccAggrRegs, &numMemRegions);
        if ((sdlResult != SDL_PASS) || (numMemRegions == (uint32_t)0U))
        {
            retVal = SDL_EFAIL;
        }
    }

    if (retVal == SDL_PASS)
    {
        /* Record the Init configuration */
        SDL_ECC_instance[eccMemType].eccInitConfig = *pECCInitConfig;

		/* Enable the parity ECC interrupts */
		memParityCtrl.validCfg               = SDL_ECC_AGGR_VALID_PARITY_ERR;
		memParityCtrl.intrEnableParityErr    = TRUE;
		retVal = SDL_ecc_aggrIntrEnableCtrl(eccAggrRegs,
								&memParityCtrl);

        /* Enable the single bit ECC interrupts */
        /* Note: The following statement enables interrupts for all RAMs */
        sdlResult = SDL_ecc_aggrEnableIntrs(eccAggrRegs,
                                SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT);
        if (sdlResult != SDL_PASS)
        {
            retVal = SDL_EFAIL;
        }
    }

    if (retVal == SDL_PASS)
    {
            /* Enable the Double bit ECC Interrupts */
            sdlResult = SDL_ecc_aggrEnableIntrs(eccAggrRegs,
                                SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT);
            if (sdlResult != SDL_PASS)
            {
                retVal = SDL_EFAIL;
            }
    }

    if (retVal == SDL_PASS)
    {
        /* Enable ECC */
        for ( i = ((uint32_t)0U); i < pECCInitConfig->numRams; i++)
        {

            /* Get memory Sub type to be configured */
            memSubType = pECCInitConfig->pMemSubTypeList[i];

            /* Get the corresponding ram Id */
            retVal = SDL_ECC_getRamId(eccMemType, memSubType, &ramId, &ramIdType);
            if (retVal == SDL_PASS)
            {

                /* Get the corresponding ram Id */
                retVal = SDL_ECC_getAggregatorType(eccMemType, memSubType, &injectOnlyFlag);
            }
#if defined (SUBSYS_MSS)
            if (retVal == SDL_PASS)
            {
                if (injectOnlyFlag == ((uint32_t)1U))
                {
                    /* Call SDL APIs to enable ECC, specific to the module */
                    retVal = SDL_ECC_configECCRam(ramId);
                    if (retVal != SDL_PASS)
                    {
                        retVal = SDL_EFAIL;
                    }
                }
                else
                {

                    /* Enables ECC, ecc checkmreg, rmw parity errors */
                    sdlResult = SDL_ecc_aggrConfigEccRam(eccAggrRegs,
                                            ramId, (bool)true, (bool)true, (bool)true);
                    if (sdlResult != SDL_PASS)
                    {
                        retVal = SDL_EFAIL;
                    }
                }
            }
#endif
            if (retVal != SDL_PASS)
            {
                break;
            }
        }

        /* Initialize object for self test */
        SDL_ECC_instance[eccMemType].eccErrorFlag = SDL_ECC_ERROR_FLAG_NONE;
        SDL_ECC_instance[eccMemType].eccSelfTestErrorType = SDL_INJECT_ECC_NO_ERROR;
        SDL_ECC_instance[eccMemType].eccSelfTestRamId = (SDL_ECC_INVALID_SELF_TEST_RAM_ID);
        SDL_ECC_instance[eccMemType].eccSelfTestAddr = NULL;
        SDL_ECC_instance[eccMemType].eccSelfTestcheckerType = SDL_ECC_INVALID_CHECKER_TYPE;
    }


    return retVal;
}

/** ============================================================================*
 *
 * \brief   Refresh memory to make sure ECC is generated
 *
 *
 * \param1  memAddr: Address to refresh ( Need to aligned to 32 bit)
 * \param2  size: Size of memory region to refresh
 *                Should be multiple of 4
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures

 */
static int32_t SDL_ECC_memoryRefresh(uint32_t *memAddr, size_t size)
{
    uint32_t i;
    int32_t result = SDL_PASS;
    volatile uint32_t tmpValue;

    /* Simply read and copy back data */
    for (i = ((uint32_t)0U); i < (size>>(uint32_t)2U); i++)
    {
        tmpValue = memAddr[i];
        memAddr[i] = tmpValue;
    }
    return result;
}

/** ============================================================================
 *
 * \brief   Initializes Memory to be ready for ECC error detection.
 *          Assumes ECC is already enabled.
 *
 * \param  eccMemType ECC memory type
 * \param  memSubType: Memory subtype
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */

/**
 * Design: PROC_SDL-1284,PROC_SDL-1285
 */
int32_t SDL_ECC_initMemory (SDL_ECC_MemType eccMemType,
                               SDL_ECC_MemSubType memSubType)
{
    int32_t result=SDL_PASS;
    SDL_MemConfig_t memConfig;
    uint32_t ramId, ramIdType;

    /* Get Ram Id to check if memSubType is of Wrapper RAM ID type */
    result = SDL_ECC_getRamId(eccMemType, memSubType, &ramId, &ramIdType);

    if ((result == SDL_PASS) && ((uint32_t)ramIdType == (uint32_t)SDL_ECC_RAM_ID_TYPE_WRAPPER))
    {
        /* Get memory configuration for memSubType of Wrapper RAM ID type */
        (void)SDL_ECC_getMemConfig(eccMemType, memSubType, &memConfig);

        /* Initialize only if readable */
        if (memConfig.readable)
        {
            /* Initialised the whole memory so that ECC is updated */
            result = SDL_ECC_memoryRefresh((uint32_t *)memConfig.memStartAddr,
                                       (size_t)memConfig.size);
        }
    }
    /* If RAM ID was found, but ramIdType is SDL_ECC_RAM_ID_TYPE_INTERCONNECT,
     * then no initialization is required and success is returned */

    return result;
}

/** ============================================================================
 *
 * \brief   Does access to trigger ECC
 *
 * \param  pMemoryAccessAddr: Memory Access Address
 *
 * @return  None

 */
static uint32_t SDL_ECC_triggerAccessForEcc(const uint32_t *pMemoryAccessAddr)
{
    volatile uint32_t testLocationValue;


   /* Read value to trigger ECC error injection */
   testLocationValue = *(pMemoryAccessAddr);
   return testLocationValue;
}

/** ============================================================================
 *
 * \brief   Runs self test by injecting and error and monitor response
 *          Assumes ECC is already enabled.
 *
 * \param1  eccMemType ECC memory type
 * \param2  memSubType: Memory subtype
 * \param3  errorType: ECC Self test type
 * \param4  pECCErrorConfig: Memory configuration self test area
 * \param4  selfTestTimeOut: Number of retries before time out
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */

/**
 * Design: PROC_SDL-1286,PROC_SDL-1287
 */
int32_t SDL_ECC_selfTest(SDL_ECC_MemType eccMemType,
                                  SDL_ECC_MemSubType memSubType,
                                  SDL_ECC_InjectErrorType errorType,
                                  const SDL_ECC_InjectErrorConfig_t *pECCErrorConfig,
                                  uint32_t selfTestTimeOut)
{
    int32_t retVal = SDL_PASS;
    uint32_t timeCount = 0U;
#if defined (SUBSYS_MSS)
    uint32_t retVal2 = 0U;
#endif
    uint32_t testLocationPreserve;
    uint32_t ramId, ramIdType;
    uint32_t *testLocationAddress;
    SDL_MemConfig_t memConfig;
    SDL_GrpChkConfig_t grpChkConfig;

    if (pECCErrorConfig == NULL)
    {
        retVal = SDL_EBADARGS;
    }

    memConfig.readable = (bool)false;
    grpChkConfig.grpChkType = SDL_ECC_INVALID_CHECKER_TYPE;

    if (retVal == SDL_PASS)
    {
        /* Configure error configuration based on Test type */
        /* Get Ram Id */
        retVal = SDL_ECC_getRamId(eccMemType, memSubType,
                                  &ramId, &ramIdType);
        if (retVal == SDL_PASS)
        {
            /* Get memory configuration only for Wrapper RAM ID's */
            retVal = SDL_ECC_getMemConfig(eccMemType, memSubType, &memConfig);

        }

        if (retVal == SDL_PASS)
        {
            /* Get actual location address for the memory */
            testLocationAddress = pECCErrorConfig->pErrMem;

            if (memConfig.readable)
            {
                /* Store test location value */
                testLocationPreserve = *(testLocationAddress);
            }

            /* Set self test error flag */
            SDL_ECC_instance[eccMemType].eccErrorFlag = SDL_ECC_ERROR_FLAG_STARTING;
            SDL_ECC_instance[eccMemType].eccSelfTestErrorType = errorType;
            SDL_ECC_instance[eccMemType].eccSelfTestRamId = ramId;
            SDL_ECC_instance[eccMemType].eccSelfTestAddr = testLocationAddress;
            SDL_ECC_instance[eccMemType].eccSelfTestcheckerType = grpChkConfig.grpChkType;
            /* Inject error */
            retVal = SDL_ECC_injectError(eccMemType,
                                         memSubType, errorType,
                                         pECCErrorConfig);
        }

        if (retVal == SDL_PASS)
        {
            if (memConfig.readable)
            {
                /* Trigger access for ECC error injection to complete */
               (void) SDL_ECC_triggerAccessForEcc(testLocationAddress);
            }
#if defined (SUBSYS_MSS)
            if ((eccMemType == SDL_R5FSS0_CORE0_ECC_AGGR) ||
               (eccMemType == SDL_R5FSS0_CORE1_ECC_AGGR)|| 
               (eccMemType == SDL_R5FSS1_CORE0_ECC_AGGR) ||
               (eccMemType == SDL_R5FSS1_CORE1_ECC_AGGR))
            {

                /* Wait for error to take effect */
                while((SDL_ECC_instance[eccMemType].eccErrorFlag != SDL_ECC_ERROR_FLAG_TRIGGERED)
                      && (timeCount < selfTestTimeOut))
                {
                    /* In cases there are no interrupts for the ECC event poll directly */
                    retVal2 = SDL_ECC_pollErrorEvent(eccMemType, memSubType, errorType);
                    if (retVal2 == SDL_ECC_EVENT_FOUND)
                    {
                        SDL_ECC_instance[eccMemType].eccErrorFlag = SDL_ECC_ERROR_FLAG_TRIGGERED;
                    }

                    /* Increment timeout counter */
                    timeCount++;
                }
            }
            else
            {
#endif
                /* Trigger access for ECC error injection to complete */
                (void) SDL_ECC_triggerAccessForEcc(testLocationAddress);
                /* Wait for error to take effect */
                while((SDL_ECC_instance[eccMemType].eccErrorFlag != SDL_ECC_ERROR_FLAG_TRIGGERED)
                      && (timeCount < selfTestTimeOut))
                {
                    /* Increment timeout counter */
                    timeCount++;
                }
#if defined (SUBSYS_MSS)
            }
#endif
            /* Check expected error occurred or timeout */
            if ((SDL_ECC_instance[eccMemType].eccErrorFlag != SDL_ECC_ERROR_FLAG_TRIGGERED))
            {
                retVal = SDL_EFAIL;
            }
            else
            {
                if (memConfig.readable)
                {
                    /* correct error injected if not autocorrect by hardware */
                    *(testLocationAddress) = testLocationPreserve;
                }
            }

            /* Reset self test error flag */
            SDL_ECC_instance[eccMemType].eccErrorFlag = SDL_ECC_ERROR_FLAG_NONE;
            SDL_ECC_instance[eccMemType].eccSelfTestErrorType = SDL_INJECT_ECC_NO_ERROR;
            SDL_ECC_instance[eccMemType].eccSelfTestRamId = (SDL_ECC_INVALID_SELF_TEST_RAM_ID);
            SDL_ECC_instance[eccMemType].eccSelfTestAddr = NULL;
            SDL_ECC_instance[eccMemType].eccSelfTestcheckerType = SDL_ECC_INVALID_CHECKER_TYPE;

        }

    }
    return retVal;
}

/** ============================================================================
 *
 * \brief   Get bit location in word.
 *
 * \param1  bitMask: Input bitmask
 * \param2  startBitLocation: Bit location to start from
 * \param3  pPbitLocation: Pointer to Next location of 1b found
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_getBitLocation(uint32_t bitMask,
                                         uint32_t startBitLocation,
                                         uint32_t *pPbitLocation)
{
    int32_t result = SDL_PASS;
    uint32_t bitCount;

    if (startBitLocation >= SDL_ECC_BITS_PER_WORD)
    {
        result = SDL_EFAIL;
    }
    else
    {
        /* Find first bit error for single bit */
        for (bitCount=startBitLocation; bitCount < SDL_ECC_BITS_PER_WORD; bitCount++)
        {
            if ((bitMask
                & (((uint32_t)1U) << bitCount)) != 0U  )
            {
                *pPbitLocation = bitCount;
                break;
            }
        }
        if ( bitCount >= SDL_ECC_BITS_PER_WORD)
        {
            result = SDL_EFAIL;
        }
    }
    return result;
}

/** ============================================================================
 *
 * \brief   Injects ECC error at specified location
 *          Assumes ECC is already enabled.
 *
 * \param1  memType: Memory type for self test
 * \param2  memSubType: Memory subtype
 * \param3  errorType: ECC error type
 * \param4  pECCErrorConfig: Pointer to Error configuration
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */

/**
 * Design: PROC_SDL-1288,PROC_SDL-1289
 */
int32_t SDL_ECC_injectError(SDL_ECC_MemType eccMemType,
                              SDL_ECC_MemSubType memSubType,
                              SDL_ECC_InjectErrorType errorType,
                              const SDL_ECC_InjectErrorConfig_t *pECCErrorConfig)
{
    uint32_t regValue;
    volatile uint32_t regValue2;
    uint32_t firstBitLocation, secondBitLocation;
    uint32_t errAddrOffset;
    SDL_ecc_aggrRegs *eccAggrRegs;
    uint32_t ramId, ramIdType = 0U;
    int32_t retVal = SDL_PASS;
    int32_t sdlRetval;
    SDL_MemConfig_t memConfig;

    if (pECCErrorConfig == NULL)
    {
        retVal = SDL_EBADARGS;
    }

    if (retVal == SDL_PASS)
    {
        /* Based on ECC MemType (i.e. which aggregator), find the appropriate base address
         * for that ECC Aggegator. */
        retVal = SDL_ECC_getAggrBaseAddr(eccMemType, &eccAggrRegs);
    }

    if (retVal == SDL_PASS)
    {
        /* Get Ram Id */
        retVal = SDL_ECC_getRamId(eccMemType, memSubType,
                                 &ramId, &ramIdType);
    }

    if (retVal == SDL_PASS)
    {
        /* Get memory configuration */
        retVal = SDL_ECC_getMemConfig(eccMemType, memSubType, &memConfig);

        if ((retVal == SDL_PASS) && (memConfig.readable == (bool)true))
        {
            if ( ((uintptr_t)pECCErrorConfig->pErrMem) < memConfig.memStartAddr)
            {
                retVal = SDL_EFAIL;
            }
            else
            {
                if((eccMemType == SDL_R5FSS0_CORE0_ECC_AGGR) || (eccMemType == SDL_R5FSS0_CORE1_ECC_AGGR)
                   || (eccMemType == SDL_R5FSS1_CORE0_ECC_AGGR)|| (eccMemType == SDL_R5FSS1_CORE1_ECC_AGGR))
                {
                    if((memSubType > 0x14U) && (memSubType < 0x17U))
                    {
                        /* Calculate error offset */
                        errAddrOffset =  ((uintptr_t)pECCErrorConfig->pErrMem - memConfig.memStartAddr)
                                        / ((memConfig.stride)*2U);
                    }
                    else if((memSubType > 0x16U) && (memSubType < 0x1BU))
                    {
                        /* Calculate error offset */
                        errAddrOffset =  ((uintptr_t)pECCErrorConfig->pErrMem - memConfig.memStartAddr)
                                        / ((memConfig.stride)*4U);
                    }
                    else
                    {
                        /* Calculate error offset */
                        errAddrOffset =  ((uintptr_t)pECCErrorConfig->pErrMem - memConfig.memStartAddr)
                                        / (memConfig.stride);
                    }
                }
                else
                {
                    /* Calculate error offset */
                    errAddrOffset =  ((uintptr_t)pECCErrorConfig->pErrMem - memConfig.memStartAddr)
                                    / ((memConfig.stride));
                }
            }

            if (retVal == SDL_PASS)
            {
                /* Set error Address in ECC Wrapper RAM ID */
                sdlRetval = SDL_ecc_aggrWriteEccRamErrCtrlReg(eccAggrRegs,
                                                              ramId, 0U,
                                                              errAddrOffset);
                if (sdlRetval != SDL_PASS)
                {
                    retVal = SDL_EFAIL;
                }
            }
        }

        if (retVal == SDL_PASS)
        {
            /* Read ECC Ram Control Register */
            sdlRetval = SDL_ecc_aggrReadEccRamCtrlReg(eccAggrRegs,
                                                     ramId, &regValue);
            if (sdlRetval != SDL_PASS)
            {
                retVal = SDL_EFAIL;
            }
        }

        if (retVal == SDL_PASS)
        {

            switch (errorType)
            {
                case SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE:
                case SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT:
                case SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE:
                case SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT:
                    /* Get bit location */
                    retVal = SDL_ECC_getBitLocation(pECCErrorConfig->flipBitMask,
                                                    0U, &firstBitLocation);
                    if (retVal != SDL_PASS)
                    {
                        break;
                    }
                    /* Write bit error configuration for single bit */
                    sdlRetval = SDL_ecc_aggrWriteEccRamErrCtrlReg(eccAggrRegs,
                                                    ramId, 1U, firstBitLocation);
                    if (sdlRetval != SDL_PASS)
                    {
                        retVal = SDL_EFAIL;
                        break;
                    }

                    if ((errorType == SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) ||
                        (errorType == SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE))
                    {
                        /* Configure settings for inject error once  */
                        regValue |= SDL_ECC_RAM_CTRL_ERROR_ONCE_MASK;
                    }
                    else
                    {
                        /* Configure settings for inject error repeat */
                        regValue = (regValue & (~SDL_ECC_RAM_CTRL_ERROR_ONCE_MASK));
                    }
                    if ((errorType == SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE) ||
                        (errorType == SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT))
                    {
                        /* Configure settings for single bit error, specific row */
                        regValue = (regValue
                                    & (~(SDL_ECC_RAM_CTRL_FORCE_N_ROW_MASK | SDL_ECC_RAM_CTRL_FORCE_DED_MASK)))
                                | SDL_ECC_RAM_CTRL_FORCE_SEC_MASK;
                    }
                    else
                    {
                        /* Configure settings for Single bit error N Row */
                        regValue = (regValue
                                    & (~SDL_ECC_RAM_CTRL_FORCE_DED_MASK))
                                | SDL_ECC_RAM_CTRL_FORCE_SEC_MASK
                                | SDL_ECC_RAM_CTRL_FORCE_N_ROW_MASK;
                    }
                    break;

                case SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE:
                case SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT:
                case SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE:
                case SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT:
                    /* Get bit location */
                    retVal = SDL_ECC_getBitLocation(pECCErrorConfig->flipBitMask,
                                            0U, &firstBitLocation);
                    if (retVal != SDL_PASS)
                    {
                        break;
                    }

                    /* Get Second bit location */
                    retVal = SDL_ECC_getBitLocation(pECCErrorConfig->flipBitMask,
                                                firstBitLocation+(uint32_t)1U, &secondBitLocation);
                    if (retVal != SDL_PASS)
                    {
                        break;
                    }

                    /* Record double bit error position */
                    regValue2 = firstBitLocation | (secondBitLocation << (uint32_t)16U);

                    /* Set bit error configuration settings to register */
                    sdlRetval = SDL_ecc_aggrWriteEccRamErrCtrlReg(eccAggrRegs,
                                            ramId, 1U, regValue2);
                    if (sdlRetval != SDL_PASS)
                    {
                        retVal = SDL_EFAIL;
                        break;
                    }
                    if ((errorType == SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE) ||
                            (errorType == SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE))
                    {
                        /* Configure settings for Double bit error */
                        regValue |= SDL_ECC_RAM_CTRL_ERROR_ONCE_MASK;
                    }
                    else
                    {
                        /* Configure settings for single bit error */
                        regValue = (regValue & (~SDL_ECC_RAM_CTRL_ERROR_ONCE_MASK));
                    }

                    if ((errorType == SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE) ||
                        (errorType == SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT))
                    {
                        /* Configure settings for double bit error, specific row */
                        regValue = (regValue
                                    & (~(SDL_ECC_RAM_CTRL_FORCE_N_ROW_MASK+SDL_ECC_RAM_CTRL_FORCE_SEC_MASK)))
                                | SDL_ECC_RAM_CTRL_FORCE_DED_MASK;
                    }
                    else
                    {
                        /* Configure settings for Double bit error N Row*/
                        regValue = (regValue
                                    & (~SDL_ECC_RAM_CTRL_FORCE_SEC_MASK))
                                | SDL_ECC_RAM_CTRL_FORCE_DED_MASK
                                | SDL_ECC_RAM_CTRL_FORCE_N_ROW_MASK;
                    }
                    break;

                default:
                    break;
            }
        }

        if (retVal == SDL_PASS)
        {
            /* Write bit error configuration to register */
            sdlRetval = SDL_ecc_aggrWriteEccRamCtrlReg(eccAggrRegs,
                                                       ramId, regValue);
            if (sdlRetval != SDL_PASS)
            {
                retVal = SDL_EFAIL;
            }
        }

        if (retVal == SDL_PASS)
        {
            /* Just read back ctrl register to confirm write */
            /* NOTE: The read value may not be same as what is written as some fields
             * in the register are not writable or can self clear
             */
            sdlRetval = SDL_ecc_aggrReadEccRamCtrlReg(eccAggrRegs,
                                                ramId,
                                                (uint32_t *)((uint32_t)&(regValue2)));
            if (sdlRetval != SDL_PASS)
            {
                retVal = SDL_EFAIL;
            }
        }
    } /* ramIdType == SDL_ECC_RAM_ID_TYPE_WRAPPER*/
    return retVal;
}

/**
 * Design: PROC_SDL-1290
 */
int32_t SDL_ECC_getStaticRegisters (SDL_ECC_MemType eccMemType, SDL_ECC_staticRegs *pStaticRegs)
{
    int32_t retVal = SDL_PASS;
    SDL_ecc_aggrRegs *eccAggrRegs;

    retVal = SDL_ECC_getAggrBaseAddr(eccMemType, &eccAggrRegs);

    if (pStaticRegs == NULL)
    {
        retVal = SDL_EBADARGS;
    }

    if (retVal == SDL_PASS)
    {
        retVal = SDL_ecc_aggrReadStaticRegs(eccAggrRegs, pStaticRegs);
    }

    return retVal;
}

/** ============================================================================
 *
 * \brief   Get Ram Id for given memory and memory subtype
 *
 * \param1  eccMemType: Memory type for self test
 * \param2  memSubType: Memory subtype for self test
 * \param3  pRAMId: pointer to return Ram Id
 * \param4  pRAMIdType: pointer to return Ram Id Type
 *
 * @return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_getRamId(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType,
    uint32_t *pRAMId, uint32_t *pRAMIdType)
{
    int32_t retVal = SDL_PASS;
    const SDL_RAMIdEntry_t *ramTable;

    retVal = SDL_ECC_checkMemoryType(eccMemType, memSubType);
    if (retVal == SDL_PASS)
    {
        ramTable = SDL_ECC_aggrTable[eccMemType].ramTable;
        *pRAMId = ramTable[memSubType].RAMId;
        *pRAMIdType = ramTable[memSubType].ramIdType;
    }

    return retVal;
}

/** ============================================================================
 *
 * \brief   Check ECC check valid memory and memory subtype
 *
 * \param1  eccMemType: Memory type for self test
 * \param2  memSubType: Memory subtype for self test
 *
 * @return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_checkMemoryType(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType)
{
    int32_t retVal = SDL_PASS;

    if (eccMemType < SDL_ECC_MEMTYPE_MAX)
    {
        if (memSubType >= SDL_ECC_aggrTable[eccMemType].numRams)
        {
            retVal = SDL_EFAIL;
        }
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return retVal;
}

/** ============================================================================
 *
 * \brief   Get ECC aggregator type for given memory and memory subtype
 *
 * \param1  eccMemType: Memory type for self test
 * \param2  memSubType: Memory subtype for self test
 * \param3  pInjectOnly: pointer to variable indicating ECC aggregator
 *                       inject only
 *
 * @return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_getAggregatorType(SDL_ECC_MemType eccMemType,
                                            SDL_ECC_MemSubType memSubType,
                                            uint32_t *pIinjectOnly)
{
    int32_t retVal = SDL_PASS;
    const SDL_RAMIdEntry_t *ramTable;

    retVal = SDL_ECC_checkMemoryType(eccMemType, memSubType);
    if (retVal == SDL_PASS)
    {
        ramTable = SDL_ECC_aggrTable[eccMemType].ramTable;
        *pIinjectOnly = ramTable[memSubType].aggregatorTypeInjectOnly;
    }

    return retVal;
}

/** ============================================================================
 *
 * \brief   Get ECC memory configuration for given memory subtype (only valid for
 *          Wrapper RAM ID's).
 *
 * \param1  memSubType: Memory subtype for which we are searching for memory
 *                      configuration
 * \param2  memEntryTable: Memory configuration table for all Wrapper RAM ID's
 *                         for a particular memType (i.e. ECC aggregator).
 * \param3  tableSize: Size of the memory configuration table to search

 * \param4  pMemConfig: pointer to memory configuration structure that will be
 *                      filled upon successful retrieval.
 *
 * @return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_searchMemEntryTable(SDL_ECC_MemSubType memSubType,
                                              const SDL_MemConfig_t memEntryTable[],
                                              uint32_t tableSize,
                                              SDL_MemConfig_t *pMemConfig)
{
    int32_t retVal = SDL_EFAIL;
    uint32_t length = tableSize;
    uint32_t first, last, middle;

    /* Binary search, as the assumption is that memSubTypes in table are
     * in order */
    first = ((uint32_t)0U);
    last = length - ((uint32_t)1U);
    middle = (first + last) / 2U;

    while (first <= last)
    {
        if (memEntryTable[middle].memSubType < memSubType)
        {
            first = middle + ((uint32_t)1U);
        }
        else if (memEntryTable[middle].memSubType == memSubType)
        {
            /* Fill the memory configuration structure */
            *pMemConfig = memEntryTable[middle];
            retVal = SDL_PASS;
            break;
        }
        else
        {
            last = middle - ((uint32_t)1U);
        }

        middle = (first + last) / 2U;
    }
    return retVal;
}

/** ============================================================================
 *
 * \brief   Get Memory configuration for given memory type  and memory subtype
 *
 * \param1  eccMemType: Memory type for self test
 * \param2  memSubType: Memory subtype for self test
 * \param3  pMemConfig: pointer to return memory configuration
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_getMemConfig(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType,
                              SDL_MemConfig_t *pMemConfig)
{
    int32_t retVal = SDL_PASS;
    uint32_t tableSize;
    const SDL_MemConfig_t *memEntryTable;

    retVal = SDL_ECC_checkMemoryType(eccMemType, memSubType);
    if (retVal == SDL_PASS)
    {
        memEntryTable = SDL_ECC_aggrTable[eccMemType].memConfigTable;
        tableSize = SDL_ECC_aggrTable[eccMemType].numMemEntries;
        retVal = SDL_ECC_searchMemEntryTable(memSubType,
                                             memEntryTable,
                                             tableSize,
                                             pMemConfig);
    }

   return retVal;
}
/** ============================================================================
 *
 * \brief   Get Ecc Aggregator Base Address for given memory type
 *
 * \param1  eccMemType: Memory type for self test
 * \param2  pEccAggr: pointer to Ecc Aggregator address
 *
 * @return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_getAggrBaseAddr(SDL_ECC_MemType eccMemType, SDL_ecc_aggrRegs **pEccAggr)
{
    int32_t retVal = SDL_PASS;

    if (eccMemType < SDL_ECC_MEMTYPE_MAX)
    {
      *pEccAggr = SDL_ECC_aggrTransBaseAddressTable[eccMemType];
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return retVal;
}

/** ============================================================================
 *
 * \brief   Injects ECC TCM Parity error
 *
 * \param1  eccMemType: Memory type for ECC AGGR
 * \param2  memSubType: Memory subtype
 * \param3  bitValue  : Bit Value to set particular register
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */

 /**
 * Design: PROC_SDL-5796
 */

int32_t SDL_ECC_tcmParity(SDL_ECC_MemType eccMemType,
							  SDL_ECC_MemSubType memSubType,
							  uint32_t bitValue)
{
	int32_t retValue= SDL_PASS;

	switch(eccMemType)
	{
		case SDL_R5SS0_CPU0_TCM:
		case SDL_R5SS1_CPU0_TCM:
			retValue = SDL_ECC_tcmparityerrForce_Regs(eccMemType, memSubType, bitValue);
			break;

		default :
			retValue= SDL_EFAIL;
			break;
	}

	return retValue;
}

/** ============================================================================
 *
 * \brief   Set ECC TCM Parity Error force Register
 *
 * \param1  eccMemType: Memory type for ECC AGGR
 * \param2  memSubType: Memory subtype
 * \param3  RegValue  : Value to set particular register
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
static int32_t SDL_ECC_tcmparityerrForce_Regs(SDL_ECC_MemType eccMemType,
							  SDL_ECC_MemSubType memSubType,
							  uint32_t RegValue)
{
	int32_t retVal = SDL_EFAIL;

	if(eccMemType == SDL_R5SS0_CPU0_TCM)
	{
		switch (memSubType)
		{
			case SDL_R5FSS0_CORE0_ATCM0:
			case SDL_R5FSS0_CORE0_B0TCM0:
			case SDL_R5FSS0_CORE0_B1TCM0:
			case SDL_R5FSS0_CORE1_ATCM1:
			case SDL_R5FSS0_CORE1_B0TCM1:
			case SDL_R5FSS0_CORE1_B1TCM1:
				/* Write to error force register the inject error to core 0*/
				SDL_REG32_WR(SDL_R5SS0_TCM_ADDRPARITY_ERRFORCE, RegValue);
				retVal = SDL_PASS;
				break;
			default :
				retVal = SDL_EFAIL;
				break;
		}
	}
	else if(eccMemType == SDL_R5SS1_CPU0_TCM)
	{
		switch (memSubType)
		{
			case SDL_R5FSS1_CORE0_ATCM0:
			case SDL_R5FSS1_CORE0_B0TCM0:
			case SDL_R5FSS1_CORE0_B1TCM0:
			case SDL_R5FSS1_CORE1_ATCM1:
			case SDL_R5FSS1_CORE1_B0TCM1:
			case SDL_R5FSS1_CORE1_B1TCM1:
				/* Write to error force register the inject error core 1*/
				SDL_REG32_WR(SDL_R5SS1_TCM_ADDRPARITY_ERRFORCE, RegValue);
				retVal = SDL_PASS;
				break;
			default :
				retVal = SDL_EFAIL;
				break;
		}
	}
	else
	{
		retVal = SDL_EFAIL;
	}

	return 	retVal;

}

int32_t SDL_cleartcmStatusRegs(uint32_t clearVal)
{
	/*clearing status and status raw register for R5FSS0_0 */
	SDL_REG32_WR(SDL_R5FSS0_CORE0_TCM_ERR_STATUS,clearVal);
	SDL_REG32_WR(SDL_R5FSS0_CORE0_TCM_ERR_STATUS_RAW,clearVal);
	/*clearing status and status raw register for R5FSS0_1 */
	SDL_REG32_WR(SDL_R5FSS0_CORE1_TCM_ERR_STATUS,clearVal);
	SDL_REG32_WR(SDL_R5FSS0_CORE1_TCM_ERR_STATUS_RAW,clearVal);
	/*clearing status and status raw register for R5FSS1_0 */
	SDL_REG32_WR(SDL_R5FSS1_CORE0_TCM_ERR_STATUS,clearVal);
	SDL_REG32_WR(SDL_R5FSS1_CORE0_TCM_ERR_STATUS_RAW,clearVal);
	/*clearing status and status raw register for R5FSS1_1 */
	SDL_REG32_WR(SDL_R5FSS1_CORE1_TCM_ERR_STATUS,clearVal);
	SDL_REG32_WR(SDL_R5FSS1_CORE1_TCM_ERR_STATUS_RAW,clearVal);

	return 0;
}

/**
* Design: PROC_SDL-5796
*/

int32_t SDL_ECC_tpccParity(SDL_ECC_MemType eccMemType,
							  uint32_t bitValue,
							  uint32_t paramregvalue,
							  uint32_t regval)
{
	int32_t result=0;

	switch(eccMemType)
	{
		/* EDMA Parity */
		case SDL_TPCC0:
			result = SDL_ECC_enableParityerr(bitValue, paramregvalue, regval);
			break;
		default :
			result=0;
			break;
	}

	return 	result;

}

static int32_t SDL_ECC_enableParityerr(uint32_t setmask, uint32_t paramregs, uint32_t paramval)
{
	uint32_t result=0u;
	uint32_t disabletestmode = 0x01u;

	/* Enable test mode */
	SDL_REG32_WR(SDL_R5FSS0_CORE0_TPCC0_PARITY_CTRL,setmask);
	/* Writeinto param register for data integrity */
	SDL_REG32_WR(paramregs, paramval);
	result = SDL_REG32_RD(paramregs);
	/* Disable test mode */
	SDL_REG32_WR(SDL_R5FSS0_CORE0_TPCC0_PARITY_CTRL,(setmask & disabletestmode));

	return ((int32_t)result);

}

/** ============================================================================
 *
 * \brief   Enables TMU Parity error
 *
 */

 /**
 * Design: PROC_SDL-7374
 */

void SDL_ECC_enableTMUROMParity(void)
{
    uint32_t tmuParityCtrl;

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_EN;
    /* Write to TMU0_ROM_PARITY_EN  in
       TMU_R5SS0_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_EN;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_EN;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_EN;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);
}

/** ============================================================================
 *
 * \brief   Enables TMU Parity error Force Error
 *
 */

 /**
 * Design: PROC_SDL-7451
 */

void SDL_ECC_enableTMUROMParityForceError(void)
{
    uint32_t tmuParityCtrl;

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_FORCE_ERR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_FORCE_ERR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_FORCE_ERR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl |= SDL_TMU0_ROM_PARITY_FORCE_ERR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);
}

/** ============================================================================
 *
 * \brief   Disables TMU Parity error
 *
 */

 /**
 * Design: PROC_SDL-7375
 */

void SDL_ECC_disableTMUROMParity(void)
{
    uint32_t tmuParityCtrl;

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFEU;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFEU;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFEU;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFEU;
    /* Write 0 to TMU0_ROM_PARITY_EN in
       SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);
}

/** ============================================================================
 *
 * \brief   Disables TMU Parity Force Error
 *
 */

 /**
 * Design: PROC_SDL-7452
 */

void SDL_ECC_disableTMUROMParityErrorForce(void)
{
    uint32_t tmuParityCtrl;

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFDU;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFDU;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFDU;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl &= 0xFFFFFFFDU;
    /* Write 0 to SDL_TMU0_ROM_PARITY_FORCE_ERR in
       SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);
}

/** ============================================================================
 *
 * \brief   Clear TMU Parity error
 *
 */

 /**
 * Design: PROC_SDL-7453
 */

void SDL_ECC_clearTMUROMParityError(void)
{
    uint32_t tmuParityCtrl;

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl &= SDL_TMU0_ROM_PARITY_ERR_CLR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_ERR_CLR in
       SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl &= SDL_TMU0_ROM_PARITY_ERR_CLR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_ERR_CLR in
       SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL);
    tmuParityCtrl &= SDL_TMU0_ROM_PARITY_ERR_CLR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_ERR_CLR in
       SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE0_ROM_PARITY_CTRL, tmuParityCtrl);

    tmuParityCtrl = SDL_REG32_RD(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL);
    tmuParityCtrl &= SDL_TMU0_ROM_PARITY_ERR_CLR;
    /* Write 0 to SDL_TMU0_ROM_PARITY_ERR_CLR in
       SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL register */
    SDL_REG32_WR(SDL_TMU_R5SS1_CORE1_ROM_PARITY_CTRL, tmuParityCtrl);
}
