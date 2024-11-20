/*
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
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/r5/v0/sdl_mcu_armss_ccmr5.h>
#include <sdl/r5/v0/sdl_ip_ccm.h>
#include <sdl/sdl_ecc.h>
#include <sdl/r5/v0/am263px/sdl_soc_ccm.h>

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
volatile bool Erroresm = false;
/* ========================================================================== */
/*                             Macros                                 */
/* ========================================================================== */

/** ---------------------------------------------------------------------------
 * \Macro SDL_CCM
 * \brief Defines the values for CCM test error flag used to track self test
 * ----------------------------------------------------------------------------
 */

#define SDL_CCM_ALL_STATUS_BITS (SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1_STE1_MASK \
                                            | SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1_STET1_MASK \
                                            | SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1_STC1_MASK \
                                            | SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1_CMPE1_MASK)

#define        SDL_ESM_CCM_0_SELF_TEST_ERR_INT   SDL_ESM0_CCM_0_SELFTEST_ERR
   /**< R5F0 CCM Interrupt source Self test error */
#define        SDL_ESM_CCM_0_LOCKSTEP_COMPARE   SDL_ESM0_CCM_0_LOCKSTEP_COMPARE_ERR
   /**< R5F0 CCM Interrupt lockstep error */
#define        SDL_ESM_CCM_1_SELF_TEST_ERR_INT   SDL_ESM0_CCM_1_SELFTEST_ERR
   /**< R5F1 CCM Interrupt source Self test error */
#define        SDL_ESM_CCM_1_LOCKSTEP_COMPARE   SDL_ESM0_CCM_1_LOCKSTEP_COMPARE_ERR
   /**< R5F1 CCM Interrupt source Self test error */
#define        SDL_ESM_R5F0_VIM_COMPARE_ERR_INT   SDL_ESM0_R5FSS0_R5FSS0_VIM_COMPARE_ERR_PULSE_0
   /**< R5F0 VIM Interrupt source Self test error */
#define        SDL_ESM_R5F1_VIM_COMPARE_ERR_INT   SDL_ESM0_R5FSS1_R5FSS1_VIM_COMPARE_ERR_PULSE_0
   /**< R5F1 VIM Interrupt source Self test error */
#define        SDL_ESM_R5F0_TMU_COMPARE_ERR_INT   SDL_ESM0_R5FSS0_TMU_COMPARE_ERR
   /**< R5F0 TMU Interrupt lockstep error */
#define        SDL_ESM_R5F1_TMU_COMPARE_ERR_INT   SDL_ESM0_R5FSS1_TMU_COMPARE_ERR
   /**< R5F1 TMU Interrupt lockstep error */
#define        SDL_ESM_R5F0_RL2_COMPARE_ERR_INT   SDL_ESM0_R5FSS0_RL2_COMPARE_ERR
   /**< R5F0 RL2 Interrupt lockstep error */
#define        SDL_ESM_R5F1_RL2_COMPARE_ERR_INT   SDL_ESM0_R5FSS1_RL2_COMPARE_ERR
   /**< R5F1 RL2 Interrupt lockstep error */

#define SDL_INTR_PRIORITY_LVL             1U
#define SDL_ENABLE_ERR_PIN                1U
#define SDL_ESM_MAX_EVENT_MAP_WORDS       32U

#define SDL_MCU_ARMSS_VIM_NULL_ADDR       ((void *) 0 )
/** ---------------------------------------------------------------------------
 * \enum SDL_CCM_ErrorFlag
 * \brief Defines the values for CCM test error flag used to track self test
 * ----------------------------------------------------------------------------
 */
typedef enum {
    SDL_CCM_ERROR_FLAG_NONE=0,
    /**< Error flag no action */
    SDL_CCM_ERROR_FLAG_INPROGRESS=1,
    /**< Error flag to indicate self test in progress */
    SDL_CCM_ERROR_FLAG_TRIGGERED=2,
    /**< Error flag to indicate error triggerred */
} SDL_CCM_ErrorFlag;


/** ---------------------------------------------------------------------------
 * \brief This structure defines the elements of CCM software instance
 * ----------------------------------------------------------------------------
 */
/* Design: PROC_SDL-2118  */
typedef struct SDL_CCM_instance_s
{
   volatile SDL_CCM_ErrorFlag selfTestErrorFlag;
   /**< Flag to track self test */
}  SDL_CCM_Instance_t;

/* SDL CCM Instance */
static SDL_CCM_Instance_t SDL_CCM_instance;

uint32_t SDL_CCM_eventBitMap_param[SDL_ESM_MAX_EVENT_MAP_WORDS] =
{
     0x00000000u, 0x00000000u, 0x00780880u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
     0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
};
/* ========================================================================== */
/*                    Local functions declaration                             */
/* ========================================================================== */

static int32_t SDL_CCM_CheckSelfTestErrorSource(SDL_CCM_MonitorType *monitorType);

static int32_t SDL_CCM_getMonitorKeyRegister(SDL_CCM_MonitorType monitorType,
                                                SDL_McuArmssCcmR5RegId *pCCMR5RegId);

static int32_t SDL_CCM_getMonitorStatusRegister(SDL_CCM_MonitorType monitorType,
                                                   SDL_McuArmssCcmR5RegId *pCCMR5RegId);

static int32_t SDL_CCM_ESM_callBackFunction (SDL_ESM_Inst instance, SDL_ESM_IntType intrType,
                                             uint32_t grpChannel, uint32_t index, uint32_t intSrc,
                                             void *arg);
/* ========================================================================== */
/*                    Local functions definition                              */
/* ========================================================================== */

static int32_t SDL_CCM_CheckSelfTestErrorSource(SDL_CCM_MonitorType *monitorType)
{
    uint32_t statusValue;
    int32_t retVal = SDL_PASS;

    /* Read status register of CPU output compare block  */
    (void)SDL_armR5ReadCCMRegister (BASEADDRESS,
                                           SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID,
                                           &statusValue,
                                           NULL);

        /* If status bit set, return output compare block */
        if ((statusValue & SDL_CCM_ALL_STATUS_BITS) != 0u) {
            *monitorType =  SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK;
        } else {

            /* Read status register of VIM compare block  */
            (void)SDL_armR5ReadCCMRegister (BASEADDRESS,
                                                   SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID,
                                                   &statusValue,
                                                   NULL);
            /* If status bit set, return VIM monitor block */
            if ((statusValue & SDL_CCM_ALL_STATUS_BITS) != 0u) {
                 *monitorType =  SDL_CCM_MONITOR_TYPE_VIM;
             } else {

                 /* Read status register of Inactivity monitor block  */
                 (void)SDL_armR5ReadCCMRegister (BASEADDRESS,
                                                        SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID,
                                                        &statusValue,
                                                        NULL);

                 /* If status bit set, return Inactivity monitor block */
                 if ((statusValue & SDL_CCM_ALL_STATUS_BITS) != 0u) {
                      *monitorType =  SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR;
                  } else {
                      retVal = SDL_EFAIL;
                  }
             }
        }

    return retVal;
}

static int32_t SDL_CCM_getMonitorKeyRegister(SDL_CCM_MonitorType monitorType,
                                                SDL_McuArmssCcmR5RegId *pCCMR5RegId)
{
    int32_t result = SDL_PASS;

        /* Set default value */
        *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID;

        /* This gets the key register based on monitorType*/
        switch(monitorType) {
            case SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_VIM:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_TMU:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMKEYR5_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_RL2:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMKEYR6_REGID;
                break;

            default:
                /* Invalid argument */
                result = SDL_EBADARGS;
                break;
        }
    return result;
}

static int32_t SDL_CCM_getMonitorStatusRegister(SDL_CCM_MonitorType monitorType,
                                                   SDL_McuArmssCcmR5RegId *pCCMR5RegId)
{
    int32_t result = SDL_PASS;

        /* Get the monitor type register based on monitor type */
        switch(monitorType) {
            case SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_VIM:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_TMU:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMSR5_REGID;
                break;

            case SDL_CCM_MONITOR_TYPE_RL2:
                *pCCMR5RegId = SDL_MCU_ARMSS_CCMR5_CCMSR6_REGID;
                break;

            default:
                /* Invalid argument */
                result = SDL_EBADARGS;
                break;
        }
    return result;
}

static int32_t SDL_CCM_getMonitorTypeFromIntSrc (uint32_t intSrc,
                                          SDL_CCM_MonitorType *pMonitorType)
{
    int32_t result = SDL_PASS;

    /* Set default value */
    *pMonitorType = SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK;

    /* This finds the monitor type based on the interrupt source */
    switch(intSrc) {
        case SDL_ESM0_R5FSS0_R5FSS0_COMPARE_ERR_PULSE_0:
            *pMonitorType = SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK;
            break;

        case SDL_ESM0_R5FSS0_R5FSS0_VIM_COMPARE_ERR_PULSE_0:
            *pMonitorType = SDL_CCM_MONITOR_TYPE_VIM;
            break;

        case SDL_ESM0_R5FSS0_R5FSS0_BUS_MONITOR_ERR_PULSE_0:
            *pMonitorType = SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR;
            break;

        case SDL_ESM0_R5FSS0_TMU_COMPARE_ERR:
            *pMonitorType = SDL_CCM_MONITOR_TYPE_TMU;
            break;

        case SDL_ESM0_R5FSS0_RL2_COMPARE_ERR:
            *pMonitorType = SDL_CCM_MONITOR_TYPE_RL2;
            break;
        default:
            /* Invalid value return fail */
            result = SDL_EFAIL;
            break;
    }
    return result;
}

static int32_t SDL_CCM_ESM_callBackFunction (SDL_ESM_Inst instance, SDL_ESM_IntType intrType,
                                             uint32_t grpChannel, uint32_t index, uint32_t intSrc,
                                             void *arg)
{
    uint32_t status;
    uint32_t polarityRegValue, keyRegValue;
    SDL_McuArmssCcmR5RegId monitorTypeStatusRegister = SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID;
    SDL_McuArmssCcmR5RegId monitorTypeKeyRegister;
    SDL_CCM_MonitorType monitorType;
    int32_t retVal = SDL_EFAIL;
    bool moduleIndependentEvent = (bool)false;

    /* Check if it is self test related interrupt */
    switch (intSrc)
    {
        case SDL_ESM_CCM_0_SELF_TEST_ERR_INT:
        case SDL_ESM_CCM_0_LOCKSTEP_COMPARE:
        case SDL_ESM_CCM_1_SELF_TEST_ERR_INT:
        case SDL_ESM_CCM_1_LOCKSTEP_COMPARE:
        case SDL_ESM_R5F0_VIM_COMPARE_ERR_INT:
        case SDL_ESM_R5F1_VIM_COMPARE_ERR_INT:
        case SDL_ESM_R5F0_TMU_COMPARE_ERR_INT:
        case SDL_ESM_R5F1_TMU_COMPARE_ERR_INT:
        case SDL_ESM_R5F0_RL2_COMPARE_ERR_INT:
        case SDL_ESM_R5F1_RL2_COMPARE_ERR_INT:
            /* These events can come for any of the CCM block. Read status register to see which self test */
            retVal = SDL_CCM_CheckSelfTestErrorSource(&monitorType);
            if ( retVal != SDL_PASS)
            {
                moduleIndependentEvent = (bool)true;
            }
            break;

        default:
            /* Get the monitor type based on interrupt source */
            (void)SDL_CCM_getMonitorTypeFromIntSrc(intSrc, &monitorType);
            break;
    }

    /* Read polarity convert register */
    /* Read status register 1 */
	retVal = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[0],
                                        SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID,
                                        &polarityRegValue,
                                        NULL);
	if(retVal == SDL_PASS)
	{
	    if(polarityRegValue != (uint32_t)0U)
        {
	        /* If polarity reverted; switch back to 0 */
					(void)SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[0],
	                                                   SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID,
	                                                   0u,
	                                                       NULL);
	        if (SDL_CCM_instance.selfTestErrorFlag == SDL_CCM_ERROR_FLAG_INPROGRESS )
            {
	            SDL_CCM_instance.selfTestErrorFlag = SDL_CCM_ERROR_FLAG_TRIGGERED;
	        }
	    }
	    if (moduleIndependentEvent)
        {
	        if (SDL_CCM_instance.selfTestErrorFlag == SDL_CCM_ERROR_FLAG_INPROGRESS )
            {
	            SDL_CCM_instance.selfTestErrorFlag = SDL_CCM_ERROR_FLAG_TRIGGERED;
	        }
	    }
        else
        {
	        /* Get the status register and monitor type for the interrupt source */
	        (void)SDL_CCM_getMonitorStatusRegister(monitorType, &monitorTypeStatusRegister);
	    }
	}

    if (retVal == SDL_PASS)
    {
        /* Read status register 1 */
        (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[0],
                                               monitorTypeStatusRegister,
                                               &status,
                                               NULL);

        /* Get the Key register corresponding to the monitor type */
        retVal = SDL_CCM_getMonitorKeyRegister(monitorType, &monitorTypeKeyRegister);
    }

    if (retVal == SDL_PASS)
    {
        /* Read polarity convert register */
         /* Read status register 1 */
         retVal = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[0],
                                                monitorTypeKeyRegister,
                                                &keyRegValue,
                                                NULL);
    }

    if (retVal == SDL_PASS)
    {
        if (keyRegValue == (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE)
        {
            /* Switch it back to active mode */
            (void)SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[0],
                                                       monitorTypeKeyRegister,
                                                       ((uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE),
                                                       NULL);
        }
            /* Clear status register */
        retVal = SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[0],
                                                       monitorTypeStatusRegister,
                                                       status,
                                                    NULL);
    }
    return retVal;
}

/* SDL functions */

/**
 *   Design: PROC_SDL-2047
 */
int32_t SDL_CCM_init(SDL_CCM_Inst instance, uint32_t index)
{
    int32_t retVal = SDL_PASS;

    if(instance >= SDL_CCM_MAX_INSTANCE)
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        /* Enable Cpu output compare block active */
        retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                   SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID,
                                                   (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE,
                                                   NULL);

        if (retVal == SDL_PASS)
        {

            /* Enable Vim compare active */
            retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID,
                                                       (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE,
                                                       NULL);
        }

        if (retVal == SDL_PASS)
        {
            /* Enable Inactivity monitor compare block active */
            retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID,
                                                       (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE,
                                                       NULL);
        }

        if (retVal == SDL_PASS)
        {
            /* Enable TMU compare block active */
            retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMKEYR5_REGID,
                                                       (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE,
                                                       NULL);
        }

        if (retVal == SDL_PASS)
        {
            /* Enable RL2 compare block active */
            retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMKEYR6_REGID,
                                                       (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE,
                                                       NULL);
        }

        if (retVal == SDL_PASS)
        {

            /* Clear any pending errors */

            retVal = SDL_CCM_clearError(instance, SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK);
            if(retVal == SDL_PASS)
            {
                retVal = SDL_CCM_clearError(instance, SDL_CCM_MONITOR_TYPE_VIM);
            }
            if(retVal == SDL_PASS)
            {
                retVal = SDL_CCM_clearError(instance, SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR);
            }
            if(retVal == SDL_PASS)
            {
                retVal = SDL_CCM_clearError(instance, SDL_CCM_MONITOR_TYPE_TMU);
            }
            if(retVal == SDL_PASS)
            {
                retVal = SDL_CCM_clearError(instance, SDL_CCM_MONITOR_TYPE_RL2);
            }

            if (retVal == SDL_PASS)
            {
                /* Register error interrupt call back function with MCU ESM handler */
                retVal = SDL_ESM_registerCCMCallback(ESM_INSTANCE, SDL_CCM_eventBitMap_param,
                                              &SDL_CCM_ESM_callBackFunction,
                                              NULL);
            }
        }
        /* Clear self test flag */
        SDL_CCM_instance.selfTestErrorFlag = SDL_CCM_ERROR_FLAG_NONE;
    }
    return retVal;
}

/**
 *   Design: PROC_SDL-2058
 */
int32_t SDL_CCM_verifyConfig(SDL_CCM_Inst instance)
{
    uint32_t keyValue;
    int32_t sdlResult, retVal = SDL_PASS;

    if(instance >= SDL_CCM_MAX_INSTANCE)
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        /* Read back Key register */
        sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                              SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID,
                                              (uint32_t *)&keyValue, NULL);
        if ((sdlResult != SDL_PASS) || (keyValue != (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE))
        {
                retVal = SDL_EFAIL;
        }

        if (retVal == SDL_PASS)
        {
            /* Read back Key register */
            sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                  SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID,
                                                  (uint32_t *)&keyValue,
                                                  NULL);
            if ((sdlResult != SDL_PASS) || (keyValue != (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE))
            {
                    retVal = SDL_EFAIL;
            }
        }

        if (retVal == SDL_PASS)
        {
            /* Read back Key register */
            sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                  SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID,
                                                  (uint32_t *)&keyValue,
                                                  NULL);
            if ((sdlResult != SDL_PASS) || (keyValue != (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE))
            {
                    retVal = SDL_EFAIL;
            }
        }

        if (retVal == SDL_PASS)
        {
            /* Read back Key register */
            sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                  SDL_MCU_ARMSS_CCMR5_CCMKEYR5_REGID,
                                                  (uint32_t *)&keyValue,
                                                  NULL);
            if ((sdlResult != SDL_PASS) || (keyValue != (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE))
            {
                    retVal = SDL_EFAIL;
            }
        }

        if (retVal == SDL_PASS)
        {
            /* Read back Key register */
            sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                  SDL_MCU_ARMSS_CCMR5_CCMKEYR6_REGID,
                                                  (uint32_t *)&keyValue,
                                                  NULL);
            if ((sdlResult != SDL_PASS) || (keyValue != (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE))
            {
                    retVal = SDL_EFAIL;
            }
        }
    }
    return retVal;
}

/**
 *   Design: PROC_SDL-2050
 */
int32_t SDL_CCM_getStaticRegisters(SDL_CCM_Inst instance, SDL_CCM_staticRegs *pStaticRegs)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t readVal;
    int32_t  result;

    if((instance >= SDL_CCM_MAX_INSTANCE) || (pStaticRegs == NULL))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                        SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID,
                                        &readVal, &result);
        if(result == SDL_PASS)
        {
            pStaticRegs->CCMKEYR1 = readVal;
            (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                            SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID,
                                            &readVal, &result);
        }

        if(result == SDL_PASS)
        {
            pStaticRegs->CCMKEYR2 = readVal;
            (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                            SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID,
                                            &readVal, &result);
        }

        if(result == SDL_PASS)
        {
            pStaticRegs->CCMKEYR3 = readVal;
            (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                            SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID,
                                            &readVal, &result);
        }

        if(result == SDL_PASS)
        {
            pStaticRegs->CCMKEYR5 = readVal;
            (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                            SDL_MCU_ARMSS_CCMR5_CCMKEYR5_REGID,
                                            &readVal, &result);
        }

        if(result == SDL_PASS)
        {
            pStaticRegs->CCMKEYR6 = readVal;
            (void)SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                            SDL_MCU_ARMSS_CCMR5_CCMKEYR6_REGID,
                                            &readVal, &result);
        }

        if(result == SDL_PASS)
        {
            pStaticRegs->CCMPOLCNTRL = readVal;
            sdlResult = SDL_PASS;
        }
    }
    return sdlResult;
}

/**
 *   Design: PROC_SDL-2051
 */
 int32_t SDL_CCM_clearError(SDL_CCM_Inst instance, SDL_CCM_MonitorType monitorType)
 {
    int32_t sdlResult = SDL_PASS;
    int32_t  result = SDL_EFAIL;

    if(instance >= SDL_CCM_MAX_INSTANCE)
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        switch (monitorType)
        {
            case SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK:
            {

                (void)SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID,
                                                SDL_CCM_ALL_STATUS_BITS,
                                                &result);
                break;
            }
            case SDL_CCM_MONITOR_TYPE_VIM:
            {
                (void)SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID,
                                                SDL_CCM_ALL_STATUS_BITS,
                                                &result);
                break;
            }
            case SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR:
            {
                (void)SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID,
                                                SDL_CCM_ALL_STATUS_BITS,
                                                &result);
                break;
            }
            case SDL_CCM_MONITOR_TYPE_TMU:
            {
                (void)SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                SDL_MCU_ARMSS_CCMR5_CCMSR5_REGID,
                                                SDL_CCM_ALL_STATUS_BITS,
                                                &result);
                break;
            }
            case SDL_CCM_MONITOR_TYPE_RL2:
            {
                (void)SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                SDL_MCU_ARMSS_CCMR5_CCMSR6_REGID,
                                                SDL_CCM_ALL_STATUS_BITS,
                                                &result);
                break;
            }
            default:
            {
                sdlResult = SDL_EBADARGS;
                break;
            }
        }

        if(result != SDL_PASS)
        {
            sdlResult = SDL_EFAIL;
        }
    }
    return sdlResult;
 }

/**
 *   Design: PROC_SDL-2048
 */
int32_t SDL_CCM_selfTest (SDL_CCM_Inst instance,
                             SDL_CCM_MonitorType monitorType,
                             SDL_CCM_SelfTestType testType,
                             uint32_t polarityInversionMask,
                             uint32_t timeoutCnt)
{
    int32_t retVal = SDL_EFAIL;
    int32_t sdlResult;
    SDL_McuArmssCcmR5OpModeKey selfTestTypeValue;
    uint32_t statusValue;
    uint32_t timesCount=0;
    SDL_McuArmssCcmR5RegId monitorTypeKeyRegister;
    SDL_McuArmssCcmR5RegId monitorTypeStatusRegister;
    if((instance >= SDL_CCM_MAX_INSTANCE) || (monitorType > SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Get the Key register for the monitor type */
        sdlResult = SDL_CCM_getMonitorKeyRegister(monitorType, &monitorTypeKeyRegister);
    }

    if ( sdlResult == SDL_PASS )
    {
        switch(testType) {

            case SDL_CCM_SELFTEST_TYPE_NORMAL:
                selfTestTypeValue = SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE;
                break;

            case SDL_CCM_SELFTEST_POLARITY_INVERSION:
            case SDL_CCM_SELFTEST_TYPE_ERROR_FORCING:
                selfTestTypeValue = SDL_MCU_ARMSS_CCMR5_MKEY_ERR_FORCE_MODE;
                break;

            default:
                sdlResult = SDL_EBADARGS;
                break;
        }
    }

    /* Check for error in earlier steps */
    if (sdlResult == SDL_PASS)
    {
        /* Get the status register for the monitor type */
        sdlResult = SDL_CCM_getMonitorStatusRegister(monitorType, &monitorTypeStatusRegister);
    }

    if (sdlResult == SDL_PASS)
    {
        /* Clear Status register */
        sdlResult = SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                   monitorTypeStatusRegister,
                                                   SDL_CCM_ALL_STATUS_BITS,
                                                   NULL);

    }

    /* Check for error in earlier steps */
    if (sdlResult == SDL_PASS)
    {
        /* Initialise Error flag to be starting */
        SDL_CCM_instance.selfTestErrorFlag = SDL_CCM_ERROR_FLAG_INPROGRESS;

        if(polarityInversionMask != (uint32_t)0U)
        {
                /* Configure mode to initiate self test polarity inversion */
                retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                        SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID,
                                                        (uint32_t)polarityInversionMask,
                                                    NULL);
        }
        else
        {
             /* Configure mode to initiate normal self test */
             retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                        monitorTypeKeyRegister,
                                                        (uint32_t)selfTestTypeValue,
                                                        NULL);
        }
        if ( retVal != SDL_PASS)
        {
            sdlResult = SDL_EFAIL;
        }
    }

    /* Check for error in earlier steps */
    if (sdlResult == SDL_PASS)
    {
        /* Wait for error to take effect */
        while((timeoutCnt == (uint32_t)0U) || (timesCount < timeoutCnt))
        {
            /* Check if the self test completed */
            retVal = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                   monitorTypeStatusRegister,
                                                   &statusValue,
                                                   NULL);

            if (retVal != SDL_PASS)
            {
                sdlResult = SDL_EFAIL;
            }

            /* Check if the status bit is set to indicate completion of self test */
            if(((statusValue & SDL_CCM_ALL_STATUS_BITS) != 0u)
               || (SDL_CCM_instance.selfTestErrorFlag == SDL_CCM_ERROR_FLAG_TRIGGERED)
               || (sdlResult != SDL_PASS))
            {
                break;
            }
            /* Increment timeout counter */
            timesCount++;

        }
    }
    if(testType == SDL_CCM_SELFTEST_POLARITY_INVERSION)
    {
        if(SDL_CCM_instance.selfTestErrorFlag != SDL_CCM_ERROR_FLAG_TRIGGERED)
        {
            sdlResult = SDL_EFAIL;
        }
    }
    if (sdlResult == SDL_PASS) {
        /* Switch it back to active mode */
        retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                  monitorTypeKeyRegister,
                                                  (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE,
                                                  NULL);

         if (retVal != SDL_PASS)
         {
             sdlResult = SDL_EFAIL;
         }
     }

    if (sdlResult == SDL_PASS)
    {
        /* Check expected error occurred or timeout */
        if (timesCount >= timeoutCnt) {
            if (statusValue != (uint32_t)0U)
            {
                /* Clear status register */
                (void) SDL_armR5ConfigureCCMRegister (SDL_CCM_baseAddress[instance],
                                                      monitorTypeStatusRegister,
                                                      statusValue,
                                                      NULL);
            }
            if(timeoutCnt != (uint32_t)0U)
            {
                sdlResult = SDL_EFAIL;
            }
        }
    }

    /* Reset error flag to be none */
    SDL_CCM_instance.selfTestErrorFlag = SDL_CCM_ERROR_FLAG_NONE;

    return sdlResult;
}

/**
 *   Design: PROC_SDL-2049
 */
int32_t SDL_CCM_injectError (SDL_CCM_Inst instance, SDL_CCM_MonitorType monitorType)
{

    int32_t sdlResult = SDL_EFAIL;
    int32_t retVal;
    SDL_McuArmssCcmR5RegId monitorTypeKeyRegister;

    if((instance >= SDL_CCM_MAX_INSTANCE) || (monitorType > SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Get the key register address corresponding to the monitor type */
        retVal = SDL_CCM_getMonitorKeyRegister(monitorType, &monitorTypeKeyRegister);
        if (retVal == SDL_PASS) {

            /* Configure error forcing mode to inject error by writing to the key register */
            retVal = SDL_armR5ConfigureCCMRegister(SDL_CCM_baseAddress[instance],
                                                       monitorTypeKeyRegister,
                                                       (uint32_t)SDL_MCU_ARMSS_CCMR5_MKEY_ERR_FORCE_MODE,
                                                       NULL);

            if (retVal == SDL_PASS)
            {
                sdlResult = SDL_PASS;
            }
            else{
                sdlResult = SDL_EFAIL;
            }
        }
    }
    return sdlResult;
}

/**
 *   Design: PROC_SDL-2059
 */
int32_t SDL_CCM_getErrorType(SDL_CCM_Inst instance, uint32_t intSrc, SDL_CCM_MonitorType *monitorType)
{
    int32_t sdlResult;
    uint32_t statusValue;
    int32_t retVal = SDL_PASS;

    if((instance >= SDL_CCM_MAX_INSTANCE) || (monitorType == NULL))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        *monitorType =  SDL_CCM_MONITOR_TYPE_NONE;
        switch(intSrc)
        {
            case SDL_ESM0_R5FSS0_R5FSS0_COMPARE_ERR_PULSE_0:
            {
                /* Read status register of CPU output compare block  */
                sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                   SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID,
                                                   &statusValue,
                                                   NULL);
                /* If status bit set, return output compare block */
                if ((sdlResult != SDL_PASS) || ((statusValue & SDL_CCM_ALL_STATUS_BITS) == 0u))
                {
                    retVal = SDL_EFAIL;
                }
                else
                {
                        *monitorType =  SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK;
                }
                break;
            }
            case SDL_ESM0_R5FSS0_R5FSS0_VIM_COMPARE_ERR_PULSE_0:
            {
                /* Read status register of VIM compare block  */
                sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID,
                                                       &statusValue,
                                                       NULL);
                /* If status bit set, return output compare block */
                if ((sdlResult != SDL_PASS) || ((statusValue & SDL_CCM_ALL_STATUS_BITS) == 0u))
                {
                    retVal = SDL_EFAIL;
                }
                else
                {
                        *monitorType =  SDL_CCM_MONITOR_TYPE_VIM;
                }
                break;
            }
            case SDL_ESM0_R5FSS0_R5FSS0_CPU_MISCOMPARE_PULSE_0:
            {
                /* Read status register of Inactivity monitor block  */
                sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID,
                                                       &statusValue,
                                                       NULL);

                /* If status bit set, return output compare block */
                if ((sdlResult != SDL_PASS) || ((statusValue & SDL_CCM_ALL_STATUS_BITS) == 0u))
                {
                    retVal = SDL_EFAIL;
                }
                else
                {
                        *monitorType =  SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR;
                }
                break;
            }
            case SDL_ESM0_R5FSS0_TMU_COMPARE_ERR:
            {
                /* Read status register of TMU compare block  */
                sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMSR5_REGID,
                                                       &statusValue,
                                                       NULL);

                /* If status bit set, return output compare block */
                if ((sdlResult != SDL_PASS) || ((statusValue & SDL_CCM_ALL_STATUS_BITS) == 0u))
                {
                    retVal = SDL_EFAIL;
                }
                else
                {
                        *monitorType =  SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR;
                }
                break;
            }
            case SDL_ESM0_R5FSS0_RL2_COMPARE_ERR:
            {
                /* Read status register of RL2 compare block  */
                sdlResult = SDL_armR5ReadCCMRegister (SDL_CCM_baseAddress[instance],
                                                       SDL_MCU_ARMSS_CCMR5_CCMSR6_REGID,
                                                       &statusValue,
                                                       NULL);

                /* If status bit set, return output compare block */
                if ((sdlResult != SDL_PASS) || ((statusValue & SDL_CCM_ALL_STATUS_BITS) == 0u))
                {
                    retVal = SDL_EFAIL;
                }
                else
                {
                        *monitorType =  SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR;
                }
                break;
            }
            default:
            {
                 *monitorType =  SDL_CCM_MONITOR_TYPE_INVALID;
                 retVal = SDL_EBADARGS;
                 break;
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-2119
 */
int32_t SDL_VIM_cfgIntr( SDL_vimRegs *pRegs, uint32_t intrNum, uint32_t pri, SDL_VimIntrMap intrMap, SDL_VimIntrType intrType, uint32_t vecAddr )
{
    int32_t     retVal = SDL_EFAIL;
    uint32_t    bitNum, groupNum, regVal, regMask;
    void       *pChkRegs = (void *) pRegs;
    uint32_t    maxIntrs = 0;

    if (pChkRegs != SDL_MCU_ARMSS_VIM_NULL_ADDR)
    {
        maxIntrs   = pRegs->INFO;
        groupNum = intrNum / SDL_VIM_NUM_INTRS_PER_GROUP;
        /* Condition "(vecAddr - 1U)" is need for THUMB Mode as TI ARM CLANG marks LSB as '1' */
        if( (intrNum  < maxIntrs)                             &&
            (pri <= SDL_VIM_PRI_INT_VAL_MAX)                  &&
            (intrMap <= SDL_VIM_INTR_MAP_FIQ)                 &&
            (intrType <= SDL_VIM_INTR_TYPE_PULSE)             &&
            (((vecAddr & SDL_VIM_VEC_INT_VAL_MASK) == vecAddr) ||
                (((vecAddr - (uint32_t)1U) & SDL_VIM_VEC_INT_VAL_MASK) == (vecAddr - (uint32_t)1U))) )
        {
            bitNum = intrNum & (SDL_VIM_NUM_INTRS_PER_GROUP-1U);

            /* Configure INTMAP */
            regMask = (uint32_t)(1U) << bitNum;
            regVal = SDL_REG32_RD( &pRegs->GRP[groupNum].INTMAP );
            regVal &= ~regMask;
            regVal |= intrMap;
            SDL_REG32_WR( &pRegs->GRP[groupNum].INTMAP, regVal );

            /* Configure INTTYPE */
            regMask = (uint32_t)(1U) << bitNum;
            regVal = SDL_REG32_RD( &pRegs->GRP[groupNum].INTTYPE );
            regVal &= ~regMask;
            regVal |= intrType << bitNum;
            SDL_REG32_WR( &pRegs->GRP[groupNum].INTTYPE, regVal );

            /* Configure PRI */
            SDL_REG32_WR( &pRegs->PRI[intrNum].INT, SDL_FMK( VIM_PRI_INT_VAL, pri ) );

            /* Configure VEC */
            SDL_REG32_WR( &pRegs->VEC[intrNum].INT, vecAddr );
                retVal = SDL_PASS;
        }
    }

    return retVal;
}
/**
 * Design: PROC_SDL-2120
 */
int32_t SDL_VIM_verifyCfgIntr( SDL_vimRegs *pRegs, uint32_t intrNum, uint32_t pri, SDL_VimIntrMap intrMap, SDL_VimIntrType intrType, uint32_t vecAddr )
{
    int32_t  retVal = SDL_EFAIL;
    uint32_t bitNum, groupNum;
    uint32_t intrMapVal, intrTypeVal, priVal, vecVal;
    void       *pChkRegs = (void *) pRegs;
    uint32_t    maxIntrs = 0;

    if (pChkRegs != SDL_MCU_ARMSS_VIM_NULL_ADDR)
    {
        maxIntrs   = pRegs->INFO;
        groupNum = intrNum / SDL_VIM_NUM_INTRS_PER_GROUP;

        /* Condition "(vecAddr - 1U)" is need for THUMB Mode as TI ARM CLANG marks LSB as '1' */
        if( (intrNum < maxIntrs)                              &&
            (pri <= SDL_VIM_PRI_INT_VAL_MAX)                  &&
            (intrMap <= SDL_VIM_INTR_MAP_FIQ)                 &&
            (intrType <= SDL_VIM_INTR_TYPE_PULSE)             &&
            (((vecAddr & SDL_VIM_VEC_INT_VAL_MASK) == vecAddr) ||
                (((vecAddr - (uint32_t)1U) & SDL_VIM_VEC_INT_VAL_MASK) == (vecAddr - (uint32_t)1U))))
        {
            bitNum = intrNum & (SDL_VIM_NUM_INTRS_PER_GROUP-1U);

            /* Read INTMAP */
            intrMapVal  = SDL_REG32_RD( &pRegs->GRP[groupNum].INTMAP );
            /* Get the interrupt map value */
            intrMapVal  = intrMapVal >> bitNum;
            intrMapVal &= (uint32_t)(0x1U);

            /* Read INTTYPE */
            intrTypeVal  = SDL_REG32_RD( &pRegs->GRP[groupNum].INTTYPE );
            /* Get the interrupt type value */
            intrTypeVal  = intrTypeVal >> bitNum;
            intrTypeVal &= (uint32_t)(0x1U);

            /* Read PRI */
            priVal = SDL_REG32_RD( &pRegs->PRI[intrNum].INT);

            /* Read VEC */
            vecVal = SDL_REG32_RD( &pRegs->VEC[intrNum].INT);
                retVal = SDL_PASS;
        }
    }

    if (retVal != SDL_EFAIL)
    {
        /* verify if parameter matches */
        if ((intrMapVal != intrMap) ||
            (intrTypeVal != (uint32_t)intrType) ||
            (priVal != pri) ||
            (vecVal != vecAddr))
        {
            retVal = SDL_EFAIL;
        }
    }

    return retVal;
}
/**
 * Design: PROC_SDL-2121
 */
int32_t SDL_VIM_getStaticRegs( SDL_vimRegs *pRegs, SDL_vimStaticRegs *pStaticRegs)
{
    int32_t  retVal = SDL_PASS;
    uint32_t i, maxIntrs, num_groups;
    void    *pChkRegs = (void *) pRegs;

    if ( (pChkRegs == SDL_MCU_ARMSS_VIM_NULL_ADDR) ||
         (pStaticRegs == SDL_MCU_ARMSS_VIM_NULL_ADDR) )
    {
        /* No actions - API fails to read back */
        retVal = SDL_EFAIL;
    }

    if (retVal == SDL_PASS)
    {
        pStaticRegs->PID  = SDL_REG32_RD(&pRegs->PID);
        pStaticRegs->INFO = SDL_REG32_RD(&pRegs->INFO);
        pStaticRegs->IRQVEC = SDL_REG32_RD(&pRegs->IRQVEC);
        pStaticRegs->FIQVEC = SDL_REG32_RD(&pRegs->FIQVEC);

        maxIntrs = pRegs->INFO;
        num_groups = maxIntrs / SDL_VIM_NUM_INTRS_PER_GROUP;

        for (i = ((uint32_t) (0u)); i < num_groups; i++)
        {
            pStaticRegs->GRP[i].INTMAP      = SDL_REG32_RD(&pRegs->GRP[i].INTMAP);
            pStaticRegs->GRP[i].INTR_EN_CLR = SDL_REG32_RD(&pRegs->GRP[i].INTR_EN_CLR);
            pStaticRegs->GRP[i].INTR_EN_SET = SDL_REG32_RD(&pRegs->GRP[i].INTR_EN_SET);
            pStaticRegs->GRP[i].INTTYPE     = SDL_REG32_RD(&pRegs->GRP[i].INTTYPE);
        }

        for (i = ((uint32_t) (0u)); i < maxIntrs; i++)
        {
            pStaticRegs->PRI[i].INT         = SDL_REG32_RD(&pRegs->PRI[i].INT);
            pStaticRegs->VEC[i].INT         = SDL_REG32_RD(&pRegs->VEC[i].INT);
        }
        /* Remaining values zero it */
        for (i = num_groups; i < SDL_VIM_MAX_INTR_GROUPS; i++)
        {
            pStaticRegs->GRP[i].INTMAP      = (uint32_t)0u;
            pStaticRegs->GRP[i].INTR_EN_CLR = (uint32_t)0u;
            pStaticRegs->GRP[i].INTR_EN_SET = (uint32_t)0u;
            pStaticRegs->GRP[i].INTTYPE     = (uint32_t)0u;
        }
        for (i = maxIntrs; i < (SDL_VIM_MAX_INTR_GROUPS * SDL_VIM_NUM_INTRS_PER_GROUP); i++)
        {
            pStaticRegs->PRI[i].INT         = (uint32_t)0u;
            pStaticRegs->VEC[i].INT         = (uint32_t)0u;
        }

    }

    return (retVal);
}
/* Nothing past this point */
