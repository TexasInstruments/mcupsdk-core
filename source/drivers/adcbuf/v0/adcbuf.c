/*
 * Copyright (C) 2021-23 Texas Instruments Incorporated
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
 *  \file   adcbuf.c
 *
 *  \brief  This file contains the implementation of the APIs present in the
 *          device abstraction layer file of ADCBUF.
 *          This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/adcbuf.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Number of register bits to configure threshold */
#define ADCBUF_NUMBITS_CHIRPTHRESHOLD   (5U)

/* Number of register bits to configure number of samples */
#define ADCBUF_NUMBITS_NUMBER_SAMPLES   (16U)

/* Number of register bits to configure channel address offset */
#define ADCBUF_NUMBITS_CHAN_ADDR_OFFSET (11U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static ADCBuf_Handle ADCBUF_MMWave_open(ADCBuf_Handle handle, const ADCBuf_Params *params);

static void ADCBUFSrcSelect(CSL_rss_ctrlRegs *rssCtrlRegs, uint32_t source);
static void ADCBUFSetPingNumChirpThreshhold(CSL_rss_ctrlRegs *rssCtrlRegs, uint32_t threshhold);
static void ADCBUFSetPongNumChirpThreshhold(CSL_rss_ctrlRegs *rssCtrlRegs, uint32_t threshhold);
static void ADCBUFContinuousModeCtrl(CSL_rss_ctrlRegs *rssCtrlRegs, uint32_t mode);
static void ADCBUFContinuousModeStart(CSL_rss_ctrlRegs *rssCtrlRegs, uint16_t numSamples);
static void ADCBUFContinuousModeStop(CSL_rss_ctrlRegs *rssCtrlRegs);
static void ADCBUFConfigureDataFormat(CSL_rss_ctrlRegs *rssCtrlRegs, uint8_t dataFormat, uint8_t interleave, uint8_t iqConfig);
static void ADCBUFChannelEnSetOffset(CSL_rss_ctrlRegs *rssCtrlRegs, uint8_t channel, uint16_t offset);
static void ADCBUFChannelDisable(CSL_rss_ctrlRegs *rssCtrlRegs, uint8_t channel);
static void ADCBUFTestPatternConfig(CSL_rss_ctrlRegs *rssCtrlRegs, const ADCBuf_TestPatternConf *testPatternConf);
static void ADCBUFTestPatternStart(CSL_rss_ctrlRegs *rssCtrlRegs, uint32_t numOfClks);
static void ADCBUFTestPatternStop(CSL_rss_ctrlRegs *rssCtrlRegs);
static void ADCBUFCQConfig(CSL_rss_ctrlRegs *rssCtrlRegs, ADCBuf_CQConf *cqCfg);
static int32_t ADCBUFDriverParamsCheck(const ADCBuf_Params *params);
static int32_t ADCBUFCmdParamCheck(ADCBufMMWave_CMD cmd, void* arg);
static uint32_t ADCBUFIsChannelEnabled(CSL_rss_ctrlRegs *rssCtrlRegs, uint32_t channel);
static int32_t ADCBUFcheckForTimeout(uint32_t addr,
                                     uint32_t timeToWaitInTicks,
                                     uint32_t value);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Default ADC parameters structure */
const ADCBuf_Params gADCBufDefaultParams =
{
    /* ADC buffer source, DFE or HIL */
    ADCBUF_SOURCE_DFE,
    /* Continuous mode selection */
    0,
    /* Ping buffer Chirp Threshold for non-continous operation */
    1,
    /* Pong buffer Chirp Threshold for non-continous operation */
    1,
    /* Custom configuration */
    NULL
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t ADCBUFcheckForTimeout(uint32_t addr,
                                     uint32_t timeToWaitInTicks,
                                     uint32_t value)
{
    uint32_t curTicks = ClockP_getTicks();
    uint32_t elaspedTicks, done = 0;
    int32_t status = SystemP_SUCCESS;

    do{
        if(CSL_REG32_RD(addr) == value)
        {
            status = SystemP_SUCCESS;
            done = 1;
        }
        if(done == 0U)
        {
            elaspedTicks =  ClockP_getTicks() - curTicks;

            if(elaspedTicks >= timeToWaitInTicks)
            {
                status = SystemP_TIMEOUT;
                done = 1;
            }
        }
    } while (done == 0);

    return status;
}

void ADCBuf_init(uint32_t timeout)
{
    uint32_t           i;
    ADCBuf_Config      *config;
    ADCBuf_Attrs       *attrs;
    CSL_rss_ctrlRegs   *rssCtrl;
    int32_t            status = SystemP_SUCCESS;

    /* Call each driver's init function */
    for(i = 0; i < gADCBufConfigNum; i++)
    {
        config = &gADCBufConfig[i];
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        rssCtrl = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /*
         * Initialize ADCBUF Ping Memory.
         */
        /* Clear MEMINIT DONE before initiating MEMINIT */
        CSL_FINS(rssCtrl->RSS_ADCBUF_PING_MEMINIT_DONE, RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE_RSS_ADCBUF_PING_MEMINIT_DONE_DONE, 1U);

        status = ADCBUFcheckForTimeout((uint32_t)&rssCtrl->RSS_ADCBUF_PING_MEMINIT_DONE, timeout, 0U);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Start ADCBUF Ping memory Initialization */
        CSL_FINS(rssCtrl->RSS_ADCBUF_PING_MEMINIT, RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_RSS_ADCBUF_PING_MEMINIT_START, 1U);
        status = ADCBUFcheckForTimeout((uint32_t)&rssCtrl->RSS_ADCBUF_PING_MEMINIT_DONE, timeout, 1U);
        DebugP_assert(status == SystemP_SUCCESS);

        /*
         * Initialize ADCBUF Pong Memory.
         */
        /* Clear MEMINIT DONE before initiating MEMINIT */
        CSL_FINS(rssCtrl->RSS_ADCBUF_PONG_MEMINIT_DONE, RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE_RSS_ADCBUF_PONG_MEMINIT_DONE_DONE, 1U);

        status = ADCBUFcheckForTimeout((uint32_t)&rssCtrl->RSS_ADCBUF_PONG_MEMINIT_DONE, timeout, 0U);
        DebugP_assert(status == SystemP_SUCCESS);

        CSL_FINS(rssCtrl->RSS_ADCBUF_PONG_MEMINIT, RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_RSS_ADCBUF_PONG_MEMINIT_START, 1U);
        status = ADCBUFcheckForTimeout((uint32_t)&rssCtrl->RSS_ADCBUF_PONG_MEMINIT_DONE, timeout, 1U);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    return;
}

void ADCBuf_deinit(void)
{
    return;
}

ADCBuf_Handle ADCBuf_open(uint8_t index, const ADCBuf_Params *params)
{
    ADCBuf_Handle handle;

    if(index >= gADCBufConfigNum)
    {
        handle = (ADCBuf_Handle)NULL;
    }
    else
    {
        /* If params are NULL use defaults */
        if(params == NULL)
        {
            params = (ADCBuf_Params *) &gADCBufDefaultParams;
        }

        /* Get handle for this driver instance */
        handle = (ADCBuf_Handle)&(gADCBufConfig[index]);

        /* Open the ADCBUF mmWave Driver: */
        handle = ADCBUF_MMWave_open(handle, params);
    }

    return (handle);
}

void ADCBuf_Params_init(ADCBuf_Params *params)
{
    if(params != NULL)
    {
        *params = gADCBufDefaultParams;
    }
}

/**
 *  @b Description
 *  @n
 *      Close ADCBUF Driver. ADCBUF_MMWave_open() has to be called first.
 *
 *  @param[in]  handle
 *      ADCBUF Instance Handle
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void ADCBuf_close(ADCBuf_Handle handle)
{
    ADCBuf_Config      *config;
    ADCBuf_Object      *obj;

    DebugP_assert(handle != (ADCBuf_Handle)NULL);

    /* Get the pointer to the object */
    config = (ADCBuf_Config *) handle;
    obj = (ADCBuf_Object* ) config->object;

    /* Mark the module as available */
    obj->isOpen = FALSE;

    return;
}

/**
 *  @b Description
 *  @n
 *      Get/Set ADCBUF_MMWave specific ADCBUF functions
 *
 *  @param[in]  handle
 *      ADCBUF Instance Handle
 *  @param[in]  cmd
 *      Command to the ADCBUF driver
 *  @param[in]  arg
 *      Argument for the driver command
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Status of the command execution
 */
int32_t ADCBuf_control(ADCBuf_Handle handle, uint8_t cmd, void *arg)
{
    ADCBuf_Config          *config;
    ADCBuf_Attrs           *attrs;
    CSL_rss_ctrlRegs       *rssCtrlRegs;
    int32_t                status = SystemP_SUCCESS;
    ADCBuf_dataFormat      *dataFormat;
    ADCBuf_RxChanConf      *rxChanConf;
    ADCBuf_TestPatternConf *testPatternConf;
    ADCBuf_CQConf          *cqConf;
    uint32_t                chirpThreshold;
    uint8_t                 channel;
    uint32_t                channelMask;

    if(handle == (ADCBuf_Handle) NULL)
    {
        status = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Get the Object from ADCBuf Handle */
        config = (ADCBuf_Config *) handle;
        attrs = (ADCBuf_Attrs *) config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /* Stop doesn't need any error check */
        if(cmd != ADCBufMMWave_CMD_STOP_TEST_PATTERN)
        {
            status = ADCBUFCmdParamCheck((ADCBufMMWave_CMD)cmd, arg);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        switch(cmd)
        {
            case ADCBufMMWave_CMD_SET_SRC:
                ADCBUFSrcSelect(rssCtrlRegs, *(uint32_t *)arg);
                break;

            case ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD:
                chirpThreshold = *(uint32_t *)arg;
                ADCBUFSetPingNumChirpThreshhold(rssCtrlRegs, chirpThreshold - 1U);
                break;

            case ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD:
                chirpThreshold = *(uint32_t *)arg;
                ADCBUFSetPongNumChirpThreshhold(rssCtrlRegs, chirpThreshold - 1U);
                break;

            case ADCBufMMWave_CMD_SET_CONTINUOUS_MODE:
                ADCBUFContinuousModeCtrl(rssCtrlRegs, *(uint32_t *)arg);
                break;

            case ADCBufMMWave_CMD_START_CONTINUOUS_MODE:
                ADCBUFContinuousModeStart(rssCtrlRegs, *(uint16_t *)arg);
                break;

            case ADCBufMMWave_CMD_STOP_CONTINUOUS_MODE:
                ADCBUFContinuousModeStop(rssCtrlRegs);
                break;

            case ADCBufMMWave_CMD_CONF_DATA_FORMAT:
                dataFormat = (ADCBuf_dataFormat *)arg;
                ADCBUFConfigureDataFormat(rssCtrlRegs,
                                          dataFormat->adcOutFormat,
                                          dataFormat->channelInterleave,
                                          dataFormat->sampleInterleave);
                break;

            case ADCBufMMWave_CMD_CHANNEL_ENABLE:
                rxChanConf = (ADCBuf_RxChanConf *)arg;
                ADCBUFChannelEnSetOffset(rssCtrlRegs, rxChanConf->channel, rxChanConf->offset);
                break;

            case ADCBufMMWave_CMD_CHANNEL_DISABLE:
                channelMask = *(uint32_t *)arg;
                for(channel = 0; channel < SOC_ADCBUF_NUM_RX_CHANNEL; channel++)
                {
                    if(channelMask & ((uint32_t)0x1U << channel))
                    {
                        ADCBUFChannelDisable(rssCtrlRegs, channel);
                    }
                }
                break;

            case ADCBufMMWave_CMD_CONF_TEST_PATTERN:
                testPatternConf = (ADCBuf_TestPatternConf *)arg;
                ADCBUFTestPatternConfig(rssCtrlRegs, testPatternConf);
                break;

            case ADCBufMMWave_CMD_START_TEST_PATTERN:
                ADCBUFTestPatternStart(rssCtrlRegs, *(uint32_t *)arg);
                break;

            case ADCBufMMWave_CMD_STOP_TEST_PATTERN:
                ADCBUFTestPatternStop(rssCtrlRegs);
                break;

            case ADCBufMMWave_CMD_CONF_CQ:
                cqConf = (ADCBuf_CQConf *)arg;
                ADCBUFCQConfig(rssCtrlRegs, cqConf);
                break;

            default:
                status = ADCBUF_STATUS_UNDEFINEDCMD;
                break;
        }
    }

    return (status);
}

/**
 *  @b Description
 *  @n
 *      Get address of ADC buffer for a given receive channel
 *
 *  @param[in]  handle
 *      ADCBuf Instance Handle
 *  @param[in]  channel
 *      ADCBuf receive channel number
 *  @param[in]  errCode
 *      Error code populated by driver if error condition is detected
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   ADCBuf Physical adderss of the receive channel
 *  @retval
 *      Error  -    0U
 */
uint32_t ADCBuf_getChanBufAddr(ADCBuf_Handle handle, uint8_t channel, int32_t *errCode)
{
    ADCBuf_Config           *config;
    ADCBuf_Attrs            *attrs;
    CSL_rss_ctrlRegs       *rssCtrlRegs;
    uint32_t                 chanAddress = (uint32_t) 0U;

    /* Parameter check */
    if((channel >= SOC_ADCBUF_NUM_RX_CHANNEL) || (handle == (ADCBuf_Handle)NULL))
    {
        /* Out of range channel number or invalid handle */
        *errCode = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Sanity check Object handle */
        config = (ADCBuf_Config *) handle;
        DebugP_assert(config->object != (ADCBuf_Object *) NULL);
        attrs = (ADCBuf_Attrs *) config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /* Set default value for errCode */
        *errCode = SystemP_SUCCESS;

        /* Check if the channel is enabled? */
        if(ADCBUFIsChannelEnabled(rssCtrlRegs, channel) != (uint32_t)0U)
        {
            uint32_t addrOffset;

            switch(channel)
            {
                case 0U:
                    addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG2,
                                                    CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_MASK,
                                                    CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_SHIFT);
                    break;
                case 1U:
                    addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG2,
                                                    CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_MASK,
                                                    CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_SHIFT);
                    break;

                case 2U:
                    addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG3,
                                                    CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_MASK,
                                                    CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_SHIFT);
                    break;
                default:
                case 3U:
                    addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG3,
                                                    CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_MASK,
                                                    CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_SHIFT);
                    break;
            }

            /* Calculate the physical address for the channel */
            chanAddress = attrs->adcbufBaseAddr + ((uint32_t)addrOffset << 4U);
        }
        else
        {
            /* Channel is not enabled */
            *errCode = ADCBUF_STATUS_INVALID_PARAMS;
        }
    }

    return (chanAddress);
}

/**
 *  @b Description
 *  @n
 *      Get address of CQ buffer for a given CQ
 *
 *  @param[in]  handle
 *      ADCBuf Instance Handle
 *  @param[in]  cqType
 *      CQ type that request the buffer address
 *  @param[in]  errCode
 *      Error code populated by driver if error condition is detected
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   CQ buffer Physical adderss of the requested CQ type
 *  @retval
 *      Error  -    0U
 */
uint32_t ADCBUF_MMWave_getCQBufAddr(ADCBuf_Handle handle,
                                    ADCBufMMWave_CQType cqType,
                                    int32_t *errCode)
{
    ADCBuf_Config            *config;
    CSL_rss_ctrlRegs        *rssCtrlRegs;
    ADCBuf_Attrs             *attrs;
    uint32_t                 chanAddress = 0U;
    uint32_t                 addrOffset;

    /* Set default value for errCode */
    *errCode = SystemP_SUCCESS;

    /* Parameter check */
    if(handle == (ADCBuf_Handle)NULL)
    {
        /* Out of range channel number or invalid handle */
        *errCode = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Sanity check Object handle */
        config = (ADCBuf_Config *) handle;
        DebugP_assert(config->object != (ADCBuf_Object *)NULL);
        attrs = (ADCBuf_Attrs *)config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        switch(cqType)
        {
            case ADCBufMMWave_CQType_CQ0:
                addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->CQCFG1,
                                                CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_MASK,
                                                CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_SHIFT);
                break;

            case ADCBufMMWave_CQType_CQ1:
                addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->CQCFG1,
                                                CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_MASK,
                                                CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_SHIFT);
                break;

            case ADCBufMMWave_CQType_CQ2:
                addrOffset = CSL_REG32_FEXT_RAW(&rssCtrlRegs->CQCFG1,
                                                CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_MASK,
                                                CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_SHIFT);
                break;

            default:
                *errCode = ADCBUF_STATUS_INVALID_PARAMS;
                break;
        }

        if(*errCode == SystemP_SUCCESS)
        {
            /* Calculate the physical address for the channel */
            chanAddress = attrs->cqbufBaseAddr + ((uint32_t)addrOffset << 4U);
        }
    }

    return chanAddress;
}

/**
 *  @b Description
 *  @n
 *      Open ADCBUF Driver
 *
 *  @param[in]  handle
 *      ADCBuf Instance Handle
 *  @param[in]  params
 *      Parameter block for the ADCBUF Instance. If NULL, it will use default values.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   ADCBUF driver handle
 *  @retval
 *      Error     -   NULL
 */
static ADCBuf_Handle ADCBUF_MMWave_open(ADCBuf_Handle handle, const ADCBuf_Params *params)
{
    ADCBuf_Config          *config;
    ADCBuf_Object          *obj = NULL;
    ADCBuf_Attrs           *attrs;
    CSL_rss_ctrlRegs      *rssCtrlRegs;
    ADCBuf_Handle           retHandle = NULL;

    /* Sanity check handle */
    DebugP_assert(handle != (ADCBuf_Handle)NULL);
    DebugP_assert(params != (const ADCBuf_Params *)NULL);

    /* Get the pointer to the object and attrs */
    config = (ADCBuf_Config *) handle;
    DebugP_assert(config->attrs != NULL);
    DebugP_assert(config->object != NULL);
    attrs = (ADCBuf_Attrs *)config->attrs;

    /* Validate params */
    if(ADCBUFDriverParamsCheck(params) < 0)
    {
         /* Error: Invalid chirpThreshold, put in debug log */
        DebugP_log("ADCBUF: Invalid chirp Threshold setting(%d)\n", params->chirpThresholdPing);
    }
    else
    {
        /* Allocate memory for the driver: */
        obj = config->object;

        /* Check if driver is not opened */
        if(obj->isOpen != TRUE)
        {
            /* Initialize the memory: */
            memset((void *)obj, 0, sizeof(ADCBuf_Object));

            /* Get the DSS register base address  */
            rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

            /* Configuration of ADCBUF params */
            ADCBUFSrcSelect(rssCtrlRegs, (uint32_t)params->source);
            ADCBUFContinuousModeCtrl(rssCtrlRegs, params->continousMode);

            /* Configurate chirp threshold */
            ADCBUFSetPingNumChirpThreshhold(rssCtrlRegs, ((uint32_t)params->chirpThresholdPing - 1U));
            ADCBUFSetPongNumChirpThreshhold(rssCtrlRegs, ((uint32_t)params->chirpThresholdPong - 1U));

            /* Mark the handle as being used */
            obj->isOpen = TRUE;

            retHandle = config;
        }
    }

    return (retHandle);
}

/**
 *  @b Description
 *  @n
 *      Selects the source of ADCBUF (DFE or HIL).
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  source
 *      Source of the ADCBuf data.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFSrcSelect(CSL_rss_ctrlRegs  *rssCtrlRegs, uint32_t source)
{
    /* Setup the ADC buffer source */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->DMMSWINT1,
                       CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_MASK,
                       CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_SHIFT,
                       source);
}

/**
 *  @b Description
 *  @n
 *      Set number of Chirps  threshold for Ping buffer to trigger ping/pong buffer switch
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  threshold
 *      Number of chirps
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFSetPingNumChirpThreshhold(CSL_rss_ctrlRegs  *rssCtrlRegs, uint32_t threshhold)
{
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG4,
                       CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING_MASK,
                       CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING_SHIFT,
                       threshhold);
}

/**
 *  @b Description
 *  @n
 *      Set number of Chirps  threshhold for Pong buffer to trigger ping/pong buffer switch
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  threshold
 *      Number of chirps
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFSetPongNumChirpThreshhold(CSL_rss_ctrlRegs  *rssCtrlRegs, uint32_t threshhold)
{
    /* Number of chirps for Pong buffer */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG4,
                       CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG_MASK,
                       CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG_SHIFT,
                       threshhold);
}

/**
 *  @b Description
 *  @n
 *      ADC Buffer continuous mode control. Set to 1 to operate in continuous mode
 *
 *  @param[in]  rssCtrlRegs
 *      Base Address to the RSS CTRL Register Base
 *  @param[in]  mode
 *      Continuous mode, 1 to enable, 0 to disable.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFContinuousModeCtrl(CSL_rss_ctrlRegs  *rssCtrlRegs, uint32_t mode)
{
    /* Setup the continuous mode control */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                       CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN_MASK,
                       CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN_SHIFT,
                       mode);
}

/**
 *  @b Description
 *  @n
 *      Start ADC Buffer in continuous mode.
 *
 *  @param[in]  rssCtrlRegs
 *      Base address to the RSS CTRL Register Base
 *  @param[in]  numSamples
 *      Number of samples to be saved in ping/pong buffer before buffer switch.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFContinuousModeStart(CSL_rss_ctrlRegs  *rssCtrlRegs,   uint16_t numSamples)
{
    /* Starts the continuous mode operation */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                       CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTRTPL_MASK,
                       CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTRTPL_SHIFT,
                       1U);

    /* Setup the sample count */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG4,
                       CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_MASK,
                       CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_SHIFT,
                       (uint32_t) numSamples);
}

/**
 *  @b Description
 *  @n
 *      Stop ADC Buffer in continuous mode.
 *
 *  @param[in]  rssCtrlRegs
 *      Base address to the RSS CTRL Register Base
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFContinuousModeStop(CSL_rss_ctrlRegs  *rssCtrlRegs)
{
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                       CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTOPPL_MASK,
                       CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTOPPL_SHIFT,
                       1U);
}

/**
 *  @b Description
 *  @n
 *      Configure ADC Buffer data format.
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  dataFormat
 *      0 for complex, 1 for real data format.
 *  @param[in]  interleave
 *      Enable interleave mode.
 *  @param[in]  iqConfig
 *      In complex mode, 0 to store as Q(MSB) + I(LSB), 1 to store as I(MSB) +Q(LSB)
 *      In Real mode, this field is don't care
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFConfigureDataFormat(CSL_rss_ctrlRegs  *rssCtrlRegs, uint8_t dataFormat, uint8_t interleave, uint8_t iqConfig)
{
    if(dataFormat == 0)    /* Complex data format */
    {
        /* The requested data format is complex */
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                        CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_MASK,
                        CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_SHIFT,
                        (uint32_t)dataFormat);

        /* Setup the IQ swap configuration */
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                        CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP_MASK,
                        CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP_SHIFT,
                        (uint32_t)iqConfig);
    }
    else
    {
        /* The requested data format is real */
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                        CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_MASK,
                        CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_SHIFT,
                        (uint32_t)dataFormat);
    }

    /* Update the interleave mode */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE_MASK,
                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE_SHIFT,
                    (uint32_t)interleave);
}

/**
 *  @b Description
 *  @n
 *      Enable ADC Buffer RX channels
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  channel
 *      RX channel number
 *  @param[in]  offset
 *      Address offset in Ping/Pong buffer
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFChannelEnSetOffset(CSL_rss_ctrlRegs  *rssCtrlRegs, uint8_t channel, uint16_t offset)
{
    switch(channel)
    {
        case 0U:
            /* Enable the channel */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_SHIFT,
                               1U);

            /* Setup the offset */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG2,
                               CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_SHIFT,
                               ((uint32_t)offset >> 4U));
            break;
        case 1U:
            /* Enable the channel */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_SHIFT,
                               1U);

            /* Setup the offset */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG2,
                               CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_SHIFT,
                               ((uint32_t)offset >> 4U));
            break;
        case 2U:
            /* Enable the channel */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_SHIFT,
                               1U);

            /* Setup the offset */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG3,
                               CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_SHIFT,
                               ((uint32_t)offset >> 4U));
            break;
        case 3U:
            /* Enable the channel */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_SHIFT,
                               1U);

            /* Setup the offset */
            CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG3,
                               CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_MASK,
                               CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_SHIFT,
                               ((uint32_t)offset >> 4U));
            break;

        default:
            /* Not  supported channels, code should not end up here */
            DebugP_assert(0);
            break;
    }
}

/**
 *  @b Description
 *  @n
 *      Disable ADC Buffer RX channels
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  channel
 *      RX channel number
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFChannelDisable(CSL_rss_ctrlRegs  *rssCtrlRegs, uint8_t channel)
{
    /* Disable the channel */
    if(channel == 0)
    {
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_MASK,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_SHIFT,
                            0U);
    }
    else if(channel == 1)
    {
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_MASK,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_SHIFT,
                            0U);
    }
    else if(channel == 2)
    {
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_MASK,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_SHIFT,
                            0U);
    }
    else
    {
        CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG1,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_MASK,
                            CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_SHIFT,
                            0U);
    }
}

/**
 *  @b Description
 *  @n
 *      Configure Test pattern for ADC Buffer. Based
*   on the passed values, it sets up the offset and value for each successive
*   sample for the test pattern IQ data, for each of the 4 channels. It also
*   configures period between successive samples of test pattern, and number of
*   samples to store(per channel) in each Ping and Pong register in continuous
*   mode of ADC Buffer.
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  testPatternConf
 *      Configuratio of test pattern for all channels
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFTestPatternConfig(CSL_rss_ctrlRegs  *rssCtrlRegs, const ADCBuf_TestPatternConf *testPatternConf)
{
    /* Setup the test pattern */
    /* RX1 Config */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX1ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IINCR_SHIFT,
                        testPatternConf->rxConfig[0].rxIInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX1ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IOFFSET_SHIFT,
                        testPatternConf->rxConfig[0].rxIOffset);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX1QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QINCR_SHIFT,
                        testPatternConf->rxConfig[0].rxQInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX1QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QOFFSET_SHIFT,
                        testPatternConf->rxConfig[0].rxQOffset);

    /* RX2 Config */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX2ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IINCR_SHIFT,
                        testPatternConf->rxConfig[1].rxIInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX2ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IOFFSET_SHIFT,
                        testPatternConf->rxConfig[1].rxIOffset);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX2QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QINCR_SHIFT,
                        testPatternConf->rxConfig[1].rxQInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX2QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QOFFSET_SHIFT,
                        testPatternConf->rxConfig[1].rxQOffset);

    /* RX3 Config */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX3ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IINCR_SHIFT,
                        testPatternConf->rxConfig[2].rxIInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX3ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IOFFSET_SHIFT,
                        testPatternConf->rxConfig[2].rxIOffset);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX3QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QINCR_SHIFT,
                        testPatternConf->rxConfig[2].rxQInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX3QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QOFFSET_SHIFT,
                        testPatternConf->rxConfig[2].rxQOffset);

    /* RX4 Config */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX4ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IINCR_SHIFT,
                        testPatternConf->rxConfig[3].rxIInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX4ICFG,
                        CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IOFFSET_SHIFT,
                        testPatternConf->rxConfig[3].rxIOffset);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX4QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QINCR_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QINCR_SHIFT,
                        testPatternConf->rxConfig[3].rxQInc);

    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNRX4QCFG,
                        CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QOFFSET_MASK,
                        CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QOFFSET_SHIFT,
                        testPatternConf->rxConfig[3].rxQOffset);

    /* Setup the period */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNVLDCFG,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_MASK,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_SHIFT,
                        testPatternConf->period);

    /* Setup the sample count */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->ADCBUFCFG4,
                        CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_MASK,
                        CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_SHIFT,
                        (testPatternConf->numSamples));
}

/**
 *  @b Description
 *  @n
 *      Starts the test pattern generation
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  numOfClks
 *      Number of Interconnect clocks between
 *      successive samples for the test pattern gen.
 *      Valid Range is 0x00 to 0xFF.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFTestPatternStart(CSL_rss_ctrlRegs  *rssCtrlRegs, uint32_t numOfClks)
{
    /* Lower the clock */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNVLDCFG,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_MASK,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_SHIFT,
                        numOfClks);
    /* Test pattern start */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNVLDCFG,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_MASK,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_SHIFT,
                        0x7U);
}

/**
 *  @b Description
 *  @n
 *      Stop the test pattern generation
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFTestPatternStop(CSL_rss_ctrlRegs  *rssCtrlRegs)
{
    CSL_REG32_FINS_RAW(&rssCtrlRegs->TESTPATTERNVLDCFG,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_MASK,
                        CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_SHIFT,
                        0x0U);
}

/**
 *  @b Description
 *  @n
 *      Configure the Chirp Quality parameters
 *
 *  @param[in]  rssCtrlRegs
 *      Pointer to the RSS CTRL Register Base
 *  @param[in]  cqCfg
 *      Pointer to the CQ configuration
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static void ADCBUFCQConfig(CSL_rss_ctrlRegs *rssCtrlRegs, ADCBuf_CQConf *cqCfg)
{
    /* Configure the CQ data width */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->CQCFG1,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQDATAWIDTH_MASK,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQDATAWIDTH_SHIFT,
                        cqCfg->cqDataWidth);

    /* Configure the if 96 bit pack mode */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->CQCFG1,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ96BITPACKEN_MASK,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ96BITPACKEN_SHIFT,
                        cqCfg->cq96BitPackEn);

    /* Configure the CQ1 base address */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->CQCFG1,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_MASK,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_SHIFT,
                        ((uint32_t)cqCfg->cq0AddrOffset >> 4U));

    /* Configure the CQ2 base address */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->CQCFG1,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_MASK,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_SHIFT,
                        ((uint32_t)cqCfg->cq1AddrOffset >> 4U));

    /* Configure the CQ3 base address */
    CSL_REG32_FINS_RAW(&rssCtrlRegs->CQCFG1,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_MASK,
                        CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_SHIFT,
                        ((uint32_t)cqCfg->cq2AddrOffset >> 4U));
}

/**
 *  @b Description
 *  @n
 *      Parameter check for input parameters. This function should be called when a parameter is expected.
 *  Hence arg should be a valid pointer.
 *
 *  @param[in]  params
 *      Parameter block for the ADCBUF Instance. If NULL, it will use default values.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static int32_t ADCBUFDriverParamsCheck(const ADCBuf_Params *params)
{
    int32_t     retCode = SystemP_SUCCESS;
    uint32_t    paramVal;

    /* Check continuous mode of the ADCBUF */
    paramVal = params->continousMode;
    retCode += ADCBUFCmdParamCheck(ADCBufMMWave_CMD_SET_CONTINUOUS_MODE, (void *)&paramVal);
    paramVal = params->chirpThresholdPing;
    retCode += ADCBUFCmdParamCheck(ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD, (void *)&paramVal);
    paramVal = params->chirpThresholdPong;
    retCode += ADCBUFCmdParamCheck(ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD, (void *)&paramVal);

    return (retCode);
}

/**
 *  @b Description
 *  @n
 *      Parameter check for input parameters. This function should be called when a parameter is expected.
 *  Hence arg should be a valid pointer.
 *
 *  @param[in]  cmd
 *      ADCBUF control command
 *  @param[in]  arg
 *      ACCBUF control arguments.
 *
 *  \ingroup ADCBUF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      N/A
 */
static int32_t ADCBUFCmdParamCheck(ADCBufMMWave_CMD cmd, void* arg)
{
    ADCBuf_dataFormat  *dataFormat;
    ADCBuf_RxChanConf  *rxChanConf;
    ADCBuf_CQConf      *cqConf;
    uint32_t            paramVal;
    int32_t             retCode = SystemP_SUCCESS;

    /* Validate the pointer to the command arguments
     * validate argument is 4 bytes  aligned.
     */
    if((arg == (void *)NULL) || (((uint32_t)arg % 4U) != 0))
    {
        retCode = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        switch (cmd)
        {
            /* 1 bit setting Command */
            case ADCBufMMWave_CMD_SET_SRC:
            case ADCBufMMWave_CMD_SET_CONTINUOUS_MODE:
                paramVal = *(uint32_t *)arg;
                if(paramVal >= ((uint32_t)0x1U << 1U))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_CONF_DATA_FORMAT:
                dataFormat = (ADCBuf_dataFormat *)arg;
                if((dataFormat->adcOutFormat >= (uint8_t)(0x1U << 1U))  ||
                   (dataFormat->channelInterleave >= (uint8_t)(0x1U << 1U)) ||
                   (dataFormat->sampleInterleave >= (uint8_t)(0x1U << 1U)))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_CONF_CQ:
                cqConf = (ADCBuf_CQConf *)arg;
                if((cqConf->cqDataWidth > (uint8_t)(0x1U<<2U)) ||
                   (cqConf->cq96BitPackEn > (uint8_t)(0x1U << 1U)))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

              case ADCBufMMWave_CMD_START_CONTINUOUS_MODE:
                /* 16 bits for NUMBER OF SAMPLES */
                paramVal = *(uint32_t *)arg;
                if(paramVal >= ((uint32_t)0x1U << ADCBUF_NUMBITS_NUMBER_SAMPLES))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_CHANNEL_ENABLE:
                rxChanConf = (ADCBuf_RxChanConf *)arg;
                /* Hardware supports channels 0-3 */
                if((rxChanConf->channel >= SOC_ADCBUF_NUM_RX_CHANNEL) ||
                    (((uint32_t)rxChanConf->offset >> 4) >= ((uint32_t)0x1U << ADCBUF_NUMBITS_CHAN_ADDR_OFFSET)))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_CHANNEL_DISABLE:
                paramVal = *(uint32_t *)arg;
                /* Hardware supports channels 0-3 */
                if(((paramVal & 0xFFFFFFF0U) != 0U) || (paramVal == 0U))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_START_TEST_PATTERN:
                paramVal = *(uint32_t *)arg;
                if(paramVal > 0xFFU)
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD:
            case ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD:
                if(((*(uint8_t *)arg) == 0) || ((*(uint8_t *)arg) > (uint8_t)(0x1U <<ADCBUF_NUMBITS_CHIRPTHRESHOLD)))
                {
                    retCode = ADCBUF_STATUS_INVALID_PARAMS;
                }
                break;

            case ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD:
                retCode = ADCBUF_STATUS_INVALID_CMD;
                break;

            default:
                break;
        }
    }

    return (retCode);
}

static uint32_t ADCBUFIsChannelEnabled(CSL_rss_ctrlRegs  *rssCtrlRegs, uint32_t channel)
{
    uint32_t retVal = 0U;

    if(channel == 0)
    {
        retVal = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG1,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_MASK,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_SHIFT);
    }
    else if(channel == 1)
    {
        retVal = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG1,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_MASK,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_SHIFT);
    }
    else if(channel == 2)
    {
        retVal = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG1,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_MASK,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_SHIFT);
    }
    else
    {
        retVal = CSL_REG32_FEXT_RAW(&rssCtrlRegs->ADCBUFCFG1,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_MASK,
                                    CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_SHIFT);
    }

    return (retVal);
}

int32_t ADCBUF_verifySrcSelCfg(ADCBuf_Handle handle, uint32_t source)
{
    int32_t            status = SystemP_SUCCESS;
    uint32_t           sourceSel;
    ADCBuf_Config      *config;
    ADCBuf_Attrs       *attrs;
    CSL_rss_ctrlRegs   *rssCtrlRegs;

    if((handle == (ADCBuf_Handle) NULL)   ||
       (source > ADCBUF_SOURCE_SELECT_MAX))
    {
        status = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Get the Object from ADCBuf Handle */
        config = (ADCBuf_Config *) handle;
        attrs = (ADCBuf_Attrs *) config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /* Read source select configuration field from DMMSWINT1 register*/
        sourceSel = CSL_FEXT(rssCtrlRegs->DMMSWINT1,
                             RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN);

        if(sourceSel != source)
        {
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

int32_t ADCBUF_verifyChirpThreshold(ADCBuf_Handle handle,
                                    uint32_t pingThreshCfg,
                                    uint32_t pongThreshCfg)
{
    int32_t            status = SystemP_SUCCESS;
    uint32_t           pingThresh;
    uint32_t           pongThresh;
    ADCBuf_Config      *config;
    ADCBuf_Attrs       *attrs;
    CSL_rss_ctrlRegs   *rssCtrlRegs;

    if((handle == (ADCBuf_Handle) NULL)            ||
       (pingThreshCfg > ADCBUF_PING_THRESHOLD_MAX) ||
       (pongThreshCfg > ADCBUF_PONG_THRESHOLD_MAX))
    {
        status = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Get the Object from ADCBuf Handle */
        config = (ADCBuf_Config *) handle;
        attrs = (ADCBuf_Attrs *) config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /* Read ping threshold configuration field */
        pingThresh = CSL_FEXT(rssCtrlRegs->ADCBUFCFG4,
                              RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING);

        if((pingThresh + 1U) != pingThreshCfg)
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Read pong threshold configuration field */
        pongThresh = CSL_FEXT(rssCtrlRegs->ADCBUFCFG4,
                              RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG);

        if((pongThresh + 1U) != pongThreshCfg)
        {
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

int32_t ADCBUF_verifyContinuousModeCfg(ADCBuf_Handle handle, uint32_t continuousModeCfg)
{
    int32_t            status = SystemP_SUCCESS;
    uint32_t           continuousMode;
    ADCBuf_Config      *config;
    ADCBuf_Attrs       *attrs;
    CSL_rss_ctrlRegs   *rssCtrlRegs;

    if((handle == (ADCBuf_Handle) NULL)   ||
       (continuousModeCfg > ADCBUF_CONTINUOUS_MODE_MAX))
    {
        status = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Get the Object from ADCBuf Handle */
        config = (ADCBuf_Config *) handle;
        attrs = (ADCBuf_Attrs *) config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /* Read ADCBUFCONTMODEEN field of ADCBUFCFG1 register */
        continuousMode = CSL_FEXT(rssCtrlRegs->ADCBUFCFG1,
                                  RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN);

        if(continuousMode != continuousModeCfg)
        {
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

int32_t ADCBUF_verifyDataFormatCfg(ADCBuf_Handle handle,
                                   uint32_t dataFormatcfg,
                                   uint32_t interleavecfg,
                                   uint32_t iqConfig)
{
    int32_t            status = SystemP_SUCCESS;
    uint32_t           dataFormat;
    uint32_t           interleave;
    uint32_t           iqCfg;
    ADCBuf_Config      *config;
    ADCBuf_Attrs       *attrs;
    CSL_rss_ctrlRegs   *rssCtrlRegs;

    if((handle == (ADCBuf_Handle) NULL)   ||
       (dataFormatcfg > ADCBUF_DATA_FMT_MAX) ||
       (interleavecfg > ADCBUF_WRITEMODE_MAX) ||
       (iqConfig  > ADCBUF_IQSWAP_CFG_MAX))
    {
        status = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Get the Object from ADCBuf Handle */
        config = (ADCBuf_Config *) handle;
        attrs = (ADCBuf_Attrs *) config->attrs;
        rssCtrlRegs = (CSL_rss_ctrlRegs *) attrs->baseAddr;

        /* Read ADCBUFREALONLYMODE Complex Data mode or Real data mode */
        dataFormat = CSL_FEXT(rssCtrlRegs->ADCBUFCFG1,
                              RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE);

        if(dataFormat != dataFormatcfg)
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Read ADCBUFWRITEMODE field for interleave/Non-interleave Mode*/
        interleave = CSL_FEXT(rssCtrlRegs->ADCBUFCFG1,
                              RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE);

        if(interleave != interleavecfg)
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Read ADCBUFWRITEMODE field for interleave/Non-interleave Mode*/
        iqCfg = CSL_FEXT(rssCtrlRegs->ADCBUFCFG1,
                              RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP);

        if(iqCfg != iqConfig)
        {
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

int32_t ADCBUF_readStaticRegs(ADCBuf_Handle handle, ADCBUF_StaticRegs *pStaticRegs)
{
    int32_t                status = SystemP_SUCCESS;
    ADCBuf_Config          *config;
    ADCBuf_Attrs           *attrs;

    if ((handle == (ADCBuf_Handle) NULL) ||
       (pStaticRegs == (NULL_PTR)))
    {
        status = ADCBUF_STATUS_INVALID_PARAMS;
    }
    else
    {
        /* Get the Object from ADCBuf Handle */
        config = (ADCBuf_Config *) handle;
        attrs = config->attrs;

        pStaticRegs->ADCBUFCFG1   = HW_RD_REG32((attrs->baseAddr + CSL_RSS_CTRL_ADCBUFCFG1));
        pStaticRegs->ADCBUFCFG2   = HW_RD_REG32((attrs->baseAddr + CSL_RSS_CTRL_ADCBUFCFG2));
        pStaticRegs->ADCBUFCFG3   = HW_RD_REG32((attrs->baseAddr + CSL_RSS_CTRL_ADCBUFCFG3));
        pStaticRegs->ADCBUFCFG4   = HW_RD_REG32((attrs->baseAddr + CSL_RSS_CTRL_ADCBUFCFG4));
        pStaticRegs->DMMSWINT1    = HW_RD_REG32((attrs->baseAddr + CSL_RSS_CTRL_DMMSWINT1));
        pStaticRegs->CQCFG1       = HW_RD_REG32((attrs->baseAddr + CSL_RSS_CTRL_CQCFG1));

        status = SystemP_SUCCESS;
    }

    return status;
}

