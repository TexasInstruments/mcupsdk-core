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
 *  \defgroup DRV_ADCBUF_MODULE APIs for ADCBUF
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the ADCBUF module.
 *
 *  @{
 */

/**
 *  \file v0/adcbuf.h
 *
 *  \brief This file contains the prototypes of the APIs present in the
 *         device abstraction layer file of ADCBUF.
 *         This also contains some related macros.
 */

#ifndef ADCBUF_V0_H_
#define ADCBUF_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** \brief ADCBUF driver error base */
#define MMWAVE_ERRNO_ADCBUF_BASE        (-2400)

/**
 *  \defgroup ADCBUF_ERROR_CODES Error Codes
 *  \ingroup DRV_ADCBUF_MODULE
 *
 * @{
 */

/**
 * \brief   Generic error status code returned by ADCBuf_control().
 *
 * ADCBuf_control() returns ADCBuf_STATUS_ERROR if the control code was not executed
 * successfully.
 */
#define ADCBUF_STATUS_ERROR             (MMWAVE_ERRNO_ADCBUF_BASE - 1)
/**
 * \brief   An error status code returned by ADCBuf_control() for undefined
 * command codes.
 *
 * ADCBuf_control() returns ADCBuf_STATUS_UNDEFINEDCMD if the control code is not
 * recognized by the driver implementation.
 */
#define ADCBUF_STATUS_UNDEFINEDCMD      (MMWAVE_ERRNO_ADCBUF_BASE - 2)
/**
 * \brief   An error status code returned by ADCBuf_control() for feature not implemented.
 *
 * ADCBuf_control() returns ADCBuf_STATUS_NOT_IMPLEMENTED if the control command
 * was not supported.
 */
#define ADCBUF_STATUS_NOT_IMPLEMENTED   (MMWAVE_ERRNO_ADCBUF_BASE - 3)
/**
 * \brief   An error status code returned by ADCBuf_control() for invalid input parameters.
 *
 * ADCBuf_control() returns ADCBuf_STATUS_INVALID_PARAMS if the control code gets
 * invalid parameters.
 */
#define ADCBUF_STATUS_INVALID_PARAMS    (MMWAVE_ERRNO_ADCBUF_BASE - 4)
/**
 * \brief   An error status code returned by ADCBuf_control() for invalid
 * command codes.
 *
 * ADCBuf_control() returns ADCBuf_STATUS_INVALID_CMD if the control code is not
 * recognized by the driver implementation.
 */
#define ADCBUF_STATUS_INVALID_CMD       (MMWAVE_ERRNO_ADCBUF_BASE - 5)
/** @} */

/**
 * \brief ADCBUF CQ monitoring type
 * \details used to define CQ monitoring types.
 */
typedef uint32_t ADCBufMMWave_CQType;

#define ADCBufMMWave_CQType_CQ0         ((uint32_t) 0) /**< CQ type for CQ0. */
#define ADCBufMMWave_CQType_CQ1         ((uint32_t) 1) /**< CQ type for CQ1. */
#define ADCBufMMWave_CQType_CQ2         ((uint32_t) 2) /**< CQ type for CQ2. */
#define ADCBufMMWave_CQType_MAX_CQ      ((uint32_t) 3) /**< Maximum number of CQ which can be supported. */

/**
 * \brief  Macro defines maximum value of Source Selection.
 */
#define ADCBUF_SOURCE_SELECT_MAX         ((uint32_t)(0x1U))

/**
 * \brief  Macro defines maximum value of Number of Chirps in
 *         PING memory.
 */
#define ADCBUF_PING_THRESHOLD_MAX         ((uint32_t)(0x1FU))

/**
 * \brief  Macro defines maximum value of Number of Chirps in
 *         PONG memory.
 */
#define ADCBUF_PONG_THRESHOLD_MAX         ((uint32_t)(0x1FU))

/**
 * \brief  Macro defines maximum value of CONTINUOUS mode enable.
 */
#define ADCBUF_CONTINUOUS_MODE_MAX         ((uint32_t)(0x1U))

/**
 * \brief  Macro defines maximum value of DATA FORMAT Complex
 *         data or Real data mode.
 */
#define ADCBUF_DATA_FMT_MAX                 ((uint32_t)(0x1U))

/**
 * \brief  Macro defines maximum value of Write mode interleave or
 *         Non-interleave.
 */
#define ADCBUF_WRITEMODE_MAX                 ((uint32_t)(0x1U))

/**
 * \brief  Macro defines maximum value of IQSWAP mac value.
 */
#define ADCBUF_IQSWAP_CFG_MAX                 ((uint32_t)(0x1U))

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief A handle that is returned from an ADCBuf_open() call */
typedef void *ADCBuf_Handle;

/**
 *  \addtogroup ADCBUF_DRIVER_EXTERNAL_DATA_STRUCTURE      ADCBUF Driver External Data Structures
 *  \ingroup DRV_ADCBUF_MODULE
 *
 * @{
 */

/**
 * \brief ADCBUF Source selection
 * \details The structure is used to define ADCBUF driver commands.
 */
typedef enum ADCBufSource_e
{
    /** \brief Select DFE as source */
    ADCBUF_SOURCE_DFE = 0,
    /** \brief Select HIL as source */
    ADCBUF_SOURCE_HIL
} ADCBufSource;

/**
 * \brief ADCBUF Command
 * \details The structure is used to define ADCBUF driver commands.
 */
typedef enum ADCBufMMWave_CMD_e
{
    /**
     * \brief   set ADCBUF source, it can be DFE or VIN.
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_SET_SRC    = 0,
    /**
     * \brief   Set number of Chirps to be stored in each Ping and Pong buffer.
     *          It should be programmed one less the actual number needed .
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD,
    /**
     * \brief   Set number of Chirps to be stored in each Ping buffer.
     *          It should be programmed one less the actual number needed .
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
    /**
     * \brief   Set number of Chirps to be stored in each  Pong buffer.
     *          It should be programmed one less the actual number needed .
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
    /**
     * \brief   Enables/Disables Continuous mode for ADCBUF.
     *             1 to enable continuous mode.
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_SET_CONTINUOUS_MODE,
    /**
     * \brief   Starts Continuous mode for ADCBUF.
     *          Number of Samples to store in Ping/Pong buffer needs to be provided.
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_START_CONTINUOUS_MODE,
    /**
     * \brief   Stops Continuous mode for ADCBUF.
     *          The size of the argument size for this command is 0 bytes
     */
    ADCBufMMWave_CMD_STOP_CONTINUOUS_MODE,
    /**
     * \brief   Configures ADCBUF data format.
     *          The size of the argument size for this command is size of \ref ADCBuf_dataFormat
     */
    ADCBufMMWave_CMD_CONF_DATA_FORMAT,
    /**
     * \brief   Enable RX channels and configures the address offset in ADCBUF for the channel.
     *          The size of the argument size for this command is size of \ref ADCBuf_RxChanConf
     */
    ADCBufMMWave_CMD_CHANNEL_ENABLE,
    /**
     * \brief   Disable RX channels specified with RX channel bitmask
     *          The size of the argument size for this command is 4 bytes
     */
    ADCBufMMWave_CMD_CHANNEL_DISABLE,
    /**
     * \brief   Test pattern configuration.
     *          The size of the argument size for this command is size of \ref ADCBuf_TestPatternConf
     */
    ADCBufMMWave_CMD_CONF_TEST_PATTERN,
    /**
     * \brief   Starts Test pattern generation. Reboot is required when switching from Test pattern mode
     *          to normal operation mode.
     *          The size of the argument size for this command is 0 bytes
     */
    ADCBufMMWave_CMD_START_TEST_PATTERN,
    /**
     * \brief   Stops Test pattern generation.
     *          The size of the argument size for this command is 0 bytes
     */
    ADCBufMMWave_CMD_STOP_TEST_PATTERN,
    /**
     * \brief   Chirp Quality configuration.
     *          The size of the argument size for this command is size of \ref ADCBuf_CQConf
     */
    ADCBufMMWave_CMD_CONF_CQ,
    /**
     * \brief  Last command.
     */
    ADCBufMMWave_CMD_LAST
} ADCBufMMWave_CMD;

/**
 * \brief ADC Buffer data format Parameters
 * \details The structure is used to define ADC Buffer data format.
 */
typedef struct ADCBuf_dataFormat_t
{
    /** \brief ADC out Format
         0 - Complex Data mode
         1 - Real data mode
      */
    uint8_t     adcOutFormat;
    /** \brief Sample interleave - IQswap
         0 - I is stored in LSB, Q is stored in MSB
         1 - Q is stored in LSB, I is stored in MSB
      */
    uint8_t     sampleInterleave;
    /** \brief channel interleave
         0 - interleaved
         1 - non-interleaved
      */
    uint8_t     channelInterleave;
} ADCBuf_dataFormat;

/**
 * \brief ADC Buffer RX channel configuration Parameters
 * \details The structure is used to define ADC Buffer RX Channel configuration.
 */
typedef struct ADCBuf_RxChanConf_t
{
    /** \brief RX channel id - 0~3 */
    uint8_t   channel;
    /** \brief Address offset for the channel in Ping/Pong buffer
      Used only in non-interleaved mode, it must be 16 bytes aligned.
     */
    uint16_t  offset;
} ADCBuf_RxChanConf;

/**
 * \brief ADC Buffer test pattern configuration Parameters
 * \details The structure is used to define ADC Buffer test pattern configuration.
 */
typedef struct ADCBuf_rxTestPatternConf_t
{
    /** \brief I sample offset */
    uint16_t rxIOffset;
    /** \brief I sample incremental value */
    uint16_t rxIInc;
    /** \brief Q sample offset */
    uint16_t rxQOffset;
    /** \brief Q sample incremental value */
    uint16_t rxQInc;
} ADCBuf_rxTestPatternConf;

/**
 * \brief ADC Buffer test pattern configuration Parameters
 * \details The structure is used to define ADC Buffer test pattern configuration.
 */
typedef struct ADCBuf_TestPatternConf_t
{
    /** \brief Test pattern configuration for 4 channels */
    ADCBuf_rxTestPatternConf rxConfig[SOC_ADCBUF_NUM_RX_CHANNEL];
    /** \brief Periodicity of the pattern */
    uint16_t               period;
    /** \brief Sample count to store in ADC buffer */
    uint16_t               numSamples;
} ADCBuf_TestPatternConf;

/**
 * \brief Chirp Quality(CQ) configuration Parameters
 * \details The structure is used to define Chirp Quality configuration.
 */
typedef struct ADCBuf_CQConf_t
{
    /** \brief 0x0 or 0x1:Raw16, 0x2:Raw12, 0x3:Raw14 */
    uint8_t   cqDataWidth;
    /** \brief Set in case of 3 channel mode  */
    uint8_t   cq96BitPackEn;
    /** \brief CQ0 Address offset : 16bytes aligned address for Storing CQ0 */
    uint16_t  cq0AddrOffset;
    /** \brief CQ1 Address offset : 16 bytes aligned address for Storing CQ1 */
    uint16_t  cq1AddrOffset;
    /** \brief CQ2 Address offset : 16 bytes aligned address for Storing CQ2 */
    uint16_t  cq2AddrOffset;
} ADCBuf_CQConf;

/**
 *  \brief ADC Parameters
 *
 *  ADC Parameters are used to with the ADCBuf_open() call. Default values for
 *  these parameters are set using ADCBuf_Params_init().
 */
typedef struct ADCBuf_Params_t
{
    /** \brief   ADC buffer source, DFE or HIL */
    ADCBufSource        source;
    /** \brief   Continuous mode selection */
    uint8_t             continousMode;
    /** \brief   AWR2944: Ping buffer Chirp Threshold for non-continous operation */
    uint8_t             chirpThresholdPing;
    /** \brief   xwr16xx/xwr18xx/xwr68xx: Pong buffer Chirp Threshold for non-continous operation */
    uint8_t             chirpThresholdPong;
    /** \brief   Custom configuration, Pointer to a device specific extension of the ADCBuf_Params */
    void                *custom;
} ADCBuf_Params;

/** @} */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/** \brief ADCBuf instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t            baseAddr;
    /**< Peripheral base address */
    uint32_t            interruptNum;
    /**< Interrupt Number */
    uint32_t            adcbufBaseAddr;
    /**< ADC Buffer base address */
    uint32_t            cqbufBaseAddr;
    /**< CQ Buffer base address */
} ADCBuf_Attrs;

/**
 *  \brief ADCBuf driver object
 */
typedef struct
{
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
} ADCBuf_Object;

/**
 *  \brief ADCBuf Global configuration
 *
 *  The ADCBuf_Config structure contains a set of pointers used to characterise
 *  the ADC driver implementation.
 *
 *  This structure needs to be defined before calling ADCBuf_init() and it must
 *  not be changed thereafter.
 */
typedef struct
{
    ADCBuf_Attrs       *attrs;
    /**< Pointer to driver specific attributes */
    ADCBuf_Object      *object;
    /**< Pointer to driver specific data object */
} ADCBuf_Config;

/** \brief Externally defined driver configuration array */
extern ADCBuf_Config gADCBufConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t      gADCBufConfigNum;

/**
 * \brief  ADCBUF static registers list.
 */
typedef struct
{
    volatile uint32_t ADCBUFCFG1;
    /**< ADCBUFCFG1 Register */
    volatile uint32_t ADCBUFCFG2;
    /**< ADCBUFCFG4 Register */
    volatile uint32_t ADCBUFCFG3;
    /**< ADCBUFCFG3 Register */
    volatile uint32_t ADCBUFCFG4;
    /**< ADCBUFCFG4 Register */
    volatile uint32_t DMMSWINT1;
    /**< DMMSWINT1 Register */
    volatile uint32_t CQCFG1;
    /**< CQCFG1 Register */
} ADCBUF_StaticRegs;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \defgroup ADCBUF_DRIVER_EXTERNAL_FUNCTION       ADCBUF Driver External Functions
 *  \ingroup DRV_ADCBUF_MODULE
 *  \brief The section has a list of all the exported API which the applications need to
 *   invoke in order to use the driver
 * @{
 */
/**
 *  \brief Description
 *  \n
 *      This function initializes the ADC module. This function must be called
 *      before any other functions are called.
 *
 *  \param[in] timeout Amount of time in units of ticks to wait
 */
void ADCBuf_init(uint32_t timeout);

/**
 *  \brief Driver deinit function
 */
void ADCBuf_deinit(void);

/**
 *  \brief Description
 *  \n
 *      This function sets all fields of a specified ADCBuf_Params structure to their
 *      default values.
 *
 *  \param[in]  params      A pointer to ADCBuf_Params structure for initialization
 *
 *  Default values are:
 *                      source             = ADCBUF_SOURCE_DFE,
 *                      continousMode      = 0;
 *                      chirpThresholdPing = 1,
 *                      chirpThresholdPong = 1,
 *                      custom             = NULL
 *
 */
void ADCBuf_Params_init(ADCBuf_Params *params);

/**
 *  \brief Description
 *  \n
 *      This function opens a given ADCBuf peripheral.
 *
 *  \param[in]  index   Logical peripheral number for the ADCBuf indexed into
 *                      the ADCBuf_config table
 *
 *  \param[in]  params  Pointer to an parameter block, if NULL it will use
 *                      default values.
 *
 *  \return An ADCBuf_Handle on success or a NULL on an error or if it has been
 *          opened already. If NULL is returned further ADC API calls will
 *          result in undefined behaviour.
 */
ADCBuf_Handle ADCBuf_open(uint8_t index, const ADCBuf_Params *params);

/**
 *  \brief Description
 *  \n Function to close an ADC peripheral specified by the ADC handle
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 */
void ADCBuf_close(ADCBuf_Handle handle);

/**
 *  \brief Description
 *  \n
 *      Function performs implementation specific features on a given
 *      ADCBuf_Handle.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *  \param[in] cmd
 *      A command value defined by the driver specific implementation \ref ADCBufMMWave_CMD. \n
 *  \param[in] arg
 *      A pointer to an optional R/W (read/write) argument that is accompanied with cmd.
 *      arg should be 4 bytes aligned.
 *  \return Success     - \ref SystemP_SUCCESS
 *          Error       - one of \ref ADCBUF_ERROR_CODES
 */
int32_t ADCBuf_control(ADCBuf_Handle handle, uint8_t cmd, void *arg);

/**
 *  \brief Description
 *  \n
 *      This function gets the physical address of ADCBuf for a given receive channel.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *  \param[in] channel
 *      Receive channel number.
 *  \param[in] errCode
 *      Pointer to an error code populated by the driver.
 *  \return Success     - \ref SystemP_SUCCESS
 *          Error       - one of \ref ADCBUF_ERROR_CODES
 */
uint32_t ADCBuf_getChanBufAddr(ADCBuf_Handle handle, uint8_t channel, int32_t *errCode);

/**
 *  \brief Description
 *  \n
 *      This function gets the physical address of chirp info(CQ) buffer for a given CQ type.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *  \param[in] cqType
 *      Type of CQ that request the address.
 *  \param[in] errCode
 *      Pointer to an error code populated by the driver.
 *  \return Success     - \ref SystemP_SUCCESS
 *          Error       - one of \ref ADCBUF_ERROR_CODES
 */
uint32_t ADCBUF_MMWave_getCQBufAddr(ADCBuf_Handle handle, ADCBufMMWave_CQType cqType, int32_t *errCode);

/**
 *  \brief Description
 *  \n
 *      This API verifies source selection configuration for ADCBUF peripheral.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *
 *  \param[in]  source  Source selected for ADCBUF peripheral read from DMMSWINT1 register
 *                      0 -> Write to ADC BUF memory will happen from DFE
 *                      1 -> Write to CQ memory will happen from ADCBUF_W slave port in DSS interconnect
 *                           using DMM as master.
 *
 *  \return  status   ADCBUF channel configuration status
 *                    SystemP_SUCCESS:     success
 *                    ADCBUF_STATUS_INVALID_PARAMS: failure, indicate the bad input arguments
 *                    SystemP_FAILURE: failure, indicate verification failed
 */
int32_t ADCBUF_verifySrcSelCfg(ADCBuf_Handle handle, uint32_t source);

/**
 *  \brief Description
 *  \n
 *      This API verifies ping threshold configuration for ADCBUF peripheral.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *
 *  \param[in]  pingThreshCfg Number of chirps to be stored in Ping Buffer
 *
 *  \param[in]  pongThreshCfg Number of chirps to be stored in Pong Buffer
 *
 *  \return  status    ADCBUF channel configuration status
 *                     SystemP_SUCCESS:     success
 *                     ADCBUF_STATUS_INVALID_PARAMS: failure, indicate the bad input arguments
 *                     SystemP_FAILURE: failure, indicate verification failed
 */
int32_t ADCBUF_verifyChirpThreshold(ADCBuf_Handle handle, uint32_t pingThreshCfg, uint32_t pongThreshCfg);

/**
 *  \brief Description
 *  \n
 *      This API verifies continuous mode configuration for ADCBUF peripheral.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *
 *  \param[in]  continuousModeCfg continuous mode config for ADCBUF peripheral
 *
 *  \return  status    ADCBUF channel configuration status
 *                     SystemP_SUCCESS:     success
 *                     ADCBUF_STATUS_INVALID_PARAMS: failure, indicate the bad input arguments
 *                     SystemP_FAILURE: failure, indicate verification failed
 */
int32_t ADCBUF_verifyContinuousModeCfg(ADCBuf_Handle handle, uint32_t continuousModeCfg);

/**
 *  \brief Description
 *  \n
 *      This API will verify the configuration of ADCBUF, dataFormat and interleave
 *      iqConfig.
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *
 *  \param[in]  dataFormat Complex Data mode or Real data mode
 *
 *  \param[in]  interleave ADCBUF Write mode interleave or Non-interleave
 *
 *  \param[in]  iqConfig IQSwap configuration
 *
 *  \return  status    ADCBUF channel configuration status
 *                     SystemP_SUCCESS:     success
 *                     ADCBUF_STATUS_INVALID_PARAMS: failure, indicate the bad input arguments
 *                     SystemP_FAILURE: failure, indicate verification failed
 */
int32_t ADCBUF_verifyDataFormatCfg(ADCBuf_Handle handle, uint32_t dataFormat, uint32_t interleave, uint32_t iqConfig);


/**
 *  \brief Description
 *  \n
 *     This API is used to read static registers of ADCBUF module.
 *     This API needs to be called after the initial configuration is done and
 *     hence mutliple read between static registers do not change the values
 *
 *  \param[in]  handle
 *      Handle to the ADCBUF instance obtained through call to \ref ADCBuf_open.
 *
 *  \param   pStaticRegs     pointer to static registers to be read
 *
 *
 *  \return
 *                     SystemP_SUCCESS:     success
 *                     ADCBUF_STATUS_INVALID_PARAMS: failure, indicate the bad input arguments
 */
int32_t ADCBUF_readStaticRegs(ADCBuf_Handle handle, ADCBUF_StaticRegs *pStaticRegs);

/** @} */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef ADCBUF_V0_H_ */

/** @} */
