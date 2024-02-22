/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2024
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
/**
 *  @ingroup SDL_MCRC_MODULE
 *  @defgroup SDL_MCRC_API MCRC API
 *  @section MCRC Overview
 *         MCRC programming sequence:
 *         1. SDL_MCRC_channelReset()
 *         2. SDL_MCRC_init()
 *         3. SDL_MCRC_config()
 *         4. SDL_MCRC_enableIntr() - if needed
 *         5. Transfer data to PSA signature for generation of signature value
 *             - DMA is utilized in Semi-CPU mode
 *             - CPU need to pump data into PSA register in Full CPU mode.
 *         6. SDL_MCRC_getPSASectorSig()
 *             - After complete transfer read MCRC signature.
 *
 *         MCRC signature generated depends on amount of data copied into
 *         PSA register at a time/in single write. Though data pattern can be
 *         8, 16, 32, or 64 bit, data copied into PSA register is always 64 bit
 *         wide. If data pattern is less than 64 bit, then it is padded with
 *         zeros to make it 64 bit write.
 *
 *  @{
 */
/**
 *  \file     sdl_mcrc.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of MCRC.
 *            This also contains some related macros.
 */
#ifndef SDL_MCRC_H_
#define SDL_MCRC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_mcrc_hw.h"
#include <sdl/include/sdlr.h>
#include <sdl/mcrc/v0/soc/sdl_mcrc_soc.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * \brief  Macro defines mask for all the interrupts for a channel.
 */
#define SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL   \
    (uint32_t) ((uint32_t)SDL_MCRC_INTS_CH1_CCITENS_MASK |  \
                (uint32_t)SDL_MCRC_INTR_CH1_FAILENR_MASK |  \
                (uint32_t)SDL_MCRC_INTR_CH1_OVERENR_MASK |  \
                (uint32_t)SDL_MCRC_INTR_CH1_UNDERENR_MASK | \
                (uint32_t)SDL_MCRC_INTR_CH1_TIME_OUT_ENR_MASK)

/**
 * \brief  Macro defines maximum value of MCRC Pattern Count.
 */
#define SDL_MCRC_PATTERN_COUNT_MAX          (0x000FFFFFU)

/**
 * \brief  Macro defines maximum value of MCRC Sector Count.
 */
#define SDL_MCRC_SECTOR_COUNT_MAX           (0x0000FFFFU)

/**
 * \brief  Macro defines maximum value of MCRC Block Complete Timeout Counter
 *         Preload.
 */
#define SDL_MCRC_BCTOPLD_MAX                (0x00FFFFFFU)

/**
 * \brief  Macro defines maximum value of MCRC Watchdog Timeout Counter Preload.
 */
#define SDL_MCRC_WDTOPLD_MAX                (0x00FFFFFFU)

/**
 * \brief  Macro defines value of MCRC Control Register.
 */

#define SDL_MCRC_TYPE_16BIT                 (0x4CU)

#define SDL_MCRC_TYPE_32BIT                 (0x54U)

#define SDL_MCRC_TYPE_64BIT                 (0x44U)

#define SDL_MCRC_DATAWIDTH_SEL_64BIT        (0x00U)

#define SDL_MCRC_DATAWIDTH_SEL_16BIT        (0x01U)

#define SDL_MCRC_DATAWIDTH_SEL_32BIT        (0x02U)

/**
 * \brief  Max number of channels supported in MCRC.
 */
#define SDL_MCRC_MAX_NUM_OF_CHANNELS        (4U)

/**
 *  \anchor SDL_MCRC_ModeType
 *  \name MCRC Operation Mode
 *  @{
 */

/**
 * \brief  MCRC operation mode supported. MCRC can either operate
 *         in Semi-CPU, Full-CPU or Auto mode.
 */
typedef uint32_t SDL_MCRC_ModeType;

#define SDL_MCRC_OPERATION_MODE_DATA        (SDL_MCRC_CTRL2_CH1_MODE_DATA)
/**< Configure MCRC operation mode to Data Captures */
#define SDL_MCRC_OPERATION_MODE_AUTO        (SDL_MCRC_CTRL2_CH1_MODE_AUTO)
/**< Configure MCRC operation mode to Auto */
#define SDL_MCRC_OPERATION_MODE_SEMICPU     (SDL_MCRC_CTRL2_CH1_MODE_SEMICPU)
/**< Configure MCRC operation mode to Semi-CPU */
#define SDL_MCRC_OPERATION_MODE_FULLCPU     (SDL_MCRC_CTRL2_CH1_MODE_FULLCPU)
/**< Configure MCRC operation mode to Full-CPU */
/** @} */

/**
 *  \anchor SDL_MCRC_Channel_t
 *  \name MCRC channel
 *  @{
 */

/**
 * \brief  MCRC channel supported.
 */
typedef uint32_t SDL_MCRC_Channel_t;
#define SDL_MCRC_CHANNEL_1                  (0x1U)
/**< Select channel 1 for operation */
#define SDL_MCRC_CHANNEL_2                  (0x2U)
/**< Select channel 2 for operation */
#define SDL_MCRC_CHANNEL_3                  (0x3U)
/**< Select channel 3 for operation */
#define SDL_MCRC_CHANNEL_4                  (0x4U)
/**< Select channel 4 for operation */
/** @} */

/**
 * \brief This enumerator defines the Data size for input MCRC DATA
 *
 */
typedef enum {
    SDL_MCRC_DATA_8_BIT = 1,
    /**< 8 Bit data packed */
    SDL_MCRC_DATA_16_BIT = 2,
    /**< 16 Bit data packed */
    SDL_MCRC_DATA_32_BIT = 3,
    /**< 32 Bit data packed */
} SDL_MCRC_DataBitSize;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief  Structure for accessing MCRC register data which are 64 bit wide.
 */
typedef struct SDL_MCRC_Signature
{
    uint32_t regL;
    /**< Lower 32 bits of the 64 bit data of MCRC Signature register */
    uint32_t regH;
    /**< Upper 32 bits of the 64 bit data of MCRC Signature register */
}SDL_MCRC_Signature_t;

/**
 * \brief  Structure for accessing MCRC registers address which are 64 bit wide.
 */
typedef struct SDL_MCRC_SigantureRegAddr
{
    uint32_t regL;
    /**< Lower 32 bits of the 64 bit MCRC Signature register address */
    uint32_t regH;
    /**< Upper 32 bits of the 64 bit MCRC Signature register address */
}SDL_MCRC_SignatureRegAddr_t;

/**
 * \brief  MCRC channel static registers list.
 */
typedef struct {
    volatile uint32_t PCOUNT;
    /**< Pattern Counter Preload Register */
    volatile uint32_t SCOUNT;
    /**< Sector Counter Preload Register */
    volatile uint32_t WDTOPLD;
    /**< Watchdog Timeout Counter Preload Register */
    volatile uint32_t BCTOPLD;
    /**< Block Complete Timeout Counter Preload Register */
} SDL_MCRC_ChannelStaticRegs;

/**
 * \brief  MCRC static registers list.
 */
typedef struct {
    SDL_MCRC_ChannelStaticRegs channelRegs[SDL_MCRC_MAX_NUM_OF_CHANNELS];
    /**< Channel Specific Static Registers */
    volatile uint32_t CTRL0;
    /**< CTRL0 Register */
    volatile uint32_t CTRL1;
    /**< CTRL1 Register */
    volatile uint32_t BUS_SEL;
    /**< Data Bus Tracing Select Register */
} SDL_MCRC_StaticRegs_t;

/**
 * \brief  Structure for MCRC CPU inputs data
 */
typedef struct SDL_MCRC_DataConfig
{
    uint32_t *pMCRCData;
    /**< Pointer to Data used for MCRC  */
    uint32_t size;
    /**< Size of Data in Bytes  */
    SDL_MCRC_DataBitSize dataBitSize;
    /**< Data Bit size  */
} SDL_MCRC_DataConfig_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Initialize MCRC channel and will configure watchdog and
 *          block preload value for given channel.
 *
 * \param   instance          MCRC instance either MCU or Main.
 *
 * \param   channel         Channel number to be initializaed.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param  watchdogPreload
 *                          It is used to check if DMA does supply a block of
 *                          data responding to a request in a given time frame.
 * \param   blockPreload
 *                          It is used to check if MCRC for an entire block is
 *                          completed in a given time frame.
 *
 * \return  status          MCRC channel initialization status.
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 *
 */
int32_t SDL_MCRC_init(SDL_MCRC_InstType     instance,
                      SDL_MCRC_Channel_t    channel,
                      uint32_t              watchdogPreload,
                      uint32_t              blockPreload);

/**
 * \brief   Verify the MCRC watchdog and block preload value initialized for given
 *          channel.
 *
 * \param   instance          MCRC instance either MCU or Main.
 *
 * \param   channel           Channel number to be verified initialization.
 *                            Values given by #SDL_MCRC_Channel_t.
 * \param   watchdogPreload   It is used to check if DMA does supply a block of
 *                            data responding to a request in a given time frame.
 * \param   blockPreload It is used to check if MCRC for an entire block is
 *                            completed in a given time frame.
 *
 * \return  status           MCRC channel verfiy initialization status.
 *                           SDL_PASS:     success
 *                           SDL_EBADARGS: failure, indicate the bad input arguments
 *                           SDL_EFAIL:    failure, indicate verify initialization failed
 *
 *
 */
int32_t SDL_MCRC_verifyInit(SDL_MCRC_InstType     instance,
                            SDL_MCRC_Channel_t    channel,
                            uint32_t              watchdogPreload,
                            uint32_t              blockPreload);

/**
 * \brief   This API will configure MCRC mode, pattern and sector count for
 *          given channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number to be configured.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   patternCount    Number of data patterns in one sector to be compressed.
 * \param   sectorCount     Number of sectors in a block of memory.
 * \param   mode            MCRC operational mode.
 *                          Refer #SDL_MCRC_ModeType.
 *
 * \return  status          MCRC channel configuration status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_config(SDL_MCRC_InstType     instance,
                        SDL_MCRC_Channel_t    channel,
                        uint32_t              patternCount,
                        uint32_t              sectorCount,
                        SDL_MCRC_ModeType     mode);

/**
 * \brief   This API will verify the configure of MCRC mode, pattern and sector
 *           count for given channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number to be verified configuration.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   patternCount    Number of data patterns in one sector to be compressed.
 * \param   sectorCount     Number of sectors in a block of memory.
 * \param   mode            MCRC operational mode.
 *                          Refer #SDL_MCRC_ModeType.
 *
 * \return  status          MCRC channel configuration status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *                          SDL_EFAIL:    failure, indicate verify configure failed
 *
 *
 */
int32_t SDL_MCRC_verifyConfig(SDL_MCRC_InstType     instance,
                              SDL_MCRC_Channel_t    channel,
                              uint32_t              patternCount,
                              uint32_t              sectorCount,
                              SDL_MCRC_ModeType     mode);

/**
 * \brief   This API is used to reset the MCRC channel
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which reset is to be done.
 *                          Values given by #SDL_MCRC_Channel_t.
 *
 * \return  status          MCRC channel reset status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_channelReset(SDL_MCRC_InstType  instance,
                              SDL_MCRC_Channel_t channel);

/**
 * \brief   This API is used to set the PSA seed value without compression
 *          for given Channel.
 *
 * \param   instance          MCRC instance either MCU or Main.
 * \param   channel           Channel number for which PSA seed value to be set.
 *                            Values given by #SDL_MCRC_Channel_t.
 * \param   pSeedSign         Pointer to MCRC PSA seed signature values.
 *                            Refer struct #SDL_MCRC_Signature_t for details.
 *
 * \return  status            MCRC channel set PSA seed value status
 *                            SDL_PASS:     success
 *                            SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_setPSASeedSig(SDL_MCRC_InstType           instance,
                               SDL_MCRC_Channel_t          channel,
                               const SDL_MCRC_Signature_t *pSeedSign);

/**
 * \brief   This API is used to get sector signature Value/
 *          MCRC value for given channel.
 *
 * \param   instance          MCRC instance either MCU or Main.
 * \param   channel           Channel number which was configured for MCRC calculation.
 *                            Values given by #SDL_MCRC_Channel_t.
 * \param   pSecSign          Pointer to MCRC PSA sector signature values
 *                            Refer struct #SDL_MCRC_Signature_t.
 *
 * \return  status            MCRC channel get PSA sector signaure value status
 *                            SDL_PASS:     success
 *                            SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_getPSASectorSig(SDL_MCRC_InstType     instance,
                                 SDL_MCRC_Channel_t    channel,
                                 SDL_MCRC_Signature_t *pSecSign);
/**
 * \brief   This API is used to get the pending interrupts for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which pending interrupt is to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 *
 * \param   pIntrStatus     Pointer to pending interrupt status/occurred.
 *
 * \return  status          MCRC get pending interrupts status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_getIntrStatus(SDL_MCRC_InstType     instance,
                               SDL_MCRC_Channel_t    channel,
                               uint32_t             *pIntrStatus);

/**
 * \brief   This API is used to enable interrupts for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which interrupt is to enable.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   intrMask        Interrupts to enable.
 *
 * \return  status          MCRC enable interrupts status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_enableIntr(SDL_MCRC_InstType     instance,
                            SDL_MCRC_Channel_t    channel,
                            uint32_t              intrMask);

/**
 * \brief   This API is used to disable interrupts for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which interrupt is to disable.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   intrMask        Interrupts to disable.
 *
 * \return  status          MCRC disable interrupts status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_disableIntr(SDL_MCRC_InstType  instance,
                             SDL_MCRC_Channel_t channel,
                             uint32_t           intrMask);

/**
 * \brief   This API is used to clear interrupts for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which interrupt is to clear.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   intrMask        Interrupts to clear status.
 *
 * \return  status          MCRC clear interrupts status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_clearIntr(SDL_MCRC_InstType  instance,
                           SDL_MCRC_Channel_t channel,
                           uint32_t           intrMask);

/**
 * \brief   This API is used to check if MCRC is busy for given Channel.
 *
 *   @n  During Auto or Semi-CPU mode, the busy flag is set when the first data
 *       pattern of the block is compressed and remains set until the the last
 *       data pattern of the block is compressed. The flag is cleared when the
 *       last data pattern of the block is compressed.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which busy status is to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 *
 * \param   pBusyFlag       Pointer to busy flag.
 *                          1U: MCRC channel is busy.
 *                          0U: MCRC channel is free.
 *
 * \return  status          MCRC channel check busy status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_isBusy(SDL_MCRC_InstType   instance,
                        SDL_MCRC_Channel_t  channel,
                        uint32_t           *pBusyFlag);

/**
 * \brief   This API is used to get the current sector number of which the
 *          signature verification fails in AUTO mode for given channel
 *
 *   @n  When a sector fails, the erroneous sector number is logged and the MCRC
 *       fail interrupt is generated.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which the curent sector number is to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   pCurSecNum      Pointer to current sector number.
 *
 * \return  status          MCRC channel get current sector number status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_getCurSecNum(SDL_MCRC_InstType     instance,
                              SDL_MCRC_Channel_t    channel,
                              uint32_t             *pCurSecNum);

/**
 * \brief   This API is used to get current known good signature value/
 *          MCRC value for given channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which current MCRC value is to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   pPSAsig         Pointer to current MCRC PSA signature values
 *                          Refer struct #SDL_MCRC_Signature_t.
 *
 * \return  status          MCRC channel get current PSA signaure value status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_getPSASig(SDL_MCRC_InstType     instance,
                           SDL_MCRC_Channel_t    channel,
                           SDL_MCRC_Signature_t *pPSAsig);

/**
 * \brief   This API is used to read static registers of MCRC module.
 *          This API needs to be called after the initial configuration is done and
 *          hence mutliple read between static registers do not change the values
 *
 * \param   instance        MCRC instance either MCU or Main.
 *
 * \param   pStaticRegs     pointer to static registers to be read
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 */
int32_t SDL_MCRC_readStaticReg (SDL_MCRC_InstType     instance,
                                SDL_MCRC_StaticRegs_t *pStaticRegs);

/**
 * \brief   This API is used to get current known good signature value/
 *          MCRC value for given channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which current MCRC value is to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   pCurPSASig      Pointer to current MCRC PSA signature values
 *                          Refer struct #SDL_MCRC_Signature_t.
 *
 * \return  status          MCRC channel get current PSA signaure value status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */

int32_t SDL_MCRC_getCurPSASig(SDL_MCRC_InstType     instance,
                              SDL_MCRC_Channel_t    channel,
                              SDL_MCRC_Signature_t *pCurPSASig);
/**
 * \brief   This API is used to get the PSA register address for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which PSA register address is
 *                          to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   pMCRCregAddr    Pointer to MCRC PSA register address structure.
 *                          Refer structure #SDL_MCRC_SignatureRegAddr_t.
 *
 * \return  status          MCRC channel get PSA signature register address status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
 int32_t SDL_MCRC_getPSASigRegAddr(SDL_MCRC_InstType instance, SDL_MCRC_Channel_t channel,
                                   SDL_MCRC_SignatureRegAddr_t *pMCRCregAddr);

/**
 *
 * \brief   This API is used to compute the signature for CPU-only mode and it check if the generated MCRC signature value
 *          matches with the reference signature value.
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel: MCRC channel number to be used
 * \param   pDataConfig: Pointer to data configuration
 * \param   sectSignVal: Generated sector signature value.
 *
 * \return  SDL_PASS: Success; SDL_FAIL: Failure; SDL_EBADARGS: Bad arguments error
 */
int32_t SDL_MCRC_computeSignCPUmode (SDL_MCRC_InstType instance,
                                     SDL_MCRC_Channel_t channel,
                                     const SDL_MCRC_DataConfig_t *pDataConfig,
                                     SDL_MCRC_Signature_t *sectSignVal);
/**
 * \brief   This API is used to get the MCRC register address for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which MCRC register address is
 *                          to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   pMCRCregAddr    Pointer to MCRC register address structure.
 *                          Refer structure #SDL_MCRC_SignatureRegAddr_t.
 *
 * \return  status          MCRC channel get MCRC signature register address status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_getCRCRegAddr(SDL_MCRC_InstType instance,
                          SDL_MCRC_Channel_t           channel,
                          SDL_MCRC_SignatureRegAddr_t *pMCRCregAddr);
/**
 * \brief   This API is used to configure the MCRC type for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number
 *
 * \return  status          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_configCRCType(SDL_MCRC_InstType instance,
					 SDL_MCRC_Channel_t       channel);

/**
 * \brief   This API is used to configure the MCRC data width for given Channel.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   channel         Channel number for which MCRC register address is
 *                          to get.
 *                          Values given by #SDL_MCRC_Channel_t.
 * \param   datawidth       DataWidth
 *                          Values given by macro SDL_MCRC_DATAWIDTH_SEL_xxBIT.
 *
 * \return  status          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input
 *                                        arguments
 *
 */
int32_t SDL_MCRC_configDataWidth(SDL_MCRC_InstType instance,
					           SDL_MCRC_Channel_t channel, uint32_t datawidth);

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif

/** @} */
