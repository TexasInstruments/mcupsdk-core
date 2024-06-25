/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     ecc_func.h
 *
 *  \brief    This file contains ECC functions defines.
 *
 *  \details  ECC functional tests
 **/
 
#ifndef ECC_FUNC_H
#define ECC_FUNC_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>
/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_R5FSS0_CORE0_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_R5FSS0_CORE0_ECC_AGGR_subMemTypeList[SDL_R5FSS0_CORE0_ECC_AGGR_NUM_RAMS] =
{
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_R5FSS0_CORE0_ECC_AGGR_initConfig =
{
    .numRams = SDL_R5FSS0_CORE0_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_R5FSS0_CORE0_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_R5FSS0_CORE1_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_R5FSS0_CORE1_ECC_AGGR_subMemTypeList[SDL_R5FSS0_CORE1_ECC_AGGR_NUM_RAMS] =
{
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID,
    SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_R5FSS0_CORE1_ECC_AGGR_initConfig =
{
    .numRams = SDL_R5FSS0_CORE1_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_R5FSS0_CORE1_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MSS_ECC_AGG_MSS
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MSS_ECC_AGG_MSS_subMemTypeList[SDL_MSS_ECC_AGG_MSS_NUM_RAMS] =
{
    SDL_MSS_ECC_AGG_MSS_MSS_L2RAMA_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_L2RAMB_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_RETRAM_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_RAM_ID,
    SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MSS_ECC_AGG_MSS_initConfig =
{
    .numRams = SDL_MSS_ECC_AGG_MSS_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MSS_ECC_AGG_MSS_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_DSS_ECC_AGG
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_DSS_ECC_AGG_subMemTypeList[SDL_DSS_ECC_AGG_NUM_RAMS] =
{
    SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_CM4_RAM_B0_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_CM4_RAM_B1_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_CM4_RAM_B2_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_RSS_TPTC_A0_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_RSS_TPTC_A1_FIFO_ECC_RAM_ID,
    SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_DSS_ECC_AGG_initConfig =
{
    .numRams = SDL_DSS_ECC_AGG_NUM_RAMS - 1U,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_DSS_ECC_AGG_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MSS_MCANA_ECC
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MSS_MCANA_ECC_subMemTypeList[SDL_MSS_MCANA_ECC_NUM_RAMS] =
{
    SDL_MSS_MCANA_ECC_MSS_MCANA_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MSS_MCANA_ECC_initConfig =
{
    .numRams = SDL_MSS_MCANA_ECC_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MSS_MCANA_ECC_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MSS_MCANB_ECC
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MSS_MCANB_ECC_subMemTypeList[SDL_MSS_MCANB_ECC_NUM_RAMS] =
{
    SDL_MSS_MCANB_ECC_MSS_MCANB_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MSS_MCANB_ECC_initConfig =
{
    .numRams = SDL_MSS_MCANB_ECC_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MSS_MCANB_ECC_subMemTypeList[0]),
    /**< Sub type list */
};

typedef struct
{
    SDL_ECC_InitConfig_t *initConfig;
    char *aggrName;
} SDL_Test_EccConfig;

static SDL_Test_EccConfig ECC_Test_config[SDL_ECC_MEMTYPE_MAX] =
{
    {
        &ECC_Test_SDL_R5FSS0_CORE0_ECC_AGGR_initConfig,
        "SDL_R5FSS0_CORE0_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_R5FSS0_CORE1_ECC_AGGR_initConfig,
        "SDL_R5FSS0_CORE1_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_MSS_ECC_AGG_MSS_initConfig,
        "SDL_MSS_ECC_AGG_MSS"
    },
    {
        &ECC_Test_SDL_DSS_ECC_AGG_initConfig,
        "SDL_DSS_ECC_AGG"
    },
    {
        &ECC_Test_SDL_MSS_MCANA_ECC_initConfig,
        "SDL_MSS_MCANA_ECC"
    },
    {
        &ECC_Test_SDL_MSS_MCANB_ECC_initConfig,
        "SDL_MSS_MCANB_ECC"
    }
};
#endif
