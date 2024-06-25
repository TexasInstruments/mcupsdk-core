/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file     ecc_func_am261x.h
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


#include <sdl/include/am261x/sdlr_soc_ecc_aggr.h>
/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_SOC_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_SOC_ECC_AGGR_subMemTypeList[SDL_SOC_ECC_AGGR_NUM_RAMS] =
{
    SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_TPTC_A1_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_SOC_ECC_AGGR_initConfig =
{
    .numRams = SDL_SOC_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_SOC_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

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
 * This structure holds the list of Ram Ids for memory subtypes SDL_R5FSS1_CORE0_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_R5FSS1_CORE0_ECC_AGGR_subMemTypeList[SDL_R5FSS1_CORE0_ECC_AGGR_NUM_RAMS] =
{
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID,
    SDL_R5FSS1_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_R5FSS1_CORE0_ECC_AGGR_initConfig =
{
    .numRams = SDL_R5FSS1_CORE0_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_R5FSS1_CORE0_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_R5FSS1_CORE1_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_R5FSS1_CORE1_ECC_AGGR_subMemTypeList[SDL_R5FSS1_CORE1_ECC_AGGR_NUM_RAMS] =
{
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK2_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_IDATA_BANK3_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID,
    SDL_R5FSS1_CORE1_ECC_AGGR_CPU1_KS_VIM_RAMECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_R5FSS1_CORE1_ECC_AGGR_initConfig =
{
    .numRams = SDL_R5FSS1_CORE1_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_R5FSS1_CORE1_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_HSM_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_HSM_ECC_AGGR_subMemTypeList[SDL_HSM_ECC_AGGR_NUM_RAMS] =
{
    SDL_HSM_ECC_AGGR_RAMB0_RAM_ID,
    SDL_HSM_ECC_AGGR_RAMB1_RAM_ID,
    SDL_HSM_ECC_AGGR_RAMB2_RAM_ID,
    SDL_HSM_ECC_AGGR_RAMB3_RAM_ID,
    SDL_HSM_ECC_AGGR_SECUREB4_RAM_ID,
    SDL_HSM_ECC_AGGR_MBOX_RAM_ID,
    SDL_HSM_ECC_AGGR_SECURE_RAM_ID,
    SDL_HSM_ECC_AGGR_ROM_RAM_ID,
    SDL_HSM_ECC_AGGR_TPTC_A0_RAM_ID,
    SDL_HSM_ECC_AGGR_TPTC_A1_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_HSM_ECC_AGGR_initConfig =
{
    .numRams = SDL_HSM_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_HSM_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_subMemTypeList[SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_NUM_RAMS] =
{
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_initConfig =
{
    .numRams = SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS] =
{
    SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig =
{
    .numRams = SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS] =
{
    SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig =
{
    .numRams = SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS] =
{
    SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig =
{
    .numRams = SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS] =
{
    SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig =
{
    .numRams = SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_subMemTypeList[0]),
    /**< Sub type list */
};

/** ------------------------------------------------------------------------------------
 * This structure holds the list of Ram Ids for memory subtypes SDL_CPSW3GCSS_ECC_AGGR
 * -------------------------------------------------------------------------------------
 */
static SDL_ECC_MemSubType ECC_Test_SDL_CPSW3GCSS_ECC_AGGR_subMemTypeList[SDL_CPSW3GCSS_ECC_AGGR_NUM_RAMS] =
{
    SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_ALE_RAM_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL1_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL2_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL3_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL4_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL5_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW_3GC_CORE_ECC_ECC_CTRL6_ECC_RAM_ID,
    SDL_CPSW3GCSS_ECC_AGGR_CPSW3GCSS_EST_RAM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_SDL_CPSW3GCSS_ECC_AGGR_initConfig =
{
    .numRams = SDL_CPSW3GCSS_ECC_AGGR_NUM_RAMS,
    /**< Number of Rams ECC is enabled */
    .pMemSubTypeList = &(ECC_Test_SDL_CPSW3GCSS_ECC_AGGR_subMemTypeList[0]),
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
        &ECC_Test_SDL_SOC_ECC_AGGR_initConfig,
        "SDL_SOC_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_R5FSS0_CORE0_ECC_AGGR_initConfig,
        "SDL_R5FSS0_CORE0_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_R5FSS0_CORE1_ECC_AGGR_initConfig,
        "SDL_R5FSS0_CORE1_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_R5FSS1_CORE0_ECC_AGGR_initConfig,
        "SDL_R5FSS1_CORE0_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_R5FSS1_CORE1_ECC_AGGR_initConfig,
        "SDL_R5FSS1_CORE1_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_HSM_ECC_AGGR_initConfig,
        "SDL_HSM_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_initConfig,
        "SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig,
        "SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig,
        "SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig,
        "SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_initConfig,
        "SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR"
    },
    {
        &ECC_Test_SDL_CPSW3GCSS_ECC_AGGR_initConfig,
        "SDL_CPSW3GCSS_ECC_AGGR"
    },
};
#endif
