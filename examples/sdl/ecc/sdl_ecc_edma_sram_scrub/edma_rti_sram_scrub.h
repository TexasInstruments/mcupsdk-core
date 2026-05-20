/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

#ifndef _EDMA_RTI_SRAM_SCRUB_H_
#define _EDMA_RTI_SRAM_SCRUB_H_

#include <stdint.h>
#include <stdbool.h>
#include <sdl/sdl_ecc.h>
#include <drivers/esm/v1/cslr_esm.h>

#if defined(SOC_AM263X)
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <sdl/include/am263x/sdlr_mss_ecc_agga.h>
#endif
#if defined(SOC_AM263PX)
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>
#include <sdl/include/am263px/sdlr_mss_ecc_agga.h>
#endif
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_ESM_HI_PRI_RESETVAL             (0xFFFFFFFFU)
#define SDL_ESM_LOW_PRI_RESETVAL            (0xFFFFFFFFU)

/* SRAM related defines */
#define APP_SRAM_SCRUB_START_ADDR           (0x70000000U)
#define APP_SRAM_SCRUB_BANK_SIZE_BYTES      (0x00080000U)  /* 512 KB */
#define APP_SRAM_SCRUB_NUM_BANKS            (3U)

#define APP_SRAM_SCRUB_ECC_ROW_WIDTH_BYTES  (8U)

/* Chunk of 4 x 64-bit Beats */
#define APP_CHUNK_SIZE_BITS                 (4U * 64U)
#define APP_CHUNK_SIZE_BYTES                (APP_CHUNK_SIZE_BITS / 8U)

#define ESM_REGISTERS                       ((volatile CSL_esmRegs *) CSL_TOP_ESM_U_BASE)

#define SDL_MSS_L2_MEM_INIT_DONE_ADDR                   (SDL_MSS_CTRL_U_BASE+SDL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR                 (SDL_ECC_AGG_R5SS0_CORE0_U_BASE+SDL_MSS_ECC_AGGA_ERROR_STATUS1)

// ESM EN disable interrupt value
#define ESM_EN_DISABLE_VALUE                (0U)

// ESM EN enable interrupt value
#define ESM_EN_ENABLE_VALUE                 (0b1111)

/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */
extern int32_t SDL_cleartcmStatusRegs(uint32_t clearVal);

/* ========================================================================== */
/*                          Function Declaration                              */
/* ========================================================================== */

void     App_configEccEsm(void);
uint32_t App_getEccSecStatus(void);
bool     App_checkEccSecStatusPending(uint32_t bankIdx, uint32_t secStatus);
uint32_t App_getEccFaultAddr(uint32_t bankIdx);
void     App_clearEccFault(uint32_t bankIdx);
void     App_configDma(void);
void     App_getEdmaChunkAddr(uint32_t* startAddrPtr, uint32_t* endAddrPtr);
uint8_t* App_getEdmaBuffAddr(void);
void     App_serviceSecInterrupt(void);
bool     ECCAGG_writeRegister(uint8_t endpointId, uint16_t registerOffset, uint32_t val);
uint32_t ECCAGG_readRegister(uint8_t endpointId, uint16_t registerOffset);
void     ESM_init(void);
int32_t  ECC_funcTest(void);
int32_t ECC_Test_run_MSS_L2RAMB_1BitInjectTest(void);

#endif /* _EDMA_RTI_SRAM_SCRUB_H_ */
