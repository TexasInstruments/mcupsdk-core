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
 *  @addtogroup SDL_HWA_API API's for HWA
    @{
 */

#ifndef SDL_HWA_SOC_H_
#define SDL_HWA_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am273x/sdlr_soc_baseaddress.h>


#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_HWA_CFG                   (uint32_t)SDL_DSS_HWA_CFG_U_BASE
#define SDL_HWA_BUS_CFG               (uint32_t)SDL_DSS_CTRL_U_BASE
#define SDL_HWA_DMA0_ADDRESS          (uint32_t)SDL_DSS_HWA_DMA0_U_BASE
#define SDL_HWA_DMA1_ADDRESS          (uint32_t)SDL_DSS_HWA_DMA1_U_BASE
/**
 * \brief This enumerator defines the HWA memories
 *
 */
typedef enum {
    SDL_HWA_DMEM0 = 0,
    /**< HWA Data memories 0 */
    SDL_HWA_DMEM1 = 1,
    /**< HWA Data memories 1 */
    SDL_HWA_DMEM2 = 2,
    /**< HWA Data memories 2 */
    SDL_HWA_DMEM3 = 3,
    /**< HWA Data memories 3 */
    SDL_HWA_DMEM4 = 4,
    /**< HWA Data memories 4 */
    SDL_HWA_DMEM5 = 5,
    /**< HWA Data memories 5 */
    SDL_HWA_DMEM6 = 6,
    /**< HWA Data memories 6 */
    SDL_HWA_DMEM7 = 7,
    /**< HWA Data memories 7 */
    SDL_HWA_WINDOW_RAM = 8,
    /**< Window RAM memories */
    SDL_HWA_FSM_LOCKSTEP = 9,
    /**<  HWA FSM Lockstep */
    SDL_HWA_INVALID = 10,
    /*Memblock */
} SDL_HWA_MemBlock;

/**
 * \brief This enumerator defines the HWA IDs
 *
 */
typedef enum {
    SDL_HWA_DMA0_MEM_ID = 0,
    /**< HWA DMA 0  */
    SDL_HWA_DMA1_MEM_ID = 1,
    /**< HWA DMA 1  */
    SDL_HWA_WINDOW_RAM_MEM_ID = 2,
    /**< Window RAM  */
    SDL_HWA_INVALID_ID = 3,
    /*INVALID Memblock  ID*/
} SDL_HWA_MemID;
/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */
/**
 * \brief   This API is used to get the memory block base address.
 *
 * \param   memID             HWA IDs for DMA0, DMA1 and Window RAM
 *
 * \param   memBlock          HWA memories for DMA0/DMA1's block
 *
 * \param   baseAddr          pointer to base addressof the memories
 *
 * \return  status            return the base address of th instance.
 *                            SDL_PASS:     success
 *                            SDL_EBADARGS: failure, indicate the bad input arguments
 *                            SDL_EFAIL:    failure, indicate verify failed
 */
int32_t SDL_HWA_getMemblockBaseaddr(SDL_HWA_MemID memID, SDL_HWA_MemBlock memBlock,
                             uint32_t *baseAddr);

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
