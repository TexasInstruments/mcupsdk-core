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
@addtogroup SDL_ECC_BUS_SAFETY_MACROS
@{
*/

#ifndef SDL_MSS_CR5_SOC_H_
#define SDL_MSS_CR5_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am273x/sdlr_soc_baseaddress.h>
#include <sdl/include/am273x/sdlr_mss_ctrl.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_ECC_BUS_SAFETY_MSS_BUS_CFG           (uint32_t)SDL_MSS_CTRL_U_BASE
#define SDL_ECC_BUS_SAFETY_DSS_BUS_CFG           (uint32_t)SDL_DSS_CTRL_U_BASE

#define  SDL_DSS_CMC_COMP_U_END                  (SDL_DSS_CMC_COMP_U_BASE + 0X3FFCU)
#define  SDL_DSS_MCRC_U_END                      (SDL_DSS_MCRC_U_BASE + 0x144U)
#define  SDL_DSS_CBUFF_FIFO_U_END                (SDL_DSS_CBUFF_FIFO_U_BASE + 0X3FFCU)
#define  SDL_DSS_CM4_MBOX_U_END                  (SDL_DSS_CM4_MBOX_U_BASE + 0XFFCU)
#define SDL_DSS_MDO_FIFO_U_END                   (SDL_DSS_MDO_FIFO_U_BASE + SDL_DSS_MDO_FIFO_U_SIZE)

#define SDL_DSS_L3_BANKA_ADDRESS                 SDL_DSS_L3_U_BASE
#define SDL_DSS_L3_BANK_SIZE                     (0x100000U)
#define SDL_DSS_L3_BANKB_ADDRESS                 SDL_DSS_L3_BANKA_ADDRESS+SDL_DSS_L3_BANK_SIZE
#define SDL_DSS_L3_BANKC_ADDRESS                 SDL_DSS_L3_BANKB_ADDRESS+SDL_DSS_L3_BANK_SIZE
#define SDL_DSS_L3_BANKD_ADDRESS                 SDL_DSS_L3_BANKC_ADDRESS+SDL_DSS_L3_BANK_SIZE
#define SDL_DSS_L3_END_ADDRESS                   (SDL_DSS_L3_U_BASE+SDL_DSS_L3_U_SIZE)

#define SDL_DSS_HWA_DMA0_U_BASE_END              (SDL_DSS_HWA_DMA0_U_BASE+SDL_DSS_HWA_DMA0_U_SIZE)
#define SDL_DSS_HWA_DMA1_U_BASE_END              (SDL_DSS_HWA_DMA1_U_BASE+SDL_DSS_HWA_DMA1_U_SIZE)

#define SDL_DSS_MAILBOX_U_BASE_END               (SDL_DSS_MAILBOX_U_BASE+SDL_DSS_MAILBOX_U_SIZE)

#define SDL_MSS_MBOX_U_BASE_END                  (SDL_MSS_MBOX_U_BASE+SDL_MSS_MAILBOX_U_SIZE)

#define SDL_DSS_L2_U_BASE_END                    (SDL_DSS_L2_U_BASE+SDL_DSS_L2_U_SIZE)

#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE   (SDL_MSS_CTRL_R5A_AHB_BASE )
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE   (SDL_MSS_CTRL_R5B_AHB_BASE )
#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_END   (SDL_MSS_CTRL_R5A_AHB_BASE + SDL_MSS_CTRL_R5A_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_END   (SDL_MSS_CTRL_R5B_AHB_BASE + SDL_MSS_CTRL_R5B_AHB_SIZE)

#define SDL_MSS_CR5A_TCM_U_BASE (SDL_MSS_TCMA_CR5A_U_BASE)
#define SDL_MSS_CR5B_TCM_U_BASE (SDL_MSS_TCMA_CR5B_U_BASE)
#define SDL_MSS_CR5A_TCM_U_END  (SDL_MSS_TCMA_CR5A_U_BASE + SDL_MSS_TCMA_CR5A_U_SIZE)
#define SDL_MSS_CR5B_TCM_U_END  (SDL_MSS_TCMA_CR5B_U_BASE + SDL_MSS_TCMA_CR5B_U_SIZE)


#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
