/********************************************************************
*
* MCU_R5 MEMORYMAP header file
*
* Copyright (C) 2020 Texas Instruments Incorporated.
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
#ifndef CSLR_SOC_R5_BASEADDRESS_H_
#define CSLR_SOC_R5_BASEADDRESS_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/awr294x/cslr_soc_baseaddress.h>
#ifdef __cplusplus
extern "C"
{
#endif

/* MSS System Memory */
#define CSL_MSS_TCMA_RAM_BASE               (0x00000000U)               /* TCM RAM-A */
#define CSL_MSS_TCMA_RAM_SIZE               CSL_MSS_TCMA_CR5A_U_SIZE    /* Extended on Share Memory + Default 32 KB */

#define CSL_MSS_TCMB_RAM_BASE               (0x00080000U)               /* TCM RAM-B */
#define CSL_MSS_TCMB_RAM_SIZE               CSL_MSS_TCMB_CR5A_U_SIZE    /* Extended on Share Memory + Default 32 KB */

#define CSL_MSS_L2_RAM_BASE                 CSL_MSS_L2_U_BASE   /* L2 RAM in two Banks */
#define CSL_MSS_L2_RAM_SIZE                 CSL_MSS_L2_U_SIZE   /* Total: 960K BANK_A: 512K, BANK_B: 486K */

#define CSL_MSS_L3_RAM_BASE                 CSL_DSS_L3_U_BASE   /* L3 RAM shared by MSS and DSS */
#define CSL_MSS_L3_RAM_SIZE                 CSL_DSS_L3_U_SIZE   /* Total: 3.5625M */

#define CSL_GLOB_MSS_TCMA_RAM_BASE          CSL_MSS_TCMA_CR5A_U_BASE
#define CSL_GLOB_MSS_TCMB_RAM_BASE          CSL_MSS_TCMB_CR5A_U_BASE
#define CSL_GLOB_MSS_L2_RAM_BASE            (0xC0200000U)


#define CSL_MCU_NAVSS0_MCRC_BASE            CSL_MSS_MCRC_U_BASE
#define CSL_MCU_CPSW0_NUSS_BASE             CSL_MSS_CPSW_U_BASE
#define CSL_MCU_RTI0_CFG_BASE               CSL_MSS_WDT_U_BASE
#define CSL_MCU_ESM0_CFG_BASE               CSL_MSS_ESM_U_BASE
#define CSL_MCU_RTITMR0_CFG_BASE            CSL_MSS_RTIA_U_BASE
#define CSL_MCU_RTITMR1_CFG_BASE            CSL_MSS_RTIB_U_BASE
#define CSL_MCU_RTITMR2_CFG_BASE            CSL_MSS_RTIC_U_BASE

#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_R5_BASEADDRESS_H_ */
