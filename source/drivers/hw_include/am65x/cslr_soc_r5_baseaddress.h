/********************************************************************
* Copyright (C) 2024 Texas Instruments Incorporated.
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
#ifndef CSLR_MCU_R5_MEMORYMAP_H_
#define CSLR_MCU_R5_MEMORYMAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

#define CSL_MCU_ATCM_BASE                                                                      (0x0u)
#define CSL_MCU_ATCM_SIZE                                                                      (0x8000u)
#define CSL_MCU_RAT_REGION0_BASE                                                               (0x8000u)
#define CSL_MCU_RAT_REGION0_SIZE                                                               (0x3fff8000u)
#define CSL_MCU_NON_RAT_SOC_REGION0_BASE                                                       (0x40008000u)
#define CSL_MCU_NON_RAT_SOC_REGION0_SIZE                                                       (0xf78000u)
#define CSL_MCU_VIC_CFG_BASE                                                                   (0x40f80000u)
#define CSL_MCU_VIC_CFG_SIZE                                                                   (0x4000u)
#define CSL_MCU_NON_RAT_SOC_REGION1_BASE                                                       (0x40f84000u)
#define CSL_MCU_NON_RAT_SOC_REGION1_SIZE                                                       (0xc000u)
#define CSL_MCU_RAT_CFG_BASE                                                                   (0x40f90000u)
#define CSL_MCU_RAT_CFG_SIZE                                                                   (0x1000u)
#define CSL_MCU_NON_RAT_SOC_REGION2_BASE                                                       (0x40f91000u)
#define CSL_MCU_NON_RAT_SOC_REGION2_SIZE                                                       (0x6f000u)
#define CSL_MCU_RAT_REGION1_BASE                                                               (0x41000000u)
#define CSL_MCU_RAT_REGION1_SIZE                                                               (0xbf000000u)
#define CSL_MCU_BTCM_BASE                                                                      (0x41010000u)
#define CSL_MCU_BTCM_SIZE                                                                      (0x8000u)
#define CSL_MCU_RAT_REGION3_BASE                                                               (0x41018000u)
#define CSL_MCU_RAT_REGION3_SIZE                                                               (0x3efe8000u)
#define CSL_MCU_RAT_REGION4_BASE                                                               (0x80000000u)
#define CSL_MCU_RAT_REGION4_SIZE                                                               (0x40000000u)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_MCU_R5_MEMORYMAP_H_ */
