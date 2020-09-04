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
#ifndef CSLR_SOC_DSP_BASEADDRESS_H_
#define CSLR_SOC_DSP_BASEADDRESS_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/awr294x/cslr_soc_baseaddress.h>
#ifdef __cplusplus
extern "C"
{
#endif

/* DSS System Memory */
#define CSL_DSP_L1P_U_BASE                 (0xE00000U)
#define CSL_DSP_L1P_SIZE                   CSL_DSS_L1P_U_SIZE
#define CSL_DSP_L1D_U_BASE                 (0xF00000U)
#define CSL_DSP_L1D_SIZE                   CSL_DSS_L1D_U_SIZE
#define CSL_DSP_L2_U_BASE                  (0x800000U)
#define CSL_DSP_L2_RAM_SIZE                CSL_DSS_L2_U_SIZE
#define CSL_DSP_L3_U_BASE                  CSL_DSS_L3_U_BASE   /* L3 RAM shared by MSS and DSS */
#define CSL_DSP_L3_RAM_SIZE                CSL_DSS_L3_U_SIZE   /* Total: 3.5625M */

#define CSL_GLOB_DSP_L1P_U_BASE            CSL_DSS_L1P_U_BASE
#define CSL_GLOB_DSP_L1D_U_BASE            CSL_DSS_L1D_U_BASE
#define CSL_GLOB_DSP_L2_U_BASE             CSL_DSS_L2_U_BASE

#define CSL_DSP_ICFG_U_BASE                (0x1800000U)
#define CSL_C66X_COREPAC_REG_BASE_ADDRESS_REGS  (CSL_DSP_ICFG_U_BASE)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_DSP_BASEADDRESS_H_ */
