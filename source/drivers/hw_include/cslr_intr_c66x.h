/********************************************************************
*
* C66X INTR Map Header file
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
#ifndef CSLR_INTR_C66X_H_
#define CSLR_INTR_C66X_H_

#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif

/* List of intr sources specific to C66X Core. */
#define CSL_DSS_INTR_IDMAINT0                           13          /* IDMA Channel 0 Interrupt */
#define CSL_DSS_INTR_IDMAINT1                           14          /* IDMA Channel 1 Interrupt */
#define CSL_DSS_INTR_INTERR                             96          /* Dropped DSP interrupt */
#define CSL_DSS_INTR_EMC_IDMAERR                        97          /* Invalid IDMA params */
#define CSL_DSS_INTR_PBISTINTERR                        98          /* PBIST interrupt. May not be available on all SoCs */
#define CSL_DSS_INTR_MDMAERR                            110         /* MDMA/XMC error intr */
#define CSL_DSS_INTR_PMC_ED                             113         /* 1-bit error: L1P */
#define CSL_DSS_INTR_UMCED1                             116         /* 1-bit error: L2 */
#define CSL_DSS_INTR_UMCED2                             117         /* 2-bit error: L2 */
#define CSL_DSS_INTR_PDC_INT                            118         /* Powerdomain intr */
#define CSL_DSS_INTR_SYS_CMPA                           119         /* CFG space violations */
#define CSL_DSS_INTR_PMC_CMPA                           120         /* CPU violation: L1PMPU */
#define CSL_DSS_INTR_PMC_DMPA                           121         /* DMA violation: L1PMPU */
#define CSL_DSS_INTR_DMC_CMPA                           122         /* CPU violation: L1DMPU */
#define CSL_DSS_INTR_DMC_DMPA                           123         /* DMA violation: L1DMPU */
#define CSL_DSS_INTR_UMCCMPA                            124         /* CPU violation: L2MPU */
#define CSL_DSS_INTR_UMCDMPA                            125         /* DMA violation: L2MPU */
#define CSL_DSS_INTR_EMC_CMPA                           126         /* CFG space violations from external access */
#define CSL_DSS_INTR_EMC_BUSERR                         127         /* Abort returned from external peripherals */

#ifdef __cplusplus
}
#endif
#endif /* CSLR_INTR_C66X_H_*/
