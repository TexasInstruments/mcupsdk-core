/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

/*
 *  ======== hwa.c ========
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hwa.h>
#include <drivers/hwa/v0/soc/hwa_soc.h>

#define HWA_PARAM_CHECK

#define HWA_GET_DRIVER_STRUCT(handle) \
{\
     ptrHWADriver = (HWA_Object *)handle;\
}

HWA_InterruptCtx HwaParamsetIntr[SOC_HWA_NUM_PARAM_SETS];

/* TPCC_A, TPCC_B, and TPCC_C all have the same mapping for HWA_DMA */
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ1 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ1) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ1 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ1) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ1 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ1))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ2 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ2) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ2 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ2) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ2 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ2))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ3 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ3) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ3 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ3) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ3 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ3))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ4 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ4) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ4 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ4) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ4 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ4))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ5 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ5) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ5 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ5) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ5 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ5))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ6 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ6) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ6 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ6) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ6 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ6))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ7 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ7) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ7 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ7) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ7 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ7))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ8 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ8) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ8 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ8) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ8 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ8))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ9 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ9) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ9 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ9) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ9 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ9))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ10 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ10) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ10 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ10) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ10 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ10))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ11 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ11) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ11 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ11) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ11 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ11))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ12 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ12) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ12 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ12) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ12 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ12))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ13 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ13) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ13 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ13) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ13 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ13))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ14 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ14) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ14 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ14) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ14 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ14))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ15 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ15) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ15 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ15) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ15 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ15))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ16 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ16) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ16 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ16) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ16 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ16))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ17 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ17) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ17 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ17) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ17 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ17))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ18 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ18) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ18 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ18) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ18 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ18))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ19 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ19) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ19 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ19) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ19 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ19))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ20 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ20) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ20 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ20) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ20 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ20))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ21 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ21) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ21 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ21) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ21 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ21))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ22 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ22) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ22 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ22) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ22 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ22))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ23 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ23) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ23 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ23) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ23 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ23))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ24 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ24) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ24 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ24) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ24 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ24))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ25 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ25) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ25 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ25) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ25 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ25))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ26 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ26) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ26 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ26) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ26 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ26))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ27 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ27) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ27 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ27) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ27 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ27))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ28 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ28) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ28 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ28) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ28 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ28))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ29 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ29) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ29 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ29) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ29 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ29))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ30 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ30) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ30 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ30) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ30 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ30))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
#if ((EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ31 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ31) || \
     (EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ31 != EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ31) || \
     (EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ31 != EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ31))
#error assumption of TPCC A, B, C HWA event lines being identical is not satisfied.
#endif
/* HWA dma destination channel index to EDMA channel id mapping */
uint8_t  gHwaEDMAChanMapping [32] =
{
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ1,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ2,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ3,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ4,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ5,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ6,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ7,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ8,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ9,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ10,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ11,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ12,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ13,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ14,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ15,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ16,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ17,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ18,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ19,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ20,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ21,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ22,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ23,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ24,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ25,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ26,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ27,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ28,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ29,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ30,
    EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ31
};

/* INTERNAL FUNCTIONS */

/* PROTOTYPES */
static int32_t HWA_getDriverAccess(HWA_Object *ptrHWADriver, bool checkConfigProgress, bool checkParamProgress, uint32_t paramsetIdx);
static void HWA_releaseDriverAccess(HWA_Object *ptrHWADriver, bool configProgress, bool paramProgress, uint32_t paramsetIdx);
static int32_t HWA_validateParamSetConfig(HWA_Object *ptrHWADriver, HWA_ParamConfig *paramConfig);
static void HWA_deleteInstance(HWA_Object *ptrHWADriver);
static void HWA_paramDoneIntr1ISR(void *arg);
static void HWA_paramDoneIntr2ISR(void *arg);
static void HWA_allParamDoneISR(void *arg);
static void HWA_allALTParamDoneISR(void *arg);

static inline void HWA_doReset(DSSHWACCRegs  *ctrlBaseAddr);
static void HWA_startRAMInit(DSSHWACCRegs  *ctrlBaseAddr, uint32_t ramMemBankMask);
static void HWA_waitRAMInit(DSSHWACCRegs  *ctrlBaseAddr, uint32_t ramMemBankMask);

/** @addtogroup HWA_DRIVER_INTERNAL_FUNCTION
 @{ */
/*!
 *  @brief  Function to validate the state of the driver and gain access.
 *
 *  @param  ptrHWADriver         Internal HWA driver object that will be checked for sanity
 *
 *  @param  checkConfigProgress  flag to indicate whether to check driver's configInProgress bit
 *
 *  @param  checkParamProgress   flag to indicate whether to check driver's paramSetMapInProgress bitmap
 *
 *  @param  paramsetIdx          should be valid if checkParamProgress is set to true.
 *
 *  @return 0 if driver access is allowed; error code if access is denied.
 *
 */
static int32_t HWA_getDriverAccess(HWA_Object *ptrHWADriver, bool checkConfigProgress, bool checkParamProgress, uint32_t paramsetIdx)
{
    int32_t retCode = 0;
    uintptr_t           key;

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check state of driver */
    if ((ptrHWADriver == NULL)  || (ptrHWADriver->refCnt == 0U))
    {
        retCode = HWA_ENOINIT;
    }
    else
    {
        /* check config first */
        if (checkConfigProgress==true)
        {
            if (ptrHWADriver->configInProgress == 1U)
            {
                retCode = HWA_EINUSE;
            }
            else
            {
                ptrHWADriver->configInProgress = 1U;
            }
        }
        /* next check paramset */
        else if (checkParamProgress==true)
        {
            /*first check for valid index */
            if (paramsetIdx>=ptrHWADriver->hwAttrs->numHwaParamSets)
            {
                retCode = HWA_EINVAL;
            }
            /* next check for inProgress bit */
            else if ((ptrHWADriver->paramSetMapInProgress & (uint32_t)(1U <<paramsetIdx))==1)
            {
                retCode = HWA_EINUSE;
            }
            else
            {
                /* no error */
                retCode = 0;
                ptrHWADriver->paramSetMapInProgress =
                    ptrHWADriver->paramSetMapInProgress | (uint32_t)(1U <<paramsetIdx);

            }
        }
        else
        {
            /* no error */
            retCode = 0;
        }
    }

    /* Restore the interrupts: */
    HwiP_restore(key);

    return retCode;
}



/*!
 *  @brief  Function to release access to the driver.
 *
 *  @param  ptrHWADriver         Internal HWA driver object that will be checked for sanity
 *
 *  @param  configProgress  flag to indicate whether to release driver's configInProgress bit
 *
 *  @param  paramProgress   flag to indicate whether to release driver's paramSetMapInProgress bitmap
 *
 *  @param  paramsetIdx          should be valid if paramProgress is set to true.
 *
 *  @return none
 *
 */
static void HWA_releaseDriverAccess(HWA_Object *ptrHWADriver, bool configProgress, bool paramProgress, uint32_t paramsetIdx)
{
    uintptr_t           key;

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check config first */
    if (configProgress==true)
    {
        ptrHWADriver->configInProgress = 0;
    }
    /* next check paramset */
    if (paramProgress==true)
    {
        ptrHWADriver->paramSetMapInProgress=
            (ptrHWADriver->paramSetMapInProgress & (uint32_t)(~(1U <<paramsetIdx)));
    }

    /* Restore the interrupts: */
    HwiP_restore(key);

    return;
}


/*!
 *  @brief  Function to validate the user passed paramConfig. This function gets compiled out
 *          if HWA_PARAM_CHECK is disabled.
 *
 *  @param  ptrHWADriver         Internal HWA driver object that will be checked for sanity
 *
 *  @param  paramConfig          user supplied paramSet Config
 *
 *  @return 0 if valid paramSet config if provided else a valid error code.
 *
 */
static int32_t HWA_validateParamSetConfig(HWA_Object *ptrHWADriver, HWA_ParamConfig *paramConfig)
{
    int32_t retCode = 0;
#ifdef HWA_PARAM_CHECK
    if (paramConfig == NULL)
    {
        /* invalid config */
        retCode = HWA_EINVAL;
    }
    else
    {
        /* general configuration */
        if((paramConfig->triggerMode > HWA_TRIG_MODE_SOFTWARE2) ||
           ((paramConfig->triggerSrc > (ptrHWADriver->hwAttrs->numDmaChannels - 1)) &&
                (paramConfig->triggerMode == HWA_TRIG_MODE_DMA)) ||
           ((paramConfig->triggerSrc > (SOC_HWA_NUM_CSIRX_IRQS - 1)) &&
                (paramConfig->triggerMode == HWA_TRIG_MODE_HARDWARE)) ||
           (paramConfig->accelMode > HWA_ACCELMODE_NONE) ||
           (paramConfig->contextswitchCfg > HWA_PARAMSET_CONTEXTSWITCH_FORCE_ENABLE))
        {
            /* invalid config */
            retCode = HWA_EINVAL_PARAMSET_GENERALCONFIG;
        }
        /* source configuration */
        else if (
            (paramConfig->source.srcAcnt >= (1U << 12U)) ||
            (paramConfig->source.srcBcnt >= (1U << 12U)) ||
            (paramConfig->source.srcAIdx > 65535U) ||
            (paramConfig->source.srcBIdx > 65535U) ||
            (paramConfig->source.srcAcircShift >= (1U << 12U)) ||
            (paramConfig->source.srcBcircShift >= (1U << 12U)) ||
            (paramConfig->source.srcAcircShiftWrap >= (1U << 4U)) ||
            (paramConfig->source.srcBcircShiftWrap >= (1U << 4U)) ||
            (paramConfig->source.srcCircShiftWrap3 >= (1U << 3U)) ||
            (paramConfig->source.srcRealComplex > HWA_SAMPLES_FORMAT_REAL) ||
            (paramConfig->source.srcWidth > HWA_SAMPLES_WIDTH_32BIT) ||
            (paramConfig->source.srcSign > HWA_SAMPLES_SIGNED) ||
            (paramConfig->source.srcConjugate > HWA_FEATURE_BIT_ENABLE) ||
            (paramConfig->source.srcScale > 8U) ||
            (paramConfig->source.srcIQSwap > HWA_FEATURE_BIT_ENABLE) ||
            (paramConfig->source.shuffleMode > HWA_SRC_SHUFFLE_AB_MODE_BDIM) ||
            (paramConfig->source.wrapComb >= (1U << 20U)) ||
            (paramConfig->source.shuffleStart >= (1U << 4U)))
        {
            /* invalid config */
            retCode = HWA_EINVAL_PARAMSET_SOURCE;
        }
        else if (  /* dst configuration */
            (paramConfig->dest.dstAcnt >= (1U << 12U)) ||
            (paramConfig->dest.dstRealComplex > HWA_SAMPLES_FORMAT_REAL) ||
            (paramConfig->dest.dstWidth > HWA_SAMPLES_WIDTH_32BIT) ||
            (paramConfig->dest.dstSign > HWA_SAMPLES_SIGNED) ||
            (paramConfig->dest.dstAIdx > 65535U) ||
            (paramConfig->dest.dstBIdx > 65535U) ||
            (paramConfig->dest.dstConjugate > HWA_FEATURE_BIT_ENABLE) ||
            (paramConfig->dest.dstScale > 8U) ||
            (paramConfig->dest.dstSkipInit >= (1U << 10U)) ||
            (paramConfig->dest.dstIQswap > HWA_FEATURE_BIT_ENABLE))
        {
            /* invalid config */
            retCode = HWA_EINVAL_PARAMSET_DEST;
        }
        /* check the src, and dst address */
        else if (paramConfig->accelMode != HWA_ACCELMODE_NONE)
        {
           /* same memory bank*/
           if ((paramConfig->source.srcAddr >> 14U) == (paramConfig->dest.dstAddr >> 14U))
           {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_SRCDST_ADDRESS;
           }
        }
        if (paramConfig->accelMode == HWA_ACCELMODE_FFT)
        {
            /* if FFT mode, then fftEn should be checked for correct values */
            if ((paramConfig->accelModeArgs.fftMode.fftEn > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.butterflyScaling >= (1U << 12U)) ||
                (paramConfig->accelModeArgs.fftMode.windowEn > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.winSymm > HWA_FFT_WINDOW_SYMMETRIC) ||
                (paramConfig->accelModeArgs.fftMode.windowMode > HWA_WINDOW_MODE_COMPLEX) ||
                ( (paramConfig->accelModeArgs.fftMode.windowMode == HWA_WINDOW_MODE_18BITREAL) &&
                  (paramConfig->accelModeArgs.fftMode.windowStart > 2047U) )||
                ((paramConfig->accelModeArgs.fftMode.windowMode == HWA_WINDOW_MODE_16BITREAL) &&
                 (paramConfig->accelModeArgs.fftMode.windowStart > 4095U)) ||
                ((paramConfig->accelModeArgs.fftMode.windowMode == HWA_WINDOW_MODE_COMPLEX) &&
                 (paramConfig->accelModeArgs.fftMode.windowStart > 2047U)) ||
                (paramConfig->accelModeArgs.fftMode.fftSize3xEn > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.butterflyScalingFFT3x > HWA_FFT3x_BFLY_SCALING_LSBROUNDED) ||
                (paramConfig->accelModeArgs.fftMode.bpmEnable > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.bpmPhase > (1U << 4U))
                )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_FFTMODE_GENERALCONFIG;
            }
            else if ((/* if fftEn is set to Enable, fftSize3xEn is disabled then fftSize should be checked for correct values for 2^N*/
                (paramConfig->accelModeArgs.fftMode.fftEn == HWA_FEATURE_BIT_ENABLE) &&
                (!paramConfig->accelModeArgs.fftMode.fftSize3xEn) &&
                (!paramConfig->accelModeArgs.fftMode.fftSizeDim2) &&
                (
                    /* minimum fftsize is 2, N= 1 to 11*/
                (paramConfig->accelModeArgs.fftMode.fftSize < 1U) ||
                    (paramConfig->accelModeArgs.fftMode.fftSize > 11U)
                    )
                ) ||
                ( /* if fftEn is set to Enable, fftSize3xEn is enabled, then fftSize should be checked for correct values for 3x2^N */
                (paramConfig->accelModeArgs.fftMode.fftEn == HWA_FEATURE_BIT_ENABLE) &&
                    (paramConfig->accelModeArgs.fftMode.fftSize3xEn) &&
                    (!paramConfig->accelModeArgs.fftMode.fftSizeDim2) &&
                    /*fftSize is 0 - 9, minimum fftsize is 3 */
                    (paramConfig->accelModeArgs.fftMode.fftSize > 9U)
                    ) ||
                    (
                        /* if fftEn is set to Enable, fftSize3xEn is disabled, 2D fft is enabled */
                (paramConfig->accelModeArgs.fftMode.fftEn == HWA_FEATURE_BIT_ENABLE) &&
                        (!paramConfig->accelModeArgs.fftMode.fftSize3xEn) &&
                        (paramConfig->accelModeArgs.fftMode.fftSizeDim2) &&
                        ((paramConfig->accelModeArgs.fftMode.fftSizeDim2 > 10U) ||
                         (paramConfig->accelModeArgs.fftMode.fftSizeDim2 > paramConfig->accelModeArgs.fftMode.fftSize))
                        )
                )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_FFTMODE_SIZE;
            }
            else if (
                (paramConfig->accelModeArgs.fftMode.postProcCfg.magLogEn > HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED) ||
                (paramConfig->accelModeArgs.fftMode.postProcCfg.fftOutMode > HWA_FFT_MODE_OUTPUT_SUM_STATS) ||
                (paramConfig->accelModeArgs.fftMode.postProcCfg.max2Denable > HWA_FEATURE_BIT_ENABLE) ||
                ((paramConfig->accelModeArgs.fftMode.postProcCfg.max2Denable == HWA_FEATURE_BIT_ENABLE)
                    && ((paramConfig->source.srcAcnt > 255U) || (paramConfig->source.srcAcnt < 1U)  //valid SRCACNT is 1 to 255
                        || (paramConfig->source.srcBcnt > 1023U)   //valid SRCBCNT is 0 to 1023
                        )
                    ) ||
                    (paramConfig->accelModeArgs.fftMode.postProcCfg.histogramMode > HWA_HISTOGRAM_MODE_CDF_THRESHOLD) ||
                    ((paramConfig->accelModeArgs.fftMode.postProcCfg.histogramMode > HWA_HISTOGRAM_MODE_DISABLED) &&
                      ((paramConfig->accelModeArgs.fftMode.postProcCfg.histogramScaleSelect < 7U) ||
                       (paramConfig->accelModeArgs.fftMode.postProcCfg.histogramScaleSelect > 13U) ||
                       (paramConfig->accelModeArgs.fftMode.postProcCfg.histogramSizeSelect < 3U) ||
                       (paramConfig->accelModeArgs.fftMode.postProcCfg.histogramSizeSelect > 6U)  ||
                       (paramConfig->source.srcAcnt < 1U) || (paramConfig->source.srcAcnt > 63U)
                      )
                    )
                )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_FFTMODE_POSTPROC;
            }
            else if (
                (paramConfig->accelModeArgs.fftMode.preProcCfg.dcEstResetMode > HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET_ZEROLPCONT) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.dcSubEnable > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.dcSubSelect > HWA_DCSUB_SELECT_DCEST) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.chanCombEn > HWA_FEATURE_BIT_ENABLE)  ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.zeroInsertEn > HWA_FEATURE_BIT_ENABLE) ||
                /* for fft stitching, first paramset, using the winInterpolateMode, but the cmultMode is set to disable,
                   only second paramset is set to stitching mode */
                ((paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_DISABLE) &&
                 (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.winInterpolateMode > HWA_FFT_WINDOW_INTERPOLATE_MODE_4K))
                )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_FFTMODE_PREPROC;
            }
            else if (
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdEnable > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdMode > HWA_INTERFTHRESH_MODE_MAG_AND_MAGDIFF) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdSelect > HWA_INTERFTHRESH_SELECT_EST_INDIVIDUAL) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfStat.resetMode > HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET_ZEROLPCONT) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.enable > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.countThreshold > 31U) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.pathSelect > HWA_INTERFMITIGATION_PATH_UNUSED) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.leftHystOrder > 15U) ||
                (paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.rightHystOrder > 15U))
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_FFTMODE_PREPROC_INTERF;
            }
            else if (
                (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode > HWA_COMPLEX_MULTIPLY_MODE_FREQSHIFT_FREQINCREMENT) ||
                ( (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_FFT_STITCHING) /*stitching mode*/
                  && ((paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.twiddlePattern != HWA_FFT_STITCHING_TWID_PATTERN_8K) &&
                      (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.twiddlePattern != HWA_FFT_STITCHING_TWID_PATTERN_4K)
                     )
                )||
                ((paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_SCALAR_MULT)
                  &&(paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.scalerMultiply.scaleCmultScaleEn > HWA_FEATURE_BIT_ENABLE)
                ) ||
                ((paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT)
                    &&(paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.cmultScaleEn > HWA_FEATURE_BIT_ENABLE)
                ) ||
                ((paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_RECURSIVE_WIN)
                   &&
                    (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.recursiveWin.recwinModeSel > HWA_FFT_STITCHING_TWID_PATTERN_EXE_COUNT)
                )||
                ((paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_LUT_FREQ_DEROTATE)
                  &&
                (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.lutFreqDerotate.ramIdxIncrMode > HWA_LUT_FREQ_DEROTATE_RAMIDX_NONINCR)
                )
                )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_FFTMODE_PREPROC_COMPLEXMULT;
            }
        }
        if (paramConfig->accelMode == HWA_ACCELMODE_CFAR)
        {
            if ((paramConfig->accelModeArgs.cfarMode.numGuardCells >= (1U << 3U)) ||
                (paramConfig->accelModeArgs.cfarMode.nAvgDivFactor > 8U) ||
                (paramConfig->accelModeArgs.cfarMode.nAvgMode > HWA_NOISE_AVG_MODE_CFAR_OS) ||
                (paramConfig->accelModeArgs.cfarMode.operMode > HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX_LINEARCFAR) ||
                ((paramConfig->accelModeArgs.cfarMode.outputMode > HWA_CFAR_OUTPUT_MODE_FILTER_LARGE_PEAK) &&
                 (!paramConfig->accelModeArgs.cfarMode.cfarAdvOutMode)) ||
                (paramConfig->accelModeArgs.cfarMode.peakGroupEn > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.cfarMode.cyclicModeEn > HWA_FEATURE_BIT_ENABLE) ||
                (paramConfig->accelModeArgs.cfarMode.cfarAdvOutMode > HWA_FEATURE_BIT_ENABLE)
                )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_CFARMODE_GENERALCONFIG;
            }
            else if       /* cfar os */
                ((paramConfig->accelModeArgs.cfarMode.nAvgMode == HWA_NOISE_AVG_MODE_CFAR_OS) &&
                (   (paramConfig->accelModeArgs.cfarMode.cfarOsKvalue > 128U) ||
                    (paramConfig->accelModeArgs.cfarMode.cfarOsEdgeKScaleEn > HWA_FEATURE_BIT_ENABLE) ||
                    (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight)
                    || ((paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 0U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 4U)
                        && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 6U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 8U)
                        && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 12U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 16U)
                        && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 24U) && (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft != 32U)
                        ))
                    )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_CFARMODE_OSCONFIG;
            }
            else if  /* cfar CA */
                ((paramConfig->accelModeArgs.cfarMode.nAvgMode != HWA_NOISE_AVG_MODE_CFAR_OS) &&
                ((paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft >= (1U << 6U)) ||
                    (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight >= (1U << 6U)) ||
                    (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft == 1U) ||
                    (paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight == 1U)
                    )
                    )
            {
                /* invalid config */
                retCode = HWA_EINVAL_PARAMSET_CFARMODE_CACONFIG;
            }

        }
        if ((paramConfig->accelMode == HWA_ACCELMODE_COMPRESS) &&
                 (
                     (paramConfig->accelModeArgs.compressMode.compressDecompress > HWA_CMP_DCMP_DECOMPRESS) ||
                     (paramConfig->accelModeArgs.compressMode.ditherEnable >  HWA_FEATURE_BIT_ENABLE) ||
                     ((paramConfig->accelModeArgs.compressMode.method != HWA_COMPRESS_METHOD_BFP) &&
                     (paramConfig->accelModeArgs.compressMode.method != HWA_COMPRESS_METHOD_EGE)) ||
                     ((paramConfig->accelModeArgs.compressMode.passSelect != HWA_COMPRESS_PATHSELECT_BOTHPASSES) &&
                      (paramConfig->accelModeArgs.compressMode.passSelect != HWA_COMPRESS_PATHSELECT_SECONDPASS) )||
                     (paramConfig->accelModeArgs.compressMode.headerEnable >  HWA_FEATURE_BIT_ENABLE)   ||
                     (paramConfig->accelModeArgs.compressMode.scaleFactorBW >= ( 1U << 3U)) ||
                     (paramConfig->accelModeArgs.compressMode.BFPMantissaBW > ( 1 << 5U)) ||
                     ((paramConfig->accelModeArgs.compressMode.method == HWA_COMPRESS_METHOD_EGE) &&
                     ((paramConfig->accelModeArgs.compressMode.EGEKarrayLength > 3U ) ||(paramConfig->accelModeArgs.compressMode.EGEKarrayLength < 1U)))||
#if defined (SOC_AWR294X)
                     /* Below slefLfsr, cmpRoundEn and DecrImageBitw are valid for AWR294x ES2.0 device only. */
                     ((paramConfig->accelModeArgs.compressMode.selLfsr > 1 ) && (ptrHWADriver->isES2P0Device == true)) ||
                     ((paramConfig->accelModeArgs.compressMode.cmpRoundEn > 1 ) && (ptrHWADriver->isES2P0Device == true) && (paramConfig->accelModeArgs.compressMode.ditherEnable == HWA_FEATURE_BIT_ENABLE)) ||
                     ((paramConfig->accelModeArgs.compressMode.compressDecompress == HWA_CMP_DCMP_COMPRESS) && (paramConfig->source.srcRealComplex == HWA_SAMPLES_FORMAT_COMPLEX) &&
                      (paramConfig->accelModeArgs.compressMode.decrImagBitw > 1 ) && (ptrHWADriver->isES2P0Device == true)) ||
                      ((paramConfig->accelModeArgs.compressMode.compressDecompress == HWA_CMP_DCMP_DECOMPRESS) && (paramConfig->dest.dstRealComplex == HWA_SAMPLES_FORMAT_COMPLEX) &&
                      (paramConfig->accelModeArgs.compressMode.decrImagBitw > 1 ) && (ptrHWADriver->isES2P0Device == true)) ||
#endif

                      /* if first pass is disabled */
                      ( (paramConfig->accelModeArgs.compressMode.passSelect == HWA_COMPRESS_PATHSELECT_SECONDPASS) &&
                            (paramConfig->accelModeArgs.compressMode.scaleFactor >  (1U << 5U) ) &&
                        (
                           (paramConfig->accelModeArgs.compressMode.method == HWA_COMPRESS_METHOD_EGE) &&
                           (paramConfig->accelModeArgs.compressMode.EGEKidx > 31U)
                        )
                      )
                 )
            )
        {
            /* invalid config params */
            retCode = HWA_EINVAL_PARAMSET_COMPRESSMODE;
        }
        if ((paramConfig->accelMode == HWA_ACCELMODE_LOCALMAX) &&
                 (
                    (paramConfig->accelModeArgs.localMaxMode.thresholdBitMask > HWA_LOCALMAX_THRESH_BITMASK_BOTH_DIS) ||
                    (paramConfig->accelModeArgs.localMaxMode.thresholdMode > HWA_LOCALMAX_THRESH_SELECT_DIMBRAM_DIMCRAM)  ||
                    (paramConfig->accelModeArgs.localMaxMode.dimBNonCyclic > HWA_FEATURE_BIT_ENABLE) ||
                    (paramConfig->accelModeArgs.localMaxMode.dimCNonCyclic > HWA_FEATURE_BIT_ENABLE)

                 )
            )
        {
            /* invalid config params */
            retCode = HWA_EINVAL_PARAMSET_LOCALMAXMODE;
        }
    }
#endif
    return retCode;
}


/*!
 *  @brief  Function to de-initialize HWA specified by the passed driver object. No driver
 *  functions should be invoked after this call.
 *
 *  @pre    HWA_open() has been called
 *
 *  @param  ptrHWADriver         Internal HWA driver object
 *
 *  @return none
 *
 *  @sa     HWA_close()
 */
static void HWA_deleteInstance(HWA_Object *ptrHWADriver)
{
    uint32_t            index  = ptrHWADriver->instanceNum;

    /* un-register the interrupts */
    HwiP_destruct(&ptrHWADriver->hwiHandleParamSet);
    HwiP_destruct(&ptrHWADriver->hwiHandleDone);
    HwiP_destruct(&ptrHWADriver->hwiHandleParamSetALT);
    HwiP_destruct(&ptrHWADriver->hwiHandleDoneALT);

    /* free the allocated memory */
   if (ptrHWADriver->interruptCtxParamSet != NULL) {
        ptrHWADriver->interruptCtxParamSet = NULL;
    }

    gHwaObjectPtr[index]=NULL; /* reset the driver's cached handle too */
}


/*!
 *  @brief  Function to handle each paramset completion ISR for CPU interrupt 1
 *
 *  @pre    HWA_open() has been called
 *
 *  @param  arg         Internal HWA driver object
 *
 *  @return none
 *
 *  @sa     HWA_enableParamSetInterrupt()
 */
static void HWA_paramDoneIntr1ISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;
    uint32_t             paramDoneFlag0, paramDoneFlag1;
    uint32_t            loopCnt;
    DSSHWACCRegs        *ctrlBaseAddr;
    uint32_t             interruptParamDone0, interruptParamDone1;
    uint32_t             interruptParamMask0, interruptParamMask1;
    uint64_t             interruptDoneParam;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        interruptParamDone0 = 0;
        interruptParamDone1 = 0;
        interruptParamMask0 = (uint32_t)(ptrHWADriver->interrupt1ParamSetMask & 0xFFFFFFFFU);
        interruptParamMask1 = (uint32_t)((ptrHWADriver->interrupt1ParamSetMask >> 32U) & 0xFFFFFFFFU);

        /* read the interrupt flag from PARAMDONESTAT register */
        paramDoneFlag0 = ctrlBaseAddr->PARAM_DONE_SET_STATUS[0];

        /* clear the paramsets with interrupt enabled  */
        interruptParamDone0 = paramDoneFlag0 & interruptParamMask0;
        ctrlBaseAddr->PARAM_DONE_CLR[0] = interruptParamDone0;

        /* read the interrupt flag from PARAMDONESTAT register */
        paramDoneFlag1 = ctrlBaseAddr->PARAM_DONE_SET_STATUS[1];

        /* clear the paramsets with interrupt enabled   */
        interruptParamDone1 = paramDoneFlag1 & interruptParamMask1;
        ctrlBaseAddr->PARAM_DONE_CLR[1] = interruptParamDone1;

        interruptDoneParam = 0;
        interruptDoneParam = interruptParamDone0 | ( (uint64_t)interruptParamDone1 << 32U);

        /* now process the interrupts, with interrupt enabled and paramdone is set */
        for(loopCnt = 0U; (interruptDoneParam != 0U) && (loopCnt < 64U ); loopCnt++)
        {
            if((interruptDoneParam & ((uint64_t)1U << loopCnt)) != 0U)
            {
                HWA_InterruptCtx *interruptCtx = &ptrHWADriver->interruptCtxParamSet[loopCnt];
                if (interruptCtx->callbackFn != NULL)
                {
                    (interruptCtx->callbackFn)(HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1, loopCnt, interruptCtx->callbackArg);
                }
                interruptDoneParam &= ~((uint64_t)1U << loopCnt);
            }
        }

    }
    else
    {
        /* Throw fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }
}

/*!
*  @brief  Function to handle each paramset in ALT thread completion ISR
*
*  @pre    HWA_open() has been called
*
*  @param  arg         Internal HWA driver object
*
*  @return none
*
*  @sa     HWA_enableParamSetInterrupt()
*/
static void HWA_paramDoneIntr2ISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;
    uint32_t             paramDoneFlag0, paramDoneFlag1;
    uint32_t            loopCnt;
    DSSHWACCRegs        *ctrlBaseAddr;
    uint32_t             interruptParamDone0, interruptParamDone1;
    uint32_t             interruptParamMask0, interruptParamMask1;
    uint64_t             interruptDoneParam;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        interruptParamDone0 = 0;
        interruptParamDone1 = 0;
        interruptParamMask0 = (uint32_t)(ptrHWADriver->interrupt2ParamSetMask & 0xFFFFFFFFU);
        interruptParamMask1 = (uint32_t)((ptrHWADriver->interrupt2ParamSetMask >> 32U) & 0xFFFFFFFFU);

        /* read the interrupt flag from PARAMDONESTAT register */
        paramDoneFlag0 = ctrlBaseAddr->PARAM_DONE_SET_STATUS[0];

        /* clear the paramsets with interrupt enabled  */
        interruptParamDone0 = paramDoneFlag0 & interruptParamMask0;
        ctrlBaseAddr->PARAM_DONE_CLR[0] = interruptParamDone0;

        /* read the interrupt flag from PARAMDONESTAT register */
        paramDoneFlag1 = ctrlBaseAddr->PARAM_DONE_SET_STATUS[1];

        /* clear the paramsets with interrupt enabled   */
        interruptParamDone1 = paramDoneFlag1 & interruptParamMask1;
        ctrlBaseAddr->PARAM_DONE_CLR[1] = interruptParamDone1;

        interruptDoneParam = 0;
        interruptDoneParam = interruptParamDone0 | ( (uint64_t) interruptParamDone1 << 32U);

        /* now process the interrupts, with interrupt enabled and paramdone is set */
        for (loopCnt = 0U; (interruptDoneParam != 0U) && (loopCnt < 64U); loopCnt++)
        {
            if ((interruptDoneParam & ((uint64_t)1U << loopCnt)) != 0U)
            {
                HWA_InterruptCtx *interruptCtx = &ptrHWADriver->interruptCtxParamSet[loopCnt];
                if (interruptCtx->callbackFn != NULL)
                {
                    (interruptCtx->callbackFn)(HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2, loopCnt, interruptCtx->callbackArg);
                }
                interruptDoneParam &= ~((uint64_t)1U << loopCnt);
            }
        }
    }
    else
    {
        /* Throw fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }
}


/*!
 *  @brief  Function to handle completion ISR after all paramset are executed in the background thread.
 *
 *  @pre    HWA_open() has been called
 *
 *  @param  arg         Internal HWA driver object
 *
 *  @return none
 *
 *  @sa     HWA_enableDoneInterrupt()
 */
static void HWA_allParamDoneISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))
    {
        /* now process the interrupt */
        HWA_DoneInterruptCtx *interruptCtx = &ptrHWADriver->interruptCtxDone;
        if ((interruptCtx->bIsEnabled==true) && (interruptCtx->callbackFn != NULL))
        {
            (interruptCtx->callbackFn)(HWA_THREAD_BACKGROUNDCONTEXT, interruptCtx->callbackArg);
        }
    }
    else
    {
        /* Fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }
}


/*!
*  @brief  Function to handle completion ISR after all paramset are executed in the ALT thread.
*
*  @pre    HWA_open() has been called
*
*  @param  arg         Internal HWA driver object
*
*  @return none
*
*  @sa     HWA_enableDoneInterrupt()
*/
static void HWA_allALTParamDoneISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))
    {
        /* now process the interrupt */
        HWA_DoneInterruptCtx *interruptCtx = &ptrHWADriver->interruptCtxDoneALT;
        if ((interruptCtx->bIsEnabled == true) && (interruptCtx->callbackFn != NULL))
        {
            (interruptCtx->callbackFn)(HWA_THREAD_ALTCONTEXT, interruptCtx->callbackArg);
        }
    }
    else
    {
        /* Fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }
}

/*!
*  @brief  Function to handle error on accessing to Local RAM.
*
*  @pre    HWA_open() has been called
*
*  @param  arg         Internal HWA driver object
*
*  @return none
*
*/
static void HWA_localRamErrorISR(void *arg)
{
    HWA_Object          *ptrHWADriver = NULL;

    HWA_GET_DRIVER_STRUCT(arg); /* this fills the ptrHWADriver */

    if ((ptrHWADriver != NULL) && (ptrHWADriver->refCnt > 0U))
    {
        /* Local RAM error is a critical error. */
        DebugP_assert(0U);
    }
    else
    {
        /* Fatal error as driver is not in valid state */
        DebugP_assert(0U);
    }
}

/*!
 *  @brief  Function to perform the hardware reset of HWA peripheral
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 *  @sa     HWA_close(), HWA_open(), HWA_reset()
 */
static inline void HWA_doReset(DSSHWACCRegs  *ctrlBaseAddr)
{
    CSL_FINSR (ctrlBaseAddr->HWA_ENABLE, HWA_ENABLE_HWA_RESET_END, HWA_ENABLE_HWA_RESET_START, 0x7U);
    CSL_FINSR (ctrlBaseAddr->HWA_ENABLE, HWA_ENABLE_HWA_RESET_END, HWA_ENABLE_HWA_RESET_START, 0x0U);

    return;
}

/*!
*  @brief  Function to clear the histogram RAM
*
*  @param  ctrlBaseAddr    HWA peripheral's control base address
*  @param  ramMemBankMask  Bit mask to specify HWA MEM BANK,
*                           see \ref HWA_APP_MEMINIT_CFG for correct values
*                           HWA_APP_MEMINIT_PARAM_RAM
*                           HWA_APP_MEMINIT_WINDOW_RAM
*                           HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM
*                           HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM
*                           HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM
*                           HWA_APP_MEMINIT_HIST_EVEN_RAM
*                           HWA_APP_MEMINIT_HIST_ODD_RAM
*                           HWA_APP_MEMINIT_MEMBANK_ALL
*
*  @return none
*
*  @sa     HWA_close(), HWA_open(), HWA_reset()
*/
static void HWA_startRAMInit(DSSHWACCRegs  *ctrlBaseAddr, uint32_t ramMemBankMask)
{
    uint32_t memBankInit = 0;

    if (ramMemBankMask & HWA_APP_MEMINIT_PARAM_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_PARAM_RAM;
    }

    if (ramMemBankMask & HWA_APP_MEMINIT_WINDOW_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_WINDOW_RAM;
    }

    if (ramMemBankMask & HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM;
    }

    if (ramMemBankMask & HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM;
    }

    if (ramMemBankMask & HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM;
    }

    if (ramMemBankMask & HWA_APP_MEMINIT_HIST_EVEN_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_HIST_EVEN_RAM;
    }

    if (ramMemBankMask & HWA_APP_MEMINIT_HIST_ODD_RAM)
    {
        memBankInit |= HWA_APP_MEMINIT_HIST_ODD_RAM;
    }

    /* Start Memory Initializations for selected memory Banks. */
    ctrlBaseAddr->MEM_INIT_START = memBankInit;

    return;
}

/*!
*  @brief  Function to wait for RAM initialization to complete
*
*  @param  ctrlBaseAddr    HWA peripheral's control base address
*  @param  ramMemBankMask  Bit mask to specify HWA MEM BANK,
*                           see \ref HWA_APP_MEMINIT_CFG for correct values
*                           HWA_APP_MEMINIT_PARAM_RAM
*                           HWA_APP_MEMINIT_WINDOW_RAM
*                           HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM
*                           HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM
*                           HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM
*                           HWA_APP_MEMINIT_HIST_EVEN_RAM
*                           HWA_APP_MEMINIT_HIST_ODD_RAM
*                           HWA_APP_MEMINIT_MEMBANK_ALL
*
*  @return none
*
*  @sa     HWA_startRAMInit()
*/
static void HWA_waitRAMInit(DSSHWACCRegs  *ctrlBaseAddr, uint32_t ramMemBankMask)
{
    uint32_t clearMemInitMask = 0;

    if(ramMemBankMask & HWA_APP_MEMINIT_PARAM_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PARAM_RAM_END, MEM_INIT_DONE_PARAM_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_PARAM_RAM;
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_WINDOW_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_WINDOW_RAM_END, MEM_INIT_DONE_WINDOW_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_WINDOW_RAM;
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_EVEN_RAM_END, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_EVEN_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM;
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_ODD_RAM_END, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_ODD_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM;
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PER_ITERATION_MAX_VAL_RAM_END, MEM_INIT_DONE_PER_ITERATION_MAX_VAL_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM;
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_HIST_EVEN_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_HIST_EVEN_RAM_END, MEM_INIT_DONE_HIST_EVEN_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_HIST_EVEN_RAM;
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_HIST_ODD_RAM)
    {
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_HIST_ODD_RAM_END, MEM_INIT_DONE_HIST_ODD_RAM_START) != 1);
        clearMemInitMask |= HWA_APP_MEMINIT_HIST_ODD_RAM;
    }

    /* Clear Memory Initialization done bits for selected memory Banks. */
    ctrlBaseAddr->MEM_INIT_DONE = clearMemInitMask;

    if(ramMemBankMask & HWA_APP_MEMINIT_PARAM_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_PARAM_RAM_END, MEM_INIT_STATUS_PARAM_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PARAM_RAM_END, MEM_INIT_DONE_PARAM_RAM_START) != 0);
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_WINDOW_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_WINDOW_RAM_END, MEM_INIT_STATUS_WINDOW_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_WINDOW_RAM_END, MEM_INIT_DONE_WINDOW_RAM_START) != 0);
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_PER_SAMPLE_MAX_VAL_EVEN_RAM_END, MEM_INIT_STATUS_PER_SAMPLE_MAX_VAL_EVEN_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_EVEN_RAM_END, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_EVEN_RAM_START) != 0);
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_PER_SAMPLE_MAX_VAL_ODD_RAM_END, MEM_INIT_STATUS_PER_SAMPLE_MAX_VAL_ODD_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_ODD_RAM_END, MEM_INIT_DONE_PER_SAMPLE_MAX_VAL_ODD_RAM_START) != 0);
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_PER_ITERATION_MAX_VAL_RAM_END, MEM_INIT_STATUS_PER_ITERATION_MAX_VAL_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_PER_ITERATION_MAX_VAL_RAM_END, MEM_INIT_DONE_PER_ITERATION_MAX_VAL_RAM_START) != 0);
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_HIST_EVEN_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_HIST_EVEN_RAM_END, MEM_INIT_STATUS_HIST_EVEN_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_HIST_EVEN_RAM_END, MEM_INIT_DONE_HIST_EVEN_RAM_START) != 0);
    }

    if(ramMemBankMask & HWA_APP_MEMINIT_HIST_ODD_RAM)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_HIST_ODD_RAM_END, MEM_INIT_STATUS_HIST_ODD_RAM_START) != 0);
        while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_HIST_ODD_RAM_END, MEM_INIT_DONE_HIST_ODD_RAM_START) != 0);
    }

    return;
}

/*!
 *  @brief  Function to start DMEM0 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem0Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM0 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM0_END, MEM_INIT_STATUS_DMEM0_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM0 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM0_END, MEM_INIT_DONE_DMEM0_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM0_END, MEM_INIT_DONE_DMEM0_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM0_END, MEM_INIT_START_DMEM0_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM0 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem0Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM0_END, MEM_INIT_DONE_DMEM0_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM0_END, MEM_INIT_DONE_DMEM0_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM0_END, MEM_INIT_STATUS_DMEM0_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM0_END, MEM_INIT_DONE_DMEM0_START) != 0);
}

/*!
 *  @brief  Function to start DMEM1 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem1Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM1 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM1_END, MEM_INIT_STATUS_DMEM1_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM1 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM1_END, MEM_INIT_DONE_DMEM1_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM1_END, MEM_INIT_DONE_DMEM1_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM1_END, MEM_INIT_START_DMEM1_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM1 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */

static void HWA_waitDmem1Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM1_END, MEM_INIT_DONE_DMEM1_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM1_END, MEM_INIT_DONE_DMEM1_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM1_END, MEM_INIT_STATUS_DMEM1_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM1_END, MEM_INIT_DONE_DMEM1_START) != 0);
}

/*!
 *  @brief  Function to start DMEM2 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem2Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM2 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM2_END, MEM_INIT_STATUS_DMEM2_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM2 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM2_END, MEM_INIT_DONE_DMEM2_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM2_END, MEM_INIT_DONE_DMEM2_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM2_END, MEM_INIT_START_DMEM2_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM2 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem2Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM2_END, MEM_INIT_DONE_DMEM2_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM2_END, MEM_INIT_DONE_DMEM2_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM2_END, MEM_INIT_STATUS_DMEM2_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM2_END, MEM_INIT_DONE_DMEM2_START) != 0);
}

/*!
 *  @brief  Function to start DMEM3 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem3Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM3 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM3_END, MEM_INIT_STATUS_DMEM3_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM3 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM3_END, MEM_INIT_DONE_DMEM3_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM3_END, MEM_INIT_DONE_DMEM3_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM3_END, MEM_INIT_START_DMEM3_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM3 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem3Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM3_END, MEM_INIT_DONE_DMEM3_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM3_END, MEM_INIT_DONE_DMEM3_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM3_END, MEM_INIT_STATUS_DMEM3_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM3_END, MEM_INIT_DONE_DMEM3_START) != 0);
}

/*!
 *  @brief  Function to start DMEM4 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem4Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM4 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM4_END, MEM_INIT_STATUS_DMEM4_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM4 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM4_END, MEM_INIT_DONE_DMEM4_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM4_END, MEM_INIT_DONE_DMEM4_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM4_END, MEM_INIT_START_DMEM4_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM4 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem4Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM4_END, MEM_INIT_DONE_DMEM4_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM4_END, MEM_INIT_DONE_DMEM4_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM4_END, MEM_INIT_STATUS_DMEM4_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM4_END, MEM_INIT_DONE_DMEM4_START) != 0);
}

/*!
 *  @brief  Function to start DMEM5 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem5Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM5 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM5_END, MEM_INIT_STATUS_DMEM5_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM5 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM5_END, MEM_INIT_DONE_DMEM5_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM5_END, MEM_INIT_DONE_DMEM5_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM5_END, MEM_INIT_START_DMEM5_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM5 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem5Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM5_END, MEM_INIT_DONE_DMEM5_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM5_END, MEM_INIT_DONE_DMEM5_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM5_END, MEM_INIT_STATUS_DMEM5_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM5_END, MEM_INIT_DONE_DMEM5_START) != 0);
}

/*!
 *  @brief  Function to start DMEM6 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem6Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM6 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM6_END, MEM_INIT_STATUS_DMEM6_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM6 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM6_END, MEM_INIT_DONE_DMEM6_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM6_END, MEM_INIT_DONE_DMEM6_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM6_END, MEM_INIT_START_DMEM6_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM6 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem6Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM6_END, MEM_INIT_DONE_DMEM6_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM6_END, MEM_INIT_DONE_DMEM6_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM6_END, MEM_INIT_STATUS_DMEM6_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM6_END, MEM_INIT_DONE_DMEM6_START) != 0);
}

/*!
 *  @brief  Function to start DMEM7 memory initialization
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_startDmem7Init(DSSHWACCRegs *ctrlBaseAddr)
{
    /* Check MEM_INIT_STATUS_DMEM7 is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM7_END, MEM_INIT_STATUS_DMEM7_START) != 0);

    /* Clear MEM_INIT_STATUS_DMEM7 DONE before initiating MEMINIT */
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM7_END, MEM_INIT_DONE_DMEM7_START, 1);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM7_END, MEM_INIT_DONE_DMEM7_START) != 0);

    CSL_FINSR(ctrlBaseAddr->MEM_INIT_START, MEM_INIT_START_DMEM7_END, MEM_INIT_START_DMEM7_START, 1);
}

/*!
 *  @brief  Function to wait for DMEM7 memory initialization completion
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_waitDmem7Init(DSSHWACCRegs *ctrlBaseAddr)
{
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM7_END, MEM_INIT_DONE_DMEM7_START) != 1);
    CSL_FINSR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM7_END, MEM_INIT_DONE_DMEM7_START, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_STATUS, MEM_INIT_STATUS_DMEM7_END, MEM_INIT_STATUS_DMEM7_START) != 0);
    while (CSL_FEXTR(ctrlBaseAddr->MEM_INIT_DONE, MEM_INIT_DONE_DMEM7_END, MEM_INIT_DONE_DMEM7_START) != 0);
}

/*!
 *  @brief  Function to perform DMEM0-DMEM7 memory initialization of HWA peripheral
 *
 *  @param  ctrlBaseAddr         HWA peripheral's control base address
 *
 *  @return none
 *
 */
static void HWA_dmemInit(DSSHWACCRegs *ctrlBaseAddr)
{
    /* DMEM0 */
    HWA_startDmem0Init(ctrlBaseAddr);
    HWA_waitDmem0Init(ctrlBaseAddr);

    /* DMEM1 */
    HWA_startDmem1Init(ctrlBaseAddr);
    HWA_waitDmem1Init(ctrlBaseAddr);

    /* DMEM2 */
    HWA_startDmem2Init(ctrlBaseAddr);
    HWA_waitDmem2Init(ctrlBaseAddr);

    /* DMEM3 */
    HWA_startDmem3Init(ctrlBaseAddr);
    HWA_waitDmem3Init(ctrlBaseAddr);

    /* DMEM4 */
    HWA_startDmem4Init(ctrlBaseAddr);
    HWA_waitDmem4Init(ctrlBaseAddr);

    /* DMEM5 */
    HWA_startDmem5Init(ctrlBaseAddr);
    HWA_waitDmem5Init(ctrlBaseAddr);

    /* DMEM6 */
    HWA_startDmem6Init(ctrlBaseAddr);
    HWA_waitDmem6Init(ctrlBaseAddr);

    /* DMEM7 */
    HWA_startDmem7Init(ctrlBaseAddr);
    HWA_waitDmem7Init(ctrlBaseAddr);

    return;
}
/** @}*/

/* EXTERNAL FUNCTIONS */


/*!
 *  @brief  Function to initialize the HWA module
 *
 *  @pre    This function must be called once per system and before
 *          any other HWA driver APIs. It resets the HWA H/W instances in the system.
 *
 */
void HWA_init(void)
{
    return;
}

void HWA_deinit(void)
{
    return;
}

/*!
 *  @brief  Function to initialize HWA specified by the
 *  particular index value.
 *
 *  @pre    HWA_init() has been called
 *
 *  @param  index         HWA instance number
 *  @param  hwaCfg        pointer to HWA configuration, if set to NULL, default values will be used.
 *  @param  errCode       [out] valid errorCode if NULL handle returned.
 *
 *  @return A HWA_Handle upon success. NULL if an error occurs.
 *
 *  @sa     HWA_init()
 *  @sa     HWA_close()
 */
extern HWA_Handle HWA_open(uint32_t index, HWA_OpenConfig * hwaCfg, int32_t* errCode)

{
    HWA_Handle          handle = NULL;
    uint32_t            memReqSize = 0U;
    uintptr_t           key;
    int32_t             retCode = 0;
    HwiP_Params         hwiPrms;

    if(index >= gHwaConfigNum)
    {
        retCode = HWA_EOUTOFRANGE;
    }
    else /* start of if valid index */
    {

        /* Disable preemption while opening the driver */
        key = HwiP_disable();

        /*
         * check if driver is already init
         */
        if (gHwaObjectPtr[index] == NULL)
        {
            /* Allocate memory for the driver and initialize it */
            gHwaObjectPtr[index] = &gHwaObject[index];
            memset ((void *)gHwaObjectPtr[index], 0U, sizeof(HWA_Object));
            gHwaObjectPtr[index]->hwAttrs = &gHwaAttrs[index];
            gHwaObjectPtr[index]->instanceNum = index;

            /* initialize internal driver struture memory */
            /* malloc interruptCtrx */
            memReqSize = (uint32_t)(sizeof(HWA_InterruptCtx) * gHwaObjectPtr[index]->hwAttrs->numHwaParamSets);
            gHwaObjectPtr[index]->interruptCtxParamSet = &HwaParamsetIntr[0];
            memset ((void *)gHwaObjectPtr[index]->interruptCtxParamSet, 0U, memReqSize);
            if (gHwaObjectPtr[index]->interruptCtxParamSet == NULL)
            {
                /* Error: Out of memory */
                DebugP_log("Debug: HWA Driver (%d) Out of memory for interruptCtxParamSet(requested size: %d)\n",
                              index,memReqSize);
                retCode = HWA_EOUTOFMEM;
            }
            else
            {
                memset(gHwaObjectPtr[index]->interruptCtxParamSet,0U,memReqSize);
                /* dont set the handle yet as we dont know if all the mem allocs are going to succeed */
            }
        }

        /*
         * now do config if we got the driver memory allocated and primed
         */
        if ((retCode==0U) && (gHwaObjectPtr[index] != NULL) && (gHwaObjectPtr[index]->refCnt==0U))
        {
            DSSHWACCRegs *ctrlBaseAddr = (DSSHWACCRegs *)gHwaObjectPtr[index]->hwAttrs->ctrlBaseAddr;
            DSSHWACCPARAMRegs *paramBaseAddr = (DSSHWACCPARAMRegs *)(gHwaObjectPtr[index]->hwAttrs->paramBaseAddr);

            /* Register interrupt handlers */
            /* Each Paramset done interrupt 1 */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_paramDoneIntr1ISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNum1ParamSet;
            if(hwaCfg == NULL)
            {
                hwiPrms.priority    = HWA_PARAMSETDONE_INTERRUPT1_PRIORITY;
            }
            else
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.paramsetDone1;
            }
            hwiPrms.isPulse         = true;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleParamSet, &hwiPrms);
            if(SystemP_SUCCESS != retCode)
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }

            /* All programmed Paramsets done interrupt */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_allParamDoneISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNumDone;
            if(hwaCfg == NULL)
            {
                hwiPrms.priority    = HWA_DONE_INTERRUPT_PRIORITY;
            }
            else
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.backgroundDone;
            }
            hwiPrms.isPulse         = true;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleDone, &hwiPrms);
            if(SystemP_SUCCESS != retCode)
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }

            /* Each Paramset done interrupt 2 */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_paramDoneIntr2ISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNum2ParamSet;
            if(hwaCfg == NULL)
            {
                hwiPrms.priority    = HWA_PARAMSETDONE_INTERRUPT2_PRIORITY;
            }
            else
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.paramsetDone2;
            }
            hwiPrms.isPulse         = true;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleParamSetALT, &hwiPrms);
            if(SystemP_SUCCESS != retCode)
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }

            /* All programmed Paramsets HWA ALT done interrupt */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_allALTParamDoneISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNumDoneALT;
            if(hwaCfg == NULL)
            {
                hwiPrms.priority    = HWA_ALTDONE_INTERRUPT_PRIORITY;
            }
            else
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.ALTDone;
            }
            hwiPrms.isPulse         = true;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleDoneALT, &hwiPrms);
            if(SystemP_SUCCESS != retCode)
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }

            /* Configure HWA_LOCAL_RAM_ERR interrupt */
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args            = gHwaObjectPtr[index];
            hwiPrms.callback        = &HWA_localRamErrorISR;
            hwiPrms.intNum          = gHwaObjectPtr[index]->hwAttrs->intNumLocalRamErr;
            if(hwaCfg == NULL)
            {
                hwiPrms.priority    = HWA_LOCAL_RAM_ERR_PRIORITY;
            }
            else
            {
                hwiPrms.priority    = hwaCfg->interruptPriority.loalRamErr;
            }
            hwiPrms.isPulse         = false;
            /* Register interrupts */
            retCode = HwiP_construct(&gHwaObjectPtr[index]->hwiHandleLocalRamErr, &hwiPrms);
            if(SystemP_SUCCESS != retCode)
            {
                DebugP_log("Error Could not register ISR !!!\n");
            }

            /* Enable HWA_LOCAL_RAM_ERR interrupt */
            HwiP_enableInt(gHwaObjectPtr[index]->hwAttrs->intNumLocalRamErr);

            {
                //uint32_t * dbgAckCtl1Ptr;
                CSL_dss_ctrlRegs        *dssCtrlRegsPtr;

                /* unlock the HWA registers */
                ctrlBaseAddr->LOCK0_KICK0 = 0x01234567U;
                ctrlBaseAddr->LOCK0_KICK1 = 0xFEDCBA8U;


                /* set DBG_ACK_CTL1_DSS_HWA in DSS_CTRL:DBG_ACK_CTL1*/
                //dbgAckCtl1Ptr = (uint32_t *) (gHwaObjectPtr[index]->hwAttrs->dssBaseAddr + 0x58C);
                //*dbgAckCtl1Ptr |= 0x10000000;   /* set bit 28- 30 (DBG_ACK_CTL1_DSS_HWA) to 1*/
                dssCtrlRegsPtr  = (CSL_dss_ctrlRegs *) gHwaObjectPtr[index]->hwAttrs->dssBaseAddr;
                CSL_FINSR(dssCtrlRegsPtr->DBG_ACK_CTL1,30U,28U, 1U);

                /*disable accelerator and clock*/
                ctrlBaseAddr->HWA_ENABLE = 0x0U;

                /*now reset the hwa*/
                HWA_doReset(ctrlBaseAddr);
                /*leave hw_acc disabled but enable the clock*/
                CSL_FINSR(ctrlBaseAddr->HWA_ENABLE,
                          HWA_ENABLE_HWA_CLK_EN_END,
                          HWA_ENABLE_HWA_CLK_EN_START,
                          0x7U);

                /* TODO: 1 bit dynamic clock  */
                /*
                CSL_FINSR(ctrlBaseAddr->HWA_ENABLE,
                          HWA_ENABLE_HWA_DYN_CLK_EN_END,
                          HWA_ENABLE_HWA_DYN_CLK_EN_START,
                          0x1U);
                */

                /*TODO: check if needed for real silicon*/
                /* reset paramset */
                 memset((void *)paramBaseAddr, 0U, sizeof(DSSHWACCPARAMRegs)*gHwaObjectPtr[index]->hwAttrs->numHwaParamSets);
                 /* clear the PARAM_DONE_SET_STATUS_0 and PARAM_DONE_SET_STATUS_1*/
                 ctrlBaseAddr->PARAM_DONE_CLR[0] = 0xFFFFFFFFU;
                 ctrlBaseAddr->PARAM_DONE_CLR[1] = 0xFFFFFFFFU;
                 /* clear the TRIGGER_SET_STATUS_0 and TRIGGER_SET_STATUS_1*/
                 ctrlBaseAddr->TRIGGER_SET_IN_CLR[0] = 0xFFFFFFFFU;
                 ctrlBaseAddr->TRIGGER_SET_IN_CLR[1] = 0xFFFFFFFFU;
                 /* clear the FFTCLIP register */
                 ctrlBaseAddr->CLR_FFTCLIP = 0x1U;

                 /* Initialize DMEM0-DMEM7 meory regions. */
                 HWA_dmemInit(ctrlBaseAddr);

                 /* clear the HISTOGRAM RAM */
                 HWA_startRAMInit(ctrlBaseAddr, (uint32_t)(HWA_APP_MEMINIT_HIST_EVEN_RAM | HWA_APP_MEMINIT_HIST_ODD_RAM));
                 HWA_waitRAMInit(ctrlBaseAddr, (uint32_t)(HWA_APP_MEMINIT_HIST_EVEN_RAM | HWA_APP_MEMINIT_HIST_ODD_RAM));
                 /* clear other clip registers*/
                 ctrlBaseAddr->CLR_CLIP_MISC = 0x1U;
            }
        }

        /*
         * check before returning
         */
        if (retCode == 0U)
        {
            /* increment reference count */
            gHwaObjectPtr[index]->refCnt++;
            /* Setup the return handle: */
            handle = (void *)gHwaObjectPtr[index];
        }
        else
        {
            handle = NULL;
            /* errCode is already set */
            HWA_deleteInstance(gHwaObjectPtr[index]);
        }

#if defined (SOC_AWR294X)

        /* Get the PG version here. */
        if(SOC_rcmGetEfusePGVer() == SOC_RCM_ES2_PG_VER)
        {
            gHwaObjectPtr[index]->isES2P0Device = true;
        }
        else
        {
            gHwaObjectPtr[index]->isES2P0Device = false;
        }

#endif

        /* Restore the interrupts: */
        HwiP_restore(key);

    }/* end of if valid index */

    /* return */
    if (errCode!=NULL)
    {
        *errCode = retCode;
    }
    return handle;
}


/*!
 *  @brief  Function to close a HWA peripheral specified by the HWA handle
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle      A HWA_Handle returned from HWA_open()
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_close(HWA_Handle handle)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    uintptr_t           key;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* Disable preemption while opening the driver */
    key = HwiP_disable();

    if (ptrHWADriver!=NULL)
    {
        if ((ptrHWADriver->configInProgress == 0U) && (ptrHWADriver->paramSetMapInProgress == 0U))
        {
            /* decrement the refCount and if this is the last reference, delete the instance */
            if (ptrHWADriver->refCnt>0U)
            {
                ptrHWADriver->refCnt--;
            }
            if (ptrHWADriver->refCnt==0U)
            {
                uint32_t *mssToprcmRegPtr;
                ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
                /* disable the HWA state machine */
                CSL_FINSR(ctrlBaseAddr->HWA_ENABLE, HWA_ENABLE_HWA_EN_END, HWA_ENABLE_HWA_EN_START,0x0U);
                /* do hardware reset */
                HWA_doReset(ctrlBaseAddr);
                /* what else to reset??, lock the register  */
                ctrlBaseAddr->LOCK0_KICK0 = 0x0;
                ctrlBaseAddr->LOCK0_KICK1 = 0x0;

                /* unlock the param mem */
                mssToprcmRegPtr = (uint32_t *)0x02141008U;  //TPR:MSS_TOPRCM:LOCK0_KICK0 physical address
                *mssToprcmRegPtr = 0x0;
                mssToprcmRegPtr = (uint32_t *)0x0214100CU;  //TPR:MSS_TOPRCM:LOCK1_KICK1 physical address
                *mssToprcmRegPtr = 0x0;
              //  mssToprcmRegPtr = (uint32_t *)0x02140044;  //TPR:MSS_TOPRCM:SYS_CLK_DIV_VAL
              //  *mssToprcmRegPtr = 0x111;


                /* delete the instance */
                HWA_deleteInstance(ptrHWADriver);
            }
        }
        else
        {
            /* return inuse code */
            retCode = HWA_EINUSE;
        }
    }
    else
    {
        /* return invalid code */
        retCode = HWA_EINVAL;
    }
    /* Restore the interrupts */
    HwiP_restore(key);

    return retCode;
}


/*!
 *  @brief  Function to reset the internal state machine of the HWA
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */

int32_t HWA_reset(HWA_Handle handle)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        HWA_doReset(ctrlBaseAddr);
        HWA_releaseDriverAccess(ptrHWADriver,(bool)true,(bool)false,0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to clear the Histogram RAM in HWA
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*  @param  ramMemBankMask  Bit mask to specify HWA MEM BANK,
*                           see \ref HWA_APP_MEMINIT_CFG for correct values
*                           HWA_APP_MEMINIT_PARAM_RAM
*                           HWA_APP_MEMINIT_WINDOW_RAM
*                           HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_EVEN_RAM
*                           HWA_APP_MEMINIT_PER_SAMPLE_MAX_VAL_ODD_RAM
*                           HWA_APP_MEMINIT_PER_ITER_MAX_VAL_RAM
*                           HWA_APP_MEMINIT_HIST_EVEN_RAM
*                           HWA_APP_MEMINIT_HIST_ODD_RAM
*                           HWA_APP_MEMINIT_MEMBANK_ALL
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/

extern int32_t HWA_initializeRAM(HWA_Handle handle, uint32_t ramMemBankMask)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs        *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        HWA_startRAMInit(ctrlBaseAddr, ramMemBankMask);
        HWA_waitRAMInit(ctrlBaseAddr, ramMemBankMask);
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to set the common HWA configuration parameters
 *          needed for the next operations/iterations/paramsets of HWA
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  commonConfig    HWA Common Config Parameters
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_configCommon(HWA_Handle handle, HWA_CommonConfig *commonConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs        *ctrlBaseAddr;
    int32_t              ii;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (commonConfig == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG) &&
                  ((commonConfig->numLoops > HWA_MAXNUM_LOOPS) ||
                   (commonConfig->paramStartIdx > (ptrHWADriver->hwAttrs->numHwaParamSets - 1)) ||
                   (commonConfig->paramStopIdx > (ptrHWADriver->hwAttrs->numHwaParamSets - 1)))
                 )
               ||((commonConfig->configMask & HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT) &&
                   ((commonConfig->numLoopsALT > HWA_MAXNUM_LOOPS) ||
                    (commonConfig->paramStartIdxALT > (ptrHWADriver->hwAttrs->numHwaParamSets - 1)) ||
                    (commonConfig->paramStopIdxALT > (ptrHWADriver->hwAttrs->numHwaParamSets - 1)))
                 ) )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_PARAMSET;
        }
        else if ((commonConfig->configMask & HWA_COMMONCONFIG_CONTEXTSWITCH_TRIG_CFG) &&
                 (((commonConfig->contextswitchTriggerMode != HWA_CONTEXTSWITCH_TRIG_MODE_DMA) &&
                   (commonConfig->contextswitchTriggerMode != HWA_CONTEXTSWITCH_TRIG_MODE_HARDWARE) &&
                   (commonConfig->contextswitchTriggerMode != HWA_CONTEXTSWITCH_TRIG_MODE_SOFTWARE) ) ||
                   ((commonConfig->contextswitchTriggerMode == HWA_CONTEXTSWITCH_TRIG_MODE_HARDWARE) &&
                    (commonConfig->contextswitchTriggerSrc > 19U)) || //hardware trigger, Valid programmation 0-19
                   ((commonConfig->contextswitchTriggerMode == HWA_CONTEXTSWITCH_TRIG_MODE_DMA) &&
                    (commonConfig->contextswitchTriggerSrc > (ptrHWADriver->hwAttrs->numDmaChannels - 1) ))
                ))
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_PARAMSET_ALT;
        }
        else if (
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_BPMCFG) &&
             ((commonConfig->fftConfig.bpmRate > (1U << 10U)) || (commonConfig->fftConfig.bpmRate < 1U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE) &&
            (commonConfig->fftConfig.twidDitherEnable > HWA_FEATURE_BIT_ENABLE)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LFSRSEED) &&
             (commonConfig->fftConfig.lfsrSeed > (1U << 29U)) ) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_FFTSUMDIV) && (commonConfig->fftConfig.fftSumDiv > (1U << 5U)))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_FFTCONFIG;
        }
        else if (
                 (commonConfig->configMask & HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT) &&
                 ((commonConfig->dcEstimateConfig.scale > (1U << 9U)) || (commonConfig->dcEstimateConfig.shift > 14U))
               )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_DCEST;
        }
        else if (
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE) && (commonConfig->cfarConfig.thresholdScale > (1U << 18U)))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_CFAR;
        }
        else if (
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFAR_DET_THR) && (commonConfig->cfarDetThresConfig.cfarDetthreshold > (1U << 23U)))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_CFAR_DET_THR;
        }
        else if (
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAG) &&
             ((commonConfig->interfConfig.sumMagShift > 6U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAGDIFF) &&
             ((commonConfig->interfConfig.sumMagDiffShift > 6U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM) && (commonConfig->interfConfig.mitigationWindowParam[0] > (1U << 5U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM) && (commonConfig->interfConfig.mitigationWindowParam[1] > (1U << 5U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM) && (commonConfig->interfConfig.mitigationWindowParam[2] > (1U << 5U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM) && (commonConfig->interfConfig.mitigationWindowParam[3] > (1U << 5U))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM) && (commonConfig->interfConfig.mitigationWindowParam[4] > (1U << 5U)))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_INTERFERENCE;
        }
        else if (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_TWIDINCR_DELTA_FRAC) &&
                  ((commonConfig->complexMultiplyConfig.twiddleDeltaFrac > 511) || (commonConfig->complexMultiplyConfig.twiddleDeltaFrac < -512))) ||
                 ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_RECWIN_RESET) && (commonConfig->complexMultiplyConfig.recWindowReset > HWA_FEATURE_BIT_ENABLE))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_COMPLEXMULT;
        }
        else if (((commonConfig->configMask & HWA_COMMONCONFIG_MASK_MAX2D_OFFSETBOTHDIM) && (commonConfig->advStatConfig.max2DoffsetDim1 > (1 << 24))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_MAX2D_OFFSETBOTHDIM) && (commonConfig->advStatConfig.max2DoffsetDim2 > (1 << 24))) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CDFCNT_THRESHOLD) && (commonConfig->advStatConfig.cdfCntThresh > 4095U))
            )
        {
            /* invalid config */
            retCode = HWA_EINVAL_COMMON_REGISTER_ADVSTAT;
        }
        else if ( //((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMB_THRESHOLDSW) && (commonConfig->localMaxConfig.dimBThreshold > ( 1 << 16U))) ||
             //((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMC_THRESHOLDSW) && (commonConfig->localMaxConfig.dimCThreshold > (1 << 16U)))||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMBTHRESH_OFFSET) && (commonConfig->localMaxConfig.dimBBaseAddress > (1U << 12U)))||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMCTHRESH_OFFSET) && (commonConfig->localMaxConfig.dimCBaseAddress > (1U << 12U)))
            )
        {
            retCode = HWA_EINVAL_COMMON_REGISTER_LOCALMAXIMUM;
        }
        else if (
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[0] >= 32U)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[1] >= 32U)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[2] >= 32U)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[3] >= 32U)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[4] >= 32U)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[5] >= 32U)) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[6] >= 32U)) ||
#if defined (SOC_AWR294X)
            /* Below LFSRSEED0, LFSRSEED1 are valid for AWR294x ES2.0 device only. */
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CMP_LFSRSEED0) && (commonConfig->compressConfig.cmpLfsrSeed0 > (1U << 29U)) && (ptrHWADriver->isES2P0Device == true) ) ||
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_CMP_LFSRSEED1) && (commonConfig->compressConfig.cmpLfsrSeed1 > (1U << 29U)) && (ptrHWADriver->isES2P0Device == true) ) ||
#endif
            ((commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM) && (commonConfig->compressConfig.EGEKparam[7] >= 32U))
           )
        {
            /* invalid config params */
            retCode = HWA_EINVAL_COMMON_REGISTER_COMPRESS;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG)
            {
                CSL_FINSR(ctrlBaseAddr->PARAM_RAM_LOOP,
                          PARAM_RAM_LOOP_NUMLOOPS_END,
                          PARAM_RAM_LOOP_NUMLOOPS_START,
                          commonConfig->numLoops);

                CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX,
                          PARAM_RAM_IDX_PARAM_START_IDX_END,
                          PARAM_RAM_IDX_PARAM_START_IDX_START,
                          commonConfig->paramStartIdx);

                CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX,
                          PARAM_RAM_IDX_PARAM_END_IDX_END,
                          PARAM_RAM_IDX_PARAM_END_IDX_START,
                          commonConfig->paramStopIdx);
            }
            /* set the alt config */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG_ALT)
            {
                CSL_FINSR(ctrlBaseAddr->PARAM_RAM_LOOP_ALT,
                          PARAM_RAM_LOOP_ALT_NUMLOOPS_END,
                          PARAM_RAM_LOOP_ALT_NUMLOOPS_START,
                          commonConfig->numLoopsALT);

                CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX_ALT,
                          PARAM_RAM_IDX_ALT_PARAM_START_IDX_END,
                          PARAM_RAM_IDX_ALT_PARAM_START_IDX_START,
                          commonConfig->paramStartIdxALT);

                CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX_ALT,
                          PARAM_RAM_IDX_ALT_PARAM_END_IDX_END,
                          PARAM_RAM_IDX_ALT_PARAM_END_IDX_START,
                          commonConfig->paramStopIdxALT);
            }
            /* set the CS config */
            if (commonConfig->configMask & HWA_COMMONCONFIG_CONTEXTSWITCH_TRIG_CFG)
            {
                CSL_FINSR(ctrlBaseAddr->CS_CONFIG,
                          CS_CONFIG_CS_TRIGMODE_END,
                          CS_CONFIG_CS_TRIGMODE_START,
                          commonConfig->contextswitchTriggerMode);

                if ((commonConfig->contextswitchTriggerMode == HWA_CONTEXTSWITCH_TRIG_MODE_DMA) ||
                    (commonConfig->contextswitchTriggerMode == HWA_CONTEXTSWITCH_TRIG_MODE_HARDWARE))
                {
                    CSL_FINSR(ctrlBaseAddr->CS_CONFIG,
                              CS_CONFIG_CS_TRGSRC_END,
                              CS_CONFIG_CS_TRGSRC_START,
                              commonConfig->contextswitchTriggerSrc);
                }

            }
            /* set BPM rate BPM pattern*/
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_BPMCFG)
            {
                CSL_FINSR(ctrlBaseAddr->BPM_RATE,
                          BPM_RATE_BPM_RATE_END,
                          BPM_RATE_BPM_RATE_START,
                          commonConfig->fftConfig.bpmRate);

                for (ii = 0; ii < 8; ii++)
                {
                    ctrlBaseAddr->BPM_PATTERN[ii] = commonConfig->fftConfig.bpmPattern[ii];
                 }
            }

            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE)
            {
                /* enable the twiddle dither, load the random pattern seed*/
                 CSL_FINSR(ctrlBaseAddr->DITHER_TWID_EN,
                           DITHER_TWID_EN_DITHER_TWID_EN_END,
                           DITHER_TWID_EN_DITHER_TWID_EN_START,
                           commonConfig->fftConfig.twidDitherEnable);
            }
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_LFSRSEED)
            {
                 CSL_FINSR(ctrlBaseAddr->LFSR_SEED,
                           LFSR_SEED_LFSR_SEED_END,
                           LFSR_SEED_LFSR_SEED_START,
                           commonConfig->fftConfig.lfsrSeed);
                /* Pulse (set and reset)single register-bit REG_LFSRLOAD */
                 CSL_FINSR(ctrlBaseAddr->LFSR_LOAD,
                           LFSR_LOAD_LFSR_LOAD_END,
                           LFSR_LOAD_LFSR_LOAD_START, 1U);
                 CSL_FINSR(ctrlBaseAddr->LFSR_LOAD,
                           LFSR_LOAD_LFSR_LOAD_END,
                           LFSR_LOAD_LFSR_LOAD_START, 0U);
            }
            /* set fft sum div */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_FFTSUMDIV)
            {
                CSL_FINSR(ctrlBaseAddr->FFTSUMDIV,
                          FFTSUMDIV_FFTSUMDIV_END,
                          FFTSUMDIV_FFTSUMDIV_START,
                          commonConfig->fftConfig.fftSumDiv);
            }
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFARTHRESHOLDSCALE)
            {
                CSL_FINSR(ctrlBaseAddr->CFAR_THRESH,
                          CFAR_THRESH_CFAR_THRESH_END,
                          CFAR_THRESH_CFAR_THRESH_START,
                          commonConfig->cfarConfig.thresholdScale);
            }

            /* CFAR_DET_THR Configuration. */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_CFAR_DET_THR)
            {
                CSL_FINSR(ctrlBaseAddr->CFAR_THRESH,
                          CFAR_DET_THR_CFAR_DET_THR_END,
                          CFAR_DET_THR_CFAR_DET_THR_START,
                          commonConfig->cfarDetThresConfig.cfarDetthreshold);
            }

            /* set DCEST scale and shift */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT)
            {
                CSL_FINSR(ctrlBaseAddr->DC_EST_CTRL,
                          DC_EST_CTRL_DC_EST_SCALE_END,
                          DC_EST_CTRL_DC_EST_SCALE_START,
                          commonConfig->dcEstimateConfig.scale);

                CSL_FINSR(ctrlBaseAddr->DC_EST_CTRL,
                          DC_EST_CTRL_DC_EST_SHIFT_END,
                          DC_EST_CTRL_DC_EST_SHIFT_START,
                          commonConfig->dcEstimateConfig.shift);

            }
            /* if DC SUB is enabled, and DCSUB_SELECT is set to 0, set the pramgrammed DC values used in subtraction */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_DCSUB_SWVAL)
            {
                for (ii = 0; ii < 12; ii++)
                {
                    CSL_FINSR(ctrlBaseAddr->DC_I_SW[ii],
                              DC_EST_I_0_VAL_DC_EST_I_0_VAL_END,
                              DC_EST_I_0_VAL_DC_EST_I_0_VAL_START,
                              commonConfig->dcSubtractConfig.swIVal[ii]);
                    CSL_FINSR(ctrlBaseAddr->DC_Q_SW[ii],
                              DC_EST_Q_0_VAL_DC_EST_Q_0_VAL_END,
                              DC_EST_Q_0_VAL_DC_EST_Q_0_VAL_START,
                              commonConfig->dcSubtractConfig.swQVal[ii]);
                }
            }
            /* set the interference threshold for magnitude */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFMAG_THRESHOLD)
            {
                for (ii = 0; ii < 12; ii++)
                {
                    CSL_FINSR(ctrlBaseAddr->INTF_LOC_THRESH_MAG_SW[ii],
                              INTF_LOC_THRESH_MAG0_SW_INTF_LOC_THRESH_MAG0_SW_END,
                              INTF_LOC_THRESH_MAG0_SW_INTF_LOC_THRESH_MAG0_SW_START,
                              commonConfig->interfConfig.thresholdMagSw[ii]);
                }

            }
            /* set the interference threshold for matnitude diff */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFMAGDIFF_THRESHOLD)
            {
                for (ii = 0; ii < 12; ii++)
                {
                    CSL_FINSR(ctrlBaseAddr->INTF_LOC_THRESH_MAGDIFF_SW[ii],
                              INTF_LOC_THRESH_MAGDIFF0_SW_INTF_LOC_THRESH_MAGDIFF0_SW_END,
                              INTF_LOC_THRESH_MAGDIFF0_SW_INTF_LOC_THRESH_MAGDIFF0_SW_START,
                              commonConfig->interfConfig.thresholdMagDiffSw[ii]);
                }

            }

            /* set the interference sum magn unsigned scaler */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAG)
            {
                /* magSum scale */
                CSL_FINSR(ctrlBaseAddr->INTF_STATS_CTRL,
                          INTF_STATS_CTRL_INTF_STATS_MAG_SCALE_END,
                          INTF_STATS_CTRL_INTF_STATS_MAG_SCALE_START,
                          commonConfig->interfConfig.sumMagScale);
                /* magSum shift*/
                CSL_FINSR(ctrlBaseAddr->INTF_STATS_CTRL,
                          INTF_STATS_CTRL_INTF_STATS_MAG_SHIFT_END,
                          INTF_STATS_CTRL_INTF_STATS_MAG_SHIFT_START,
                          commonConfig->interfConfig.sumMagShift);
            }
            /* set the interference sum magn unsigned scaler */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERFSUM_MAGDIFF)
            {
                /* magDiffSum scale */
                CSL_FINSR(ctrlBaseAddr->INTF_STATS_CTRL,
                          INTF_STATS_CTRL_INTF_STATS_MAGDIFF_SCALE_END,
                          INTF_STATS_CTRL_INTF_STATS_MAGDIFF_SCALE_START,
                          commonConfig->interfConfig.sumMagDiffScale);
                /* magDiffSum shift*/
                CSL_FINSR(ctrlBaseAddr->INTF_STATS_CTRL,
                          INTF_STATS_CTRL_INTF_STATS_MAGDIFF_SHIFT_END,
                          INTF_STATS_CTRL_INTF_STATS_MAGDIFF_SHIFT_START,
                          commonConfig->interfConfig.sumMagDiffShift);
            }
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_COMPLEXMULT_SCALECONST)
            {
                /* set the ICMULT_SCALE0, and QCMULT_SCALE0*/
                CSL_FINSR(ctrlBaseAddr->ICMULT_SCALE[0],
                    ICMULT_SCALE0_ICMULT_SCALE0_END,
                    ICMULT_SCALE0_ICMULT_SCALE0_START,
                    commonConfig->complexMultiplyConfig.Iscale[0]);

                CSL_FINSR(ctrlBaseAddr->QCMULT_SCALE[0],
                    QCMULT_SCALE0_QCMULT_SCALE0_END,
                    QCMULT_SCALE0_QCMULT_SCALE0_START,
                    commonConfig->complexMultiplyConfig.Qscale[0]);
                /* set the rest 11 scale registers to 0*/
                for (ii = 1; ii < 12; ii++)
                {
                    ctrlBaseAddr->ICMULT_SCALE[ii] = 0;
                    ctrlBaseAddr->QCMULT_SCALE[ii] = 0;
                }
            }

            /* set the cmult scale In, and Qn*/
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_COMPLEXMULT_SCALEARRAY)
            {
                for (ii = 0; ii < 12; ii++)
                {
                    CSL_FINSR(ctrlBaseAddr->ICMULT_SCALE[ii],
                              ICMULT_SCALE0_ICMULT_SCALE0_END,
                              ICMULT_SCALE0_ICMULT_SCALE0_START,
                              commonConfig->complexMultiplyConfig.Iscale[ii]);

                    CSL_FINSR(ctrlBaseAddr->QCMULT_SCALE[ii],
                              QCMULT_SCALE0_QCMULT_SCALE0_END,
                              QCMULT_SCALE0_QCMULT_SCALE0_START,
                              commonConfig->complexMultiplyConfig.Qscale[ii]);

                }
            }

           /* set the cmult twidincr register based on the cmult_mode*/
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_TWIDINCR_DELTA_FRAC)
            {
                /*TWIDINCR_DELTA_FRAC can be added to twidincr to compensate, */
                CSL_FINSR(ctrlBaseAddr->TWID_INCR_DELTA_FRAC,
                          TWID_INCR_DELTA_FRAC_TWID_INCR_DELTA_FRAC_END,
                          TWID_INCR_DELTA_FRAC_TWID_INCR_DELTA_FRAC_START,
                          commonConfig->complexMultiplyConfig.twiddleDeltaFrac);

            } /* end of HWA_COMMONCONFIG_MASK_TWIDINCRDELTAFRAC */

            /* set the rec window reset */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_RECWIN_RESET)
            {
                /*set recwindown reset for recursive windowing */
                CSL_FINSR(ctrlBaseAddr->RECWIN_RESET_SW,
                          RECWIN_RESET_SW_RECWIN_RESET_SW_END,
                          RECWIN_RESET_SW_RECWIN_RESET_SW_START,
                          commonConfig->complexMultiplyConfig.recWindowReset);

            } /* end of HWA_COMMONCONFIG_MASK_RECWIN_RESET */


            /* set the chan comb vec and size*/
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_CHANCOMB_VEC_SIZE)
            {
                /* 256 bits, */
                for (ii = 0; ii < 8; ii++)
                {
                    ctrlBaseAddr->CHAN_COMB_VEC[ii] = commonConfig->chanCombConfig.vector[ii];
                }

                CSL_FINSR(ctrlBaseAddr->CHAN_COMB_SIZE,
                          CHAN_COMB_SIZE_CHAN_COMB_SIZE_END,
                          CHAN_COMB_SIZE_CHAN_COMB_SIZE_START,
                          commonConfig->chanCombConfig.size);
            }

            /* set the zero insert number and mask */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_ZEROINSERT_NUM_MASK)
            {
#if defined (SOC_AWR294X)
                if(ptrHWADriver->isES2P0Device == true)
                {
                    /* Zero insertion for For AWR294x ES2.0 device*/
                    CSL_FINSR(ctrlBaseAddr->ZERO_INSERT_NUM,
                            10,
                            0,
                            commonConfig->zeroInsertConfig.number);
                }
                else
                {
                    CSL_FINSR(ctrlBaseAddr->ZERO_INSERT_NUM,
                            ZERO_INSERT_NUM_ZERO_INSERT_NUM_END,
                            ZERO_INSERT_NUM_ZERO_INSERT_NUM_START,
                            commonConfig->zeroInsertConfig.number);

                    /* Zero insertion for For AWR294x ES1.0 device and AM273x devices. */
                    for (ii = 0; ii < 8; ii++)
                    {
                        ctrlBaseAddr->ZERO_INSERT_MASK[ii] = commonConfig->zeroInsertConfig.mask[ii];
                    }
                }
#endif

#if defined (SOC_AM273X)
                CSL_FINSR(ctrlBaseAddr->ZERO_INSERT_NUM,
                          ZERO_INSERT_NUM_ZERO_INSERT_NUM_END,
                          ZERO_INSERT_NUM_ZERO_INSERT_NUM_START,
                          commonConfig->zeroInsertConfig.number);
                for (ii = 0; ii < 8; ii++)
                {
                    ctrlBaseAddr->ZERO_INSERT_MASK[ii] = commonConfig->zeroInsertConfig.mask[ii];
                }
#endif
            }

            /* set the advanced statistics config for 2D max, both dim offset */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_MAX2D_OFFSETBOTHDIM)
            {
                CSL_FINSR(ctrlBaseAddr->MAX2D_OFFSET_DIM1,
                          MAX2D_OFFSET_DIM1_MAX2D_OFFSET_DIM1_END,
                          MAX2D_OFFSET_DIM1_MAX2D_OFFSET_DIM1_START,
                          commonConfig->advStatConfig.max2DoffsetDim1);

                CSL_FINSR(ctrlBaseAddr->MAX2D_OFFSET_DIM2,
                          MAX2D_OFFSET_DIM2_MAX2D_OFFSET_DIM2_END,
                          MAX2D_OFFSET_DIM2_MAX2D_OFFSET_DIM2_START,
                          commonConfig->advStatConfig.max2DoffsetDim2);
            }
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_CDFCNT_THRESHOLD)
            {
                CSL_FINSR(ctrlBaseAddr->CDF_CNT_THRESH,
                          CDF_CNT_THRESH_CDF_CNT_THRESH_END,
                          CDF_CNT_THRESH_CDF_CNT_THRESH_START,
                          commonConfig->advStatConfig.cdfCntThresh);
            }


            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMB_THRESHOLDSW)
            {
                CSL_FINSR(ctrlBaseAddr->LM_THRESH_VAL,
                          LM_THRESH_VAL_DIMB_THRESH_VAL_END,
                          LM_THRESH_VAL_DIMB_THRESH_VAL_START,
                          commonConfig->localMaxConfig.dimBThreshold);
            }

            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMBTHRESH_OFFSET)
            {
                CSL_FINSR(ctrlBaseAddr->LM_2DSTATS_BASE_ADDR,
                          LM_2DSTATS_BASE_ADDR_BASE_ADDR_DIMB_END,
                          LM_2DSTATS_BASE_ADDR_BASE_ADDR_DIMB_START,
                          commonConfig->localMaxConfig.dimBBaseAddress);

            }
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMC_THRESHOLDSW)
            {
                CSL_FINSR(ctrlBaseAddr->LM_THRESH_VAL,
                          LM_THRESH_VAL_DIMC_THRESH_VAL_END,
                          LM_THRESH_VAL_DIMC_THRESH_VAL_START,
                          commonConfig->localMaxConfig.dimCThreshold);
            }
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_LOCALMAXDIMCTHRESH_OFFSET)
            {
                CSL_FINSR(ctrlBaseAddr->LM_2DSTATS_BASE_ADDR,
                          LM_2DSTATS_BASE_ADDR_BASE_ADDR_DIMC_END,
                          LM_2DSTATS_BASE_ADDR_BASE_ADDR_DIMC_START,
                          commonConfig->localMaxConfig.dimCBaseAddress);

            }

            /* add interference mitigation window param */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM)
            {
                for (ii = 0; ii < 5; ii++)
                {
                    CSL_FINSR(ctrlBaseAddr->INTF_MITG_WINDOW_PARAM[ii],
                              INTF_MITG_WINDOW_PARAM_0_INTF_MITG_WINDOW_PARAM_0_END,
                              INTF_MITG_WINDOW_PARAM_0_INTF_MITG_WINDOW_PARAM_0_START,
                              commonConfig->interfConfig.mitigationWindowParam[ii]);
                }
            }
            /* add compression EGE algorithm k values */
            if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM)
            {

                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,
                          CMP_EGE_K0123_CMP_EGE_K0_END,
                          CMP_EGE_K0123_CMP_EGE_K0_START,
                          commonConfig->compressConfig.EGEKparam[0]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,
                          CMP_EGE_K0123_CMP_EGE_K1_END,
                          CMP_EGE_K0123_CMP_EGE_K1_START,
                          commonConfig->compressConfig.EGEKparam[1]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,
                          CMP_EGE_K0123_CMP_EGE_K2_END,
                          CMP_EGE_K0123_CMP_EGE_K2_START,
                          commonConfig->compressConfig.EGEKparam[2]);

                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K0123,
                          CMP_EGE_K0123_CMP_EGE_K3_END,
                          CMP_EGE_K0123_CMP_EGE_K3_START,
                          commonConfig->compressConfig.EGEKparam[3]);

                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567,
                          CMP_EGE_K4567_CMP_EGE_K4_END,
                          CMP_EGE_K4567_CMP_EGE_K4_START,
                          commonConfig->compressConfig.EGEKparam[4]);

                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567,
                          CMP_EGE_K4567_CMP_EGE_K5_END,
                          CMP_EGE_K4567_CMP_EGE_K5_START,
                          commonConfig->compressConfig.EGEKparam[5]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567,
                          CMP_EGE_K4567_CMP_EGE_K6_END,
                          CMP_EGE_K4567_CMP_EGE_K6_START,
                          commonConfig->compressConfig.EGEKparam[6]);
                CSL_FINSR(ctrlBaseAddr->CMP_EGE_K4567,
                          CMP_EGE_K4567_CMP_EGE_K7_END,
                          CMP_EGE_K4567_CMP_EGE_K7_START,
                          commonConfig->compressConfig.EGEKparam[7]);

            }

#if defined (SOC_AWR294X)
            /* Below register configuration is specific to AWR294x PG2.0 devices. */
            if(ptrHWADriver->isES2P0Device == true)
            {
                if(commonConfig->configMask & HWA_COMMONCONFIG_MASK_SW_RESTART_LOOP)
                {
                    CSL_FINSR(ctrlBaseAddr->SW_RESTART_LOOP,
                            HWA_SW_RESTART_LOOP_SW_RESTART_LOOP_END,
                            HWA_SW_RESTART_LOOP_SW_RESTART_LOOP_START,
                            commonConfig->swRestartLoop);
                }
                if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_CMP_LFSRSEED0)
                {
                    CSL_FINSR(ctrlBaseAddr->REG_CMP_LFSRSEED_0,
                            REG_CMP_LFSRSEED_0_REG_CMP_LFSRSEED_0_END,
                            REG_CMP_LFSRSEED_0_REG_CMP_LFSRSEED_0_START,
                            commonConfig->compressConfig.cmpLfsrSeed0);
                    /* Pulse (set and reset)single register-bit REG_CMP_LFSRLOAD0 */
                    CSL_FINSR(ctrlBaseAddr->REG_CMP_LFSRLOAD_0,
                            REG_CMP_LFSRLOAD_0_REG_CMP_LFSRLOAD_0_END,
                            REG_CMP_LFSRLOAD_0_REG_CMP_LFSRLOAD_0_START, 1U);
                    CSL_FINSR(ctrlBaseAddr->REG_CMP_LFSRLOAD_0,
                            REG_CMP_LFSRLOAD_0_REG_CMP_LFSRLOAD_0_END,
                            REG_CMP_LFSRLOAD_0_REG_CMP_LFSRLOAD_0_START, 0U);
                }
                if (commonConfig->configMask & HWA_COMMONCONFIG_MASK_CMP_LFSRSEED1)
                {
                    CSL_FINSR(ctrlBaseAddr->REG_CMP_LFSRSEED_1,
                            REG_CMP_LFSRSEED_1_REG_CMP_LFSRSEED_1_END,
                            REG_CMP_LFSRSEED_1_REG_CMP_LFSRSEED_1_START,
                            commonConfig->compressConfig.cmpLfsrSeed1);
                    /* Pulse (set and reset)single register-bit REG_CMP_LFSRLOAD1 */
                    CSL_FINSR(ctrlBaseAddr->REG_CMP_LFSRLOAD_1,
                            REG_CMP_LFSRLOAD_1_REG_CMP_LFSRLOAD_1_END,
                            REG_CMP_LFSRLOAD_1_REG_CMP_LFSRLOAD_1_START, 1U);
                    CSL_FINSR(ctrlBaseAddr->REG_CMP_LFSRLOAD_1,
                            REG_CMP_LFSRLOAD_1_REG_CMP_LFSRLOAD_1_END,
                            REG_CMP_LFSRLOAD_1_REG_CMP_LFSRLOAD_1_START, 0U);
                }
            }
#endif
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}

DSSHWACCRegs *HWA_getCommonCtrlAddr(HWA_Handle handle)
{
    int32_t             retCode = 0;
    DSSHWACCRegs       *ctrlBaseAddr = NULL;
    HWA_Object         *ptrHWADriver;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if(retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)(ptrHWADriver->hwAttrs->ctrlBaseAddr);
    }
    HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);

    return (ctrlBaseAddr);
}

DSSHWACCPARAMRegs *HWA_getParamSetAddr(HWA_Handle handle, uint8_t paramsetIdx)
{
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs  *paramBaseAddr = NULL;
    HWA_Object         *ptrHWADriver;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) true, paramsetIdx);
    if(retCode==0)
    {
        paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr + (paramsetIdx * sizeof(DSSHWACCPARAMRegs)));
    }
    HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) true, paramsetIdx);

    return (paramBaseAddr);
}

/*!
 *  @brief  Function to set the HWA configuration parameters for a given paramSet
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  paramsetIdx     A valid paramSet index for which the paramConfig is provided.
 *
 *  @param  paramConfig     HWA ParamSet Config Parameters
 *
 *  @param  dmaConfig       [out] This parameter is set by the driver with values that user
 *                                should use to program the source trigger DMA. user should provide
 *                                a valid buffer here if the triggerMode is set to DMA in paramConfig
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_configParamSet(HWA_Handle handle, uint8_t paramsetIdx, HWA_ParamConfig *paramConfig, HWA_SrcDMAConfig *dmaConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;
    DSSHWACCRegs *ctrlBaseAddr;
    DSSHWACCPARAMRegs paramReg={0};
    uint32_t           *dst;
    uint32_t           *src;
    uint32_t           ii;


    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) false,(bool) true,paramsetIdx);
    if (retCode==0)
    {
        /* validate the paramConfig */
        retCode = HWA_validateParamSetConfig(ptrHWADriver,paramConfig);
        if (retCode==0)
        {
            /* valid driver access and valid paramConfig */
            paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr+paramsetIdx*sizeof(DSSHWACCPARAMRegs));

            /* TODO check Get paramset interrupt settings which will be preserved, DMATRIGEN(1) and ACC2DMA_TRIGDST(5) */
            paramReg.HEADER = CSL_FEXTR(paramBaseAddr->HEADER, HEADER_HWA2DMA_TRIGDST_END, HEADER_DMATRIG_EN_START) << HEADER_DMATRIG_EN_START;
            /* Get paramset interrupt settings which will be preserved, CPU_INTR1_EN and CPU_INTR2_EN */
            paramReg.HEADER |= (CSL_FEXTR(paramBaseAddr->HEADER,HEADER_CPU_INTR1_EN_END,HEADER_CPU_INTR2_EN_START) << HEADER_CPU_INTR2_EN_START);

            /* All register start with value 0 except HEADER */
            /* trigger mode*/
            paramReg.HEADER |= CSL_FMKR(HEADER_TRIGMODE_END, HEADER_TRIGMODE_START,paramConfig->triggerMode);
            if (paramConfig->triggerMode == HWA_TRIG_MODE_DMA)
            {
                paramReg.HEADER |= CSL_FMKR(HEADER_HWA_TRIGSRC_END, HEADER_HWA_TRIGSRC_START,paramConfig->triggerSrc);
                /* provide the user with the DMA programming config to facilitate the DMA triggering of HWA */
                /* copy the triger signature to  DMA2HWA_TRIG register to trigger HWA*/
                if (dmaConfig != NULL)
                {
                    ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

                    dmaConfig->destAddr = (uint32_t)(&ctrlBaseAddr->DMA2HWA_TRIG);
                    dmaConfig->srcAddr = (uint32_t)(&ctrlBaseAddr->SIGDMACHDONE[paramConfig->triggerSrc]);
                    dmaConfig->aCnt = sizeof(uint32_t);
                    dmaConfig->bCnt = 1U;
                    dmaConfig->cCnt = 1U;
                }
            }
            if (paramConfig->triggerMode == HWA_TRIG_MODE_HARDWARE)
            {
                paramReg.HEADER |= CSL_FMKR(HEADER_HWA_TRIGSRC_END, HEADER_HWA_TRIGSRC_START,paramConfig->triggerSrc);
            }


            paramReg.HEADER      |= CSL_FMKR(HEADER_ACCEL_MODE_END, HEADER_ACCEL_MODE_START,paramConfig->accelMode);
            if (paramConfig->contextswitchCfg == HWA_PARAMSET_CONTEXTSWITCH_DISABLE)
            {
                paramReg.HEADER |= CSL_FMKR(HEADER_CONTEXTSW_EN_END, HEADER_CONTEXTSW_EN_START, 0U);
                paramReg.HEADER |= CSL_FMKR(HEADER_FORCED_CONTEXTSW_EN_END, HEADER_FORCED_CONTEXTSW_EN_START,0U);
            }
            else if (paramConfig->contextswitchCfg == HWA_PARAMSET_CONTEXTSWITCH_NONFORCE_ENABLE)
            {
                paramReg.HEADER |= CSL_FMKR(HEADER_CONTEXTSW_EN_END, HEADER_CONTEXTSW_EN_START, 1U);
                paramReg.HEADER |= CSL_FMKR(HEADER_FORCED_CONTEXTSW_EN_END, HEADER_FORCED_CONTEXTSW_EN_START, 0U);
            }
            else
            {
                paramReg.HEADER |= CSL_FMKR(HEADER_CONTEXTSW_EN_END, HEADER_CONTEXTSW_EN_START, 0U);
                paramReg.HEADER |= CSL_FMKR(HEADER_FORCED_CONTEXTSW_EN_END, HEADER_FORCED_CONTEXTSW_EN_START, 1U);
            }
            paramReg.SRC         |= CSL_FMKR(SRC_SRCADDR_END, SRC_SRCADDR_START,paramConfig->source.srcAddr);
            paramReg.SRCA        |= CSL_FMKR(SRCA_SRCACNT_END, SRCA_SRCACNT_START,paramConfig->source.srcAcnt);
            paramReg.SRCA        |= CSL_FMKR(SRCA_SRCAINDX_END, SRCA_SRCAINDX_START,paramConfig->source.srcAIdx);
            paramReg.SRCB        |= CSL_FMKR(SRCB_BCNT_END, SRCB_BCNT_START,paramConfig->source.srcBcnt);
            paramReg.SRCB        |= CSL_FMKR(SRCB_SRCBINDX_END, SRCB_SRCBINDX_START,paramConfig->source.srcBIdx);
            paramReg.CIRCSHIFT   |= CSL_FMKR(CIRCSHIFT_SRCA_CIRCSHIFT_END, CIRCSHIFT_SRCA_CIRCSHIFT_START, paramConfig->source.srcAcircShift);
            paramReg.CIRCSHIFT2  |= CSL_FMKR(CIRCSHIFT2_SRCA_CIRCSHIFTWRAP_END,
                                             CIRCSHIFT2_SRCA_CIRCSHIFTWRAP_START,
                                             paramConfig->source.srcAcircShiftWrap);
            paramReg.SRC         |= CSL_FMKR(SRC_SRCREAL_END, SRC_SRCREAL_START,paramConfig->source.srcRealComplex);
            paramReg.SRC         |= CSL_FMKR(SRC_SRC16b32b_END, SRC_SRC16b32b_START,paramConfig->source.srcWidth);
            paramReg.SRC         |= CSL_FMKR(SRC_SRCSIGNED_END, SRC_SRCSIGNED_START,paramConfig->source.srcSign);
            paramReg.SRC         |= CSL_FMKR(SRC_SRCCONJ_END, SRC_SRCCONJ_START,paramConfig->source.srcConjugate);
            paramReg.SRC         |= CSL_FMKR(SRC_SRCSCAL_END, SRC_SRCSCAL_START,paramConfig->source.srcScale);

            paramReg.CIRCSHIFT   |= CSL_FMKR(CIRCSHIFT_SRCB_CIRCSHIFT_END, CIRCSHIFT_SRCB_CIRCSHIFT_START, paramConfig->source.srcBcircShift);
            paramReg.CIRCSHIFT2  |= CSL_FMKR(CIRCSHIFT2_SRCB_CIRCSHIFTWRAP_END,
                                             CIRCSHIFT2_SRCB_CIRCSHIFTWRAP_START,
                                             paramConfig->source.srcBcircShiftWrap);

            if (paramConfig->accelMode == HWA_ACCELMODE_LOCALMAX)
            {
                paramReg.SRCC    |= CSL_FMKR(SRCC_CCNT_END, SRCC_CCNT_START, paramConfig->source.srcCcnt);
                paramReg.SRCC    |= CSL_FMKR(SRCC_SRCCINDX_END, SRCC_SRCCINDX_START, paramConfig->source.srcCIdx);
            }
            paramReg.CIRCSHIFT2   |= CSL_FMKR(CIRCSHIFT2_SRC_CIRCSHIFTWRAP3X_END,
                                              CIRCSHIFT2_SRC_CIRCSHIFTWRAP3X_START,
                                              paramConfig->source.srcCircShiftWrap3);
            paramReg.SRC          |= CSL_FMKR(SRC_SRCIQSWAP_END, SRC_SRCIQSWAP_START, paramConfig->source.srcIQSwap);
            paramReg.SRC          |= CSL_FMKR(SRC_SHUFFLE_AB_END, SRC_SHUFFLE_AB_START, paramConfig->source.shuffleMode);

            paramReg.DST  |= CSL_FMKR(DST_DSTIQSWAP_END, DST_DSTIQSWAP_START, paramConfig->dest.dstIQswap);
            paramReg.DST  |= CSL_FMKR(DST_DSTADDR_END, DST_DSTADDR_START,paramConfig->dest.dstAddr);
            paramReg.DSTA |= CSL_FMKR(DSTA_DSTACNT_END, DSTA_DSTACNT_START,paramConfig->dest.dstAcnt);
            paramReg.DSTA |= CSL_FMKR(DSTA_DSTAINDX_END, DSTA_DSTAINDX_START,paramConfig->dest.dstAIdx);
            paramReg.DSTB |= CSL_FMKR(DSTB_DSTBINDX_END, DSTB_DSTBINDX_START,paramConfig->dest.dstBIdx);
            paramReg.DST  |= CSL_FMKR(DST_DSTREAL_END, DST_DSTREAL_START,paramConfig->dest.dstRealComplex);
            paramReg.DST  |= CSL_FMKR(DST_DST16b32b_END, DST_DST16b32b_START,paramConfig->dest.dstWidth);
            paramReg.DST  |= CSL_FMKR(DST_DSTSIGNED_END, DST_DSTSIGNED_START,paramConfig->dest.dstSign);
            paramReg.DST  |= CSL_FMKR(DST_DSTCONJ_END, DST_DSTCONJ_START,paramConfig->dest.dstConjugate);
            paramReg.DST  |= CSL_FMKR(DST_DSTSCAL_END, DST_DSTSCAL_START,paramConfig->dest.dstScale);
            paramReg.DSTB |= CSL_FMKR(DSTB_DST_SKIP_INIT_END, DSTB_DST_SKIP_INIT_START,paramConfig->dest.dstSkipInit);


            if (paramConfig->accelMode==HWA_ACCELMODE_FFT)
            {
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_FFT_EN_END,
                                                                        BFLYFFT_FFT_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.fftEn);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_FFTSIZE_END,
                                                                        BFLYFFT_FFTSIZE_START,
                                                                        paramConfig->accelModeArgs.fftMode.fftSize);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_BFLY_SCALING_END,
                                                                        BFLYFFT_BFLY_SCALING_START,
                                                                        paramConfig->accelModeArgs.fftMode.butterflyScaling);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_WINDOW_EN_END,
                                                                        POSTPROCWIN_WINDOW_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.windowEn);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_WINDOW_START_END,
                                                                        POSTPROCWIN_WINDOW_START_START,
                                                                        paramConfig->accelModeArgs.fftMode.windowStart);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_WINSYMM_END,
                                                                        POSTPROCWIN_WINSYMM_START,
                                                                        paramConfig->accelModeArgs.fftMode.winSymm);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_WINDOW_INTERP_FRACTION_END,
                                                                        POSTPROCWIN_WINDOW_INTERP_FRACTION_START,
                                                       paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.winInterpolateMode);

                /*absEn(bit17), log2en(bit16)*/
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_ABS_EN_END,
                                                                        POSTPROCWIN_LOG2_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.postProcCfg.magLogEn);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_FFT_OUTPUT_MODE_END,
                                                                        POSTPROCWIN_FFT_OUTPUT_MODE_START,
                                                                        paramConfig->accelModeArgs.fftMode.postProcCfg.fftOutMode);

                if (paramConfig->accelModeArgs.fftMode.postProcCfg.fftOutMode != HWA_FFT_MODE_OUTPUT_DEFAULT)
                {
                    /* DST ACNT*/
                    CSL_FINSR(paramReg.DSTA, DSTA_DSTACNT_END, DSTA_DSTACNT_START, 4095U);
                    /* DST AINDX */
                    CSL_FINSR(paramReg.DSTA, DSTA_DSTAINDX_END, DSTA_DSTAINDX_START, 8U);
                    /* DST BINDX */
                    CSL_FINSR(paramReg.DSTB, DSTB_DSTBINDX_END, DSTB_DSTBINDX_START, 8U);
                     /* DST REAL */
                    CSL_FINSR(paramReg.DST, DST_DSTREAL_END, DST_DSTREAL_START, 0U);
                    /* DST 16b32b */
                    CSL_FINSR(paramReg.DST, DST_DST16b32b_END, DST_DST16b32b_START, 1U);
                 }

                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_WINDOW_MODE_END,
                                                                        POSTPROCWIN_WINDOW_MODE_START,
                                                                        paramConfig->accelModeArgs.fftMode.windowMode);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_FFTSIZE3X_EN_END,
                                                                        BFLYFFT_FFTSIZE3X_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.fftSize3xEn);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_FFTSIZE_DIM2_END,
                                                                        BFLYFFT_FFTSIZE_DIM2_START,
                                                                        paramConfig->accelModeArgs.fftMode.fftSizeDim2);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_BFLY_SCALING_FFT3X_END,
                                                                        BFLYFFT_BFLY_SCALING_FFT3X_START,
                                                                        paramConfig->accelModeArgs.fftMode.butterflyScalingFFT3x);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_BPM_EN_END,
                                                                        BFLYFFT_BPM_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.bpmEnable);
                paramReg.accelModeParam.FFTPATH.BFLYFFT     |= CSL_FMKR(BFLYFFT_BPM_PHASE_END,
                                                                        BFLYFFT_BPM_PHASE_START,
                                                                        paramConfig->accelModeArgs.fftMode.bpmPhase);
                paramReg.accelModeParam.FFTPATH.BFLYFFT |= CSL_FMKR(BFLYFFT_ZEROINSERT_EN_END,
                                                                    BFLYFFT_ZEROINSERT_EN_START,
                                                                    paramConfig->accelModeArgs.fftMode.preProcCfg.zeroInsertEn);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_MAX2D_EN_END,
                                                                        POSTPROCWIN_MAX2D_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.postProcCfg.max2Denable);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_HIST_MODE_END,
                                                                         PREPROC1_HIST_MODE_START,
                                                                         paramConfig->accelModeArgs.fftMode.postProcCfg.histogramMode);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_HIST_SCALE_SEL_END,
                                                                        POSTPROCWIN_HIST_SCALE_SEL_START,
                                                                        paramConfig->accelModeArgs.fftMode.postProcCfg.histogramScaleSelect);
                paramReg.accelModeParam.FFTPATH.POSTPROCWIN |= CSL_FMKR(POSTPROCWIN_HIST_SIZE_SEL_END,
                                                                        POSTPROCWIN_HIST_SIZE_SEL_START,
                                                                        paramConfig->accelModeArgs.fftMode.postProcCfg.histogramSizeSelect);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_DCEST_RESET_MODE_END,
                                                                         PREPROC1_DCEST_RESET_MODE_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.dcEstResetMode);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_DCSUB_EN_END,
                                                                         PREPROC1_DCSUB_EN_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.dcSubEnable);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_DCSUB_SEL_END,
                                                                         PREPROC1_DCSUB_SEL_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.dcSubSelect);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_LOC_THRESH_EN_END,
                                                                         PREPROC1_INTF_LOC_THRESH_EN_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdEnable);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_LOC_THRESH_MODE_END,
                                                                         PREPROC1_INTF_LOC_THRESH_MODE_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdMode);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_LOC_THRESH_SEL_END,
                                                                         PREPROC1_INTF_LOC_THRESH_SEL_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfLocalize.thresholdSelect);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_STATS_RESET_MODE_END,
                                                                         PREPROC1_INTF_STATS_RESET_MODE_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfStat.resetMode);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_CHANCOMB_EN_END,
                                                                         PREPROC1_CHANCOMB_EN_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.chanCombEn);

                paramReg.accelModeParam.FFTPATH.WRAPCOMB     |= CSL_FMKR(WRAPCOMB_WRAP_COMB_END, WRAPCOMB_WRAP_COMB_START, paramConfig->source.wrapComb);
                paramReg.accelModeParam.FFTPATH.WRAPCOMB     |= CSL_FMKR(WRAPCOMB_SHUFFLE_INDX_START_OFFSET_END,
                                                                         WRAPCOMB_SHUFFLE_INDX_START_OFFSET_START,
                                                                         paramConfig->source.shuffleStart);

                /* add interference mitigation module configuration */
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_MITG_EN_END,
                                                                         PREPROC1_INTF_MITG_EN_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.enable);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_MITG_CNT_THRESH_END,
                                                                         PREPROC1_INTF_MITG_CNT_THRESH_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.countThreshold);
                paramReg.accelModeParam.FFTPATH.PREPROC2     |= CSL_FMKR(PREPROC2_INTF_MITG_LEFT_HYST_ORD_END,
                                                                         PREPROC2_INTF_MITG_LEFT_HYST_ORD_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.leftHystOrder);
                paramReg.accelModeParam.FFTPATH.PREPROC2     |= CSL_FMKR(PREPROC2_INTF_MITG_RIGHT_HYST_ORD_END,
                                                                         PREPROC2_INTF_MITG_RIGHT_HYST_ORD_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.rightHystOrder);
                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_INTF_MITG_PATH_SEL_END,
                                                                         PREPROC1_INTF_MITG_PATH_SEL_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.interfMitigation.pathSelect);

                paramReg.accelModeParam.FFTPATH.PREPROC1     |= CSL_FMKR(PREPROC1_CMULT_MODE_END, PREPROC1_CMULT_MODE_START,
                                                                          paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode);

                /* if cmul is frequency shifter mode*/
                if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_FREQ_SHIFTER)
                {
                    /*TWIDINCR specify the de-rotation frequency, how much the phase should change for each successive input */
                    paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(PREPROC2_TWIDINCR_END,
                                                                         PREPROC2_TWIDINCR_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.freqShift.freqShiftTwiddleIncr);
                }

                /* cmul is slow dft mode */
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_SLOW_DFT)
                {
                    paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(PREPROC2_TWIDINCR_END,
                                                                        PREPROC2_TWIDINCR_START,
                                                                        paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.slowDFT.startFreq);
                    /* TWID_INCR_DELTA_FRAC = 0 */
                }
                /* cmul is fft stitching mode */
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_FFT_STITCHING)
                {
                    /* set the twid pattern in last 2 bits, for 4K and 8K, 12 msb set to 0*/
                    if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.twiddlePattern == HWA_FFT_STITCHING_TWID_PATTERN_4K)
                    {
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(1U, 0U, HWA_FFT_STITCHING_TWID_PATTERN_4K);
                    }
                    else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.FFTstitching.twiddlePattern == HWA_FFT_STITCHING_TWID_PATTERN_8K)
                    {
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(1U, 0U, HWA_FFT_STITCHING_TWID_PATTERN_8K);
                    }
                    else
                    {
                        /* invalid config params */
                        retCode = HWA_EINVAL;
                    }
                }
                /* cmul is HWA_COMPLEX_MULTIPLY_MODE_SCALR_MULT */
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_SCALAR_MULT)
                {
                    paramReg.accelModeParam.FFTPATH.PREPROC1 |= CSL_FMKR(PREPROC1_CMULT_SCALE_EN_END,
                                                                        PREPROC1_CMULT_SCALE_EN_START,
                                                                        paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.scalerMultiply.scaleCmultScaleEn);

                }
                /* cmul is HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT,  */
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT)
                {
                    paramReg.accelModeParam.FFTPATH.PREPROC1 |= CSL_FMKR(PREPROC1_CMULT_SCALE_EN_END,
                                                                        PREPROC1_CMULT_SCALE_EN_START,
                                                                     paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.cmultScaleEn);

                    /* 12 msb set as ram address offset */
                    paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(13U, 2U,
                                                                      paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode1.vecMultiMode1RamAddrOffset);


                }
                /* HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT_2 */
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_VECTOR_MULT_2)
                {
                    /* 12 msb set as ram address offset */
                    paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(13U, 2U,
                                                                        paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.vectorMultiplyMode2.vecMultiMode2RamAddrOffset);

                }
                /* HWA_COMPLEX_MULTIPLY_MODE_RECURSIVE_WIN */
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_RECURSIVE_WIN)
                {
                    paramReg.accelModeParam.FFTPATH.PREPROC1 |= CSL_FMKR(PREPROC1_RECWIN_MODE_END,
                                                                         PREPROC1_RECWIN_MODE_START,
                                                                        paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.recursiveWin.recwinModeSel);

                }
                /* cmul is in HWA_COMPLEX_MULTIPLY_MODE_LUT_FREQ_DEROTATE*/
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_LUT_FREQ_DEROTATE)
                {
                    /* 6 lsb is ram offset */
                    paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(6U, 0U, paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.lutFreqDerotate.ramAddrOffset);
                    /* bit 13, 12 is ram index increment mode */
                    if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.lutFreqDerotate.ramIdxIncrMode == HWA_LUT_FREQ_DEROTATE_RAMIDX_AUTOINCR_WRAPAROUND)
                    {
                        /* bit 13 is 1, increment mode */
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(13U, 13U, 1U);
                        /* bit 12 wrap around */
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(12U, 12U, 0U);
                    }
                    else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.lutFreqDerotate.ramIdxIncrMode == HWA_LUT_FREQ_DEROTATE_RAMIDX_AUTOINCR_SATURATED)
                    {
                        /* bit 13 is 1, increment mode */
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(13U, 13U, 1U);
                        /* bit 12 saturate */
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(12U, 12U, 1U);
                    }
                    else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.lutFreqDerotate.ramIdxIncrMode == HWA_LUT_FREQ_DEROTATE_RAMIDX_NONINCR)
                    {
                        /* bit 13 is 0, non-increment mode */
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(13U, 13U, 0U);
                        /* bit 12, does not care */
                        paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(12U, 12U, 0U);
                    }
                }
                else if (paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode == HWA_COMPLEX_MULTIPLY_MODE_FREQSHIFT_FREQINCREMENT)
                {
                    paramReg.accelModeParam.FFTPATH.PREPROC2 |= CSL_FMKR(PREPROC2_TWIDINCR_END,
                                                                         PREPROC2_TWIDINCR_START,
                                                                         paramConfig->accelModeArgs.fftMode.preProcCfg.complexMultiply.modeCfg.freqShiftWithFreIncrement.twiddleIncr);

                    /*TWIDINCR_DELTA_FRAC can be added to twidincr to compensate in static register */

                }
             }

            if (paramConfig->accelMode==HWA_ACCELMODE_CFAR)
            {
                paramReg.accelModeParam.CFARPATH.CFARCFG  |= CSL_FMKR(CFARCFG_CFAR_AVG_LEFT_END, CFARCFG_CFAR_AVG_LEFT_START,paramConfig->accelModeArgs.cfarMode.numNoiseSamplesLeft);
                paramReg.accelModeParam.CFARPATH.CFARCFG  |= CSL_FMKR(CFARCFG_CFAR_AVG_RIGHT_END, CFARCFG_CFAR_AVG_RIGHT_START,paramConfig->accelModeArgs.cfarMode.numNoiseSamplesRight);
                paramReg.accelModeParam.CFARPATH.CFARCFG  |= CSL_FMKR(CFARCFG_CFAR_GUARD_INT_END, CFARCFG_CFAR_GUARD_INT_START,paramConfig->accelModeArgs.cfarMode.numGuardCells);
                paramReg.accelModeParam.CFARPATH.CFARCFG  |= CSL_FMKR(CFARCFG_CFAR_NOISEDIV_END, CFARCFG_CFAR_NOISEDIV_START,paramConfig->accelModeArgs.cfarMode.nAvgDivFactor);
                paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_CA_MODE_END, CFAREN_CFAR_CA_MODE_START,paramConfig->accelModeArgs.cfarMode.nAvgMode);
                switch (paramConfig->accelModeArgs.cfarMode.operMode)
                {
                    case HWA_CFAR_OPER_MODE_LOG_INPUT_REAL:            // CFAR_LOG_MODE=1, CFAR_INP_MODE=1, CFAR_ABS_MODE=dont care
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_LOG_MODE_END, CFAREN_CFAR_LOG_MODE_START, 1U);
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_INP_MODE_END, CFAREN_CFAR_INP_MODE_START, 1U);
                    break;
                    case HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX:          // CFAR_LOG_MODE=1, CFAR_INP_MODE=0, CFAR_ABS_MODE=3
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_LOG_MODE_END, CFAREN_CFAR_LOG_MODE_START, 1U);
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_ABS_MODE_END, CFAREN_CFAR_ABS_MODE_START, 3U);
                    break;
                    case HWA_CFAR_OPER_MODE_MAG_INPUT_REAL:             // CFAR_LOG_MODE=0, CFAR_INP_MODE=1, CFAR_ABS_MODE=dont care
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_INP_MODE_END, CFAREN_CFAR_INP_MODE_START, 1U);
                    break;
                    case HWA_CFAR_OPER_MODE_MAG_INPUT_COMPLEX:         // CFAR_LOG_MODE=0, CFAR_INP_MODE=0, CFAR_ABS_MODE=2
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_ABS_MODE_END, CFAREN_CFAR_ABS_MODE_START, 2U);
                    break;
                    case HWA_CFAR_OPER_MODE_MAG_SQR_INPUT_REAL:         // CFAR_LOG_MODE=0, CFAR_INP_MODE=1, CFAR_ABS_MODE=dont care
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_INP_MODE_END, CFAREN_CFAR_INP_MODE_START, 1U);
                    break;
                    case HWA_CFAR_OPER_MODE_MAG_SQR_INPUT_COMPLEX:              // CFAR_LOG_MODE=0, CFAR_INP_MODE=0, CFAR_ABS_MODE=0
                        break;
                    case HWA_CFAR_OPER_MODE_LOG_INPUT_COMPLEX_LINEARCFAR:        // CFAR_LOG_MODE=0, CFAR_INP_MODE=0, CFAR_ABS_MODE=3
                        paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_ABS_MODE_END, CFAREN_CFAR_ABS_MODE_START, 3U);
                    break;
                    default:
                    break;
                }
                if (!paramConfig->accelModeArgs.cfarMode.cfarAdvOutMode)
                {
                    paramReg.accelModeParam.CFARPATH.CFAREN |= CSL_FMKR(CFAREN_CFAR_OUT_MODE_END, CFAREN_CFAR_OUT_MODE_START, paramConfig->accelModeArgs.cfarMode.outputMode);
                }
                else
                {
                    paramReg.accelModeParam.CFARPATH.CFAREN |= CSL_FMKR(CFAREN_CFAR_ADV_OUT_MODE_END, CFAREN_CFAR_ADV_OUT_MODE_START, (uint32_t)paramConfig->accelModeArgs.cfarMode.cfarAdvOutMode);
                    /* set CFAR_OUT_MODE to 1*/
                    paramReg.accelModeParam.CFARPATH.CFAREN |= CSL_FMKR(CFAREN_CFAR_OUT_MODE_END, CFAREN_CFAR_OUT_MODE_START, 1U);
                 }
                paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_GROUPING_EN_END, CFAREN_CFAR_GROUPING_EN_START,paramConfig->accelModeArgs.cfarMode.peakGroupEn);
                paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_CYCLIC_END, CFAREN_CFAR_CYCLIC_START,paramConfig->accelModeArgs.cfarMode.cyclicModeEn);
                paramReg.accelModeParam.CFARPATH.CFARCFG  |= CSL_FMKR(CFARCFG_CFAR_OS_KVAL_END, CFARCFG_CFAR_OS_KVAL_START, paramConfig->accelModeArgs.cfarMode.cfarOsKvalue);
                paramReg.accelModeParam.CFARPATH.CFAREN   |= CSL_FMKR(CFAREN_CFAR_OS_NON_CYC_VARIANT_EN_END,
                                                                      CFAREN_CFAR_OS_NON_CYC_VARIANT_EN_START,
                                                                      paramConfig->accelModeArgs.cfarMode.cfarOsEdgeKScaleEn);

            }
            if (paramConfig->accelMode==HWA_ACCELMODE_NONE)
            {
               /* nothing needs to be done here */
            }
            if (paramConfig->accelMode == HWA_ACCELMODE_COMPRESS)
            {
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_SCALEFAC_END, CMPDCMP_CMP_SCALEFAC_START, paramConfig->accelModeArgs.compressMode.scaleFactor);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_EGE_OPT_K_INDX_END, CMPDCMP_CMP_EGE_OPT_K_INDX_START, paramConfig->accelModeArgs.compressMode.EGEKidx);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_PASS_SEL_END, CMPDCMP_CMP_PASS_SEL_START, paramConfig->accelModeArgs.compressMode.passSelect);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_HEADER_EN_END, CMPDCMP_CMP_HEADER_EN_START, paramConfig->accelModeArgs.compressMode.headerEnable);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_SCALEFAC_BW_END, CMPDCMP_CMP_SCALEFAC_BW_START, paramConfig->accelModeArgs.compressMode.scaleFactorBW);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_BFP_MANTISSA_BW_END, CMPDCMP_CMP_BFP_MANTISSA_BW_START, paramConfig->accelModeArgs.compressMode.BFPMantissaBW);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_EGE_K_ARR_LEN_END, CMPDCMP_CMP_EGE_K_ARR_LEN_START, paramConfig->accelModeArgs.compressMode.EGEKarrayLength);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_METHOD_END, CMPDCMP_CMP_METHOD_START, paramConfig->accelModeArgs.compressMode.method);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_DCMP_END, CMPDCMP_CMP_DCMP_START, paramConfig->accelModeArgs.compressMode.compressDecompress);
                paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP   |= CSL_FMKR(CMPDCMP_CMP_DITHER_EN_END, CMPDCMP_CMP_DITHER_EN_START, paramConfig->accelModeArgs.compressMode.ditherEnable);
#if defined (SOC_AWR294X)
                /* Compression paramset configuration of AWR294x ES2.0 only.*/
                if(ptrHWADriver->isES2P0Device == true)
                {
                    paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP2   |= CSL_FMKR(CMPDCMP2_REG_CMP_SEL_LFSR_END, CMPDCMP2_REG_CMP_SEL_LFSR_START , paramConfig->accelModeArgs.compressMode.selLfsr);
                    paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP2   |= CSL_FMKR(CMPDCMP2_REG_CMP_ROUND_EN_END, CMPDCMP2_REG_CMP_ROUND_EN_START, paramConfig->accelModeArgs.compressMode.cmpRoundEn);
                    paramReg.accelModeParam.CMPDCMPPATH.CMPDCMP2   |= CSL_FMKR(CMPDCMP2_REG_CMP_BFP_DECR_IMAG_BITW_END, CMPDCMP2_REG_CMP_BFP_DECR_IMAG_BITW_START, paramConfig->accelModeArgs.compressMode.decrImagBitw);
                }
#endif
            }
            if (paramConfig->accelMode == HWA_ACCELMODE_LOCALMAX)
            {
                paramReg.accelModeParam.LOCALMAXPATH.LOCALMAX |= CSL_FMKR(LOCALMAX_LM_THRESH_MODE_END, LOCALMAX_LM_THRESH_MODE_START, paramConfig->accelModeArgs.localMaxMode.thresholdMode);
                paramReg.accelModeParam.LOCALMAXPATH.LOCALMAX |= CSL_FMKR(LOCALMAX_LM_THRESH_BITMASK_END, LOCALMAX_LM_THRESH_BITMASK_START, paramConfig->accelModeArgs.localMaxMode.thresholdBitMask);
                paramReg.accelModeParam.LOCALMAXPATH.LOCALMAX |= CSL_FMKR(LOCALMAX_LM_NEIGH_BITMASK_END, LOCALMAX_LM_NEIGH_BITMASK_START, paramConfig->accelModeArgs.localMaxMode.neighbourBitmask);
                paramReg.accelModeParam.LOCALMAXPATH.LOCALMAX |= CSL_FMKR(LOCALMAX_LM_DIMB_NONCYCLIC_END, LOCALMAX_LM_DIMB_NONCYCLIC_START, paramConfig->accelModeArgs.localMaxMode.dimBNonCyclic);
                paramReg.accelModeParam.LOCALMAXPATH.LOCALMAX |= CSL_FMKR(LOCALMAX_LM_DIMC_NONCYCLIC_END, LOCALMAX_LM_DIMC_NONCYCLIC_START, paramConfig->accelModeArgs.localMaxMode.dimCNonCyclic);
                paramReg.accelModeParam.LOCALMAXPATH.WRAPCOMB |= CSL_FMKR(WRAPCOMB_WRAP_COMB_END, WRAPCOMB_WRAP_COMB_START, paramConfig->source.wrapComb);
                paramReg.accelModeParam.LOCALMAXPATH.WRAPCOMB |= CSL_FMKR(WRAPCOMB_SHUFFLE_INDX_START_OFFSET_END, WRAPCOMB_SHUFFLE_INDX_START_OFFSET_START, paramConfig->source.shuffleStart);
           }
            /* All register is constructed, write the value to hardware */
            dst = (uint32_t *)paramBaseAddr;
            src = (uint32_t *)&paramReg;
            for (ii = 0; ii < 16; ii++)
            {
                dst[ii] = src[ii];
            }
        }

        HWA_releaseDriverAccess(ptrHWADriver,(bool) false,(bool) true,paramsetIdx);
    }

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to get the config to program the DMA for a given DMA Trigger channel.
 *          Application should use the returned config to program the DMA so that it can then
 *          in turn trigger the paramset. Application should make sure that the channel provided
 *          here in dmaTriggerSrc should match the \ref HWA_ParamConfig_t::dmaTriggerSrc passed
 *          to HWA_configParamSet()
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  dmaTriggerSrc   Same as \ref HWA_ParamConfig_t::dmaTriggerSrc of the paramset for
 *                          whom this DMA is getting configured
 *
 *  @param  dmaConfig       [out]This parameter is set by the driver with values that user
 *                               should use to program the source trigger DMA. user should provide
 *                               a valid buffer here if the triggerMode is set to DMA in paramConfig
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getDMAconfig(HWA_Handle handle, uint8_t dmaTriggerSrc, HWA_SrcDMAConfig *dmaConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCRegs *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if ((dmaTriggerSrc > (ptrHWADriver->hwAttrs->numDmaChannels -1)) || (dmaConfig == NULL))
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            dmaConfig->destAddr = (uint32_t)(&ctrlBaseAddr->DMA2HWA_TRIG);
            /* Read signatures from local core memory instead of HWA registers */
            dmaConfig->srcAddr = (uint32_t) &ctrlBaseAddr->SIGDMACHDONE[dmaTriggerSrc];
            dmaConfig->aCnt = sizeof(uint32_t);
            dmaConfig->bCnt = 1U;
            dmaConfig->cCnt = 1U;
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    return(retCode);
}

/*!
 *  @brief  Function to get HWA processing Memory information including address,
 *          size and number of banks.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  memInfo         Pointer to save HWA processing memory information
 *
  *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getHWAMemInfo(HWA_Handle handle, HWA_MemInfo *memInfo)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (memInfo == NULL)
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            memInfo->baseAddress    = ptrHWADriver->hwAttrs->accelMemBaseAddr;
            memInfo->numBanks       = SOC_HWA_NUM_MEM_BANKS;
            memInfo->bankSize       = ptrHWADriver->hwAttrs->accelMemSize / memInfo->numBanks;
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    return(retCode);
}

/*!
*  @brief  Function to poll the PARAM_DONE_SET_STATUS_0 and PARAM_DONE_SET_STATUS_1 registers to check if the
*          specified paramsets are finished.
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  numParamSets    number of paramsets need to poll.
*
*  @param  paramsetsArray  the specified paramset array
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_paramSetDonePolling(HWA_Handle handle, uint8_t numParamSets, uint8_t *paramsetsArray)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t              retCode = 0;
    volatile uint32_t             * paramDoneRegister1;
    volatile uint32_t             * paramDoneRegister2;
    uint32_t             checkBits1 = 0;
    uint32_t             checkBits2 = 0;
    uint32_t             paramCount;
    DSSHWACCRegs        *ctrlBaseAddr;
    uint8_t              pollingFlag;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

     /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((numParamSets == 0) || (numParamSets > ptrHWADriver->hwAttrs->numHwaParamSets))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if (paramsetsArray == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            pollingFlag = 1U;

            for (paramCount = 0; paramCount < numParamSets; paramCount++)
            {
                /* check if interrupt is enabled for each paramset */
                if ((ptrHWADriver->interrupt1ParamSetMask & (1<< paramsetsArray[paramCount])) ||
                    (ptrHWADriver->interrupt2ParamSetMask & (1 << paramsetsArray[paramCount])))
                {
                    pollingFlag = 0U;
                    break;
                }
            }
            if (pollingFlag)
            {
                ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
                paramDoneRegister1 = (uint32_t *)&ctrlBaseAddr->PARAM_DONE_SET_STATUS[0];
                paramDoneRegister2 = (uint32_t *)&ctrlBaseAddr->PARAM_DONE_SET_STATUS[1];

                for (paramCount = 0; paramCount < numParamSets; paramCount++)
                {
                    if (paramsetsArray[paramCount] < (uint8_t) 32)
                    {
                        checkBits1 |= (1U << paramsetsArray[paramCount]);
                    }
                    else
                    {
                        checkBits2 |= (1U << (paramsetsArray[paramCount] - 32));
                    }
                }
                while ((*paramDoneRegister1 & checkBits1) != checkBits1);
                while ((*paramDoneRegister2 & checkBits2) != checkBits2);
                /* clear the paramdone registers*/
                ctrlBaseAddr->PARAM_DONE_CLR[0] = checkBits1;
                ctrlBaseAddr->PARAM_DONE_CLR[1] = checkBits2;
            }
            else
            {
                retCode = HWA_PARAMSET_POLLINGNOTALLOWED;
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to poll the PARAM_DONE_SET_STATUS_0 or PARAM_DONE_SET_STATUS_1 registers to check if one single
*          specified paramset is finished.
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  paramsetIndex    the specified paramset index
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_singleParamSetDonePolling(HWA_Handle handle, uint8_t paramsetIndex)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t              retCode = 0;
    volatile uint32_t             * paramDoneRegister;
    uint32_t             checkBits = 0;
    DSSHWACCRegs         *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (paramsetIndex > ptrHWADriver->hwAttrs->numHwaParamSets)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            /* check to see if the paramset interrupt is disable */
            if ((ptrHWADriver->interrupt1ParamSetMask & (1U << paramsetIndex)) || (ptrHWADriver->interrupt2ParamSetMask & (1U << paramsetIndex)))
            {
                /* polling not allowed */
                retCode = HWA_PARAMSET_POLLINGNOTALLOWED;
            }
            else
            {
                ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

                if (paramsetIndex < 32U)
                {
                    paramDoneRegister = (uint32_t *)&ctrlBaseAddr->PARAM_DONE_SET_STATUS[0];
                    checkBits = (1U << paramsetIndex);
                }
                else
                {
                    paramDoneRegister = (uint32_t *)&ctrlBaseAddr->PARAM_DONE_SET_STATUS[1];
                    checkBits = (1U << (paramsetIndex - 32));
                }

                while (!(*paramDoneRegister & checkBits))
                {

                }
                /* clear the paramdone registers*/
                if (paramsetIndex < 32U)
                {
                    ctrlBaseAddr->PARAM_DONE_CLR[0] = checkBits;
                }
                else
                {
                    ctrlBaseAddr->PARAM_DONE_CLR[1] = checkBits;
                }
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to get the dma destination index with a given EDMA channel number
 *          This function assumes the EDMA channel number is from the first EDMA instance.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  edmaChanId      EDMA channell id
 *
 *  @param  hwaDestChan     Pointer to save destination channel index
 *
 *  @return     =0          Success
 *              <0          Error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getDMAChanIndex(HWA_Handle handle, uint8_t edmaChanId, uint8_t *hwaDestChan)
{
    HWA_Object      *ptrHWADriver = NULL;
    int32_t         retCode = 0;
    uint8_t         index;
    bool            foundChan = false;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (hwaDestChan == NULL)
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            for(index = 0; index < ptrHWADriver->hwAttrs->numDmaChannels; index ++)
            {
                if(edmaChanId == gHwaEDMAChanMapping[index])
                {
                    foundChan = true;
                    *hwaDestChan = index;
                    break;
                }
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    if(foundChan == false)
    {
        retCode = HWA_EINVAL;
    }
    return(retCode);
}

/*!
 *  @brief  Function to get the edma EDMA channel number from a given HWA paramset destination channel.
 *          This function assumes the EDMA channel number is from the first EDMA instance.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  hwaDMAdestChan  Destination channle id set in a paramset
 *
 *  @return     >=0         Upon success, EDMA channel number
 *              <0          Error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_getEDMAChanId(HWA_Handle handle, uint8_t hwaDMAdestChan)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (hwaDMAdestChan > (ptrHWADriver->hwAttrs->numDmaChannels-1))
        {
            /* invalid params */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            retCode = gHwaEDMAChanMapping[hwaDMAdestChan];
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    return(retCode);
}

/*!
*  @brief  Function to set the HWA RAM : HWA_RAM_TYPE_WINDOW_RAM, HWA_RAM_TYPE_VECTORMULTIPLY_RAM, HWA_RAM_TYPE_LUT_FREQ_DEROTATE_RAM or HWA_RAM_TYPE_SHUFFLE_RAM
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  ramType         Use defines \ref HWA_RAM_TYPE
*
*  @param  data            data pointer that needs to be copied to RAM
*
*  @param  dataSize        Size of data to be copied in size of bytes
*
*  @param  startIdx        start index (in terms of bytes) within RAM where data needs to be copied
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_configRam(HWA_Handle handle, uint8_t ramType, uint8_t *data, uint32_t dataSize, uint32_t startIdx)
{
    HWA_Object            *ptrHWADriver = NULL;
    int32_t               retCode = 0;
    uint8_t      * ramBaseAddr;
#ifdef HWA_PARAM_CHECK
    uint32_t       ramSizeInBytes;
#endif

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        ramSizeInBytes = gHwaRamCfg[ramType].ramSizeInBytes;
        if (data == NULL)
        {
            /* invalid data */
            retCode = HWA_EINVAL;
        }
        else if ((ramType != HWA_RAM_TYPE_WINDOW_RAM) && (ramType != HWA_RAM_TYPE_VECTORMULTIPLY_RAM)  &&
                 (ramType != HWA_RAM_TYPE_LUT_FREQ_DEROTATE_RAM) && (ramType != HWA_RAM_TYPE_SHUFFLE_RAM))
        {
            /* invalid config params */
            retCode = HWA_EINVAL;
        }
        else if ((dataSize == 0) || (dataSize > ramSizeInBytes) || (startIdx >= ramSizeInBytes) || ((startIdx + dataSize) > ramSizeInBytes))
        {
            /* invalid data size */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ramBaseAddr = (uint8_t *) gHwaRamCfg[ramType].ramBaseAddress;

            /* copy the RAM contents */
            memcpy((void *)&ramBaseAddr[startIdx],data,dataSize);
        }

        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to get the RAM starting address for one specified RAM type
*
*  @param  ramType           the RAM type see \ref HWA_RAM_TYPE for correct RAM types defined in HWA

*  @return the RAM starting address, or NULL if error occurs
*
*  @sa     HWA_open()
*/
extern uint32_t HWA_getRamAddress(uint8_t ramType)
{
    uint32_t      ramBaseAddr;
#ifdef HWA_PARAM_CHECK
        if (ramType > HWA_RAM_TYPE_HISTOGRAM_RAM)
        {
            /* invalid config params */
            ramBaseAddr = 0;
        }
        else
#endif
        {
            ramBaseAddr = gHwaRamCfg[ramType].ramBaseAddress;
        }

    return ramBaseAddr;
}

/*!
*  @brief  Function to read the HWA 2D statistics output RAM, including HWA_RAM_TYPE_2DSTAT_ITER_VAL,
*          HWA_RAM_TYPE_2DSTAT_ITER_IDX, HWA_RAM_TYPE_2DSTAT_SAMPLE_VAL, HWA_RAM_TYPE_2DSTAT_SAMPLE_IDX
*   or HWA_RAM_TYPE_HISTOGRAM_RAM, except the HWA_RAM_TYPE_HIST_THRESH_RAM
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  ramType         Use defines \ref HWA_RAM_TYPE, except HWA_RAM_TYPE_HIST_THRESH_RAM
*
*  @param  data            data pointer that needs to be copied to RAM
*
*  @param  dataSize        Size of data to be copied in size of bytes
*
*  @param  startIdx        start index (in terms of bytes) within RAM where data needs to be copied
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_readRam(HWA_Handle handle, uint8_t ramType, uint8_t *data, uint32_t dataSize, uint32_t startIdx)
{
    HWA_Object            *ptrHWADriver = NULL;
    int32_t               retCode = 0;
    uint8_t      * ramBaseAddr;
#ifdef HWA_PARAM_CHECK
    uint32_t       ramSizeInBytes;
#endif

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        ramSizeInBytes = gHwaRamCfg[ramType].ramSizeInBytes;
        if (data == NULL)
        {
            /* invalid data */
            retCode = HWA_EINVAL;
        }
        else if ((ramType != HWA_RAM_TYPE_2DSTAT_ITER_VAL) && (ramType != HWA_RAM_TYPE_2DSTAT_ITER_IDX) &&
            (ramType != HWA_RAM_TYPE_2DSTAT_SAMPLE_VAL) && (ramType != HWA_RAM_TYPE_2DSTAT_SAMPLE_IDX) &&
            (ramType != HWA_RAM_TYPE_HISTOGRAM_RAM)
            )
        {
            /* invalid config params */
            retCode = HWA_EINVAL;
        }
        else if ((dataSize == 0) || (dataSize > ramSizeInBytes) || (startIdx >= ramSizeInBytes) || ((startIdx + dataSize) > ramSizeInBytes))
        {
            /* invalid data size */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ramBaseAddr = (uint8_t *)gHwaRamCfg[ramType].ramBaseAddress;

            /* copy the ram contents to the output buffer */
            memcpy(data, (void *)&ramBaseAddr[startIdx],  dataSize);
        }

        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to read the HWA HWA_RAM_TYPE_HIST_THRESH_RAM RAM
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*
*  @param  cdfThresholdResult   buffer the CDF threshold results see \ref HWA_CdfThreshold,
*
*  @param  numSampleIndices    the number of sample indices, which needs to be read out.
*
*  @param  startSampleIdx    the start sample indice
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_readHistThresholdRam(HWA_Handle handle, HWA_CdfThreshold *cdfThresholdResult, uint8_t numSampleIndices, uint8_t startSampleIdx)
{
    HWA_Object            *ptrHWADriver = NULL;
    int32_t                retCode = 0;
    uint32_t              *ramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((numSampleIndices > 64) || (startSampleIdx > 63) || (startSampleIdx + numSampleIndices > 64))
        {
            /* invalid data size */
            retCode = HWA_EINVAL;
        }
        else if (cdfThresholdResult == NULL)
        {
            /* invalid data size */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ramBaseAddr = (uint32_t *)gHwaRamCfg[HWA_RAM_TYPE_HIST_THRESH_RAM].ramBaseAddress;
            memcpy((void *)cdfThresholdResult, (void *)&ramBaseAddr[startSampleIdx], sizeof(uint32_t) * numSampleIndices);

        }

        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to enable the CPU and/or DMA interrupt after a paramSet completion.
*          The CPU interrupt for every paramset completion may not be supported on all
*          devices - see \ref HWA_Attrs::isConcurrentAccessAllowed
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  paramsetIdx     A valid paramSet index for which the intrConfig is provided.
*
*  @param  intrConfig      HWA Interrupt Config Parameters
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_enableParamSetInterrupt(HWA_Handle handle, uint8_t paramsetIdx, HWA_InterruptConfig *intrConfig)
{
    HWA_Object          *ptrHWADriver = NULL;
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) false,(bool) true,paramsetIdx);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (intrConfig == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if ((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_DMA) &&
                 (intrConfig->dma.dstChannel > (ptrHWADriver->hwAttrs->numDmaChannels-1)))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else if (((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1) ||(intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2)) &&
                 (ptrHWADriver->hwAttrs->isConcurrentAccessAllowed == (bool) false))
        {
            /* This version of HWA IP doesn't allow concurrent read of HWACCREGx registers when HWA is active.
             * While enabling of this CPU interrupt is fine, it is the handling of this interrupt in ISR that
             * would require read of HWACCREG4 register when HWA is active which is prohibited.
             */
            retCode = HWA_ENOTSUPP;
        }
        else
#endif
        {
            paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr+paramsetIdx*sizeof(DSSHWACCPARAMRegs));
            if ((intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1) ||(intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2))
            {
                /* handle this flag for IPs/devices where concurrent read access of common registers are allowed
                   while HWA is executing paramsets */
                uintptr_t           key;

                /* Disable preemption while setting the registers; since these are accessed via ISR as well */
                key = HwiP_disable();

                /* save the interrupt context */
                ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackFn = intrConfig->cpu.callbackFn;
                ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackArg= intrConfig->cpu.callbackArg;

                /* enable the interrupt to CPU in H/W, set the CR5INTERN */
                if (intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1)
                {
                    HwiP_enableInt(ptrHWADriver->hwAttrs->intNum1ParamSet);

                    /* enable the Interrupt 1*/
                    CSL_FINSR(paramBaseAddr->HEADER, HEADER_CPU_INTR1_EN_END, HEADER_CPU_INTR1_EN_START, 1U);
                    ptrHWADriver->interrupt1ParamSetMask |= ( ((uint64_t) 1U) << paramsetIdx);
                    /* disable the interrupt 2, clear same paramset in interrupt 2 mask*/
                    CSL_FINSR(paramBaseAddr->HEADER, HEADER_CPU_INTR2_EN_END, HEADER_CPU_INTR2_EN_START, 0U);
                    ptrHWADriver->interrupt2ParamSetMask &= (~( ((uint64_t)1U) << paramsetIdx));

                }
                if (intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2)
                {
                    HwiP_enableInt(ptrHWADriver->hwAttrs->intNum2ParamSet);

                    /* enable the interrupt 2*/
                    CSL_FINSR(paramBaseAddr->HEADER, HEADER_CPU_INTR2_EN_END, HEADER_CPU_INTR2_EN_START, 1U);
                    ptrHWADriver->interrupt2ParamSetMask |= ( ((uint64_t) 1U) << paramsetIdx);
                    /* disable the interrupt 1, clear same paramse in interrupt 1 mask*/
                    CSL_FINSR(paramBaseAddr->HEADER, HEADER_CPU_INTR1_EN_END, HEADER_CPU_INTR1_EN_START, 0U);
                    ptrHWADriver->interrupt1ParamSetMask &= (~( ((uint64_t) 1U) << paramsetIdx));
                }
                /* Restore the interrupts */
                HwiP_restore(key);
            }
            if (intrConfig->interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_DMA)
            {
                /* set the destination channel number , set ACC2DMA_TRIGDST bits*/
                CSL_FINSR(paramBaseAddr->HEADER, HEADER_HWA2DMA_TRIGDST_END, HEADER_HWA2DMA_TRIGDST_START,intrConfig->dma.dstChannel);
                /* enable the interrupt to DMA, set DMATRIGEN bits */
                CSL_FINSR(paramBaseAddr->HEADER, HEADER_DMATRIG_EN_END, HEADER_DMATRIG_EN_START,1U);
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) false,(bool) true,paramsetIdx);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to enable the CPU interrupt after all programmed paramSets have been completed in the background or ALT thread.
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  threadIdx       the thread Idx, see \ref HWA_THREAD for the correct value
*
*  @param  callbackFn      user defined callback function to be called when this interrupt is generated
*
*  @param  callbackArg     user defined callback arg for the callback function
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_enableDoneInterrupt(HWA_Handle handle, uint8_t threadIdx, HWA_Done_IntHandlerFuncPTR callbackFn, void * callbackArg )
{
    HWA_Object          *ptrHWADriver = NULL;
    uintptr_t           key;
    int32_t             retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check state of driver */
    if ((ptrHWADriver == NULL)  || (ptrHWADriver->refCnt == 0U))
    {
        retCode = HWA_ENOINIT;
    }
    else
    {
        /* enable the interrupt to CPU in H/W */
        if (threadIdx == HWA_THREAD_BACKGROUNDCONTEXT)
        {
            /* save the interrupt context */
            ptrHWADriver->interruptCtxDone.bIsEnabled = true;
            ptrHWADriver->interruptCtxDone.callbackFn = callbackFn;
            ptrHWADriver->interruptCtxDone.callbackArg = callbackArg;
            HwiP_enableInt(ptrHWADriver->hwAttrs->intNumDone);
        }
        else
        {
            /* save the interrupt context */
            ptrHWADriver->interruptCtxDoneALT.bIsEnabled = true;
            ptrHWADriver->interruptCtxDoneALT.callbackFn = callbackFn;
            ptrHWADriver->interruptCtxDoneALT.callbackArg = callbackArg;
            HwiP_enableInt(ptrHWADriver->hwAttrs->intNumDoneALT);
        }

        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* Restore the interrupts: */
    HwiP_restore(key);

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to disable the CPU and/or DMA interrupt after a paramSet completion.
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle              A HWA_Handle returned from HWA_open()
 *
 *  @param  paramsetIdx         A valid paramSet index for which the interrupt is to be disabled
 *
 *  @param  interruptTypeFlag   Flag to indicate if CPU and/or DMA interrupts are to be disabled
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_disableParamSetInterrupt(HWA_Handle handle, uint8_t paramsetIdx, uint8_t interruptTypeFlag)
{
    HWA_Object          *ptrHWADriver = NULL;
    uintptr_t           key;
    int32_t             retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) false,(bool) true,paramsetIdx);
    if (retCode==0)
    {
        paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr+paramsetIdx*sizeof(DSSHWACCPARAMRegs));

        if ((interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1) || (interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2))
        {

            /* Disable preemption while setting the interrupt context */
            key = HwiP_disable();

            /* update the interrupt context */
            ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackFn = NULL;
            ptrHWADriver->interruptCtxParamSet[paramsetIdx].callbackArg= NULL;

            /* disable the interrupt to CPU in H/W, clear CR5INTREN bit*/
            if (interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1)
            {
                CSL_FINSR(paramBaseAddr->HEADER, HEADER_CPU_INTR1_EN_END, HEADER_CPU_INTR1_EN_START, 0U);
                ptrHWADriver->interrupt1ParamSetMask &= (~( ((uint64_t)1U) << paramsetIdx));
            }
            if (interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR2)
            {
                CSL_FINSR(paramBaseAddr->HEADER, HEADER_CPU_INTR2_EN_END, HEADER_CPU_INTR2_EN_START, 0U);
                ptrHWADriver->interrupt2ParamSetMask &= (~( ((uint64_t)1U) << paramsetIdx));
            }
            /* Restore the interrupts */
            HwiP_restore(key);
        }
        if (interruptTypeFlag & HWA_PARAMDONE_INTERRUPT_TYPE_DMA)
        {
            /* disable the interrupt to DMA, clear DMATRIGEN bits */
            CSL_FINSR(paramBaseAddr->HEADER, HEADER_DMATRIG_EN_END, HEADER_DMATRIG_EN_START,0U);
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) false,(bool) true,paramsetIdx);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to disable the CPU interrupt after all programmed paramSets have been completed.
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  threadIdx       the thread Idx, see \ref HWA_THREAD for the correct value
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_disableDoneInterrupt(HWA_Handle handle, uint8_t threadIdx)
{
    HWA_Object              *ptrHWADriver = NULL;
    uintptr_t                key;
    int32_t                  retCode = 0;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* Disable preemption while setting the registers */
    key = HwiP_disable();

    /* check state of driver */
    if ((ptrHWADriver == NULL)  || (ptrHWADriver->refCnt == 0U))
    {
        retCode = HWA_ENOINIT;
    }
    else
    {
        if (threadIdx == HWA_THREAD_BACKGROUNDCONTEXT)
        {
            /* save the interrupt context */
            ptrHWADriver->interruptCtxDone.bIsEnabled = false;
            ptrHWADriver->interruptCtxDone.callbackFn = NULL;
            ptrHWADriver->interruptCtxDone.callbackArg = NULL;

            /* disable the interrupt to CPU in H/W */
            HwiP_disableInt(ptrHWADriver->hwAttrs->intNumDone);
        }
        else
        {
            /* save the interrupt context */
            ptrHWADriver->interruptCtxDoneALT.bIsEnabled = false;
            ptrHWADriver->interruptCtxDoneALT.callbackFn = NULL;
            ptrHWADriver->interruptCtxDoneALT.callbackArg = NULL;
            HwiP_disableInt(ptrHWADriver->hwAttrs->intNumDoneALT);
        }
    }

    /* Restore the interrupts */
    HwiP_restore(key);

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to enable the state machine of the HWA. This should be called after
 *          paramset and RAM have been programmed
 *
 *  @pre    HWA_open() HWA_ConfigCommon() HWA_ConfigParamSet HWA_ConfigRam has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  flagEnDis       Enable/Disable Flag: 0-disable, 1-enable
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_enable(HWA_Handle handle, uint8_t flagEnDis)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* set the ACCENABLE bits to 0x7 */
        if (flagEnDis == 1U)
        {
              /*bit ACCENABLE to enable bits*/
              CSL_FINSR(ctrlBaseAddr->HWA_ENABLE, HWA_ENABLE_HWA_EN_END, HWA_ENABLE_HWA_EN_START, 0x7U);

        }
        /* set the ACCENABLE bits to 0x0 */
        else
        {
            CSL_FINSR(ctrlBaseAddr->HWA_ENABLE, HWA_ENABLE_HWA_EN_END, HWA_ENABLE_HWA_EN_START, 0x0U);

        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to enable or disable the context switching in hwa
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  flagEnDis       Enable/Disable Flag: 0-disable, 1-enable
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_enableContextSwitch()
*/
extern int32_t HWA_enableContextSwitch(HWA_Handle handle, uint8_t flagEnDis)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
          /* set the CS_CONFIG_CS_ENABLE bits to 0x1 */
        if (flagEnDis == 1U)
        {
            /*bit ACCENABLE to enable bits*/
            CSL_FINSR(ctrlBaseAddr->CS_CONFIG, CS_CONFIG_CS_ENABLE_END, CS_CONFIG_CS_ENABLE_START, 0x1U);

        }
        /* set the CS_CONFIG_CS_ENABLE bits to 0x0 */
        else
        {
            CSL_FINSR(ctrlBaseAddr->CS_CONFIG, CS_CONFIG_CS_ENABLE_END, CS_CONFIG_CS_ENABLE_START, 0x0U);

        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}
/*!
*  @brief  Function to manually trigger the execution of the state machine via software,
*           the software trigger through either FW2HWA_TRIG_0 or FW2HWA_TRIG_1 register
*           if the trigger is set to HWA_TRIG_MODE_SOFTWARE from DSP, trigger is done through FW2HWA_TRIG_0
*           if the trigger is set to HWA_TRIG_MODE_SOFTWARE2 from DSP, trigger is done through FW2HWA_TRIG_1
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*  @param  swTriggerType   see maros \ref HWA_TRIG_MODE, takes value either HWA_TRIG_MODE_SOFTWARE
*                           or HWA_TRIG_MODE_SOFTWARE2
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
int32_t HWA_setSoftwareTrigger(HWA_Handle handle, uint8_t swTriggerType)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        if(swTriggerType == HWA_TRIG_MODE_SOFTWARE)
        {
            CSL_FINSR(ctrlBaseAddr->FW2HWA_TRIG_0,
                    FW2HWA_TRIG_0_FW2HWA_TRIGGER_0_END,
                    FW2HWA_TRIG_0_FW2HWA_TRIGGER_0_START,
                    1U);
        }
        else if (swTriggerType == HWA_TRIG_MODE_SOFTWARE2)
        {
            CSL_FINSR(ctrlBaseAddr->FW2HWA_TRIG_1,
                    FW2HWA_TRIG_0_FW2HWA_TRIGGER_0_END,
                    FW2HWA_TRIG_1_FW2HWA_TRIGGER_1_END,
                    1U);
        }
        else
        {
            retCode = HWA_EINVAL;
        }

        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to manually trigger the execution of the state machine via software in context switch,
*           the software trigger through CS_FW2ACC_TRIG register
*
*  @pre    HWA_open() and HWA_enableContextSwitch() have been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_setContextswitchSoftwareTrigger(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        CSL_FINSR(ctrlBaseAddr->CS_FW2ACC_TRIG,
                  CS_FW2ACC_TRIG_FW2HWA_TRIGGER_CS_END,
                  CS_FW2ACC_TRIG_FW2HWA_TRIGGER_CS_START,
                  1U);
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to manually trigger the execution of the state machine via DMA trigger in context switch
*
*  @pre    HWA_open() and HWA_enableContextSwitch() have been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  idx            DMA channel number for whom software should simulate the trigger
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_setContextswitchDMAManualTrigger(HWA_Handle handle, uint8_t idx)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

#ifdef HWA_PARAM_CHECK
        if (idx > SOC_HWA_NUM_DMA_CHANNEL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif

        /* set the DMA2ACCTRIG bits to DMA Channel */
        CSL_FINSR(ctrlBaseAddr->DMA2HWA_TRIG, idx, idx, 1U);

        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function for software to reset the DC accumulators or interference statistics accumulators
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*  @param  accumulatortype  see maros \ref HWA_ACCUMULATORREG_TYPE, takes value either HWA_ACCUMULATORREG_TYPE_DC
*                           or HWA_ACCUMULATORREG_TYPE_INTERF
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_softwareResetAccumulators(HWA_Handle handle, uint8_t accumulatortype)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        if (accumulatortype == HWA_ACCUMULATORREG_TYPE_DC)
        {
            CSL_FINSR(ctrlBaseAddr->DC_EST_RESET_SW,
                      DC_EST_RESET_SW_DC_EST_RESET_SW_END,
                      DC_EST_RESET_SW_DC_EST_RESET_SW_START,
                      1U);
        }
        else
        {
            CSL_FINSR(ctrlBaseAddr->INTF_STATS_RESET_SW,
                      INTF_STATS_RESET_SW_INTF_STATS_RESET_SW_END,
                      INTF_STATS_RESET_SW_INTF_STATS_RESET_SW_START,
                      1U);
        }

        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function resets the param set counter used in recurise windowing mode with REC_WIN_MODE_SEL is set to 1
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_softwareResetRecursiveWinKvalue(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        CSL_FINSR(ctrlBaseAddr->RECWIN_RESET_SW,
                  RECWIN_RESET_SW_RECWIN_RESET_SW_END,
                  RECWIN_RESET_SW_RECWIN_RESET_SW_START,
                  1U);
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function resets the TWID_INCR_DELTA_FRAC register if complex multiply is configured as frequency shifter mode with frequency increment
*
*  @pre    HWA_open() has been called.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_softwareResetTwidIncrDeltaFrac(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        CSL_FINSR(ctrlBaseAddr->TWID_INCR_DELTA_FRAC_RESET_SW,
                  TWID_INCR_DELTA_FRAC_RESET_SW_TWID_INCR_DELTA_FRAC_RESET_SW_END,
                  TWID_INCR_DELTA_FRAC_RESET_SW_TWID_INCR_DELTA_FRAC_RESET_SW_START,
                  1U);
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to manually trigger the execution of the state machine waiting on DMA via software
 *
 *  @pre    HWA_open() has been called.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  idx            DMA channel number for whom software should simulate the trigger
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open()
 */
int32_t HWA_setDMA2ACCManualTrig(HWA_Handle handle, uint8_t idx)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
#ifdef HWA_PARAM_CHECK
        if (idx > SOC_HWA_NUM_DMA_CHANNEL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif

        /* set the DMA2ACCTRIG bits to DMA Channel */
        CSL_FINSR(ctrlBaseAddr->DMA2HWA_TRIG, idx, idx,1U);

        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to set the source address for one paramset
*
*  @pre    HWA_open() has been called.
*
*  @param  handle             A HWA_Handle returned from HWA_open()
*
*  @param  paramparamsetIdxIdx           the paramset index
*
*  @param  sourceAddress      source address
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open()
*/
extern int32_t HWA_setSourceAddress(HWA_Handle handle, uint16_t paramsetIdx, uint32_t sourceAddress)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCPARAMRegs *paramBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) true, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if (paramsetIdx > SOC_HWA_NUM_PARAM_SETS)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            paramBaseAddr = (DSSHWACCPARAMRegs *)(ptrHWADriver->hwAttrs->paramBaseAddr + paramsetIdx * sizeof(DSSHWACCPARAMRegs));
            CSL_FINSR(paramBaseAddr->SRC, SRC_SRCADDR_END, SRC_SRCADDR_START, sourceAddress);
        }

        HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) true, 0);
    }
    return (retCode);

}


/*!
 *  @brief  Function to read the Clip Status registers
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  clipStatusResult  the clip status result
 *                        if it is HWA_CLIPREG_TYPE_DCACC, the 12 lsb are the clip status of bot I/Q of DC acculuators 0 to 11
 *                        if it is HWA_CLIPREG_TYPE_DCEST, the 12 lsb are the clip sttus of DC estimatsion (both I/Q combined)
 *                        if it is HWA_CLIPREG_TYPE_DCSUB, the last lsb is the DC subtraction clip status
 *                        if it is HWA_CLIPREG_TYPE_INTFSTATS_MAGACC or HWA_CLIPREG_TYPE_INTFSTATS_MAGDIFFACC, the 12 lsb are the interference magnitude or magnitude diff accumulator clip status
 *                        if it is HWA_CLIPREG_TYPE_INTFSTATS_MAGSUM or HWA_CLIPREG_TYPE_INTFSTATS_MAGDIFFSUM, the 1 bit in lsb is the interference magnitude sum or magnitude diff sum clip status
 *                        if it is HWA_CLIPREG_TYPE_INTFSTATS_MAGTHRESHOLD or HWA_CLIPREG_TYPE_INTFSTATS_MAGDIFFTHRESHOLD, the 12 lsb are the interference magnitude difference threshold clip status
 *                        if it is HWA_CLIPREG_TYPE_TWIDINCR_DELTAFRAC, the last 1 lsb is the clip status for TWID_INCR_DELTA_FRAC accumulator
 *                        if it is HWA_CLIPREG_TYPE_CHANCOMB, the last lsb is the clip status of the channel combination
 *                        if it is HWA_CLIPREG_TYPE_FFT, the last 13 lsb indicates any saturation/clipping for each FFT butterfly stages
 *                        if it is HWA_CLIPREG_TYPE_INPUTFORMAT or HWA_CLIPREG_TYPE_OUTPUTFORMAT, the alst lsb is the clip stauts for input or output formatter
 *
 *  @param  type            see \ref HWA_CLIPREG_TYPE macro for correct clip register values .
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
extern int32_t HWA_readClipStatus(HWA_Handle handle, uint16_t *clipStatusResult, uint8_t type)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    *clipStatusResult = 0;
    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (type > HWA_CLIPREG_TYPE_OUTPUTFORMAT)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

            if (type == HWA_CLIPREG_TYPE_DCACC)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->DC_ACC_CLIP_STATUS,
                                                        DC_ACC_CLIP_STATUS_DC_ACC_CLIP_STATUS_END,
                                                        DC_ACC_CLIP_STATUS_DC_ACC_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_DCEST)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->DC_EST_CLIP_STATUS,
                                                        DC_EST_CLIP_STATUS_DC_EST_CLIP_STATUS_END,
                                                        DC_EST_CLIP_STATUS_DC_EST_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_DCSUB)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->DC_SUB_CLIP, DC_SUB_CLIP_DC_SUB_CLIP_END, DC_SUB_CLIP_DC_SUB_CLIP_START);
            }
            else if (type == HWA_CLIPREG_TYPE_INTFSTATS_MAGACC)
            {

                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_ACC_CLIP_STATUS,
                                                  INTF_STATS_ACC_CLIP_STATUS_INTF_STATS_MAG_ACCUMULATOR_CLIP_STATUS_END,
                                                  INTF_STATS_ACC_CLIP_STATUS_INTF_STATS_MAG_ACCUMULATOR_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_INTFSTATS_MAGDIFFACC)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_ACC_CLIP_STATUS,
                                                        INTF_STATS_ACC_CLIP_STATUS_INTF_STATS_MAGDIFF_ACCUMULATOR_CLIP_STATUS_END,
                                                        INTF_STATS_ACC_CLIP_STATUS_INTF_STATS_MAGDIFF_ACCUMULATOR_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_INTFSTATS_MAGSUM)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_SUM_MAG_VAL_CLIP_STATUS,
                                                       INTF_STATS_SUM_MAG_VAL_CLIP_STATUS_INTF_STATS_SUM_MAG_VAL_CLIP_STATUS_END,
                                                       INTF_STATS_SUM_MAG_VAL_CLIP_STATUS_INTF_STATS_SUM_MAG_VAL_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_INTFSTATS_MAGDIFFSUM)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_SUM_MAGDIFF_VAL_CLIP_STATUS,
                                                        INTF_STATS_SUM_MAGDIFF_VAL_CLIP_STATUS_INTF_STATS_SUM_MAGDIFF_VAL_CLIP_STATUS_END,
                                                        INTF_STATS_SUM_MAGDIFF_VAL_CLIP_STATUS_INTF_STATS_SUM_MAGDIFF_VAL_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_INTFSTATS_MAGTHRESHOLD)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_THRESH_CLIP_STATUS,
                                                  INTF_STATS_THRESH_CLIP_STATUS_INTF_STATS_THRESH_MAG_CLIP_STATUS_END,
                                                  INTF_STATS_THRESH_CLIP_STATUS_INTF_STATS_THRESH_MAG_CLIP_STATUS_START);

            }
            else if (type == HWA_CLIPREG_TYPE_INTFSTATS_MAGDIFFTHRESHOLD)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_THRESH_CLIP_STATUS,
                                                  INTF_STATS_THRESH_CLIP_STATUS_INTF_STATS_THRESH_MAGDIFF_CLIP_STATUS_END,
                                                  INTF_STATS_THRESH_CLIP_STATUS_INTF_STATS_THRESH_MAGDIFF_CLIP_STATUS_START);
            }

            else if (type == HWA_CLIPREG_TYPE_TWIDINCR_DELTAFRAC)
            {
                *clipStatusResult = (uint8_t)CSL_FEXTR(ctrlBaseAddr->TWID_INCR_DELTA_FRAC_CLIP_STATUS,
                                                 TWID_INCR_DELTA_FRAC_CLIP_STATUS_TWID_INCR_DELTA_FRAC_CLIP_STATUS_END,
                                                 TWID_INCR_DELTA_FRAC_CLIP_STATUS_TWID_INCR_DELTA_FRAC_CLIP_STATUS_START);
           }
            else if (type == HWA_CLIPREG_TYPE_CHANCOMB)
            {
                *clipStatusResult = (uint8_t)CSL_FEXTR(ctrlBaseAddr->CHANNEL_COMB_CLIP_STATUS,
                                                 CHANNEL_COMB_CLIP_STATUS_CHANNEL_COMB_CLIP_STATUS_END,
                                                 CHANNEL_COMB_CLIP_STATUS_CHANNEL_COMB_CLIP_STATUS_START);
            }
            else if (type == HWA_CLIPREG_TYPE_FFT)
            {
                 /* read the FFTCLCIPSTAT bits of FFTCLIP */
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->FFT_CLIP, FFT_CLIP_FFT_CLIP_END, FFT_CLIP_FFT_CLIP_START);
            }
            else if (type == HWA_CLIPREG_TYPE_INPUTFORMAT)
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->IP_OP_FORMATTER_CLIP_STATUS,
                                                        IP_OP_FORMATTER_CLIP_STATUS_IP_FORMATTER_CLIP_STATUS_END,
                                                        IP_OP_FORMATTER_CLIP_STATUS_IP_FORMATTER_CLIP_STATUS_START);
            }
#ifdef HWA_PARAM_CHECK
            else
#else
            else if (type == HWA_CLIPREG_TYPE_OUTPUTFORMAT)
#endif
            {
                *clipStatusResult = (uint16_t)CSL_FEXTR(ctrlBaseAddr->IP_OP_FORMATTER_CLIP_STATUS,
                                           IP_OP_FORMATTER_CLIP_STATUS_OP_FORMATTER_CLIP_STATUS_END,
                                           IP_OP_FORMATTER_CLIP_STATUS_OP_FORMATTER_CLIP_STATUS_START);
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to clear the Clip Status registers
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *  @param  type            see \ref HWA_CLIPREG_TYPE macro for correct clip register values .
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_clearClipStatus(HWA_Handle handle, uint8_t type)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (type > HWA_CLIPREG_TYPE_OUTPUTFORMAT)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            if (type == HWA_CLIPREG_TYPE_FFT)
            {
                CSL_FINSR(ctrlBaseAddr->CLR_FFTCLIP, CLR_FFTCLIP_CLR_FFTCLIP_END, CLR_FFTCLIP_CLR_FFTCLIP_START, 1U);
            }
            else //for all other type
            {
                /* channel_comb_clip_status
                   dc_acc_clip_status
                   dc_est_clip_status
                   intf_stats_mag_accumulator_clip_status
                   intf_stats_magdiff_accumulator_clip_status
                   intf_stats_thresh_mag_clip_status
                   intf_stats_thresh_magdiff_clip_status
                   intf_stats_sum_mag_val_clip_status
                   intf_stats_sum_magdiff_val_clip_status
                   twid_incr_delta_frac_clip_status
                   ip_formatter_clip_status
                   op_formatter_clip_status
                   dc_sub_clip
                */
                CSL_FINSR(ctrlBaseAddr->CLR_CLIP_MISC,
                          CLR_CLIP_MISC_CLR_CLIP_STATUS_END,
                          CLR_CLIP_MISC_CLR_CLIP_STATUS_START, 1U);

            }
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to read the 4 sets of 'MAX' statistics register
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  pStats          pointer to a memory of type HWA_Stats where value of all the
 *                          Max and statistics Registers would be copied
 *
 *  @param  numIter         number of iterations to read. Value 1-4 should be provided.
 *                          User is expected to provide enough space for the pStats to hold 'numIter' worth of HWA_Stats
 *                          Ex: HWA_Stats appHWAStats[3]; HWA_readStatsReg(appHWAhandle,appHWAStats,3);
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_readStatsReg(HWA_Handle handle, HWA_Stats *pStats, uint8_t numIter)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if ((pStats == NULL) || (numIter>4) || (numIter==0))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            uint8_t i=0;
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read the MAX and SUM registers*/
            for (i=0;i<numIter;i++)
            {
                pStats[i].maxValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->MAX_VALUE[i], MAX1_VALUE_MAX1_VALUE_END, MAX1_VALUE_MAX1_VALUE_START);
                pStats[i].maxIndex = (uint16_t)CSL_FEXTR(ctrlBaseAddr->MAX_INDEX[i], MAX1_INDEX_MAX1_INDEX_END, MAX1_INDEX_MAX1_INDEX_START);
                pStats[i].iSumLSB  = ctrlBaseAddr->I_SUM_VALUE[i].accValLSB;
                pStats[i].iSumMSB  = (uint8_t)CSL_FEXTR(ctrlBaseAddr->I_SUM_VALUE[i].accValMSB, I_SUM1_MSB_I_SUM1_MSB_END, I_SUM1_MSB_I_SUM1_MSB_START);
                pStats[i].qSumLSB  = ctrlBaseAddr->Q_SUM_VALUE[i].accValLSB;
                pStats[i].qSumMSB  = (uint8_t)CSL_FEXTR(ctrlBaseAddr->Q_SUM_VALUE[i].accValMSB, Q_SUM1_MSB_Q_SUM1_MSB_END, Q_SUM1_MSB_Q_SUM1_MSB_START);
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to read the DC_EST_I/Q register
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  pbuf            pointer to a memory where value of the
*                          DC_EST_I/Q Registers would be copied, even element is I value, and odd element is Q value
*
*  @param  startIdx        The estimated DC results for the first RX channel Idx, must be less than 11.
*
*  @param  size            The number of Rx channels needed to be read.
*                          It should be atleast 12.
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
*/
extern int32_t HWA_readDCEstimateReg(HWA_Handle handle, cmplx32ImRe_t *pbuf, uint8_t startIdx,  uint8_t size)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs             *ctrlBaseAddr;
    uint32_t                 realTempValue,imagTempValue;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((pbuf == NULL) || ((startIdx + size) > 12U) || (size == 0U))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            uint8_t i = 0U;
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read the DC estimate I and Q values registers*/
            for (i = 0U; i<size; i++)
            {
                realTempValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->DC_EST_I_VAL[startIdx + i], DC_EST_I_0_VAL_DC_EST_I_0_VAL_END, DC_EST_I_0_VAL_DC_EST_I_0_VAL_START);
                if (realTempValue & 0x800000)
                {
                    pbuf[i].real = (int32_t)(0xFF000000 + realTempValue);
                }
                else
                {
                    pbuf[i].real = (int32_t)realTempValue;
                }
                imagTempValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->DC_EST_Q_VAL[startIdx + i], DC_EST_Q_0_VAL_DC_EST_Q_0_VAL_END, DC_EST_Q_0_VAL_DC_EST_Q_0_VAL_START);
                if (imagTempValue & 0x800000)
                {
                    pbuf[i].imag = (int32_t)(0xFF000000 + imagTempValue);
                }
                else
                {
                    pbuf[i].imag = (int32_t)imagTempValue;
                }

            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to read the interference threshold MAG or MAGDIFF Accumulator register
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  accBuf          pointer to the accumulator result buffer for MAG or MAGDIFF
*
*  @param  type            see \ref HWA_INTERFERENCE_THRESHOLD_TYPE macro for the either MAG or MAGDIFF accumulator .
*
*  @param  startIdx        The first accumulator channel needs to be read, must be less than 11.
*
*  @param  size            The number of channels needed to be read.
*                          It should be less than 12.
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
*/
extern int32_t HWA_readIntfAccReg(HWA_Handle handle, uint64_t *accBuf, uint8_t type, uint8_t startIdx, uint8_t size)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs             *ctrlBaseAddr;
    uint32_t                lsbValue, msbValue;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((accBuf == NULL) || ((startIdx + size) > 12U) || (size == 0U) || (type > HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            uint8_t i = 0;
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read interference magnitude accumulator*/
            if (type == HWA_INTERFERENCE_THRESHOLD_TYPE_MAG)
            {
                for (i = 0; i < size; i++)
                {
                    accBuf[i] = 0;
                    lsbValue =  ctrlBaseAddr->INTF_STATS_MAG_ACC[startIdx + i].accValLSB;
                    msbValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_MAG_ACC[startIdx + i].accValMSB,
                                                   INTF_STATS_MAG_ACC_0_MSB_INTF_STATS_MAG_ACC_0_MSB_END,
                                                   INTF_STATS_MAG_ACC_0_MSB_INTF_STATS_MAG_ACC_0_MSB_START);
                    accBuf[i] |= (uint64_t)msbValue ;
                    accBuf[i] = (accBuf[i] << 32);
                    accBuf[i] |= (uint64_t)lsbValue;


                }
            }
#ifdef HWA_PARAM_CHECK
            else  //(type == HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF)
#else
            /* read interference magnitude difference accumulator*/
            else if (type == HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF)
#endif
            {
                for (i = 0; i < size; i++)
                {
                    accBuf[i] = 0;
                    lsbValue = ctrlBaseAddr->INTF_STATS_MAGDIFF_ACC[startIdx + i].accValLSB;
                    msbValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->INTF_STATS_MAGDIFF_ACC[startIdx + i].accValMSB,
                                                   INTF_STATS_MAGDIFF_ACC_0_MSB_INTF_STATS_MAGDIFF_ACC_0_MSB_END,
                                                   INTF_STATS_MAGDIFF_ACC_0_MSB_INTF_STATS_MAGDIFF_ACC_0_MSB_START);
                    accBuf[i] |= msbValue;
                    accBuf[i] = (accBuf[i] << 32);
                    accBuf[i] |= lsbValue;
                }
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to read the DC estimation accumulator register,
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  accbuf           pointer to a memory for accumulator results.
*
*  @param  startIdx        The first accumulator channel needs to be read, must be less than 11.
*
*  @param  size            The number of channels needed to be read.
*                          It should be less than 12.
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
*/
extern int32_t HWA_readDCAccReg(HWA_Handle handle, cmplx64ImRe_t *accbuf, uint8_t startIdx, uint8_t size)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs             *ctrlBaseAddr;
    uint32_t                 lsbValue, msbValue;
    uint64_t                 tempValue;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((accbuf == NULL) || ((startIdx + size) > 12) || (size == 0) )
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            uint8_t i = 0;
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read the DC accumulator registers*/
            for (i = 0; i < size; i++)
            {
                tempValue = 0;
                lsbValue = (uint32_t)ctrlBaseAddr->DC_ACC_I_VAL[startIdx + i].accValLSB;
                msbValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->DC_ACC_I_VAL[startIdx + i].accValMSB,
                                                                DC_ACC_I_0_VAL_MSB_DC_ACC_I_0_VAL_MSB_END,
                                                                DC_ACC_I_0_VAL_MSB_DC_ACC_I_0_VAL_MSB_START);
                if (msbValue & 0x8)
                    msbValue += 0xFFFFFFF0;
                tempValue = msbValue;
                tempValue = (tempValue << 32);
                tempValue |= lsbValue;
                accbuf[i].real = (int64_t)tempValue;

                lsbValue = (uint32_t)ctrlBaseAddr->DC_ACC_Q_VAL[startIdx + i].accValLSB;
                msbValue = (uint32_t)CSL_FEXTR(ctrlBaseAddr->DC_ACC_Q_VAL[startIdx + i].accValMSB,
                                                                    DC_ACC_Q_0_VAL_MSB_DC_ACC_Q_0_VAL_MSB_END,
                                                                    DC_ACC_Q_0_VAL_MSB_DC_ACC_Q_0_VAL_MSB_START);
                if (msbValue & 0x8)
                    msbValue += 0xFFFFFFF0;
                tempValue = msbValue;
                tempValue = (tempValue << 32);
                tempValue |= lsbValue;
                accbuf[i].imag = (int64_t)tempValue;

            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to read the PEAKCNT register
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  pbuf            pointer to a memory where value of the
 *                          PEAKCNT Registers would be copied
 *
 *  @param  size            size (in bytes) of the pbuf register provided.
 *                          It should be atleast 2 bytes.
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_readCFARPeakCountReg(HWA_Handle handle, uint8_t *pbuf, uint8_t size)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;
    uint16_t                *peakCnt;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if ((size<2) || (pbuf == NULL))
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            peakCnt = (uint16_t*)pbuf;
            /* read the FFTPEAKCNT register */
            *peakCnt = (uint16_t)CSL_FEXTR(ctrlBaseAddr->CFAR_PEAKCNT, CFAR_PEAKCNT_CFAR_PEAKCNT_END, CFAR_PEAKCNT_CFAR_PEAKCNT_START);
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to read the number of samples that exceeded the threshold in a chirp
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  numInterfSamplesChirp       number of samples that exceeded the interference threshold in a chirp
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
*/
extern int32_t HWA_readInterfChirpCountReg(HWA_Handle handle, uint16_t *numInterfSamplesChirp)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
         /* read the INTF_LOC_COUNT_ALL_CHIRP register, 12 bits */
        *numInterfSamplesChirp = (uint16_t)CSL_FEXTR(ctrlBaseAddr->INTF_LOC_COUNT_ALL_CHIRP,
                                           INTF_LOC_COUNT_ALL_CHIRP_INTF_LOC_COUNT_ALL_CHIRP_END,
                                           INTF_LOC_COUNT_ALL_CHIRP_INTF_LOC_COUNT_ALL_CHIRP_START);
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to read the number of samples that exceeded the threshold in a frame
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  numInterfSamplesFrame    number of samples exceeded the interference threshold in a frame
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
*/
extern int32_t HWA_readInterfFrameCountReg(HWA_Handle handle, uint32_t *numInterfSamplesFrame)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

                                   /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* read the LOC_COUNT_ALL_FRAME register, 20 bits */
        *numInterfSamplesFrame = (uint32_t)CSL_FEXTR(ctrlBaseAddr->INTF_LOC_COUNT_ALL_FRAME,
                                             INTF_LOC_COUNT_ALL_FRAME_INTF_LOC_COUNT_ALL_FRAME_END,
                                             INTF_LOC_COUNT_ALL_FRAME_INTF_LOC_COUNT_ALL_FRAME_START);
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}


/*!
 *  @brief  Function to read the debug registers (paramcurr, loopcou, acc_trig_in_stat)
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @param  pStats          pointer to a memory of type HWA_debugStats where value of the
 *                          RDSTATUS and HWACCREG12 Registers would be copied
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_readDebugReg(HWA_Handle handle, HWA_DebugStats *pStats)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
#ifdef HWA_PARAM_CHECK
        if (pStats == NULL)
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read the Debug registers*/
            /* read paramaddr register*/
            pStats->currentParamSet = (uint8_t)CSL_FEXTR(ctrlBaseAddr->PARAMADDR, PARAMADDR_PARAMADDR_END, PARAMADDR_PARAMADDR_START);
            /* read PARAMADDR_CPUINTR0*/
            pStats->paramSetIdxCpuIntr0 = (uint8_t)CSL_FEXTR(ctrlBaseAddr->PARAMADDR_CPUINTR[0], PARAMADDR_CPUINTR0_PARAMADDR_END, PARAMADDR_CPUINTR0_PARAMADDR_START);
            /* read PARAMADDR_CPUINTR1*/
            pStats->paramSetIdxCpuIntr1 = (uint8_t)CSL_FEXTR(ctrlBaseAddr->PARAMADDR_CPUINTR[1], PARAMADDR_CPUINTR1_PARAMADDR_END, PARAMADDR_CPUINTR1_PARAMADDR_START);

            /* read FSM_STATE*/
            pStats->fsmStateInfo = (uint8_t)CSL_FEXTR(ctrlBaseAddr->FSM_STATE, FSM_STATE_FSM_STATE_END, FSM_STATE_FSM_STATE_START);

            /* read LOOPCNT register*/
            pStats->currentLoopCount = (uint16_t)CSL_FEXTR(ctrlBaseAddr->LOOP_CNT, LOOP_CNT_LOOP_CNT_END, LOOP_CNT_LOOP_CNT_START);
            pStats->otherThreadLoopCount = (uint16_t)CSL_FEXTR(ctrlBaseAddr->LOOP_CNT, LOOP_CNT_LOOP_CNT_ALT_END, LOOP_CNT_LOOP_CNT_ALT_START);
            /* trigger status*/
            pStats->trigStatus[0] = ctrlBaseAddr->TRIGGER_SET_STATUS[0];
            pStats->trigStatus[1] = ctrlBaseAddr->TRIGGER_SET_STATUS[1];
        }
        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}

/*!
 *  @brief  Function to clear the debug registers (acc_trig_in_clr)
 *
 *  @pre    HWA_open() has been called and HWA is not executing paramsets.
 *
 *  @param  handle          A HWA_Handle returned from HWA_open()
 *
 *  @return 0 upon success. error code if an error occurs.
 *
 *  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
 */
int32_t HWA_clearDebugReg(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs   *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    if (retCode==0)
    {
         ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
        /* clear the Debug Reg */
        /* 2 trigger_IN_CLR register, */
        CSL_FINSR(ctrlBaseAddr->TRIGGER_SET_IN_CLR[0],
                  TRIGGER_SET_IN_CLR_0_TRIGGER_SET_IN_CLR_0_END,
                  TRIGGER_SET_IN_CLR_0_TRIGGER_SET_IN_CLR_0_START,
                  1U);
        CSL_FINSR(ctrlBaseAddr->TRIGGER_SET_IN_CLR[1],
                  TRIGGER_SET_IN_CLR_1_TRIGGER_SET_IN_CLR_1_END,
                  TRIGGER_SET_IN_CLR_1_TRIGGER_SET_IN_CLR_1_START,
                  1U);

        HWA_releaseDriverAccess(ptrHWADriver,(bool) true,(bool) false,0);
    }

    /* return */
    return retCode;
}


/*!
*  @brief  Function to read the interference statistics INTF_LOC_THRESH_MAG_VAL or INTF_LOC_THRESH_MAG_VAL registers
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  pbuf            pointer to a memory where the register values are copied into
*
*  @param  startIdx        The first channel index needs to be read, must be less than 11.
*
*  @param  size            The number of channels needed to be read.
*                          It should be less than 12.
*  @param  type            see \ref HWA_INTERFERENCE_THRESHOLD_TYPE
*
*  @return 0 upon success. error code if an error occurs.
*
*  @sa     HWA_open(), HWA_Attrs::isConcurrentAccessAllowed
*/
extern int32_t HWA_readInterfThreshReg(HWA_Handle handle, uint32_t *pbuf, uint8_t startIdx, uint8_t size, uint8_t type)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs    *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */

    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    if (retCode == 0)
    {
#ifdef HWA_PARAM_CHECK
        if ((pbuf == NULL) || ((startIdx + size) > 12) || (size == 0) || (type > HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF) )
        {
            /* invalid config */
            retCode = HWA_EINVAL;
        }
        else
#endif
        {
            uint8_t i = 0;
            ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;
            /* read interference mag registers*/
            if (type == HWA_INTERFERENCE_THRESHOLD_TYPE_MAG)
            {
                for (i = 0; i < size; i++)
                {
                    pbuf[i] = (uint32_t)CSL_FEXTR(ctrlBaseAddr->INTF_LOC_THRESH_MAG_VAL[startIdx + i],
                                                  INTF_LOC_THRESH_MAG0_VAL_INTF_LOC_THRESH_MAG0_VAL_END,
                                                  INTF_LOC_THRESH_MAG0_VAL_INTF_LOC_THRESH_MAG0_VAL_START);
                }
            }
            else //HWA_INTERFERENCE_THRESHOLD_TYPE_MAGDIFF
            {
                for (i = 0; i < size; i++)
                {
                    pbuf[i] = (uint32_t)CSL_FEXTR(ctrlBaseAddr->INTF_LOC_THRESH_MAGDIFF_VAL[startIdx + i],
                                                  INTF_LOC_THRESH_MAGDIFF0_VAL_INTF_LOC_THRESH_MAGDIFF0_VAL_END,
                                                  INTF_LOC_THRESH_MAGDIFF0_VAL_INTF_LOC_THRESH_MAGDIFF0_VAL_START);
                }
            }
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) true, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to control the suspend mode of the peripheral when the controlling processor (where this driver is instantiated) is halted
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  flagEnDis       0- HWA is not suspended and continues to run in the background even when controlling processor is halted
*                          1- HWA is suspended/halted when controlling processor is halted
*  @return 0 upon success. error code if an error occurs.
*
*/
extern int32_t  HWA_controlPeripheralSuspendMode(HWA_Handle handle, uint8_t flagEnDis)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    CSL_dss_ctrlRegs        *dssCtrlRegsPtr;
    //uint32_t                *dbgAckCtl1Ptr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */
    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) false, 0);
    if (retCode == 0)
    {
       // dbgAckCtl1Ptr = (uint32_t *) (ptrHWADriver->hwAttrs->dssBaseAddr + 0x58C);
        dssCtrlRegsPtr = (CSL_dss_ctrlRegs *) ptrHWADriver->hwAttrs->dssBaseAddr;
        /* set the DBG_ACK_CTL1_DSS_HWA, bit 28-30  bits to 0x1 */
        if (flagEnDis == 1U)
        {
            //*dbgAckCtl1Ptr |= 0x10000000;
            CSL_FINSR(dssCtrlRegsPtr->DBG_ACK_CTL1, 30U,28U, 1U);
        }
        else
        {
            //*dbgAckCtl1Ptr &= 0x8FFFFFFF;
            CSL_FINSR(dssCtrlRegsPtr->DBG_ACK_CTL1, 30U,28U, 0U);
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to enable/disable single-stepping approach which pauses the HWA execution after each param-set.
*
*  @pre    HWA_open() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @param  flagEnDis       0- Disables single stepping approach
*                          1- Enabled single stepping approach
*  @return 0 upon success. error code if an error occurs.
*
*/
extern int32_t  HWA_configureSingleStep(HWA_Handle handle, uint8_t flagEnDis)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */
    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        if (flagEnDis == 1U)
        {
            CSL_FINSR(ctrlBaseAddr->SINGLE_STEP_EN, SINGLE_STEP_EN_SINGLE_STEP_EN_END,SINGLE_STEP_EN_SINGLE_STEP_EN_START, 1U);
        }
        else
        {
            CSL_FINSR(ctrlBaseAddr->SINGLE_STEP_EN, SINGLE_STEP_EN_SINGLE_STEP_EN_END,SINGLE_STEP_EN_SINGLE_STEP_EN_START, 0U);
        }
        HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) false, 0);
    }

    /* return */
    return retCode;
}

/*!
*  @brief  Function to Trigger single-step. This triggers state machine to execute one parameter-set at a time and
*          wait for the next single step trigger.
*
*  @pre    HWA_open(), HWA_configureSingleStep() has been called and HWA is not executing paramsets.
*
*  @param  handle          A HWA_Handle returned from HWA_open()
*
*  @return 0 upon success. error code if an error occurs.
*
*/
extern int32_t  HWA_triggerSingleStep(HWA_Handle handle)
{
    HWA_Object              *ptrHWADriver = NULL;
    int32_t                  retCode = 0;
    DSSHWACCRegs            *ctrlBaseAddr;

    HWA_GET_DRIVER_STRUCT(handle); /* this fills the ptrHWADriver */
    /* validate driver acccess */
    retCode = HWA_getDriverAccess(ptrHWADriver, (bool) false, (bool) false, 0);
    if (retCode == 0)
    {
        ctrlBaseAddr = (DSSHWACCRegs *)ptrHWADriver->hwAttrs->ctrlBaseAddr;

        CSL_FINSR(ctrlBaseAddr->SINGLE_STEP_TRIG, SINGLE_STEP_TRIG_SINGLE_STEP_TRIG_END, SINGLE_STEP_TRIG_SINGLE_STEP_TRIG_START, 1U);

        HWA_releaseDriverAccess(ptrHWADriver, (bool) false, (bool) false, 0);
    }

    /* return */
    return retCode;
}

