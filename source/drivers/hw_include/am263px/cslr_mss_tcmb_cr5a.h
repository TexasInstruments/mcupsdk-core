/********************************************************************
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
 *  Name        : cslr_mss_tcmb_cr5a.h
*/
#ifndef CSLR_MSS_TCMB_CR5A_H_
#define CSLR_MSS_TCMB_CR5A_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t START;
    volatile uint8_t  Resv_65532[65528];
    volatile uint32_t END;
} CSL_mss_tcmb_cr5aRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_TCMB_CR5A_START                                                (0x00000000U)
#define CSL_MSS_TCMB_CR5A_END                                                  (0x0000FFFCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* START */

#define CSL_MSS_TCMB_CR5A_START_START_MASK                                     (0xFFFFFFFFU)
#define CSL_MSS_TCMB_CR5A_START_START_SHIFT                                    (0x00000000U)
#define CSL_MSS_TCMB_CR5A_START_START_RESETVAL                                 (0x00000000U)
#define CSL_MSS_TCMB_CR5A_START_START_MAX                                      (0xFFFFFFFFU)

#define CSL_MSS_TCMB_CR5A_START_RESETVAL                                       (0x00000000U)

/* END */

#define CSL_MSS_TCMB_CR5A_END_END_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_TCMB_CR5A_END_END_SHIFT                                        (0x00000000U)
#define CSL_MSS_TCMB_CR5A_END_END_RESETVAL                                     (0x00000000U)
#define CSL_MSS_TCMB_CR5A_END_END_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_TCMB_CR5A_END_RESETVAL                                         (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
