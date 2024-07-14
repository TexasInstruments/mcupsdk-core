/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CSLR_VMAP_H_
#define CSLR_VMAP_H_

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
    volatile uint32_t CTRL;                      /* Control register */
    volatile uint32_t REQID;                     /* Requester ID mask and value register */
    volatile uint32_t VIRTID;                    /* Virt ID  register */
} CSL_vmapRegs_trans;


typedef struct {
    CSL_vmapRegs_trans TRANS[32];
} CSL_vmapRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_VMAP_TRANS_CTRL(TRANS)                                             (0x00000000U+((TRANS)*0xCU))
#define CSL_VMAP_TRANS_REQID(TRANS)                                            (0x00000004U+((TRANS)*0xCU))
#define CSL_VMAP_TRANS_VIRTID(TRANS)                                           (0x00000008U+((TRANS)*0xCU))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CTRL */

#define CSL_VMAP_TRANS_CTRL_EN_MASK                                            (0x00000001U)
#define CSL_VMAP_TRANS_CTRL_EN_SHIFT                                           (0x00000000U)
#define CSL_VMAP_TRANS_CTRL_EN_MAX                                             (0x00000001U)

/* REQID */

#define CSL_VMAP_TRANS_REQID_MASK_MASK                                         (0xFFFF0000U)
#define CSL_VMAP_TRANS_REQID_MASK_SHIFT                                        (0x00000010U)
#define CSL_VMAP_TRANS_REQID_MASK_MAX                                          (0x0000FFFFU)

#define CSL_VMAP_TRANS_REQID_RID_MASK                                          (0x0000FFFFU)
#define CSL_VMAP_TRANS_REQID_RID_SHIFT                                         (0x00000000U)
#define CSL_VMAP_TRANS_REQID_RID_MAX                                           (0x0000FFFFU)

/* VIRTID */

#define CSL_VMAP_TRANS_VIRTID_VID_MASK                                         (0x00000FFFU)
#define CSL_VMAP_TRANS_VIRTID_VID_SHIFT                                        (0x00000000U)
#define CSL_VMAP_TRANS_VIRTID_VID_MAX                                          (0x00000FFFU)

#ifdef __cplusplus
}
#endif
#endif
