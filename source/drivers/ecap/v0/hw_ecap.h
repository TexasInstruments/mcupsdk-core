/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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

#ifndef HW_ECAP_H_
#define HW_ECAP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ECAP_TSCTR  ((uint32_t) 0x0U)
#define ECAP_CTRPHS ((uint32_t) 0x4U)
#define ECAP_CAP1   ((uint32_t) 0x8U)
#define ECAP_CAP2   ((uint32_t) 0xCU)
#define ECAP_CAP3   ((uint32_t) 0x10U)
#define ECAP_CAP4   ((uint32_t) 0x14U)
#define ECAP_ECCTL1 ((uint32_t) 0x28U)
#define ECAP_ECCTL2 ((uint32_t) 0x2AU)
#define ECAP_ECEINT ((uint32_t) 0x2CU)
#define ECAP_ECFLG  ((uint32_t) 0x2EU)
#define ECAP_ECCLR  ((uint32_t) 0x30U)
#define ECAP_ECFRC  ((uint32_t) 0x32U)
#define ECAP_REVID  ((uint32_t) 0x5CU)

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* TSCTR */

#define ECAP_TSCTR_TSCTR ((uint32_t)(0xFFFFFFFFu))
#define ECAP_TSCTR_TSCTR_SHIFT ((uint32_t)(0x00000000u))


/* CTRPHS */

#define ECAP_CTRPHS_CTRPHS ((uint32_t)(0xFFFFFFFFu))
#define ECAP_CTRPHS_CTRPHS_SHIFT ((uint32_t)(0x00000000u))


/* CAP1 */

#define ECAP_CAP1_CAP1 ((uint32_t)(0xFFFFFFFFu))
#define ECAP_CAP1_CAP1_SHIFT ((uint32_t)(0x00000000u))


/* CAP2 */

#define ECAP_CAP2_CAP2 ((uint32_t)(0xFFFFFFFFu))
#define ECAP_CAP2_CAP2_SHIFT ((uint32_t)(0x00000000u))


/* CAP3 */

#define ECAP_CAP3_CAP3 ((uint32_t)(0xFFFFFFFFu))
#define ECAP_CAP3_CAP3_SHIFT ((uint32_t)(0x00000000u))


/* CAP4 */

#define ECAP_CAP4_CAP4 ((uint32_t)(0xFFFFFFFFu))
#define ECAP_CAP4_CAP4_SHIFT ((uint32_t)(0x00000000u))


/* ECCTL1 */

#define ECAP_ECCTL1_FREE_SOFT ((uint32_t)(0xC000u))
#define ECAP_ECCTL1_FREE_SOFT_SHIFT ((uint32_t)(0x000Eu))
#define ECAP_ECCTL1_FREE_SOFT_STOP ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_FREE_SOFT_RUNUNTIL0 ((uint32_t)(0x0001u))

#define ECAP_ECCTL1_PRESCALE_MASK                           (0x00003E00U)
#define ECAP_ECCTL1_PRESCALE_SHIFT                          (9U)
#define ECAP_ECCTL1_PRESCALE_RESETVAL                       (0x00000000U)
#define ECAP_ECCTL1_PRESCALE_MAX                            (0x0000001fU)

#define ECAP_ECCTL1_CAPLDEN ((uint32_t)(0x0100u))
#define ECAP_ECCTL1_CAPLDEN_SHIFT ((uint32_t)(0x0008u))

#define ECAP_ECCTL1_CTRRST4_SHIFT                   ((uint32_t)(0x0007u))
#define ECAP_ECCTL1_CTRRST4_MASK                    ((uint32_t)(0x0080u))
#define ECAP_ECCTL1_CTRRST4_NORESET                 ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CTRRST4_RESET                   ((uint32_t)(0x0001u))

#define ECAP_ECCTL1_CAP4POL_SHIFT                   ((uint32_t)(0x0006u))
#define ECAP_ECCTL1_CAP4POL_MASK                    ((uint32_t)(0x0040u))
#define ECAP_ECCTL1_CAP4POL_FE                      ((uint32_t)(0x0001u))
#define ECAP_ECCTL1_CAP4POL_RE                      ((uint32_t)(0x0000u))

#define ECAP_ECCTL1_CTRRST3_SHIFT                   ((uint32_t)(0x0005u))
#define ECAP_ECCTL1_CTRRST3_MASK                    ((uint32_t)(0x0020u))
#define ECAP_ECCTL1_CTRRST3_NO_RESET                ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CTRRST3_RESET                   ((uint32_t)(0x0001u))

#define ECAP_ECCTL1_CAP3POL_SHIFT                   ((uint32_t)(0x0004u))
#define ECAP_ECCTL1_CAP3POL_MASK                    ((uint32_t)(0x0010u))
#define ECAP_ECCTL1_CAP3POL_FE                      ((uint32_t)(0x0001u))
#define ECAP_ECCTL1_CAP3POL_RE                      ((uint32_t)(0x0000u))

#define ECAP_ECCTL1_CTRRST2_SHIFT                   ((uint32_t)(0x0003u))
#define ECAP_ECCTL1_CTRRST2_MASK                    ((uint32_t)(0x0008u))
#define ECAP_ECCTL1_CTRRST2_NO_RESET                ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CTRRST2_RESET                   ((uint32_t)(0x0001u))

#define ECAP_ECCTL1_CAP2POL_SHIFT                   ((uint32_t)(0x0002u))
#define ECAP_ECCTL1_CAP2POL_MASK                    ((uint32_t)(0x0004u))
#define ECAP_ECCTL1_CAP2POL_RE                      ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CAP2POL_FE                      ((uint32_t)(0x0001u))

#define ECAP_ECCTL1_CTRRST1_SHIFT                   ((uint32_t)(0x0001u))
#define ECAP_ECCTL1_CTRRST1_MASK                    ((uint32_t)(0x0002u))
#define ECAP_ECCTL1_CTRRST1_NO_RESET                ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CTRRST1_RESET                   ((uint32_t)(0x0001u))

#define ECAP_ECCTL1_CAP1POL_SHIFT                   ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CAP1POL_MASK                    ((uint32_t)(0x0001u))
#define ECAP_ECCTL1_CAP1POL_RE                      ((uint32_t)(0x0000u))
#define ECAP_ECCTL1_CAP1POL_FE                      ((uint32_t)(0x0001u))

/* ECCTL2 */

#define ECAP_ECCTL2_APWMPOL_MASK ((uint32_t)(0x0400u))
#define ECAP_ECCTL2_APWMPOL_SHIFT ((uint32_t)(0x000Au))

#define ECAP_ECCTL2_CAP_APWM_SHIFT ((uint32_t)(0x0009u))
#define ECAP_ECCTL2_CAP_APWM_MASK  ((uint32_t)(0x0200u))

#define ECAP_ECCTL2_SWSYNC_MASK ((uint32_t)(0x0100u))
#define ECAP_ECCTL2_SWSYNC_SHIFT ((uint32_t)(0x0008u))

#define ECAP_ECCTL2_SYNCO_SEL_MASK ((uint32_t)(0x00C0u))
#define ECAP_ECCTL2_SYNCO_SEL_SHIFT ((uint32_t)(0x0006u))

#define ECAP_ECCTL2_SYNCI_EN_MASK ((uint32_t)(0x0020u))
#define ECAP_ECCTL2_SYNCI_EN_SHIFT ((uint32_t)(0x0005u))

#define ECAP_ECCTL2_TSCTRSTOP_MASK ((uint32_t)(0x0010u))
#define ECAP_ECCTL2_TSCTRSTOP_SHIFT ((uint32_t)(0x0004u))

#define ECAP_ECCTL2_RE_ARM ((uint32_t)(0x0008u))
#define ECAP_ECCTL2_RE_ARM_SHIFT ((uint32_t)(0x0003u))

#define ECAP_ECCTL2_STOP_WRAP_MASK ((uint32_t)(0x0006u))
#define ECAP_ECCTL2_STOP_WRAP_SHIFT ((uint32_t)(0x0001u))
#define ECAP_ECCTL2_STOP_WRAP_CAP1 ((uint32_t)(0x0000u))
#define ECAP_ECCTL2_STOP_WRAP_CAP2 ((uint32_t)(0x0001u))
#define ECAP_ECCTL2_STOP_WRAP_CAP3 ((uint32_t)(0x0002u))
#define ECAP_ECCTL2_STOP_WRAP_CAP4 ((uint32_t)(0x0003u))

#define ECAP_ECCTL2_CONT_ONESHT ((uint32_t)(0x0001u))
#define ECAP_ECCTL2_CONT_ONESHT_SHIFT ((uint32_t)(0x0000u))


/* ECEINT */


#define ECAP_ECEINT_CTR_CMP ((uint32_t)(0x0080u))
#define ECAP_ECEINT_CTR_CMP_SHIFT ((uint32_t)(0x0007u))

#define ECAP_ECEINT_CTR_PRD ((uint32_t)(0x0040u))
#define ECAP_ECEINT_CTR_PRD_SHIFT ((uint32_t)(0x0006u))

#define ECAP_ECEINT_CTROVF ((uint32_t)(0x0020u))
#define ECAP_ECEINT_CTROVF_SHIFT ((uint32_t)(0x0005u))

#define ECAP_ECEINT_CEVT4 ((uint32_t)(0x0010u))
#define ECAP_ECEINT_CEVT4_SHIFT ((uint32_t)(0x0004u))

#define ECAP_ECEINT_CEVT3 ((uint32_t)(0x0008u))
#define ECAP_ECEINT_CEVT3_SHIFT ((uint32_t)(0x0003u))

#define ECAP_ECEINT_CEVT2 ((uint32_t)(0x0004u))
#define ECAP_ECEINT_CEVT2_SHIFT ((uint32_t)(0x0002u))

#define ECAP_ECEINT_CEVT1 ((uint32_t)(0x0002u))
#define ECAP_ECEINT_CEVT1_SHIFT ((uint32_t)(0x0001u))



/* ECFLG */


#define ECAP_ECFLG_CTR_CMP ((uint32_t)(0x0080u))
#define ECAP_ECFLG_CTR_CMP_SHIFT ((uint32_t)(0x0007u))

#define ECAP_ECFLG_CTR_PRD ((uint32_t)(0x0040u))
#define ECAP_ECFLG_CTR_PRD_SHIFT ((uint32_t)(0x0006u))

#define ECAP_ECFLG_CTROVF ((uint32_t)(0x0020u))
#define ECAP_ECFLG_CTROVF_SHIFT ((uint32_t)(0x0005u))

#define ECAP_ECFLG_CEVT4 ((uint32_t)(0x0010u))
#define ECAP_ECFLG_CEVT4_SHIFT ((uint32_t)(0x0004u))

#define ECAP_ECFLG_CEVT3 ((uint32_t)(0x0008u))
#define ECAP_ECFLG_CEVT3_SHIFT ((uint32_t)(0x0003u))

#define ECAP_ECFLG_CEVT2 ((uint32_t)(0x0004u))
#define ECAP_ECFLG_CEVT2_SHIFT ((uint32_t)(0x0002u))

#define ECAP_ECFLG_CEVT1 ((uint32_t)(0x0002u))
#define ECAP_ECFLG_CEVT1_SHIFT ((uint32_t)(0x0001u))

#define ECAP_ECFLG_INT ((uint32_t)(0x0001u))
#define ECAP_ECFLG_INT_SHIFT ((uint32_t)(0x0000u))


/* ECCLR */


#define ECAP_ECCLR_CTR_CMP ((uint32_t)(0x0080u))
#define ECAP_ECCLR_CTR_CMP_SHIFT ((uint32_t)(0x0007u))

#define ECAP_ECCLR_CTR_PRD ((uint32_t)(0x0040u))
#define ECAP_ECCLR_CTR_PRD_SHIFT ((uint32_t)(0x0006u))

#define ECAP_ECCLR_CTROVF ((uint32_t)(0x0020u))
#define ECAP_ECCLR_CTROVF_SHIFT ((uint32_t)(0x0005u))

#define ECAP_ECCLR_CEVT4 ((uint32_t)(0x0010u))
#define ECAP_ECCLR_CEVT4_SHIFT ((uint32_t)(0x0004u))

#define ECAP_ECCLR_CEVT3 ((uint32_t)(0x0008u))
#define ECAP_ECCLR_CEVT3_SHIFT ((uint32_t)(0x0003u))

#define ECAP_ECCLR_CEVT2 ((uint32_t)(0x0004u))
#define ECAP_ECCLR_CEVT2_SHIFT ((uint32_t)(0x0002u))

#define ECAP_ECCLR_CEVT1 ((uint32_t)(0x0002u))
#define ECAP_ECCLR_CEVT1_SHIFT ((uint32_t)(0x0001u))

#define ECAP_ECCLR_INT ((uint32_t)(0x0001u))
#define ECAP_ECCLR_INT_SHIFT ((uint32_t)(0x0000u))


/* ECFRC */


#define ECAP_ECFRC_CTR_CMP ((uint32_t)(0x0080u))
#define ECAP_ECFRC_CTR_CMP_SHIFT ((uint32_t)(0x0007u))

#define ECAP_ECFRC_CTR_PRD ((uint32_t)(0x0040u))
#define ECAP_ECFRC_CTR_PRD_SHIFT ((uint32_t)(0x0006u))

#define ECAP_ECFRC_CTROVF ((uint32_t)(0x0020u))
#define ECAP_ECFRC_CTROVF_SHIFT ((uint32_t)(0x0005u))

#define ECAP_ECFRC_CEVT4 ((uint32_t)(0x0010u))
#define ECAP_ECFRC_CEVT4_SHIFT ((uint32_t)(0x0004u))

#define ECAP_ECFRC_CEVT3 ((uint32_t)(0x0008u))
#define ECAP_ECFRC_CEVT3_SHIFT ((uint32_t)(0x0003u))

#define ECAP_ECFRC_CEVT2 ((uint32_t)(0x0004u))
#define ECAP_ECFRC_CEVT2_SHIFT ((uint32_t)(0x0002u))

#define ECAP_ECFRC_CEVT1 ((uint32_t)(0x0002u))
#define ECAP_ECFRC_CEVT1_SHIFT ((uint32_t)(0x0001u))



/* REVID */

#define ECAP_REVID_REV          ((uint32_t)(0xFFFFFFFFu))
#define ECAP_REVID_REV_SHIFT         ((uint32_t)(0x00000000u))

#ifdef __cplusplus
}
#endif
#endif
