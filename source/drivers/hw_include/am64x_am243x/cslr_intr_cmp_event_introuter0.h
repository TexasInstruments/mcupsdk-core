/********************************************************************
*
* CMP_EVENT_INTROUTER0 INTERRUPT MAP. header file
*
* Copyright (C) 2015-2019 Texas Instruments Incorporated.
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
#ifndef CSLR_CMP_EVENT_INTROUTER0_INTERRUPT_MAP_H_
#define CSLR_CMP_EVENT_INTROUTER0_INTERRUPT_MAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: CMP_EVENT_INTROUTER0
*/

#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_0                                (0U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_1                                (1U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_2                                (2U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_3                                (3U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_4                                (4U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_5                                (5U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_6                                (6U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_HOST_INTR_REQ_7                                (7U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_0                                (8U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_1                                (9U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_2                                (10U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_3                                (11U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_4                                (12U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_5                                (13U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_6                                (14U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_HOST_INTR_REQ_7                                (15U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_0                            (16U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_1                            (17U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_2                            (18U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_3                            (19U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_4                            (20U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_5                            (21U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_6                            (22U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_7                            (23U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_8                            (24U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_9                            (25U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_10                           (26U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_11                           (27U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_12                           (28U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_13                           (29U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_14                           (30U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP0_CMP_INTR_REQ_15                           (31U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_0                            (32U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_1                            (33U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_2                            (34U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_3                            (35U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_4                            (36U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_5                            (37U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_6                            (38U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_7                            (39U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_8                            (40U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_9                            (41U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_10                           (42U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_11                           (43U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_12                           (44U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_13                           (45U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_14                           (46U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_15                           (47U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_0                            (48U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_1                            (49U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_2                            (50U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_3                            (51U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_4                            (52U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_5                            (53U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_6                            (54U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_7                            (55U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_8                            (56U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_9                            (57U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_10                           (58U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_11                           (59U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_12                           (60U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_13                           (61U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_14                           (62U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP0_CMP_INTR_REQ_15                           (63U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_0                            (64U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_1                            (65U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_2                            (66U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_3                            (67U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_4                            (68U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_5                            (69U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_6                            (70U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_7                            (71U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_8                            (72U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_9                            (73U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_10                           (74U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_11                           (75U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_12                           (76U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_13                           (77U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_14                           (78U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG1_PR1_IEP1_CMP_INTR_REQ_15                           (79U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_CPSW0_CPTS_COMP_0                                             (80U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_PCIE0_PCIE_CPTS_COMP_0                                        (81U)
#define CSLR_CMP_EVENT_INTROUTER0_IN_CPTS0_CPTS_COMP_0                                             (82U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_CMP_EVENT_INTROUTER0_INTERRUPT_MAP_H_ */

