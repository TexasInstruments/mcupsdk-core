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
#ifndef CSLR_MAIN2MCU_PLS_INTRTR0_INTERRUPT_MAP_H_
#define CSLR_MAIN2MCU_PLS_INTRTR0_INTERRUPT_MAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: MAIN2MCU_PLS_INTRTR0
*/

#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM0_EPWM_ETINT_0                                          (1U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM1_EPWM_ETINT_0                                          (2U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM2_EPWM_ETINT_0                                          (3U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM3_EPWM_ETINT_0                                          (4U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM4_EPWM_ETINT_0                                          (5U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM5_EPWM_ETINT_0                                          (6U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM0_EPWM_TRIPZINT_0                                       (7U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM1_EPWM_TRIPZINT_0                                       (8U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM2_EPWM_TRIPZINT_0                                       (9U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM3_EPWM_TRIPZINT_0                                       (10U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM4_EPWM_TRIPZINT_0                                       (11U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EHRPWM5_EPWM_TRIPZINT_0                                       (12U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EQEP0_EQEP_INT_0                                              (13U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EQEP1_EQEP_INT_0                                              (14U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_EQEP2_EQEP_INT_0                                              (15U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_ECAP0_ECAP_INT_0                                              (16U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_GLUELOGIC_SOCA_INT_GLUE_SOCA_INT_0                            (17U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_GLUELOGIC_SOCB_INT_GLUE_SOCB_INT_0                            (18U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG0_PR1_RX_SOF_INTR_REQ_0                              (19U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG0_PR1_RX_SOF_INTR_REQ_1                              (20U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG0_PR1_TX_SOF_INTR_REQ_0                              (21U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG0_PR1_TX_SOF_INTR_REQ_1                              (22U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG1_PR1_RX_SOF_INTR_REQ_0                              (23U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG1_PR1_RX_SOF_INTR_REQ_1                              (24U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG1_PR1_TX_SOF_INTR_REQ_0                              (25U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG1_PR1_TX_SOF_INTR_REQ_1                              (26U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG2_PR1_RX_SOF_INTR_REQ_0                              (27U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG2_PR1_RX_SOF_INTR_REQ_1                              (28U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG2_PR1_TX_SOF_INTR_REQ_0                              (29U)
#define CSLR_MAIN2MCU_PLS_INTRTR0_IN_PRU_ICSSG2_PR1_TX_SOF_INTR_REQ_1                              (30U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_MAIN2MCU_PLS_INTRTR0_INTERRUPT_MAP_H_ */

