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
 *  Name        : cslr_icssm_intr_xbar.h
*/
#ifndef CSLR_ICSSM_INTR_XBAR_H_
#define CSLR_ICSSM_INTR_XBAR_H_

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
    XBAR INPUT Macros
**************************************************************************/

#define ICSS_XBAR_LIN0_INTR_REQ0	0
#define ICSS_XBAR_LIN0_INTR_REQ1	1
#define ICSS_XBAR_LIN1_INTR_REQ0	2
#define ICSS_XBAR_LIN1_INTR_REQ1	3
#define ICSS_XBAR_LIN2_INTR_REQ0	4
#define ICSS_XBAR_LIN2_INTR_REQ1	5
#define ICSS_XBAR_LIN3_INTR_REQ0	6
#define ICSS_XBAR_LIN3_INTR_REQ1	7
#define ICSS_XBAR_LIN4_INTR_REQ0	8
#define ICSS_XBAR_LIN4_INTR_REQ1	9
#define ICSS_XBAR_USART0_IRQ	10
#define ICSS_XBAR_USART1_IRQ	11
#define ICSS_XBAR_USART2_IRQ	12
#define ICSS_XBAR_USART3_IRQ	13
#define ICSS_XBAR_USART4_IRQ	14
#define ICSS_XBAR_USART5_IRQ	15
#define ICSS_XBAR_I2C0_IRQ	16
#define ICSS_XBAR_I2C1_IRQ	17
#define ICSS_XBAR_I2C2_IRQ	18
#define ICSS_XBAR_I2C3_IRQ	19
#define ICSS_XBAR_SPI0_IRQ	20
#define ICSS_XBAR_SPI1_IRQ	21
#define ICSS_XBAR_SPI2_IRQ	22
#define ICSS_XBAR_SPI3_IRQ	23
#define ICSS_XBAR_SPI4_IRQ	24
#define ICSS_XBAR_QSPI_INTR	25
#define ICSS_XBAR_TPCC_INTG	26
#define ICSS_XBAR_TPCC_INT0	27
#define ICSS_XBAR_TPCC_INT1	28
#define ICSS_XBAR_TPCC_INT2	29
#define ICSS_XBAR_TPCC_INT3	30
#define ICSS_XBAR_TPCC_INT4	31
#define ICSS_XBAR_TPCC_INT5	32
#define ICSS_XBAR_TPCC_INT6	33
#define ICSS_XBAR_TPCC_INT7	34
#define ICSS_XBAR_TPCC_ERRINT	35
#define ICSS_XBAR_TPCC_MPINT	36
#define ICSS_XBAR_TPCC_ERINT_0	37
#define ICSS_XBAR_TPCC_ERINT_1	38
#define ICSS_XBAR_MCANSS0_EXT_TS_ROLLOVER_LVL_INT	39
#define ICSS_XBAR_MCANSS0_MCAN_LVL_INT_0	40
#define ICSS_XBAR_MCANSS0_MCAN_LVL_INT_1	41
#define ICSS_XBAR_MCANSS1_EXT_TS_ROLLOVER_LVL_INT	42
#define ICSS_XBAR_MCANSS1_MCAN_LVL_INT_0	43
#define ICSS_XBAR_MCANSS1_MCAN_LVL_INT_1	44
#define ICSS_XBAR_MCANSS2_EXT_TS_ROLLOVER_LVL_INT	45
#define ICSS_XBAR_MCANSS2_MCAN_LVL_INT_0	46
#define ICSS_XBAR_MCANSS2_MCAN_LVL_INT_1	47
#define ICSS_XBAR_MCANSS3_EXT_TS_ROLLOVER_LVL_INT	48
#define ICSS_XBAR_MCANSS3_MCAN_LVL_INT_0	49
#define ICSS_XBAR_MCANSS3_MCAN_LVL_INT_1	50
#define ICSS_XBAR_MAILBOX_PRU_REQ_0	51
#define ICSS_XBAR_MAILBOX_PRU_REQ_1	52
#define ICSS_XBAR_MAILBOX_PRU_ACK_0	53
#define ICSS_XBAR_MAILBOX_PRU_ACK_1	54
#define ICSS_XBAR_GPIO_INT_XBAR_OUT_0	55
#define ICSS_XBAR_GPIO_INT_XBAR_OUT_1	56
#define ICSS_XBAR_GPIO_INT_XBAR_OUT_2	57
#define ICSS_XBAR_GPIO_INT_XBAR_OUT_3	58

/**************************************************************************
    XBAR OUTPUT Macros
**************************************************************************/

#define ICSS_XBAR_ICSS_MODULE_0         0
#define ICSS_XBAR_ICSS_MODULE_1         1
#define ICSS_XBAR_ICSS_MODULE_2         2
#define ICSS_XBAR_ICSS_MODULE_3         3
#define ICSS_XBAR_ICSS_MODULE_4         4
#define ICSS_XBAR_ICSS_MODULE_5         5
#define ICSS_XBAR_ICSS_MODULE_6         6
#define ICSS_XBAR_ICSS_MODULE_7         7
#define ICSS_XBAR_ICSS_MODULE_8         8
#define ICSS_XBAR_ICSS_MODULE_9         9
#define ICSS_XBAR_ICSS_MODULE_10        10
#define ICSS_XBAR_ICSS_MODULE_11        11
#define ICSS_XBAR_ICSS_MODULE_12        12
#define ICSS_XBAR_ICSS_MODULE_13        13
#define ICSS_XBAR_ICSS_MODULE_14        14
#define ICSS_XBAR_ICSS_MODULE_15        15

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t MUXCNTL[16];
} CSL_icssm_intr_xbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_ICSSM_INTR_XBAR_PID                                                (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL(MUXCNTL)                                   (0x00000004U+((MUXCNTL)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_MASK                                    (0xC0000000U)
#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_SHIFT                                   (0x0000001EU)
#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_RESETVAL                                (0x00000001U)
#define CSL_ICSSM_INTR_XBAR_PID_SCHEME_MAX                                     (0x00000003U)

#define CSL_ICSSM_INTR_XBAR_PID_BU_MASK                                        (0x30000000U)
#define CSL_ICSSM_INTR_XBAR_PID_BU_SHIFT                                       (0x0000001CU)
#define CSL_ICSSM_INTR_XBAR_PID_BU_RESETVAL                                    (0x00000002U)
#define CSL_ICSSM_INTR_XBAR_PID_BU_MAX                                         (0x00000003U)

#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_MASK                                  (0x0FFF0000U)
#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_SHIFT                                 (0x00000010U)
#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_RESETVAL                              (0x00000694U)
#define CSL_ICSSM_INTR_XBAR_PID_FUNCTION_MAX                                   (0x00000FFFU)

#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_MASK                                    (0x0000F800U)
#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_SHIFT                                   (0x0000000BU)
#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_RESETVAL                                (0x00000010U)
#define CSL_ICSSM_INTR_XBAR_PID_RTLVER_MAX                                     (0x0000001FU)

#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_MASK                                    (0x00000700U)
#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_SHIFT                                   (0x00000008U)
#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_RESETVAL                                (0x00000001U)
#define CSL_ICSSM_INTR_XBAR_PID_MAJREV_MAX                                     (0x00000007U)

#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_MASK                                    (0x000000C0U)
#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_SHIFT                                   (0x00000006U)
#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_RESETVAL                                (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_PID_CUSTOM_MAX                                     (0x00000003U)

#define CSL_ICSSM_INTR_XBAR_PID_MINREV_MASK                                    (0x0000003FU)
#define CSL_ICSSM_INTR_XBAR_PID_MINREV_SHIFT                                   (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_PID_MINREV_RESETVAL                                (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_PID_MINREV_MAX                                     (0x0000003FU)

#define CSL_ICSSM_INTR_XBAR_PID_RESETVAL                                       (0x66948100U)

/* MUXCNTL */

#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_MASK                            (0x00010000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_SHIFT                           (0x00000010U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_RESETVAL                        (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_MAX                             (0x00000001U)

#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_MASK                                (0x0000003FU)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_SHIFT                               (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_RESETVAL                            (0x00000000U)
#define CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_MAX                                 (0x0000003FU)

#define CSL_ICSSM_INTR_XBAR_MUXCNTL_RESETVAL                                   (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
