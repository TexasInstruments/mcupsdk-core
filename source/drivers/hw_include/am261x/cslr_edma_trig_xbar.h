/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_edma_trig_xbar.h
*/
#ifndef CSLR_EDMA_TRIG_XBAR_H_
#define CSLR_EDMA_TRIG_XBAR_H_

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

#define DMA_TRIG_XBAR_LIN0_RXDMA	        0
#define DMA_TRIG_XBAR_LIN0_TXDMA	        1
#define DMA_TRIG_XBAR_LIN1_RXDMA	        2
#define DMA_TRIG_XBAR_LIN1_TXDMA	        3
#define DMA_TRIG_XBAR_LIN2_RXDMA	        4
#define DMA_TRIG_XBAR_LIN2_TXDMA	        5
#define DMA_TRIG_XBAR_I2C0_TX	            6
#define DMA_TRIG_XBAR_I2C0_RX	            7
#define DMA_TRIG_XBAR_I2C1_TX	            8
#define DMA_TRIG_XBAR_I2C1_RX	            9
#define DMA_TRIG_XBAR_I2C2_TX	            10
#define DMA_TRIG_XBAR_I2C2_RX	            11
#define DMA_TRIG_XBAR_SPI0_DMA_READ_REQ0	12
#define DMA_TRIG_XBAR_SPI0_DMA_READ_REQ1	13
#define DMA_TRIG_XBAR_SPI0_DMA_WRITE_REQ0	14
#define DMA_TRIG_XBAR_SPI0_DMA_WRITE_REQ1	15
#define DMA_TRIG_XBAR_SPI1_DMA_READ_REQ0	16
#define DMA_TRIG_XBAR_SPI1_DMA_READ_REQ1	17
#define DMA_TRIG_XBAR_SPI1_DMA_WRITE_REQ0	18
#define DMA_TRIG_XBAR_SPI1_DMA_WRITE_REQ1	19
#define DMA_TRIG_XBAR_SPI2_DMA_READ_REQ0	20
#define DMA_TRIG_XBAR_SPI2_DMA_READ_REQ1	21
#define DMA_TRIG_XBAR_SPI2_DMA_WRITE_REQ0	22
#define DMA_TRIG_XBAR_SPI2_DMA_WRITE_REQ1	23
#define DMA_TRIG_XBAR_SPI3_DMA_READ_REQ0	24
#define DMA_TRIG_XBAR_SPI3_DMA_READ_REQ1	25
#define DMA_TRIG_XBAR_SPI3_DMA_WRITE_REQ0	26
#define DMA_TRIG_XBAR_SPI3_DMA_WRITE_REQ1	27
#define DMA_TRIG_XBAR_RTI0_DMA_0	        28
#define DMA_TRIG_XBAR_RTI0_DMA_1	        29
#define DMA_TRIG_XBAR_RTI0_DMA_2	        30
#define DMA_TRIG_XBAR_RTI0_DMA_3	        31
#define DMA_TRIG_XBAR_RTI1_DMA_0	        32
#define DMA_TRIG_XBAR_RTI1_DMA_1	        33
#define DMA_TRIG_XBAR_RTI1_DMA_2	        34
#define DMA_TRIG_XBAR_RTI1_DMA_3	        35
#define DMA_TRIG_XBAR_RTI2_DMA_0	        36
#define DMA_TRIG_XBAR_RTI2_DMA_1	        37
#define DMA_TRIG_XBAR_RTI2_DMA_2	        38
#define DMA_TRIG_XBAR_RTI2_DMA_3	        39
#define DMA_TRIG_XBAR_RTI3_DMA_0	        40
#define DMA_TRIG_XBAR_RTI3_DMA_1	        41
#define DMA_TRIG_XBAR_RTI3_DMA_2	        42
#define DMA_TRIG_XBAR_RTI3_DMA_3	        43
#define DMA_TRIG_XBAR_MCANSS0_TX_DMA_0	    44
#define DMA_TRIG_XBAR_MCANSS0_TX_DMA_1	    45
#define DMA_TRIG_XBAR_MCANSS0_TX_DMA_2	    46
#define DMA_TRIG_XBAR_MCANSS0_TX_DMA_3	    47
#define DMA_TRIG_XBAR_MCANSS1_TX_DMA_0	    48
#define DMA_TRIG_XBAR_MCANSS1_TX_DMA_1	    49
#define DMA_TRIG_XBAR_MCANSS1_TX_DMA_2	    50
#define DMA_TRIG_XBAR_MCANSS1_TX_DMA_3	    51
#define DMA_TRIG_XBAR_USART0_DMA_0	        52
#define DMA_TRIG_XBAR_USART0_DMA_1	        53
#define DMA_TRIG_XBAR_USART1_DMA_0	        54
#define DMA_TRIG_XBAR_USART1_DMA_1	        55
#define DMA_TRIG_XBAR_USART2_DMA_0	        56
#define DMA_TRIG_XBAR_USART2_DMA_1	        57
#define DMA_TRIG_XBAR_USART3_DMA_0	        58
#define DMA_TRIG_XBAR_USART3_DMA_1	        59
#define DMA_TRIG_XBAR_USART4_DMA_0	        60
#define DMA_TRIG_XBAR_USART4_DMA_1	        61
#define DMA_TRIG_XBAR_USART5_DMA_0	        62
#define DMA_TRIG_XBAR_USART5_DMA_1	        63
#define DMA_TRIG_XBAR_MCRC_DMA_EVENT_0	    64
#define DMA_TRIG_XBAR_MCRC_DMA_EVENT_1	    65
#define DMA_TRIG_XBAR_MCRC_DMA_EVENT_2	    66
#define DMA_TRIG_XBAR_MCRC_DMA_EVENT_3	    67
#define DMA_TRIG_XBAR_OSPI0_INTR	        68
#define DMA_TRIG_XBAR_GPIO_INT_XBAR_OUT_4	69
#define DMA_TRIG_XBAR_GPIO_INT_XBAR_OUT_5	70
#define DMA_TRIG_XBAR_GPIO_INT_XBAR_OUT_6	71
#define DMA_TRIG_XBAR_GPIO_INT_XBAR_OUT_7	72
#define DMA_TRIG_XBAR_SOC_TIMESYNC_XBAR1_OUT_0	73
#define DMA_TRIG_XBAR_SOC_TIMESYNC_XBAR1_OUT_1	74
#define DMA_TRIG_XBAR_SOC_TIMESYNC_XBAR0_OUT_0	75
#define DMA_TRIG_XBAR_SOC_TIMESYNC_XBAR0_OUT_1	76
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_0	77
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_1	78
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_2	79
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_3	80
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_4	81
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_5	82
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_6	83
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_7	84
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_8	85
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_9	86
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_10	87
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_11	88
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_12	89
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_13	90
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_14	91
#define DMA_TRIG_XBAR_DMA_XBAR_OUT_15	92
#define DMA_TRIG_XBAR_MMC_DMA_RD	    93
#define DMA_TRIG_XBAR_MMC_DMA_WR	    94
#define DMA_TRIG_XBAR_DTHE_SHA_DMA_REQ0	95
#define DMA_TRIG_XBAR_DTHE_SHA_DMA_REQ1	96
#define DMA_TRIG_XBAR_DTHE_SHA_DMA_REQ2	97
#define DMA_TRIG_XBAR_DTHE_SHA_DMA_REQ3	98
#define DMA_TRIG_XBAR_DTHE_SHA_DMA_REQ4	99
#define DMA_TRIG_XBAR_DTHE_SHA_DMA_REQ5	100
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ0	101
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ1	102
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ2	103
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ3	104
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ4	105
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ5	106
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ6	107
#define DMA_TRIG_XBAR_DTHE_AES_DMA_REQ7	108
#define DMA_TRIG_XBAR_MCANSS0_FE_0	109
#define DMA_TRIG_XBAR_MCANSS0_FE_1	110
#define DMA_TRIG_XBAR_MCANSS0_FE_2	111
#define DMA_TRIG_XBAR_MCANSS0_FE_3	112
#define DMA_TRIG_XBAR_MCANSS0_FE_4	113
#define DMA_TRIG_XBAR_MCANSS0_FE_5	114
#define DMA_TRIG_XBAR_MCANSS0_FE_6	115
#define DMA_TRIG_XBAR_MCANSS1_FE_0	116
#define DMA_TRIG_XBAR_MCANSS1_FE_1	117
#define DMA_TRIG_XBAR_MCANSS1_FE_2	118
#define DMA_TRIG_XBAR_MCANSS1_FE_3	119
#define DMA_TRIG_XBAR_MCANSS1_FE_4	120
#define DMA_TRIG_XBAR_MCANSS1_FE_5	121
#define DMA_TRIG_XBAR_MCANSS1_FE_6	122
#define DMA_TRIG_XBAR_GPMC_SDMAREQ	123
#define DMA_TRIG_XBAR_OSPI1_INTR	124

/**************************************************************************
    XBAR OUTPUT Macros
**************************************************************************/

#define DMA_TRIG_XBAR_EDMA_MODULE_0    0
#define DMA_TRIG_XBAR_EDMA_MODULE_1    1
#define DMA_TRIG_XBAR_EDMA_MODULE_2    2
#define DMA_TRIG_XBAR_EDMA_MODULE_3    3
#define DMA_TRIG_XBAR_EDMA_MODULE_4    4
#define DMA_TRIG_XBAR_EDMA_MODULE_5    5
#define DMA_TRIG_XBAR_EDMA_MODULE_6    6
#define DMA_TRIG_XBAR_EDMA_MODULE_7    7
#define DMA_TRIG_XBAR_EDMA_MODULE_8    8
#define DMA_TRIG_XBAR_EDMA_MODULE_9    9
#define DMA_TRIG_XBAR_EDMA_MODULE_10    10
#define DMA_TRIG_XBAR_EDMA_MODULE_11    11
#define DMA_TRIG_XBAR_EDMA_MODULE_12    12
#define DMA_TRIG_XBAR_EDMA_MODULE_13    13
#define DMA_TRIG_XBAR_EDMA_MODULE_14    14
#define DMA_TRIG_XBAR_EDMA_MODULE_15    15
#define DMA_TRIG_XBAR_EDMA_MODULE_16    16
#define DMA_TRIG_XBAR_EDMA_MODULE_17    17
#define DMA_TRIG_XBAR_EDMA_MODULE_18    18
#define DMA_TRIG_XBAR_EDMA_MODULE_19    19
#define DMA_TRIG_XBAR_EDMA_MODULE_20    20
#define DMA_TRIG_XBAR_EDMA_MODULE_21    21
#define DMA_TRIG_XBAR_EDMA_MODULE_22    22
#define DMA_TRIG_XBAR_EDMA_MODULE_23    23
#define DMA_TRIG_XBAR_EDMA_MODULE_24    24
#define DMA_TRIG_XBAR_EDMA_MODULE_25    25
#define DMA_TRIG_XBAR_EDMA_MODULE_26    26
#define DMA_TRIG_XBAR_EDMA_MODULE_27    27
#define DMA_TRIG_XBAR_EDMA_MODULE_28    28
#define DMA_TRIG_XBAR_EDMA_MODULE_29    29
#define DMA_TRIG_XBAR_EDMA_MODULE_30    30
#define DMA_TRIG_XBAR_EDMA_MODULE_31    31
#define DMA_TRIG_XBAR_EDMA_MODULE_32    32
#define DMA_TRIG_XBAR_EDMA_MODULE_33    33
#define DMA_TRIG_XBAR_EDMA_MODULE_34    34
#define DMA_TRIG_XBAR_EDMA_MODULE_35    35
#define DMA_TRIG_XBAR_EDMA_MODULE_36    36
#define DMA_TRIG_XBAR_EDMA_MODULE_37    37
#define DMA_TRIG_XBAR_EDMA_MODULE_38    38
#define DMA_TRIG_XBAR_EDMA_MODULE_39    39
#define DMA_TRIG_XBAR_EDMA_MODULE_40    40
#define DMA_TRIG_XBAR_EDMA_MODULE_41    41
#define DMA_TRIG_XBAR_EDMA_MODULE_42    42
#define DMA_TRIG_XBAR_EDMA_MODULE_43    43
#define DMA_TRIG_XBAR_EDMA_MODULE_44    44
#define DMA_TRIG_XBAR_EDMA_MODULE_45    45
#define DMA_TRIG_XBAR_EDMA_MODULE_46    46
#define DMA_TRIG_XBAR_EDMA_MODULE_47    47
#define DMA_TRIG_XBAR_EDMA_MODULE_48    48
#define DMA_TRIG_XBAR_EDMA_MODULE_49    49
#define DMA_TRIG_XBAR_EDMA_MODULE_50    50
#define DMA_TRIG_XBAR_EDMA_MODULE_51    51
#define DMA_TRIG_XBAR_EDMA_MODULE_52    52
#define DMA_TRIG_XBAR_EDMA_MODULE_53    53
#define DMA_TRIG_XBAR_EDMA_MODULE_54    54
#define DMA_TRIG_XBAR_EDMA_MODULE_55    55
#define DMA_TRIG_XBAR_EDMA_MODULE_56    56
#define DMA_TRIG_XBAR_EDMA_MODULE_57    57
#define DMA_TRIG_XBAR_EDMA_MODULE_58    58
#define DMA_TRIG_XBAR_EDMA_MODULE_59    59
#define DMA_TRIG_XBAR_EDMA_MODULE_60    60
#define DMA_TRIG_XBAR_EDMA_MODULE_61    61
#define DMA_TRIG_XBAR_EDMA_MODULE_62    62
#define DMA_TRIG_XBAR_EDMA_MODULE_63    63

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t INTR_MUXCNTL[64];
} CSL_edma_trig_xbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_EDMA_TRIG_XBAR_PID                                                 (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL(INTR_MUXCNTL)                               (0x00000004U+((INTR_MUXCNTL)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_EDMA_TRIG_XBAR_PID_SCHEME_MASK                                     (0xC0000000U)
#define CSL_EDMA_TRIG_XBAR_PID_SCHEME_SHIFT                                    (0x0000001EU)
#define CSL_EDMA_TRIG_XBAR_PID_SCHEME_RESETVAL                                 (0x00000001U)
#define CSL_EDMA_TRIG_XBAR_PID_SCHEME_MAX                                      (0x00000003U)

#define CSL_EDMA_TRIG_XBAR_PID_BU_MASK                                         (0x30000000U)
#define CSL_EDMA_TRIG_XBAR_PID_BU_SHIFT                                        (0x0000001CU)
#define CSL_EDMA_TRIG_XBAR_PID_BU_RESETVAL                                     (0x00000002U)
#define CSL_EDMA_TRIG_XBAR_PID_BU_MAX                                          (0x00000003U)

#define CSL_EDMA_TRIG_XBAR_PID_FUNCTION_MASK                                   (0x0FFF0000U)
#define CSL_EDMA_TRIG_XBAR_PID_FUNCTION_SHIFT                                  (0x00000010U)
#define CSL_EDMA_TRIG_XBAR_PID_FUNCTION_RESETVAL                               (0x00000694U)
#define CSL_EDMA_TRIG_XBAR_PID_FUNCTION_MAX                                    (0x00000FFFU)

#define CSL_EDMA_TRIG_XBAR_PID_RTLVER_MASK                                     (0x0000F800U)
#define CSL_EDMA_TRIG_XBAR_PID_RTLVER_SHIFT                                    (0x0000000BU)
#define CSL_EDMA_TRIG_XBAR_PID_RTLVER_RESETVAL                                 (0x00000010U)
#define CSL_EDMA_TRIG_XBAR_PID_RTLVER_MAX                                      (0x0000001FU)

#define CSL_EDMA_TRIG_XBAR_PID_MAJREV_MASK                                     (0x00000700U)
#define CSL_EDMA_TRIG_XBAR_PID_MAJREV_SHIFT                                    (0x00000008U)
#define CSL_EDMA_TRIG_XBAR_PID_MAJREV_RESETVAL                                 (0x00000001U)
#define CSL_EDMA_TRIG_XBAR_PID_MAJREV_MAX                                      (0x00000007U)

#define CSL_EDMA_TRIG_XBAR_PID_CUSTOM_MASK                                     (0x000000C0U)
#define CSL_EDMA_TRIG_XBAR_PID_CUSTOM_SHIFT                                    (0x00000006U)
#define CSL_EDMA_TRIG_XBAR_PID_CUSTOM_RESETVAL                                 (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_PID_CUSTOM_MAX                                      (0x00000003U)

#define CSL_EDMA_TRIG_XBAR_PID_MINREV_MASK                                     (0x0000003FU)
#define CSL_EDMA_TRIG_XBAR_PID_MINREV_SHIFT                                    (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_PID_MINREV_RESETVAL                                 (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_PID_MINREV_MAX                                      (0x0000003FU)

#define CSL_EDMA_TRIG_XBAR_PID_RESETVAL                                        (0x66948100U)

/* INTR_MUXCNTL */

#define CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_MASK                             (0x00010000U)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_SHIFT                            (0x00000010U)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_MAX                              (0x00000001U)

#define CSL_EDMA_TRIG_XBAR_MUXCNTL_ENABLE_MASK                                 (0x000000FFU)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL_ENABLE_SHIFT                                (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL_ENABLE_RESETVAL                             (0x00000000U)
#define CSL_EDMA_TRIG_XBAR_MUXCNTL_ENABLE_MAX                                  (0x000000FFU)

#define CSL_EDMA_TRIG_XBAR_MUXCNTL_RESETVAL                                    (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
