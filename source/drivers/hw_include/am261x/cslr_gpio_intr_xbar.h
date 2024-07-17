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
 *  Name        : cslr_gpio_intr_xbar.h
*/
#ifndef CSLR_GPIO_INTR_XBAR_H_
#define CSLR_GPIO_INTR_XBAR_H_

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

#define GPIO_INT_XBAR_GPIO_MUX_0	    0
#define GPIO_INT_XBAR_GPIO_MUX_1	    1
#define GPIO_INT_XBAR_GPIO_MUX_2	    2
#define GPIO_INT_XBAR_GPIO_MUX_3	    3
#define GPIO_INT_XBAR_GPIO_MUX_4	    4
#define GPIO_INT_XBAR_GPIO_MUX_5	    5
#define GPIO_INT_XBAR_GPIO_MUX_6	    6
#define GPIO_INT_XBAR_GPIO_MUX_7	    7
#define GPIO_INT_XBAR_GPIO_MUX_8	    8
#define GPIO_INT_XBAR_GPIO_MUX_9	    9
#define GPIO_INT_XBAR_GPIO_MUX_10	10
#define GPIO_INT_XBAR_GPIO_MUX_11	11
#define GPIO_INT_XBAR_GPIO_MUX_12	12
#define GPIO_INT_XBAR_GPIO_MUX_13	13
#define GPIO_INT_XBAR_GPIO_MUX_14	14
#define GPIO_INT_XBAR_GPIO_MUX_15	15
#define GPIO_INT_XBAR_GPIO_MUX_16	16
#define GPIO_INT_XBAR_GPIO_MUX_17	17
#define GPIO_INT_XBAR_GPIO_MUX_18	18
#define GPIO_INT_XBAR_GPIO_MUX_19	19
#define GPIO_INT_XBAR_GPIO_MUX_20	20
#define GPIO_INT_XBAR_GPIO_MUX_21	21
#define GPIO_INT_XBAR_GPIO_MUX_22	22
#define GPIO_INT_XBAR_GPIO_MUX_23	23
#define GPIO_INT_XBAR_GPIO_MUX_24	24
#define GPIO_INT_XBAR_GPIO_MUX_25	25
#define GPIO_INT_XBAR_GPIO_MUX_26	26
#define GPIO_INT_XBAR_GPIO_MUX_27	27
#define GPIO_INT_XBAR_GPIO_MUX_28	28
#define GPIO_INT_XBAR_GPIO_MUX_29	29
#define GPIO_INT_XBAR_GPIO_MUX_30	30
#define GPIO_INT_XBAR_GPIO_MUX_31	31
#define GPIO_INT_XBAR_GPIO_MUX_32	32
#define GPIO_INT_XBAR_GPIO_MUX_33	33
#define GPIO_INT_XBAR_GPIO_MUX_34	34
#define GPIO_INT_XBAR_GPIO_MUX_35	35
#define GPIO_INT_XBAR_GPIO_MUX_36	36
#define GPIO_INT_XBAR_GPIO_MUX_37	37
#define GPIO_INT_XBAR_GPIO_MUX_38	38
#define GPIO_INT_XBAR_GPIO_MUX_39	39
#define GPIO_INT_XBAR_GPIO_MUX_40	40
#define GPIO_INT_XBAR_GPIO_MUX_41	41
#define GPIO_INT_XBAR_GPIO_MUX_42	42
#define GPIO_INT_XBAR_GPIO_MUX_43	43
#define GPIO_INT_XBAR_GPIO_MUX_44	44
#define GPIO_INT_XBAR_GPIO_MUX_45	45
#define GPIO_INT_XBAR_GPIO_MUX_46	46
#define GPIO_INT_XBAR_GPIO_MUX_47	47
#define GPIO_INT_XBAR_GPIO_MUX_48	48
#define GPIO_INT_XBAR_GPIO_MUX_49	49
#define GPIO_INT_XBAR_GPIO_MUX_50	50
#define GPIO_INT_XBAR_GPIO_MUX_51	51
#define GPIO_INT_XBAR_GPIO_MUX_52	52
#define GPIO_INT_XBAR_GPIO_MUX_53	53
#define GPIO_INT_XBAR_GPIO_MUX_54	54
#define GPIO_INT_XBAR_GPIO_MUX_55	55
#define GPIO_INT_XBAR_GPIO_MUX_56	56
#define GPIO_INT_XBAR_GPIO_MUX_57	57
#define GPIO_INT_XBAR_GPIO_MUX_58	58
#define GPIO_INT_XBAR_GPIO_MUX_59	59
#define GPIO_INT_XBAR_GPIO_MUX_60	60
#define GPIO_INT_XBAR_GPIO_MUX_61	61
#define GPIO_INT_XBAR_GPIO_MUX_62	62
#define GPIO_INT_XBAR_GPIO_MUX_63	63
#define GPIO_INT_XBAR_GPIO_MUX_64	64
#define GPIO_INT_XBAR_GPIO_MUX_65	65
#define GPIO_INT_XBAR_GPIO_MUX_66	66
#define GPIO_INT_XBAR_GPIO_MUX_67	67
#define GPIO_INT_XBAR_GPIO_MUX_68	68
#define GPIO_INT_XBAR_GPIO_MUX_69	69
#define GPIO_INT_XBAR_GPIO_MUX_70	70
#define GPIO_INT_XBAR_GPIO_MUX_71	71
#define GPIO_INT_XBAR_GPIO_MUX_72	72
#define GPIO_INT_XBAR_GPIO_MUX_73	73
#define GPIO_INT_XBAR_GPIO_MUX_74	74
#define GPIO_INT_XBAR_GPIO_MUX_75	75
#define GPIO_INT_XBAR_GPIO_MUX_76	76
#define GPIO_INT_XBAR_GPIO_MUX_77	77
#define GPIO_INT_XBAR_GPIO_MUX_78	78
#define GPIO_INT_XBAR_GPIO_MUX_79	79
#define GPIO_INT_XBAR_GPIO_MUX_80	80
#define GPIO_INT_XBAR_GPIO_MUX_81	81
#define GPIO_INT_XBAR_GPIO_MUX_82	82
#define GPIO_INT_XBAR_GPIO_MUX_83	83
#define GPIO_INT_XBAR_GPIO_MUX_84	84
#define GPIO_INT_XBAR_GPIO_MUX_85	85
#define GPIO_INT_XBAR_GPIO_MUX_86	86
#define GPIO_INT_XBAR_GPIO_MUX_87	87
#define GPIO_INT_XBAR_GPIO_MUX_88	88
#define GPIO_INT_XBAR_GPIO_MUX_89	89
#define GPIO_INT_XBAR_GPIO_MUX_90	90
#define GPIO_INT_XBAR_GPIO_MUX_91	91
#define GPIO_INT_XBAR_GPIO_MUX_92	92
#define GPIO_INT_XBAR_GPIO_MUX_93	93
#define GPIO_INT_XBAR_GPIO_MUX_94	94
#define GPIO_INT_XBAR_GPIO_MUX_95	95
#define GPIO_INT_XBAR_GPIO_MUX_96	96
#define GPIO_INT_XBAR_GPIO_MUX_97	97
#define GPIO_INT_XBAR_GPIO_MUX_98	98
#define GPIO_INT_XBAR_GPIO_MUX_99	99
#define GPIO_INT_XBAR_GPIO_MUX_100	100
#define GPIO_INT_XBAR_GPIO_MUX_101	101
#define GPIO_INT_XBAR_GPIO_MUX_102	102
#define GPIO_INT_XBAR_GPIO_MUX_103	103
#define GPIO_INT_XBAR_GPIO_MUX_104	104
#define GPIO_INT_XBAR_GPIO_MUX_105	105
#define GPIO_INT_XBAR_GPIO_MUX_106	106
#define GPIO_INT_XBAR_GPIO_MUX_107	107
#define GPIO_INT_XBAR_GPIO_MUX_108	108
#define GPIO_INT_XBAR_GPIO_MUX_109	109
#define GPIO_INT_XBAR_GPIO_MUX_110	110
#define GPIO_INT_XBAR_GPIO_MUX_111	111
#define GPIO_INT_XBAR_GPIO_MUX_112	112
#define GPIO_INT_XBAR_GPIO_MUX_113	113
#define GPIO_INT_XBAR_GPIO_MUX_114	114
#define GPIO_INT_XBAR_GPIO_MUX_115	115
#define GPIO_INT_XBAR_GPIO_MUX_116	116
#define GPIO_INT_XBAR_GPIO_MUX_117	117
#define GPIO_INT_XBAR_GPIO_MUX_118	118
#define GPIO_INT_XBAR_GPIO_MUX_119	119
#define GPIO_INT_XBAR_GPIO_MUX_120	120
#define GPIO_INT_XBAR_GPIO_MUX_121	121
#define GPIO_INT_XBAR_GPIO_MUX_122	122
#define GPIO_INT_XBAR_GPIO_MUX_123	123
#define GPIO_INT_XBAR_GPIO_MUX_124	124
#define GPIO_INT_XBAR_GPIO_MUX_125	125
#define GPIO_INT_XBAR_GPIO_MUX_126	126
#define GPIO_INT_XBAR_GPIO_MUX_127	127
#define GPIO_INT_XBAR_GPIO_MUX_128	128
#define GPIO_INT_XBAR_GPIO_MUX_129	129
#define GPIO_INT_XBAR_GPIO_MUX_130	130
#define GPIO_INT_XBAR_GPIO_MUX_131	131
#define GPIO_INT_XBAR_GPIO_MUX_132	132
#define GPIO_INT_XBAR_GPIO_MUX_133	133
#define GPIO_INT_XBAR_GPIO_MUX_134	134
#define GPIO_INT_XBAR_GPIO_MUX_135	135
#define GPIO_INT_XBAR_GPIO_MUX_136	136
#define GPIO_INT_XBAR_GPIO_MUX_137	137
#define GPIO_INT_XBAR_GPIO_MUX_138	138
#define GPIO_INT_XBAR_GPIO_MUX_139	139
#define GPIO_INT_XBAR_GPIO_MUX_140	140
#define GPIO_INT_XBAR_GPIO_MUX_141	141
#define GPIO_INT_XBAR_GPIO_MUX_142	142
#define GPIO_INT_XBAR_GPIO_MUX_143	143
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_0	144
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_1	145
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_2	146
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_3	147
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_4	148
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_5	149
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_6	150
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_7	151
#define GPIO_INT_XBAR_GPIO_0_BANK_INTR_8	152
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_0	153
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_1	154
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_2	155
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_3	156
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_4	157
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_5	158
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_6	159
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_7	160
#define GPIO_INT_XBAR_GPIO_1_BANK_INTR_8	161

/**************************************************************************
    XBAR OUTPUT Macros
**************************************************************************/

#define GPIO_INT_XBAR_ICSS_XBAR_0               0
#define GPIO_INT_XBAR_ICSS_XBAR_1               1
#define GPIO_INT_XBAR_ICSS_XBAR_2               2
#define GPIO_INT_XBAR_ICSS_XBAR_3               3
#define GPIO_INT_XBAR_DMA_TRIG_XBAR_0           4
#define GPIO_INT_XBAR_DMA_TRIG_XBAR_1           5
#define GPIO_INT_XBAR_DMA_TRIG_XBAR_2           6
#define GPIO_INT_XBAR_DMA_TRIG_XBAR_3           7
#define GPIO_INT_XBAR_SOC_TIMESYNC_XBAR1_0      8
#define GPIO_INT_XBAR_SOC_TIMESYNC_XBAR1_1      9
#define GPIO_INT_XBAR_SOC_TIMESYNC_XBAR1_2      10
#define GPIO_INT_XBAR_SOC_TIMESYNC_XBAR1_3      11
#define GPIO_INT_XBAR_SOC_TIMESYNC_XBAR1_4      12
#define GPIO_INT_XBAR_SOC_TIMESYNC_XBAR1_5      13
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_0      14
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_1      15
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_2      16
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_3      17
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_4      18
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_5      19
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_6      20
#define GPIO_INT_XBAR_SOC_TIMESYNC_ICSS1_7      21
#define GPIO_INT_XBAR_VIM_MODULE0_0             22
#define GPIO_INT_XBAR_VIM_MODULE0_1             23
#define GPIO_INT_XBAR_VIM_MODULE0_2             24
#define GPIO_INT_XBAR_VIM_MODULE0_3             25
#define GPIO_INT_XBAR_VIM_MODULE1_0             26
#define GPIO_INT_XBAR_VIM_MODULE1_1             27
#define GPIO_INT_XBAR_VIM_MODULE1_2             28
#define GPIO_INT_XBAR_VIM_MODULE1_3             29

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t INTR_MUXCNTL[22];
} CSL_gpio_intr_xbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GPIO_INTR_XBAR_PID                                                 (0x00000000U)
#define CSL_GPIO_INTR_XBAR_MUXCNTL(INTR_MUXCNTL)                               (0x00000004U+((INTR_MUXCNTL)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_GPIO_INTR_XBAR_PID_SCHEME_MASK                                     (0xC0000000U)
#define CSL_GPIO_INTR_XBAR_PID_SCHEME_SHIFT                                    (0x0000001EU)
#define CSL_GPIO_INTR_XBAR_PID_SCHEME_RESETVAL                                 (0x00000001U)
#define CSL_GPIO_INTR_XBAR_PID_SCHEME_MAX                                      (0x00000003U)

#define CSL_GPIO_INTR_XBAR_PID_BU_MASK                                         (0x30000000U)
#define CSL_GPIO_INTR_XBAR_PID_BU_SHIFT                                        (0x0000001CU)
#define CSL_GPIO_INTR_XBAR_PID_BU_RESETVAL                                     (0x00000002U)
#define CSL_GPIO_INTR_XBAR_PID_BU_MAX                                          (0x00000003U)

#define CSL_GPIO_INTR_XBAR_PID_FUNCTION_MASK                                   (0x0FFF0000U)
#define CSL_GPIO_INTR_XBAR_PID_FUNCTION_SHIFT                                  (0x00000010U)
#define CSL_GPIO_INTR_XBAR_PID_FUNCTION_RESETVAL                               (0x00000694U)
#define CSL_GPIO_INTR_XBAR_PID_FUNCTION_MAX                                    (0x00000FFFU)

#define CSL_GPIO_INTR_XBAR_PID_RTLVER_MASK                                     (0x0000F800U)
#define CSL_GPIO_INTR_XBAR_PID_RTLVER_SHIFT                                    (0x0000000BU)
#define CSL_GPIO_INTR_XBAR_PID_RTLVER_RESETVAL                                 (0x00000010U)
#define CSL_GPIO_INTR_XBAR_PID_RTLVER_MAX                                      (0x0000001FU)

#define CSL_GPIO_INTR_XBAR_PID_MAJREV_MASK                                     (0x00000700U)
#define CSL_GPIO_INTR_XBAR_PID_MAJREV_SHIFT                                    (0x00000008U)
#define CSL_GPIO_INTR_XBAR_PID_MAJREV_RESETVAL                                 (0x00000001U)
#define CSL_GPIO_INTR_XBAR_PID_MAJREV_MAX                                      (0x00000007U)

#define CSL_GPIO_INTR_XBAR_PID_CUSTOM_MASK                                     (0x000000C0U)
#define CSL_GPIO_INTR_XBAR_PID_CUSTOM_SHIFT                                    (0x00000006U)
#define CSL_GPIO_INTR_XBAR_PID_CUSTOM_RESETVAL                                 (0x00000000U)
#define CSL_GPIO_INTR_XBAR_PID_CUSTOM_MAX                                      (0x00000003U)

#define CSL_GPIO_INTR_XBAR_PID_MINREV_MASK                                     (0x0000003FU)
#define CSL_GPIO_INTR_XBAR_PID_MINREV_SHIFT                                    (0x00000000U)
#define CSL_GPIO_INTR_XBAR_PID_MINREV_RESETVAL                                 (0x00000000U)
#define CSL_GPIO_INTR_XBAR_PID_MINREV_MAX                                      (0x0000003FU)

#define CSL_GPIO_INTR_XBAR_PID_RESETVAL                                        (0x66948100U)

/* INTR_MUXCNTL */

#define CSL_GPIO_INTR_XBAR_MUXCNTL_INT_ENABLE_MASK                             (0x00010000U)
#define CSL_GPIO_INTR_XBAR_MUXCNTL_INT_ENABLE_SHIFT                            (0x00000010U)
#define CSL_GPIO_INTR_XBAR_MUXCNTL_INT_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_GPIO_INTR_XBAR_MUXCNTL_INT_ENABLE_MAX                              (0x00000001U)

#define CSL_GPIO_INTR_XBAR_MUXCNTL_ENABLE_MASK                                 (0x000000FFU)
#define CSL_GPIO_INTR_XBAR_MUXCNTL_ENABLE_SHIFT                                (0x00000000U)
#define CSL_GPIO_INTR_XBAR_MUXCNTL_ENABLE_RESETVAL                             (0x00000000U)
#define CSL_GPIO_INTR_XBAR_MUXCNTL_ENABLE_MAX                                  (0x000000FFU)

#define CSL_GPIO_INTR_XBAR_INTR_MUXCNTL_RESETVAL                               (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
