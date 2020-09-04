/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 */

#ifndef ICSS_INTC_DEFINES_H_
#define ICSS_INTC_DEFINES_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 *  @brief  ICSSG Interrupt Mapping Defines
 *
 *  • Events to Channel Mapping
 *  • Channel to Host Mapping
 */

/**
 *  @brief  PRU_ICSSG_0 & PRU_ICSSG_1 Interrupt Signals:
 *
 *  • All 160 PRU_ICSSG system events are interrupt inputs. System events 64 through 159 are external and are
 *    generated from different peripherals. System events 0 through 63 are internal and are generated from internal
 *    for PRU_ICSSG sources.
 */
/*       Defines         |     Event Number         */
#define ICSS_INTC_EVENT_0           0
#define ICSS_INTC_EVENT_1           1
#define ICSS_INTC_EVENT_2           2
#define ICSS_INTC_EVENT_3           3
#define ICSS_INTC_EVENT_4           4
#define ICSS_INTC_EVENT_5           5
#define ICSS_INTC_EVENT_6           6
#define ICSS_INTC_EVENT_7           7
#define ICSS_INTC_EVENT_8           8
#define ICSS_INTC_EVENT_9           9
#define ICSS_INTC_EVENT_10          10
#define ICSS_INTC_EVENT_11          11
#define ICSS_INTC_EVENT_12          12
#define ICSS_INTC_EVENT_13          13
#define ICSS_INTC_EVENT_14          14
#define ICSS_INTC_EVENT_15          15
#define ICSS_INTC_EVENT_16          16
#define ICSS_INTC_EVENT_17          17
#define ICSS_INTC_EVENT_18          18
#define ICSS_INTC_EVENT_19          19
#define ICSS_INTC_EVENT_20          20
#define ICSS_INTC_EVENT_21          21
#define ICSS_INTC_EVENT_22          22
#define ICSS_INTC_EVENT_23          23
#define ICSS_INTC_EVENT_24          24
#define ICSS_INTC_EVENT_25          25
#define ICSS_INTC_EVENT_26          26
#define ICSS_INTC_EVENT_27          27
#define ICSS_INTC_EVENT_28          28
#define ICSS_INTC_EVENT_29          29
#define ICSS_INTC_EVENT_30          30
#define ICSS_INTC_EVENT_31          31
#define ICSS_INTC_EVENT_32          32
#define ICSS_INTC_EVENT_33          33
#define ICSS_INTC_EVENT_34          34
#define ICSS_INTC_EVENT_35          35
#define ICSS_INTC_EVENT_36          36
#define ICSS_INTC_EVENT_37          37
#define ICSS_INTC_EVENT_38          38
#define ICSS_INTC_EVENT_39          39
#define ICSS_INTC_EVENT_40          40
#define ICSS_INTC_EVENT_41          41
#define ICSS_INTC_EVENT_42          42
#define ICSS_INTC_EVENT_43          43
#define ICSS_INTC_EVENT_44          44
#define ICSS_INTC_EVENT_45          45
#define ICSS_INTC_EVENT_46          46
#define ICSS_INTC_EVENT_47          47
#define ICSS_INTC_EVENT_48          48
#define ICSS_INTC_EVENT_49          49
#define ICSS_INTC_EVENT_50          50
#define ICSS_INTC_EVENT_51          51
#define ICSS_INTC_EVENT_52          52
#define ICSS_INTC_EVENT_53          53
#define ICSS_INTC_EVENT_54          54
#define ICSS_INTC_EVENT_55          55
#define ICSS_INTC_EVENT_56          56
#define ICSS_INTC_EVENT_57          57
#define ICSS_INTC_EVENT_58          58
#define ICSS_INTC_EVENT_59          59
#define ICSS_INTC_EVENT_60          60
#define ICSS_INTC_EVENT_61          61
#define ICSS_INTC_EVENT_62          62
#define ICSS_INTC_EVENT_63          63
#define ICSS_INTC_EVENT_64          64
#define ICSS_INTC_EVENT_65          65
#define ICSS_INTC_EVENT_66          66
#define ICSS_INTC_EVENT_67          67
#define ICSS_INTC_EVENT_68          68
#define ICSS_INTC_EVENT_69          69
#define ICSS_INTC_EVENT_70          70
#define ICSS_INTC_EVENT_71          71
#define ICSS_INTC_EVENT_72          72
#define ICSS_INTC_EVENT_73          73
#define ICSS_INTC_EVENT_74          74
#define ICSS_INTC_EVENT_75          75
#define ICSS_INTC_EVENT_76          76
#define ICSS_INTC_EVENT_77          77
#define ICSS_INTC_EVENT_78          78
#define ICSS_INTC_EVENT_79          79
#define ICSS_INTC_EVENT_80          80
#define ICSS_INTC_EVENT_81          81
#define ICSS_INTC_EVENT_82          82
#define ICSS_INTC_EVENT_83          83
#define ICSS_INTC_EVENT_84          84
#define ICSS_INTC_EVENT_85          85
#define ICSS_INTC_EVENT_86          86
#define ICSS_INTC_EVENT_87          87
#define ICSS_INTC_EVENT_88          88
#define ICSS_INTC_EVENT_89          89
#define ICSS_INTC_EVENT_90          90
#define ICSS_INTC_EVENT_91          91
#define ICSS_INTC_EVENT_92          92
#define ICSS_INTC_EVENT_93          93
#define ICSS_INTC_EVENT_94          94
#define ICSS_INTC_EVENT_95          95
#define ICSS_INTC_EVENT_96          96
#define ICSS_INTC_EVENT_97          97
#define ICSS_INTC_EVENT_98          98
#define ICSS_INTC_EVENT_99          99
#define ICSS_INTC_EVENT_100         100
#define ICSS_INTC_EVENT_101         101
#define ICSS_INTC_EVENT_102         102
#define ICSS_INTC_EVENT_103         103
#define ICSS_INTC_EVENT_104         104
#define ICSS_INTC_EVENT_105         105
#define ICSS_INTC_EVENT_106         106
#define ICSS_INTC_EVENT_107         107
#define ICSS_INTC_EVENT_108         108
#define ICSS_INTC_EVENT_109         109
#define ICSS_INTC_EVENT_110         110
#define ICSS_INTC_EVENT_111         111
#define ICSS_INTC_EVENT_112         112
#define ICSS_INTC_EVENT_113         113
#define ICSS_INTC_EVENT_114         114
#define ICSS_INTC_EVENT_115         115
#define ICSS_INTC_EVENT_116         116
#define ICSS_INTC_EVENT_117         117
#define ICSS_INTC_EVENT_118         118
#define ICSS_INTC_EVENT_119         119
#define ICSS_INTC_EVENT_120         120
#define ICSS_INTC_EVENT_121         121
#define ICSS_INTC_EVENT_122         122
#define ICSS_INTC_EVENT_123         123
#define ICSS_INTC_EVENT_124         124
#define ICSS_INTC_EVENT_125         125
#define ICSS_INTC_EVENT_126         126
#define ICSS_INTC_EVENT_127         127
#define ICSS_INTC_EVENT_128         128
#define ICSS_INTC_EVENT_129         129
#define ICSS_INTC_EVENT_130         130
#define ICSS_INTC_EVENT_131         131
#define ICSS_INTC_EVENT_132         132
#define ICSS_INTC_EVENT_133         133
#define ICSS_INTC_EVENT_134         134
#define ICSS_INTC_EVENT_135         135
#define ICSS_INTC_EVENT_136         136
#define ICSS_INTC_EVENT_137         137
#define ICSS_INTC_EVENT_138         138
#define ICSS_INTC_EVENT_139         139
#define ICSS_INTC_EVENT_140         140
#define ICSS_INTC_EVENT_141         141
#define ICSS_INTC_EVENT_142         142
#define ICSS_INTC_EVENT_143         143
#define ICSS_INTC_EVENT_144         144
#define ICSS_INTC_EVENT_145         145
#define ICSS_INTC_EVENT_146         146
#define ICSS_INTC_EVENT_147         147
#define ICSS_INTC_EVENT_148         148
#define ICSS_INTC_EVENT_149         149
#define ICSS_INTC_EVENT_150         150
#define ICSS_INTC_EVENT_151         151
#define ICSS_INTC_EVENT_152         152
#define ICSS_INTC_EVENT_153         153
#define ICSS_INTC_EVENT_154         154
#define ICSS_INTC_EVENT_155         155
#define ICSS_INTC_EVENT_156         156
#define ICSS_INTC_EVENT_157         157
#define ICSS_INTC_EVENT_158         158
#define ICSS_INTC_EVENT_159         159

/**
 *  @brief Channel Mapping
 *
 *  • Any of the 160 internal interrupts can be mapped to any of the 20 channels.
 *  • Multiple interrupts can be mapped to a single channel.
 *  • An interrupt should not be mapped to more than one channel.
 *  • Any of the 20 channels can be mapped to any of the 20 host interrupts. It is recommended to map channel
 *    “x” to host interrupt “x”, where x is from 0 to 19.
 *  • A channel should not be mapped to more than one host interrupt
 *  • For channels mapping to the same host interrupt, lower number channels have higher priority.
 *  • For interrupts on same channel, priority is determined by the hardware interrupt number. The lower the
 *    interrupt number, the higher the priority
 */
#define ICSS_INTC_CHANNEL_0                0
#define ICSS_INTC_CHANNEL_1                1
#define ICSS_INTC_CHANNEL_2                2
#define ICSS_INTC_CHANNEL_3                3
#define ICSS_INTC_CHANNEL_4                4
#define ICSS_INTC_CHANNEL_5                5
#define ICSS_INTC_CHANNEL_6                6
#define ICSS_INTC_CHANNEL_7                7
#define ICSS_INTC_CHANNEL_8                8
#define ICSS_INTC_CHANNEL_9                9
#define ICSS_INTC_CHANNEL_10               10
#define ICSS_INTC_CHANNEL_11               11
#define ICSS_INTC_CHANNEL_12               12
#define ICSS_INTC_CHANNEL_13               13
#define ICSS_INTC_CHANNEL_14               14
#define ICSS_INTC_CHANNEL_15               15
#define ICSS_INTC_CHANNEL_16               16
#define ICSS_INTC_CHANNEL_17               17
#define ICSS_INTC_CHANNEL_18               18
#define ICSS_INTC_CHANNEL_19               19

/**
 *  @brief  Generation of 20 Host Interrupts
 *
 *  • Host Interrupt 0 is connected to bit 30 in register 31 (R31) of PRU0 and PRU1 in parallel.
 *  • Host Interrupt 1 is connected to bit 31 in register 31 (R31) for PRU0 and PRU1 in parallel.
 *  • Host Interrupt: 2 through 9 are exported from PRU_ICSSG internal INTC for signaling the device level interrupt
 *    controllers (pulse and level provided).
 *  • Host Interrupt 10 is connected to bit 30 in register 31 (R31) to both RTU_PRU0 and RTU_PRU1 in parallel.
 *  • Host Interrupt 11 is connected to bit 31 in register 31 (R31) to both RTU_PRU0 and RTU_PRU1 in parallel.
 *  • Host Interrupts 12 through 19 are connected to each of the 4 Task Managers.
 */
#define ICSS_INTC_HOST_INTR_0           0
#define ICSS_INTC_HOST_INTR_1           1
#define ICSS_INTC_HOST_INTR_2           2
#define ICSS_INTC_HOST_INTR_3           3
#define ICSS_INTC_HOST_INTR_4           4
#define ICSS_INTC_HOST_INTR_5           5
#define ICSS_INTC_HOST_INTR_6           6
#define ICSS_INTC_HOST_INTR_7           7
#define ICSS_INTC_HOST_INTR_8           8
#define ICSS_INTC_HOST_INTR_9           9
#define ICSS_INTC_HOST_INTR_10          10
#define ICSS_INTC_HOST_INTR_11          11
#define ICSS_INTC_HOST_INTR_12          12
#define ICSS_INTC_HOST_INTR_13          13
#define ICSS_INTC_HOST_INTR_14          14
#define ICSS_INTC_HOST_INTR_15          15
#define ICSS_INTC_HOST_INTR_16          16
#define ICSS_INTC_HOST_INTR_17          17
#define ICSS_INTC_HOST_INTR_18          18
#define ICSS_INTC_HOST_INTR_19          19

#define ICSS_INTC_HOST_INTR_0_HOSTEN_MASK      0x00001
#define ICSS_INTC_HOST_INTR_1_HOSTEN_MASK      0x00002
#define ICSS_INTC_HOST_INTR_2_HOSTEN_MASK      0x00004
#define ICSS_INTC_HOST_INTR_3_HOSTEN_MASK      0x00008
#define ICSS_INTC_HOST_INTR_4_HOSTEN_MASK      0x00010
#define ICSS_INTC_HOST_INTR_5_HOSTEN_MASK      0x00020
#define ICSS_INTC_HOST_INTR_6_HOSTEN_MASK      0x00040
#define ICSS_INTC_HOST_INTR_7_HOSTEN_MASK      0x00080
#define ICSS_INTC_HOST_INTR_8_HOSTEN_MASK      0x00100
#define ICSS_INTC_HOST_INTR_9_HOSTEN_MASK      0x00200
#define ICSS_INTC_HOST_INTR_10_HOSTEN_MASK     0x00400
#define ICSS_INTC_HOST_INTR_11_HOSTEN_MASK     0x00800
#define ICSS_INTC_HOST_INTR_12_HOSTEN_MASK     0x01000
#define ICSS_INTC_HOST_INTR_13_HOSTEN_MASK     0x02000
#define ICSS_INTC_HOST_INTR_14_HOSTEN_MASK     0x04000
#define ICSS_INTC_HOST_INTR_15_HOSTEN_MASK     0x08000
#define ICSS_INTC_HOST_INTR_16_HOSTEN_MASK     0x10000
#define ICSS_INTC_HOST_INTR_17_HOSTEN_MASK     0x20000
#define ICSS_INTC_HOST_INTR_18_HOSTEN_MASK     0x40000
#define ICSS_INTC_HOST_INTR_19_HOSTEN_MASK     0x80000

#define SYS_EVT_POLARITY_LOW        0
#define SYS_EVT_POLARITY_HIGH       1

#define SYS_EVT_TYPE_PULSE          0
#define SYS_EVT_TYPE_EDGE           1

/**
 *  @brief AM64x - R5F core interrupt mapping
 *
 *  Interrupt Input           |  Line |  Interrupt ID Source Interrupt
 *  =====================================================================
 *  R5FSS0_CORE0_INTR_IN_120  |  120  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_0
 *  R5FSS0_CORE0_INTR_IN_121  |  121  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_1
 *  R5FSS0_CORE0_INTR_IN_122  |  122  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_2
 *  R5FSS0_CORE0_INTR_IN_123  |  123  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_3
 *  R5FSS0_CORE0_INTR_IN_124  |  124  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_4
 *  R5FSS0_CORE0_INTR_IN_125  |  125  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_5
 *  R5FSS0_CORE0_INTR_IN_126  |  126  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_6
 *  R5FSS0_CORE0_INTR_IN_127  |  127  |  PRU_ICSSG0_PR1_HOST_INTR_PEND_7
 *  ---------------------------------------------------------------------
 *  R5FSS0_CORE0_INTR_IN_240  |  240  |  PRU_ICSSG1_PR1_RX_SOF_INTR_REQ_0
 *  R5FSS0_CORE0_INTR_IN_241  |  241  |  PRU_ICSSG1_PR1_RX_SOF_INTR_REQ_1
 *  R5FSS0_CORE0_INTR_IN_242  |  242  |  PRU_ICSSG1_PR1_TX_SOF_INTR_REQ_0
 *  R5FSS0_CORE0_INTR_IN_243  |  243  |  PRU_ICSSG1_PR1_TX_SOF_INTR_REQ_1
 *  R5FSS0_CORE0_INTR_IN_244  |  244  |  PRU_ICSSG0_PR1_RX_SOF_INTR_REQ_0
 *  R5FSS0_CORE0_INTR_IN_245  |  245  |  PRU_ICSSG0_PR1_RX_SOF_INTR_REQ_1
 *  R5FSS0_CORE0_INTR_IN_246  |  246  |  PRU_ICSSG0_PR1_TX_SOF_INTR_REQ_0
 *  R5FSS0_CORE0_INTR_IN_247  |  247  |  PRU_ICSSG0_PR1_TX_SOF_INTR_REQ_1
 *  ---------------------------------------------------------------------
 *  R5FSS0_CORE0_INTR_IN_248  |  248  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_0
 *  R5FSS0_CORE0_INTR_IN_249  |  249  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_1
 *  R5FSS0_CORE0_INTR_IN_250  |  250  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_2
 *  R5FSS0_CORE0_INTR_IN_251  |  251  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_3
 *  R5FSS0_CORE0_INTR_IN_252  |  252  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_4
 *  R5FSS0_CORE0_INTR_IN_253  |  253  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_5
 *  R5FSS0_CORE0_INTR_IN_254  |  254  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_6
 *  R5FSS0_CORE0_INTR_IN_255  |  255  |  PRU_ICSSG1_PR1_HOST_INTR_PEND_7
 */


#ifdef __cplusplus
}
#endif

#endif /* ICSS_INTC_DEFINES_H_ */
