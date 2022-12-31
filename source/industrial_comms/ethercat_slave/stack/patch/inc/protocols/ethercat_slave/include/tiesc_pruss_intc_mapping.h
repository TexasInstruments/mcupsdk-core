/**
 * tiesc_pruss_intc_mapping.h
 *
 * PRUSS INTC mapping for TI ESC application
 *
*/
/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

/*
 * ============================================================================
 * Copyright (c) Texas Instruments Inc 2011-12
 *
 * Use of this software is controlled by the terms and conditions found in the
 * license agreement under which this software has been supplied or provided.
 * ============================================================================
*/
//SYS_EVT_16-31 can be used for generating interrupts for IPC with hosts/prus etc

#define PD_WD_EXPIRY_EVENT   9
#define PDI_WD_EXPIRY_EVENT  10
#define LATCH1_IN_EVENT      11
#define LATCH0_IN_EVENT      12
#define SYNC1_OUT_EVENT      13
#define SYNC0_OUT_EVENT      14
#define PRU_ARM_EVENT0       18 //SYS_EVT_AL_EVENT_REQUEST
//#define PRU_ARM_EVENT1       17 //SYS_EVT_HOST_CMD_HIGH_ACK: Not used any more
#define ARM_PRU_EVENT0       16 //SYS_EVT_HOST_CMD
#define ARM_PRU_EVENT1       19 //SYS_EVT_HOST_CMD_LOW
#define PRU_ARM_EVENT2       20 //SYS_EVT_HOST_CMD_LOW_ACK
#define PRU0_RX_ERR32_EVENT  33
#define PRU0_RX_OVF_EVENT    38
#define PORT0_TX_UNF_EVENT   39
#define PORT0_TX_OVF_EVENT   40
#define MII_LINK0_EVENT      41
#define PRU1_RX_ERR32_EVENT  45
#define PRU1_RX_OVF_EVENT    50
#define PORT1_TX_UNF_EVENT   51
#define PORT1_TX_OVF_EVENT   52
#define MII_LINK1_EVENT      53
#define ICSS1_PRUSS0_HOST_INTR5     56



// ICSS0 event map
#define ICSS0_IEP_TIM_CAP_CMP_PEND  7
#define ICSS0_PRU_ENDAT_TRIGGER     16
#define ICSS0_PRU_ADC_MAN_TRIG      17
#define ICSS0_PRU_ARM_FOC_TRIG      18
#define ICSS0_PRU_ENDAT_COMPLETE    19

#define ICSS0_EPWM3_INTR            43
#define ICSS0_EPWM4_INTR            46
#define ICSS0_EPWM5_INTR            37 //EPWM5 SOC

#define CHANNEL0                0
#define CHANNEL1                1
#define CHANNEL2                2
#define CHANNEL3                3
#define CHANNEL4                4
#define CHANNEL5                5
#define CHANNEL6                6
#define CHANNEL7                7
#define CHANNEL8                8
#define CHANNEL9                9

#define PRU0                    0
#define PRU1                    1
#define PRU_EVTOUT0             2
#define PRU_EVTOUT1             3
#define PRU_EVTOUT2             4
#define PRU_EVTOUT3             5
#define PRU_EVTOUT4             6
#define PRU_EVTOUT5             7
#define PRU_EVTOUT6             8
#define PRU_EVTOUT7             9

#define HOST_AL_EVENT       (20-CHANNEL2)+CHANNEL5
#define HOST_SYNC0_EVENT    (20-CHANNEL2)+CHANNEL3
#define HOST_SYNC1_EVENT    (20-CHANNEL2)+CHANNEL4
//#define   HOST_CMD_HIGH_ACK_EVENT (20-CHANNEL2)+CHANNEL4
#define HOST_CMD_LOW_ACK_EVENT  (20-CHANNEL2)+CHANNEL6

#define PRU0_HOSTEN_MASK            0x0001
#define PRU1_HOSTEN_MASK            0x0002
#define PRU_EVTOUT0_HOSTEN_MASK     0x0004
#define PRU_EVTOUT1_HOSTEN_MASK     0x0008
#define PRU_EVTOUT2_HOSTEN_MASK     0x0010
#define PRU_EVTOUT3_HOSTEN_MASK     0x0020
#define PRU_EVTOUT4_HOSTEN_MASK     0x0040
#define PRU_EVTOUT5_HOSTEN_MASK     0x0080
#define PRU_EVTOUT6_HOSTEN_MASK     0x0100
#define PRU_EVTOUT7_HOSTEN_MASK     0x0200

#define SYS_EVT_POLARITY_LOW        0
#define SYS_EVT_POLARITY_HIGH       1

#define SYS_EVT_TYPE_PULSE          0
#define SYS_EVT_TYPE_EDGE           1

#define PRU_ICSS1_INTC_INITDATA {   \
        { PD_WD_EXPIRY_EVENT, PDI_WD_EXPIRY_EVENT, LATCH1_IN_EVENT, LATCH0_IN_EVENT,\
            SYNC1_OUT_EVENT, SYNC0_OUT_EVENT, ARM_PRU_EVENT0, ARM_PRU_EVENT1, PRU_ARM_EVENT0, /*PRU_ARM_EVENT1,*/ PRU_ARM_EVENT2,\
                             PRU0_RX_ERR32_EVENT, PRU0_RX_OVF_EVENT, PORT0_TX_UNF_EVENT, PORT0_TX_OVF_EVENT, MII_LINK0_EVENT,\
                             PRU1_RX_ERR32_EVENT, PRU1_RX_OVF_EVENT, PORT1_TX_UNF_EVENT, PORT1_TX_OVF_EVENT, MII_LINK1_EVENT, ICSS1_PRUSS0_HOST_INTR5, 0xFF },\
        { {PD_WD_EXPIRY_EVENT, CHANNEL1,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_EDGE },\
            {PDI_WD_EXPIRY_EVENT, CHANNEL1,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_EDGE}, \
            {LATCH1_IN_EVENT, CHANNEL1,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE },\
            {LATCH0_IN_EVENT, CHANNEL1,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE },\
            {SYNC1_OUT_EVENT,CHANNEL4, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},{SYNC0_OUT_EVENT, CHANNEL3,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_EDGE },\
            {ARM_PRU_EVENT0, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE },{ARM_PRU_EVENT1, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PRU_ARM_EVENT0, CHANNEL5, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PRU_ARM_EVENT2,CHANNEL6, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PRU0_RX_ERR32_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PRU0_RX_OVF_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PORT0_TX_UNF_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PORT0_TX_OVF_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {MII_LINK0_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
            {PRU1_RX_ERR32_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PRU1_RX_OVF_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PORT1_TX_UNF_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {PORT1_TX_OVF_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
            {MII_LINK1_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
            {ICSS1_PRUSS0_HOST_INTR5, CHANNEL2,SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_EDGE }, \
            {0xFF,0xFF,0xFF,0xFF}},\
        { {CHANNEL0,PRU0}, {CHANNEL1, PRU1}, {CHANNEL2, PRU_EVTOUT0}, {CHANNEL3, PRU_EVTOUT1}, {CHANNEL4, PRU_EVTOUT2}, {CHANNEL5, PRU_EVTOUT3}, {CHANNEL6, PRU_EVTOUT4}, {0xFF,0xFF} },\
        (PRU0_HOSTEN_MASK | PRU1_HOSTEN_MASK | PRU_EVTOUT0_HOSTEN_MASK | PRU_EVTOUT1_HOSTEN_MASK | PRU_EVTOUT2_HOSTEN_MASK | PRU_EVTOUT3_HOSTEN_MASK | PRU_EVTOUT4_HOSTEN_MASK)\
    }
