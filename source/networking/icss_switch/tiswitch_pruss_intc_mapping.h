/**
* @file tiswitch_pruss_intc_mapping.h
*
* @brief Pruss interrupt mapping related macros
*
* \par
* Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
* \par
*/

#ifdef __cplusplus
extern "C" {
#endif

/*SYS_EVT_16-31 can be used for generating interrupts for IPC with hosts/prus etc*/
#define IEP_TIM_CAP_CMP_EVENT   7
#define SYNC1_OUT_EVENT      13
#define SYNC0_OUT_EVENT      14
#define PRU_ARM_EVENT0       20 /*SYS_EVT_AL_EVENT_REQUEST*/
#define PRU_ARM_EVENT1       21
#define PRU_ARM_EVENT2       22
#define PRU_ARM_EVENT3       23
#define PRU_ARM_EVENT4       24
#define PRU_ARM_EVENT5       25
#define PRU_ARM_EVENT6       26
#define ARM_PRU_EVENT        16 /*SYS_EVT_HOST_CMD*/
#define PRU0_RX_ERR32_EVENT  33
#define PORT1_TX_UNDERFLOW   39
#define PORT1_TX_OVERFLOW    40
#define MII_LINK0_EVENT      41
#define PORT1_RX_EOF_EVENT   42
#define PRU1_RX_ERR32_EVENT  45
#define PORT2_TX_UNDERFLOW   51
#define PORT2_TX_OVERFLOW    53
#define PORT2_RX_EOF_EVENT   54
#define MII_LINK1_EVENT      53

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

#define PRUSS_INTC_INITDATA {   \
        { PRU_ARM_EVENT0, PRU_ARM_EVENT1, PRU_ARM_EVENT2, PRU_ARM_EVENT3, PRU_ARM_EVENT4, PRU_ARM_EVENT5, PORT1_RX_EOF_EVENT,\
            PORT2_RX_EOF_EVENT,MII_LINK0_EVENT,MII_LINK1_EVENT,0xFF},  \
        { {PORT1_RX_EOF_EVENT,CHANNEL0, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},\
            {PORT2_RX_EOF_EVENT,CHANNEL1, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE}, \
            {PRU_ARM_EVENT0,CHANNEL2, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},\
            {PRU_ARM_EVENT1, CHANNEL2, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},  \
            {PRU_ARM_EVENT2,CHANNEL4, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},   \
            {PRU_ARM_EVENT3, CHANNEL5, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},  \
            {PRU_ARM_EVENT4, CHANNEL5, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},  \
            {MII_LINK0_EVENT, CHANNEL7, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE}, \
            {MII_LINK1_EVENT, CHANNEL7, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE}, \
            {PRU_ARM_EVENT5, CHANNEL8, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE}, \
            {0xFF,0xFF,0xFF,0xFF}},  \
        { {CHANNEL0,PRU0}, {CHANNEL1, PRU1}, {CHANNEL2, PRU_EVTOUT0}, {CHANNEL3, PRU_EVTOUT1}, \
            {CHANNEL4, PRU_EVTOUT2}, {CHANNEL5, PRU_EVTOUT3},{CHANNEL6, PRU_EVTOUT4},{CHANNEL7, PRU_EVTOUT6}, {CHANNEL8, PRU_EVTOUT5}\
            , {0xFF,0xFF} },  \
        (PRU0_HOSTEN_MASK | PRU1_HOSTEN_MASK | PRU_EVTOUT0_HOSTEN_MASK | PRU_EVTOUT1_HOSTEN_MASK \
         | PRU_EVTOUT2_HOSTEN_MASK | PRU_EVTOUT3_HOSTEN_MASK| PRU_EVTOUT4_HOSTEN_MASK| PRU_EVTOUT5_HOSTEN_MASK | PRU_EVTOUT6_HOSTEN_MASK|PRU_EVTOUT7_HOSTEN_MASK) /*Enable PRU0/1, PRU_EVTOUT0/1/2 */ \
    }

#ifdef __cplusplus
}
#endif
