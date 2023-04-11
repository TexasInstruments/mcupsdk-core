/*!
* \file pru_IOLink_internal.h
*
* \brief
* PRU Integration: IOLink internal interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-20
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined __PRU_IOLINK_INTERNAL_H__)
#define __PRU_IOLINK_INTERNAL_H__		1

#include <stdint.h>
#include <stdbool.h>

#include <osal.h>
#include <IOLM.h>
#include "IOLM_Phy.h"

#if (defined OSAL_LINUX) || (defined OSAL_TIRTOS)
#include <ti/drv/pruss/pruicss.h>
#elif (defined OSAL_FREERTOS)
#include <FreeRTOS.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/pruicss.h>
#else
#error "Unknown OS"
#endif

// PRU Description
#if (IOLM_PORT_COUNT != 8 && IOLM_PORT_COUNT != 16)
#error PORT_COUNT is not supported by Sitara PRU PL
#endif

#define IOLM_PRU_MAX_INST   1
#define IOLM_PRU_MAX_PORTS  8

/* status definition */
#define IOLINK_PRU_STATUS_IDLE             (0x0U)
#define IOLINK_PRU_STATUS_COMPLETE         (0x1U)

/* LUT's of PRU 0 */
#define pru0LutAdr          (0x0E00U)
#define sampleratelutAdr    (0x0E00U)
#define startbitfilterAdr   (0x0F00U)
/* LUT's of PRU 1 */
#define pru1LutAdr          (0x0E00U)
#define averagingfilterAdr  (0x0E00U)
#define paritylutAdr        (0x0F00U)

#define PRU_IOL_COMPLETE_EVENT  16
#define PRU_IOL_STARTUP_EVENT   17
#define PRU_IOL_TIMER_EVENT     x

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
{ PRU_IOL_COMPLETE_EVENT,PRU_IOL_STARTUP_EVENT,0xFF},  \
{ {PRU_IOL_COMPLETE_EVENT ,CHANNEL3, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},\
  {PRU_IOL_STARTUP_EVENT ,CHANNEL2, SYS_EVT_POLARITY_HIGH ,SYS_EVT_TYPE_PULSE},\
  {0xFF,0xFF,0xFF,0xFF}},  \
{ {CHANNEL0,PRU0}, {CHANNEL1, PRU1}, {CHANNEL2, PRU_EVTOUT0}, {CHANNEL3, PRU_EVTOUT1}, \
  {CHANNEL4, PRU_EVTOUT2}, {CHANNEL5, PRU_EVTOUT3},{CHANNEL6, PRU_EVTOUT4},{CHANNEL7, PRU_EVTOUT6}, {CHANNEL8, PRU_EVTOUT7}\
  , {0xFF,0xFF} },  \
  (PRU0_HOSTEN_MASK | PRU1_HOSTEN_MASK | PRU_EVTOUT0_HOSTEN_MASK | PRU_EVTOUT1_HOSTEN_MASK \
  | PRU_EVTOUT2_HOSTEN_MASK | PRU_EVTOUT3_HOSTEN_MASK| PRU_EVTOUT4_HOSTEN_MASK| PRU_EVTOUT5_HOSTEN_MASK | PRU_EVTOUT6_HOSTEN_MASK|PRU_EVTOUT7_HOSTEN_MASK) /*Enable PRU0/1, PRU_EVTOUT0/1/2 */ \
}

#if (defined OSAL_FREERTOS) && (OSAL_FREERTOS == 1)
#define PRU_IOLM_TASK_SIZE_DIVIDER  sizeof(configSTACK_DEPTH_TYPE)
#define PRU_IOLM_TASK_STACK_TYPE    StackType_t
#else
#define PRU_IOLM_TASK_SIZE_DIVIDER  sizeof(uint32_t)
#define PRU_IOLM_TASK_STACK_TYPE    uint8_t
#endif

#define PRU_IOLM_TIMER_TASK_SIZE    (0x2000U / PRU_IOLM_TASK_SIZE_DIVIDER)

static PRU_IOLM_TASK_STACK_TYPE     IOLM_timerTaskStack_s[PRU_IOLM_TIMER_TASK_SIZE] \
                                        __attribute__((aligned(32), section(".threadstack"))) = {0};

// PRU Register description
#pragma pack(push, 1)
typedef struct IOLM_PL_SPruChan
{
    volatile uint32_t ctrl;
    volatile uint16_t status;
    volatile uint16_t ereror;
}OSAL_STRUCT_PACKED IOLM_PL_SPruChan_t;

typedef struct IOLM_PL_SpruChanMem
{
    volatile uint32_t unknown;
    volatile uint32_t txpin;
    volatile uint32_t engpioaddr;
    volatile uint32_t engpiopin;
}OSAL_STRUCT_PACKED IOLM_PL_SpruChanMem_t;

typedef struct IOLM_PL_SPruMem
{
    volatile uint8_t status;
    volatile uint8_t control;
    volatile uint8_t maxRespTime;
    volatile uint8_t fwRev;

    volatile IOLM_PL_SPruChan_t aChanCtrl[8];

    volatile uint8_t aReserved[0xBC];

    volatile uint8_t aaRxBuffer[8][128];
    volatile uint8_t aaTxBuffer[8][256];

    volatile IOLM_PL_SpruChanMem_t channelmem[8];

    volatile uint8_t aReserved2[128];

    volatile uint8_t aLut[0x200];

}OSAL_STRUCT_PACKED IOLM_PL_SPruMem_t;

typedef struct IOLM_PL_SPruMemTimer
{
    volatile uint8_t timerconfig;
    volatile uint8_t wakeconfig;
    volatile uint8_t wakestatus;
    volatile uint8_t reserved;
    volatile uint32_t channelTimer[8];
    volatile uint32_t channelWake[8];
}OSAL_STRUCT_PACKED IOLM_PL_SPruMemTimer_t;
#pragma pack(pop)

typedef enum IOLM_PL_EPortState {
    IOLM_PL_PortOff = 0,
    IOLM_PL_PortOn = 1
} IOLM_PL_EPortState_t;

/* Port Specific PRU data */
typedef struct IOLM_PL_PortInst
{
    uint8_t portNumber;
    IOLM_PL_EPortState_t state;
    IOLM_PL_ePortMode_t mode;

    uint32_t  txDataLength;
    uint32_t  rxDataLength;
    uint8_t* pRxBuffer;
    uint8_t awaitingRsp;
    bool cycleTimerActive;
    bool pinMuxPending;
#if IOLM_PHY_FRAMEHANDLER == 1
    bool startupActive;
#endif
} IOLM_PL_PortInst_t;

/* General PRU data */
typedef struct IOLM_PL_Inst
{
    PRUICSS_Handle pruHandle;

    const IOLM_PL_sPruIccsCfg_t *pPruIccsCfg;

    volatile IOLM_PL_SPruMem_t* pruDram0;
    volatile IOLM_PL_SPruMemTimer_t* pruDram1;
    volatile uint8_t* pruBase;
    volatile uint8_t* pruIntc;

    IOLM_PL_PortInst_t port[IOLM_PRU_MAX_PORTS];
} IOLM_PL_Inst_t;

/* Port specific data, independent from PRU */
typedef struct IOLM_PL_GeneralPort
{
    uint8_t portNumber;
    IOLM_PL_PortInst_t *pPruPort;
    OSAL_TIMER_SHandle_t* timer;
}IOLM_PL_GeneralPort_t;

/* General data, independent from PRU */
typedef struct IOLM_PL_General
{
    void* pTimerTask;
    PRU_IOLM_TASK_STACK_TYPE* pTimerTaskStack;
    OSAL_CMDQUEUE_SHandle_t* pTimerQueue;

    IOLM_PL_GeneralPort_t port[IOLM_PORT_COUNT];
}IOLM_PL_General_t;

#endif /* __PRU_IOLINK_INTERNAL_H__ */
