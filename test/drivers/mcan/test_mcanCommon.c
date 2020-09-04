/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \file st_mcanCommon.c
 *
 *  \brief Common code that can be shared across test case files.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include "test_mcan.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** brief 32K Timer frequency */
#define APP_32K_TIMER_FREQ        (32000U)
/** brief Overflow count value for 32bit counter */
#define APP_COUNTER_MAX_COUNT     (0xFFFFFFFFU)
/** \brief Offset of DRM SUSPEND_CTRL31 register */
#define DRM_SUSPEND_CTRL31                 (0x27CU)
/** \brief DRM_SUSPEND_CTRL31 is mapped to COUNTER_32K Suspend Output line */
#define DRM_SUSPEND_CTRL_COUNTER_32K       (SOC_I_DRM_BASE + DRM_SUSPEND_CTRL31)
/** \brief DRM SUSPEND Source as M4 */
#define DRM_SUSPEND_SRC_IPU_C0             (0x3U)

/* Below macros are taken from mcan.c and are only needed for testing purpose */
/**
 * \brief  Mask and shift for Tx Buffers elements.
 */
#define MCANSS_TX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_TX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_TX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_TX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_TX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_TX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_TX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_TX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_TX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_TX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_TX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_TX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_TX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_TX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_TX_BUFFER_ELEM_EFC_SHIFT                          (23U)
#define MCANSS_TX_BUFFER_ELEM_EFC_MASK                           (0x00800000U)
#define MCANSS_TX_BUFFER_ELEM_MM_SHIFT                           (24U)
#define MCANSS_TX_BUFFER_ELEM_MM_MASK                            (0xFF000000U)

/**
 * \brief  Mask and shift for Rx Buffers elements.
 */
#define MCANSS_RX_BUFFER_ELEM_ID_SHIFT                           (0U)
#define MCANSS_RX_BUFFER_ELEM_ID_MASK                            (0x1FFFFFFFU)
#define MCANSS_RX_BUFFER_ELEM_RTR_SHIFT                          (29U)
#define MCANSS_RX_BUFFER_ELEM_RTR_MASK                           (0x20000000U)
#define MCANSS_RX_BUFFER_ELEM_XTD_SHIFT                          (30U)
#define MCANSS_RX_BUFFER_ELEM_XTD_MASK                           (0x40000000U)
#define MCANSS_RX_BUFFER_ELEM_ESI_SHIFT                          (31U)
#define MCANSS_RX_BUFFER_ELEM_ESI_MASK                           (0x80000000U)
#define MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT                         (0U)
#define MCANSS_RX_BUFFER_ELEM_RXTS_MASK                          (0x0000FFFFU)
#define MCANSS_RX_BUFFER_ELEM_DLC_SHIFT                          (16U)
#define MCANSS_RX_BUFFER_ELEM_DLC_MASK                           (0x000F0000U)
#define MCANSS_RX_BUFFER_ELEM_BRS_SHIFT                          (20U)
#define MCANSS_RX_BUFFER_ELEM_BRS_MASK                           (0x00100000U)
#define MCANSS_RX_BUFFER_ELEM_FDF_SHIFT                          (21U)
#define MCANSS_RX_BUFFER_ELEM_FDF_MASK                           (0x00200000U)
#define MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT                         (24U)
#define MCANSS_RX_BUFFER_ELEM_FIDX_MASK                          (0x7F000000U)
#define MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT                         (31U)
#define MCANSS_RX_BUFFER_ELEM_ANMF_MASK                          (0x80000000U)

/**
 * \brief  Mask and shift for Standard Message ID Filter Elements.
 */
#define MCANSS_STD_ID_FILTER_SFID2_SHIFT                         (0U)
#define MCANSS_STD_ID_FILTER_SFID2_MASK                          (0x000003FFU)
#define MCANSS_STD_ID_FILTER_SFID1_SHIFT                         (16U)
#define MCANSS_STD_ID_FILTER_SFID1_MASK                          (0x03FF0000U)
#define MCANSS_STD_ID_FILTER_SFEC_SHIFT                          (27U)
#define MCANSS_STD_ID_FILTER_SFEC_MASK                           (0x38000000U)
#define MCANSS_STD_ID_FILTER_SFT_SHIFT                           (30U)
#define MCANSS_STD_ID_FILTER_SFT_MASK                            (0xC0000000U)

/**
 * \brief  Extended Message ID Filter Element.
 */
#define MCANSS_EXT_ID_FILTER_EFID2_SHIFT                        (0U)
#define MCANSS_EXT_ID_FILTER_EFID2_MASK                         (0x1FFFFFFFU)
#define MCANSS_EXT_ID_FILTER_EFID1_SHIFT                        (0U)
#define MCANSS_EXT_ID_FILTER_EFID1_MASK                         (0x1FFFFFFFU)
#define MCANSS_EXT_ID_FILTER_EFEC_SHIFT                         (29U)
#define MCANSS_EXT_ID_FILTER_EFEC_MASK                          (0xE0000000U)
#define MCANSS_EXT_ID_FILTER_EFT_SHIFT                          (30U)
#define MCANSS_EXT_ID_FILTER_EFT_MASK                           (0xC0000000U)

/**
 * \brief  Mask and shift for Tx Event FIFO elements.
 */
#define MCANSS_TX_EVENT_FIFO_ELEM_ID_SHIFT                      (0U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ID_MASK                       (0x1FFFFFFFU)
#define MCANSS_TX_EVENT_FIFO_ELEM_RTR_SHIFT                     (29U)
#define MCANSS_TX_EVENT_FIFO_ELEM_RTR_MASK                      (0x20000000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_XTD_SHIFT                     (30U)
#define MCANSS_TX_EVENT_FIFO_ELEM_XTD_MASK                      (0x40000000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ESI_SHIFT                     (31U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ESI_MASK                      (0x80000000U)

#define MCANSS_TX_EVENT_FIFO_ELEM_TXTS_SHIFT                    (0U)
#define MCANSS_TX_EVENT_FIFO_ELEM_TXTS_MASK                     (0x0000FFFFU)
#define MCANSS_TX_EVENT_FIFO_ELEM_DLC_SHIFT                     (16U)
#define MCANSS_TX_EVENT_FIFO_ELEM_DLC_MASK                      (0x000F0000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_BRS_SHIFT                     (20U)
#define MCANSS_TX_EVENT_FIFO_ELEM_BRS_MASK                      (0x00100000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_FDF_SHIFT                     (21U)
#define MCANSS_TX_EVENT_FIFO_ELEM_FDF_MASK                      (0x00200000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ET_SHIFT                      (22U)
#define MCANSS_TX_EVENT_FIFO_ELEM_ET_MASK                       (0x00C00000U)
#define MCANSS_TX_EVENT_FIFO_ELEM_MM_SHIFT                      (24U)
#define MCANSS_TX_EVENT_FIFO_ELEM_MM_MASK                       (0xFF000000U)

/* Print buffer character limit for prints- UART or CCS Console */
#define APP_PRINT_BUFFER_SIZE                       (4000)

#if defined (SOC_AM263X)
#define CONFIG_MCAN_TS_INTRNUM                      (CSLR_R5FSS0_CORE0_INTR_MCAN0_EXT_TS_ROLLOVER_LVL_INT_0)
#elif defined (SOC_AM62X)
#define CONFIG_MCAN_TS_INTRNUM                      (CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCAN0_COMMON_0_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0)
#else
#define CONFIG_MCAN_TS_INTRNUM                      (CSLR_R5FSS0_CORE0_INTR_MCAN0_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0)
#endif
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
extern SemaphoreP_Object   gTxDoneSem;
extern SemaphoreP_Object   gRxDoneSem;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This API is used to introduce error.
 *
 * \param   testParams        structure to details of testcase to run.
 *
 * \retvsl  status            Run status
 */
int32_t mcanTestFunc(st_mcanTestcaseParams_t *testParams);

/**
 * \brief   This function will configure X-BAR for MCAN interrupts
 *
 * \param   none.
 *
 * \retval  status      configuration status.
 */
int32_t App_mcanRegisterInterrupt();
int32_t App_mcanUnRegisterInterrupt();

/**
 * \brief   This is Interrupt Service Routine for MCAN interrupt 0.
 *
 * \param   none.
 *
 * \retval  none.
 */
void App_mcanIntr0ISR(void *arg);

/**
 * \brief   This is Interrupt Service Routine for MCAN interrupt 1.
 *
 * \param   none.
 *
 * \retval  none.
 */
void App_mcanIntr1ISR(void *arg);

/**
 * \brief   This is Interrupt Service Routine for MCAN ECC interrupt.
 *
 * \param   none.
 *
 * \retval  none.
 */
void App_mcanECCIntrISR(void *handle);

/**
 * \brief   This is Interrupt Service Routine for MCAN TimeStamp interrupt.
 *
 * \param   none.
 *
 * \retval  none.
 */
void App_mcanTSIntrISR(void *arg);

/**
 * \brief   This API will index of the current bit timing configuration within
 *          'canFDBitTimings'.
 *
 * \param   rxMsg           received message object.
 *
 * \retval  index           Array index within 'canFDBitTimings'.
 *                          Return 0xFFFFFFFF if not found.
 */
uint32_t App_getBitConfigParamId(const MCAN_BitTimingParams *bitTimings);

/**
 * \brief   This function used to insert delays
 *
 * \param   timeout         Delay value in ms.
 *
 * \retval  none.
 */
void App_delayFunc(uint32_t timeout);

/**
 * \brief   This function used to insert delays
 *
 * \param   value           Initial Value.
 * \param   elapsedValue    Number of clock ticks since 'value'.
 *
 * \retval  none.
 */
void App_mcanGetIntStatus(uint32_t baseAddr);

/**
 * \brief   This API will write the message object to Message RAM.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_TxBufElement.
 *
 * \return  None.
 */
void App_mcanWriteMsg(uint32_t                 baseAddr,
                      uint32_t                 elemAddr,
                      const MCAN_TxBufElement *elem);

/**
 * \brief   This API will read the message object from Message RAM.
 *
 * \param   baseAddr        Base Address of the MCAN Registers.
 * \param   elemAddr        Address of the message object.
 * \param   elem            Message Object.
 *                          Refer struct #MCAN_RxBufElement.
 *
 * \return  None.
 */
void App_mcanReadMsg(uint32_t           baseAddr,
                     uint32_t           elemAddr,
                     MCAN_RxBufElement *elem);

/**
 * \brief   This API will return payload depending on 'dlc'  field.
 *
 * \param   dlc             Data Length Code.
 *
 * \return  data size       Size of the payload.
 */
uint32_t App_mcangetMsgObjSize(uint32_t elemSize);

/**
 * \brief   This API will return message object size.
 *
 * \param   dlc             Data Length Code.
 *
 * \return  message object size
 *                          Size of the message object stored in Message RAM.
 */
uint32_t App_mcanGetDataSize(uint32_t dlc);

static int32_t App_mcanRegIntrInternal(void);
static int32_t App_mcanConfigureIrqRouter (uint32_t inputIntrNum, uint32_t outputIntrNum);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint32_t          gMcanBaseAddr = DEF_MCAN_MODULE;
uint32_t          gMcanAppdataSize[16] =
{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
volatile uint32_t gMcanECCIntrFlag = 1U;
volatile uint32_t gMcanIsrIntr0Status = 0U;
volatile uint32_t gMcanIsrIntr1Status = 0U;
volatile uint32_t gMcanExtTSIntrFlag = 1U;
volatile uint32_t gMcanModuleIdx = 0U;
extern volatile uint32_t isrPrintEnable;
HwiP_Object       gMcanHwiObject, gMcanHwiObject1, gMcanHwiObject2, gMcanHwiObject3;
HwiP_Object       gMcanHwiObject4, gMcanHwiObject5;
MCAN_ECCErrStatus gMcaneccErr;
extern MCAN_BitTimingParams canFDBitTimings[];
extern uint32_t bitTimingsListSize;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t App_mcanRegisterInterrupt(uint32_t modIdx)
{
    int32_t           status = CSL_PASS;
    HwiP_Params       hwiPrms;

    /* Register MCAN0 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN0_INTR;
    hwiPrms.callback    = &App_mcanIntr0ISR;
    status              = HwiP_construct(&gMcanHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Register MCAN0 Line 1 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN0_INTR + 1U;
    hwiPrms.callback    = &App_mcanIntr1ISR;
    status              = HwiP_construct(&gMcanHwiObject3, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Register MCAN1 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN1_INTR;
    hwiPrms.callback    = &App_mcanIntr0ISR;
    status              = HwiP_construct(&gMcanHwiObject1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

#if defined (SOC_AM263X)
    /* Register MCAN0 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN2_INTR;
    hwiPrms.callback    = &App_mcanIntr0ISR;
    status              = HwiP_construct(&gMcanHwiObject4, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Register MCAN0 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN3_INTR;
    hwiPrms.callback    = &App_mcanIntr0ISR;
    status              = HwiP_construct(&gMcanHwiObject5, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

#endif

    /* Register TS interrupt */
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    /* MCAN External TS Interrupt is registerd in ESM Module */

#else
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_MCAN_TS_INTRNUM;
    hwiPrms.callback    = &App_mcanTSIntrISR;
    status              = HwiP_construct(&gMcanHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
    return status;
}

int32_t App_mcanUnRegisterInterrupt(uint32_t modIdx)
{
    int32_t           status = CSL_PASS;

    /* De-Construct Tx/Rx Semaphore objects */
    HwiP_destruct(&gMcanHwiObject);
    HwiP_destruct(&gMcanHwiObject1);
    HwiP_destruct(&gMcanHwiObject2);
    HwiP_destruct(&gMcanHwiObject3);
#if defined (SOC_AM263X)
    HwiP_destruct(&gMcanHwiObject4);
    HwiP_destruct(&gMcanHwiObject5);
#endif

    return status;
}

void App_mcanIntr0ISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus);
    gMcanIsrIntr0Status = (intrStatus &
                          (~MCAN_getIntrLineSelectStatus(gMcanBaseAddr)));
    if(isrPrintEnable == (uint32_t)TRUE)
    {
        DebugP_log("\nInterrupt Status: 0x%x.\n", intrStatus);
    }
    if ((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) == MCAN_INTR_SRC_TRANS_COMPLETE)
    {
        SemaphoreP_post(&gTxDoneSem);
    }

    uint32_t rxIntrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG |
         MCAN_INTR_SRC_RX_FIFO0_NEW_MSG |
         MCAN_INTR_SRC_RX_FIFO1_NEW_MSG |
         MCAN_INTR_SRC_HIGH_PRIO_MSG;
    if ((gMcanIsrIntr0Status & rxIntrMask) != 0U)
    {
        /* If any of the Rx interrupts are set,
         * Post the RxSem
         */
        SemaphoreP_post(&gRxDoneSem);
    }
}

void App_mcanIntr1ISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus);
    gMcanIsrIntr0Status = (intrStatus &
                          (MCAN_getIntrLineSelectStatus(gMcanBaseAddr)));
    if(isrPrintEnable == (uint32_t)TRUE)
    {
        DebugP_log("\nInterrupt Status: 0x%x.\n", intrStatus);
    }
    if ((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) == MCAN_INTR_SRC_TRANS_COMPLETE)
    {
        SemaphoreP_post(&gTxDoneSem);
    }

    uint32_t rxIntrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG |
         MCAN_INTR_SRC_RX_FIFO0_NEW_MSG |
         MCAN_INTR_SRC_RX_FIFO1_NEW_MSG |
         MCAN_INTR_SRC_HIGH_PRIO_MSG;
    if ((gMcanIsrIntr0Status & rxIntrMask) != 0U)
    {
        /* If any of the Rx interrupts are set,
         * Post the RxSem
         */
        SemaphoreP_post(&gRxDoneSem);
    }
}

#if defined (SOC_AM263X)
void App_mcanIntr2ISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus);
    gMcanIsrIntr0Status = (intrStatus &
                          (MCAN_getIntrLineSelectStatus(gMcanBaseAddr)));
    if(isrPrintEnable == (uint32_t)TRUE)
    {
        DebugP_log("\nInterrupt Status: 0x%x.\n", intrStatus);
    }
    if ((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) == MCAN_INTR_SRC_TRANS_COMPLETE)
    {
        SemaphoreP_post(&gTxDoneSem);
    }

    uint32_t rxIntrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG |
         MCAN_INTR_SRC_RX_FIFO0_NEW_MSG |
         MCAN_INTR_SRC_RX_FIFO1_NEW_MSG |
         MCAN_INTR_SRC_HIGH_PRIO_MSG;
    if ((gMcanIsrIntr0Status & rxIntrMask) != 0U)
    {
        /* If any of the Rx interrupts are set,
         * Post the RxSem
         */
        SemaphoreP_post(&gRxDoneSem);
    }
}

void App_mcanIntr3ISR(void *arg)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(gMcanBaseAddr);
    MCAN_clearIntrStatus(gMcanBaseAddr, intrStatus);
    gMcanIsrIntr0Status = (intrStatus &
                          (MCAN_getIntrLineSelectStatus(gMcanBaseAddr)));
    if(isrPrintEnable == (uint32_t)TRUE)
    {
        DebugP_log("\nInterrupt Status: 0x%x.\n", intrStatus);
    }
    if ((gMcanIsrIntr0Status & MCAN_INTR_SRC_TRANS_COMPLETE) == MCAN_INTR_SRC_TRANS_COMPLETE)
    {
        SemaphoreP_post(&gTxDoneSem);
    }

    uint32_t rxIntrMask = MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG |
         MCAN_INTR_SRC_RX_FIFO0_NEW_MSG |
         MCAN_INTR_SRC_RX_FIFO1_NEW_MSG |
         MCAN_INTR_SRC_HIGH_PRIO_MSG;
    if ((gMcanIsrIntr0Status & rxIntrMask) != 0U)
    {
        /* If any of the Rx interrupts are set,
         * Post the RxSem
         */
        SemaphoreP_post(&gRxDoneSem);
    }
}

#endif

void App_mcanECCIntrISR(void *arg)
{
    MCAN_ECCErrStatus mcanECCErr;

    MCAN_eccGetErrorStatus(gMcanBaseAddr, &gMcaneccErr);
    if (MCAN_eccGetIntrStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC))
    {
        MCAN_eccClearIntrStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC);
        if(isrPrintEnable == (uint32_t)TRUE)
        {
            DebugP_log("ECC SEC interrupt happened.\n");
        }
    }
    if (MCAN_eccGetIntrStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED))
    {
        MCAN_eccClearIntrStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED);
        if(isrPrintEnable == (uint32_t)TRUE)
        {
            DebugP_log("ECC DED interrupt happened.\n");
        }
    }
    if (1U == gMcaneccErr.secErr)
    {
        MCAN_eccClearErrorStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC);
        MCAN_eccGetErrorStatus(gMcanBaseAddr, &mcanECCErr);
        while (mcanECCErr.secErr == 1U)
        {
            MCAN_eccGetErrorStatus(gMcanBaseAddr, &mcanECCErr);
        }
        MCAN_eccWriteEOI(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_SEC);
    }
    if (1U == gMcaneccErr.dedErr)
    {
        MCAN_eccClearErrorStatus(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED);
        MCAN_eccGetErrorStatus(gMcanBaseAddr, &mcanECCErr);
        while (mcanECCErr.dedErr == 1U)
        {
            MCAN_eccGetErrorStatus(gMcanBaseAddr, &mcanECCErr);
        }
        MCAN_eccWriteEOI(gMcanBaseAddr, MCAN_ECC_ERR_TYPE_DED);
    }
    gMcanECCIntrFlag = 0U;
}

void App_mcanTSIntrISR(void *arg)
{
    if(isrPrintEnable == (uint32_t)TRUE)
    {
        if (MCAN_extTSIsIntrEnable(gMcanBaseAddr) == (uint32_t)TRUE)
        {
            DebugP_log("Time Stamp overflow happened.\r\n");
            DebugP_log("Unserviced Interrupt Count: %d\r\n", MCAN_extTSGetUnservicedIntrCount(gMcanBaseAddr));
        }
    }
    MCAN_extTSClearRawStatus(gMcanBaseAddr);
    gMcanExtTSIntrFlag = 0U;
    MCAN_extTSWriteEOI(gMcanBaseAddr);
}

uint32_t App_getBitConfigParamId(const MCAN_BitTimingParams *bitTimings)
{
    uint32_t loopCnt;

    for(loopCnt = 0U ; loopCnt < bitTimingsListSize ; loopCnt++)
    {
        if ((canFDBitTimings[loopCnt].nomRatePrescalar == bitTimings->nomRatePrescalar) &&
            (canFDBitTimings[loopCnt].nomTimeSeg1 == bitTimings->nomTimeSeg1) &&
            (canFDBitTimings[loopCnt].nomTimeSeg2 == bitTimings->nomTimeSeg2) &&
            (canFDBitTimings[loopCnt].nomSynchJumpWidth == bitTimings->nomSynchJumpWidth) &&
            (canFDBitTimings[loopCnt].dataRatePrescalar == bitTimings->dataRatePrescalar) &&
            (canFDBitTimings[loopCnt].dataTimeSeg1 == bitTimings->dataTimeSeg1) &&
            (canFDBitTimings[loopCnt].dataTimeSeg2 == bitTimings->dataTimeSeg2) &&
            (canFDBitTimings[loopCnt].dataSynchJumpWidth == bitTimings->dataSynchJumpWidth))
        {
            break;
        }
    }
    if(loopCnt == bitTimingsListSize)
    {
        loopCnt = 0xFFFFFFFFU;
    }

    return loopCnt;
}

void App_delayFunc(uint32_t timeout)
{
}

void App_mcanGetIntStatus(uint32_t baseAddr)
{
    uint32_t intrStatus;

    intrStatus = MCAN_getIntrStatus(baseAddr);
    MCAN_clearIntrStatus(baseAddr, intrStatus);
    gMcanIsrIntr0Status = (intrStatus &
                          (~MCAN_getIntrLineSelectStatus(baseAddr)));
    if(isrPrintEnable == (uint32_t)TRUE)
    {
        DebugP_log("\nInterrupt Status: 0x%x.\n", intrStatus);
    }
}

void App_mcanReadMsg(uint32_t           baseAddr,
                     uint32_t           elemAddr,
                     MCAN_RxBufElement *elem)
{
    uint32_t regVal = 0U, loopCnt = 0U;

    regVal   = HW_RD_REG32(baseAddr + elemAddr);
    elem->id = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ID_MASK)
                           >> MCANSS_RX_BUFFER_ELEM_ID_SHIFT);
    elem->rtr = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RTR_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_RTR_SHIFT);
    elem->xtd = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_XTD_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_XTD_SHIFT);
    elem->esi = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ESI_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_ESI_SHIFT);

    elemAddr  += 4U;
    regVal     = HW_RD_REG32(baseAddr + elemAddr);
    elem->rxts = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_RXTS_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_RXTS_SHIFT);
    elem->dlc = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_DLC_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_DLC_SHIFT);
    elem->brs = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_BRS_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_BRS_SHIFT);
    elem->fdf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FDF_MASK)
                            >> MCANSS_RX_BUFFER_ELEM_FDF_SHIFT);
    elem->fidx = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_FIDX_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_FIDX_SHIFT);
    elem->anmf = (uint32_t) ((regVal & MCANSS_RX_BUFFER_ELEM_ANMF_MASK)
                             >> MCANSS_RX_BUFFER_ELEM_ANMF_SHIFT);
    elemAddr += 4U;

    loopCnt = 0U;
    /* Reading words from message RAM and forming payload bytes out of it */
    while ((4U <= (App_mcanGetDataSize(elem->dlc) - loopCnt)) &&
           (0U != (App_mcanGetDataSize(elem->dlc) - loopCnt)))
    {
        regVal = HW_RD_REG32(baseAddr + elemAddr);
        elem->data[loopCnt]       = (uint8_t)(regVal & 0x000000FFU);
        elem->data[(loopCnt + 1U)] = (uint8_t)((regVal & 0x0000FF00U) >> 8U);
        elem->data[(loopCnt + 2U)] = (uint8_t)((regVal & 0x00FF0000U) >> 16U);
        elem->data[(loopCnt + 3U)] = (uint8_t)((regVal & 0xFF000000U) >> 24U);
        elemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Reading remaining bytes from message RAM */
    if (0U < (App_mcanGetDataSize(elem->dlc) - loopCnt))
    {
        regVal = HW_RD_REG32(baseAddr + elemAddr);
        elem->data[loopCnt]       = (uint8_t)(regVal & 0x000000FFU);
        elem->data[(loopCnt + 1U)] = (uint8_t)((regVal & 0x0000FF00U) >> 8U);
        elem->data[(loopCnt + 2U)] = (uint8_t)((regVal & 0x00FF0000U) >> 16U);
    }
}

void App_mcanWriteMsg(uint32_t                 baseAddr,
                      uint32_t                 elemAddr,
                      const MCAN_TxBufElement *elem)
{
    uint32_t regVal = 0, loopCnt = 0U;

    regVal  = 0U;
    regVal |= (((uint32_t) (elem->id << MCANSS_TX_BUFFER_ELEM_ID_SHIFT)) |
               ((uint32_t) (elem->rtr << MCANSS_TX_BUFFER_ELEM_RTR_SHIFT)) |
               ((uint32_t) (elem->xtd << MCANSS_TX_BUFFER_ELEM_XTD_SHIFT)) |
               ((uint32_t) (elem->esi << MCANSS_TX_BUFFER_ELEM_ESI_SHIFT)));
    HW_WR_REG32(baseAddr + elemAddr, regVal);
    elemAddr += 4U;

    regVal  = 0U;
    regVal |= ((uint32_t) (elem->dlc << MCANSS_TX_BUFFER_ELEM_DLC_SHIFT)) |
              ((uint32_t) (elem->brs << MCANSS_TX_BUFFER_ELEM_BRS_SHIFT)) |
              ((uint32_t) (elem->fdf << MCANSS_TX_BUFFER_ELEM_FDF_SHIFT)) |
              ((uint32_t) (elem->efc << MCANSS_TX_BUFFER_ELEM_EFC_SHIFT)) |
              ((uint32_t) (elem->mm << MCANSS_TX_BUFFER_ELEM_MM_SHIFT));
    HW_WR_REG32(baseAddr + elemAddr, regVal);
    elemAddr += 4U;

    loopCnt = 0U;
    /* Framing words out of the payload bytes and writing it to message RAM */
    while ((4U <= (App_mcanGetDataSize(elem->dlc) - loopCnt)) &&
           (0U != (App_mcanGetDataSize(elem->dlc) - loopCnt)))
    {
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + elemAddr, regVal);
        elemAddr += 4U;
        loopCnt  += 4U;
    }
    /* Framing a word out of remaining payload bytes and writing it to
     * message RAM */
    if (0U < (App_mcanGetDataSize(elem->dlc) - loopCnt))
    {
        regVal  = 0U;
        regVal |= ((uint32_t)elem->data[loopCnt] |
                   ((uint32_t)elem->data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t)elem->data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t)elem->data[(loopCnt + 3U)] << 24U));
        HW_WR_REG32(baseAddr + elemAddr, regVal);
    }
}

uint32_t App_mcanGetDataSize(uint32_t dlc)
{
    uint32_t dataSize[16] = {0,  1,  2,  3,  4,  5,  6, 7, 8,
                             12, 16, 20, 24, 32, 48, 64};

    return (dataSize[dlc]);
}

uint32_t App_mcangetMsgObjSize(uint32_t elemSize)
{
    uint32_t objSize[8] = {4, 5, 6, 7, 8, 10, 14, 18};

    return (objSize[elemSize]);
}
