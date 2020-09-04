#include <stdio.h>
//! [include]
#include <drivers/fsi.h>
//! [include]
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>

/* FSI TXCLK - 50 MHz */
#define FSI_APP_TXCLK_FREQ              (50 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (500 * 1000 * 1000)
/* FSI TX prescaler value for TXCLKIN of 100 MHz. / 2 is provided as TXCLK = TXCLKIN/2 */
#define FSI_APP_TX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_TXCLK_FREQ / 2U)

#define FSI_APP_LOOP_COUNT              (100U)
/* User data to be sent with Data frame */
#define FSI_APP_TX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE    (16U)
/* 0x0U for 1 lane and 0x1U for two lane */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_TX_DATA_FRAME_TAG       (0x1U)

/* FSI RX Macros */
#define CONFIG_FSI_RX0_BASE_ADDR (CSL_FSIRX0_CFG_BASE)
#define CONFIG_FSI_RX0_INTR1 (16U)
#define CONFIG_FSI_RX0_INTR2 (17U)

/* FSI TX Macros */
#define CONFIG_FSI_TX0_BASE_ADDR (CSL_FSITX0_CFG_BASE)
#define CONFIG_FSI_TX0_INTR1 (28U)
#define CONFIG_FSI_TX0_INTR2 (28U)

/* Index of FSI TX/RX buffer, gBudIdx + FSI_APP_FRAME_DATA_WORD_SIZE should be <= 16 */
uint16_t gRxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
uint16_t gTxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];

static HwiP_Object gFsiTxHwiObject, gFsiRxHwiObject;
static SemaphoreP_Object gFsiTxSemObject, gFsiRxSemObject;


static int32_t Fsi_appTxConfig(uint32_t txBaseAddr)
{
    //! [FSI_tx_config]
    int32_t     status;

    /* TX init and reset */
    status = FSI_performTxInitialization(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    status += FSI_resetTxModule(txBaseAddr, FSI_TX_MASTER_CORE_RESET);
    FSI_clearTxModuleReset(txBaseAddr, FSI_TX_MASTER_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setTxSoftwareFrameSize(txBaseAddr, FSI_APP_FRAME_DATA_WORD_SIZE);
    status += FSI_setTxDataWidth(txBaseAddr, FSI_APP_N_LANES);

    /* Setting frame config */
    status += FSI_setTxUserDefinedData(txBaseAddr, FSI_APP_TX_USER_DATA);
    status += FSI_setTxFrameTag(txBaseAddr, FSI_APP_TX_DATA_FRAME_TAG);
    status += FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_NWORD_DATA);
    //! [FSI_tx_config]
    return status;
}

static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr)
{
    //! [FSI_rx_config]
    int32_t     status;

    /* RX init and reset */
    status  = FSI_performRxInitialization(rxBaseAddr);
    status += FSI_resetRxModule(rxBaseAddr, FSI_RX_MASTER_CORE_RESET);
    FSI_clearRxModuleReset(rxBaseAddr, FSI_RX_MASTER_CORE_RESET);

    /* Setting for requested transfer params */
    status += FSI_setRxSoftwareFrameSize(rxBaseAddr, FSI_APP_FRAME_DATA_WORD_SIZE);
    status += FSI_setRxDataWidth(rxBaseAddr, FSI_APP_N_LANES);
    status += FSI_setRxBufferPtr(rxBaseAddr, 0U);
    //! [FSI_rx_config]
    return status;
}

//! [FSI_interrupt]
static void Fsi_appTxCallback(void *args)
{
    
    uint32_t txBaseAddr = (uint32_t) args;

    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVT_FRAME_DONE);
    SemaphoreP_post(&gFsiTxSemObject);

    /* 
    ** Handle tx callback function here
    */
    return;
}

static void Fsi_appRxCallback(void *args)
{
    uint32_t rxBaseAddr = (uint32_t) args;

    FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVT_DATA_FRAME);
    SemaphoreP_post(&gFsiRxSemObject);

    /* 
    ** Handle rx callback function here
    */
    return;
}

static int32_t Fsi_appIntrInit(uint32_t txBaseAddr, uint32_t rxBaseAddr)
{
    int32_t     status;
    uint32_t    txIntrNum, rxIntrNum;
    HwiP_Params txHwiPrms, rxHwiPrms;

    /*
     * TX interrupt config and registration
     */
    txIntrNum = CONFIG_FSI_TX0_INTR1;
    status = SemaphoreP_constructBinary(&gFsiTxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    HwiP_Params_init(&txHwiPrms);
    txHwiPrms.intNum = txIntrNum;
    txHwiPrms.callback = Fsi_appTxCallback;
    txHwiPrms.args = (void *) txBaseAddr;
    HwiP_construct(&gFsiTxHwiObject, &txHwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable TX frame done interrupt */
    status += FSI_enableTxInterrupt(txBaseAddr, FSI_INT1, FSI_TX_EVT_FRAME_DONE);

    /*
     * RX interrupt config and registration
     */
    rxIntrNum = CONFIG_FSI_RX0_INTR1;
    status = SemaphoreP_constructBinary(&gFsiRxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    HwiP_Params_init(&rxHwiPrms);
    rxHwiPrms.intNum = rxIntrNum;
    rxHwiPrms.callback = Fsi_appRxCallback;
    rxHwiPrms.args = (void *) rxBaseAddr;
    HwiP_construct(&gFsiRxHwiObject, &rxHwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable RX frame done interrupt */
    status += FSI_enableRxInterrupt(rxBaseAddr, FSI_INT1, FSI_RX_EVT_DATA_FRAME);

    return status;
}

static void Fsi_appIntrDeInit(uint32_t txBaseAddr, uint32_t rxBaseAddr)
{
    /* TX interrupt deinit */
    FSI_disableTxInterrupt(txBaseAddr, FSI_INT1, FSI_TX_EVTMASK);
    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVTMASK);
    HwiP_destruct(&gFsiTxHwiObject);
    SemaphoreP_destruct(&gFsiTxSemObject);

    /* RX interrupt deinit */
    FSI_disableRxInterrupt(rxBaseAddr, FSI_INT1, FSI_TX_EVTMASK);
    FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVTMASK);
    HwiP_destruct(&gFsiRxHwiObject);
    SemaphoreP_destruct(&gFsiRxSemObject);
    return;
}
 //! [FSI_interrupt]
