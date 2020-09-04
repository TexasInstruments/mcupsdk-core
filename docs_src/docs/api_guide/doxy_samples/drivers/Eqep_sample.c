
#include <stdio.h>
//! [include]
#include <drivers/eqep.h>
//! [include]
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_soc.h>

/* EQEP Interrupt Sources */
#define EQEP_INT_ALL            (EQEP_INT_GLOBAL            | \
                                 EQEP_INT_POS_CNT_ERROR     | \
                                 EQEP_INT_PHASE_ERROR       | \
                                 EQEP_INT_DIR_CHANGE        | \
                                 EQEP_INT_WATCHDOG          | \
                                 EQEP_INT_UNDERFLOW         | \
                                 EQEP_INT_OVERFLOW          | \
                                 EQEP_INT_POS_COMP_READY    | \
                                 EQEP_INT_POS_COMP_MATCH    | \
                                 EQEP_INT_STROBE_EVNT_LATCH | \
                                 EQEP_INT_INDEX_EVNT_LATCH  | \
                                 EQEP_INT_UNIT_TIME_OUT     | \
                                 EQEP_INT_QMA_ERROR)

/* Number of EQEP EVENTS */
#define EQEP_EVENT_CNT          (10U)

/* Frequency of EQEP signal for testing unit timeout event in Hz. */
#define EQEP_SIGNAL_TEST_FREQ               (500U)

/* Frequence of unit timeout event in Hz.
 * Should be less than the EQEP signal frequency to capture multiple clocks at
 * Timeout event. So configuring it to 1/10th the EQEP signal frequency. */
#define EQEP_UNIT_TIMEOUT_FREQ              (EQEP_SIGNAL_TEST_FREQ / EQEP_EVENT_CNT)

/* Number of times the EQEP pattern is generated. */
#define EQEP_PATTERN_GEN_LOOP_COUNT         (EQEP_SIGNAL_TEST_FREQ / EQEP_EVENT_CNT)

/* EQEP Instance Macros */
#define CONFIG_EQEP0_BASE_ADDR (CSL_EQEP0_REG_BASE)
#define CONFIG_EQEP0_FCLK (125000000)
#define CONFIG_EQEP0_INTR (143)
#define CONFIG_EQEP0_INTR_IS_PULSE (TRUE)

/* Global variables and objects */
static HwiP_Object           gEqepHwiObject;
/* Variable to hold base address of EQEP/GPIO that is used */
uint32_t                     gEqepBaseAddr;
/* EQEP interrupt handle. */
static SemaphoreP_Object     gEqepSyncSem;

/* Isr count at different events. */
volatile uint32_t            gEqepIsrCnt = 0U;
/* Pos Count capture at different events. */
uint32_t                     gEqepPosCnt[EQEP_EVENT_CNT];

/* Static Function Declarations */
static void App_eqepIntrISR(void *arg);

static int32_t App_eqepIntrReg()
{
    //! [App_eqepIntrReg]
    int32_t             status = SystemP_SUCCESS;
    HwiP_Params         hwiPrms;

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EQEP0_INTR;
    hwiPrms.callback    = &App_eqepIntrISR;
    hwiPrms.isPulse     = CONFIG_EQEP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEqepHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    //! [App_eqepIntrReg]

    return status;
}

static void App_eqepConfigPositionMeasurementMode(void)
{
    //! [App_eqepConfigPositionMeasurementMode]
    /* Configure the decoder for quadrature mode, counting rising edge
      (that is, 1x resolution) */
    EQEP_setDecoderConfig(gEqepBaseAddr, (EQEP_CONFIG_1X_RESOLUTION |
                                        EQEP_CONFIG_QUADRATURE |
                                        EQEP_CONFIG_NO_SWAP));

    /* Configure the position counter to reset on an index event */
    EQEP_setPositionCounterConfig(gEqepBaseAddr, EQEP_POSITION_RESET_IDX,
                                  CSL_EQEP_QPOSCNT_QPOSCNT_MAX);

    /* Configure the position counter to be latched on rising edge index */
    EQEP_setLatchMode(gEqepBaseAddr, EQEP_LATCH_RISING_INDEX);

    /* Enable EQEP Module and Interrupt */
    EQEP_enableModule(gEqepBaseAddr);
    EQEP_enableInterrupt(gEqepBaseAddr, EQEP_INT_INDEX_EVNT_LATCH);
    //! [App_eqepConfigPositionMeasurementMode]
}

static void App_eqepConfigFrequencyMeasurementMode(void)
{
    //! [App_eqepConfigFrequencyMeasurementMode]
    /* Configure the decoder for up-count mode, counting both rising and
       falling edges (that is, 2x resolution) */
    EQEP_setDecoderConfig(gEqepBaseAddr, (EQEP_CONFIG_2X_RESOLUTION |
                                          EQEP_CONFIG_UP_COUNT |
                                          EQEP_CONFIG_NO_SWAP));

    /* Configure the position counter to reset on an unit timeout event */
    EQEP_setPositionCounterConfig(gEqepBaseAddr,
                                  EQEP_POSITION_RESET_UNIT_TIME_OUT,
                                  CSL_EQEP_QPOSCNT_QPOSCNT_MAX);

    /* Enable the unit timer, setting the frequency to EQEP_UNIT_TIMEOUT_FREQ */
    EQEP_enableUnitTimer(gEqepBaseAddr, (CONFIG_EQEP0_FCLK / (EQEP_UNIT_TIMEOUT_FREQ)));

    /* Configure the position counter to be latched on a unit time out */
    EQEP_setLatchMode(gEqepBaseAddr, EQEP_LATCH_UNIT_TIME_OUT);

    /* Enable the EQEP module */
    EQEP_enableModule(gEqepBaseAddr);

    /* Configure and enable the edge-capture unit. The capture clock divider is
       SYSCLKOUT/128. The unit-position event divider is QCLK/8. */
    EQEP_setCaptureConfig(gEqepBaseAddr, EQEP_CAPTURE_CLK_DIV_128,
                          EQEP_UNIT_POS_EVNT_DIV_8);
    EQEP_enableCapture(gEqepBaseAddr);

    /* Enable unit timeout interrupt. */
    EQEP_enableInterrupt(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
    //! [App_eqepConfigFrequencyMeasurementMode]
}

static void App_eqepIntrISR(void *arg)
{
    uint16_t intEnabled, intStatus;
    //! [App_eqepIntrISR]
    intStatus = EQEP_getInterruptStatus(gEqepBaseAddr);
    intEnabled = EQEP_getEnabledInterrupt(gEqepBaseAddr);
    if (((intStatus & EQEP_INT_INDEX_EVNT_LATCH) != 0) &&
        ((intEnabled & EQEP_INT_INDEX_EVNT_LATCH) != 0))
    {
        /* Get the position latch count */
        if (gEqepIsrCnt < EQEP_EVENT_CNT)
        {
            gEqepPosCnt[gEqepIsrCnt] = EQEP_getIndexPositionLatch(gEqepBaseAddr);
        }
        if (gEqepIsrCnt == (EQEP_EVENT_CNT - 1))
        {
            EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
            SemaphoreP_post(&gEqepSyncSem);
        }
        gEqepIsrCnt++;
    }
    if (((intStatus & EQEP_INT_UNIT_TIME_OUT) != 0) &&
        ((intEnabled & EQEP_INT_UNIT_TIME_OUT) != 0))
    {
        /* Get the position latch count */
        if (gEqepIsrCnt < EQEP_EVENT_CNT)
        {
            gEqepPosCnt[gEqepIsrCnt] = EQEP_getPositionLatch(gEqepBaseAddr);
        }
        if (gEqepIsrCnt == (EQEP_EVENT_CNT - 1))
        {
            EQEP_disableInterrupt(gEqepBaseAddr, EQEP_INT_ALL);
            SemaphoreP_post(&gEqepSyncSem);
        }
        gEqepIsrCnt++;
    }

    /* Clear EQEP Interrupt. */
    EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_ALL);
    //! [App_eqepIntrISR]

    return;
}

static void App_eqepIntrDeReg()
{
    //! [App_eqepIntrDeReg]
    HwiP_destruct(&gEqepHwiObject);
    //! [App_eqepIntrDeReg]
}
