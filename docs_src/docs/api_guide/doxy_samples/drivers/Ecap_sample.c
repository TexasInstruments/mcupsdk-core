
#include <stdio.h>
//! [include]
#include <drivers/ecap.h>
//! [include]
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_soc.h>
/* ECAP Interrupt Sources */
#define ECAP_INT_ALL                    (ECAP_CEVT1_INT  | \
                                         ECAP_CEVT2_INT  | \
                                         ECAP_CEVT3_INT  | \
                                         ECAP_CEVT4_INT  | \
                                         ECAP_CNTOVF_INT | \
                                         ECAP_PRDEQ_INT  | \
                                         ECAP_CMPEQ_INT)
/* ECAP Instance Macros */
#define CONFIG_ECAP0_BASE_ADDR (CSL_ECAP0_CTL_STS_BASE)
#define CONFIG_ECAP0_FCLK (125000000)
#define CONFIG_ECAP0_INTR (140)
#define CONFIG_ECAP0_INTR_IS_PULSE (TRUE)

/* Global variables and objects */
static HwiP_Object       gEcapHwiObject;
static SemaphoreP_Object gEcapSyncSemObject;
/* Variable to hold base address of EPWM/ECAP that is used */
uint32_t gEcapBaseAddr = CONFIG_ECAP0_BASE_ADDR;;

/* Static Function Declarations */
static void App_ecapIntrISR(void *arg);

static int32_t App_ecapIntrReg()
{
    //! [App_ecapIntrReg]
    int32_t             status = SystemP_SUCCESS;
    HwiP_Params         hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_ECAP0_INTR;
    hwiPrms.callback    = &App_ecapIntrISR;
    hwiPrms.isPulse     = CONFIG_ECAP0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEcapHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    //! [App_ecapIntrReg]

    return status;
}

static void App_ecapConfigOperMode(void)
{
    //! [App_ecapConfigOperMode]
    /* Enable capture mode */
    ECAP_operatingModeSelect(gEcapBaseAddr, ECAP_CAPTURE_MODE);
    //! [App_ecapConfigOperMode]
}

static void App_ecapConfigOneShotMode(void)
{
    //! [App_ecapConfigOneShotMode]
    /* One shot mode, stop capture at event 4 */
    ECAP_oneShotModeConfig(gEcapBaseAddr, ECAP_CAPTURE_EVENT4_STOP);
    //! [App_ecapConfigOneShotMode]
}

static void App_ecapCaptureEvtPolarityConfig(void)
{
    //! [App_ecapCaptureEvtPolarityConfig]
    /* Set polarity of the events to rising, falling, rising, falling edge */
    ECAP_captureEvtPolarityConfig(gEcapBaseAddr,
                                 ECAP_CAPTURE_EVENT_RISING,
                                 ECAP_CAPTURE_EVENT_FALLING,
                                 ECAP_CAPTURE_EVENT_RISING,
                                 ECAP_CAPTURE_EVENT_FALLING);
    //! [App_ecapCaptureEvtPolarityConfig]
}

static void App_ecapIntrISR(void *arg)
{
    //! [App_ecapIntrISR]
    /* Clear Ecap Interrupt. */
    ECAP_intrStatusClear(gEcapBaseAddr, ECAP_INT_ALL);
    /* Clear Global Interrupt Flag. */
    ECAP_globalIntrClear(gEcapBaseAddr);

    SemaphoreP_post(&gEcapSyncSemObject);
    //! [App_ecapIntrISR]
    return;
}

static void App_ecapIntrDeReg()
{
    //! [App_ecapIntrDeReg]
    HwiP_destruct(&gEcapHwiObject);
    //! [App_ecapIntrDeReg]
}
