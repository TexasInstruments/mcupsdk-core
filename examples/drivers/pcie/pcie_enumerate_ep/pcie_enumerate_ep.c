/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>
#include <drivers/pcie.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* maximum number of distinct PCIe MSI IRQ vectors */
#define PCIE_MSI_IRQ_MAX        32

/* Number of used MSI IRQ */
#define PCIE_MSI_IRQ_NUM            0U

/* Layout of our PCIE0_DAT0 area:
 * 0x68000000..0x6bffffff DMA area (64MB -> max. 32MB DMA window due to alignment)
 * 0x6fffff00..0x6fffffff MSI area (256 byte)
 */
#define PCIE_RGN_OFF_DMA                        0x0U
#define PCIE_RGN_IDX_DMA                        0U

#define PCIE_RGN_OFF_MSI                        0x7ffff00U
#define PCIE_RGN_IDX_MSI                        31U
#define PCIE_RGN_WNDSIZE_MSI                    0xffU

#define PCIE_MAP_OUTBND_LOWER_BASE_ADDR_DMA     CSL_PCIE0_DAT0_BASE + PCIE_RGN_OFF_DMA
#define PCIE_MAP_OUTBND_LOWER_BASE_ADDR_MSI     CSL_PCIE0_DAT0_BASE + PCIE_RGN_OFF_MSI

/* Sample device "control" register (bar0.config.ctrl) */
#define SMPL_CTRL_INIT_DONE                     0x1
#define SMPL_CTRL_RESET                         0x2
#define SMPL_CTRL_COPY                          0x4
#define SMPL_CTRL_TEST_MSI                      0x8
#define SMPL_CTRL_TEST_BARs                     0x10

/* Sample device BAR0 "data" area (bar0.data) */
#define SMPL_BAR0_DATA_SIZE                     (16 * 1024)
/* Sample device BAR1 data buffer size */
#define SMPL_BAR1_DATA_SIZE                     (1 * 1024 * 1024)
/* Sample device BAR2 data buffer size */
#define SMPL_BAR2_DATA_SIZE                     (1 * 1024 * 1024)

/* Size of our BAR0 (needs to be a power of two) */
#define SMPL_BAR0_SIZE                          (32 * 1024)

struct bar0
{
    /* "registers" (RC<->EP config & status) */
    volatile struct config {
        uint32_t ctrl;
        uint32_t stat;
        uint64_t dma_addr;
        uint32_t dma_len;
    } config;

    /* padding for MPU granularity */
    uint8_t  pad_config[128 - sizeof (struct config)];

    /* data buffer */
    uint32_t data[SMPL_BAR0_DATA_SIZE / sizeof (uint32_t)];

    /* padding for PCIe region size */
    uint8_t  pad_data[SMPL_BAR0_SIZE - 128 - SMPL_BAR0_DATA_SIZE];
};

/* assign our bar0_mem to a dedicated section so that we can control placement */
struct bar0 bar0_mem __attribute__((section(".bar0_mem")));

/* data buffer to test BAR1 & BAR2 functionality */
uint32_t bar1_data[SMPL_BAR1_DATA_SIZE / sizeof (uint32_t)] __attribute__((section(".bar1_data")));
uint32_t bar2_data[SMPL_BAR2_DATA_SIZE / sizeof (uint32_t)] __attribute__((section(".bar2_data")));

/* runtime assigned start of RC's DMA buffer in our PCIE0_DAT0 region */
static void *dma_ptr;

/* MSI parameters as configured by the RC (read from MSI capability) */
struct msi_params
{
    int      enable;
    uint64_t addr;
    uint32_t data;
    int      count;
};

struct msi_params msi_params;

/* prepared MSI send parameters ready for call to Pcie_epSendMsiIrq */
Pcie_sendMsiParams msi_send_params;

/* Our Pcie_task needs to handle certain notifications from ISRs */
#define PCIE_NTF_LINKDOWN   (1 << 0)
#define PCIE_NTF_D1OR3      (1 << 1)
#define PCIE_NTF_HOTRST     (1 << 2)

#define PCIE_TASK_PRI  (configMAX_PRIORITIES-1)
#define PCIE_TASK_SIZE (16384U/sizeof(configSTACK_DEPTH_TYPE))

TaskHandle_t gPcieTask;
StackType_t gPcieTaskStack[PCIE_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gPcieTaskObj;

/* Our application task needs to handle notifcations from ISR and PCIe task */
#define APPL_NTF_IRQ        (1 << 0)
#define APPL_NTF_PCIE_READY (1 << 1)
#define APPL_NTF_PCIE_HALT  (1 << 2)

extern TaskHandle_t gMainTask;

/*****************************************************************************
 * PowerState IRQ callback
 ****************************************************************************/
void PwrStateIsr(void * args)
{
    BaseType_t doTaskSwitch = 0;
    Pcie_Handle handle = (Pcie_Handle) args;

    Pcie_ackPwrStateIrq(handle);

    xTaskNotifyFromISR(gPcieTask,
                       PCIE_NTF_D1OR3,
                       eSetBits,
                       &doTaskSwitch);

    portYIELD_FROM_ISR(doTaskSwitch);
}

/*****************************************************************************
 * LinkDownState IRQ callback
 ****************************************************************************/
void LinkDownStateIsr(void * args)
{
    BaseType_t doTaskSwitch = 0;
    Pcie_Handle handle = (Pcie_Handle) args;

    Pcie_ackLnkDwnStateIrq(handle);

    xTaskNotifyFromISR(gPcieTask,
                       PCIE_NTF_LINKDOWN,
                       eSetBits,
                       &doTaskSwitch);

    portYIELD_FROM_ISR(doTaskSwitch);
}

/*****************************************************************************
 * Hot Reset IRQ callback
 ****************************************************************************/
void HotResetIsr(void * args)
{
    BaseType_t doTaskSwitch = 0;

    xTaskNotifyFromISR(gPcieTask,
                       PCIE_NTF_HOTRST,
                       eSetBits,
                       &doTaskSwitch);

    portYIELD_FROM_ISR(doTaskSwitch);
}

/*****************************************************************************
 * Downstream IRQ callback
 ****************************************************************************/
void DwnStrIsr(void * args)
{
    BaseType_t doTaskSwitch = 0;
    Pcie_Handle handle = (Pcie_Handle) args;

    Pcie_ackDwnStrIrq(handle);

    xTaskNotifyFromISR(gMainTask,
                       APPL_NTF_IRQ,
                       eSetBits,
                       &doTaskSwitch);

    portYIELD_FROM_ISR(doTaskSwitch);
}

/*****************************************************************************
 * Initialization of PowerState IRQ
 ****************************************************************************/
int Pcie_initPwrStateIrq(Pcie_Handle handle)
{
    int status = SystemP_SUCCESS;
    HwiP_Params hwiParams_pwrState;
    HwiP_Object hwiObj_pwrState;

    HwiP_Params_init(&hwiParams_pwrState);

    /* register core interrupt */
    hwiParams_pwrState.intNum = CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_PWR_STATE_PULSE_0;
    hwiParams_pwrState.callback = &PwrStateIsr;
    hwiParams_pwrState.args = handle;
    hwiParams_pwrState.isPulse = 1;

    status = HwiP_construct(&hwiObj_pwrState, &hwiParams_pwrState);

    DebugP_assert(SystemP_SUCCESS == status);

    /* Enabling PowerState interrupt at controller level */
    status = Pcie_setPwrStateIrq(handle, 1);

    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

/*****************************************************************************
 * Initialization of LinkDownState IRQ
 ****************************************************************************/
int Pcie_initLinkDownStateIrq(Pcie_Handle handle)
{
    int status = SystemP_SUCCESS;
    HwiP_Params hwiParams_lnkState;
    HwiP_Object hwiObj_lnkState;

    HwiP_Params_init(&hwiParams_lnkState);

    /* register core interrupt */
    hwiParams_lnkState.intNum = CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_LINK_STATE_PULSE_0;
    hwiParams_lnkState.callback = &LinkDownStateIsr;
    hwiParams_lnkState.args = handle;
    hwiParams_lnkState.isPulse = 1;

    status = HwiP_construct(&hwiObj_lnkState, &hwiParams_lnkState);

    DebugP_assert(SystemP_SUCCESS == status);

    /* Enabling LinkDownState interrupt at controller level */
    status = Pcie_setLnkDwnStateIrq(handle, 1);

    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

/*****************************************************************************
 * Initialization of Hot Reset IRQ
 ****************************************************************************/
int Pcie_initHotResetIrq(Pcie_Handle handle)
{
    int status = SystemP_SUCCESS;
    HwiP_Params hwiParams_hotReset;
    HwiP_Object hwiObj_hotReset;

    HwiP_Params_init(&hwiParams_hotReset);

    /* register core interrupt */
    hwiParams_hotReset.intNum = CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_HOT_RESET_PULSE_0;
    hwiParams_hotReset.callback = &HotResetIsr;
    hwiParams_hotReset.args = handle;
    hwiParams_hotReset.isPulse = 1;

    status = HwiP_construct(&hwiObj_hotReset, &hwiParams_hotReset);

    DebugP_assert(SystemP_SUCCESS == status);

    /* Enabling Hot Reset interrupt at controller level */
    status = Pcie_setHotResetIrq(handle, 1);

    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}
/*****************************************************************************
 * Initialization of Downstream IRQ
 ****************************************************************************/
int Pcie_initDwnStrIrq(Pcie_Handle handle)
{
    int status = SystemP_SUCCESS;
    HwiP_Params hwiParams_dwn;
    HwiP_Object hwiObj_dwn;

    HwiP_Params_init(&hwiParams_dwn);

    /* register core interrupt */
    hwiParams_dwn.intNum = CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_DOWNSTREAM_PULSE_0;
    hwiParams_dwn.callback = &DwnStrIsr;
    hwiParams_dwn.args = handle;
    hwiParams_dwn.isPulse = 1;

    status = HwiP_construct(&hwiObj_dwn, &hwiParams_dwn);

    DebugP_assert(SystemP_SUCCESS == status);

    /* Enabling Downstream interrupt at controller level */
    status = Pcie_setDwnStrIrq(handle, 1);

    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

/*****************************************************************************
 * Outbound mapping for DMA memory
 ****************************************************************************/
int Pcie_mapOutbnd_DMA(Pcie_Handle handle, int enable)
{
    int status = SystemP_SUCCESS;
    Pcie_AtuRegionParams regionParamsDMA;

    memset(&regionParamsDMA, 0, sizeof(regionParamsDMA));

    if (enable)
    {
        uint64_t target_addr = bar0_mem.config.dma_addr;
        uint32_t size = bar0_mem.config.dma_len;
        uint64_t target_aligned;
        uint32_t size_aligned;
        int num_bits;

        /*
         * local base address, target base address and length need to be aligned to a power of two
         */

        /* use the number of bits which are equal for the first and last address to
         * calculate the number of bits required for the region size */
        num_bits = 64 - __builtin_clzll(target_addr ^ (target_addr + size - 1));

        size_aligned = (1 << num_bits) - 1;
        target_aligned = target_addr & ~(size_aligned - 1);

        /* our local base address is offset by the alignment, too */
        dma_ptr = (void *)(target_addr - target_aligned + PCIE_MAP_OUTBND_LOWER_BASE_ADDR_DMA);

        regionParamsDMA.enableRegion      = 1;
        regionParamsDMA.regionDir         = PCIE_ATU_REGION_DIR_OUTBOUND;
        regionParamsDMA.tlpType           = PCIE_TLP_TYPE_MEM;
        regionParamsDMA.lowerBaseAddr     = PCIE_MAP_OUTBND_LOWER_BASE_ADDR_DMA;
        regionParamsDMA.upperBaseAddr     = 0;
        regionParamsDMA.lowerTargetAddr   = (uint32_t) ( target_aligned & 0xffffffff);
        regionParamsDMA.upperTargetAddr   = (uint32_t) ((target_aligned >> 32) & 0xffffffff);
        regionParamsDMA.regionWindowSize  = size_aligned;

        DebugP_log("Mapping DMA buffer at 0x%llx - size 0x%x...\r\n", target_aligned, size_aligned);
    }
    else
    {
        DebugP_log("Disabling DMA buffer mapping...\r\n");

        regionParamsDMA.enableRegion = 0;
        dma_ptr = NULL;
    }

    status = Pcie_atuRegionConfig (handle, PCIE_LOCATION_LOCAL, PCIE_RGN_IDX_DMA, &regionParamsDMA);

    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

/*****************************************************************************
 * Outbound mapping for MSI IRQs
 ****************************************************************************/
int Pcie_mapOutbnd_MSI(Pcie_Handle handle, int enable)
{
    int status = SystemP_SUCCESS;
    Pcie_AtuRegionParams regionParamsMSI;

    memset(&regionParamsMSI, 0, sizeof(regionParamsMSI));

    if (enable)
    {
        regionParamsMSI.enableRegion = 1;
        regionParamsMSI.regionDir         = PCIE_ATU_REGION_DIR_OUTBOUND;
        regionParamsMSI.tlpType           = PCIE_TLP_TYPE_MEM;
        regionParamsMSI.lowerBaseAddr     = PCIE_MAP_OUTBND_LOWER_BASE_ADDR_MSI;
        regionParamsMSI.upperBaseAddr     = 0x0;
        regionParamsMSI.regionWindowSize  = PCIE_RGN_WNDSIZE_MSI;
        regionParamsMSI.lowerTargetAddr   = (uint32_t)(msi_params.addr & ~0xff);
        regionParamsMSI.upperTargetAddr   = (uint32_t)(msi_params.addr >> 32);

        DebugP_log("Mapping MSI target at 0x%llx - size 0x%x...\r\n", msi_params.addr & ~0xff, PCIE_RGN_WNDSIZE_MSI);
    }
    else
    {
        DebugP_log("Disabling MSI mapping...\r\n");

        regionParamsMSI.enableRegion = 0;
    }

    status = Pcie_atuRegionConfig (handle, PCIE_LOCATION_LOCAL, PCIE_RGN_IDX_MSI, &regionParamsMSI);

    DebugP_assert(SystemP_SUCCESS == status);

    return status;
}

/*****************************************************************************
 * Prints negotiated PCIe link parameters
 ****************************************************************************/
void printLinkParams(Pcie_Handle handle, Pcie_Gen *gen, uint32_t *numLanes)
{
    char *printGenParam = NULL;

    printGenParam = (char*) pvPortMalloc(30*sizeof(char));

    switch(*gen)
    {
        case PCIE_GEN1:
            strcpy(printGenParam, "PCIe Gen1 with 2.5 GT/s speed");
            break;
        case PCIE_GEN2:
            strcpy(printGenParam, "PCIe Gen2 with 5.0 GT/s speed");
            break;
        case PCIE_GEN3:
            strcpy(printGenParam, "PCIe Gen3 with 8.0 GT/s speed");
            break;
    }

    DebugP_log("PCIe link parameter: %s, number of lanes: %d\r\n", printGenParam, *numLanes);

    vPortFree(printGenParam);
}

/*****************************************************************************
 *
 ****************************************************************************/
int32_t Pcie_onLinkDetect(Pcie_Handle handle)
{
    int32_t status  = SystemP_SUCCESS;
    Pcie_Gen gen;
    uint32_t numLanes;
    static int requireConfigure = 1;

    /* Get negotiated PCIe Link Parameter */
    status = Pcie_getLinkParams(handle, &gen, &numLanes);

    if (status == SystemP_SUCCESS)
    {
        /* Print negotiated PCIe Link Parameter */
        printLinkParams(handle, &gen, &numLanes);
    }

    /* set slot clock configuration bit - we're using the connector's reference clock */
    if (status == SystemP_SUCCESS)
    {
        if (requireConfigure)
        {
            status = Pcie_setSlotClockCnfg(handle, 1);

            if (status == SystemP_SUCCESS)
            {
                /* Configure Endpoint */
                status = Pcie_cfgEP(handle);
            }

            if (status == SystemP_SUCCESS)
            {
                /* This configuration is only necessary once after power-up */
                requireConfigure = 0;
            }
        }
    }

    return status;
}

/*****************************************************************************
 * MSI IRQ configuration
 ****************************************************************************/
int Pcie_configMSI(Pcie_Handle handle, int enable)
{
    int status  = SystemP_SUCCESS;
    Pcie_MsiParams msi_regs;

    memset(&msi_regs, 0, sizeof(msi_regs));
    memset(&msi_params, 0, sizeof(msi_params));
    memset(&msi_send_params, 0, sizeof(msi_send_params));

    if (enable)
    {
        status = Pcie_getMsiRegs(handle, PCIE_LOCATION_LOCAL, &msi_regs);

        DebugP_assert(SystemP_SUCCESS == status);

        msi_params.enable  = msi_regs.enable;
        msi_params.addr    = (((uint64_t) msi_regs.loAddr) << 2) | (((uint64_t) msi_regs.upAddr) << 32);
        msi_params.data    = msi_regs.data;
        msi_params.count   = 1 << msi_regs.multMsgEn;

        if (msi_params.enable)
            DebugP_log("PCIe: MSI enabled with %d vector(s) using address %llx and data %x\r\n",
                       msi_params.count, msi_params.addr, msi_params.data);

        if (status == SystemP_SUCCESS && msi_params.enable == 1)
        {
            msi_send_params.addr   = PCIE_MAP_OUTBND_LOWER_BASE_ADDR_MSI + (msi_params.addr & 0xff);
            msi_send_params.data   = msi_params.data;
            msi_send_params.intNum = PCIE_MSI_IRQ_NUM;
        }
        else if (status == SystemP_SUCCESS && msi_params.enable == 0)
        {
            DebugP_log("Configure MSI failed. MSI not enabled\r\n");
            return SystemP_FAILURE;
        }
    }

    return status;
}

/*****************************************************************************
 * Copies data from Bar0 into the total available DMA memory
 ****************************************************************************/
void copyLoop(void)
{
    uint32_t  dma_left = bar0_mem.config.dma_len;
    void     *dma_addr = dma_ptr;

    /* RC might have modified our data buffer, invalidate cache content */
    CacheP_inv (&bar0_mem.data, sizeof (bar0_mem.data), CacheP_TYPE_ALL);

    while (dma_left)
    {
        uint32_t copy_cnt;

        copy_cnt = (dma_left > sizeof (bar0_mem.data)) ? sizeof (bar0_mem.data) : dma_left;

        memcpy(dma_addr, &bar0_mem.data, copy_cnt);

        dma_addr += copy_cnt;
        dma_left -= copy_cnt;
    }
}

/*****************************************************************************
 * PCIe state handling task
 ****************************************************************************/
void pcie_task(void *args)
{
    Pcie_Handle handle;
    uint32_t pcieNtf;
    int linkUp = 0;
    Pcie_PwrState pwrState;
    int pwrStatePending = 0;
    int linkPending = 0;
    int pcieReady = 0;

    /* get handle to our PCIe instance (was opened during Drivers_open) */
    handle = gPcieConfig[CONFIG_PCIE0].object->handle;

    Pcie_initPwrStateIrq(handle);
    Pcie_initLinkDownStateIrq(handle);
    Pcie_initDwnStrIrq(handle);
    Pcie_initHotResetIrq(handle);

    DebugP_log("PCIe: EP initialized and waiting for link\r\n");

    while (1)
    {
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1);
        BaseType_t xResult;

        /* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait(pdFALSE,    /* Don't clear bits on entry. */
                                  UINT32_MAX, /* Clear all bits on exit. */
                                  &pcieNtf,  /* Stores the notified value. */
                                  xMaxBlockTime);

        if (xResult == pdPASS)
        {
            if (pcieNtf & PCIE_NTF_LINKDOWN)
            {
                linkUp = 0;
                DebugP_log("PCIe: lost PCIe link\r\n");
            }

            if (pcieNtf & PCIE_NTF_D1OR3)
            {
                DebugP_log("PCIe: power state entry\r\n");
                pwrStatePending = 1;
            }

            if (pcieNtf & PCIE_NTF_HOTRST)
            {
                DebugP_log("PCIe: hot reset detected\r\n");
            }
        }
        else
        {
            /* no notifications received, so poll for a link if it's not yet up */
            if (!linkUp)
                linkPending = 1;
        }

        /* poll for link up (IRQ only signals link down) */
        if (linkPending && Pcie_isLinkUp(handle) == SystemP_SUCCESS)
        {
            DebugP_log("PCIe: link detected\r\n");
            linkUp = 1;
            linkPending = 0;

            /* we have a link, configure EP */
            Pcie_onLinkDetect(handle);
        }

        /* check power state if we're polling for D0 or if we couldn't read it before */
        if (linkUp && ((pwrState != PCIE_PWR_STATE_D0) || pwrStatePending))
        {
            Pcie_PwrState newPwrState;

            if (Pcie_getPwrState(handle, &newPwrState) == SystemP_SUCCESS)
            {
                pwrStatePending = 0;

                if (pwrState != newPwrState && newPwrState == PCIE_PWR_STATE_D0)
                    DebugP_log("EP is in D0 state\r\n");
                else if (pwrState != newPwrState && newPwrState == PCIE_PWR_STATE_D1)
                    DebugP_log("EP is in D1 state\r\n");
                else if (pwrState != newPwrState && newPwrState == PCIE_PWR_STATE_D3hot)
                    DebugP_log("EP is in D3hot state\r\n");

                pwrState = newPwrState;
            }
            else
                pwrStatePending = 1;

        }

        if (linkUp && !pwrStatePending && pwrState == PCIE_PWR_STATE_D0 && pcieReady == 0)
        {
            /* if we have a link, we know power state is current, and it's D0,
             * and if we haven't done before, signal the application that PCIe is
             * ready for it to run
             */
            DebugP_log("PCIe: signaling APPL ready\r\n");
            xTaskNotify(gMainTask,
                        APPL_NTF_PCIE_READY,
                        eSetBits);
            pcieReady = 1;
        }
        else if (pcieReady && (!linkUp || pwrState != PCIE_PWR_STATE_D0))
        {
            /* if the application believes PCIe is ready but we no longer
             * have a link or are not in D0 anymore, signal the application
             * that PCIe is not longer ready for it to run
             */
            DebugP_log("PCIe: signaling APPL halt\r\n");
            xTaskNotify(gMainTask,
                        APPL_NTF_PCIE_HALT,
                        eSetBits);
            pcieReady = 0;
        }
    }

}

/*****************************************************************************
 * Main application task
 ****************************************************************************/
void pcie_enumerate_ep_main (void *args)
{
    Pcie_Handle handle;
    uint32_t appl_ntf;
    BaseType_t xResult;
    int pcieReady = 0;
    int applConfigured = 0;
    int configAppl = 0;
    int unconfigAppl = 0;
    int runAppl = 0;
    int runMsiTest = 0;
    int runBARsTest = 0;

    Drivers_open();
    Board_driversOpen();

    /* get handle to our PCIe instance (was opened during Drivers_open) */
    handle = gPcieConfig[CONFIG_PCIE0].object->handle;

    gPcieTask = xTaskCreateStatic(pcie_task,   /* Pointer to the function that implements the task. */
                                  "pcie_task", /* Text name for the task.  This is to facilitate debugging only. */
                                  PCIE_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  PCIE_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPcieTaskStack,  /* pointer to stack base */
                                  &gPcieTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gPcieTask != NULL);

    memset (&bar0_mem, 0, sizeof(bar0_mem));

    /* invalidate bar0_mem.data buffer that is going to be written by the RC */
    CacheP_inv (&bar0_mem.data, sizeof (bar0_mem.data), CacheP_TYPE_ALL);

    /* main loop */
    while (1)
    {
        /* wait for pcie status changes or events from RC */
        xResult = xTaskNotifyWait(pdFALSE,    /* Don't clear bits on entry. */
                                  UINT32_MAX, /* Clear all bits on exit. */
                                  &appl_ntf,  /* Stores the notified value. */
                                  portMAX_DELAY);
        configASSERT(xResult == pdTRUE);

        runBARsTest = runMsiTest = configAppl = unconfigAppl = runAppl = 0;

        if (appl_ntf & APPL_NTF_PCIE_READY)
        {
            DebugP_log("APPL: pcie ready\r\n");
            pcieReady = 1;
        }

        if (appl_ntf & APPL_NTF_PCIE_HALT)
        {
            DebugP_log("APPL: pcie not ready\r\n");
            pcieReady = 0;
            if (applConfigured)
                unconfigAppl = 1;
        }

        if (pcieReady && (appl_ntf & APPL_NTF_IRQ))
        {
            if (!applConfigured && bar0_mem.config.ctrl & SMPL_CTRL_INIT_DONE)
            {
                configAppl = 1;
            }

            if (applConfigured && bar0_mem.config.ctrl & SMPL_CTRL_RESET)
            {
                unconfigAppl = 1;
            }

            if (applConfigured && bar0_mem.config.ctrl & SMPL_CTRL_COPY)
            {
                runAppl = 1;
            }

            if (applConfigured && bar0_mem.config.ctrl & SMPL_CTRL_TEST_MSI)
            {
                runMsiTest = 1;
            }

            if (applConfigured && bar0_mem.config.ctrl & SMPL_CTRL_TEST_BARs)
            {
                runBARsTest = 1;
            }
        }

        if (configAppl)
        {
            /* RC is done configuring EP, configure MSI settings and create
             * outbound mappings for MSI and DMA
             */
            Pcie_configMSI(handle, 1);
            Pcie_mapOutbnd_MSI(handle, 1);
            Pcie_mapOutbnd_DMA(handle, 1);

            DebugP_log("APPL: EP configured\r\n");

            applConfigured = 1;
        }
        else if (unconfigAppl)
        {
            /* RC requested a reset, disable outbound mappings and prepare for new configuration */
            Pcie_mapOutbnd_MSI(handle, 0);
            Pcie_mapOutbnd_DMA(handle, 0);

            /* RC requested a reset, disable outbound mappings and prepare for new configuration */
            Pcie_configMSI(handle, 0);

            DebugP_log("APPL: EP unconfigured\r\n");

            applConfigured = 0;
        }
        else if (runAppl)
        {
            /* invalidate bar0_mem.data buffer that is going to be written by the RC */
            CacheP_inv (&bar0_mem.data, sizeof (bar0_mem.data), CacheP_TYPE_ALL);

            copyLoop();

            /* send MSI to RC to acknowledge completion of copy loop */
            msi_send_params.intNum = PCIE_MSI_IRQ_NUM;

            if (Pcie_epSendMsiIrq(handle, msi_send_params) != SystemP_SUCCESS)
            {
                DebugP_log("APPL: error sending MSI to RC\r\n");
                return;
            }

            DebugP_log("DMA test done\r\n");
        }
        else if (runMsiTest)
        {
            /*
             * Apply MSI test by sending maximum amount of distinct enabled
             * MSI IRQs in ascending order
             */
            for (int i = 0; i < msi_params.count; i++)
            {
                msi_send_params.intNum = i;

                DebugP_log("Send MSI IRQ nr. %d\r\n", msi_send_params.intNum);

                if (Pcie_epSendMsiIrq(handle, msi_send_params) != SystemP_SUCCESS)
                {
                    DebugP_log("APPL: error sending MSI to RC\r\n");
                    return;
                }
            }

            DebugP_log("MSI test done\r\n");
        }
        else if (runBARsTest)
        {
            /* check if data buffer for BAR1 contains expected values */
            for (int i = 0; i < SMPL_BAR1_DATA_SIZE / sizeof (uint32_t); i++)
            {
                if (bar1_data[i] != i)
                {
                    DebugP_log("BAR1 test failed\r\n");
                    break;
                }
            }

            /* check if data buffer for BAR2 contains expected values */
            for (int i = 0; i < SMPL_BAR2_DATA_SIZE / sizeof (uint32_t); i++)
            {
                if (bar2_data[i] != ~i)
                {
                    DebugP_log("BAR2 test failed\r\n");
                    break;
                }
            }

            /*
             * on successful data check send MSI IRQ to indicate RC
             * that BAR test is passed
             */
            msi_send_params.intNum = PCIE_MSI_IRQ_NUM;

            if (Pcie_epSendMsiIrq(handle, msi_send_params) != SystemP_SUCCESS)
            {
                DebugP_log("APPL: error sending MSI to RC\r\n");
                return;
            }

            DebugP_log("BAR test done\r\n");
        }
    }

    Board_driversClose();
    Drivers_close();

    return;
}
