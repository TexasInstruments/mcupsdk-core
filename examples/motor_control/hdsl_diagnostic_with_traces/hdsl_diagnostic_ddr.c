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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>

#include <drivers/pruicss.h>
#include <drivers/udma.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "hdsl_diagnostic_ddr.h"

#include <motor_control/position_sense/hdsl/include/hdsl_drv.h>
#include <motor_control/position_sense/hdsl/include/pruss_intc_mapping.h>
#include <motor_control/position_sense/hdsl/firmware/hdsl_master_icssg_bin.h>
#include <motor_control/position_sense/hdsl/firmware/hdsl_master_icssg_sync_bin.h>

#ifdef HDSL_AM64xE1_TRANSCEIVER
#include <board/ioexp/ioexp_tca6424.h>
#endif

#define PRUICSS_PRUx  PRUICSS_PRU1

#define ENDAT_EN (0x1 << 26)
/* OCP as clock, div 24 */
#define ENDAT_TX_CFG (0x10 | (23 << 16))
/* OCP as clock, div 3, 8x OSR */
#define ENDAT_RX_CFG (0x10 | (2 << 16) | 7 | 0x08)
#define CTR_EN (1 << 3)
#define MAX_WAIT 20000

/* DDR Trace is triggered for each H-Frame. SYS_EVENT_21 is triggered for each
   H-Frame from PRU. SYS_EVENT_21 is mapped to PRU_ICSSG0_PR1_HOST_INTR_PEND_3 of R5F
   in INTC Mapping. */

/*Event number for SYS_EVENT_21 (pr1_pru_mst_intr<5>_int_req) */
#define HDSL_DDR_TRACE_ICSS_INTC_EVENT_NUM (21U)

/* R5F Interrupt number for DDR Traces */
#define HDSL_DDR_TRACE_R5F_IRQ_NUM  (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3)

extern PRUICSS_Config gPruicssConfig[2];

Udma_ChHandle   chHandle;

/* To store user input to start DDR copy*/
volatile uint8_t start_copy;

/* To store user input number of count of DDR copy*/
uint16_t ddr_trace_count;

/* To save log count during DDR copy*/
uint32_t temp_count_arr[NUM_RESOURCES];

/* To save log v-frame count during DDR copy*/
uint32_t v_frames_count_arr[NUM_RESOURCES];

/* To copy HDSL Interface structure ddr location */
struct hdslInterface gHdslInterface_ddr[NUM_RESOURCES] __attribute__((aligned(128), section(".hdslInterface_ddr_mem")));

HwiP_Object         gPRUHwiObject;
/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;
PRUICSS_IntcInitData gPruss0_intc_initdata = PRU_ICSS0_INTC_INITDATA;
PRUICSS_Handle gPruIcss1Handle;
PRUICSS_IntcInitData gPruss1_intc_initdata = PRU_ICSS1_INTC_INITDATA;

static void *gPru_cfg, *gPru_ctrl;

static char gUart_buffer[256];

void *gPru_dramx;

int get_pos=1;

uint32_t gMulti_turn, gRes;
uint64_t gMask;

uint32_t gPc_addr;
uint32_t gPc_data;

uint8_t gPc_addrh, gPc_addrl, gPc_offh, gPc_offl, gPc_buf0, gPc_buf1, gPc_buf2, gPc_buf3, gPc_buf4, gPc_buf5, gPc_buf6, gPc_buf7;


#ifdef HDSL_AM64xE1_TRANSCEIVER
static TCA6424_Config  gTCA6424_Config;
#endif

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gUdmaTestDoneSem;
/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

void App_udmaEventCb(Udma_EventHandle eventHandle, uint32_t eventType, void *appData);
static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length);

void sync_calculation(void)
{
    uint8_t ES;
    uint16_t wait_before_start;
    uint32_t counter, period, index;
    volatile uint32_t cap6_rise0, cap6_rise1, cap6_fall0, cap6_fall1;
    uint8_t EXTRA_EDGE_ARR[8] = {0x00 ,0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};
    uint32_t minm_bits = 112, cycle_per_bit = 24, max_stuffing = 26, stuffing_size = 6, cycle_per_overclock_bit =3, minm_extra_size = 4, sync_param_mem_start = 0xDC;
    uint32_t cycles_left, additional_bits, minm_cycles, time_gRest, extra_edge, extra_size, num_of_stuffing, extra_size_remainder, stuffing_remainder, bottom_up_cycles;

    /*measure of SYNC period starts*/
    ES =  HDSL_get_sync_ctrl();
    volatile uint32_t* carp6_rise_addr =   (uint32_t*)(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CAPR6_REG0);
    volatile uint32_t* carp6_fall_addr =   (uint32_t*)(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CAPF6_REG0);
    cap6_rise0 = *(carp6_rise_addr);
    cap6_fall0 = *(carp6_fall_addr);
    cap6_rise1 = cap6_rise0;
    cap6_fall1 = cap6_fall0;
    counter = 0;

    for(index = 0 ; index <2 ; index++)
    {
        cap6_rise0 = cap6_rise1;
        cap6_fall0 = cap6_fall1;
        while((cap6_fall0 == cap6_fall1) || (cap6_rise0 == cap6_rise1))
        {
            cap6_rise1 = *(carp6_rise_addr);
            cap6_fall1 = *(carp6_fall_addr);
            counter++;
            if(counter > MAX_WAIT)
            {
                DebugP_log("\rSYNC PULSE NOT FOUND, WAITING FOR SYNC PULSE\n");
                counter = 0;
            }
        }
    }

    period = cap6_rise1 - cap6_rise0;
    /*measure of SYNC period ends*/

    minm_cycles = minm_bits * ES * cycle_per_bit;
    cycles_left = period - minm_cycles;
    time_gRest = (cycles_left % cycle_per_bit) / cycle_per_overclock_bit;
    additional_bits = cycles_left / cycle_per_bit;
    extra_edge = EXTRA_EDGE_ARR[time_gRest];
    num_of_stuffing = additional_bits / stuffing_size;
    extra_size = additional_bits % stuffing_size;
    extra_size = extra_size + minm_extra_size * ES;
    if(num_of_stuffing > ES * max_stuffing)
    {
        extra_size = extra_size + (((num_of_stuffing) - (max_stuffing * ES)) * stuffing_size);
        num_of_stuffing = ES * max_stuffing;
    }
    extra_size_remainder = extra_size % ES;
    extra_size = extra_size / ES;
    stuffing_remainder = num_of_stuffing % ES;
    num_of_stuffing = num_of_stuffing / ES;
    bottom_up_cycles = (minm_cycles - minm_extra_size * ES * cycle_per_bit);
    bottom_up_cycles = bottom_up_cycles + (stuffing_size * (ES * num_of_stuffing + stuffing_remainder))*cycle_per_bit;
    bottom_up_cycles = bottom_up_cycles + ((ES * extra_size  + extra_size_remainder) * cycle_per_bit ) + time_gRest * cycle_per_overclock_bit;
    wait_before_start = (84 * cycle_per_bit)+((8 - time_gRest)*cycle_per_overclock_bit)+(num_of_stuffing * stuffing_size * cycle_per_bit);
    if(stuffing_remainder != 0)
    {
        wait_before_start = wait_before_start+(stuffing_size * cycle_per_bit);
    }
    wait_before_start = wait_before_start - 51;
    if(extra_size < 4 || extra_size > 9)
    {
        DebugP_log("\rERROR: ES or period selected is Invalid \n");
    }
    DebugP_log("\r********************************************************************\n");
    DebugP_log("\rSYNC MODE: period = %d\n", period);
    DebugP_log("\rSYNC MODE: ES = %d\n", ES);
    DebugP_log("\rSYNC MODE: counter = %d\n", counter);
    DebugP_log("\rSYNC MODE: wait_before_start = %d\n", wait_before_start);
    DebugP_log("\rSYNC MODE: bottom_up_cycles = %d\n", bottom_up_cycles);
    DebugP_log("\rSYNC MODE: extra_size = %d\n", extra_size);
    DebugP_log("\rSYNC MODE: temp_gRest = %d\n", time_gRest);
    DebugP_log("\rSYNC MODE: extra_edge = %d\n", extra_edge);
    DebugP_log("\rSYNC MODE: num_of_stuffing = %d\n", num_of_stuffing);
    DebugP_log("\rSYNC MODE: extra_size_remainder = %d\n", extra_size_remainder);
    DebugP_log("\rSYNC MODE: stuffing_remainder = %d\n", stuffing_remainder);
    DebugP_log("\r********************************************************************\n");

    sync_param_mem_start =sync_param_mem_start + (uint32_t)gPru_dramx;
    HWREGB(sync_param_mem_start) = extra_size;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = num_of_stuffing;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = extra_edge;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = time_gRest;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = extra_size_remainder;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = stuffing_remainder;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGH(sync_param_mem_start) = wait_before_start;

}

void udma_copy(uint8_t *srcBuf,uint8_t *destBuf,uint32_t length)
{
    int32_t         retVal = UDMA_SOK;
    uint64_t        pDesc;
    uint32_t        trRespStatus;
    uint8_t        *trpdMem = &gUdmaTestTrpdMem[0U];
    uint64_t        trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);

    /* Init TR packet descriptor */
    App_udmaTrpdInit(chHandle, trpdMem, destBuf, srcBuf, length);

    /* Submit TRPD to channel */
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle), trpdMemPhy);
    DebugP_assert(UDMA_SOK == retVal);

    /* Wait for return descriptor in completion ring - this marks transfer completion */
    SemaphoreP_pend(&gUdmaTestDoneSem, SystemP_WAIT_FOREVER);

    retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
    DebugP_assert(UDMA_SOK == retVal);

    /* Check TR response status */
    CacheP_inv(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

    trRespStatus = UdmaUtils_getTrpdTr15Response(trpdMem, 1U, 0U);
    //DebugP_log("\r\n.trRespStatus = %u ", trRespStatus);
    DebugP_assert(CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE == trRespStatus);


    /* Validate data in ddr */
    CacheP_inv(destBuf, length, CacheP_TYPE_ALLD);
}

void App_udmaEventCb(Udma_EventHandle eventHandle, uint32_t eventType, void *appData)
{

    if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
    {

        SemaphoreP_post(&gUdmaTestDoneSem);

    }
}

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length)
{
    CSL_UdmapTR15  *pTr;
    uint32_t        cqRingNum = Udma_chGetCqRingNum(chHandle);

    /* Make TRPD with TR15 TR type */
    UdmaUtils_makeTrpdTr15(trpdMem, 1U, cqRingNum);

    /* Setup TR */
    pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
    pTr->icnt0    = length;
    pTr->icnt1    = 1U;
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
    pTr->dicnt0   = length;
    pTr->dicnt1   = 1U;
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);

    /* Perform cache writeback */
    CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

    return;
}

void process_request(int menu){

    switch(menu)
    {

        case MENU_HDSL_REG_INTO_DDR:
            traces_into_ddr();
            break;
        case MENU_HDSL_REG_INTO_DDR_GPIO:
            TC_input_start_copy();
            break;
        default:
            DebugP_log( "\r| ERROR: invalid request\n");
            break;
    }
}

void hdsl_pruss_init(void)
{
    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

    gPruIcss1Handle = PRUICSS_open(CONFIG_PRU_ICSS1);

    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);

    /* clear ICSS0 PRU data RAM */
    gPru_dramx = (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + PRUICSS_DATARAM(PRUICSS_PRUx));
    memset(gPru_dramx, 0, (4 * 1024));

    gPru_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->cfgRegBase);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_GPCFG1, ENDAT_EN);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, ENDAT_TX_CFG);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_EDPRU1RXCFGREGISTER, ENDAT_RX_CFG);

    PRUICSS_intcInit(gPruIcss0Handle, &gPruss0_intc_initdata);

    PRUICSS_intcInit(gPruIcss1Handle, &gPruss1_intc_initdata);

    /* configure C28 to PRU_ICSS_CTRL and C29 to EDMA + 0x1000 */
    /*6.4.14.1.1 ICSSG_PRU_CONTROL RegisterPRU_ICSSG0_PR1_PDSP0_IRAM 00B0 2400h*/
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x0240);
    /*IEP1 base */
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C29, 0x0002F000);

    /* enable cycle counter */
    gPru_ctrl =  (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE);

    HW_WR_REG32(gPru_ctrl, CTR_EN);

}

void hdsl_pruss_load_run_fw(void)
{
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);

    if(HDSL_get_sync_ctrl() == 0)
    {
        /*free run*/
        PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                    0, (uint32_t *) Hiperface_DSL2_0,
                    sizeof(Hiperface_DSL2_0));
    }
    else
    {
        /*sync_mode*/
        PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                        0, (uint32_t *) Hiperface_DSL_SYNC2_0,
                        sizeof(Hiperface_DSL_SYNC2_0));
    }

    PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);

    /*Run firmware*/
    PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
}

void hdsl_init()
{
    uint8_t ES;
    uint32_t period;
    HwiP_Params     hwiPrms;
    uint32_t        intrNum;
    intrNum          = HDSL_DDR_TRACE_R5F_IRQ_NUM;

    hdsl_pruss_init();

    /* Register PRU interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = (void*)&HDSL_IsrFxn;
    HwiP_construct(&gPRUHwiObject, &hwiPrms);

    HDSL_iep_init(gPruIcss0Handle, gPru_cfg,gPru_dramx);
    DebugP_log( "\r\nEnter ES(number of frames per sync period), note ES=0 for FREE RUN mode: \n");
    DebugP_scanf("%d", &ES);
    HDSL_set_sync_ctrl(ES);
        if(ES != 0)
        {
            DebugP_log("\r\nEnter ES(number of frames per sync period), note ES=0 for FREE RUN mode: ");
            DebugP_scanf("%d", &ES);
            DebugP_log("\r\nEnter period for SYNC PULSE in unit of cycles(1 cycle = 4.44ns):");
            DebugP_scanf("%d",&period);

            HDSL_enable_sync_signal(ES,period);
            HDSL_generate_memory_image();
            sync_calculation();
            hdsl_pruss_load_run_fw();
        }

        else{
            DebugP_log( "\r\nFREE RUN MODE\n");
            HDSL_generate_memory_image();
            hdsl_pruss_load_run_fw();
        }
}

static void HDSL_IsrFxn()
{
    static uint64_t v_frames_count=0;
    static int temp = 0;
    uint8_t        *srcBuf;//= (uint8_t*)gHdslInterface;
    uint8_t        *destBuf = (uint8_t*)&gHdslInterface_ddr[0U];
    uint32_t        length;// = sizeof(hdslInterface);

    srcBuf = (uint8_t*)HDSL_get_src_loc();
    length = HDSL_get_length();
    PRUICSS_clearEvent(gPruIcss0Handle, HDSL_DDR_TRACE_ICSS_INTC_EVENT_NUM);
    /* No of v-frames count */
    v_frames_count++;

    if((start_copy == 1) && (temp < ddr_trace_count))
    {

        /* Init buffers and TR packet descriptor */
        destBuf = (uint8_t*)&gHdslInterface_ddr[temp];

        /* start UDMA copying data from src to dest */
        udma_copy(srcBuf,destBuf,length);

        //temp_count_arr[temp] = temp_count++;
        v_frames_count_arr[temp] = v_frames_count;
        temp++;
    }
    else
    {
        start_copy = 0;
        temp = 0;
    }

}

static void display_menu(void)
{
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r|                                    MENU                                      |\n");
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| %2d : HDSL registers into DDR                                                 |\n", MENU_HDSL_REG_INTO_DDR);
    DebugP_log("\r| %2d : HDSL registers into DDR using GPIO                                                |\n", MENU_HDSL_REG_INTO_DDR_GPIO);
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| enter value: ");
}

void traces_into_ddr(void)
{
    int i= 0;
    uint32_t length;
    length = HDSL_get_length();

    DebugP_log("\r\n sizeof(hdslInterface)_count = %u", length);
    DebugP_log("\r\n Start address of DDR location = %u", DDR_START_OFFSET);
    DebugP_log("\r\n End address of DDR location = %u", DDR_END_OFFSET);
    DebugP_log("\r\n No of HDSL-Interface-Register-Structure to copy = %u", ddr_trace_count);

    start_copy = 1;

    /* Testing:
     * To test this input use function TC_input_start_copy()
     * Connect the header pins J16 and J17 together
     * J16 is used as input for copy start
     * J17 is used to create high/low pulses each second
     */

    while(start_copy)
    {
        ClockP_sleep(1);
    }

    for(i=0;i< ddr_trace_count; i++)
    {
        DebugP_log("\r\n %u .....v_frames_count = %u ",i, v_frames_count_arr[i]);
    }

}

/* run this test case function to continuously make the start copy input pin high and low every 1 second  */
void TC_input_start_copy()
{
    int i= 0, j =0;

    /* Read the value of J16 header on board */
    start_copy = (uint8_t) GPIO_pinRead(CONFIG_GPIO_COPY_BASE_ADDR, CONFIG_GPIO_COPY_PIN);

    while(j < 10)
    {
        /* Set the J17 header pin on board as LOW (Active low) */
        GPIO_pinWriteLow(CONFIG_GPIO_TEST_COPY_BASE_ADDR, CONFIG_GPIO_TEST_COPY_PIN);
        /* Wait for 1 second */
        ClockP_sleep(1);

        for(i=0;i< ddr_trace_count; i++)
        {
            DebugP_log("\r\n %u .....v_frames_count = %u ",(j*ddr_trace_count + i), v_frames_count_arr[i]);
        }

        /* Set the J17 header pin on board as HIGH */
        GPIO_pinWriteHigh(CONFIG_GPIO_TEST_COPY_BASE_ADDR, CONFIG_GPIO_TEST_COPY_PIN);
        /* Wait for 1 second */
        ClockP_sleep(1);

        j++;
    }
}

static int get_menu(void)
{
    unsigned int cmd;


    if(DebugP_scanf("%d\n", &cmd) < 0 || cmd >= MENU_LIMIT)
    {
        DebugP_log( "| WARNING: invalid option, Safe position selected\r\n");
        cmd = MENU_HDSL_REG_INTO_DDR;
        DebugP_log( "\r| Enter 0 :Fast Position \n Enter 1: Safe Position 1 \n Enter 2: Safe Position 2 \r\n ");

           if((DebugP_scanf("%d\n", &get_pos) < 0) || get_pos > 2)
           {
                   DebugP_log( "\r| WARNING: invalid position value\n");

           }
    }

    if (cmd == MENU_HDSL_REG_INTO_DDR)
    {
       DebugP_log("\r| How many traces you want to copy : ");
       if(DebugP_scanf("%u\n", &ddr_trace_count) < 0 || ddr_trace_count >= NUM_RESOURCES)
       {
           DebugP_log("\r| WARNING: invalid data\n|\n|\n");
           return MENU_INVALID;
       }
    }
    if (cmd == MENU_HDSL_REG_INTO_DDR_GPIO)
    {
       DebugP_log("\r| How many traces you want to copy : ");
       if(DebugP_scanf("%u\n", &ddr_trace_count) < 0 || ddr_trace_count >= NUM_RESOURCES)
       {
           DebugP_log("\r| WARNING: invalid data\n|\n|\n");
           return MENU_INVALID;
       }
    }
    return cmd;
}



#ifdef HDSL_AM64xE1_TRANSCEIVER
static void hdsl_i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    /* P20 = LED 3 bits, pin, 2 bits port.*/
    uint32_t            ioIndex = 0x10;
    TCA6424_Params      tca6424Params;

    TCA6424_Params_init(&tca6424Params);

    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);

    if(status == SystemP_SUCCESS)
    {
        /* Set output to HIGH before config so that LED start with On state */
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        ioIndex = 0x0a;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);


    }
    TCA6424_close(&gTCA6424_Config);
}
#endif

void hdsl_diagnostic_main(void *arg)
{
    uint32_t val, acc_bits, pos_bits;
    uint8_t ureg;
    int32_t retVal = UDMA_SOK, status;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* UDMA initialization */
    chHandle = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */

    /* Make the J16 pin on board as input for copy start */
    GPIO_setDirMode(CONFIG_GPIO_COPY_BASE_ADDR, CONFIG_GPIO_COPY_PIN, GPIO_DIRECTION_INPUT);

    /* For Testing copy input:  */
    /* Make the J17 header pin on board as output as copy start test pin */
    GPIO_setDirMode(CONFIG_GPIO_TEST_COPY_BASE_ADDR, CONFIG_GPIO_TEST_COPY_PIN, GPIO_DIRECTION_OUTPUT);

    /* Channel enable */
    retVal = Udma_chEnable(chHandle);
    DebugP_assert(UDMA_SOK == retVal);
    status = SemaphoreP_constructBinary(&gUdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    #ifndef HDSL_AM64xE1_TRANSCEIVER
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. This is required to remap EnDAT signals correctly via Interface card.*/
    HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x80));

    /*Configure GPIO42 for HDSL mode.*/
    GPIO_setDirMode(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN, CONFIG_GPIO0_DIR);
    GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
    #else
    /* Configure g_mux_en to 0 in ICSSG_SA_MX_REG Register. */
    HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x00));
    /*Configure GPIO42 for HDSL mode. New transceiver card needs the pin to be configured as input*/
    HW_WR_REG32(0x000F41D4, 0x00050001);   /* PRG0_PRU1_GPI9 as input */
    hdsl_i2c_io_expander(NULL);
    #endif

    DebugP_log( "\n\n Hiperface DSL diagnostic\n");
    hdsl_init();
    DebugP_log( "\r\n HDSL setup finished\n");
    /*need some extra time for SYNC mode since frames are longer*/
    ClockP_sleep(1);

    for (ureg = HDSL_get_master_qm(), val = 0; !(ureg & 0x80); ureg = HDSL_get_master_qm(), val++, ClockP_usleep(10))
    {
        if (val > 100)
        { /* wait 1ms to detect, increase if reqd. */
            while(1)
            {
                DebugP_log( "\r\nHiperface DSL encoder not detected\n\n");
                ClockP_usleep(5000);
            }
        }
    }

    DebugP_log( "\r\n");
    DebugP_log( "\r|------------------------------------------------------------------------------|\n");
    DebugP_log( "\r|                            Hiperface DSL diagnostic                          |\n");
    DebugP_log( "\r|------------------------------------------------------------------------------|\n");
    DebugP_log( "\r|\n");
    DebugP_log( "\r| Quality monitoring value: %u\n", ureg & 0xF);

    ureg = HDSL_get_edges();
    DebugP_log( "\r| Edges: 0x%x\n", ureg);

    ureg = HDSL_get_delay();
    DebugP_log( "\r| Cable delay: %u\tRSSI: %u\n", ureg & 0xF, (ureg & 0xF0) >> 4);

    val =HDSL_get_enc_id(0) | (HDSL_get_enc_id(1) << 8) |
              (HDSL_get_enc_id(2) << 16);
    acc_bits = val & 0xF;
    acc_bits += 8;
    pos_bits = (val & 0x3F0) >> 4;
    pos_bits += acc_bits;
    DebugP_log( "\r| Encoder ID: 0x%x", val);
    DebugP_log( "(");
    DebugP_log( "Acceleration bits: %u ,", acc_bits);
    DebugP_log( "Position bits: %u,", pos_bits);
    DebugP_log( "%s", val & 0x400 ? " Bipolar position" : " Unipolar position");
    DebugP_log( ")\r|\n");

    DebugP_log( "\r| Enter single turn bits: ");
    if((DebugP_scanf("%d\n", &gRes) < 0) || gRes > pos_bits)
    {
            DebugP_log( "\r| WARNING: invalid single turn bits, assuming single turn encoder\n");
            gRes = pos_bits;
    }
    gMulti_turn = pos_bits - gRes;
    gMask = pow(2, gRes) - 1;
    if (gMulti_turn)
    {
        DebugP_log( "\r| Multi turn bits: %u\n", gMulti_turn);
    }

    while(1)
    {

        int menu;
        display_menu();

        menu = get_menu();
        process_request(menu);
        DebugP_log( "|\r \n");
        DebugP_log( "\r%s", gUart_buffer);
        DebugP_log( "\r|\n\r|\n\r|\n");
    }

    Board_driversClose();
    Drivers_close();
}
