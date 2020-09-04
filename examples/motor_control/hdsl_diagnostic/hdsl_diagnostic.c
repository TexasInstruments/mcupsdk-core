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

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "hdsl_diagnostic.h"
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

extern PRUICSS_Config gPruicssConfig[2];

struct hdslvariables *hdslvariables;

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

void process_request(int menu){
    uint64_t ret_status=0;
    int vpos=1;
    uint64_t val0, val2;
    uint8_t ureg, ureg1;
    float pos0, pos2;

    switch(menu)
    {
        case MENU_SAFE_POSITION:
            if (vpos==1){
                    val0=HDSL_get_pos(0);
                    if(val0!=-1){

                        DebugP_log( "|\r  fast position read successfully \n");
                    }

                    val2=HDSL_get_pos(2);
                               if(val2!=-1){
                                   DebugP_log( "|\r  safe position 2 read successfully \n");
                               }

                   pos2 = (float)(val2 & gMask) / (float)(gMask + 1) * (float)360;

            }
            else{

                    val0=HDSL_get_pos(1);

                    if(val0!=-1){

                        DebugP_log( "|\r  safe position 1 read successfully \n");
                     }
            }

            pos0 = (float)(val0 & gMask) / (float)(gMask + 1) * (float)360;

            ureg = HDSL_get_rssi();
            ureg1 = HDSL_get_qm();

               if ( gMulti_turn)
               {
                   uint64_t turn, turn2;

                   turn = val0 & ~gMask;
                   turn >>= gRes;

                   turn2 = val2 & ~gMask;
                   turn2 >>= gRes;

                   DebugP_log( "|\r Angle: %10.6f\tTurn: %llu\t RSSI: %u\n\r| SafePos2: %10.6f\tTurn: %llu\t QM:  %u\n ", pos0, turn, ureg, pos2, turn2, ureg1 );
               }
               else
               {
                   DebugP_log( "|\r Angle: %10.6f\n", pos0);
               }
            break;
        case MENU_QUALITY_MONITORING:
            ret_status= HDSL_get_qm();
            if(ret_status!=-1){
                DebugP_log( "|\r Quality monitoring value: %u\n", ret_status);
                        }
            break;
        case MENU_EVENTS:
            ret_status=HDSL_get_events();
            if(ret_status!=-1){
                DebugP_log( "|\r Events: 0x%x\n", ret_status);
                        }
            break;
        case MENU_SUMMARY:
            ret_status= HDSL_get_sum();
            if(ret_status!=-1){
                DebugP_log( "|\r Summarized slave status: 0x%x\n", ret_status);
                        }
            break;
        case MENU_ACC_ERR_CNT:
            ret_status= HDSL_get_acc_err_cnt();
            if(ret_status!=-1){
                DebugP_log( "|\r Acceleration error counter: %u\n", ret_status);
                        }
            break;
        case MENU_RSSI:
            ret_status=HDSL_get_rssi();
            if(ret_status!=-1){
                DebugP_log( "|\r RSSI: %u\n", ret_status);
                        }
            break;
        case MENU_PC_SHORT_MSG_WRITE:
            ret_status= HDSL_write_pc_short_msg(gPc_addr, gPc_data);
            if(ret_status!=-1){
                DebugP_log( "|\r Parameter channel short message written\n");
                        }
            break;

        case MENU_PC_SHORT_MSG_READ:
            TC_read_pc_short_msg();
            break;

        case MENU_DIRECT_READ_RID0_LENGTH8:
            direct_read_rid0_length8();
            break;
        case MENU_DIRECT_READ_RID0_LENGTH8_OFFSET6:
            direct_read_rid0_length8_offset6();
            break;
        case MENU_DIRECT_READ_RID81_LENGTH8:
            direct_read_rid81_length8();
            break;
        case MENU_DIRECT_READ_RID81_LENGTH2:
            direct_read_rid81_length2();
            break;
        case MENU_INDIRECT_WRITE_RID0_LENGTH8:
            indirect_write_rid0_length8();
            break;
        case MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0:
            indirect_write_rid0_length8_offset0();
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

    hdsl_pruss_init();

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

void TC_read_pc_short_msg(void)
{
    /* using 1ms delay */
    HDSL_write_pc_short_msg(0x7f,0xab);

    ClockP_usleep(1000);

    gPc_data=HDSL_read_pc_short_msg(0xC0);

    DebugP_log("\r Parameter channel short message read 0x40 data should be 0x01 (in default state)= %u", gPc_data);
    ClockP_usleep(1000);
    gPc_data=HDSL_read_pc_short_msg(0xFF);

    DebugP_log("\r Parameter channel short message read PING(0x7F) register should be 0xab = %u", gPc_data);
}

void TC_write_pc_short_msg(void)
{
    HDSL_write_pc_short_msg(0x7f,0xab);

    ClockP_usleep(1000);

    gPc_data=HDSL_read_pc_short_msg(0xC0);

    DebugP_log("\r Parameter channel short message read 0x40 data =%u should be 0x01", gPc_data);

    gPc_data=HDSL_read_pc_short_msg(0xFF);

    DebugP_log("\r Parameter channel short message read PING(0x7F) register  =%u should be 0xab ", gPc_data);

    ClockP_usleep(1000);

    HDSL_write_pc_short_msg(0x7f,0xcd);

    ClockP_usleep(1000);

    gPc_data=HDSL_read_pc_short_msg(0xC0);

    DebugP_log("\r Parameter channel short message read 0x40 data =%u should be 0x01", gPc_data);

    ClockP_usleep(1000);

    gPc_data=HDSL_read_pc_short_msg(0xFF);

    DebugP_log("\r Parameter channel short message read PING(0x7F) register  =%u should be 0xcd", gPc_data);
}

static void display_menu(void)
{
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r|                                    MENU                                      |\n");
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| %2d : Safe position                                                           |\n", MENU_SAFE_POSITION);
    DebugP_log("\r| %2d : Quality monitoring                                                      |\n", MENU_QUALITY_MONITORING);
    DebugP_log("\r| %2d : Events                                                                  |\n", MENU_EVENTS);
    DebugP_log("\r| %2d : Summarized slave status                                                 |\n", MENU_SUMMARY);
    DebugP_log("\r| %2d : Acceleration error counter                                              |\n", MENU_ACC_ERR_CNT);
    DebugP_log("\r| %2d : RSSI                                                                    |\n", MENU_RSSI);
    DebugP_log("\r| %2d : Parameter channel short message write                                   |\n", MENU_PC_SHORT_MSG_WRITE);
    DebugP_log("\r| %2d : Parameter channel short message read                                    |\n", MENU_PC_SHORT_MSG_READ);
    DebugP_log("\r| %2d : Parameter channel long message write                                    |\n", MENU_PC_LONG_MSG_WRITE);
    DebugP_log("\r| %2d : Access on RID 0h, direct read access with length 8                      |\n", MENU_DIRECT_READ_RID0_LENGTH8);
    DebugP_log("\r| %2d : Access on RID 81h, direct read access with length 8                     |\n", MENU_DIRECT_READ_RID81_LENGTH8);
    DebugP_log("\r| %2d : Access on RID 81h, direct read access with length 2                     |\n", MENU_DIRECT_READ_RID81_LENGTH2);
    DebugP_log("\r| %2d : Access on RID 0h, indirect write, length 8, with offset 0               |\n", MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0);
    DebugP_log("\r| %2d : Access on RID 0h; indirect write, length 8, without offset value        |\n", MENU_INDIRECT_WRITE_RID0_LENGTH8);
    DebugP_log("\r| %2d : Access on RID 0h, direct read, length 8, with offset 6                  |\n", MENU_DIRECT_READ_RID0_LENGTH8_OFFSET6);
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| enter value: ");
}

void  direct_read_rid0_length8(void)
{
    uint8_t dir = 0x01;

    gPc_addrh = 0xec;
    gPc_addrl = 0x00;
    gPc_offh = 0x80;
    gPc_offl = 0x00;

    HDSL_set_pc_addr( gPc_addrh, gPc_addrl, gPc_offh, gPc_offl);

    HDSL_set_pc_ctrl(dir);

    ClockP_sleep(1);

    gPc_buf0 = HDSL_read_pc_buffer(0);

    if(gPc_buf0 == 82)
    {
        gPc_buf1 = HDSL_read_pc_buffer(1);
        if(gPc_buf1 == 79)
        {
            gPc_buf2 = HDSL_read_pc_buffer(2);
            if(gPc_buf2 == 79)
            {
                gPc_buf3 = HDSL_read_pc_buffer(3);
                if(gPc_buf3 == 84)
                {
                    DebugP_log("\r PASS \n");
                }
                else
                {
                    DebugP_log("\r FAIL: gPc_buf3 != T = %u\n", gPc_buf3);
                }
            }
            else
            {
                DebugP_log("\r FAIL: gPc_buf2 != O = %u\n", gPc_buf2);
            }
        }
        else
        {
            DebugP_log("\r FAIL: gPc_buf1 != O = %u\n", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r FAIL: gPc_buf0 != R = %u\n", gPc_buf0);
    }
}

void  direct_read_rid81_length8(void)
{
    uint8_t dir = 0x01;
    gPc_addrh = 0xec;
    gPc_addrl = 0x81;
    gPc_offh = 0x80;
    gPc_offl = 0x00;

    HDSL_set_pc_addr( gPc_addrh, gPc_addrl, gPc_offh, gPc_offl);

    HDSL_set_pc_ctrl(dir);

    ClockP_sleep(1);

    gPc_buf0 = HDSL_read_pc_buffer(0);
    if(gPc_buf0 == 82)
    {
        gPc_buf1 = HDSL_read_pc_buffer(1);
        if(gPc_buf1 == 69)
        {
            gPc_buf2 = HDSL_read_pc_buffer(2);
            if(gPc_buf2 == 83)
            {
                gPc_buf3 = HDSL_read_pc_buffer(3);
                if(gPc_buf3 == 79)
                {
                    gPc_buf4 = HDSL_read_pc_buffer(4);
                    if(gPc_buf4 == 76)
                    {
                        gPc_buf5 = HDSL_read_pc_buffer(5);
                        if(gPc_buf5 == 85)
                        {
                            gPc_buf6 = HDSL_read_pc_buffer(6);
                            if(gPc_buf6 == 84)
                            {
                                gPc_buf7 = HDSL_read_pc_buffer(7);
                                if(gPc_buf7 == 78)
                                {
                                    DebugP_log("\r PASS \n");
                                }
                                else
                                {
                                    DebugP_log("\r FAIL: gPc_buf7 != N \n", gPc_buf7);
                                }
                            }
                            else
                            {
                                DebugP_log("\r FAIL: gPc_buf6 != T \n", gPc_buf6);
                            }
                        }
                        else
                        {
                            DebugP_log("\r FAIL: gPc_buf5 != U \n", gPc_buf5);
                        }
                    }
                    else
                    {
                        DebugP_log("\r FAIL: gPc_buf4 != L \n", gPc_buf4);
                    }

                }
                else
                {
                    DebugP_log("\r FAIL: gPc_buf3 != O \n", gPc_buf3);
                }
            }
            else
            {
                DebugP_log("\r FAIL: gPc_buf2 != S = %u\n", gPc_buf2);
            }
        }
        else
        {
            DebugP_log("\r FAIL: gPc_buf1 != E = %u\n", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r FAIL: gPc_buf0 != R = %u\n", gPc_buf0);
    }
}

void  direct_read_rid81_length2(void)
{
    uint8_t dir = 0x01;

    gPc_addrh = 0xe4;
    gPc_addrl = 0x81;
    gPc_offh = 0x80;
    gPc_offl = 0x03;

    HDSL_set_pc_addr( gPc_addrh, gPc_addrl, gPc_offh, gPc_offl);

    HDSL_set_pc_ctrl(dir);

    ClockP_sleep(1);

    gPc_buf0 = HDSL_read_pc_buffer(0);

    if(gPc_buf0 == 0x00)
    {
        gPc_buf1 = HDSL_read_pc_buffer(1);
        if(gPc_buf1 == 0x0f)
        {
            DebugP_log("\r PASS \n");

        }
        else
        {
            DebugP_log("\r FAIL: gPc_buf1 != 0x0f = %u\n", gPc_buf1);
        }

    }
    else
    {
        DebugP_log("\r FAIL: gPc_buf0 != 0x00 = %u\n", gPc_buf0);
    }
}

void  indirect_write_rid0_length8_offset0(void)
{
    uint8_t dir = 0x01;

    gPc_addrh = 0xbc;
    gPc_addrl = 0x00;
    gPc_offh = 0x80;
    gPc_offl = 0x00;

    HDSL_set_pc_addr( gPc_addrh, gPc_addrl, gPc_offh, gPc_offl);

    HDSL_set_pc_ctrl(dir);

    ClockP_sleep(1);

    gPc_buf0 = HDSL_read_pc_buffer(0);
    if(gPc_buf0 == 0x41)
    {
        gPc_buf1 = HDSL_read_pc_buffer(1);
        if(gPc_buf1 == 0x10)
        {
            DebugP_log("\r PASS \n");
        }
        else
        {
            DebugP_log("\r FAIL: gPc_buf1 != 0x10 = %u\n", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r FAIL: gPc_buf0 != 0x41 = %u\n", gPc_buf0);
    }

}

void  indirect_write_rid0_length8(void)
{
    uint8_t dir = 0x01;

    gPc_addrh = 0x9c;
    gPc_addrl = 0x00;
    gPc_offh = 0x80;
    gPc_offl = 0x00;

    HDSL_set_pc_addr( gPc_addrh, gPc_addrl, gPc_offh, gPc_offl);

    HDSL_set_pc_ctrl(dir);

    ClockP_sleep(1);

    gPc_buf0 = HDSL_read_pc_buffer(0);
    if(gPc_buf0 == 0x41)
    {
        gPc_buf1 = HDSL_read_pc_buffer(1);
        if(gPc_buf1 == 0x10)
        {
            DebugP_log("\r PASS \n");
        }
        else
        {
            DebugP_log("\r FAIL: gPc_buf1 != 0x10 = %u\n", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r FAIL: gPc_buf0 != 0x41 = %u\n", gPc_buf0);
    }
}

void  direct_read_rid0_length8_offset6(void)
{
    uint8_t  dir=0x00;

    gPc_addrh = 0xec;
    gPc_addrl = 0x00;
    gPc_offh = 0x80;
    gPc_offl = 0x06;

    HDSL_set_pc_addr( gPc_addrh, gPc_addrl, gPc_offh, gPc_offl);

    HDSL_set_pc_ctrl(dir);

    ClockP_sleep(1);

    gPc_buf0 = HDSL_read_pc_buffer(0);
    if(gPc_buf0 == 0x41)
    {
        gPc_buf1 = HDSL_read_pc_buffer(1);
        if(gPc_buf1 == 0x10)
        {
            DebugP_log("\r PASS \n");
        }
        else
        {
            DebugP_log("\r FAIL: gPc_buf1 != 0x10 = %u\n", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r FAIL: gPc_buf0 != 0x41 = %u\n", gPc_buf0);
    }
}

static int get_menu(void)
{
    unsigned int cmd;


    if(DebugP_scanf("%d\n", &cmd) < 0 || cmd >= MENU_LIMIT)
    {
        DebugP_log( "| WARNING: invalid option, Safe position selected\r\n");
        cmd = MENU_SAFE_POSITION;
        DebugP_log( "\r| Enter 0 :Fast Position \n Enter 1: Safe Position 1 \n Enter 2: Safe Position 2 \r\n ");

           if((DebugP_scanf("%d\n", &get_pos) < 0) || get_pos > 2)
           {
                   DebugP_log( "\r| WARNING: invalid position value\n");

           }
    }

    if (cmd == MENU_PC_SHORT_MSG_WRITE)
    {
        DebugP_log( "\r| enter addgRess (hex value): ");
        if(DebugP_scanf("%x\n", &gPc_addr) < 0 || gPc_addr > 0x3f)
        {
            DebugP_log( "\r| WARNING: invalid addgRess\n|\n|\n");
            return MENU_INVALID;
        }
        DebugP_log( "\r| enter data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_data) < 0 || gPc_data > 0xff)
        {
            DebugP_log( "\r| WARNING: invalid data\n|\n|\n");
            return MENU_INVALID;
        }
    }

    if (cmd == MENU_PC_LONG_MSG_WRITE)
    {

        DebugP_log("\r| enter addgRess High (hex value): ");
        if(DebugP_scanf("%x\n", &gPc_addrh) < 0 || gPc_addrh > 0x3f)
        {
            DebugP_log("\r| WARNING: invalid addgRess High\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter addgRess Low (hex value): ");
        if(DebugP_scanf("%x\n", &gPc_addrl) < 0 || gPc_addrl > 0x3f)
        {
            DebugP_log("\r| WARNING: invalid addgRess Low\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter parameter channel offset high (hex value): ");
        if(DebugP_scanf("%x\n", &gPc_offh) < 0 || gPc_offh > 0x3f)
        {
            DebugP_log("\r| WARNING: invalid parameter channel offset high\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter parameter channel offset low (hex value): ");
        if(DebugP_scanf("%x\n", &gPc_offl) < 0 || gPc_offl > 0x3f)
        {
            DebugP_log("\r| WARNING: invalid parameter channel offset low\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 0 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf0) < 0 || gPc_buf0 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 0 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 1 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf1) < 0 || gPc_buf1 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 1 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 2 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf2) < 0 || gPc_buf2 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 2 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 3 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf3) < 0 || gPc_buf3 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 3 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 4 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf4) < 0 || gPc_buf4 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 4 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 5 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf5) < 0 || gPc_buf5 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 5 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 6 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf6) < 0 || gPc_buf6 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 6 data\n|\n|\n");
            return MENU_INVALID;
        }

        DebugP_log("\r| enter buffer 7 data (hex value): ");
        if (DebugP_scanf("%x\n", &gPc_buf7) < 0 || gPc_buf7 > 0xff)
        {
            DebugP_log("\r| WARNING: invalid buffer 7 data\n|\n|\n");
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

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

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
    ClockP_usleep(1000);

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
