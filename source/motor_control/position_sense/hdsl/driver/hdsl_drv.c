/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  "AS IS" AND ANY EXPhdslvariables->gResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include <motor_control/position_sense/hdsl/include/hdsl_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>

struct hdslInterface *gHdslInterface;

void HDSL_iep_init(PRUICSS_Handle gPruIcss0Handle, void *gPru_cfg,void *gPru_dramx)

{
     gHdslInterface = (struct hdslInterface *)gPru_dramx;
     //gHdslInterface2 = (struct hdslInterface *)(gPru_dramx + 0x90);
    void *pru_iep;

    pru_iep = (void *)(((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->iep0RegBase);

    HW_WR_REG32((uint32_t)pru_iep + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG,
                (CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CNT_ENABLE_MASK | (1 << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT)));
    /* Use OCP as IEP CLK src */
    HW_WR_REG32((uint32_t)gPru_cfg + CSL_ICSSCFG_IEPCLK, CSL_ICSSCFG_IEPCLK_OCP_EN_MASK);
}

int HDSL_enable_sync_signal(uint8_t ES,uint32_t period)
{
    /*programm here*/
    uint32_t start_time = 10000;

    uint32_t inEvent;
    uint32_t outEvent_latch;
    uint32_t outEvent_gpio;
    uint32_t iep_base;

    iep_base = CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE;
    /*Enable IEP. Enable the Counter and set the DEFAULT_INC and CMP_INC to 1.*/
    HWREG(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG) = 0x111;

    /*Enable SYNC0 and program pulse width*/
    /*Enable SYNC and SYNC0*/
    HWREG(iep_base + CSL_ICSS_G_PR1_IEP1_SLV_SYNC_CTRL_REG) |= 0x03;
    /*Enable cyclic mod*/
    HWREG(iep_base + CSL_ICSS_G_PR1_IEP1_SLV_SYNC_CTRL_REG) |= 0x20;
    /*32504 4500 = num_of_cycles, 50Khz signals, 20us time period//(pulse_width / 4) - 1; */
    HWREG(iep_base + CSL_ICSS_G_PR1_IEP1_SLV_SYNC_PWIDTH_REG) = (period)/2;
    /* (period / 4) - 1; */
    HWREG(iep_base + CSL_ICSS_G_PR1_IEP1_SLV_SYNC0_PERIOD_REG) =(period);

    /*Program CMP1*/
    HWREG(iep_base + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG) |= 0x00000004;
    /*<start time>; Ensure this start time is in future*/
    HWREG(iep_base + CSL_ICSS_G_PR1_IEP1_SLV_CMP1_REG0) = start_time;

    /*TSR configuration:*/
    inEvent = SYNCEVENT_INTRTR_IN_27;
    outEvent_latch = SYNCEVT_RTR_SYNC10_EVT;
    outEvent_gpio = SYNCEVT_RTR_SYNC30_EVT;

    HWREG(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + outEvent_latch) = inEvent | 0x10000;
    HWREG(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + outEvent_gpio) = inEvent | 0x10000;
    HWREG(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + SYNCEVT_RTR_SYNC28_EVT) = inEvent | 0x10000;

    return 1;
}

uint64_t HDSL_get_pos(int position_id)
{

uint64_t val;
    switch(position_id){
        case 0:
            /* Fast Position */
                    val = gHdslInterface->POS0 | (gHdslInterface->POS1 << 8) |
                           (gHdslInterface->POS2 << 16) | (gHdslInterface->POS3 << 24);
                    val |= (uint64_t)gHdslInterface->POS4 << 32;
                    return val;

                break;

        case 1:
            /* Safe Position 1 */
                    val = gHdslInterface->VPOS0 | (gHdslInterface->VPOS1 << 8) |
                           (gHdslInterface->VPOS2 << 16) | (gHdslInterface->VPOS3 << 24);
                    val |= (uint64_t)gHdslInterface->VPOS4 << 32;
                    return val;

                break;


        case 2:
            /* Safe Position 2 */
                    val = gHdslInterface->VPOS20 | (gHdslInterface->VPOS21 << 8) |
                           (gHdslInterface->VPOS22 << 16) | (gHdslInterface->VPOS23 << 24);
                    val |= (uint64_t)gHdslInterface->VPOS24 << 32;

                    return val;
                break;

        default:
                return -1;


           }
}

uint8_t HDSL_get_qm()
{
    uint8_t ureg = gHdslInterface->MASTER_QM & 0xF;


    return ureg;
}


 uint16_t HDSL_get_events()
{
    uint16_t ureg = gHdslInterface->EVENT_L | (gHdslInterface->EVENT_H << 8);


    return ureg;
}

uint8_t HDSL_get_sum()
{
    uint8_t ureg = gHdslInterface->SUMMARY;


    return ureg;
}

uint8_t HDSL_get_acc_err_cnt()
{
    uint8_t ureg = gHdslInterface->ACC_ERR_CNT & 0x1F;


    return ureg;
}

uint8_t HDSL_get_rssi()
{
    uint8_t ureg = (gHdslInterface->DELAY & 0xF0) >> 4;
    return ureg;


}

int HDSL_write_pc_short_msg(uint32_t gPc_addr,uint32_t gPc_data)
{
    gHdslInterface->S_PC_DATA = gPc_data;
    gHdslInterface->SLAVE_REG_CTRL =  gPc_addr;
    return 1;
}

uint32_t HDSL_read_pc_short_msg(uint32_t gPc_addr)
{

    gHdslInterface->SLAVE_REG_CTRL =  gPc_addr;
    return  gHdslInterface->S_PC_DATA;

}

uint8_t HDSL_read_pc_buffer(uint8_t buff)
{

    switch(buff)
    {
    case 0:
        return (gHdslInterface->PC_BUFFER0);
        break;
    case 1:
        return (gHdslInterface->PC_BUFFER1);
        break;
    case 2:
        return (gHdslInterface->PC_BUFFER2);
        break;
    case 3:
        return (gHdslInterface->PC_BUFFER3);
        break;
    case 4:
        return (gHdslInterface->PC_BUFFER4);
        break;
    case 5:
        return (gHdslInterface->PC_BUFFER5);
        break;
    case 6:
        return (gHdslInterface->PC_BUFFER6);
        break;
    case 7:
        return (gHdslInterface->PC_BUFFER7);
        break;

    default:
        return 0;
        break;

    }
}

void HDSL_write_pc_buffer(uint8_t gPc_buf0,uint8_t gPc_buf1,uint8_t gPc_buf2,uint8_t gPc_buf3,uint8_t gPc_buf4,uint8_t gPc_buf5,uint8_t gPc_buf6,uint8_t gPc_buf7)
  {
    gHdslInterface->PC_BUFFER0 = gPc_buf0;
    gHdslInterface->PC_BUFFER1 = gPc_buf1;
    gHdslInterface->PC_BUFFER2 = gPc_buf2;
    gHdslInterface->PC_BUFFER3 = gPc_buf3;
    gHdslInterface->PC_BUFFER4 = gPc_buf4;
    gHdslInterface->PC_BUFFER5 = gPc_buf5;
    gHdslInterface->PC_BUFFER6 = gPc_buf6;
    gHdslInterface->PC_BUFFER7 = gPc_buf7;
}

uint8_t HDSL_get_sync_ctrl()
{
    return (gHdslInterface->SYNC_CTRL);
}

void HDSL_set_sync_ctrl(uint8_t val )
{
    gHdslInterface->SYNC_CTRL=val;
}

uint8_t HDSL_get_master_qm()
{
    return gHdslInterface->MASTER_QM;
}

uint8_t HDSL_get_edges()
{
    return gHdslInterface->EDGES;
}

void HDSL_set_pc_addr(uint8_t gPc_addrh,uint8_t gPc_addrl,uint8_t gPc_offh,uint8_t gPc_offl)
  {

    gHdslInterface->PC_ADD_L = gPc_addrl;
    gHdslInterface->PC_ADD_H = gPc_addrh;

    gHdslInterface->PC_OFF_L = gPc_offl;
    gHdslInterface->PC_OFF_H = gPc_offh;

}

void HDSL_set_pc_ctrl(uint8_t value)
{
    gHdslInterface->PC_CTRL = value;
}

uint8_t HDSL_get_delay()
{
    return gHdslInterface->DELAY;
}

uint8_t HDSL_get_enc_id(int byte)
{

    switch(byte){
    case 0:
        return gHdslInterface->ENC_ID0;
    case 1:
        return gHdslInterface->ENC_ID1;
    case 2:
        return gHdslInterface->ENC_ID2;

    default:
        return -1;

    }
}

void* HDSL_get_src_loc()
{
    return (void *)gHdslInterface;
}

uint32_t HDSL_get_length()
{
    return sizeof(*gHdslInterface);
}
