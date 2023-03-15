/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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
 *  "AS IS" AND ANY EXPS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

/* Should move the below to sysconfig  generated code */
    HDSL_Config hdslConfig0;
    HDSL_Config hdslConfig1;
    HDSL_Config hdslConfig2;

void hdsl_enable_load_share_mode(void *gPru_cfg ,uint32_t  PRU_SLICE)
{
    //HW_WR_REG32(0x30026104) |= 0x0800;
    uint32_t rgval;
    if(PRU_SLICE==1)
    {
       rgval = HW_RD_REG32((uint8_t *)gPru_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER);
       rgval |= CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT_SHARE_EN_MASK;
       HW_WR_REG32((uint8_t *)gPru_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)gPru_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER);
        rgval |= CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT_SHARE_EN_MASK;
      HW_WR_REG32((uint8_t *)gPru_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, rgval);
    }

}
HDSL_Handle HDSL_open(PRUICSS_Handle icssgHandle, uint32_t icssCore,uint8_t PRU_mode)
{
    /*
        HDSL memory map:
        RTU_PRU core:   0x0000 - 0x06FF
        PRU core:       0x0700 - 0x0DFF
        TX_PRU core:    0x0E00 - 0x1500
    */
    uint32_t DMEM_BASE_OFFSET_RTU_PRU1=0;
    uint32_t DMEM_BASE_OFFSET_PRU1=0x700;
    uint32_t DMEM_BASE_OFFSET_TX_PRU1=0xE00;
    HDSL_Handle hdslHandle;
    if (PRU_mode==0)
    {
        hdslHandle = &hdslConfig0;
        hdslHandle->baseMemAddr = (uint32_t *)(((PRUICSS_HwAttrs *)(icssgHandle->hwAttrs))->pru1DramBase);
    }
    else
    {
        if(icssCore == PRUICSS_RTU_PRU1)
        {
            hdslHandle = &hdslConfig0;
            hdslHandle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssgHandle->hwAttrs))->pru1DramBase)+DMEM_BASE_OFFSET_RTU_PRU1);
        }
        else if(icssCore == PRUICSS_PRU1)
        {
            hdslHandle = &hdslConfig1;
            hdslHandle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssgHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_PRU1);
        }
        else if(icssCore == PRUICSS_TX_PRU1)
        {
            hdslHandle = &hdslConfig2;
            hdslHandle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssgHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_TX_PRU1);
        }
        else
        {
            hdslHandle = NULL;
        }
    }

    if (hdslHandle != NULL)
    {
        hdslHandle->icssgHandle   = icssgHandle;
        hdslHandle->icssCore      = icssCore;
        hdslHandle->hdslInterface = (HDSL_Interface *) hdslHandle->baseMemAddr;
        hdslHandle->multi_turn    = 0;
    }

    return hdslHandle;
}

void HDSL_iep_init(HDSL_Handle hdslHandle)
{
    HW_WR_REG32((uint32_t)(((PRUICSS_HwAttrs *)(hdslHandle->icssgHandle->hwAttrs))->iep0RegBase) + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG,
                (CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CNT_ENABLE_MASK | (1 << CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT)));
    /* Use OCP as IEP CLK src */
    HW_WR_REG32((uint32_t)(((PRUICSS_HwAttrs *)(hdslHandle->icssgHandle->hwAttrs))->cfgRegBase) + CSL_ICSSCFG_IEPCLK, CSL_ICSSCFG_IEPCLK_OCP_EN_MASK);
}

int HDSL_enable_sync_signal(uint8_t ES, uint32_t period)
{
    /*program here*/
    uint32_t start_time = 10000;

    uint32_t inEvent;
    uint32_t outEvent_latch;
    uint32_t outEvent_gpio;
    uint32_t iep_base = CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE;

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

uint64_t HDSL_get_pos(HDSL_Handle hdslHandle, int position_id)
{
    uint64_t val;
    HDSL_Interface *hdslInterfaceStruct = hdslHandle->hdslInterface;
    switch(position_id){
        case 0:
            /* Fast Position */
                    val = hdslInterfaceStruct->POS0 | (hdslInterfaceStruct->POS1 << 8) |
                           (hdslInterfaceStruct->POS2 << 16) | (hdslInterfaceStruct->POS3 << 24);
                    val |= (uint64_t)hdslInterfaceStruct->POS4 << 32;
                    return val;

                break;

        case 1:
            /* Safe Position 1 */
                    val = hdslInterfaceStruct->VPOS0 | (hdslInterfaceStruct->VPOS1 << 8) |
                           (hdslInterfaceStruct->VPOS2 << 16) | (hdslInterfaceStruct->VPOS3 << 24);
                    val |= (uint64_t)hdslInterfaceStruct->VPOS4 << 32;
                    return val;

                break;

        case 2:
            /* Safe Position 2 */
                    val = hdslInterfaceStruct->VPOS20 | (hdslInterfaceStruct->VPOS21 << 8) |
                           (hdslInterfaceStruct->VPOS22 << 16) | (hdslInterfaceStruct->VPOS23 << 24);
                    val |= (uint64_t)hdslInterfaceStruct->VPOS24 << 32;

                    return val;
                break;

        default:
                return -1;
    }
}

uint8_t HDSL_get_qm(HDSL_Handle hdslHandle)
{
    uint8_t ureg = hdslHandle->hdslInterface->MASTER_QM & 0xF;
    return ureg;
}


uint16_t HDSL_get_events(HDSL_Handle hdslHandle)
{
    uint16_t ureg = hdslHandle->hdslInterface->EVENT_L | (hdslHandle->hdslInterface->EVENT_H << 8);
    return ureg;
}

uint8_t HDSL_get_safe_events(HDSL_Handle hdslHandle)
{
    return hdslHandle->hdslInterface->EVENT_S;
}

uint16_t HDSL_get_online_status_d(HDSL_Handle hdslHandle)
{
    uint16_t ureg = hdslHandle->hdslInterface->ONLINE_STATUS_D;
    return ureg;
}

uint16_t HDSL_get_online_status_1(HDSL_Handle hdslHandle)
{
    uint16_t ureg =hdslHandle->hdslInterface->ONLINE_STATUS_1;
    return ureg;
}

uint16_t HDSL_get_online_status_2(HDSL_Handle hdslHandle)
{
    uint16_t ureg = hdslHandle->hdslInterface->ONLINE_STATUS_2;
    return ureg;
}
uint8_t HDSL_get_sum(HDSL_Handle hdslHandle)
{
    uint8_t ureg = hdslHandle->hdslInterface->SAFE_SUM;
    return ureg;
}


uint8_t HDSL_get_acc_err_cnt(HDSL_Handle hdslHandle)
{
    return (uint8_t) (hdslHandle->hdslInterface->ACC_ERR_CNT & 0x1F);
}

uint8_t HDSL_get_rssi(HDSL_Handle hdslHandle)
{
    uint8_t ureg = (hdslHandle->hdslInterface->DELAY & 0xF0) >> 4;
    return ureg;
}

int32_t HDSL_write_pc_short_msg(HDSL_Handle hdslHandle,uint8_t addr, uint8_t data, uint64_t timeout)
{
    uint64_t end;
    end = ClockP_getTimeUsec() + timeout;

    while((hdslHandle->hdslInterface->EVENT_S & 0x1) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    hdslHandle->hdslInterface->S_PC_DATA = data;
    hdslHandle->hdslInterface->SLAVE_REG_CTRL =  addr;
    while((hdslHandle->hdslInterface->EVENT_S & 0x1) != 0)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    while((hdslHandle->hdslInterface->EVENT_S & 0x1) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    return SystemP_SUCCESS;
}

int32_t HDSL_read_pc_short_msg(HDSL_Handle hdslHandle,uint8_t addr, uint8_t *data, uint64_t timeout)
{

    uint64_t end;
    end = ClockP_getTimeUsec() + timeout;

    while((hdslHandle->hdslInterface->EVENT_S & 0x1) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    hdslHandle->hdslInterface->S_PC_DATA = 0;
    hdslHandle->hdslInterface->SLAVE_REG_CTRL =  (addr | (1<<7));
    while((hdslHandle->hdslInterface->EVENT_S & 0x1) != 0)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    while((hdslHandle->hdslInterface->EVENT_S & 0x1) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    *data = hdslHandle->hdslInterface->S_PC_DATA;
    return SystemP_SUCCESS;
}

uint8_t HDSL_read_pc_buffer(HDSL_Handle hdslHandle, uint8_t buff_off)
{
    switch(buff_off)
    {
    case 0:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER0);
        break;
    case 1:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER1);
        break;
    case 2:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER2);
        break;
    case 3:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER3);
        break;
    case 4:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER4);
        break;
    case 5:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER5);
        break;
    case 6:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER6);
        break;
    case 7:
        return (uint8_t) (hdslHandle->hdslInterface->PC_BUFFER7);
        break;
    default:
        return 0;
        break;
    }
}

void HDSL_write_pc_buffer(HDSL_Handle hdslHandle, uint8_t pc_buf0, uint8_t pc_buf1, uint8_t pc_buf2, uint8_t pc_buf3, uint8_t pc_buf4, uint8_t pc_buf5, uint8_t pc_buf6, uint8_t pc_buf7)
{
    hdslHandle->hdslInterface->PC_BUFFER0 = pc_buf0;
    hdslHandle->hdslInterface->PC_BUFFER1 = pc_buf1;
    hdslHandle->hdslInterface->PC_BUFFER2 = pc_buf2;
    hdslHandle->hdslInterface->PC_BUFFER3 = pc_buf3;
    hdslHandle->hdslInterface->PC_BUFFER4 = pc_buf4;
    hdslHandle->hdslInterface->PC_BUFFER5 = pc_buf5;
    hdslHandle->hdslInterface->PC_BUFFER6 = pc_buf6;
    hdslHandle->hdslInterface->PC_BUFFER7 = pc_buf7;
}

uint8_t HDSL_get_sync_ctrl(HDSL_Handle hdslHandle)
{
    return (uint8_t) (hdslHandle->hdslInterface->SYNC_CTRL);
}

void HDSL_set_sync_ctrl(HDSL_Handle hdslHandle, uint8_t val)
{
    hdslHandle->hdslInterface->SYNC_CTRL = val;
}

uint8_t HDSL_get_master_qm(HDSL_Handle hdslHandle)
{
    return (uint8_t) hdslHandle->hdslInterface->MASTER_QM;
}

uint8_t HDSL_get_edges(HDSL_Handle hdslHandle)
{
    return (uint8_t) hdslHandle->hdslInterface->EDGES;
}

void HDSL_set_pc_addr(HDSL_Handle hdslHandle, uint8_t pc_addrh, uint8_t pc_addrl, uint8_t pc_offh, uint8_t pc_offl)
{
    hdslHandle->hdslInterface->PC_ADD_L = pc_addrl;
    hdslHandle->hdslInterface->PC_ADD_H = pc_addrh;

    hdslHandle->hdslInterface->PC_OFF_L = pc_offl;
    hdslHandle->hdslInterface->PC_OFF_H = pc_offh;
}

void HDSL_set_pc_ctrl(HDSL_Handle hdslHandle, uint8_t value)
{
    hdslHandle->hdslInterface->PC_CTRL = value;
}

uint8_t HDSL_get_delay(HDSL_Handle hdslHandle)
{
    return (uint8_t) hdslHandle->hdslInterface->DELAY;
}

uint8_t HDSL_get_enc_id(HDSL_Handle hdslHandle, int byte)
{
    switch(byte)
    {
        case 0:
            return (uint8_t) hdslHandle->hdslInterface->ENC_ID0;
        case 1:
            return (uint8_t) hdslHandle->hdslInterface->ENC_ID1;
        case 2:
            return (uint8_t) hdslHandle->hdslInterface->ENC_ID2;
        default:
            return -1;
    }
}

void* HDSL_get_src_loc(HDSL_Handle hdslHandle)
{
    /* returns HDSL interface struct memory location */
    return (void *)hdslHandle->hdslInterface;
}

uint32_t HDSL_get_length(HDSL_Handle hdslHandle)
{
    return sizeof(*(hdslHandle->hdslInterface));
}

