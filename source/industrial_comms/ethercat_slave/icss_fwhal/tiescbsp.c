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
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h> // For memset/memcpy

/* get the firmware loaded as header files! */
#include <ecat_frame_handler_bin.h>  // EtherCAT frame handler
#include <ecat_host_interface_bin.h> // EtherCAT host interface

#include <industrial_comms/ethercat_slave/icss_fwhal/tiesc_pruss_intc_mapping.h>
#include <industrial_comms/ethercat_slave/icss_fwhal/tiescbsp.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/mdio.h>
#include <drivers/spinlock.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* DMEM 0 */
#define SYNC_PERMISSION_UPDATE_ADDR_OFFSET      0x500
#define SYNC_PERMISSION_UPDATE_ECAT_OFFSET      0x518
#define SYNC_PERMISSION_UPDATE_PDI_OFFSET       0x522
#define SYNC_PERMISSION_UPDATE_ECAT_SIZE        10
#define SYNC_PERMISSION_UPDATE_PDI_SIZE         10

/* TX Minimum Inter packet gap for MII 100M speed */
#define TIESC_TX_MIN_IPG                        (0x17U)

static bsp_params g_bsp_params;

static t_sm_properties sm_properties[MAX_SYNC_MAN];

static SemaphoreP_Object semcmdlow_object;

#ifndef USE_ECAT_TIMER
volatile uint32_t ecat_timer_inc_p_ms;
#endif

volatile uint32_t *pru_frame_proc = (uint32_t *)FrameProc;
volatile uint32_t *pru_host_proc = (uint32_t *)HostProc;
volatile uint32_t pru_frame_proc_len = sizeof(FrameProc);
volatile uint32_t pru_host_proc_len = sizeof(HostProc);

static uint32_t current_low, prev_low;//Current and previous IEP time stamps

uint32_t pd_read_addr_err, pd_write_addr_err;
uint32_t pdi_read_fail_cnt, pdi_write_fail_cnt;

static uint8_t eeprom_updated = 0;
static uint32_t eeprom_updated_time = 0;

uint8_t eeprom_cache[TIESC_EEPROM_SIZE];

#ifdef ENABLE_PDI_REG_PERMISSIONS
uint16_t pdi_reg_perm_array[256];
#endif

#if ECAT_TIMER_INT
void APPL_1MsTimerIsr(void);
#endif

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

void bsp_esc_sync_reg_perm_update(PRUICSS_Handle pruIcssHandle);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */


void * tiesc_memcpy(uint8_t *dst, const uint8_t *src, uint32_t size_bytes)
{
    memcpy(dst, src, size_bytes);
    return dst;
}

void * tiesc_memset(uint8_t *dst, int8_t val, uint32_t size_bytes)
{
    memset(dst, val, size_bytes);
    return dst;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief Interrupt service routine for the interrupts from the EtherCAT Slave Controller
*////////////////////////////////////////////////////////////////////////////////////////

void EcatIsr(void *args)
{
#ifndef ENABLE_PDI_TASK
    g_bsp_params.pdi_isr();
#endif
    bsp_clear_digio_out((PRUICSS_Handle)args, PDI_ISR_EDIO_NUM);
    PRUICSS_clearEvent((PRUICSS_Handle)args, PRU_ARM_EVENT0);

    ASSERT_DMB();
    ASSERT_DSB();
}

#ifndef SUPPORT_CMDACK_POLL_MODE
void EscCmdLowAckIsr(void *args)
{
    PRUICSS_clearEvent((PRUICSS_Handle)args, PRU_ARM_EVENT2);
    ASSERT_DMB();
    ASSERT_DSB();

}
#endif

void Sync0Isr(void *args)
{
    PRUICSS_clearEvent((PRUICSS_Handle)args, SYNC0_OUT_EVENT);
    ASSERT_DMB();
    ASSERT_DSB();

#ifndef ENABLE_SYNC_TASK
    g_bsp_params.sync0_isr();
#endif
}

inline void Sync1Isr(void *args)
{
    PRUICSS_clearEvent((PRUICSS_Handle)args, SYNC1_OUT_EVENT);
    ASSERT_DMB();
    ASSERT_DSB();

#ifndef ENABLE_SYNC_TASK
    g_bsp_params.sync1_isr();
#endif
}

inline uint32_t bsp_get_timer_register(void)
{
    uint32_t ret;
#ifdef USE_ECAT_TIMER
    bsp_get_local_sys_time(&current_low, 0);
#else
    current_low = OSAL_Timestamp_get32();
#endif

    if(current_low > prev_low)
    {
        ret = current_low - prev_low;
    }

    else
    {
        ret = (0xFFFFFFFF - prev_low) + current_low;
    }

    return ret;
}

inline void bsp_clear_timer_register(void)
{
#ifdef USE_ECAT_TIMER
    bsp_get_local_sys_time(&prev_low, 0);
#else
    prev_low = OSAL_Timestamp_get32();
#endif
}

void bsp_get_local_sys_time(uint32_t *systime_low, uint32_t *systime_high)
{
    uint32_t ecat_systime_low, ecat_systime_high;
    volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
            ((((PRUICSS_HwAttrs *)((g_bsp_params.pruicss_handle)->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(0));

    if(systime_high)
    {
        bsp_send_command_to_firmware((g_bsp_params.pruicss_handle), CMD_DL_USER_READ_SYS_TIME, 0, 0);
        bsp_global_mutex_lock();
        ecat_systime_low = pHost2PruIntfc->system_time_low;
        ecat_systime_high = pHost2PruIntfc->system_time_high;
        bsp_global_mutex_unlock();
    }

    else
    {
        uint32_t iep_adjust_offset_value = 0;
        ecat_systime_low = bsp_pruss_iepreg_read((g_bsp_params.pruicss_handle), CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0) - iep_adjust_offset_value;
    }

    if(systime_low)
    {
        *systime_low =  ecat_systime_low;
    }

    if(systime_high)
    {
        *systime_high = ecat_systime_high;
    }
}

void bsp_get_latch0_posedge_time(PRUICSS_Handle pruIcssHandle,
                                 uint32_t *systime_low, uint32_t *systime_high)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_array(ESC_ADDR_LATCH0_POS_EDGE, PDI_PERM_READ,
                                      8))
    {
        pdi_read_fail_cnt++;
        return;
    }
#endif
    while(bsp_hwspinlock_lock(0));

    bsp_global_mutex_lock();

    if(systime_low)
    {
        *systime_low  = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH0_POS_EDGE) >> 2)]);
    }

    if(systime_high)
    {
        *systime_high = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH0_POS_EDGE + 4) >> 2)]);
    }

    bsp_global_mutex_unlock();
    bsp_hwspinlock_unlock(0);
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                 LATCH0_POS_EDGE, 0);
}

void bsp_get_latch0_negedge_time(PRUICSS_Handle pruIcssHandle,
                                 uint32_t *systime_low, uint32_t *systime_high)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_array(ESC_ADDR_LATCH0_NEG_EDGE, PDI_PERM_READ,
                                      8))
    {
        pdi_read_fail_cnt++;
        return;
    }
#endif
    while(bsp_hwspinlock_lock(0));

    bsp_global_mutex_lock();

    if(systime_low)
    {
        *systime_low  = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH0_NEG_EDGE) >> 2)]);
    }

    if(systime_high)
    {
        *systime_high = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH0_NEG_EDGE + 4) >> 2)]);
    }

    bsp_global_mutex_unlock();
    bsp_hwspinlock_unlock(0);
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                 LATCH0_NEG_EDGE, 0);
}
void bsp_get_latch1_posedge_time(PRUICSS_Handle pruIcssHandle,
                                 uint32_t *systime_low, uint32_t *systime_high)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_array(ESC_ADDR_LATCH1_POS_EDGE, PDI_PERM_READ,
                                      8))
    {
        pdi_read_fail_cnt++;
        return;
    }
#endif
    while(bsp_hwspinlock_lock(0));

    bsp_global_mutex_lock();

    if(systime_low)
    {
        *systime_low  = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH1_POS_EDGE) >> 2)]);
    }

    if(systime_high)
    {
        *systime_high = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH1_POS_EDGE + 4) >> 2)]);
    }

    bsp_global_mutex_unlock();
    bsp_hwspinlock_unlock(0);
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                 LATCH1_POS_EDGE, 0);
}
void bsp_get_latch1_negedge_time(PRUICSS_Handle pruIcssHandle,
                                 uint32_t *systime_low, uint32_t *systime_high)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_array(ESC_ADDR_LATCH1_NEG_EDGE, PDI_PERM_READ,
                                      8))
    {
        pdi_read_fail_cnt++;
        return;
    }
#endif
    while(bsp_hwspinlock_lock(0));

    bsp_global_mutex_lock();

    if(systime_low)
    {
        *systime_low  = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH1_NEG_EDGE) >> 2)]);
    }

    if(systime_high)
    {
        *systime_high = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                            pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                                    ESC_ADDR_LATCH1_NEG_EDGE + 4) >> 2)]);
    }

    bsp_global_mutex_unlock();
    bsp_hwspinlock_unlock(0);
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                 LATCH1_NEG_EDGE, 0);
}

#ifdef SYSTEM_TIME_PDI_CONTROLLED
void bsp_pdi_write_system_time(PRUICSS_Handle pruIcssHandle, uint32_t systime)
{
    volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
            ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(0));
    *(uint32_t *)(&pHost2PruIntfc->resp1low) = systime;
    ASSERT_DMB();

    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_SYSTIME_PDI_CONTROL,
                                 WRITE_SYSTIME, 0);
}
void bsp_pdi_write_system_timeoffset(PRUICSS_Handle pruIcssHandle,
                                     unsigned long long systime)
{
    bsp_write(pruIcssHandle, (uint8_t *)&systime, ESC_ADDR_SYSTIME_OFFSET, 8);
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_SYSTIME_PDI_CONTROL,
                                 WRITE_SYSTIME_OFFSET, 0);
}
void bsp_pdi_write_systime_delay(PRUICSS_Handle pruIcssHandle,
                                 uint32_t systime)
{
    bsp_write_dword(pruIcssHandle, systime, ESC_ADDR_SYSTIME_DELAY);
}
void bsp_pdi_write_filterconfig(PRUICSS_Handle pruIcssHandle,
                                uint16_t speedcount_start, uint8_t speedcount_filtdepth,
                                uint8_t systime_filtdepth)
{
    bsp_write_word(pruIcssHandle, speedcount_start, ESC_ADDR_SPEEDCOUNTER_START);
    bsp_write_word(pruIcssHandle,
                   (speedcount_filtdepth << 8 | systime_filtdepth),
                   ESC_ADDR_TIMEDIFF_FILTDEPTH);
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_SYSTIME_PDI_CONTROL,
                                 WRITE_FILTER_CONFIG, 0);
}

void bsp_pdi_latch0_control(PRUICSS_Handle pruIcssHandle, uint8_t val)
{
    uint32_t reg_val = HW_RD_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase +
                                    CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG);

    if(val & 0x1)  //Latch0 Pos Edge single shot
    {
        reg_val |= 0x40;
    }

    else
    {
        reg_val &= ~0x40;
    }

    if(val & 0x2)  //Latch0 Neg Edge single shot
    {
        reg_val |= 0x80;
    }

    else
    {
        reg_val &= ~0x80;
    }

    HW_WR_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase +
          CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG, reg_val);
    ASSERT_DMB();

}

void bsp_pdi_latch1_control(PRUICSS_Handle pruIcssHandle, uint8_t val)
{
    uint32_t reg_val = HW_RD_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase +
                                   CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG);

    if(val & 0x1)   //Latch1 Pos Edge single shot
    {
        reg_val |= 0x100;
    }

    else
    {
        reg_val &= ~0x100;
    }

    if(val & 0x2)  //Latch1 Neg Edge single shot
    {
        reg_val |= 0x200;
    }

    else
    {
        reg_val &= ~0x200;
    }

    HW_WR_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase +
          CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG, reg_val);
    ASSERT_DMB();

}
#endif
inline void bsp_set_pdi_wd_trigger_mode(PRUICSS_Handle pruIcssHandle,
                                        uint32_t mode)
{
    //PDI WD trigger also depends on DIGIO h/w events in ICSS and by default triggered on RX SOF events on both ports
    //This function can be used to override this behaviour in h/w - it is not possible to disable DIGIO dependency and make
    //PDI WD trigger manually in firmware for every command send - so allow user to configure - trigger mechansim
    //LATCH_IN  event or SYNC0/1_out - New application default is LATCH_IN event

    HW_WR_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_CTRL_REG, mode);
    ASSERT_DMB();

}

inline void bsp_set_digio_sw_dataout_enable(PRUICSS_Handle pruIcssHandle)
{
    uint32_t regval;
    regval = HW_RD_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_EXP_REG);
    regval |= 1;
    HW_WR_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_EXP_REG, regval);
    ASSERT_DMB();

}

inline void bsp_set_digio_out(PRUICSS_Handle pruIcssHandle, uint8_t num)
{
    uint32_t regval;
    regval = HW_RD_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_DATA_OUT_REG);
    regval |= (1 << num);
    HW_WR_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_DATA_OUT_REG, regval);
    ASSERT_DMB();

}

inline void bsp_clear_digio_out(PRUICSS_Handle pruIcssHandle, uint8_t num)
{
    uint32_t regval;
    regval = HW_RD_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_DATA_OUT_REG);
    regval &= ~(1 << num);
    HW_WR_REG32(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_DIGIO_DATA_OUT_REG, regval);
    ASSERT_DMB();

}

void bsp_eeprom_emulation_init(void)
{
    uint32_t itr = 0;
    uint16_t u16Crc = 0x00FF, i, j;
    int invalid_crc_flag = 0;

    if(SystemP_SUCCESS != g_bsp_params.eeprom_read((uint8_t *)eeprom_cache, TIESC_EEPROM_SIZE))
    {
        invalid_crc_flag = 1;
    }
    else
    {
        uint32_t temp_reg = 0;

        for(i = 0; i < 14; i++)
        {
            u16Crc ^= eeprom_cache[i];

            for(j = 0; j < 8; j++)
            {
                if(u16Crc & 0x80)
                {
                    u16Crc = (u16Crc << 1) ^ 0x07;
                }

                else
                {
                    u16Crc <<= 1;
                }
            }
        }

        /*only low Byte shall be written*/
        u16Crc &= 0x00FF;

        if(u16Crc != eeprom_cache[14])
        {
            invalid_crc_flag = 1;
        }

        /*This check compares the data at offset 24*/
        temp_reg = *(uint32_t *)((uint8_t *)(eeprom_cache + (0xC << 1)));
        if(temp_reg != (uint8_t)(*((uint8_t *)(g_bsp_params.default_tiesc_eeprom + 24))))
        {
            invalid_crc_flag = 1;
        }
    }

    if(1 == invalid_crc_flag)
    {

        for(itr = 0; itr < TIESC_EEPROM_SIZE; itr++)
        {
            eeprom_cache[itr] = (uint8_t)(*((uint8_t *)(g_bsp_params.default_tiesc_eeprom + itr)));
        }

    }

#if ESC_EEPROM_EMULATION
    *(g_bsp_params.eeprom_pointer_for_stack) = eeprom_cache;
#endif
}

int32_t bsp_eeprom_load_esc_registers(PRUICSS_Handle pruIcssHandle,
                                      int32_t reload_flag)
{
    //Validate CRC before loading to ESC registers
    uint16_t u16Crc = 0x00FF, i, j;
    int32_t invalid_crc_flag = 0;

    for(i = 0; i < 14; i++)
    {
        u16Crc ^= eeprom_cache[i];

        for(j = 0; j < 8; j++)
        {
            if(u16Crc & 0x80)
            {
                u16Crc = (u16Crc << 1) ^ 0x07;
            }

            else
            {
                u16Crc <<= 1;
            }
        }
    }

    /*only low Byte shall be written*/
    u16Crc &= 0x00FF;

    if(u16Crc != eeprom_cache[14])
    {
        //printf("Error: EEPROM validation failed for config area\r\n");
        invalid_crc_flag = 1;
    }

    /*write new claculated Crc to Esc Config area*/
    if(!reload_flag)
    {
        //Not a reload operation - but initial loading
        //0x150 and 0x152 not reloaded according to beckhoff
        if(!invalid_crc_flag)
        {
            bsp_write_byte(pruIcssHandle, eeprom_cache[8], ESC_ADDR_CONFIG_STATION_ALIAS);
            bsp_write_byte(pruIcssHandle, eeprom_cache[9],
                           ESC_ADDR_CONFIG_STATION_ALIAS + 1);
            bsp_write_byte(pruIcssHandle, eeprom_cache[0], ESC_ADDR_PDI_CONTROL);
            bsp_write_byte(pruIcssHandle, eeprom_cache[1], ESC_ADDR_PDI_CONTROL + 1);
        }

        else
        {
            return -1;
        }
    }

    if(!invalid_crc_flag)
    {
        bsp_write_byte(pruIcssHandle, eeprom_cache[4], ESC_ADDR_SYNC_PULSE_LENGTH);
        bsp_write_byte(pruIcssHandle, eeprom_cache[5],
                       ESC_ADDR_SYNC_PULSE_LENGTH + 1);
    }

    else
    {
        return -1;
    }

    return 0;
}
int32_t bsp_eeprom_emulation_reload(PRUICSS_Handle pruIcssHandle)
{
    return bsp_eeprom_load_esc_registers(pruIcssHandle, 1);
}

void bsp_eeprom_emulation_command_ack(PRUICSS_Handle pruIcssHandle)
{
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_EEPROM_CMD_ACK, 0, 0);
}

void bsp_eeprom_emulation_flush(void)
{
    g_bsp_params.eeprom_write((uint8_t *)eeprom_cache, TIESC_EEPROM_SIZE);
}

void bsp_eeprom_emulation_exit(void)
{
    bsp_eeprom_emulation_flush();
}

void bsp_esc_reg_perm_init(PRUICSS_Handle pruIcssHandle)
{
    int i, j = 0;
    (void) j; /*To remove unused variable warning */
    volatile t_register_properties *pRegPerm = (volatile t_register_properties *)
            ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(1));

    for(i = 0; i < 10; i++)
    {
        pRegPerm->reg_properties[i] = TIESC_PERM_READ_ONLY;
    }

    pRegPerm->reg_properties[0x10] = TIESC_PERM_RW; //Configured Station address
    pRegPerm->reg_properties[0x11] = TIESC_PERM_RW; //Configured Station address
    pRegPerm->reg_properties[0x12] =
        TIESC_PERM_READ_ONLY; //Configured Station alias
    pRegPerm->reg_properties[0x13] =
        TIESC_PERM_READ_ONLY; //Configured Station alias
    pRegPerm->reg_properties[0x100] = TIESC_PERM_RW; //DL control
    pRegPerm->reg_properties[0x101] = TIESC_PERM_RW; //DL control
    pRegPerm->reg_properties[0x102] = TIESC_PERM_RW; //DL control
    pRegPerm->reg_properties[0x103] = TIESC_PERM_RW; //DL control
    pRegPerm->reg_properties[0x108] = TIESC_PERM_RW; //Physical RW offset
    pRegPerm->reg_properties[0x109] = TIESC_PERM_RW; //Physical RW offset
    pRegPerm->reg_properties[0x110] = TIESC_PERM_READ_ONLY; //ESC DL status
    pRegPerm->reg_properties[0x111] = TIESC_PERM_READ_ONLY; //ESC DL status
    pRegPerm->reg_properties[0x120] = TIESC_PERM_RW; //AL control
    pRegPerm->reg_properties[0x121] = TIESC_PERM_RW; //AL control
    pRegPerm->reg_properties[0x130] = TIESC_PERM_READ_ONLY; //ESC AL status
    pRegPerm->reg_properties[0x131] = TIESC_PERM_READ_ONLY; //ESC AL status
    pRegPerm->reg_properties[0x134] = TIESC_PERM_READ_ONLY; //ESC AL status code
    pRegPerm->reg_properties[0x135] = TIESC_PERM_READ_ONLY; //ESC AL status code
    pRegPerm->reg_properties[0x140] = TIESC_PERM_READ_ONLY; //ESC PDI control
    pRegPerm->reg_properties[0x141] = TIESC_PERM_READ_ONLY; //ESC configuration
    pRegPerm->reg_properties[0x150] = TIESC_PERM_READ_ONLY; //Onchip configuration
    pRegPerm->reg_properties[0x151] =
        TIESC_PERM_READ_ONLY; //SYNC/LATCH PDI configuration
    pRegPerm->reg_properties[0x152] =
        TIESC_PERM_READ_ONLY; //Onchip extended configuration
    pRegPerm->reg_properties[0x153] =
        TIESC_PERM_READ_ONLY; //Onchip extended configuration
    pRegPerm->reg_properties[0x200] = TIESC_PERM_RW; //ECAT event mask
    pRegPerm->reg_properties[0x201] = TIESC_PERM_RW; //ECAT event mask
    pRegPerm->reg_properties[0x204] = TIESC_PERM_READ_ONLY; //AL event mask
    pRegPerm->reg_properties[0x205] = TIESC_PERM_READ_ONLY; //AL event mask
    pRegPerm->reg_properties[0x206] = TIESC_PERM_READ_ONLY; //AL event mask
    pRegPerm->reg_properties[0x207] = TIESC_PERM_READ_ONLY; //AL event mask
    pRegPerm->reg_properties[0x210] = TIESC_PERM_READ_ONLY; //ECAT event request
    pRegPerm->reg_properties[0x211] = TIESC_PERM_READ_ONLY; //ECAT event request
    pRegPerm->reg_properties[0x220] = TIESC_PERM_READ_ONLY; //AL event request
    pRegPerm->reg_properties[0x221] = TIESC_PERM_READ_ONLY; //AL event request
    pRegPerm->reg_properties[0x222] = TIESC_PERM_READ_ONLY; //AL event request
    pRegPerm->reg_properties[0x223] = TIESC_PERM_READ_ONLY; //AL event request
    pRegPerm->reg_properties[0x300] = TIESC_PERM_RW; //Invalid frame counter Port0
    pRegPerm->reg_properties[0x301] = TIESC_PERM_RW; //RX_ERR counter Port0
    pRegPerm->reg_properties[0x302] = TIESC_PERM_RW; //Invalid frame counter Port1
    pRegPerm->reg_properties[0x303] = TIESC_PERM_RW; //RX_ERR counter Port1
    pRegPerm->reg_properties[0x308] = TIESC_PERM_RW; //Forwarded Error Port0
    pRegPerm->reg_properties[0x309] = TIESC_PERM_RW; //Forwarded Error Port1
    pRegPerm->reg_properties[0x30C] = TIESC_PERM_RW; //ECAT processing unit counter
    pRegPerm->reg_properties[0x310] = TIESC_PERM_RW; //Link lost counter Port0
    pRegPerm->reg_properties[0x311] = TIESC_PERM_RW; //Link lost counter Port1
    pRegPerm->reg_properties[0x400] = TIESC_PERM_RW; //Watchdog divider
    pRegPerm->reg_properties[0x401] = TIESC_PERM_RW; //Watchdog divider
    pRegPerm->reg_properties[0x410] = TIESC_PERM_RW; //Watchdog time PDI
    pRegPerm->reg_properties[0x411] = TIESC_PERM_RW; //Watchdog time PDI
    pRegPerm->reg_properties[0x420] = TIESC_PERM_RW; //Watchdog time PD
    pRegPerm->reg_properties[0x421] = TIESC_PERM_RW; //Watchdog time PD
    pRegPerm->reg_properties[0x440] = TIESC_PERM_READ_ONLY; //Watchdog process data
    pRegPerm->reg_properties[0x441] = TIESC_PERM_READ_ONLY; //Watchdog process data
    pRegPerm->reg_properties[0x442] = TIESC_PERM_RW; //Watchdog counter PD
    pRegPerm->reg_properties[0x443] = TIESC_PERM_RW; //Watchdog counter PDI
    pRegPerm->reg_properties[0x500] = TIESC_PERM_RW; //EEPROM configuration
    pRegPerm->reg_properties[0x501] =
        TIESC_PERM_READ_ONLY; //EEPROM PDI access state

    for(i = 0; i < 8; i++)
    {
        pRegPerm->reg_properties[0x502 + i] = TIESC_PERM_RW;
    }

    for(i = 0; i < 6; i++)
    {
        pRegPerm->reg_properties[0x50A + i] = TIESC_PERM_READ_ONLY;
    }

    for(i = 0; i < 6; i++)
    {
        pRegPerm->reg_properties[0x510 + i] = TIESC_PERM_RW;
    }

    for(i = 0; i < 8; i++)    //8 FMMUs
    {
        tiesc_memset((uint8_t *)&pRegPerm->reg_properties[0x600 + i * 16], TIESC_PERM_RW,
               13);
    }

    for(i = 0; i < 8; i++)    //8 SMs
    {
        tiesc_memset((uint8_t *)&pRegPerm->reg_properties[0x800 + i * 8], TIESC_PERM_RW, 5);
        tiesc_memset((uint8_t *)&pRegPerm->reg_properties[0x805 + i * 8],
               TIESC_PERM_READ_ONLY, 1);
        tiesc_memset((uint8_t *)&pRegPerm->reg_properties[0x806 + i * 8], TIESC_PERM_RW, 1);
        tiesc_memset((uint8_t *)&pRegPerm->reg_properties[0x807 + i * 8],
               TIESC_PERM_READ_ONLY, 1);
    }

    pRegPerm->reg_properties[0x900] = TIESC_PERM_RW;

    for(i = 0; i < 15; i++)
    {
        pRegPerm->reg_properties[0x901 + i] = TIESC_PERM_READ_ONLY;
    }

#ifndef SYSTEM_TIME_PDI_CONTROLLED

    for(i = 0; i < 4; i++)
    {
        pRegPerm->reg_properties[0x910 + i] = TIESC_PERM_RW;
    }

    for(i = 0; i < 4; i++)
    {
        pRegPerm->reg_properties[0x914 + i] = TIESC_PERM_READ_ONLY;
    }

#else

    for(i = 0; i < 8; i++)
    {
        pRegPerm->reg_properties[0x910 + i] = TIESC_PERM_READ_ONLY;
    }

#endif

    for(i = 0; i < 8; i++)
    {
        pRegPerm->reg_properties[0x918 + i] = TIESC_PERM_READ_ONLY;
    }

#ifndef SYSTEM_TIME_PDI_CONTROLLED

    for(i = 0; i < 12; i++)
    {
        pRegPerm->reg_properties[0x920 + i] = TIESC_PERM_RW;
    }

#else

    for(i = 0; i < 12; i++)
    {
        pRegPerm->reg_properties[0x920 + i] = TIESC_PERM_READ_ONLY;
    }

#endif
    pRegPerm->reg_properties[0x92C] = TIESC_PERM_READ_ONLY; //System Time Difference
    pRegPerm->reg_properties[0x92D] = TIESC_PERM_READ_ONLY; //System Time Difference
    pRegPerm->reg_properties[0x92E] = TIESC_PERM_READ_ONLY; //System Time Difference
    pRegPerm->reg_properties[0x92F] = TIESC_PERM_READ_ONLY; //System Time Difference
#ifndef SYSTEM_TIME_PDI_CONTROLLED
    pRegPerm->reg_properties[0x930] = TIESC_PERM_RW; //Speed counter Start
    pRegPerm->reg_properties[0x931] = TIESC_PERM_RW; //Speed counter Start
#else
    pRegPerm->reg_properties[0x930] = TIESC_PERM_READ_ONLY; //Speed counter Start
    pRegPerm->reg_properties[0x931] = TIESC_PERM_READ_ONLY; //Speed counter Start
#endif
    pRegPerm->reg_properties[0x932] = TIESC_PERM_READ_ONLY; //Speed counter Diff
    pRegPerm->reg_properties[0x933] = TIESC_PERM_READ_ONLY; //Speed counter Diff

#ifndef SYSTEM_TIME_PDI_CONTROLLED
    pRegPerm->reg_properties[0x934] =
        TIESC_PERM_RW; //System Time Difference Filter Depth
    pRegPerm->reg_properties[0x935] = TIESC_PERM_RW; //Speed counter Filter Depth
#else
    pRegPerm->reg_properties[0x934] =
        TIESC_PERM_READ_ONLY; //System Time Difference Filter Depth
    pRegPerm->reg_properties[0x935] =
        TIESC_PERM_READ_ONLY; //Speed counter Filter Depth
#endif
    pRegPerm->reg_properties[0x980] = TIESC_PERM_RW;
#ifndef SYSTEM_TIME_PDI_CONTROLLED
    pRegPerm->reg_properties[0x981] = TIESC_PERM_RW; //Sync Activation
#else
    pRegPerm->reg_properties[0x981] = TIESC_PERM_READ_ONLY; //Sync Activation
#endif

    pRegPerm->reg_properties[0x982] =
        TIESC_PERM_READ_ONLY; //Pulse length of Sync Signals
    pRegPerm->reg_properties[0x983] =
        TIESC_PERM_READ_ONLY; //Pulse length of Sync Signals
    pRegPerm->reg_properties[0x98E] = TIESC_PERM_READ_ONLY; //SYNC0 status
    pRegPerm->reg_properties[0x98F] = TIESC_PERM_READ_ONLY; //SYNC1 status

#ifndef SYSTEM_TIME_PDI_CONTROLLED

    for(i = 0; i < 8; i++)
    {
        pRegPerm->reg_properties[0x990 + i] = TIESC_PERM_RW;
    }

#else

    for(i = 0; i < 8; i++)
    {
        pRegPerm->reg_properties[0x990 + i] = TIESC_PERM_READ_ONLY;
    }

#endif

    for(i = 0; i < 8; i++)
    {
        pRegPerm->reg_properties[0x998 + i] = TIESC_PERM_READ_ONLY;
    }

#ifndef SYSTEM_TIME_PDI_CONTROLLED

    for(i = 0; i < 10; i++)
    {
        pRegPerm->reg_properties[0x9A0 + i] = TIESC_PERM_RW;
    }

#else

    for(i = 0; i < 10; i++)
    {
        pRegPerm->reg_properties[0x9A0 + i] = TIESC_PERM_READ_ONLY;
    }

#endif

    for(i = 0; i < 34; i++)
    {
        pRegPerm->reg_properties[i + 0x9AE] = TIESC_PERM_READ_ONLY;
    }

    //TI vendor specific registers
    for(i = 0; i < 16; i++)
    {
        pRegPerm->reg_properties[0xE00 + i] = TIESC_PERM_READ_ONLY;
    }

    //TI vendor specific registers
    for(i = 0; i < 22; i++)
    {
        pRegPerm->reg_properties[0xE10 + i] = TIESC_PERM_RW;
    }

    pRegPerm->reg_properties[0xEE0] = TIESC_PERM_RW; //APP_RELOAD_FLAG_REG

#ifdef ENABLE_PDI_REG_PERMISSIONS
    tiesc_memset(&pdi_reg_perm_array[0], 0, 128);

    for(i = 0; i < 10; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array, i); //PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x10));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x12));// PDI_PERM_RW;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x100));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x108));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x110));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x120));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x130));// PDI_PERM_RW;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x134));// PDI_PERM_RW;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x140));// PDI_PERM_RW;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array,
                                    (i + 0x150));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x200));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x204));// PDI_PERM_RW;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x210));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x220));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 13; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x300));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x310));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x400));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x410));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x420));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x440));// PDI_PERM_READ_ONLY;
    }

    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x500);// PDI_PERM_READ_ONLY;

    for(i = 0; i < 15; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x501));// PDI_PERM_RW;
    }

    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x510);// PDI_PERM_READ_ONLY;

    for(i = 0; i < 6; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x511));// PDI_PERM_RW;
    }

    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x516);// PDI_PERM_READ_ONLY;
    bsp_set_pdi_perm_read_write(pdi_reg_perm_array, 0x517);// PDI_PERM_RW;

    for(i = 0; i < 8; i++)    //8 FMMUs
    {
        for(j = 0; j < 16; j++)
        {
            bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                       (0x600 + i * 16 + j));    //PDI_PERM_READ_ONLY
        }
    }

    for(i = 0; i < 8; i++)    //8 SMs
    {
        for(j = 0; j < 7; j++)
        {
            bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                       (0x800 + i * 8 + j));    //PDI_PERM_READ_ONLY
        }

        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (0x807 + i * 8)); //PDI_PERM_RW
    }

#ifndef SYSTEM_TIME_PDI_CONTROLLED

    for(i = 0; i < 54; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x900));// PDI_PERM_READ_ONLY;
    }

#else

    for(i = 0; i < 16; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x900));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 8; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x910));// PDI_PERM_RW;
    }

    for(i = 0; i < 8; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x918));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 12; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x920));// PDI_PERM_RW;
    }

    for(i = 0; i < 4; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x92c));// PDI_PERM_READ_ONLY;
    }

    bsp_set_pdi_perm_read_write(pdi_reg_perm_array, 0x930);// PDI_PERM_RW;
    bsp_set_pdi_perm_read_write(pdi_reg_perm_array, 0x931);// PDI_PERM_RW;
    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x932);// PDI_PERM_READ_ONLY;
    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x933);// PDI_PERM_READ_ONLY;
    bsp_set_pdi_perm_read_write(pdi_reg_perm_array, 0x934);// PDI_PERM_RW;
    bsp_set_pdi_perm_read_write(pdi_reg_perm_array, 0x935);// PDI_PERM_RW;
#endif

    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x981);// PDI_PERM_READ_ONLY;

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0x982));// PDI_PERM_RW;
    }

    bsp_set_pdi_perm_read_only(pdi_reg_perm_array, 0x984);// PDI_PERM_READ_ONLY;

    for(i = 0; i < 2; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x98E));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 26; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x990));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 34; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0x9AE));// PDI_PERM_READ_ONLY;
    }

    //TI vendor specific registers
    for(i = 0; i < 8; i++)
    {
        bsp_set_pdi_perm_read_only(pdi_reg_perm_array,
                                   (i + 0xE00));// PDI_PERM_READ_ONLY;
    }

    for(i = 0; i < 30; i++)
    {
        bsp_set_pdi_perm_read_write(pdi_reg_perm_array, (i + 0xE08));// PDI_PERM_RW;
    }

#endif
    ASSERT_DMB();


    bsp_esc_sync_reg_perm_update(pruIcssHandle);
}


void bsp_esc_sync_reg_perm_update(PRUICSS_Handle pruIcssHandle)
{
    uint16_t register_properties_base_offset = sizeof(((t_register_properties *)
            0)->reserved);

    uint8_t *pRegPermUpdateAddr = (uint8_t *)
                                  ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
                                   PRUICSS_DATARAM(0)) + SYNC_PERMISSION_UPDATE_ADDR_OFFSET;

    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) ESC_SYSTEMTIME_OFFSET + register_properties_base_offset);

    pRegPermUpdateAddr += 2;
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) 0x04);
    pRegPermUpdateAddr += 2;

    /*  Register System Time Offset */
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) ESC_SYSTEMTIME_OFFSET_OFFSET + register_properties_base_offset);
    pRegPermUpdateAddr += 2;
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) 0x04);
    pRegPermUpdateAddr += 2;

    /*  Register Speed Counter Start */
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) ESC_SPEED_COUNTER_START_OFFSET + register_properties_base_offset);
    pRegPermUpdateAddr += 2;
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) 0x06);
    pRegPermUpdateAddr += 2;

    /*  Register SYNC Activation register */
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) ESC_DC_SYNC_ACTIVATION_OFFSET + register_properties_base_offset);
    pRegPermUpdateAddr += 2;
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) 0x01);
    pRegPermUpdateAddr += 2;

    /*  Register Start Time Cyclic Operation */
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) ESC_DC_START_TIME_CYCLIC_OFFSET + register_properties_base_offset);
    pRegPermUpdateAddr += 2;
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) 0x08);
    pRegPermUpdateAddr += 2;

    /*  Register SYNCx Cycle Time */
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) ESC_DC_SYNC0_CYCLETIME_OFFSET + register_properties_base_offset);
    pRegPermUpdateAddr += 2;
    HW_WR_REG16(pRegPermUpdateAddr, (uint16_t) 0x0A);
    pRegPermUpdateAddr += 2;

    pRegPermUpdateAddr = (uint8_t *)
                         ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
                          PRUICSS_DATARAM(0)) + SYNC_PERMISSION_UPDATE_ECAT_OFFSET;

    tiesc_memset(pRegPermUpdateAddr, TIESC_PERM_RW, SYNC_PERMISSION_UPDATE_ECAT_SIZE);

    pRegPermUpdateAddr = (uint8_t *)
                         ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
                          PRUICSS_DATARAM(0)) + SYNC_PERMISSION_UPDATE_PDI_OFFSET;

    tiesc_memset(pRegPermUpdateAddr, TIESC_PERM_READ_ONLY,
           SYNC_PERMISSION_UPDATE_PDI_SIZE);
    ASSERT_DMB();

}


int16_t bsp_pruss_mdio_phy_read(PRUICSS_Handle pruIcssHandle, uint8_t phyaddr,
                              uint8_t regoffset, uint16_t *regval)
{
    if((bsp_read_byte(pruIcssHandle, ESC_ADDR_MI_ECAT_ACCESS) & 0x1)
            || (bsp_read_byte(pruIcssHandle, ESC_ADDR_MI_PDI_ACCESS) & 0x2))
    {
        return -1;
    }

    //Acquire PDI access over MDIO/MI interface
    bsp_write_byte(pruIcssHandle, bsp_read_byte(pruIcssHandle,
                   ESC_ADDR_MI_PDI_ACCESS) | 1, ESC_ADDR_MI_PDI_ACCESS);
    MDIO_phyRegRead((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->miiMdioRegBase),
                    NULL, phyaddr, regoffset, regval);

    //Release PDI access over MDIO/MI interface
    bsp_write_byte(pruIcssHandle, bsp_read_byte(pruIcssHandle,
                   ESC_ADDR_MI_PDI_ACCESS) & ~1, ESC_ADDR_MI_PDI_ACCESS);
    return 0;
}

int16_t bsp_pruss_mdio_phy_write(PRUICSS_Handle pruIcssHandle, uint8_t phyaddr,
                               uint8_t regoffset, uint16_t regval)
{
    if((bsp_read_byte(pruIcssHandle, ESC_ADDR_MI_ECAT_ACCESS) & 0x1)
            || (bsp_read_byte(pruIcssHandle, ESC_ADDR_MI_PDI_ACCESS) & 0x2))
    {
        return -1;
    }

    //Acquire PDI access over MDIO/MI interface
    bsp_write_byte(pruIcssHandle, bsp_read_byte(pruIcssHandle,
                   ESC_ADDR_MI_PDI_ACCESS) | 1, ESC_ADDR_MI_PDI_ACCESS);
    MDIO_phyRegWrite((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->miiMdioRegBase),
                     NULL, phyaddr, regoffset, regval);
    //Release PDI access over MDIO/MI interface
    bsp_write_byte(pruIcssHandle, bsp_read_byte(pruIcssHandle,
                   ESC_ADDR_MI_PDI_ACCESS) & ~1, ESC_ADDR_MI_PDI_ACCESS);
    return 0;
}

int16_t bsp_pruss_mdio_init(PRUICSS_Handle pruIcssHandle,
                          t_mdio_params *pmdio_params)
{
    if((bsp_read_byte(pruIcssHandle, ESC_ADDR_MI_ECAT_ACCESS) & 0x1)
            || (bsp_read_byte(pruIcssHandle, ESC_ADDR_MI_PDI_ACCESS) & 0x2))
    {
        return -1;
    }

    //Acquire PDI access over MDIO/MI interface
    bsp_write_byte(pruIcssHandle, bsp_read_byte(pruIcssHandle,
                   ESC_ADDR_MI_PDI_ACCESS) | 1, ESC_ADDR_MI_PDI_ACCESS);
    //Configure MDIO clock = 200/(clkdiv+1)
    // Disable preamble , enable fault detection

    //Indicate PHY address to firmware via vendor specfic registers
    bsp_write_byte(pruIcssHandle, pmdio_params->addr0, ESC_ADDR_TI_PORT0_PHYADDR);
    bsp_write_byte(pruIcssHandle, pmdio_params->addr1, ESC_ADDR_TI_PORT1_PHYADDR);
    bsp_write_byte(pruIcssHandle, (1 << PDI_ISR_EDIO_NUM),
                   ESC_ADDR_TI_PDI_ISR_PINSEL);

    bsp_write_dword(pruIcssHandle,
                    (pmdio_params->link0pol << pmdio_params->addr0) |
                    (pmdio_params->link1pol << pmdio_params->addr1),
                    ESC_ADDR_TI_PHY_LINK_POLARITY);

    g_bsp_params.ethphy_init(pruIcssHandle, pmdio_params->addr0, pmdio_params->addr1, pmdio_params->enhancedlink_enable);

    //Release PDI access over MDIO/MI interface
    bsp_write_byte(pruIcssHandle, (bsp_read_byte(pruIcssHandle,
                                   ESC_ADDR_MI_PDI_ACCESS) & ~1), ESC_ADDR_MI_PDI_ACCESS);
    return 0;
}


uint32_t bsp_pruss_mdio_phy_link_state(PRUICSS_Handle pruIcssHandle,
                                       uint8_t phyaddr)
{
    volatile uint32_t regval;
    regval = bsp_pruss_mdioreg_read(pruIcssHandle, CSL_ICSS_PR1_MDIO_V1P7_MDIO_LINK_REG);
#if TIESC_MDIO_RX_LINK_ENABLE
    regval ^= bsp_read_dword(pruIcssHandle, ESC_ADDR_TI_PHY_LINK_POLARITY);
#endif
    return (regval & (1 << phyaddr));
}

PRUICSS_IntcInitData pruss1_intc_initdata = PRU_ICSS1_INTC_INITDATA;

void bsp_params_init(bsp_params *init_params)
{
    init_params->pruicss_handle = NULL;
    init_params->interrupt_offset = 0;
    init_params->eeprom_read = NULL;
    init_params->eeprom_write = NULL;
    init_params->spinlock_base_address = 0;
    init_params->ethphy_init = NULL;
    init_params->enhancedlink_enable = TIESC_MDIO_RX_LINK_DISABLE;
    init_params->link0_polarity = TIESC_LINK_POL_ACTIVE_HIGH;
    init_params->link1_polarity = TIESC_LINK_POL_ACTIVE_HIGH;
    init_params->phy0_address = 0;
    init_params->phy1_address = 0;
    init_params->default_tiesc_eeprom = NULL;
    init_params->eeprom_pointer_for_stack = NULL;
    init_params->pdi_isr = NULL;
    init_params->sync0_isr = NULL;
    init_params->sync1_isr = NULL;
    init_params->phy_rx_err_reg = 0;
    init_params->pruicssClkFreq = TIESC_PRUICSS_CLOCK_FREQUENCY_200_MHZ;
}

int32_t bsp_init(bsp_params *init_params)
{
    int32_t                 status = SystemP_SUCCESS;
    PRUICSS_Handle          pruIcssHandle = NULL;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = NULL;

#ifndef USE_ECAT_TIMER
    TypesP_FreqHz  frg;
#endif

    t_mdio_params mdioParamsInit;

    if(init_params->pruicss_handle == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        g_bsp_params.pruicss_handle = init_params->pruicss_handle;
        pruIcssHandle = g_bsp_params.pruicss_handle;
    }

    if(status == SystemP_SUCCESS)
    {
        g_bsp_params.interrupt_offset = init_params->interrupt_offset;
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->eeprom_read == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.eeprom_read = init_params->eeprom_read;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->eeprom_write == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.eeprom_write = init_params->eeprom_write;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        g_bsp_params.spinlock_base_address = init_params->spinlock_base_address;
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->ethphy_init == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.ethphy_init = init_params->ethphy_init;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        if((init_params->enhancedlink_enable != TIESC_MDIO_RX_LINK_DISABLE) && (init_params->enhancedlink_enable != TIESC_MDIO_RX_LINK_ENABLE))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.enhancedlink_enable = init_params->enhancedlink_enable;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        if((init_params->link0_polarity != TIESC_LINK_POL_ACTIVE_LOW) && (init_params->link0_polarity != TIESC_LINK_POL_ACTIVE_HIGH))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.link0_polarity = init_params->link0_polarity;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        if((init_params->link1_polarity != TIESC_LINK_POL_ACTIVE_LOW) && (init_params->link1_polarity != TIESC_LINK_POL_ACTIVE_HIGH))
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.link1_polarity = init_params->link1_polarity;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        g_bsp_params.phy0_address = init_params->phy0_address;
        g_bsp_params.phy1_address = init_params->phy1_address;
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->default_tiesc_eeprom == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.default_tiesc_eeprom = init_params->default_tiesc_eeprom;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->eeprom_pointer_for_stack == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.eeprom_pointer_for_stack = init_params->eeprom_pointer_for_stack;
        }
    }

#ifndef ENABLE_PDI_TASK
    if(status == SystemP_SUCCESS)
    {
        if(init_params->pdi_isr == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.pdi_isr = init_params->pdi_isr;
        }
    }
#endif

#ifndef ENABLE_SYNC_TASK
    if(status == SystemP_SUCCESS)
    {
        if(init_params->sync0_isr == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.sync0_isr = init_params->sync0_isr;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->sync1_isr == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.sync1_isr = init_params->sync1_isr;
        }
    }
#endif

    if(status == SystemP_SUCCESS)
    {
        g_bsp_params.phy_rx_err_reg = init_params->phy_rx_err_reg;
    }

    if(status == SystemP_SUCCESS)
    {
        if(init_params->pruicssClkFreq > TIESC_PRUICSS_CLOCK_FREQUENCY_333_MHZ)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            g_bsp_params.pruicssClkFreq = init_params->pruicssClkFreq;
        }
    }

    if(status == SystemP_SUCCESS)
    {

        pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruIcssHandle->hwAttrs);

        // init timer data
        current_low = 0;
        pd_read_addr_err = pd_write_addr_err = 0;
        pdi_read_fail_cnt = pdi_write_fail_cnt = 0;
        prev_low = 0;
#ifndef USE_ECAT_TIMER
        OSAL_getCpuFreq(&frg);
        ecat_timer_inc_p_ms = (frg.lo / 1000);
#endif

        volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
                ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
                PRUICSS_DATARAM(0));
        volatile t_register_properties *pRegPerm = (volatile t_register_properties *)
                ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
                PRUICSS_DATARAM(1));

        bsp_hwspinlock_init();

        PRUICSS_intcInit(pruIcssHandle, &pruss1_intc_initdata);

        /* initialize ESC DPRAM pointer microcontroller specific to the beginning of the physical memory of the ESC,
        the macro MAKE_PTR_TO_ESC should be defined in tieschw.h */
        tiesc_memset((uint8_t *)sm_properties, 0, sizeof(sm_properties));

        PRUICSS_initMemory(pruIcssHandle, PRUICSS_SHARED_RAM);
        PRUICSS_initMemory(pruIcssHandle, PRUICSS_DATARAM(0));
        PRUICSS_initMemory(pruIcssHandle, PRUICSS_DATARAM(1));
        tiesc_memset((uint8_t *)&pRegPerm->reg_properties, 3,
            4 * 1024);  //Init PRU1 data ram
        ASSERT_DMB();

        bsp_pruss_cmd_intfc_write_word(0xFF, &pHost2PruIntfc->cmdlow);

        if(g_bsp_params.pruicssClkFreq == TIESC_PRUICSS_CLOCK_FREQUENCY_200_MHZ)
        {
            bsp_write_word(pruIcssHandle, TIESC_PORT0_TX_DELAY_200_MHZ_CLOCK,
                        ESC_ADDR_TI_PORT0_TX_START_DELAY);
            bsp_write_word(pruIcssHandle, TIESC_PORT0_TX_DELAY_200_MHZ_CLOCK,
                        ESC_ADDR_TI_PORT1_TX_START_DELAY);
        }
        else if(g_bsp_params.pruicssClkFreq == TIESC_PRUICSS_CLOCK_FREQUENCY_333_MHZ)
        {
            bsp_write_word(pruIcssHandle, TIESC_PORT0_TX_DELAY_333_MHZ_CLOCK,
                        ESC_ADDR_TI_PORT0_TX_START_DELAY);
            bsp_write_word(pruIcssHandle, TIESC_PORT0_TX_DELAY_333_MHZ_CLOCK,
                        ESC_ADDR_TI_PORT1_TX_START_DELAY);
        }


        mdioParamsInit.clkdiv = TIESC_MDIO_CLKDIV * 2;

        mdioParamsInit.addr0 = g_bsp_params.phy0_address;
        mdioParamsInit.addr1 = g_bsp_params.phy1_address;

        mdioParamsInit.enhancedlink_enable = g_bsp_params.enhancedlink_enable;

        if(TIESC_MDIO_RX_LINK_ENABLE == mdioParamsInit.enhancedlink_enable)
        {
            //Enhanced link detection enabled
            mdioParamsInit.link0pol = g_bsp_params.link0_polarity;
            mdioParamsInit.link1pol = g_bsp_params.link1_polarity;
        }
        else
        {
            //Enhanced link detection disabled
            mdioParamsInit.link0pol = TIESC_LINK_POL_ACTIVE_HIGH;
            mdioParamsInit.link1pol = TIESC_LINK_POL_ACTIVE_HIGH;
        }

        bsp_pruss_mdio_init(pruIcssHandle, &mdioParamsInit);

        bsp_write_word(pruIcssHandle, 0x00, ESC_ADDR_TI_EDMA_LATENCY_ENHANCEMENT);
        bsp_write_word(pruIcssHandle, g_bsp_params.phy_rx_err_reg, ESC_ADDR_TI_PHY_RX_ER_REG);
        bsp_write_word(pruIcssHandle, g_bsp_params.pruicssClkFreq, ESC_ADDR_TI_PRU_CLK_FREQUENCY);

        bsp_esc_reg_perm_init(pruIcssHandle);
        //Trigger PDI WD on  LATCH_IN or every command send to firmware
        bsp_set_pdi_wd_trigger_mode(pruIcssHandle, PDI_WD_TRIGGER_LATCH_IN);
        bsp_set_digio_sw_dataout_enable(pruIcssHandle);

        /* Use the TX interface clock for the TX_IPG counter, If this is not
         * enabled, 100Mbit/s MII is not supported when the PRU_ICSSG is
         * operating at frequencies < 250MHz */
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_IPG_WIRE_CLK_EN0, 0x1);
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_IPG_WIRE_CLK_EN1, 0x1);

        /* Note that it is important to update TX_IPG1 before TX_IPG0 as in
         * WIRE_CLK mode TX_IPG0 write is required to load the IPG value to HW */
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1_TX_IPG1, TIESC_TX_MIN_IPG);
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0_TX_IPG0, TIESC_TX_MIN_IPG);

        /* Set RX maximum frame size to 2000 bytes. Default is 1522 bytes. */
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0_RX_MAX_FRM0, TIESC_MAX_FRAME_SIZE);
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1_RX_MAX_FRM1, TIESC_MAX_FRAME_SIZE);

        PRUICSS_resetCore(pruIcssHandle, PRUICSS_PRU0);
        PRUICSS_resetCore(pruIcssHandle, PRUICSS_PRU1);

        ASSERT_DMB();

        PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU0);
        PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU1);

        /*TODO: Confirm the following changes*/
        /* PRU firmwares are loaded as header files in application  */
        PRUICSS_writeMemory(pruIcssHandle, PRUICSS_IRAM_PRU(0) , 0,
                            (uint32_t *) pru_frame_proc,
                            pru_frame_proc_len);

        PRUICSS_writeMemory(pruIcssHandle, PRUICSS_IRAM_PRU(1) , 0,
                            (uint32_t *) pru_host_proc,
                            pru_host_proc_len);

        PRUICSS_enableCore(pruIcssHandle, PRUICSS_PRU1);
        PRUICSS_enableCore(pruIcssHandle, PRUICSS_PRU0);

        bsp_eeprom_emulation_init();        //Load eeprom file to memory

        if(bsp_eeprom_load_esc_registers(pruIcssHandle, 0) == -1)
        {
            uint16_t EEPROMReg = 0;
            EEPROMReg = bsp_read_word(pruIcssHandle, ESC_ADDR_EEPROM_CTRL);
            EEPROMReg |= ESC_EEPROM_ERROR_CRC;
            bsp_write_word(pruIcssHandle, EEPROMReg, ESC_ADDR_EEPROM_CTRL);
            bsp_pdi_write_indication(pruIcssHandle, ESC_ADDR_EEPROM_CTRL, sizeof(uint16_t), EEPROMReg);
        }

        SemaphoreP_constructBinary(&semcmdlow_object, 1);
    }

    return status;
}

void bsp_exit(PRUICSS_Handle pruIcssHandle)
{
    SemaphoreP_destruct(&semcmdlow_object);

    bsp_eeprom_emulation_exit();        //Flush the EEPROM cache to file

    PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU0);
    PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU1);

    bsp_params_init(&g_bsp_params);
}


void bsp_start_esc_isr(PRUICSS_Handle pruIcssHandle)
{
    /* enable the ESC-interrupt microcontroller specific,
       the macro ENABLE_ESC_INT should be defined in ecat_def.h */
    // enable RTOS int
    uint32_t evtOutNum;
    uint32_t pruIsrNum = 0;
    uint8_t wait_enable = 0;

#ifdef ENABLE_PDI_TASK
    wait_enable = 1;
#else
    wait_enable = 0;
#endif

    pruIsrNum = HOST_AL_EVENT + g_bsp_params.interrupt_offset - 20;
    evtOutNum = HOST_AL_EVENT - 20;

    PRUICSS_registerIrqHandler(pruIcssHandle,
                               evtOutNum,
                               pruIsrNum,
                               1,
                               wait_enable,
                               &EcatIsr
                              );

#ifndef SUPPORT_CMDACK_POLL_MODE
    pruIsrNum = HOST_CMD_LOW_ACK_EVENT + g_bsp_params.interrupt_offset - 20;
    evtOutNum = HOST_CMD_LOW_ACK_EVENT - 20;

    PRUICSS_registerIrqHandler(pruIcssHandle,
                               evtOutNum,
                               pruIsrNum,
                               1,
                               1,
                               &EscCmdLowAckIsr);
#endif

#ifdef ENABLE_SYNC_TASK
    wait_enable = 1;
#else
    wait_enable = 0;
#endif

    pruIsrNum = HOST_SYNC0_EVENT + g_bsp_params.interrupt_offset - 20;
    evtOutNum = HOST_SYNC0_EVENT - 20;

    PRUICSS_registerIrqHandler(pruIcssHandle,
                               evtOutNum,
                               pruIsrNum,
                               1,
                               wait_enable,
                               &Sync0Isr);

    pruIsrNum = HOST_SYNC1_EVENT + g_bsp_params.interrupt_offset - 20;
    evtOutNum = HOST_SYNC1_EVENT - 20;

    PRUICSS_registerIrqHandler(pruIcssHandle,
                               evtOutNum,
                               pruIsrNum,
                               1,
                               wait_enable,
                               &Sync1Isr);
}

void bsp_send_command_to_firmware(PRUICSS_Handle pruIcssHandle,
                                  uint32_t command,
                                  uint16_t param1, uint16_t param2)
{
    volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
            ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(0));
    {
#ifdef ENABLE_PDI_TASK
        SemaphoreP_pend(&semcmdlow_object, SystemP_WAIT_FOREVER);
#else
        uintptr_t key1 = HwiP_disable();
#endif
        bsp_pruss_cmd_intfc_write_word(command, &pHost2PruIntfc->cmdlow);
        bsp_pruss_cmd_intfc_write_word(param1, &pHost2PruIntfc->param1low);
        bsp_pruss_cmd_intfc_write_word(param2, &pHost2PruIntfc->param2low);
#ifdef SUPPORT_CMDACK_POLL_MODE
        bsp_pruss_cmd_intfc_write_word(1, &pHost2PruIntfc->cmdlow_ack);
#endif
        PRUICSS_sendEvent((g_bsp_params.pruicss_handle), ARM_PRU_EVENT1);
        ASSERT_DMB();
        ASSERT_DSB();
        {
#ifdef SUPPORT_CMDACK_POLL_MODE
            volatile uint16_t ack;

            do
            {
                ack = bsp_pruss_cmd_intfc_read_word(&pHost2PruIntfc->cmdlow_ack);
            }
            while(ack);
#ifndef ENABLE_PDI_TASK
            HwiP_restore(key1);
#endif
#else
            uint32_t evtoutNum = HOST_CMD_LOW_ACK_EVENT - 20;
#ifndef ENABLE_PDI_TASK
            HwiP_restore(key1);
#endif
            PRUICSS_waitEvent((PRUICSS_Handle)(g_bsp_params.pruicss_handle), evtoutNum);
#endif
        }
        bsp_pruss_cmd_intfc_write_word(0xFF,    &pHost2PruIntfc->cmdlow);
#ifdef ENABLE_PDI_TASK
        SemaphoreP_post(&semcmdlow_object);
#endif
    }
}

void bsp_pdi_post_read_indication(PRUICSS_Handle pruIcssHandle,
                                  uint16_t address, uint16_t length)
{

    if((address <= ESC_ADDR_SM_WD_STATUS)
            && ((address + length) > ESC_ADDR_SM_WD_STATUS))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_PD_WD_STATUS, 0,
                                     0);
    }

    else if((address <= ESC_ADDR_SYNC_STATUS)
            && ((address + length) > ESC_ADDR_SYNC_STATUS))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_SYNC_STATUS, SYNC0,
                                     0);
    }

    else if((address <= (ESC_ADDR_SYNC_STATUS + 1))
            && ((address + length) > ESC_ADDR_SYNC_STATUS + 1))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_SYNC_STATUS, SYNC1,
                                     0);
    }

    else if((address <= ESC_ADDR_ALCONTROL)
            && ((address + length) > ESC_ADDR_ALCONTROL))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_AL_CONTROL, 0, 0);
    }

    else if(((address <= ESC_ADDR_SM0_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM0_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM1_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM1_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM2_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM2_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM3_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM3_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM4_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM4_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM5_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM5_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM6_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM6_ACTIVATE)) ||
            ((address <= ESC_ADDR_SM7_ACTIVATE)
             && ((address + length) > ESC_ADDR_SM7_ACTIVATE)))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_SM_ACTIVATE,
                                     (address - ESC_ADDR_SYNCMAN) >> 3, 0);
    }

    else if((address <= ESC_ADDR_LATCH0_POS_EDGE)
            && ((address + length) > ESC_ADDR_LATCH0_POS_EDGE))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                     LATCH0_POS_EDGE, 0);
    }

    else if((address <= ESC_ADDR_LATCH0_NEG_EDGE)
            && ((address + length) > ESC_ADDR_LATCH0_NEG_EDGE))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                     LATCH0_NEG_EDGE, 0);
    }

    else if((address <= ESC_ADDR_LATCH1_POS_EDGE)
            && ((address + length) > ESC_ADDR_LATCH1_POS_EDGE))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                     LATCH1_POS_EDGE, 0);
    }

    else if((address <= ESC_ADDR_LATCH1_NEG_EDGE)
            && ((address + length) > ESC_ADDR_LATCH1_NEG_EDGE))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_READ_LATCH_TIME,
                                     LATCH1_NEG_EDGE, 0);
    }

    if(address == sm_properties[MAILBOX_WRITE].physical_start_addr)
    {
        bsp_pdi_mbx_read_start(pruIcssHandle);
    }

}


void bsp_pdi_write_indication(PRUICSS_Handle pruIcssHandle, uint16_t address,
                              uint16_t length, uint16_t value)
{
    uint16_t regval = 0;

    if((address <= ESC_ADDR_ALSTATUS) && ((address + length) > ESC_ADDR_ALSTATUS))
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_WRITE_AL_STATUS, 0, 0);
    }

    else if(((address <= ESC_ADDR_SM0_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM0_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM1_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM1_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM2_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM2_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM3_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM3_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM4_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM4_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM5_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM5_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM6_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM6_PDI_CONTROL)) ||
            ((address <= ESC_ADDR_SM7_PDI_CONTROL)
             && ((address + length) > ESC_ADDR_SM7_PDI_CONTROL)))
    {
        uint8_t channel = (address - ESC_ADDR_SYNCMAN) >> 3;
        uint16_t sm_address = ESC_ADDR_SYNCMAN + (channel * SIZEOF_SM_REGISTER);
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_WRITE_SM_PDI_CTRL,
                                     (channel << 8) | value, 0);

        while(bsp_pdi_sm_config_ongoing(pruIcssHandle));

        bsp_set_sm_properties(pruIcssHandle, channel, bsp_read_word(pruIcssHandle,
                              sm_address),
                              bsp_read_word(pruIcssHandle, (sm_address + 2)));
    }

    else if((address <= ESC_ADDR_EEPROM_CTRL)
            && ((address + length) > ESC_ADDR_EEPROM_CTRL))
    {
        /* Note - Here we are assuming that, Command is always two byte  */
        regval = SWAPWORD(value);

        if((regval & ESC_EEPROM_CMD_WRITE_MASK) &&
                !(regval & ESC_EEPROM_ERROR_CMD_ACK))
        {
            bsp_set_eeprom_update_status(1);
            bsp_set_eeprom_updated_time();
        }

        bsp_eeprom_emulation_command_ack(pruIcssHandle);
    }

#ifdef SYSTEM_TIME_PDI_CONTROLLED

    else if((address <= ESC_ADDR_LATCH0_CONTROL)
            && ((address + length) > ESC_ADDR_LATCH0_CONTROL))
    {
        bsp_pdi_latch0_control(pruIcssHandle, (value & 0xFF));
    }

    else if((address <= ESC_ADDR_LATCH1_CONTROL)
            && ((address + length) > ESC_ADDR_LATCH1_CONTROL))
    {
        bsp_pdi_latch1_control(pruIcssHandle, (value & 0xFF));
    }

#endif

    if(address == sm_properties[MAILBOX_READ].physical_start_addr)
    {
        bsp_pdi_mbx_write_start(pruIcssHandle);
    }

}

void bsp_pdi_mbx_read_start(PRUICSS_Handle pruIcssHandle)
{
    /*TODO: Review this line*/
    uint16_t  ALEvent = bsp_read_word_isr((g_bsp_params.pruicss_handle), ESC_ADDR_AL_EVENT_REQ);
//    uint16_t  ALEvent = HW_GetALEventRegister_Isr();

    if(ALEvent & MBX_WRITE_EVENT)
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_CLEAR_AL_EVENT_LOW,
                                     ~MBX_WRITE_EVENT, 0);
    }
}

void bsp_pdi_mbx_read_complete(PRUICSS_Handle pruIcssHandle)
{
    /* get address of the receive mailbox sync manager */
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_ACK_MBX_READ,
                                 sm_properties[MAILBOX_WRITE].physical_start_addr,
                                 sm_properties[MAILBOX_WRITE].length);
}
void bsp_pdi_mbx_write_start(PRUICSS_Handle pruIcssHandle)
{
    /*TODO: Review this line*/
    uint16_t  ALEvent = bsp_read_word_isr((g_bsp_params.pruicss_handle), ESC_ADDR_AL_EVENT_REQ);
//    uint16_t  ALEvent = HW_GetALEventRegister_Isr();

    if(ALEvent & MBX_READ_EVENT)
    {
        bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_CLEAR_AL_EVENT_LOW,
                                     ~MBX_READ_EVENT, 0);
    }
}
void bsp_pdi_mbx_write_complete(PRUICSS_Handle pruIcssHandle)
{
    /* get address of the send mailbox sync manager */
    bsp_send_command_to_firmware(pruIcssHandle, CMD_DL_USER_ACK_MBX_WRITE,
                                 sm_properties[MAILBOX_READ].physical_start_addr,
                                 sm_properties[MAILBOX_READ].length);
}

__inline int16_t bsp_get_sm_index(uint16_t address, uint16_t len)
{
    int16_t sm_index = -1, i;

    for(i = 2 ; i < MAX_SYNC_MAN; i++)
    {
        if((address >= sm_properties[i].physical_start_addr)
                && (address + len <= sm_properties[i].physical_start_addr +
                    sm_properties[i].length))
        {
            sm_index = i;
            break;
        }
    }

    return sm_index;
}

__inline uint16_t bsp_get_process_data_address(PRUICSS_Handle pruIcssHandle,
        uint16_t address, uint16_t len, int16_t *p_sm_index)
{
    uint16_t addr = 0;
    uint16_t count;
    int16_t sm_index = bsp_get_sm_index(address, len);
    volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
            ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(0));

    //Find corresponding SYNC manager and store index in sm_index
    if(sm_index == -1)
    {
        return 0;
    }

    if(p_sm_index)
    {
        *p_sm_index = sm_index;
    }

    /* called while SM is disabled ?! should never happen; just in case... */
    if(sm_properties[sm_index].physical_start_addr == 0)
    {
        return 0;
    }

    /* are we already accessing this sm ?! in this case "lock_state" will be
     * "LOCK_PD_BUF_HOST_ACCESS_START" and we do not need the loop... */
    if(pHost2PruIntfc->sm_processdata[sm_index - 2].lock_state !=
            LOCK_PD_BUF_HOST_ACCESS_START)
    {
        for(count = 0; count < LOCK_PD_BUF_CHECK_AVAILABILITY_RETRY_COUNT; count++)
        {
            /* May we access the buffer (LOCK_PD_BUF_AVAILABLE_FOR_HOST)? */
            if(pHost2PruIntfc->sm_processdata[sm_index - 2].lock_state ==
                    LOCK_PD_BUF_AVAILABLE_FOR_HOST)
            {
                pHost2PruIntfc->sm_processdata[sm_index - 2].lock_state =
                    LOCK_PD_BUF_HOST_ACCESS_START;
                ASSERT_DMB();
                break;
            }
        }
    }

    if(pHost2PruIntfc->sm_processdata[sm_index - 2].lock_state ==
            LOCK_PD_BUF_HOST_ACCESS_START)
    {
        addr = pHost2PruIntfc->sm_processdata[sm_index - 2].addr;
        addr = addr + (address - sm_properties[sm_index].physical_start_addr);
    }

    return addr;
}

__inline void bsp_process_data_access_complete(PRUICSS_Handle pruIcssHandle,
        uint16_t address, uint16_t len, int16_t sm_index)
{
    volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
            ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(0));

    //Find corresponding SYNC manager and store index in sm_index
    /* Assumption : sm_index > 1 as first two SMs are Mailbox SMs */
    if(((sm_index < 2) || (sm_index >= MAX_SYNC_MAN)))
    {
        return;
    }

    if((address >= sm_properties[sm_index].physical_start_addr) &&
            (address + len >= sm_properties[sm_index].  physical_start_addr +
             sm_properties[sm_index].length))
    {
        if(pHost2PruIntfc->sm_processdata[sm_index - 2].lock_state ==
                LOCK_PD_BUF_HOST_ACCESS_START)
        {
            pHost2PruIntfc->sm_processdata[sm_index - 2].lock_state =
                LOCK_PD_BUF_HOST_ACCESS_FINISH;
            ASSERT_DMB();
        }
    }
}

#ifdef __TI_ARM__
void bsp_set_sm_properties(PRUICSS_Handle pruIcssHandle, uint8_t sm,
                                  uint16_t address, uint16_t length)

#else
inline void bsp_set_sm_properties(PRUICSS_Handle pruIcssHandle, uint8_t sm,
                                  uint16_t address, uint16_t length)
#endif
{
    sm_properties[sm].physical_start_addr = address;
    sm_properties[sm].length = length;
}

inline t_sm_properties *bsp_get_sm_properties(uint8_t sm)
{
    return &sm_properties[sm];
}

/** @addtogroup ECAT_ESC_REG_ACCESS
 @{ */

inline uint32_t bsp_get_pdi_read_access_fail_cnt()
{
    return pdi_read_fail_cnt;
}

inline uint32_t bsp_get_pdi_write_access_fail_cnt()
{
    return pdi_write_fail_cnt;
}
/**
@}
*/

inline void bsp_set_pdi_perm_read_only(uint16_t *perm_array, uint16_t address)
{
    *(perm_array + (address >> 4)) |= (1 << (address & 0x0F));
}

inline void bsp_set_pdi_perm_read_write(uint16_t *perm_array, uint16_t address)
{
    *(perm_array + (address >> 4)) &= ~(1 << (address & 0x0F));
}

inline uint8_t bsp_is_pdi_perm_read_only(uint16_t *perm_array, uint16_t address)
{
    uint16_t i, pos;

    i = address >> 4;
    pos = 1 << (address & 0x0F);

    if(*(perm_array + i) & pos)
    {
        return 1;
    }

    else
    {
        return 0;
    }
}

inline uint8_t  bsp_get_pdi_access_perm(uint16_t address, uint8_t access)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    uint8_t retval = 1;

    if(address < 4096)
    {
        retval = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, address);
        retval = !(retval & !access);
    }

    return retval;
#else
    return 1;
#endif
}

inline uint8_t  bsp_pdi_access_perm_word(uint16_t address, uint8_t access)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    uint8_t retval = 1;

    if((address < 4095) && (address + 1 < 4096))
    {
        uint8_t t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, address);
        retval = !(t_perm & !access);

        t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, (address + 1));
        retval &= !(t_perm & !access);
    }

    return retval;
#else
    return 1;
#endif

}

inline uint8_t  bsp_pdi_access_perm_dword(uint16_t address, uint8_t access)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    uint8_t retval = 1;

    if((address < 4093) && (address + 3 < 4096))
    {
        uint8_t t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, address);
        retval = !(t_perm & !access);

        t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, (address + 1));
        retval &= !(t_perm & !access);

        t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, (address + 2));
        retval &= !(t_perm & !access);

        t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, (address + 3));
        retval &= !(t_perm & !access);
    }

    return retval;
#else
    return 1;
#endif
}

inline uint8_t  bsp_pdi_access_perm_array(uint16_t address, uint8_t access,
        uint16_t size)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    uint8_t retval = 1;

    if((address < 4096 - size) && ((address + size - 1) < 4096))
    {
        uint16_t itr = 1;
        uint8_t t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, address);

        retval = !(t_perm & !access);

        for(; itr < size ; itr++)
        {
            t_perm = bsp_is_pdi_perm_read_only(pdi_reg_perm_array, (address + itr));
            retval &= !(t_perm & !access);
        }
    }

    return retval;
#else
    return 1;
#endif

}

uint32_t bsp_read_dword(PRUICSS_Handle pruIcssHandle, uint16_t address)
{
    uint32_t DWordValue;
    uint32_t end_addr = sm_properties[MAILBOX_WRITE].physical_start_addr +
                        sm_properties[MAILBOX_WRITE].length;

#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_dword(address, PDI_PERM_READ))
    {
        pdi_read_fail_cnt++;
        return 0;
    }
#endif
    DWordValue = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                             address) >> 2)]);

    if((address >= (end_addr - 4)) &&
            (address < end_addr))
    {
        bsp_pdi_mbx_read_complete(pruIcssHandle);
    }

    else
    {
        bsp_pdi_post_read_indication(pruIcssHandle, address, sizeof(uint32_t));
    }
    return DWordValue;
}

uint32_t bsp_read_dword_isr(PRUICSS_Handle pruIcssHandle, uint16_t address)
{
    uint32_t DWordValue;
    DWordValue = (((uint32_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                             address) >> 2)]);
    return DWordValue;
}

uint16_t bsp_read_word(PRUICSS_Handle pruIcssHandle, uint16_t address)
{
    uint16_t WordValue;
    uint16_t end_addr = sm_properties[MAILBOX_WRITE].physical_start_addr +
                        sm_properties[MAILBOX_WRITE].length;

#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_word(address, PDI_PERM_READ))
    {
        pdi_read_fail_cnt++;
        return 0;
    }
#endif
    WordValue = (((uint16_t *)(((PRUICSS_HwAttrs *)(
                                    pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                            address) >> 1)]);

    if((address >= (end_addr - 2)) &&
            (address < end_addr))
    {
        bsp_pdi_mbx_read_complete(pruIcssHandle);
    }

    else
    {
        bsp_pdi_post_read_indication(pruIcssHandle, address, sizeof(uint16_t));
    }
    return WordValue;
}

uint16_t bsp_read_word_isr(PRUICSS_Handle pruIcssHandle, uint16_t address)
{
    uint16_t WordValue;
    WordValue = (((uint16_t *)(((PRUICSS_HwAttrs *)(
                                    pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM))[((
                                            address) >> 1)]);
    return WordValue;
}

uint8_t bsp_read_byte(PRUICSS_Handle pruIcssHandle, uint16_t address)
{
    uint8_t ByteValue;

#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_get_pdi_access_perm(address, PDI_PERM_READ))
    {
        pdi_read_fail_cnt++;
        return 0;
    }
#endif
    uint8_t *pEsc = (uint8_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM);
    ByteValue = pEsc[address];

    if(address == (sm_properties[MAILBOX_WRITE].physical_start_addr +
                   sm_properties[MAILBOX_WRITE].length - 1))
    {
        bsp_pdi_mbx_read_complete(pruIcssHandle);
    }

    else
    {
        bsp_pdi_post_read_indication(pruIcssHandle, address, sizeof(uint8_t));
    }
    return ByteValue;
}

inline uint8_t bsp_read_byte_isr(PRUICSS_Handle pruIcssHandle, uint16_t address)
{
    uint8_t ByteValue;
    uint8_t *pEsc = (uint8_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM);
    ByteValue = pEsc[address];
    return ByteValue;
}

void bsp_read(PRUICSS_Handle pruIcssHandle, uint8_t *pdata, uint16_t address,
              uint16_t len)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_array(address, PDI_PERM_READ, len))
    {
        pdi_read_fail_cnt++;
        return;
    }
#endif
    uint8_t *pEsc = (uint8_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM);
    tiesc_memcpy(pdata, &pEsc[address], len);
}

void bsp_write_dword(PRUICSS_Handle pruIcssHandle, uint32_t val,
                     uint16_t address)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_dword(address, PDI_PERM_WRITE))
    {
        pdi_write_fail_cnt++;
        return;
    }
#endif
    (((uint32_t *)(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr
                   + PRUICSS_SHARED_RAM))[((address) >> 2)]) = val;
    ASSERT_DMB();
}

void bsp_write_word(PRUICSS_Handle pruIcssHandle, uint16_t val,
                    uint16_t address)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_word(address, PDI_PERM_WRITE))
    {
        pdi_write_fail_cnt++;
        return;
    }
#endif
    (((uint16_t *)(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr
                   + PRUICSS_SHARED_RAM))[((address) >> 1)]) = val;
    ASSERT_DMB();
}

void bsp_write_byte(PRUICSS_Handle pruIcssHandle, uint8_t val, uint16_t address)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_get_pdi_access_perm(address, PDI_PERM_WRITE))
    {
        pdi_write_fail_cnt++;
        return;
    }
#endif
    uint8_t *pEsc = (uint8_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM);
    pEsc[address] = val;
    ASSERT_DMB();
}

void bsp_write(PRUICSS_Handle pruIcssHandle, uint8_t *pdata, uint16_t address,
               uint16_t len)
{
#ifdef ENABLE_PDI_REG_PERMISSIONS
    if(0 == bsp_pdi_access_perm_array(address, PDI_PERM_WRITE, len))
    {
        pdi_write_fail_cnt++;
        return;
    }
#endif
    uint8_t *pEsc = (uint8_t *)(((PRUICSS_HwAttrs *)(
                                     pruIcssHandle->hwAttrs))->baseAddr + PRUICSS_SHARED_RAM);
    tiesc_memcpy(&pEsc[address], pdata, len);
    ASSERT_DMB();
}

inline uint32_t bsp_pruss_mdioreg_read(PRUICSS_Handle pruIcssHandle,
                                       uint32_t regoffset)
{
    uint32_t regval;
    regval = HW_RD_REG32((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->miiMdioRegBase) + regoffset);
    return regval;
}

inline void bsp_pruss_mdioreg_write(PRUICSS_Handle pruIcssHandle, uint32_t val,
                                    uint32_t regoffset)
{
    HW_WR_REG32(((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->miiMdioRegBase) + regoffset), val);
    ASSERT_DMB();
}

uint32_t bsp_pruss_iepreg_read(PRUICSS_Handle pruIcssHandle, uint32_t regoffset)
{
    uint32_t regval;
    regval = HW_RD_REG32((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase) + regoffset);

    return regval;
}

inline void bsp_pruss_iepreg_write(PRUICSS_Handle pruIcssHandle, uint32_t val,
                                   uint32_t regoffset)
{
    HW_WR_REG32(((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase) + regoffset), val);
    ASSERT_DMB();
}

inline uint16_t bsp_pruss_cmd_intfc_read_word(volatile uint16_t *ptr)
{
    uint16_t val;
    val = *ptr;
    return val;
}

inline void bsp_pruss_cmd_intfc_write_word(uint16_t val, volatile uint16_t *ptr)
{
    *ptr = val;
    ASSERT_DMB();
}

inline uint8_t bsp_pdi_sm_config_ongoing(PRUICSS_Handle pruIcssHandle)
{
    volatile t_host_interface *pHost2PruIntfc = (volatile t_host_interface *)
            ((((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->baseAddr) +
             PRUICSS_DATARAM(0));
    return pHost2PruIntfc->sm_config_ongoing;
}

inline void bsp_hwspinlock_init(void)
{
    uint32_t regval;
    uintptr_t icssgBaseAddr = (((PRUICSS_HwAttrs *)((g_bsp_params.pruicss_handle)->hwAttrs))->baseAddr);

    /* Setting up RAT config to map SOC Spinlock to C23 constant of PRUICSS */
    /* Mapping 0xC0000000 (C23 constant of PRUICSS) to 0x30E00800 (SPINLOCK) with size 2^11 = 0x800 */
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x24), (0xC0000000u)); //rat0 base0
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x28), (g_bsp_params.spinlock_base_address)); //rat0 trans_low0
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x2C), (0x00000000u)); //rat0 trans_high0
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x20), (1u << 31) | (15u)); //rat0 ctrl0

    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x24), (0xC0000000u)); //rat0 base0
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x28), (g_bsp_params.spinlock_base_address)); //rat0 trans_low0
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x2C), (0x00000000u)); //rat0 trans_high0
    HW_WR_REG32((icssgBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x20), (1u << 31) | (15u)); //rat0 ctrl0

    for(regval = 0; regval < 8; regval++)
    {
        bsp_hwspinlock_unlock(regval);
    }
}

inline uint32_t bsp_hwspinlock_lock(int num)
{
    /*TODO: Test this function*/
    if(Spinlock_lock(g_bsp_params.spinlock_base_address, num) == SPINLOCK_LOCK_STATUS_FREE)
    {
        return 0;
    }

    else
    {
        return 1;
    }
}

inline void bsp_hwspinlock_unlock(int num)
{
    /*TODO: Test this function*/
    Spinlock_unlock(g_bsp_params.spinlock_base_address, num);
}

void bsp_global_mutex_lock(void)
{
    //escGlobalIArg = GateAll_enter (escGlobalGateHandle);
    //Disable PDI and SYNC0 ISR at ARM INTC (rather than global IRQ disable)
    HwiP_disableInt(HOST_AL_EVENT + g_bsp_params.interrupt_offset);
    HwiP_disableInt(HOST_SYNC0_EVENT + g_bsp_params.interrupt_offset);
    HwiP_disableInt(HOST_SYNC1_EVENT + g_bsp_params.interrupt_offset);
}

void bsp_global_mutex_unlock(void)
{
    //GateAll_leave (escGlobalGateHandle, escGlobalIArg);
    //Enable back PDI and SYNC0 ISR at ARM INTC
    HwiP_enableInt(HOST_AL_EVENT + g_bsp_params.interrupt_offset);
    HwiP_enableInt(HOST_SYNC0_EVENT + g_bsp_params.interrupt_offset);
    HwiP_enableInt(HOST_SYNC1_EVENT + g_bsp_params.interrupt_offset);
}

void bsp_set_eeprom_update_status(uint8_t status)
{
    eeprom_updated = status;
}

inline uint8_t bsp_get_eeprom_update_status(void)
{
    return eeprom_updated;
}

inline uint8_t *bsp_get_eeprom_cache_base(void)
{
    return eeprom_cache;
}

inline void bsp_set_eeprom_updated_time()
{
#ifdef USE_ECAT_TIMER
    bsp_get_local_sys_time(&eeprom_updated_time, NULL);
#else
    eeprom_updated_time = Timestamp_get32();
#endif
}

inline uint32_t bsp_get_eeprom_updated_time()
{
    return eeprom_updated_time;
}

inline void bsp_set_pru_firmware(uint32_t *frameProc, uint32_t frameProcLen,
                                 uint32_t *hostProc, uint32_t hostProcLen)
{
    pru_frame_proc = frameProc;
    pru_host_proc = hostProc;
    pru_frame_proc_len = frameProcLen;
    pru_host_proc_len = hostProcLen;
}
