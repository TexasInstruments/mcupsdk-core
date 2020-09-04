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

#ifndef HDSL_DRV_H_
#define HDSL_DRV_H_

/**
 *  \defgroup HDSL_API_MODULE APIs for HDSL Encoder
 *  \ingroup MOTOR_CONTROL_API
 *
 * Here is the list of APIs used for EnDAT encoder communication protocol
 *
 *  @{
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "drivers/pruicss/g_v0/pruicss.h"
#include <stdlib.h>

#include <stdbool.h>

#include <kernel/dpl/DebugP.h>

#include <drivers/pruicss.h>
#include <drivers/hw_include/cslr_soc.h>
#include "hdsl_interface.h"
#include <drivers/hw_include/hw_types.h>


#define MAX_WAIT 20000
#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))
#define HWREGB(x)                                                              \
        (*((volatile uint8_t *)(x)))
#define HWREGH(x)                                                              \
        (*((volatile uint16_t *)(x)))
/*TSR configuration:*/

/*inEvent value:*/
/* ICSSG_0_EDC1_SYNC0 ICSSG0 IEP1 sync event 0 Pulse */
#define SYNCEVENT_INTRTR_IN_27 27

/*outEvent values:*/
/*SYNC0_OUT Pin Selectable timesync event 24 Edge (4+(24*4)) */
#define SYNCEVT_RTR_SYNC28_EVT 0x64
/* SYNC1_OUT Pin Selectable timesync event 25 Edge (4+(25*4)) */
#define SYNCEVT_RTR_SYNC29_EVT 0x68
/* SYNC2_OUT Pin Selectable timesync event 26 Edge (4+(26*4)) */
#define SYNCEVT_RTR_SYNC30_EVT 0x6C
/* SYNC3_OUT Pin Selectable timesync event 27 Edge (4+(27*4)) */
#define SYNCEVT_RTR_SYNC31_EVT 0x70
/* ICSSG0_PR1_EDC1_LATCH0_IN PRU_ICSSG0 (4+(10*4)) */
#define SYNCEVT_RTR_SYNC10_EVT 0x2C


enum {
    MENU_SAFE_POSITION,
    MENU_QUALITY_MONITORING,
    MENU_EVENTS,
    MENU_SUMMARY,
    MENU_ACC_ERR_CNT,
    MENU_RSSI,
    MENU_PC_SHORT_MSG_WRITE,
    MENU_PC_SHORT_MSG_READ,
    MENU_PC_LONG_MSG_WRITE,
    MENU_HDSL_REG_INTO_DDR,
    MENU_HDSL_REG_INTO_DDR_GPIO,
    MENU_DIRECT_READ_RID0_LENGTH8,
    MENU_DIRECT_READ_RID81_LENGTH8,
    MENU_DIRECT_READ_RID81_LENGTH2,
    MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0,
    MENU_INDIRECT_WRITE_RID0_LENGTH8,
    MENU_DIRECT_READ_RID0_LENGTH8_OFFSET6,
    MENU_LIMIT,
    MENU_INVALID,
};

/**
 *  \brief      Initialize IEP and Use OCP as IEP CLK src
 *
 *  \param[in]  gPruIcss0Handle
 *  \param[in]  gPru_cfg
 *  \param[in]  gPru_dramx
 *
 */

void HDSL_iep_init(PRUICSS_Handle gPruIcss0Handle, void *gPru_cfg,void *gPru_dramx);


/**
 *  \brief      Enable IEP            <br>
 *              *Enable SYNC0 and program pulse width   <br>
 *              Enable cyclic mod   <br>
 *              Program CMP1     <br>
 *              TSR configuration <br>
 *
 *  \param[in]  ES
 *  \param[in]  period
 *  \retval     1 for successful enable sync signal
 *
 */
int HDSL_enable_sync_signal(uint8_t ES,uint32_t period);

/**
 *  \brief      Calculate fast position,safe position1,safe position2
 *
 *  \param[in]  position_id
 *  \retval     position value in integer for successful position return, -1 for error in position return
 *
 */
uint64_t HDSL_get_pos(int position_id);


/**
 *  \brief   Taking quality monitoring value
 *
 *  \param[in]  None
 *  \retval    8 bit integer QM value
 */
uint8_t HDSL_get_qm();

/**
*  \brief       taking values of High bytes event(EVENT_H),Low bytes event(EVENT_L)
 *
 *  \param[in]  None

 *  \retval       16 bit integer concatenated values of both EVENT_H,EVENT_L
 *
 */
uint16_t HDSL_get_events();

/**
*  \brief   Getting Summarized slave status
 *
 *  \param[in] None

 *  \retval     8 bit integer value of summarized status
 *
 */
uint8_t HDSL_get_sum();

/**
 *  \brief  Acceleration error counter
 *
 *  \param[in]  None
 *
 *  \retval     8 bit integer value of acceleration error counter
 *
 */
uint8_t HDSL_get_acc_err_cnt();
/**
*  \brief  Read RSSI value
 *
 *  \param[in]  None
 *
 *  \retval     8 bit RSSI integer value
 *
 */
uint8_t HDSL_get_rssi();
/**
 *  \brief  Write Response of Short message parameters channel Read for safe1 channel(S_PC_DATA) with gPc_data and Short message control value(SLAVE_REG_CTRL) in hdsl interface
 *
 *  \param[in]  gPc_addr
 *  \param[in]  gPc_data
 *
 *  \retval    1 for successfully write of pc short messege in hdsl interface
 *
 */
int HDSL_write_pc_short_msg(uint32_t gPc_addr,uint32_t gPc_data);

/**
 *  \brief  Read Response of Short message parameters channel Read for safe1 channel(S_PC_DATA) and write Short message control value(SLAVE_REG_CTRL) with gPc_addr in hdsl interface
 *
 *  \param[in]  gPc_addr

 *  \retval  S_PC_DATA from hdsl interface
 *
 */
uint32_t HDSL_read_pc_short_msg(uint32_t gPc_addr);

/**
 *  \brief  Write PC_AAD_L ,PC_ADD_H ,PC_OFF_L,PC_OFF_H and PC_CTRL values in hdsl interface
 *
 *  \param[in]  gPc_addrh
 *  \param[in]  gPc_addrl
 *  \param[in]  gPc_offh
 *  \param[in]  gPc_offl
 *
 *
 */
void HDSL_set_pc_addr(uint8_t gPc_addrh,uint8_t gPc_addrl,uint8_t gPc_offh,uint8_t gPc_offl);


/**
 *  \brief  To set the direction read/write for long message communication
 *
 *  \param[in]  value
 *
 */
void HDSL_set_pc_ctrl(uint8_t value);

/**
 *  \brief  Write Parameters channel buffer for different bytes(bytes 0-7)
 *
 *  \param[in]  gPc_buf0
 *  \param[in]  gPc_buf1
 *  \param[in]  gPc_buf2
 *  \param[in]  gPc_buf3
 *  \param[in]  gPc_buf4
 *  \param[in]  gPc_buf5
 *  \param[in]  gPc_buf6
 *  \param[in]  gPc_buf7
 *
 */
void HDSL_write_pc_buffer(uint8_t gPc_buf0,uint8_t gPc_buf1,uint8_t gPc_buf2,uint8_t gPc_buf3,uint8_t gPc_buf4,uint8_t gPc_buf5,uint8_t gPc_buf6,uint8_t gPc_buf7);

/**
 *  \brief  read Parameters channel buffer for different bytes(bytes 0-7)
 *
 *  \param[in]  buff_off
 *  \retval 8 bit integer value of PC_BUFFER from hdsl interface
 *
 */
uint8_t HDSL_read_pc_buffer(uint8_t buff_off);

/**
 *  \brief  read Synchronization control value
 *
 *  \retval 8 bit integer value of SYNC_CTRL from hdsl interface
 *
 */

uint8_t HDSL_get_sync_ctrl();

/**
 *  \brief  write Synchronization control value
 *
 *  \param[in]  val
 *
 */

void HDSL_set_sync_ctrl(uint8_t val );

  /**
 *  \brief  read Quality monitoring value
 *
 *  \param[in]  None
 *  \retval 8 bit integer value of MASTER_QM from hdsl interface
 *
 */

uint8_t HDSL_get_master_qm();

/**
 *  \brief  read Cable bit sampling time control
 *
 *  \param[in]  None
 *  \retval 8 bit integer value of EDGES from hdsl interface
 *
 */
uint8_t HDSL_get_edges();
/**
 *  \brief  read Run time delay of system cable and signal strength
 *
 *  \param[in]  buff
 *  \retval 8 bit integer value of DELAY from hdsl interface
 *
 */
uint8_t HDSL_get_delay();
/**
 *  \brief  Read encoder id bytes(byte no. 0-2)
 *
 *  \param[in]  byte
 *  \retval  8 bit encoder bytes data from hdsl interface
 *
 */
uint8_t HDSL_get_enc_id(int byte);

void HDSL_generate_memory_image(void);

void* HDSL_get_src_loc();

uint32_t HDSL_get_length();



#ifdef __cplusplus
}
#endif

/** @} */
#endif
