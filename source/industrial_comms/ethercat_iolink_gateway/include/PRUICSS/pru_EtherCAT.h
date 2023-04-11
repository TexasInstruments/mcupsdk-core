/*!
* \file pru_EtherCAT.h
*
* \brief
* PRU Integration: EtherCAT specific interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-20
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined __PRU_ETHERCAT_H__)
#define __PRU_ETHERCAT_H__		1

#include <osal.h>

/* PDK */
//#include <protocols/ethercat_slave/include/tieschw.h>
#if (defined SOC_AM335x) || (defined SOC_AM437x) || (defined SOC_AM572x) || (defined SOC_AM574x) || (defined SOC_AM65XX) || (defined SOC_AM64X) || (defined SOC_AM243X) || (defined SOC_AM263X)
#define ENABLE_ONLINE_FIRMWARE_UPGRADE
#ifdef ECAT_LIMITED_DEMO
//Lib Mode
#define TIESC_REVISION_NUMBER               (0x00000001)
#else
//Patch mode
#define TIESC_REVISION_NUMBER               (0x00000011)
#endif

#define TIESC_EEPROM_SIZE                   (0x800)

/**
 * @def PDI_WD_TRIGGER_RX_SOF
 *  Watchdog RX Start of Frame Trigger
 */
#define PDI_WD_TRIGGER_RX_SOF               (0 << 4)

/**
 * @def PDI_WD_TRIGGER_LATCH_IN
 *  Watchdog LATCH IN Trigger
 */
#define PDI_WD_TRIGGER_LATCH_IN             (1 << 4)

/**
 * @def PDI_WD_TRIGGER_SYNC0_OUT
 *  Watchdog SYNC0 Trigger
 */
#define PDI_WD_TRIGGER_SYNC0_OUT            (2 << 4)

/**
 * @def PDI_WD_TRIGGER_SYNC1_OUT
 *  Watchdog SYNC1 Trigger
 */
#define PDI_WD_TRIGGER_SYNC1_OUT            (3 << 4)

#if ENABLE_MULTIPLE_SM_ACCESS_IN_SINGLE_DATAGRAM
#define TIESC_PORT0_TX_DELAY                (0x98)
#else
#define TIESC_PORT0_TX_DELAY                (0x48)
#endif
#define TIESC_PORT1_TX_DELAY                TIESC_PORT0_TX_DELAY

#define TIESC_MDIO_CLKDIV                   (79) //For 2.5MHz MDIO clock: 200/(TIESC_MDIO_CLKDIV+1)

#define TIESC_LINK_POL_ACTIVE_LOW           (1)
#define TIESC_LINK_POL_ACTIVE_HIGH          (0)

/* Uncomment following to enable DC feature of system time compensation via PDI interface instead of ECAT interface
   for synchronizing two independent EtherCAT networks */
//#define SYSTEM_TIME_PDI_CONTROLLED
/*Comment to following to enable PDI  ISR and SYNC ISR in HWI context */
#define  ENABLE_PDI_TASK
/* #define ENABLE_PDI_SWI */
#define  ENABLE_SYNC_TASK

//#define ARM_INTERRUPT_OFFSET    0

/* /#if (defined __TIAM335XGENERIC__) || (defined SOC_AM335x) || (defined SOC_AM437x) || (defined SOC_AM572x) */
#else
#error Required to configure MCU platform here !
#endif

#if (defined OSAL_LINUX)
#define ARM_INTERRUPT_OFFSET        (0)
#elif (defined OSAL_TIRTOS) || (defined OSAL_FREERTOS) || (defined OSAL_FREERTOS_JACINTO)
#if (defined SOC_AM335x)
    #define ARM_INTERRUPT_OFFSET    (0)
#elif (defined SOC_AM437x)
    #define ARM_INTERRUPT_OFFSET    (32)  /* 52 - 20 */
#elif (defined SOC_AM572x) || (defined SOC_AM574x)
    #define ARM_INTERRUPT_OFFSET    (132)
#elif (defined SOC_K2G)
    #define ARM_INTERRUPT_OFFSET_ICSS0 (236)
    #define ARM_INTERRUPT_OFFSET_ICSS1 (244)
#elif (defined SOC_AM65XX)
    #if defined (__aarch64__)
        /* A53 */
        #define ARM_INTERRUPT_OFFSET_ICSS0 (286-20)
        #define ARM_INTERRUPT_OFFSET_ICSS1 (294-20)
        #define ARM_INTERRUPT_OFFSET_ICSS2 (302-20)
    #else
        /* R5F */
        #define ARM_INTERRUPT_OFFSET_ICSS0 (160-20)
        #define ARM_INTERRUPT_OFFSET_ICSS1 (168-20)
        #define ARM_INTERRUPT_OFFSET_ICSS2 (176-20)
    #endif
#elif (defined SOC_AM64X) || (defined SOC_AM243X)
    #if (defined __aarch64__)
        /* A53 */
        #define ARM_INTERRUPT_OFFSET_ICSS0 (286-20)
        #define ARM_INTERRUPT_OFFSET_ICSS1 (294-20)
        #define ARM_INTERRUPT_OFFSET_ICSS2 (302-20)
    #else
        /* R5F */
        #define ARM_INTERRUPT_OFFSET_ICSS0 (120-20)
        #define ARM_INTERRUPT_OFFSET_ICSS1 (248-20)
    #endif
#elif (defined SOC_AM263X)
     /* R5F */
     #define ARM_INTERRUPT_OFFSET_ICSS0 (120-20)
     #define ARM_INTERRUPT_OFFSET_ICSS1 (248-20)
#else
 #error Required to configure valid MCU platform
#endif
#else
#error "Unknown OS"
#endif

#if (defined SOC_AM64X) || (defined SOC_AM243X)
#define MCU_getARMInterruptOffset(pruInst) \
    ( (0 == (pruInst))?((int16_t)ARM_INTERRUPT_OFFSET_ICSS0):((int16_t)ARM_INTERRUPT_OFFSET_ICSS1) )

#elif (defined SOC_AM65XX)
#define MCU_getARMInterruptOffset       ((int16_t)(ARM_INTERRUPT_OFFSET_ICSS0)) /* always zero, IRQs are mapped to low, depending on PRU */
#else
#define MCU_getARMInterruptOffset       ((int16_t)(ARM_INTERRUPT_OFFSET))
#endif

#define     MBX_WRITE_EVENT             ((uint16_t) 0x0100)
#define     MBX_READ_EVENT              ((uint16_t) 0x0200)

//Below constants are not defined in esc.h
#define ESC_ADDR_REV_TYPE                       (0x000)
#define ESC_ADDR_BUILD                          (0x002)
#define ESC_ADDR_PORTDESC                       (0x007)
#define ESC_ADDR_CONFIG_STATION_ALIAS           (0x012)

#define ESC_DL_CONTROL                          (0x100)
#define ESC_ADDR_DLSTATUS                       (0x110)
#define ESC_ADDR_ALCONTROL                      (0x120)
#define ESC_ADDR_ALSTATUS                       (0x130)
#define ESC_ADDR_PDI_CONTROL                    (0x140)
#define ESC_PDI_CONTROL_ELD_ALL_PORTS_MASK      (1 << 1)
#define ESC_ADDR_PDI_CONFIG                     (0x150)
#define ESC_ADDR_AL_EVENT_MASK                  (0x204)
#define ESC_ADDR_AL_EVENT_REQ                   (0x220)
#define ESC_ADDR_RX_ERRCOUNT                    (0x300)
#define ESC_ADDR_FRX_ERRCOUNT                   (0x308)
#define ESC_ADDR_EPU_ERRCOUNT                   (0x30C)
#define ESC_ADDR_SM_WD_STATUS                   (0x440)
#define ESC_ADDR_EEPROM_CTRL                    (0x502)
#define ESC_ADDR_MI_ECAT_ACCESS                 (0x516)
#define ESC_ADDR_MI_PDI_ACCESS                  (0x517)
#define ESC_MII_PDI_ACCESS                      (0x00000001)

#define EC_API_SLV_SEeprom_t_CMD_MASK           (0x0700)  //Description (0x502.8:10): Command bit mask
#define EC_API_SLV_SEeprom_t_CMD_READ_MASK      (0x0100)  //Description (0x502.8): Currently executed read command
#define EC_API_SLV_SEeprom_t_CMD_WRITE_MASK     (0x0200)  //Description (0x502.9): Initialize Write Command
#define EC_API_SLV_SEeprom_t_CMD_RELOAD_MASK    (0x0400)  //Description (0x502.10): Trigger EEPROM reload
#define EC_API_SLV_SEeprom_t_ERROR_MASK         (0x7800)  //Description : Mask all EEPROM error bits; Checksum error (0x0502.11); EEPROM not loaded (0x0502.12); Missing EEPROM Acknowledge (0x0502.13); Write Error (0x0502.14)
#define EC_API_SLV_SEeprom_t_ERROR_CRC          (0x0800)  //Description (0x502.11): EEPROM CRC Error
#define EC_API_SLV_SEeprom_t_ERROR_CMD_ACK      (0x2000)  //Description (0x502.13): EEPROM Busy
#define EC_API_SLV_SEeprom_t_BUSY_MASK          (0x8000)  //Description (0x502.15): EEPROM Busy

#define ESC_ADDR_SYNCMAN                        (0x800)

#define ESC_ADDR_SM1_STATUS                     (0x80D)
#define SM_STATUS_MBX_FULL                      (0x08)

#define ESC_ADDR_SM0_STATUS                     (0x805)
#define ESC_ADDR_SM0_ACTIVATE                   (0x806+(8*0))
#define ESC_ADDR_SM1_ACTIVATE                   (0x806+(8*1))
#define ESC_ADDR_SM2_ACTIVATE                   (0x806+(8*2))
#define ESC_ADDR_SM3_ACTIVATE                   (0x806+(8*3))
#define ESC_ADDR_SM4_ACTIVATE                   (0x806+(8*4))
#define ESC_ADDR_SM5_ACTIVATE                   (0x806+(8*5))
#define ESC_ADDR_SM6_ACTIVATE                   (0x806+(8*6))
#define ESC_ADDR_SM7_ACTIVATE                   (0x806+(8*7))
#define ESC_ADDR_SM0_PDI_CONTROL                (0x807+(8*0))
#define ESC_ADDR_SM1_PDI_CONTROL                (0x807+(8*1))
#define ESC_ADDR_SM2_PDI_CONTROL                (0x807+(8*2))
#define ESC_ADDR_SM3_PDI_CONTROL                (0x807+(8*3))
#define ESC_ADDR_SM4_PDI_CONTROL                (0x807+(8*4))
#define ESC_ADDR_SM5_PDI_CONTROL                (0x807+(8*5))
#define ESC_ADDR_SM6_PDI_CONTROL                (0x807+(8*6))
#define ESC_ADDR_SM7_PDI_CONTROL                (0x807+(8*7))

#define SM_PDI_CONTROL_SM_DISABLE               (1)

#define ESC_ADDR_SYSTIME                        (0x910)
#define ESC_ADDR_SYSTIME_HIGH                   (0x914)
#define ESC_ADDR_SYSTIME_OFFSET                 (0x920)
#define ESC_ADDR_SYSTIME_DELAY                  (0x928)
#define ESC_ADDR_SPEEDCOUNTER_START             (0x930)
#define ESC_ADDR_TIMEDIFF_FILTDEPTH             (0x934)
#define ESC_ADDR_SPEEDDIFF_FILTDEPTH            (0x935)
#define ESC_ADDR_SYNC_PULSE_LENGTH              (0x982)
#define ESC_ADDR_SYNC_STATUS                    (0x98E)
#define ESC_ADDR_LATCH0_CONTROL                 (0x9A8)
#define ESC_ADDR_LATCH1_CONTROL                 (0x9A9)
#define ESC_ADDR_LATCH0_POS_EDGE                (0x9B0)
#define ESC_ADDR_LATCH0_NEG_EDGE                (0x9B8)
#define ESC_ADDR_LATCH1_POS_EDGE                (0x9C0)
#define ESC_ADDR_LATCH1_NEG_EDGE                (0x9C8)
#define ESC_ADDR_TI_PORT0_ACTIVITY              (0xE00)
#define ESC_ADDR_TI_PORT1_ACTIVITY              (0xE04)
#define ESC_ADDR_TI_PORT0_PHYADDR               (0xE08)
#define ESC_ADDR_TI_PORT1_PHYADDR               (0xE09)
#define ESC_ADDR_TI_PDI_ISR_PINSEL              (0xE0A)
#define ESC_ADDR_TI_PHY_LINK_POLARITY           (0XE0C)
#define ESC_ADDR_TI_PORT0_TX_START_DELAY        (0xE10)
#define ESC_ADDR_TI_PORT1_TX_START_DELAY        (0xE12)
#define ESC_ADDR_TI_ESC_RESET                   (0xE14)
#define ESC_ADDR_TI_EDMA_LATENCY_ENHANCEMENT    (0xE24)
#define TI_ESC_RST_CMD_U                        (0x545352)
#define TI_ESC_RST_CMD_L                        (0x747372)

#define ESC_ADDR_MEMORY                         (0x1000)

#define CMD_DL_USER_CLEAR_AL_EVENT_HIGH         (0x0)
#define CMD_DL_USER_GET_BUFFER_READ_ADDR        (0x1)
#define CMD_DL_USER_GET_BUFFER_WRITE_ADDR       (0x2)
#define CMD_DL_USER_SET_BUFFER_WRITE_DONE       (0x3)
/**
 * @def CMD_DL_USER_ACK_MBX_READ
 *  Mailbox read ACK
 */
#define CMD_DL_USER_ACK_MBX_READ                (0x4)
/**
 * @def CMD_DL_USER_ACK_MBX_WRITE
 *  Mailbox write ACK
 */
#define CMD_DL_USER_ACK_MBX_WRITE               (0x5)
/**
 * @def CMD_DL_USER_EEPROM_CMD_ACK
 *  User EEPROM ACK
 */
#define CMD_DL_USER_EEPROM_CMD_ACK              (0x6)
/**
 * @def CMD_DL_USER_READ_SYNC_STATUS
 *  User Read sync status
 */
#define CMD_DL_USER_READ_SYNC_STATUS            (0x7)
#define SYNC0                                   (0)
#define SYNC1                                   (1)
/**
 * @def CMD_DL_USER_READ_AL_CONTROL
 *  User Read AL Control
 */
#define CMD_DL_USER_READ_AL_CONTROL             (0x8)
/**
 * @def CMD_DL_USER_WRITE_AL_STATUS
 *  User Read AL Status
 */
#define CMD_DL_USER_WRITE_AL_STATUS             (0x9)
/**
 * @def CMD_DL_USER_READ_PD_WD_STATUS
 *  User Read PD_WD Status
 */
#define CMD_DL_USER_READ_PD_WD_STATUS           (0xA)
/**
 * @def CMD_DL_USER_READ_SM_ACTIVATE
 *  User Read SM Activate
 */
#define CMD_DL_USER_READ_SM_ACTIVATE            (0xB)
/**
 * @def CMD_DL_USER_WRITE_SM_PDI_CTRL
 *  User Write SM PDI control
 */
#define CMD_DL_USER_WRITE_SM_PDI_CTRL           (0xC)
/**
 * @def CMD_DL_USER_READ_LATCH_TIME
 *  User Read latch time
 */
#define CMD_DL_USER_READ_LATCH_TIME             (0xD)
#define LATCH0_POS_EDGE                         (0)
#define LATCH0_NEG_EDGE                         (1)
#define LATCH1_POS_EDGE                         (2)
#define LATCH1_NEG_EDGE                         (3)

/**
 * @def CMD_DL_USER_READ_SYS_TIME
 *  User Read sys time
 */
#define CMD_DL_USER_READ_SYS_TIME               (0xE)
/**
 * @def CMD_DL_USER_CLEAR_AL_EVENT_LOW
 *  User clear AL event low
 */
#define CMD_DL_USER_CLEAR_AL_EVENT_LOW          (0xF)
#ifdef SYSTEM_TIME_PDI_CONTROLLED
/**
* @def CMD_DL_USER_SYSTIME_PDI_CONTROL
*  User SYStime PDI control
*/
#define CMD_DL_USER_SYSTIME_PDI_CONTROL         (0x10)
#define WRITE_SYSTIME                           (0)
#define WRITE_SYSTIME_OFFSET                    (1)
#define WRITE_FILTER_CONFIG                     (2)
#endif

#ifndef TIMER_INT_HEADER
#define TIMER_INT_HEADER
#endif


#ifndef EE_BUSY_TIMEOUT_VALUE
#define EE_BUSY_TIMEOUT_VALUE                   (0x2000)
#endif

#define ESC_RD                                  (0x02)    /* read acces to ESC */
#define ESC_WR                                  (0x04)    /* write acces to ESC */

#define ESC_PDI_INTERFACE_ON_CHIP_BUS           (0x80)
#define ESC_PDI_INTERFACE_SPI_SLAVE             (0x05)

/* esc.h */
#define ESC_SYSTEMTIME_OFFSET_OFFSET            (0x0920)
#define ESC_SPEED_COUNTER_START_OFFSET          (0x0930)
#define ESC_DC_START_TIME_CYCLIC_OFFSET         (0x0990)

#define DRIFTCTRL_TASK_SYNC_ZERO_CROSS_ADJUST   (0xE0) //PRU_DMEM0

#if (defined __cplusplus)
extern "C" {
#endif

extern void     PRU_EC_fbRun                    (void*                  pHandle_p);
extern void     PRU_EC_FW_set                   (void*                  pHandle_p);
extern uint32_t PRU_EC_FW_insert                (void*                  pHandle_p
                                                ,uint8_t                pruIdx_p
                                                ,uint32_t*              pFirmware_p
                                                ,uint32_t               firmwareSize_p);
extern void     PRU_EC_FW_load                  (void);
extern bool     PRU_isEtherCATDevice            (void);
extern uint32_t PRU_EC_TIMER_getRegister        (void*                  pCtxt_p);
extern uint32_t PRU_EC_TIMER_getRegister_ovl    (void*                  pCtxt_p
                                                ,bool*                  pOverFlow_p);
extern void     PRU_EC_TIMER_clearRegister      (void*                  pCtxt_p);

#if (defined __cplusplus)
}
#endif

#endif /* __PRU_ETHERCAT_H__ */
