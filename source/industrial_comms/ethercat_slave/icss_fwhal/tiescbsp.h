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

#ifndef TIESC_BSP_H_
#define TIESC_BSP_H_

/**
 * \defgroup INDUSTRIAL_COMMS_MODULE APIs for Industrial Communication Protocol FWHALs
 *
 * This module contains APIs which are used by the Industrial Communication Protocol FWHALs(Firmware and Hardware Abstraction Layer).
 */

/**
 *  \defgroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE APIs for Ethercat Slave FWHAL
 *  \ingroup INDUSTRIAL_COMMS_MODULE
 *
 *  EtherCAT FWHAL(Firmware and Hardware Abstraction Layer) APIs implement the key
 *  interface between EtherCAT Slave Controller Emulation firmware and EtherCAT stack.
 *
 *  @{
 */

/**
 *  \file tiescbsp.h
 */

/**
 * \defgroup ETHERCAT_FWHAL_INITIALIZATION Initialization of stack-firmware interface
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_ESC_FW_CMD     ESC Firmware Command API
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_ESI_EEPROM     ESI EEPROM Emulation support
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_SYNC_PROP      Sync Manager properties management
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_SYNC_ACC_CTRL  Sync Manager access and control APIs
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_SYNC_MAILBOX   Sync Manager Mailbox mode handling
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_SYNC_BUFFER    Sync Manager Buffer mode handling
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_ESC_REG_ACCESS ESC Register/Memory access. Special handling APIs
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_PRU_MDIO       PRU-ICSS MDIO host side APIs
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_PRU_DIGIO      PRU-ICSS DIGIO host side APIs
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_SPINLOCK       Spinlock APIs for concurrent Host/Firmware shared memory access
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_SYSTIME_PDI    System Time PDI controlled APIs
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 *  These APIs are available only if SYSTEM_TIME_PDI_CONTROLLED macro is enabled
 * \defgroup ETHERCAT_FWHAL_TIMER          Timer APIs for EtherCAT stack
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_MUTEX          Mutex APIs for EtherCAT stack
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_INTMGMNT       Interrupt management APIs for EtherCAT stack
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 * \defgroup ETHERCAT_FWHAL_PRU_FW         PRU Firmware header mapping API
 * \ingroup INDUSTRIAL_COMMS_ETHERCAT_SLAVE_FWHAL_MODULE
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifndef DISABLE_UART_PRINT
#include <stdio.h>
#endif

#include <industrial_comms/ethercat_slave/icss_fwhal/tiesc_pruss_intc_mapping.h>
#include <drivers/pruicss.h>
#include <industrial_comms/ethercat_slave/icss_fwhal/tiesc_def.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/**
 * \brief TIESC_MAX_FRAME_SIZE
 *  Maximum frame size cutoff of 2000 bytes
 */
#define TIESC_MAX_FRAME_SIZE    (0x7CF)

/*Single datagram accessing contiguous multiple FMMU mapped areas in  a single slave for process data
is supported now by TI ESC firmware.
Process path latency in TI ESC is high when this support is active
For specific use cases (4SM with 3 FMMUs or multiple FMMUs (in a given ESC) are not accessed in a single datagram)
process path latency improvement can be achieved by disabling below define */
#define ENABLE_MULTIPLE_SM_ACCESS_IN_SINGLE_DATAGRAM     0

#define    MAX_SYNC_MAN                         8
#define    SIZEOF_SM_REGISTER                   8

#define TIESC_EEPROM_SIZE         0x800

#define    MAILBOX_WRITE                        0
#define    MAILBOX_READ                         1
#define    PROCESS_DATA_OUT                     2
#define    PROCESS_DATA_IN                      3

#define     MBX_WRITE_EVENT                 ((uint16_t) 0x0100)
#define     MBX_READ_EVENT                  ((uint16_t) 0x0200)

//Below constants are not defined in esc.h
#define ESC_ADDR_REV_TYPE       0x000
#define ESC_ADDR_BUILD          0x002

#define ESC_ADDR_CONFIG_STATION_ALIAS   0x012
#define ESC_ADDR_DLSTATUS       0x110
#define ESC_ADDR_ALCONTROL      0x120
#define ESC_ADDR_ALSTATUS       0x130
#define ESC_ADDR_PDI_CONTROL    0x140
#define ESC_PDI_CONTROL_ELD_ALL_PORTS_MASK (1 << 1)
#define ESC_ADDR_PDI_CONFIG     0x150
#define ESC_ADDR_AL_EVENT_MASK  0x204
#define ESC_ADDR_AL_EVENT_REQ   0x220
#define ESC_ADDR_SM_WD_STATUS   0x440
#define ESC_ADDR_EEPROM_CTRL    0x502
#define ESC_ADDR_MI_ECAT_ACCESS 0x516
#define ESC_ADDR_MI_PDI_ACCESS  0x517

#define ESC_EEPROM_CMD_MASK                     0x0700 //Description (0x502.8:10): Command bit mask
#define ESC_EEPROM_CMD_READ_MASK                0x0100 //Description (0x502.8): Currently executed read command
#define ESC_EEPROM_CMD_WRITE_MASK               0x0200 //Description (0x502.9): Initialize Write Command
#define ESC_EEPROM_CMD_RELOAD_MASK              0x0400 //Description (0x502.10): Trigger EEPROM reload
#define ESC_EEPROM_ERROR_MASK                   0x7800 //Description : Mask all EEPROM error bits; Checksum error (0x0502.11); EEPROM not loaded (0x0502.12); Missing EEPROM Acknowledge (0x0502.13); Write Error (0x0502.14)
#define ESC_EEPROM_ERROR_CRC                    0x0800 //Description (0x502.11): EEPROM CRC Error
#define ESC_EEPROM_ERROR_CMD_ACK                0x2000 //Description (0x502.13): EEPROM Busy
#define ESC_EEPROM_BUSY_MASK                    0x8000  //Description (0x502.15): EEPROM Busy

#define ESC_ADDR_SYNCMAN 0x800

#define ESC_ADDR_SM1_STATUS 0x80D
#define SM_STATUS_MBX_FULL  0x08

#define ESC_ADDR_SM0_STATUS     0x805
#define ESC_ADDR_SM0_ACTIVATE   0x806
#define ESC_ADDR_SM1_ACTIVATE   0x806+8
#define ESC_ADDR_SM2_ACTIVATE   0x806+8*2
#define ESC_ADDR_SM3_ACTIVATE   0x806+8*3
#define ESC_ADDR_SM4_ACTIVATE   0x806+8*4
#define ESC_ADDR_SM5_ACTIVATE   0x806+8*5
#define ESC_ADDR_SM6_ACTIVATE   0x806+8*6
#define ESC_ADDR_SM7_ACTIVATE   0x806+8*7
#define ESC_ADDR_SM0_PDI_CONTROL    0x807
#define ESC_ADDR_SM1_PDI_CONTROL    0x807+8
#define ESC_ADDR_SM2_PDI_CONTROL    0x807+8*2
#define ESC_ADDR_SM3_PDI_CONTROL    0x807+8*3
#define ESC_ADDR_SM4_PDI_CONTROL    0x807+8*4
#define ESC_ADDR_SM5_PDI_CONTROL    0x807+8*5
#define ESC_ADDR_SM6_PDI_CONTROL    0x807+8*6
#define ESC_ADDR_SM7_PDI_CONTROL    0x807+8*7

#define SM_PDI_CONTROL_SM_DISABLE      1

#define ESC_ADDR_SYSTIME            0x910
#define ESC_ADDR_SYSTIME_HIGH       0x914
#define ESC_ADDR_SYSTIME_OFFSET     0x920
#define ESC_ADDR_SYSTIME_DELAY      0x928
#define ESC_ADDR_SPEEDCOUNTER_START 0x930
#define ESC_ADDR_TIMEDIFF_FILTDEPTH 0x934
#define ESC_ADDR_SPEEDDIFF_FILTDEPTH 0x935
#define ESC_ADDR_SYNC_PULSE_LENGTH  0x982
#define ESC_ADDR_SYNC_STATUS        0x98E
#define ESC_ADDR_LATCH0_CONTROL     0x9A8
#define ESC_ADDR_LATCH1_CONTROL     0x9A9
#define ESC_ADDR_LATCH0_POS_EDGE    0x9B0
#define ESC_ADDR_LATCH0_NEG_EDGE    0x9B8
#define ESC_ADDR_LATCH1_POS_EDGE    0x9C0
#define ESC_ADDR_LATCH1_NEG_EDGE    0x9C8
#define ESC_ADDR_TI_PORT0_ACTIVITY  0xE00
#define ESC_ADDR_TI_PORT1_ACTIVITY  0xE04
#define ESC_ADDR_TI_PORT0_PHYADDR   0xE08
#define ESC_ADDR_TI_PORT1_PHYADDR   0xE09
#define ESC_ADDR_TI_PDI_ISR_PINSEL  0xE0A
#define ESC_ADDR_TI_PHY_LINK_POLARITY   0XE0C
#define ESC_ADDR_TI_PORT0_TX_START_DELAY    0xE10
#define ESC_ADDR_TI_PORT1_TX_START_DELAY    0xE12
#define ESC_ADDR_TI_ESC_RESET       0xE14
#define ESC_ADDR_TI_EDMA_LATENCY_ENHANCEMENT    0xE24
#define ESC_ADDR_TI_PHY_RX_ER_REG               0xE28
#define ESC_ADDR_TI_PRU_CLK_FREQUENCY           0xE34
#define ESC_ADDR_TI_MDIO_MANUAL_MODE            0xE35
#define ESC_ADDR_TI_ENHANCED_LINK_DETECT        0xE36
#define TI_ESC_RST_CMD_U    0x545352
#define TI_ESC_RST_CMD_L    0x747372

#define ESC_ADDR_MEMORY         0x1000

#define CMD_DL_USER_CLEAR_AL_EVENT_HIGH     0x0
#define CMD_DL_USER_GET_BUFFER_READ_ADDR    0x1
#define CMD_DL_USER_GET_BUFFER_WRITE_ADDR   0x2
#define CMD_DL_USER_SET_BUFFER_WRITE_DONE   0x3
/**
 * \brief CMD_DL_USER_ACK_MBX_READ
 *  Mailbox read ACK
 */
#define CMD_DL_USER_ACK_MBX_READ            0x4
/**
 * \brief CMD_DL_USER_ACK_MBX_WRITE
 *  Mailbox write ACK
 */
#define CMD_DL_USER_ACK_MBX_WRITE           0x5
/**
 * \brief CMD_DL_USER_EEPROM_CMD_ACK
 *  User EEPROM ACK
 */
#define CMD_DL_USER_EEPROM_CMD_ACK          0x6
/**
 * \brief CMD_DL_USER_READ_SYNC_STATUS
 *  User Read sync status
 */
#define CMD_DL_USER_READ_SYNC_STATUS        0x7
#define SYNC0   0
#define SYNC1   1
/**
 * \brief CMD_DL_USER_READ_AL_CONTROL
 *  User Read AL Control
 */
#define CMD_DL_USER_READ_AL_CONTROL         0x8
/**
 * \brief CMD_DL_USER_WRITE_AL_STATUS
 *  User Read AL Status
 */
#define CMD_DL_USER_WRITE_AL_STATUS         0x9
/**
 * \brief CMD_DL_USER_READ_PD_WD_STATUS
 *  User Read PD_WD Status
 */
#define CMD_DL_USER_READ_PD_WD_STATUS       0xA
/**
 * \brief CMD_DL_USER_READ_SM_ACTIVATE
 *  User Read SM Activate
 */
#define CMD_DL_USER_READ_SM_ACTIVATE        0xB
/**
 * \brief CMD_DL_USER_WRITE_SM_PDI_CTRL
 *  User Write SM PDI control
 */
#define CMD_DL_USER_WRITE_SM_PDI_CTRL       0xC
/**
 * \brief CMD_DL_USER_READ_LATCH_TIME
 *  User Read latch time
 */
#define CMD_DL_USER_READ_LATCH_TIME         0xD
#define LATCH0_POS_EDGE 0
#define LATCH0_NEG_EDGE 1
#define LATCH1_POS_EDGE 2
#define LATCH1_NEG_EDGE 3

/**
 * \brief CMD_DL_USER_READ_SYS_TIME
 *  User Read sys time
 */
#define CMD_DL_USER_READ_SYS_TIME           0xE
/**
 * \brief CMD_DL_USER_CLEAR_AL_EVENT_LOW
 *  User clear AL event low
 */
#define CMD_DL_USER_CLEAR_AL_EVENT_LOW      0xF
#ifdef SYSTEM_TIME_PDI_CONTROLLED
/**
* \brief CMD_DL_USER_SYSTIME_PDI_CONTROL
*  User SYStime PDI control
*/
#define CMD_DL_USER_SYSTIME_PDI_CONTROL    0x10
#define WRITE_SYSTIME              0
#define WRITE_SYSTIME_OFFSET       1
#define WRITE_FILTER_CONFIG        2
#endif

#define SWAPWORD
#define SWAPDWORD

#define ICSS_MDIO_USRPHYSEL_LINKINT_ENABLE 0x40
#define ICSS_MDIO_USRPHYSEL_LINKSTAT_MLINK 0x80

#define TIESC_PERM_RW       0x0
#define TIESC_PERM_WRITE_ONLY 0x1
#define TIESC_PERM_READ_ONLY 0x2

#define TIESC_PERM_WRITE TIESC_PERM_WRITE_ONLY
#define TIESC_PERM_READ TIESC_PERM_READ_ONLY

#define PDI_PERM_RW        0x0
#define PDI_PERM_READ_ONLY 0x1

#define PDI_PERM_WRITE PDI_PERM_RW
#define PDI_PERM_READ PDI_PERM_READ_ONLY

#define TIESC_MDIO_CLKDIV   79 //For 2.5MHz MDIO clock: 200/(TIESC_MDIO_CLKDIV+1)

#define TIESC_MDIO_RX_LINK_DISABLE  0 //Slow MDIO state m/c based link detection
#define TIESC_MDIO_RX_LINK_ENABLE   1 //Fast link detect using RXLINK forward from PHY to MDIO MLINK
#define TIESC_LINK_POL_ACTIVE_LOW       1
#define TIESC_LINK_POL_ACTIVE_HIGH      0

/**
 * \brief PDI_WD_TRIGGER_RX_SOF
 *  Watchdog RX Start of Frame Trigger
 */
#define PDI_WD_TRIGGER_RX_SOF       (0 << 4)

/**
 * \brief PDI_WD_TRIGGER_LATCH_IN
 *  Watchdog LATCH IN Trigger
 */
#define PDI_WD_TRIGGER_LATCH_IN     (1 << 4)

/**
 * \brief PDI_WD_TRIGGER_SYNC0_OUT
 *  Watchdog SYNC0 Trigger
 */
#define PDI_WD_TRIGGER_SYNC0_OUT    (2 << 4)

/**
 * \brief PDI_WD_TRIGGER_SYNC1_OUT
 *  Watchdog SYNC1 Trigger
 */
#define PDI_WD_TRIGGER_SYNC1_OUT    (3 << 4)

#if ENABLE_MULTIPLE_SM_ACCESS_IN_SINGLE_DATAGRAM
#define TIESC_PORT0_TX_DELAY_200_MHZ_CLOCK    0x98
#else
#define TIESC_PORT0_TX_DELAY_200_MHZ_CLOCK    0x48
#endif
#define TIESC_PORT1_TX_DELAY_200_MHZ_CLOCK    TIESC_PORT0_TX_DELAY_200_MHZ_CLOCK

#if ENABLE_MULTIPLE_SM_ACCESS_IN_SINGLE_DATAGRAM
#define TIESC_PORT0_TX_DELAY_333_MHZ_CLOCK    0xA0
#else
#define TIESC_PORT0_TX_DELAY_333_MHZ_CLOCK    0x50
#endif
#define TIESC_PORT1_TX_DELAY_333_MHZ_CLOCK    TIESC_PORT0_TX_DELAY_333_MHZ_CLOCK

#define PDI_ISR_EDIO_NUM    7 //GPMC_CSN(2) -> pr1_edio_data_out7 for ICEv2.J4.Pin21

/* PDI  side register protection using register permission table (4KB) in memory - disable if you care for performance and memory foot print */
/* #define ENABLE_PDI_REG_PERMISSIONS */

/* Use ESC system time instead of SYS/BIOS Timestamp_get32 for timing info */
#define USE_ECAT_TIMER


/* Uncomment following to enable DC feature of system time compensation via PDI interface instead of ECAT interface
   for synchronizing two independent EtherCAT networks */
//#define SYSTEM_TIME_PDI_CONTROLLED
/*Comment to following to enable PDI  ISR and SYNC ISR in HWI context */
#define  ENABLE_PDI_TASK
#define  ENABLE_SYNC_TASK

/**
 * \brief SUPPORT_CMDACK_POLL_MODE
 * If PDI and SYNC ISR is handled in HWI context (similar to SSC)
 * interrupt mode of CMDACK won't work as they are low priority than
 * SYNC and PDI ISR - use polling instead
 */
/* #define  SUPPORT_CMDACK_POLL_MODE */

#if defined (__aarch64__)
#define ASSERT_DMB()  __asm__("    dmb ISH")
#define ASSERT_DSB()  __asm__("    dsb ISH")
#else
#define ASSERT_DMB() __asm__ __volatile__ ("    dmb" "\n\t": : : "memory")
#define ASSERT_DSB() __asm__ __volatile__ ("    dsb" "\n\t": : : "memory")
#endif

#ifdef USE_ECAT_TIMER
#define ECAT_TIMER_INC_P_MS 1000000
#else
#define ECAT_TIMER_INC_P_MS ecat_timer_inc_p_ms /* ARM frequency: Will be detected during bsp_init*/
extern volatile uint32_t ecat_timer_inc_p_ms;
#endif

#define ESC_SYSTEMTIME_OFFSET_OFFSET        0x0920
#define ESC_SPEED_COUNTER_START_OFFSET      0x0930
#define ESC_DC_START_TIME_CYCLIC_OFFSET     0x0990

#define DRIFTCTRL_TASK_SYNC_ZERO_CROSS_ADJUST   0xE0 //PRU_DMEM0

/**
 * \brief LOCK_PD_BUF_AVAILABLE_FOR_HOST
 *  LOCK available for HOST
 */
#define LOCK_PD_BUF_AVAILABLE_FOR_HOST  0
/**
 * \brief LOCK_PD_BUF_HOST_ACCESS_START
 *  Lock available for host access start
 */
#define LOCK_PD_BUF_HOST_ACCESS_START   1
/**
 * \brief LOCK_PD_BUF_HOST_ACCESS_FINISH
 *  Lock available for host access finish
 */
#define LOCK_PD_BUF_HOST_ACCESS_FINISH  2

/**
 * \brief LOCK_PD_BUF_CHECK_AVAILABILITY_RETRY_COUNT
 *  Number of times the lock_state of SM is checked for availability in
 *  \ref bsp_get_process_data_address function, in case the lock_state is not
 *  \ref LOCK_PD_BUF_HOST_ACCESS_START
 */
#define LOCK_PD_BUF_CHECK_AVAILABILITY_RETRY_COUNT    (10U)

/**
 * \brief TIESC_PRUICSS_CLOCK_FREQUENCY_200_MHZ
 *  PRU-ICSS Core Clock and IEP Clock running at 200 MHz
 */
#define TIESC_PRUICSS_CLOCK_FREQUENCY_200_MHZ (0)

/**
 * \brief TIESC_PRUICSS_CLOCK_FREQUENCY_333_MHZ
 *  PRU-ICSS Core Clock and IEP Clock running at 333 MHz
 */
#define TIESC_PRUICSS_CLOCK_FREQUENCY_333_MHZ (1)

/**
 * \brief TIESC_MDIO_HW_MODE
 *  To set the SoC HW mode configuration for MDIO communication
 */
#define TIESC_MDIO_HW_MODE                    (0)

/**
 * \brief TIESC_MDIO_MANUAL_MODE_FW
 *  To set the manual mode configuration for MDIO communication using FW
 */
#define TIESC_MDIO_MANUAL_MODE_FW             (1)

typedef int32_t (*bsp_eeprom_read_t)(uint8_t *buf, uint32_t len);
typedef int32_t (*bsp_eeprom_write_t)(uint8_t *buf, uint32_t len);
typedef void (*bsp_init_spinlock_t)(void);
typedef uint32_t (*bsp_hwspinlock_lock_t)(int num);
typedef void (*bsp_hwspinlock_unlock_t)(int num);
typedef void (*bsp_ethphy_init_t)(PRUICSS_Handle pruIcssHandle, uint8_t phy0addr, uint8_t phy1addr, uint8_t enhancedlink_enable);
typedef int8_t (*bsp_get_phy_address_t)(uint8_t instance, uint8_t portNumber);
typedef void (*bsp_ethercat_stack_isr_function)(void);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Struct for FWHAL initialization Parameters
 */
typedef struct bsp_params_s
{
    PRUICSS_Handle                      pruicss_handle;
    /**< PRUICSS Handle*/
    int32_t                             interrupt_offset;
    /**< Interrupt Input Line number on ARM processor for PRU_ICSSG_PR1_HOST_INTR_PEND_0
     *   interrupt. Out of 8 interrupts PRU_ICSSG_PR1_HOST_INTR_PEND_0 to
     *   PRU_ICSSG_PR1_HOST_INTR_PEND_7, following 4 are used for EtherCAT: \n
     *   PRU_ICSSG_PR1_HOST_INTR_PEND_1 : DC SYNC0 OUT \n
     *   PRU_ICSSG_PR1_HOST_INTR_PEND_2 : DC SYNC1 OUT \n
     *   PRU_ICSSG_PR1_HOST_INTR_PEND_3 : PDI Interrupt \n
     *   PRU_ICSSG_PR1_HOST_INTR_PEND_4 : ESC Command Acknowledgement*/
    bsp_eeprom_read_t                   eeprom_read;
    /**< Function pointer to EEPROM Read function */
    bsp_eeprom_write_t                  eeprom_write;
    /**< Function pointer to EEPROM Read function */
    uint32_t                            spinlock_base_address;
    /**< Base address for HW spinlock. This is needed to prevent concurrent
     *   Host/Firmware shared memory access while reading latch timestamps. */
    bsp_ethphy_init_t                   ethphy_init;
    /**< Callback function for EtherCAT specific Ethernet PHY initialization*/
    uint8_t                             enhancedlink_enable;
    /**< Enable enhanced link detection using MII RXLINK and PHY's enhanced link detection features.
     *   TIESC_MDIO_RX_LINK_ENABLE for enable, TIESC_MDIO_RX_LINK_DISABLE for disable */
    uint32_t                            link0_polarity;
    /**< Link Polarity for Port 0*/
    uint32_t                            link1_polarity;
    /**< Link Polarity for Port 1*/
    uint32_t                            phy0_address;
    /**< Ethernet PHY Address for Port 0*/
    uint32_t                            phy1_address;
    /**< Ethernet PHY Address for Port 1*/
    const unsigned char                 *default_tiesc_eeprom;
    /**< Pointer to EEPROM array corresponding to ESI XML */
    uint8_t                             **eeprom_pointer_for_stack;
    /**< Double pointer to eeprom variable which will be used by stack.
         \ref bsp_init will populate this pointer appropriately */
    bsp_ethercat_stack_isr_function     pdi_isr;
    /**< PDI IRQ handler in the EtherCAT slave stack. Needed only if ENABLE_PDI_TASK is not enabled */
    bsp_ethercat_stack_isr_function     sync0_isr;
    /**< SYNC0 IRQ handler in the EtherCAT slave stack. Needed only if ENABLE_SYNC_TASK is not enabled */
    bsp_ethercat_stack_isr_function     sync1_isr;
    /**< SYNC1 IRQ handler in the EtherCAT slave stack. Needed only if ENABLE_SYNC_TASK is not enabled */
    uint16_t                            phy_rx_err_reg;
    /**< Address of PHY register maintaining RX Error (RX_ERR) count during frame(when RX_DV is asserted).
     *   This value will be configured in TI ESC's PHY RX Error Counter Register (0x0E28-0x0E29) */
    uint8_t                             pruicssClkFreq;
    /**< PRU-ICSS Core Clock and IEP Clock Frequency.
     *   Set TIESC_PRUICSS_CLOCK_FREQUENCY_200_MHZ/TIESC_PRUICSS_CLOCK_FREQUENCY_333_MHZ. Default is TIESC_PRUICSS_CLOCK_FREQUENCY_200_MHZ.
     *   NOTE : Only applicable for PRU-ICSSG (AM64x/AM243x). Not applicable for PRU-ICSSM(AM263x). */
    uint8_t                             mdioManualMode;
    /**< MDIO MANUAL MODE using PRU FW selection.
     *   When set to 1, it enables usage of PRU FW for MDIO communication with PHYs, else uses MDIO HW only.
     *   NOTE : Only applicable for PRU-ICSSG (AM64x/AM243x). Not applicable for PRU-ICSSM(AM263x). */
} bsp_params;

/**
 * \brief Struct for host to PRU-ICSS command interface
 */
typedef struct
{
    uint8_t sm_buf_index;     /**< Sync manager buff index */
    uint8_t lock_state;       /**< Lock state. Can have \ref LOCK_PD_BUF_AVAILABLE_FOR_HOST, \ref LOCK_PD_BUF_HOST_ACCESS_START, \ref LOCK_PD_BUF_HOST_ACCESS_FINISH      */
    uint16_t addr;            /**< Address */
} t_sm_processdata;

/**
 * \brief Struct for host to PRU-ICSS command interface
 * Starts at PRU0 DMEM
 */
typedef struct
{
    uint8_t reserved1[0x90];          /**< Reserved */
    uint32_t system_time_low;         /**< System Time low */
    uint32_t system_time_high;        /**< System Time high */
    uint8_t sm_config_ongoing;        /**< Sync manageer config */
    uint8_t reserved2[7];             /**< Reserved */
    uint16_t cmdlow;                  /**< CMD low */
    uint16_t cmdlow_ack;              /**< CMD low ack */
    uint16_t param1low;               /**< PARAM1 low */
    uint16_t param2low;               /**< PARAM2 low */
    uint16_t resp1low;                /**< RESP1 low */
    uint16_t resp2low;                /**< RESP2 low */
#ifndef SYSTEM_TIME_PDI_CONTROLLED
    uint8_t reserved3[212];           /**< Reserved */
#else
    uint8_t reserved3[24];            /**< Reserved */
    uint32_t systime_offset_low;      /**< System time offset low */
    uint32_t systime_offset_high;     /**< System time offset high */
    uint8_t reserved4[180];           /**< Reserved */
#endif
    t_sm_processdata sm_processdata[6]; /**< Sync manager process data */
} t_host_interface;

/**
 * \brief Struct for register permission array
 */
typedef struct
{
    uint8_t reserved[1024];       /**< Reserved */
    uint8_t reg_properties[4096]; /**< Register properties */
} t_register_properties;

typedef struct
{
    uint16_t physical_start_addr;
    uint16_t length;
} t_sm_properties;

/**
 * \brief Struct for MDIO initialization parameters
 */
typedef struct
{
    uint16_t clkdiv;      /**< MDIO clkdiv. MDIO clock = 200/(clkdiv+1) */
    uint8_t addr0;        /**< Address of the PHY hooked to PRU-ICSS MII0 */
    uint8_t addr1;        /**< Address of the PHY hooked to PRU-ICSS MII1 */
    uint8_t link0pol; /* LINK_MII signal polarity of PHY hooked to PRU-ICSS MII0. 1: Active lLow 0: Active High */
    uint8_t link1pol; /* LINK_MII signal polarity of PHY hooked to PRU-ICSS MII1. 1: Active lLow 0: Active High */
    uint8_t enhancedlink_enable;  /**< Enable enhanced link detection using MII RXLINK and TLK110 enhanced link detection features*/
} t_mdio_params;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/** \addtogroup ETHERCAT_FWHAL_INITIALIZATION
 @{ */

/**
*  \brief Initialize the members of bsp_params with default values.
*
*  \param init_params  Structure of type bsp_paramss
*
*/
void bsp_params_init(bsp_params *init_params);

/**
*  \brief Initialize the EtherCAT FWHAL \n
*         It does following things: \n
*         1. Setup PRU-ICSS interrupt controller for EtherCAT application \n
*         2. Initialize the PRU data memories \n
*         3. Setup register permissions by invoking bsp_esc_reg_perm_init \n
*         4. Load and start EtherCAT PRU firmware \n
*         5. Initialize EEPROM emulation \n
*         6. Initialize command semaphore \n
*
*  \param init_params  Structure of type bsp_params
*
*  \retval  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
*
*/
extern int32_t bsp_init(bsp_params *init_params);

/**
*  \brief Sets up register permissions for ECAT side access for TI ESC,
*  if ENABLE_PDI_REG_PERMISSIONS is defined in tiescbsp.h, then this function
*  also initializes register permissions for PDI side access from stack/application.
*
*  NOTE : ENABLE_PDI_REG_PERMISSIONS
*  This feature is not enabled by default in Industrial SDK for optimizing
*  memory and performance. TI_ESC has Onchip PDI interface where Host CPU
*  has direct access to ESC registers as they are emulated using PRU_ICSS
*  shared data memory.If ENABLE_PDI_REG_PERMISSIONS is defined and bsp_read/write_XXX
*  are used by application for all register accesses then ONLY PDI access
*  check is enforced. If application bypasses the use of bsp_read/write_xxx API
*  (say HWREG to ESC register in PRU_ICSS memory), whole scheme won't work.
*  ENABLE_PDI_REG_PERMISSIONS is a software scheme and has overhead of 4KB data memory
*  to maintain permission array and additional overhead of access check for every read/write
*  to registers. ENABLE_PDI_REG_PERMISSIONS is enabled and bsp_read/write is enabled
*  customer needs to set correct permissions in pdi_reg_perm_array, say for implementing
*  vendor specific register for ESC to activate a host side feature.
*
*  Struct for register permission array
*  Starts at PRU1 DMEM
*  \ref t_register_properties;
*
*  \param pruIcssHandle  PRU-ICSS Handle
*
*/
extern void bsp_esc_reg_perm_init(PRUICSS_Handle
                                 pruIcssHandle);  //Internal API - invoked by bsp_init

/**
*  \brief Register IRQ handlers for various PRU-ICSS interrupts from firmware
*  to host to clear corresponding events in PRU-ICSS INTC
*
*  \param pruIcssHandle  PRU-ICSS Handle
*
*/
extern void bsp_start_esc_isr(PRUICSS_Handle pruIcssHandle);

/**
*  \brief Cleanup of EtherCAT FWHAL \n
*         It does following things: \n
*         1. Delete command semaphore \n
*         2. Flush the EEPROM cache to non-volatile memory \n
*         3. Disable PRUs \n
*
*  \param pruIcssHandle  PRU-ICSS Handle
*
*/
extern void bsp_exit(PRUICSS_Handle pruIcssHandle);

/**
*  \brief Configure PDI WD trigger mode,
*  PDI WD is triggered automatically by h/w on RX_SOF(port0/port1), latch0/1 input high,
*  SYNC0/1 out high. PDI WD is also triggered whenever host sends a command to firmware.
*  PDI WD may not expire if host stops sending commands to firmware alone, this will occur
*  only if configured h/w events do not occur during WD period
*
*  \param pruIcssHandle  PRU-ICSS Handle
*  \param mode   \ref PDI_WD_TRIGGER_RX_SOF \n
*                \ref PDI_WD_TRIGGER_LATCH_IN \n
*                \ref PDI_WD_TRIGGER_SYNC0_OUT \n
*                \ref PDI_WD_TRIGGER_SYNC1_OUT \n
*
*/
extern void bsp_set_pdi_wd_trigger_mode(PRUICSS_Handle pruIcssHandle,
                                       uint32_t mode);
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_ESC_FW_CMD
 @{ */

/**
*  \brief Send command and parameters from stack to firmware to perform some
*  action based on stack state or in response to AL event interrupt or SYNC interrupt from ESC
*
*  \param pruIcssHandle  PRU-ICSS Handle
*  \param command    Command Id to be executed by ESC firmware
*  \param param1     parameter1 corresponding to the command
*  \param param2     parameter2 corresponding to the command
*
*  \details
*  Command                        | Id            | Param1                                    | Param 2       | Return Val    | Remarks                                                                              |
*  -------------------------------| ------------- | ----------------------------------------- | ------------- | ------------- | ------------------------------------------------------------------------------------ |
*  CMD_DL_USER_ACK_MBX_READ       | 4             | address                                   | length        | NA            | Indicate PDI side mailbox read completion to firmware                                |
*  CMD_DL_USER_ACK_MBX_WRITE      | 5             | address                                   | length        | NA            | Indicate PDI side mailbox write to firmware                                          |
*  CMD_DL_USER_EEPROM_CMD_ACK     | 6             | NA                                        | NA            | NA            | Acknowledge EEPROM emulation command execution by ARM host stack                     |
*  CMD_DL_USER_READ_SYNC_STATUS   | 7             | sync_sel                                  | NA            | NA            | Indicate SYNC0/1 status register read by PDI to firmware                             |
*  CMD_DL_USER_READ_AL_CONTROL    | 8             | NA                                        | NA            | NA            | Indicate AL control register read by PDI to firmware                                 |
*  CMD_DL_USER_WRITE_AL_STATUS    | 9             | NA                                        | NA            | NA            | Indicate AL status register write by PDI to firmware                                 |
*  CMD_DL_USER_READ_PD_WD_STATUS  | 10            | NA                                        | NA            | NA            | Indicate PD watchdog status register read by PDI to firmware updating register status|
*  CMD_DL_USER_READ_SM_ACTIVATE   | 11            | sm_index                                  | NA            | NA            | Indicate SM activate register read by PDI to firmware                                |
*  CMD_DL_USER_WRITE_SM_PDI_CTRL  | 12            | (sm_index<<8) OR pdi control reg value    | NA            | NA            | Indicate SM PDI control write to control SM from PDI to firmware                     |
*  CMD_DL_USER_READ_LATCH_TIME    | 13            | latch_sel                                 | NA            | NA            | Indicate Latch0/1 time register read by PDI to firmware                              |
*  CMD_DL_USER_READ_SYS_TIME      | 14            | NA                                        | NA            | NA            | Indicate System Time register read by PDI to firmware                                |
*  CMD_DL_USER_CLEAR_AL_EVENT_LOW | 15            | event_mask                                | NA            | NA            | Clear events not set in event_mask in AL event request register                      |
*  CMD_DL_USER_SYSTIME_PDI_CONTROL| 16            | reg_sel                                   | NA            | NA            | Indicate to the firmware DC register update from PDI side                            |
*
*  Struct for host to PRU ICSS command interface - \ref t_host_interface
*
*  IMPORTANT NOTE on SUPPORT_CMDACK_POLL_MODE:
*  EtherCAT firmware supports both poll mode and interrupt mode for command acknowledge.
*  It is found that poll mode offers better latency in given scenarios but will waste CPU cycles busy waiting
*  for command response from PRU. If you need to switch to command ack interrupt mode comment the define for
*  SUPPORT_CMDACK_POLL_MODE in tiescbsp.h
*/
extern void bsp_send_command_to_firmware(PRUICSS_Handle pruIcssHandle,
                                        uint32_t command, uint16_t param1, uint16_t param2);
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_ESI_EEPROM
 @{ */

/**
*  \brief  Initialize the EEPROM cache in volatile RAM. If the non-volatile storage
*          has valid data(read is performed using eeprom_read callback from \ref bsp_params
*          passed during \ref bsp_init), load from there. Otherwise it loads predefined
*          EEPROM in application using default_tiesc_eeprom from \ref bsp_params
*          passed during \ref bsp_init.
*/
extern void bsp_eeprom_emulation_init(void);

/**
*  \brief For loading ESC registers from EEPROM during first boot/reload after validating CRC
*
*  \param pruIcssHandle PRUSS handle
*  \param reload_flag  If set reload command is being executed else first boot in progress
*
*  \retval 0: On successful load of registers -1: On CRC error
*
*/
extern int32_t bsp_eeprom_load_esc_registers(PRUICSS_Handle pruIcssHandle,
        int32_t reload_flag);

/**
*  \brief Perform reload operation after validating EEPROM CRC
*
*  \param pruIcssHandle PRUSS handle
*
*  \retval 0: On successful load of registers -1: On CRC error
*
*/
extern int32_t bsp_eeprom_emulation_reload(PRUICSS_Handle pruIcssHandle);

/**
*  \brief Perform reload operation after validating EEPROM CRC, Wrapper API for SSC
*
*  \param pruIcssHandle PRUSS handle
*
*/
extern void bsp_eeprom_emulation_command_ack(PRUICSS_Handle pruIcssHandle);

/**
*  \brief Flush the EEPROM cache to non-volatile storage. Write is performed
*         using eeprom_write callback from \ref bsp_params passed during \ref bsp_init.
*
*/
extern void bsp_eeprom_emulation_flush(void);

/**
*  \brief Call EEPROM flush on exit
*
*
*/
extern void bsp_eeprom_emulation_exit(void);

/**
*  \brief  Return pointer to volatile EEPROM cache in FWHAL for processing to access the EEPROM
*
*  \retval Pointer to volatile EEPROM cache start to the stack/application for EEPROM access
*
*/
extern uint8_t *bsp_get_eeprom_cache_base(void);

/**
*  \brief Set EEPROM update time
*
*/
void bsp_set_eeprom_updated_time(void);

/**
*  \brief Get EEPROM Updated time
*
*  \retval EEPROM updated time
*
*/
uint32_t bsp_get_eeprom_updated_time(void);

/**
*  \brief Indicate to FWHAL whether EEPROM is written for flushing to non-volatile
*          storage. Typically called on EEPROM write detection from stack.
*
*  \param status Update the FWHAL EEPROM update status.
*
*/
extern void bsp_set_eeprom_update_status(uint8_t status);

/**
*  \brief Read the EEPROM update status from FWHAL. Typically called from low priority
*         task periodically check EEPROM dirty status for flush.
*
*  \retval EEPROM updated status
*
*/
extern uint8_t bsp_get_eeprom_update_status(void);

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_SYNC_PROP
 @{ */
/**
*  \brief Set the address, length info from register to FWHAL layer. During INIT
*         to PREOP transition in Mailbox mode. During SAFEOP to OP transition in
*         Buffer mode.
*
*  \param pruIcssHandle PRUSS Handle
*  \param sm SyncManager index to set
*  \param address physical start address of SyncManager
*  \param len length of start address of SyncManager
*
*
*/
extern void bsp_set_sm_properties(PRUICSS_Handle pruIcssHandle, uint8_t sm,
                                 uint16_t address, uint16_t len);
/**
*  \brief Get the pointer to requested SM properties. It is used for
*         Buffer/Mailbox read/write detection from Host PDI interface to indicate
*         to the firmware.
*
*  \param sm SyncManager index to get properties from FWHAL
*
*  \retval pointer to requested sm_properties array element in FWHAL
*
*/
extern t_sm_properties *bsp_get_sm_properties(uint8_t sm);

extern int16_t bsp_get_sm_index(uint16_t address, uint16_t len);
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_SYNC_ACC_CTRL
 @{ */
/**
*  \brief Checks whether firmware has finished updating internal state for SM
*         configuration change initiated by stack/PDI
*
*  \param pruIcssHandle PRUSS Handle
*
*  \retval 0 : SM configuration finished Otherwise : SM configuration ongoing
*
*/
extern uint8_t bsp_pdi_sm_config_ongoing(PRUICSS_Handle pruIcssHandle);
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_SYNC_MAILBOX
 @{ */

/**
*  \brief Indicates to the firmware that PDI side read from write mailbox has started
*
*  \param pruIcssHandle PRUSS Handle
*/
extern void bsp_pdi_mbx_read_start(PRUICSS_Handle pruIcssHandle);

/**
*  \brief Indicates to the firmware that PDI side read from write mailbox has completed
*
*  \param pruIcssHandle PRUSS Handle
*/
extern void bsp_pdi_mbx_read_complete(PRUICSS_Handle pruIcssHandle);

/**
*  \brief Indicates to the firmware that PDI side write to read mailbox has started
*
*  \param pruIcssHandle PRUSS Handle
*
*/
extern void bsp_pdi_mbx_write_start(PRUICSS_Handle pruIcssHandle);

/**
*  \brief  Indicates to the firmware that PDI side write to read mailbox has completed
*
*  \param pruIcssHandle PRUSS Handle
*
*/
extern void bsp_pdi_mbx_write_complete(PRUICSS_Handle pruIcssHandle);

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_SYNC_BUFFER
 @{ */
/**
*  \brief  Get the actual address of the buffer for PDI side read/write from host
*          in 3-buffer mode
*
*  \param pruIcssHandle PRUSS Handle
*  \param address SM buffer address for PDI side access
*  \param len SM Buffer length for PDI side access
*  \param p_sm_index Matching SM index corresponding to address
*
*  \retval actual PDI address in memory.
           0 in case of error, or lock unavailability (See \ref LOCK_PD_BUF_CHECK_AVAILABILITY_RETRY_COUNT)
*
*/
extern uint16_t bsp_get_process_data_address(PRUICSS_Handle pruIcssHandle,
        uint16_t address, uint16_t len, int16_t *p_sm_index);

/**
*  \brief  This API is invoked after PDI side completes read/write to PD address
*          returned by bsp_get_process_data_address to indicate this to firmware
*          for swapping buffers etc
*
*  \param pruIcssHandle PRUSS Handle
*  \param address SM buffer address for PDI side access
*  \param len SM buffer length for PDI side access
*  \param sm_index SM index of this process data buffer
*
*/
extern void bsp_process_data_access_complete(PRUICSS_Handle pruIcssHandle,
        uint16_t address, uint16_t len, int16_t sm_index);

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_ESC_REG_ACCESS
 @{ */
/**
*  \brief  Read a byte value at 'Address' from ESC memory: SM mailbox (single buffer)
*          mapped or register area
*
*  \param pruIcssHandle PRUSS Handle
*  \param address ESC address to read from PDI
*
*  \retval Value at address
*
*/
extern uint8_t bsp_read_byte(PRUICSS_Handle pruIcssHandle, uint16_t address);

/**
*  \brief  Read a 16-bit value at 'Address' from ESC memory: SM mailbox (single buffer)
*          mapped or register area
*
*  \param pruIcssHandle PRUSS Handle
*  \param address ESC address to read from PDI
*
*  \retval Value at address
*
*/
extern uint16_t bsp_read_word(PRUICSS_Handle pruIcssHandle, uint16_t address);

/**
*  \brief  Read a 32-bit value at 'Address' from ESC memory: SM mailbox (single buffer)
*          mapped or register area
*
*  \param pruIcssHandle PRUSS Handle
*  \param address ESC address to read from PDI
*
*  \retval Value at address
*
*/
extern uint32_t bsp_read_dword(PRUICSS_Handle pruIcssHandle, uint16_t address);

/**
*  \brief   Read a byte array at 'address' from ESC memory
*
*  \param pruIcssHandle PRUSS Handle
*  \param pdata Pointer to the array in application
*  \param address ESC address to read from PDI
*  \param len Number of bytes to read from ESC memory
*
*/
extern void bsp_read(PRUICSS_Handle pruIcssHandle, uint8_t *pdata,
                    uint16_t address,
                    uint16_t len);

/**
*  \brief Read a byte value at 'Address' from ESC process data memory: SM buffer
*         (3-buffer) mapped area
*
*  \param pruIcssHandle PRUSS Handle
*  \param address ESC Address to read from PDI
*  \retval Value at address
*
*/
extern uint8_t bsp_read_byte_isr(PRUICSS_Handle pruIcssHandle, uint16_t address);

/**
*  \brief  Read a 16-bit value at 'Address' from ESC process data memory: SM buffer
*          (3-buffer) mapped area
*
*  \param pruIcssHandle PRUSS Handle
*  \param address ESC address to read from PDI
*  \retval Value at address
*
*/
extern uint16_t bsp_read_word_isr(PRUICSS_Handle pruIcssHandle,
                                 uint16_t address);

/**
*  \brief   Read a 32-bit value at 'Address' from ESC process data memory: SM buffer
*           (3-buffer) mapped area
*
*  \param pruIcssHandle PRUSS Handle
*  \param address ESC address to read from PDI
*  \retval Value at address
*
*/
extern uint32_t bsp_read_dword_isr(PRUICSS_Handle pruIcssHandle,
                                  uint16_t address);

/**
*  \brief  Invoked after reading a register or mailbox buffer from PDI side \n
*
*           Handles following registers: \n
*           SM WD status \n
*           AL control   \n
*           SYNC0/1 status   \n
*           SM Activate registers    \n
*           Latch0/1 time Postive/Negative edge  \n
*           Also detects read mailbox start address access from PDI side \n
*
*  \param pruIcssHandle PRUSS Handle
*  \param address Start address of ESC read from PDI
*  \param length Length of ESC read from PDI
*
*/
extern void bsp_pdi_post_read_indication(PRUICSS_Handle pruIcssHandle,
                                        uint16_t address, uint16_t length);
/**
*  \brief  Invoked after writing a register or mailbox buffer from PDI side \n
*
*           Handles following registers: \n
*           AL status \n
*           SM PDI control registers \n
*           Also detects write mailbox start address access from PDI side \n
*
*  \param pruIcssHandle PRUSS Handle
*  \param address Start address of ESC write from PDI
*  \param length Length of ESC write from PDI
*  \param value Value to be written?
*
*/
extern void bsp_pdi_write_indication(PRUICSS_Handle pruIcssHandle,
                                    uint16_t address, uint16_t length,
                                    uint16_t value);
/**
*  \brief   Write a byte value at 'address' in ESC memory
*
*  \param pruIcssHandle PRUSS Handle
*  \param val 8-bit value to write
*  \param address ESC address read from PDI
*
*/
extern void bsp_write_byte(PRUICSS_Handle pruIcssHandle, uint8_t val,
                          uint16_t address);

/**
*  \brief  Write a 16-bit value at 'address' in ESC memory
*
*  \param pruIcssHandle PRUSS Handle
*  \param val 16-bit value to write
*  \param address ESC address read from PDI
*
*/
extern void bsp_write_word(PRUICSS_Handle pruIcssHandle, uint16_t val,
                          uint16_t address);

/**
*  \brief  Write a 32-bit value at 'address' in ESC memory
*
*  \param pruIcssHandle PRUSS Handle
*  \param val 32-bit value to write
*  \param address ESC address read from PDI
*
*/
extern void bsp_write_dword(PRUICSS_Handle pruIcssHandle, uint32_t val,
                           uint16_t address);

/**
*  \brief  Write 'len' bytes from pdata to 'address' in ESC memory
*
*  \param pruIcssHandle PRUSS Handle
*  \param pdata Pointer to byte array in application
*  \param address ESC address read from PDI
*  \param len Length of data
*
*/
extern void bsp_write(PRUICSS_Handle pruIcssHandle, uint8_t *pdata,
                     uint16_t address, uint16_t len);

/**
*  \brief  Read a 32-bit value from PRU-ICSS MDIO register at 'regoffset'
*
*  \param pruIcssHandle PRUSS Handle
*  \param regoffset PRU-ICSS MDIO register offset
*  \retval Value read
*
*/
extern uint32_t bsp_pruss_mdioreg_read(PRUICSS_Handle pruIcssHandle,
                                      uint32_t regoffset);

/**
*  \brief  Write a 32-bit value from PRU-ICSS MDIO register at 'regoffset'
*
*  \param pruIcssHandle PRUSS Handle
*  \param val 32-bit value to write to PRU-ICSS MDIO
*  \param regoffset PRU-ICSS MDIO register offset
*
*/
extern void bsp_pruss_mdioreg_write(PRUICSS_Handle pruIcssHandle, uint32_t val,
                                   uint32_t regoffset);

/**
*  \brief Read a 32-bit value from PRU-ICSS IEP register at 'regoffset'
*
*  \param pruIcssHandle PRUSS Handle
*  \param regoffset PRU-ICSS IEP register offset
*  \retval Read value
*
*/
extern uint32_t bsp_pruss_iepreg_read(PRUICSS_Handle pruIcssHandle,
                                     uint32_t regoffset);

/**
*  \brief  Write a 32-bit value from PRU-ICSS IEP register at 'regoffset'
*
*  \param pruIcssHandle PRUSS Handle
*  \param val 32-bit value to write to PRU-ICSS IEP
*  \param regoffset PRU-ICSS MDIO register offset
*
*/
extern void bsp_pruss_iepreg_write(PRUICSS_Handle pruIcssHandle, uint32_t val,
                                  uint32_t regoffset);

/**
*  \brief  Read a 16-bit value from PRU-ICSS IEP command interface
*
*  \param val 16-bit value to write
*  \param ptr Pointer to Host - PRU-ICSS command interface in PRU-ICSS data memory
*
*/
extern void bsp_pruss_cmd_intfc_write_word(uint16_t val, volatile uint16_t *ptr);

/**
*  \brief   Read a 16-bit value from PRU-ICSS IEP command interface
*
*  \param ptr PRU-ICSS command interface pointer
*  \retval 16-bit value read
*
*/
extern uint16_t bsp_pruss_cmd_intfc_read_word(volatile uint16_t *ptr);

/**
*  \brief  Checks if the PDI register [byte] has the requested access permission
*          and returns the result
*
*  \param address PDI Register address
*  \param access  Permission to be checked ( Read, Write, Read&Write )
*  \retval 1 if the register has the requested permission, 0 if otherwise
*
*/
extern uint8_t  bsp_get_pdi_access_perm(uint16_t address, uint8_t access);

/**
*  \brief  Checks if the PDI register [Two bytes] has the requested access permission
*          and returns the result
*
*  \param address PDI Register address
*  \param access  Permission to be checked ( Read, Write, Read&Write )
*  \retval 1 if the register has the requested permission, 0 if otherwise
*
*/
extern uint8_t  bsp_pdi_access_perm_word(uint16_t address, uint8_t access);

/**
*  \brief  Checks if the PDI register [Four bytes] has the requested access permission
*          and returns the result
*
*  \param address PDI Register address
*  \param access  Permission to be checked ( Read, Write, Read&Write )
*  \retval 1 if the register has the requested permission, 0 if otherwise
*
*/
extern uint8_t  bsp_pdi_access_perm_dword(uint16_t address, uint8_t access);

/**
*  \brief  Checks if all PDI registers starting from 'address' has the requested
*          access permission and returns the result
*
*  \param address PDI Register address
*  \param access  Permission to be checked ( Read, Write, Read&Write )
*  \param size Number of register bytes
*  \retval 1 if the register has the requested permission, 0 if otherwise
*
*/
extern uint8_t  bsp_pdi_access_perm_array(uint16_t address, uint8_t access,
        uint16_t size);

/**
*  \brief  Set the PDI register [byte] access permission to read only
*
*  \param perm_array  Pointer to the Permissions array
*  \param address PDI Register address
*
*/
extern void bsp_set_pdi_perm_read_only(uint16_t *perm_array, uint16_t address);

/**
*  \brief  Set the PDI register [byte] access permission to read and write
*
*  \param perm_array  Pointer to the Permissions array
*  \param address PDI Register address
*
*/
extern void bsp_set_pdi_perm_read_write(uint16_t *perm_array, uint16_t address);

/**
*  \brief  Checks if the PDI register [byte] has read only access permission and
*          returns the result
*
*  \param perm_array  Pointer to the Permissions array
*  \param address PDI Register address
*
*  \retval 1 if the register has read only permission, 0 if otherwise
*
*/
extern uint8_t bsp_is_pdi_perm_read_only(uint16_t *perm_array, uint16_t address);

/**
*  \brief  Returns the count of PDI read access failures
*
*  \retval PDI read access fail count
*
*/
extern uint32_t bsp_get_pdi_read_access_fail_cnt();

/**
*  \brief  Returns the count of PDI write access failures
*
*  \retval PDI write access fail count
*
*/
extern uint32_t bsp_get_pdi_write_access_fail_cnt();

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_PRU_MDIO
 @{ */

/**
*  \brief  Initializes PRU-ICSS MDIO for EtherCAT firmware to communicate with PHYs.
*          Must be called after powering on PRU-ICSS domain and before PRU firmware
*          is loaded and executed on both PRUs. This is called from \ref bsp_init.
*
*  \param pruIcssHandle PRUSS Handle
*  \param pmdio_params  MDIO configuration
*  \retval 0 on success, <0 on failure
*
*/
extern int16_t bsp_pruss_mdio_init(PRUICSS_Handle pruIcssHandle,
                                  t_mdio_params *pmdio_params);

/**
*  \brief  API to read PHY register via PRU-ICSS MDIO
*
*  \param pruIcssHandle PRUSS Handle
*  \param phyaddr Select the PHY to read using PHY address
*  \param regoffset  Register offset in PHY to read
*  \param regval  Pointer to the variable to hold the register value read
*  \retval  0 : Success -1 : MDIO access error
*
*/
extern int16_t bsp_pruss_mdio_phy_read(PRUICSS_Handle pruIcssHandle,
                                      uint8_t phyaddr, uint8_t regoffset, uint16_t *regval);

/**
*  \brief  API to write PHY register via PRU-ICSS MDIO
*
*  \param pruIcssHandle PRUSS Handle
*  \param phyaddr Select the PHY to read using PHY address
*  \param regoffset  Register offset in PHY to read
*  \param regval  Value to write to the PHY register
*  \retval  0 : Success -1 : MDIO access error
*
*/
extern int16_t bsp_pruss_mdio_phy_write(PRUICSS_Handle pruIcssHandle,
                                       uint8_t phyaddr, uint8_t regoffset, uint16_t regval);

/**
*  \brief  Get the link status for selected PHY, this API considers MII_link signal
*          polarity differences and recommended when TIESC_MDIO_RX_LINK_ENABLE is
*          enabled for enhanced link detection
*
*  \param pruIcssHandle PRUSS Handle
*  \param phyaddr Select the PHY to read using PHY address
*  \retval  0: Link Down Otherwise: Link Up
*
*/
extern uint32_t bsp_pruss_mdio_phy_link_state(PRUICSS_Handle pruIcssHandle,
        uint8_t phyaddr);
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_PRU_DIGIO
 @{ */

/**
*  \brief  Configure digio for sw controlled dataout mode
*
*  \param pruIcssHandle PRUSS Handle
*/
extern  void bsp_set_digio_sw_dataout_enable(PRUICSS_Handle pruIcssHandle);

/**
*  \brief  Set selected digital output pin
*
*  \param pruIcssHandle PRUSS Handle
*  \param num digital output selection (num =0 to 7)
*/
extern void bsp_set_digio_out(PRUICSS_Handle pruIcssHandle, uint8_t num);

/**
*  \brief  Clear selected digital output pin
*
*  \param pruIcssHandle PRUSS Handle
*  \param num digital output selection (num =0 to 7)
*/
extern void bsp_clear_digio_out(PRUICSS_Handle pruIcssHandle, uint8_t num);

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_SPINLOCK
 @{ */
/**
*  \brief   Initialize SOC spinlock, enable clocks and init spinlock instance 0 through 7 to unlocked state
*/
extern void bsp_hwspinlock_init(void);

/**
*  \brief   Acquire selected spinlock instance
*
*  \param num Spinlock instance(0: DC latch)
*  \retval 0: success 1: already locked
*/
extern uint32_t bsp_hwspinlock_lock(int num);

/**
*  \brief  Release selected spinlock instance
*
*  \param num Spinlock instance
*/
extern void bsp_hwspinlock_unlock(int num);

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_SYSTIME_PDI
 @{ */
#ifdef SYSTEM_TIME_PDI_CONTROLLED
/**
*  \brief  Enable latch single shot/continuous mode for Latch0 via PDI interface.
*          If bit is clear latch will be configured in continuous mode.
*
*  \param pruIcssHandle PRUSS Handle
*  \param val Bitmask to enable singleshot mode [Bit0 : posedge Bit1: negedge]
*/
extern void bsp_pdi_latch0_control(PRUICSS_Handle pruIcssHandle, uint8_t val);

/**
*  \brief  Enable latch single shot/continuous mode for Latch1 via PDI interface.
*          If bit is clear latch will be configured in continuous mode.
*
*  \param pruIcssHandle PRUSS Handle
*  \param val Bitmask to enable singleshot mode [Bit0 : posedge Bit1: negedge]
*/
extern void bsp_pdi_latch1_control(PRUICSS_Handle pruIcssHandle, uint8_t val);

/**
*  \brief  Write 32-bit system time to firmware to trigger drift compensation
*
*  \param pruIcssHandle PRUSS Handle
*  \param systime 32-bit system time value
*/
extern void bsp_pdi_write_system_time(PRUICSS_Handle pruIcssHandle,
                                     uint32_t systime);
/**
*  \brief Write 64-bit System Time Offset (0x920) register and indicate to the firmware
*
*  \param pruIcssHandle PRUSS Handle
*  \param systime 64-bit system time value
*/
extern void bsp_pdi_write_system_timeoffset(PRUICSS_Handle pruIcssHandle,
        unsigned long long systime);
/**
*  \brief Write 32-bit System Time Delay (0x928) register
*
*  \param pruIcssHandle PRUSS Handle
*  \param systime 32-bit system time delay value
*/
extern void bsp_pdi_write_systime_delay(PRUICSS_Handle pruIcssHandle,
                                       uint32_t systime);
/**
*  \brief Initialize the System Time and SpeedCounter Diff filters from PDI side
*
*  \param pruIcssHandle PRUSS Handle
*  \param speedcount_start 16-bit Speed Counter Start value at (0x930)
*  \param speedcount_filtdepth Filter depth 0-16 (0x935)
*  \param systime_filtdepth Filter depth 0-16 (0x934)
*/
extern void bsp_pdi_write_filterconfig(PRUICSS_Handle pruIcssHandle,
                                      uint16_t speedcount_start,
                                      uint8_t speedcount_filtdepth, uint8_t systime_filtdepth);
#endif
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_TIMER
 @{ */
/**
*  \brief Returns the time difference from last call of \ref bsp_clear_timer_register
*         to this \ref bsp_get_timer_register. It handles overflow.
*
*  \retval 32-bit time elapsed since last call to \ref bsp_clear_timer_register
*/
extern uint32_t bsp_get_timer_register(void);

/**
*  \brief Update the time when \ref bsp_clear_timer_register last invoked.
*         This is a wrapper API used by SSC.
*/
extern void bsp_clear_timer_register(void);
/**
*  \brief  Return EtherCAT time base for application use
*
*  \param systime_low stores system time low (32-bit LSW)
*  \param systime_high stores system time high (32-bit MSW)
*/
extern void bsp_get_local_sys_time(uint32_t *systime_low,
                                  uint32_t *systime_high);

/**
*  \brief  Return latch0 posedge timestamp for application use(nanosec resolution)
*
*  \param pruIcssHandle PRUSS handle
*  \param systime_low stores system time low (32-bit LSW)
*  \param systime_high stores system time high (32-bit MSW)
*/
extern void bsp_get_latch0_posedge_time(PRUICSS_Handle pruIcssHandle,
                                       uint32_t *systime_low, uint32_t *systime_high);

/**
*  \brief  Return latch0 negedge timestamp for application use(nanosec resolution)
*
*  \param pruIcssHandle PRUSS handle
*  \param systime_low stores system time low (32-bit LSW)
*  \param systime_high stores system time high (32-bit MSW)
*/
extern void bsp_get_latch0_negedge_time(PRUICSS_Handle pruIcssHandle,
                                       uint32_t *systime_low, uint32_t *systime_high);

/**
*  \brief  Return latch1 posedge timestamp for application use(nanosec resolution)
*
*  \param pruIcssHandle PRUSS handle
*  \param systime_low stores system time low (32-bit LSW)
*  \param systime_high stores system time high (32-bit MSW)
*/
extern void bsp_get_latch1_posedge_time(PRUICSS_Handle pruIcssHandle,
                                       uint32_t *systime_low, uint32_t *systime_high);

/**
*  \brief  Return latch0 negedge timestamp for application use(nanosec resolution)
*
*  \param pruIcssHandle PRUSS handle
*  \param systime_low stores system time low (32-bit LSW)
*  \param systime_high stores system time high (32-bit MSW)
*/
extern void bsp_get_latch1_negedge_time(PRUICSS_Handle pruIcssHandle,
                                       uint32_t *systime_low, uint32_t *systime_high);

/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_MUTEX
 @{ */

/**
*  \brief  Critical section enter API using semaphore/mutex/interrupt disable primitives from RTOS.
*  Implemented using \ref HwiP_disableInt API.
*/
extern void bsp_global_mutex_lock(void);

/**
*  \brief  Critical section leave API using semaphore/mutex/interrupt enable primitives from RTOS.
*  Implemented using \ref HwiP_enableInt API.
*/
extern void bsp_global_mutex_unlock(void);
/**
@}
*/

/** \addtogroup ETHERCAT_FWHAL_INTMGMNT
 @{ */
/**
*  \brief    SYNC0 IRQ handler
*/
void Sync0Isr(void *args);
/**
*  \brief    SYNC1 IRQ Handler
*/
void Sync1Isr(void *args);
/**
*  \brief    ECAT IRQ Handler
*/
void EcatIsr(void *args);

#ifndef SUPPORT_CMDACK_POLL_MODE
/**
*  \brief    ESC CMD Low ACK IRQ Handler
*/
void EscCmdLowAckIsr(void *args);
#endif

/**
@}
*/


/** \addtogroup ETHERCAT_FWHAL_PRU_FW
 @{ */
/**
*  \brief This function internally sets the location from which PRU firmwares
*         can be loaded.
*
*  \param frameProc Pointer to a buffer containing HRT PRU firmware (ecat_frame_handler_bin.h)
*  \param frameProcLen Size of the HRT PRU firmware
*  \param hostProc Pointer to a buffer containing SRT PRU firmware (ecat_host_interface_bin.h)
*  \param hostProcLen Size of the SRT PRU firmware
*/
extern void bsp_set_pru_firmware(uint32_t *frameProc, uint32_t frameProcLen,
                                uint32_t *hostProc, uint32_t hostProcLen);
/**
@}
*/

/** @} */

#ifdef __cplusplus
}
#endif

#endif/*TIESC_BSP_H_ */
