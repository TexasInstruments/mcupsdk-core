/*!
* \file pru.h
*
* \brief
* PRU Integration Interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-02-14
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

#if !(defined __PRU_H__)
#define __PRU_H__		1

#include <osal.h>
#if ((defined ECATSLAVE_SO) && (ECATSLAVE_SO==1) || ((defined ETHERNETIP_SO) && (ETHERNETIP_SO==1)) || ((defined PROFINETIO_SO) && (PROFINETIO_SO==1))) // defined if ECATSLV is compiled as a DLL
#if (defined PRUAPI_EXPORTS) // defined if we are building the ECATSLAVE DLL (instead of using it)
#define PRU_API OSAL_DLL_EXPORT
#else
#define PRU_API OSAL_DLL_IMPORT
#endif // ECSLVAPI_EXPORTS
#define PRU_LOC OSAL_DLL_LOCAL
#else // ECATSLAVE_SO is not defined: this means ECATSLAVE is a static lib.
#define PRU_API
#define PRU_LOC
#endif // ECATSLAVE_SO

/* PDK */
#if (defined OSAL_LINUX) || (defined OSAL_TIRTOS) || (defined OSAL_FREERTOS_JACINTO)
#define CSL_ICSSM_INTC_SECR0            CSL_ICSSINTC_SECR0
#define CSL_ICSSM_INTC_SECR1            CSL_ICSSINTC_SECR1
#define CSL_ICSSM_INTC_HIDISR           CSL_ICSSINTC_HIDISR
#define CSL_ICSSM_INTC_HIEISR           CSL_ICSSINTC_HIEISR
#define CSL_ICSSM_INTC_REVID            CSL_ICSSINTC_REVID
#else
#define CSL_ICSSM_INTC_SECR0            CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0
#define CSL_ICSSM_INTC_SECR1            CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG1
#define CSL_ICSSM_INTC_HIDISR           CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_HINT_ENABLE_CLR_INDEX_REG
#define CSL_ICSSM_INTC_HIEISR           CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_HINT_ENABLE_SET_INDEX_REG
#define CSL_ICSSM_INTC_REVID            CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_REVISION_REG

#define CSL_ICSSIEP_COUNT_REG0          CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG0
#define CSL_ICSSIEP_DIGIO_CTRL_REG      CSL_ICSS_G_PR1_IEP1_SLV_DIGIO_CTRL_REG
#define CSL_ICSSIEP_DIGIO_EXP_REG       CSL_ICSS_G_PR1_IEP1_SLV_DIGIO_EXP_REG
#define CSL_ICSSIEP_GLOBAL_CFG_REG      CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG
#define CSL_ICSSIEP_GLOBAL_STATUS_REG   CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_STATUS_REG
#endif

/* only defined in FreeRTOS */
#if !(defined MDIO_LINKSEL_MDIO_MODE)
#define MDIO_LINKSEL_MDIO_MODE  (0)
#endif

#if !(defined MDIO_LINKSEL_MLINK_MODE)
#define MDIO_LINKSEL_MLINK_MODE (1)
#endif

#define PDI_ISR_EDIO_NUM                7 //GPMC_CSN(2) -> pr1_edio_data_out7 for ICEv2.J4.Pin21

#define PRU_ESC_PERM_RW                 0x0
#define PRU_ESC_PERM_WRITE_ONLY         0x1
#define PRU_ESC_PERM_READ_ONLY          0x2

#define PRU_ESC_PERM_WRITE              TIESC_PERM_WRITE_ONLY
#define PRU_ESC_PERM_READ               TIESC_PERM_READ_ONLY

/* DMEM 0 */
#define SYNC_PERMISSION_UPDATE_ADDR_OFFSET      0x500
#define SYNC_PERMISSION_UPDATE_ECAT_OFFSET      0x518
#define SYNC_PERMISSION_UPDATE_PDI_OFFSET       0x522
#define SYNC_PERMISSION_UPDATE_ECAT_SIZE        0x10
#define SYNC_PERMISSION_UPDATE_PDI_SIZE         0x10

/* Use ESC system time instead of SYS/BIOS Timestamp_get32 for timing info */
#if !(defined USE_ECAT_TIMER)
 #define USE_ECAT_TIMER
#endif

/* No direct UART PRINT here */
#define DISABLE_UART_PRINT

/*! Size of the register properties array */
#define PRU_SIZE_OF_REG_PROPERTIES          4096

/*! <!-- Description: -->
 *
 *  \brief
 *  Write 32 Bit Register
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  addr_p      address to write to
 *  \param[in]  value_p     value.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <pru.h>
 *
 *  // the Call
 *  PRU_WR_REG32(addr, value);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup pru
 *
 * */
static inline void PRU_WR_REG32(uint32_t addr_p, uint32_t value_p)
{
    *(volatile uint32_t *) addr_p = value_p;
#ifndef MEM_BARRIER_DISABLE
    OSAL_ASSERT_DSB();
#endif
    return;
}

typedef enum PRU_eFW_type
{
  PRUFW_ethercat    = 0x1000,
  PRUFW_profinet    = 0x1001,
} PRU_eFW_type_t;

/**
 * @brief Struct for host to PRU-ICSS command interface
 */
typedef struct PRU_SM_SProcessData
{
    uint8_t     sm_buf_index;   /**< Sync manager buff index */
    uint8_t     lock_state;     /**< Lock state. Can have @ref LOCK_PD_BUF_AVAILABLE_FOR_HOST, @ref LOCK_PD_BUF_HOST_ACCESS_START, @ref LOCK_PD_BUF_HOST_ACCESS_FINISH      */
    uint16_t    addr;           /**< Address */
} PRU_SM_SProcessData_t;

/**
 * @brief Struct for host to PRU-ICSS command interface
 * Starts at PRU0 DMEM @ 0x4a300000
 */
typedef struct PRU_HOST_SInterface
{
    uint8_t                 reserved1[0x90];        /**< Reserved */
    uint32_t                system_time_low;        /**< System Time low */
    uint32_t                system_time_high;       /**< System Time high */
    uint8_t                 sm_config_ongoing;      /**< Sync manageer config */
    uint8_t                 reserved2[7];           /**< Reserved */
    uint16_t                cmdlow;                 /**< CMD low */
    uint16_t                cmdlow_ack;             /**< CMD low ack */
    uint16_t                param1low;              /**< PARAM1 low */
    uint16_t                param2low;              /**< PARAM2 low */
    uint16_t                resp1low;               /**< RESP1 low */
    uint16_t                resp2low;               /**< RESP2 low */
#ifndef SYSTEM_TIME_PDI_CONTROLLED
    uint8_t                 reserved3[212];         /**< Reserved */
#else
    uint8_t                 reserved3[24];          /**< Reserved */
    uint32_t                systime_offset_low;     /**< System time offset low */
    uint32_t                systime_offset_high;    /**< System time offset high */
    uint8_t                 reserved4[180];         /**< Reserved */
#endif
    PRU_SM_SProcessData_t   sm_processdata[6];      /**< Sync manager process data */
} PRU_HOST_SInterface_t;

/**
 * @brief Struct for register permission array
 */
typedef struct  PRU_REG_SProperties
{
    uint8_t                 reserved[1024];         /**< Reserved */
    uint8_t                 reg_properties[PRU_SIZE_OF_REG_PROPERTIES]; /**< Register properties */
} OSAL_STRUCT_PACKED PRU_REG_SProperties_t;

typedef void    (*PRU_measurement_t)        (void* pContext_p, uint32_t measureChannel_p, bool channelOn_p);

#if (defined __cplusplus)
extern "C" {
#endif

extern void         PRU_errWriteIncrement   (void);
extern void         PRU_errReadIncrement    (void);

extern void         PRU_prepare             (void);
extern void         PRU_init                (uint32_t                   logicPruSelect_p
                                            ,int32_t                    irqBaseOffset_p);
extern void         PRU_FB_startPhy         (void);
extern void         PRU_exit                (void);
extern void         PRU_FW_determine        (PRU_eFW_type_t             targetFirmware_p);
extern void         PRU_FW_load             (void);
extern PRU_API void PRU_FW_start            (void);
extern void         PRU_FW_stop             (void);
extern void         PRU_IRQ_enableAll       (void);
extern void         PRU_GMUTEX_lock         (void);
extern void         PRU_GMUTEX_unlock       (void);

extern int32_t      PRU_EVT_wait            (uint32_t                   event_p);
extern void         PRU_EVT_clear           (uint32_t                   event_p);

extern void         PRU_FW_sendLowCommand   (uint32_t                   command_p
                                            ,uint16_t                   param1_p
                                            ,uint16_t                   param2_p);
extern void         PRU_read                (uint8_t*                   pData_p
                                            ,uint16_t                   address_p
                                            ,uint16_t                   length_p);
extern uint8_t      PRU_readByte            (uint16_t                   address_p);
extern uint8_t      PRU_ISR_readByte        (uint16_t                   address_p);
extern uint16_t     PRU_readWord            (uint16_t                   address_p);
extern uint16_t     PRU_ISR_readWord        (uint16_t                   address_p);
extern uint32_t     PRU_readDword           (uint16_t                   address_p);
extern uint32_t     PRU_ISR_readDword       (uint16_t                   address_p);
extern void         PRU_MBX_readMem         (uint8_t*                   pData_p
                                            ,uint16_t                   address_p
                                            ,uint16_t                   length_p);
extern void         PRU_writeByte           (uint8_t                    value_p
                                            ,uint16_t                   address_p);
extern void         PRU_writeWord           (uint16_t                   value_p
                                            ,uint16_t                   address_p);
extern void         PRU_writeDword          (uint32_t                   value_p
                                            ,uint16_t                   address_p);
extern void         PRU_write               (uint8_t*                   pData_p
                                            ,uint16_t                   address_p
                                            ,uint16_t                   length_p);
extern void         PRU_MBX_writeMem        (uint8_t*                   pData_p
                                            ,uint16_t                   address_p
                                            ,uint16_t                   length_p);

extern void         PRU_LOCSYS_getTime      (uint32_t*                  pSystime_low_p
                                            ,uint32_t*                  pSystime_high_p);
extern void         PRU_LOCSYS_getTime_ovl  (uint32_t*                  pSystime_low_p
                                            ,bool*                      pOverflow_p);
extern void         PRU_PDI_writeIndication (uint16_t                   address_p
                                            ,uint16_t                   length_p
                                            ,uint16_t                   value_p);
extern void         PRU_PDI_writeMbxComplete(void);
extern uint16_t     PRU_PD_getAddress       (uint16_t                   address_p
                                            ,uint16_t                   length_p
                                            ,int16_t*                   pSm_index_p);
extern void         PRU_PD_accessComplete   (uint16_t                   address_p
                                            ,uint16_t                   length_p
                                            ,int16_t                    sm_index_p);

extern uint32_t     PRU_TIMER_getRegister   (void);
extern void         PRU_TIMER_clearRegister (void);

extern void         PRU_cbRegisterMsrmt     (void*                      pContext_p
                                            ,PRU_measurement_t          cbFunc_p);

extern bool         PRU_PHY_configure       (uint8_t                    phyIdx_p
                                            ,uint8_t                    phyAddr_p
                                            ,bool                       invertLinkPolarity_p
                                            ,bool                       useRxLink_p);
extern uint32_t     PRU_PHY_readRegEx       (uint8_t                    phyId_p
                                            ,uint32_t                   regNum_p
                                            ,uint16_t*                  pData_p);
extern void         PRU_PHY_writeRegEx      (uint8_t                    phyId_p
                                            ,uint32_t                   regNum_p
                                            ,uint16_t                   writeValue_p);
extern void         PRU_PHY_reset           (void);
extern void         PRU_PHY_enableMagnetics (void);
extern void         PRU_PHY_disableMagnetics(void);

extern bool         PRU_PHY_getPhyPowerMode (uint8_t                    phyId_p);
extern void         PRU_PHY_setPhyPowerMode (uint8_t                    phyId_p
                                            ,bool                       powerDown_p);
extern void         PRU_PHY_setPhyLinkConfig(uint8_t                    phyId_p,
                                             bool                       autoNeg_p,
                                             uint16_t                   linkSpeed_p,
                                             bool                       fullDuplex_p,
                                             uint32_t*                  pResult_p);

#if (defined DEBUGHELPERS) && (DEBUGHELPERS > 0)
extern void         PRU_dumpPortState       (void);
#endif

#if (defined __cplusplus)
}
#endif

#endif /* __PRU_H__ */
