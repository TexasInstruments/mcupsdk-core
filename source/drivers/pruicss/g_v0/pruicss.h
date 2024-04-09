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

/**
 *  \defgroup DRV_PRUICSS_MODULE APIs for PRUICSS
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the PRUICSS module. Programmable
 *  Real-Time Unit and Industrial Communication Subsystem(PRUICSS) allows
 *  implementation of various high-performance industrial control algorithms and
 *  industrial interface standards.
 *
 *  @{
 */

#ifndef PRUICSS_H_
#define PRUICSS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_icss.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor PRUICSS_Cores
 *  \name PRUICSS Cores
 *
 *  Different cores present in PRUICSS
 *
 *  @{
 */
#define PRUICSS_PRU0                                (0U)
#define PRUICSS_PRU1                                (1U)
#define PRUICSS_RTU_PRU0                            (2U)
#define PRUICSS_RTU_PRU1                            (3U)
#define PRUICSS_TX_PRU0                             (4U)
#define PRUICSS_TX_PRU1                             (5U)
/** @} */

/**
 * \brief Number of different cores present in PRUICSS
 */
#define PRUICSS_NUM_CORES                           (6U)

/**
 *  \anchor PRUICSS_MemoryOffsets
 *  \name PRUICSS Memory Offsets
 *
 *  Offsets for different memory inside each PRUICSS instance.
 *
 *  @{
 */
/** \brief Data RAM0/RAM1. n = 0 for RAM0, n = 1 for RAM1 */
#define PRUICSS_DATARAM(n)                          (0x00000U + ((n) * 0x02000U))
/** \brief IRAM for RTU_PRU0/RTU_PRU1. n = 0 for RTU_PRU0,n = 1 for RTU_PRU1 */
#define PRUICSS_IRAM_RTU_PRU(n)                     (0x04000U + ((n) * 0x02000U))
/** \brief IRAM for TX_PRU0/TX_PRU1. n = 0 for TX_PRU0,n = 1 for TX_PRU1 */
#define PRUICSS_IRAM_TX_PRU(n)                      (0x0A000U + ((n) * 0x02000U))
/** \brief Data RAM2 (shared) */
#define PRUICSS_SHARED_RAM                          (0x10000U)
/** \brief IRAM for PRU0/PRU1. n = 0 for PRU0,n = 1 for PRU1 */
#define PRUICSS_IRAM_PRU(n)                         (0x34000U + ((n) * 0x04000U))
/** @} */

/**
 * \brief Number of Host Interrupts supported in PRUICSS Local INTC
 */
#define PRUICSS_INTC_NUM_HOST_INTERRUPTS            (20U)

/**
 * \brief  Number of Output Interrupt channels supported in PRUICSS Local INTC
 */
#define PRUICSS_INTC_NUM_CHANNELS                   (20U)

/**
 * \brief  Number of Events that can be captured in PRUICSS Local INTC
 */
#define PRUICSS_INTC_NUM_SYS_EVTS                   (160U)


/**
 *  \anchor PRUICSS_ConstantTableEntry
 *  \name Index for PRU Constant Table Entries
 *
 *  Index to be used while making calls to #PRUICSS_setConstantTblEntry()
 *
 *  @{
 */
#define PRUICSS_CONST_TBL_ENTRY_C24                 (0U)
#define PRUICSS_CONST_TBL_ENTRY_C25                 (1U)
#define PRUICSS_CONST_TBL_ENTRY_C26                 (2U)
#define PRUICSS_CONST_TBL_ENTRY_C27                 (3U)
#define PRUICSS_CONST_TBL_ENTRY_C28                 (4U)
#define PRUICSS_CONST_TBL_ENTRY_C29                 (5U)
#define PRUICSS_CONST_TBL_ENTRY_C30                 (6U)
#define PRUICSS_CONST_TBL_ENTRY_C31                 (7U)
#define PRUICSS_NUM_CONST_TBL_ENTRY                 (8U)
/** @} */

/**
 *  \anchor PRUICSS_GpMuxSelectModes
 *  \name GP MUX Select Modes
 *
 *  @{
 */
#define PRUICSS_GP_MUX_SEL_MODE_GP                  (0U)
#define PRUICSS_GP_MUX_SEL_MODE_ENDAT               (1U)
#define PRUICSS_GP_MUX_SEL_MODE_MII                 (2U)
#define PRUICSS_GP_MUX_SEL_MODE_SD                  (3U)
/** @} */

/**
 * \brief Number of different GP MUX Selections modes
 */
#define PRUICSS_NUM_GP_MUX_SEL_MODES                (4U)

/**
 *  \anchor PRUICSS_GpiModes
 *  \name GPI Modes
 *
 *  @{
 */
/** \brief Direct connect of PRU[0/1]_r31_status[29:0] */
#define PRUICSS_GPI_MODE_DIRECT_R31_STATUS          (0U)
/** \brief Parallel 16-bit capture mode. PRU[0/1]_r31_status [15:0] is
            captured using PRU[0/1]_r31_status [16] */
#define PRUICSS_GPI_MODE_PARALLEL_16_BIT_CAPTURE    (1U)
/** \brief 29-bit shift. PRU[0/1]_r31_status[0] -> r31_status[0] ->
           r31_status[1] -> r_status[28] -> bit bucket */
#define PRUICSS_GPI_MODE_29_BIT_SHIFT               (2U)
/** \brief mii_rt mode. Selects mii_rt PRU[0/1] bus */
#define PRUICSS_GPI_MODE_MII_RT                     (3U)
/** @} */

/**
 * \brief Number of different GPI modes
 */
#define PRUICSS_NUM_GPI_MODES                       (4U)

/**
 *  \anchor PRUICSS_SaMuxModes
 *  \name PRUICSS SA MUX Modes
 *
 *  @{
 */
/** \brief Default/Legacy Mode */
#define PRUICSS_SA_MUX_MODE_DEFAULT                 (0U)
/** \brief Few SD and EnDAT pins get remapped to enable different usecase */
#define PRUICSS_SA_MUX_MODE_SD_ENDAT                (1U)
/** @} */

/**
 * \brief Number of different PRUICSS SA MUX Modes
 */
#define PRUICSS_NUM_SA_MUX_MODES                    (2U)

/**
 *  \anchor PRUICSS_IcssCfgMiiModes
 *  \name ICSS_G_CFG MII Modes
 *
 *  @{
 */
/** \brief MII Mode */
#define PRUICSS_ICSS_CFG_MII_MODE_MII               (0U)
/** \brief RGMII Mode */
#define PRUICSS_ICSS_CFG_MII_MODE_RGMII             (1U)
/** \brief SGMII mode. It is supported only for PRU_ICSSG instance */
#define PRUICSS_ICSS_CFG_MII_MODE_SGMII             (2U)
/** @} */

/**
 * \brief Number of different ICSS_G_CFG MII Modes
 */
#define PRUICSS_NUM_ICSS_CFG_MII_MODES              (3U)

/**
 *  \anchor PRUICSS_TxFifos
 *  \name PRUICSS TX FIFO Modes
 *
 *  @{
 */
#define PRUICSS_TX_L1_FIFO                          (0U)
#define PRUICSS_TX_L2_FIFO                          (1U)

/** @} */

typedef void (*PRUICSS_IrqHandler)(void* buf);

/**
 * \brief  A handle that is returned from a #PRUICSS_open() call. This handle
 *         is required for calling other APIs.
 */
typedef struct PRUICSS_Config_s         *PRUICSS_Handle;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/*!
 *  \brief Data structure used for system event to channel mapping in PRUICSS
 *
 */
typedef struct PRUICSS_SysevtToChannelMap_s
{
    uint8_t sysevt;
    /**< System event number */
    uint8_t channel;
    /**< Channel number */
    uint8_t polarity;
    /**< Interrupt polarity [ 0 = active low , 1 = active high ] */
    uint8_t type;
    /**< Interrupt type [ 0 = level or pulse interrupt , 1 = edge interrupt ] */
} PRUICSS_SysevtToChannelMap;

/*!
 *  \brief Data structure used for channel to host map in PRUICSS
 *
 */
typedef struct PRUICSS_ChannelToHostMap_s
{
    int16_t channel;
    /**< Channel number */
    int16_t host;
    /**< Host number */
} PRUICSS_ChannelToHostMap;

/*!
 *  \brief PRUICSS Interrupt controller initialisation data structure
 *
 */
typedef struct PRUICSS_IntcInitData_s
{
    uint8_t                     sysevtsEnabled[PRUICSS_INTC_NUM_SYS_EVTS];
    /**< Enabled SYSEVTs - Range:0..159
     *   {-1} indicates end of list */
    PRUICSS_SysevtToChannelMap  sysevtToChannelMap[PRUICSS_INTC_NUM_SYS_EVTS];
    /**< SysEvt to Channel map. SYSEVTs - Range:0..159, Channels - Range:0..19
     *   {-1, -1} indicates end of list */
    PRUICSS_ChannelToHostMap    channelToHostMap[PRUICSS_INTC_NUM_CHANNELS];
    /**< Channel to Host map. Channels - Range:0..19, HOSTs - Range:0..19
     *   {-1, -1} indicates end of list */
    uint32_t                    hostEnableBitmask;
    /**< 20-bit mask to enable hosts from Host0-Host19
     * { Host0..1   : PRU0/1,
     *   Host2..9   : PRUEVT_OUT0..7,
     *   Host10..11 : RTU_PRU0/1,
     *   Host12..19 : Task Managers }
     *  */
} PRUICSS_IntcInitData;

/*!
 *  \brief PRUICSS IRQ handler mapping structure
 *
 */
typedef struct PRUICSS_IrqFunMap_s
{
    PRUICSS_IrqHandler  irqHandler;
    /**< Interrupt handler*/
    HwiP_Object         hwiObj;
    /**< HW Interrupt object */
    uint8_t             waitEnable;
    /**< Flag to check if a binary semaphore is needed to wait on */
    SemaphoreP_Object   semObj;
    /**< Binary semaphore to wait on */
    PRUICSS_Handle      pruicssHandle;
} PRUICSS_IrqFunMap;

/*!
 *  \brief      PRUICSS Object
 *
 *  PRUICSS driver specific data object
 *
 *  The application must not access any member variables of this structure.
 */
typedef struct PRUICSS_Object_s
{
    uint32_t            pruicssVersion;
    /**< PRUICSS Version obtained from ICSSG_PID_REG register*/
    PRUICSS_IrqFunMap   pruEvntOutFnMapArray[PRUICSS_INTC_NUM_HOST_INTERRUPTS];
    /* IRQ Handler mapping structure for all host interrupts*/
} PRUICSS_Object;

/*!
 *  \brief      PRUICSS Hardware Attributes
 *
 *  PRUICSS driver specific hardware attributes structure. It contains PRUICSS
 *  base address of peripherals for each PRUICSS instance.
 *
 *  The application must not access any member variables of this structure.
 */
typedef struct PRUICSS_HWAttrs
{
    uint32_t    instance;
    uintptr_t   baseAddr;
    uintptr_t   pru0CtrlRegBase;
    uintptr_t   pru1CtrlRegBase;
    uintptr_t   intcRegBase;
    uintptr_t   cfgRegBase;
    uintptr_t   uartRegBase;
    uintptr_t   iep0RegBase;
    uintptr_t   iep1RegBase;
    uintptr_t   ecapRegBase;
    uintptr_t   miiRtCfgRegBase;
    uintptr_t   miiGRtCfgRegBase;
    uintptr_t   miiMdioRegBase;
    uintptr_t   pru0DramBase;
    uintptr_t   pru1DramBase;
    uintptr_t   pru0IramBase;
    uintptr_t   pru1IramBase;
    uintptr_t   sharedDramBase;
    uintptr_t   rtu0IramBase;
    uintptr_t   rtu1IramBase;
    uintptr_t   rtu0CtrlRegBase;
    uintptr_t   rtu1CtrlRegBase;
    uintptr_t   txPru0CtrlRegBase;
    uintptr_t   txPru1CtrlRegBase;
    uintptr_t   txPru0IramBase;
    uintptr_t   txPru1IramBase;
    uint32_t    pru0DramSize;
    uint32_t    pru1DramSize;
    uint32_t    pru0IramSize;
    uint32_t    pru1IramSize;
    uint32_t    sharedDramSize;
    uint32_t    rtu0IramSize;
    uint32_t    rtu1IramSize;
    uint32_t    txPru0IramSize;
    uint32_t    txPru1IramSize;
} PRUICSS_HwAttrs;

/*!
 *  \brief PRUICSS Global configuration structure
 *
 */
typedef struct PRUICSS_Config_s
{
    PRUICSS_Object    *object;
    /**< Pointer to a driver specific data object */
    PRUICSS_HwAttrs   *hwAttrs;
    /**< Pointer to a driver specific hardware attributes structure */
} PRUICSS_Config;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function initializes the PRUICSS module
 *
 * \return  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRUICSS_init(void);

/**
 *  \brief  This function de-initializes the PRUICSS module
 *
 * \return  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRUICSS_deinit(void);

/**
 * \brief   This function creates the handle for a PRUICSS instance
 *
 * \param   index  Index of config to use in the *gPruIcssConfig* array
 *
 * \return  A #PRUICSS_Handle on success or a NULL on an error
 */
PRUICSS_Handle PRUICSS_open(uint32_t index);

/**
 * \brief   This function deletes the handle for a PRUICSS instance
 *
 * \param   handle  #PRUICSS_Handle returned from #PRUICSS_open()
 *
 * \return  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRUICSS_close(PRUICSS_Handle handle);

/**
 * \brief  This function does Interrupt-Channel-host mapping
 *
 * \param   handle              #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   intcInitData        Pointer to structure of type
 *                              #PRUICSS_IntcInitData containing mapping
 *                              information
 *
 * \return  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRUICSS_intcInit(PRUICSS_Handle              handle,
                         const PRUICSS_IntcInitData  *intcInitData);

/**
 * \brief  This function registers an Interrupt Handler for an event
 *
 * \param   handle          #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruEvtoutNum    The ARM side event number
 * \param   intrNum         Interrupt number of R5F
 * \param   eventNum        Event number of PRUICSS
 * \param   waitEnable      Flag specifying whether application can wait on this
 *                          interrupt using #PRUICSS_waitEvent() function.
 *                          1 for enable, 0 for disable
 * \param   irqHandler      Pointer to a function which will be called on
 *                          interrupt
 *
 * \return  #SystemP_SUCCESS in case of successful registration, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_registerIrqHandler(PRUICSS_Handle           handle,
                                   uint32_t                 pruEvtoutNum,
                                   int32_t                  intrNum,
                                   int32_t                  eventNum,
                                   uint8_t                  waitEnable,
                                   PRUICSS_IrqHandler       irqHandler);

/**
 * \brief   This function resets the PRU
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum      PRU core number from \ref PRUICSS_Cores
 *
 * \return  #SystemP_SUCCESS in case of successful reset, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_resetCore(PRUICSS_Handle handle, uint8_t pruNum);

/**
 * \brief   This function disables the PRU
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum      PRU core number from \ref PRUICSS_Cores
 *
 * \return  #SystemP_SUCCESS in case of successful disable, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_disableCore(PRUICSS_Handle handle, uint8_t pruNum);

/**
 * \brief   This function enables the PRU
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum      PRU core number from \ref PRUICSS_Cores
 *
 * \return  #SystemP_SUCCESS in case of successful enable, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_enableCore(PRUICSS_Handle handle, uint8_t pruNum);

/**
 *
 *  \brief   This function initializes the PRU memory to zero
 *
 * \param    handle         #PRUICSS_Handle returned from #PRUICSS_open()
 * \param    pruicssMem     PRUICSS Memory macro from \ref PRUICSS_MemoryOffsets
 *
 * \return  Word length written or 0 on error
 *
 */
uint32_t PRUICSS_initMemory(PRUICSS_Handle handle, uint32_t pruicssMem);

/**
 *
 *  \brief   This function writes the given data to PRU memory
 *
 * \param    handle         #PRUICSS_Handle returned from #PRUICSS_open()
 * \param    pruicssMem     PRUICSS Memory macro from \ref PRUICSS_MemoryOffsets
 * \param    wordoffset     Offset at which the write will happen
 * \param    source_mem     Source memory[ Array of uint32_t integers ]
 * \param    bytelength     Total number of bytes to be writen
 *
 * \return  Word length written or 0 on error
 *
 */
uint32_t PRUICSS_writeMemory(PRUICSS_Handle   handle,
                             uint32_t         pruicssMem,
                             uint32_t         wordoffset,
                             const uint32_t   *source_mem,
                             uint32_t         bytelength);

/**
 *
 *  \brief   This function reads from PRU memory and stores in block of memory
 *
 * \param     handle        #PRUICSS_Handle returned from #PRUICSS_open()
 * \param     pruicssMem    PRUICSS Memory macro from \ref PRUICSS_MemoryOffsets
 * \param     wordoffset    Offset at which the read will happen
 * \param     dest_mem      Destination memory[ Array of uint32_t integers ]
 * \param     bytelength    Total number of bytes to be read
 *
 * \return  Word length read or 0 on error
 *
 */
uint32_t PRUICSS_readMemory(PRUICSS_Handle    handle,
                            uint32_t          pruicssMem,
                            uint32_t          wordoffset,
                            uint32_t          *dest_mem,
                            uint32_t          bytelength);

/**
 * \brief  This function generates an INTC event. It should be called only after
 *         successful execution of #PRUICSS_intcInit().
 *
 * \param   handle     #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   eventnum   The INTC Event number
 *
 * \return  #SystemP_SUCCESS in case of successful transition, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_sendEvent(PRUICSS_Handle handle, uint32_t eventnum);

/**
 * \brief  This function waits for a system event to happen. It should be called
 *         only after successful execution of #PRUICSS_intcInit().
 *
 * \param   handle              #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruEvtoutNum        The AINTC Event number
 *
 * \return  #SystemP_SUCCESS in case of successful transition, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_waitEvent(PRUICSS_Handle handle, uint32_t pruEvtoutNum);

/**
 * \brief   This function clears an INTC event. It should be called only after
 *          successful execution of #PRUICSS_intcInit().
 *
 * \param   handle     #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   eventnum   The INTC Event number
 *
 * \return  #SystemP_SUCCESS in case of successful transition, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_clearEvent(PRUICSS_Handle handle, uint32_t eventnum);

/**
 * \brief  This function generates an INTC event, waits for AINTC event and
 *         clears an INTC event
 *
 * \param   handle         #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   sendEventNum   Event number
 * \param   pruEvtoutNum   PRU Event number
 * \param   ackEventNum    Acknowledgement event number
 *
 * \return  #SystemP_SUCCESS in case of successful transition, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_sendWaitClearEvent(PRUICSS_Handle    handle,
                                   uint32_t          sendEventNum,
                                   uint32_t          pruEvtoutNum,
                                   uint32_t          ackEventNum);

/**
 * \brief   Get PRUICSS version number from ICSSCFG_REVID register
 *
 * \param   handle     #PRUICSS_Handle returned from #PRUICSS_open()
 *
 * \return  PRU ICSS version
 */
uint32_t PRUICSS_getVersion(PRUICSS_Handle handle);

/**
 * \brief   This API updates the constant table for specified constant table
 *          entry which have write permissions
 *
 * \param   handle              #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum              PRU core number from \ref PRUICSS_Cores
 * \param   constantTblEntry    contant table entry being updated
 * \param   constantTblVal      contant table value to be written
 *
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setConstantTblEntry(PRUICSS_Handle  handle,
                                    uint8_t         pruNum,
                                    int32_t         constantTblEntry,
                                    uint32_t        constantTblVal);

/**
 * \brief  This API configures the source of the IEP clock to be either
 *          IEP CLK as the source or ICSSGn_CORE_CLK.
 *
 * \param   handle  #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   source  source of the IEP clock (0 for IEP CLK, 1 for ICSSGn_CORE_CLK)
 *
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setIepClkSrc(PRUICSS_Handle handle, uint32_t source);

/**
 * \brief  This API sets the GP Mux Select mode for a specific PRU
 *
 * \param   handle  #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum  PRUICSS_PRU0 or PRUICSS_PRU1
 * \param   mode    Mode from \ref PRUICSS_GpMuxSelectModes
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setGpMuxSelect(PRUICSS_Handle handle, uint8_t pruNum, uint8_t mode);

/**
 * \brief  This API sets the GPI mode for a specific PRU
 *
 * \param   handle  #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum  PRUICSS_PRU0 or PRUICSS_PRU1
 * \param   mode    Mode from \ref PRUICSS_GpiModes
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setGpiMode(PRUICSS_Handle handle, uint8_t pruNum, uint8_t mode);

/**
 * \brief  This API sets the G_MUX_EN mux field in ICSSG_SA_MX_REG register
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   mode        Mode from \ref PRUICSS_SaMuxModes
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setSaMuxMode(PRUICSS_Handle handle, uint8_t mode);

/**
 * \brief   This function enables/disables the PRU Cycle Counter for a core
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruNum      PRU core number from \ref PRUICSS_Cores
 * \param   enable      0 for disable, 1 for enable
 *
 * \return  #SystemP_SUCCESS in case of successful enable, #SystemP_FAILURE
 *          otherwise
 */
int32_t PRUICSS_configureCycleCounter(PRUICSS_Handle handle, uint8_t pruNum, uint8_t enable);

/**
 * \brief  This API enables/disables the counter in IEP module
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   enable      0 for disabling the counter, 1 for enabling the counter
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_controlIepCounter(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t enable);

/**
 * \brief  This API sets the default increment value for counter in IEP module.
 *         This value is used when compensation is not active.
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   value       Increment value. Maximum value allowed is 15
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setIepCounterIncrementValue(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t value);

/**
 * \brief  This API reads the state of efuse bits from ICSSG_HWDIS_REG register
 *
 * \param   handle  #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   data    data read from the register
 *
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_readEfuse(PRUICSS_Handle handle, uint8_t *data);

/**
 * \brief  This API sets the MII mode in MII_G_RT_ICSS_G_CFG register
 *
 * \param   handle      #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   miiInstance 0 for MII0_MODE, 1 for MII1_MODE
 * \param   mode        Mode from \ref PRUICSS_IcssCfgMiiModes
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setIcssCfgMiiMode(PRUICSS_Handle handle, uint8_t miiInstance, uint8_t mode);

/**
 * \brief  This API configures the TX FIFO to be used
 *
 * \param   handle          #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   fifoInstance    FIFO from \ref PRUICSS_TxFifos
 * \param   enable          0 for disable, 1 for enable
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_setIcssCfgTxFifo(PRUICSS_Handle handle, uint8_t fifoInstance, uint8_t enable);

/**
 * \brief Return PRUICSS attributes
 *
 * \param instance          PRUICSS instance
 *
 * \return \ref PRUICSS_HwAttrs, else NULL if instance is invalid
 */
const PRUICSS_HwAttrs *PRUICSS_getAttrs(uint32_t instance);

/**
 * \brief  This API disables pruIcssCore, loads IMEM, resets pruIcssCore, enables pruIcssCore
 *
 * \param   handle          #PRUICSS_Handle returned from #PRUICSS_open()
 * \param   pruIcssCore     pruIcssCore from \ref PRUICSS_Cores
 * \param   pruFirmware     firmware array
 * \param   byteLength      size of firmware array
 * \return  #SystemP_SUCCESS on success, #SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_loadFirmware(PRUICSS_Handle handle, uint8_t pruIcssCore, const uint32_t pruFirmware[], uint32_t byteLength);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef PRUICSS_H_ */
