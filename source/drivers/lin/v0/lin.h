/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \defgroup DRV_LIN_MODULE APIs for LIN
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the LIN.
 *
 *  @{
 */

/**
 *  \file v0/lin.h
 *
 *  \brief This file contains the prototype of LIN driver APIs
 */

#ifndef LIN_V0_H_
#define LIN_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_lin.h>
#include <drivers/hw_include/hw_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** \brief  LIN IO DFT Key which when written in IODFTENA
 *          enables the User and Previledge mode Writes */
#define LIN_IO_DFT_KEY                      (0xAU)

/** \brief  LIN/SCI Wakeup signal is sent by sending an
 *          byte with value 0xF0 */
#define LIN_WAKEUP_KEY                      (0xF0U)

/**
 *  \anchor LIN_ParityIDMasks
 *  \name   Lin Parity ID Masks
 *
 *  Note: Parity ID Masks for LIN
 *
 *  @{
 */
#define LIN_ID0                             (0x1U)
#define LIN_ID1                             (0x2U)
#define LIN_ID2                             (0x4U)
#define LIN_ID3                             (0x8U)
#define LIN_ID4                             (0x10U)
#define LIN_ID5                             (0x20U)
/** @} */

/**
 *  \anchor LIN_IntFlags
 *  \name   Lin Interrupt Flag Parameters
 *
 *  Note: Definition for the intFlags parameter of LIN_enableInterrupt(),
 *  LIN_disableInterrupt(), LIN_clearInterruptStatus(),
 *  LIN_setInterruptLevel0() and LIN_setInterruptLevel1().
 *
 *  @{
 */
#define LIN_INT_WAKEUP                      (0x00000002U) /* Wakeup */
#define LIN_INT_TO                          (0x00000010U) /* Time out */
#define LIN_INT_TOAWUS                      (0x00000040U) /* Time out after wakeup signal */
#define LIN_INT_TOA3WUS                     (0x00000080U) /* Time out after 3 wakeup signals */
#define LIN_INT_TX                          (0x00000100U) /* Transmit buffer ready */
#define LIN_INT_RX                          (0x00000200U) /* Receive buffer ready */
#define LIN_INT_ID                          (0x00002000U) /* Received matching identifier */
#define LIN_INT_PE                          (0x01000000U) /* Parity error */
#define LIN_INT_OE                          (0x02000000U) /* Overrun error */
#define LIN_INT_FE                          (0x04000000U) /* Framing error */
#define LIN_INT_NRE                         (0x08000000U) /* No response error */
#define LIN_INT_ISFE                        (0x10000000U) /* Inconsistent sync field error */
#define LIN_INT_CE                          (0x20000000U) /* Checksum error */
#define LIN_INT_PBE                         (0x40000000U) /* Physical bus error */
#define LIN_INT_BE                          (0x80000000U) /* Bit error */
#define LIN_INT_ALL                         (0xFF0023D2U) /* All interrupts */
/** @} */

/**
 *  \anchor LIN_GetIntFlags
 *  \name   Lin Get Interrupt Flag
 *
 *  Note: Definitions for the return value of LIN_getInterruptStatus().
 *
 *  @{
 */
#define LIN_FLAG_BREAK                      (CSL_LIN_SCIFLR_BRKDT_MASK)
#define LIN_FLAG_WAKEUP                     (CSL_LIN_SCIFLR_WAKEUP_MASK)
#define LIN_FLAG_IDLE                       (CSL_LIN_SCIFLR_IDLE_MASK)
#define LIN_FLAG_BUSY                       (CSL_LIN_SCIFLR_BUSY_MASK)
#define LIN_FLAG_TO                         (CSL_LIN_SCIFLR_TIMEOUT_MASK)
#define LIN_FLAG_TOAWUS                     (CSL_LIN_SCIFLR_TOAWUS_MASK)
#define LIN_FLAG_TOA3WUS                    (CSL_LIN_SCIFLR_TOA3WUS_MASK)
#define LIN_FLAG_TXRDY                      (CSL_LIN_SCIFLR_TXRDY_MASK)
#define LIN_FLAG_RXRDY                      (CSL_LIN_SCIFLR_RXRDY_MASK)
#define LIN_FLAG_TXWAKE                     (CSL_LIN_SCIFLR_TXWAKE_MASK)
#define LIN_FLAG_TXEMPTY                    (CSL_LIN_SCIFLR_TXEMPTY_MASK)
#define LIN_FLAG_RXWAKE                     (CSL_LIN_SCIFLR_RXWAKE_MASK)
#define LIN_FLAG_TXID                       (CSL_LIN_SCIFLR_IDTXFLAG_MASK)
#define LIN_FLAG_RXID                       (CSL_LIN_SCIFLR_IDRXFLAG_MASK)
#define LIN_FLAG_PE                         (CSL_LIN_SCIFLR_PE_MASK)
#define LIN_FLAG_OE                         (CSL_LIN_SCIFLR_OE_MASK)
#define LIN_FLAG_FE                         (CSL_LIN_SCIFLR_FE_MASK)
#define LIN_FLAG_NRE                        (CSL_LIN_SCIFLR_NRE_MASK)
#define LIN_FLAG_ISFE                       (CSL_LIN_SCIFLR_ISFE_MASK)
#define LIN_FLAG_CE                         (CSL_LIN_SCIFLR_CE_MASK)
#define LIN_FLAG_PBE                        (CSL_LIN_SCIFLR_PBE_MASK)
#define LIN_FLAG_BE                         (CSL_LIN_SCIFLR_BE_MASK)
/** @} */

/**
 *  \anchor LIN_IntOffset
 *  \name   Lin Interrupt Offset
 *
 *  Note: Definitions for the return value of LIN_getInterruptLine0Offset()
 *        and LIN_getInterruptLine1Offset().
 *
 *  @{
 */
#define LIN_VECT_NONE                       (0x00)
#define LIN_VECT_WAKEUP                     (0x01)
#define LIN_VECT_ISFE                       (0x02)
#define LIN_VECT_PE                         (0x03)
#define LIN_VECT_ID                         (0x04)
#define LIN_VECT_PBE                        (0x05)
#define LIN_VECT_FE                         (0x06)
#define LIN_VECT_BREAK                      (0x07)
#define LIN_VECT_CE                         (0x08)
#define LIN_VECT_OE                         (0x09)
#define LIN_VECT_BE                         (0x0A)
#define LIN_VECT_RX                         (0x0B)
#define LIN_VECT_TX                         (0x0C)
#define LIN_VECT_NRE                        (0x0D)
#define LIN_VECT_TOAWUS                     (0x0E)
#define LIN_VECT_TOA3WUS                    (0x0F)
#define LIN_VECT_TO                         (0x10)
/** @} */

/**
 *  \anchor LIN_ErrorDef
 *  \name   Lin Error Defination
 *
 *  Note: Definitions for the LIN errors parameter of
 *        LIN_enableModuleErrors() and LIN_disableModuleErrors().
 *
 *  @{
 */
#define LIN_ALL_ERRORS                      (0xF0000000U)
#define LIN_BIT_ERROR                       (0x80000000U)
#define LIN_BUS_ERROR                       (0x40000000U)
#define LIN_CHECKSUM_ERROR                  (0x20000000U)
#define LIN_ISF_ERROR                       (0x10000000U)
/** @} */

/**
 *  \anchor LIN_SCIErrorDef
 *  \name   Lin Serial Communication Interface Error Defination
 *
 *  Note: Definitions for the SCI errors parameter of
 *        LIN_enableSCIModuleErrors() and LIN_disableSCIModuleErrors().
 *
 *  @{
 */
#define LIN_SCI_ALL_ERRORS                  (0x7000000U)
#define LIN_SCI_FRAME_ERROR                 (0x4000000U)
#define LIN_SCI_PARITY_ERROR                (0x2000000U)
#define LIN_SCI_BREAK_ERROR                 (0x1000000U)
/** @} */

/**
 *  \anchor LIN_SCIErrorDefMisc
 *  \name   Lin Serial Communication Interface Error Defination
 *
 *  Note: Definitions for the intFlags parameter of LIN_enableSCIInterrupt(),
 *        LIN_disableSCIInterrupt(), LIN_clearSCIInterruptStatus(),
 *        LIN_setSCIInterruptLevel0() and LIN_setSCIInterruptLevel1().
 *
 *  @{
 */
#define LIN_SCI_INT_BREAK                   (0x1U)
#define LIN_SCI_INT_WAKEUP                  (0x2U)
#define LIN_SCI_INT_TX                      (0x100U)
#define LIN_SCI_INT_RX                      (0x200U)
#define LIN_SCI_INT_TX_DMA                  (0x10000U)
#define LIN_SCI_INT_RX_DMA                  (0x20000U)
#define LIN_SCI_INT_PARITY                  (0x1000000U)
#define LIN_SCI_INT_OVERRUN                 (0x2000000U)
#define LIN_SCI_INT_FRAME                   (0x4000000U)
#define LIN_SCI_INT_ALL                     (0x7000303U)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief  The following are defines for the \e type parameter of the
 *          LIN_enableExtLoopback() function. */
typedef enum
{
    LIN_LOOPBACK_DIGITAL = 0U,    //!< Digital Loopback Mode
    LIN_LOOPBACK_ANALOG  = 1U     //!< Analog Loopback Mode
} LIN_LoopbackType;

/** \brief  The following are defines for the \e path parameter of the
 *          LIN_enableExtLoopback() function. */
typedef enum
{
    LIN_ANALOG_LOOP_NONE = 0U,    //!< Default path for digital loopback mode
    LIN_ANALOG_LOOP_TX   = 0U,    //!< Analog loopback through transmit pin
    LIN_ANALOG_LOOP_RX   = 1U     //!< Analog loopback through receive pin
} LIN_AnalogLoopback;

/** \brief  The following are defines for the \e mode parameter of the
 *          LIN_setCommMode() function. */
typedef enum
{
    LIN_COMM_LIN_USELENGTHVAL = 0x0000U,
    LIN_COMM_LIN_ID4ID5LENCTL = 0x0001U
} LIN_CommMode;

/** \brief  The following are defines for the \e mode parameter of the
 *          LIN_setSCICommMode() function. */
typedef enum
{
    LIN_COMM_SCI_IDLELINE     = 0x0000U,
    LIN_COMM_SCI_ADDRBIT      = 0x0001U
} LIN_SCICommMode;

/** \brief  The following are defines for the \e mode parameter of the LIN_setLINMode()
 *          function. */
typedef enum
{
    LIN_MODE_LIN_SLAVE        = 0x0U,
    LIN_MODE_LIN_MASTER       = 0x1U
} LIN_LINMode;

/** \brief  The following are defines for the \e line parameter of the
 *          LIN_enableGlobalInterrupt(), LIN_disableGlobalInterrupt(),
 *          LIN_clearGlobalInterruptStatus(), and LIN_getGlobalInterruptStatus()
 *          functions. */
typedef enum
{
    LIN_INTERRUPT_LINE0       = 0x0U,
    LIN_INTERRUPT_LINE1       = 0x1U
} LIN_InterruptLine;

/** \brief  The following are defines for the \e type parameter of the
 *          LIN_setMessageFiltering() function. */
typedef enum
{
    LIN_MSG_FILTER_IDBYTE  = 0x0U,
    LIN_MSG_FILTER_IDSLAVE = 0x1U
} LIN_MessageFilter;

/** \brief  The following are defines for the \e type parameter of the
 *          LIN_setChecksumType() function. */
typedef enum
{
    LIN_CHECKSUM_CLASSIC  = 0x0U,
    LIN_CHECKSUM_ENHANCED = 0x1U
} LIN_ChecksumType;

/** \brief  The following are defines for the \e mode parameter of the
 *          LIN_setDebugSuspendMode() function. */
typedef enum
{
    LIN_DEBUG_FROZEN   = 0x0U,    //!< Freeze module during debug
    LIN_DEBUG_COMPLETE = 0x1U     //!< Complete Tx/Rx before Freezing
} LIN_DebugMode;

/** \brief  The following are defines for the \e mask parameter of the
 *          LIN_setPinSampleMask() function. */
typedef enum
{
    LIN_PINMASK_NONE         = 0x0U,
    LIN_PINMASK_CENTER       = 0x1U,
    LIN_PINMASK_CENTER_SCLK  = 0x2U,
    LIN_PINMASK_CENTER_2SCLK = 0x3U
} LIN_PinSampleMask;

/** \brief  The following are defines for the \e parity parameter of the
 *          LIN_enableSCIParity() function. */
typedef enum
{
    LIN_SCI_PAR_ODD   = 0x0U,
    LIN_SCI_PAR_EVEN  = 0x1U
} LIN_SCIParityType;

/** \brief  The following are defines for the \e number parameter of the
 *          LIN_setSCIStopBits() function. */
typedef enum
{
    LIN_SCI_STOP_ONE   = 0x0U,
    LIN_SCI_STOP_TWO   = 0x1U
} LIN_SCIStopBits;

/** \brief  The following are defines for the \e pin parameter of the
 *          LIN_getPinStatus() function. */
typedef enum
{
    LIN_PINTYPE_TX = 0x4U,
    LIN_PINTYPE_RX = 0x2U
} LIN_PinType;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *
 *  \brief  This function initializes the LIN module
 *
 *  \param  base       Base Address of the LIN Registers.
 *
 */
void LIN_initModule(uint32_t base);

/**
 *
 *  \brief  This function reads the received data
 *
 *  \param  base            Base Address of the LIN Registers.
 *  \param  data            Pointer pointing to the buffer where the data is to be saved.
 *
 */
void LIN_getData(uint32_t base, uint16_t * const data);

/**
 *
 *  \brief  This function sends the data
 *
 *  \param  base            Base Address of the LIN Registers.
 *  \param  data            Pointer pointing to the buffer where the data to send is saved.
 *
 */
void LIN_sendData(uint32_t base, uint16_t *data);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/**
 *  \brief  Checks a LIN base address.
 *
 *  \param  base is the base address of the LIN controller.
 *
 *  \note   This function determines if a LIN controller base address is valid.
 *
 *  \return Returns \b true if the base address is valid and \b false
 *          otherwise.
 *
 */
static inline Bool
LIN_isBaseValid(uint32_t base)
{
    #if defined (SOC_AM263X)
    return(
           (base == CSL_LIN0_U_BASE) ||
           (base == CSL_LIN1_U_BASE) ||
           (base == CSL_LIN2_U_BASE) ||
           (base == CSL_LIN3_U_BASE) ||
           (base == CSL_LIN4_U_BASE)
          );
    #else
    return TRUE;
    #endif /* defined (SOC_AM263X) */
}

/**
 *  \brief  Sets the LIN mode.
 *
 *  \param  base is the base address of the LIN controller.
 *  \param  mode is the desired mode (slave or master).
 *
 *  \note   In LIN mode only, this function sets the mode of the
 *          LIN mode to either slave or master. The \e mode parameter
 *          should be passed a value of \b LIN_MODE_LIN_SLAVE or \b LIN_MODE_LIN_MASTER
 *          to configure the mode of the LIN module specified by \e base.
 *
 */
static inline void
LIN_setLINMode(uint32_t base, LIN_LINMode mode)
{
    /* Paramter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Program LIN Mode */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_CLK_MASTER_MASK, CSL_LIN_SCIGCR1_CLK_MASTER_SHIFT, mode);
}

/**
 *  \brief  Set Maximum Baud Rate Prescaler.
 *
 *  \param  base is the base address of the LIN controller.
 *  \param  clock is the device system clock (Hz).
 *
 *  In LIN mode only, this function is used to set the maximum baud rate
 *  prescaler used during synchronization phase of a slave module if the
 *  ADAPT bit is set. The maximum baud rate prescaler is used by the wakeup
 *  and idle timer counters for a constant 4 second expiration time relative
 *  to a 20kHz rate.
 *
 *  \note   Use LIN_enableAutomaticBaudrate() to set the ADAPT bit and enable
 *          automatic bit rate mod detection.
 *
 */
static inline void
LIN_setMaximumBaudRate(uint32_t base, uint32_t clock)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Calculate maximum baud rate prescaler */
    HW_WR_FIELD32_RAW((base + CSL_LIN_MBRSR), CSL_LIN_MBRSR_MBR_MASK, CSL_LIN_MBRSR_MBR_SHIFT, (clock / 20000U));
}

/**
 *  \brief  Set Message filtering Type.
 *
 *  \param  base is the LIN module base address.
 *  \param  type is the mask filtering comparison type.
 *
 *  In LIN mode only, this function sets the message filtering type. The \e
 *  type parameter can be one of the following values:
 *  - \b LIN_MSG_FILTER_IDBYTE   - Filtering uses LIN message ID Byte
 *  - \b LIN_MSG_FILTER_IDSLAVE  - Filtering uses the Slave Task ID Byte
 *
 */
static inline void
LIN_setMessageFiltering(uint32_t base, LIN_MessageFilter type)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Sets the message filtering type */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_HGENCTRL_MASK, CSL_LIN_SCIGCR1_HGENCTRL_SHIFT, type);
}

/**
 *  \brief  Enable Parity mode.
 *
 *  \param  base is the LIN module base address.
 *
 *  In LIN mode only, this function enables the parity check.
 *
 */
static inline void
LIN_enableParity(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Enable the parity mode */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_PARITYENA_MASK, CSL_LIN_SCIGCR1_PARITYENA_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable Parity mode.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function disables the parity check.
 *
 */
static inline void
LIN_disableParity(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Enable the parity mode */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_PARITYENA_MASK, CSL_LIN_SCIGCR1_PARITYENA_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Generate Parity Identifier.
 *
 *  \param  identifier is the LIN header ID byte
 *
 *  In LIN mode only, this function generates the identifier parity bits and
 *  appends them to the identifier.
 *
 *  \note An ID must be generated with parity before header generation in
 *  LIN master mode when parity is enabled using the function
 *  LIN_enableParity().
 *
 *  \return Returns the identifier appended with parity bits.
 *
 */
static inline uint16_t
LIN_generateParityID(uint16_t identifier)
{
    uint16_t p0, p1, parityIdentifier;

    /* Calculate parity bits and generate updated identifier */
    p0 = ((identifier & LIN_ID0) ^ ((identifier & LIN_ID1) >> 1U) ^
          ((identifier & LIN_ID2) >> 2U) ^ ((identifier & LIN_ID4) >> 4U));
    p1 = !(((identifier & LIN_ID1) >> 1U) ^ ((identifier & LIN_ID3) >> 3U) ^
           ((identifier & LIN_ID4) >> 4U) ^ ((identifier & LIN_ID5) >> 5U));
    parityIdentifier = identifier | ((p0 << 6U) | (p1 << 7U));

    return(parityIdentifier);
}

/**
 *  \brief  Set ID Byte.
 *
 *  \param  base is the LIN module base address
 *  \param  identifier is the LIN header ID byte
 *
 *  In LIN mode only, this function sets the message ID byte. In master mode,
 *  writing to this ID initiates a header transmission. In slave task, this
 *  ID is used for message filtering when HGENCTRL is 0.
 *
 */
static inline void
LIN_setIDByte(uint32_t base, uint16_t identifier)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_FIELD32_RAW((base + CSL_LIN_LINID), CSL_LIN_LINID_IDBYTE_MASK, CSL_LIN_LINID_IDBYTE_SHIFT, identifier);
}

/**
 *  \brief  Set ID-SlaveTask.
 *
 *  \param  base is the LIN module base address
 *  \param  identifier is the Received ID comparison ID
 *
 *  In LIN mode only, this function sets the identifier to which the received
 *  ID of an incoming Header will be compared in order to decide whether a RX
 *  response, a TX response, or no action is required.
 *
 */
static inline void
LIN_setIDSlaveTask(uint32_t base, uint16_t identifier)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_FIELD32_RAW((base + CSL_LIN_LINID), CSL_LIN_LINID_IDSLAVETASKBYTE_MASK, CSL_LIN_LINID_IDSLAVETASKBYTE_SHIFT, identifier);
}

/**
 *  \brief  Send LIN wakeup signal.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function sends the LIN wakeup signal to terminate
 *  the sleep mode of any LIN node connected to the bus.
 *
 */
static inline void
LIN_sendWakeupSignal(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set key in Byte 0 (MSB) of transmit buffer 0 register */
    HW_WR_FIELD32_RAW((base + CSL_LIN_LINTD0), CSL_LIN_LINTD0_TD3_MASK, CSL_LIN_LINTD0_TD3_SHIFT, (uint16_t)LIN_WAKEUP_KEY);

    /* Transmit TDO for wakeup */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR2), CSL_LIN_SCIGCR2_GENWU_MASK, CSL_LIN_SCIGCR2_GENWU_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Entering LIN sleep signal.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function puts the LIN module into a low-power, sleep
 *  mode. This can also be called to forcefully enter sleep when there is no
 *  activity on the bus.
 *
 *  \note If this function is called while the receiver is actively receiving
 *  data and the wakeup interrupt is disabled, then the module will delay
 *  sleep mode from being entered until completion of reception.
 *
 */
static inline void
LIN_enterSleep(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Entering Powerdown */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR2), CSL_LIN_SCIGCR2_POWERDOWN_MASK, CSL_LIN_SCIGCR2_POWERDOWN_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Send Checksum Byte.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function enables the transmitter with extended
 *  frames to send a checkbyte.
 *
 */
static inline void
LIN_sendChecksum(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Setting the Check Sum */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR2), CSL_LIN_SCIGCR2_SC_MASK, CSL_LIN_SCIGCR2_SC_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Trigger Checksum Compare.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function enables the receiver for extended frames
 *  to trigger a checksum compare.
 *
 */
static inline void
LIN_triggerChecksumCompare(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Comparing the Check Sum */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR2), CSL_LIN_SCIGCR2_CC_MASK, CSL_LIN_SCIGCR2_CC_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Check Tx buffer ready flag.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function checks to see if the Tx ready flag is set
 *  indicating that the Tx buffer(s) is/are ready to get another character.
 *
 *  \return Returns \b true if the TX ready flag is set, else returns \b false
 *
 */
static inline Bool
LIN_isTxReady(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Check TXRDY BIT */
    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_TXRDY_MASK) ==
           CSL_LIN_SCIFLR_TXRDY_MASK);
}

/**
 *  \brief  Set LIN Frame Length.
 *
 *  \param  base is the LIN module base address
 *  \param  length is the number of bytes.
 *
 *  In LIN mode only, this function sets the number of bytes in the response
 *  field.
 *
 *  The \e length parameter must be in a range between 1 and 8.
 *
 */
static inline void
LIN_setFrameLength(uint32_t base, uint16_t length)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((length > 0U) && (length < 9U));

    /* Clear and set frame length value */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIFORMAT), CSL_LIN_SCIFORMAT_LENGTH_MASK, CSL_LIN_SCIFORMAT_LENGTH_SHIFT, ((uint32_t)length - (uint32_t)1U));
}

/**
 *  \brief  Set LIN communication mode
 *
 *  \param  base is the LIN module base address
 *  \param  mode is the selected communication mode
 *
 *  In LIN mode only, this function is used to choose how the length of data is
 *  conveyed. This choice relates to the version of LIN being used. The \e mode
 *  parameter can have one of two values:
 *  - \b LIN_COMM_LIN_USELENGTHVAL will use the length set with the
 *    LIN_setFrameLength() function.
 *  - \b LIN_COMM_LIN_ID4ID5LENCTL will use ID4 and ID5 for length control.
 *
 */
static inline void
LIN_setCommMode(uint32_t base, LIN_CommMode mode)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Write communication mode selection to the appropriate bit. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_COMMMODE_MASK, CSL_LIN_SCIGCR1_COMMMODE_SHIFT, (uint16_t)mode);
}

/**
 *  \brief  Sets the transmit ID mask
 *
 *  \param  base is the LIN module base address
 *  \param  mask is the mask value to be set
 *
 *  In LIN mode only, this function sets the mask used for filtering an
 *  incoming ID message to determine if the TX ID flag should be set.
 *
 */
static inline void
LIN_setTxMask(uint32_t base, uint16_t mask)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear previous mask value and set new mask */
    HW_WR_FIELD32_RAW((base + CSL_LIN_LINMASK), CSL_LIN_LINMASK_TXIDMASK_MASK, CSL_LIN_LINMASK_TXIDMASK_SHIFT, mask);
}

/**
 *  \brief  Sets the receive ID mask
 *
 *  \param  base is the LIN module base address
 *  \param  mask is the mask value to be set
 *
 *  In LIN mode only, this function sets the mask used for filtering an
 *  incoming ID message to determine if the ID RX flag should be set.
 *
 */
static inline void
LIN_setRxMask(uint32_t base, uint16_t mask)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear previous mask value and set new mask */
    HW_WR_FIELD32_RAW((base + CSL_LIN_LINMASK), CSL_LIN_LINMASK_RXIDMASK_MASK, CSL_LIN_LINMASK_RXIDMASK_SHIFT, (uint32_t)mask);
}

/**
 *  \brief  Gets the transmit ID mask
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function gets the mask used for filtering an
 *  incoming ID message to determine if the TX ID flag should be set.
 *
 */
static inline uint16_t
LIN_getTxMask(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Get Tx Mask status */
    return(HW_RD_FIELD32_RAW((base + CSL_LIN_LINMASK), CSL_LIN_LINMASK_TXIDMASK_MASK, CSL_LIN_LINMASK_TXIDMASK_SHIFT));
}

/**
 *  \brief  Gets the receive ID mask
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function gets the mask used for filtering an
 *  incoming ID message to determine if the ID RX flag should be set.
 *
 *  \return  Returns the Transmit ID Mask.
 *
 */
static inline uint16_t
LIN_getRxMask(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Get Tx Mask status */
    return(HW_RD_FIELD32_RAW((base + CSL_LIN_LINMASK), CSL_LIN_LINMASK_RXIDMASK_MASK, CSL_LIN_LINMASK_RXIDMASK_SHIFT));
}

/**
 *  \brief  Gets the receive ID mask
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, checks to see if the Rx ready bit is set indicating that
 *  a valid message frame has been received.
 *
 *  \return  Returns \b true if the Rx ready flag is set, else returns \b false.
 *
 */
static inline Bool
LIN_isRxReady(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Ready Rx ready flag and return status */
    return(HW_RD_FIELD32_RAW((base + CSL_LIN_SCIFLR), CSL_LIN_SCIFLR_RXRDY_MASK, CSL_LIN_SCIFLR_RXRDY_SHIFT));
}

/**
 *  \brief  Get last received identifier
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function gets the last received identifier.
 *
 *  \return Returns the Received Identifier.
 *
 */
static inline uint16_t
LIN_getRxIdentifier(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Ready Rx ready flag and return status */
    return(HW_RD_FIELD32_RAW((base + CSL_LIN_LINID), CSL_LIN_LINID_RECEIVEDID_MASK, CSL_LIN_LINID_RECEIVEDID_SHIFT));
}

/**
 *  \brief  Checks for Tx ID Match Received
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function checks if an ID is received with a TX match
 *  and no ID-parity error.
 *
 *  \return Returns \b true if a valid ID is matched, else returns \b false.
 *
 */
static inline Bool
LIN_isTxMatch(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read Tx ID flag and return status */
    return(HW_RD_FIELD32_RAW((base + CSL_LIN_SCIFLR), CSL_LIN_SCIFLR_IDTXFLAG_MASK, CSL_LIN_SCIFLR_IDTXFLAG_SHIFT));
}

/**
 *  \brief  Checks for Rx ID Match Received
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function checks if an ID is received with a RX match
 *  and no ID-parity error.
 *
 *  \return Returns \b true if a valid ID is matched, else returns \b false.
 *
 */
static inline Bool
LIN_isRxMatch(uint32_t base)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read Tx ID flag and return status */
    return(HW_RD_FIELD32_RAW((base + CSL_LIN_SCIFLR), CSL_LIN_SCIFLR_IDRXFLAG_MASK, CSL_LIN_SCIFLR_IDRXFLAG_SHIFT));
}

/**
 *  \brief  Enable interrupts
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of the interrupt sources to be enabled
 *
 *  In LIN mode only, this function enables the interrupts for the specified
 *  interrupt sources.
 *
 *   The \e intFlags parameter can be set to the following value to set
 *   all the flag bits:
 *   - \b LIN_INT_ALL     - All Interrupts
 *
 *   To set individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *   - \b LIN_INT_WAKEUP  - Wakeup
 *   - \b LIN_INT_TO      - Time out
 *   - \b LIN_INT_TOAWUS  - Time out after wakeup signal
 *   - \b LIN_INT_TOA3WUS - Time out after 3 wakeup signals
 *   - \b LIN_INT_TX      - Transmit buffer ready
 *   - \b LIN_INT_RX      - Receive buffer ready
 *   - \b LIN_INT_ID      - Received matching identifier
 *   - \b LIN_INT_PE      - Parity error
 *   - \b LIN_INT_OE      - Overrun error
 *   - \b LIN_INT_FE      - Framing error
 *   - \b LIN_INT_NRE     - No response error
 *   - \b LIN_INT_ISFE    - Inconsistent sync field error
 *   - \b LIN_INT_CE      - Checksum error
 *   - \b LIN_INT_PBE     - Physical bus error
 *   - \b LIN_INT_BE      - Bit error
 *
 *
 */
static inline void
LIN_enableInterrupt(uint32_t base, uint32_t intFlags)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set Interrupt Flags */
    HW_WR_REG32_RAW((base + CSL_LIN_SCISETINT), HW_RD_REG32_RAW(base + CSL_LIN_SCISETINT)|intFlags);
}

/**
 *  \brief  Disable interrupts
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of the interrupt sources to be disabled.
 *
 *  In LIN mode only, this function disables the interrupts for the specified
 *  interrupt sources.
 *
 *   The \e intFlags parameter can be set to the following value to clear
 *   all the flag bits:
 *   - \b LIN_INT_ALL     - All Interrupts
 *
 *   To clear individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *   - \b LIN_INT_WAKEUP  - Wakeup
 *   - \b LIN_INT_TO      - Time out
 *   - \b LIN_INT_TOAWUS  - Time out after wakeup signal
 *   - \b LIN_INT_TOA3WUS - Time out after 3 wakeup signals
 *   - \b LIN_INT_TX      - Transmit buffer ready
 *   - \b LIN_INT_RX      - Receive buffer ready
 *   - \b LIN_INT_ID      - Received matching identifier
 *   - \b LIN_INT_PE      - Parity error
 *   - \b LIN_INT_OE      - Overrun error
 *   - \b LIN_INT_FE      - Framing error
 *   - \b LIN_INT_NRE     - No response error
 *   - \b LIN_INT_ISFE    - Inconsistent sync field error
 *   - \b LIN_INT_CE      - Checksum error
 *   - \b LIN_INT_PBE     - Physical bus error
 *   - \b LIN_INT_BE      - Bit error
 *
 *
 */
static inline void
LIN_disableInterrupt(uint32_t base, uint32_t intFlags)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear Interrupt Flags */
    HW_WR_REG32_RAW((base + CSL_LIN_SCICLEARINT), intFlags);
}

/**
 *  \brief  Clear interrupt status
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of the interrupt sources to be cleared.
 *
 *  In LIN mode only, this function clears the specified status flags.
 *
 *   The \e intFlags parameter can be set to the following value to clear
 *   all the flag bits:
 *   - \b LIN_INT_ALL     - All Interrupts
 *
 *   To clear individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *   - \b LIN_INT_WAKEUP  - Wakeup
 *   - \b LIN_INT_TO      - Time out
 *   - \b LIN_INT_TOAWUS  - Time out after wakeup signal
 *   - \b LIN_INT_TOA3WUS - Time out after 3 wakeup signals
 *   - \b LIN_INT_TX      - Transmit buffer ready
 *   - \b LIN_INT_RX      - Receive buffer ready
 *   - \b LIN_INT_ID      - Received matching identifier
 *   - \b LIN_INT_PE      - Parity error
 *   - \b LIN_INT_OE      - Overrun error
 *   - \b LIN_INT_FE      - Framing error
 *   - \b LIN_INT_NRE     - No response error
 *   - \b LIN_INT_ISFE    - Inconsistent sync field error
 *   - \b LIN_INT_CE      - Checksum error
 *   - \b LIN_INT_PBE     - Physical bus error
 *   - \b LIN_INT_BE      - Bit error
 *
 *
 */
static inline void
LIN_clearInterruptStatus(uint32_t base, uint32_t intFlags)
{
    /* Parameter Validation */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear Status Flags */
    HW_WR_REG32_RAW((base + CSL_LIN_SCIFLR), intFlags);
}

/**
 *  \brief  Set interrupt level to 0
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of the interrupt sources to be cleared.
 *
 *  In LIN mode only, this function sets the specified interrupt sources to
 *  level 0.
 *
 *   The \e intFlags parameter can be set to the following value to set
 *   all the flag bits:
 *   - \b LIN_INT_ALL     - All Interrupts
 *
 *   To set individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *   - \b LIN_INT_WAKEUP  - Wakeup
 *   - \b LIN_INT_TO      - Time out
 *   - \b LIN_INT_TOAWUS  - Time out after wakeup signal
 *   - \b LIN_INT_TOA3WUS - Time out after 3 wakeup signals
 *   - \b LIN_INT_TX      - Transmit buffer ready
 *   - \b LIN_INT_RX      - Receive buffer ready
 *   - \b LIN_INT_ID      - Received matching identifier
 *   - \b LIN_INT_PE      - Parity error
 *   - \b LIN_INT_OE      - Overrun error
 *   - \b LIN_INT_FE      - Framing error
 *   - \b LIN_INT_NRE     - No response error
 *   - \b LIN_INT_ISFE    - Inconsistent sync field error
 *   - \b LIN_INT_CE      - Checksum error
 *   - \b LIN_INT_PBE     - Physical bus error
 *   - \b LIN_INT_BE      - Bit error
 *
 *
 */
static inline void
LIN_setInterruptLevel0(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear Status Flags */
    HW_WR_REG32_RAW((base + CSL_LIN_SCICLEARINTLVL), intFlags);
}

/**
 *  \brief  Set interrupt level to 1
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of the interrupt sources to be cleared.
 *
 *  In LIN mode only, this function sets the specified interrupt sources to
 *  level 1.
 *
 *   The \e intFlags parameter can be set to the following value to set
 *   all the flag bits:
 *   - \b LIN_INT_ALL     - All Interrupts
 *
 *   To set individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *   - \b LIN_INT_WAKEUP  - Wakeup
 *   - \b LIN_INT_TO      - Time out
 *   - \b LIN_INT_TOAWUS  - Time out after wakeup signal
 *   - \b LIN_INT_TOA3WUS - Time out after 3 wakeup signals
 *   - \b LIN_INT_TX      - Transmit buffer ready
 *   - \b LIN_INT_RX      - Receive buffer ready
 *   - \b LIN_INT_ID      - Received matching identifier
 *   - \b LIN_INT_PE      - Parity error
 *   - \b LIN_INT_OE      - Overrun error
 *   - \b LIN_INT_FE      - Framing error
 *   - \b LIN_INT_NRE     - No response error
 *   - \b LIN_INT_ISFE    - Inconsistent sync field error
 *   - \b LIN_INT_CE      - Checksum error
 *   - \b LIN_INT_PBE     - Physical bus error
 *   - \b LIN_INT_BE      - Bit error
 *
 *
 */
static inline void
LIN_setInterruptLevel1(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set interrupt levels to 1 */
    HW_WR_REG32_RAW((base + CSL_LIN_SCISETINTLVL), HW_RD_REG32_RAW(base + CSL_LIN_SCISETINTLVL)|intFlags);
}

/**
 *  \brief  Enable Module Errors for Testing
 *
 *  \param  base is the LIN module base address
 *  \param  errors is the specified errors to be enabled
 *
 *  In LIN mode only, this function enables the specified errors in the module
 *  for testing. The \e errors parameter can be a logical OR-ed result of the
 *  following values or \b LIN_ALL_ERRORS can be used to enable all of them:
 *  - \b LIN_BIT_ERROR      - Simulates a bit error
 *  - \b LIN_BUS_ERROR      - Simulates a physical bus error
 *  - \b LIN_CHECKSUM_ERROR - Simulates a checksum error
 *  - \b LIN_ISF_ERROR      - Simulates an inconsistent synch field error
 *
 *  \note To disable these errors, use the LIN_disableModuleErrors() function.
 *
 */
static inline void
LIN_enableModuleErrors(uint32_t base, uint32_t errors)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear the IO DFT Enable Key & Enable write access*/
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, (uint32_t)LIN_IO_DFT_KEY);

    /* Enable specified error bits */
    HW_WR_REG32_RAW((base + CSL_LIN_IODFTCTRL), HW_RD_REG32_RAW(base + CSL_LIN_IODFTCTRL)|errors);

    /* Clear the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, (uint32_t)CSL_FALSE);
}

/**
 *  \brief  Disable Module Errors for Testing
 *
 *  \param  base is the LIN module base address
 *  \param  errors is the specified errors to be disabled
 *
 *  In LIN mode only, this function disables the specified errors in the module
 *  for testing. The \e errors parameter can be a logical OR-ed result of the
 *  following values or \b LIN_ALL_ERRORS can be used to enable all of them:
 *  - \b LIN_BIT_ERROR      - Simulates a bit error
 *  - \b LIN_BUS_ERROR      - Simulates a physical bus error
 *  - \b LIN_CHECKSUM_ERROR - Simulates a checksum error
 *  - \b LIN_ISF_ERROR      - Simulates an inconsistent synch field error
 *
 */
static inline void
LIN_disableModuleErrors(uint32_t base, uint32_t errors)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear the IO DFT Enable Key & Enable write access */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, (uint32_t)LIN_IO_DFT_KEY);

    /* Disable specified error bits */
    HW_WR_REG32_RAW((base + CSL_LIN_IODFTCTRL), HW_RD_REG32_RAW(base + CSL_LIN_IODFTCTRL)&~(errors));

    /* Clear the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, (uint32_t)CSL_FALSE);
}

/**
 *  \brief  Enable Automatic Baudrate Adjustment
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function enables the automatic baudrate adjustment
 *  mode during the detection of the Synch Field.
 *
 *  \note The baudrate selection register will be updated automatically by a
 *  slave node if this mode is enabled.
 *
 */
static inline void
LIN_enableAutomaticBaudrate(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Specified error bits */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_ADAPT_MASK, CSL_LIN_SCIGCR1_ADAPT_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable Automatic Baudrate Adjustment
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function disables the automatic baudrate adjustment
 *  mode during the detection of the Synch Field. This results in a fixed baud rate.
 *
 */
static inline void
LIN_disableAutomaticBaudrate(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Specified error bits */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_ADAPT_MASK, CSL_LIN_SCIGCR1_ADAPT_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Stops LIN Extended Frame Communication
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN mode only, this function stops the extended frame communication.
 *  Once stopped, the bit is automatically cleared.
 *
 *  \note This function can only be called during extended frame communication.
 *
 */
static inline void
LIN_stopExtendedFrame(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set stop bit. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_STOPEXTFRAME_MASK, CSL_LIN_SCIGCR1_STOPEXTFRAME_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Set Checksum Type
 *
 *  \param  base is the LIN module base address
 *  \param  type is the checksum type
 *
 *  In LIN mode only, this function sets the checksum type. The \e type
 *  parameter can be one of the following two values:
 *  - \b LIN_CHECKSUM_CLASSIC  - Checksum Classic
 *  - \b LIN_CHECKSUM_ENHANCED - Checksum Enhanced
 *
 */
static inline void
LIN_setChecksumType(uint32_t base, LIN_ChecksumType type)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set stop bit. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_CTYPE_MASK, CSL_LIN_SCIGCR1_CTYPE_SHIFT, type);
}

/**
 *  \brief  Set Sync Break Extend and Delimiter
 *
 *  \param base is the LIN module base address
 *  \param syncBreak is the sync break extend value
 *  \param delimiter is the sync delimiter value
 *
 *  In LIN mode only, this function sets the 3-bit sync break extend value
 *  and the 2-bit sync delimiter compare value.
 *
 *  The \e break parameter can be a value between 0 to 7. Details:
 *  - \b 0 - Sync Break has no additional T-bit
 *  - \b 1 - Sync Break has 1 additional T-bit
 *  - \b ...
 *  - \b 7 - Sync Break has 7 additional T-bits
 *
 *  The \e delimiter parameter can be a value between 1 to 4. Details:
 *  - \b 1 - Delimiter has 1 T-bit
 *  - \b 2 - Delimiter has 2 T-bits
 *  - \b 3 - Delimiter has 3 T-bits
 *  - \b 4 - Delimiter has 4 T-bits
 *
 */
static inline void
LIN_setSyncFields(uint32_t base, uint16_t syncBreak, uint16_t delimiter)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert(syncBreak < 8U);
    DebugP_assert((delimiter >= 1U) && (delimiter < 5U));

    /* Clear sync values and set new values */
    HW_WR_FIELD32_RAW((base + CSL_LIN_LINCOMP), (CSL_LIN_LINCOMP_SDEL_MASK|CSL_LIN_LINCOMP_SBREAK_MASK),
                        CSL_LIN_LINCOMP_SDEL_SHIFT, (syncBreak | (delimiter - 1U)));
}

/**
 *  \brief  Enable SCI Mode
 *
 *  \param base is the LIN module base address
 *
 *  This function enables the LIN peripheral to function as a SCI.
 *
 */
static inline void
LIN_enableSCIMode(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_LINMODE_MASK,
                        CSL_LIN_SCIGCR1_LINMODE_SHIFT, CSL_FALSE);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_CLK_MASTER_MASK,
                        CSL_LIN_SCIGCR1_CLK_MASTER_SHIFT, CSL_TRUE);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_TIMINGMODE_MASK,
                    CSL_LIN_SCIGCR1_TIMINGMODE_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable SCI Mode
 *
 *  \param base is the LIN module base address
 *
 *  This function disables the SCI mode of the LIN peripheral.
 *
 */
static inline void
LIN_disableSCIMode(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Disable SCI communications mode */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_CLK_MASTER_MASK,
                        CSL_LIN_SCIGCR1_CLK_MASTER_SHIFT, CSL_FALSE);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_TIMINGMODE_MASK,
                    CSL_LIN_SCIGCR1_TIMINGMODE_SHIFT, CSL_FALSE);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_LINMODE_MASK,
                        CSL_LIN_SCIGCR1_LINMODE_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Set SCI communication mode
 *
 *  \param base is the LIN module base address
 *  \param mode is the selected communication mode
 *
 *  In SCI mode only, this function is used to select between idle-line mode
 *  and address-bit mode. The \e mode parameter can have one of the following
 *  values:
 *  - \b LIN_COMM_SCI_IDLELINE - Idle-line mode.
 *  - \b LIN_COMM_SCI_ADDRBIT  - Address-bit mode.
 *
 */
static inline void
LIN_setSCICommMode(uint32_t base, LIN_SCICommMode mode)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_COMMMODE_MASK,
                    CSL_LIN_SCIGCR1_COMMMODE_SHIFT, mode);
}

/**
 *  \brief  Enable SCI Parity mode.
 *
 *  \param base is the LIN module base address
 *  \param parity is the SCI parity type
 *
 *  In SCI mode only, this function enables the parity check and sets the
 *  parity type. The \e parity parameter can one of the following values:
 *  - \b LIN_SCI_PAR_ODD  - Sets Odd parity
 *  - \b LIN_SCI_PAR_EVEN - Sets Even parity
 *
 */
static inline void
LIN_enableSCIParity(uint32_t base, LIN_SCIParityType parity)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_PARITYENA_MASK,
                    CSL_LIN_SCIGCR1_PARITYENA_SHIFT, 1U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_PARITY_MASK,
                CSL_LIN_SCIGCR1_PARITY_SHIFT, parity);
}

/**
 *  \brief  Disable SCI Parity mode.
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function disables the parity check.
 *
 */
static inline void
LIN_disableSCIParity(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_PARITYENA_MASK,
                    CSL_LIN_SCIGCR1_PARITYENA_SHIFT, 0U);
}

/**
 *  \brief  Set the number of stop bits for SCI.
 *
 *  \param base is the LIN module base address
 *  \param number is the number of stop bits
 *
 *  In SCI mode only, this function sets the number of stop bits transmitted.
 *  The \e number parameter can be one of the following values:
 *  - \b LIN_SCI_STOP_ONE - Set one stop bit
 *  - \b LIN_SCI_STOP_TWO - Set two stop bits
 *
 */
static inline void
LIN_setSCIStopBits(uint32_t base, LIN_SCIStopBits number)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_STOP_MASK,
                    CSL_LIN_SCIGCR1_STOP_SHIFT, number);
}

/**
 *  \brief  Enable SCI Sleep mode.
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function enables the receive sleep mode
 *  functionality.
 *
 *  \note The receiver still operates when the sleep mode is enabled, however,
 *  RXRDY is updated and SCIRD is loaded with new data only when an address
 *  frame is detected.
 *
 */
static inline void
LIN_enableSCISleepMode(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_SLEEP_MASK,
                    CSL_LIN_SCIGCR1_SLEEP_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable SCI Sleep mode.
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function disables the receive sleep mode
 *  functionality.
 *
 */
static inline void
LIN_disableSCISleepMode(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_SLEEP_MASK,
                    CSL_LIN_SCIGCR1_SLEEP_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Enter SCI Local Low-Power Mode
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function enters the SCI local low-power mode.
 *
 *  \note If this function is called while the receiver is actively receiving
 *  data and the wakeup interrupt is disabled, then the module will delay
 *  sleep mode from being entered until completion of reception.
 *
 */
static inline void
LIN_enterSCILowPower(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR2), CSL_LIN_SCIGCR2_POWERDOWN_MASK,
                    CSL_LIN_SCIGCR2_POWERDOWN_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Exit SCI Local Low-Power Mode
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function exits the SCI local low-power mode.
 *
 */
static inline void
LIN_exitSCILowPower(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR2), CSL_LIN_SCIGCR2_POWERDOWN_MASK,
                    CSL_LIN_SCIGCR2_POWERDOWN_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Set SCI character length
 *
 *  \param base is the LIN module base address
 *  \param numBits is the number of bits per character.
 *
 *  In SCI mode only, this function sets the number of bits per character.
 *
 *  The \e numBits parameter must be in a range between 1 and 8.
 *
 */
static inline void
LIN_setSCICharLength(uint32_t base, uint16_t numBits)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);
    DebugP_assert((numBits > 0U) && (numBits < 9U));

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIFORMAT), CSL_LIN_SCIFORMAT_CHAR_MASK,
                    CSL_LIN_SCIFORMAT_CHAR_SHIFT, (uint32_t)(numBits - 1U));
}

/**
 *  \brief  Set SCI Frame Length
 *
 *  \param base is the LIN module base address
 *  \param length is the number of characters
 *
 *  In SCI mode only, this function sets the number of characters in the
 *  response field.
 *
 *  The \e numBits parameter must be in a range between 1 and 8.
 *
 */
static inline void
LIN_setSCIFrameLength(uint32_t base, uint16_t length)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);
    DebugP_assert((length > 0U) && (length < 9U));

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIFORMAT), CSL_LIN_SCIFORMAT_LENGTH_MASK,
                    CSL_LIN_SCIFORMAT_LENGTH_SHIFT, (uint32_t)(length - 1U));
}

/**
 *  \brief  Check if new SCI data is ready to be read
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function checks to see if the Rx ready bit is set
 *  indicating that a new data has been received.
 *
 *  \return Returns \b true if the Rx ready flag is set, else returns \b false.
 *
 */
static inline Bool
LIN_isSCIDataAvailable(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_RXRDY_MASK) == CSL_LIN_SCIFLR_RXRDY_MASK);
}

/**
 *  \brief  Check if Space is available in SCI Transmit Buffer
 *
 *  \param base is the LIN module base address
 *
 *  In SCI mode only, this function checks to see if the Tx ready flag is set
 *  indicating that the Tx buffer(s) is/are ready to get another character.
 *
 *  \return Returns \b true if the TX ready flag is set, else returns \b false.
 *
 */
static inline Bool
LIN_isSCISpaceAvailable(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_TXRDY_MASK) == CSL_LIN_SCIFLR_TXRDY_MASK);
}

/**
 *  \brief  Reads a SCI character without Blocking
 *
 *  \param base is the LIN module base address
 *  \param emulation sets whether the data is being read by an emulator or not
 *
 *  In SCI mode only, this function gets the byte of data received. The
 *  \e emulation parameter can have one of the following values:
 *  - \b true - Emulator is being used, the RXRDY flag won't be cleared
 *  - \b false - Emulator isn't being used, the RXRDY flag will be cleared
 *               automatically on read
 *
 *  \note
 *  -# If the SCI receives data that is fewer than 8 bits in length, the
 *  data is left-justified and padded with trailing zeros.
 *  -# To determine if new data is available to read, use the function
 *  LIN_isSCIDataAvailable().
 *
 *  \return Returns the received data.
 *
 */
static inline uint16_t
LIN_readSCICharNonBlocking(uint32_t base, Bool emulation)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Read specific data register */
    return(emulation ? (HW_RD_REG32_RAW(base + CSL_LIN_SCIED) & CSL_LIN_SCIED_ED_MASK) :
            (HW_RD_REG32_RAW(base + CSL_LIN_SCIRD) & CSL_LIN_SCIRD_RD_MASK));
}

/**
 *  \brief  Reads a SCI character with Blocking
 *
 *  \param base is the LIN module base address
 *  \param emulation sets whether the data is being read by an emulator or not
 *
 *  In SCI mode only, this function gets the byte of data received. If new data
 *  isn't available, this function will wait until new data arrives. The
 *  \e emulation parameter can have one of the following values:
 *  - \b true - Emulator is being used, the RXRDY flag won't be cleared
 *  - \b false - Emulator isn't being used, the RXRDY flag will be cleared
 *               automatically on read
 *
 *  \note If the SCI receives data that is fewer than 8 bits in length, the
 *        data is left-justified and padded with trailing zeros.
 *
 *  \return Returns the received data.
 *
 */
static inline uint16_t
LIN_readSCICharBlocking(uint32_t base, Bool emulation)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Wait until a character is available in buffer. */
    while(!LIN_isSCIDataAvailable(base))
    {
    }

    /* Read specific data register */
    return(emulation ? (HW_RD_REG32_RAW(base + CSL_LIN_SCIED) & CSL_LIN_SCIED_ED_MASK) :
            (HW_RD_REG32_RAW(base + CSL_LIN_SCIRD) & CSL_LIN_SCIRD_RD_MASK));
}

/**
 *  \brief  Sends a SCI character without Blocking
 *
 *  \param base is the LIN module base address
 *  \param data is the byte of data to be transmitted
 *
 *  In SCI mode only, this function sets the byte of data to be transmitted
 *  without blocking.
 *
 *  \note The transmit ready flag gets set when this buffer is ready to be
 *   loaded with another byte of data. Use LIN_isSCISpaceAvailable() to
 *   determine if space is available to write another character.
 *
 */
static inline void
LIN_writeSCICharNonBlocking(uint32_t base, uint16_t data)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);
    DebugP_assert(data <= CSL_LIN_SCITD_TD_MASK);

    /* Set the Tx Data */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCITD), CSL_LIN_SCITD_TD_MASK, CSL_LIN_SCITD_TD_SHIFT, data);
}

/**
 *  \brief  Sends a SCI character with blocking
 *
 *  \param base is the LIN module base address
 *  \param data is the byte of data to be transmitted
 *
 *  In SCI mode only, this function sets the byte of data to be transmitted
 *  with blocking functionality. If the buffer isn't ready to get new data
 *  written to, this function will wait until space is available.
 *
 */
static inline void
LIN_writeSCICharBlocking(uint32_t base, uint16_t data)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);
    DebugP_assert(data <= CSL_LIN_SCITD_TD_MASK);

    /* Wait until space is available in the transmit buffer. */
    while(!LIN_isSCISpaceAvailable(base))
    {
    }

    /* Set the Tx Data */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCITD), CSL_LIN_SCITD_TD_MASK, CSL_LIN_SCITD_TD_SHIFT, data);
}

/**
 *  \brief  Enable SCI Module Errors for Testing
 *
 *  \param base is the LIN module base address
 *  \param errors is the specified errors to be enabled
 *
 *  In SCI mode only, this function enables the specified errors in the module
 *  for testing. The \e errors parameter can be a logical OR-ed result of the
 *  following values or \b LIN_SCI_ALL_ERRORS can be used to enable all of
 *  them:
 *  - \b LIN_SCI_FRAME_ERROR   - Simulates a frame error
 *  - \b LIN_SCI_PARITY_ERROR  - Simulates a parity error
 *  - \b LIN_SCI_BREAK_ERROR   - Simulates a break detect error
 *
 *  \note To disable these errors, use the LIN_disableSCIModuleErrors()
 *  function.
 *
 */
static inline void
LIN_enableSCIModuleErrors(uint32_t base, uint32_t errors)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Clear the IO DFT Enable Key & Enable write access */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IO_DFT_KEY);

    /* Enable specified error bits */
    HW_WR_REG32_RAW((base + CSL_LIN_IODFTCTRL), (HW_RD_REG32_RAW(base + CSL_LIN_IODFTCTRL)|errors));

    /* Clear the IO DFT Enable Key & Enable write access */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Disable SCI Module Errors for Testing
 *
 *  \param base is the LIN module base address
 *  \param errors is the specified errors to be disabled
 *
 *  In SCI mode only, this function disables the specified errors in the module
 *  for testing. The \e errors parameter can be a logical OR-ed result of the
 *  following values or \b LIN_SCI_ALL_ERRORS can be used to enable all of
 *  them:
 *  - \b LIN_SCI_FRAME_ERROR   - Simulates a frame error
 *  - \b LIN_SCI_PARITY_ERROR  - Simulates a parity error
 *  - \b LIN_SCI_BREAK_ERROR   - Simulates a break detect error
 *
 */
static inline void
LIN_disableSCIModuleErrors(uint32_t base, uint32_t errors)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Clear the IO DFT Enable Key & Enable write access */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IO_DFT_KEY);

    /* Disable specified error bits */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), errors, 0U, CSL_FALSE);

    /* Clear the IO DFT Enable Key & Enable write access */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Enable SCI interrupts
 *
 *  \param base is the LIN module base address
 *  \param intFlags is the bit mask of the interrupt sources to be enabled.
 *
 *  In SCI mode only, this function enables the interrupts for the specified
 *  interrupt sources.
 *
 *  The \e intFlags parameter can be set to the following value to set
 *  all the flag bits:
 *  - \b LIN_SCI_INT_ALL     - All Interrupts
 *
 *
 *  To set individual flags, the \e intFlags parameter can be the logical
 *  OR of any of the following:
 *  - \b LIN_SCI_INT_BREAK    - Break Detect
 *  - \b LIN_SCI_INT_WAKEUP   - Wakeup
 *  - \b LIN_SCI_INT_TX       - Transmit Buffer
 *  - \b LIN_SCI_INT_RX       - Receive Buffer
 *  - \b LIN_SCI_INT_TX_DMA   - DMA Transmit
 *  - \b LIN_SCI_INT_RX_DMA   - DMA Receive
 *  - \b LIN_SCI_INT_PARITY   - Parity Error
 *  - \b LIN_SCI_INT_OVERRUN  - Overrun Error
 *  - \b LIN_SCI_INT_FRAME    - Framing Error
 *
 */
static inline void
LIN_enableSCIInterrupt(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Set specified interrupts */
    HW_WR_REG32_RAW((base + CSL_LIN_SCISETINT), (HW_RD_REG32_RAW(base + CSL_LIN_SCISETINT)|intFlags));
}

/**
 *  \brief  Disable SCI interrupts
 *
 *  \param base is the LIN module base address
 *  \param intFlags is the bit mask of the interrupt sources to be disabled.
 *
 *  In SCI mode only, this function disables the interrupts for the specified
 *  interrupt sources.
 *
 *  The \e intFlags parameter can be set to the following value to disable
 *  all the flag bits:
 *  - \b LIN_SCI_INT_ALL     - All Interrupts
 *
 *
 *  To disable individual flags, the \e intFlags parameter can be the logical
 *  OR of any of the following:
 *  - \b LIN_SCI_INT_BREAK    - Break Detect
 *  - \b LIN_SCI_INT_WAKEUP   - Wakeup
 *  - \b LIN_SCI_INT_TX       - Transmit Buffer
 *  - \b LIN_SCI_INT_RX       - Receive Buffer
 *  - \b LIN_SCI_INT_TX_DMA   - DMA Transmit
 *  - \b LIN_SCI_INT_RX_DMA   - DMA Receive
 *  - \b LIN_SCI_INT_PARITY   - Parity Error
 *  - \b LIN_SCI_INT_OVERRUN  - Overrun Error
 *  - \b LIN_SCI_INT_FRAME    - Framing Error
 *
 */
static inline void
LIN_disableSCIInterrupt(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Set specified interrupts */
    HW_WR_REG32_RAW((base + CSL_LIN_SCICLEARINT), (HW_RD_REG32_RAW(base + CSL_LIN_SCICLEARINT)|intFlags));
}

/**
 *  \brief  Clear SCI interrupt status
 *
 *  \param base is the LIN module base address
 *  \param intFlags is the bit mask of the interrupt sources to be cleared.
 *
 *  In SCI mode only, this function clears the specified status flags.
 *
 *  The \e intFlags parameter can be set to the following value to clear
 *  all the flag bits:
 *  - \b LIN_SCI_INT_ALL     - All Interrupts
 *
 *
 *  To clear individual flags, the \e intFlags parameter can be the logical
 *  OR of any of the following:
 *  - \b LIN_SCI_INT_BREAK    - Break Detect
 *  - \b LIN_SCI_INT_WAKEUP   - Wakeup
 *  - \b LIN_SCI_INT_TX       - Transmit Buffer
 *  - \b LIN_SCI_INT_RX       - Receive Buffer
 *  - \b LIN_SCI_INT_TX_DMA   - DMA Transmit
 *  - \b LIN_SCI_INT_RX_DMA   - DMA Receive
 *  - \b LIN_SCI_INT_PARITY   - Parity Error
 *  - \b LIN_SCI_INT_OVERRUN  - Overrun Error
 *  - \b LIN_SCI_INT_FRAME    - Framing Error
 *
 */
static inline void
LIN_clearSCIInterruptStatus(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Set specified interrupts */
    HW_WR_REG32_RAW((base + CSL_LIN_SCIFLR), (HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR)|intFlags));
}

/**
 *  \brief  Set interrupt level to 0
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of interrupt sources to be configured
 *
 *  In SCI mode only, this function sets the specified interrupt sources to
 *  level 0.
 *
 *   The \e intFlags parameter can be set to the following value to set
 *   all the flag bits:
 *   - \b LIN_SCI_INT_ALL     - All Interrupts
 *
 *   To set individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *  - \b LIN_SCI_INT_BREAK    - Break Detect
 *  - \b LIN_SCI_INT_WAKEUP   - Wakeup
 *  - \b LIN_SCI_INT_TX       - Transmit Buffer
 *  - \b LIN_SCI_INT_RX       - Receive Buffer
 *  - \b LIN_SCI_INT_TX_DMA   - DMA Transmit
 *  - \b LIN_SCI_INT_RX_DMA   - DMA Receive
 *  - \b LIN_SCI_INT_PARITY   - Parity Error
 *  - \b LIN_SCI_INT_OVERRUN  - Overrun Error
 *  - \b LIN_SCI_INT_FRAME    - Framing Error
 *
 *
 */
static inline void
LIN_setSCIInterruptLevel0(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Clear Status Flags */
    HW_WR_REG32_RAW((base + CSL_LIN_SCICLEARINTLVL), (HW_RD_REG32_RAW(base + CSL_LIN_SCICLEARINTLVL)|intFlags));
}

/**
 *  \brief  Set interrupt level to 1
 *
 *  \param  base is the LIN module base address
 *  \param  intFlags is the bit mask of interrupt sources to be configured
 *
 *  In SCI mode only, this function sets the specified interrupt sources to
 *  level 1.
 *
 *   The \e intFlags parameter can be set to the following value to set
 *   all the flag bits:
 *   - \b LIN_SCI_INT_ALL     - All Interrupts
 *
 *   To set individual flags, the \e intFlags parameter can be the logical
 *   OR of any of the following:
 *  - \b LIN_SCI_INT_BREAK    - Break Detect
 *  - \b LIN_SCI_INT_WAKEUP   - Wakeup
 *  - \b LIN_SCI_INT_TX       - Transmit Buffer
 *  - \b LIN_SCI_INT_RX       - Receive Buffer
 *  - \b LIN_SCI_INT_TX_DMA   - DMA Transmit
 *  - \b LIN_SCI_INT_RX_DMA   - DMA Receive
 *  - \b LIN_SCI_INT_PARITY   - Parity Error
 *  - \b LIN_SCI_INT_OVERRUN  - Overrun Error
 *  - \b LIN_SCI_INT_FRAME    - Framing Error
 *
 */
static inline void
LIN_setSCIInterruptLevel1(uint32_t base, uint32_t intFlags)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Set interrupt levels to 1 */
    HW_WR_REG32_RAW((base + CSL_LIN_SCISETINTLVL), HW_RD_REG32_RAW(base + CSL_LIN_SCISETINTLVL)|intFlags);
}

/**
 *  \brief  Check if SCI Receiver is Idle
 *
 *  \param  base is the LIN module base address
 *
 *  In SCI mode only, this function checks if the receiver is in an idle state.
 *  level 1.
 *
 *  \return  Returns \b true if the state is idle, else returns \b false.
 *
 */
static inline Bool
LIN_isSCIReceiverIdle(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Read Rx Idle flag and return status */
    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_IDLE_MASK) == 0U);
}

/**
 *  \brief  Gets the SCI Transmit Frame Type
 *
 *  \param  base is the LIN module base address
 *
 *  In SCI mode only, this function gets the transmit frame type which can be
 *  either data or an address.
 *
 *  \return  Returns \b true if the frame will be an address, and returns
 *  \b false if the frame will be data.
 *
 */
static inline Bool
LIN_getSCITxFrameType(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Read Rx Idle flag and return status */
    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_WAKEUP_MASK) == CSL_LIN_SCIFLR_WAKEUP_MASK);
}

/**
 *  \brief  Gets the SCI Receiver Frame Type
 *
 *  \param  base is the LIN module base address
 *
 *  In SCI mode only, this function gets the receiver frame type which can be
 *  either an address or not an address.
 *
 *  \return  Returns \b true if the frame is an address, and returns
 *  \b false if the frame isn't an address.
 *
 */
static inline Bool
LIN_getSCIRxFrameType(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Read Rx Idle flag and return status */
    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_RXWAKE_MASK) == CSL_LIN_SCIFLR_RXWAKE_MASK);
}

/**
 *  \brief  Check if SCI Detected a Break Condition
 *
 *  \param  base is the LIN module base address
 *
 *  In SCI mode only, this function checks if the module detected a break
 *  condition on the Rx pin.
 *
 *  \return  Returns \b true if break detected, else returns \b false.
 *
 */
static inline Bool
LIN_isSCIBreakDetected(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert((HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LINMODE_MASK) ==
           0U);

    /* Read Rx Idle flag and return status */
    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_BRKDT_MASK) == CSL_LIN_SCIFLR_BRKDT_MASK);
}

/**
 *  \brief  Enables the LIN module.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN and SCI mode, this function sets the RESET bit of the SCIGCR0
 *  register. Registers in this module are not writable until this has been
 *  done. Additionally, the transmit and receive pin control functionality is
 *  enabled.
 *
 */
static inline void
LIN_enableModule(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set reset bit. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR0), CSL_LIN_SCIGCR0_RESET_MASK, CSL_LIN_SCIGCR0_RESET_SHIFT, CSL_TRUE);

    /* Enable TX and RX pin control functionality. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIPIO0), CSL_LIN_SCIPIO0_RXFUNC_MASK, CSL_LIN_SCIPIO0_RXFUNC_SHIFT, CSL_TRUE);
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIPIO0), CSL_LIN_SCIPIO0_TXFUNC_MASK, CSL_LIN_SCIPIO0_TXFUNC_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable the LIN module.
 *
 *  \param  base is the LIN module base address
 *
 *  In LIN and SCI mode, this function clears the RESET bit of the SCIGCR0
 *  register. Registers in this module are not writable when this bit is
 *  cleared. Additionally, the transmit and receive pin control functionality
 *  is disabled.
 *
 */
static inline void
LIN_disableModule(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Disable TX and RX pin control functionality. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIPIO0), CSL_LIN_SCIPIO0_RXFUNC_MASK, CSL_LIN_SCIPIO0_RXFUNC_SHIFT, CSL_FALSE);
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIPIO0), CSL_LIN_SCIPIO0_TXFUNC_MASK, CSL_LIN_SCIPIO0_TXFUNC_SHIFT, CSL_FALSE);

    /* Reset reset bit. */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR0), CSL_LIN_SCIGCR0_RESET_MASK, CSL_LIN_SCIGCR0_RESET_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Set Baud Rate Prescaler.
 *
 *  \param base is the LIN module base address
 *  \param prescaler is the 24-bit integer prescaler
 *  \param divider is the 4-bit fractional divider
 *
 *  In LIN and SCI mode, this function is used to set the baudrate based on
 *  the \e prescaler and \e divider values.
 *
 *  P = Prescaler  \n
 *  M = Fractional Divider \n
 *  Bitrate = (SYSCLOCK) / ((P + 1 + M/16) * 16) \n
 *
 */
static inline void
LIN_setBaudRatePrescaler(uint32_t base, uint32_t prescaler,
                         uint32_t divider)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert(prescaler <= (CSL_LIN_BRSR_SCI_LIN_PSL_MASK | CSL_LIN_BRSR_SCI_LIN_PSH_MASK));
    DebugP_assert(divider <= (CSL_LIN_BRSR_M_MASK >> CSL_LIN_BRSR_M_SHIFT));

    /* Set baud rate prescaler and divider. */
    HW_WR_REG32_RAW((base + CSL_LIN_BRSR), (prescaler | (divider << CSL_LIN_BRSR_M_SHIFT)));
}

/**
 *  \brief  Enable Transmit Data Transfer.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function enables the transfer of data from
 *  SCITD or TDy to the transmit shift register.
 *
 */
static inline void
LIN_enableDataTransmitter(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Enable transmit bit. */
    HW_WR_REG32_RAW((base + CSL_LIN_SCIGCR1), HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1)|CSL_LIN_SCIGCR1_TXENA_MASK);
}

/**
 *  \brief  Disable Transmit Data Transfer.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function disables the transfer of data from
 *  SCITD or TDy to the transmit shift register.
 *
 */
static inline void
LIN_disableDataTransmitter(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Enable transmit bit. */
    HW_WR_REG32_RAW((base + CSL_LIN_SCIGCR1), HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1)&~(CSL_LIN_SCIGCR1_TXENA_MASK));
}

/**
 *  \brief  Enable Receive Data Transfer.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function enables the receiver to transfer data
 *  from the shift buffer register to the receive buffer or multi-buffer.
 *
 */
static inline void
LIN_enableDataReceiver(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Enable receive bit. */
    HW_WR_REG32_RAW((base + CSL_LIN_SCIGCR1), HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1)|CSL_LIN_SCIGCR1_RXENA_MASK);
}

/**
 *  \brief  Disable Receive Data Transfer.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function disables the receiver to transfer data
 *  from the shift buffer register to the receive buffer or multi-buffer.
 *
 */
static inline void
LIN_disableDataReceiver(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Disable receive bit. */
    HW_WR_REG32_RAW((base + CSL_LIN_SCIGCR1), HW_RD_REG32_RAW(base + CSL_LIN_SCIGCR1)&~(CSL_LIN_SCIGCR1_RXENA_MASK));
}

/**
 *  \brief  Perform software reset.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function will reset the LIN state machine and
 *  clear all pending flags. It is required to call this function after a
 *  wakeup signal has been sent.
 *
 *  To enter the reset state separately, use LIN_enterSoftwareReset(). To come
 *  out of reset, use LIN_exitSoftwareReset().
 *
 */
static inline void
LIN_performSoftwareReset(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_SWNRST_MASK, CSL_LIN_SCIGCR1_SWNRST_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Put LIN into its reset state.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function will reset the LIN state machine and
 *  clear all pending flags. It is required to call this function after a
 *  wakeup signal has been sent. When in this state, changes to the
 *  configuration of this module may be made.
 *
 *  To take LIN out of the reset state and back into the ready state, use
 *  LIN_exitSoftwareReset().
 *
 */
static inline void
LIN_enterSoftwareReset(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_SWNRST_MASK, CSL_LIN_SCIGCR1_SWNRST_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Put LIN into its ready state.
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function will put LIN into its ready state.
 *  Transmission and reception can be done in this state. While in the ready
 *  state, configuration of the module should not be changed.
 *
 *  To put the module into its reset state, use LIN_enterSoftwareReset().
 *
 */
static inline void
LIN_exitSoftwareReset(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_SWNRST_MASK, CSL_LIN_SCIGCR1_SWNRST_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Check if Bus is Busy
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function checks if the receiver bus is busy
 *  receiving a frame.
 *
 *  \return  Returns \b true if the bus is busy, else returns \b false.
 *
 */
static inline Bool
LIN_isBusBusy(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_BUSY_MASK) == CSL_LIN_SCIFLR_BUSY_MASK);
}

/**
 *  \brief  Check if the Transmit Buffer is Empty
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function checks if the transmit buffer is empty
 *  or not.
 *
 *  \return  Returns \b true if the Tx buffer is empty, else returns \b false.
 *
 */
static inline Bool
LIN_isTxBufferEmpty(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR) & CSL_LIN_SCIFLR_TXEMPTY_MASK) == CSL_LIN_SCIFLR_TXEMPTY_MASK);
}

/**
 *  \brief  Enable External Loopback mode for self test
 *
 *  \param base is the LIN module base address
 *  \param loopbackType is the loopback type (analog or digital)
 *  \param path sets the transmit or receive pin to be included in the
 *  communication path (Analog loopback mode only)
 *
 *  In LIN and SCI mode, this function enables the external Loopback mode for
 *  self test. The \e loopbackType parameter can be one of the following
 *  values:
 *  - \b LIN_LOOPBACK_DIGITAL - Digital Loopback
 *  - \b LIN_LOOPBACK_ANALOG  - Analog Loopback
 *
 *  The \e path parameter is only applicable in analog loopback mode and can
 *  be one of the following values:
 *  - \b LIN_ANALOG_LOOP_NONE - Default option for digital loopback mode
 *  - \b LIN_ANALOG_LOOP_TX   - Enables analog loopback through the Tx pin
 *  - \b LIN_ANALOG_LOOP_RX   - Enables analog loopback through the Rx pin
 *
 */
static inline void
LIN_enableExtLoopback(uint32_t base, LIN_LoopbackType loopbackType,
                      LIN_AnalogLoopback path)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IO_DFT_KEY);

    /* Set loopback Type */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_LPBENA_MASK, CSL_LIN_IODFTCTRL_LPBENA_SHIFT, loopbackType);

    /* Set Analog Loopback Path */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_RXPENA_MASK, CSL_LIN_IODFTCTRL_RXPENA_SHIFT, path);
}

/**
 *  \brief  Disable External Loopback mode for self test
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function disables the external Loopback mode.
 *
 *  \note This function also resets the analog loopback communication path to
 *  the default transmit pin.
 *
 */
static inline void
LIN_disableExtLoopback(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set loopback Type */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_LPBENA_MASK, CSL_LIN_IODFTCTRL_LPBENA_SHIFT, CSL_FALSE);

    /* Set Analog Loopback Path */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_RXPENA_MASK, CSL_LIN_IODFTCTRL_RXPENA_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Enable Internal Loopback mode for self test
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function enables the internal Loopback mode for
 *  self test.
 *
 */
static inline void
LIN_enableIntLoopback(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set loopback Type */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_LOOPBACK_MASK, CSL_LIN_SCIGCR1_LOOPBACK_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable Internal Loopback mode for self test
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function disables the internal Loopback mode for
 *  self test.
 *
 */
static inline void
LIN_disableIntLoopback(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Set loopback Type */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_LOOPBACK_MASK, CSL_LIN_SCIGCR1_LOOPBACK_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Get Interrupt Flags Status
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function returns the interrupt status register.
 *  The following flags can be used to mask the value returned:
 *  - \b LIN_FLAG_BREAK   - Break Detect Flag (SCI mode only)
 *  - \b LIN_FLAG_WAKEUP  - Wake-up Flag
 *  - \b LIN_FLAG_IDLE    - Receiver in Idle State (SCI mode only)
 *  - \b LIN_FLAG_BUSY    - Busy Flag
 *  - \b LIN_FLAG_TO      - Bus Idle Timeout Flag (LIN mode only)
 *  - \b LIN_FLAG_TOAWUS  - Timeout after Wakeup Signal (LIN mode only)
 *  - \b LIN_FLAG_TOA3WUS - Timeout after 3 Wakeup Signals (LIN mode only)
 *  - \b LIN_FLAG_TXRDY   - Transmitter Buffer Ready Flag
 *  - \b LIN_FLAG_RXRDY   - Receiver Buffer Ready Flag
 *  - \b LIN_FLAG_TXWAKE  - Transmitter Wakeup Method Select (SCI mode only)
 *  - \b LIN_FLAG_TXEMPTY - Transmitter Empty Flag
 *  - \b LIN_FLAG_RXWAKE  - Receiver Wakeup Detect Flag
 *  - \b LIN_FLAG_TXID    - Identifier on Transmit Flag (LIN mode only)
 *  - \b LIN_FLAG_RXID    - Identifier on Receive Flag (LIN mode only)
 *  - \b LIN_FLAG_PE      - Parity Error Flag
 *  - \b LIN_FLAG_OE      - Overrun Error Flag
 *  - \b LIN_FLAG_FE      - Framing Error Flag
 *  - \b LIN_FLAG_NRE     - No-Response Error Flag (LIN mode only)
 *  - \b LIN_FLAG_ISFE    - Inconsistent Synch Field Error Flag (LIN mode only)
 *  - \b LIN_FLAG_CE      - Checksum Error Flag (LIN mode only)
 *  - \b LIN_FLAG_PBE     - Physical Bus Error Flag (LIN mode only)
 *  - \b LIN_FLAG_BE      - Bit Error Flag (LIN mode only)
 *
 *  \return Returns the status flag register.
 *
 */
static inline uint32_t
LIN_getInterruptStatus(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read and return the flag register */
    return(HW_RD_REG32_RAW(base + CSL_LIN_SCIFLR));
}

/**
 *  \brief  Get the Interrupt Level
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function gets the interrupt level status for
 *  all interrupt sources.
 *
 *  \return Returns the value of the interrupt level register.
 *
 */
static inline uint32_t
LIN_getInterruptLevel(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read and return the flag register */
    return(HW_RD_REG32_RAW(base + CSL_LIN_SCISETINTLVL));
}

/**
 *  \brief  Gets the Interrupt Vector Offset for Line 0
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function gets the offset for interrupt line 0.
 *  A read to the specified line register updates its value to the next highest
 *  priority pending interrupt in the flag register and clears the flag
 *  corresponding to the offset that was read.
 *
 *  \note The flags for the receive and the transmit interrupts cannot be
 *  cleared by reading the corresponding offset vector in this function.
 *
 *  The following are values that can be returned:
 *  - \b LIN_VECT_NONE    - No Interrupt
 *  - \b LIN_VECT_WAKEUP  - Wakeup
 *  - \b LIN_VECT_ISFE    - Inconsistent-sync-field Error
 *  - \b LIN_VECT_PE      - Parity Error
 *  - \b LIN_VECT_ID      - ID Interrupt
 *  - \b LIN_VECT_PBE     - Physical Bus Error
 *  - \b LIN_VECT_FE      - Frame Error
 *  - \b LIN_VECT_BREAK   - Break detect
 *  - \b LIN_VECT_CE      - Checksum Error
 *  - \b LIN_VECT_OE      - Overrun Error
 *  - \b LIN_VECT_BE      - Bit Error
 *  - \b LIN_VECT_RX      - Receive Interrupt
 *  - \b LIN_VECT_TX      - Transmit Interrupt
 *  - \b LIN_VECT_NRE     - No-response Error
 *  - \b LIN_VECT_TOAWUS  - Timeout after wakeup signal
 *  - \b LIN_VECT_TOA3WUS - Timeout after 3 wakeup signals
 *  - \b LIN_VECT_TO      - Timeout (Bus Idle)
 *
 *  \return Returns the value of the interrupt level register.
 *
 */
static inline uint16_t
LIN_getInterruptLine0Offset(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read and return the flag register */
    return(HW_RD_REG32_RAW(base + CSL_LIN_SCIINTVECT0) & CSL_LIN_SCIINTVECT0_INTVECT0_MASK);
}

/**
 *  \brief  Gets the Interrupt Vector Offset for Line 1
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function gets the offset for interrupt line 1.
 *  A read to the specified line register updates its value to the next highest
 *  priority pending interrupt in the flag register and clears the flag
 *  corresponding to the offset that was read.
 *
 *  \note The flags for the receive and the transmit interrupts cannot be
 *  cleared by reading the corresponding offset vector in this function.
 *
 *  The following are values that can be returned:
 *  - \b LIN_VECT_NONE    - No Interrupt
 *  - \b LIN_VECT_WAKEUP  - Wakeup
 *  - \b LIN_VECT_ISFE    - Inconsistent-sync-field Error
 *  - \b LIN_VECT_PE      - Parity Error
 *  - \b LIN_VECT_ID      - ID Interrupt
 *  - \b LIN_VECT_PBE     - Physical Bus Error
 *  - \b LIN_VECT_FE      - Frame Error
 *  - \b LIN_VECT_BREAK   - Break detect
 *  - \b LIN_VECT_CE      - Checksum Error
 *  - \b LIN_VECT_OE      - Overrun Error
 *  - \b LIN_VECT_BE      - Bit Error
 *  - \b LIN_VECT_RX      - Receive Interrupt
 *  - \b LIN_VECT_TX      - Transmit Interrupt
 *  - \b LIN_VECT_NRE     - No-response Error
 *  - \b LIN_VECT_TOAWUS  - Timeout after wakeup signal
 *  - \b LIN_VECT_TOA3WUS - Timeout after 3 wakeup signals
 *  - \b LIN_VECT_TO      - Timeout (Bus Idle)
 *
 *  \return Returns the interrupt vector offset for interrupt line 1.
 *
 */
static inline uint16_t
LIN_getInterruptLine1Offset(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read and return the flag register */
    return(HW_RD_REG32_RAW(base + CSL_LIN_SCIINTVECT1) & CSL_LIN_SCIINTVECT1_INTVECT1_MASK);
}

/**
 *  \brief  Enable Multi-buffer Mode
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function enables the multi-buffer mode.
 *
 */
static inline void
LIN_enableMultibufferMode(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read and return the flag register */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_MBUFMODE_MASK, CSL_LIN_SCIGCR1_MBUFMODE_SHIFT, CSL_TRUE);
}

/**
 *  \brief  Disable Multi-buffer Mode
 *
 *  \param base is the LIN module base address
 *
 *  In LIN and SCI mode, this function disables the multi-buffer mode.
 *
 */
static inline void
LIN_disableMultibufferMode(uint32_t base)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Read and return the flag register */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_MBUFMODE_MASK, CSL_LIN_SCIGCR1_MBUFMODE_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Set Transmit Pin Delay
 *
 *  \param base is the LIN module base address
 *  \param delay is number of clock delays for the Tx pin (0 to 7)
 *
 *  In LIN and SCI mode, this function sets the delay by which the value on
 *  the transmit pin is delayed so that the value on the receive pin is
 *  asynchronous.
 *
 *  \note This is not applicable to the Start bit.
 *
 */
static inline void
LIN_setTransmitDelay(uint32_t base, uint16_t delay)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));
    DebugP_assert(delay < 8U);

    /* Clear and Set the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IO_DFT_KEY);

    /* Set the delay value */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_TXSHIFT_MASK, CSL_LIN_IODFTCTRL_TXSHIFT_SHIFT, delay);

    /* Clear the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Set Pin Sample Mask
 *
 *  \param base is the LIN module base address
 *  \param mask is the pin sample mask to be set
 *
 * In LIN and SCI mode, this function sets sample number at which the transmit
 * pin value that is being transmitted will be inverted to verify the
 * receive pin samples correctly with the majority detection circuitry.
 * The \e mask parameter can be one of the following values:
 *   - \b LIN_PINMASK_NONE         - No mask
 *   - \b LIN_PINMASK_CENTER       - Invert Tx Pin value at T-bit center
 *   - \b LIN_PINMASK_CENTER_SCLK  - Invert Tx Pin value at T-bit center + SCLK
 *   - \b LIN_PINMASK_CENTER_2SCLK - Invert Tx Pin value at T-bit center + 2 SCLK
 *
 */
static inline void
LIN_setPinSampleMask(uint32_t base, LIN_PinSampleMask mask)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear and Set the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, LIN_IO_DFT_KEY);

    /* Set the delay value */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_PINSAMPLEMASK_MASK, CSL_LIN_IODFTCTRL_PINSAMPLEMASK_SHIFT, mask);

    /* Clear the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_IODFTCTRL), CSL_LIN_IODFTCTRL_IODFTENA_MASK, CSL_LIN_IODFTCTRL_IODFTENA_SHIFT, CSL_FALSE);
}

/**
 *  \brief  Set the Debug Suspended Mode
 *
 *  \param base is the LIN module base address
 *  \param mode is the debug mode
 *
 * In LIN and SCI mode, this function sets how the module operates when the
 * program is suspended and being debugged with an emulator. The \e mode
 * parameter can be one of the following values:
 * - \b LIN_DEBUG_FROZEN - The module state machine is frozen; transmissions
 * and LIN counters are halted until debug mode is exited.
 * - \b LIN_DEBUG_COMPLETE - The module continues to operate until the
 * current transmit and receive functions are complete.
 *
 */
static inline void
LIN_setDebugSuspendMode(uint32_t base, LIN_DebugMode mode)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    /* Clear and Set the IO DFT Enable Key */
    HW_WR_FIELD32_RAW((base + CSL_LIN_SCIGCR1), CSL_LIN_SCIGCR1_CONT_MASK, CSL_LIN_SCIGCR1_CONT_SHIFT, mode);
}

/**
 *  \brief  Enables a LIN global interrupt.
 *
 *  \param base is the LIN module base address
 *  \param line is specified interrupt vector line
 *
 *  In LIN and SCI mode, this function globally enables an interrupt
 *  corresponding to a specified interrupt line. The \e line parameter can be
 *  one of the following enumerated values:
 *
 *  - \b LIN_INTERRUPT_LINE0      - Interrupt Vector Line 0
 *  - \b LIN_INTERRUPT_LINE1      - Interrupt Vector Line 1
 *
 */
static inline void
LIN_enableGlobalInterrupt(uint32_t base, LIN_InterruptLine line)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_REG32_RAW((base + CSL_LIN_LIN_GLB_INT_EN), HW_RD_REG32_RAW(base + CSL_LIN_LIN_GLB_INT_EN)|(CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK<<(uint16_t)line));
}

/**
 *  \brief  Disables a LIN global interrupt.
 *
 *  \param base is the LIN module base address
 *  \param line is specified interrupt vector line
 *
 *  In LIN and SCI mode, this function globally disables an interrupt
 *  corresponding to a specified interrupt line. The \e line parameter can be
 *  one of the following enumerated values:
 *
 *  - \b LIN_INTERRUPT_LINE0      - Interrupt Vector Line 0
 *  - \b LIN_INTERRUPT_LINE1      - Interrupt Vector Line 1
 *
 */
static inline void
LIN_disableGlobalInterrupt(uint32_t base, LIN_InterruptLine line)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_REG32_RAW((base + CSL_LIN_LIN_GLB_INT_EN), HW_RD_REG32_RAW(base + CSL_LIN_LIN_GLB_INT_EN)&(~(CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK<<(uint16_t)line)));
}

/**
 *  \brief  Clears a LIN global interrupt flag.
 *
 *  \param base is the LIN module base address
 *  \param line is specified interrupt vector line
 *
 *  In LIN and SCI mode, this function clears the global interrupt flag that
 *  corresponds to a specified interrupt line. The \e line parameter can be
 *  one of the following enumerated values:
 *
 *  - \b LIN_INTERRUPT_LINE0      - Interrupt Vector Line 0
 *  - \b LIN_INTERRUPT_LINE1      - Interrupt Vector Line 1
 *
 */
static inline void
LIN_clearGlobalInterruptStatus(uint32_t base, LIN_InterruptLine line)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    HW_WR_REG32_RAW((base + CSL_LIN_LIN_GLB_INT_CLR), (CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK<<(uint16_t)line));
}

/**
 *  \brief  Returns a LIN global interrupt flag status.
 *
 *  \param base is the LIN module base address
 *  \param line is specified interrupt vector line
 *
 *  In LIN and SCI mode, this function returns the status of a global interrupt
 *  flag that corresponds to a specified interrupt line. The \e line parameter
 *  can be one of the following enumerated values:
 *
 *  - \b LIN_INTERRUPT_LINE0      - Interrupt Vector Line 0
 *  - \b LIN_INTERRUPT_LINE1      - Interrupt Vector Line 1
 *
 *  \return Returns \b true if the interrupt flag is set. Return \b false if not.
 *
 */
static inline Bool
LIN_getGlobalInterruptStatus(uint32_t base, LIN_InterruptLine line)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    return((HW_RD_REG32_RAW(base + CSL_LIN_LIN_GLB_INT_FLG) & (CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK<<(uint16_t)line))
                                                        == (CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK<<(uint16_t)line));
}

/**
 *  \brief  Get the Status of LIN TX/RX Pin status.
 *
 *  \param base is the LIN module base address
 *  \param pin is the type of pin that is being read.
 *
 *  In LIN and SCI mode, this function indicates the current status of LINTX
 *  and LINRX pins. the \e pin parameter can be either LIN_PINTYPE_TX or
 *  LIN_PINTYPE_RX.
 *
 *  \return Returns \b true if the pin is high else \b false if low.
 *
 */
static inline Bool
LIN_getPinStatus(uint32_t base, LIN_PinType pin)
{
    /* Check the arguments. */
    DebugP_assert(LIN_isBaseValid(base));

    return((HW_RD_REG32_RAW(base + CSL_LIN_SCIPIO2) & pin) == pin);
}

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef LIN_V0_H_ */

/** @} */
