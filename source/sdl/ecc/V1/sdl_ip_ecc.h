/**
 * @file  V1/sdl_ip_ecc.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the ECC Aggregator Ip.
 *
 *
 *  @n   (C) Copyright 2022, Texas Instruments, Inc.
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
 *
*/
#ifndef SDL_ECC_AGGR_H
#define SDL_ECC_AGGR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <sdl/ecc/sdlr_ecc.h>

/**
 * @ingroup SDL_IP_MODULE
 * @defgroup SDL_ECC_AGGR_API APIs for SDL ECC (ECC_AGGR)
 *
 *
 * To increase functional safety and system reliability the memories (for example,
 * FIFOs, queues, SRAMs and others) in many device modules and subsystems are
 * protected by error correcting code (ECC). This is accomplished through an
 * ECC aggregator and ECC wrapper. The ECC aggregator is connected to
 * these memories (hereinafter ECC RAMs) and involved in the ECC process.
 * Each memory is surrounded by an ECC wrapper which performs the ECC detection
 * and correction. The wrapper communicates via serial interface with the
 * aggregator which has memory mapped configuration interface.
 * The ECC aggregator is also connected to interconnect ECC components that
 * protect the command, address and data buses of the system interconnect.
 * ECC is calculated for the data bus and parity and redundancy for the command
 * and address buses. Each interconnect ECC component has the same serial
 * interface for communication with the aggregator as the ECC wrapper.
 * An ECC aggregator may be connected to both endpoints the ECC wrapper and
 * interconnect ECC component.
 * The ECC aggregator, ECC wrapper and interconnect ECC component are considered
 * as single entity and are hereinafter referred to as ECC aggregator unless
 * otherwise explicitly specified.
 * The design focusses on SDL function layer providing the configuration for
 * ECC RAM ID, force ECC ram error, ECC aggregator interrupt handling features.
 *
 */
/**
@defgroup SDL_ECC_AGGR_DATASTRUCT  ECC_AGGR Data Structures
@ingroup SDL_ECC_AGGR_API
*/
/**
@defgroup SDL_ECC_AGGR_FUNCTION  ECC_AGGR Functions
@ingroup SDL_ECC_AGGR_API
*/
/**
@defgroup SDL_ECC_AGGR_ENUM ECC_AGGR Enumerated Data Types
@ingroup SDL_ECC_AGGR_API
*/
/**
@defgroup SDL_ECC_AGGR_MACROS ECC_AGGR Macros
@ingroup SDL_ECC_AGGR_API
*/
/**
 *  @addtogroup SDL_ECC_AGGR_MACROS
    @{
 *
 */
/** No interrupt */
#define SDL_ECC_AGGR_INTR_SRC_NONE                      ((uint32_t) 0U)
/** Single-bit Error Correcting (SEC) */
#define SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT                ((uint32_t) 1U)
/** Double-bit Error Detection (DED) */
#define SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT                ((uint32_t) 2U)
/** Two or more successive SEC errors */
#define SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS  ((uint32_t) 3U)
/** Denotes an invalid interrupt source */
#define SDL_ECC_AGGR_INTR_SRC_INVALID                   ((uint32_t) 4U)

/**
 * @brief This defines the types of possible ECC error controller instances
 *
 *
 */
/** Error controller instance 1 */
#define SDL_ECC_AGGR_SELECT_ERR_CTRL1                   (0U)
/** Error Controller instance 2 */    
#define SDL_ECC_AGGR_SELECT_ERR_CTRL2                   (1U)
/** Maximum number of RAM Error Controller registers */    
#define SDL_ECC_AGGR_MAX_NUM_RAM_ERR_CTRL               (2U)
    
/**
 * @brief This defines the types of possible ECC error status instances
 *
 *
 */
/** Error Status instance 1 */
#define SDL_ECC_AGGR_SELECT_ERR_STAT1                   (0U)
/** Error Status instance 2 */
#define SDL_ECC_AGGR_SELECT_ERR_STAT2                   (1U)
/** Error Status instance 3 */
#define SDL_ECC_AGGR_SELECT_ERR_STAT3                   (2U)
/** Maximum number of RAM Error Status registers */    
#define SDL_ECC_AGGR_MAX_NUM_RAM_ERR_STAT               (3U)
    

/**
 * @brief This defines the number of enable registers
 *
 *
 */
#define SDL_ECC_AGGR_NUM_ENABLE_REGISTERS               (8U)
/** Valid Timeout Error parameter */
#define SDL_ECC_AGGR_VALID_TIMEOUT_ERR                  (1U)
/** Valid Timeout Error parameter */
#define SDL_ECC_AGGR_VALID_PARITY_ERR                   (2U)

/** Zero inject pattern */
#define SDL_ECC_AGGR_INJECT_PATTERN_ZERO                ((uint32_t) 0U)
/** Inject pattern 0xF */
#define SDL_ECC_AGGR_INJECT_PATTERN_F                   ((uint32_t) 1U)
/** Inject pattern 0xA */
#define SDL_ECC_AGGR_INJECT_PATTERN_A                   ((uint32_t) 2U)
/** Inject pattern 0x5 */
#define SDL_ECC_AGGR_INJECT_PATTERN_5                   ((uint32_t) 3U)
/* Max Inject pattern */
#define SDL_ECC_EGGR_INJECT_PATTERN_MAX                 (SDL_ECC_AGGR_INJECT_PATTERN_A)

/** Normal errors */
#define SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL               ((uint32_t) 0U)
/** Inject errors */
#define SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT               ((uint32_t) 1U)

/** @} */

/**
 *  @addtogroup SDL_ECC_AGGR_ENUM
    @{
 *
 */

/**
 * @brief This enumerator defines the types of possible ECC errors
 *
 *
 */

/**
 * Design: PROC_SDL-1272
 */
typedef uint32_t SDL_Ecc_AggrIntrSrc;


/**
 * @brief This enumerator defines the types of possible EDC errors
 *
 * Design: PROC_SDL-1274
 *
 */
typedef uint32_t SDL_Ecc_AggrEDCErrorSubType;


/**
 * @brief This defines the valid ecc aggr error configuration
 *
 *
 */
typedef uint8_t  SDL_ecc_aggrValid;

/**
 * @brief This enumerator defines the types of ECC patterns
 *
 * Design: PROC_SDL-1273
 *
 */
typedef uint32_t SDL_Ecc_injectPattern;
/** @} */

/**
 *  @addtogroup SDL_ECC_AGGR_DATASTRUCT
    @{
 *
 */

/**
 * @brief   This structure contains error forcing
 *          information used by the SDL_ecc_aggrForceEccRamError function.
 *
 * Design: PROC_SDL-1275
 *
 */
typedef struct
{
    /** Identifies the interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, or SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS) */
    SDL_Ecc_AggrIntrSrc intrSrc;
    /** Row address where SEC or DED needs to be applied (ignored if bNextRow is true). */
    uint32_t            eccRow;
    /** Bit position starting with LSB for SEC or 1st bit position starting with LSB for DED error injection. */
    uint32_t            eccBit1;
    /** 2nd bit position starting from LSB for DED error injection. */
    uint32_t            eccBit2;
    /** If true, the force_sec/force_ded will inject an error to the specified row only once. */
    bool                bOneShotMode;
    /** Force single or double-bit error on the next RAM row (the eccRow field is ignored). */
    bool                bNextRow;
} SDL_Ecc_AggrErrorInfo;

/**
 * @brief   This structure contains error status information returned by the
 *          SDL_ecc_aggrGetEccRamGetErrorStatus function.
 *
 * Design: PROC_SDL-1276
 *
 */
typedef struct
{
    /** Control register bit flip error. */
    bool                controlRegErr;
    /** Successive Single bit error. */
    bool                successiveSingleBitErr;
    /** ECC Serial VBUS timeout error. */
    bool                sVBUSTimeoutErr;
    /** Single bit error writeback pend status. */
    bool                writebackPend;
    /** Parity error count :  0-2 Number of errors, 3 means 3 or more errors. */
    uint32_t             parityErrorCount;
    /** Indicates the row/address where the error occurred. */
    uint32_t            eccRow;
    /** Indicates the first data bit that is in error counting from LSB (Single bit error only). */
    uint32_t            eccBit1;
    /** Single bit Error count : 0-2 Number of errors, 3 means 3 or more errors. */
    uint32_t             singleBitErrorCount;
    /** Double bit Error count : 0-2 Number of errors, 3 means 3 or more errors. */
    uint32_t             doubleBitErrorCount;
} SDL_Ecc_AggrEccRamErrorStatusInfo;

/**
 * @brief   This structure contains the ECC aggr enable error config.
 *
 * Design: PROC_SDL-1279
 *
 */
typedef struct {
    /** timeout interrupt enable */
    bool                        intrEnableTimeoutErr;
    /** parity interrupt enable */
    bool                        intrEnableParityErr;
    /** valid Ecc aggr control */
    SDL_ecc_aggrValid           validCfg;
} SDL_ecc_aggrEnableCtrl;

/**
 * @brief   This structure contains the ECC aggr status config.
 *
 * Design: PROC_SDL-1280
 *
 */
typedef struct {
    /** timeout interrupt status set control, TRUE: Sets the count*/
    bool                        intrStatusSetTimeoutErr;
    /** parity interrupt status set control */
    bool                        intrStatusSetParityErr;
    /** timeout count value */
    uint32_t                    timeOutCnt;
    /** parity error count value */
    uint32_t                    parityCnt;
    /** valid Ecc aggr control */
    SDL_ecc_aggrValid           validCfg;
} SDL_ecc_aggrStatusCtrl;



/**
 * @brief   This structure contains the static register group for Ecc aggregator
 *          used by the SDL_ecc_aggrReadStaticRegs function.
 *
 * Design: PROC_SDL-1281
 *
 */
typedef struct {
    /** Aggregator Revision Register */
    uint32_t REV;
    /** ECC Control register */
    uint32_t ECC_CTRL;
    /** ECC Err Control1 register */
    uint32_t ECC_ERR_CTRL1;
    /** ECC Err Control2 register */
    uint32_t ECC_ERR_CTRL2;
    /** ECC_SEC_ENABLE_SET_REG registers */
    uint32_t ECC_SEC_ENABLE_SET_REG[SDL_ECC_AGGR_NUM_ENABLE_REGISTERS];
    /** ECC_SEC_ENABLE_SET_REG registers */
    uint32_t ECC_SEC_ENABLE_CLR_REG[SDL_ECC_AGGR_NUM_ENABLE_REGISTERS];
    /** ECC_DED_ENABLE_SET_REG registers */
    uint32_t ECC_DED_ENABLE_SET_REG[SDL_ECC_AGGR_NUM_ENABLE_REGISTERS];
    /** ECC_DED_ENABLE_SET_REG registers */
    uint32_t ECC_DED_ENABLE_CLR_REG[SDL_ECC_AGGR_NUM_ENABLE_REGISTERS];
} SDL_ECC_staticRegs;

/** @} */

/**
 *  @addtogroup SDL_ECC_AGGR_FUNCTION
    @{
 *
 */

/**
 *   @n@b SDL_ecc_aggrGetRevision
 *
 *   @b Description
 *   @n This function returns the revision information for the ECC Aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  pRev         A pointer (of type uint32_t) to hold the rev ID
 *                      value of the ECC Aggregator module
 *                      Revision information. See design specification for details.
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *
 *
 */
int32_t SDL_ecc_aggrGetRevision(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t *pRev);

/**
 *   @n@b SDL_ecc_aggrGetNumRams
 *
 *   @b Description
 *   @n This function returns the number of RAMs serviced by this ECC Aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  pNumRams     A pointer (of type uint32_t) to hold the value of
 *                      number of RAMs serviced by this ECC Aggregator
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrGetNumRams(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t *pNumRams);

/**
 *   @n@b SDL_ecc_aggrReadEccRamReg
 *
 *   @b Description
 *   @n This function reads the specified ECC wrapper register from the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  regOffset    Byte offset of the ECC wrapper register to read (must be in the
 *                      range of 0x10..0x24)
 *   @n  pRegVal      A pointer (of uint32_t) to hold the value of the
 *                      specified ECC wrapper register
 *
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the arguments such as ramId and/or regOffset are invalid
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrReadEccRamReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t regOffset, uint32_t *pRegVal);

/**
 *   @n@b SDL_ecc_aggrReadEccRamWrapRevReg
 *
 *   @b Description
 *   @n This function reads the ECC wrapper revision register from the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *
 *   @n  pRegVal A pointer (of type uint32_t) to hold the
 *                value of the ECC wrapper revision register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrReadEccRamWrapRevReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t *pRegVal);

/**
 *   @n@b SDL_ecc_aggrReadEccRamCtrlReg
 *
 *   @b Description
 *   @n This function reads the specified ECC wrapper control register from the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *
 *   @n  pRegVal A pointer (of type uint32_t) to hold the
 *                value of the ECC control register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrReadEccRamCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t *pRegVal);

/**
 *   @n@b SDL_ecc_aggrReadEccRamErrCtrlReg
 *
 *   @b Description
 *   @n This function reads the specified ECC wrapper error control register from the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  instSelect   Instance selector (0..1) of the specified register to read
 *
 *   @n  pRegVal A pointer (of type uint32_t) to hold the
 *                value of the ECC Error control register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 */
int32_t SDL_ecc_aggrReadEccRamErrCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t instSelect, uint32_t *pRegVal);

/**
 *   @n@b SDL_ecc_aggrReadEccRamErrStatReg
 *
 *   @b Description
 *   @n This function reads the specified ECC wrapper error status register from the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  instSelect   Instance selector (0..1) of the specified register to read
 *
 *   @n  pRegVal A pointer (of type uint32_t) to hold the
 *                value of the ECC error status register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 */
int32_t SDL_ecc_aggrReadEccRamErrStatReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t instSelect, uint32_t *pRegVal);

/**
 *   @n@b SDL_ecc_aggrWriteEccRamReg
 *
 *   @b Description
 *   @n This function writes a value into the specified ECC wrapper register in the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  regOffset    Byte offset of the ECC wrapper register to write (must be in the
 *                      range of 0x10..0x24)
 *   @n  val          The 32-bit value to write into the register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrWriteEccRamReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t regOffset, uint32_t val);

/**
 *   @n@b SDL_ecc_aggrWriteEccRamCtrlReg
 *
 *   @b Description
 *   @n This function write a value into the ECC wrapper control register in the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  val          The 32-bit value to write into the register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrWriteEccRamCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t val);

/**
 *   @n@b SDL_ecc_aggrWriteEccRamErrCtrlReg
 *
 *   @b Description
 *   @n This function writes a value into the specified ECC wrapper error control
 *      register in the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  instSelect   Instance selector (0..1) of the specified register to write
 *   @n  val          The 32-bit value to write into the register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrWriteEccRamErrCtrlReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t instSelect, uint32_t val);

/**
 *   @n@b SDL_ecc_aggrWriteEccRamErrStatReg
 *
 *   @b Description
 *   @n This function write a value into the specified ECC wrapper error status register in the specific ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                      address of the ECC Aggregator module
 *   @n  ramId        RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  instSelect   Instance selector (0..1) of the specified register to write
 *   @n  val          The 32-bit value to write into the register
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrWriteEccRamErrStatReg(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, uint32_t instSelect, uint32_t val);

/**
 *   @n@b SDL_ecc_aggrConfigEccRam
 *
 *   @b Description
 *   @n This function is used to configure the ECC capabilities of the specified ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs   A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                        address of the ECC Aggregator module
 *   @n  ramId          RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  bEnable        If true, then enable ECC generation. ECC is completely bypassed if both fEnable and fEccCheck are 0.
 *   @n  bEccCheck      If true, then enable ECC check. ECC is completely bypassed if both fEnable and fEccCheck are 0.
 *   @n  bfEnableRMW    If true, then enable read-modify-write on partial word writes
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrConfigEccRam(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, bool bEnable, bool bEccCheck, bool bEnableRMW);

/**
 *   @n@b SDL_ecc_aggrVerifyConfigEccRam
 *
 *   @b Description
 *   @n This function is used to verify the configure the ECC capabilities of the specified ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs   A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                        address of the ECC Aggregator module
 *   @n  ramId          RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  bEnable        If true, then enable ECC generation. ECC is completely bypassed if both fEnable and fEccCheck are 0.
 *   @n  bEccCheck      If true, then enable ECC check. ECC is completely bypassed if both fEnable and fEccCheck are 0.
 *   @n  bfEnableRMW    If true, then enable read-modify-write on partial word writes
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success, indicates the configuration is successful
 *   @n  SDL_EFAIL     Failure, indicates the expected configuration failed
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  SDL_ecc_aggrConfigEccRam() is called previously with the same arguments
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrVerifyConfigEccRam(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, bool bEnable, bool bEccCheck, bool bEnableRMW);

/**
 *   @n@b SDL_ecc_aggrGetEccRamErrorStatus
 *
 *   @b Description
 *   @n This function is used to get the error status of the specified ECC Wrapper type RAM id
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  pEccErrorStatus    A pointer to a SDL_Ecc_AggrEccRamErrorStatusInfo structure containing
 *                            returned error information
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrGetEccRamErrorStatus(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrEccRamErrorStatusInfo *pEccErrorStatus);

/**
 *   @n@b SDL_ecc_aggrForceEccRamError
 *
 *   @b Description
 *   @n This function is used to force an ECC error on the specified ECC Wrapper type RAM id
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  pEccForceError     A pointer to a SDL_Ecc_AggrErrorInfo structure containing
 *                            force error information
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrForceEccRamError(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, const SDL_Ecc_AggrErrorInfo *pEccForceError);

/**
 *   @n@b SDL_ecc_aggrAckIntr
 *
 *   @b Description
 *   @n This function is used to acknowledged a pending interrupt and to send
 *      the next interrupt to the host
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT or
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) to acknowledge
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrAckIntr(SDL_ecc_aggrRegs *pEccAggrRegs, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrIsEccRamIntrPending
 *
 *   @b Description
 *   @n This function returns the pending interrupt status of the specified interrupt
 *      source from the specified ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT,
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, or SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS)
 *
 *   @n  pIsPend            A Pointer (of type bool) to hold the state as below
 *     true               An interrupt of the specified type is pending
 *     false              An interrupt of the specified type is not pending, or the ramId and/or intrSrc
 *                            arguments are invalid
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrIsEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool *pIsPend);

/**
 *   @n@b SDL_ecc_aggrSetEccRamIntrPending
 *
 *   @b Description
 *   @n This function is used to forceably set the pending status of the
 *      specified interrupt source on the specified ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT,
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, or SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrSetEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrSetEccRamNIntrPending
 *
 *   @b Description
 *   @n This function is used to forceably set the pending status of the
 *      specified interrupt source on the specified ECC RAM
 *      Note that multiple events can be triggerred in one call.
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT,
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, or SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS)
 *   @n  numEvents          1 - 3 are valid number of events to set; 0 or > 3 invalid.
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrSetEccRamNIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, uint32_t numEvents);

/**
 *   @n@b SDL_ecc_aggrClrEccRamIntrPending
 *
 *   @b Description
 *   @n This function is used to clear the pending status of the
 *      specified interrupt source on the specified ECC RAM
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT,
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, or SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrClrEccRamIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrClrEccRamIntrPending
 *
 *   @b Description
 *   @n This function is used to clear the pending status of the
 *      specified interrupt source on the specified ECC RAM
 *      Note that multiple events can be cleared in one call.
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT,
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, or SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrClrEccRamNIntrPending(SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, uint32_t numEvents);

/**
 *   @n@b SDL_ecc_aggrIsIntrPending
 *
 *   @b Description
 *   @n This function returns the pending interrupt status for the specified
 *      ECC interrupt source from the ECC RAM specified by ramId.
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT or
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT; valid only for the
 *                            SDL_ecc_aggrIsIntrPending function)
 *
 *   @n pIsPend            A pointer (of type bool) to hold the status as below
 *     true               An interrupt of the specified type is pending
 *     false              An interrupt of the specified type is not pending, or the ramId and/or intrSrc
 *                            arguments are invalid
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrIsIntrPending(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc, bool *pIsPend);

/**
 *   @n@b SDL_ecc_aggrIsAnyIntrPending
 *
 *   @b Description
 *   @n This function returns the pending interrupt status for any
 *      interrupt source from the ECC RAM specified by ramId.
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrIsAnyIntrPending(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, bool *pIsPend);

/**
 *   @n@b SDL_ecc_aggrEnableIntr
 *
 *   @b Description
 *   @n This function enables the interrupt for the specified
 *      interrupt source on the RAM specified by ramId
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT or
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrEnableIntr(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrDisableIntr
 *
 *   @b Description
 *   @n This function disables the interrupt for the specified
 *      interrupt source on the RAM specified by ramId
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT or
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicate the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrDisableIntr(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrEnableAllIntr
 *
 *   @b Description
 *   @n This function enables the interrupts for all of the available
 *      interrupt sources on the RAM specified by ramId
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrEnableAllIntr(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId);

/**
 *   @n@b SDL_ecc_aggrDisableAllIntr
 *
 *   @b Description
 *   @n This function disables the interrupts for all of the available
 *      interrupt sources on the RAM specified by ramId
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  ramId              RAM identifier (0..SDL_ecc_aggrGetNumRams()-1)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrDisableAllIntr(const SDL_ecc_aggrRegs *pEccAggrRegs, uint32_t ramId);

/**
 *   @n@b SDL_ecc_aggrEnableIntrs
 *
 *   @b Description
 *   @n This function enables the interrupt for the specified interrupt source on all
 *      ECC RAMs serviced by this aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT or
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrEnableIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrDisableIntrs
 *
 *   @b Description
 *   @n This function disables the interrupts for the specified interrupt source on all
 *      ECC RAMs serviced by this aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  intrSrc            The interrupt source (SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT or
 *                            SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT)
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrDisableIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs, SDL_Ecc_AggrIntrSrc intrSrc);

/**
 *   @n@b SDL_ecc_aggrEnableAllIntrs
 *
 *   @b Description
 *   @n This function enables the interrupts for all of the available
 *      interrupt sources on all ECC RAMs serviced by this aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrEnableAllIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs);

/**
 *   @n@b SDL_ecc_aggrDisableAllIntrs
 *
 *   @b Description
 *   @n This function disables the interrupts for all of the available
 *      interrupt sources on all ECC RAMs serviced by this aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrDisableAllIntrs(const SDL_ecc_aggrRegs *pEccAggrRegs);

/**
 *   @n@b SDL_ecc_aggrReadStaticRegs
 *
 *   @b Description
 *   @n This function reads the static registers for ECC aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  pEccAggrStaticRegs  A pointer (of type SDL_ECC_staticRegs*) to
 *                             hold the static register values
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrReadStaticRegs(SDL_ecc_aggrRegs *pEccAggrRegs, SDL_ECC_staticRegs *pEccAggrStaticRegs);

/**
 *   @n@b SDL_ecc_aggrIntrEnableCtrl
 *
 *   @b Description
 *   @n This function reads the static registers for ECC aggregator
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  pEnableCtrl        A pointer (of type SDL_ecc_aggrEnableCtrl*) to
 *                             hold the interrupt enable control values
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrIntrEnableCtrl(SDL_ecc_aggrRegs *pEccAggrRegs, const SDL_ecc_aggrEnableCtrl *pEnableCtrl);


/**
 *   @n@b SDL_ecc_aggrIntrStatusCtrl
 *
 *   @b Description
 *   @n This function writes the ECC Aggregator Status registers for timeout and
 *      parity count values
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  pStatusCtrl        A pointer (of type SDL_ecc_aggrStatusCtrl*) to
 *                             hold the interrupt status control and count values
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */
int32_t SDL_ecc_aggrIntrStatusCtrl(SDL_ecc_aggrRegs *pEccAggrRegs, const SDL_ecc_aggrStatusCtrl *pStatusCtrl);

/**
 *   @n@b SDL_ecc_aggrIntrGetStatus
 *
 *   @b Description
 *   @n This function reads the ECC Aggregator Status registers for timeout and
 *      parity count values
 *
 *   @b Arguments
 *   @n  pEccAggrRegs       A pointer (of type SDL_ecc_aggrRegs*) to the base
 *                            address of the ECC Aggregator module
 *   @n  pStatusCtrl        A pointer (of type SDL_ecc_aggrStatusCtrl*) to
 *                             read the timeout and parity count values
 *
 *   <b> Return Value </b>
 *   @n  SDL_PASS      Success
 *   @n  SDL_EBADARGS  Failure, indicates the bad input arguments
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 */

int32_t SDL_ecc_aggrIntrGetStatus(const SDL_ecc_aggrRegs *pEccAggrRegs, SDL_ecc_aggrStatusCtrl *pStatusCtrl);


/** @} */

#ifdef __cplusplus
}
#endif

#endif
