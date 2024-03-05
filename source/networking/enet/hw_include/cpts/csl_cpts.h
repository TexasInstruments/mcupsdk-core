/**
 * @file  csl_cpts.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the Time synchronization submodule of EMAC.
 *
 *  ============================================================================
 *  @n   (C) Copyright 2017-2019, Texas Instruments, Inc.
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

#ifndef CSL_CPTS_H_
#define CSL_CPTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <cslr_cpts.h>

/**
 * @defgroup CSL_CPTS_API CPTS API
 *
 */

/**
@defgroup CSL_CPTS_DATASTRUCT  CPTS Data Structures
@ingroup CSL_CPTS_API
*/
/**
@defgroup CSL_CPTS_FUNCTION  CPTS Functions
@ingroup CSL_CPTS_API
*/
/**
@defgroup CSL_CPTS_ENUM CPTS Enumerated Data Types
@ingroup CSL_CPTS_API
*/

/** @addtogroup CSL_CPTS_DATASTRUCT
 @{ */


/** @brief
 *
 *  Defines CPTS timestamp output bits
 */
typedef uint32_t CSL_CPTS_TS_OUTPUT_BIT;
#define CPTS_TS_OUTPUT_BIT_DISABLED         ((uint32_t) 0U)
#define CPTS_TS_OUTPUT_BIT_17               ((uint32_t) 1U)
#define CPTS_TS_OUTPUT_BIT_18               ((uint32_t) 2U)
#define CPTS_TS_OUTPUT_BIT_19               ((uint32_t) 3U)
#define CPTS_TS_OUTPUT_BIT_20               ((uint32_t) 4U)
#define CPTS_TS_OUTPUT_BIT_21               ((uint32_t) 5U)
#define CPTS_TS_OUTPUT_BIT_22               ((uint32_t) 6U)
#define CPTS_TS_OUTPUT_BIT_23               ((uint32_t) 7U)
#define CPTS_TS_OUTPUT_BIT_24               ((uint32_t) 8U)
#define CPTS_TS_OUTPUT_BIT_25               ((uint32_t) 9U)
#define CPTS_TS_OUTPUT_BIT_26               ((uint32_t) 10U)
#define CPTS_TS_OUTPUT_BIT_27               ((uint32_t) 11U)
#define CPTS_TS_OUTPUT_BIT_28               ((uint32_t) 12U)
#define CPTS_TS_OUTPUT_BIT_29               ((uint32_t) 13U)
#define CPTS_TS_OUTPUT_BIT_30               ((uint32_t) 14U)
#define CPTS_TS_OUTPUT_BIT_31               ((uint32_t) 15U)

/** @brief
 *
 *  Defines PPM Correction Direction
 */
typedef uint32_t CSL_CPTS_TS_PPM_DIR;
#define CSL_CPTS_TS_PPM_DIR_INCREASE        ((uint32_t) 0U)
#define CSL_CPTS_TS_PPM_DIR_DECREASE        ((uint32_t) 1U)

typedef uint32_t CSL_CPTS_GENF_PPM_DIR;
#define CSL_CPTS_GENF_PPM_DIR_INCREASE        ((uint32_t) 1U)
#define CSL_CPTS_GENF_PPM_DIR_DECREASE        ((uint32_t) 0U)

typedef uint32_t CSL_CPTS_ESTF_PPM_DIR;
#define CSL_CPTS_ESTF_PPM_DIR_INCREASE        ((uint32_t) 1U)
#define CSL_CPTS_ESTF_PPM_DIR_DECREASE        ((uint32_t) 0U)

/** @brief
 *
 *  Holds the CPTS control register info.
 */
typedef struct {
    /**  Time Sync Enable:  When disabled (cleared to zero), the RCLK domain is
         held in reset. */
    uint32_t                cptsEn;

    /**  Interrupt Test: When set, this bit allows the raw interrupt to be written to
         facilitate interrupt test. */
    uint32_t                intTest;

    /**  TS_COMP Polarity: 0 - TS_COMP is asserted low; 1: TS_COMP is asserted high */
    uint32_t                tsCompPolarity;

    /**  Host Receive Timestamp Enable: When set, Timestamps enabled on received packets to host */
    uint32_t                tstampEn;

    /**  Sequence Enable:
         0: The timestamp value increments with the selected RFTCLK
         1: The timestamp for received packets is the sequence number of the received packet
      */
    uint32_t                seqEn;

    /** 64-bit mode:
        0: The timestamp is 32-bits with the upper 32-bits forced to zero.
        1: The timestamp is 64-bits.
      */
    uint32_t                ts64bMode;

    /**  TS_COMP Toggle mode - 0: TS_COMP is in non-toggle mode; 1: TS_COMP is in toggle mode */
    uint32_t                tsCompToggle;

    /**  Hardware push 1-8 enable */
    uint32_t                tsHwPushEn[8];

    /**  TS_SYNC output timestamp counter bit select */
    CSL_CPTS_TS_OUTPUT_BIT  tsOutputBitSel;

    /**  Disable Timestamp Ethernet receive events:
            0: Receive events are enabled
            1: Receive events are disabled
    */
    uint32_t                tsDisableRxEvents;
    
    /**  GENF (and ESTF) clear enable:
            0: A TS_GENFn (or TS_ESTFn) output is not cleared when the associated ts_genf_length[31:0] (or ts_estf_length[31:0]) is cleared to zero
            1: A TS_GENFn (or TS_ESTFn) output is cleared when the associated ts_genf_length[31:0] (or ts_estf_length[31:0]) is cleared to zero
    */
    uint32_t                tsGenfClrEn;

} CSL_CPTS_CONTROL;

/**
@}
*/

/** @addtogroup CSL_CPTS_FUNCTION
@{ */

/********************************************************************************
********************** Time Synchronization (CPTS) Submodule ********************
********************************************************************************/


/** ============================================================================
 *   @n@b CSL_CPTS_getCptsVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the CPTS module identification and version
 *      information.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pVersionInfo        CSL_CPTS_VERSION structure that needs to be populated
                            with the version info read from the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_IDVER_REG_MINOR_VER,
 *      CPTS_IDVER_REG_MAJOR_VER,
 *      CPTS_IDVER_REG_RTL_VER,
 *      CPTS_IDVER_REG_TX_IDENT
 *
 *   @b Example
 *   @verbatim
        CSL_CPTS_VERSION    versionInfo;

        CSL_CPTS_getCptsVersionInfo (pCptsRegs, &versionInfo);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_getCptsVersionInfo (
    const CSL_cptsRegs  *pCptsRegs,
    CSL_CPTS_VERSION*   pVersionInfo
);

/** ============================================================================
 *   @n@b CSL_CPTS_isCptsEnabled
 *
 *   @b Description
 *   @n This function indicates if time sync is enabled or not.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   CPTS enabled.
 *   @n  FALSE                  CPTS disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_CONTROL_REG_CPTS_EN
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPTS_isCptsEnabled (pCptsRegs) == TRUE)
        {
            // CPTS on
        }
        else
        {
            // CPTS off
        }
     @endverbatim
 * =============================================================================
 */
extern uint32_t CSL_CPTS_isCptsEnabled (
    const CSL_cptsRegs  *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_enableCpts
 *
 *   @b Description
 *   @n This function configures the CPTS control register to enable time sync.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CPTS_CONTROL_REG_CPTS_EN=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPTS_enableCpts (pCptsRegs);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_enableCpts (
    CSL_cptsRegs    *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_disableCpts
 *
 *   @b Description
 *   @n This function configures the CPTS control register to disable time sync.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CPTS_CONTROL_REG_CPTS_EN=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPTS_disableCpts (pCptsRegs);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_disableCpts (
    CSL_cptsRegs    *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_getCntllReg
 *
 *   @b Description
 *   @n This function retreives the contents of CPTS control register
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pCntlCfg            CSL_CPTS_CONTROL that needs to be populated with
                            contents of CPTS control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_CONTROL_REG_CPTS_EN
 *      CPTS_CONTROL_REG_INT_TEST
 *      CPTS_CONTROL_REG_TS_COMP_POLARITY
 *      CPTS_CONTROL_REG_TSTAMP_EN
 *      CPTS_CONTROL_REG_SEQUENCE_EN
 *      CPTS_CONTROL_REG_MODE
 *      CPTS_CONTROL_REG_TS_COMP_TOG
 *      CPTS_CONTROL_REG_HW1_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW2_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW3_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW4_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW5_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW6_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW7_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW8_TS_PUSH_EN
 *      CPTS_CONTROL_REG_TS_SYNC_SEL
 *      CPTS_CONTROL_REG_TS_RX_NO_EVENT
 *      CPTS_CONTROL_REG_TS_GENF_CLR_EN
 *
 *   @b Example
 *   @verbatim
        CSL_CPTS_CONTROL     cntlCfg;

        CSL_CPTS_getCntlReg (pCptsRegs, &cntlCfg);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_getCntlReg (
    const CSL_cptsRegs  *pCptsRegs,
    CSL_CPTS_CONTROL*   pCntlCfg
);

/** ============================================================================
 *   @n@b CSL_CPTS_setCntllReg
 *
 *   @b Description
 *   @n This function sets up the contents of CPTS control register
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pCntlCfg            CSL_CPTS_CONTROL contain settings for
                            CPTS control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CPTS_CONTROL_REG_CPTS_EN
 *      CPTS_CONTROL_REG_INT_TEST
 *      CPTS_CONTROL_REG_TS_COMP_POLARITY
 *      CPTS_CONTROL_REG_TSTAMP_EN
 *      CPTS_CONTROL_REG_SEQUENCE_EN
 *      CPTS_CONTROL_REG_MODE
 *      CPTS_CONTROL_REG_TS_COMP_TOG
 *      CPTS_CONTROL_REG_HW1_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW2_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW3_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW4_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW5_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW6_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW7_TS_PUSH_EN
 *      CPTS_CONTROL_REG_HW8_TS_PUSH_EN
 *      CPTS_CONTROL_REG_TS_SYNC_SEL
 *      CPTS_CONTROL_REG_TS_RX_NO_EVENT
 *      CPTS_CONTROL_REG_TS_GENF_CLR_EN
 *
 *   @b Example
 *   @verbatim
        CSL_CPTS_CONTROL     cntlCfg;

        CSL_CPTS_setCntlReg (pCptsRegs, &cntlCfg);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setCntlReg (
    CSL_cptsRegs        *pCptsRegs,
    const CSL_CPTS_CONTROL*   pCntlCfg
);

/** ============================================================================
 *   @n@b CSL_CPTS_getRFTCLKSelectReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the reference clock select
 *      register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pRefClockSelect     Reference clock select value read from hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_RFTCLK_SEL_REG_RFTCLK_SEL
 *
 *   @b Example
 *   @verbatim
        uint32_t          refClockSelect;

        CSL_CPTS_getRFTCLKSelectReg (pCptsRegs, &refClockSelect);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_getRFTCLKSelectReg (
    const CSL_cptsRegs  *pCptsRegs,
    uint32_t*           pRefClockSelect
);

/** ============================================================================
 *   @n@b CSL_CPTS_setRFTCLKSelectReg
 *
 *   @b Description
 *   @n This function sets up the reference clock select value. The Reference
 *      clock value can be setup only when the CPTS enable bit is cleared in
 *      the CPTS control register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        refClockSetVal      Reference clock select value to configure.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPTS_setTimeSyncControlReg () must be called to clear the
 *      CPTS enable bit before calling this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_RFTCLK_SEL_REG_RFTCLK_SEL
 *
 *   @b Example
 *   @verbatim
        uint32_t          refClockSelect;

        refClockSelect  =   0;

        CSL_CPTS_setRFTCLKSelectReg (pCptsRegs, refClockSelect);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setRFTCLKSelectReg (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            refClockSetVal
);

/** ============================================================================
 *   @n@b CSL_CPTS_TSEventPush
 *
 *   @b Description
 *   @n This function writes an 1 to the Time Stamp Event Push register to
 *      generate a timestamp event. The time stamp value is the time of the write
 *      of this register, not the time of the event read. The time stamp value can then
 *      be read on interrupt via the event registers.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_PUSH_REG_TS_PUSH
 *
 *   @b Example
 *   @verbatim
        CSL_CPTS_TSEventPush (pCptsRegs);
     @endverbatim
 *
 *   @note: Software should not push a second time stamp event onto the event
 *          FIFO until the first time stamp value has been read from the event FIFO
 *          (there should be only one time stamp event in the event FIFO at any
 *          given time).
 * =============================================================================
 */
extern void CSL_CPTS_TSEventPush (
    CSL_cptsRegs        *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_getTSLoadValReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Time Stamp Load Value register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pTSLoadVal          Time stamp load value read from hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL
 *   @n CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL
 *
 *   @b Example
 *   @verbatim
        uint32_t          tsLoadVal[2];

        CSL_CPTS_getTSLoadValReg (pCptsRegs, tsLoadVal);
     @endverbatim
 *
 *   @note: When reading this register, the value read is not the time
 *          stamp, but is the value that was last written to this register.
 * =============================================================================
 */
extern void CSL_CPTS_getTSLoadValReg (
    const CSL_cptsRegs  *pCptsRegs,
    uint32_t*           pTSLoadVal
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSLoadValReg
 *
 *   @b Description
 *   @n This function sets up the Time Stamp Load Value.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsLoadValLo         Time stamp load value (lower 32-bits) to configure
        tsLoadValHi         Time stamp load value (upper 32-bits) to configure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL
 *   @n CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL
 *
 *   @b Example
 *   @verbatim
        uint32_t tsLoadValLo, tsLoadValHi;

        tsLoadValLo = tsLoadValHi = 0;

        CSL_CPTS_setTSLoadValReg (pCptsRegs, tsLoadValLo, tsLoadValHi);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSLoadValReg (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsLoadValLo,
    uint32_t            tsLoadValHi
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSVal
 *
 *   @b Description
 *   @n This function sets the Time Stamp Value.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsValLo             Time stamp value (lower 32-bits) to be loaded.
        tsValHi             Time stamp value (upper 32-bits) to be loaded.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL
 *   @n CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL
 *   @n CPTS_TS_LOAD_EN_REG_TS_LOAD_EN
 *
 *   @b Example
 *   @verbatim
        uint32_t tsValLo, tsValHi;

        tsValLo = tsValHi = 0;

        CSL_CPTS_setTSVal (pCptsRegs, tsValLo, tsValHi);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSVal (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsValLo,
    uint32_t            tsValHi
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSCompVal
 *
 *   @b Description
 *   @n This function sets the Time Stamp Compare Value and triggers the Time Stamp
 *      Comparsion operation.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsCompValLo         Time stamp compare value (lower 32-bits) to be loaded.
        tsCompValHi         Time stamp compare value (upper 32-bits) to be loaded.
        tsCompLen           Length of the TS_COMP output pluse in non-toggle mode
                            Half Period of the TS_COMP wave in toggle mode
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_COMP_VAL_REG_TS_COMP_VAL
 *   @n CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_VAL
 *   @n CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH
 *
 *   @b Example
 *   @verbatim
        uint32_t          tsCompValLo, tsCompValHi;
        uint32_t          tsCompLen;

        tsCompValLo =   0x3000;
        tsCompValHi =   0;
        tsCompLen   =   1000;

        CSL_CPTS_setTSCompVal (pCptsRegs, tsCompValLo, tsCompValHi, tsCompLen);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSCompVal (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsCompValLo,
    uint32_t            tsCompValHi,
    uint32_t            tsCompLen
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSCompNudge
 *
 *   @b Description
 *   @n This function sets the Time Stamp Compare Nudge Value to adjust the phase
 *      of time compare wave in Toggle mode. This two's complement number is added
 *      to the ts_comp_length[23:0] value to increase or decrease the TS_COMP length
 *      by the ts_comp_nudge amount. Only a single high or low time is adjusted
 *      and the tsCompNudge value is cleared to zero when the nudge has occurred.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsCompNudge         Time stamp compare nudge value [-128, 127]
 *
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_COMP_NUDGE_REG_NUDGE
 *
 *   @b Example
 *   @verbatim
        int32_t        tsCompNudge;

        tsCompNudge   =   -2;

        CSL_CPTS_setTSCompNudge (pCptsRegs, tsCompNudge);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSCompNudge (
    CSL_cptsRegs        *pCptsRegs,
    int32_t             tsCompNudge
);

/** ============================================================================
 *   @n@b CSL_CPTS_getTSAddVal
 *
 *   @b Description
 *   @n This function retrieves the contents of the Time Stamp Add Value register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pTsAddVal           Time stamp add value read from hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  tsAddVal           Time stamp add value read from hardware.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_TS_ADD_VAL_REG_ADD_VAL
 *
 *   @b Example
 *   @verbatim
        uint32_t          tsAddVal;

        tsAddVal = CSL_CPTS_getTSAddVal (pCptsRegs);
     @endverbatim
 * =============================================================================
 */
extern uint32_t CSL_CPTS_getTSAddVal (
    CSL_cptsRegs        *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSAddVal
 *
 *   @b Description
 *   @n This function sets the Time Stamp Add Value to adjust the 64-bit
 *      timestamp value. The tsAddVal[2:0] is added to 1 to comprise the
 *      timestamp increment value. The timestamp increment value is added to the
 *      current timestamp (time_stamp[63:0]) on each RCLK.
 *
 *      Note that this function is only applicable for 64-bit timestamp mode.
 *      In 32-bit timestamp mode, 0 is written.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsAddVal            Time stamp add value [0, 7]
 *
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_ADD_VAL_REG_ADD_VAL
 *
 *   @b Example
 *   @verbatim
        uint32_t          tsAddVal;

        tsAddVal   =   3;

        CSL_CPTS_setTSAddVal (pCptsRegs, tsAddVal);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSAddVal (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsAddVal
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSNudge
 *
 *   @b Description
 *   @n This function sets the Time Stamp Nudge Value to adjust the 64-bit
 *      timestamp value. This two's complement number is added to the
 *      time_stamp[63:0] value to increase or decrease the timestamp value by
 *      the tsNudge amount.  The tsNudge value is cleared to zero when the nudge
 *      has occurred.
 *
 *      Note that this function is only applicable for 64-bit timestamp mode.
 *      In 32-bit timestamp mode, this function does nothing.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsNudge             Time stamp nudge value [-128, 127]
 *
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL
 *
 *   @b Example
 *   @verbatim
        int32_t        tsNudge;

        tsNudge   =   -2;

        CSL_CPTS_setTSNudge (pCptsRegs, tsNudge);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSNudge (
    CSL_cptsRegs        *pCptsRegs,
    int32_t             tsNudge
);

/** ============================================================================
 *   @n@b CSL_CPTS_getTSPpm
 *
 *   @b Description
 *   @n This function retrieves the contents of the Time Stamp PPM Value register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pTSPpm              PPM value read from hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL
 *   @n CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL
 *
 *   @b Example
 *   @verbatim
        uint32_t          tsPpmVal[2];

        CSL_CPTS_getTSPpm (pCptsRegs, tsPpmVal);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_getTSPpm (
    const CSL_cptsRegs  *pCptsRegs,
    uint32_t            tsPpmVal[2]
);

/** ============================================================================
 *   @n@b CSL_CPTS_setTSPpm
 *
 *   @b Description
 *   @n This function sets the parts per million or parts per hour 64-bit
 *      timestamp adjustment value. Writing a non-zero tsPpm value enables PPM
 *      operations. The adjustment is up or down depending on the tsPpmDir
 *      value. The timestamp value is increased by the PPM value when tsPpmDir
 *      is CSL_CPTS_TS_PPM_DIR_INCREASE and decreased by the PPM value when
 *      tsPpmDir is CSL_CPTS_TS_PPM_DIR_DECREASE.
 *
 *      Note that this function is only applicable for 64-bit timestamp mode.
 *      In 32-bit timestamp mode, this function does nothing.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        tsPpm               Time stamp nudge value [-128, 127]
        tsPpmDir            Adjustment direction
 *
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_CONTROL_REG_TS_PPM_DIR
 *   @n CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL
 *   @n CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL
 *
 *   @b Example
 *   @verbatim
        uint32_t                      tsPpmValLo,
        uint32_t                      tsPpmValHi,
        CSL_CPTS_TS_PPM_DIR         tsPpmDir;

        tsPpmValLo   =   10000UL;
        tsPpmValHi   =   0;
        tsPpmDir = CSL_CPTS_TS_PPM_DIR_INCREASE;

        CSL_CPTS_setTSPpm (pCptsRegs, tsPpmValLo, tsPpmValHi, tsPpmDir);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setTSPpm (
    CSL_cptsRegs        *pCptsRegs,
    uint32_t            tsPpmValLo,
    uint32_t            tsPpmValHi,
    CSL_CPTS_TS_PPM_DIR tsPpmDir
);

/** ============================================================================
 *   @n@b CSL_CPTS_isRawInterruptStatusBitSet
 *
 *   @b Description
 *   @n This function checks the Time Sync Raw Pending Interrupt Register to
 *      determine if there is one or more events in the event FIFO.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   One or more events in FIFO. Raw interrupt status
 *                              bit set.
 *   @n  FALSE                  No events in FIFO. Raw interrupt status bit cleared.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_INTSTAT_RAW_REG_TS_PEND_RAW
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPTS_isRawInterruptStatusBitSet (pCptsRegs) == TRUE)
        {
           // interrupt set
        }
        else
        {
            // interrupt bit not set
        }
     @endverbatim
 * =============================================================================
 */
extern uint32_t CSL_CPTS_isRawInterruptStatusBitSet (
    const CSL_cptsRegs  *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_isMaskedInterruptStatusBitSet
 *
 *   @b Description
 *   @n This function checks the Time Sync Interrupt Status Masked Register to
 *      determine if there is one or more events in the event FIFO.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   One or more events in FIFO. masked interrupt status
 *                              bit set.
 *   @n  FALSE                  No events in FIFO. masked interrupt status bit cleared.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_INTSTAT_MASKED_REG_TS_PEND
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPTS_isMaskedInterruptStatusBitSet (pCptsRegs) == TRUE)
        {
           // masked interrupt set
        }
        else
        {
            // masked interrupt bit not set
        }
     @endverbatim
 * =============================================================================
 */
extern uint32_t CSL_CPTS_isMaskedInterruptStatusBitSet (
    const CSL_cptsRegs  *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_isInterruptEnabled
 *
 *   @b Description
 *   @n This function indicates if Time sync interrupts are enabled.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Time sync interrupts enabled.
 *   @n  FALSE                  Time sync interrupts disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_INT_ENABLE_REG_TS_PEND_EN
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPTS_isInterruptEnabled (pCptsRegs) == TRUE)
        {
           // interrupts enabled
        }
        else
        {
            // interrupts disabled
        }
     @endverbatim
 * =============================================================================
 */
extern uint32_t CSL_CPTS_isInterruptEnabled (
    const CSL_cptsRegs  *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_enableInterrupt
 *
 *   @b Description
 *   @n This function enables the interrupts in Time sync submodule.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CPTS_INT_ENABLE_REG_TS_PEND_EN=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPTS_enableInterrupt (pCptsRegs);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_enableInterrupt (
    CSL_cptsRegs    *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_disableInterrupt
 *
 *   @b Description
 *   @n This function disables the interrupts in Time sync submodule.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CPTS_INT_ENABLE_REG_TS_PEND_EN=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPTS_disableInterrupt (pCptsRegs);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_disableInterrupt (
    CSL_cptsRegs    *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_popEvent
 *
 *   @b Description
 *   @n This function sets up the Event pop bit in Event pop register. This
 *      initiates an event to be popped off the event FIFO. Popping an event discards
 *      the event and causes the next event, if any, to be moved to the top of
 *      the FIFO ready to be read by software on interrupt.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CPTS_EVENT_POP_REG_EVENT_POP=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPTS_popEvent (pCptsRegs);

     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_popEvent (
    CSL_cptsRegs    *pCptsRegs
);

/** ============================================================================
 *   @n@b CSL_CPTS_getEventInfo
 *
 *   @b Description
 *   @n This function retrieves the contents of the Event Low, Event Middle and Event High
 *      registers.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        pEventInfo          CSL_CPTS_EVENTINFO structure that needs to be filled
                            with time sync event info read from hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_EVENT_LOW_REG_TIME_STAMP,
 *      CPTS_EVENT_MIDDLE_REG_SEQUENCE_ID,
 *      CPTS_EVENT_MIDDLE_REG_MESSAGE_TYPE,
 *      CPTS_EVENT_MIDDLE_REG_EVENT_TYPE,
 *      CPTS_EVENT_MIDDLE_REG_PORT_NUMBER,
 *      CPTS_EVENT_HIGH_REG_DOMAIN
 *
 *   @b Example
 *   @verbatim
        CSL_CPTS_EVENTINFO          eventInfo;

        CSL_CPTS_getEventInfo (pCptsRegs, &eventInfo);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_getEventInfo (
    const CSL_cptsRegs  *pCptsRegs,
    CSL_CPTS_EVENTINFO* pEventInfo
);

/** ============================================================================
 *   @n@b CSL_CPTS_getGENFnLength
 *
 *   @b Description
 *   @n This function retrieves the contents of the GENFn length register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        genfIndex           Index of the GENFn to configure
        pGenfLength         GENFn length value obtained from hardware
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n   0 = success
 *   @n  -1 = genfIndex is invalid (outside of the valid range)
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_TS_GENF_LENGTH_REG_LENGTH
 *
 *   @b Example
 *   @verbatim
        uint32_t          genfLength, genfIndex;

        genfIndex = 0;

        CSL_CPTS_getGENFnLength (pCptsRegs, genfIndex, &genfLength);
     @endverbatim
 * =============================================================================
 */
extern int32_t CSL_CPTS_getGENFnLength (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        genfIndex,
    uint32_t*       pGenfLength
);

/** ============================================================================
 *   @n@b CSL_CPTS_setupGENFn
 *
 *   @b Description
 *   @n This function sets the Time Stamp Compare Value of GENFn and triggers
 *      the Time Stamp Comparsion operation.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs   Pointer to CSL_cptsRegs structure
        genfIndex   Index of the GENFn to configure
        length      Length of the GENFn output pulse in RCLK periods
        compare     Time stamp compare value (64-bit value) to be loaded
        polarityInv 0 = TS_GENFn is asserted low
                    1 = TS_GENFn is asserted high
        ppmAdjust   PPM adjustment value
        ppmDir      CSL_CPTS_TS_PPM_DIR_INCREASE(0) = A single RCLK is added to
                    the generate function counter at the PPM rate which has the
                    effect of decreasing the generate function frequency by the
                    PPM amount.
                    CSL_CPTS_TS_PPM_DIR_DECREASE(1) = A single RCLK is subtracted
                    from the generate function counter at the PPM rate which
                    has the effect of increasing the generate function
                    frequency by the PPM amount.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n   0 = success
 *   @n  -1 = genfIndex is invalid (outside of the valid range)
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_GENF_LENGTH_REG_LENGTH
 *   @n CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW
 *   @n CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH
 *   @n CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW
 *   @n CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH
 *   @n CPTS_TS_GENF_CONTROL_REG_PPM_DIR
 *   @n CPTS_TS_GENF_CONTROL_REG_POLARITY_INV
 *
 *   @b Example
 *   @verbatim
        uint64_t          tsGENFnCompVal;
        uint32_t          tsGENFnLen, tsGENFnIndex;

        tsGENFnIndex   = 0;
        tsGENFnCompVal = 0x3000U;
        tsGENFnLen     = 1000;

        CSL_CPTS_setupGENFn( pCptsRegs, tsGENFnIndex, tsGENFnLen, tsGENFnCompVal, 0, 0, CSL_CPTS_TS_PPM_DIR_INCREASE );
     @endverbatim
 * =============================================================================
 */
extern int32_t CSL_CPTS_setupGENFn(
    CSL_cptsRegs *pCptsRegs,
    uint32_t genfIndex,
    uint32_t length,
    uint64_t compare,
    uint32_t polarityInv,
    uint64_t ppmAdjust,
    CSL_CPTS_TS_PPM_DIR ppmDir
);

/** ============================================================================
 *   @n@b CSL_CPTS_setGENFnNudge
 *
 *   @b Description
 *   @n This function adjusts the GENFn cycle length by the specified 2's
 *      complement value. For example, a value of -2 will subtract 2 RCLKs from
 *      the ts_genfN_length[31:0] value. A value of 1 will add 1 RCLK to the
 *      ts_genfN_comp_length[23:0] value.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs   Pointer to CSL_cptsRegs structure
        genfIndex   Index of the GENFn to configure
        tsNudge     Nudge value
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n   0 = success
 *   @n  -1 = genfIndex is invalid (outside of the valid range)
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_GENF_NUDGE_REG_NUDGE
 *
 *   @b Example
 *   @verbatim
        int32_t tsNudge = -2;

        CSL_CPTS_setGENFnNudge( pCptsRegs, tsNudge );
     @endverbatim
 * =============================================================================
 */
extern int32_t CSL_CPTS_setGENFnNudge (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        genfIndex,
    int32_t         tsNudge
);

/** ============================================================================
 *   @n@b CSL_CPTS_setGENFnPpm
 *
 *   @b Description
 *   @n This function sets the parts per million or parts per hour adjustment value.
 *      Writing a non-zero ppm value enables PPM operations. The adjustment is up or
 *      down depending on the ppmDir value. When ts_genfN_ppm_dir is set a single
 *      RCLK time is subtracted from the generate function counter which has the effect
 *      of increasing the generate function frequency by the PPM amount.  When ts_genfN_ppm_dir
 *      is clear a single RCLK time is added to the generate function counter which has
 *      the effect of decreasing the generate function frequency by the PPM amount.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        genfIndex           Index of the GENFn to configure
        ppmValLo            Lower 32 bits of the PPM value
        ppmValHi            Higher 10 bits of the PPM value
        ppmDir              CSL_CPTS_GENF_PPM_DIR_INCREASE(1) = A single RCLK is added to
                            the generate function counter at the PPM rate which has the
                            effect of decreasing the generate function frequency by the
                            PPM amount.
                            CSL_CPTS_GENF_PPM_DIR_DECREASE(0) = A single RCLK is subtracted
                            from the generate function counter at the PPM rate which
                            has the effect of increasing the generate function
                            frequency by the PPM amount.
 *
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_GENF_CONTROL_REG_PPM_DIR
 *   @n CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH
 *   @n CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW
 *
 *   @b Example
 *   @verbatim
        uint32_t                      genfIndex;
        uint32_t                      tsPpmValLo,
        uint32_t                      tsPpmValHi,
        CSL_CPTS_GENF_PPM_DIR         tsPpmDir;

        genfIndex    =   0;
        tsPpmValLo   =   10000UL;
        tsPpmValHi   =   0;
        tsPpmDir = CSL_CPTS_GENF_PPM_DIR_INCREASE;

        CSL_CPTS_setTSPpm (pCptsRegs, genfIndex, tsPpmValLo, tsPpmValHi, tsPpmDir);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setGENFnPpm (
    CSL_cptsRegs           *pCptsRegs,
    uint32_t               genfIndex,
    uint32_t               ppmValLo,
    uint32_t               ppmValHi,
    CSL_CPTS_GENF_PPM_DIR  ppmDir
);

/** ============================================================================
 *   @n@b CSL_CPTS_getESTFnLength
 *
 *   @b Description
 *   @n This function retrieves the contents of the ESTFn length register.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        estfIndex           Index of the ESTFn to configure
        pEstfLength         ESTFn length value obtained from hardware
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n   0 = success
 *   @n  -1 = estfIndex is invalid (outside of the valid range)
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPTS_TS_ESTF_LENGTH_REG_LENGTH
 *
 *   @b Example
 *   @verbatim
        uint32_t          estfLength, estfIndex;

		estfIndex = 0;

        CSL_CPTS_getESTFnLength (pCptsRegs, estfIndex, &estfLength);
     @endverbatim
 * =============================================================================
 */
extern int32_t CSL_CPTS_getESTFnLength (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        estfIndex,
    uint32_t*       pEstfLength
);

/** ============================================================================
 *   @n@b CSL_CPTS_setupESTFn
 *
 *   @b Description
 *   @n This function sets the Time Stamp Compare Value of ESTFn and triggers
 *      the Time Stamp Comparsion operation.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs   Pointer to CSL_cptsRegs structure
        estfIndex   Index of the ESTFn to configure
        length      Length of the ESTFn output pulse in RCLK periods
        compare     Time stamp compare value (64-bit value) to be loaded
        polarityInv 0 = TS_ESTFn is asserted low
                    1 = TS_ESTFn is asserted high
        ppmAdjust   PPM adjustment value
        ppmDir      CSL_CPTS_TS_PPM_DIR_INCREASE(0) = A single RCLK is added to
                    the generate function counter at the PPM rate which has the
                    effect of decreasing the generate function frequency by the
                    PPM amount.
                    CSL_CPTS_TS_PPM_DIR_DECREASE(1) = A single RCLK is subtracted
                    from the generate function counter at the PPM rate which
                    has the effect of increasing the generate function
                    frequency by the PPM amount.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n   0 = success
 *   @n  -1 = estfIndex is invalid (outside of the valid range)
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_ESTF_LENGTH_REG_LENGTH
 *   @n CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW
 *   @n CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH
 *   @n CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW
 *   @n CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH
 *   @n CPTS_TS_ESTF_CONTROL_REG_PPM_DIR
 *   @n CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV
 *
 *   @b Example
 *   @verbatim
        uint64_t          tsESTFnCompVal;
        uint32_t          tsESTFnLen, tsESTFnIndex;

        tsESTFnIndex   = 0;
        tsESTFnCompVal = 0x3000U;
        tsESTFnLen     = 1000;

        CSL_CPTS_setupESTFn( pCptsRegs, tsESTFnIndex, tsESTFnLen, tsESTFnCompVal, 0, 0, CSL_CPTS_TS_PPM_DIR_INCREASE );
     @endverbatim
 * =============================================================================
 */
extern int32_t CSL_CPTS_setupESTFn(
    CSL_cptsRegs *pCptsRegs,
    uint32_t estfIndex,
    uint32_t length,
    uint64_t compare,
    uint32_t polarityInv,
    uint64_t ppmAdjust,
    CSL_CPTS_TS_PPM_DIR ppmDir
);

/** ============================================================================
 *   @n@b CSL_CPTS_setESTFnNudge
 *
 *   @b Description
 *   @n This function adjusts the ESTFn cycle length by the specified 2's
 *      complement value. For example, a value of -2 will subtract 2 RCLKs from
 *      the ts_ESTFn_length[31:0] value. A value of 1 will add 1 RCLK to the
 *      ts_ESTFn_comp_length[23:0] value.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs   Pointer to CSL_cptsRegs structure
        estfIndex   Index of the ESTFn to configure
        tsNudge     Nudge value
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n   0 = success
 *   @n  -1 = estfIndex is invalid (outside of the valid range)
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_ESTF_NUDGE_REG_NUDGE
 *
 *   @b Example
 *   @verbatim
        int32_t tsNudge = -2;

        CSL_CPTS_setESTFnNudge( pCptsRegs, tsNudge );
     @endverbatim
 * =============================================================================
 */
extern int32_t CSL_CPTS_setESTFnNudge (
    CSL_cptsRegs    *pCptsRegs,
    uint32_t        estfIndex,
    int32_t         tsNudge
);

/** ============================================================================
 *   @n@b CSL_CPTS_setESTFnPpm
 *
 *   @b Description
 *   @n This function sets the parts per million or parts per hour adjustment value.
 *      Writing a non-zero ppm value enables PPM operations. The adjustment is up or
 *      down depending on the ppmDir value. When ts_estfN_ppm_dir is set a single
 *      RCLK time is subtracted from the generate function counter which has the effect
 *      of increasing the generate function frequency by the PPM amount.  When ts_estfN_ppm_dir
 *      is clear a single RCLK time is added to the generate function counter which has
 *      the effect of decreasing the generate function frequency by the PPM amount.
 *
 *   @b Arguments
     @verbatim
        pCptsRegs           Pointer to CSL_cptsRegs structure
        genfIndex           Index of the ESTFn to configure
        ppmValLo            Lower 32 bits of the PPM value
        ppmValHi            Higher 10 bits of the PPM value
        ppmDir              CSL_CPTS_ESTF_PPM_DIR_INCREASE(1) = A single RCLK is added to
                            the generate function counter at the PPM rate which has the
                            effect of decreasing the generate function frequency by the
                            PPM amount.
                            CSL_CPTS_ESTF_PPM_DIR_DECREASE(0) = A single RCLK is subtracted
                            from the generate function counter at the PPM rate which
                            has the effect of increasing the generate function
                            frequency by the PPM amount.
 *
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Affects
 *   @n CPTS_TS_ESTF_CONTROL_REG_PPM_DIR
 *   @n CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH
 *   @n CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW
 *
 *   @b Example
 *   @verbatim
        uint32_t                      estfIndex;
        uint32_t                      tsPpmValLo,
        uint32_t                      tsPpmValHi,
        CSL_CPTS_ESTF_PPM_DIR         tsPpmDir;

        genfIndex    =   0;
        tsPpmValLo   =   10000UL;
        tsPpmValHi   =   0;
        tsPpmDir = CSL_CPTS_ESTF_PPM_DIR_INCREASE;

        CSL_CPTS_setTSPpm (pCptsRegs, estfIndex, tsPpmValLo, tsPpmValHi, tsPpmDir);
     @endverbatim
 * =============================================================================
 */
extern void CSL_CPTS_setESTFnPpm (
    CSL_cptsRegs           *pCptsRegs,
    uint32_t               estfIndex,
    uint32_t               ppmValLo,
    uint32_t               ppmValHi,
    CSL_CPTS_ESTF_PPM_DIR  ppmDir
);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_CPTS_H_ */
