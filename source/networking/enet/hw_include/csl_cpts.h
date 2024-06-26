/**
 * @file  csl_cpts.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the Time synchronization submodule of EMAC.
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2019, Texas Instruments, Inc.
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

#ifndef CSL_CPTS_TOP_H_
#define CSL_CPTS_TOP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_soc.h>

#include <cslr_cpts.h>


/**
@defgroup CSL_CPTS_SYMBOL  CPTS Symbols Defined
@ingroup CSL_CPTS_API
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

/**
@addtogroup CSL_CPTS_SYMBOL
@{
*/

/**
@}
*/

/** @addtogroup CSL_CPTS_DATASTRUCT
 @{ */

/** @brief
 *
 *  Defines CPTS event types.
 */
/**  Time stamp push event */
#define     CSL_CPTS_EVENTTYPE_TS_PUSH          0

/**  Time stamp rollover event (32-bit mode only) */
#define     CSL_CPTS_EVENTTYPE_TS_ROLLOVER      1

/**  Time stamp Half Rollover event (32-bit mode only) */
#define     CSL_CPTS_EVENTTYPE_TS_HALFROLLOVER  2

/**  Hardware Time stamp push event */
#define     CSL_CPTS_EVENTTYPE_HW_TS_PUSH       3

/**  Ethernet receive event */
#define     CSL_CPTS_EVENTTYPE_ETH_RECEIVE      4

/**  Ethernet Transmit event */
#define     CSL_CPTS_EVENTTYPE_ETH_TRANSMIT     5

/**  Time stamp compare event (non-toggle mode only) */
#define     CSL_CPTS_EVENTTYPE_TS_COMP          6

/**  Host event */
#define     CSL_CPTS_EVENTTYPE_HOST             7

/** @brief
 *
 *  Holds the Time sync submodule's version info.
 */
typedef struct {
    /**  Minor version value */
    Uint32      minorVer;

    /**  Major version value */
    Uint32      majorVer;

    /**  RTL version value */
    Uint32      rtlVer;

    /**  Identification value */
    Uint32      id;
} CSL_CPTS_VERSION;

/** @brief
 *
 *  Holds Time sync event info contents.
 */
typedef struct {
    /**  32-bit Event Time stamp  */
    Uint32      timeStamp;

    /**  Upper 32-bit Event Time stamp provided by new generation CPTS */
    Uint32      timeStampHi;

    /**  Event Sequence Id */
    Uint32      seqId;

    /**  Event Message Type */
    Uint32      msgType;

    /**  Event Type */
    Uint32      eventType;

    /**  EMAC Port number */
    Uint32      portNo;

    /**  Event Domain */
    Uint32      domain;
} CSL_CPTS_EVENTINFO;

/**
@}
*/

/** ============================================================================
 *
 * @defgroup CSL_CPTS_API Time Sync Submodule (CPTS)
 * @ingroup CSL_CPSW_API
 *
 *
 * @subsection xxx Overview
 *
 * CSL CPTS API  has common implementation for multiple SoCs along with below SoC specific
 * implementation with the mapping as below.
 *      -# cpsw/V0 - SOC_K2K/SOC_K2H
 *      -# cpsw/V1 - SOC_K2E/SOC_K2L
 *      -# cpsw/V2 - SOC_K2G
 *      -# cpsw/V4 - SOC_C6678
 *      -# cpts/V0 - SOC_AM65XX
 *
 * @note: there may not be any ip specific implementation for that soc, if it contains only cslr_* files
 *
 * ============================================================================
 */

#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined (SOC_AM273X) || defined (SOC_AWR294X) || defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)

#include <cpts/csl_cpts.h>

#endif /* SOC_XXXXX */

#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/

