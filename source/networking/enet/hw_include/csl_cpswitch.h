/**
 * @file  csl_cpswitch.h
 *
 * @brief
 *  API CSL header file for CPSW Ethernet switch module CSL.
 *
 *  Contains the different control command and status query functions definations
 *
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

#ifndef CSL_CPSWITCH_H_
#define CSL_CPSWITCH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_soc.h>


/**
 *  \ingroup CSL_IP_MODULE
 *  \defgroup CSL_CPSWITCH CPSW CSL-FL
 *
 *  @{
 *
 * CSL CPSW API  has implementation for multiple SoCs with below SoC map
 *      -# cpsw v5 - SOC_J721E/SOC_AM65XX
 *
 * Modules:
 * - <b> CPSW Data Structures     </b> (See \ref CSL_CPSWITCH_DATASTRUCT) <br>
 * - <b> CPSW Functions </b> (See \ref CSL_CPSWITCH_FUNCTION) <br>
 * - <b> GMAC Sliver submodule (CPGMAC_SL)   </b> (See \ref CSL_CPGMAC_SL_API)   <br>
 * - <b> CPTS API   </b> (See \ref CSL_CPTS_API)   <br>
 * - <b> SGMII Functions   </b> (See \ref CSL_SGMII_FUNCTION)   <br>
 * - <b> MDIO - V5   </b> (See \ref CSL_MDIO_API_V5)   <br>
 * - <b> CPSW_SS_S Data Structures  </b> (See \ref CSL_CPSW_SS_S_DATASTRUCT)   <br>
*/
/** @} */

/**
@defgroup CSL_CPSWITCH_DATASTRUCT  CPSW Data Structures
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_CPSWITCH_FUNCTION  CPSW Functions
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_CPGMAC_SL_API GMAC Sliver submodule (CPGMAC_SL)
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_CPTS_API CPTS API
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_SGMII_FUNCTION SGMII Functions
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_MDIO_API_V5 MDIO - V5
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_CPSW_SS_S_DATASTRUCT  CPSW_SS_S Data Structures
@ingroup CSL_CPSWITCH
*/
/**
@defgroup CSL_CPSW_SS_S_FUNCTION  CPSW_SS_S Functions
@ingroup CSL_CPSWITCH
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

#if defined(SOC_AM65XX) || defined(SOC_J721E) || defined (SOC_J74202) || defined (SOC_J721S2) || defined (SOC_J7200) || defined(SOC_AM64X) || defined(SOC_AM243X) || defined (SOC_AM62X) || defined (SOC_AM273X) || defined (SOC_AWR294X) || defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)

#include <cpsw/V5/csl_cpsw.h>
#include <emac/V5/csl_cpgmac_sl.h>
/* If split CSLR is provided then no need for v5 csl_mdio.h */
#include <mdio/csl_mdio.h>
/* If split CSLR is provided then no need for v1 csl_cpts.h */
#include <cpts/csl_cpts.h>

#endif /* SOC_XXXXX */

#if defined(SOC_J721E) || defined (SOC_J74202) || defined (SOC_J721S2) || defined (SOC_J7200) || defined(SOC_AM65XX) || defined(SOC_AM64X) || defined(SOC_AM243X) || defined (SOC_AM62X)
#include <cpsw/V5/V5_0/csl_cpsw_ss.h>
#include <sgmii/V5/csl_cpsgmii.h>

#endif /* SOC_XXXXX */

#if defined(SOC_AM273X) || defined (SOC_AWR294X)
#include <cpsw/V5/V5_1/csl_cpsw_ss.h>
#include <cpdma/csl_cpdma.h>

#elif defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
#include <cpsw/V5/V5_2/csl_cpsw_ss.h>
#include <cpdma/csl_cpdma.h>

#endif /* SOC_XXXXX */



/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/
