/**
 * @file csl_cpsgmii_def.h
 *
 * @brief
 *  Header file for functional layer of CSL SGMII.
 *
 *  It contains the various enumerations, structure definitions and function
 *  declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2011, Texas Instruments, Inc.
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

/** ============================================================================
 *
 * @defgroup CSL_SGMII_API SGMII
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 * @subsection References
 *    -# Ethernet Media Access Controller User Guide
 * ============================================================================
 */

#ifndef CSL_CPSGMII_DEF_H
#define CSL_CPSGMII_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_soc.h>

/**
@defgroup CSL_SGMII_SYMBOL  SGMII Symbols Defined
@ingroup CSL_SGMII_API
*/
/**
@defgroup CSL_SGMII_DATASTRUCT  SGMII Data Structures
@ingroup CSL_SGMII_API
*/
/**
@defgroup CSL_SGMII_FUNCTION  SGMII Functions
@ingroup CSL_SGMII_API
*/
/**
@defgroup CSL_SGMII_ENUM SGMII Enumerated Data Types
@ingroup CSL_SGMII_API
*/

/**
@addtogroup CSL_SGMII_SYMBOL
@{
*/

/**
@}
*/

/**
@addtogroup CSL_SGMII_ENUM
@{
*/

/** @brief
 *
 *  Duplex mode enumerators.
 */
typedef enum
{
    /** Half duplex mode */
   CSL_SGMII_HALF_DUPLEX    =   0,

    /** Full duplex mode */
   CSL_SGMII_FULL_DUPLEX    =   1
} CSL_SGMII_DUPLEXMODE;

/** @brief
 *
 *  Link speed enumerators.
 */
typedef enum
{
    /** 10 Mbps */
   CSL_SGMII_10_MBPS        =   0,

    /** 100 Mbps */
   CSL_SGMII_100_MBPS       =   1,

    /** 1000 Mbps / Gigabit */
   CSL_SGMII_1000_MBPS      =   2
} CSL_SGMII_LINKSPEED;


/**
@}
*/

/** @brief
 *
 *  Holds the SGMII module version info.
 */
typedef struct {
    /**  Minor revision value */
    Uint32      minor_version;

    /**  Major revision value */
    Uint32      major_version;

    /**  RTL Version */
    Uint32      rtl_version;

    /**  Identification Value */
    Uint32      ident_val;
}CSL_SGMII_VERSION;

/** @brief
 *
 *  Holds the SGMII status info.
 */
typedef struct {
    /**  Link Status */
    Uint32      bIsLinkUp;

    /**  Auto-negotiation error indicator */
    Uint32      bIsAutoNegError;

    /**  Auto-negotiation completion status */
    Uint32      bIsAutoNegComplete;

    /**  Lock Status */
    Uint32      bIsLocked;
}CSL_SGMII_STATUS;

/** @brief
 *
 *  SGMII advertised ability configuration
 *  info.
 */
typedef struct {
    /**  Link Status */
    Uint32                  bLinkUp;

    /**  Duplex Mode */
    CSL_SGMII_DUPLEXMODE    duplexMode;

    /**  Link speed */
    CSL_SGMII_LINKSPEED     linkSpeed;

#if defined(SOC_J721E) || defined (SOC_J74202) || defined (SOC_J721S2) || defined (SOC_J7200) || defined(SOC_AM65XX) || defined(SOC_AM64X) || defined(SOC_AM243X) || defined (SOC_AM62X)
    /**  Link speed */
    CSL_SGMII_MODE          sgmiiMode;
#endif

}CSL_SGMII_ADVABILITY;

#ifdef __cplusplus
}
#endif

#endif /* CSL_CPSGMII_DEF_H */
