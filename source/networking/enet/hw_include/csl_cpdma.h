/**
 * @file  csl_cpdma.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the CPDMA submodule of EMAC(CPSW).
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2020, Texas Instruments, Inc.
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

#ifndef CSL_CPDMA_TOP_H_
#define CSL_CPDMA_TOP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_soc.h>
#include <cslr_cpdma.h>


/**
@defgroup CSL_CPDMA_SYMBOL  CPDMA Symbols Defined
@ingroup CSL_CPDMA_API
*/
/**
@defgroup CSL_CPDMA_DATASTRUCT  CPDMA Data Structures
@ingroup CSL_CPDMA_API
*/
/**
@defgroup CSL_CPDMA_FUNCTION  CPDMA Functions
@ingroup CSL_CPDMA_API
*/
/**
@defgroup CSL_CPDMA_ENUM CPDMA Enumerated Data Types
@ingroup CSL_CPDMA_API
*/

/**
@addtogroup CSL_CPDMA_SYMBOL
@{
*/

/**
@}
*/

/** @addtogroup CSL_CPDMA_DATASTRUCT
 @{ */

/** @brief
 *
 *  Defines CPDMA event types.
 */
/**  Time stamp push event */
#define     CSL_CPTS_EVENTTYPE_TS_PUSH          0

/**
@}
*/

/** ============================================================================
 *
 * @defgroup CSL_CPDMA_API CPDMA Submodule (CPDMA)
 * @ingroup CSL_CPSW_API
 *
 *
 * @subsection xxx Overview
 *
 * CSL CPDMA API  has common implementation for multiple SoCs along with below SoC specific
 * implementation with the mapping as below.
 *      -# cpsw/V0 - SOC_AM273X SOC_AWR294X
 *
 * @note: there may not be any ip specific implementation for that soc, if it contains only cslr_* files
 *
 * ============================================================================
 */
#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)

#include <cpdma/csl_cpdma.h>

#endif /* SOC_XXXXX */

#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/

