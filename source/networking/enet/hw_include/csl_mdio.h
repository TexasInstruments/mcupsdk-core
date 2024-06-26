/**
 * @file csl_mdio.h
 *
 * @brief
 *  Header file for functional layer of CSL MDIO.
 *
 *  It contains the various enumerations, structure definitions and function
 *  declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2017, Texas Instruments, Inc.
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

#ifndef CSL_MDIO_TOP_H
#define CSL_MDIO_TOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_soc.h>

#include <cslr_mdio.h>
#include <csl_mdio_def.h>

/** ============================================================================
 *
 * @defgroup CSL_MDIO_API MDIO
 * @ingroup CSL_MDIO_API
 *
 *
 * @subsection xxx Overview
 *
 * CSL_MDIO_API  has common implementation for multiple SoCs along with below SoC specific
 * implementation with the mapping as below.
 *      -# mdio v0 - SOC_K2H/SOC_K2K/SOC_C6678
 *      -# mdio v1 - SOC_K2E/SOC_K2L
 *      -# mdio v2 - SOC_AM574x/SOC_AM572x/SOC_AM571x/SOC_DRA72X/SOC_DRA75X/SOC_DRA78X
 *      -# mdio v3 - SOC_C6657/SOC_OMAPL137/SOC_OMAPL138
 *
 * @note: there may not be any ip specific implementation for that soc, if it contains only cslr_* files
 *
 * ============================================================================
 */

#if defined(SOC_AM64X) || defined(SOC_AM243X) || defined (SOC_AM273X) || defined (SOC_AWR294X) || defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)

#include <mdio/csl_mdio.h>

#endif /* DEVICE_XXXXX */



#ifdef __cplusplus
}
#endif

#endif /* CSL_MDIO_H */
