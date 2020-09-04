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

#ifndef IPNISOMDRV_H_
#define IPNISOMDRV_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                          Doxygen                                           */
/* ========================================================================== */
/**
 * \defgroup PN_ISOM Isochronous Mode APIs
 * \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "PN_Handle.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
* \internal
* \def PNISO_MAX_NUM_EVENTS
*      Maximum number of ISOM events supported in a network cycle by firmware
*/
#define PNISO_MAX_NUM_EVENTS        2

/**
* \internal
* \def PNISO_MODE_NONE
*      Used to Disable ISOM mode
*/
#define PNISO_MODE_NONE             0

/** @addtogroup PN_ISOM
 @{ */

/**
* \def PNISO_MODE_INTERRUPT
*      ISOM Interrupt Mode
*/
#define PNISO_MODE_INTERRUPT        1
 /**
* \def PNISO_MODE_GPIO
*      ISOM GPIO Mode
*/
#define PNISO_MODE_GPIO             2
/**
* \def PNISO_MODE_DISABLE
*      Disable ISOM mode
*/
#define PNISO_MODE_DISABLE          0
/**
* \def PNISO_MODE_ENABLE
*      Enable ISOM mode
*/
#define PNISO_MODE_ENABLE           1

/**
 * \brief ISOM handle
 */
typedef uint32_t * PNISOM_Handle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief  API to initialize the ISOM event/signal. Returns the ISOM handle. The Event/signal will be
 *         disabled by default
 *
 * \param pnHandle Profinet Handle
 * \param[in] isoMode   ISOM type - PNISO_MODE_GPIO/PNISO_MODE_INTERRUPT
 * \param[in] timeval   time in nanosecond to trigger the event/signal (Reference is Start of Cycle)
 * \param[in] duration  Duration of signal in nanoseconds
 *
 * \retval ISOM handle on success
 */
PNISOM_Handle PN_ISO_initGPIOEvent(PN_Handle pnHandle, uint8_t isoMode, uint32_t timeval, uint32_t duration);
/**
 * \brief  API to enable the ISOM event/signal generation.
 *
 * \param pnHandle    Profinet Handle
 * \param isoHandle   ISOM handle that needs to be enabled
 *
 */
void PN_ISO_enableGPIOEvent(PN_Handle pnHandle,PNISOM_Handle isoHandle);
/**
 * \brief  API to disable the ISOM event/signal generation.
 *
 * \param pnHandle    Profinet Handle
 * \param isoHandle   ISOM handle that needs to be disabled
 *
 */
void PN_ISO_disableGPIOEvent(PN_Handle pnHandle,PNISOM_Handle isoHandle);
/**
 * \brief  API to De init the ISOM event/signal.
 *
 * \param pnHandle    Profinet Handle
 * \param isoHandle   ISOM handle that needs to be de initialized
 *
 */
void PN_ISO_deInitGPIOEvent(PN_Handle pnHandle,PNISOM_Handle isoHandle);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* IPNISOMDRV_H_ */
