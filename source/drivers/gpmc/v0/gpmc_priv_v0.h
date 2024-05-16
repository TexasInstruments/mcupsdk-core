/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file v0/gpmc_priv_v0.h
 *
 *  \brief GPMC module common APIs file.
 */

/**
 *  \defgroup DRV_GPMC_MODULE APIs for GPMC
 *  \ingroup DRV_MODULE
 *
 *  This file containing the GPMC API.
 *
 *  The GPMC header file should be included in only GPMC module files:
 *  \code
 *  #include "gpmc_priv_v0.h"
 *  \endcode
 *
 *  @{
 */

#ifndef _GPMC_PRIV_V0__H_
#define _GPMC_PRIV_V0__H_


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/gpmc.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_gpmc.h>
#include <kernel/dpl/CacheP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
*  \name GPMC MODULE timeouts
*
*   Macros which can be used to define timeouts for GPMC module.
*
* @{
*/
#define GPMC_MODULE_RESET_WAIT_TIME_MAX              (10 * 1000)   /*10ms*/
#define GPMC_WAIT_PIN_STATUS_WAIT_TIME_MAX           (10 * 1000U)  /*1ms*/
#define GPMC_WAIT_PIN_STATUS_WAIT_TIME_MIN           (0U)          /*1ms*/
#define GPMC_ELM_ERR_STATUS_TIMEOUT_MAX              (10 * 1000U)  /*1ms*/
/** @} */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  Function to disable GPMC interrupt.
 *
 *  \param  baseAddr  GPMC Peripheral base address
 *
 *  \param  interupt  Macro value for GPMC Interrupt Flag.
 *
 */
void GPMC_disableInterupt(uint32_t baseAddr, uint32_t interupt);

/**
 *  \brief  Function to enable GPMC interrupt.
 *
 *  \param  baseAddr  GPMC Peripheral base address
 *
 *  \param  interupt  Macro value for GPMC Interrupt Flag.
 *
 */
void GPMC_enableInterupt(uint32_t baseAddr, uint32_t interupt);

/**
 *  \brief  Function to clear GPMC interrupt status.
 *
 *  \param  baseAddr  GPMC Peripheral base address
 *
 *  \param  interupt  Macro value for GPMC Interrupt Flag.
 *
 */
void GPMC_interuptStatusClear(uint32_t baseAddr, uint32_t interupt);

/**
 *  \brief  Function to read GPMC interrupt status.
 *
 *  \param  baseAddr  GPMC Peripheral base address
 *
 *  \param  interupt  Macro value for GPMC Interrupt Flag.
 *
 */
uint32_t  GPMC_interuptStatusGet(uint32_t baseAddr, uint32_t interupt);

int32_t GPMC_waitPinInteruptStatusReadyWaitTimeout(GPMC_Handle handle,
                                uint32_t timeOut);

int32_t GPMC_waitPinStatusReadyWaitTimeout(GPMC_Handle handle,
                                uint32_t timeOut);

/**
 *  \brief  Function to check for DMA restricted regions.
 *
 *  \param  handle  An #GPMC_Handle returned from an #GPMC_open()
 *
 *  \param  addr  Address to be checked
 *
 *  \return TRUE or FALSE
 */
int32_t GPMC_isDmaRestrictedRegion(GPMC_Handle handle, uint32_t addr);

/**
 *  \brief  Function to check WAIT pin status.
 *
 *  \param  baseAddr  GPMC Chip Select base address.
 *
 *  \param  pin  GPMC WAIT pin
 *
 *  \return Status of WAIT pin
 */
uint32_t GPMC_waitPinStatusGet(uint32_t baseAddr, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif  /* _GPMC_PRIV_V0__H_ */

/** @} */

