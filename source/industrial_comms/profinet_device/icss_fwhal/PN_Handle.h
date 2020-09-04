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

#ifndef PN_HANDLE_H_
#define PN_HANDLE_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                          Handle Definition                                 */
/* ========================================================================== */
#include <stdint.h>
#include <kernel/dpl/HwiP.h>

/* ========================================================================== */
/*                          Handle Declarations                               */
/* ========================================================================== */

/*
*  \brief     PN_IntAttrs
*             Structure storing the interrupt configurations
*/
typedef struct PN_IntAttrs_s
{
    HwiP_Object interruptObject;
    /*! PRUICSS Interrupt number       */
    uint32_t pruIntNum;
    /*! Core Interrupt number           */
    uint32_t coreIntNum;
    /*! SoC event ID           */
    uint32_t socEvId;
    /*! HWI priority                   */
    uint32_t intPrio;
    /*! HWI args                   */
    void *args;
    /*! Interrupt routine              */
    void *isrFnPtr;
    /*! Callback to the stack within the ISR       */
    void *callBackPtr;
} PN_IntAttrs;

/*
*  \brief     PN_IntConfig
*             Structure storing the different interrupt configurations of Profinet
*/
typedef struct PN_IntConfig_s
{
    /*! Interrupt configuration of PN PPM       */
    PN_IntAttrs ppmIntConfig;
    /*! Interrupt configuration of PN CPM       */
    PN_IntAttrs cpmIntConfig;
    /*! Interrupt configuration of PN DHT       */
    PN_IntAttrs dhtIntConfig;
    /*! Interrupt configuration of PN PTCP      */
    PN_IntAttrs ptcpIntConfig;
#ifdef ENABLE_LATCH_SUPPORT
    /*! Interrupt configuration of PN Latch      */
    PN_IntAttrs latchIntConfig;
#endif
} PN_IntConfig;

/**
 * \brief Alias for Profinet Handle containing base addresses and modules
 */
typedef struct PN_Config_s *PN_Handle;


#ifdef __cplusplus
}
#endif

#endif /* PN_HANDLE_H_ */
