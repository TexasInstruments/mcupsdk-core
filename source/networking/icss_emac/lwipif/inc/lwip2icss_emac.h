/*
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  lwip2enet.h
 *
 * \brief Header file for the LwIP to Enet helper functions.
 */

#ifndef LWIP2ENET_H_
#define LWIP2ENET_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* Standard language headers */
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/* OS/Posix headers */
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

/* Project dependency headers */
#include <networking/icss_emac/icss_emac.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
#define EMAC_MAC_ADDR_LEN                   6U

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief
 *  Packet device information
 *
 * \details
 *  This structure caches the device info.
 */
typedef struct Lwip2EMAC_OBJECT_
{
    /*! lwIP network interface */
    struct netif *netif;

    /** Initialization flag.*/
    uint32_t initDone;

    /** Link is up flag. */
    uint32_t linkIsUp;

    /*! Pointer to packets ready to be given to the Stack */
    struct pbuf* rxPbufPkt;

    /*
     * Clock handle for triggering the packet Rx notify
     */
    ClockP_Object pollLinkClkObj;

    /*
     * Handle to counting shutdown semaphore, which all subtasks created in the
     * open function must post before the close operation can complete.
     */
    SemaphoreP_Object shutDownSemObj;


    /*
     * Handle to input task that sends polls the link status
     */
    TaskP_Object lwipif2lwipPollTaskObj;

    /*
     * Handle to Binary Semaphore LWIP_LWIPIF_input when Rx packet queue is ready
     */
    SemaphoreP_Object pollLinkSemObj;

    /** Boolean to indicate shutDownFlag status of translation layer.*/
    volatile bool shutDownFlag;

    /** Low level driver Handle containing base addresses and modules*/
    ICSS_EMAC_Handle emacHandle;

    /** Ticks for this timer*/
    uint32_t              aleTicks;

    /**ALE ageout timer active*/
    uint32_t               aleTimerActive;

}
Lwip2EMAC_Object, *Lwip2Emac_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

// extern Lwip2Emac_Stats gLwip2EmacStats;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*
 * Functions Provided by our translation layer code
 */
extern Lwip2Emac_Handle Lwip2Emac_open(struct netif *netif);

extern void Lwip2Emac_close(Lwip2Emac_Handle hlwip2Emac);

extern int32_t Lwip2Emac_serviceRx(void *icssEmacHandleVoidPtr, void *queueNum, void *userArg);

extern int32_t Lwip2emac_sendTxPackets(Lwip2Emac_Handle hLwip2Emac, struct pbuf *p);

/*TODO: Check on poll function and IOCTL*/
extern int32_t Lwip2Emac_ioctl(Lwip2Emac_Handle hlwip2Emac,
                              uint32_t cmd,
                              void *param,
                              uint32_t size);

extern void Lwip2Emac_periodicFxn(Lwip2Emac_Handle hLwip2Emac);
extern int32_t Lwip2Emac_getDriverLinkStatus(Lwip2Emac_Handle hLwip2Emac);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */


#ifdef __cplusplus
}
#endif

#endif /* LWIP2ENET_H_ */
