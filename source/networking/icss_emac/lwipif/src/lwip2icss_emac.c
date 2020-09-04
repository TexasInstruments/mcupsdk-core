/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>
#include <assert.h>
#include <string.h>

/* xdc header - should be first file included as due to order dependency */

/* OS/Posix headers */

/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip2icss_emac.h"

/**
 * ICSS EMAC specific header files
 */
#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>

#include "lwip2lwipif_priv.h"


/*---------------------------------------------------------------------------*\
 |                             Extern Declarations                             |
 \*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*\
 |                            Local Macros/Defines                             |
 \*---------------------------------------------------------------------------*/
/** Timeperiod (in ticks) after which Learning Age out function is called to age the table. This divided by 100
 * gives the number of BIOS ticks*/
#define LWIP_ICSS_ALE_AGE_OUT_TIME            ((uint32_t)300000)
/*---------------------------------------------------------------------------*\
 |                         Local Function Declarations                         |
 \*---------------------------------------------------------------------------*/
static void Lwip2Emac_allocRxPackets(Lwip2Emac_Handle hLwip2Emac);

static int32_t Lwip2Emac_startRxTx(Lwip2Emac_Handle hLwip2Emac);

static void Lwip2Emac_stopRxTx(Lwip2Emac_Handle hLwip2Emac);

static void Lwip2Emac_timerCb(ClockP_Object *hClk, void * arg);

static void Lwip2Emac_createTimer(Lwip2Emac_Handle hLwip2Emac);

int32_t Lwip2Emac_serviceRx(void *icssEmacHandleVoidPtr, void *queueNum, void *userArg);
/*---------------------------------------------------------------------------*\
 |                         Local Variable Declarations                         |
 \*---------------------------------------------------------------------------*/

static Lwip2EMAC_Object gLwip2EmacObj;

/*---------------------------------------------------------------------------*\
 |                         Global Variable Declarations                        |
 \*---------------------------------------------------------------------------*/

// Lwip2Emac_Stats gLwip2EmacStats;
/*---------------------------------------------------------------------------*\
 |                             Extern Declarations                             |
 \*---------------------------------------------------------------------------*/
extern int32_t app_getEmacHandle(Lwip2Emac_Handle hLwip2Emac);

/*!
 *  @b Lwip2Emac_initConfig
 *  @n
 *      Initialises the lower level driver handle and related attributes in interface handle
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *
 *  \retval
 *      void
 */
static void Lwip2Emac_initConfig(Lwip2Emac_Handle hLwip2Emac)
{
    int32_t status = SystemP_SUCCESS;

    /* Create semaphore objects, init shutDownFlag status */
    hLwip2Emac->shutDownFlag = false;

    /*Getting Emac driver Handle from application to fill driver handle in hLwip2Emac Handle*/
    status = app_getEmacHandle(hLwip2Emac);
    if(hLwip2Emac->emacHandle == NULL)
    {
        DebugP_log("[LWIPIF_LWIP]ICSS EMAC Handle is not initilialised.\r\n");
    }

    uint32_t semInitCnt              = 0U;
    status = SemaphoreP_constructBinary(&hLwip2Emac->shutDownSemObj, semInitCnt);
    DebugP_assert(status == SystemP_SUCCESS);

    /*Allocating one PBuf initially for Rx side*/
    Lwip2Emac_allocRxPackets(hLwip2Emac);
    /*Getting Link status from Emac driver*/
    int32_t port = Lwip2Emac_getDriverLinkStatus(hLwip2Emac);

    if(port)
    {
        DebugP_log("[LWIPIF_LWIP]Link is Up on port %d\r\n",port);
    }
    else
    {
        DebugP_log("[LWIPIF_LWIP]Link is down");
    }

     /* Init aleTicks */
    hLwip2Emac->aleTicks=(LWIP_ICSS_ALE_AGE_OUT_TIME / 100U);

     /* Init aleTimerActive */
    hLwip2Emac->aleTimerActive=0;
}

/**
 *  @breif Lwip2emac__getDriverLinkStatus
 *  Gets link status from emac driver handle
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2Emac structure pointer.
 *
 *  \retval
 *      portNumber
*/
int32_t Lwip2Emac_getDriverLinkStatus(Lwip2Emac_Handle hLwip2Emac)
{
    uint8_t portNumber = 0;
    if(1 == ((ICSS_EMAC_Object *)hLwip2Emac->emacHandle->object)->linkStatus[0])
    {
        hLwip2Emac->linkIsUp = 1;
        portNumber = 1;
    }
    else if(1 == ((ICSS_EMAC_Object *)hLwip2Emac->emacHandle->object)->linkStatus[1])
    {
        hLwip2Emac->linkIsUp = 1;
        portNumber = 2;
    }

    return (portNumber);
}

/**
 * Initializes Ethernet peripheral hardware
 *
 *  *  \param[in]  netifpNETIFDevice
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      Lwip2Emac_Handle
 *
 */
Lwip2Emac_Handle Lwip2Emac_open(struct netif *netif)
{
    Lwip2Emac_Handle hLwip2Emac;
    hLwip2Emac = &gLwip2EmacObj;

    /* Initialize the allocated memory block. */
    memset(hLwip2Emac, 0, sizeof(Lwip2EMAC_Object));

    /* lwIP interface relevant for this adaptation layer */
    hLwip2Emac->netif = netif;

    /* Initialize EMAC driver */
    Lwip2Emac_initConfig(hLwip2Emac);

    hLwip2Emac->initDone = TRUE;

    return hLwip2Emac;
}

/*!
 *  @b Lwip2emac_close
 *  @n
 *      Closes Ethernet peripheral and disables interrupts.
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Emac_close(Lwip2Emac_Handle hLwip2Emac)
{

    DebugP_assert(NULL != hLwip2Emac);

    /* Set the translation layer shutdown flag */
    hLwip2Emac->shutDownFlag = true;

    pbuf_free(hLwip2Emac->rxPbufPkt);

    /* Delete the semaphore objects */
    SemaphoreP_destruct(&hLwip2Emac->shutDownSemObj);

    /* TODO: Terminate and delete tasks */

    /* Clear the allocated translation */
    memset(hLwip2Emac, 0, sizeof(Lwip2EMAC_Object));
}

/*!
 *  @b Lwip2Emac_sendTxPackets
 *  @n
 *      Routine to send out queued Tx packets to the hardware driver
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *
 *  \retval
 *      void
 */
int32_t Lwip2Emac_sendTxPackets(Lwip2Emac_Handle hLwip2Emac, struct pbuf *p)
{
    uint32_t retVal = SystemP_SUCCESS;
    uint32_t txPort;
    int32_t ret;
    ICSS_EMAC_TxArgument txArgs;

    /*Check for link status*/
    Lwip2Emac_getDriverLinkStatus(hLwip2Emac);

    if (hLwip2Emac->linkIsUp)
    {
        if(ICSS_EMAC_MODE_SWITCH == (((ICSS_EMAC_Attrs*)hLwip2Emac->emacHandle->attrs)->portMask))
        { /*Switch Mode*/
            txPort = ICSS_EMAC_PORT_0;
        }
        else if(ICSS_EMAC_MODE_MAC1 == (((ICSS_EMAC_Attrs*)hLwip2Emac->emacHandle->attrs)->portMask))
        {
            txPort = ICSS_EMAC_PORT_1;
        }
        else if(ICSS_EMAC_MODE_MAC2 == (((ICSS_EMAC_Attrs*)hLwip2Emac->emacHandle->attrs)->portMask))
        {
            txPort = ICSS_EMAC_PORT_2;
        }
        else
        {
            txPort = ICSS_EMAC_PORT_0;
        }
        txArgs.icssEmacHandle = hLwip2Emac->emacHandle;
        txArgs.lengthOfPacket = p->len;
        txArgs.portNumber = txPort;
        txArgs.queuePriority = ICSS_EMAC_QUEUE4;
        txArgs.srcAddress = (uint8_t *)p->payload;

        ret = ICSS_EMAC_txPacket(&txArgs, NULL);
        pbuf_free(p);

        if(ret != 0)
        {
            retVal = SystemP_FAILURE;
        }

    }
    return (retVal);

}


/*!
 *  @b Lwip2Enet_ioctl
 *  @n
 *  Low level driver Ioctl interface. This interface can be used for
 *  ALE configuration, control, statistics
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *  \param[in]  cmd
 *      Ioctl command.
 *  \param[in]  pBuf
 *      Ioctl buffer with commands and params to set/get
 *      configuration from hardware.
 *  \param[in]  size
 *      Size of Ioctl buffer.
 *
 *  \retval
 *      void
 */
int32_t Lwip2Emac_ioctl(Lwip2Emac_Handle hLwip2Emac,
                       uint32_t cmd,
                       void *param,
                       uint32_t size)
{
    /*TO DO*/
    return 0;
}

/**
* @brief     This function should be called every time there is an EMAC device Rx
*             interrupt to process the packet
* @details  Note that the application has the responsibility for mapping the
*             physical device index to the correct EMAC_serviceCheck() function. If
*             more than one EMAC device is on the same interrupt, the function must be
*             called for each device.
*
* @param  hLwip2Emac       handle to the opened EMAC device
* @param  queueNum         priority Queue
* @param  userArg          Args

* @retval     SystemP_SUCCESS (0)
*                SystemP_FALIURE   - Packet provided by firmware has invalid length
* @pre     Lwip2Emac_open must be called before calling this API.
*
*/
int32_t Lwip2Emac_serviceRx(void *icssEmacHandleVoidPtr, void *queueNum, void *userArg)
{
    uint32_t retVal = SystemP_SUCCESS;
    uint32_t  packetLength;
    ICSS_EMAC_RxArgument rxArgs;
    Lwip2Emac_Handle hLwip2emac = (Lwip2Emac_Handle)userArg;

    if(hLwip2emac->initDone)
    {
        rxArgs.icssEmacHandle = hLwip2emac->emacHandle;

        rxArgs.queueNumber = ((uint32_t )queueNum);

        struct pbuf* hPbufPacket = hLwip2emac->rxPbufPkt;

        /*Rx Task*/
        if (hPbufPacket)
        {
                rxArgs.destAddress =  (uint32_t)hPbufPacket->payload;
                packetLength =  ICSS_EMAC_rxPktGet(&rxArgs, NULL);
                if (packetLength > 0)
                {
                    /* Fill in the necessary packet header fields */
                    hPbufPacket->len = hPbufPacket->tot_len = (uint16_t)(packetLength) & (0xFFFFU);
                    /* Pass the received packet to the LwIP stack */
                    LWIPIF_LWIP_input(hLwip2emac->netif, hPbufPacket);
                }
                else
                {
                    retVal = SystemP_FAILURE;
                }
        }
        else
        {
            retVal = SystemP_FAILURE;
            DebugP_log("[LWIPIF_LWIP]Packet Dropped!: Rx pbuf is empty");

        }
    }
    else
    {
        DebugP_log("[LWIPIF_LWIP]Packet Dropped!: Rx callback is called before Interface layer handle initilaisation");
        /*TO DO: Update low level driver Rx related pointers before dropping packet*/
        retVal = SystemP_FAILURE;
    }
    return (retVal);
}

/*!
 *  @b Lwip2Emac_getHandle
 *  @n
 *  Provides Lwip2EMAC_Object structure pointer to application for setting userArg in Rx callback during low level driver init.
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2Emac_getHandle(Lwip2Emac_Handle *AppLwipHandle)
{
    *(AppLwipHandle) = &gLwip2EmacObj;

    return;
}
/*---------------------------------------------------------------------------*\
 |                           Local Function Definitions                        |
 \*---------------------------------------------------------------------------*/
/*!
 *  @b Lwip2Emac_periodicFxn
 *  @n
 *  Periodic function for polling, updates link status
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *
 *  \retval
 *      void
 */
 void Lwip2Emac_periodicFxn(Lwip2Emac_Handle hLwip2Emac)
{
    int32_t port = Lwip2Emac_getDriverLinkStatus(hLwip2Emac);

    static uint32_t   aleTickCount = 0;
    ICSS_EMAC_IoctlCmd ioctlParams;

    ICSS_EMAC_Handle icsshandle = hLwip2Emac->emacHandle;
    ICSS_EMAC_StormPrevention* strmPreventionEnable1=NULL;
    ICSS_EMAC_StormPrevention* strmPreventionEnable2=NULL;

    if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EMAC_Attrs*)icsshandle->attrs)->portMask)
    {
        if(((ICSS_EMAC_Attrs*)icsshandle->attrs)->learningEnable)
        {
            /*Increment counter in learning for ageing*/
            ioctlParams.command = ICSS_EMAC_LEARN_CTRL_INC_COUNTER;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_LEARNING_CTRL, 0, (void*)&ioctlParams);

            if( (0u != hLwip2Emac->aleTimerActive) && (0 != hLwip2Emac->aleTicks))
            {
                /* Ageoutnow. aleTickCount can become greater if timeout period is changed in between */
                aleTickCount = aleTickCount + 1U;
                if( (aleTickCount) >=  (hLwip2Emac->aleTicks))
                {
                    ioctlParams.command = ICSS_EMAC_LEARN_CTRL_AGEING;
                    ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_LEARNING_CTRL, ICSS_EMAC_PORT_1, (void*)&ioctlParams);
                    ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_LEARNING_CTRL, ICSS_EMAC_PORT_2, (void*)&ioctlParams);
                    aleTickCount = 0;
                }
            }
        }
    }


    /*Reset the credit values used for Storm prevention*/
    if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EMAC_Attrs*)icsshandle->attrs)->portMask)
    {
        strmPreventionEnable1 = (ICSS_EMAC_StormPrevention*)(((ICSS_EMAC_Object*)(icsshandle)->object)->stormPrev);
        strmPreventionEnable2 = ((ICSS_EMAC_StormPrevention*)(((ICSS_EMAC_Object*)(icsshandle)->object)->stormPrev)) + 1;
        if((strmPreventionEnable1->suppressionEnabledBC) | (strmPreventionEnable2->suppressionEnabledBC))
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_BC;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, 0, (void*)&ioctlParams);
        }
        if((strmPreventionEnable1->suppressionEnabledMC) | (strmPreventionEnable2->suppressionEnabledMC))
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_MC;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, 0, (void*)&ioctlParams);
        }
        if((strmPreventionEnable1->suppressionEnabledUC) | (strmPreventionEnable2->suppressionEnabledUC))
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_UC;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, 0, (void*)&ioctlParams);
        }
    }
    else
    {
        strmPreventionEnable1 = (ICSS_EMAC_StormPrevention*)(((ICSS_EMAC_Object*)(icsshandle)->object)->stormPrev);
        if(strmPreventionEnable1->suppressionEnabledBC)
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_BC;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, (uint8_t)port, (void*)&ioctlParams);
        }
        if(strmPreventionEnable1->suppressionEnabledMC)
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_MC;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, (uint8_t)port, (void*)&ioctlParams);
        }
        if(strmPreventionEnable1->suppressionEnabledUC)
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_UC;
            ICSS_EMAC_ioctl(icsshandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, (uint8_t)port, (void*)&ioctlParams);
        }
    }
}

/*!
 *  @b Lwip2Emac_allocRxPackets
 *  @n
 *  Allocates one PBuf at init time for Rx
 *
 *  \param[in]  hLwip2Emac
 *      Lwip2EMAC_Object structure pointer.
 *
 *  \retval
 *      void
 */
static void Lwip2Emac_allocRxPackets(Lwip2Emac_Handle hLwip2Emac)
{
    struct pbuf* hPbufPacket;
    uint32_t bufSize;

    bufSize = PBUF_POOL_BUFSIZE;
    /*Allocating Pbuf for Rx*/
    hPbufPacket = pbuf_alloc(PBUF_RAW, bufSize, PBUF_POOL);
    if (hPbufPacket != NULL)
    {
        hPbufPacket->tot_len = hPbufPacket->len;
        hLwip2Emac->rxPbufPkt = hPbufPacket;
    }
    else
    {
        DebugP_log("[LWIPIF_LWIP]ERROR: Rx Pbuf_alloc() failure.!\n");
    }
}