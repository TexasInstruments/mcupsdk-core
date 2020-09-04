/* =============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2017
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
 *  \file utils_prf_baremetal.c
 *
 *  \brief Profiling API utility file for baremetal.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include "udma_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    Bool        isAlloc;
    Char        name[32];
    TaskP_Object *pTsk;
    UInt64      totalTskThreadTime;
} Utils_PrfLoadObj;

typedef struct
{
    Utils_PrfTsHndl  tsObj[UTILS_PRF_MAX_HNDL];
    Utils_PrfLoadObj loadObj[UTILS_PRF_MAX_HNDL];
} Utils_PrfObj;

typedef struct
{
    UInt64 totalSwiThreadTime;
    UInt64 totalHwiThreadTime;
    UInt64 totalTime;
    UInt64 totalIdlTskTime;
} Utils_AccPrfLoadObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Utils_PrfObj        gUtils_prfObj;
static Utils_AccPrfLoadObj gUtils_accPrfLoadObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

Int32 Utils_prfInit(void)
{
    memset(&gUtils_prfObj, 0, sizeof (gUtils_prfObj));
    memset(&gUtils_accPrfLoadObj, 0, sizeof (Utils_AccPrfLoadObj));

    return (0);
}

Int32 Utils_prfDeInit(void)
{
    return (0);
}

Utils_PrfTsHndl *Utils_prfTsCreate(const Char *name)
{
    UInt32 hndlId;
    UInt32 cookie;
    Utils_PrfTsHndl *pHndl = NULL;

    cookie = HwiP_disable();

    for (hndlId = 0; hndlId < UTILS_PRF_MAX_HNDL; hndlId++)
    {
        pHndl = &gUtils_prfObj.tsObj[hndlId];

        if (FALSE == pHndl->isAlloc)
        {
            /* One less for NULL character */
            strncpy(pHndl->name, name, ((UInt32) sizeof (pHndl->name) - 1U));
            pHndl->name[sizeof (pHndl->name) - 1U] = (UInt8) '\0';
            pHndl->isAlloc = (Bool) TRUE;
            Utils_prfTsReset(pHndl);
            break;
        }
    }

    HwiP_restore(cookie);

    return (pHndl);
}

Int32 Utils_prfTsDelete(Utils_PrfTsHndl *pHndl)
{
    pHndl->isAlloc = (Bool) FALSE;
    return (0);
}

UInt64 Utils_prfTsBegin(Utils_PrfTsHndl *pHndl)
{
    pHndl->startTs = Utils_prfTsGet64();

    return (pHndl->startTs);
}

UInt64 Utils_prfTsEnd(Utils_PrfTsHndl *pHndl, UInt32 numFrames)
{
    return (Utils_prfTsDelta(pHndl, pHndl->startTs, numFrames));
}

UInt64 Utils_prfTsDelta(Utils_PrfTsHndl *pHndl,
                           UInt64              startTime,
                           UInt32              numFrames)
{
    UInt64 endTs;
    UInt32 cookie;

    endTs = Utils_prfTsGet64();

    cookie = HwiP_disable();

    pHndl->totalTs += (endTs - pHndl->startTs);
    pHndl->count++;
    pHndl->numFrames += numFrames;

    HwiP_restore(cookie);

    return (endTs);
}

Int32 Utils_prfTsReset(Utils_PrfTsHndl *pHndl)
{
    UInt32 cookie;

    cookie = HwiP_disable();

    pHndl->totalTs   = 0;
    pHndl->count     = 0;
    pHndl->numFrames = 0;

    HwiP_restore(cookie);

    return (0);
}

UInt64 Utils_prfTsGet64(void)
{
    UInt64 curTs = (UInt64) 0U;
    return (curTs);
}

Int32 Utils_prfTsPrint(Utils_PrfTsHndl *pHndl, uint32_t resetAfterPrint, uint32_t trace)
{
    UInt32       cpuKhz;
    UInt32       timeMs, fps, fpc;
    cpuKhz       = 24000U;


    timeMs = pHndl->totalTs / cpuKhz;

    if(0U == timeMs)
    {
        fps = 0U;
    }
    else
    {
        fps = (pHndl->numFrames * (UInt32) 1000U) / timeMs;
    }
    if(0U == pHndl->count)
    {
        fpc = 0U;
    }
    else
    {
        fpc = pHndl->numFrames / pHndl->count;
    }

    GT_7trace(
        trace, GT_INFO,
        " %d: PRF : %s : t: %d ms, count: %d, frames: %d, fps: %d, fpc: %d \r\n",
        AppUtils_getCurTimeInMsec(),
        pHndl->name,
        timeMs,       /* in msecs    */
        pHndl->count,
        pHndl->numFrames,
        fps,       /* frames per second */
        fpc        /* frames per count */
        );

    if (resetAfterPrint)
    {
        Utils_prfTsReset(pHndl);
    }

    return (0);
}

Int32 Utils_prfTsPrintAll(uint32_t resetAfterPrint, uint32_t trace)
{
    UInt32 hndlId;
    Utils_PrfTsHndl *pHndl;

    GT_0trace(trace, GT_INFO, "\r\n");

    for (hndlId = 0; hndlId < UTILS_PRF_MAX_HNDL; hndlId++)
    {
        pHndl = &gUtils_prfObj.tsObj[hndlId];

        if (TRUE == pHndl->isAlloc)
        {
            Utils_prfTsPrint(pHndl, resetAfterPrint, trace);
        }
    }

    GT_0trace(trace, GT_INFO, "\r\n");

    return (0);
}

Int32 Utils_prfLoadRegister(TaskP_Object *pTsk, const Char *name)
{
    UInt32 hndlId;
    UInt32 cookie;
    Int32  status = SystemP_FAILURE;
    Utils_PrfLoadObj *pHndl;

    cookie = HwiP_disable();

    for (hndlId = 0; hndlId < UTILS_PRF_MAX_HNDL; hndlId++)
    {
        pHndl = &gUtils_prfObj.loadObj[hndlId];

        if (FALSE == pHndl->isAlloc)
        {
            pHndl->isAlloc = (Bool) TRUE;
            pHndl->pTsk    = pTsk;
            /* One less for NULL character */
            strncpy(pHndl->name, name, ((UInt32) sizeof (pHndl->name) - 1U));
            pHndl->name[sizeof (pHndl->name) - 1U] = (UInt8) '\0';
            status = SystemP_SUCCESS;
            break;
        }
    }

    HwiP_restore(cookie);

    return (status);
}

Int32 Utils_prfLoadUnRegister(TaskP_Object *pTsk)
{
    UInt32 hndlId;
    UInt32 cookie;
    Int32  status = SystemP_FAILURE;
    Utils_PrfLoadObj *pHndl;

    cookie = HwiP_disable();

    for (hndlId = 0; hndlId < UTILS_PRF_MAX_HNDL; hndlId++)
    {
        pHndl = &gUtils_prfObj.loadObj[hndlId];

        if ((TRUE == pHndl->isAlloc) && (pHndl->pTsk == pTsk))
        {
            pHndl->isAlloc = (Bool) FALSE;
            status         = SystemP_SUCCESS;
            break;
        }
    }

    HwiP_restore(cookie);

    return (status);
}

Int32 Utils_prfLoadPrintAll(uint32_t printTskLoad, uint32_t trace)
{
    UInt32 hwiLoad, swiLoad, tskLoad, hndlId, cpuLoad;
    Utils_PrfLoadObj *pHndl;

    hwiLoad = (UInt32) ((gUtils_accPrfLoadObj.totalHwiThreadTime *
                         (UInt64) 100U) / gUtils_accPrfLoadObj.totalTime);
    swiLoad = (UInt32) ((gUtils_accPrfLoadObj.totalSwiThreadTime *
                         (UInt64) 100U) / gUtils_accPrfLoadObj.totalTime);
    cpuLoad = (UInt32) 100U -
              (UInt32) ((gUtils_accPrfLoadObj.totalIdlTskTime *
                         (UInt64) 100U) /
                        gUtils_accPrfLoadObj.totalTime);

    GT_0trace(trace, GT_INFO, "\r\n");
    GT_4trace(trace, GT_INFO,
              " %d: LOAD: CPU: %d%%, HWI: %d%%, SWI:%d%% \r\n",
              AppUtils_getCurTimeInMsec(),
              cpuLoad,
              hwiLoad,
              swiLoad);

    if (((Bool) TRUE) == printTskLoad)
    {
        for (hndlId = 0; hndlId < UTILS_PRF_MAX_HNDL; hndlId++)
        {
            pHndl = &gUtils_prfObj.loadObj[hndlId];

            if (TRUE == pHndl->isAlloc)
            {
                tskLoad = (UInt32) ((pHndl->totalTskThreadTime *
                                     (UInt64) 100U) /
                                    gUtils_accPrfLoadObj.totalTime);

                GT_3trace(trace, GT_INFO,
                          " %d: LOAD: TSK: %s: %d%% \r\n",
                          AppUtils_getCurTimeInMsec(),
                          pHndl->name,
                          tskLoad);
            }
        }
    }

    GT_0trace(trace, GT_INFO, "\r\n");

    return (0);
}

void Utils_prfLoadCalcStart(void)
{
    return;
}

void Utils_prfLoadCalcStop(void)
{
    return;
}

void Utils_prfLoadCalcReset(void)
{
    UInt32 hndlId;
    Utils_PrfLoadObj *pHndl;

    gUtils_accPrfLoadObj.totalHwiThreadTime = 0;
    gUtils_accPrfLoadObj.totalSwiThreadTime = 0;
    gUtils_accPrfLoadObj.totalTime          = 0;
    gUtils_accPrfLoadObj.totalIdlTskTime    = 0;

    /* Reset the performace loads accumulator */
    for (hndlId = 0; hndlId < UTILS_PRF_MAX_HNDL; hndlId++)
    {
        pHndl = &gUtils_prfObj.loadObj[hndlId];

        if (((Bool) TRUE == pHndl->isAlloc) &&
            (pHndl->pTsk != NULL))
        {
            pHndl->totalTskThreadTime = 0;
        }
    }

    return;
}

/* Function called by Loadupdate for each update cycle */
void Utils_prfLoadUpdate(void)
{
    return;
}
