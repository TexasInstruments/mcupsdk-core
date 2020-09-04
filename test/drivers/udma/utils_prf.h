/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \ingroup UTILS_API
 *  \defgroup UTILS_PRF_API Profiling API
 *  @{
 */

/**
 *  \file utils_prf.h
 *
 *  \brief Profiling API.
 *
 *   - APIs to measure and print elasped time @ 64-bit precision
 *      - Utils_prfTsXxxx
 *
 *   - APIs to measure and print CPU load at task, HWI, SWI, global level
 *      - Utils_prfLoadXxxx
 */

#ifndef UTILS_PRF_H_
#define UTILS_PRF_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/TaskP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Maximum supported profiling objects */
#define UTILS_PRF_MAX_HNDL              (64U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Profiling load object.
 */
typedef struct
{
    int32_t cpuLoad;
    /**< CPU load. */
    int32_t hwiLoad;
    /**< HWI load. */
    int32_t swiLoad;
    /**< SWI load. */
    int32_t tskLoad;
    /**< TSK load. */
} Utils_PrfLoad;

/** \brief Typedef for the loadupdate function for the user. */
typedef void (*Utils_loadUpdate)(Utils_PrfLoad *prfLoad);

/**
 *  \brief Profiling time-stamp object.
 */
typedef struct
{
    char        name[32];
    /**< Name. */
    uint32_t    isAlloc;
    /**< Indicates if the object is initialized (used). */
    uint64_t    startTs;
    /**< Start time stamp value. */
    uint64_t    totalTs;
    /**< Total time stamp value. */
    uint32_t    count;
    /**< Number of times time stamp delta is calculated. */
    uint32_t    numFrames;
    /**< Total number of frames being used for time-stamping. */
} Utils_PrfTsHndl;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Initializes the profiling utility.
 *
 *  This function must be called before using any peformance or Timestamp utils
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfInit(void);

/**
 *  \brief De-initializes the profiling utility.
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfDeInit(void);

/**
 *  \brief Creates the handle for the time stamp taking.
 *
 *  \param name     [IN] Name of the time stamp object
 *
 *  \return Valid handle on success, else NULL
 */
Utils_PrfTsHndl *Utils_prfTsCreate(const char *name);

/**
 *  \brief Deletes the handle for the timestamp.
 *
 *  \param pHndl    [IN] Handle to be deleted
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfTsDelete(Utils_PrfTsHndl *pHndl);

/**
 *  \brief Start taking the timestamp.
 *
 *  \param pHndl    [IN] Handle to be time stamp object
 *
 *  \return Initial time stamp value on success
 */
uint64_t Utils_prfTsBegin(Utils_PrfTsHndl *pHndl);

/**
 *  \brief Stop taking the timestamp.
 *
 *  \param pHndl     [IN] Handle to be time stamp object
 *  \param numFrames [IN] Number of associated with the time stamp
 *
 *  \return Final time stamp value on success
 */
uint64_t Utils_prfTsEnd(Utils_PrfTsHndl *pHndl, uint32_t numFrames);

/**
 *  \brief Update the CPU load information for all profile handles
 */
void Utils_prfLoadUpdate(void);

/**
 *  \brief Calculates the difference between the timestamp.
 *
 *  \param pHndl     [IN] Handle to be time stamp object
 *  \param startTime [IN] Start time stamp value
 *  \param numFrames [IN] Number of associated with the time stamp
 *
 *  \return Final time stamp value on success
 */
uint64_t Utils_prfTsDelta(Utils_PrfTsHndl *pHndl,
                          uint64_t         startTime,
                          uint32_t         numFrames);

/**
 *  \brief Resets the timestamp counter for that handle.
 *
 *  \param pHndl     [IN] Handle to be time stamp object
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfTsReset(Utils_PrfTsHndl *pHndl);

/**
 *  \brief Gets the 64-bit timer ticks
 *
 *  \return Current 64-bit timer ticks value on success
 */
uint64_t Utils_prfTsGet64(void);

/**
 *  \brief Prints the timestamp difference and resets the counter thereafter
 *         depending on specified resetAfterPrint parameter.
 *
 *  \param pHndl           [IN] Handle to be time stamp object
 *  \param resetAfterPrint [IN] Indicates whether time stamp values should be
 *                              reset after printing
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfTsPrint(Utils_PrfTsHndl *pHndl, uint32_t resetAfterPrint, uint32_t trace);

/**
 *  \brief Prints the timestamp difference for all registered handles and resets
 *         the counter thereafter depending on specified resetAfterPrint
 *         parameter.
 *
 *  \param resetAfterPrint [IN] Indicates whether time stamp values should be
 *                              reset after printing
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfTsPrintAll(uint32_t resetAfterPrint, uint32_t trace);

/**
 *  \brief Registers a task for load calculation.
 *
 *  \param pTsk            [IN] Handle to task object to be registered for load
 *                              calculation
 *  \param name            [IN] Name to be associated with the registered handle
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfLoadRegister(TaskP_Object *pTsk, const char *name);

/**
 *  \brief Un-registers a task for load calculation.
 *
 *  \param pTsk            [IN] Handle to task object to be unregistered for
 *                              load calculation
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfLoadUnRegister(TaskP_Object *pTsk);

/**
 *  \brief Prints loads for all the registered tasks. Also prints information
 *         for each task depending on the specified printTskLoad parameter.
 *
 *  \param printTskLoad    [IN] Indicates whether load information for each
 *                              registered task should be printed.
 *
 *  \return 0 on success, else failure
 */
int32_t Utils_prfLoadPrintAll(uint32_t printTskLoad, uint32_t trace);

/**
 *  \brief Start taking the performance load for all the registered tasks.
 */
void Utils_prfLoadCalcStart(void);

/**
 *  \brief Stop taking the load for all the registered tasks.
 */
void Utils_prfLoadCalcStop(void);

/**
 *  \brief Reset the load calculation mainly for next cycle of run.
 */
void  Utils_prfLoadCalcReset(void);

#ifdef __cplusplus
}
#endif

#endif /* ifndef UTILS_PRF_H_ */

/* @} */
