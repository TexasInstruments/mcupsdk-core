/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \defgroup DRV_PMU_MODULE APIs for PMU
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the PMU module. The APIs
 *  can be used by other drivers to get access to PMU and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file pmu.h
 *
 *  \brief PMU Driver API/interface file.
 */

#ifndef PMU_V0_H_
#define PMU_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/pmu/r5f/csl_arm_r5_pmu.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief Data structure used in #PMU_Config
 *
 *  It specifies the name and type of a particular PMU event
 */
typedef struct
{    
    uint32_t type;
    /* Type of the event. Refer \ref R5_PMU_EventTypes */
    const char* name;
    /* Name of the event, user can give any logical name */
} PMU_EventCfg;

/**
 *  \brief Data structure to be used with PMU_init
 *
 *  It specifies the counters to be enabled for a particular PMU profiling instance
 */
typedef struct
{
    uint32_t bCycleCounter;
    uint32_t numEventCounters;
    PMU_EventCfg *eventCounters;

} PMU_Config;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  Function to initialize the PMU for profiling
 *
 *  \param  cfg      #PMU_Config
 *
 *  \return #SystemP_SUCCESS if command read was successful; else error on failure
 *
 */
int32_t PMU_init(PMU_Config *cfg);

/**
 *  \brief  Function to start profiling with the current configuration
 *
 *  \param  name    Name of the profile point. Should be a const char *
 *
 *  \return #SystemP_SUCCESS if command read was successful; else error on failure
 *
 */
int32_t PMU_profileStart(const char *name);

/**
 *  \brief  Function to end profiling with the current configuration
 *
 *  Make sure that a \ref PMU_profileStart is called with the same name before this 
 *  is invoked.
 *
 *  \param  name   Name of the profile point. Should be a const char *
 *
 *  \return #SystemP_SUCCESS if command read was successful; else error on failure
 *
 */
int32_t PMU_profileEnd(const char *name);

/**
 *  \brief  Function to print the profiling stats for a particular profiling point
 *
 *  This has to be called after \ref PMU_profileEnd is called for the point
 * 
 *  \param  name   Name of the profile point. Should be a const char *
 *
 */
void PMU_profilePrintEntry(const char *name);

/**
 *  \brief  Function to print the profiling stats for all the points done till now.
 *
 *  This has to be called after \ref PMU_profileEnd is called for at least one point.
 *
 */
void PMU_profilePrint(void);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PMU_V0_H_ */


/** @} */