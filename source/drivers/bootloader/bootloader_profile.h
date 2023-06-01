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

/**
 *  \file bootloader_profile.h
 *
 *  \brief Bootloader Driver profiling API/interface file.
 */

#ifndef BOOTLOADER_PROFILE_H_
#define BOOTLOADER_PROFILE_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */
/**
 *  \brief  This function initializes the profiler object and resets the cycleCounter
 */
void Bootloader_profileReset(void);

/**
 *  \brief  This function logs the string passed along with the PMU timestamp to an internal data-structure
 *
 *  \param  pointName Name of profile point as string
 */
void Bootloader_profileAddProfilePoint(char *pointName);

/**
 *  \brief  This function prints all the saved logs/profile points
 */
void Bootloader_profilePrintProfileLog(void);
/**
 *  \brief  This function updates the profile data structure with appimage size
 *
 *  \param  size Size of the appimage in bytes
 */
void Bootloader_profileUpdateAppimageSize(uint32_t size);

/**
 *  \brief  This function updates the profile data structure with boot media and clock speed
 * 
 *  \param clk Clock speed of the boot media - OSPI, QSPI, MMCSD
 * 
 *  \param id Boot media ID, defined in bootloader.h
 */
void Bootloader_profileUpdateMediaAndClk(uint32_t clk, uint32_t id);

/**
 *  \brief  This function adds the core passed to the profile data structure
 *
 *  \param  coreId CSL Core Id of the CPU
 */
void Bootloader_profileAddCore(uint32_t coreId);


#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_PROFILE_H_ */