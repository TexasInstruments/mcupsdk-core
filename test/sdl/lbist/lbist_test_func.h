/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**
 *  \file     lbist_test_func.h
 *
 *  \brief    This file contains LBIST test function structures
 *
 *  \details  LBIST Test function structures
 **/
#ifndef LBIST_TEST_FUNC_H
#define LBIST_TEST_FUNC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_lbist.h>


/* ========================================================================== */
/*                                Data Structures                             */
/* ========================================================================== */



typedef struct LBIST_TestHandle_s
{
  /** Core name */
  char coreName[16];
  /** Core instance */
  SDL_LBIST_inst instance;
  /** Indicate secondary core need to be handled */
  bool secondaryCoreNeeded;
  bool wfiCheckNeeded;
  /** Secondary core name */
  char secCoreName[16];
  /** Mask used to check CPU Status */
  uint32_t cpuStatusFlagMask;
  /** Core Processor Id */
  uint32_t tisciProcId;
  /** Secondary Core Processor Id */
  uint32_t tisciSecProcId;
  /** Core Device Id */
  uint32_t tisciDeviceId;
  /** Secondary Core Device Id */
  uint32_t tisciSecDeviceId;
  /** Number of Auxiliary devices needed for the test */
  uint32_t      numAuxDevices;
  /** List of Auxiliary devices needed for the test */
  uint32_t     *auxDeviceIdsP;
} LBIST_TestHandle_t;

#ifdef __cplusplus
}
#endif

#endif /* LBIST_TEST_FUNC_H */

/* Nothing past this point */
