/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/uart.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ASM_SUPPORT
#define SELF_TCM_READ_ADDRESS           (0x00001000U)
#define SELF_TCM_WRITE_ADDRESS          (0x00007F00U)
#define SELF_TCM_WRITE_ADDRESS1         (0x00007F08U)
#define SRAM_READ_ADDRESS               (0x70000000U)
#define SRAM_WRITE_ADDRESS              (0x70000000U)
#define SRAM_WRITE_ADDRESS1             (0x70000008U)
#define NON_SELF_TCM_ACCESS_ADDRESS     (0x78400000U)
#define NON_SELF_TCM_ACCESS_ADDRESS1    (0x78400008U)
#define FLASH_ACCESS_ADDRESS            (0x60000000U)

typedef struct {
    double cycles;
} benchmark_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

benchmark_t latencyCalculate_Write32(uint32_t address, uint32_t address1) __attribute__ ((section ("TCM_function.TCM_function_attr")));

benchmark_t latencyCalculate_Write64(uint32_t address) __attribute__ ((section ("TCM_function.TCM_function_attr")));

benchmark_t latencyCalculate_Read32(uint32_t address) __attribute__ ((section ("TCM_function.TCM_function_attr")));

benchmark_t latencyCalculate_Read64(uint32_t address) __attribute__ ((section ("TCM_function.TCM_function_attr")));
