/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on
*/

/* Topic: Description
	This file contains  declarations of the various benchmark functions.
*/

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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "core_portme.h"
#if HAS_STDIO
#include <stdio.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Configuration: TOTAL_DATA_SIZE
	Define total size for data algorithms will operate on
*/
#ifndef TOTAL_DATA_SIZE
#define TOTAL_DATA_SIZE     2*1000
#endif

#define SEED_ARG            0
#define SEED_FUNC           1
#define SEED_VOLATILE       2

#define MEM_STATIC          0
#define MEM_MALLOC          1
#define MEM_STACK           2


#if HAS_PRINTF
#define ee_printf printf
#endif

#if MAIN_HAS_NORETURN
#define MAIN_RETURN_VAL
#define MAIN_RETURN_TYPE void
#else
#define MAIN_RETURN_VAL 0
#define MAIN_RETURN_TYPE int
#endif

/* Algorithm IDS */
#define ID_LIST 	        (1<<0)
#define ID_MATRIX 	        (1<<1)
#define ID_STATE 	        (1<<2)
#define ALL_ALGORITHMS_MASK (ID_LIST|ID_MATRIX|ID_STATE)
#define NUM_ALGORITHMS      3


/*matrix benchmark related stuff */
#define MATDAT_INT          1
#if MATDAT_INT
typedef ee_s16 MATDAT;
typedef ee_s32 MATRES;
#else
typedef ee_f16 MATDAT;
typedef ee_f32 MATRES;
#endif

/* Typedef: secs_ret
	For machines that have floating point support, get number of seconds as a double.
	Otherwise an unsigned int.
*/
#if HAS_FLOAT
typedef double secs_ret;
#else
typedef ee_u32 secs_ret;
#endif

typedef struct MAT_PARAMS_S
{
	int N;
	MATDAT *A;
	MATDAT *B;
	MATRES *C;
} mat_params;

/* state machine related stuff */
/* List of all the possible states for the FSM */
typedef enum CORE_STATE
{
	CORE_START=0,
	CORE_INVALID,
	CORE_S1,
	CORE_S2,
	CORE_INT,
	CORE_FLOAT,
	CORE_EXPONENT,
	CORE_SCIENTIFIC,
	NUM_CORE_STATES
} core_state_e ;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Helper structure to hold results */
typedef struct RESULTS_S
{
	/* inputs */
	ee_s16	seed1;		/* Initializing seed */
	ee_s16	seed2;		/* Initializing seed */
	ee_s16	seed3;		/* Initializing seed */
	void	*memblock[4];	/* Pointer to safe memory location */
	ee_u32	size;		/* Size of the data */
	ee_u32 iterations;		/* Number of iterations to execute */
	ee_u32	execs;		/* Bitmask of operations to execute */
	struct list_head_s *list;
	mat_params mat;
	/* outputs */
	ee_u16	crc;
	ee_u16	crclist;
	ee_u16	crcmatrix;
	ee_u16	crcstate;
	ee_s16	err;
	/* ultithread specific */
	core_portable port;
} core_results;

/* list data structures */
typedef struct list_data_s
{
	ee_s16 data16;
	ee_s16 idx;
} list_data;

typedef struct list_head_s
{
	struct list_head_s *next;
	struct list_data_s *info;
} list_head;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Actual benchmark execution in iterate */
void *iterate(void *pres);
void start_time(void);
void stop_time(void);
CORE_TICKS get_time(void);
secs_ret time_in_secs(CORE_TICKS ticks);

/* Misc useful functions */
ee_u16 crcu8(ee_u8 data, ee_u16 crc);
ee_u16 crc16(ee_s16 newval, ee_u16 crc);
ee_u16 crcu16(ee_u16 newval, ee_u16 crc);
ee_u16 crcu32(ee_u32 newval, ee_u16 crc);
ee_u8 check_data_types();
void *portable_malloc(ee_size_t size);
void portable_free(void *p);
ee_s32 parseval(char *valstring);

/* Multicore execution handling */
#if (MULTITHREAD>1)
ee_u8 core_start_parallel(core_results *res);
ee_u8 core_stop_parallel(core_results *res);
#endif

/* list benchmark functions */
list_head *core_list_init(ee_u32 blksize, list_head *memblock, ee_s16 seed);
ee_u16 core_bench_list(core_results *res, ee_s16 finder_idx);

/* state benchmark functions */
void core_init_state(ee_u32 size, ee_s16 seed, ee_u8 *p);
ee_u16 core_bench_state(ee_u32 blksize, ee_u8 *memblock,
		ee_s16 seed1, ee_s16 seed2, ee_s16 step, ee_u16 crc);

/* matrix benchmark functions */
ee_u32 core_init_matrix(ee_u32 blksize, void *memblk, ee_s32 seed, mat_params *p);
ee_u16 core_bench_matrix(mat_params *p, ee_s16 seed, ee_u16 crc);