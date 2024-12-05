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

#include <stdio.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/optiflash.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/am263px/cslr_rl2_of_r5fss0_core0.h>
#include "csl_arm_r5_pmu.h"
#include "appprofile.h"
#include "load.h"

/**
 * @brief Minimum FLC region size
 * 
 */
#define APP_MINIMUM_FLC_REGION_SIZE             (32*1024U)

/**
 * @brief Number of peripheral register read done by load function 1
 * 
 * the actual number of reads are, this number, times 256
 * 
 */
#define LOAD_FNX1_READ_REGISTER_CNT         4
/**
 * @brief Number of peripheral register writes done by load function 1
 * 
 */
#define LOAD_FNX1_WRITE_REGISTER_CNT        1
/**
 * @brief Number of computation type A done by load function 1
 * 
 */
#define LOAD_FNX1_COMPUTATION_A_CNT         1
/**
 * @brief Number of computation type B done by load function 1
 * 
 */
#define LOAD_FNX1_COMPUTATION_B_CNT         4

/**
 * @brief Number of peripheral register read done by load function 2
 * 
 * the actual number of reads are, this number, times 256
 * 
 */
#define LOAD_FNX2_READ_REGISTER_CNT         2
/**
 * @brief Number of peripheral register writes done by load function 2
 * 
 */
#define LOAD_FNX2_WRITE_REGISTER_CNT        1
/**
 * @brief Number of computation type A done by load function 2
 * 
 */
#define LOAD_FNX2_COMPUTATION_A_CNT         1
/**
 * @brief Number of computation type B done by load function 2
 * 
 */
#define LOAD_FNX2_COMPUTATION_B_CNT         1
/**
 * @brief Number of peripheral register read done by load function 3
 * 
 * the actual number of reads are, this number, times 256
 * 
 */
#define LOAD_FNX3_READ_REGISTER_CNT         1
/**
 * @brief Number of peripheral register writes done by load function 3
 * 
 */
#define LOAD_FNX3_WRITE_REGISTER_CNT        1
/**
 * @brief Number of computation type A done by load function 3
 * 
 */
#define LOAD_FNX3_COMPUTATION_A_CNT         1
/**
 * @brief Number of computation type B done by load function 3
 * 
 */
#define LOAD_FNX3_COMPUTATION_B_CNT         0
/**
 * @brief Number of peripheral register read done by load function 4
 * 
 * the actual number of reads are, this number, times 256
 * 
 */
#define LOAD_FNX4_READ_REGISTER_CNT         0
/**
 * @brief Number of peripheral register writes done by load function 4
 * 
 */
#define LOAD_FNX4_WRITE_REGISTER_CNT        1
/**
 * @brief Number of computation type A done by load function 4
 * 
 */
#define LOAD_FNX4_COMPUTATION_A_CNT         1
/**
 * @brief Number of computation type B done by load function 4
 * 
 */
#define LOAD_FNX4_COMPUTATION_B_CNT         1

/**
 * @brief Linker symbol for start of FLC overlay region in SRAM
 * 
 */
extern char FLC_OVL_RGN_END;

/**
 * @brief Linker symbol for end of FLC overlay region in SRAM
 * 
 */
extern char FLC_OVL_RGN_START;

/**
 * @brief Function pointer typedef for the testing function 
 * 
 */
typedef void (*pProfFnx_t) (void);

/**
 * @brief Type 1 Load function that will be executed from flash 
 * 
 */
void load_function_t1_xip(void);

/**
 * @brief Type 1 Load function that will be executed from SRAM 
 * 
 */
void load_function_t1_sram(void);

/**
 * @brief Type 2 Load function that will be executed from flash 
 * 
 */
void load_function_t2_sram(void);

/**
 * @brief Type 2 Load function that will be executed from SRAM 
 * 
 */
void load_function_t2_xip(void);

/**
 * @brief Type 3 Load function that will be executed from flash 
 * 
 */
void load_function_t3_sram(void);

/**
 * @brief Type 3 Load function that will be executed from SRAM 
 * 
 */
void load_function_t3_xip(void);

/**
 * @brief Type 4 Load function that will be executed from flash 
 * 
 */
void load_function_t4_sram(void);

/**
 * @brief Type 4 Load function that will be executed from SRAM 
 * 
 */
void load_function_t4_xip(void);


void ovlAccn(uint32_t function_address)
{
    uint32_t flc_region_size = (uint32_t)&FLC_OVL_RGN_END - (uint32_t)&FLC_OVL_RGN_START;

    DebugP_assert(flc_region_size >= APP_MINIMUM_FLC_REGION_SIZE);

    FLC_RegionInfo region;
    region.baseAddress = CSL_RL2_REGS_R5SS0_CORE0_U_BASE;
    region.destinationStartAddress = (uint32_t)&FLC_OVL_RGN_START;
    region.regionId = 0;
    region.sourceStartAddress = function_address;
    region.sourceEndAddress = function_address + flc_region_size;

    FLC_configureRegion(&region);

    /*
        Increase the transaction buffer size of FLC IP so that CPU can send request, 
        back to back, to achieve maximum read throughput. This is a workaround and its 
        value should be set to 0 back after after its being used or if RL2 is being used.
    */
    *((volatile uint32_t*)(CSL_RL2_REGS_R5SS0_CORE0_U_BASE + 0x104)) |= 4<<24;

    FLC_startRegion(&region);
}

static inline void profile_fnx( pProfFnx_t pFnx, const char tag[], int ovlEn)
{
    volatile uint32_t cyc = CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
    volatile uint32_t ix = CSL_armR5PmuReadCntr(0);
    volatile uint32_t icm = CSL_armR5PmuReadCntr(1);
    volatile uint32_t ica = CSL_armR5PmuReadCntr(2);

    CacheP_wbInvAll(CacheP_TYPE_ALL);
    if(ovlEn)
    {
        ovlAccn((uint32_t)pFnx);
    }
    pFnx();

    // final measurement
    uint32_t endCyc =  CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
    cyc = endCyc > cyc ? endCyc - cyc : (uint64_t)(0xffffffff + endCyc) - cyc;
    ix = CSL_armR5PmuReadCntr(0) - ix;
    icm = CSL_armR5PmuReadCntr(1) - icm;
    ica = CSL_armR5PmuReadCntr(2) - ica;

    /* Cycles Per Instruction: here the average value is being calculated */
    float cpi = (float)cyc / (float)ix;

    DebugP_log("%-15s:%-15d%-15d%3.1f \r\n",tag, cyc , ix  , cpi);
}

void sram_overlay_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    config_pmu();

    DebugP_log("\r\nTest started\r\n");

    DebugP_log("%-15s:%-15s%-15s%s\r\n", "Label", "CPU Cycles", "CPU Instr.", "CPI");

    profile_fnx(load_function_t1_sram,  "T1 SRAM", FALSE);
    profile_fnx(load_function_t1_xip,   "T1 XIP ", FALSE);
    profile_fnx(load_function_t1_xip,   "T1 OVL ", TRUE);
    DebugP_log("\r\n");

    profile_fnx(load_function_t2_sram,  "T2 SRAM", FALSE);
    profile_fnx(load_function_t2_xip,   "T2 XIP ", FALSE);
    profile_fnx(load_function_t2_xip,   "T2 OVL ", TRUE);
    DebugP_log("\r\n");

    profile_fnx(load_function_t3_sram,  "T3 SRAM", FALSE);
    profile_fnx(load_function_t3_xip,   "T3 XIP ", FALSE);
    profile_fnx(load_function_t3_xip,   "T3 OVL ", TRUE);
    DebugP_log("\r\n");

    profile_fnx(load_function_t4_sram,  "T4 SRAM", FALSE);
    profile_fnx(load_function_t4_xip,   "T4 XIP ", FALSE);
    profile_fnx(load_function_t4_xip,   "T4 OVL ", TRUE);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void load_function_t1_xip(void)
{
    COMP_BLOCK_256(
        LOAD_FNX1_READ_REGISTER_CNT, 
        LOAD_FNX1_WRITE_REGISTER_CNT, 
        LOAD_FNX1_COMPUTATION_A_CNT, 
        LOAD_FNX1_COMPUTATION_B_CNT
    );
}

void load_function_t1_sram(void)
{
    COMP_BLOCK_256(
        LOAD_FNX1_READ_REGISTER_CNT,
        LOAD_FNX1_WRITE_REGISTER_CNT, 
        LOAD_FNX1_COMPUTATION_A_CNT,
        LOAD_FNX1_COMPUTATION_B_CNT
    );
}

void load_function_t2_sram(void)
{
    COMP_BLOCK_256(
        LOAD_FNX2_READ_REGISTER_CNT, 
        LOAD_FNX2_WRITE_REGISTER_CNT, 
        LOAD_FNX2_COMPUTATION_A_CNT, 
        LOAD_FNX2_COMPUTATION_B_CNT
    );
}

void load_function_t2_xip(void)
{
    COMP_BLOCK_256(
        LOAD_FNX2_READ_REGISTER_CNT, 
        LOAD_FNX2_WRITE_REGISTER_CNT, 
        LOAD_FNX2_COMPUTATION_A_CNT, 
        LOAD_FNX2_COMPUTATION_B_CNT
    );
}

void load_function_t3_sram(void)
{

    COMP_BLOCK_256(
        LOAD_FNX3_READ_REGISTER_CNT, 
        LOAD_FNX3_WRITE_REGISTER_CNT, 
        LOAD_FNX3_COMPUTATION_A_CNT, 
        LOAD_FNX3_COMPUTATION_B_CNT
    );
}

void load_function_t3_xip(void)
{
    COMP_BLOCK_256(
        LOAD_FNX3_READ_REGISTER_CNT, 
        LOAD_FNX3_WRITE_REGISTER_CNT, 
        LOAD_FNX3_COMPUTATION_A_CNT, 
        LOAD_FNX3_COMPUTATION_B_CNT
    );
}

void load_function_t4_sram(void)
{
    COMP_BLOCK_256(
        LOAD_FNX4_READ_REGISTER_CNT, 
        LOAD_FNX4_WRITE_REGISTER_CNT, 
        LOAD_FNX4_COMPUTATION_A_CNT, 
        LOAD_FNX4_COMPUTATION_B_CNT
    );
}

void load_function_t4_xip(void)
{    
    COMP_BLOCK_256(
        LOAD_FNX4_READ_REGISTER_CNT, 
        LOAD_FNX4_WRITE_REGISTER_CNT, 
        LOAD_FNX4_COMPUTATION_A_CNT, 
        LOAD_FNX4_COMPUTATION_B_CNT
    );
}