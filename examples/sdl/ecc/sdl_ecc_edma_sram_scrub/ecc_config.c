/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "edma_rti_sram_scrub.h"
#include <sdl/sdl_ecc.h>
#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/TimerP.h>
#include <dpl_interface.h>
#include <drivers/soc.h>
#if defined(SOC_AM263X)
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define ESM_REGISTERS                                   ((volatile SDL_esmRegs *) CSL_TOP_ESM_U_BASE)

#define SDL_MSS_L2_MEM_INIT_ADDR                        (SDL_MSS_CTRL_U_BASE+SDL_MSS_CTRL_L2IOCRAM_MEM_INIT)
#define SDL_MSS_L2_MEM_INIT_DONE_ADDR                   (SDL_MSS_CTRL_U_BASE+SDL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR                 (SDL_ECC_AGG_R5SS0_CORE0_U_BASE+SDL_MSS_ECC_AGGA_ERROR_STATUS1)
#define SDL_ECC_MSS_L2_BANK_MEM_INIT                    (0x0CU) /*Bank 3*/
#define SDL_EXAMPLE_ECC_AGGR                            SDL_SOC_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                          SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID
/* ECC Agg SEC Enable defines */
#define APP_ECC_AGGR_SEC_ENABLE_SET_REG0_ADDR           (CSL_ECC_AGG_TOP_U_BASE + CSL_MSS_ECC_AGG_MSS_SEC_ENABLE_SET_REG0)

#define APP_ECC_AGGR_SEC_ENABLE_SET_REG0_VAL            (CSL_MSS_ECC_AGG_MSS_SEC_ENABLE_SET_REG0_MSS_L2SLV0_ENABLE_SET_MASK | \
                                                        CSL_MSS_ECC_AGG_MSS_SEC_ENABLE_SET_REG0_MSS_L2SLV1_ENABLE_SET_MASK | \
                                                        CSL_MSS_ECC_AGG_MSS_SEC_ENABLE_SET_REG0_MSS_L2SLV2_ENABLE_SET_MASK)

/* ECC Agg SEC Status defines */
#define APP_ECC_AGGR_SEC_STATUS_REG0_ADDR               (CSL_ECC_AGG_TOP_U_BASE + CSL_MSS_ECC_AGG_MSS_SEC_STATUS_REG0)

#define APP_ECC_AGGR_SEC_STATUS_PEND_VAL_SRAM_BANK_0    (CSL_MSS_ECC_AGG_MSS_SEC_STATUS_REG0_MSS_L2SLV0_PEND_MASK)
#define APP_ECC_AGGR_SEC_STATUS_PEND_VAL_SRAM_BANK_1    (CSL_MSS_ECC_AGG_MSS_SEC_STATUS_REG0_MSS_L2SLV1_PEND_MASK)
#define APP_ECC_AGGR_SEC_STATUS_PEND_VAL_SRAM_BANK_2    (CSL_MSS_ECC_AGG_MSS_SEC_STATUS_REG0_MSS_L2SLV2_PEND_MASK)

/* ECC Agg SEC EOI defines */
#define APP_ECC_AGGR_SEC_EOI_REG_ADDR                   (CSL_ECC_AGG_TOP_U_BASE + CSL_MSS_ECC_AGG_MSS_SEC_EOI_REG)
#define APP_ECC_AGGR_SEC_EOI_REG_ACK_VAL                (CSL_MSS_ECC_AGG_MSS_SEC_EOI_REG_EOI_WR_MASK)

/* ECC Agg SEC Error Status defines */
#define APP_ECC_AGGR_ERROR_STATUS1_ADDR_OFS             (CSL_MSS_ECC_AGG_MSS_ERROR_STATUS1)
#define APP_ECC_AGGR_ERROR_STATUS2_ADDR_OFS             (CSL_MSS_ECC_AGG_MSS_ERROR_STATUS2)
#define APP_ECC_AGGR_ERROR_STATUS3_ADDR_OFS             (CSL_MSS_ECC_AGG_MSS_ERROR_STATUS3)

#define APP_ECC_AGGR_ERROR_STATUS1_CLEAR_VAL            (CSL_MSS_ECC_AGG_MSS_ERROR_STATUS1_CLR_ECC_SEC_MASK | \
                                                        CSL_MSS_ECC_AGG_MSS_ERROR_STATUS1_CLR_ECC_OTHER_MASK | \
                                                        CSL_MSS_ECC_AGG_MSS_ERROR_STATUS1_CLR_PARITY_ERR_MASK | \
                                                        CSL_MSS_ECC_AGG_MSS_ERROR_STATUS1_CLR_CTRL_REG_ERR_MASK)
#define APP_ECC_AGGR_ERROR_STATUS3_CLEAR_VAL            (CSL_MSS_ECC_AGG_MSS_ERROR_STATUS3_CLR_SVBUS_TIMEOUT_ERR_MASK)


#define ECC_SEC_INT                                     (19U)
#define ECC_DED_INT                                     (20U)

#define ESM_GROUPS                                      (4U)

// ESM EN enable interrupt value
#define ESM_EN_ENABLE_VALUE                             (0b1111)
#define CFG_ERR_INT                                     (0U)
#define LOW_PRIO_INT                                    (1U)
#define HIGH_PRIO_INT                                   (2U)

#define ESM_REGISTERS                                   ((volatile SDL_esmRegs *) CSL_TOP_ESM_U_BASE)
#define GET_EVENT_GROUP(event)                          ((event) / 32U)
#define GET_EVENT_BIT(event)                            ((event) % 32U)

#define SDL_ESM_HI_PRI_RESETVAL                         (0xFFFFFFFFU)
#define SDL_ESM_LOW_PRI_RESETVAL                        (0xFFFFFFFFU)

/* SRAM Banks Vector ID defines */
#define APP_SRAM_BANK_0_VECTOR_ID                       (0U)
#define APP_SRAM_BANK_1_VECTOR_ID                       (1U)
#define APP_SRAM_BANK_2_VECTOR_ID                       (2U)

#define SDL_MSS_L2_MAX_MEM_SECTIONS                     (1U)

#define ECC_VECTOR_REG_ADDR                             (CSL_ECC_AGG_TOP_U_BASE + CSL_MSS_ECC_AGG_MSS_ECC_VECTOR)
/* ========================================================================== */
/*                            Typedefs                                        */
/* ========================================================================== */
typedef struct {
    uint32_t eccVectorIdx;
    uint32_t eccStatusPendVal;
} App_SramBankInfo;

/* ========================================================================== */
/*                            Variables                                       */
/* ========================================================================== */
volatile bool esmError = false;

static App_SramBankInfo gSramBanksInfo[APP_SRAM_SCRUB_NUM_BANKS] =  {
    {APP_SRAM_BANK_0_VECTOR_ID, APP_ECC_AGGR_SEC_STATUS_PEND_VAL_SRAM_BANK_0},
    {APP_SRAM_BANK_1_VECTOR_ID, APP_ECC_AGGR_SEC_STATUS_PEND_VAL_SRAM_BANK_1},
    {APP_SRAM_BANK_2_VECTOR_ID, APP_ECC_AGGR_SEC_STATUS_PEND_VAL_SRAM_BANK_2}
};

/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */
extern void ESM_lowPriorityISR(void* args);

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
static void App_eccAggSecEnable(void);
static inline uint32_t App_getEccVectorIdx(uint32_t bankIdx);
static inline uint32_t App_getEccStatusPendVal(uint32_t bankIdx);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{
    int32_t retVal = 0;
    const uint32_t interruptStatus = ESM_REGISTERS->LOW_PRI;

    // Interrupt is no longer asserted
    if(interruptStatus == SDL_ESM_LOW_PRI_RESETVAL)
    { 
        retVal = 1; 
    }
    else 
    {
        uint32_t levelInterrupt = (interruptStatus & SDL_ESM_LOW_PRI_LVL_MASK);

        // service interrupt based on source
        switch(levelInterrupt)
        {
            case ECC_SEC_INT:
                App_serviceSecInterrupt();
                break;
            default:
                // shouldn't enter here
                break;
        }

        // clear interrupt bit
        ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)levelInterrupt)].STS |= (1UL << GET_EVENT_BIT((uint8_t)levelInterrupt));
        ESM_REGISTERS->EOI |= (LOW_PRIO_INT & SDL_ESM_EOI_KEY_MASK);
    }
return retVal;
}

void App_configEccEsm(void)
{
    ESM_init();
    App_eccAggSecEnable();
}

uint32_t App_getEccSecStatus(void)
{
    return HW_RD_REG32(APP_ECC_AGGR_SEC_STATUS_REG0_ADDR);
}

bool App_checkEccSecStatusPending(uint32_t bankIdx, uint32_t secStatus)
{
    uint32_t pendVal = App_getEccStatusPendVal(bankIdx);

    return ((secStatus & pendVal) == pendVal);
}

uint32_t App_getEccFaultAddr(uint32_t bankIdx)
{
    uint32_t eccVectorIdx = App_getEccVectorIdx(bankIdx);

    uint32_t eccRowAddress = ECCAGG_readRegister(eccVectorIdx, APP_ECC_AGGR_ERROR_STATUS2_ADDR_OFS);

    return (APP_SRAM_SCRUB_START_ADDR + (bankIdx * APP_SRAM_SCRUB_BANK_SIZE_BYTES) + (eccRowAddress * APP_SRAM_SCRUB_ECC_ROW_WIDTH_BYTES));
}

void App_clearEccFault(uint32_t bankIdx)
{
    uint32_t eccVectorIdx = App_getEccVectorIdx(bankIdx);

    uint32_t status1 = ECCAGG_readRegister(eccVectorIdx, APP_ECC_AGGR_ERROR_STATUS1_ADDR_OFS);
    uint32_t status3 = ECCAGG_readRegister(eccVectorIdx, APP_ECC_AGGR_ERROR_STATUS3_ADDR_OFS);

    ECCAGG_writeRegister(eccVectorIdx, APP_ECC_AGGR_ERROR_STATUS1_ADDR_OFS, status1 | APP_ECC_AGGR_ERROR_STATUS1_CLEAR_VAL);
    ECCAGG_writeRegister(eccVectorIdx, APP_ECC_AGGR_ERROR_STATUS3_ADDR_OFS, status3 | APP_ECC_AGGR_ERROR_STATUS3_CLEAR_VAL);

    /* Acknowledge in EOI register */
    HW_WR_REG32(APP_ECC_AGGR_SEC_EOI_REG_ADDR, APP_ECC_AGGR_SEC_EOI_REG_ACK_VAL);
}

uint32_t ECCAGG_readRegister(uint8_t endpointId, uint16_t registerOffset)
{
    // Read mask includes register offset to read from, setting read bit to true, and endpoint ID
    uint32_t read_mask = ((uint32_t)registerOffset << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_ADDRESS_SHIFT) | 
                                                       (1UL << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_SHIFT) | 
                                              (endpointId << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_ECC_VECTOR_SHIFT);
    HW_WR_REG32(ECC_VECTOR_REG_ADDR, read_mask);

    // measured worst-case for this serial read is 35ms based on GPIO toggle
    volatile uint32_t counter = 0x800000; //roughly 70 ms wait cycle assuming below loop takes 2 cycles
    do
    {
        __asm("NOP");
    } while ((counter-- != 0U) && (((HW_RD_REG32(ECC_VECTOR_REG_ADDR) >> CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_DONE_SHIFT) & 1UL) == 0UL)); // TRM says to poll this bit until its set, indicating successful read

    return HW_RD_REG32(CSL_ECC_AGG_TOP_U_BASE + registerOffset);
}

bool ECCAGG_writeRegister(uint8_t endpointId, uint16_t registerOffset, uint32_t val)
{
    // Write mask explicitly clears read bit, sets endpoint ID
    uint32_t write_mask = (0UL << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_SHIFT) | (endpointId << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_ECC_VECTOR_SHIFT);
    HW_WR_REG32(ECC_VECTOR_REG_ADDR, write_mask);
    HW_WR_REG32((CSL_ECC_AGG_TOP_U_BASE + registerOffset), val);

    // Ensure write went through
    uint32_t read_result = ECCAGG_readRegister(endpointId, registerOffset);
    return (read_result == val);
}
/* ========================================================================== */
/*                          Internal Function Definitions                     */
/* ========================================================================== */
static void App_eccAggSecEnable(void)
{
    HW_WR_REG32(APP_ECC_AGGR_SEC_ENABLE_SET_REG0_ADDR, APP_ECC_AGGR_SEC_ENABLE_SET_REG0_VAL);
}

static inline uint32_t App_getEccVectorIdx(uint32_t bankIdx)
{
    return gSramBanksInfo[bankIdx].eccVectorIdx;
}

static inline uint32_t App_getEccStatusPendVal(uint32_t bankIdx)
{
    return gSramBanksInfo[bankIdx].eccStatusPendVal;
}

/* Nothing past this point */
