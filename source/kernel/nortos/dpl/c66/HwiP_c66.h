/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

#ifndef HWIP_C66_H_
#define HWIP_C66_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define HwiP_INTC_BASE_ADDR             (CSL_DSP_ICFG_U_BASE)

/** \brief Start CPU vector ID from which the ECM is mapped/
 *  ECM 0 -> 12, ECM 1 -> 13 and so on */
#define HwiP_VECTID_ECM_START           (12U)
/** \brief Number of CPU interrupts reserved for ECM starting from above */
#define HwiP_NUM_ECM                    (4U)
/** \brief Each ECM can handle 32 SOC events */
#define HwiP_EVENTS_PER_ECM             (32U)
/** \brief Number of user interrupts that can be supported */
#define HwiP_MAX_EVENTS                 (HwiP_NUM_ECM * HwiP_EVENTS_PER_ECM)

/** \brief Number of reserved hardware interrupts (INT0 through INT3) */
#define HwiP_INTC_NUM_RESV_INTR         (4U)
/** \brief Number of maskable hardware interrupts (INT4 through INT15) */
#define HwiP_INTC_NUM_MASK_INTR         (12U)
/** \brief Number of hardware interrupts (INT0 through INT15) */
#define HwiP_INTC_NUM_INTR              (HwiP_INTC_NUM_RESV_INTR + HwiP_INTC_NUM_MASK_INTR)

typedef void (*HwiP_IntcIsr)(void);

/**
 * \brief typedef for RTOS hook to change Raise access from User to Supervisor Privilege.
 */
typedef int32_t (*HwiP_raisePrivilegeFxnPtr)(void);

/**
 * \brief typedef for RTOS hook to restore Privilege access.
 */
typedef void (*HwiP_restorPrivilegeFxnPtr)(int32_t runningPrivilege);

/**
 *  \brief Enumeration of the exception
 *  These are the symbols used along with the Exception Clear API
 */
typedef enum
{
    HWIP_INTC_EXCEPTION_NMI = 31,
    /**< Symbol for NMI */
    HWIP_INTC_EXCEPTION_EXT = 30,
    /**< Symbol for External Exception */
    HWIP_INTC_EXCEPTION_INT = 1,
    /**< Symbol for Internal Exception*/
    HWIP_INTC_EXCEPTION_SW = 0
    /**< Symbol for Software Exception */
} HwiP_IntcExcep;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief Intc regsister overlay */
typedef struct  {
    volatile uint32_t EVTFLAG[4];
    volatile uint8_t RSVD0[16];
    volatile uint32_t EVTSET[4];
    volatile uint8_t RSVD1[16];
    volatile uint32_t EVTCLR[4];
    volatile uint8_t RSVD2[48];
    volatile uint32_t EVTMASK[4];
    volatile uint8_t RSVD3[16];
    volatile uint32_t MEVTFLAG[4];
    volatile uint8_t RSVD4[16];
    volatile uint32_t EXPMASK[4];
    volatile uint8_t RSVD5[16];
    volatile uint32_t MEXPFLAG[4];
    volatile uint8_t RSVD6[20];
    volatile uint32_t INTMUX1;
    volatile uint32_t INTMUX2;
    volatile uint32_t INTMUX3;
    volatile uint8_t RSVD7[112];
    volatile uint32_t INTXSTAT;
    volatile uint32_t INTXCLR;
    volatile uint32_t INTDMASK;
} HwiP_IntcRegs;

typedef volatile HwiP_IntcRegs         *HwiP_IntcRegsOvly;

typedef struct HwiP_IntcVect_s
{
    uint32_t            currVectId;
    HwiP_IntcIsr        isr[HwiP_INTC_NUM_INTR];
} HwiP_IntcVect;

typedef struct HwiP_Ctrl_s
{
    HwiP_IntcRegsOvly   pIntcRegs;
    HwiP_FxnCallback    isr[HwiP_MAX_EVENTS];
    void               *isrArgs[HwiP_MAX_EVENTS];
    HwiP_FxnCallback    nmiHandler;
    void               *nmiArgs;
} HwiP_Ctrl;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern HwiP_Ctrl gHwiCtrl;
extern HwiP_IntcVect gHwiIntcIntrTable;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void HwiP_assignIntrHandlers(void);
void HwiP_intcEcmDispatcher(uint32_t ecmId);

/* HWI ASM fxns */
void HwiP_Entry(void);
void HwiP_intcIvpSet(void);
uint32_t HwiP_intcInterruptEnable(uint32_t vectId);
uint32_t HwiP_intcInterruptDisable(uint32_t vectId);
void HwiP_intcInterruptClear(uint32_t vectId);
void HwiP_intcInterruptSet(uint32_t vectId);
void HwiP_intcInterruptRestore(uint32_t vectId, uint32_t restoreVal);
uint32_t HwiP_intcQueryInterruptStatus(uint32_t vectId);
int32_t HwiP_intcGlobalEnable(uint32_t *prevState);
int32_t HwiP_intcGlobalDisable(uint32_t *prevState);
int32_t HwiP_intcGlobalRestore(uint32_t prevState);
int32_t HwiP_intcGlobalNmiEnable(void);
int32_t HwiP_intcGlobalExcepEnable(void);
int32_t HwiP_intcGlobalExtExcepEnable(void);
int32_t HwiP_intcGlobalExcepClear(HwiP_IntcExcep exc);

/**
 * \brief Set Priority to Interrupt
 *
 * \param intNum [in]   Interrupt number
 * \param priority [in] Priority
 */
void HwiP_setPri(uint32_t intNum, uint32_t priority);

/**
 * \brief App callback invoked at end of ISR handler.
 *
 * - Application must implement this function.
 *
 * \note Invoked from ISR context so callback should return ASAP
 * \note Currently enabled only for C66x SafeRTOS port
 */
void HwiP_appInterruptHandlerHook(uint32_t interruptVectorNum );

/**
 * \brief Function to register Privilege access function.
 *
 * \param hookFxnPtr  [in] callback function pointer
 */
void HwiP_registerRaisePrivilegeHandlerHook(HwiP_raisePrivilegeFxnPtr hookFxnPtr);

/**
 * \brief Function to register Restore Privilege access function.
 *
 * \param hookFxnPtr  [in] callback function pointer
 */
void HwiP_registerRestorePrivilegeHandlerHook(HwiP_restorPrivilegeFxnPtr hookFxnPtr);


#ifdef __cplusplus
}
#endif

#endif /* HWIP_C66_H_ */
