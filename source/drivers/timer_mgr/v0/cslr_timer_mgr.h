/********************************************************************
 * Copyright (C) 2017 Texas Instruments Incorporated.
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
 *
 *  Name        : cslr_timer_mgr.h
*/
#ifndef CSLR_TIMER_MGR_H_
#define CSLR_TIMER_MGR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t CNTL;
    volatile uint32_t COUNTER;
    volatile uint8_t  Resv_160[148];
    volatile uint32_t TIMEOUT_STATUS0;
    volatile uint32_t TIMEOUT_STATUS1;
    volatile uint32_t TIMEOUT_STATUS_BANK0;
    volatile uint32_t TIMEOUT_STATUS_BANK1;
    volatile uint8_t  Resv_256[80];
    volatile uint32_t STATUS[64];
} CSL_timer_mgrRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TIMER_MGR_PID                                                      (0x00000000U)
#define CSL_TIMER_MGR_CNTL                                                     (0x00000004U)
#define CSL_TIMER_MGR_COUNTER                                                  (0x00000008U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0                                          (0x000000A0U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1                                          (0x000000A4U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK0                                     (0x000000A8U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK1                                     (0x000000ACU)
#define CSL_TIMER_MGR_STATUS(STATUS)                                           (0x00000100U+((STATUS)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_TIMER_MGR_PID_SCHEME_MASK                                          (0xC0000000U)
#define CSL_TIMER_MGR_PID_SCHEME_SHIFT                                         (0x0000001EU)
#define CSL_TIMER_MGR_PID_SCHEME_MAX                                           (0x00000003U)

#define CSL_TIMER_MGR_PID_BU_MASK                                              (0x30000000U)
#define CSL_TIMER_MGR_PID_BU_SHIFT                                             (0x0000001CU)
#define CSL_TIMER_MGR_PID_BU_MAX                                               (0x00000003U)

#define CSL_TIMER_MGR_PID_FUNCTION_MASK                                        (0x0FFF0000U)
#define CSL_TIMER_MGR_PID_FUNCTION_SHIFT                                       (0x00000010U)
#define CSL_TIMER_MGR_PID_FUNCTION_MAX                                         (0x00000FFFU)

#define CSL_TIMER_MGR_PID_RTL_VER_MASK                                         (0x0000F800U)
#define CSL_TIMER_MGR_PID_RTL_VER_SHIFT                                        (0x0000000BU)
#define CSL_TIMER_MGR_PID_RTL_VER_MAX                                          (0x0000001FU)

#define CSL_TIMER_MGR_PID_MAJOR_REV_MASK                                       (0x00000700U)
#define CSL_TIMER_MGR_PID_MAJOR_REV_SHIFT                                      (0x00000008U)
#define CSL_TIMER_MGR_PID_MAJOR_REV_MAX                                        (0x00000007U)

#define CSL_TIMER_MGR_PID_CUSTOM_MASK                                          (0x000000C0U)
#define CSL_TIMER_MGR_PID_CUSTOM_SHIFT                                         (0x00000006U)
#define CSL_TIMER_MGR_PID_CUSTOM_MAX                                           (0x00000003U)

#define CSL_TIMER_MGR_PID_MINOR_REV_MASK                                       (0x0000003FU)
#define CSL_TIMER_MGR_PID_MINOR_REV_SHIFT                                      (0x00000000U)
#define CSL_TIMER_MGR_PID_MINOR_REV_MAX                                        (0x0000003FU)

/* CNTL */

#define CSL_TIMER_MGR_CNTL_MASS_ENABLE_MASK                                    (0x00001000U)
#define CSL_TIMER_MGR_CNTL_MASS_ENABLE_SHIFT                                   (0x0000000CU)
#define CSL_TIMER_MGR_CNTL_MASS_ENABLE_MAX                                     (0x00000001U)

#define CSL_TIMER_MGR_CNTL_MAX_TIMER_MASK                                      (0x00000FFEU)
#define CSL_TIMER_MGR_CNTL_MAX_TIMER_SHIFT                                     (0x00000001U)
#define CSL_TIMER_MGR_CNTL_MAX_TIMER_MAX                                       (0x000007FFU)

#define CSL_TIMER_MGR_CNTL_ENABLE_MASK                                         (0x00000001U)
#define CSL_TIMER_MGR_CNTL_ENABLE_SHIFT                                        (0x00000000U)
#define CSL_TIMER_MGR_CNTL_ENABLE_MAX                                          (0x00000001U)

#define CSL_TIMER_MGR_CNTL_ENABLE_VAL_DISALBED                                 (0x0U)
#define CSL_TIMER_MGR_CNTL_ENABLE_VAL_ENABLED                                  (0x1U)

/* COUNTER */

#define CSL_TIMER_MGR_COUNTER_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_TIMER_MGR_COUNTER_VAL_SHIFT                                        (0x00000000U)
#define CSL_TIMER_MGR_COUNTER_VAL_MAX                                          (0xFFFFFFFFU)

/* TIMEOUT_STATUS0 */

#define CSL_TIMER_MGR_TIMEOUT_STATUS0_VALID0_MASK                              (0x00800000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0_VALID0_SHIFT                             (0x00000017U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0_VALID0_MAX                               (0x00000001U)

#define CSL_TIMER_MGR_TIMEOUT_STATUS0_EXPIRED_TIMER0_MASK                      (0x007FF000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0_EXPIRED_TIMER0_SHIFT                     (0x0000000CU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0_EXPIRED_TIMER0_MAX                       (0x000007FFU)

#define CSL_TIMER_MGR_TIMEOUT_STATUS0_NUM_EXPIRED_TIMERS_MASK                  (0x00000FFFU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0_NUM_EXPIRED_TIMERS_SHIFT                 (0x00000000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS0_NUM_EXPIRED_TIMERS_MAX                   (0x00000FFFU)

/* TIMEOUT_STATUS1 */

#define CSL_TIMER_MGR_TIMEOUT_STATUS1_VALID2_MASK                              (0x00800000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_VALID2_SHIFT                             (0x00000017U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_VALID2_MAX                               (0x00000001U)

#define CSL_TIMER_MGR_TIMEOUT_STATUS1_EXPIRED_TIMER2_MASK                      (0x007FF000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_EXPIRED_TIMER2_SHIFT                     (0x0000000CU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_EXPIRED_TIMER2_MAX                       (0x000007FFU)

#define CSL_TIMER_MGR_TIMEOUT_STATUS1_VALID1_MASK                              (0x00000800U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_VALID1_SHIFT                             (0x0000000BU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_VALID1_MAX                               (0x00000001U)

#define CSL_TIMER_MGR_TIMEOUT_STATUS1_EXPIRED_TIMER1_MASK                      (0x000007FFU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_EXPIRED_TIMER1_SHIFT                     (0x00000000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS1_EXPIRED_TIMER1_MAX                       (0x000007FFU)

/* TIMEOUT_STATUS_BANK0 */

#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK0_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK0_VAL_SHIFT                           (0x00000000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK0_VAL_MAX                             (0xFFFFFFFFU)

/* TIMEOUT_STATUS_BANK1 */

#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK1_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK1_VAL_SHIFT                           (0x00000000U)
#define CSL_TIMER_MGR_TIMEOUT_STATUS_BANK1_VAL_MAX                             (0xFFFFFFFFU)

/* STATUS */

#define CSL_TIMER_MGR_STATUS_VAL_MASK                                          (0xFFFFFFFFU)
#define CSL_TIMER_MGR_STATUS_VAL_SHIFT                                         (0x00000000U)
#define CSL_TIMER_MGR_STATUS_VAL_MAX                                           (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t SETUP;
    volatile uint32_t CONTROL;
    volatile uint8_t  Resv_256[248];
} CSL_timer_mgr_timersRegs_page_entry;


typedef struct {
    CSL_timer_mgr_timersRegs_page_entry ENTRY[16];
} CSL_timer_mgr_timersRegs_page;


typedef struct {
    CSL_timer_mgr_timersRegs_page PAGE[128];
} CSL_timer_mgr_timersRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_SETUP(PAGE, ENTRY)                     (0x00000000U+((PAGE)*0x1000U)+((ENTRY)*0x100U))
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL(PAGE, ENTRY)                   (0x00000004U+((PAGE)*0x1000U)+((ENTRY)*0x100U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* SETUP */

#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_SETUP_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_SETUP_COUNT_SHIFT                      (0x00000000U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_SETUP_COUNT_MAX                        (0xFFFFFFFFU)

/* CONTROL */

#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_AUTORESET_MASK                 (0x00000100U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_AUTORESET_SHIFT                (0x00000008U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_AUTORESET_MAX                  (0x00000001U)

#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_EXPIRED_MASK                   (0x00000004U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_EXPIRED_SHIFT                  (0x00000002U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_EXPIRED_MAX                    (0x00000001U)

#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_SET_MASK                       (0x00000002U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_SET_SHIFT                      (0x00000001U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_SET_MAX                        (0x00000001U)

#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_ENABLE_MASK                    (0x00000001U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_TIMER_MGR_TIMERS_PAGE_ENTRY_CONTROL_ENABLE_MAX                     (0x00000001U)

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t EVENTIDX[2048];
} CSL_timer_mgr_oesRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TIMER_MGR_OES_EVENTIDX(EVENTIDX)                                   (0x00000000U+((EVENTIDX)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* EVENTIDX */

#define CSL_TIMER_MGR_OES_EVENTIDX_VAL_MASK                                    (0x0000FFFFU)
#define CSL_TIMER_MGR_OES_EVENTIDX_VAL_SHIFT                                   (0x00000000U)
#define CSL_TIMER_MGR_OES_EVENTIDX_VAL_MAX                                     (0x0000FFFFU)

#ifdef __cplusplus
}
#endif
#endif
