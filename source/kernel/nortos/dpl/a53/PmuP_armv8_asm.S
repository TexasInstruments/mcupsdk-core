
/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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


/* FUNCTION DEF: void PmuP_disableCounters(void); */
        .global PmuP_disableCycleCounterIntr
        .type PmuP_disableCycleCounterIntr  , %function
PmuP_disableCycleCounterIntr:
        mov     x1, #0x80000000
        msr     pmintenclr_el1, x1          /* Disable interrupt request */
        ret

/* FUNCTION DEF: void PmuP_clearCycleCounterOverflowStatus(void); */
        .global PmuP_clearCycleCounterOverflowStatus
        .type PmuP_clearCycleCounterOverflowStatus  , %function
PmuP_clearCycleCounterOverflowStatus:
        mov     x1, #0x80000000
        msr     pmovsclr_el0, x1
        ret

/* FUNCTION DEF: void PmuP_resetAndEnableAllCounters(void); */
        .global PmuP_resetAndEnableAllCounters
        .type PmuP_resetAndEnableAllCounters  , %function
PmuP_resetAndEnableAllCounters:
        mrs     x0, pmcr_el0
        orr     x0, x0, #0x7
        msr     pmcr_el0, x0                /* Reset and enable all counters */
        ret

/* FUNCTION DEF: uint64_t CycleCounterP_getCount64(void); */
        .global CycleCounterP_getCount64
        .type CycleCounterP_getCount64, %function
CycleCounterP_getCount64:
        mrs     x0, pmccntr_el0           /* Read cycle count */
        ret

/* FUNCTION DEF: void PmuP_startCycleCounter(void); */
        .global PmuP_startCycleCounter
        .type PmuP_startCycleCounter , %function
PmuP_startCycleCounter:
        mov     x1, #0x80000000
        msr     pmcntenset_el0, x1
        ret