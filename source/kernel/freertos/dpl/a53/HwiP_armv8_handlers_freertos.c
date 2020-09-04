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

#include <stddef.h>
#include <kernel/nortos/dpl/a53/HwiP_armv8_gic.h>
#include <kernel/nortos/dpl/a53/common_armv8.h>

#ifdef SMP_FREERTOS
uint32_t gHwiInIsrFlagCore0 = 0;
uint32_t gHwiInIsrFlagCore1 = 0;
#else
uint32_t gHwiInIsrFlag = 0;
#endif

void HwiP_intrHandler()
{
    uint64_t     intNum;

#ifdef SMP_FREERTOS
    uint64_t coreId;
    coreId = Armv8_getCoreId();
    if(coreId == 0)
    {
        gHwiInIsrFlagCore0++;
    }
    else if(coreId == 1)
    {
        gHwiInIsrFlagCore1++;
    }
#else
    gHwiInIsrFlag++;
#endif

    /* Acknowledge Interrupt */
    HwiP_readSystemReg(s3_0_c12_c12_0, intNum); /* icc_iar1_el1 */

    /* Ignore spurious ints */
    if (intNum < HWIP_GICD_SPI_INTR_ID_MAX)
    {
        HwiP_enable();
        if (gHwiCtrl.isr[intNum] != NULL)
        {

            /* Call user ISR function in system mode and then return back to IRQ mode */
            gHwiCtrl.isr[intNum](gHwiCtrl.isrArgs[intNum]);
        }
        HwiP_disable();

        /* Signal End of Interrupt */
        HwiP_writeSystemReg(s3_0_c12_c12_1, intNum); /* icc_eoir1_el1 */
    }

#ifdef SMP_FREERTOS
    if(coreId == 0)
    {
        gHwiInIsrFlagCore0--;
    }
    else if(coreId == 1)
    {
        gHwiInIsrFlagCore1--;
    }
#else
    gHwiInIsrFlag--;
#endif

}


void HwiP_defaultHandler(void *dummy)
{
    /* Go into an infinite loop.*/
    volatile uint32_t loop = 1U;
    while (1U == loop)
    {
        ;
    }
}


void HwiP_abortHandler(uint64_t *excStack)
{
    /* Read the ESR_EL1 register */
    uint32_t esr_el1, exception_class;
    volatile uint32_t loop = 1U;
    esr_el1 = excStack[1];

    exception_class = (esr_el1 >> 26);

    switch (exception_class)
    {
        /* Instruction abort */
        case 0x20:
        case 0x21:
        /* Data abort */
        case 0x24:
        case 0x25:
        /* Illegal execution state or PC/SP alignment fault */
        case 0x0E:
        case 0x22:
        case 0x26:
        /* SVC */
        case 0x15:
        default:
            /* Go into an infinite loop.*/
            while (1U == loop)
            {
                ;
            }
            break;
    }
}