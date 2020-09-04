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

#ifndef HWIP_ARMV8_GIC_H
#define HWIP_ARMV8_GIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/a53/cslr_gic500.h>
#include <kernel/nortos/dpl/a53/common_armv8.h>

/* External interrupt from the system
 * ID00-ID15: SGI (Software generated interrupts)
 * ID16-ID31: PPI (Private Peripheral interrupts)
 * ID32-ID991: SPI (Shared peripheral interrupts)
 */
#define HWIP_GICD_SGI_PPI_INTR_ID_MAX   (32u)
#define HWIP_GICD_SPI_INTR_ID_MAX       (992u)
#define HWIP_GICD_SPI_INTR_COUNT_MAX    (960U)
#define HwiP_MAX_INTERRUPTS             (1024u)

#define HWIP_GIC_BASE_ADDR              (0x1800000UL)

#define HWIP_GIC_DEFAULT_PRIORITY       ((uint32_t) 0x9U)

/* Array Size Definition Macros */
#define HWIP_GIC_NUM_GICR_CORE                               4u

#define HwiP_writeSystemReg(regName, val) \
    {                                                                \
        __asm__ __volatile__ (                                       \
            "msr " #regName ", %0"                                   \
            :: "r" (val)                                             \
        );                                                           \
    }

#define HwiP_readSystemReg(regName, val)  \
    {                                                                \
        __asm__ __volatile__ (                                       \
            "mrs %0, " #regName                                      \
            : "=r" (val)                                             \
        );                                                           \
    }


/** \brief Spi Interrupt type - (Level / Edge ) triggered */
typedef enum {
	HWIP_SPIINTTYPE_LEVEL=0, /**< Level */
	HWIP_SPIINTTYPE_EDGE=3, /**< Edge */
} HwiP_SpiIntType_t;


typedef struct HwiP_Ctrl_s {

    HwiP_FxnCallback isr[HwiP_MAX_INTERRUPTS];
    void *isrArgs[HwiP_MAX_INTERRUPTS];

} HwiP_Ctrl;


extern HwiP_Ctrl gHwiCtrl;

#ifdef SMP_FREERTOS

/* Flag to check if execution state is in ISR for core0 */
extern uint32_t	gHwiInIsrFlagCore0;
/* Flag to check if execution state is in ISR for core1 */
extern uint32_t	gHwiInIsrFlagCore1;

#else

/* Flag to check if execution state is in ISR */
extern uint32_t	gHwiInIsrFlag;

#endif

/**
 *
 * \brief The Default Interrupt Handler.
 *
 * This is the default interrupt handler for all interrupts.  It simply loops
 * forever so that the system state is preserved for observation by a
 * debugger.  Since interrupts should be disabled before unregistering the
 * corresponding handler, this should never be called.
 *
 * \param     dummy argument.
 *
 **/
void HwiP_defaultHandler(void *dummy);

#ifdef __cplusplus
}
#endif

#endif /* HWIP_ARMV8_GIC_H */
