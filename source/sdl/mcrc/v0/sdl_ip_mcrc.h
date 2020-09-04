/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 */

#ifndef SDL_IP_MCRC_H_
#define SDL_IP_MCRC_H_

#include <stddef.h>
#include <stdbool.h>
#include "sdl_mcrc.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 *  \defgroup SDL_IP_MCRC_API MCRC Low-Level API
 *  \ingroup SDL_MCRC_MODULE
 *
 *  This module contains the Low-Level APIs to program and use the MCRC module.
 *
 *  @{
 */

/**
 * \brief  The max value that can be passed to Bus Tracing Control APIs as the
 *         ctrlFlag value.
 */
#define SDL_MCRC_MAX_CTRL_FLAG_VAL                     (1U)

/**
 *  \anchor SDL_MCRC_IntrPriority_t
 *  \name MCRC Interrupt priority
 *  @{
 */

/**
 * \brief  The offset for the highest pending priority interrupt. These interrupt
 *         offset returned in #SDL_MCRC_getHighestPriorityIntrStatus function
 */
typedef uint32_t SDL_MCRC_IntrPriority_t;

#define SDL_MCRC_INTR_PRIORITY_CH1_FAIL                (0x1U)
/**< Offset return for channel 1 fail interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH2_FAIL                (0x2U)
/**< Offset return for channel 1 fail interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH3_FAIL                (0x3U)
/**< Offset return for channel 1 fail interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH4_FAIL                (0x4U)
/**< Offset return for channel 4 fail interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH1_COMPRESSION_DONE    (0x9U)
/**< Offset return for channel 1 compression done interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH2_COMPRESSION_DONE    (0xaU)
/**< Offset return for channel 2 compression done interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH3_COMPRESSION_DONE    (0xbU)
/**< Offset return for channel 3 compression done interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH4_COMPRESSION_DONE    (0xcU)
/**< Offset return for channel 4 compression done interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH1_OVERRUN             (0x11U)
/**< Offset return for channel 1 overrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH2_OVERRUN             (0x12U)
/**< Offset return for channel 2 overrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH3_OVERRUN             (0x13U)
/**< Offset return for channel 3 overrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH4_OVERRUN             (0x14U)
/**< Offset return for channel 4 overrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH1_UNDERRUN            (0x19U)
/**< Offset return for channel 1 underrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH2_UNDERRUN            (0x1aU)
/**< Offset return for channel 2 underrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH3_UNDERRUN            (0x1bU)
/**< Offset return for channel 3 underrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH4_UNDERRUN            (0x1cU)
/**< Offset return for channel 4 underrun interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH1_TIMEOUT             (0x21U)
/**< Offset return for channel 1 timeout interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH2_TIMEOUT             (0x22U)
/**< Offset return for channel 2 timeout interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH3_TIMEOUT             (0x23U)
/**< Offset return for channel 3 timeout interrupt */
#define SDL_MCRC_INTR_PRIORITY_CH4_TIMEOUT             (0x24U)
/**< Offset return for channel 4 timeout interrupt */
/** @} */

/**
 *  \anchor SDL_MCRC_DataBusMask_t
 *  \name Data Bus Mask
 *  @{
 */
/**
 * \brief  MCRC data bus type mask selected for tracing control
 */
typedef uint32_t SDL_MCRC_DataBusMask_t;

#define SDL_MCRC_DATA_BUS_ITCM_MASK     (SDL_MCRC_MCRC_BUS_SEL_ITC_MEN_MASK)
/**< Select tracing control of instruction TCM */
#define SDL_MCRC_DATA_BUS_DTCM_MASK     (SDL_MCRC_MCRC_BUS_SEL_DTC_MEN_MASK)
/**< Select tracing control of data TCM */
#define SDL_MCRC_DATA_BUS_VBUSM_MASK    (SDL_MCRC_MCRC_BUS_SEL_MEN_MASK)
/**< Select tracing control of VBUSM */
#define SDL_MCRC_DATA_BUS_MASK_ALL      (SDL_MCRC_MCRC_BUS_SEL_ITC_MEN_MASK | \
                                    SDL_MCRC_MCRC_BUS_SEL_DTC_MEN_MASK | \
                                    SDL_MCRC_MCRC_BUS_SEL_MEN_MASK)
/**< Select tracing control of all data buses */
/** @} */

/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */

/**
 * \brief   This API is used to get the pending interrupt with highest priority
 *
 * \param   instance        MCRC instance either MCU or Main.
 *
 * \param   pIntVecAddr     Pointer to highest priority pending interrupt vector address
 *                          defined in #SDL_MCRC_IntrPriority_t
 *
 * \return  status          MCRC get interrupt vector address status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */

int32_t SDL_MCRC_getHighestPriorityIntrStatus(SDL_MCRC_InstType instance, uint32_t *pIntVecAddr);

/**
 * \brief   This API is used to control the MCRC data bus tracing.
 *
 *   @n  Data tracing is only available on channel 1, when it is enabled, the
 *       operation mode is automatically reset to data capture mode on channel 1
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   ctrlFlag        Data bus tracing control flag.
 *                          1U: enable data tracing.
 *                          0U: disable data tracing.
 * \param   dataBusMask     Data bus mask bits for which what data buses are to be
 *                          selected. Values given by #SDL_MCRC_DataBusMask_t.
 * \param   busEnableMask   Data bus enable mask bits for which what data buses are to
 *                          be enabled or disabled. Values given by #SDL_MCRC_DataBusMask_t.
 *
 * \return  status          MCRC data bus tracing control status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_MCRC_dataBusTracingCtrl(SDL_MCRC_InstType instance,
                                    uint32_t         ctrlFlag,
                                    SDL_MCRC_DataBusMask_t dataBusMask,
                                    SDL_MCRC_DataBusMask_t busEnableMask);

/**
 * \brief   This API is used to verify the control the MCRC data bus tracing.
 *
 * \param   instance        MCRC instance either MCU or Main.
 * \param   ctrlFlag        Data bus tracing control flag.
 *                          1U:  enable data tracing.
 *                          0U: disable data tracing.
 * \param   dataBusMask     Data bus mask bits for which what data buses are to be
 *                          selected. Values given by #SDL_MCRC_DataBusMask_t.
 * \param   busEnableMask   Data bus enable mask bits for which what data buses are to
 *                          be enabled or disabled. Values given by #SDL_MCRC_DataBusMask_t.
 *
 * \return  status          MCRC verify data bus tracing control status
 *                          SDL_PASS:     success
 *                          SDL_EBADARGS: failure, indicate the bad input arguments
 *                          SDL_EFAIL:    failure, indicate verify failed
 *
 */
int32_t SDL_MCRC_verifyBusTracingCtrl(SDL_MCRC_InstType     instance,
                                      uint32_t         ctrlFlag,
                                      SDL_MCRC_DataBusMask_t dataBusMask,
                                      SDL_MCRC_DataBusMask_t busEnableMask);

#ifdef __cplusplus
}
#endif
#endif /* SDL_IP_MCRC_H_ */

/** @} */
