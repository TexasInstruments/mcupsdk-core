/* Copyright (C) 2022 Texas Instruments Incorporated.
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

/**
 *  \defgroup SDL_DPL_MODULE APIs for SDL DPL
 *  \ingroup SDL_MODULE
 *
 *  This module contains APIs for using configuring the DPL function
 *  pointers to be used by SDL, and also defines the DPL APIs used by
 *  SDL for services such as interrupt registration.
 *
 *  @{
 */

/**
 *   @file  sdl_dpl.h
 *
 *   @brief  This file contains the SDL DPL API's.
 *
 *   Provides the APIs for DPL.
 */

#ifndef SDL_DPL_H_
#define SDL_DPL_H_

#include <stdint.h>
#include <sdl/include/sdl_types.h>

/* Data Structures and Types */

/**
 * \brief   Prototype for the interrupt callback function
 */
typedef void (*pSDL_DPL_InterruptCallbackFunction)(void *arg);

/**
 * \brief   This structure contains the parameters for interrupt registration
 *          through the SDL DPL interface.
 */
typedef struct SDL_DPL_HwipParams_s
{
    int32_t intNum;
    /**< Interrupt Vector Id */
    pSDL_DPL_InterruptCallbackFunction callback;
    /**< Callback function for the interrupt */
    uintptr_t callbackArg;
    /**< Argument passed to the callback function */
} SDL_DPL_HwipParams;

typedef void* pSDL_DPL_HwipHandle;

/**
 * \brief   Prototype for the interrupt enable/disable functions
 */
typedef int32_t (*pSDL_DPL_InterruptFunction)(int32_t intNum);

/**
 * \brief   Prototype for the interrupt registration function
 */
typedef pSDL_DPL_HwipHandle (*pSDL_DPL_RegisterFunction)(SDL_DPL_HwipParams *pParams);

/**
 * \brief   Prototype for the interrupt de-register function
 */
typedef int32_t (*pSDL_DPL_DeregisterFunction)(pSDL_DPL_HwipHandle handle);

/**
 * \brief   Prototype for the delay function
 */
typedef int32_t (*pSDL_DPL_DelayFunction)(int32_t ndelay);

/**
 * \brief   Prototype for address translation function
 */
typedef void* (*pSDL_DPL_AddrTranslateFunction)(uint64_t addr, uint32_t size);

/**
 * @brief   Prototype for the interrupt global disable function
 */
typedef int32_t (*pSDL_DPL_globalDisableInterruptsFunction)(uintptr_t *key);

/**
 * @brief   Prototype for the interrupt global restore function
 */
typedef int32_t (*pSDL_DPL_globalRestoreInterruptsFunction)(uintptr_t key);

/**
 * \brief   This structure contains the pointers for the DPL interfaces
 *          provided by the application to SDL_DPL_init.
 */
typedef struct SDL_DPL_Interface_s
{
    /**< Pointer to interrupt enable function */
    pSDL_DPL_InterruptFunction enableInterrupt;
    /**< Pointer to interrupt disable function */
    pSDL_DPL_InterruptFunction disableInterrupt;
    /**< Pointer to interrupt registration function */
    pSDL_DPL_RegisterFunction registerInterrupt;
    /**< Pointer to inerrupt de-register function */
    pSDL_DPL_DeregisterFunction deregisterInterrupt;
    /**< Pointer to delay function */
    pSDL_DPL_DelayFunction delay;
    /**< Pointer to global interrupt disable function */
    pSDL_DPL_globalDisableInterruptsFunction globalDisableInterrupts;
    /**< Pointer to global interrupt enable function */
    pSDL_DPL_globalRestoreInterruptsFunction globalRestoreInterrupts;
    /**< Pointer to address translation function */
    pSDL_DPL_AddrTranslateFunction addrTranslate;
} SDL_DPL_Interface;

/* Functions */

/**
 *  \brief DPL init
 *
 *  This function initializes the DPL interface structure with the
 *  functions provided by the application. These functions are application
 *  dependent, so it is required to be passed by the user.
 *
 *  \param dplInterface     [IN]  DPL interface structure.
 *
 *  \return The SDL error code for the API.
 *                                 If dplInterface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_DPL_init(SDL_DPL_Interface *dplInterface);

/**
 *  \brief DPL enable interrupt
 *
 *  This function will enable the specific interrupt number passed.
 *
 *  \param intNum           [IN]  Interrupt Number
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_DPL_enableInterrupt(int32_t intNum);

/**
 *  \brief DPL disable interrupt
 *
 *  This function will disable the specific interrupt number passed.
 *
 *  \param intNum           [IN]  Interrupt Number
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_DPL_disableInterrupt(int32_t intNum);

/**
 *  \brief DPL register interrupt
 *
 *  This function will register the specific interrupt number passed.
 *
 *  \param pParams          [IN]  Parameters for interrupt registration
 *  \param handle           [OUT] Handle for this registered interrupt
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_DPL_registerInterrupt(SDL_DPL_HwipParams *pParams, pSDL_DPL_HwipHandle *handle);

/**
 *  \brief DPL deregister interrupt
 *
 *  This function will deregister the specific interrupt number passed.
 *
 *  \param handle           [IN] Handle for the registered interrupt
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_DPL_deregisterInterrupt(pSDL_DPL_HwipHandle handle);

/**
 *  \brief DPL delay
 *
 *  This function is used assign delay in the function
 *
 *  \param ndelay           [IN] delay in microseconds
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */

int32_t SDL_DPL_delay(int32_t ndelay);

/**
 *  \brief DPL Address translation function
 *
 *  This function is used by the SDL to get a translation for a 64-bit address to
 *  local address space. It is expected that the requested adddress will remain available
 *  at the returned address and not be removed.
 *
 *  \param addr             [IN] Memory address to be translated
 *  \param size             [IN] Size of the memory
 *
 *  \return The translated address or (-1) for failure.
 */
void *SDL_DPL_addrTranslate(uint64_t addr, uint32_t size);

/**
 *  \brief DPL globally disable interrupts
 *
 *  This function will disable interrupts globally. Interrupts can be
 *  enabled with the globalRestoreInterrupts() function. Usually used for
 *  critical sections.
 *
 *  The returned key is used to restore the context once interrupts are
 *  restored.
 *
 *  \param key              [OUT] key to use when restoring interrupts
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */

int32_t SDL_DPL_globalDisableInterrupts(uintptr_t *key);

/**
 *  \brief DPL globally enable interrupts
 *
 *  This function will enable interrupts globally. Usually used for
 *  critical sections.
 *
 *  The key is used to restore the context.
 *
 *  \param key              [IN] key to use when restoring interrupts
 *
 *  \return The SDL error code for the API.
 *                                 If function pointer in interface is NULL: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_DPL_globalRestoreInterrupts(uintptr_t key);

/** @} */

#endif /* SDL_DPL_H_ */
