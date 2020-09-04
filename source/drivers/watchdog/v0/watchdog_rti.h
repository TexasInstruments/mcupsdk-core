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

/**
 *  \defgroup DRV_WDT_MODULE APIs for WDT
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the WDT.
 *
 *  @{
 */

/**
 *  \file v0/watchdog_rti.h
 *
 *  \brief This file contains the prototype of WDT driver APIs
 */

#ifndef WATCHDOG_RTI_H_
#define WATCHDOG_RTI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdbool.h>
#include <drivers/hw_include/cslr_watchdog.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief  macro to clear the status.
 */
#define WATCHDOG_CLEAR_STATUS                   (0xFFU)

/* ========================================================================== */
/*                            Structures                                      */
/* ========================================================================== */

/**
 * @brief
 *  Watchdog Driver HW configuration
 *
 * @details
 *  The structure is used to store the hardware specific configuration which is
 *  passed to each driver instance
 */
typedef struct Watchdog_HwAttrs_s
{
    /**
     * @brief   Instance of the Watchdog to be used.
     */

     uint8_t            instance;
    /**
     * @brief   Base address of the Watchdog address space to be used.
     */
    uintptr_t           baseAddr; 

    /**
     * @brief   WDT clock frequency.
     */
    uint32_t            wdtClkFrequency;
} Watchdog_HwAttrs;

/**
 *  \brief      Watchdog debug stall settings
 *
 *  This enumeration defines the debug stall modes for the Watchdog. On some
 *  targets, the Watchdog timer will continue to count down while a debugging
 *  session is halted. To avoid unwanted resets, the Watchdog can be set to
 *  stall while the processor is stopped by the debugger.
 */
typedef enum Watchdog_DebugMode_e {
    Watchdog_DEBUG_STALL_ON = 0, /**< Watchdog will be stalled at breakpoints */
    Watchdog_DEBUG_STALL_OFF = 1 /**< Watchdog will keep running at breakpoints */
} Watchdog_DebugMode;

/**
 *  \brief      Watchdog reset mode settings
 *
 *  This enumeration defines the reset modes for the Watchdog. The Watchdog can
 *  be configured to either generate a reset upon timeout or simply produce a
 *  periodic interrupt.
 */
typedef enum Watchdog_ResetMode_e {
    Watchdog_RESET_OFF  = 0xAU, /**< Timeouts generate NMI interrupt only */
    Watchdog_RESET_ON   = 0x5U   /**< Generates reset after timeout */
} Watchdog_ResetMode;

/**
 *  \brief      Watchdog Window Size settings
 *
 *  This enumeration defines the size of the digital watchdog window size.
 */
typedef enum Watchdog_WindowSize_e {
    Watchdog_WINDOW_100_PERCENT     = CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_100_PERCENT, /**< Window size is 100% */
    Watchdog_WINDOW_50_PERCENT      = CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_50_PERCENT, /**< Window size is 50% */
    Watchdog_WINDOW_25_PERCENT      = CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_25_PERCENT, /**< Window size is 25% */
    Watchdog_WINDOW_12_5_PERCENT    = CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_12_5_PERCENT, /**< Window size is 12.5% */
    Watchdog_WINDOW_6_25_PERCENT    = CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_6_25_PERCENT, /**< Window size is 6.25% */
    Watchdog_WINDOW_3_125_PERCENT   = CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_3_125_PERCENT  /**< Window size is 3.125% */
} Watchdog_WindowSize;

/**
*  \brief      Watchdog Handle
*/
typedef struct Watchdog_Config_s *Watchdog_Handle;

/**
 *  \brief      Watchdog callback pointer
 *
 *  This is the typedef for the function pointer that will allow a callback
 *  function to be specified in the Watchdog_Params structure. The function
 *  will take Watchdog_Handle of the Watchdog and user argument as arguments.
 *
 */
typedef void (*Watchdog_Callback)(Watchdog_Handle handle, void *callbackFxnArgs);

/**
 *  \brief      Watchdog Parameters
 *
 *  Watchdog parameters are used with the Watchdog_open() call. Default
 *  values for these parameters are set using Watchdog_Params_init().
 *
 */
typedef struct Watchdog_Params_t {
    Watchdog_Callback       callbackFxn;        /**< Pointer to callback. Valid when resetMode = Watchdog_RESET_OFF. */
    void                    *callbackFxnArgs;   /**< User argument for callback function */
    Watchdog_ResetMode      resetMode;          /**< Mode to enable resets. */
    Watchdog_DebugMode      debugStallMode;     /**< Mode to stall watchdog at breakpoints. */
    Watchdog_WindowSize     windowSize;         /**< Windowed watchdog window size. */
    uint32_t                expirationTime;     /**< Expiration time in millisecond (ms). */
} Watchdog_Params;

/**
 *  \brief      Watchdog Global configuration
 *
 *  The Watchdog_Config structure contains a set of pointers used to
 *  characterize the Watchdog driver implementation.
 *
 *  This structure needs to be defined before calling Watchdog_init() and
 *  it must not be changed thereafter.
 *
 *  \sa     Watchdog_init()
 */
typedef struct Watchdog_Config_s {

    /** Pointer to a driver specific data object */
    void                    *object;

    /** Pointer to a driver specific hardware attributes structure */
    void              const *hwAttrs;

} Watchdog_Config;

/**
 * @brief
 *  Watchdog Driver Status
 *
 * @details
 *  The enumeration describes the status of the Watchdog Driver Instance
 */
typedef enum Watchdog_DriverState_e
{
    /**
     * @brief   Driver is uninitialized.
     */
    Watchdog_DriverState_UNINIT = 0,

    /**
     * @brief   Driver is operational.
     */
    Watchdog_DriverState_OPERATIONAL = 1
}Watchdog_DriverState;

/**
 * @brief
 *  Watchdog Driver Master Control Block
 *
 * @details
 *  The structure is used to hold all the pertinent information with respect
 *  to the Watchdog Driver.
 */
typedef struct Watchdog_MCB_t
{
    /**
     * @brief   Watchdog driver internal state
     */
    Watchdog_DriverState         state;

    /**
     * @brief   Watchdog Parameters which were used to initialize the driver instance
     */
    Watchdog_Params              params;

    /**
     * @brief   Number of interrupts received. Valid only when resetMode = Watchdog_RESET_OFF
     */
    uint32_t                    interruptsRxed;

    /**
     * @brief   Number of times watchdog was serviced
     */
    uint32_t                    watchdogCleared;
} Watchdog_MCB;


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Set DWWD reaction.
 *
 * \param   handle      Watchdog Handle
 *
 * \param   dwwdReaction   DWWD reaction for violation/expiration.
 *                         Values given by macro
 *
 *
 * \note
 * 1. DWWD need to be serviced if Reaction is changed when DWWD is
 *    enabled and Window is Open,to take immediate effect. If not
 *    serviced, DWWD will generated previously configured Reaction
 *    irrespective of current configuration.
 * 2. If DWWD is enabled and Window is Close then any change in
 *    Reaction will take immediate effect. DWWD need not to be serviced
 *    in this case.
 */
void Watchdog_setReaction(Watchdog_Handle handle, uint32_t dwwdReaction);

/**
 * \brief   Get DWWD Window Size.
 *
 * \param   handle      Watchdog Handle
 *
 * \return  Watchdog Window size.
 *
 */

uint32_t Watchdog_getWindowSize(Watchdog_Handle handle);

/**
 * \brief   Set DWWD Window Size.
 *
 * \param   handle      Watchdog Handle
 *
 * \param   dwwdWindowSize  DWWD Window Size.
 *                          Values given by macro
 *
 *
 * \note
 * 1. DWWD need to be serviced if Window Size is changed when DWWD is
 *    enabled and Window is Open,to take immediate effect. If not
 *    serviced, DWWD Window Size will not be changed
 *    irrespective of current configuration.
 * 2. If DWWD is enabled and Window is Close then any change in
 *    Window Size will take immediate effect. DWWD need not to be serviced
 *    in this case.
 */
void Watchdog_setWindowSize(Watchdog_Handle handle, uint32_t dwwdWindowSize);

/**
 *  \brief The function checks for Closed Window.
 *
 *  \param  handle          Watchdog Handle
 *
 *  \return closed window status.
 */

bool Watchdog_isClosedWindow(Watchdog_Handle handle);

/**
 *  \brief The function clears the Watchdog to prevent a reset signal
 *      from being generated if the module is in Watchdog_RESET_ON reset mode.
 *
 *  \param  handle      Watchdog Handle
 *
 */
void Watchdog_clear(Watchdog_Handle handle);


/**
 *  \brief The function closes a Watchdog peripheral specified by the Watchdog
 *          handle. It stops (holds) the Watchdog counting on applicable
 *          platforms.
 *
 *  \param  handle      Watchdog Handle
 *
 */
void Watchdog_close(Watchdog_Handle handle);

/**
 *  \brief The functions initializes the Watchdog module.
 *
 *
 */
void Watchdog_init(void);

/**
 *  \brief The functions de-initializes the Watchdog module.
 *
 *
 */
void Watchdog_deinit(void);

/**
 *  \brief  Opens a Watchdog object with the index and parameters specified, and
 *          returns a Watchdog_Handle.
 *
 *  \param  index         Logical peripheral number for the Watchdog indexed
 *                        into the Watchdog_Config table
 *
 *  \param  params        Pointer to an parameter block, if NULL it will use
 *                        default values.
 *
 *  \return A Watchdog_Handle on success or a NULL on an error or if it has been
 *          opened already.
 */
Watchdog_Handle Watchdog_open(uint8_t index, Watchdog_Params *params);

/**
 *  \brief Function to set default values of Watchdog_Params in params
 *
 *  \param params   [IN] pointer to the structure to be initialized
 */
void Watchdog_paramsInit(Watchdog_Params *params);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_RTI_H_ */

/** @} */

