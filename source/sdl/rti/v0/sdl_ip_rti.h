/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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

#ifndef SDL_IP_RTI_H_
#define SDL_IP_RTI_H_

#include <sdl/include/sdl_types.h>
#include <sdl/rti/v0/soc/sdl_rti_soc.h>
#include <sdl/include/hw_types.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *
 * \ingroup  SDL_IP_MODULE
 * \defgroup SDL_RTI_API APIs for SDL RTI
 *
 *    The Digital Watchdog Timer(DWT) generates reset after a programmable
 *    period, if not serviced within that period. In DWT, time-out
 *    boundary is configurable.
 *    In DWWD, along with configurable time-out boundary, the start time
 *    boundary is also configurable. The DWWD can generate Reset or
 *    Interrupt, if not serviced within window(Open Window) defined by
 *    start time and time-out boundary. Also the DWWD can generate Reset or
 *    Interrupt if serviced outside Open Window (within Closed Window).
 *    Generation of Reset or Interrupt depends on the DWWD Reaction
 *    configuration.
 *
 */
 /**
 * \defgroup SDL_IP_RTI_DATASTRUCT  RTI Data Structures
 * \ingroup SDL_RTI_API
 * @{
 */

/**
 * \brief  List of Static Registers for RTI DWWD
 */
typedef struct
{
    /** RTI DWD Control register */
    uint32_t    RTI_DWDCTRL;
    /** RTI DWD Preload register */
    uint32_t    RTI_DWDPRLD;
    /** RTI DWD Reaction Control register */
    uint32_t    RTI_WWDRXNCTRL;
    /** RTI DWD Window Size Control */
    uint32_t    RTI_WWDSIZECTRL;
} SDL_RTI_staticRegs;


/**
 * \brief  List of Config Parameters for RTI DWWD
 */

typedef struct{
    /* Parameter to set preload value  */
    uint32_t         SDL_RTI_dwwdPreloadVal;
    /* Parameter to set window size  */
    uint32_t        SDL_RTI_dwwdWindowSize;
    /* Parameter to set Reaction  */
    uint32_t        SDL_RTI_dwwdReaction;
}SDL_RTI_configParms;

/** @} */
/********************************************************************************************************
*   Below are the Declarations of Low Level Functions
********************************************************************************************************/
/**
* \defgroup SDL_IP_RTI_FUNCTION  RTI IP Functions
* \ingroup SDL_RTI_API
* @{
*/

/**
* \brief This API will check the Window Size for DWWD.
*
* \param dwwdWindowSize variable which holds window size
* which should be verified
*
* \return status Success of the window configuration
* - Success: SDL_PASS
* - Fail : SDL_EFAIL
*
*/
int32_t SDL_RTI_chkWindowSize(uint32_t dwwdWindowSize);

/**
* \brief This API will check the reaction to perform when error is detected from DWWD.
*
* \param dwwdReaction Variable which holds the reaction to be verified.
*
* \return status Success of the window configuration
* - Success: SDL_PASS
* - Fail : SDL_EFAIL
*
*/
int32_t SDL_RTI_chkReaction(uint32_t dwwdReaction);

/**
 * \brief   This API will return current configured Window Size.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 *
 * \param   pWinSize        pointer to Configured Window Size.
 *                          Refer macro #RTI_WindowSize_t
 *
 * \return  status          Success of the window configuration
 *                                - Success: SDL_PASS
 *                                - Fail   : SDL_EFAIL
 *
 */
int32_t  SDL_RTI_getWindowSize(uint32_t baseAddr, uint32_t *pWinSize);

/**
 * \brief   Set DWWD preload value.
 *          From this value down counter starts down counting.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 *
 * \param   dwwdPreloadVal  Down counter preload value.
 *                          Refer DWWD Down Counter Overview.
 *
 * \return  status          Success of the window configuration
 *                                - Success: SDL_PASS
 *                                - Fail   : SDL_EFAIL
 */
int32_t  SDL_RTI_setPreload(uint32_t baseAddr, uint32_t dwwdPreloadVal);

/**
 * \brief   This API will return current configured Preload value.
 *
 * \param   baseAddr        Base Address of the RTI instance.
 *
 * \param   pPreloadVal     pointer to current preload value
 *
 * \return  status          Success of the window configuration
 *                                - Success: SDL_PASS
 *                                - Fail   : SDL_EFAIL
 *
 */
int32_t  SDL_RTI_getPreload(uint32_t baseAddr, uint32_t *pPreloadVal);

/** @} */

/**
* \anchor SDL_RTIWinSizeValues
* \name   RTI DWD possible Window Size Values
*/

#define    RTI_DWWD_WINDOWSIZE_100_PERCENT      (RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT)
    /**< Configure DWWD window size to 100% */
#define    RTI_DWWD_WINDOWSIZE_50_PERCENT       (RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT)
    /**< Configure DWWD window size to 50% */
#define    RTI_DWWD_WINDOWSIZE_25_PERCENT       (RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT)
    /**< Configure DWWD window size to 25% */
#define    RTI_DWWD_WINDOWSIZE_12_5_PERCENT     (RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT)
    /**< Configure DWWD window size to 12.5% */
#define    RTI_DWWD_WINDOWSIZE_6_25_PERCENT     (RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT)
    /**< Configure DWWD window size to 6.25% */
#define    RTI_DWWD_WINDOWSIZE_3_125_PERCENT    (RTI_RTIDWWDSIZECTRL_DWWDSIZE_3_125_PERCENT)
    /**< Configure DWWD window size to 3.125% */

/**
 * \brief  type to select the DWWD window size.
 *
 *         Window Size : In what percentage of time-out value i.e open window,
 *         servicing DWWD is allowed.
 *         Configurable Window Sizes : 100%, 50%, 25%, 12.5%, 6.25%, 3.125%.
 *         Refer to for DWWD Window Sizes Overview.
 */
typedef uint32_t RTI_WindowSize_t;

/**
* \anchor SDL_RTIReactionValues
* \name   RTI DWD possible Reaction Values
*/

/**
 * \brief  macro to select the DWWD reaction after violation or
 *         expiration of DWWD timer.
 *
 *         DWWD can either generate reset or interrupt.
 */
#define    RTI_DWWD_REACTION_GENERATE_RESET  (RTI_RTIDWWDRXNCTRL_DWWDRXN_RESET)
    /**< Configure DWWD reaction to generate reset */
#define    RTI_DWWD_REACTION_GENERATE_NMI    (RTI_RTIDWWDRXNCTRL_DWWDRXN_INTERRUPT)
    /**< Configure DWWD reaction to generate
     * interrupt. The actual interrupt
     * used depends on the SOC and cross bar
     * mapping. */
#define      RTI_DWWD_REACTION_INVALID         (0x46U)

/**
* \anchor SDL_RTIDWWDStatusValues
* \name   SDL DWD possible Status Values
*/

#define    RTI_DWWD_STATUS_KEY_SEQ_VIOLATION          (RTI_RTIWDSTATUS_KEYST_MASK)
    /**< Key sequence violation mask */
#define    RTI_DWWD_STATUS_TIME_WINDOW_VIOLATION      (RTI_RTIWDSTATUS_DWWD_ST_MASK)
    /**< Window violation mask */
#define    RTI_DWWD_STATUS_ENDTIME_WINDOW_VIOLATION   (RTI_RTIWDSTATUS_END_TIME_VIOL_MASK)
    /**< End time window violation mask */
#define    RTI_DWWD_STATUS_STARTTIME_WINDOW_VIOLATION (RTI_RTIWDSTATUS_START_TIME_VIOL_MASK)
    /**< Start time window violation mask */
#define    RTI_DWWD_STATUS_LAST_RESET                 (RTI_RTIWDSTATUS_DWDST_MASK)
    /**< DWWD last reset status mask */

/**
 * \brief  type to report the DWWD status.
 *
 *         Violations(Following bit-field will get set) :
 *         DWWD Status : If last reset is generated by DWWD.
 *         Key sequence violation : If wrong sequence is written to enable DWWD.
 *         Start Time Violation : If DWWD is serviced within closed window.
 *         End Time Violation : If DWWD is not serviced.
 *         Time Window Violation : If any of Start/End Time Violation happened.
 */
typedef uint32_t RTI_Status_t;

/****************************************************************************************************
*   Register Definitions
****************************************************************************************************/


#define RTI_RTIDWDCTRL                                                      (0x90U)
#define RTI_RTIDWDPRLD                                                      (0x94U)
#define RTI_RTIWDSTATUS                                                     (0x98U)
#define RTI_RTIWDKEY                                                        (0x9cU)

#define RTI_RTIDWWDRXNCTRL                                                  (0xa4U)
#define RTI_RTIDWWDSIZECTRL                                                 (0xa8U)
#define RTI_RTIDWDCNTR                                                      (0xa0U)
#define RTI_COMP0                                                           (0x50U)

#define STATUS_VLD                                                          (1U)

/****************************************************************************************************
*   Field Definition Macros
****************************************************************************************************/



#define RTI_RTIDWDCTRL_DWDCTRL_ENABLE                                       (0xA98559DAU)

#define RTI_DWWDPRLD_MULTIPLIER_SHIFT                                       (13U)
#define RTI_DWD_MIN_PRELOAD_VAL                                             (0x1FFFU)

#define RTI_RTIDWDPRLD_INVALID                                              (0xFFFFFFU)
#define RTI_RTIDWDPRLD_DWDPRLD_SHIFT                                        (0U)
#define RTI_RTIDWDPRLD_DWDPRLD_MASK                                         (0x00000fffU)
#define RTI_RTIDWDPRLD_DWDPRLD_MAX                                          (0x0FFFU)

#define RTI_RTIDWDCNTR_DWDCNTR_15_0_SHIFT                                   (0U)
#define RTI_RTIDWDCNTR_DWDCNTR_15_0_MASK                                    (0x0000ffffU)

#define RTI_RTIDWDCNTR_DWDCNTR_24_16_SHIFT                                  (16U)
#define RTI_RTIDWDCNTR_DWDCNTR_24_16_MASK                                   (0x01ff0000U)

#define RTI_RTIWDSTATUS_DWDST_SHIFT                                         (1U)
#define RTI_RTIWDSTATUS_DWDST_MASK                                          (0x00000002U)

#define RTI_RTIWDSTATUS_DWWD_ST_SHIFT                                       (5U)
#define RTI_RTIWDSTATUS_DWWD_ST_MASK                                        (0x00000020U)

#define RTI_RTIWDKEY_WDKEY_SHIFT                                            (0U)
#define RTI_RTIWDKEY_WDKEY_MASK                                             (0x0000ffffU)
#define RTI_RTIWDKEY_WDKEY_FIRST_WRITE                                      (0x0000E51AU)
#define RTI_RTIWDKEY_WDKEY_SECOND_WRITE                                     (0x0000A35CU)

#define RTI_RTIDWWDRXNCTRL_DWWDRXN_SHIFT                                    (0U)
#define RTI_RTIDWWDRXNCTRL_DWWDRXN_MASK                                     (0x0000000fU)
#define RTI_RTIDWWDRXNCTRL_DWWDRXN_RESET                                    (0x5U)
#define RTI_RTIDWWDRXNCTRL_DWWDRXN_INTERRUPT                                (0xAU)

#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_INVALID                                (0U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_SHIFT                                  (0U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_MASK                                   (0x00ffffffU)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT                            (0x00000005U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT_SHIFT                      (0x0)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT                             (0x00000050U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT_SHIFT                       (0x1)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT                             (0x00000500U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT_SHIFT                       (0x2)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT                           (0x00005000U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT_SHIFT                     (0x3)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT                           (0x00050000U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT_SHIFT                     (0x4)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_3_125_PERCENT                          (0x00500000U)
#define RTI_RTIDWWDSIZECTRL_DWWDSIZE_3_125_PERCENT_SHIFT                    (0x5)

/**
 *  Design: DID_TAG(PROC_SDL-1487)
 */

static inline int32_t SDL_RTI_writeWinSz(uint32_t baseAddr, uint32_t dwwdWindowSize)
{
    int32_t sdlResult;
    /* Writing window size to RTI_WWDSIZECTRL Register (Offset = A8h) */
    HW_WR_FIELD32( baseAddr + RTI_RTIDWWDSIZECTRL,
                    RTI_RTIDWWDSIZECTRL_DWWDSIZE,
                    dwwdWindowSize);
    sdlResult = SDL_PASS;

    return sdlResult;
}

/**
 *  Design: DID_TAG(PROC_SDL-1492)
 */

static inline int32_t SDL_RTI_writeReaction(uint32_t baseAddr, uint32_t dwwdReaction)
{
    int32_t sdlResult;

    /* Writing reaction to RTI_WWDRXNCTRL Register (Offset = A4h) */
    /* 5h = This is the default value
       Ah = The windowed watchdog will generate a non-maskable interrupt */
     (HW_WR_FIELD32(baseAddr + RTI_RTIDWWDRXNCTRL,
                    RTI_RTIDWWDRXNCTRL_DWWDRXN,
                    dwwdReaction));
    sdlResult = SDL_PASS;

    return sdlResult;
}

/**
 *  Design: DID_TAG(PROC_SDL-1493)
 */

static inline uint32_t SDL_RTI_readReaction(uint32_t baseAddr)
{
    uint32_t dwwdReaction;

    /* Get Windowed Watchdog Reaction */
    dwwdReaction = HW_RD_FIELD32(baseAddr + RTI_RTIDWWDRXNCTRL,
                                 RTI_RTIDWWDRXNCTRL_DWWDRXN);

    return dwwdReaction;
}


#ifdef __cplusplus
}
#endif
#endif  /* HW_RTI_H_ */
