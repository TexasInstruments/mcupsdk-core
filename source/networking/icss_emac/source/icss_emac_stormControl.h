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

#ifndef ICSS_EMAC_STORM_CONTROL_H_
#define ICSS_EMAC_STORM_CONTROL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
* @def ICSS_EMAC_DEFAULT_CREDITS
*      (packets per microsec) This is 1% of the bandwidth
*/
#define ICSS_EMAC_DEFAULT_CREDITS   (2000U)
/**
* @def ICSS_EMAC_DEFAULT_PKT_SIZE
*      default packet size used to compute number of BC/MC packets allowed in a 100ms window
*/
#define ICSS_EMAC_DEFAULT_PKT_SIZE  60
/**
* @def ICSS_EMAC_MAX_PERCENTAGE
*      Max number of BC/MC packets blocked expressed as a percentage
*/
#define ICSS_EMAC_MAX_PERCENTAGE    50
/**
* @def ICSS_EMAC_BC_STORM_PREVENTION
*      flag indicating the settings should be done broadcast traffic
*/
#define ICSS_EMAC_BC_STORM_PREVENTION  0
/**
* @def ICSS_EMAC_MC_STORM_PREVENTION
*      flag indicating the settings should be done multicast traffic
*/
#define ICSS_EMAC_MC_STORM_PREVENTION  1
/**
* @def ICSS_EMAC_UC_STORM_PREVENTION
*      flag indicating the settings should be done unicast traffic
*/
#define ICSS_EMAC_UC_STORM_PREVENTION  2

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * @brief Storm prevention control variables
 */
typedef struct ICSS_EMAC_StormPrevention_s
{
    /** enable/disable storm prevention*/
    uint16_t suppressionEnabledBC;
    uint16_t suppressionEnabledMC;
    uint16_t suppressionEnabledUC;
    /** Number of packets allowed in a time interval*/
    uint16_t creditsBC;
    uint16_t creditsMC;
    uint16_t creditsUC;
} ICSS_EMAC_StormPrevention;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
* @brief Initialize Storm Prevention
* @param portnum Port Number
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @param spType storm prevention type, weather BC/MC/UC
* @retval none
*/
void ICSS_EMAC_initStormPreventionTable(uint8_t             portnum,
                                        ICSS_EMAC_Handle    icssEmacHandle,
                                        uint8_t             spType);
/**
* @brief Disable Storm Prevention for a particular port
* @param portnum  Port number for which Storm Prevention must be disabled
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @param spType storm prevention type, weather BC/MC/UC
* @retval none
*/
void ICSS_EMAC_disableStormPrevention(uint8_t           portnum,
                                      ICSS_EMAC_Handle  icssEmacHandle,
                                      uint8_t           spType);

/**
* @brief   Set the credit value for broadcast packets used in storm prevention
*
* @param creditValue number of packets allowed in a given time
* @param stormPrevPtr Pointer to Storm Prevention member instance for that port
* @param spType storm prevention type, weather BC/MC/UC
*
* @retval none
*/
void ICSS_EMAC_setCreditValue(uint16_t                  creditValue,
                              ICSS_EMAC_StormPrevention *stormPrevPtr,
                              uint8_t                   spType);

/**
* @brief Enable Storm Prevention for a particular port
* @param portnum  Port number for which Storm Prevention must be disabled
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @param spType storm prevention type, weather BC/MC/UC
* @retval none
*/
void ICSS_EMAC_enableStormPrevention(uint8_t            portnum,
                                     ICSS_EMAC_Handle   icssEmacHandle,
                                     uint8_t            spType);

/**
* @brief   Reset the credits at the end of time period, this time period is common to both broadcast and multicast packets
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @param spType storm prevention type, weather BC/MC/UC
* @retval none
*/
void ICSS_EMAC_resetStormPreventionCounter(ICSS_EMAC_Handle icssEmacHandle,
                                           uint8_t          spType);

/**
* @brief   Sets the stormPreventionOffset, suppressionEnabled and creditsPtr depending on storm prevention type
*
* @param stormPreventionOffsetPtr pointer to storm prevention offset
* @param suppressionEnabledPtr pointer to stoprt prevention enbled/disabled info
* @param creditsPtr pointer to credits for storm prevention
* @param spType storm prevention type, weather BC/MC/UC
* @param icssEmacHandle emac handle
* @param stormPrevPtr storm prevention pointer
*
* @retval none
*/
void ICSS_EMAC_checkStormPreventionType(uint32_t                    **stormPreventionOffsetPtr,
                                        uint16_t                    **suppressionEnabledPtr,
                                        uint16_t                    **creditsPtr,
                                        uint8_t                     spType,
                                        ICSS_EMAC_Handle            icssEmacHandle,
                                        ICSS_EMAC_StormPrevention   *stormPrevPtr);

/**
* @brief   byte by byte memcpy
*
* @param dst_ptr pointer of destination
* @param src_ptr pointer of destination
*
* @retval none
*/
void ICSS_EMAC_byteCopy(uint8_t *dst_ptr, uint8_t *src_ptr, uint32_t size_bytes);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ICSS_EMAC_STORM_CONTROL_H_ */
