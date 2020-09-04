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

#ifndef ICSS_EMAC_STATISTICS_H_
#define ICSS_EMAC_STATISTICS_H_

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

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
* @brief Stats structure for statistics on Host (ARM)
*
*/
typedef struct ICSS_EMAC_HostStatistics_s
{
    volatile uint32_t txUcast;                  /**Number of unicast packets sent*/
    volatile uint32_t txBcast;                  /**Number of broadcast packets sent*/
    volatile uint32_t txMcast;                  /**Number of multicast packets sent*/
    volatile uint32_t txOctets;                 /**Number of bytes sent*/
    volatile uint32_t rxUcast;                  /**Number of unicast packets rcvd*/
    volatile uint32_t rxBcast;                  /**Number of broadcast packets rcvd*/
    volatile uint32_t rxMcast;                  /**Number of multicast packets rcvd*/
    volatile uint32_t rxOctets;                 /**Number of Rx packets*/
    volatile uint32_t rxUnknownProtocol;        /**Number of packets with unknown protocol*/
    volatile uint32_t txDroppedPackets;         /**Number of packets with unknown protocol*/
    /*
    * Debug variables, these are not part of standard MIB. Useful for debugging
    */
    volatile uint32_t linkBreak;                /**Number of link breaks*/
    volatile uint32_t txCollisionDroppedPackets;/**Number of packets dropped as a result of collision not resolved*/
    volatile uint32_t txNumCollision;           /**Number of collisions occured*/
} ICSS_EMAC_HostStatistics;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
* @brief   Read statistics from PRU DRAM Memory into structure
* @param portNum Port Number. 1/2
* @param icssEmacHandle EMAC Handle. Provides PRUSS memory map
*
* @retval none
*/
void ICSS_EMAC_readStats(ICSS_EMAC_Handle           icssEmacHandle,
                         uint8_t                    portNum,
                         ICSS_EMAC_PruStatistics    *pruStatStructPtr);

/**
* @brief   clear switch statistics
* @param portNum Port Number. 1/2
* @param icssEmacHandle EMAC Handle. Provides PRUSS memory map
*
* @retval none
*/
void ICSS_EMAC_purgeStats(ICSS_EMAC_Handle icssEmacHandle, uint8_t portNum);

/**
* @brief   Do the stats initialization, call at the beginning of the frame
* @param portNum Port Number. 1/2
* @param icssEmacHandle EMAC Handle. Provides PRUSS memory map
*
* @retval none
*/
void ICSS_EMAC_initStats(ICSS_EMAC_Handle icssEmacHandle, uint8_t portNum);

/**
* @brief   Call every time a packet is received, updates Rx related statistics
*
* @param macAddr pointer to mac address, stored in 8 bit locations
* @param packet_len packet length in bytes
* @param protIdent Protocol Type
* @param hostStatsPtr Pointer to Host statistics for the relevant port
*
* @retval none
*/
void ICSS_EMAC_updateRxStats(const uint8_t              *macAddr,
                             uint32_t                   packet_len,
                             uint16_t                   protIdent,
                             ICSS_EMAC_HostStatistics   *hostStatsPtr);
/**
* @brief   Call every time a packet is transmitted, updates Tx related statistics
*
* @param macAddr pointer to mac address, stored in 8 bit locations
* @param packet_len packet length in bytes
* @param hostStatsPtr Pointer to Host statistics for the relevant port
*
* @retval none
*/
void ICSS_EMAC_updateTxStats(const uint8_t              *macAddr,
                             uint32_t                   packet_len,
                             ICSS_EMAC_HostStatistics   *hostStatsPtr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ICSS_EMAC_STATISTICS_H_ */
