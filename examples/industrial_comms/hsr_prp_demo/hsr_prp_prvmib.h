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

#ifndef HSR_PRP_PRIVMIB_H
#define HSR_PRP_PRIVMIB_H

#include "lwip/apps/snmp.h"
#include "lwip/apps/snmp_mib2.h"
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * @brief Interface Count
 * @param none
 * @return port count
 */
uint16_t numberOfInterfaces();

/**
 * @brief Interface Description
 * @param interface
 * @return pointer to interface name
 */
char *getIfDescr(int interface);

/**
 * @brief Interface Type
 * @param none
 * @return 1
 */
int getIfType();

/**
 * @brief Returns size of largest Datagram
 * @param none
 * @return packet size
 */
u32_t getIfMtu();

/**
 * @brief Interface Speed
 * @param interface
 * @return speed(Mbps)
 */
u32_t getIfSpeed(int interface);

/**
 * @brief Interface Status
 * @param ifAdminStatus
 * @param ifOperStatus
 * @param ifLastChange
 * @param port_no
 * @return none
 */
void getIfStatus(u32_t *ifAdminStatus, u32_t *ifOperStatus, u32_t *ifLastChange, int port_no);

/**
 * @brief Gets pointer to Host Statistics & Pru Statistics
 * @param port_no
 * @return none
 */
void getDriverStats(int port_no);

/**
 * @brief Returns no. of Rx Octets Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfInOctets(int port_no);

/**
 * @brief Returns no. of Rx Unicast Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfInUcastPkts(int port_no);

/**
 * @brief Returns no. of Rx Broadcast & Multicast Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfInNUcastPkts(int port_no);

/**
 * @brief Returns no. of Discarded Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfInDiscards(int port_no);

/**
 * @brief Returns no. of In Error Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfInErrors(int port_no);

/**
 * @brief Returns no. of Unknown Protocol Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfInUnkownProtos(int port_no);

/**
 * @brief Returns no. of Tx Octets Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfOutOctets(int port_no);

/**
 * @brief Returns no. of Tx Unicast Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfOutUcastPkts(int port_no);

/**
 * @brief Returns no. of Tx Broadcast & Multicast Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfOutNUcastPkts(int port_no);

/**
 * @brief Returns no. of Tx Discard Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfOutDiscards(int port_no);

/**
 * @brief Returns no. of Tx Error Packets
 * @param port_no
 * @return packet count
 */
u32_t getIfOutErrors(int port_no);

void lwip_privmib_init(void);
void snmp_example_init(void);

#endif /*HSR_PRP_PRIVMIB_H*/
