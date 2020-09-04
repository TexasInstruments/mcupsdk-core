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


#ifndef HSR_PRP_MENU_H_
#define HSR_PRP_MENU_H_
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdbool.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <stdint.h>

#include <networking/icss_emac/icss_emac.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_common.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_handle.h>
#ifdef ICSS_PROTOCOL_RED
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_nodeTable.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_snmp.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_statistics.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_config.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red.h>
#endif /* ICSS_PROTOCOL_RED */
#include <networking/icss_timesync/icss_timeSync.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*2 frames per second. Test runs for 2 minutes*/
#define NUM_HSR_TEST_FRAMES          240
#define HSR_PRP_TEST_FRAME_SIZE      100

/*PTP min/max offset values*/
#define PTP_MIN_OFFSET_INIT_VAL     999999
#define PTP_MAX_OFFSET_INIT_VAL     -100000

/**Starting offset for an IP header in TCP/IP packet*/
#define START_OF_IP_HEADER (14)
/**Starting offset for checksum in IP header in TCP/IP packet*/
#define START_OF_IP_CHECKSUM (START_OF_IP_HEADER + 10)
/**Starting offset for length field in IP header in TCP/IP packet*/
#define START_OF_IP_LENGTH (START_OF_IP_HEADER + 2)
/**Starting offset for UDP header in TCP/IP packet*/
#define START_OF_UDP_HEADER (34)
/**Starting offset for checksum field in UDP header in TCP/IP packet*/
#define START_OF_UDP_CHECKSUM (START_OF_UDP_HEADER + 6)
/**Starting offset for length field in UDP header in TCP/IP packet*/
#define START_OF_UDP_LENGTH  (START_OF_UDP_HEADER + 4)
/**Starting offset for protocol field in IP header in TCP/IP packet*/
#define START_OF_IP_PROTOCOL (START_OF_IP_HEADER + 9)
/**Starting offset for IP address field in IP header in TCP/IP packet*/
#define START_OF_IP_ADDRESS (START_OF_IP_HEADER + 12)
/**Starting offset for payload data in UDP header in TCP/IP packet*/
#define START_OF_PAYLOAD (START_OF_UDP_HEADER + DEFAULT_UDP_HEADER_SIZE)
/**standard size for Ethernet Header (src MAC + dst MAC + EthType) in 802.3 packet*/
#define DEFAULT_ETH_HEADER_SIZE 14
/**standard size for IP header in TCP/IP packet*/
#define DEFAULT_IP_HEADER_SIZE 20
/**standard size for UDP header in TCP/IP packet*/
#define DEFAULT_UDP_HEADER_SIZE 8
/**standard size for UDP + IP header in TCP/IP packet*/
#define DEFAULT_HEADER_SIZE  42
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/**
 * @brief Print the helper menu for HSR/PRP
 * @param none
 * @return none
 */
void printHelpMenu();
/**
 * @brief Print the header on top of menu
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void printConfig(ICSS_EMAC_Handle emachandle);

/**
 * @brief Print PTP Sub Menu
 * @param none
 * @return none
 */
void printPTPSubMenu(void);

/**
 * @brief Print Multicast Sub Menu
 * @param none
 * @return none
 */
void printMulticastFilterSubMenu(void);

/**
 * @brief Print VLAN Sub Menu
 * @param none
 * @return none
 */
void printVlanFilterSubMenu(void);

/**
* @brief Print important PTP variables which can be used for tracking
* @param ptpHandle Pointer to PTP Handle
* @return none
*/
void printPTPStatus(TimeSync_ParamsHandle_t ptpHandle);
/**
 * @brief Print HSR/PRP Stats
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void printStatistics(ICSS_EMAC_Handle emachandle);
/**
 * @brief Print HSR/PRP LRE Stats
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void printLreStatistics(ICSS_EMAC_Handle emachandle);
/**
 * @brief Print HSR/PRP configuration
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void printConfiguration(ICSS_EMAC_Handle emachandle);
/**
 * @brief Returns the node type (DANH/DANP) when given the node entry no
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
char *getNodeTypePrintStr(uint8_t nodeEntryStatus);
/**
 * @brief Print Node Table (SNMP)
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void printSnmpNodeTable(hsrPrpHandle *hsrPrphandle);
/**
 * @brief Show Node Table entries
 * @param hsrPrphandle
 * @return none
 */
void printNodeTable(hsrPrpHandle *hsrPrphandle);
/**
 * @brief Run the basic visual redundancy test. Configure device as Rx/Tx node
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void runRxTxTest(ICSS_EMAC_Handle emachandle);
/**
 * @brief Transmit HSR PRP test frames
 * @param emachandle Pointer to ICSS EMAC Handle
 * @return none
 */
void txHSRPRPTestFrame(ICSS_EMAC_Handle emachandle);
/**
 * @brief Check for HSR/PRP test frame marker and toggle Digital IO based on payload
 * @param pointer to frame
 * @return none
 */
void parseAndCheckHSRPRPTestFrame(uint8_t *tempFrame);
/**
*  @brief The function is used to remove Assigned IP adddress.
*  @param none
*  @return none
*
*/
void removeIPAddress();
/**
*  @brief The function checks for valid IP address, removes existing one
*  and adds a new one.
*  @param none
*  @return none
*
*/
uint8_t addIPAddress(uint32_t dwIPAddress);
/**
 *  @brief  The function is used to validate an IP adddress.
 *
 *  @param   ip_str [in] IP address pointer
 *
 *  @retval TRUE  - If valid IP address
 *          FALSE - If invalid IP address
 *
 */
int isValidIP(const char *ip_str);
/**
 *  @brief  The function is used to validate an MAC adddress.
 *
 *  @param   macid_str [in] MAC address pointer
 *
 *  @retval TRUE  - If valid IP address
 *          FALSE - If invalid IP address
 *
 */
int isValidMacid(char *macid_str);
/**
 *  @brief  The function changes the endianness of the IP Address
 *
 *  @param   dwIPAddress [in] IP address pointer
 *
 *  @retval  none
 *
 */
void changeIPEndianness(uint32_t *dwIPAddress);
/**
 *  @brief  The function checks the input for a valid IP Address and writes it to QSPI
 *
 *  @param   none
 *
 *  @retval  none
 *
 */
uint8_t readAndAssignIPAddress();

/**
 *  @brief  The function fetches the Duplicate Discard value
 *
 *  @param   nodeEntryStatus status field in the nodesTable
 *
 *  @retval  Duplicate Discard value
 *
 */
uint8_t getDuplicateDiscardPrintVal(uint8_t nodeEntryStatus);

#ifdef PTP_TESTING
void hsrprp_monitorPTPStatusTask(void *args);
#endif

/**
 *  @brief  The function to configure interrupt pacging implementation in firmware
 *
 *  @param   emachandle Pointer to ICSS EMAC Handle
 *  @param   intrPacStatus user input choice
 *
 *  @retval  none
 *
 */
void intr_pac_config(ICSS_EMAC_Handle icssEmacHandle, uint32_t intrPacStatus);

/**
 *  @brief  The function to configure Storm prevention related settings
 *
 *  @param   none
 *
 *  @retval  none
 *
 */
void stormPreventionSettingsMenu(void);
/**
 *  @brief  The function to enable or disable storm prevention
 *
 *  @param   none
 *
 *  @retval  none
 *
 */
void enableDisablestormPrevention(void);

/**
 *  @brief  The function to enable or disable storm prevention
 *
 *  @param   type: UC/BC/MC packet type is it aplicable for
 *  @param status: enable/disable
 *
 *  @retval  none
 *
 */
void stormPrevention(ICSS_EMAC_Handle icssEmacHandle, uint8_t type, uint8_t status);

/**
* @brief Copy MAC ID to a stream
* @param src - pointer to stream
* @param macID pointer to MAC ID
*/
void addMACID(uint8_t *src, uint8_t *macID);

#endif /* HSR_PRP_MENU_H_ */
