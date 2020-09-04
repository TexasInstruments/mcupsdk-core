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

#ifndef _TIESC_EOE_FOE_H_
#define _TIESC_EOE_FOE_H_

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if EOE_SUPPORTED
#define    TIESC_ARP_HW_ADDR_SPACE_ETHERNET_SW  0x0100 /**< Hardware Address Space: 1 = Ethernet*/
#define    TIESC_ETHERNET_FRAME_TYPE_IP_SW      0x0008 /**< EtherType IP*/
#define    TIESC_ETHERNET_FRAME_TYPE_ARP1_SW    0x0608 /**< EtherType ARP*/

#define    TIESC_ARP_IP_HEADER_LEN                28 /**< ARP/IP Header length*/

#define    TIESC_ARP_OPCODE_REQUEST_SW            0x0100 /**< ARP Request opcode*/
#define    TIESC_ARP_OPCODE_REPLY_SW              0x0200 /**< ARP Reply opcode*/
#define    TIESC_ETHERNET_MAX_FRAME_LEN         1514 /**< Max Ethernet frame length*/


#define    TIESC_IP_HEADER_MINIMUM_LEN    20/**< Minimum IP header length*/
#define    TIESC_IP_PROTOCOL_ICMP         1 /**< Protocol ICMP*/

#define    TIESC_ICMP_TYPE_ECHO                   8 /**< Echo*/
#define    TIESC_ICMP_TYPE_ECHO_REPLY             0 /**< Echo Reply*/

#if FOE_SUPPORTED

#define    TIESC_ECAT_FOE_ERRCODE_NOTFOUND            0x8001 /**< The file requested by an FoE upload service could not be found on the server*/
#define    TIESC_ECAT_FOE_ERRCODE_BOOTSTRAPONLY       0x8008 /**< FoE only supported in Bootstrap*/
#define    TIESC_ECAT_FOE_ERRCODE_NOTINBOOTSTRAP      0x8009 /**< This file may not be accessed in BOOTSTRAP state*/

#endif /* FOE_SUPPORTED */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** @brief mac address structure for EoE*/
typedef struct
{
    uint8_t b[6];
} __attribute__((packed))
TIESC_ETHERNET_ADDRESS;

/**
* @brief Ethernet header
*/
typedef struct
{
    TIESC_ETHERNET_ADDRESS    destination;  /**< Destination MAC address*/
    TIESC_ETHERNET_ADDRESS    source;       /**< Source MAC address*/
    uint16_t              frame_type;          /**< EtherType */
} __attribute__((packed))
TIESC_ETHERNET_FRAME;

#define    TIESC_ETHERNET_ADDRESS_LEN      sizeof(TIESC_ETHERNET_ADDRESS) /**< MAC address length*/
#define    TIESC_ETHERNET_FRAME_LEN        sizeof(TIESC_ETHERNET_FRAME) /**< Ethernet header size*/

/**
* @brief ARP/IP frame Header
*/
typedef struct
{
    uint16_t
    hw_addr_space;  /**< Hardware Address Space: 1 = Ethernet*/
    uint16_t                  prot_addr_space;  /**< ETHERNET_FRAME_TYPE_IP*/
    uint8_t                   length_hw_addr;  /**< Length of Hardware address (6)*/
    uint8_t                   length_prot_addr;  /**< Length of Port address (4)*/
    uint16_t                  opcode;  /**< 1 = request, 2 = reply*/
    TIESC_ETHERNET_ADDRESS  mac_source; /**< Source MAC*/
    uint16_t                  ip_source[2]; /**< Source IP*/
    TIESC_ETHERNET_ADDRESS  mac_dest; /**< Destination MAC*/
    uint16_t                  ip_dest[2]; /**< Destination IP*/
} __attribute__((packed))
TIESC_ARP_IP_HEADER;

/**
* @brief IP frame Header
*/
typedef struct
{
    uint8_t        x;  /**< Version and header length*/
    uint8_t        tos; /**< Type of service*/
    uint16_t       length; /**< Total length*/
    uint16_t       identifier; /**< Identification*/
    uint16_t       fragment; /**< Flags and Fragment offset*/
    uint8_t        ttl; /**< Time to live*/
    uint8_t        protocol; /**< following protocol*/
    uint16_t       cksum; /**< Checksum*/
    uint16_t       src[2]; /**< Source IP*/
    uint16_t       dest[2]; /**< Destination IP*/
} __attribute__((packed))
TIESC_IP_HEADER;

/**
* @brief ICMP frame Header
*/
typedef struct
{
    uint8_t       type; /**< Type*/
    uint8_t       code; /**< Code*/
    uint16_t      checksum; /**< Checksum*/
    uint16_t      id; /**< ID*/
    uint16_t      seqNo; /**< Sequence number*/
} __attribute__((packed))
TIESC_ICMP_HEADER;

/**
* @brief IP (ICMP) Frame structure
*/
typedef struct
{
    TIESC_ETHERNET_FRAME        Ether; /**< Ethernet header*/
    TIESC_IP_HEADER             Ip; /**< IP header*/
    union
    {
        TIESC_ICMP_HEADER       Icmp; /**< ICMP header*/
        uint8_t                   Data[(TIESC_ETHERNET_MAX_FRAME_LEN) -
                                       (TIESC_ETHERNET_FRAME_LEN) - (TIESC_IP_HEADER_MINIMUM_LEN)]; /**< payload*/
    } IpData; /**< IP data*/
} __attribute__((packed))
TIESC_ETHERNET_IP_ICMP_MAX_FRAME;

#endif /* EOE_SUPPORTED */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
*  \brief This function performs SPI flash write for remaining bytes in receive
          data buffer once firmware download is finished by EtherCAT master.
*/
void tiesc_boot_2_init_handler();

void tiesc_foe_eoe_init(void);

#endif /* _TIESC_EOE_FOE_H_*/
