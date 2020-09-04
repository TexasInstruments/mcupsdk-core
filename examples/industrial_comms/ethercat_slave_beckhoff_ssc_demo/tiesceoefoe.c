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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ecat_def.h>
#include <applInterface.h>

#include <stdlib.h> /* for malloc, free */
#include <string.h> /* for memset, memcpy, memcmp */
#include <industrial_comms/ethercat_slave/icss_fwhal/tiescbsp.h>
#include <tiesceoefoe.h>
#include <tiescsoc.h>
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if FOE_SUPPORTED
#define FW_CIRC_BUFF_LEN  2048
#endif /* FOE_SUPPORTED */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if EOE_SUPPORTED
const    uint8_t    broadcast_ethernet_address[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

extern uint8_t aIpAdd[4];
extern uint8_t aMacAdd[6];
#endif /* EOE_SUPPORTED */

#if FOE_SUPPORTED
uint32_t file_size;
uint32_t file_write_offset;
uint8_t data_value = 0x00; /* Used to fill upload file with dummy data */

const uint16_t tiesc_firmware_download_header[4] = { 0x4345, 0x5441, 0x5746, 0x5F5F }; /* "ECATFW__" */

extern unsigned char bBootMode; /**< \brief Indicates in slave is in BOOT mode*/

static uint32_t fw_download_flag;
static uint32_t fw_write_offset;
static uint32_t flash_block_size;
volatile static uint32_t write_indx = 0;
volatile static uint32_t read_indx = 0;
volatile static uint8_t cir_buff[FW_CIRC_BUFF_LEN];

#endif /* FOE_SUPPORTED */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

#if EOE_SUPPORTED
extern uint16_t EOE_SendFrameRequest(uint16_t *pData, uint16_t length);

uint16_t tiesc_cal_checksum(uint16_t *data, uint16_t len);

void tiesc_eoe_receive(uint16_t *frame_array, uint16_t frame_size);
#endif /* EOE_SUPPORTED */

#if FOE_SUPPORTED
/**
*  \brief This function should be called when a firmware download command is received
*         from EtherCAT master
*
*  \param password password received from master
*/
void tiesc_start_fw_download(uint32_t password);

/**
*  \brief Stores the received data as part of firmware download from EtherCAT master.
*         This may get called multiple times during firmware download. This function
*         performs OSPI flash writes for every 256-bytes received from master.
*
*  \param pData Pointer to data buffer
*  \param Size  Number of bytes received
*/
void tiesc_store_fw_data(uint16_t *pData, uint16_t Size);

void tiesc_incr_cir_buff_index(uint32_t *index);

void tiesc_write_to_cir_buff(uint8_t d_byte);

uint32_t tiesc_get_cir_buff_avail_bytes();

uint32_t tiesc_read_from_cir_buff(uint8_t *outBuff, uint32_t buff_size);

uint16_t tiesc_foe_read(uint16_t *pName, uint16_t nameSize, uint32_t password,
                        uint16_t maxBlockSize, uint16_t *pData);

uint16_t tiesc_foe_read_data(uint32_t offset, uint16_t maxBlockSize,
                             uint16_t *pData);

uint16_t tiesc_foe_read_data(uint32_t offset, uint16_t maxBlockSize,
                             uint16_t *pData);

uint16_t tiesc_foe_write_data(uint16_t *pData, uint16_t Size,
                              unsigned char bDataFollowing);

uint16_t tiesc_foe_write(uint16_t *pName, uint16_t nameSize, uint32_t password);
#endif /* FOE_SUPPORTED */

/* ========================================================================== */
/*                       Function Defintions                                  */
/* ========================================================================== */

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

#if EOE_SUPPORTED

uint16_t tiesc_cal_checksum(uint16_t *data, uint16_t len)
{
    uint32_t crc = 0, crc_low, crc_high;
    uint16_t ret_crc;

    while(len > 1)
    {
        crc += SWAPWORD(*data);
        data++;
        len -= 2;
    }

    if(len == 1)
    {
        crc += *((uint8_t *)data);
    }

    crc_low = crc & 0xFFFF;
    crc_high = (crc >> 16) & 0xFFFF;
    crc = crc_low + crc_high;

    crc_high = (crc >> 16) & 0xFFFF;
    crc += crc_high;

    if(crc == 0xFFFF)
    {
        crc = 0;
    }

    ret_crc = (uint16_t)crc;
    ret_crc = ~ret_crc;
    return(ret_crc);
}

void tiesc_eoe_receive(uint16_t *frame_array, uint16_t frame_size)
{
    switch(SWAPWORD(((TIESC_ETHERNET_FRAME *)frame_array)->frame_type))
    {
        case TIESC_ETHERNET_FRAME_TYPE_ARP1_SW:
            {
#if STATIC_ETHERNET_BUFFER
                TIESC_ETHERNET_FRAME *send_frame = (TIESC_ETHERNET_FRAME *)aEthernetSendBuffer;
#else
                TIESC_ETHERNET_FRAME *send_frame = (TIESC_ETHERNET_FRAME *) malloc(frame_size);
#endif
                TIESC_ARP_IP_HEADER *arp_ip = (TIESC_ARP_IP_HEADER *) &send_frame[1];

                memcpy(send_frame, frame_array, frame_size);

                if((memcmp(broadcast_ethernet_address, send_frame->destination.b, 4) == 0)
                        && (arp_ip->length_hw_addr == TIESC_ETHERNET_ADDRESS_LEN)
                        && (arp_ip->length_prot_addr == sizeof(uint32_t))
                        && (arp_ip->hw_addr_space == SWAPWORD(TIESC_ARP_HW_ADDR_SPACE_ETHERNET_SW))
                        && (arp_ip->prot_addr_space == SWAPWORD(TIESC_ETHERNET_FRAME_TYPE_IP_SW))
                        && (arp_ip->opcode == SWAPWORD(TIESC_ARP_OPCODE_REQUEST_SW))
                  )
                {
                    memcpy(send_frame->destination.b, send_frame->source.b, 6);
                    memcpy(send_frame->source.b, &aMacAdd[0], 6);

                    memcpy(arp_ip->mac_dest.b, arp_ip->mac_source.b, 6);
                    memcpy(arp_ip->mac_source.b, &aMacAdd[0], 6);
                    memcpy(arp_ip->ip_dest, arp_ip->ip_source, 4);
                    memcpy(arp_ip->ip_source, aIpAdd, 4);

                    arp_ip->opcode = SWAPWORD(TIESC_ARP_OPCODE_REPLY_SW);

                    EOE_SendFrameRequest((uint16_t *) send_frame,
                                         TIESC_ARP_IP_HEADER_LEN + TIESC_ETHERNET_FRAME_LEN);
                }

                else
                {
#if STATIC_ETHERNET_BUFFER
                    send_frame = NULL;
#else

                    if(send_frame != NULL)
                    {
                        free(send_frame);
                        send_frame = NULL;
                    }

#endif
                }
            }
            break;

        case TIESC_ETHERNET_FRAME_TYPE_IP_SW:
            {
#if STATIC_ETHERNET_BUFFER
                TIESC_ETHERNET_IP_ICMP_MAX_FRAME *ip_header = (TIESC_ETHERNET_IP_ICMP_MAX_FRAME
                        *) aEthernetSendBuffer;
#else
                TIESC_ETHERNET_IP_ICMP_MAX_FRAME *ip_header = (TIESC_ETHERNET_IP_ICMP_MAX_FRAME
                        *) malloc(frame_size);
#endif

                memcpy(ip_header, frame_array, frame_size);

                if((ip_header->Ip.protocol == TIESC_IP_PROTOCOL_ICMP)
                        && (ip_header->IpData.Icmp.type == TIESC_ICMP_TYPE_ECHO)
                        && (memcmp(ip_header->Ip.dest, aIpAdd, 4) == 0)
                  )
                {
                    uint16_t length;
                    uint16_t lo = 0;
                    uint16_t hi = 0;
                    uint32_t tmp;

                    lo = ((ip_header->Ip.length) & 0xff) << 8;
                    hi = ip_header->Ip.length >> 8;
                    length = hi + lo;
                    memcpy(&tmp, ip_header->Ip.src, 4);
                    memcpy(ip_header->Ip.src, ip_header->Ip.dest, 4);
                    memcpy(ip_header->Ip.dest, &tmp, 4);

                    ip_header->IpData.Icmp.type = TIESC_ICMP_TYPE_ECHO_REPLY;

                    memcpy(ip_header->Ether.destination.b, ip_header->Ether.source.b, 6);
                    memcpy(ip_header->Ether.source.b, aMacAdd, 6);

                    ip_header->Ip.cksum = 0;
                    ip_header->Ip.cksum = SWAPWORD(tiesc_cal_checksum((uint16_t *) &ip_header->Ip,
                                                   TIESC_IP_HEADER_MINIMUM_LEN));
                    ip_header->IpData.Icmp.checksum = 0;
                    ip_header->IpData.Icmp.checksum = SWAPWORD(tiesc_cal_checksum((
                                                          uint16_t *) &ip_header->IpData.Icmp, (uint16_t)(length - 20)));
                    EOE_SendFrameRequest((uint16_t *) ip_header,
                                         (uint16_t)(TIESC_IP_HEADER_MINIMUM_LEN + length));
                }

                else
                {
#if STATIC_ETHERNET_BUFFER
                    ip_header = (TIESC_ETHERNET_IP_ICMP_MAX_FRAME *) NULL;
#else

                    if(ip_header != NULL)
                    {
                        free(ip_header);
                        ip_header = NULL;
                    }

#endif
                }
            }
            break;
    }
}
#endif /* EOE_SUPPORTED */

#if FOE_SUPPORTED

void tiesc_start_fw_download(uint32_t password)
{
    fw_download_flag = 1;
    DebugP_log("FW download started\n\r");
}

void tiesc_store_fw_data(uint16_t *pData, uint16_t Size)
{
    uint32_t itr1 = 0;
    uint8_t *temp_ptr = (uint8_t *)pData;
    uint8_t data_buff[256];
    uint32_t blockNum, pageNum;      /* Block, page number */

    for(itr1 = 0 ; itr1 < Size  ; itr1++)
    {
        tiesc_write_to_cir_buff(temp_ptr[itr1]);
    }

    while(tiesc_get_cir_buff_avail_bytes() >= 256)
    {
        /* Call Flash write code from here */
        if((fw_write_offset & (flash_block_size - 1)) == 0)
        {
            /* Erase Flash block */
            Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0],fw_write_offset, &blockNum, &pageNum);
            Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blockNum);
        }

        if(tiesc_read_from_cir_buff(data_buff, 256) >= 256)
        {
            Flash_write(gFlashHandle[CONFIG_FLASH0], fw_write_offset, (uint8_t *)data_buff, 256);
            fw_write_offset += 256;
        }
    }
}

void tiesc_boot_2_init_handler()
{
    uint8_t data_buff[256];
    uint32_t blockNum, pageNum;      /* Block, page number */

    /*
    * Make sure that firware is completely written to SPI flash
    */
    while(tiesc_get_cir_buff_avail_bytes() > 0)
    {
        /* Call Flash write code from here */
        if((fw_write_offset & (flash_block_size - 1)) == 0)
        {
            /* Erase Flash block */
            Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0],fw_write_offset, &blockNum, &pageNum);
            Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blockNum);
        }

        if(tiesc_read_from_cir_buff(data_buff, 256) > 0)
        {
            Flash_write(gFlashHandle[CONFIG_FLASH0], fw_write_offset, (uint8_t *)data_buff, 256);
            fw_write_offset += 256;
        }
    }

    /*
    * Set FW reload flag from here if a new binary is recieved
    */
    if(fw_download_flag)
    {
        DebugP_log("FW download completed\n\r");
        /* Reset the flag */
        fw_download_flag = 0;
    }
}

void tiesc_incr_cir_buff_index(uint32_t *index)
{
    (*index)++;

    if(*index == FW_CIRC_BUFF_LEN)
    {
        *index = 0;
    }
}

void tiesc_write_to_cir_buff(uint8_t d_byte)
{
    cir_buff[ write_indx ] = d_byte;
    tiesc_incr_cir_buff_index((uint32_t *)&write_indx);

    if(write_indx == read_indx)
    {
        tiesc_incr_cir_buff_index((uint32_t *)&read_indx);
    }
}

uint32_t tiesc_get_cir_buff_avail_bytes()
{
    if(write_indx >= read_indx)
    {
        return write_indx - read_indx;
    }
    else
    {
        return ((FW_CIRC_BUFF_LEN - read_indx) + write_indx);
    }
}

uint32_t tiesc_read_from_cir_buff(uint8_t *outBuff, uint32_t buff_size)
{
    uint32_t itr1 = 0;
    uint32_t avail_bytes = tiesc_get_cir_buff_avail_bytes();

    if(avail_bytes == 0)
    {
        return 0;
    }

    for(itr1 = 0 ; (itr1 < avail_bytes) && (itr1 < buff_size) ; itr1++)
    {
        outBuff[itr1] = cir_buff[read_indx];
        tiesc_incr_cir_buff_index((uint32_t *)&read_indx);
    }

    return avail_bytes;
}

uint16_t tiesc_foe_read(uint16_t *pName, uint16_t nameSize, uint32_t password,
                        uint16_t maxBlockSize, uint16_t *pData)
{
    uint16_t size = 0;

    if(file_size > 0)
    {
        if(file_size >= maxBlockSize)
        {
            size = maxBlockSize;
        }

        else
        {
            size = (unsigned short) file_size;
        }

        memset(pData, data_value, size);
        data_value++;
    }

    else
    {
        return TIESC_ECAT_FOE_ERRCODE_NOTFOUND;
    }

    return size;
}

uint16_t tiesc_foe_read_data(uint32_t offset, uint16_t maxBlockSize,
                             uint16_t *pData)
{
    uint16_t size = 0;

    if(file_size < offset)
    {
        return 0;
    }

    size = (uint16_t)(file_size - offset);

    if(size > maxBlockSize)
    {
        size = maxBlockSize;
    }

    memset(pData, data_value, size);
    data_value++;

    return size;
}

uint16_t tiesc_foe_write_data(uint16_t *pData, uint16_t Size,
                              unsigned char bDataFollowing)
{
    if(bBootMode)
    {
        tiesc_store_fw_data(pData, Size);
    }
    else if(bDataFollowing)
    {
        file_write_offset += Size;
    }
    else
    {
        file_size = file_write_offset + Size;
        file_write_offset = 0;
    }

    return 0;
}

uint16_t tiesc_foe_write(uint16_t *pName, uint16_t nameSize, uint32_t password)
{
    if((nameSize >= sizeof(tiesc_firmware_download_header))
            && (pName[0] == tiesc_firmware_download_header[0])
            && (pName[1] == tiesc_firmware_download_header[1])
            && (pName[2] == tiesc_firmware_download_header[2])
      )
    {
        if(bBootMode)
        {
            tiesc_start_fw_download(password);
            return 0;
        }
        else
        {
            return TIESC_ECAT_FOE_ERRCODE_BOOTSTRAPONLY;
        }
    }
    else if(bBootMode)
    {
        return TIESC_ECAT_FOE_ERRCODE_NOTINBOOTSTRAP;
    }
    else
    {
        data_value = 0x00;
        file_size = 0;
        return 0;
    }
}

#endif /* FOE_SUPPORTED */

void tiesc_foe_eoe_init(void)
{
#if EOE_SUPPORTED
    pAPPL_EoeReceive = tiesc_eoe_receive;
#endif

#if FOE_SUPPORTED
    pAPPL_FoeRead = tiesc_foe_read;
    pAPPL_FoeReadData = tiesc_foe_read_data;
    pAPPL_FoeWriteData = tiesc_foe_write_data;
    pAPPL_FoeWrite = tiesc_foe_write;
    file_size = 0;
    fw_download_flag = 0;
    tiesc_getFoeFlashOffset(&fw_write_offset);
    flash_block_size = ((Flash_Attrs *)Flash_getAttrs(CONFIG_FLASH0))->blockSize;
#endif
}
