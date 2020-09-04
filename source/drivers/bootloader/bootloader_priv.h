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

#ifndef BOOTLOADER_PRIV_H_
#define BOOTLOADER_PRIV_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* Magic number and tokens for RPRC format */
#define BOOTLOADER_RPRC_MAGIC_NUMBER   0x43525052
#define BOOTLOADER_RPRC_RESOURCE       0
#define BOOTLOADER_RPRC_BOOTADDR       5

#define BOOTLOADER_MAX_INPUT_FILES 10
#define BOOTLOADER_META_HDR_MAGIC_STR 0x5254534D /* MSTR in ascii */
#define BOOTLOADER_META_HDR_MAGIC_END 0x444E454D /* MEND in ascii */

#define BOOTLOADER_MAX_ADDR_REGIONS 2

/* Maximum number of reserved memory sections across all SOCs */
#define BOOTLOADER_MAX_RESERVED_SECTIONS 2

#define BOOTLOADER_DEVICE_ID 55U

typedef struct Bootloader_RprcFileHeader_s 
{
    uint32_t magic;
    uint32_t entry;
    uint32_t rsvdAddr;
    uint32_t sectionCount;
    uint32_t version;

} Bootloader_RprcFileHeader;

typedef struct Bootloader_RprcSectionHeader_s 
{
    uint32_t addr;
    uint32_t rsvdAddr;
    uint32_t size;
    uint32_t rsvdCrc;
    uint32_t rsvd;

} Bootloader_RprcSectionHeader;

typedef struct Bootloader_MetaHeaderStart_s
{
    uint32_t magicStr;
    uint32_t numFiles;
    uint32_t devId;
    uint32_t rsvd;

} Bootloader_MetaHeaderStart;

typedef struct Bootloader_MetaHeaderCore_s
{
    uint32_t coreId;
    uint32_t imageOffset;

} Bootloader_MetaHeaderCore;

typedef struct Bootloader_MetaHeaderEnd_s
{
    uint32_t rsvd;
    uint32_t magicStringEnd;

} Bootloader_MetaHeaderEnd;

typedef struct 
{
	uint32_t cpuLocalAddr;
	uint32_t socAddr;
	uint32_t regionSize;

} Bootloader_addrRegionInfo;

typedef struct 
{
	uint32_t numRegions;
	Bootloader_addrRegionInfo addrRegionInfo[BOOTLOADER_MAX_ADDR_REGIONS];

} Bootloader_CoreAddrTranslateInfo;

typedef struct
{
    uint32_t memStart;
    uint32_t memEnd;
} Bootloader_memSection;

typedef struct
{
    uint32_t numSections;
    Bootloader_memSection memSection[BOOTLOADER_MAX_RESERVED_SECTIONS];
} Bootloader_resMemSections;

/**
 * \brief API to get the scratch memory limits used by SBL. If the application tries to load in this region, it might over write SBL
 */
Bootloader_resMemSections* Bootloader_socGetSBLMem(void);

uint32_t Bootloader_getX509CertLen(uint8_t *x509_cert_ptr);
uint32_t Bootloader_getMsgLen(uint8_t *x509_cert_ptr, uint32_t x509_cert_size);


#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_PRIV_H_ */