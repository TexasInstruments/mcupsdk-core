/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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
 *  \file bootloader.c
 *
 *  \brief Bootloader Driver API source file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/CacheP.h>

#include <drivers/bootloader.h>
#ifdef DRV_VERSION_MMCSD_V0
#include <drivers/bootloader/bootloader_mmcsd_raw.h>
#endif
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <string.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*RPRC image ID for linux load only images */
#define RPRC_LINUX_LOAD_ONLY_IMAGE_ID (21U)
#define SEGMENT_MAP_NOTE_TYPE (0xBBBB7777)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

extern Bootloader_Config gBootloaderConfig[];
extern uint32_t gBootloaderConfigNum;

#ifdef DRV_VERSION_MMCSD_V0
extern Bootloader_MemArgs gMemBootloaderArgs;
#endif

uint8_t gElfHBuffer[ELF_HEADER_MAX_SIZE];
uint8_t gNoteSegBuffer[ELF_NOTE_SEGMENT_MAX_SIZE];
uint8_t gPHTBuffer[ELF_MAX_SEGMENTS * ELF_P_HEADER_MAX_SIZE];

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

Bootloader_Handle Bootloader_open(uint32_t instanceNum, Bootloader_Params *openParams)
{
    Bootloader_Config *config = NULL;

    if(instanceNum < gBootloaderConfigNum)
    {
        config = &gBootloaderConfig[instanceNum];
        if(config->fxns && config->fxns->imgOpenFxn && config->args)
        {
            int32_t status;

            status = config->fxns->imgOpenFxn(config->args, openParams);
            if(status != SystemP_SUCCESS)
            {
                config = NULL;
            }
        }
    }

    return (Bootloader_Handle)config;
}

void Bootloader_close(Bootloader_Handle handle)
{
    Bootloader_Config *config = (Bootloader_Config *)handle;

    if(config && config->fxns && config->fxns->imgCloseFxn)
    {
        config->fxns->imgCloseFxn(handle, config->args);
    }
}

int32_t Bootloader_initCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo)
{
    int32_t status = SystemP_SUCCESS;

    status = Bootloader_socCpuRequest(cpuInfo->cpuId);

    status = Bootloader_socCpuSetClock(cpuInfo->cpuId, cpuInfo->clkHz);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_Config *config = (Bootloader_Config *)handle;
        status = Bootloader_socCpuPowerOnReset(cpuInfo->cpuId,config->socCoreOpMode);
    }

    return status;
}

int32_t Bootloader_loadCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo)
{
    int32_t status = SystemP_SUCCESS;

    status = Bootloader_socCpuRequest(cpuInfo->cpuId);

    status = Bootloader_socCpuSetClock(cpuInfo->cpuId, cpuInfo->clkHz);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_Config *config = (Bootloader_Config *)handle;
        status = Bootloader_socCpuPowerOnReset(cpuInfo->cpuId,config->socCoreOpMode);
    }

    if(SystemP_SUCCESS == status)
    {
        if( cpuInfo->rprcOffset != BOOTLOADER_INVALID_ID)
        {
            status = Bootloader_rprcImageLoad(handle, cpuInfo);
        }
    }

    return status;
}

int32_t Bootloader_runCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo)
{
    int32_t status = SystemP_SUCCESS;
    uintptr_t entryPoint = cpuInfo->entryPoint;

    if( cpuInfo->rprcOffset == BOOTLOADER_INVALID_ID)
    {
        /* boot a dummy while(1) loop */
        entryPoint = 0;
    }
    status = Bootloader_socCpuResetRelease(cpuInfo->cpuId, entryPoint);

    if(SystemP_SUCCESS == status)
    {
        status = Bootloader_socCpuRelease(cpuInfo->cpuId);
    }

    if(status == SystemP_SUCCESS)
    {
        uint64_t cpuHz;

        cpuHz = Bootloader_socCpuGetClock(cpuInfo->cpuId);
        if(cpuHz > 0)
        {
            DebugP_logInfo("CPU %s is initialized to %d Hz !!!\r\n",
                Bootloader_socGetCoreName(cpuInfo->cpuId), (uint32_t)cpuHz);
        }
    }

    return status;
}

int32_t Bootloader_loadSelfCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo, uint32_t skipLoad)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t cpuId = cpuInfo->cpuId;

    status = Bootloader_socCpuRequest(cpuId);
    if(SystemP_SUCCESS == status)
    {
        status = Bootloader_socCpuSetClock(cpuId, cpuInfo->clkHz);
    }
    if(SystemP_SUCCESS == status)
    {
        if( cpuInfo->rprcOffset != BOOTLOADER_INVALID_ID)
        {
            status = Bootloader_rprcImageParseEntryPoint(handle, cpuInfo);
        }
    }
    if(SystemP_SUCCESS == status)
    {
        uintptr_t entryPoint = cpuInfo->entryPoint;

        if( cpuInfo->rprcOffset == BOOTLOADER_INVALID_ID)
        {
            entryPoint = 0;
        }
        status = Bootloader_socCpuSetEntryPoint(cpuId, entryPoint);
    }
    if(SystemP_SUCCESS == status)
    {
        status = Bootloader_socMemInitCpu(cpuId);
    }
    if((SystemP_SUCCESS == status) && (skipLoad == FALSE))
    {
        if( cpuInfo->rprcOffset != BOOTLOADER_INVALID_ID)
        {
            status = Bootloader_rprcImageLoad(handle, cpuInfo);
        }
    }
    if(status == SystemP_SUCCESS)
    {
        uint64_t cpuHz;

        cpuHz = Bootloader_socCpuGetClock(cpuId);
        if(cpuHz > 0)
        {
            DebugP_logInfo("CPU %s is initialized to %d Hz !!!\r\n", Bootloader_socGetCoreName(cpuId), (uint32_t)cpuHz);
        }
    }
    return status;
}

int32_t Bootloader_runSelfCpu(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_SUCCESS;

    DebugP_logInfo("All done, reseting self ...\r\n\n");
    status = Bootloader_socCpuResetReleaseSelf();
    /* control will not reach here */

    return status;
}

int32_t Bootloader_bootCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo)
{
    int32_t status = SystemP_SUCCESS;

    status = Bootloader_loadCpu(handle, cpuInfo);

    if(status == SystemP_SUCCESS)
    {
        status = Bootloader_runCpu(handle, cpuInfo);
    }

    return status;
}

int32_t Bootloader_rprcImageParseEntryPoint(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_RprcFileHeader header;

    Bootloader_Config *config = (Bootloader_Config *)handle;

    config->fxns->imgSeekFxn(cpuInfo->rprcOffset, config->args);
    status = config->fxns->imgReadFxn(&header, sizeof(Bootloader_RprcFileHeader), config->args);

    cpuInfo->entryPoint = (uintptr_t)header.entry;

    return status;
}

int32_t Bootloader_cpuSetAppEntryPoint(Bootloader_BootImageInfo *bootImageInfo, uint32_t bDualSelfR5F)
{
    int32_t status = SystemP_SUCCESS;
    uintptr_t entryPoint;
    uint32_t cpuId;

    cpuId = CSL_CORE_ID_R5FSS0_0;
    entryPoint = bootImageInfo->cpuInfo[cpuId].entryPoint;

    status = Bootloader_socCpuSetAppEntryPoint(cpuId, entryPoint);

    if(bDualSelfR5F == TRUE)
    {
        cpuId = CSL_CORE_ID_R5FSS0_1;
        entryPoint = bootImageInfo->cpuInfo[cpuId].entryPoint;

        status = Bootloader_socCpuSetAppEntryPoint(cpuId, entryPoint);
    }

    return status;
}

int32_t Bootloader_bootSelfCpu(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i = 0;
    uint32_t *selfCpuList  = Bootloader_socGetSelfCpuList();

    while( selfCpuList[i] != BOOTLOADER_INVALID_ID)
    {
        status = Bootloader_loadSelfCpu(handle, &bootImageInfo->cpuInfo[ selfCpuList[i] ], FALSE);
        if(status!=SystemP_SUCCESS)
        {
            break;
        }
        i++;
    }

    if(status == SystemP_SUCCESS)
    {
        status = Bootloader_runSelfCpu(handle, bootImageInfo);
    }

    return status;
}

int32_t Bootloader_rprcImageLoad(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo)
{
    Bootloader_RprcFileHeader     header;
    Bootloader_RprcSectionHeader section;

    int32_t status = SystemP_SUCCESS;

    Bootloader_Config *config = (Bootloader_Config *)handle;

    config->fxns->imgSeekFxn(cpuInfo->rprcOffset, config->args);
    status = config->fxns->imgReadFxn(&header, sizeof(Bootloader_RprcFileHeader), config->args);

    if(header.magic != BOOTLOADER_RPRC_MAGIC_NUMBER)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        cpuInfo->entryPoint = (uintptr_t)header.entry;

        uint32_t i;

        for(i=0; i<header.sectionCount; i++)
        {
            status = config->fxns->imgReadFxn(&section, sizeof(Bootloader_RprcSectionHeader), config->args);

            section.addr = Bootloader_socTranslateSectionAddr(cpuInfo->cpuId, section.addr);

            /* Add check for SBL reserved memory */
            Bootloader_resMemSections *resMem;
            uint32_t resSectionCnt, start, end;
            resMem = Bootloader_socGetSBLMem();
            for (resSectionCnt = 0; resSectionCnt < resMem->numSections; resSectionCnt++)
            {
                start = resMem->memSection[resSectionCnt].memStart;
                end = resMem->memSection[resSectionCnt].memEnd;
                if((section.addr > start) && (section.addr < end))
                {
                    status = SystemP_FAILURE;
                    DebugP_logError("Application image has a load address (0x%08X) in the SBL reserved memory range!!\r\n", section.addr);
                    break;
                }
            }
            if (status == SystemP_SUCCESS)
            {
                status = config->fxns->imgReadFxn((void *)(uintptr_t)(section.addr), section.size, config->args);
                config->bootImageSize += section.size;
            }
        }
    }

    return status;
}

uint32_t Bootloader_getX509CertLen(uint8_t *x509_cert_ptr)
{
    uint32_t certLen = 0;
    uint8_t *pCertLen = (uint8_t *)&certLen;

    if ( *x509_cert_ptr != 0x30)
    {
        return 0;
    }

    certLen = *(x509_cert_ptr + 1);

    /* If you need more than 2 bytes to store the cert length  */
    /* it means that the cert length is greater than 64 Kbytes */
    /* and we do not support it                                */
    if ((certLen > 0x80) &&
        (certLen != 0x82))
    {
        return 0;
    }

    if ( certLen == 0x82)
    {
        *pCertLen = *(x509_cert_ptr + 3);
        *(pCertLen + 1) = *(x509_cert_ptr + 2);

        /* add current offset from start of x509 cert */
        certLen += 3;
    }
    else
    {
        /* add current offset from start of x509 cert  */
        /* if cert len was obtained from 2nd byte i.e. */
        /* cert size is 127 bytes or less              */
        certLen += 1;
    }

    /* certLen now contains the offset of the last byte */
    /* of the cert from the ccert_start. To get the size */
    /* of certificate, add 1                             */

    return certLen + 1;
}

uint8_t *Bootloader_findSeq(uint8_t *x509_cert_ptr, uint32_t x509_cert_size, uint8_t *seq_oid, uint8_t seq_len)
{
    uint8_t *x509_cert_end = x509_cert_ptr + x509_cert_size - seq_len;

    /* searching for the following byte seq in the cert */
    /* seq_id(0x30) seq_len(< 0x80) 0x06 0x09 0x2B...   */
    while (x509_cert_ptr < x509_cert_end)
    {
        if ((*x509_cert_ptr == seq_oid[0]) &&
            (*(x509_cert_ptr + 2) == seq_oid[2]) &&
            (*(x509_cert_ptr - 2) == 0x30))
        {
            if ((memcmp((const void *)x509_cert_ptr, (const void *)seq_oid, seq_len)) == 0)
            {
                /* return start boot_seq */
                return (x509_cert_ptr - 2);
            }
        }
        x509_cert_ptr++;
    }

    return NULL;
}

uint32_t Bootloader_getMsgLen(uint8_t *x509_cert_ptr, uint32_t x509_cert_size)
{
    uint8_t *boot_seq_ptr;
    uint32_t msg_len = 0, boot_seq_len;
    uint8_t *msg_len_ptr = (uint8_t *)&msg_len;
    uint8_t boot_seq_oid[11];
    Bootloader_socGetBootSeqOid(boot_seq_oid);
    boot_seq_ptr = Bootloader_findSeq(x509_cert_ptr, x509_cert_size, boot_seq_oid, sizeof(boot_seq_oid));

    /* length of seq is stored in the byte after the 0x30 seq_id */
    /* length of seq is stored as offset of the last byte of seq */
    /* from current offset. Jump to the end of the boot seq as   */
    /* the length of the message  is the last field of this seq  */
    boot_seq_len = *(++boot_seq_ptr);
    boot_seq_ptr = boot_seq_ptr + boot_seq_len;

    /* The last integer in this sequence is the msg length    */
    /* integers are tagged 0x20, so search backwards for 0x20 */
    /* The msg size can be encoded in 1, 2, 3 or 4 bytes      */
    /* 0x02 0x01 0x##                                         */
    /* 0x02 0x02 0x## 0x##                                    */
    /* 0x02 0x03 0x## 0x## 0x##                               */
    /* 0x02 0x04 0x## 0x## 0x## 0x##                          */
    if ( (*(boot_seq_ptr - 5) == 0x02) &&
         (*(boot_seq_ptr - 4) == 0x04) )
    {
        /* msg length encoded in 4 bytes */
        *msg_len_ptr = *boot_seq_ptr;
        *(msg_len_ptr + 1) = *(boot_seq_ptr - 1);
        *(msg_len_ptr + 2) = *(boot_seq_ptr - 2);
        *(msg_len_ptr + 3) = *(boot_seq_ptr - 3);
    }
    else if ( (*(boot_seq_ptr - 4) == 0x02) &&
         (*(boot_seq_ptr - 3) == 0x03) )
    {
        /* msg length encoded in 3 bytes */
        *msg_len_ptr = *boot_seq_ptr;
        *(msg_len_ptr + 1) = *(boot_seq_ptr - 1);
        *(msg_len_ptr + 2) = *(boot_seq_ptr - 2);
    }
    else if ( (*(boot_seq_ptr - 3) == 0x02) &&
         (*(boot_seq_ptr - 2) == 0x02) )
    {
        /* msg length encoded in 2 bytes */
        *msg_len_ptr = *boot_seq_ptr;
        *(msg_len_ptr + 1) = *(boot_seq_ptr - 1);
    }
    else if ( (*(boot_seq_ptr - 2) == 0x02) &&
         (*(boot_seq_ptr - 1) == 0x01) )
    {
        /* msg length encoded in 1 byte */
        *msg_len_ptr = *boot_seq_ptr;
    }

    return msg_len;
}

/* This API should only be called after all the rprc loading is complete */
uint32_t Bootloader_getMulticoreImageSize(Bootloader_Handle handle)
{
    uint32_t size = 0;

    if(handle != NULL)
    {
        Bootloader_Config *config = (Bootloader_Config *)handle;
        size = config->bootImageSize;
    }
    return size;
}

/* This API should only be called after the bootimage is parsed */
uint32_t Bootloader_isCorePresent(Bootloader_Handle handle, uint32_t cslCoreId)
{
    uint32_t retVal = FALSE;

    if(handle != NULL)
    {
        Bootloader_Config *config = (Bootloader_Config *)handle;
        if((config->coresPresentMap & (1 << cslCoreId)) != 0)
        {
            retVal = TRUE;
        }
    }

    return retVal;
}

uint32_t Bootloader_getBootMedia(Bootloader_Handle handle)
{
    uint32_t media = BOOTLOADER_INVALID_ID;

    if(NULL != handle)
    {
        Bootloader_Config *config = (Bootloader_Config *)handle;
        media = config->bootMedia;
    }

    return media;
}

int32_t Bootloader_verifyMulticoreImage(Bootloader_Handle handle)
{
    int32_t status = SystemP_FAILURE, authStatus = SystemP_FAILURE;
    uint32_t certLen = 0U, imageLen = 0U;
    uint32_t certLoadAddr = 0xFFFFFFFFU;

    Bootloader_Config *config = (Bootloader_Config *)handle;

    if(config->fxns->imgReadFxn == NULL || config->fxns->imgSeekFxn == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        uint8_t x509Header[4];
        if((config->bootMedia == BOOTLOADER_MEDIA_MEM) ||
                (config->bootMedia == BOOTLOADER_MEDIA_PCIE))
        {
            Bootloader_MemArgs *memArgs = (Bootloader_MemArgs *)(config->args);
            certLoadAddr = memArgs->appImageBaseAddr;
        }
        else if(config->bootMedia == BOOTLOADER_MEDIA_FLASH)
        {
            Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)(config->args);
            certLoadAddr = flashArgs->appImageOffset + SOC_getFlashDataBaseAddr();
        }
#ifdef DRV_VERSION_MMCSD_V0
        else if(config->bootMedia == BOOTLOADER_MEDIA_EMMC)
        {
            Bootloader_MmcsdArgs *mmcsdArgs = (Bootloader_MmcsdArgs *)(config->args);
            certLoadAddr = mmcsdArgs->appImageOffset;

            config->fxns->imgReadFxn(x509Header, 4, config->args);
            config->fxns->imgSeekFxn(0, config->args);

            if(config->scratchMemPtr != NULL)
            {
                certLen = Bootloader_getX509CertLen(x509Header);
                config->fxns->imgReadFxn((void *)config->scratchMemPtr, 0x800, config->args);

                imageLen = Bootloader_getMsgLen((uint8_t *)config->scratchMemPtr, certLen);

                uint32_t totalLen = (certLen + imageLen + 128) & ~(127);

                config->fxns->imgSeekFxn(0, config->args);
                config->fxns->imgReadFxn((void *)config->scratchMemPtr, totalLen, config->args);

                certLoadAddr = (uint32_t)(&(config->scratchMemPtr[0]));

                config->fxns->imgSeekFxn(0, config->args);
            }
        }
#endif
        if (config->bootMedia != BOOTLOADER_MEDIA_EMMC)
        {
            /* Read first 4 bytes of the appimage to determine if it is signed with x509 certificate */
            config->fxns->imgReadFxn(x509Header, 4, config->args);
            config->fxns->imgSeekFxn(0, config->args);

            certLen = Bootloader_getX509CertLen(x509Header);
            
            if(config->bootMedia == BOOTLOADER_MEDIA_FLASH)
            {
                Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)(config->args);
                
                /* Enable DAC mode as Flash is accessed in memory mapped mode */
                flashArgs->enableDacMode = TRUE;
                if(config->fxns->imgCustomFxn)
                {
                    config->fxns->imgCustomFxn(config->args);                    
                }
            }
            
            imageLen = Bootloader_getMsgLen((uint8_t *)certLoadAddr, certLen);
        }
        /* Get the 128B cache-line aligned image length */
        uint32_t cacheAlignedLen = (certLen + imageLen + 128) & ~(127);

        /* Write back and invalidate the cache before passing to HSM */
        CacheP_wbInv((void *)certLoadAddr, cacheAlignedLen, CacheP_TYPE_ALL);

        /* Check if the certificate length is within valid range */
        if((certLen > 0x100) && (certLen < 0x800))
        {
            if(config->disableAppImageAuth == TRUE)
            {
                /* NOTE: This is an option to skip image authentication in a signed
                image to aid initial development on HS devices. If the user has
                opted to disable image authentication, do not authenticate/decrypt.
                Skip the certificate length and start loading as in GP */
                authStatus = SystemP_SUCCESS;
            }
            else
            {
                authStatus = Bootloader_socAuthImage(certLoadAddr);
            }

            /* Invalidate the cache before reading in case HSM decrypted image */
            CacheP_inv((void *)certLoadAddr, cacheAlignedLen, CacheP_TYPE_ALL);

            if(config->bootMedia == BOOTLOADER_MEDIA_BUFIO)
            {
                /* Authentication will fail in Buf Io because we don't have full data yet,
                so make it pass here for testing. Default behaviour is to assert. */

                /* authStatus = SystemP_SUCCESS; */
                DebugP_assertNoLog(authStatus == SystemP_SUCCESS);
            }

            if(authStatus == SystemP_FAILURE)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Authentication passed, all good. Now re-init bootloader params to point to image start instead of start of x509 certificate */
                if((config->bootMedia == BOOTLOADER_MEDIA_MEM) ||
                    (config->bootMedia == BOOTLOADER_MEDIA_PCIE))
                {
                    Bootloader_MemArgs *memArgs = (Bootloader_MemArgs *)(config->args);
                    memArgs->appImageBaseAddr += certLen;
                }
                else if(config->bootMedia == BOOTLOADER_MEDIA_FLASH)
                {
                    Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)(config->args);
                    flashArgs->appImageOffset += certLen;
                    flashArgs->curOffset = flashArgs->appImageOffset;
                }
                else if(config->bootMedia == BOOTLOADER_MEDIA_BUFIO)
                {
                    Bootloader_BufIoArgs *bufIoArgs = (Bootloader_BufIoArgs *)(config->args);
                    bufIoArgs->appImageOffset += certLen;
                    bufIoArgs->curOffset = bufIoArgs->appImageOffset;
                }
#ifdef DRV_VERSION_MMCSD_V0
                else if(config->bootMedia == BOOTLOADER_MEDIA_EMMC)
                {
                    Bootloader_MmcsdArgs *mmcsdArgs = (Bootloader_MmcsdArgs *)(config->args);
                    mmcsdArgs->appImageOffset += certLen;
                    mmcsdArgs->curOffset = mmcsdArgs->appImageOffset;
                                        
                    /* At this point the image is in RAM, change bootmedia from emmc to memory */
                    gMemBootloaderArgs.appImageBaseAddr = (uint32_t)(config->scratchMemPtr) + certLen;
                    gMemBootloaderArgs.curOffset = 0U;
                    
                    config->fxns = &gBootloaderMemFxns;
                    config->args = &gMemBootloaderArgs;
                    config->bootMedia = BOOTLOADER_MEDIA_MEM;
                }
#endif
                status = SystemP_SUCCESS;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
        
        if(config->bootMedia == BOOTLOADER_MEDIA_FLASH)
        {
            Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)(config->args);
            
            /* Disable DAC mode for Flash */
            flashArgs->enableDacMode = FALSE;
            if(config->fxns->imgCustomFxn)
            {
                config->fxns->imgCustomFxn(config->args);                    
            }
        }
    }

    return status;
}

int32_t auth_start(uintptr_t certOffset)
{
    return 0;
}

int32_t auth_update(uintptr_t startAddr, uint32_t size)
{
    return 0;
}

int32_t auth_finish()
{
    int32_t status = status = SystemP_SUCCESS;

    return status;
}

int32_t Bootloader_parseNoteSegment(Bootloader_Handle handle,
                                    uint32_t noteSegmentSz,
                                    uint32_t *segmentMapIdx)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t idx = 0;
    Bootloader_ELFNote *notePtr = (Bootloader_ELFNote *)gNoteSegBuffer;
    uint32_t sgMpIdx = 0;
    uint32_t alignSize = 4U;

    while(idx < noteSegmentSz)
    {
        /* Read the type */
        idx += (ELF_NOTE_NAMESZ_SIZE + ELF_NOTE_DESCSZ_SIZE + ELF_NOTE_TYPE_SIZE);
        if(notePtr->type == SEGMENT_MAP_NOTE_TYPE)
        {
            sgMpIdx = idx + notePtr->namesz;
            if(notePtr->namesz % alignSize != 0)
            {
                sgMpIdx += (alignSize - (notePtr->namesz % alignSize));
            }
            break;
        }
        else
        {
            idx += notePtr->namesz + notePtr->descsz;
            if(notePtr->namesz % alignSize != 0)
            {
                idx += (alignSize - (notePtr->namesz % alignSize));
            }
            if(notePtr->descsz % alignSize != 0)
            {
                idx += (alignSize - (notePtr->descsz % alignSize));
            }
            notePtr = (Bootloader_ELFNote *)((uintptr_t)gNoteSegBuffer + idx);
        }
    }

    if(sgMpIdx == 0)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        *segmentMapIdx = sgMpIdx;
    }

    return status;
}

int32_t Bootloader_parseAndLoadMultiCoreELF(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t certOffset = 0U;
    uint32_t elfClass = ELFCLASS_32;
    uint32_t elfSize = ELF_HEADER_32_SIZE;
    uint32_t segmentMapIdx = 0;

    Bootloader_Config *config = (Bootloader_Config *)handle;

    if(config->fxns->imgReadFxn == NULL || config->fxns->imgSeekFxn == NULL)
    {
        status = SystemP_FAILURE;
    }
	config->fxns->imgSeekFxn(0, config->args);

    uint32_t doAuth = ((Bootloader_socIsAuthRequired() == TRUE) && (config->isAppimageSigned == TRUE));

    /* Security will come later, here it should start the streaming AUTH by sending x.509 first */
    if((status == SystemP_SUCCESS) && (doAuth == TRUE))
    {
        status = auth_start(certOffset);
    }

    if(status == SystemP_SUCCESS)
    {
        uint8_t x509Header[4];
        config->fxns->imgReadFxn(x509Header, 4, config->args);
        uint32_t imgOffset = Bootloader_getX509CertLen(x509Header);
        uint32_t rdSz = ELFCLASS_IDX + 1;

        /* parse ELF and verify ELF signature */
        config->fxns->imgSeekFxn(imgOffset, config->args);
        status = config->fxns->imgReadFxn(gElfHBuffer, rdSz, config->args);
        char ELFSTR[] = { 0x7F, 'E', 'L', 'F' };
        if(memcmp(ELFSTR, gElfHBuffer, 4) != 0)
        {
            status = SystemP_FAILURE;
        }
        elfClass = gElfHBuffer[ELFCLASS_IDX];
        config->fxns->imgSeekFxn((imgOffset + rdSz), config->args);
        if(elfClass == ELFCLASS_64)
        {
            elfSize = ELF_HEADER_64_SIZE;
        }
        status = config->fxns->imgReadFxn((gElfHBuffer + rdSz), (elfSize - rdSz), config->args);
    }

    if((status == SystemP_SUCCESS) && (doAuth == TRUE))
    {
        auth_update((uintptr_t)gElfHBuffer, elfSize);
    }

    Bootloader_ELFH32 *elfPtr32 = (Bootloader_ELFH32 *)gElfHBuffer;
    Bootloader_ELFH64 *elfPtr64 = (Bootloader_ELFH64 *)gElfHBuffer;

    uint32_t phtSize = ((elfPtr32->e_phnum) * (elfPtr32->e_phentsize));

    if(elfClass == ELFCLASS_64)
    {
        phtSize = ((elfPtr64->e_phnum) * (elfPtr64->e_phentsize));
    }

    uint32_t numSegments = elfPtr32->e_phnum;
    if(elfClass == ELFCLASS_64)
    {
        numSegments = elfPtr64->e_phnum;
    }

    /* Check if number of PHT entries are <= MAX */
    if(numSegments > ELF_MAX_SEGMENTS)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        uint32_t phoff = elfPtr32->e_phoff;
        if(elfClass == ELFCLASS_64)
        {
            phoff = elfPtr64->e_phoff;
        }
        config->fxns->imgSeekFxn(phoff, config->args);
    }
    status = config->fxns->imgReadFxn((void *)(gPHTBuffer), phtSize, config->args);

    if(status == SystemP_SUCCESS && (doAuth == TRUE))
    {
        auth_update((uintptr_t)(gPHTBuffer), phtSize);
    }

    Bootloader_ELFPH32 *elfPhdrPtr32 = (Bootloader_ELFPH32 *)gPHTBuffer;
    Bootloader_ELFPH64 *elfPhdrPtr64 = (Bootloader_ELFPH64 *)gPHTBuffer;

    /* Note segment is always first */

    uint32_t noteSegmentSz = elfPhdrPtr32[0].filesz;
    uint32_t noteSegmentOffset = elfPhdrPtr32[0].offset;

    if(elfClass == ELFCLASS_64)
    {
        noteSegmentSz = elfPhdrPtr64[0].filesz;
        noteSegmentOffset = elfPhdrPtr64[0].offset;
    }

    config->fxns->imgSeekFxn(noteSegmentOffset, config->args);
    status = config->fxns->imgReadFxn((void *)(gNoteSegBuffer), noteSegmentSz, config->args);

    if(status == SystemP_SUCCESS)
    {
        status = Bootloader_parseNoteSegment(handle, noteSegmentSz, &segmentMapIdx);
    }

    if(status == SystemP_SUCCESS && (doAuth == TRUE))
    {
        status = auth_update((uintptr_t)gNoteSegBuffer, noteSegmentSz);
    }

    int32_t i;
	uint8_t initCpuDone[4] = {0};

    if(elfClass == ELFCLASS_32)
    {
        for(i = 1; i < elfPtr32->e_phnum; i++)
        {
            if(elfPhdrPtr32[i].filesz != 0)
            {
                if(elfPhdrPtr32[i].type == PT_LOAD)
                {
					uint8_t cpuId = gNoteSegBuffer[segmentMapIdx + i - 1];
					if(!initCpuDone[cpuId])
					{
						status = Bootloader_initCpu(handle, &bootImageInfo->cpuInfo[cpuId]);
						config->coresPresentMap |= 1 << cpuId;
						Bootloader_profileAddCore(cpuId);
						initCpuDone[cpuId] = 1;
					}
                    config->fxns->imgSeekFxn(elfPhdrPtr32[i].offset, config->args);
                    uint32_t addr = Bootloader_socTranslateSectionAddr(gNoteSegBuffer[segmentMapIdx + i - 1], elfPhdrPtr32[i].vaddr);
                    config->fxns->imgReadFxn((void *)addr, elfPhdrPtr32[i].filesz, config->args);
					config->bootImageSize += elfPhdrPtr32[i].filesz;
                    if(doAuth == TRUE)
                    {
                        status = auth_update((uintptr_t)addr, elfPhdrPtr32[i].filesz);
                    }
                }
                else
                {
                    /* Ignore this segment */
                }
            }
            else
            {
                /* NO LOAD segment, do nothing */
            }
        }
    }
    else
    {
        for(i = 1; i < elfPtr64->e_phnum; i++)
        {
            if(elfPhdrPtr64[i].filesz != 0)
            {
                if(elfPhdrPtr64[i].type == PT_LOAD)
                {
                    config->fxns->imgSeekFxn(elfPhdrPtr64[i].offset, config->args);
                    uint32_t addr = Bootloader_socTranslateSectionAddr(gNoteSegBuffer[segmentMapIdx + i - 1], elfPhdrPtr64[i].vaddr);
                    config->fxns->imgReadFxn((void *)addr, elfPhdrPtr64[i].filesz, config->args);
                    if(doAuth == TRUE)
                    {
                        status = auth_update((uintptr_t)addr, elfPhdrPtr64[i].filesz);
                    }
                }
                else
                {
                    /* Ignore this segment */
                }
            }
            else
            {
                /* NO LOAD segment, do nothing */
            }
        }
    }

    if(status == SystemP_SUCCESS && (doAuth == TRUE))
    {
        status = auth_finish();
    }

    return status;
}

int32_t Bootloader_parseMultiCoreAppImage(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_Config *config = (Bootloader_Config *)handle;

    if(config->fxns->imgReadFxn == NULL || config->fxns->imgSeekFxn == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        Bootloader_MetaHeaderStart mHdrStr;
        Bootloader_MetaHeaderCore  mHdrCore[BOOTLOADER_MAX_INPUT_FILES];

        /* Verify the multicore image if authentication is required */
        if((Bootloader_socIsAuthRequired() == TRUE) && (config->isAppimageSigned == TRUE))
        {
            /* Device is HS, verify image. */
            status = Bootloader_verifyMulticoreImage(handle);
        }
        else
        {
            /* Device is GP, no authentication required OR appimage is not signed, boot like GP */
            status = SystemP_SUCCESS;
        }

        if(SystemP_SUCCESS == status)
        {
            /* seek to 0 */
            config->fxns->imgSeekFxn(0, config->args);

            memset(&mHdrCore[0], 0xFF, BOOTLOADER_MAX_INPUT_FILES*sizeof(Bootloader_MetaHeaderCore));

            status = config->fxns->imgReadFxn(&mHdrStr, sizeof(Bootloader_MetaHeaderStart), config->args);

            if(mHdrStr.magicStr != BOOTLOADER_META_HDR_MAGIC_STR)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* TODO */
                /* Check for device Id later if needed, just a warning */

                /* Read all the core offset addresses */
                uint32_t i;

                for(i=0U; i<mHdrStr.numFiles; i++)
                {
                    status = config->fxns->imgReadFxn(&mHdrCore[i], sizeof(Bootloader_MetaHeaderCore), config->args);
                    /* TODO: Figure out how to add boot media specific offset */
                }

                /* Parse individual rprc files */
                for(i=0U; i<mHdrStr.numFiles; i++)
                {
                    if(mHdrCore[i].coreId != (0xFFFFFFFFU))
                    {
                        uint32_t cslCoreId = Bootloader_socRprcToCslCoreId(mHdrCore[i].coreId);
                        Bootloader_CpuInfo *cpuInfo = &bootImageInfo->cpuInfo[cslCoreId];
                        cpuInfo->rprcOffset = mHdrCore[i].imageOffset;
                        cpuInfo->entryPoint = 0;
                        cpuInfo->cpuId      = cslCoreId;
                        config->coresPresentMap |= (1 << cslCoreId);
                    }
                }
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

void Bootloader_Params_init(Bootloader_Params *params)
{
    params->memArgsAppImageBaseAddr = BOOTLOADER_INVALID_ID;
}

void Bootloader_CpuInfo_init(Bootloader_CpuInfo *cpuInfo)
{
    cpuInfo->cpuId = CSL_CORE_ID_MAX;
    cpuInfo->clkHz = 0U;
    cpuInfo->rprcOffset = BOOTLOADER_INVALID_ID;
    cpuInfo->entryPoint = BOOTLOADER_INVALID_ID;
}

void Bootloader_BootImageInfo_init(Bootloader_BootImageInfo *bootImageInfo)
{
    uint16_t i;

    for(i=0; i<CSL_CORE_ID_MAX; i++)
    {
        Bootloader_CpuInfo_init(&bootImageInfo->cpuInfo[i]);
        bootImageInfo->cpuInfo[i].cpuId = i;
        bootImageInfo->cpuInfo[i].clkHz = Bootloader_socCpuGetClkDefault(i);
    }
}

#if defined (SOC_AM64X)
/* Linux image load is applicable only for am64x. */
int32_t Bootloader_parseAndLoadLinuxAppImage(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_Config *config = (Bootloader_Config *)handle;

    if(config->fxns->imgReadFxn == NULL || config->fxns->imgSeekFxn == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        Bootloader_MetaHeaderStart mHdrStr;
        Bootloader_MetaHeaderCore  mHdrCore[BOOTLOADER_MAX_INPUT_FILES];

        /* Verify the multicore image if authentication is required */
        if(Bootloader_socIsAuthRequired() == TRUE)
        {
            /* Device is HS, verify image. */
            status = Bootloader_verifyMulticoreImage(handle);
        }
        else
        {
            /* Device is GP, no authentication required */
            status = SystemP_SUCCESS;
        }

        if(SystemP_SUCCESS == status)
        {

            memset(&mHdrCore[0], 0xFF, BOOTLOADER_MAX_INPUT_FILES*sizeof(Bootloader_MetaHeaderCore));

            status = config->fxns->imgReadFxn(&mHdrStr, sizeof(Bootloader_MetaHeaderStart), config->args);

            if(mHdrStr.magicStr != BOOTLOADER_META_HDR_MAGIC_STR)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* TODO */
                /* Check for device Id later if needed, just a warning */

                /* Read all the core offset addresses */
                uint32_t i;

                for(i=0U; i<mHdrStr.numFiles; i++)
                {
                    status = config->fxns->imgReadFxn(&mHdrCore[i], sizeof(Bootloader_MetaHeaderCore), config->args);
                    /* TODO: Figure out how to add boot media specific offset */
                }

                /* Parse individual rprc files */
                for(i=0U; i<mHdrStr.numFiles; i++)
                {
                    /* Load the load only linux images */
                    if(mHdrCore[i].coreId == RPRC_LINUX_LOAD_ONLY_IMAGE_ID)
                    {
                        Bootloader_CpuInfo load_only_image;
                        load_only_image.rprcOffset = mHdrCore[i].imageOffset;
                        load_only_image.entryPoint = 0;

                        /* Set CPU ID as A53 as linux runs on A53 */
                        load_only_image.cpuId = CSL_CORE_ID_A53SS0_0;

                        /**************/
                        if (status == SystemP_SUCCESS)
                        {
                            status = Bootloader_rprcImageLoad(handle, &load_only_image);
                        }

                        continue;
                    }

                    if(mHdrCore[i].coreId != (0xFFFFFFFFU))
                    {
                        uint32_t cslCoreId = Bootloader_socRprcToCslCoreId(mHdrCore[i].coreId);
                        Bootloader_CpuInfo *cpuInfo = &bootImageInfo->cpuInfo[cslCoreId];
                        cpuInfo->rprcOffset = mHdrCore[i].imageOffset;
                        cpuInfo->entryPoint = 0;
                        cpuInfo->cpuId      = cslCoreId;
                        config->coresPresentMap |= (1 << cslCoreId);
                    }
                }
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

int32_t Bootloader_runSelfCpuWithLinux(void)
{
    int32_t status = SystemP_SUCCESS;

    DebugP_logInfo("All done, reseting self ...\r\n\n");
    status = Bootloader_socCpuResetReleaseSelf();
    /* control will not reach here */

    return status;
}

#endif
