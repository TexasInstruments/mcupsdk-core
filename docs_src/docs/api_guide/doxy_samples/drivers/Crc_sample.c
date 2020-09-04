
#include <stdio.h>
//! [include]
#include <drivers/crc.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>

uint32_t        gCrcBaseAddr = 0;//Dummy address for compilation

void crc_init(void)
{
//! [init]
    int32_t         status;
    CRC_Channel_t   crcChNumber = CRC_CHANNEL_1;

    status = CRC_channelReset(gCrcBaseAddr, crcChNumber);
    DebugP_assert(status == SystemP_SUCCESS);

    status = CRC_initialize(gCrcBaseAddr, crcChNumber, 0, 0);
    DebugP_assert(status == SystemP_SUCCESS);
//! [init]
}

#if defined (DRV_VERSION_CRC_V0)
void crc_config(void)
{
//! [config]
    int32_t         status;
    uint32_t        patternCnt, sectCnt, mode;
    CRC_Channel_t    crcChNumber = CRC_CHANNEL_1;

    patternCnt  = 100;      /* Calculate CRC for 100 words */
    sectCnt     = 1;
    mode        = CRC_OPERATION_MODE_FULLCPU;   /* CPU mode */
    status = CRC_configure(gCrcBaseAddr, crcChNumber, patternCnt, sectCnt, mode);
    DebugP_assert(status == SystemP_SUCCESS);
//! [config]
}

void crc_operation(void)
{
//! [operation]
    int32_t                 status;
    CRC_SignatureRegAddr   psaSignRegAddr;
    CRC_Signature         sectSignVal;
    uint32_t                patternCnt;
    uint32_t                loopCnt, data = 0xAA55AA55;
    CRC_Channel_t           crcChNumber = CRC_CHANNEL_1;

    /* Get CRC PSA signature register address */
    status = CRC_getPSASigRegAddr(gCrcBaseAddr, crcChNumber, &psaSignRegAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Perform CRC operation - write data to PSA signature register */
    patternCnt  = 100;      /* Calculate CRC for 100 words */
    for(loopCnt = 0; loopCnt < patternCnt; loopCnt++)
    {
        HW_WR_REG32(psaSignRegAddr.regL, data);     /* 32-bit mode */
    }

    /* Get the calculated CRC signature value */
    status = CRC_getPSASectorSig(gCrcBaseAddr, crcChNumber, &sectSignVal);
    DebugP_assert(status == SystemP_SUCCESS);
//! [operation]
}
#endif

#if defined (DRV_VERSION_CRC_V1)
void crc_config(void)
{
//! [config_v1]
    int32_t         status;
    CRC_Config     config;
    CRC_Channel_t    crcChNumber = CRC_CHANNEL_1;

    config.mode         = CRC_OPERATION_MODE_FULLCPU;
    config.type         = CRC_TYPE_16BIT;
    config.dataLen      = CRC_DATALENGTH_16BIT;
    config.bitSwap      = CRC_BITSWAP_MSB;
    config.byteSwap     = CRC_BYTESWAP_ENABLE;
    config.patternCount = 100;
    config.sectorCount  = 1;
    status = CRC_configure(gCrcBaseAddr, crcChNumber, &config);
    DebugP_assert(status == SystemP_SUCCESS);
//! [config_v1]
}

void crc_operation(void)
{
//! [operation_v1]
    int32_t                 status;
    CRC_SignatureRegAddr   psaSignRegAddr;
    CRC_Signature           sectSignVal;
    uint32_t                patternCnt;
    uint32_t                loopCnt, data = 0xAA55AA55;
    CRC_Channel_t            crcChNumber = CRC_CHANNEL_1;

    /* Get CRC PSA signature register address */
    status = CRC_getPSASigRegAddr(gCrcBaseAddr, crcChNumber, &psaSignRegAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Perform CRC operation - write data to PSA signature register */
    patternCnt  = 100;      /* Calculate CRC for 100 words */
    for(loopCnt = 0; loopCnt < patternCnt; loopCnt++)
    {
        HW_WR_REG32(psaSignRegAddr.regL, data);     /* 32-bit mode */
    }

    /* Get the calculated CRC signature value */
    status = CRC_getPSASectorSig(gCrcBaseAddr, crcChNumber, &sectSignVal);
    DebugP_assert(status == SystemP_SUCCESS);
//! [operation_v1]
}
#endif
