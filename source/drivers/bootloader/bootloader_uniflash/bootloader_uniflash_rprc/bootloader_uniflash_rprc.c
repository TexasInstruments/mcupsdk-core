#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash_common.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <string.h>
#include <board/flash.h>

static int32_t Bootloader_uniflashFlashOrVerifyXipFile(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize);

static int32_t Bootloader_uniflashFlashOrVerifyRprcXipFile(uint32_t flashIndex, uint8_t *buf, uint8_t *verifyBuf, uint32_t verifyBufSize)
{
    Bootloader_RprcFileHeader     header;
    Bootloader_RprcSectionHeader section;
    int32_t status = SystemP_SUCCESS;

    memcpy(&header, buf, sizeof(Bootloader_RprcFileHeader));
    buf += sizeof(Bootloader_RprcFileHeader);
    if(header.magic != BOOTLOADER_RPRC_MAGIC_NUMBER)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        uint32_t i;

        for(i=0; i<header.sectionCount; i++)
        {
            memcpy(&section, buf, sizeof(Bootloader_RprcSectionHeader));
            buf+= sizeof(Bootloader_RprcSectionHeader);

            if((verifyBuf != NULL) && (verifyBufSize != 0))
            {
            	/* verify the section */
                status = Bootloader_uniflashFlashVerifyFile(flashIndex, buf, section.size, verifyBuf, verifyBufSize, section.addr);
            }
            else
            {
                /* flash the section */
                status = Bootloader_uniflashFlashFile(flashIndex, buf, section.size, section.addr);
            }
            buf += section.size;

            if(status != SystemP_SUCCESS)
                break;
        }
    }
    return status;
}

static int32_t Bootloader_uniflashFlashOrVerifyXipFile(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize)
{
    int32_t status = SystemP_SUCCESS;
    Bootloader_MetaHeaderStart mHdrStr;
    Bootloader_MetaHeaderCore  mHdrCore[BOOTLOADER_MAX_INPUT_FILES];
    uint8_t *ptr = buf;

    memset(&mHdrCore[0], 0xFF, BOOTLOADER_MAX_INPUT_FILES*sizeof(Bootloader_MetaHeaderCore));
    memcpy(&mHdrStr, ptr, sizeof(Bootloader_MetaHeaderStart));
    ptr += sizeof(Bootloader_MetaHeaderStart);

    if(mHdrStr.magicStr != BOOTLOADER_META_HDR_MAGIC_STR)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Read all the core offset addresses */
        uint32_t i;

        for(i=0U; i<mHdrStr.numFiles; i++)
        {
            memcpy(&mHdrCore[i], ptr, sizeof(Bootloader_MetaHeaderCore));
            ptr += sizeof(Bootloader_MetaHeaderCore);
        }

        /* Parse individual rprc files */
        for(i=0U; i<mHdrStr.numFiles; i++)
        {
            if(mHdrCore[i].coreId != (0xFFFFFFFFU))
            {
                /* flash or verify the file */
                status = Bootloader_uniflashFlashOrVerifyRprcXipFile(flashIndex, buf + mHdrCore[i].imageOffset, verifyBuf, verifyBufSize);
                if(status != SystemP_SUCCESS)
                    break;
            }
        }
    }
    return status;
}

int32_t Bootloader_Uniflash_RPRC_flashXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize)
{
	int32_t status = SystemP_FAILURE;
	status = Bootloader_uniflashFlashOrVerifyXipFile(flashIndex, fileBuf, fileSize, NULL, 0);
	return status;
}

int32_t Bootloader_Uniflash_RPRC_flashVerifyXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize)
{
	int32_t status = SystemP_FAILURE;
	status = Bootloader_uniflashFlashOrVerifyXipFile(flashIndex, fileBuf, fileSize, verifyBuf, verifyBufSize);
	return status;
}
