/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef FLASH_NAND_GPMC_H_
#define FLASH_NAND_GPMC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/gpmc.h>

typedef struct
{
    uint32_t bbOffset;
    uint32_t eccAlgo;
    uint32_t eccOffset;
    uint32_t eccByteCount;
    uint32_t eccSteps;
} Flash_NandGpmc_Attrs;

typedef struct {

    GPMC_Handle gpmcHandle;
    Flash_NandGpmc_Attrs attrs;
    uint8_t  *dataMemScratch;
    uint8_t  *eccMemScratch;
} Flash_NandGpmcObject;

/* Flash Device specific extern */
extern Flash_DevConfig gFlashNandGpmcDeviceDefines_MT29F8G08ADAFAH4;
extern Flash_Attrs gFlashNandGpmcAttrs_MT29F8G08ADAFAH4;

/* Flash specific externs */
extern Flash_Fxns gFlashNandGpmcFxns;


/* Read status command definitions */
#define NAND_READ_STATUS_FAIL_MASK    (0x01) /* fail mask */
#define NAND_READ_STATUS_DEVRDY_MASK  (0x20) /* device ready mask */
#define NAND_READ_STATUS_WRPROT_MASK  (0x80) /* write protect mask */

/* Bad block definitions */
#define NAND_BAD_BLK_OFFSET           (0x0U) /* offset address of bad blk mark in spare area*/
#define NAND_BLK_GOOD_MARK            (0xFFU)
#define NAND_BLK_BAD_MARK             (0x0U)

/** Macro to specify the number of bytes per transaction. */
#define NAND_SECTOR_SIZE_BYTES                        (512U)

#define NAND_BAD_BLOCK_MARKER_LENGTH                  (2U)

/** Time out value in Micro seconds */
#define NAND_DEVICE_RESET_TIMEOUT                     (10 * 1000U) /* 2ms*/

#define NAND_DEVICE_BUSY_TIMEOUT                      (10 * 1000U) /* 2ms*/


/* Macro defining the Hamming Code ECC Offset value. */
#define NAND_ECC_1BIT_HAMMINGCODE_OOB_OFFSET          (1U)

/* Macro defining the Hamming Code ECC Byte Count value. */
#define NAND_ECC_1BIT_HAMMINGCODE_BYTECNT             (3U)

/* Macro defining the 4-bit Reed-Solomon ECC Offset value. */
#define NAND_ECC_RS_4BIT_OOB_OFFSET                   (0U)

/* Macro defining the 4-bit Reed-Solomon ECC Byte Count value. */
#define NAND_ECC_RS_4BIT_UNUSED_BYTECNT               (6U)

/* Macro defining the 4-bit Reed-Solomon ECC Byte Count value. */
#define NAND_ECC_RS_4BIT_BYTECNT                      (10U)

/* Macro defining the 8-bit BCH ECC Offset value. */
#define NAND_ECC_BCH_8BIT_OOB_OFFSET                  (2U)

/* Macro defining the 8-bit BCH ECC Byte Count value. */
#define NAND_ECC_BCH_8BIT_BYTECNT                     (14U)

/* Macro defining the 8-bit BCH ECC Unused Byte Count value. */
#define NAND_ECC_BCH_8BIT_UNUSED_BYTECNT              (2U)

/* Macro defining the 8-bit BCH ECC Nibble Count value. */
#define NAND_ECC_BCH_8BIT_NIBBLECNT                   (26U)

/* Macro defining the 8-bit BCH ECC unused Nibble Count value. */
#define NAND_ECC_BCH_8BIT_UNUSED_NIBBLECNT            (2U)

/* Macro defining the Last Data bit for 8-bit BCH ECC scheme. */
#define NAND_ECC_BCH_8BIT_LASTDATABIT                 ((512 + 13) * 8)

/* Macro defining the Last ECC bit for 8-bit BCH ECC scheme. */
#define NAND_ECC_BCH_8BIT_LASTECCBIT                  ((13 + 1) * 8)

/* Macro defining the Number of ECC bytes per Transfer. */
#define NAND_MAX_ECC_BYTES_PER_TRNFS                  (16)

/* Maximum ECC words per sector. */
#define NAND_MAX_ECC_WORDS_PER_TRANFS                 (4)

/* Maximum number of error bits locations per sector. */
#define NAND_ERROR_BIT_PER_SECTOR_MAX                 (16)



#ifdef __cplusplus
}
#endif


#endif /* FLASH_NAND_XSPI_H_ */