/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <drivers/flsopskd.h>
#include <drivers/ospi.h>
#include <kernel/dpl/CacheP.h>

#define FOTA_FW_MEM_SZ (2048U)
#define WBUF_SIZE (512U)
#define FOTA_STIG_SET_OPCODE (0x0002U)
#define FOTA_STIG_SET_EXOPCODE (0x0102U)
#define FOTA_STIG_SET_RDBYTECNT (0x0302U)
#define FOTA_STIG_SET_WRBYTECNT (0x0402U)
#define FOTA_STIG_SET_ADDRBYTECNT (0x0502U)
#define FOTA_STIG_SET_DUMMYCYCLES (0x0602U)
#define FOTA_STIG_SET_TXBUFF_LOW (0x0702U)
#define FOTA_STIG_SET_TXBUFF_HIGH (0x0802U)
#define FOTA_STIG_GET_RXBUFF_LOW (0x0902U)
#define FOTA_STIG_GET_RXBUFF_HIGH (0x0A02U)

#define FOTA_IS25LX064_READ_OPCODE (0x7CU)
#define FOTA_IS25LX064_READ_EXOPCODE (0x7CU)
#define FOTA_IS25LX064_READ_DUMMYCYCLE (16U)
#define FOTA_IS25LX064_ADDRESS_SIZE (4U)

#define ERASE_SECTOR_SIZE (4096U)

static void FLSOPSKD_OperationCompletionIntr(void)
{
    HwiP_clearInt(CSLR_R5FSS0_CORE1_FOTA_WR_COMPL);
}

int32_t FLSOPSKD_Init(FLSOPSKD_handle *pHandle)
{
    int32_t status = SystemP_FAILURE;
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);

    if (NULL != pHandle)
    {
        pHandle->pollEnable = TRUE;

        {
            /* Put FOTA in reset */

            CSL_REG32_FINS(&pReg->FOTA_INIT, FSS_FOTA_GENREGS_FOTA_INIT_RESET, 1);
            /* Make memaccess = 1 */
            CSL_REG32_FINS(&pReg->FOTA_INIT, FSS_FOTA_GENREGS_FOTA_INIT_MEMACCESS, 1);

            /* Poll for mem init done */
            uint32_t mem_init = CSL_REG32_RD(&pReg->FOTA_INIT);
            while ((mem_init & CSL_FSS_FOTA_GENREGS_FOTA_INIT_PDMEM_INIT_DONE_MASK) != CSL_FSS_FOTA_GENREGS_FOTA_INIT_PDMEM_INIT_DONE_MASK)
            {
                mem_init = CSL_REG32_RD(&pReg->FOTA_INIT);
            }
        }

        /* load 8051 FW && verify */
        {
            status = SystemP_SUCCESS;
            memcpy((void *)CSL_FSS_PDMEM_GENREGS_REGS_BASE, FOTA_FW_ARR, FOTA_FW_SIZE);
            CacheP_wbInv((void *)CSL_FSS_PDMEM_GENREGS_REGS_BASE, FOTA_FW_SIZE, CacheP_TYPE_ALL);
            for (uint32_t i = 0; i < FOTA_FW_SIZE && SystemP_SUCCESS == status; i++)
            {
                uint8_t data = *(uint8_t *)(CSL_FSS_PDMEM_GENREGS_REGS_BASE + i);
                if(data != FOTA_FW_ARR[i])
                {
                    status = SystemP_FAILURE;
                }
                else 
                {
                    status = SystemP_SUCCESS;
                }
            }
        }

        /* lock back the HW */
        if(SystemP_SUCCESS == status)
        {
            /* Make memaccess = 0 such that CPU can't access FOTA memories now */
            CSL_REG32_FINS(&pReg->FOTA_INIT, FSS_FOTA_GENREGS_FOTA_INIT_MEMACCESS, 0);
            /* LIFT FOTA FROM RESET */
            CSL_REG32_FINS(&pReg->FOTA_INIT, FSS_FOTA_GENREGS_FOTA_INIT_RESET, 0);
        }

        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t FLSOPSKD_Erase(FLSOPSKD_handle *pHandle, uint32_t offset)
{
    /* any non zero value will make 8051 to erase */
    int32_t status = SystemP_FAILURE;
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);

    if (NULL != pHandle)
    {
        CSL_REG32_WR(&pReg->FOTA_GP0, 1);
        CSL_REG32_WR(&pReg->FOTA_ADDR, offset);
        CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
        CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
        if (pHandle->pollEnable == TRUE)
        {
            while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
                ;
        }
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t FLSOPSKD_Write(FLSOPSKD_handle *pHandle, uint32_t destAddr, uint8_t *pSrcBuffer, uint32_t wrSize)
{
    unsigned int flash_page_size = FLASH_PAGE_SIZE;
    unsigned int sector_size = FLASH_SECTOR_SIZE;
    int32_t status = SystemP_FAILURE;
    uint32_t bytesWritten = 0;

    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);

    /* get page size and due to HW limitation, it should not be more than 512 bytes. */
    if ((NULL != pHandle) && (wrSize % (sector_size) == 0) && (destAddr % (sector_size) == 0))
    {
        do
        {

            CSL_REG32_WR(&pReg->FOTA_GP0, 0);
            CSL_REG32_WR(&pReg->FOTA_GP1, 0);
            /* FOTA ADDR */
            CSL_REG32_WR(&pReg->FOTA_ADDR, destAddr + bytesWritten);
            /* FOTA size */
            CSL_REG32_WR(&pReg->FOTA_CNT, flash_page_size);
            memcpy((void *)CSL_FSS_WBUF_GENREGS_REGS_BASE, pSrcBuffer + bytesWritten, flash_page_size);
            memcpy((void *)(CSL_FSS_WBUF_GENREGS_REGS_BASE + flash_page_size), pSrcBuffer + bytesWritten, flash_page_size);
            CacheP_wbInv((void *)CSL_FSS_WBUF_GENREGS_REGS_BASE, flash_page_size * 2, CacheP_TYPE_ALL);
            CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
            /* trigger the write */
            CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
            if (pHandle->pollEnable == TRUE)
            {
                while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
                    ;
            }
            bytesWritten += flash_page_size;
        } while (bytesWritten <= wrSize);
        status = SystemP_SUCCESS;
    }

    return status;
}

static inline void FLSOPSKD_STIGSetGeneric(uint16_t fncode, uint16_t val)
{
    uint32_t nval = (val << 16) | fncode;
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);

    CSL_REG32_WR(&pReg->FOTA_GP0, nval);
    CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
    CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
    while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
        ;
}

static void FLSOPSKD_STIGSetOpcode(uint8_t opcode)
{
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_OPCODE, opcode);
}

static void FLSOPSKD_STIGSetExopcode(uint8_t exopcode)
{
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_EXOPCODE, exopcode);
}

static void FLSOPSKD_STIGSetReadBytesCount(uint8_t nReadBytes)
{
    if (nReadBytes > 0)
    {
        nReadBytes = ((nReadBytes - 1) + 0x8) & 0xf;
    }
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_RDBYTECNT, nReadBytes);
}

static void FLSOPSKD_STIGSetWriteBytesCount(uint8_t nWriteBytes)
{
    if (nWriteBytes > 0)
    {
        nWriteBytes = ((nWriteBytes - 1) + 0x8) & 0xf;
    }
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_WRBYTECNT, nWriteBytes);
}

static void FLSOPSKD_STIGSetAddressBytesCount(uint8_t nAddressBytes)
{
    if (nAddressBytes > 0)
    {
        nAddressBytes = (0x8 + ((nAddressBytes - 1) & 0x3)) & 0xf;
    }
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_ADDRBYTECNT, nAddressBytes);
}

static void FLSOPSKD_STIGSetDummyCycles(uint8_t nDummyCycle)
{
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_DUMMYCYCLES, nDummyCycle);
}

static void FLSOPSKD_STIGSetWrite(uint64_t wrData)
{
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);
    CSL_REG32_WR(&pReg->FOTA_ADDR, wrData & 0xffffffff);
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_TXBUFF_LOW, 0);
    CSL_REG32_WR(&pReg->FOTA_ADDR, (wrData >> 32) & 0xffffffff);
    FLSOPSKD_STIGSetGeneric(FOTA_STIG_SET_TXBUFF_HIGH, 0);
}

static uint64_t FLSOPSKD_STIG_get_readregs()
{
    uint64_t res = 0;
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);
    CSL_REG32_WR(&pReg->FOTA_GP0, FOTA_STIG_GET_RXBUFF_HIGH);
    CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
    CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
    while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
        ;

    res = HWREG(CSL_FSS_FOTA_GENREGS_REGS_BASE + CSL_FSS_FOTA_GENREGS_FOTA_GP1);

    CSL_REG32_WR(&pReg->FOTA_GP0, FOTA_STIG_GET_RXBUFF_LOW);
    CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
    CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
    while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
        ;
    res <<= 32;
    res |= CSL_REG32_RD(&pReg->FOTA_GP1);
    return res;
}

int32_t FLSOPSKD_STIG(
    FLSOPSKD_handle *pHandle,
    uint8_t opCode, uint8_t exOpCode, uint8_t dCyc,
    size_t addresSize, uint32_t address,
    size_t readSize, uint8_t *pReadBuffer,
    size_t writeSize, uint8_t *pWriteBuffer)
{
    int status = SystemP_FAILURE;
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);
    uint64_t wrData = 0;
    uint64_t rdData = 0;

    if (NULL != pHandle)
    {
        FLSOPSKD_STIGSetOpcode(opCode);
        FLSOPSKD_STIGSetExopcode(exOpCode);
        FLSOPSKD_STIGSetAddressBytesCount(addresSize);
        FLSOPSKD_STIGSetWriteBytesCount(writeSize);
        FLSOPSKD_STIGSetReadBytesCount(readSize);
        FLSOPSKD_STIGSetDummyCycles(dCyc);

        for (int i = 0; i < writeSize && i < 8; i++)
            *(((uint8_t *)(&wrData)) + i) = pWriteBuffer[i];
        FLSOPSKD_STIGSetWrite(wrData);

        CSL_REG32_WR(&pReg->FOTA_ADDR, address);
        CSL_REG32_WR(&pReg->FOTA_GP0, 0x3);
        CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
        CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
        if (pHandle->pollEnable == TRUE)
        {
            while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
                ;
        }
        rdData = FLSOPSKD_STIG_get_readregs();
        for (int i = 0; i < readSize && i < 8; i++)
        {
            pReadBuffer[i] = *(((uint8_t *)(&rdData)) + i);
        }

        status = SystemP_SUCCESS;
    }

    return (status);
}

int32_t FLSOPSKD_STIGRead(FLSOPSKD_handle *pHandle, uint32_t srcAddress, uint8_t *destArry, size_t bytesToRead)
{
    const uint8_t readChunkSize = 8;
    int status = SystemP_FAILURE;

    if ((NULL != pHandle) && (NULL != destArry))
    {
        size_t bytesRead = 0;
        do
        {
            FLSOPSKD_STIG(
                pHandle,
                FOTA_IS25LX064_READ_OPCODE,
                FOTA_IS25LX064_READ_EXOPCODE,
                FOTA_IS25LX064_READ_DUMMYCYCLE,
                FOTA_IS25LX064_ADDRESS_SIZE,
                srcAddress + bytesRead,
                readChunkSize,
                destArry + bytesRead, 0, NULL);
            bytesRead += readChunkSize;
        } while (bytesRead < bytesToRead);
    }

    return (status);
}

int32_t FLSOPSKD_Get8051Version(FLSOPSKD_handle *pHandle, volatile uint32_t *pVersion)
{
    int32_t status = SystemP_FAILURE;
    const CSL_fss_fota_genregsRegs *pReg = (const CSL_fss_fota_genregsRegs *)(CSL_FSS_FOTA_GENREGS_REGS_BASE);

    if ((NULL != pHandle) && (NULL != pVersion))
    {
        CSL_REG32_WR(&pReg->FOTA_GP0, 4);
        CSL_REG32_WR(&pReg->STS_IRQ.STATUS, 1);
        CSL_REG32_FINS(&pReg->FOTA_CTRL, FSS_FOTA_GENREGS_FOTA_CTRL_GO, 1);
        if (pHandle->pollEnable == TRUE)
        {
            while (CSL_REG32_RD(&pReg->STS_IRQ.STATUS_RAW) == 0)
                ;
        }
        *pVersion = pReg->FOTA_GP1;
        status = SystemP_SUCCESS;
    }
    return status;
}

int32_t FLSOPSKD_GetSectorSize(FLSOPSKD_handle *pHandle, volatile uint32_t *pSecSize)
{
    int32_t status = SystemP_FAILURE;

    if ((NULL != pHandle) && (NULL != pSecSize))
    {
        *pSecSize = ERASE_SECTOR_SIZE;
        status = SystemP_SUCCESS;
        (void)pHandle;
    }
    return status;
}