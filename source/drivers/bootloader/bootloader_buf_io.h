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

#ifndef BOOTLOADER_BUF_IO_H
#define BOOTLOADER_BUF_IO_H

/* Buffered IO protocol commands */
#define BOOTLOADER_BUF_IO_MAGIC                      (0xBF0000BFU)
#define BOOTLOADER_BUF_IO_OK                         (0xBF000000U)
#define BOOTLOADER_BUF_IO_ERR                        (0xBF000001U)
#define BOOTLOADER_BUF_IO_FILE_RECEIVE_COMPLETE      (0xBF000002U)
#define BOOTLOADER_BUF_IO_SEND_FILE                  (0xBF000003U)

/**
 * \brief Arguments to be passed to driver implementation callbacks when boot media is SOC memory
 */
typedef struct Bootloader_BufIoArgs_s
{
    uint32_t appImageOffset;
    uint32_t curOffset;
    uint32_t tempBufSize;
    uint32_t virtMemOffset;
    uint32_t drvIdx;
    uint8_t  *tempBuf;

} Bootloader_BufIoArgs;

/* Buffered IO protocol request header */
typedef struct Bootloader_BufIoProtocolReq_s
{
    uint32_t magic;
    uint32_t virtMemOffset;
    uint32_t cmd;
    uint32_t len;

} Bootloader_BufIoProtocolReq;

extern Bootloader_Fxns gBootloaderBufIoFxns;

/**
 * \brief Function to explicitly send transfer complete message to host. This can't be
 *        called in the close function because that part of the code won't be executed
 *        if there is a self core boot
 *
 * \param ioDrvIdx [in] Driver index of the uart driver
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t BufIo_sendTransferComplete(uint32_t ioDrvIdx);

#endif  /* BOOTLOADER_BUF_IO_H */