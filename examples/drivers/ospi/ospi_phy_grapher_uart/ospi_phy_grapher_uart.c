/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ospi_phy_grapher_uart.h"

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

/**
 * @brief Static buffer for OSPI PHY tuning data
 *
 * This buffer holds the complete PHY tuning sweep results:
 * - 5 rdDelay values
 * - 128 txDLL values (per rdDelay)
 * - 128 rxDLL values (per txDLL)
 * - Total: 5 × 128 × 128 = 81,920 bytes
 */
static uint8_t gOspiPhyGrapherData[5][128][128];

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

int32_t OSPI_phyGrapherUartSend(OSPI_Handle ospiHandle,
                                UART_Handle uartHandle,
                                uint32_t flashOffset);

/* ========================================================================== */
/*                      Internal Implementation Details                        */
/* ========================================================================== */

/**
 * @brief CRC16-CCITT lookup table (polynomial 0x1021)
 *
 * This table is used for fast CRC16 calculation in XMODEM1k protocol.
 * Generated using polynomial 0x1021 with initial value 0x0000.
 */
static const uint16_t gOspiPhyGrapherCrc16Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x5295, 0x42B4, 0x7297, 0x62B6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD39D, 0xC3BC, 0xF39F, 0xE3BE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76F7, 0x66D6, 0x56B5, 0x4694,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD7DD, 0xC7FC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/**
 * @brief Calculate CRC16-CCITT for a data buffer
 *
 * Uses precomputed lookup table for fast calculation.
 *
 * @param data  Pointer to data buffer
 * @param len   Length of data in bytes
 *
 * @return CRC16 value
 */
static uint16_t OSPI_phyGrapherUartCalcCrc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        crc = ((crc << 8) ^ gOspiPhyGrapherCrc16Table[(crc >> 8) ^ data[i]]) & 0xFFFF;
    }

    return crc;
}

/**
 * @brief Read a single byte from UART with timeout
 *
 * @param uartHandle    UART driver handle
 * @param byte          Pointer to byte buffer
 * @param timeoutMs     Timeout in milliseconds
 *
 * @return 0 if byte read successfully, -1 on timeout or error
 */
static int32_t OSPI_phyGrapherUartReadByte(UART_Handle uartHandle,
                                           uint8_t *byte,
                                           uint32_t timeoutMs)
{
    UART_Transaction trans;
    int32_t status;

    UART_Transaction_init(&trans);
    trans.buf = (uint8_t *)byte;
    trans.count = 1;
    trans.timeout = timeoutMs;

    status = UART_read(uartHandle, &trans);
    return (status == UART_TRANSFER_STATUS_SUCCESS && trans.count == 1) ? 0 : -1;
}

/**
 * @brief Write bytes to UART
 *
 * @param uartHandle    UART driver handle
 * @param data          Pointer to data buffer
 * @param len           Number of bytes to write
 *
 * @return 0 if all bytes written, -1 on error
 */
static int32_t OSPI_phyGrapherUartWriteBytes(UART_Handle uartHandle,
                                             const uint8_t *data,
                                             uint32_t len)
{
    UART_Transaction trans;
    int32_t status;

    UART_Transaction_init(&trans);
    trans.buf = (uint8_t *)data;
    trans.count = len;

    status = UART_write(uartHandle, &trans);
    return (status == UART_TRANSFER_STATUS_SUCCESS && trans.count == len) ? 0 : -1;
}

/* ========================================================================== */
/*                   Phase 2: UART Protocol Functions                         */
/* ========================================================================== */

/**
 * @brief Send "READY_FOR_SWEEP\r\n" signal to host
 *
 * This is sent when the device is ready for the PHY sweep operation.
 * Used for synchronization and debugging visibility.
 *
 * @param uartHandle    UART driver handle
 *
 * @return 0 on success, -1 on error
 */
static int32_t OSPI_phyGrapherUartSendReady(UART_Handle uartHandle)
{
    const char *readyMsg = "READY_FOR_SWEEP\r\n";
    uint32_t len = strlen(readyMsg);

    DebugP_log("[OSPI PHY Grapher UART] Sending ready signal\r\n");
    int32_t ret = OSPI_phyGrapherUartWriteBytes(uartHandle, (const uint8_t*)readyMsg, len);
    /* Wait 500ms for receiver to connect and prepare command */
    ClockP_usleep(500000);
    return ret;
}

/**
 * @brief Receive and validate command header from host
 *
 * Receives 32-byte command header with the following format:
 * - Bytes 0-3: Magic number (0x47524150 = "PRAG")
 * - Bytes 4-7: Operation type (should be 0xFD)
 * - Bytes 8-11: Offset (flash offset, currently unused)
 * - Bytes 12-15: Size (data size, should be 81920)
 * - Bytes 16-31: Reserved (should be 0)
 *
 * @param uartHandle    UART driver handle
 * @param cmdBuf        Output buffer for command (must be 32 bytes)
 *
 * @return OSPI_PHY_GRAPHER_STATUS_SUCCESS if command is valid
 * @return OSPI_PHY_GRAPHER_STATUS_MAGIC_ERROR if magic number invalid
 * @return OSPI_PHY_GRAPHER_STATUS_OPTYPE_ERROR if operation type invalid
 * @return OSPI_PHY_GRAPHER_STATUS_SIZE_ERROR if data size invalid
 * @return OSPI_PHY_GRAPHER_STATUS_XMODEM_ERROR if timeout reading header
 */
static uint32_t OSPI_phyGrapherUartReceiveCommand(UART_Handle uartHandle,
                                                  uint8_t *cmdBuf)
{
    UART_Transaction trans;
    int32_t status;
    uint32_t magic;
    uint32_t optype;
    uint32_t size;

    /* NOTE: Minimal debug output to keep UART clean for subsequent data transfer */

    /* Read 32-byte header in one transaction */
    /* Wait for TX FIFO to flush and receiver to be ready */
    ClockP_usleep(500000);  /* 0.5 second wait */

    /* Read all 32 bytes at once */
    UART_Transaction_init(&trans);
    trans.buf = cmdBuf;
    trans.count = OSPI_PHY_GRAPHER_UART_HEADER_SIZE;
    trans.timeout = OSPI_PHY_GRAPHER_UART_XMODEM_TIMEOUT_MS;

    status = UART_read(uartHandle, &trans);
    if (status != UART_TRANSFER_STATUS_SUCCESS || trans.count != OSPI_PHY_GRAPHER_UART_HEADER_SIZE)
    {
        return OSPI_PHY_GRAPHER_STATUS_XMODEM_ERROR;
    }

    /* Extract and validate magic number (little-endian) */
    magic = cmdBuf[0] | (cmdBuf[1] << 8) | (cmdBuf[2] << 16) | (cmdBuf[3] << 24);
    if (magic != OSPI_PHY_GRAPHER_UART_MAGIC_REQUEST)
    {
        return OSPI_PHY_GRAPHER_STATUS_MAGIC_ERROR;
    }

    /* Extract and validate operation type */
    optype = cmdBuf[4];
    if (optype != OSPI_PHY_GRAPHER_UART_OPTYPE_PHY_TUNE)
    {
        return OSPI_PHY_GRAPHER_STATUS_OPTYPE_ERROR;
    }

    /* Extract size (bytes 12-15, little-endian) */
    size = cmdBuf[12] | (cmdBuf[13] << 8) | (cmdBuf[14] << 16) | (cmdBuf[15] << 24);
    if (size != OSPI_PHY_GRAPHER_UART_DATA_SIZE)
    {
        return OSPI_PHY_GRAPHER_STATUS_SIZE_ERROR;
    }

    return OSPI_PHY_GRAPHER_STATUS_SUCCESS;
}

/**
 * @brief Send status response to host
 *
 * Sends 16-byte status header with the following format:
 * - Bytes 0-3: Magic number (0x47524152 = "RARG")
 * - Bytes 4-7: Status code (0 = success, non-zero = error)
 * - Bytes 8-15: Reserved (all zeros)
 *
 * @param uartHandle    UART driver handle
 * @param statusCode    Status code to send
 *
 * @return 0 if success response sent, -1 on error
 */
static int32_t OSPI_phyGrapherUartSendStatus(UART_Handle uartHandle,
                                              uint32_t statusCode)
{
    uint8_t statusBuf[OSPI_PHY_GRAPHER_UART_STATUS_SIZE];
    uint32_t magic;
    uint32_t i;

    DebugP_log("[OSPI PHY Grapher UART] Sending status: 0x%08X\r\n", statusCode);

    /* Magic number (little-endian) */
    magic = OSPI_PHY_GRAPHER_UART_MAGIC_RESPONSE;
    statusBuf[0] = (magic >> 0) & 0xFF;
    statusBuf[1] = (magic >> 8) & 0xFF;
    statusBuf[2] = (magic >> 16) & 0xFF;
    statusBuf[3] = (magic >> 24) & 0xFF;

    /* Status code (little-endian) */
    statusBuf[4] = (statusCode >> 0) & 0xFF;
    statusBuf[5] = (statusCode >> 8) & 0xFF;
    statusBuf[6] = (statusCode >> 16) & 0xFF;
    statusBuf[7] = (statusCode >> 24) & 0xFF;

    /* Reserved (zeros) */
    for (i = 8; i < OSPI_PHY_GRAPHER_UART_STATUS_SIZE; i++)
    {
        statusBuf[i] = 0x00;
    }

    return OSPI_phyGrapherUartWriteBytes(uartHandle, statusBuf,
                                         OSPI_PHY_GRAPHER_UART_STATUS_SIZE);
}

/* ========================================================================== */
/*              Phase 3: XMODEM1k Transmission Functions                      */
/* ========================================================================== */

/**
 * @brief Create an XMODEM1k frame with data and CRC
 *
 * Builds a complete XMODEM1k block with the following structure:
 * - Byte 0: STX (0x02)
 * - Byte 1: Block number (0-255)
 * - Byte 2: Block number complement (~blockNum)
 * - Bytes 3-1026: Data payload (1024 bytes)
 * - Bytes 1027-1028: CRC16 (big-endian: high byte, low byte)
 *
 * @param blockNum      Block number (1-255)
 * @param data          Pointer to 1024-byte data block
 * @param frameBuf      Output buffer for frame (must be 1029 bytes)
 *
 * @return Size of frame (always 1029 bytes)
 */
static uint32_t OSPI_phyGrapherUartCreateXmodemFrame(uint32_t blockNum,
                                                     const uint8_t *data,
                                                     uint8_t *frameBuf)
{
    uint32_t idx = 0;
    uint8_t blockNumByte = blockNum & 0xFF;
    uint16_t crc;

    /* STX header */
    frameBuf[idx++] = OSPI_PHY_GRAPHER_UART_XMODEM_STX;

    /* Block number */
    frameBuf[idx++] = blockNumByte;

    /* Block number complement */
    frameBuf[idx++] = ~blockNumByte;

    /* Data payload (1024 bytes) */
    memcpy(&frameBuf[idx], data, OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE);
    idx += OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE;

    /* Calculate CRC16 for data */
    crc = OSPI_phyGrapherUartCalcCrc16(data, OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE);

    /* CRC16 (big-endian: high byte first, then low byte) */
    frameBuf[idx++] = (crc >> 8) & 0xFF;
    frameBuf[idx++] = crc & 0xFF;

    return idx;  /* Should be 1029 */
}

/**
 * @brief Send data via XMODEM1k protocol
 *
 * Transmits data in 1024-byte blocks using XMODEM1k protocol:
 * - Splits data into 1024-byte blocks
 * - Pads last block with 0x1A if needed
 * - Sends each block and waits for ACK
 * - Retries up to 10 times on NAK or timeout
 * - Sends EOT after all blocks
 * - Waits for final ACK
 *
 * @param uartHandle    UART driver handle
 * @param data          Pointer to data buffer
 * @param dataLen       Length of data in bytes
 *
 * @return SystemP_SUCCESS on successful transfer
 * @return SystemP_FAILURE on any error (CRC, timeout, max retries exceeded)
 */
static int32_t OSPI_phyGrapherUartSendXmodemData(UART_Handle uartHandle,
                                                 const uint8_t *data,
                                                 uint32_t dataLen)
{
    uint8_t frameBuf[OSPI_PHY_GRAPHER_UART_XMODEM_FRAME_SIZE];
    uint8_t blockData[OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE];
    uint32_t blockNum = 1;
    uint32_t sent = 0;
    uint8_t response;
    int32_t retries;
    uint32_t toSend;
    uint32_t frameSize;

    /* NOTE: Disable DebugP during XMODEM transmission to avoid corrupting binary data.
     * DebugP uses the same UART as XMODEM, so any debug output will corrupt the stream.
     * XMODEM data transfer must be silent (binary only, no text output).
     */

    /* Send all data in blocks */
    while (sent < dataLen)
    {
        /* Determine block size */
        toSend = ((dataLen - sent) > OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE) ?
                 OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE : (dataLen - sent);

        /* Prepare block data with padding */
        memcpy(blockData, &data[sent], toSend);
        if (toSend < OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE)
        {
            /* Pad with 0x1A (EOF marker) */
            memset(&blockData[toSend], 0x1A,
                   OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE - toSend);
        }

        /* Create XMODEM1k frame */
        frameSize = OSPI_phyGrapherUartCreateXmodemFrame(blockNum, blockData, frameBuf);

        /* Send block with retry logic */
        retries = OSPI_PHY_GRAPHER_UART_XMODEM_MAX_RETRIES;
        while (retries > 0)
        {
            /* Send frame */
            if (OSPI_phyGrapherUartWriteBytes(uartHandle, frameBuf, frameSize) != 0)
            {
                return SystemP_FAILURE;
            }

            /* Wait for ACK or NAK */
            if (OSPI_phyGrapherUartReadByte(uartHandle, &response,
                OSPI_PHY_GRAPHER_UART_XMODEM_TIMEOUT_MS) != 0)
            {
                retries--;
                continue;
            }

            if (response == OSPI_PHY_GRAPHER_UART_XMODEM_ACK)
            {
                /* Block accepted */
                sent += toSend;
                blockNum++;
                break;
            }
            else if (response == OSPI_PHY_GRAPHER_UART_XMODEM_NAK)
            {
                /* Block rejected, retry */
                retries--;
                continue;
            }
            else
            {
                /* Unexpected response, retry */
                retries--;
                continue;
            }
        }

        if (retries == 0)
        {
            return SystemP_FAILURE;
        }
    }

    /* Send EOT (End of Transmission) */
    uint8_t eot = OSPI_PHY_GRAPHER_UART_XMODEM_EOT;
    if (OSPI_phyGrapherUartWriteBytes(uartHandle, &eot, 1) != 0)
    {
        return SystemP_FAILURE;
    }

    /* Wait for final ACK */
    if (OSPI_phyGrapherUartReadByte(uartHandle, &response,
        OSPI_PHY_GRAPHER_UART_XMODEM_TIMEOUT_MS) != 0)
    {
        return SystemP_FAILURE;
    }

    if (response != OSPI_PHY_GRAPHER_UART_XMODEM_ACK)
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t OSPI_phyGrapherUartSend(OSPI_Handle ospiHandle,
                                UART_Handle uartHandle,
                                uint32_t flashOffset)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t cmdBuf[OSPI_PHY_GRAPHER_UART_HEADER_SIZE];

    DebugP_log("[OSPI PHY Grapher UART] Starting...\r\n");

    /* Step 1: Send ready signal */
    status = OSPI_phyGrapherUartSendReady(uartHandle);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("[ERROR] Failed to send ready signal\r\n");
        return status;
    }

    /* Step 2: Wait for and validate command */
    uint32_t cmd_status = OSPI_phyGrapherUartReceiveCommand(uartHandle, cmdBuf);
    if (cmd_status != SystemP_SUCCESS)
    {
        switch (cmd_status)
        {
            case OSPI_PHY_GRAPHER_STATUS_MAGIC_ERROR:
                DebugP_log("[ERROR] Invalid magic number in command\r\n");
                break;
            case OSPI_PHY_GRAPHER_STATUS_OPTYPE_ERROR:
                DebugP_log("[ERROR] Invalid operation type in command\r\n");
                break;
            case OSPI_PHY_GRAPHER_STATUS_SIZE_ERROR:
                DebugP_log("[ERROR] Invalid size in command\r\n");
                break;
            case OSPI_PHY_GRAPHER_STATUS_XMODEM_ERROR:
                DebugP_log("[ERROR] Timeout reading command header\r\n");
                break;
            default:
                DebugP_log("[ERROR] Unknown command error\r\n");
                break;
        }
        OSPI_phyGrapherUartSendStatus(uartHandle, cmd_status);
        return SystemP_FAILURE;
    }

    /* Step 3: PHY sweep - Collect tuning data from OSPI PHY */
    DebugP_log("[OSPI PHY Grapher UART] Starting PHY tuning sweep...\r\n");

    /* Now try the Grapher function */
    status = OSPI_phyTuneGrapher(ospiHandle, flashOffset, gOspiPhyGrapherData);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("[ERROR] PHY tuning sweep failed with status 0x%x\r\n", status);
        OSPI_phyGrapherUartSendStatus(uartHandle,
                                      OSPI_PHY_GRAPHER_STATUS_PHY_SWEEP_ERROR);
        return status;
    }
    DebugP_log("[OSPI PHY Grapher UART] PHY sweep complete. Data collected: 81920 bytes\r\n");

    /* Step 4: Data transmission via XMODEM1k */
    DebugP_log("[OSPI PHY Grapher UART] Starting XMODEM1k transmission...\r\n");
    ClockP_usleep(1000000);  /* Wait 1 second for receiver to be ready */
    status = OSPI_phyGrapherUartSendXmodemData(uartHandle,
                                               (const uint8_t *)gOspiPhyGrapherData,
                                               OSPI_PHY_GRAPHER_UART_DATA_SIZE);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("[ERROR] XMODEM1k transmission failed with status 0x%x\r\n", status);
        OSPI_phyGrapherUartSendStatus(uartHandle,
                                      OSPI_PHY_GRAPHER_STATUS_XMODEM_ERROR);
        return status;
    }
    DebugP_log("[OSPI PHY Grapher UART] XMODEM1k transmission complete\r\n");

    /* Step 5: Send final status */
    status = OSPI_phyGrapherUartSendStatus(uartHandle,
                                            OSPI_PHY_GRAPHER_STATUS_SUCCESS);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("[ERROR] Failed to send status\r\n");
        return status;
    }

    DebugP_log("[OSPI PHY Grapher UART] Complete\r\n");
    return SystemP_SUCCESS;
}

/* ========================================================================== */
/*                       Application Entry Point                              */
/* ========================================================================== */

/**
 * @brief OSPI PHY Grapher UART application main function
 *
 * This function is called from main.c after MCU+SDK system and driver
 * initialization. It runs the PHY tuning data transfer application.
 *
 * @param args Unused (NULL)
 */
void ospi_phy_grapher_uart_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    OSPI_Handle ospiHandle;
    UART_Handle uartHandle;
    Flash_Handle flashHandle;

    /* Open OSPI Driver, among others */
    Drivers_open();

    #if defined (SOC_AM263PX) || defined (SOC_AM261X)
        board_flash_reset(gOspiHandle[CONFIG_OSPI0]);
    #endif

    /* Open Flash drivers with OSPI instance as input */
    status = Board_driversOpen();

    uint32_t phyTuningOffset;

    DebugP_log("\n");
    DebugP_log("OSPI PHY Grapher UART Example\r\n");
    DebugP_log("==============================\r\n");
    DebugP_log("Debug output on UART0\r\n");
    DebugP_log("Binary data transfer on UART1 (CONFIG_UART1)\r\n");
    DebugP_log("==============================\r\n");

    /* Get OSPI driver handle */
    ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    DebugP_assert(ospiHandle != NULL);

    /* Get Flash driver handle */
    flashHandle = Flash_getHandle(CONFIG_FLASH0);
    DebugP_assert(flashHandle != NULL);

    /* Get PHY tuning offset from flash */
    phyTuningOffset = Flash_getPhyTuningOffset(flashHandle);

    /* Get UART1 handle for binary data transfer (separate from debug UART0) */
    uartHandle = UART_getHandle(CONFIG_UART1);
    DebugP_assert(uartHandle != NULL);
    DebugP_log("Opened CONFIG_UART1 for binary data\r\n");

    /* Run the PHY Grapher UART transfer */
    status = OSPI_phyGrapherUartSend(ospiHandle, uartHandle, phyTuningOffset);

    if (status == SystemP_SUCCESS)
    {
        DebugP_log("\nExample test passed!\r\n");
    }
    else
    {
        DebugP_log("\nExample test failed!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}
