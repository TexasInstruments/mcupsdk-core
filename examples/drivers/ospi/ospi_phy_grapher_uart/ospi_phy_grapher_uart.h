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

#ifndef OSPI_PHY_GRAPHER_UART_H
#define OSPI_PHY_GRAPHER_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/ospi.h>
#include <drivers/uart.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** @defgroup OSPI_PHY_GRAPHER_UART_PROTOCOL Protocol Constants
 *  @{
 */

/** Magic number for request command header ("PRAG" in ASCII) */
#define OSPI_PHY_GRAPHER_UART_MAGIC_REQUEST     (0x47524150U)

/** Magic number for response header ("RARG" in ASCII) */
#define OSPI_PHY_GRAPHER_UART_MAGIC_RESPONSE    (0x47524152U)

/** Size of command header (32 bytes) */
#define OSPI_PHY_GRAPHER_UART_HEADER_SIZE       (32U)

/** Size of status response header (16 bytes) */
#define OSPI_PHY_GRAPHER_UART_STATUS_SIZE       (16U)

/** Total data size: 5 rdDelays × 128 txDLL × 128 rxDLL = 81,920 bytes */
#define OSPI_PHY_GRAPHER_UART_DATA_SIZE         (81920U)

/** Operation type for PHY tuning (matches Uniflash pattern) */
#define OSPI_PHY_GRAPHER_UART_OPTYPE_PHY_TUNE   (0xFDU)

/** @} */

/** @defgroup OSPI_PHY_GRAPHER_UART_XMODEM XMODEM1k Constants
 *  @{
 */

/** XMODEM1k start-of-header byte */
#define OSPI_PHY_GRAPHER_UART_XMODEM_STX        (0x02U)

/** XMODEM1k end-of-transmission byte */
#define OSPI_PHY_GRAPHER_UART_XMODEM_EOT        (0x04U)

/** XMODEM1k acknowledge byte */
#define OSPI_PHY_GRAPHER_UART_XMODEM_ACK        (0x06U)

/** XMODEM1k negative-acknowledge byte */
#define OSPI_PHY_GRAPHER_UART_XMODEM_NAK        (0x15U)

/** XMODEM1k cancel byte */
#define OSPI_PHY_GRAPHER_UART_XMODEM_CAN        (0x18U)

/** XMODEM1k block size in bytes */
#define OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE (1024U)

/** XMODEM1k frame overhead: STX(1) + BlockNum(1) + Complement(1) + CRC(2) */
#define OSPI_PHY_GRAPHER_UART_XMODEM_OVERHEAD   (5U)

/** XMODEM1k total frame size */
#define OSPI_PHY_GRAPHER_UART_XMODEM_FRAME_SIZE \
    (OSPI_PHY_GRAPHER_UART_XMODEM_BLOCK_SIZE + OSPI_PHY_GRAPHER_UART_XMODEM_OVERHEAD)

/** XMODEM1k timeout in milliseconds */
#define OSPI_PHY_GRAPHER_UART_XMODEM_TIMEOUT_MS (5000U)

/** Maximum XMODEM1k retries per block */
#define OSPI_PHY_GRAPHER_UART_XMODEM_MAX_RETRIES (20U)

/** @} */

/**
 * @brief Status codes for PHY Grapher UART operations
 */
typedef enum {
    /** Operation completed successfully */
    OSPI_PHY_GRAPHER_STATUS_SUCCESS = 0x00000000,

    /** Invalid magic number in command header */
    OSPI_PHY_GRAPHER_STATUS_MAGIC_ERROR = 0x10000001,

    /** Invalid operation type in command header */
    OSPI_PHY_GRAPHER_STATUS_OPTYPE_ERROR = 0x20000001,

    /** Invalid size in command header */
    OSPI_PHY_GRAPHER_STATUS_SIZE_ERROR = 0x25000001,

    /** PHY tuning sweep failed */
    OSPI_PHY_GRAPHER_STATUS_PHY_SWEEP_ERROR = 0x30000001,

    /** XMODEM transmission failed (CRC, timeout, etc.) */
    OSPI_PHY_GRAPHER_STATUS_XMODEM_ERROR = 0x40000001,
} OSPI_PhyGrapherStatus;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Send OSPI PHY tuning data via UART using XMODEM1k protocol
 *
 * This function:
 * 1. Sends "READY_FOR_SWEEP\r\n" to indicate readiness
 * 2. Waits for a 32-byte command header from host
 * 3. Validates command magic and operation type
 * 4. Runs OSPI_phyTuneGrapher() to collect PHY tuning data
 * 5. Transmits data via XMODEM1k blocks (1024 bytes each)
 * 6. Sends 16-byte status response
 *
 * @param ospiHandle    OSPI driver handle
 * @param uartHandle    UART driver handle
 * @param flashOffset   Flash offset for PHY data (currently unused)
 *
 * @return SystemP_SUCCESS if transfer completed successfully
 * @return SystemP_FAILURE if any step failed
 *
 * @note This function blocks until transfer completes or times out
 * @note Data array is 81,920 bytes: [5 rdDelays][128 txDLL][128 rxDLL]
 * @note UART should be configured for 115200 baud, 8 bits, no parity, 1 stop bit
 *
 * @see OSPI_phyTuneGrapher
 * @see UART_write
 * @see UART_read
 */
int32_t OSPI_phyGrapherUartSend(OSPI_Handle ospiHandle,
                                UART_Handle uartHandle,
                                uint32_t flashOffset);

#ifdef __cplusplus
}
#endif

#endif /* OSPI_PHY_GRAPHER_UART_H */
