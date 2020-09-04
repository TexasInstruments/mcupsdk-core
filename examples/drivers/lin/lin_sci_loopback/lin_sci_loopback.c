/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/*
 *   This example configures the LIN module in SCI mode for internal loopback
 *   with interrupts. The LIN module performs as a SCI with a set character
 *   and frame length in a non-multi-buffer mode. The module is setup to
 *   continuously transmit a character, wait to receive that character, and
 *   repeat.
 *
 *  External Connections :
 *   - None.
 *
 *  Watch Variables :
 *   - rxCount - The number of RX interrupts
 *   - transmitChar - The character being transmitted
 *   - receivedChar - The character received
 *
 */

/* Included Files */
#include <kernel/dpl/DebugP.h>
#include <drivers/lin.h>
#include <ti_drivers_config.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Defines */
#define CHAR_LENGTH         (0x8)
#define FRAME_LENGTH        (0x1)

#define APP_LIN_BASE_ADDR   (CONFIG_LIN1_BASE_ADDR)
#define APP_LIN_INTR_NUM_0  (CONFIG_LIN1_INTR_NUM_0)

/* Globals */
volatile uint32_t rxCount = 0;
volatile uint32_t vectorOffset = 0;
volatile uint16_t error = 0;
uint16_t transmitChar = 0x0;
uint16_t receivedChar = 0x0;

static void dataRxISR(void *args);
static void configureSCIMode(void);

static HwiP_Object gLinHwiObject_0;

/* lin_sci_loopback_main */
void lin_sci_loopback_main(void)
{
    HwiP_Params             hwiPrms;
    int32_t                 status = SystemP_SUCCESS;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[LIN] SCI Loopback mode, application started ...\r\n");

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_LIN_INTR_NUM_0;
    hwiPrms.callback    = &dataRxISR;
    status              = HwiP_construct(&gLinHwiObject_0, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure the LIN module to operate in SCI mode */
    configureSCIMode();

    /* Enable parity check */
    LIN_enableParity(APP_LIN_BASE_ADDR);

    /* Enable global interrupt lines and clear status to known value */
    LIN_enableGlobalInterrupt(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE0);
    LIN_clearGlobalInterruptStatus(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE0);

    /*
     * Continuously transmit an 8-bit character, wait for ISR to run, and
     * verify everything was received correctly
     */
    for(;;)
    {
        vectorOffset = 0;

        /* Increment transmit character to new value */
        transmitChar++;

        /* Reset transmit character when larger than a byte */
        if(transmitChar > 0xFF)
        {
            transmitChar = 0;
        }

        /* Wait for the SCI receiver to be idle */
        while(!LIN_isSCIReceiverIdle(APP_LIN_BASE_ADDR));

        /* Transmit the byte of data */
        LIN_writeSCICharBlocking(APP_LIN_BASE_ADDR, transmitChar);

        /* Wait for ISR to trigger and read the transmitted character */
        while(vectorOffset != LIN_VECT_RX);

        /* Halt the example */
        break;
    }

    /* Check if any data errors occurred */
    if(error == 0)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Test FAILED!!\r\n");
    }

    HwiP_destruct(&gLinHwiObject_0);
    Board_driversClose();
    Drivers_close();
}

/*
 * Configure SCI Mode - This function configures the LIN module to operate as
 * an SCI with the specified settings.
*/
void
configureSCIMode(void)
{
    /* Enter LIN reset state to perform configurations */
    LIN_enterSoftwareReset(APP_LIN_BASE_ADDR);

    /* Switch LIN into SCI mode */
    LIN_enableSCIMode(APP_LIN_BASE_ADDR);

    /* Set the SCI communication mode to idle line */
    LIN_setSCICommMode(APP_LIN_BASE_ADDR, LIN_COMM_SCI_IDLELINE);

    /* Set SCI to transmit one stop bit */
    LIN_setSCIStopBits(APP_LIN_BASE_ADDR, LIN_SCI_STOP_ONE);

    /* Disable parity check */
    LIN_disableSCIParity(APP_LIN_BASE_ADDR);

    /* Disable multi-buffer mode */
    LIN_disableMultibufferMode(APP_LIN_BASE_ADDR);

    /* Module set to complete operations when halted by debugger */
    LIN_setDebugSuspendMode(APP_LIN_BASE_ADDR, LIN_DEBUG_COMPLETE);

    /* Set character length as 8-bits */
    LIN_setSCICharLength(APP_LIN_BASE_ADDR, CHAR_LENGTH);

    /* Set to 1 character in response field */
    LIN_setSCIFrameLength(APP_LIN_BASE_ADDR, FRAME_LENGTH);

    /* Enable Internal Loopback mode */
    LIN_enableIntLoopback(APP_LIN_BASE_ADDR);

    /* Enable interrupt for when a frame has been completely received */
    LIN_enableSCIInterrupt(APP_LIN_BASE_ADDR, LIN_SCI_INT_RX);

    /* Set the interrupt priority to line 0 (high) */
    LIN_setSCIInterruptLevel0(APP_LIN_BASE_ADDR, LIN_SCI_INT_RX);

    /* Exit LIN reset state */
    LIN_exitSoftwareReset(APP_LIN_BASE_ADDR);
}

/*
 * Received Data ISR - An interrupt service routine (ISR) to handle when new
 * data is received. Once received, the data is read and interrupt status
 * cleared.
 */
static void dataRxISR(void * args)
{
    /* Increment the interrupt count */
    rxCount++;

    /* Read the highest priority interrupt vector */
    vectorOffset = LIN_getInterruptLine0Offset(APP_LIN_BASE_ADDR);

    /* Read the transmitted character */
    receivedChar = LIN_readSCICharBlocking(APP_LIN_BASE_ADDR, CSL_FALSE);

    /* Check that the received character matches the transmitted character */
    if(receivedChar != transmitChar)
    {
        error++;
    }

    /* Clear module interrupt flag and global interrupt flag for line 0 */
    LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_INT_RX);
    LIN_clearGlobalInterruptStatus(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE0);
}