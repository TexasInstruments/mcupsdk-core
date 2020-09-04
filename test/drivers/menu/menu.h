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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <string.h>
#include <kernel/dpl/ClockP.h>


#define MENU_CMD_TX_BUFSIZE              (200U)
#define MENU_CMD_RX_BUFSIZE              (200U)

#define MENU_CMD_SIZE            (2U)
#define MENU_RSP_SIZE            (32U)

int32_t          transferOK;
UART_Transaction trans;
uint8_t gCmdTxBuffer[MENU_CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[MENU_CMD_RX_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
/*
 * this function uses the Uart instance specified and included from the calling
 * application, to read messages. by default, it writes to CCS_log.
 * if Uart log is enabled, it writes to the Uart console specified
 * under Debug log in syscfg.
 */

//*****************************************************************************
//! \param list_of_tests is the list of tests passed from unity test frame work
//! \param length_of_tests is number of tests included in the test frame work
//! \param UART_instance is the uart instance to be used to get input
//!
//!
//! this function uses the \b Uart_instance specified and included from
//! the calling application, to read messages. by default, it writes to CCS_log.
//! if Uart log is enabled, it writes to the Uart console specified
//! under Debug log in syscfg.
//! \return option mentioned by user
//******************************************************************************
int32_t menu(
    char list_of_tests[100][100],
    int16_t length_of_tests,
    int16_t UART_instance)
{


    DebugP_log("every entry must be of 1 character\r\n");
    DebugP_log("enter 'm' for manual selection of test cases\r\n");
    DebugP_log("enter 'a' for selection of all test cases\r\n");

    gNumBytesRead = 0U;
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = MENU_CMD_SIZE-1;
    transferOK = UART_read(gUartHandle[UART_instance], &trans);

    if((gCmdRxBuffer[0] == 'a') ||gCmdRxBuffer[0] == 'A')
    {
        return 0;
    }

    for(int iterative = 0;
            iterative <= length_of_tests;
            iterative++)
        {
            DebugP_log("%d\t:\t%s\r\n",(iterative+1), list_of_tests[iterative]);
        }

    DebugP_log("every entry must be of 2 characters\r\n");
    gNumBytesRead = 0U;
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = MENU_CMD_SIZE;
    transferOK = UART_read(gUartHandle[UART_instance], &trans);

    char option[3];
    memcpy(option, &gCmdRxBuffer[0],2);
    int Uart_read_option = (int) atof(option);

    return Uart_read_option;
}



//*****************************************************************************
//! this function uses the \b Uart_instance specified and included from
//! the calling application, to read messages. by default, it writes to CCS_log.
//! if Uart log is enabled, it writes to the Uart console specified
//! under Debug log in syscfg.
//! \return option mentioned by user
//!         : \b -1 infinite timeout is selected by user
//!         : \b  1 finite timeout is selected by user
//!         : \b  0 no timeout is selected by user
//******************************************************************************
int32_t time_out_request(int16_t UART_instance)
{
    DebugP_log("[enter single char] do you want to run with timeout? : [y/n]\r\n");
    gNumBytesRead = 0U;
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = MENU_CMD_SIZE-1;
    transferOK = UART_read(gUartHandle[UART_instance], &trans);

    if(gCmdRxBuffer[0] == 'y')
    {
        DebugP_log("[enter single char] finite time out? : [y/n]\r\n");
        gNumBytesRead = 0U;
        trans.buf   = &gCmdRxBuffer[0U];
        trans.count = MENU_CMD_SIZE-1;
        transferOK = UART_read(gUartHandle[UART_instance], &trans);
        if(gCmdRxBuffer[0] == 'y')
        {
            return 1;
        }
        else
        {
            return -1;
        }

    }
    else
    {
        return 0;
    }
}
