/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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


#define DebugP_testLog(format, ...)     \
    do { \
        _DebugP_logZone(DebugP_LOG_ZONE_INFO, format, ##__VA_ARGS__); \
    } while(0)

#define CMD_TX_BUFSIZE (200U)
#define CMD_RX_BUFSIZE (200U)

#define CMD_SIZE (64U)
#define RSP_SIZE (64U)

#define SIZE_STRING_TEST_NAME       (70)
#define TIMEOUT_UART_MENU           (10000)
#define TIMEOUT_UART_TESTER         (10000)
#define TIMEOUT_UART_INFINITE       (SystemP_WAIT_FOREVER)

#define MAX_TIMEOUT_EVENTS          (1)

typedef void void_function(void* args);
typedef struct{
    int type;
    uint16_t function_ID;
    void_function* function_ptr;
    char *function_name;
}menu_input;

int32_t          transferOK;
UART_Transaction trans;

uint8_t gCmdTxBuffer[CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[CMD_RX_BUFSIZE];

bool enableLog;
bool manual_testing;

/* Initialises the tester-DUT UART Communication.*/
void tester_int(void);
/* waits for the user uart input of char 'y'*/
void wait_for_user_uart_input(void);
/* Sends out the str into UART based on the manual_testing*/
void tester_command(char *str);


void tester_init(void)
{
    UART_Transaction_init(&trans);
    trans.timeout = TIMEOUT_UART_TESTER;


    DebugP_testLog("\r\nSending initialization command!!");

    trans.buf = &gCmdTxBuffer[0U];
    trans.count = CMD_SIZE; // if not pulled up CMD_SIZE-1 expected. Send CMD_SIZE-1 only to sync. Unexpected 1.
    strncpy(trans.buf, "123456780000000000000000ABCDEFEF", CMD_SIZE);

    transferOK = UART_write(gUartHandle[TESTER_UART], &trans);

    /* Quickly wait to read ack. No debug prints here*/
    trans.buf = &gCmdRxBuffer[0U];
    trans.count = RSP_SIZE;
    transferOK = UART_read(gUartHandle[TESTER_UART], &trans);

    DebugP_logInfo("\r\nReceived response. Tester Initialized!!");

    /*Clear TX buffer for shorter commands*/
    uint8_t ind;
    for (ind = 0; ind < CMD_SIZE; ind++)
    {
        gCmdTxBuffer[ind] = 0;
    }
}

void wait_for_user_uart_input(void)
{

    bool correct_input = false;

    while (!correct_input)
    {
        DebugP_log("please enter 'y' after setting up required configuration for test\r\n");

        /*Clearing the Rx buffer for 1 char*/
        gCmdRxBuffer[0] = '\0';

        /* forever wait until the user inputs*/
        trans.buf = &gCmdRxBuffer[0U];
        trans.timeout = TIMEOUT_UART_INFINITE;
        trans.count = 1;
        transferOK = UART_read(gUartHandle[CONFIG_UART0], &trans);

        correct_input = (gCmdRxBuffer[0] == 'y') || (gCmdRxBuffer[0] == 'Y');
    }

    /* changing timeout back to default */
    trans.timeout = TIMEOUT_UART_TESTER;

    return;
}

void tester_command(char *str)
{
    if (manual_testing)
    {
        DebugP_log("%s\r\n", str);
        wait_for_user_uart_input();
        return;
    }

    DebugP_logInfo("\r\nSending command!!");

    /* Send command*/
    trans.buf = &gCmdTxBuffer[0U];
    strncpy(trans.buf, str, strlen(str));
    trans.count = CMD_SIZE;
    transferOK = UART_write(gUartHandle[TESTER_UART], &trans);

    /* Quickly wait to read ack. No debug prints here*/
    trans.buf = &gCmdRxBuffer[0U];
    trans.count = RSP_SIZE;
    transferOK = UART_read(gUartHandle[TESTER_UART], &trans);

    uint8_t ind;

    DebugP_testLog("\r\nReceived response!! (%d %d) : ", transferOK, trans.status);
    for (ind = 0; ind < RSP_SIZE; ind++)
    {
        DebugP_testLog("%c", gCmdRxBuffer[ind]);
    }
    DebugP_testLog("\r\n");

    /*Clear TX buffer for shorter commands*/
    for (ind = 0; ind < CMD_SIZE; ind++)
    {
        gCmdTxBuffer[ind] = 0;
    }
}

uint16_t retry_count_left = MAX_TIMEOUT_EVENTS;

bool util_check_retry(void)
{

    DebugP_log("Response timeout. Number of Retries left : %d\r\n", retry_count_left);
    if(retry_count_left <= 0 )
    {
        DebugP_log("Quitting the tests\r\n");
        return false;
    }
    else
    {
        retry_count_left--;
        return true;
    }

}

void menu(uint16_t total_test_cases, menu_input* test_list, char* test_title)
{
    bool while_breaker = 0;

    while(!while_breaker)
    {
        DebugP_log("%s\r\n", test_title);

        DebugP_log("\r\n\t01\t:\tmanual testing\r\n\t02\t:\tauto testing (using Tester Applicaiton)\r\n\t03\t:\tQuit\r\n");

        UART_Transaction_init(&trans);
        uint8_t optionBuffer[3] = {0};
        int option;
        int i;

        trans.buf       = &optionBuffer[0U];
        trans.count     = 2;
        trans.timeout   = TIMEOUT_UART_MENU;

        ((UART_Config *)gUartHandle[CONFIG_UART0])->object->readTrans = NULL;
        transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
        {
            if(true == util_check_retry())
            {
                continue;
            }
            else
            {
                while_breaker = 1;
                break;
            }
            DebugP_log("\r\nInvalid input! \r\n Auto testing (using Tester Application) is selected by default");
            manual_testing = 0;
        }
        else
        {
            option = (int) atof((char *)optionBuffer);
            DebugP_log("\r\nEntered option: %02d\r\n", option);
            if(option == 1)
            {
                manual_testing = 1;
            }
            else if(option == 2)
            {
                DebugP_log("Auto testing (using Tester Application is selected)\r\n");
                DebugP_log("Please note:\r\n In Auto testing, Tester Application is used. The expected inputs to the tests are expected to be provided through the Tester Application\r\n");
            }
            else if (option == 3)
            {
                DebugP_log("Quitting\r\n");
                break;
            }
            else
            {
                DebugP_log("Invalid input.\r\n");
                continue;
            }
            if(manual_testing)
            {
                DebugP_log("Manual testing is selected\r\n");
                DebugP_log("Please note:\r\n In manual testing, please provide the required inputs for the tests\r\n");
            }
        }

        DebugP_log("Test Lists\r\n\tOption\t|\tName\r\n\t-------------------------------------------------------\r\n");

        DebugP_log("\t00\t|\t%s\r\n", "All tests and quit");

        int prev_type = -1;
        int current_type = 0;

        for(uint16_t iter = 0; iter < total_test_cases; iter = iter+1)
        {
            current_type = test_list[iter].type;
            if(current_type != prev_type)
            {
                DebugP_log("\t-------------------------------------------------------\r\n");
                DebugP_log("type %d tests\r\n\t-------------------------------------------------------\r\n", current_type);
                prev_type = current_type;
            }
            DebugP_log("\t%2.2d\t|\t%s\r\n", iter+1, test_list[iter].function_name);
        }
        DebugP_log("\t-------------------------------------------------------\r\n");

        int max_type = test_list[total_test_cases-1].type;
        for(int iter_type = 0; iter_type <= max_type; iter_type++)
        {
            DebugP_log("\t-------------------------------------------------------\r\n");
            DebugP_log("\t%2.2d\t|\tAll %d type tests\r\n", 99-(iter_type), iter_type);
            DebugP_log("\t-------------------------------------------------------\r\n");
        }

        DebugP_log("\r\nEnter option (2 characters): \r\n");

        UART_Transaction_init(&trans);
        for(i=0;i<3;i++)
        {
            optionBuffer[i]=0;
        }
        trans.buf       = &optionBuffer[0U];
        trans.count     = 2;
        trans.timeout   = TIMEOUT_UART_MENU;
        transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
        {
            DebugP_log("\r\nInvalid input! \r\n selecting all and quit by default\r\n");
            option = 0;     //""Run All tests and quit"
        }
        else
        {
            option = (int) atof((char *)optionBuffer);
            DebugP_log("\r\nEntered option: %02d\r\n", option);
        }
        char log_option;
        if(!manual_testing)
        {
            DebugP_log("Do you want to enable logs during the tests? [y/n]\t:\r\n");
            trans.buf       = &optionBuffer[0U];
            trans.count     = 1;
            trans.timeout   = TIMEOUT_UART_MENU;
            transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

            if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
            {
                DebugP_log("\r\nInvalid input! \r\n selecting disable logs by default\r\n");
                log_option = 'n';
                DebugP_logZoneDisable(DebugP_LOG_ZONE_INFO);
            }
            else
            {
                log_option = optionBuffer[0];
                DebugP_log("\r\nEntered option: %c\r\n", log_option);
            }
            if ((log_option == 'y') || (log_option == 'Y'))
            {
                enableLog = 1;
                DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO);
                DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
            }
            else
            {
                enableLog = 0;
                DebugP_logZoneDisable(DebugP_LOG_ZONE_INFO);
                DebugP_logZoneDisable(DebugP_LOG_ZONE_ERROR);
            }
        }
        else
        {
            DebugP_log("Manual testing, Logs are enabled by default\r\n");
            enableLog = 1;
        }
        switch (option)
        {
            case 0:
            {
                DebugP_log("Running All tests and exit\r\n");
                for(int iter = 0; iter < total_test_cases; iter++)
                {
                    UnityDefaultTestRun(test_list[iter].function_ptr, test_list[iter].function_name, __LINE__, test_list[iter].function_ID, NULL);
                }
                    while_breaker = 1;
                break;

            }
            default:
                {
                    if((option < 100) && (option >= (99 - (max_type))))
                    {
                        DebugP_log("option : %d\r\n", option);
                        for(int iter = 0; iter < total_test_cases; iter++)
                        {
                            if((99-option) == test_list[iter].type)
                            {
                                UnityDefaultTestRun(test_list[iter].function_ptr, test_list[iter].function_name, __LINE__, test_list[iter].function_ID, NULL);
                            }
                        }
                        break;
                    }
                    if((option-1) >= total_test_cases)
                    {
                        DebugP_log("Invalid option\r\n");
                        break;
                    }
                    UnityDefaultTestRun(test_list[option-1].function_ptr, test_list[option-1].function_name, __LINE__, test_list[option-1].function_ID, NULL);
                    break;
                }
        }
    }
}