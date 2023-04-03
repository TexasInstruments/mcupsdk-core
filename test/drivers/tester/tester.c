/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
#include <stdio.h>
#include <string.h>
#include <unity.h>
#include <drivers/sdfm.h>
#include <drivers/epwm.h>
#include <drivers/soc.h>
#include <drivers/ecap.h>
#include <drivers/edma.h>
#include <drivers/adc.h>
#include <drivers/dac.h>
#include <drivers/cmpss.h>
#include <drivers/eqep.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/ClockP.h>
#include <drivers/pinmux.h>


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */



/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* Communication GLOBALs*/
#define CMD_TX_BUFSIZE              (200U)
#define CMD_RX_BUFSIZE              (200U)

#define CMD_SIZE            (64U)
#define RSP_SIZE            (64U)

volatile uint32_t command_uart;

uint8_t gCmdTxBuffer[CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[CMD_RX_BUFSIZE];


volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
volatile int32_t          transferOK;
static UART_Transaction trans;

static bool send_ack = true;

/* PWM-ADC voltage generation Globals*/
#define MAX_PWM_VAL              (4096)
#define MAX_ANALOG_VOLTAGE       (3.29)
#define PINMUX_DOMAIN_ID_MAIN          (0U)

/* GPIO-XBAR-PWM-ECAP gloval Variables*/

/*
 * GPIO for each EPWM, xA,xB.
 */
uint8_t gpio[32][3]= {
    {0, 43, 44},   {1, 45, 46},    {2, 47, 48},    {3, 49, 50},
    {4, 51, 52},   {5, 53, 54},    {6, 55, 56},    {7, 57, 58},
    {8, -1, -1},   {9, -1, -1},    {10, -1, -1},   {11, -1, -1},
    {12, -1, -1},  {13, 69, 70},   {14, 71, 72},   {15, 73, 74},
    {16, 75, 76},  {17, 77, 78},   {18, 79, 80},   {19, 81, 82},
    {20, 83, 84},  {21, 85, 86},   {22, 87, 88},   {23, 89, 90},
    {24, 91, 92},  {25, 93, 94},   {26, 95, 96},   {27, 97, 98},
    {28, 99, 100}, {29, 101, 102}, {30, 103, 104}, {31, 105, 106}
    };

/*
 * 32 inputxbar instances to log which GPIO is selected.
 */
uint8_t inputXbar[32][3] = {
    {0, -1, -1},  {1, -1, -1},  {2, -1, -1},  {3, -1, -1},
    {4, -1, -1},  {5, -1, -1},  {6, -1, -1},  {7, -1, -1},
    {8, -1, -1},  {9, -1, -1},  {10, -1, -1}, {11, -1, -1},
    {12, -1, -1}, {13, -1, -1}, {14, -1, -1}, {15, -1, -1},
    {16, -1, -1}, {17, -1, -1}, {18, -1, -1}, {19, -1, -1},
    {20, -1, -1}, {21, -1, -1}, {22, -1, -1}, {23, -1, -1},
    {24, -1, -1}, {25, -1, -1}, {26, -1, -1}, {27, -1, -1},
    {28, -1, -1}, {29, -1, -1}, {30, -1, -1}, {31, -1, -1}
    };

/*
 * ecap instance to log which GPIO is used.
 */
uint8_t ecap[32][3] =   {
    {0, -1, -1},  {1, -1, -1},  {2, -1, -1},  {3, -1, -1},
    {4, -1, -1},  {5, -1, -1},  {6, -1, -1},  {7, -1, -1},
    {8, -1, -1},  {9, -1, -1},  {10, -1, -1}, {11, -1, -1},
    {12, -1, -1}, {13, -1, -1}, {14, -1, -1}, {15, -1, -1},
    {16, -1, -1}, {17, -1, -1}, {18, -1, -1}, {19, -1, -1},
    {20, -1, -1}, {21, -1, -1}, {22, -1, -1}, {23, -1, -1},
    {24, -1, -1}, {25, -1, -1}, {26, -1, -1}, {27, -1, -1},
    {28, -1, -1}, {29, -1, -1}, {30, -1, -1}, {31, -1, -1}
    };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * sets up the command source as the XDS UART or DUT UART
 * Updates the global variable command_uart
 */
void set_command_source(void);

/*
 * reads Commands from command_uart and stores in gCmdRxBuffer
 */
void read_uart_command(void);

/*
 * Sets the GPIOs as input pins for usage.
 */
void Util_GPIO_init(void);

int startsWith(char *str, char *substr);

/*
 * util_ECAP_init - Configure eCAP
 */
void util_ECAP_init(uint32_t base, uint32_t input);

void SyncEPWM(int8_t epwm_offset, Float32 duty_cycle, uint32_t period);

/*
 * for deinitializing the pwm
 */
void util_deinit_epwms(int8_t epwm_offset);

/*
 * sets a period of 100, and a duty cycle as passed arg
 */
void util_start_pwm_wave(uint8_t epwm_offset, Float32 duty_cycle, uint16_t period);

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    Util_GPIO_init();
    // UNITY_BEGIN();

    /* Tester Menu for Selection of Command source*/
    DebugP_log("Tester started ...\r\n");
    DebugP_log("\r\n");
    DebugP_log("'0' - Receive Manual commands...\r\n");
    DebugP_log("'1' - Receive Automated commands ...\r\n");
    DebugP_log("Select UART interface for commands:\r\n");

    /* Initiating the UART*/
    UART_Transaction_init(&trans);

    uint8_t epwm = -1, out = 0;
    int ecap_tracker = -1;


    /* Select the command source*/
    set_command_source();

    while(1)
    {
        /* sends ack by default */
        send_ack = true;

        read_uart_command();
        /* tester initialisation command */
        if( (gCmdRxBuffer[CMD_SIZE-2]=='E') && (gCmdRxBuffer[CMD_SIZE-1]=='F') )
        {
            DebugP_log("Tester Initialized!!\r\n ");
        }

        /* "provide analog voltage of 2.0000V on ADC 0 Channel 1" */
        else if (startsWith((char *)gCmdRxBuffer, "provide analog voltage"))
        {
            /* "provide analog voltage of 2.0000V on ADC 0 Channel 1" */
            /* "provide analog voltage of a.bcdeV on ADC X Channel Y" */

            /* take voltage values. */
            /* take Channel odd/even part. */

            volatile uint8_t epwm_offset;
            char adc_instance_char[1];
            adc_instance_char[0] = gCmdRxBuffer[41];
            uint8_t adc_instance_number = atoi(adc_instance_char);

            if( (adc_instance_number < 0) || (adc_instance_number > 4))
            {
                DebugP_log("Invalid ADC instance!!\r\n ");
            }
            else
            {
                char channel[1];
                // memcpy(channel, &gCmdRxBuffer[strlen("provide analog voltage of 2.0000V on ADC 0 Channel ")], 1);
                channel[0] = gCmdRxBuffer[51];
                // DebugP_log("%s\r\n", channel);
                int32_t channel_number = atoi(channel);

                if((channel_number < 0) || (channel_number > 5))
                {
                    DebugP_log("Invalid Channel selected!!");
                }
                else
                {

                    /* Setting EPWM PIN mux (overwriting if already if existed)*/
                    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
                        /* EPWM11_A -> EPWM11_A (H1) */
                        {
                            PIN_EPWM11_A,
                            ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
                        },
                        /* EPWM22_B -> EPWM22_B (F17) */
                        {
                            PIN_PR0_PRU0_GPIO9,
                            ( PIN_MODE(5) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
                        },

                        {PINMUX_END, PINMUX_END}
                    };
                    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);

                    // GPIO_setDirMode(CAPTURE_EPWM11A_BASE_ADDR, CAPTURE_EPWM11A_PIN, 0);
                    // GPIO_setDirMode(CAPTURE_EPWM14A_BASE_ADDR, CAPTURE_EPWM14A_PIN, 0);
                    /* TODO :
                        Need to add a PinMux Configuration read, verify the set or skip this step.*/

                    char voltage_str[7];
                    memcpy(voltage_str, &gCmdRxBuffer[strlen("provide analog voltage of")], 6);
                    Float32 voltage = atof(voltage_str);
                    Float32 duty_cycle = (Float32) (voltage*100)/(MAX_ANALOG_VOLTAGE);

                    DebugP_log("%d\r\n", channel_number);
                    channel_number = channel_number%2;
                    if(channel_number == 0)
                    {
                        /* even channel, energize pwm 11 A */
                        epwm_offset = 11;
                    }
                    else
                    {
                        /* odd channel, energize pwm 22 B */
                        epwm_offset = 22;
                    }
                    /* Enabling EPWM clock for instance 11,22*/
                    SOC_setEpwmTbClk(epwm_offset, true);
                    util_start_pwm_wave(epwm_offset, duty_cycle, MAX_PWM_VAL);
                    /* TODO :
                        Need to add a voltage feedback loop to verify if the set duty cycle is able to drive
                        required voltage*/

                    /* Wait for voltage stablisation*/
                    ClockP_usleep(100000);
                }
            }
        }

        /* TODO : add description to the command*/
        else if (startsWith((char *)gCmdRxBuffer, "setup epwm"))
        {
            util_deinit_epwms(8);
            char epmwInstance[3];
            int l = strlen("setup epwm ");
            memcpy(epmwInstance, &gCmdRxBuffer[l],2);
            epmwInstance[2]='\0';
            epwm = atoi(epmwInstance);

            char outChannel;
            memcpy(&outChannel, &gCmdRxBuffer[l + 3],1);
            out = (int)(outChannel - 'A' +1);
            ecap_tracker = (ecap_tracker + 1)%10;
            inputXbar[epwm][out] = ecap_tracker; // inputxbar_instance <--> ecap_instance
            ecap[epwm][out] = ecap_tracker;
           // if(epwm == -1)
            DebugP_log("%d and ecap is %d\n", epwm, ecap_tracker);
        }
        //cap ecap <ecap instance>. <pol1><pol2><pol3><pol4> <Abs/Del> <Mode>
        /* capture epwm xx in oneshot */
        else if (startsWith((char *)gCmdRxBuffer, "capture epwm"))
        {
            char epwminstance[3];
            int l = strlen("capture epwm ");
            memcpy(epwminstance, &gCmdRxBuffer[l],2);
            epwminstance[2]='\0';
            epwm = atoi(epwminstance);

            char outChannel;
            memcpy(&outChannel, &gCmdRxBuffer[l + 3],1);
            out = (int)(outChannel - 'A' +1);

            DebugP_log("%d %d %d\r\n", epwm, out, ecap[epwm][out]);
            uint32_t ecap_base=CSL_CONTROLSS_ECAP0_U_BASE + ecap[epwm][out] *0x1000;
            SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, inputXbar[epwm][out], 0, gpio[epwm][out], 0);
            util_ECAP_init(ecap_base, ECAP_INPUT_INPUTXBAR0+inputXbar[epwm][out]);

            //int8_t len=strlen("cap ecap x. ");


            char mode[2];
            int ptr = 0;
            memcpy(mode, &gCmdRxBuffer[l+5],1);
            mode[1]='\0';
            DebugP_log("Mode is %s\r\n", mode);
            if(mode[0] == 'c')
                {
                    ECAP_setCaptureMode(ecap_base, ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_4);
                    ptr = 27;
                }
            else
                {
                    ECAP_setCaptureMode(ecap_base, ECAP_ONE_SHOT_CAPTURE_MODE, ECAP_EVENT_4);
                    ptr = 26;
                }


            char polarity[5];
            memcpy(polarity, &gCmdRxBuffer[l+ptr],4);
            polarity[4]='\0';

            DebugP_log("Polarity is %s\r\n", polarity);
            int16_t event = 0;

            for(int i=0;i <4;i++)
            {
                if(polarity[i] == 'R')
                    event = (int16_t)ECAP_EVNT_RISING_EDGE;
                else
                    event = (int16_t)ECAP_EVNT_FALLING_EDGE;

                //DebugP_log("%d %d\r\n", ECAP_EVENT_1 + i, event);
                ECAP_setEventPolarity(ecap_base, ECAP_EVENT_1 + i, event);
            }

           char ts[2];
           memcpy(ts, &gCmdRxBuffer[l+ ptr + 8],1);
           ts[1]='\0';
           DebugP_log("ts is %s\r\n", ts);

           for(int i=0;i <4;i++)
            {
                if(ts[0] == 'D') //Delta
                    ECAP_enableCounterResetOnEvent(ecap_base, ECAP_EVENT_1 + i);
                else //Absolute
                    ECAP_disableCounterResetOnEvent(ecap_base, ECAP_EVENT_1 + i);
            }
        }

        /* TODO : add description to the command*/
        else if (startsWith((char *)gCmdRxBuffer, "sync with epwm"))
        {
            char epwminstance[3];
            int l = strlen("sync with epwm ");
            memcpy(epwminstance, &gCmdRxBuffer[l],2);
            epwminstance[2]='\0';
            epwm = atoi(epwminstance);

            char outChannel;
            memcpy(&outChannel, &gCmdRxBuffer[l + 3],1);
            out = (int)(outChannel - 'A' +1);

            uint32_t ecap_base=CSL_CONTROLSS_ECAP0_U_BASE + ecap[epwm][out] *0x1000;
            int epwm_offset = 8; // EPWM epwmoffset on tester syncs the ECAP on tester

            DebugP_log("%x %d %d %d\r\n", ecap_base, epwm, out, ecap[epwm][out]);

            SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 4, 0, INPUT_XBAR_GPIO120, 0); //Physical connection made from gpio on DUT to tester
            ECAP_setSyncInPulseSource(ecap_base, ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 + epwm_offset);

            uint16_t duty_cycle_arbitrary = 50;
            uint16_t period_arbitrary = 50;
            SyncEPWM(epwm_offset, (Float32)duty_cycle_arbitrary, period_arbitrary);
        }

        /* Aims to generate PWM on EWPM0 at tester with given duty cycle and period */
        else if (startsWith((char *)gCmdRxBuffer, "generate pwm with duty cycle"))
        {
            /* generate pwm with duty cycle 00 period 00000 */
            char duty_cycle_str[3]; /*2 digit decimal duty cycle */
            char period_str[6]; /* maximum period is 16bit unsigned integer and is 5 digits long in decimal */

            int length_till_duty_cycle = strlen("generate pwm with duty cycle ");
            int length_till_period = strlen("generate pwm with duty cycle XX period ");

            memcpy(duty_cycle_str, &gCmdRxBuffer[length_till_duty_cycle], 2);
            memcpy(period_str, &gCmdRxBuffer[length_till_period], 5);

            duty_cycle_str[2] = '\0';
            period_str[5] = '\0';

            Float32 duty_cycle = (Float32) atoi(duty_cycle_str);
            uint16_t period = atoi(period_str);

            SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 4, 0, INPUT_XBAR_GPIO120, 0); //Physical connection made from gpio on DUT to tester

            /* using EPWM 13 as PWM source.*/
            uint32_t epwm_offset = 13;
            // uint32_t epwm_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + epwm_offset * 0x1000;
            // EPWM_setSyncInPulseSource(epwm_base, EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT4);

            /* setup epwm and run the counter */
            SyncEPWM(epwm_offset, (Float32) duty_cycle, period);
        }
        /* halts the pwm input being generated by above command */
        else if (startsWith((char *)gCmdRxBuffer, "halt pwm input"))
        {
            /* stop epwm */
            util_deinit_epwms(13);
        }

        /* TODO : add description to the command*/
        else if (startsWith((char *)gCmdRxBuffer, "get timestamp"))
        {
            int8_t l = strlen("get timestamp ");
            char epwminstance[3];
            memcpy(epwminstance, &gCmdRxBuffer[l],2);
            epwminstance[2]='\0';
            epwm = atoi(epwminstance);

            char outChannel;
            memcpy(&outChannel, &gCmdRxBuffer[l + 3],1);
            out = (int)(outChannel - 'A' +1);

            uint8_t value = ecap[epwm][out];
            DebugP_log("%d\r\n", value);
            uint32_t ecap_base=CSL_CONTROLSS_ECAP0_U_BASE + value *0x1000;
            /*uint32_t timestamp_event1_A, timestamp_event2_A, timestamp_event3_A, timestamp_event4_A;

            timestamp_event1_A = ECAP_getEventTimeStamp(ecap_base, ECAP_EVENT_1);
            timestamp_event2_A = ECAP_getEventTimeStamp(ecap_base, ECAP_EVENT_2);
            timestamp_event3_A = ECAP_getEventTimeStamp(ecap_base, ECAP_EVENT_3);
            timestamp_event4_A = ECAP_getEventTimeStamp(ecap_base, ECAP_EVENT_4);
            */
            DebugP_log("Timestamp for epwm %u and channel is %u \n ",epwm, out);
            char str[10] ;
            char result[CMD_SIZE]="";
            /* Send command*/
            for(uint8_t i=0; i<4; i++)
            {
                uint32_t ecap_ts = ECAP_getEventTimeStamp(ecap_base, ECAP_EVENT_1+i);
                DebugP_log("%u ",ecap_ts);
                sprintf(str, "%u,", ecap_ts);
                strcat(result, str);
            }
            DebugP_log("\r\n");
            gNumBytesWritten = 0U;
            trans.buf   = &gCmdTxBuffer[0U];
            strncpy(trans.buf,result, strlen(result));
            trans.count = CMD_SIZE;
            transferOK = UART_write(gUartHandle[command_uart], &trans);
            //DebugP_log("%u %u %u %u\r\n", timestamp_event1_A, timestamp_event2_A, timestamp_event3_A, timestamp_event4_A);
            //util_deinit_epwms(0);

            /* No need for ACK to be sent*/
            send_ack = false;
        }

        /* sends trigger on GPIO 23*/
        else if (startsWith((char *)gCmdRxBuffer, "trigger on GPIO 24 for InputXbar[5] to trigger ADC"))
        {
            DebugP_log("triggering\r\n");
            GPIO_setDirMode(GPIO_23_LOOPBACK1_BASE_ADDR, GPIO_23_LOOPBACK1_PIN, GPIO_DIRECTION_OUTPUT);
            GPIO_setDirMode(GPIO_24_LOOPBACK1_BASE_ADDR, GPIO_24_LOOPBACK1_PIN, GPIO_DIRECTION_INPUT);

            /* sending high and low with a pulse of 1000 uSec long*/
            GPIO_pinWriteHigh(GPIO_23_LOOPBACK1_BASE_ADDR, GPIO_23_LOOPBACK1_PIN);
            if(GPIO_PIN_HIGH != GPIO_pinRead(GPIO_24_LOOPBACK1_BASE_ADDR, GPIO_24_LOOPBACK1_PIN))
            {
                DebugP_log("ERROR : LoopBack failed at tester, expected HIGH \r\n");
            }
            ClockP_usleep(1);
            GPIO_pinWriteLow(GPIO_23_LOOPBACK1_BASE_ADDR, GPIO_23_LOOPBACK1_PIN);
            if(GPIO_PIN_LOW != GPIO_pinRead(GPIO_24_LOOPBACK1_BASE_ADDR, GPIO_24_LOOPBACK1_PIN))
            {
                DebugP_log("ERROR : LoopBack failed at tester, expected LOW \r\n");
            }

        }
        /* Send ACK by default unless any purticular command requests not to.*/
        if (send_ack == true)
        {
            /* Send ack*/
            gNumBytesWritten = 0U;
            trans.buf   = &gCmdTxBuffer[0U];
            strncpy(trans.buf,"Ack                            .", RSP_SIZE);
            trans.count = RSP_SIZE;
            transferOK = UART_write(gUartHandle[command_uart], &trans);
            /* Quickly read command. No debug prints here*/
        }

    }

    // UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/*
 * Reads Command on UART : command_uart
 */
void set_command_source(void)
{
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = 1;
    transferOK = UART_read(gUartHandle[CONFIG_UART0], &trans);

    if(gCmdRxBuffer[0]=='0')
    {
        command_uart = CONFIG_UART0;
        DebugP_log("Tester uses CONFIG_UART0 (To XDS UART) for receiving commands\r\n");
    }
    if(gCmdRxBuffer[0]=='1')
    {
        command_uart = TESTER_UART;
        DebugP_log("Tester uses TESTER_UART (To DUT UART) for receiving commands\r\n");
    }
}

/*
 * reads Commands from command_uart and stores in gCmdRxBuffer
 */
void read_uart_command(void)
{
/* Quickly wait to read command. No debug prints here*/
    gNumBytesRead = 0U;
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = CMD_SIZE;
    transferOK = UART_read(gUartHandle[command_uart], &trans);

    DebugP_log("Received command!! (%d %d) : ", transferOK, trans.status);

    for(uint8_t index = 0; index < CMD_SIZE; index++)
    {
        // try DebugP_log("%.CMD_SIZEs\r\n");
        DebugP_log("%c", gCmdRxBuffer[index]);
    }
    DebugP_log("\r\n");
}

void Util_GPIO_init(void)
{   /* EPWM0 SYNCIN PULSE PIN*/
    GPIO_setDirMode(SYNC_EPWM0_BASE_ADDR, SYNC_EPWM0_PIN, 1);   // GPIO 70

    /* TODO : ADD THE OTHER CONNECTIONS : DONE*/
    GPIO_setDirMode(GPIO_16_BASE_ADDR, GPIO_16_PIN, 1);                         // GPIO 16
    GPIO_setDirMode(GPIO_17_BASE_ADDR, GPIO_17_PIN, 1);                         // GPIO 17
    GPIO_setDirMode(GPIO_18_BASE_ADDR, GPIO_18_PIN, 1);                         // GPIO 18
    GPIO_setDirMode(GPIO_23_LOOPBACK1_BASE_ADDR, GPIO_23_LOOPBACK1_PIN, 0);     // GPIO 23        USED FOR LOOPBACK1 OUTPUT
    GPIO_setDirMode(GPIO_24_LOOPBACK1_BASE_ADDR, GPIO_24_LOOPBACK1_PIN, 1);     // GPIO 124       USED FOR LOOPBACK1 INPUT


    /* GPIO*/
    /*TODO : PWM PINS - DONE*/
    GPIO_setDirMode(CAPTURE_EPWM0A_BASE_ADDR, CAPTURE_EPWM0A_PIN, 1);   // GPIO 49
    GPIO_setDirMode(CAPTURE_EPWM0B_BASE_ADDR, CAPTURE_EPWM0B_PIN, 1);   // GPIO 51
    GPIO_setDirMode(CAPTURE_EPWM1A_BASE_ADDR, CAPTURE_EPWM1A_PIN, 1);   // GPIO 53
    GPIO_setDirMode(CAPTURE_EPWM1B_BASE_ADDR, CAPTURE_EPWM1B_PIN, 1);   // GPIO 55
    GPIO_setDirMode(CAPTURE_EPWM2A_BASE_ADDR, CAPTURE_EPWM2A_PIN, 1);   // GPIO 50
    GPIO_setDirMode(CAPTURE_EPWM2B_BASE_ADDR, CAPTURE_EPWM2B_PIN, 1);   // GPIO 52
    GPIO_setDirMode(CAPTURE_EPWM3A_BASE_ADDR, CAPTURE_EPWM3A_PIN, 1);   // GPIO 54
    GPIO_setDirMode(CAPTURE_EPWM3B_BASE_ADDR, CAPTURE_EPWM3B_PIN, 1);   // GPIO 56
    GPIO_setDirMode(CAPTURE_EPWM4A_BASE_ADDR, CAPTURE_EPWM4A_PIN, 1);   // GPIO 57
    GPIO_setDirMode(CAPTURE_EPWM4B_BASE_ADDR, CAPTURE_EPWM4B_PIN, 1);   // GPIO 59
    GPIO_setDirMode(CAPTURE_EPWM5A_BASE_ADDR, CAPTURE_EPWM5A_PIN, 1);   // GPIO 61
    GPIO_setDirMode(CAPTURE_EPWM5B_BASE_ADDR, CAPTURE_EPWM5B_PIN, 1);   // GPIO 63
    GPIO_setDirMode(CAPTURE_EPWM6A_BASE_ADDR, CAPTURE_EPWM6A_PIN, 1);   // GPIO 58
    GPIO_setDirMode(CAPTURE_EPWM6B_BASE_ADDR, CAPTURE_EPWM6B_PIN, 1);   // GPIO 60
    GPIO_setDirMode(CAPTURE_EPWM7A_BASE_ADDR, CAPTURE_EPWM7A_PIN, 1);   // GPIO 62
    GPIO_setDirMode(CAPTURE_EPWM7B_BASE_ADDR, CAPTURE_EPWM7B_PIN, 1);   // GPIO 64

    // GPIO_setDirMode(CAPTURE_EPWM11A_BASE_ADDR, CAPTURE_EPWM11A_PIN, 1); // GPIO 65              USED FOR ADC VOLTAGE GENERATION

    GPIO_setDirMode(CAPTURE_EPWM13A_BASE_ADDR, CAPTURE_EPWM13A_PIN, 1); // GPIO 69
    GPIO_setDirMode(CAPTURE_EPWM13B_BASE_ADDR, CAPTURE_EPWM13B_PIN, 1); // GPIO 70

    // GPIO_setDirMode(CAPTURE_EPWM14A_BASE_ADDR, CAPTURE_EPWM14A_PIN, 1); // GPIO 71              DOESNT WORK PROPERLY.

    GPIO_setDirMode(CAPTURE_EPWM14B_BASE_ADDR, CAPTURE_EPWM14B_PIN, 1); // GPIO 72           DEMUX
    GPIO_setDirMode(CAPTURE_EPWM15A_BASE_ADDR, CAPTURE_EPWM15A_PIN, 1); // GPIO 73
    GPIO_setDirMode(CAPTURE_EPWM15B_BASE_ADDR, CAPTURE_EPWM15B_PIN, 1); // GPIO 74
    GPIO_setDirMode(CAPTURE_EPWM15B_BASE_ADDR, CAPTURE_EPWM16A_PIN, 1); // GPIO 75
    GPIO_setDirMode(CAPTURE_EPWM15B_BASE_ADDR, CAPTURE_EPWM16B_PIN, 1); // GPIO 76

    // TODO ADD EPWM16A EPWM16B PINS

    // GPIO_setDirMode(CAPTURE_EPWM17A_BASE_ADDR, CAPTURE_EPWM17A_PIN, 1); // GPIO 77      DNI
    // GPIO_setDirMode(CAPTURE_EPWM17B_BASE_ADDR, CAPTURE_EPWM17B_PIN, 1); // GPIO 78      DNI
    // GPIO_setDirMode(CAPTURE_EPWM18A_BASE_ADDR, CAPTURE_EPWM18A_PIN, 1); // GPIO 79      DNI
    // GPIO_setDirMode(CAPTURE_EPWM18B_BASE_ADDR, CAPTURE_EPWM18B_PIN, 1); // GPIO 80      DNI
    // GPIO_setDirMode(CAPTURE_EPWM19A_BASE_ADDR, CAPTURE_EPWM19A_PIN, 1); // GPIO 81      DNI
    // GPIO_setDirMode(CAPTURE_EPWM19B_BASE_ADDR, CAPTURE_EPWM19B_PIN, 1); // GPIO 82      DNI
    // GPIO_setDirMode(CAPTURE_EPWM20A_BASE_ADDR, CAPTURE_EPWM20A_PIN, 1); // GPIO 83      DNI
    // GPIO_setDirMode(CAPTURE_EPWM20B_BASE_ADDR, CAPTURE_EPWM20B_PIN, 1); // GPIO 84      DNI
    // GPIO_setDirMode(CAPTURE_EPWM21A_BASE_ADDR, CAPTURE_EPWM21A_PIN, 1); // GPIO 85      DNI
    // GPIO_setDirMode(CAPTURE_EPWM21B_BASE_ADDR, CAPTURE_EPWM21B_PIN, 1); // GPIO 86      DNI

    GPIO_setDirMode(CAPTURE_EPWM22A_BASE_ADDR, CAPTURE_EPWM22A_PIN, 1); // GPIO 87      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM22B_BASE_ADDR, CAPTURE_EPWM22B_PIN, 1); // GPIO 88      USED FOR ADC VOLTAGE GENERATION
    GPIO_setDirMode(CAPTURE_EPWM23A_BASE_ADDR, CAPTURE_EPWM23A_PIN, 1); // GPIO 89
    GPIO_setDirMode(CAPTURE_EPWM23B_BASE_ADDR, CAPTURE_EPWM23B_PIN, 1); // GPIO 90      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM24A_BASE_ADDR, CAPTURE_EPWM24A_PIN, 1); // GPIO 91      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM24B_BASE_ADDR, CAPTURE_EPWM24B_PIN, 1); // GPIO 92      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM25A_BASE_ADDR, CAPTURE_EPWM25A_PIN, 1); // GPIO 93      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM25B_BASE_ADDR, CAPTURE_EPWM25B_PIN, 1); // GPIO 94      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM26A_BASE_ADDR, CAPTURE_EPWM26A_PIN, 1); // GPIO 95      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM26B_BASE_ADDR, CAPTURE_EPWM26B_PIN, 1); // GPIO 96      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM27A_BASE_ADDR, CAPTURE_EPWM27A_PIN, 1); // GPIO 97      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM27B_BASE_ADDR, CAPTURE_EPWM27B_PIN, 1); // GPIO 98      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM28A_BASE_ADDR, CAPTURE_EPWM28A_PIN, 1); // GPIO 99      DEMUX
    GPIO_setDirMode(CAPTURE_EPWM28B_BASE_ADDR, CAPTURE_EPWM28B_PIN, 1); // GPIO 100     DEMUX
    GPIO_setDirMode(CAPTURE_EPWM29A_BASE_ADDR, CAPTURE_EPWM29A_PIN, 1); // GPIO 101     DEMUX
    GPIO_setDirMode(CAPTURE_EPWM29B_BASE_ADDR, CAPTURE_EPWM29B_PIN, 1); // GPIO 102     DEMUX
    GPIO_setDirMode(CAPTURE_EPWM30A_BASE_ADDR, CAPTURE_EPWM30A_PIN, 1); // GPIO 103     DEMUX
    GPIO_setDirMode(CAPTURE_EPWM30B_BASE_ADDR, CAPTURE_EPWM30B_PIN, 1); // GPIO 104     DEMUX
    GPIO_setDirMode(CAPTURE_EPWM31A_BASE_ADDR, CAPTURE_EPWM31A_PIN, 1); // GPIO 105     DEMUX
    GPIO_setDirMode(CAPTURE_EPWM31B_BASE_ADDR, CAPTURE_EPWM31B_PIN, 1); // GPIO 106     DEMUX
}

int startsWith(char *str, char *substr)
{
    size_t sub_len = strlen(substr);
    return (strncmp(str, substr, sub_len) == 0);
}

/*
 * util_ECAP_init - Configure eCAP
 */
void util_ECAP_init(uint32_t base, uint32_t input)
{

    /* Disable ,clear all capture flags and interrupts */

    ECAP_disableInterrupt(base,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                          ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                          ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                          ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                          ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                          ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                          ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(base,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                        ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                        ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                        ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                        ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                        ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                        ECAP_ISR_SOURCE_COUNTER_COMPARE));


    /* Disable CAP1-CAP4 register loads */

    ECAP_disableTimeStampCapture(base);

   /*
    Configure eCAP
       Enable capture mode.
       One shot mode, stop capture at event 4.
       Set polarity of the events to rising, falling, rising, falling edge.
       Set capture in time difference mode.
       Enable eCAP module.
     */
    ECAP_stopCounter(base);
    ECAP_enableCaptureMode(base);
    ECAP_selectECAPInput(base, input);
    ECAP_resetCounters(base);
    ECAP_startCounter(base);
    ECAP_enableLoadCounter(base);
    ECAP_enableTimeStampCapture(base);
    ECAP_reArm(base);
}


void SyncEPWM(int8_t epwm_offset, Float32 duty_cycle, uint32_t period)
{
    uint32_t epwm_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + epwm_offset * 0x1000;

    EPWM_setSyncInPulseSource(epwm_base, EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT4);
    EPWM_enableSyncOutPulseSource(epwm_base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
    EPWM_enablePhaseShiftLoad(epwm_base);
    EPWM_setPhaseShift(epwm_base, 0x0);

    util_start_pwm_wave(epwm_offset, duty_cycle, period);
}

/*
 * for deinitializing the pwm
 */
void util_deinit_epwms(int8_t epwm_offset)
{
    //Unlock CONTROLSS_CTRL
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + epwm_offset*4, 0x07);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + epwm_offset*4, 0x00);
}

/*
 * sets a period and a duty cycle of given values, to given EPWM offset
 */
void util_start_pwm_wave(uint8_t epwm_offset, Float32 duty_cycle, uint16_t period)
{
    uint32_t base;
    base = CSL_CONTROLSS_G0_EPWM0_U_BASE + (0x1000*epwm_offset);

    SOC_setEpwmTbClk(epwm_offset, TRUE);

    //Enabling pinmux
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);
    uint32_t baseA = CSL_IOMUX_EPWM0_A_CFG_REG + 8*epwm_offset;
    uint32_t baseB = CSL_IOMUX_EPWM0_B_CFG_REG + 8*epwm_offset;

    if(epwm_offset<16)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, 0x500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, 0x500) ;
    }
    else
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, 0x505) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, 0x505) ;
    }

    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod( base, period);
    // EPWM_disablePhaseShiftLoad(base);
    // EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0);

    uint32_t cmp_values = (uint32_t)((((Float32)duty_cycle * (Float32)period)/100)+1);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, cmp_values );
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, cmp_values);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    /* Configuration set. now running timeBase counter*/
    /* Note:
        If the PWM was already running, the change in duty cycle would have been
         set as soon as the CMPA/B values are changed.*/
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    DebugP_log("PWM %d is set for Period : %d, on_time : %d, Duty Cycle : %f\r\n", epwm_offset, period, cmp_values, duty_cycle);
}

/*
    Brown  - Br
    Red    - R
    Orange - O
    Yellow - y
    Green  - G
    Blue   - Bl
    Purple - P
    Grey   - Gy
    White  - W
*/

/* 	            |   DUT (CC E2)		                                |   Interconnecting component	                    |   Tester (CC)                             |
	            |   SOC Pin	                    |   HSEC pin		|                                                   |   HSEC pin    |   SOC Pin                 |
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------
Commands	    |   UART3_TX	                |   71	            |   Pull up resistor 1 to 3.3V, activity LED 1	    |   73	        |   UART3_RX                |
Commands	    |   UART3_RX	                |   73	            |   Pull up resistor 2 to 3.3V, activity LED 2	    |   71	        |   UART3_TX                |
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------
DAC test	    |   DAC_OUT	9	                |   Saleae pin 0    |                                                   |               |                           |
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------
ADC/CMPSS test	|   ADC0_AIN0	                |   9  (DEMUX)      |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC0_AIN1	                |   11 (DEMUX)      |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC0_AIN2	( 1 P )	        |   15              |   Low pass filter 1	                            |   86	        |   EPWM11_A                |
ADC/CMPSS test	|   ADC0_AIN3	( 1 W )         |   17              |   Low pass filter 2	                            |   110	        |   EPWM22_B                |
ADC/CMPSS test	|   ADC0_AIN4	( 2 Br)         |   21              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC0_AIN5	( 2 Or)         |   23              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC1_AIN0	( 1 Br)         |   12              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC1_AIN1	( 1 Or)         |   14              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC1_AIN2	( 1 P )         |   18              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC1_AIN3	( 1 W )         |   20              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC1_AIN4	( 2 Br)         |   24              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC1_AIN5	( 2 Or)         |   26              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC2_AIN0	( 3 Br)         |   31              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC2_AIN1	( 3 Or)         |   33              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC2_AIN2	( 3 Gr)         |   37              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC2_AIN3	( 3 P )         |   39              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC3_AIN0	( 2 Gr)         |   28              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC3_AIN1	( 2 P )         |   30              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC3_AIN2	( 3 Br)         |   34              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC3_AIN3	( 3 Or)         |   36              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC3_AIN4	( 3 Gr)         |   40              |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC3_AIN5	( 3 P )         |   42              |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC4_AIN0	( 2 Gr)         |   25  (DEMUX)     |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC4_AIN1	( 2 P )         |   27  (DEMUX)     |   Low pass filter 2                               |               |                           |
ADC/CMPSS test	|   ADC_CAL0	                |   25  (DEMUX)     |   Low pass filter 1                               |               |                           |
ADC/CMPSS test	|   ADC_CAL1	                |   27  (DEMUX)     |   Low pass filter 2                               |               |                           |
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------
SDFM test	    |   SDFM0_D0	                |   91	            |   AMC1304 EVM Data Out	                        |   9	        |   DAC_OUT                 |
SDFM test	    |   SDFM0_CLK0	                |   72	            |   AMC1304 EVM Clock Out	                        |   9	        |   DAC_OUT                 |
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------
	            |   PR0_PRU1_ GPIO18/GPIO120	|   70		        |                                                   |   70	        |                           |   USED FOR EPWMSYNC IN PULSE
	            |   SPI0_CS0/GPIO11	            |   73		        |                                                   |   73	        |   SPI0_CS0/GPIO11	        |   USED FOR UART
	            |   SPI1_D0/XBAROUT3/GPIO17	    |   75		        |                                                   |   75	        |   SPI1_D0/XBAROUT3/GPIO17 |
	            |   SPI1_D1/XBAROUT4/GPIO18	    |   77		        |                                                   |   77	        |   SPI1_D1/XBAROUT4/GPIO18 |
	            |   SPI1_CLK/XBAROUT2/GPIO16	|   79		        |                                                   |   79	        |   SPI1_CLK/XBAROUT2/GPIO16|
	            |   I2C1_SDA/XBAROUT8/GPIO24	|   85	Loopback	|                                                   |   85	        |   I2C1_SDA/XBAROUT8/GPIO24|   USED FOR LOOPBACK1
	            |   I2C1_SCL/XBAROUT7/GPIO23	|   87	Loopback	|                                                   |   87	        |   I2C1_SCL/XBAROUT7/GPIO23|   USED FOR LOOPBACK1
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------
EPWM test	    |   EPWM0_A	   ( 1 R )          |   49              |                                                   |   49	        |   GPIO43                  |
EPWM test	    |   EPWM0_B	   ( 1 Y )          |   51 		        |                                                   |   51	        |   GPIO44                  |
EPWM test	    |   EPWM1_A	   ( 1 Bl)          |   53 		        |                                                   |   53	        |   GPIO45                  |
EPWM test	    |   EPWM1_B	   ( 1 Gy)          |   55 		        |                                                   |   55	        |   GPIO46                  |
EPWM test	    |   EPWM2_A	   ( 1 R )          |   50 		        |                                                   |   50	        |   GPIO47                  |
EPWM test	    |   EPWM2_B	   ( 1 y )          |   52 		        |                                                   |   52	        |   GPIO48                  |
EPWM test	    |   EPWM3_A	   ( 1 Bl)          |   54 		        |                                                   |   54	        |   GPIO49                  |
EPWM test	    |   EPWM3_B	   ( 1 Gy)          |   56 		        |                                                   |   56	        |   GPIO50                  |
EPWM test	    |   EPWM4_A	   ( 1 B )          |   57 		        |                                                   |   57	        |   GPIO51                  |
EPWM test	    |   EPWM4_B	   ( 2 R )          |   59 		        |                                                   |   59	        |   GPIO52                  |
EPWM test	    |   EPWM5_A	   ( 2 Y )          |   61 		        |                                                   |   61	        |   GPIO53                  |
EPWM test	    |   EPWM5_B	   ( 2 Bl)          |   63 		        |                                                   |   63	        |   GPIO54                  |
EPWM test	    |   EPWM6_A	   ( 1 B )          |   58 		        |                                                   |   58	        |   GPIO55                  |
EPWM test	    |   EPWM6_B	   ( 2 R )          |   60 		        |                                                   |   60	        |   GPIO56                  |
EPWM test	    |   EPWM7_A	   ( 2 Y )          |   62 		        |                                                   |   62	        |   GPIO57                  |
EPWM test	    |   EPWM7_B	   ( 2 Bl)          |   64 		        |                                                   |   64	        |   GPIO58                  |
EPWM test	    |   EPWM11_A   ( 1 Or)          |   86		        |                                                   |   86	        |   GPIO65                  |   USED FOR ADC VOLTAGE GENERATION
EPWM test	    |   EPWM13_A   ( 4 G )          |   153		        |                                                   |   153	        |   GPIO69                  |
EPWM test	    |   EPWM13_B   ( 4 G )          |   154		        |                                                   |   154	        |   GPIO70                  |
EPWM test	    |   EPWM14_A   ( 7 Bl)          |   155		        |             ----DO NOT USE. DOESNT WORK----       |   155	        |   GPIO71                  |   ----DO NOT USE. DOESNT WORK PROPERLY----
EPWM test	    |   EPWM14_B   ( 7 Bl)          |   156 (DEMUX)		|                                                   |   156 (DEMUX)	|   GPIO72                  |
EPWM test	    |   EPWM15_A   ( 4 P )          |   159		        |                                                   |   159	        |   GPIO73                  |
EPWM test	    |   EPWM15_B   ( 4 P )          |   160		        |                                                   |   160	        |   GPIO74                  |
EPWM test	    |   EPWM16_A   ( 7 Gy)          |   161		        |                                                   |   161	        |   GPIO75                  |
EPWM test	    |   EPWM16_B                    |   162		        |                                                   |   162	        |   GPIO76                  |
EPWM test	    |   EPWM22_A                    |   108(DEMUX)		|                                                   |   108 (DEMUX)	|   GPIO87                  |
EPWM test	    |   EPWM22_B                    |   110		        |                                                   |   110	        |   GPIO88                  |   USED FOR ADC VOLTAGE GENERATION
EPWM test	    |   EPWM23_A                    |   121		        |                                                   |   121	        |   GPIO89                  |
EPWM test	    |   EPWM23_B                    |   122(DEMUX)		|                                                   |   122(DEMUX)	|   GPIO90                  |
EPWM test	    |   EPWM24_A                    |   123(DEMUX)		|                                                   |   123(DEMUX)	|   GPIO91                  |
EPWM test	    |   EPWM24_B                    |   124(DEMUX)		|                                                   |   124(DEMUX)	|   GPIO92                  |
EPWM test	    |   EPWM25_A                    |   125(DEMUX)		|                                                   |   125(DEMUX)	|   GPIO93                  |
EPWM test	    |   EPWM25_B                    |   126(DEMUX)		|                                                   |   126(DEMUX)	|   GPIO94                  |
EPWM test	    |   EPWM26_A                    |   127(DEMUX)		|                                                   |   127(DEMUX)	|   GPIO95                  |
EPWM test	    |   EPWM26_B                    |   128(DEMUX)		|                                                   |   128(DEMUX)	|   GPIO96                  |
EPWM test	    |   EPWM27_A                    |   129(DEMUX)		|                                                   |   129(DEMUX)	|   GPIO97                  |
EPWM test	    |   EWPW27_B                    |   130(DEMUX)		|                                                   |   130(DEMUX)	|   GPIO98                  |
EPWM test	    |   EPWM28_A                    |   131(DEMUX		|                                                   |   131(DEMUX	|   GPIO99                  |
EPWM test	    |   EPWM28_B                    |   132(DEMUX)		|                                                   |   132(DEMUX)	|   GPIO100                 |
EPWM test	    |   EPWM29_A	                |   133(DEMUX)		|                                                   |   133(DEMUX)	|   GPIO101                 |
EPWM test	    |   EPWM29_B	                |   134(DEMUX)		|                                                   |   134(DEMUX)	|   GPIO102                 |
EPWM test	    |   EPWM30_A	                |   137(DEMUX)		|                                                   |   137(DEMUX)	|   GPIO103                 |
EPWM test	    |   EPWM30_B	                |   138(DEMUX)		|                                                   |   138(DEMUX)	|   GPIO104                 |
EPWM test	    |   EPWM31_A	                |   139(DEMUX)		|                                                   |   139(DEMUX)	|   GPIO105                 |
EPWM test	    |   EPWM31_B	                |   140(DEMUX)		|                                                   |   140(DEMUX)	|   GPIO106                 |
----------------|-------------------------------|-------------------|---------------------------------------------------|---------------|---------------------------|--------------------------------   */
