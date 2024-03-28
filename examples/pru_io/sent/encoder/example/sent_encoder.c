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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/pruicss/m_v0/pruicss.h>
#include <drivers/pinmux/am263x/pinmux.h>
#include <firmware/sent_encoder_pru_bin.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 *  Value to be set in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers
 *  when using pins in GPIO mode.
 *  1 - Input
 *  0- Output
 **/
#define MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE  (0x0)

#define TX_DONE_OFFSET              (0x4U)
#define SC_NIBBLE_OFFSET            (0x8U)
#define DATA0_OFFSET                (0x9U)
#define DATA1_OFFSET                (0xAU)
#define DATA2_OFFSET                (0xBU)
#define DATA3_OFFSET                (0xCU)
#define DATA4_OFFSET                (0xDU)
#define DATA5_OFFSET                (0xEU)
#define CRC_OFFSET                  (0xFU)

#define	SENT_DATA_AND_CRC_LENGTH	(7)
#define SHORT_SERIAL_MSG_LENGTH     (16)

#define SLEEP_USECS                 (100000)

#define SENT_CMD_SINGLE_FRAME                   (0)
#define SENT_CMD_MULTIPLE_FRAMES                (1)
#define SENT_CMD_SINGLE_SHORT_SERIAL_MESSAGE    (2)
#define SENT_CMD_MULTIPLE_SHORT_SERIAL_MESSAGES (3)
#define SENT_CMD_EXIT_APP                       (4)
#define SENT_CMD_MAX                            (5)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

uint8_t dataAndCrcArray[SENT_DATA_AND_CRC_LENGTH] = {0x7, 0x4, 0x8, 0x7, 0x4, 0x8, 0x5};

uint8_t scDataArray[SHORT_SERIAL_MSG_LENGTH] = {
    0x8, 0x4, 0x4, 0x4 , /* Data 7*/
    0x0, 0x4, 0x0, 0x0 , /* Data 4*/
    0x4, 0x0, 0x0, 0x0 , /* Data 8*/
    0x4, 0x4, 0x4, 0x0 , /* CRC  E*/
};

Pinmux_PerCfg_t gPinMuxMainDomainCfg_1[] = {
            /* ICSSM_IEP pin config */

    /* PR0_PRU0_GPIO0 -> PR0_PRU0_GPIO0 (K17) */
    {
        PIN_PR0_PRU0_GPIO0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO1 -> PR0_PRU0_GPIO1 (K18) */
    {
        PIN_PR0_PRU0_GPIO1,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO2 -> PR0_PRU0_GPIO2 (J18) */
    {
        PIN_PR0_PRU0_GPIO2,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO3 -> PR0_PRU0_GPIO3 (J17) */
    {
        PIN_PR0_PRU0_GPIO3,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO4 -> PR0_PRU0_GPIO4 (K16) */
    {
        PIN_PR0_PRU0_GPIO4,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO5 -> PR0_PRU0_GPIO5 (G17) */
    {
        PIN_PR0_PRU0_GPIO5,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO6 -> PR0_PRU0_GPIO6 (K15) */
    {
        PIN_PR0_PRU0_GPIO6,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO8 -> PR0_PRU0_GPIO8 (G15) */
    {
        PIN_PR0_PRU0_GPIO8,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

void display_menu();
void get_command(uint32_t *cmd, uint32_t *count);

/**
 * @brief   Sends info to firmware about number of SENT frames to be SENT
 *
 * @param   num
 *
 * @return  void
 */
static void transmit_sent_frame(int num);

/**
 * @brief   Initialises the PRU cores and respective Data RAMs
 *
 * @param   void
 *
 * @return  none
 */
void pruicss_init(void)
{
    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);
    PRUICSS_setGpMuxSelect(gPruIcss0Handle, PRUICSS_PRU0, PRUICSS_GP_MUX_SEL_MODE_GP);

    /*PRU0 Init*/
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);

}

/**
 * @brief   Loads PRU Firmware and enables the core
 *
 * @param   void
 *
 * @return  none
 */
void pruicss_load_run_fw(void)
{
    /*PRU0 Load and Run*/
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU0);

    /*Load firmware. Set buffer = write to Pru memory */
    if(PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU0), 0,
                                     (uint32_t *) SentencoderFirmwarepru,
                                        sizeof(SentencoderFirmwarepru)))
    {
            /*Run firmware */
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU0);

    }
}

/**
 * @brief   Configures I2C controlled SOC mux to configure GPIO mode for ICSS
 *
 * @param   void
 *
 * @return  none
 */
void enable_board_mux(void)
{
    /*Required to configure SOC MUX to select ICSS pins for input. 1- PWM XBAR, 0- ICSS Pins*/
    SOC_selectIcssGpiMux(0, 0x0);

    I2C_Transaction i2cTransaction;
    uint8_t buffer[2];

    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf     = buffer;
    i2cTransaction.writeCount   = 2U;
    i2cTransaction.targetAddress = 0x20;

    /* Configure MDIO sel pin */
    /* Set output to high */
    buffer[0] = 0x02U ; /* Port 0 */
    buffer[1] = (0x03 << 2); /* Pin 2 & 3 */
    I2C_transfer(I2C_getHandle(CONFIG_I2C2), &i2cTransaction);

    /* set pin to output */
    buffer[0] = 0x6U;
    buffer[1] = ~(0x3 << 2);
    I2C_transfer(I2C_getHandle(CONFIG_I2C2), &i2cTransaction);

}

void display_menu()
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select Option                                    |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 0 : Send 1 message                                                           |");
    DebugP_log("\r\n| 1 : Send n messages                                                          |");
    DebugP_log("\r\n| 2 : Send 16 messages with 1 short serial message                             |");
    DebugP_log("\r\n| 3 : Send 16 messages with 1 short serial message n times                     |");
    DebugP_log("\r\n| 4 : Exit application                                                         |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|\n");
    DebugP_log("\r\n| Enter value: ");
}

void get_command(uint32_t *cmd, uint32_t *count)
{

    /* Check to make sure that the command issued is correct */
    if(DebugP_scanf("%d\n", cmd) < 0)
    {
        *cmd = 0;
        DebugP_log("\r\n| WARNING: invalid command, 0 will be selected\n");
    }

    /* Check to make sure that the command issued is correct */
    if(*cmd >= SENT_CMD_MAX)
    {
        *cmd = 0;
        DebugP_log("\r\n| WARNING: invalid command, 0 will be selected\n");
    }   

    if(*cmd == SENT_CMD_MULTIPLE_FRAMES || *cmd == SENT_CMD_MULTIPLE_SHORT_SERIAL_MESSAGES)
    {
        DebugP_log("\r\n| Enter count : ");

        if(DebugP_scanf("%d\n", count) < 0)
        {
            *count = 1;
            DebugP_log("\r\n| WARNING: invalid count, 1 will be selected\n");
        } 
    }
}

void encoder_main(void *args)
{
    int                 status;
    volatile uint8_t    tx_ok = 1;
    uint32_t            total_frames_tx = 0;
    uint8_t             sc_data_index = 0;
    uint32_t            cmd;
    uint32_t            count;
    uint32_t            loop;
    
    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    Pinmux_config(gPinMuxMainDomainCfg_1, PINMUX_DOMAIN_ID_MAIN);
    DebugP_log("Pin Mux done!!\r\n");

    /* Set bits for input pins in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers */
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU0_GPIO_OUT_CTRL, MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE);
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU1_GPIO_OUT_CTRL, MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE);

    enable_board_mux();

    pruicss_init();
    pruicss_load_run_fw();

    DebugP_log("\n\r******************SENT ENCODER Application**********************\n\r");
    DebugP_log("\r\nSENT PRU-ICSS firmware loaded and running\n\r");

    uint32_t *frame_tx_base = ((uint32_t *)((((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->pru0DramBase) + 0x0));

	HW_WR_REG8((uint32_t)frame_tx_base + DATA0_OFFSET, dataAndCrcArray[0]);
	HW_WR_REG8((uint32_t)frame_tx_base + DATA1_OFFSET, dataAndCrcArray[1]);
	HW_WR_REG8((uint32_t)frame_tx_base + DATA2_OFFSET, dataAndCrcArray[2]);
	HW_WR_REG8((uint32_t)frame_tx_base + DATA3_OFFSET, dataAndCrcArray[3]);
	HW_WR_REG8((uint32_t)frame_tx_base + DATA4_OFFSET, dataAndCrcArray[4]);
	HW_WR_REG8((uint32_t)frame_tx_base + DATA5_OFFSET, dataAndCrcArray[5]);
	HW_WR_REG8((uint32_t)frame_tx_base + CRC_OFFSET, dataAndCrcArray[6]);

    while(1)
    {
        display_menu();
        count = 1;
        get_command(&cmd, &count);

        if((cmd == SENT_CMD_SINGLE_FRAME) || (cmd == SENT_CMD_MULTIPLE_FRAMES))
        {
            for(loop = 0; loop < count; loop++)
            {
                /*Call Frame Send API*/
                transmit_sent_frame(1);
                /*Wait for frame to be transmitted*/
                while(tx_ok)
                {
                    tx_ok = HW_RD_REG8((uint32_t)frame_tx_base + TX_DONE_OFFSET);
                }
                ClockP_usleep(SLEEP_USECS);
            }

            DebugP_log("\n\r%d SENT Frame(s) transmitted\n\r", count);
            total_frames_tx += count ;
        }   
        else if((cmd == SENT_CMD_SINGLE_SHORT_SERIAL_MESSAGE) || (cmd == SENT_CMD_MULTIPLE_SHORT_SERIAL_MESSAGES))
        {

            for(loop = 0; loop < count; loop++)
            {
                for(sc_data_index = 0; sc_data_index < SHORT_SERIAL_MSG_LENGTH; sc_data_index++)
                {
                    HW_WR_REG8((uint32_t)frame_tx_base + SC_NIBBLE_OFFSET, scDataArray[sc_data_index]);
                    /*Call Frame Send API*/
                    transmit_sent_frame(1);
                    /*Wait for frame to be transmitted*/
                    while(tx_ok)
                    {
                        tx_ok = HW_RD_REG8((uint32_t)frame_tx_base + TX_DONE_OFFSET);
                    }
                    ClockP_usleep(SLEEP_USECS);
                }
            }

            DebugP_log("\n\r%d SENT Frames transmitted\n\r", count*16);
            total_frames_tx += count*16;
        }            
        else if(cmd == SENT_CMD_EXIT_APP)
        {
            DebugP_log("\n\rExiting Application\n\r");
            Board_driversClose();
            Drivers_close();
        }
        else
        {
            DebugP_log("\n\rSorry, did not catch that. Unknown input\n\r");
            DebugP_log("\n\r");
        }
        DebugP_log("\n\rNumber of Frames transmitted: \t %d\n\r", total_frames_tx);
    }
}

static void transmit_sent_frame(int num)
{
    uint32_t *encoder_base = ((uint32_t *)((((PRUICSS_HwAttrs const *)(gPruIcss0Handle->hwAttrs))->pru0DramBase) + 0x0));
    memcpy(encoder_base, &num, sizeof(num));

}
