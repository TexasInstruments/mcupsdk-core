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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>

#include <drivers/sciclient.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <motor_control/position_sense/endat/include/endat_drv.h>
#include <motor_control/position_sense/endat/firmware/endat_master_bin.h>
#include <motor_control/position_sense/endat/firmware/endat_master_multi_bin.h>

#define TASK_STACK_SIZE (4096)
#define TASK_PRIORITY   (6)

#define ENDAT_RX_SAMPLE_SIZE    7
#define ENDAT_RX_SESQUI_DIV (1 << 15)

#define MRS_POS_VAL2_WORD1  0x42
#define MRS_POS_VAL2_WORD2  0x43
#define MRS_POS_VAL2_WORD3  0x44

static union endat_format_data gEndat_format_data_mtrctrl[3];
static unsigned gEndat_mtrctrl_crc_err[3];
static unsigned gEndat_2_2_crc_position_err_cnt[3];
static unsigned gEndat_2_2_crc_addinfo1_err_cnt[3];
static unsigned long long gEndat_2_2_pos_val2[3];
static int gEndat_2_2_loop_mrs;
static void (*endat_fn_position_loop)(unsigned int);

uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

#define VALID_HOST_CMD(x) ((x == 100) || ((x) == 101) || ((x) == 102) || ((x) == 103) || ((x) == 104) || ((x) == 105) || \
                           ((x) == 106) || ((x) == 107) || ((x) == 108) || ((x) == 109))

#define HAVE_COMMAND_SUPPLEMENT(x) (((x) == 2) || ((x) == 3) || ((x) == 4) || ((x) == 7) || \
                                    ((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 13) || ((x) == 14) || \
                                    ((x) == 100) || ((x) == 101) || ((x)== 103) || ((x) == 105) || ((x) == 106) || ((x) == 107) || ((x) == 108) || ((x) == 109))

#define ENDAT_INPUT_CLOCK_UART_FREQUENCY 192000000
/* use uart clock only to start with */
#define ENDAT_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY

#define ENDAT_POSITION_LOOP_STOP    0
#define ENDAT_POSITION_LOOP_START   1

union position
{
    float angle;
    unsigned long long length;
};

struct endat_priv *priv;

#define PRUSS_PRU1_CTRL_CTPPR1      (CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE + 0x2C)

#define ENDAT_MULTI_CH0 (1 << 0)
#define ENDAT_MULTI_CH1 (1 << 1)
#define ENDAT_MULTI_CH2 (1 << 2)

static int gEndat_is_multi_ch;
static unsigned char gEndat_multi_ch_mask;
static unsigned int gEndat_prop_delay[3];
static unsigned int gEndat_prop_delay_max;

static char gUart_buffer[256];

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

/* buffer to handle long long printf */
char gPrintf_dump_buffer[21];

/* This function is used to convert uint64_t
 * variable tp a string as printf doesn't support
 * printing 64-bit variables
 */
char * uint64_to_str (uint64_t x)
{
    char *b = gPrintf_dump_buffer + sizeof(gPrintf_dump_buffer);
    *(--b) = '\0';
    do
    {
        *(--b) = '0' + (x % 10);
        x /= 10;
    } while (x);

    return b;
}


static void endat_pruss_init(void)
{
    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));

    /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRUx));

    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);
}

void endat_pre_init(void)
{
    endat_pruss_init();
}

void endat_pruss_load_run_fw(void)
{
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);

    /*Load firmware. Set buffer = write to Pru memory */
    if(gEndat_is_multi_ch)
    {
        PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                            0, (uint32_t *) EnDatFirmwareMulti,
                            sizeof(EnDatFirmwareMulti));
    }
    else
    {
        PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                            0, (uint32_t *) EnDatFirmware,
                            sizeof(EnDatFirmware));
    }

    PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
    /*Run firmware */
    PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
}

unsigned long endat_get_fw_version(void)
{
    return *((unsigned long *)EnDatFirmware + 2);
}

static void endat_print_menu(void)
{
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r|              Select value for the encoder command from following             |\n");
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| 1 : Encoder send position values                                             |\n");
    DebugP_log("\r| 2 : Selection of memory area                                                 |\n");
    DebugP_log("\r| 3 : Encoder receive parameter                                                |\n");
    DebugP_log("\r| 4 : Encoder send parameter                                                   |\n");
    DebugP_log("\r| 5 : Encoder receive reset                                                    |\n");
    DebugP_log("\r| 6 : Encoder send test values                                                 |\n");
    DebugP_log("\r| 7 : Encoder receive test command                                             |\n");

    if(priv->cmd_set_2_2)
    {
        DebugP_log("\r| 8 : Encoder to send position + AI(s)                                         |\n");
        DebugP_log("\r| 9 : Encoder to send position + AI(s) and receive selection of memory area    |\n");
        DebugP_log("\r| 10: Encoder to send position + AI(s) and receive parameter                   |\n");
        DebugP_log("\r| 11: Encoder to send position + AI(s) and send parameter                      |\n");
        DebugP_log("\r| 12: Encoder to send position + AI(s) and receive error reset                 |\n");
        DebugP_log("\r| 13: Encoder to send position + AI(s) and receive test command                |\n");
        DebugP_log("\r| 14: Encoder receive communication command                                    |\n");
    }

    DebugP_log("\r|                                                                              |\n");
    DebugP_log("\r|100: Configure clock                                                          |\n");
    DebugP_log("\r|101: Simulate motor control 2.1 position loop                                 |\n");
    DebugP_log("\r|102: Toggle raw data display                                                  |\n");
    DebugP_log("\r|103: Configure tST delay                                                      |\n");

    if(!gEndat_is_multi_ch)
    {
        DebugP_log("\r|104: Start continuous mode                                                    |\n");
    }

    DebugP_log("\r|105: Configure rx arm counter (account tD)                                    |\n");
    DebugP_log("\r|106: Configure rx clock disable time (for tD)                                 |\n");

    if(priv->cmd_set_2_2)
    {
        DebugP_log("\r|107: Simulate motor control 2.2 position loop %s                        |\n",
                    gEndat_is_multi_ch ? "        " : "(safety)");
    }

    DebugP_log("\r|108: Configure propagation delay (tD)                                         |\n");

    if(gEndat_is_multi_ch)
    {
        DebugP_log("\r|109: Configure wire delay                                                     |\n");
    }

    DebugP_log("\r|------------------------------------------------------------------------------|\n\r|\n");
    DebugP_log("\r| enter value: ");
}

static inline int endat_get_command(void)
{
    volatile int cmd = -1;


    if(DebugP_scanf("%d", &cmd) < 0)
    {
        cmd = -1;
    }

    if(VALID_2_1_CMD(cmd) || (priv->cmd_set_2_2 && VALID_2_2_CMD(cmd))
            || VALID_HOST_CMD(cmd))
    {
        return cmd;
    }

    DebugP_log("\r| WARNING: invalid command, EnDat 2.1 send position values command will be sent\n");
    return 1;
}

static void endat_recvd_print(int cmd, struct endat_priv *priv,
                              union endat_format_data *u, int crc)
{
    unsigned addinfo, byte1;
    unsigned long long max = pow(2, priv->single_turn_res);
    union position position;

    /* this would give wrong values if cmd is not position related, but that is okay as then this value won't be used */
    if(priv->type == rotary)
    {
        position.angle = ((float) u->position_addinfo.position.position) /
                         (float)max * (float)360;
    }

    else
    {
        position.length = u->position_addinfo.position.position * priv->step;
    }


    DebugP_log("\r|\n\r|\n");

    switch(cmd)
    {
        case 2:
        case 3:
        case 5:
        case 7:
        case 14:
            DebugP_log("\r| crc: %s\n", crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("| crc: %x\n", u->addr_params.crc);
            break;

        case 4:
            DebugP_log("\r| parameter: 0x%x, crc: %s\n", u->addr_params.params,
                        crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("\r| crc: %x\n", u->addr_params.crc);
            break;

        case 6:
            DebugP_log("\r| test value: 0x%02x%08x, crc: %s\n",
                        (unsigned)((u->test.value & 0xFF00000000) >> 32),
                        (unsigned)(u->test.value & 0xFFFFFFFF), crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("\r| crc: %x\n", u->test.crc);
            break;

        case 1:
            if(priv->multi_turn_res)
            {
                sprintf(gUart_buffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(priv->type == rotary)
                {
                    sprintf(gUart_buffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUart_buffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUart_buffer);

            DebugP_log("f1: %x, crc: %s\n", u->position_addinfo.position.f1,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            if(priv->multi_turn_res)
            {
                sprintf(gUart_buffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(priv->type == rotary)
                {
                    sprintf(gUart_buffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUart_buffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUart_buffer);

            DebugP_log("f1/f2: %x/%x, crc: %s\n", u->position_addinfo.position.f1,
                        u->position_addinfo.position.f2,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);

            if(priv->flags.info1)
            {
                addinfo = u->position_addinfo.addinfo1.addinfo;
                byte1 = (addinfo >> 16) & ((1 << 8) - 1);

                DebugP_log("\r|\n| WRN: %x  RM: %x  Busy: %x  I4-I0: %x\n",
                            (byte1 & ENDAT_STATUS_WARN_MASK) >> ENDAT_STATUS_WARN_SHIFT,
                            (byte1 & ENDAT_STATUS_RM_MASK) >> ENDAT_STATUS_RM_SHIFT,
                            (byte1 & ENDAT_STATUS_BUSY_MASK) >> ENDAT_STATUS_BUSY_SHIFT,
                            byte1 & ENDAT_INFORMATION_MASK);
                DebugP_log("\r|\n| Additional Information 1: 0x%x, crc: %s\n", addinfo,
                            crc & 0x2 ? "success" : "failure");
                DebugP_logInfo("| addinfo1 crc: %x\n",
                                u->position_addinfo.addinfo1.crc);
            }

            if(priv->flags.info2)
            {
                addinfo = u->position_addinfo.addinfo2.addinfo;
                byte1 = (addinfo >> 16) & ((1 << 8) - 1);

                DebugP_log("\r|\n| Additional Information 2: 0x%x, crc: %s\n", addinfo,
                            crc & 0x4 ? "success" : "failure");
                DebugP_logInfo("| addinfo2 crc: %x\n",
                                u->position_addinfo.addinfo2.crc);
            }

            break;

        default:
            DebugP_log("\r|\n| ERROR: print requested for invalid command\n");
            break;
    }

    DebugP_log("\r|\n\r|\n");
}

static void endat_display_raw_data(int cmd, struct endat_priv *priv)
{
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    switch(cmd)
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
            DebugP_log("\r|\n| raw data: %x %x %x %x\n|\n",
                        pruss_xchg->ch[ch].pos_word0, pruss_xchg->ch[ch].pos_word1,
                        pruss_xchg->ch[ch].pos_word2, pruss_xchg->ch[ch].pos_word3);
            break;

        default:
            DebugP_log("\r|\n| Nothing raw to display - this is not a valid command\n|\n");
            break;
    }
}

/*
 * check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that this function will be called after endat_get_command
 */
static int endat_get_command_supplement(int cmd,
                                        struct cmd_supplement *cmd_supplement)
{
    /* erase previous cmd supplements */
    memset(cmd_supplement, 0, sizeof(*cmd_supplement));

    switch(cmd)
    {
        case 2:
            DebugP_log("\r| enter MRS code (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 9:
            DebugP_log("\r| enter MRS code (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address == ENDAT_SECTION2_MEMORY)
            {
                DebugP_log("\r| enter block address (hex value): ");

                if(DebugP_scanf("%x\n", &cmd_supplement->block) < 0)
                {
                    DebugP_log("\r| ERROR: invalid block address\n|\n|\n|\n");
                    return -EINVAL;
                }

                /* better compare it with number of blocks information available in eeprom */
                if(cmd_supplement->block > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid block address\n|\n|\n|\n");
                    return -EINVAL;
                }

                cmd_supplement->has_block_address = TRUE;
            }

            break;

        case 3:
        case 10:
            DebugP_log("\r| enter parameter address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter parameter (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->data) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->data > 0xFFFF)
            {
                DebugP_log("\r| ERROR: invalid parameter\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 4:
        case 11:
            DebugP_log("\r| enter parameter address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 7:
        case 13:
            DebugP_log("\r| enter port address (hex value):");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid port address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("| ERROR: invalid port address\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 14:
            DebugP_log("\r| enter encoder address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid encoder address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid encoder address\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter instruction (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->data) < 0)
            {
                DebugP_log("\r| ERROR: invalid instruction\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->data > 0xFFFF)
            {
                DebugP_log("\r| ERROR: invalid instruction\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 100:
        case 101:
        case 107:
            DebugP_log("\r| enter frequency in Hz: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid frequency\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 103:
            DebugP_log("\r| enter tST delay counter value in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 105:
            DebugP_log("\r| enter rx arm counter in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 106:
            DebugP_log("\r| enter rx clock disable time (for tD) in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 108:
            DebugP_log("\r| enter propagation delay in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 109:
            DebugP_log("\r| enter channel number: ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                return -EINVAL;
            }

            if(!(gEndat_multi_ch_mask & 1 << cmd_supplement->address))
            {
                DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter wire delay in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        default:
            cmd = -EINVAL;
            DebugP_log("\r| ERROR: no command data required for the command\n");
            break;
    }

    return cmd;
}

static int endat_handle_user(struct cmd_supplement *cmd_supplement)
{
    int cmd;

    endat_print_menu();
    cmd = endat_get_command();

    if(HAVE_COMMAND_SUPPLEMENT(cmd))
    {
        return endat_get_command_supplement(cmd, cmd_supplement);
    }

    return cmd;
}

static int endat_calc_clock(unsigned freq, struct endat_clk_cfg *clk_cfg)
{
    unsigned ns;

    if(freq > 16000000 || (freq > 12000000 && freq < 16000000))
    {
        DebugP_log("\r| ERROR: frequency above 16MHz, between 12 & 16MHz not allowed\n|\n|\n");
        return -1;
    }

    if((freq != 16000000) && (ENDAT_INPUT_CLOCK_FREQUENCY % (freq * 8)))
        DebugP_log("\r| WARNING: exact clock divider is not possible, frequencies set would be tx: %u\trx: %u\n",
                    ENDAT_INPUT_CLOCK_FREQUENCY / (ENDAT_INPUT_CLOCK_FREQUENCY / freq),
                    ENDAT_INPUT_CLOCK_FREQUENCY / (ENDAT_INPUT_CLOCK_FREQUENCY / (freq * 8)));

    ns = 2 * 1000000000 / freq; /* rx arm >= 2 clock */

    /* should be divisible by 5 */
    if(ns % 5)
    {
        ns /= 5, ns += 1,  ns *= 5;
    }

    clk_cfg->tx_div = ENDAT_INPUT_CLOCK_FREQUENCY / freq - 1;
    clk_cfg->rx_div = ENDAT_INPUT_CLOCK_FREQUENCY / (freq * 8) - 1;
    clk_cfg->rx_en_cnt = ns;
    clk_cfg->rx_div_attr = ENDAT_RX_SAMPLE_SIZE;

    if(freq == 16000000)
    {
        clk_cfg->rx_div_attr |= ENDAT_RX_SESQUI_DIV;
    }

    DebugP_logInfo("\r| clock config values - tx_div: %u\trx_div: %u\trx_en_cnt: %u\trx_div_attr: %x\n",
                    clk_cfg->tx_div, clk_cfg->rx_div, clk_cfg->rx_en_cnt, clk_cfg->rx_div_attr);

    return 0;
}

static unsigned endat_do_sanity_tst_delay(unsigned delay)
{
    /* (unsigned short)~0 is also a multiple of 5 */
    if(delay > (unsigned short)~0)
    {
        DebugP_log("\r| ERROR: delay greater than %uns, enter lesser value\n|\n|\n",
                    (unsigned short)~0);
        return delay;
    }

    if(delay % 5)
    {
        delay += 5, delay /= 5, delay *= 5;
        DebugP_log("\r| WARNING: delay not multiple of 5ns, rounding to %uns\n|\n|\n",
                    delay);
    }

    return delay;
}

static int endat_position_loop_status;

/* position period in microsecond */
static int endat_calc_position_period(unsigned freq)
{
    /* 16KHz limitation due to the timer */
    if(freq > 16000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 16KHz\n|\n|\n");
        return -1;
    }
    else if(gEndat_is_multi_ch && freq > 8000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 8KHz in multi channel configuration\n|\n|\n");
        return -1;
    }

    return 1000000 / freq;
}

static void endat_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        endat_position_loop_status = ENDAT_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

void endat_process_position_command(unsigned int a0)
{
    unsigned crc;

    endat_command_process(priv, 1, NULL);
    endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(priv, 1, &gEndat_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat_mtrctrl_crc_err[a0]++;
    }
}

unsigned short _endat_process_2_2_position_command(int cmd,
        struct cmd_supplement *cmd_supplement, unsigned int a0)
{
    unsigned crc;

    endat_command_process(priv, cmd, cmd_supplement);
    endat_recvd_process(priv, cmd, &gEndat_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(priv, cmd, &gEndat_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat_2_2_crc_position_err_cnt[a0]++;
    }

    if(priv->flags.info1 && !(crc & 0x2))
    {
        gEndat_2_2_crc_addinfo1_err_cnt[a0]++;
    }

    endat_addinfo_track(priv, cmd, cmd_supplement);

    return gEndat_format_data_mtrctrl[a0].position_addinfo.addinfo1.addinfo & 0xFFFF;
}

void endat_process_2_2_position_command(unsigned int a0)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned short pos_word;

    if(!gEndat_is_multi_ch && priv->has_safety)
    {
        cmd = 9, cmd_supplement.address = gEndat_2_2_loop_mrs;
    }
    else
    {
        cmd = 8;
    }

    pos_word = _endat_process_2_2_position_command(cmd, &cmd_supplement, a0);

    if(gEndat_is_multi_ch || !priv->has_safety)
    {
        return;
    }

    /* WORD3 in addinfo1 */
    if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD1)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFF0000FFFFFFFF;
        gEndat_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 32;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD2;
        /* WORD1 in addinfo1 */
    }
    else if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD2)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFFFFFFFFFF0000;
        gEndat_2_2_pos_val2[a0] |= pos_word;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD3;
        /* WORD2 in addinfo1 */
    }
    else if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD3)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFFFFFF0000FFFF;
        gEndat_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 16;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD1;
    }
}

void endat_position_loop(unsigned int a0)
{
    if(endat_fn_position_loop != NULL)
    {
        if(gEndat_is_multi_ch)
        {
            int j;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_fn_position_loop(j);
                }
            }
        }
        else
        {
            endat_fn_position_loop(0);
        }
    }
}

static int32_t endat_loop_task_create(void)
{
    int32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "endat_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)endat_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\rTask2 creation failed\n");
    }

    return status ;
}

static void endat_loop_timer_create(int us)
{
    TimerP_Params timerParams;

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = CONFIG_TIMER0_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_TIMER0_INPUT_CLK_HZ;
    timerParams.periodInUsec   = us;
    timerParams.oneshotMode    = 0;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);

    return ;
}

static int endat_get_position_loop_chars(struct endat_priv *priv,
        int continuous, int is_2_2)
{
    int i;

    if(priv->multi_turn_res)
    {
        i = 34;
    }
    else
    {
        i = 16;
    }

    if(!continuous)
    {
        i += 12;
    }

    i += 4;

    if(!continuous && is_2_2)
    {
        i += 4;
    }

    return i;
}

static void endat_print_position_loop(struct endat_priv *priv, int continuous,
                                      int is_2_2, int ch)
{
    unsigned long long max = pow(2, priv->single_turn_res);
    union position position;

    if(priv->type == rotary)
    {
        position.angle = ((float)
                          gEndat_format_data_mtrctrl[ch].position_addinfo.position.position) /
                         (float)max * (float)360;
    }
    else
    {
        position.length =
            gEndat_format_data_mtrctrl[ch].position_addinfo.position.position * priv->step;
    }

    /* max value is 2x48, has 15 digits, so 16 is safe */
    if(priv->multi_turn_res)
    {
        sprintf(gUart_buffer, "%16.12f, %16s", position.angle,
                uint64_to_str(gEndat_format_data_mtrctrl[ch].position_addinfo.position.revolution));
    }
    else
    {
        if(priv->type == rotary)
        {
            sprintf(gUart_buffer, "%16.12f", position.angle);
        }
        else
        {
            sprintf(gUart_buffer, "%16s", uint64_to_str(position.length));
        }
    }

    DebugP_log("\r%s", gUart_buffer);

    if(!continuous)
    {
        if(is_2_2)
        {
            DebugP_log(", %10u", gEndat_2_2_crc_position_err_cnt[ch]);
        }
        else
        {
            DebugP_log(", %10u", gEndat_mtrctrl_crc_err[ch]);
        }
    }

    DebugP_log(",%3u",
                gEndat_format_data_mtrctrl[ch].position_addinfo.position.f1);

    if(!continuous && is_2_2)
    {
        DebugP_log(",%3u",
                    gEndat_format_data_mtrctrl[ch].position_addinfo.position.f2);
    }
}

static void endat_print_position_loop_channel_info(struct endat_priv *priv,
        int is_2_2)
{
    int k, j, i = endat_get_position_loop_chars(priv, 0, is_2_2);

    /* add 3 extra to account for spacing b/n channels */
    i += 3;
    /* find mid point */
    i /= 2;
    /* account for "CHANNEL x" display */
    i -= 4;

    for(k = i, j = 0; j < 3; j++, k = i)
        if(gEndat_multi_ch_mask & 1 << j)
        {
            while(k--)
            {
                DebugP_log("%c", ' ');
            }

            DebugP_log("\rCHANNEL %d", j);

            /* 3 - extra to account for spacing b/n channels, 9 - "CHANNEL x" length */
            k = endat_get_position_loop_chars(priv, 0, is_2_2) + 3 - i - 9;

            while(k--)
            {
                DebugP_log("%c", ' ');
            }
        }
}

static void endat_handle_prop_delay(struct endat_priv *priv,
                                    unsigned short prop_delay)
{
    if(prop_delay > priv->rx_en_cnt)
    {
        unsigned short dis = (prop_delay - priv->rx_en_cnt) * 2 / priv->rx_en_cnt;

        endat_config_rx_arm_cnt(priv, prop_delay);
        /* propagation delay - 2T */
        endat_config_rx_clock_disable(priv, dis);
    }
    else
    {
        endat_config_rx_arm_cnt(priv, priv->rx_en_cnt);
        endat_config_rx_clock_disable(priv, 0);
    }
}

static void endat_process_host_command(int cmd,
                                       struct cmd_supplement *cmd_supplement, struct endat_priv *priv)
{
    struct endat_clk_cfg clk_cfg;
    static int timer_init;

    /* clock configuration */
    if(cmd == 100)
    {
        if(endat_calc_clock(cmd_supplement->frequency, &clk_cfg) < 0)
        {
            return;
        }

        endat_config_clock(priv, &clk_cfg);

        priv->rx_en_cnt = clk_cfg.rx_en_cnt;

        if(gEndat_is_multi_ch)
        {
            int j;
            unsigned short d;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_handle_prop_delay(priv, gEndat_prop_delay[priv->channel]);
                    d = gEndat_prop_delay_max - gEndat_prop_delay[j];
                    endat_config_wire_delay(priv, d);
                }
            }
        }
        else
        {
            endat_handle_prop_delay(priv, gEndat_prop_delay[priv->channel]);
        }

        /* set tST to 2us if frequency > 1MHz, else turn it off */
        if(cmd_supplement->frequency >= 1000000)
        {
            cmd_supplement->frequency = 2000;
        }
        else
        {
            cmd_supplement->frequency = 0;
        }

        endat_process_host_command(103, cmd_supplement, priv);
        /* control loop */
    }
    else if(cmd == 101)
    {
        int us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return;
        }

        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing position loop as related Task creation failed\n|\n|\n");
            DebugP_log("\rTask_create() failed!\n");
            return;
        }

        endat_fn_position_loop = endat_process_position_command;

        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        if(priv->multi_turn_res)
        {
            DebugP_log("\r|\r\n| press enter to stop the position display|\n");

            if(gEndat_is_multi_ch)
            {
                int j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 0);
                DebugP_log("\r\n|\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("| ");
                        DebugP_log("        position,       revolution, crc errors, f1\n");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\n");
            }
            else
            {
                DebugP_log("\r|         position,       revolution, crc errors, f1|\r\n ");
            }
        }
        else
        {
            DebugP_log("|\n| press enter to stop the position display\n|\n");

            if(gEndat_is_multi_ch)
            {
                int j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 0);
                DebugP_log("\r\n|\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("| ");
                        DebugP_log("        position, crc errors, f1\n");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\n");
            }
            else
            {
                DebugP_log("|         position, crc errors, f1\n| ");
            }
        }

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
                return;
            }
            else
            {
                int i;

                if(gEndat_is_multi_ch)
                {
                    int j;

                    for(i = 0, j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            i += endat_get_position_loop_chars(priv, 0, 0);
                            DebugP_log("| ");
                            endat_print_position_loop(priv, 0, 0, j);
                            DebugP_log(" ");
                            DebugP_log("\n");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i = endat_get_position_loop_chars(priv, 0, 0);
                    endat_print_position_loop(priv, 0, 0, 0);
                }

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(500);

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }

    else if(cmd == 102)
    {
        priv->raw_data ^= 1;
    }
    else if(cmd == 103)
    {
        unsigned delay;

        delay = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(delay <= (unsigned short)~0)
        {
            endat_config_tst_delay(priv, (unsigned short) delay);
        }
    }
    else if(cmd == 104)
    {

        if(gEndat_is_multi_ch)
        {
            DebugP_log("\r| ERROR: Multi channel configuration does not support continuous clock mode\r\n|\r\n|\n");
            return;
        }

        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
            DebugP_log("Task_create() failed!\n");
            return;
        }

        endat_start_continuous_mode(priv);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        if(priv->multi_turn_res)
        {
            DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position,       revolution, f1\r\n| ");
        }
        else
        {
            DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");
        }

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                endat_stop_continuous_mode(priv);
                return;
            }
            else
            {
                int i;

                endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[0]);

                i = endat_get_position_loop_chars(priv, 1, 0);
                endat_print_position_loop(priv, 1, 0, 0);

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }
    else if(cmd == 105)
    {
        unsigned val;

        /* reuse tST delay sanity check */
        val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(val <= (unsigned short)~0)
        {
            endat_config_rx_arm_cnt(priv, (unsigned short)val);
        }
    }
    else if(cmd == 106)
    {
        unsigned short dis = cmd_supplement->frequency * 2 / priv->rx_en_cnt;

        endat_config_rx_clock_disable(priv, dis);
    }
    else if(cmd == 108)
    {
        /* reuse tST delay sanity check */
        unsigned val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(val > (unsigned short)~0)
        {
            return;
        }

        if(gEndat_is_multi_ch)
        {
            int j;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_handle_prop_delay(priv, (unsigned short)val);
                }
            }
        }
        else
        {
            endat_handle_prop_delay(priv, (unsigned short)val);
        }
    }
    else if(cmd == 109)
    {
        /* reuse tST delay sanity check */
        unsigned val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        endat_multi_channel_set_cur(priv, cmd_supplement->address);
        endat_config_wire_delay(priv, val);
    }
    else if(cmd == 107)
    {
        int us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return;
        }
        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing position loop as related Task creation failed\n|\n|\n");
            DebugP_log("\rTask_create() failed!\n");
            return;
        }

        endat_fn_position_loop = endat_process_2_2_position_command;

        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        /* reset additional info's if present */
        endat_command_process(priv, 5, NULL);
        endat_addinfo_track(priv, 5, NULL);

        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD1;

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        /* so that proper position value 2 is displayed from the begining */
        ClockP_usleep(us * 3);

        if(!gEndat_is_multi_ch && !priv->has_safety)
        {
            DebugP_log("\r|\n| encoder does not support safety, position value 2 would not be displayed\n|\n");
        }

        DebugP_log("\r|\n\r| press enter to stop the position display\n\r|\n");

        if(priv->multi_turn_res)
        {
            if(gEndat_is_multi_ch)
            {
                int j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 1);
                DebugP_log("\n\r|\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("\r| ");
                        DebugP_log("        position,       revolution, crc errors, f1, f2");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\r\n");
            }
            else
            {
                DebugP_log("\r|         position,       revolution, crc errors, f1, f2");

                if(priv->has_safety)
                {
                    DebugP_log(",      position(2),    revolution(2), crc errors(2)");
                }
                DebugP_log("\r\n| ");
            }
        }
        else
        {
            if(gEndat_is_multi_ch)
            {
                int j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 1);
                DebugP_log("\r\n|\r\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("| ");
                        DebugP_log("        position, crc errors, f1");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\n");
            }
            else
            {
                DebugP_log("|         position, crc errors, f1, f2 ");

                if(priv->has_safety)
                {
                    DebugP_log(",      position(2), crc errors(2)");
                }
                DebugP_log("\r\n| ");
            }
        }

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

                /* reset additional info1 */
                endat_command_process(priv, 5, NULL);
                endat_addinfo_track(priv, 5, NULL);
                return;
            }
            else
            {
                int i;

                if(gEndat_is_multi_ch)
                {
                    int j;

                    for(i = 0, j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            i += endat_get_position_loop_chars(priv, 0, 1);
                            DebugP_log("| ");
                            endat_print_position_loop(priv, 0, 1, j);
                            DebugP_log(" ");
                            DebugP_log("\n");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i = endat_get_position_loop_chars(priv, 0, 1);
                    endat_print_position_loop(priv, 0, 1, 0);
                }

                if(!gEndat_is_multi_ch && priv->has_safety)
                {
                    unsigned long long multi_turn, single_turn;
                    union position position2;
                    unsigned long long max = pow(2, priv->single_turn_res);

                    multi_turn = ENDAT_GET_POS_MULTI_TURN(gEndat_2_2_pos_val2[0], priv);
                    single_turn = ENDAT_GET_POS_SINGLE_TURN(gEndat_2_2_pos_val2[0], priv);

                    if(priv->type == rotary)
                    {
                        position2.angle = (float)single_turn / (float)max * (float)360;
                    }
                    else
                    {
                        position2.length = single_turn * priv->step;
                    }

                    DebugP_log(", ");

                    if(priv->multi_turn_res)
                    {
                        sprintf(gUart_buffer, "%16.12f, %16s", position2.angle, uint64_to_str(multi_turn));
                    }
                    else
                    {
                        if(priv->type == rotary)
                        {
                            sprintf(gUart_buffer, "%16.12f", position2.angle);
                        }
                        else
                        {
                            sprintf(gUart_buffer, "%16s", uint64_to_str(position2.length));
                        }
                    }

                    DebugP_log("%s", gUart_buffer);

                    DebugP_log(",    %10u", gEndat_2_2_crc_addinfo1_err_cnt[0]);
                }

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);

                if(!gEndat_is_multi_ch && priv->has_safety)
                {
                    if(priv->multi_turn_res)
                    {
                        i += 2 + 46 + 3;
                    }
                    else
                    {
                        i += 2 + 28 + 3;
                    }
                }

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }
    else
    {
        DebugP_log("\r| ERROR: non host command being requested to be handled as host command\n|\n|\n");
    }
}

static void endat_handle_rx(struct endat_priv *priv, int cmd)
{
    unsigned crc;
    union endat_format_data endat_format_data;

    if(priv->raw_data)
    {
        endat_display_raw_data(cmd, priv);
    }

    endat_recvd_process(priv, cmd,  &endat_format_data);
    crc = endat_recvd_validate(priv, cmd, &endat_format_data);
    endat_recvd_print(cmd, priv, &endat_format_data, crc);

    return;
}

static void endat_print_encoder_info(struct endat_priv *priv)
{
    DebugP_log("EnDat 2.%d %s encoder\tID: %u %s\tSN: %c %u %c\n\n",
                priv->cmd_set_2_2 ? 2 : 1,
                (priv->type == rotary) ? "rotary" : "linear",
                priv->id.binary, (char *)&priv->id.ascii,
                (char)priv->sn.ascii_msb, priv->sn.binary, (char)priv->sn.ascii_lsb);
    DebugP_log("\rPosition: %d bits ", priv->pos_res);

    if(priv->type == rotary)
    {
        DebugP_log("(singleturn: %d, multiturn: %d) ", priv->single_turn_res,
                    priv->multi_turn_res);
    }

    DebugP_log("[resolution: %d %s]", priv->step,
                priv->type == rotary ? "M/rev" : "nm");
    DebugP_log("\r\n\nPropagation delay: %dns",
                gEndat_prop_delay[priv->channel]);
    DebugP_log("\n\n\n");
}

void endat_main(void *args)
{
    int i;
    struct cmd_supplement cmd_supplement;
    char c;
    uint64_t icssgclk;

    void *pruss_cfg;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
    HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x80));

    i = endat_get_fw_version();

    DebugP_log("\n\n\n");
    DebugP_log("EnDat firmware \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    endat_pre_init();

    DebugP_log("\r\nMulti channel configuration ? [y/N]: ");

    if(DebugP_scanf("%c\n", &c) < 0 || (c != 'y' && c != 'Y'))
    {
        DebugP_log("\r\nselected single channel configuration\n");
    }
    else
    {
        DebugP_log("\r\nselected multi channel configuration\n");
        gEndat_is_multi_ch = 1;
    }

    if(gEndat_is_multi_ch)
    {
        DebugP_log("\r\nselect channels to be used in multi channel\n");
        DebugP_log("\r\nselect channel 0 [Y/n]: ");

        if(DebugP_scanf("%c\n", &c) < 0 || (c != 'n' && c != 'N'))
        {
            gEndat_multi_ch_mask |= ENDAT_MULTI_CH0;
        }
        else
        {
            gEndat_multi_ch_mask &= ~ENDAT_MULTI_CH0;
        }

        DebugP_log("\r\nselect channel 1 [Y/n]: ");

        if(DebugP_scanf("%c\n", &c) < 0 || (c != 'n' && c != 'N'))
        {
            gEndat_multi_ch_mask |= ENDAT_MULTI_CH1;
        }
        else
        {
            gEndat_multi_ch_mask &= ~ENDAT_MULTI_CH1;
        }

        DebugP_log("\r\nselect channel 2 [Y/n]: ");

        if(DebugP_scanf("%c\n", &c) < 0 || (c != 'n' && c != 'N'))
        {
            gEndat_multi_ch_mask |= ENDAT_MULTI_CH2;
        }
        else
        {
            gEndat_multi_ch_mask &= ~ENDAT_MULTI_CH2;
        }

        DebugP_log("\r\nchannels %s %s %s selected\n",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH0 ? "0" : "",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH1 ? "1" : "",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH2 ? "2" : "");

        if(!gEndat_multi_ch_mask)
        {
            DebugP_log("\r\nERROR: please select channels to be used in multi channel configuration -\n\n");
            DebugP_log("\rexit %s as no channel selected in multichannel configuration\n",
                          __func__);
            return;
        }
    }
    else
    {
        DebugP_log("\renter channel number: ");

        if(DebugP_scanf("%d\n", &i) < 0 || i > 2)
        {
            DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
            i = 0;
        }
    }

    DebugP_log("\r\n\n");

    pruss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->cfgRegBase);

	priv = endat_init((struct endat_pruss_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcss0Handle->hwAttrs))->pru1DramBase, pruss_cfg);

    endat_config_host_trigger(priv);

    if(gEndat_is_multi_ch)
    {
        endat_config_multi_channel_mask(priv, gEndat_multi_ch_mask);
    }

    else
    {
        endat_config_channel(priv, i);
    }

    /* Read the ICSSG configured clock frequency. */
    SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, &icssgclk);

    /* Configure Delays based on the ICSSG frequency*/
    /* Count = ((required delay * icssgclk)/1000) */
    priv->pruss_xchg->endat_delay_2ms = ((icssgclk/1000) * 2);
    priv->pruss_xchg->endat_delay_12ms = ((icssgclk/1000) * 12);
    priv->pruss_xchg->endat_delay_50ms = ((icssgclk/1000) * 50);

    endat_pruss_load_run_fw();

    /* check initialization ack from firmware, with a timeout of 5 second */
    i = endat_wait_initialization(priv, 5000);

    if(i < 0)
    {
        DebugP_log("\rERROR: EnDat initialization failed -\n\n");

        if(gEndat_is_multi_ch)
        {
            unsigned char tmp;

            tmp = endat_multi_channel_detected(priv) & gEndat_multi_ch_mask;
            tmp ^= gEndat_multi_ch_mask;
            DebugP_log("\r\tunable to detect encoder in channel %s %s %s\n",
                        tmp & ENDAT_MULTI_CH0 ? "0" : "",
                        tmp & ENDAT_MULTI_CH1 ? "1" : "",
                        tmp & ENDAT_MULTI_CH2 ? "2" : "");
        }
        else
        {
            DebugP_log("\r\tcheck whether encoder is connected and ensure proper connections\n");
        }

        DebugP_log("\rexit %s due to failed firmware initialization\n", __func__);
        return;
    }

    /* read encoder info at low frequency so that cable length won't affect */
    cmd_supplement.frequency = 200 * 1000;
    endat_process_host_command(100, &cmd_supplement, priv);

    if(gEndat_is_multi_ch)
    {
        int j;

        for(j = 0; j < 3; j++)
        {
            if(gEndat_multi_ch_mask & 1 << j)
            {
                endat_multi_channel_set_cur(priv, j);

                if(endat_get_encoder_info(priv) < 0)
                {
                    DebugP_log("\rEnDat initialization channel %d failed\n", j);
                    DebugP_log("\rexit %s due to failed initialization\n", __func__);
                    return;
                }

                gEndat_prop_delay[priv->channel] = endat_get_prop_delay(priv);
                DebugP_log("\n\t\t\t\tCHANNEL %d\n\n", j);
                endat_print_encoder_info(priv);
            }
        }

        gEndat_prop_delay_max = gEndat_prop_delay[0] > gEndat_prop_delay[1] ?
                               gEndat_prop_delay[0] : gEndat_prop_delay[1];
        gEndat_prop_delay_max = gEndat_prop_delay_max > gEndat_prop_delay[2] ?
                               gEndat_prop_delay_max : gEndat_prop_delay[2];

    }
    else
    {
        if(endat_get_encoder_info(priv) < 0)
        {
            DebugP_log("\rEnDat initialization failed\n");
            DebugP_log("\rexit %s due to failed initialization\n", __func__);
            return;
        }

        gEndat_prop_delay[priv->channel] = endat_get_prop_delay(priv);

        endat_print_encoder_info(priv);
    }

    /* default frequency - 8MHz for 2.2 encoders, 1MHz for 2.1 encoders */
    if(priv->cmd_set_2_2)
    {
        cmd_supplement.frequency = 8 * 1000 * 1000;
    }
    else
    {
        cmd_supplement.frequency = 1 * 1000 * 1000;
    }

    endat_process_host_command(100, &cmd_supplement, priv);


    while(1)
    {
        int cmd;

        cmd = endat_handle_user(&cmd_supplement);

        if(cmd < 0)
        {
            continue;
        }

        if(VALID_HOST_CMD(cmd))
        {
            endat_process_host_command(cmd, &cmd_supplement, priv);
            DebugP_log("\r|\n\r|\n");
            continue;
        }

        if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
        {
            continue;
        }

        if(gEndat_is_multi_ch)
        {
            int j;

            DebugP_log("\r|\n");

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", j);
                    endat_handle_rx(priv, cmd);
                }
            }
        }
        else
        {
            endat_handle_rx(priv, cmd);
        }

        /* this cannot be done except as last in loop; additional info becomes applicable from next command onwards only */
        endat_addinfo_track(priv, cmd, &cmd_supplement);
    }

    Board_driversClose();
    Drivers_close();
}

