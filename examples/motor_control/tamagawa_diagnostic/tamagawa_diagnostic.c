/**
 * tamagawa_diagnostic.c
 *
 * Copyright (c) 2022, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */



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
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include <motor_control/position_sense/tamagawa/include/tamagawa_drv.h>
#if ((CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2) == 1)
#include <motor_control/position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h>
#endif
#if ((CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2) > 1)
#include <motor_control/position_sense/tamagawa/firmware/tamagawa_master_multi_channel_bin.h>
#endif

static uint8_t gTamagawa_multi_ch_mask;
static uint32_t gTamagawa_is_multi_ch;
struct tamagawa_priv *priv;

/** \brief Global Structure pointer holding PRU-ICSSx memory Map. */
PRUICSS_Handle gPruIcssXHandle;
void *gPru_dramx;

void tamagawa_pruicss_init(void)
{
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    /* PRUICSS_PRUx holds value 0 or 1 depending on whether we are using PRU0 or PRU1 slice */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_PRUx));
    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
}

void tamagawa_pruicss_load_run_fw(void)
{
    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    /*Load firmware. Set buffer = write to Pru memory */
    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),0, (uint32_t *) TamagawaFirmware,sizeof(TamagawaFirmware));
    PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    /*Run firmware */
    PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
}

void tamagawa_display_result(struct tamagawa_priv *priv, int32_t cmd)
{
    /* Prints the position value returned by the encoder for a particular command ID */
    switch(cmd)
    {
        case DATA_ID_7:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_8:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_C:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_0:
            /* Data readout: data in one revolution */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABM: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abm, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nENID: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.enid, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tENID: 0x%x\tABM: 0x%x\tALMC: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.enid, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abm, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.almc, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.edf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.adf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.edf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.adf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        default:
            DebugP_log("\r\n| ERROR: unknown Data ID\n");
            break;
    }
}


static void tamagawa_handle_rx(struct tamagawa_priv *priv, int32_t cmd)
{
    DebugP_log("\r\n Parsing process started\n");
    /* Case of parsing failure */
    if (tamagawa_parse(cmd, priv) == -1)
    {
        DebugP_log("\r\n ERROR: Parsing failure\n");
        return;
    }
    /* Case of successful parsing, display the results after CRC check*/
    DebugP_log("\r\n Channel is  %x \n",priv->channel);
    DebugP_log("\r\n data id is %x \n",cmd);
    if (tamagawa_crc_verify(priv) == 1)
    {
        DebugP_log("\r\n CRC success \n");
        tamagawa_display_result(priv, cmd);
        return;
    }
    else
    {
        DebugP_log("\r\n CRC Failure \n");
    }

    return;
}

static enum data_id tamagawa_get_command(uint8_t *adf, uint8_t *edf)
{
    int32_t cmd;
    uint32_t val;
    /* Check to make sure that the command issued is correct */
    if(DebugP_scanf("%d\n", &cmd) < 0)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* Check to make sure that the command issued is correct */
    if(cmd >= DATA_ID_NUM)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* In case of EEPROM commands, take input for Address field for different channels selected*/
    if((cmd == DATA_ID_D) || (cmd == DATA_ID_6))
    {
        uint32_t ch = 0;
        for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
        {
            if(gTamagawa_multi_ch_mask & 1 << ch)
            {
                if(gTamagawa_is_multi_ch != 1)
                {
                    DebugP_log("\r\n| Enter EEPROM address (hex value) : ");
                }
                else
                {
                    DebugP_log("\r\n| Enter EEPROM address (hex value) for ch %d : ", ch);
                }
                if(DebugP_scanf("%x\n", &val) < 0)
                {
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: invalid EEPROM address\n|\n");
                    break;
                }

                if(val > MAX_EEPROM_ADDRESS)
                {
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: invalid EEPROM address\n|\n");
                    break;
                }

                *adf = (uint8_t)val;
                tamagawa_update_adf(priv, val, ch);
            }
        }
    }
    /* In case of EEPROM Write, take input for Address field for different channels selected*/
    if(cmd == DATA_ID_6)
    {
        uint32_t ch = 0;
        for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
        {
            if(gTamagawa_multi_ch_mask & 1 << ch)
            {
                if(gTamagawa_is_multi_ch != 1)
                {
                    DebugP_log("\r\n| Enter EEPROM data (hex value) : ");
                }
                else
                {
                    DebugP_log("\r\n| Enter EEPROM data (hex value) for ch %d : ", ch);
                }
                if(DebugP_scanf("%x\n", &val) < 0)
                {
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: invalid EEPROM data\n|\n");
                    break;
                }

                if(val > MAX_EEPROM_WRITE_DATA)
                {
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: invalid EEPROM data\n|\n");
                    break;
                }

                *edf = (uint8_t)val;
                tamagawa_update_edf(priv, val, ch);
            }
        }
    }
    return cmd;
}

static void tamagawa_display_menu(void)
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select DATA ID Code                              |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 0 : Data readout, Absolute (Data ID 0)                                       |");
    DebugP_log("\r\n| 1 : Data readout, Multi-turn (Data ID 1)                                     |");
    DebugP_log("\r\n| 2 : Data readout, Encoder-ID (Data ID 2)                                     |");
    DebugP_log("\r\n| 3 : Data readout, Absolute & Multi-turn (Data ID 3)                          |");
    DebugP_log("\r\n| 4 : Writing to EEPROM (Data ID 6)                                            |");
    DebugP_log("\r\n| 5 : Reset (Data ID 7)                                                        |");
    DebugP_log("\r\n| 6 : Reset (Data ID 8)                                                        |");
    DebugP_log("\r\n| 7 : Reset (Data ID C)                                                        |");
    DebugP_log("\r\n| 8 : Readout from EEPROM (Data ID D)                                          |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|\n|\n");
    DebugP_log("\r\n| enter value: ");
}


uint32_t tamagawa_get_fw_version(void)
{
    /* Returns the firmware version, depending on Single or Multi-channel configuration */
    return *((uint32_t *)TamagawaFirmware + 1);
}


void tamagawa_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
/*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_TAMAGAWA0_BOOSTER_PACK)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
    void *pruicss_cfg;
    uint32_t slice_value = 1;
    uint32_t selected_ch;

    tamagawa_pruicss_init();

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);

    if(PRUICSS_PRUx == 0)
    {
        slice_value = 0;
    }

    /* Initialize the priv structure according to the PRUx slice selected */
    priv = tamagawa_init((struct tamagawa_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcssXHandle->hwAttrs))->pru1DramBase, pruicss_cfg, slice_value);


    tamagawa_set_baudrate(priv, CONFIG_TAMAGAWA0_BAUDRATE);

    DebugP_log("\r\n\nTamagawa PRU-ICSS init done\n\n");

    /* Set the value of gTamagawa_multi_ch_mask based on the channels selected */
    gTamagawa_multi_ch_mask = (CONFIG_TAMAGAWA0_CHANNEL0<<0|CONFIG_TAMAGAWA0_CHANNEL1<<1|CONFIG_TAMAGAWA0_CHANNEL2<<2);

    if (CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2 > 1)
    {
        gTamagawa_is_multi_ch = 1;
    }

    if(gTamagawa_is_multi_ch)
    {
        tamagawa_config_multi_channel_mask(priv, gTamagawa_multi_ch_mask);
    }

    else
    {
        if (CONFIG_TAMAGAWA0_CHANNEL0 == 1)
        {
            selected_ch = 0;
        }

        if (CONFIG_TAMAGAWA0_CHANNEL1 == 1)
        {
            selected_ch = 1;
        }

        if (CONFIG_TAMAGAWA0_CHANNEL2 == 1)
        {
            selected_ch = 2;
        }
        tamagawa_config_channel(priv, selected_ch);
    }

    uint32_t firmware_ver;
    firmware_ver = tamagawa_get_fw_version();

    DebugP_log("\r\nTamagawa firmware \t: %x.%x.%x (%s)\n\n", (firmware_ver >> 24) & 0x7F,
                (firmware_ver >> 16) & 0xFF, firmware_ver & 0xFFFF, firmware_ver & (1 << 31) ? "internal" : "release");

    DebugP_log("\r\nChannel(s) selected: %s %s %s \n\n\n",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH0 ? "0" : "",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH1 ? "1" : "",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH2 ? "2" : "");

    /*Updating the channel mask in interface*/
    priv->tamagawa_xchg->tamagawa_interface.ch_mask =gTamagawa_multi_ch_mask;

    tamagawa_config_host_trigger(priv);

    tamagawa_pruicss_load_run_fw();
    DebugP_log("\r\nTamagawa PRU-ICSS firmware loaded and running\n\n\n");

    while(1)
    {
        /*
         * Initialized to zero to remove the compiler warning about the variable being uninitialized.
         */
        uint8_t adf = 0, edf = 0;
        enum data_id cmd;

        tamagawa_display_menu();
        cmd = tamagawa_get_command(&adf, &edf);

        if(cmd >= DATA_ID_NUM)
        {
            continue;
        }

        tamagawa_update_data_id(priv, cmd);

        /* In case of EEPROM commands, calculate the CRC for the different channels selected */
        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            uint32_t ch = 0;
            for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_update_crc(priv, cmd, ch);
                }
            }

        }

        tamagawa_command_process(priv, cmd, gTamagawa_multi_ch_mask);

        if(gTamagawa_is_multi_ch)
        {
            DebugP_log("\r\n Multi-channel mode is enabled\n\n");

            uint32_t ch;
            for(ch = 0; ch < MAX_CHANNELS; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_multi_channel_set_cur(priv, ch);
                    DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                    tamagawa_handle_rx(priv, cmd);
                }
            }
        }
        else
        {
            DebugP_log("\r\n Single-channel mode is enabled\n\n");
            tamagawa_handle_rx(priv, cmd);
        }

}
    Board_driversClose();
    Drivers_close();
}
