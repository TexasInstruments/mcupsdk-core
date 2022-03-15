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

#include <motor_control/position_sense/tamagawa/include/tamagawa_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>

static struct tamagawa_priv tamagawa_priv;

int32_t tamagawa_parse(int32_t cmd, struct tamagawa_priv *priv)
{
    /* Parses the data in tamagawa interface received from the Encoder*/
    uint32_t word0, word1, word2;
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;

    word0 = tamagawa_xchg->ch[priv->channel].pos_word0;
    word1 = tamagawa_xchg->ch[priv->channel].pos_word1;
    word2 = tamagawa_xchg->ch[priv->channel].pos_word2;

    switch(cmd)
    {
        case DATA_ID_0:
            /* Data readout: data in one revolution */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[priv->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            /* Data frames: cf(8 bits) + sf(8 bits) + abm(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf =(word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf =(word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abm = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc =(word1) & 0xFF;
            tamagawa_xchg->ch[priv->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_2:
            /* Data readout: encoder ID */
            /* Data frames: cf(8 bits) + sf(8 bits) + enid(8 bits)+ crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.enid = (word0>>8)& 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word0)& 0xFF;
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + enid(8 bits) + abm(3 frames with 8 bits data each) + almc(8 bits) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>24) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.enid =(word1 >> 16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abm = ((word1>>8)& 0xFF) | ((word1)& 0xFF)<<8 | (((word2>>16) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.almc =(word2 >> 8) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word2) & 0xFF;
            tamagawa_xchg->ch[priv->channel].pos_word2=word2<<8;
            break;

        case DATA_ID_7:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[priv->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_8:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[priv->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_C:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[priv->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            /* Data frames: cf(1frame) + adf(8 bits) + edf(8 bits) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.adf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.edf =(word0>>8)& 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word0)& 0xFF;
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            /* Data frames: cf(1frame) + adf(8 bits) + edf(8 bits) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.adf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.edf =(word0>>8)& 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word0)& 0xFF;
            break;

        default:
            return -1;
    }

    return 0;
}

static uint8_t tamagawa_crc(struct tamagawa_priv *priv, uint8_t len, uint32_t ch)
{
    /* Calculates the CRC for the currently selected channel. Uses the CRC function (X^8+1) */
    uint8_t crc = 0;
    uint8_t val;
    uint32_t  i, j;
    uint32_t word0, word1, word2;

    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;

    word0 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word0;
    word1 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word1;
    word2 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word2;

    /* In EEPROM Write, we have CF, ADF and EDF fields as bits 24-31, 16-23 and 8-15 respectively */
    /* In EEPROM Write, we have CF and ADF fields as bits 24-31 and 16-23 respectively */
    /* Size of the array is 12 in order to use 12 8 bit integers for different frames */
    uint8_t data[12];
    data[3]=word0 & 0xFF;
    data[2]=(word0 >> 8) & 0xFF;
    data[1]=(word0 >> 16) & 0xFF;
    data[0]=(word0 >> 24) & 0xFF;

    data[7]=word1 & 0xFF;
    data[6]=(word1 >> 8) & 0xFF;
    data[5]=(word1 >> 16) & 0xFF;
    data[4]=(word1 >> 24) & 0xFF;

    data[11]=word2 & 0xFF;
    data[10]=(word2 >> 8) & 0xFF;
    data[9]=(word2 >> 16) & 0xFF;
    data[8]=(word2 >> 24) & 0xFF;


    for(i = 0; i < len; i++)
    {
        for(j = 0; j < 8; j++)
        {
            val = (data[i] >> 7) ^ (crc >> 7);
            crc <<= 1;
            data[i] <<= 1;
            crc |= val;
        }
    }

    return crc;
}

void tamagawa_update_crc(struct tamagawa_priv *priv, int32_t cmd, uint32_t ch)
{
    /* Passes the values of CF(Control Field), ADF(address of EEPROM) and EDF(data for EEPROM) to the CRC calculator fucntion and update the CRC field */
    uint32_t word0;
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;
    tamagawa_eeprom_crc_reinit(priv);
    word0 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word0;

    if (cmd == DATA_ID_6)
    {
        /* Bitwise OR the 32 bit integer with the CF, ADF and EDF values left shifted 24, 16 and 8 times respectively, in order to get the correct sequence of bits for CRC calculation*/
        word0 = (word0|0x32)<<24;
        word0 = word0 |  (tamagawa_xchg->tamagawa_eeprom_interface[ch].adf<<16);
        word0 = word0 | (tamagawa_xchg->tamagawa_eeprom_interface[ch].edf<<8);
        tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = word0;
        tamagawa_xchg->tamagawa_eeprom_interface[ch].crc = tamagawa_crc(priv, 3, ch);
    }
    else
    {
        /* Bitwise OR the 32 bit integer with the CF and ADF values left shifted 24 and 16 times respectively, in order to get the correct sequence of bits for CRC calculation*/
        word0 = (word0|0xEA)<<24;
        word0 = word0 | (tamagawa_xchg->tamagawa_eeprom_interface[ch].adf<<16);
        tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = word0;
        tamagawa_xchg->tamagawa_eeprom_interface[ch].crc = tamagawa_crc(priv, 2, ch);
    }

    tamagawa_eeprom_crc_reinit(priv);
}

int32_t tamagawa_crc_verify(struct tamagawa_priv *priv)
{
    /* Verifies the CRC computed with the encoder crc */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;
    uint32_t word0;
    word0 = tamagawa_xchg->ch[priv->channel].cal_crc;

    if ((word0 & 0xFF)==1)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

void tamagawa_set_baudrate(struct tamagawa_priv *priv, double baudrate)
{
    /* Updates the values for oversample rate and division factor for Tx and Rx based on the baud rate selected */
    if (baudrate==2.5)
    {
        /* For 2.5 Mbps, Rx Divide factor is 10, Tx Divide factor is 80 and Oversampling rate is 8 */
        priv->tamagawa_xchg->tamagawa_interface.rx_div_factor= 9;
        priv->tamagawa_xchg->tamagawa_interface.tx_div_factor= 79;
        priv->tamagawa_xchg->tamagawa_interface.oversample_rate= 7;
    }
    if(baudrate==5)
    {
        /* For 5 Mbps, Rx Divide factor is 5, Tx Divide factor is 40 and Oversampling rate is 8 */
        priv->tamagawa_xchg->tamagawa_interface.rx_div_factor= 4;
        priv->tamagawa_xchg->tamagawa_interface.tx_div_factor= 39;
        priv->tamagawa_xchg->tamagawa_interface.oversample_rate= 7;
    }
}

int32_t tamagawa_command_build(struct tamagawa_priv *priv, int32_t cmd,  uint8_t gTamagawa_multi_ch_mask)
{
    /* Sets up the tamagawa command in the PRU interface buffer */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;

    /*first clear command parameters to be safe */
    memset(&tamagawa_xchg->cmd, 0, sizeof(tamagawa_xchg->cmd));
    uint32_t ch;
    switch(cmd)
    {
        case DATA_ID_0:
            /* Data readout: data in one revolution */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x20)|(0x40<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x28)|(0xC0<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x24)|(0xC0<<8);
            /* Number of expected Rx frames is 4 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 4;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 4 */
            tamagawa_xchg->cmd.word1 = (1)|(4<<8);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x2C)|(0x40<<8);
            /* Number of expected Rx frames is 11 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 11;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 11 */
            tamagawa_xchg->cmd.word1 = (1)|(0xB<<8);
            break;

        case DATA_ID_7:
            /* Reset */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x2E)|(0xC0<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_8:
            /* Reset */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x21)|(0xC0<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_C:
            /* Reset */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x23)|(0x40<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            /* Loop through all the selected channels and prepare the EEPROM Read Tx data based on the CF, ADF and CRC data */
            for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_prepare_eeprom_command(priv, cmd, ch);
                }
            }
            /* Number of expected Rx frames is 4 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 4;
            /* Number of Tx frames being sent to the encoder is 3, and the number of Rx frames to be received is 4 */
            tamagawa_xchg->cmd.word1 = (3)|(4<<8);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            /* Loop through all the selected channels and prepare the EEPROM Read Tx data based on the CF, ADF, EDF and CRC data */
            for(ch = 0 ; ch < MAX_CHANNELS; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_prepare_eeprom_command(priv, cmd, ch);
                }
            }
            /* Number of expected Rx frames is 4 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 4;
            /* Number of Tx frames being sent to the encoder is 4, and the number of Rx frames to be received is 4 */
            tamagawa_xchg->cmd.word1 = (4)|(4<<8);
            break;

        default:
            cmd = -1;
            break;
    }

    return cmd;
}

void tamagawa_command_send(struct tamagawa_priv *priv)
{
    /* Triggers sending the tamagawa command in PRU */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;
    /* Set the trigger value as 1 */
    tamagawa_xchg->config.trigger = 0x1;
}

void tamagawa_command_wait(struct tamagawa_priv *priv)
{
    /* Waits till PRU finishes tamagawa transaction */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;
    /* Wait until the trigger value is 1 */
    while(tamagawa_xchg->config.trigger & 0x1)
        ;
}

int32_t tamagawa_command_process(struct tamagawa_priv *priv, int32_t cmd, uint8_t gTamagawa_multi_ch_mask)
{
    /* Sends the tamagawa command and waits till firmware acknowledges */

    /* Use tamagawa_command_build to setup the tamagawa command in the PRU interface buffer */
    cmd = tamagawa_command_build(priv, cmd, gTamagawa_multi_ch_mask);

    if(cmd < 0)
    {
        return cmd;
    }

    /* Trigger sending the tamagawa command in PRU */
    tamagawa_command_send(priv);
    /* Wait till PRU finishes tamagawa transaction */
    tamagawa_command_wait(priv);

    /* In case of EEPROM commands, reset the command id for all the channels back to 0 */
    if (cmd == DATA_ID_6 || cmd == DATA_ID_D)
    {
        priv->tamagawa_xchg->tamagawa_eeprom_interface[0].cmd = 0;
        priv->tamagawa_xchg->tamagawa_eeprom_interface[1].cmd = 0;
        priv->tamagawa_xchg->tamagawa_eeprom_interface[2].cmd = 0;
    }

    return cmd;
}

uint32_t tamagawa_reverse_bits(int8_t data)
{
    /* Reverses the bits of the 8 bit data*/
    /* Loops through all the bits of input data. If the bit at i-th position is set for the input data, then set the bit at (N – 1 – i)-th position in reversed number.
        Here, N is the number of bits in the input data.*/
    uint32_t number_of_bits = sizeof(data) * 8;
    uint32_t reversed_num = 0;
    uint32_t current_bit_pos;
    for (current_bit_pos = 0; current_bit_pos < number_of_bits; current_bit_pos++) {
        if ((data & (1 << current_bit_pos)))
            reversed_num |= 1 << ((number_of_bits - 1) - current_bit_pos);
    }
    return reversed_num;
}

uint64_t tamagawa_prepare_eeprom_tx_data(uint64_t eeprom_tx_data, volatile uint32_t data)
{
    /* Takes the 8 bit data of CF(Control Field), ADF(address of EEPROM), EDF(data for EEPROM) and CRC, reverses it and adds the start and the stop bits */

    /* Call tamagawa_reverse_bits function in order to reverse the bits of the data */
    uint32_t reversed_num  = tamagawa_reverse_bits(data);
    /* Perform left shift of the 64 bit integer holding the EEPROM Tx data 9 times and bitwise-OR it with the reversed number to add the start bit and the reversed data*/
    eeprom_tx_data = eeprom_tx_data<<9;
    eeprom_tx_data = eeprom_tx_data|reversed_num;
    /* Perform left shift of the integer holding the EEPROM Tx data 1 more time and bitwise-OR it with 1 in order to add the stop bit */
    eeprom_tx_data = eeprom_tx_data<<1;
    eeprom_tx_data = eeprom_tx_data|1;
    return eeprom_tx_data;
}

void tamagawa_prepare_eeprom_command(struct tamagawa_priv *priv, int32_t cmd,uint32_t ch)
{
    /* Prepares the required EEPROM command from the CF(Control Field), ADF(address of EEPROM), EDF(data for EEPROM) and CRC */
    uint64_t eeprom_tx_data = 0;
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;
    if (cmd== DATA_ID_6)
    {
        /* Command preparation for EEPROM Write */
        /* Control Field value for EEPROM Write is 0x32 */
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, 0x32);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].adf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].edf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].crc);
    }
    else
    {
        /* Command preparation for EEPROM Read */
        /* Control Field value for EEPROM Read is 0xEA */
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, 0xEA);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].adf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].crc);
        /* 64 bit integer is left shifted twice to load the data byte wise into the Tx FIFO correctly */
        eeprom_tx_data <<= 2;
    }
    /* Assign the value of the prepared Tx data for EEPROM commands to the 64 bit integer eeprom_tx_data for the currently selected channel */
    tamagawa_xchg->tamagawa_eeprom_interface[ch].eeprom_tx_data = eeprom_tx_data;
}

void tamagawa_config_clock(struct tamagawa_priv *priv, struct tamagawa_clk_cfg *clk_cfg)
{
    /* Configures the tamagawa clock */
    void *pruss_cfg = priv->pruss_cfg;
    /* Configure the PRUx Rx CFG register by writing the Rx Divide Factor and Oversampling rate */
    HW_WR_REG32((uint32_t)(pruss_cfg) + tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_RXCFG, (uint32_t)(clk_cfg->rx_div << 16 | 0x8 | clk_cfg->rx_div_attr));
    /* Configure the PRUx Tx CFG register by writing the Tx Divide Factor  */
    HW_WR_REG16((uint32_t)(pruss_cfg) + tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_TXCFG + 2, (uint16_t)(clk_cfg->tx_div));
}

void tamagawa_config_host_trigger(struct tamagawa_priv *priv)
{
    /* Configures tamagawa master for host trigger mode */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;

    tamagawa_xchg->config.opmode = 0x1;
}

void tamagawa_config_periodic_trigger(struct tamagawa_priv *priv)
{
    /* Configures tamagawa master in periodic trigger mode */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;

    tamagawa_xchg->config.opmode = 0;
}

void tamagawa_config_channel(struct tamagawa_priv *priv, uint32_t ch)
{
    /* Selects the channel to be used by tamagawa master */
    struct tamagawa_xchg *tamagawa_xchg = priv->tamagawa_xchg;

    tamagawa_xchg->config.channel = 1 << ch;
    priv->channel = ch;
    tamagawa_xchg->tamagawa_interface.ch_mask = 1 << ch;
}

void tamagawa_config_multi_channel_mask(struct tamagawa_priv *priv, uint8_t mask)
{
    /* Selects mask of channels to be used in multi channel configuration */
    priv->tamagawa_xchg->config.channel = mask;
}

void tamagawa_update_data_id(struct tamagawa_priv *priv, int32_t cmd)
{
    /* Updates the data id according to the command selected */
    priv->tamagawa_xchg->tamagawa_interface.data_id= cmd;
    if(cmd == DATA_ID_6 || cmd == DATA_ID_D)
    {
        priv->tamagawa_xchg->tamagawa_eeprom_interface[0].cmd = cmd;
        priv->tamagawa_xchg->tamagawa_eeprom_interface[1].cmd = cmd;
        priv->tamagawa_xchg->tamagawa_eeprom_interface[2].cmd = cmd;
    }
}

void tamagawa_update_adf(struct tamagawa_priv *priv, uint32_t val, uint32_t ch)
{
    /* Updates the ADF field for the currently selected channel for EEPROM commands */
    priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].adf = val;
}

void tamagawa_update_edf(struct tamagawa_priv *priv, uint32_t val, uint32_t ch)
{
    /* Updates the EDF field for the currently selected channel for EEPROM Write command */
    priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].edf = val;
}

void tamagawa_eeprom_crc_reinit(struct tamagawa_priv *priv)
{
    /* Resets the integers used to calculate the CRC back to 0 for all the channels */
    uint32_t ch = 0;
    for(ch = 0; ch < MAX_CHANNELS; ch++)
    {
        priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = 0;
        priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].word1 = 0;
        priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].word2 = 0;
    }
}

uint8_t tamagawa_multi_channel_detected(struct tamagawa_priv *priv)
{
    /* Detects the channels that have been selected in multi-channel configuration */
    return priv->tamagawa_xchg->config.channel;
}

void tamagawa_multi_channel_set_cur(struct tamagawa_priv *priv, uint32_t ch)
{
    /* Set the channel that is currently being processed */
    priv->channel = ch;
}

static inline void tamagawa_config_clr_cfg0(struct tamagawa_priv *priv)
{
    void *pruss_cfg = priv->pruss_cfg;
    /* Clear the CFG0 registers for the selected PRUx slice for Channel 0 */
    HW_WR_REG32((uint32_t)(pruss_cfg) + tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG0, 0);
    /* Clear the CFG0 registers for the selected PRUx slice for Channel 1 */
    HW_WR_REG32((uint32_t)(pruss_cfg) + tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG0, 0);
    /* Clear the CFG0 registers for the selected PRUx slice for Channel 2 */
    HW_WR_REG32((uint32_t)(pruss_cfg) + tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG0, 0);
}

struct tamagawa_priv *tamagawa_init(struct tamagawa_xchg *tamagawa_xchg, void *pruss_cfg, uint32_t slice_value)
{
    /* Initializes tamagawa firmware interface address and get the pointer to struct tamagawa_priv instance */
    tamagawa_priv.tamagawa_xchg = tamagawa_xchg;
    tamagawa_priv.pruss_cfg = pruss_cfg;
    tamagawa_priv.slice_value = slice_value;

    /* If the slice value is 0, it denotes that PRU0 is selected. Assign the register offsets for PRU0 */
    if(slice_value==0)
    {
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG0 = CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG0 = CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG1 = CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG1 = CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG0 = CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG1 = CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_GPCFGx =    CSL_ICSSCFG_GPCFG0;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_RXCFG = CSL_ICSSCFG_EDPRU0RXCFGREGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_TXCFG = CSL_ICSSCFG_EDPRU0TXCFGREGISTER;
    }
    /* If the slice value is 1, it denotes that PRU1 is selected. Assign the register offsets for PRU1 */
    else if(slice_value==1)
    {
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG0 = CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG0 = CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG0 = CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG1 = CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG1 = CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG1 = CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_GPCFGx =    CSL_ICSSCFG_GPCFG1;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_RXCFG = CSL_ICSSCFG_EDPRU1RXCFGREGISTER;
        tamagawa_priv.register_offset_val.ICSS_CFG_PRUx_ED_TXCFG = CSL_ICSSCFG_EDPRU1TXCFGREGISTER;
    }

    return &tamagawa_priv;
}
