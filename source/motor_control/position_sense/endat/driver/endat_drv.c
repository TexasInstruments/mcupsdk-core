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

#include <motor_control/position_sense/endat/include/endat_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>

static struct endat_priv endat_priv;

#define ICSS_CFG_PRUx_ENDAT_CH0_CFG0    CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER
#define ICSS_CFG_PRUx_ENDAT_CH1_CFG0    CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER
#define ICSS_CFG_PRUx_ENDAT_CH2_CFG0    CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER
#define ICSS_CFG_PRUx_ENDAT_CH0_CFG1    CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER
#define ICSS_CFG_PRUx_ENDAT_CH1_CFG1    CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER
#define ICSS_CFG_PRUx_ENDAT_CH2_CFG1    CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER
#define ICSS_CFG_GPCFGx    				CSL_ICSSCFG_GPCFG1
#define ICSS_CFG_PRUx_ENDAT_RXCFG       CSL_ICSSCFG_EDPRU1RXCFGREGISTER
#define ICSS_CFG_PRUx_ENDAT_TXCFG       CSL_ICSSCFG_EDPRU1TXCFGREGISTER
/*
 * check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that functions,
 * endat_recvd_organize()
 * endat_recvd_format()
 * endat_recvd_validate()
 * endat_recvd_print()
 * will normally be called after endat_get_command
 */

static int endat_recvd_organize(int cmd, struct endat_priv *priv,
                                struct endat_data *endat_data)
{
    unsigned word0, word1, word2, word3;
    unsigned pos_bits, shift;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    memset(endat_data, 0, sizeof(*endat_data));

    word0 = pruss_xchg->ch[priv->channel].pos_word0;
    word1 = pruss_xchg->ch[priv->channel].pos_word1;
    word2 = pruss_xchg->ch[priv->channel].pos_word2;
    word3 = pruss_xchg->ch[priv->channel].pos_word3;

    switch(cmd)
    {
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 14:
            endat_data->recvd1 = word0;
            break;

        case 6:
            shift = ENDAT_RX_46BITS % (sizeof(unsigned) * 8);
            endat_data->recvd1 = (unsigned long long) word0 << shift | word1;
            break;

        case 1:
            pos_bits = priv->pos_res + ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1;

            if(pos_bits <= sizeof(unsigned) * 8)
            {
                endat_data->recvd1 = word0;
            }
            else
            {
                shift = pos_bits % (sizeof(unsigned) * 8);
                endat_data->recvd1 = (unsigned long long) word0 << shift | word1;
            }
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            pos_bits = priv->pos_res + ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1 +
                       ENDAT_NUM_BITS_F2;

            if(pos_bits <= sizeof(unsigned) * 8)
            {
                endat_data->recvd1 = word0;
            }
            else
            {
                shift = pos_bits % (sizeof(unsigned) * 8);
                endat_data->recvd1 = (unsigned long long) word0 << shift | word1;
            }
            if(priv->flags.info1 || priv->flags.info2)
            {
                endat_data->recvd2 = word2;
            }
            if(priv->flags.info1 && priv->flags.info2)
            {
                endat_data->recvd3 = word3;
            }
            break;

        default:
            return -EINVAL;
            break;
    }

    return 0;
}

/* value to be reflected should be aligned to lsb, relected value would be aligned to lsb */
static unsigned long long endat_reflect_ull_nbits(unsigned long long input,
        unsigned n)
{
    unsigned i;
    unsigned long long val = 0;

    /* val initialized to 0 above to handle n = 0 case, otherwise garbage would be returned */
    for(i = 0; i < n; i++)
    {
        val <<= 1;

        if(input & ((unsigned long long) 1 << i))
        {
            val |= 1;
        }
    }

    return val;
}

static int endat_recvd_format(int cmd, struct endat_priv *priv,
                              struct endat_data *endat_data, union endat_format_data *u)
{
    unsigned long long pos, rev;

    switch(cmd)
    {
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 14:
            u->addr_params.params = (endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC)
                                    & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
            u->addr_params.address = (endat_data->recvd1 >> (ENDAT_NUM_BITS_POSITION_CRC +
                                      ENDAT_NUM_BITS_PARAMETER)) &
                                     ((1 << ENDAT_NUM_BITS_ADDRESS) - 1);
            u->addr_params.crc = endat_data->recvd1 & ((1 << ENDAT_NUM_BITS_POSITION_CRC)
                                 - 1);
            break;

        case 6:
            u->test.value = (endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC) & (((
                                unsigned long long)1 << 40) - 1);
            u->test.f1 = (endat_data->recvd1 >> (ENDAT_NUM_BITS_POSITION_CRC + 40)) & 1;
            u->test.crc = endat_data->recvd1 & ((1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            break;

        case 1:
            pos = endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC;
            pos = pos & (((unsigned long long) 1 << priv->pos_res) - 1);       /* mask F1 */
            pos = endat_reflect_ull_nbits(pos, priv->pos_res);
            rev = (pos & (((unsigned long long) 1 << priv->pos_res) - 1)) >>
                  priv->single_turn_res;
            pos = pos & (((unsigned long long) 1 << priv->single_turn_res) - 1);
            u->position_addinfo.position.position = pos;
            u->position_addinfo.position.revolution = rev;
            u->position_addinfo.position.f1 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + priv->pos_res)) & 1;
            u->position_addinfo.position.crc = endat_data->recvd1 & ((
                                                   1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            pos = endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC;
            pos = pos & (((unsigned long long) 1 << priv->pos_res) -
                         1);  /* mask F1/F2 */
            pos = endat_reflect_ull_nbits(pos, priv->pos_res);
            rev = (pos & (((unsigned long long) 1 << priv->pos_res) - 1)) >>
                  priv->single_turn_res;
            pos = pos & (((unsigned long long) 1 << priv->single_turn_res) - 1);
            u->position_addinfo.position.position = pos;
            u->position_addinfo.position.revolution = rev;
            u->position_addinfo.position.f1 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + priv->pos_res + 1)) & 1;
            u->position_addinfo.position.f2 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + priv->pos_res)) & 1;
            u->position_addinfo.position.crc = endat_data->recvd1 & ((
                                                   1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);

            if(priv->flags.info1 || priv->flags.info2)
            {
                if(endat_priv.flags.info2)
                {
                    u->position_addinfo.addinfo2.addinfo = (endat_data->recvd2 >>
                                                            ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                    u->position_addinfo.addinfo2.crc = endat_data->recvd2 & ((
                                                           1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
                }
                else
                {
                    u->position_addinfo.addinfo1.addinfo = (endat_data->recvd2 >>
                                                            ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                    u->position_addinfo.addinfo1.crc = endat_data->recvd2 & ((
                                                           1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
                }
            }

            if(priv->flags.info1 && priv->flags.info2)
            {
                u->position_addinfo.addinfo1.addinfo = (endat_data->recvd3 >>
                                                        ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                u->position_addinfo.addinfo1.crc = endat_data->recvd3 & ((
                                                       1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            }
            break;

        default:
            return -EINVAL;
            break;
    }

    return 0;
}

int endat_recvd_process(struct endat_priv *priv, int cmd,
                        union endat_format_data *u)
{
    struct endat_data endat_data;
    int ret;

    ret = endat_recvd_organize(cmd, priv, &endat_data);

    if(ret < 0)
    {
        return ret;
    }

    return endat_recvd_format(cmd, priv, &endat_data, u);
}

#define ENDAT_USE_OTF_CRC_STATUS

#ifndef ENDAT_USE_OTF_CRC_STATUS
static unsigned int make_crc_norm(unsigned int param8, unsigned int param16)
{
    /* state of the 5 flip-flops */
    unsigned int ff[5];
    /* data bit array */
    unsigned int code[24];
    /* Auxiliary variable */
    unsigned int ex;
    /* dtetermined CRC code */
    unsigned int crc = 0;
    /* controlled variable for looping */
    signed int i;

    /* set all flip-flops to 1 */
    for(i = 0; i < 5; i++)
    {
        ff[i] = 1;
    }

    /* read 8 bit parameter into code array and convert bit sequence */
    for(i = 0; i < 8; i++)
    {
        code[i] = (param8 & 0x0080) ? 1 : 0;
        param8 <<= 1;
    }

    /* read 16 bit parameter into code array and convert bit sequence */
    for(i = 8; i < 24; i++)
    {
        code[i] = (param16 & 0x8000) ? 1 : 0;
        param16 <<= 1;
    }

    /* calculate crc analog to described h/w generator */
    for(i = 0; i < 24; i++)
    {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    /* store crc in variable */
    for(i = 4; i >= 0; i--)
    {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }

    return crc;
}

static unsigned int make_crc_pos(unsigned int clocks, unsigned int error1,
                                 unsigned int error2, unsigned int endat22,
                                 unsigned long highpos, unsigned long lowpos)
{
    /* state of the 5 flip-flops */
    unsigned int ff[5];
    /* data bit array */
    unsigned int code[66];
    /* Auxiliary variable */
    unsigned int ex;
    /* dtetermined CRC code */
    unsigned int crc = 0;
    /* controlled variable for looping */
    signed int i;

    /* set all flip-flops to 1 */
    for(i = 0; i < 5; i++)
    {
        ff[i] = 1;
    }

    /* transfer alarm bits to code array */
    if(endat22)
    {
        code[0] = error1;
        code[1] = error2;
    }
    else
    {
        code[1] = error1;
    }

    /* transfer low pos bits to array */
    for(i = 2; i < 34; i++)
    {
        code[i] = (lowpos & 0x00000001L) ? 1 : 0;
        lowpos >>= 1;
    }

    /* transfer high pos bits to array */
    for(i = 34; i < 66; i++)
    {
        code[i] = (highpos & 0x00000001L) ? 1 : 0;
        highpos >>= 1;
    }

    /* calculate crc analog to described h/w generator */
    for(i = (endat22 ? 0 : 1); i <= (clocks + 1); i++)
    {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    /* store crc in variable */
    for(i = 4; i >= 0; i--)
    {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }

    return crc;
}
#endif

/* return crc status: bit0 - position/address params/test, bit1 - additional info1, bit2 - additional info2. return -EINVAL on failure */
unsigned endat_recvd_validate(struct endat_priv *priv, int cmd,
                              union endat_format_data *u)
{
    unsigned status = 0;
#ifdef ENDAT_USE_OTF_CRC_STATUS
    unsigned char val;
#else
    unsigned int crc;
    unsigned long highpos, lowpos;
    unsigned long long test;
#endif

#ifdef ENDAT_USE_OTF_CRC_STATUS
    val = priv->pruss_xchg->ch[priv->channel].crc.status;

    if(priv->flags.info2)
    {
        status = val & 0x1;
        /* move bit 1 to bit 2 */
        status |= (val & 0x2) << 1;

        if(priv->flags.info1)
        {
            /* move bit 2 to bit 1 */
            status |= (val & 0x4) >> 1;
        }
    }

    else
    {
        /* either additional info1 only present or no additional info present */
        status = val & 0x3;
    }

#else

    switch(cmd)
    {
        case 1:
            lowpos = (u->position_addinfo.position.revolution << priv->single_turn_res |
                      u->position_addinfo.position.position) & 0xFFFFFFFF;
            highpos = (u->position_addinfo.position.revolution << priv->single_turn_res |
                       u->position_addinfo.position.position) >> 32;
            crc = make_crc_pos(priv->pos_res, u->position_addinfo.position.f1, 0, 0,
                               highpos, lowpos);

            if(u->position_addinfo.position.crc == crc)
            {
                status = 0x1;
            }
            break;

        case 6:
            test = endat_reflect_ull_nbits(u->test.value, 40);
            lowpos = test & 0xFFFFFFFF;
            highpos = test >> 32;
            crc = make_crc_pos(40, u->position_addinfo.position.f1, 0, 0, highpos,
                               lowpos);

            if(u->test.crc == crc)
            {
                status = 0x1;
            }
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            lowpos = (u->position_addinfo.position.revolution << priv->single_turn_res |
                      u->position_addinfo.position.position) & 0xFFFFFFFF;
            highpos = (u->position_addinfo.position.revolution << priv->single_turn_res |
                       u->position_addinfo.position.position) >> 32;
            crc = make_crc_pos(priv->pos_res, u->position_addinfo.position.f1,
                               u->position_addinfo.position.f2, 1, highpos, lowpos);

            if(u->position_addinfo.position.crc == crc)
            {
                status = 0x1;
            }

            if(priv->flags.info1)
            {
                crc = make_crc_norm((u->position_addinfo.addinfo1.addinfo >> 16) & 0xFF,
                                    u->position_addinfo.addinfo1.addinfo & 0xFFFF);

                if(u->position_addinfo.addinfo1.crc == crc)
                {
                    status |= 0x1 << 1;
                }
            }

            if(priv->flags.info2)
            {
                crc = make_crc_norm((u->position_addinfo.addinfo2.addinfo >> 16) & 0xFF,
                                    u->position_addinfo.addinfo2.addinfo & 0xFFFF);

                if(u->position_addinfo.addinfo2.crc == crc)
                {
                    status |= 0x1 << 2;
                }
            }

            break;

        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
            crc = make_crc_norm(u->addr_params.address, u->addr_params.params);

            if(u->addr_params.crc == crc)
            {
                status = 0x1;
            }
            break;

        default:
            break;
    }

#endif

    return status;
}

/*
 * XXX: check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that this function will be called either after endat_get_command
 * or by diagnostic initialization code where it is only 2.1 commands used
 */
int endat_command_build(struct endat_priv *priv, int cmd,
                        struct cmd_supplement *cmd_supplement)
{
    unsigned info;
    struct endat_pruss_xchg *endat_pruss_xchg = priv->pruss_xchg;

    info = 0, priv->flags.info1 ? info++ : 0, priv->flags.info2 ? info++ : 0 ;

    /*first clear command parameters to be safe */
    memset(&endat_pruss_xchg->cmd, 0, sizeof(endat_pruss_xchg->cmd));

    switch(cmd)
    {
        case 1:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSITION_VALUES;
            endat_pruss_xchg->cmd.word1 = priv->pos_rx_bits_21 | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);
            break;

        case 2:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEL_MEM_AREA;
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruss_xchg->cmd.word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            break;

        case 3:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_RECEIVE_PARAMETERS;
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
            endat_pruss_xchg->cmd.word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            break;

        case 4:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_PARAMETERS;
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruss_xchg->cmd.word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            break;

        case 5:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_RECEIVE_RESET;
            endat_pruss_xchg->cmd.word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            break;

        case 6:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_TEST_VALUES;
            endat_pruss_xchg->cmd.word1 = ENDAT_RX_46BITS | (ENDAT_TX_6BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            break;

        case 7:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_RECEIVE_TEST_COMMAND;
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruss_xchg->cmd.word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            break;

        case 8:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
            /*
             * Though this is not 2.1 command, fw expects it to be ENDAT_CMDTYP_2_1. Ideally macro should
             *  have been named ENDAT_CMDTYPE_HAVE_2_2_SUPPLEMENT instead of ENDAT_CMDTYP_2_[12] for readability
             */
            endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22)
                                           << 16);

            if(priv->flags.info1)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
            }

            if(priv->flags.info2)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
            }

            break;

        case 9:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL;

            if(cmd_supplement->has_block_address)
            {
                endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16) | (1 << 24);
                endat_pruss_xchg->cmd.word2 = cmd_supplement->address |
                                              (cmd_supplement->block << 24);
            }

            else
            {
                endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16);
                endat_pruss_xchg->cmd.word2 = cmd_supplement->address;
            }

            if(priv->flags.info1)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
            }

            if(priv->flags.info2)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
            }

            break;

        case 10:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
            endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
            endat_pruss_xchg->cmd.word2 = cmd_supplement->address;
            /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
            endat_pruss_xchg->cmd.word2 |= ((cmd_supplement->data & 0xFF) << 16) |
                                           (cmd_supplement->data & 0xFF00);

            if(priv->flags.info1)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
            }

            if(priv->flags.info2)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
            }

            break;

        case 11:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
            endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
            endat_pruss_xchg->cmd.word2 = cmd_supplement->address;

            if(priv->flags.info1)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
            }

            if(priv->flags.info2)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
            }

            break;

        case 12:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
            endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

            if(priv->flags.info1)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
            }

            if(priv->flags.info2)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
            }

            break;

        case 13:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
            endat_pruss_xchg->cmd.word1 = (priv->pos_rx_bits_22 + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
            endat_pruss_xchg->cmd.word2 = cmd_supplement->address;

            if(priv->flags.info1)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
            }

            if(priv->flags.info2)
            {
                endat_pruss_xchg->cmd.word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
            }

            break;

        case 14:
            endat_pruss_xchg->cmd.word0 = ENDAT_CMD_RECEIVE_COMMUNICATION_CMD;
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruss_xchg->cmd.word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
            endat_pruss_xchg->cmd.word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_ENDAT22) << 16);
            break;

        default:
            cmd = -EINVAL;
            break;
    }

    return cmd;
}

void endat_command_send(struct endat_priv *priv)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    pruss_xchg->config.trigger = 0x1;
}

void endat_command_wait(struct endat_priv *priv)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    while(pruss_xchg->config.trigger & 0x1)
        ;
}

int endat_command_process(struct endat_priv *priv, int cmd,
                          struct cmd_supplement *cmd_supplement)
{
    cmd = endat_command_build(priv, cmd, cmd_supplement);

    if(cmd < 0)
    {
        return cmd;
    }

    endat_command_send(priv);
    endat_command_wait(priv);
    return cmd;
}

int endat_get_2_2_angle(struct endat_priv *priv)
{
    int pos;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;
    int ch = priv->channel;


    if(!(pruss_xchg->ch[ch].crc.status & ENDAT_CRC_DATA))
    {
        return -1;
    }

    pos = pruss_xchg->ch[ch].pos_word0;

#ifdef __TI_ARM__
    pos = __rbit(pos);
#else
    asm("rbit %0,%1" : "=r"(pos) : "r"(pos));
#endif

    /* verify F1 = 0 & F2 = 1 */
    if((pos & 0x3) != 0x2)
    {
        return -1;
    }

    /* discard F1, F2 */
    pos >>= 2;

    /* mask non-angular bits */
    pos &= (1 << priv->single_turn_res) - 1;
    return pos;
}

static int endat_get_pos_res(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw, absence of delay here resulted in wrong values for pos_res */
    ClockP_usleep(1000 * 12);

    /* send parameter for word13 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_13;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    return word &= (1 << ENDAT_NUM_BITS_VALID_PAGE0_WORD13) - 1;
}

static int endat_get_multi_turn_res(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word1 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_1;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    return word &= (1 << ENDAT_NUM_BITS_VALID_PAGE1_WORD1) - 1;
}

static int endat_get_id(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word0, word1, word2;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 1 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word8 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_8;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word0 = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word9 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_9;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word1 = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word10 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_10;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word2 = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    priv->id.binary = word1 | word2 << 16;
    /* swap the two ascii's so that printing as string will give what is required */
    priv->id.ascii = ((word0 & 0xFF) << 8) | ((word0 & 0xFF00) >> 8);

    return 0;
}

static int endat_get_sn(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word0, word1, word2;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 1 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word11 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_11;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word0 = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word12 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_12;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word1 = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word13 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_13;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word2 = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    priv->sn.ascii_lsb = word0 & 0xFF;
    priv->sn.binary = ((word0 & 0xFF00) >> 8) | (word1 << 8) | ((
                          word2 & 0xFF) << 24);
    priv->sn.ascii_msb = (word2 & 0xFF00) >> 8;

    return 0;
}

static int endat_get_command_set(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 2 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word5 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_5;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    priv->cmd_set_2_2 = (word & 0x1) && !(word & 0x2);
    priv->has_safety = (word & 0x4) && !(word & 0x8);

    return 0;
}

static int endat_get_type(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw, absence of delay here resulted in wrong values for pos_res */
    ClockP_usleep(1000 * 12);

    /* send parameter for word13 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_14;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    priv->type = (word & (1 << 15)) ? rotary : linear;

    return 0;
}

static int endat_get_step(struct endat_priv *priv)
{
    int cmd;
    struct cmd_supplement cmd_supplement;
    unsigned word;
    int ch = priv->channel;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word4 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_4;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);
    word = (pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);

    /* send parameter for word5 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_5;

    if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);
    word |= ((pruss_xchg->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
             & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1)) << 16;

    return word;
}

int endat_get_encoder_info(struct endat_priv *priv)
{
    int ret;

    priv->pos_res = endat_get_pos_res(priv);

    if(priv->pos_res < 0)
    {
        return priv->pos_res;
    }

    priv->multi_turn_res = endat_get_multi_turn_res(priv);

    if(priv->multi_turn_res < 0)
    {
        return priv->multi_turn_res;
    }

    else if(!priv->multi_turn_res)
    {
        priv->single_turn_res = priv->pos_res;
    }

    else
    {
        priv->multi_turn_res = log2(priv->multi_turn_res);
        priv->single_turn_res = priv->pos_res - priv->multi_turn_res;
    }

    priv->step = endat_get_step(priv);

    if(priv->step < 0)
    {
        return priv->step;
    }

    priv->pos_rx_bits_21 = priv->pos_res + ENDAT_NUM_BITS_POSITION_CRC +
                           ENDAT_NUM_BITS_F1;
    priv->pos_rx_bits_22 = priv->pos_rx_bits_21 + ENDAT_NUM_BITS_F2;

    ret = endat_get_id(priv);

    if(ret)
    {
        return ret;
    }

    ret = endat_get_sn(priv);

    if(ret)
    {
        return ret;
    }

    ret = endat_get_type(priv);

    if(ret)
    {
        return ret;
    }

    ret = endat_get_command_set(priv);

    if(ret)
    {
        return ret;
    }

    return 0;
}

unsigned int endat_get_prop_delay(struct endat_priv *priv)
{
    return priv->pruss_xchg->ch[priv->channel].prop_delay;
}

void endat_addinfo_track(struct endat_priv *priv, int cmd,
                         struct cmd_supplement *cmd_supplement)
{
    unsigned c7_c4, c3_c0;

    /* reset stops additional info's */
    if(cmd == 5)
    {
        priv->flags.info1 = FALSE, priv->flags.info2 = FALSE;
    }

    if(cmd != 9)
    {
        return;
    }

    c7_c4 = (cmd_supplement->address & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
            ENDAT_MRS_SHIFT_C7_C4;
    c3_c0 = cmd_supplement->address & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

    if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO1)
    {
        if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
        {
            priv->flags.info1 = FALSE;
        }

        else
        {
            priv->flags.info1 = TRUE;
        }
    }

    c7_c4 = (cmd_supplement->address & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
            ENDAT_MRS_SHIFT_C7_C4;
    c3_c0 = cmd_supplement->address & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

    if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO2)
    {
        if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
        {
            priv->flags.info2 = FALSE;
        }

        else
        {
            priv->flags.info2 = TRUE;
        }
    }
}

static void endat_config_global_rx_arm_cnt(struct endat_priv *priv,
        unsigned short val)
{
    void *pruss_cfg = priv->pruss_cfg;

    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH0_CFG1 + 2) =  val;
    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH1_CFG1 + 2) =  val;
    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH2_CFG1 + 2) =  val;
}

void endat_config_rx_arm_cnt(struct endat_priv *priv, unsigned short val)
{
    void *pruss_cfg = priv->pruss_cfg;
    int ch = priv->channel;

    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH0_CFG1 + ch * 8 + 2) =  val;
}

void endat_config_wire_delay(struct endat_priv *priv, unsigned short val)
{
    void *pruss_cfg = priv->pruss_cfg;
    int ch = priv->channel;
    unsigned short regval = HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH0_CFG0 + ch *
                                   8);

    /* clear wire delay bits, keep other bits as is */
    regval &= 0xF800;
    /* restrict wire delay to wire delay bits only */
    val &= 0x7FF;
    regval |= val;

    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH0_CFG0 + ch * 8) =  regval;
}

void endat_config_clock(struct endat_priv *priv,
                        struct endat_clk_cfg *clk_cfg)
{
    void *pruss_cfg = priv->pruss_cfg;
    /* Set PRU1_ED_RX_SB_POL polarity bit, required for ICSSG (don't care for ICSSM) */
    HWREG((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_RXCFG) = clk_cfg->rx_div << 16 | 0x8 |
            clk_cfg->rx_div_attr;
    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_TXCFG + 2) = clk_cfg->tx_div;
}

void endat_config_tst_delay(struct endat_priv *priv, unsigned short delay)
{
    void *pruss_cfg = priv->pruss_cfg;

    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH0_CFG1) = delay;
    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH1_CFG1) = delay;
    HWREGH((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH2_CFG1) = delay;
}

void endat_config_rx_clock_disable(struct endat_priv *priv,
                                   unsigned short val)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;
    int ch = priv->channel;

    pruss_xchg->ch[ch].rx_clk_less = val;
}

static void endat_set_continuous_mode(struct endat_priv *priv)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    pruss_xchg->config.trigger |= (0x1 << 7 | 0x1);
}

static void endat_clear_continuous_mode(struct endat_priv *priv)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    pruss_xchg->config.trigger &= ~(0x1 << 7);
}

int endat_start_continuous_mode(struct endat_priv *priv)
{
    int cmd;

    cmd = endat_command_build(priv, 1, NULL);

    if(cmd < 0)
    {
        return cmd;
    }

    endat_set_continuous_mode(priv);

    return cmd;
}

void endat_stop_continuous_mode(struct endat_priv *priv)
{
    endat_clear_continuous_mode(priv);
    endat_command_wait(priv);
}

void endat_config_host_trigger(struct endat_priv *priv)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    pruss_xchg->config.opmode = 0x1;
}

void endat_config_periodic_trigger(struct endat_priv *priv)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    pruss_xchg->config.opmode = 0;
}

void endat_config_channel(struct endat_priv *priv, int ch)
{
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    pruss_xchg->config.channel = 1 << ch;
    priv->channel = ch;
}

void endat_config_multi_channel_mask(struct endat_priv *priv,
                                     unsigned char mask)
{
    priv->pruss_xchg->config.channel = mask;
}

unsigned char endat_multi_channel_detected(struct endat_priv *priv)
{
    return priv->pruss_xchg->config.channel;
}

void endat_multi_channel_set_cur(struct endat_priv *priv, int ch)
{
    priv->channel = ch;
}

int endat_wait_initialization(struct endat_priv *priv, unsigned timeout)
{
    int i;
    struct endat_pruss_xchg *pruss_xchg = priv->pruss_xchg;

    for(i = 0; i < timeout; i++)
    {
        if(pruss_xchg->config.status & 1)
        {
            break;
        }
        else
        {
            ClockP_usleep(1000 * 1);
        }
    }
    if(i == timeout)
    {
        return -1;
    }

    return 0;
}

static inline void endat_config_clr_cfg0(struct endat_priv *priv)
{
    void *pruss_cfg = priv->pruss_cfg;

    HWREG((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH0_CFG0) = 0;
    HWREG((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH1_CFG0) = 0;
    HWREG((uint8_t *)pruss_cfg + ICSS_CFG_PRUx_ENDAT_CH2_CFG0) = 0;
}

static inline void endat_config_endat_mode(struct endat_priv *priv)
{
    void *pruss_cfg = priv->pruss_cfg;

    HWREGB((uint8_t *)pruss_cfg + ICSS_CFG_GPCFGx + 3) =  4;
}

static void endat_hw_init(struct endat_priv *priv)
{
    struct endat_clk_cfg clk_cfg;

    /* set initial clock to 200KHz */
    /* 192MHz / 200KHz - 1 */
    clk_cfg.tx_div = 959;
    /* 192MHz / 200KHz / 8 - 1 */
    clk_cfg.rx_div = 119;
    /* 2T */
    clk_cfg.rx_en_cnt = 10000;
    /* sample size 8 */
    clk_cfg.rx_div_attr = 7;

    endat_config_endat_mode(priv);
    endat_config_clock(priv, &clk_cfg);
    endat_config_global_rx_arm_cnt(priv, clk_cfg.rx_en_cnt);
    endat_config_clr_cfg0(priv);
}

struct endat_priv *endat_init(struct endat_pruss_xchg *pruss_xchg,
                              void *pruss_cfg)
{

    endat_priv.pruss_xchg = pruss_xchg;
    endat_priv.pruss_cfg = pruss_cfg;
    endat_hw_init(&endat_priv);

    return &endat_priv;
}
