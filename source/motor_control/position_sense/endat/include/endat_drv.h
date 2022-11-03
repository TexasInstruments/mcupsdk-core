/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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


#ifndef ENDAT_DRV_H_
#define ENDAT_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>
#include <motor_control/position_sense/endat/include/endat_interface.h>
#define ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU (0U)
#define ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU (1U)
#define ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU (2U)

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))
#define HWREGB(x)                                                              \
        (*((volatile uint8_t *)(x)))
#define HWREGH(x)                                                              \
        (*((volatile uint16_t *)(x)))


/*12 words */
#define MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0   0xA1
/*16 words */
#define MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1   0xA3
/*16 words*/
#define MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2   0xA5

/* Adjust 1-bit as we transmit early */
#define     ENDAT_TX_30BITS (30 + 1)
/*Adjust 1-bit as we transmit early */
#define     ENDAT_TX_6BITS  (6 + 1)
#define     ENDAT_RX_29BITS 29
/* For EnDat 2.1: encoder to send test values */
#define     ENDAT_RX_46BITS 46

#define APP_ENDAT_WORD_0  0
#define APP_ENDAT_WORD_1  1
#define APP_ENDAT_WORD_2  2
#define APP_ENDAT_WORD_3  3
#define APP_ENDAT_WORD_4  4
#define APP_ENDAT_WORD_5  5
#define APP_ENDAT_WORD_6  6
#define APP_ENDAT_WORD_7  7
#define APP_ENDAT_WORD_8  8
#define APP_ENDAT_WORD_9  9
#define APP_ENDAT_WORD_10 10
#define APP_ENDAT_WORD_11 11
#define APP_ENDAT_WORD_12 12
#define APP_ENDAT_WORD_13 13
#define APP_ENDAT_WORD_14 14
#define APP_ENDAT_WORD_15 15
#define APP_ENDAT_WORD_16 16
#define APP_ENDAT_WORD_17 17
#define APP_ENDAT_WORD_18 18
#define APP_ENDAT_WORD_19 19

#define ENDAT_NUM_BITS_POSITION_CRC 5
#define ENDAT_NUM_BITS_F1       1
#define ENDAT_NUM_BITS_F2       1
#define ENDAT_NUM_BITS_PARAMETER    16
#define ENDAT_NUM_BITS_ADDRESS      8

#define EINVAL  1

struct endat_clk_cfg
{
    uint16_t  rx_div;
    uint16_t  tx_div;
    uint16_t  rx_en_cnt;
    uint16_t  rx_div_attr;
};

struct flags
{
    uint32_t info1 : 1;
    uint32_t info2 : 1;
};

struct id
{
    uint32_t ascii;
    uint32_t binary;
};

struct sn
{
    uint32_t ascii_msb;
    uint32_t binary;
    uint32_t ascii_lsb;
};

enum { linear, rotary };

struct endat_priv
{   int32_t pruicss_slicex;
    int32_t load_share;
    int32_t pos_res;
    int32_t single_turn_res;
    int32_t multi_turn_res;
    int32_t step;
    uint32_t pos_rx_bits_21_RTUPRU;
    uint32_t pos_rx_bits_21_PRU;
    uint32_t pos_rx_bits_21_TXPRU;
    uint32_t pos_rx_bits_22_RTUPRU;
    uint32_t pos_rx_bits_22_PRU;
    uint32_t pos_rx_bits_22_TXPRU;
    struct flags flags;
    struct id id;
    struct sn sn;
    uint32_t cmd_set_2_2;
    int32_t type;
    int32_t raw_data;
    int32_t channel;
    uint16_t rx_en_cnt;
    struct endat_pruss_xchg *pruss_xchg;
    int32_t has_safety;
    void *pruss_cfg;
};

struct cmd_supplement
{
    /*
     * uint8_t is ideal for address, but as scanf(%x) is done,
     * there could be side effects as other fields may get corrupted,
     * though it is not much of an issue as address is ensured to be
     * less than 0xff. But to be on safer side keep it unsigned,
     * same applies to block also
     */
    uint32_t address;
    uint32_t data;
    uint32_t block;
    uint8_t has_block_address;
    uint32_t frequency;
};

struct endat_data
{
    /* position */
    uint64_t recvd1;
    /* addinfo 1/2 */
    uint32_t recvd2;
    /* addinfo 1 */
    uint32_t recvd3;
};

struct endat_position
{
    uint64_t                position;
    uint64_t                revolution;
    uint8_t           f1;
    uint8_t           f2;
    uint8_t           crc;
};

struct endat_addinfo
{
    uint32_t            addinfo;
    uint8_t           crc;
};

struct endat_position_addinfo
{
    struct endat_position       position;
    struct endat_addinfo        addinfo1, addinfo2;
};

struct endat_addr_params
{
    uint8_t           address;
    uint16_t          params;
    uint8_t           crc;
};

struct endat_test_values
{
    uint64_t      value;
    uint8_t           f1;
    uint8_t           crc;
};

union endat_format_data
{
    struct endat_position_addinfo   position_addinfo;
    struct endat_addr_params    addr_params;
    struct endat_test_values    test;
};

#define VALID_2_1_CMD(x) (((x) == 1) || ((x) == 2) || ((x) == 3) || ((x) == 4) || ((x) == 5) || ((x) == 6) || ((x) == 7) )
#define VALID_2_2_CMD(x) (((x) == 8) || ((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 12) || ((x) == 13) || ((x) == 14))

#define ENDAT_NUM_ADDITIONAL_INFO_DUMMY_BIT 1
#define ENDAT_NUM_BYTES_ADDITIONAL_INFO 3

#define ENDAT_STATUS_WARN_SHIFT 7
#define ENDAT_STATUS_RM_SHIFT   6
#define ENDAT_STATUS_BUSY_SHIFT 5
#define ENDAT_STATUS_WARN_MASK  (1 << ENDAT_STATUS_WARN_SHIFT)
#define ENDAT_STATUS_RM_MASK    (1 << ENDAT_STATUS_RM_SHIFT)
#define ENDAT_STATUS_BUSY_MASK  (1 << ENDAT_STATUS_BUSY_SHIFT)
#define ENDAT_INFORMATION_MASK  0x1f

#define ENDAT_21_F1_SHIFT   31
#define ENDAT_21_F1_MASK    (1 << ENDAT_21_F1_SHIFT)
#define ENDAT_22_F1_SHIFT   ENDAT_21_F1_SHIFT
#define ENDAT_22_F1_MASK    (1 << ENDAT_22_F1_SHIFT)
#define ENDAT_F2_SHIFT      30
#define ENDAT_F2_MASK       (1 << ENDAT_F2_SHIFT)

#define ENDAT_ADDITIONAL_INFO_RX_BITS 30

#define ENDAT_SECTION2_MEMORY 0xBF

#define ENDAT_NUM_BITS_VALID_PAGE0_WORD13 6

#define ENDAT_NUM_BITS_VALID_PAGE1_WORD1 16

#define ENDAT_MRS_SHIFT_C7_C4 (4)
#define ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO1 (0x4)
#define ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO2 (0x5)
#define ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO (0xF << ENDAT_MRS_SHIFT_C7_C4)
#define ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO (0xF)
#define ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO (ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)

#define ENDAT_GET_POS_MULTI_TURN(pos, priv) (((pos) & (((unsigned long long) 1 << (priv)->pos_res) - 1)) >> (priv)->single_turn_res)
#define ENDAT_GET_POS_SINGLE_TURN(pos, priv) ((pos) & (((unsigned long long) 1 << (priv)->single_turn_res) - 1))

#include "endat_api.h"

#ifdef __cplusplus
}
#endif

#endif
