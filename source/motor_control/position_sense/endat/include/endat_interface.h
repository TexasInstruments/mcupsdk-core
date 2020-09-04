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
 
#ifndef ENDAT_INTERFACE_H_
#define ENDAT_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

/**    \brief    2.1 send position value */
#define ENDAT_CMD_SEND_POSITION_VALUES  (0x1C >> 1)
/**    \brief    2.1 select memory area */
#define ENDAT_CMD_SEL_MEM_AREA          (0x38 >> 1)
/**    \brief    2.1 receive paramter */
#define ENDAT_CMD_RECEIVE_PARAMETERS    (0x70 >> 1)
/**    \brief    2.1 send paramter */
#define ENDAT_CMD_SEND_PARAMETERS       (0x8C >> 1)
/**    \brief    2.1 receive reset */
#define ENDAT_CMD_RECEIVE_RESET         (0xA8 >> 1)
/**    \brief    2.1 send test values */
#define ENDAT_CMD_SEND_TEST_VALUES      (0x54 >> 1)
/**    \brief    2.1 receive test command */
#define ENDAT_CMD_RECEIVE_TEST_COMMAND  (0xC4 >> 1)

/**    \brief    2.2 send position value with addinfo(s) */
#define ENDAT_CMD_SEND_POSVAL_WITH_DATA         (0xE0 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & select memory area */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL    (0x24 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & receive parameter */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM     (0x6C >> 1)
/**    \brief    2.2 send position value with addinfo(s) & send parameter */
#define ENDAT_CMD_SEND_POSVAL_SEND_PARAM        (0x90 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & receive test command */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD   (0xD8 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & receive error reset */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST   (0xB4 >> 1)
/**    \brief    2.2 receive communication command */
#define ENDAT_CMD_RECEIVE_COMMUNICATION_CMD     (0x48 >> 1)

/**    \brief    command has no command supplement */
#define ENDAT_CMDTYP_NO_SUPPLEMENT  0x1
/**    \brief    position command */
#define ENDAT_CMDTYP_POSITION       (0x1 << 1)
/**    \brief    command belongs to EnDat 2.2 command set */
#define ENDAT_CMDTYP_ENDAT22        (0x1 << 2)
/**    \brief    2.2 position command with additional info 1 */
#define ENDAT_CMDTYP_HAS_ADDINFO1   (0x1 << 3)
/**    \brief    2.2 position command with additional info 2 */
#define ENDAT_CMDTYP_HAS_ADDINFO2   (0x1 << 4)

/**    \brief    position/data CRC status mask */
#define ENDAT_CRC_DATA      (0x1 << 0)
/**    \brief    additional info 1/2 CRC status mask (if either one present) <br>
                 if both present, indicates additional info 2 CRC status mask */
#define ENDAT_CRC_ADDINFOX  (0x1 << 1)
/**    \brief    additional info 1 CRC status mask (if both present) */
#define ENDAT_CRC_ADDINFO1  (0x1 << 2)

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 *    \brief    Structure defining per channel CRC information
 *
 *    \details  Firmware per channel CRC information interface
 */
struct crc
{
    volatile unsigned char  status;
    /**< CRC status,
         bit0: 1 - position/data success, 0 - position/data failure       <br>
         bit1: 1 - additional info1 success, 0 - additioanl info1 failure <br>
         bit2: 1 - additional info2 success, 0 - additioanl info2 failure */
    volatile unsigned char  err_cnt_data;
    /**< CRC position/data error count (will wraparound after 255) */
    volatile unsigned char  err_cnt_addinfox;
    /**< CRC additional info1/2 error count (will wraparound after 255) */
    volatile unsigned char  err_cnt_addinfo1;
    /**< CRC additional info1 error count (will wraparound after 255)     <br>
         applicable only when both additional info's are present */
};

/**
 *    \brief    Structure defining EnDat per channel interface
 *
 *    \details  Firmware per channel interface
 */
struct endat_pruss_ch_info
{
    volatile unsigned int   pos_word0;
    /**< Initial (<=32) position bits received including error bits */
    volatile unsigned int   pos_word1;
    /**< position bits received after the initial 32 bits (if applicable) */
    volatile unsigned int   pos_word2;
    /**< additional info 1/2 (will be additional info 2 if both present) */
    volatile unsigned int   pos_word3;
    /**< additional info 1 (if both additional 1 & 2 present) */
    struct crc      crc;
    /**< crc information */
    volatile unsigned char  num_clk_pulse;
    /**< position bits excluding SB, error, CRC (updated upon initialization) */
    volatile unsigned char  endat22_stat;
    /**< encoder command set type, 1 - 2.2 supported, 0 - 2.2 not supported  */
    volatile unsigned short rx_clk_less;
    /**< receive clocks to be reduced to handle propagation delay (to be  <br>
         updated by host, if applicable) */
    volatile unsigned int   prop_delay;
    /**< automatically estimated propagation delay */
    volatile unsigned int   resvd_int0;
    /**< reserved */
};

/**
 *    \brief    Structure defining EnDat command interface
 *
 *    \details  Firmware command interface
 */
struct endat_pruss_cmd
{
    volatile unsigned int   word0;
    /**< command,                                                         <br>
         [Byte 0] bit 7: 0(dummy), bit 6-1: command, bit 0: address bit 7 <br>
         [Byte 1] bit 7-1: address bit 6-0, bit 0: parameter bit 15       <br>
         [Byte 2] bit 7-0: parameter bit 14-7                             <br>
         [Byte 2] bit 7-1: parameter bit 6-0, bit 0: 0(dummy) */
    volatile unsigned int   word1;
    /**< command parameters,                                              <br>
         [Byte 0] receive bits, includes SB & dummy (for additional info) <br>
         [Byte 1] transmit bit                                            <br>
         [Byte 2] attributes,                                             <br>
          bit0: 1 - no command supplement, 0 - command supplement present <br>
          bit1: 1 - position command, 0 - not position command            <br>
          bit2: 1 - EnDat 2.2 command, 0 - EnDat 2.1 command              <br>
          bit3: 1 - additional info1 present, 0 - no additional info1     <br>
          bit4: 1 - additional info2 present, 0 - no additional info2     <br>
         [Byte 3] 1 - block address selected, 0 - block address not selected */
    volatile unsigned int   word2;
    /**< command supplement,                                              <br>
         [Byte 0] address                                                 <br>
         [Byte 1] parameter MSByte                                        <br>
         [Byte 2] parameter LSByte                                        <br>
         [Byte 3] block address */
};

/**
 *    \brief    Structure defining EnDat configuration interface
 *
 *    \details  Firmware configuration interface
 */
struct endat_pruss_config
{
    volatile unsigned char  opmode;
    /**< operation mode selection: 0 - periodic trigger, 1 - host trigger */
    volatile unsigned char  channel;
    /**< channel mask (1 << channel), 0 < channel < 3. This has to be      <br>
         selected before running firmware. Once initialization is complete,<br>
         it will reflect the detected channels in the selected mask.       <br>
         Multichannel can have upto 3 selected, while single channel only one */
    volatile unsigned char  trigger;
    /**< command trigger. Set LSB to send cmd, will be cleared upon cmd    <br>
         completion. Set/clear MSB to start/stop continuous clock mode.    <br>
         To start continuous mode LSB also has to be set. Note that cmd    <br>
         has to be setup before trigger */
    volatile unsigned char  status;
    /**< initialization status: 1 - upon successful. Wait around 5 seconds <br>
         after firmware has started running to confirm status */
};

/**
 *    \brief    Structure defining EnDat interface
 *
 *    \details  Firmware config, command and channel interface
 *
 */
struct endat_pruss_xchg
{
    struct endat_pruss_config   config;
    /**< config interface */
    struct endat_pruss_cmd      cmd;
    /**< command interface */
    struct endat_pruss_ch_info  ch[3];
    /**< per channel interface */
    uint16_t endat_rx_clk_config;
    uint16_t endat_tx_clk_config;
    uint32_t endat_rx_clk_cnten;
    uint32_t endat_delay_2ms;
    uint32_t endat_delay_12ms;
    uint32_t endat_delay_50ms;
};

#ifdef __cplusplus
}
#endif

#endif