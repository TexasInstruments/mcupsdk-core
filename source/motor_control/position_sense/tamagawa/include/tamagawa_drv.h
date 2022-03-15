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

#ifndef TAMAGAWA_DRV_H_
#define TAMAGAWA_DRV_H_

/**
 *  \defgroup TAMAGAWA_API_MODULE APIs for Tamagawa Encoder
 *  \ingroup MOTOR_CONTROL_API
 *
 * Here is the list of APIs used for Tamagawa encoder communication protocol
 *
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <drivers/pruicss.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief  Used to set the value of Tamagawa multi-channel mask based on the whether the Channel 0 is selected or not
 */
#define TAMAGAWA_MULTI_CH0 (1 << 0)

/**
 *  \brief  Used to set the value of Tamagawa multi-channel mask based on the whether the Channel 1 is selected or not
 */
#define TAMAGAWA_MULTI_CH1 (1 << 1)

/**
 *  \brief  Used to set the value of Tamagawa multi-channel mask based on the whether the Channel 2 is selected or not
 */
#define TAMAGAWA_MULTI_CH2 (1 << 2)

/**
 *  \brief  Used to set the maximum channels supported
 */
#define MAX_CHANNELS (3)

/**
 *  \brief  Used to set the maximum address that can be used for EEPROM Read/Write
 */
#define MAX_EEPROM_ADDRESS (127)

/**
 *  \brief  Used to set the maximum value that can be written in EEPROM
 */
#define MAX_EEPROM_WRITE_DATA (255)


/**
*    \brief    Data ID codes
*/
enum data_id
{
    DATA_ID_0,  /**< Data readout data in one revolution */
    DATA_ID_1,  /**< Data readout multi-turn data */
    DATA_ID_2,  /**< Data readout encoder ID */
    DATA_ID_3,  /**< Data readout data in one revolution, encoder ID, multi-turn, encoder error */
    DATA_ID_6,  /**< EEPROM write */
    DATA_ID_7,  /**< Reset */
    DATA_ID_8,  /**< Reset */
    DATA_ID_C,  /**< Reset */
    DATA_ID_D,  /**< EEPROM read */
    DATA_ID_NUM /**< Number of Data ID codes */
};

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining tamagawa per channel interface
 *
 *    \details  Firmware per channel interface
 */
struct tamagawa_ch_info
{
    volatile uint32_t    pos_word0;          /**<word0 for receiving Rx data  */

    volatile uint32_t   pos_word1;          /**<word1 for receiving Rx data  */

    volatile uint32_t   pos_word2;          /**<word2 for receiving Rx data  */

    volatile uint32_t  cal_crc;             /**<word for receiving the CRC  */

};

/**
 *    \brief    Structure defining Tamagawa command interface
 *
 *    \details  Firmware command interface
 */
struct tamagawa_cmd
{
    volatile uint32_t   word0;/**< command,                                                         <br>
                                    [Byte 0] control field <br> **/
    volatile uint32_t   word1;/**< command parameters,                                              <br>
                                    [Byte 0] No. of Tx frames                                        <br>
                                    [Byte 1] No. of Rx frames                                        <br>

                                    */

};

/**
 *    \brief    Structure defining Tamagawa configuration interface
 *
 *    \details  Firmware configuration interface
 */
struct tamagawa_config
{
    volatile uint8_t  opmode;/**< operation mode selection: 0 - periodic trigger, 1 - host trigger */
    volatile uint8_t  channel;/**< channel mask (1 << channel), 0 < channel < 3. This has to be      <br>
                                        selected before running firmware. Once initialization is complete,<br>
                                        it will reflect the detected channels in the selected mask.       <br>
                                        Multichannel can have upto 3 selected, while single channel only one */
    volatile uint8_t  trigger;/**< command trigger. Set LSB to send cmd, will be cleared upon cmd    <br>
                                        completion. Note that cmd has to be setup before trigger */
    volatile uint8_t  status;/**< initialization status: 1 - upon successful.  */
};

/**
 * \brief Tamagawa Interface Received data
 **/
struct rx_frames_received
{
    uint32_t abs;   /**< Data in one revolution */
    uint32_t abm;   /**< Multi-turn Data */
    uint8_t  cf;    /**< Control Frame */
    uint8_t  sf;    /**< Status Frame */
    uint8_t  enid;  /**< Encoder ID */
    uint8_t  almc;  /**< Encoder error */
    uint8_t  adf;   /**< EEPROM address */
    uint8_t  edf;   /**< EEPROM data */
    uint8_t  crc;   /**< CRC */
};
/**
 * \brief Tamagawa Interface
 */
struct tamagawa_interface
{
    uint8_t ch_mask;   //**< Mask for what channel is required*/
    volatile uint32_t  rx_div_factor;   //**< Rx Divide factor*/

    volatile uint32_t  tx_div_factor;   //**< Tx Divide factor*/

    volatile uint32_t  oversample_rate; //**< Oversampling rate*/

    uint32_t version;  /**< Firmware version */
    uint8_t  data_id;  /**< Data ID code */
    struct rx_frames_received rx_frames_received;      /**< Received data */
    uint8_t tx_frames;  /**< Number of Tx frames */
    uint8_t rx_frames;  /**< Number of Rx frames */
};

struct config
{
    uint8_t  ch0;   /**< config for channel 0 */
    uint8_t  ch1;   /**< config for channel 1 */
    uint8_t  ch2;   /**< config for channel 2 */
};

/**
 * \brief Tamagawa EEPROM Interface
 */
struct tamagawa_eeprom_interface
{
    volatile uint32_t cmd; /**< holds the value of command id for EEPROM commands */
    volatile uint32_t adf; /**< holds the value of ADF for EEPROM commands */
    volatile uint32_t edf; /**< holds the value of EDF for EEPROM Write command */
    volatile uint32_t crc; /**< holds the value of CRC for EEPROM commands */

    volatile uint32_t word0; /**< used for CRC calculation */
    volatile uint32_t word1; /**< used for CRC calculation */
    volatile uint32_t word2; /**< used for CRC calculation */

    uint64_t eeprom_tx_data; /**< used to store the bits for tx in eeprom read/write */
};


/**
 *    \brief    Structure defining Tamagawa interface
 *
 *    \details  Firmware config, command and channel interface
 *
 */
struct tamagawa_xchg
{
    struct tamagawa_config   config;/**< config interface */
    struct tamagawa_cmd      cmd;/**< command interface */
    struct tamagawa_ch_info  ch[3];/**< per channel interface */

    struct tamagawa_interface tamagawa_interface;/**< tamagawa interface */
    struct tamagawa_eeprom_interface tamagawa_eeprom_interface[3];/**< tamagawa interface for EEPROM commands */
};

/**
 * \brief   Used to configure the Tamagawa Clock.
 *
 */
struct tamagawa_clk_cfg
{
    uint16_t  rx_div;   /**< Rx Div factor*/
    uint16_t  tx_div;   /**< Tx Div factor*/
    uint16_t  rx_en_cnt;
    uint16_t  rx_div_attr;
};


/**
 * \brief   Used to store the register offsets depending on different PRU slices.
 *
 */
struct register_offsets
{
    int32_t ICSS_CFG_PRUx_ED_CH0_CFG0;
    int32_t ICSS_CFG_PRUx_ED_CH1_CFG0;
    int32_t ICSS_CFG_PRUx_ED_CH2_CFG0;
    int32_t ICSS_CFG_PRUx_ED_CH0_CFG1;
    int32_t ICSS_CFG_PRUx_ED_CH1_CFG1;
    int32_t ICSS_CFG_PRUx_ED_CH2_CFG1;
    int32_t ICSS_CFG_GPCFGx;
    int32_t ICSS_CFG_PRUx_ED_RXCFG;
    int32_t ICSS_CFG_PRUx_ED_TXCFG;
};

/**
 * \brief   Used to structures defining the Tamagawa interface, PRU slice and register offsets.
 *
 */
struct tamagawa_priv
{
    int32_t channel;    /**< Holds the ID of the current channel being used*/
    uint16_t rx_en_cnt;
    struct tamagawa_xchg *tamagawa_xchg;    /**<Structure defining Tamagawa interface*/
    void *pruss_cfg;    /**< ICSS PRU config base address*/
    int32_t slice_value;    /**< PRUx Slice being used*/
    struct register_offsets register_offset_val;    /**< Register offset values based on PRUx slice selection*/
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief      send the tamagawa command and wait till firmware acknowledges
 *
 *  \param[in]  priv            cookie returned by tamagawa_init
 *  \param[in]  cmd             tamagawa command number
 *  \param[in]  gTamagawa_multi_ch_mask  Multi-channel mask to keep track of which channels are selected
 *
 *  \retval     0       success
 *  \retval     -EINVAL failure
 *
 */
int32_t tamagawa_command_process(struct tamagawa_priv *priv, int32_t cmd, uint8_t gTamagawa_multi_ch_mask);

/**
 *  \brief      setup the tamagawa command in the PRU interface buffer
 *
 *  \param[in]  priv            cookie returned by tamagawa_init
 *  \param[in]  cmd             tamagawa command number
 *  \param[in]  gTamagawa_multi_ch_mask Multi-channel mask to keep track of which channels are selected
 *
 *  \retval     0       success
 *  \retval     -EINVAL failure
 *
 */
int32_t tamagawa_command_build(struct tamagawa_priv *priv, int32_t cmd,  uint8_t gTamagawa_multi_ch_mask);

/**
 *  \brief      trigger sending the tamagawa command in PRU
 *
 *  \param[in]  priv     cookie returned by tamagawa_init
 *
 */
void tamagawa_command_send(struct tamagawa_priv *priv);

/**
 *  \brief  wait till PRU finishes tamagawa transaction
 *
 *  \param[in]  priv     cookie returned by tamagawa_init
 *
 */
void tamagawa_command_wait(struct tamagawa_priv *priv);


/**
 *  \brief  configure tamagawa clock
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 */
void tamagawa_config_clock(struct tamagawa_priv *priv, struct tamagawa_clk_cfg *clk_cfg);

/**
 *  \brief      configure tamagawa master for host trigger mode
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *
 */
void tamagawa_config_host_trigger(struct tamagawa_priv *priv);

/**
 *  \brief      configure tamagawa master in periodic trigger mode
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *
 */
void tamagawa_config_periodic_trigger(struct tamagawa_priv *priv);

/**
 *  \brief      select channel to be used by tamagawa master
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  ch      channel to be selected
 *
 */
void tamagawa_config_channel(struct tamagawa_priv *priv, uint32_t ch);

/**
 *  \brief      select mask of channels to be used in multi channel configuration by tamagawa master
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  mask    channel mask
 *
 */
void tamagawa_config_multi_channel_mask(struct tamagawa_priv *priv, uint8_t mask);

/**
 *  \brief      select channels detected in multi channel configuration by tamagawa master.    <br>
 *              required to be invoked only if firmware indicates initialization failure    <br>
 *              to know the channels that has been detected. Initialization success implies <br>
 *              that all channels indicated has been detected.
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *
 *  \retval     mask    mask of the detected channels
 *
 */
uint8_t tamagawa_multi_channel_detected(struct tamagawa_priv *priv);

/**
 *  \brief      In multi channel configuration, select channel before receive processing in <br>
 *              multi channel configuration. After receive is complete, select each channel <br>
 *              and invoke rx API's to parse data recieved in each channel.
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  ch      channel number to be selected
 *
 */
void tamagawa_multi_channel_set_cur(struct tamagawa_priv *priv, uint32_t ch);



/**
 *  \brief      Initialize tamagawa firmware interface address and get the pointer
 *              to struct tamagawa_priv instance
 *
 *  \param[in]  tamagawa_xchg      tamagawa firmware interface address
 *  \param[in]  pruss_cfg       ICSS PRU config base address
 *  \param[in]  slice_value     PRUx slice value : 0 for PRU0 and 1 for PRU1
 *
 *  \retval     priv            pointer to struct tamagawa_priv instance
 *
 */
struct tamagawa_priv *tamagawa_init(struct tamagawa_xchg *tamagawa_xchg, void *pruss_cfg, uint32_t slice_value);

/**
 *  \brief      update the current requested command id in tamagawa interface.    <br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  cmd     tamagawa command number
 *
 */

void tamagawa_update_data_id(struct tamagawa_priv *priv, int32_t cmd);

/**
 *  \brief      update the adf(address of EEPROM) field entered by user for EEPROM command in tamagawa interface.<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  val     ADF value to be updated
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_update_adf(struct tamagawa_priv *priv, uint32_t val, uint32_t ch);

/**
 *  \brief      update the edf(data for EEPROM) field entered by user for EEPROM command in tamagawa interface.<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  val     EDF value to be updated
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_update_edf(struct tamagawa_priv *priv, uint32_t val, uint32_t ch);


/**
 *  \brief      Parse the data in tamagawa interface.<br>
 *
 *
 *  \param[in]  cmd     tamagawa command number
 *  \param[in]  priv    cookie returned by tamagawa_init
 *
 */
int32_t tamagawa_parse(int32_t cmd, struct tamagawa_priv *priv);

/**
 *  \brief      verify the CRC computed with the encoder crc.<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *
 *  \retval     1/0     if verify correctly, return 1 else return 0.
 *
 */

int32_t tamagawa_crc_verify(struct tamagawa_priv *priv);

/**
 *  \brief      Pass the values of CF(Control Field), ADF(address of EEPROM) and EDF(data for EEPROM) to the CRC calculator fucntion and update the CRC field.<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  cmd     tamagawa command number
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_update_crc(struct tamagawa_priv *priv, int32_t cmd, uint32_t ch);

/**
 *  \brief      Update the values for oversample rate and division factor for Tx and Rx.<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  baudrate     baud rate of the tamagawa encoder
 *
 */

void tamagawa_set_baudrate(struct tamagawa_priv *priv, double baudrate);

/**
 *  \brief      Reset the values of the variables used in CRC calculation to 0.<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *
 */

void tamagawa_eeprom_crc_reinit(struct tamagawa_priv *priv);

/**
 *  \brief      Reverse the bits of a number.<br>
 *
 *
 *  \param[in]  data    8 bit value for any of CF(Control Field), ADF(address of EEPROM) or EDF(data for EEPROM)
 *
 *  \retval     reversed_num    number obtained after reversing the bits of data provided
 *
 */

uint32_t tamagawa_reverse_bits(int8_t data);

/**
 *  \brief      Add the start and the stop bit to the reversed data.<br>
 *
 *
 *  \param[in]  eeprom_tx_data    holds the value of the Tx data to be sent
 *  \param[in]  data    holds the value of CF(Control Field), ADF(address of EEPROM) or EDF(data for EEPROM)
 *
 *  \retval     eeprom_tx_data    64 bit integer that holds the value of the Tx data to be sent
 *
 */

uint64_t tamagawa_prepare_eeprom_tx_data(uint64_t eeprom_tx_data, volatile uint32_t data);

/**
 *  \brief      Prepare the required EEPROM command from the CF(Control Field), ADF(address of EEPROM) and EDF(data for EEPROM).<br>
 *
 *
 *  \param[in]  priv    cookie returned by tamagawa_init
 *  \param[in]  cmd     tamagawa command number
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_prepare_eeprom_command(struct tamagawa_priv *priv, int32_t cmd, uint32_t ch);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
