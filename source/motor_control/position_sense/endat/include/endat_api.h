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

/*
 * \par
 *
 *
 *    Example API usage (host trigger mode, single channel):
 *    ------------------------------------------------------
 *
 *    A. Obtain handle
 *
 *        struct endat_priv priv = endat_init(<interface base>);
 *                [AM437x: endat_init(0x54440000);]
 *
 *    B. Configure to host trigger mode
 *
 *        endat_config_host_trigger(priv);
 *
 *    C. Select channel
 *
 *        endat_config_channel(priv, <#channel>);
 *
 *    D. Load & run the firmware
 *
 *    E. Wait for firmware to initialize, clock frequency would be set to 8MHz by the end of the following command
 *
 *        endat_wait_initialization(priv, <timeout>);
 *
 *    F. Set 200KHz frequency to read encoder information so as to not to be affected by propagation delay
 *
 *        endat_config_clock(priv, clk_cfg);
 *
 *    G. Get encoder information
 *
 *        endat_get_encoder_info(priv);
 *
 *    H. Set operating frequency, say 8MHz
 *
 *        endat_config_clock(priv, &clk_cfg);
 *
 *    I. Get the propagation delay estimated by the firmware
 *
 *        prop_delay = endat_get_prop_delay(priv);
 *
 *    J. Handle propagation delay
 *
 *        endat_config_rx_arm_cnt(priv, <prop_delay (minimum 2T, T - period of operating frequency)>);
 *        endat_config_rx_clock_disable(priv, <prop_delay - 2T (if < 0, use 0)>);
 *
 *    K. Configure tST
 *
 *        endat_config_tst_delay(priv, <tST delay (if f > 1MHz, use 2000, else 0)>);
 *
 *    L. Process the EnDat command, transmit, this returns upon receive & command is complete,
 *
 *        endat_command_process(priv, cmd, cmd_supplement);
 *
 *    M. Process the received data
 *
 *        endat_recvd_process(priv, cmd, &endat_format_data);
 *
 *    N. Do CRC verification
 *
 *        endat_recvd_validate(priv, cmd, &endat_format_data);
 *
 *    O. Update additional information state tracking, if needed
 *
 *        endat_addinfo_track(priv, cmd, &cmd_supplement);
 *
 *    Result would be in union endat_format_data
 *
 *
 *    Example API usage [simplified] (periodic trigger mode, single channel):
 *    -----------------------------------------------------------------------
 *
 *    Note: IEP CMP2 is the trigger event in periodic trigger mode and has to be setup beforehand.
 *
 *    A. Get encoder info, follow steps A, B, C, D, E, G of host trigger mode, firmware defaults to 8MHz frequency
 *
 *    B. Switch to periodic trigger mode, this will take effect after the next command completes
 *
 *        endat_config_periodic_trigger(priv);
 *
 *    C. Setup & send the 2.2 position periodic command, needs to be done only once
 *
 *        endat_command_process(priv, 8, NULL);
 *
 *    D. After giving enough time to complete the command after periodic event, to read angle for rotary encoder quickly
 *
 *        endat_get_2_2_angle(priv);
 *
 *
 *    Example API usage (periodic trigger mode, single channel):
 *    ---------------------------------------------------------
 *
 *    Note: IEP CMP2 is the trigger event in periodic trigger mode and has to be setup beforehand.
 *
 *    A. Get encoder info & set desired timings for required frequency taking care of propagation delay, steps A-K of host trigger mode
 *
 *    B. Switch to periodic trigger mode, this will take effect after the next command completes
 *
 *        endat_config_periodic_trigger(priv);
 *
 *    C. Setup the command to be periodically sent
 *
 *        endat_command_build(priv, cmd, cmd_supplement);
 *
 *    D. Send the command
 *
 *        endat_command_send(priv);
 *
 *    E. Wait for the command to finish
 *
 *        endat_command_wait(priv);
 *
 *    F. Process the received data
 *
 *        endat_recvd_process(priv, cmd, &endat_format_data);
 *
 *    G. Do CRC verification
 *
 *        endat_recvd_validate(priv, cmd, &endat_format_data);
 *
 *    H. Update additional information state tracking, if needed
 *
 *        endat_addinfo_track(priv, cmd, &cmd_supplement);
 *
 *    Result would be in union endat_format_data
 *
 *    Repeat E-H to get periodic update after the next periodic event            <br>
 *    happens, this can be done in say ISR (which could be linked to the         <br>
 *    periodic event). For a new command to be executed periodically,            <br>
 *    endat_command_build() can invoked after step H, this will take effect      <br>
 *    from the next periodic event.
 *
 *    EnDat commands:
 *    ---------------
 *
 *    Command description of command numbers passed to endat_command_process()
 *     and endat_command_build() as follows,
 *
 *     1:  Encoder send position values                                          <br>
 *     2:  Selection of memory area                                              <br>
 *     3:  Encoder receive parameter                                             <br>
 *     4:  Encoder send parameter                                                <br>
 *     5:  Encoder receive reset                                                 <br>
 *     6:  Encoder send test values                                              <br>
 *     7:  Encoder receive test command                                          <br>
 *     8:  Encoder to send position + AI(s)                                      <br>
 *     9:  Encoder to send position + AI(s) and receive selection of memory area <br>
 *     10: Encoder to send position + AI(s) and receive parameter                <br>
 *     11: Encoder to send position + AI(s) and send parameter                   <br>
 *     12: Encoder to send position + AI(s) and receive error reset              <br>
 *     13: Encoder to send position + AI(s) and receive test command             <br>
 *     14: Encoder receive communication command                                 <br>
 *
 */

#ifndef ENDAT_API_H_
#define ENDAT_API_H_

#ifdef __cplusplus
extern "C" {
#endif

 /**
 * \defgroup MOTOR_CONTROL_API APIs for Motor Control Encoders
 *
 * This module contains APIs for device drivers for various motor control encoders supported in this SDK.
 */

/**
 *  \defgroup ENDAT_API_MODULE APIs for ENDAT Encoder
 *  \ingroup MOTOR_CONTROL_API
 *
 * Here is the list of APIs used for EnDAT encoder communication protocol
 *
 *  @{
 */

/**
 *  \brief      process raw recieved data and format based on the command
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  cmd     EnDat command number
 *  \param[out] u       pointer to union for storing formatted data based on cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int endat_recvd_process(struct endat_priv *priv, int cmd,
                        union endat_format_data *u);

/**
 *  \brief      CRC result indicated in return value as follows,             <br>
 *              0th bit high    position/address/params/test CRC success     <br>
 *              0th bit low     position/address/params/test CRC failure     <br>
 *              1st bit high    if addinfo 1 present, addinfo 1 CRC success  <br>
 *              1st bit low     if addinfo 1 present, addinfo 1 CRC failure  <br>
 *              2nd bit high    if addinfo 2 present, addinfo 2 CRC success  <br>
 *              2nd bit low     if addinfo 2 present, addinfo 2 CRC failure
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  cmd     EnDat command number
 *  \param[in]  u       pointer to union having formatted data based on cmd
 *
 *  \retval     status  position/address/params/test CRC status
 *
 */
unsigned endat_recvd_validate(struct endat_priv *priv, int cmd,
                              union endat_format_data *u);

/**
 *  \brief      send the EnDat command and wait till firmware acknowledges
 *
 *  \param[in]  priv            cookie returned by endat_init
 *  \param[in]  cmd             EnDat command number
 *  \param[in]  cmd_supplement  Supplement information needed to setup EnDat cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int endat_command_process(struct endat_priv *priv, int cmd,
                          struct cmd_supplement *cmd_supplement);

/**
 *  \brief      setup the EnDat command in the PRU interface buffer
 *
 *  \param[in]  priv            cookie returned by endat_init
 *  \param[in]  cmd             EnDat command number
 *  \param[in]  cmd_supplement  Supplement information needed to setup EnDat cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int endat_command_build(struct endat_priv *priv, int cmd,
                        struct cmd_supplement *cmd_supplement);

/**
 *  \brief      trigger sending the EnDat command in PRU
 *
 *  \param[in]  priv     cookie returned by endat_init
 *
 *
 */
void endat_command_send(struct endat_priv *priv);

/**
 *  \brief  wait till PRU finishes EnDat transaction
 *
 *  \param[in]  priv     cookie returned by endat_init
 *
 *
 */
void endat_command_wait(struct endat_priv *priv);

/**
 *  \brief       update priv with position resolution, id, serial number, encoder
 *               type and supported command set
 *
 *  \param[in]   priv    cookie returned by endat_init
 *
 *  \retval      0 for success, -EINVAL for failure
 *
 */
int endat_get_encoder_info(struct endat_priv *priv);

/**
 *  \brief  get propagation delay automatically estimated by the firmware
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *  \retval     delay   estimated propogation delay
 *
 */
unsigned int endat_get_prop_delay(struct endat_priv *priv);

/**
 *  \brief  track presence of additional information in priv
 *
 *  \param[in]  priv           cookie returned by endat_init
 *  \param[in]  cmd            EnDat command number
 *  \param[in]  cmd_supplement Supplement information needed to setup EnDat cmd
 *
 *
 */
void endat_addinfo_track(struct endat_priv *priv, int cmd,
                         struct cmd_supplement *cmd_supplement);

/**
 *  \brief  configure EnDat clock
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 *
 */
void endat_config_clock(struct endat_priv *priv,
                        struct endat_clk_cfg *clk_cfg);

/**
 *  \brief  configure tST delay
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  delay   tST delay value
 *
 *
 */
void endat_config_tst_delay(struct endat_priv *priv, unsigned short delay);

/**
 *  \brief  configure rx arm counter
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  val     rx arm counter value in ns
 *
 *
 */
void endat_config_rx_arm_cnt(struct endat_priv *priv, unsigned short val);

/**
 *  \brief  configure wire delay for the selected channel
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  val     wire delay in ns
 *
 *
 */
void endat_config_wire_delay(struct endat_priv *priv, unsigned short val);

/**
 *  \brief  configure clocks to be disabled at the end of rx to account for tD
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  val     number of clocks to be disabled
 *
 *
 */
void endat_config_rx_clock_disable(struct endat_priv *priv,
                                   unsigned short val);

/**
 *  \brief       start continuous mode
 *
 *  \param[in]   priv   cookie returned by endat_init
 *
 *  \retval      0       success, -EINVAL for failure
 *
 */
int endat_start_continuous_mode(struct endat_priv *priv);

/**
 *  \brief       stop continuous mode
 *
 *  \param[in]   priv   cookie returned by endat_init
 *
 *
 */
void endat_stop_continuous_mode(struct endat_priv *priv);

/**
 *  \brief      configure EnDat master for host trigger mode
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *
 */
void endat_config_host_trigger(struct endat_priv *priv);

/**
 *  \brief      configure EnDat master in periodic trigger mode
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *
 */
void endat_config_periodic_trigger(struct endat_priv *priv);

/**
 *  \brief      select channel to be used by EnDat master
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  ch      channel to be selected
 *
 *
 */
void endat_config_channel(struct endat_priv *priv, int ch);

/**
 *  \brief      select mask of channels to be used in multi channel configuration by EnDat master
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  mask    channel mask
 *
 *
 */
void endat_config_multi_channel_mask(struct endat_priv *priv,
                                     unsigned char mask);

/**
 *  \brief      select channels detected in multi channel configuration by EnDat master.    <br>
 *              required to be invoked only if firmware indicates initialization failure    <br>
 *              to know the channels that has been detected. Initialization success implies <br>
 *              that all channels indicated has been detected.
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *  \retval     mask    mask of the detected channels
 *
 */
unsigned char endat_multi_channel_detected(struct endat_priv *priv);

/**
 *  \brief      In multi channel configuration, select channel before receive processing in <br>
 *              multi channel configuration. After receive is complete, select each channel <br>
 *              and invoke rx API's to parse data recieved in each channel.
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  ch      channel number to be selected
 *
 *
 */
void endat_multi_channel_set_cur(struct endat_priv *priv, int ch);

/**
 *  \brief      wait for EnDat master firmware to initialize
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  timeout timeout to wait for initialization
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int endat_wait_initialization(struct endat_priv *priv, unsigned timeout);

/**
 *  \brief      Initialize EnDat firmware interface address and get the pointer
 *              to struct endat_priv instance
 *
 *  \param[in]  pruss_xchg      EnDat firmware interface address
 *  \param[in]  pruss_cfg       ICSS PRU config base address
 *
 *  \retval     priv            pointer to struct endat_priv instance
 *
 */
struct endat_priv *endat_init(struct endat_pruss_xchg *pruss_xchg,
                              void *pruss_cfg);

/**
 *  \brief      Read EnDat 2.2 angular position in steps for rotary encoders      <br>
 *              (prior to invoking this, 2.2 position command has to be completed)<br>
 *              This checks the CRC status (updated on-the-fly by firmware),      <br>
 *              checks the group alarm errors (F1/F2 bits) and the angle value    <br>
 *              recieved in reverse are reversed in a highly optimized way using  <br>
 *              inline assembly.                                                  <br>
 *              Assumptions (that holds good for quite a few encoders):           <br>
 *              1. Total resolution >= 25                                         <br>
 *              2. Single turn resolution <= 30
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *  \retval     angle for angular position in steps on success, -1 for failure
 *
 */
int endat_get_2_2_angle(struct endat_priv *priv);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
