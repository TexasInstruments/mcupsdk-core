/*!
* \file osal_error.h
*
* \brief
* OSAL: Error handling interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined PROTECT_OSAL_ERROR_H)
#define PROTECT_OSAL_ERROR_H    1

#include <osal.h>

#define OSAL_NO_ERROR                           0x00000000u     //!< Default Value

#define OSAL_CAN_NO_MSG                         0x00000001u     ///!< no message available
#define OSAL_CAN_TX_FULL                        0x00000002u     ///!< TX queue is full, try later
#define OSAL_CAN_E                              0x00000003u     ///!< now everything is a error
#define OSAL_CAN_E_SPEED                        0x00000004u     ///!< Index for speed table is out of range
#define OSAL_CAN_E_FILTER                       0x00000005u     ///!< Index for acceptance filter is out of range
#define OSAL_CAN_E_NULL_POINTER                 0x00000007u     ///!< The callback pointer is zero (NULL)
#define OSAL_CAN_INVALID_MODE                   0x00000008u     //!< Mode Parameter was invalid
#define OSAL_CAN_INTERN                         0x00000009u     //!< Internal Error in Can Initialization
#define OSAL_CAN_INTERN2                        0x0000000au     //!< Internal Error in Can Initialization
#define OSAL_CAN_INV_PORT1                      0x0000000bu     //!< Invalid Port number
#define OSAL_CAN_INV_PORT2                      0x0000000cu     //!< Invalid Port number
#define OSAL_CAN_INV_PORT3                      0x0000000du     //!< Invalid Port number
#define OSAL_CAN_INV_PORT4                      0x0000000eu     //!< Invalid Port number
#define OSAL_CAN_INV_PORT5                      0x0000000fu     //!< Invalid Port number
#define OSAL_CAN_INV_PORT6                      0x00000010u     //!< Invalid Port number
#define OSAL_CAN_INV_PORT7                      0x00000011u     //!< Invalid Port number
#define OSAL_CAN_PRESCALER                      0x00000012u     //!< Invalid Prescaler
#define OSAL_CAN_INV_PORT8                      0x00000013u     //!< Invalid Port number
#define OSAL_CAN_INV_PORT9                      0x00000014u     //!< Invalid Port number
#define OSAL_CAN_INTERN3                        0x00000015u     //!< Internal Error in Can Initialization
#define OSAL_CAN_PCLK1                          0x00000016u     //!< PCLK1 has not the expected frequency

#define OSAL_CAN_TX_PRIO                        0x00010000u     //!< Invalid priority parameter
#define OSAL_CAN_TX_BUF_OVERFL_LOW              0x00010001u     //!< low priority transmit buffer overflow
#define OSAL_CAN_TX_BUF_OVERFL_HIGH             0x00010002u     //!< high priority transmit buffer overflow
#define OSAL_CAN_MONITORED_INTERN               0x00010003u     //!< Internal Error in state machine for sending monitored telegrams
#define OSAL_CAN_MONITORED_ERR                  0x00010004u     //!< CAN Send Error for the telegram occurred

#define OSAL_EE_VAR_NOT_EXIST                   0x00020000u     //!< Requested Variable from EEPROM does not exist
#define OSAL_EE_WRITE_ERROR                     0x00020001u     //!< Write Error during write to EEPROM
#define OSAL_EE_WRITE_ADR_NOT_EXIST             0x00020002u     //!< requested address for write is out of range
#define OSAL_EE_READ_ADR_NOT_EXIST              0x00020003u     //!< requested address for read is out of range
#define OSAL_EE_INIT_ERROR_01                   0x00020004u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_PAGE_FULL                       0x00020005u     //!< Internal use: EEPROM page is full
#define OSAL_EE_NO_VALID_PAGE                   0x00020006u     //!< Internal Error: no valid Page found
#define OSAL_EE_INIT_ERROR_02                   0x00020007u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_03                   0x00020008u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_04                   0x00020009u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_05                   0x0002000au     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_06                   0x0002000bu     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_07                   0x0002000cu     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_08                   0x0002000du     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_09                   0x0002000eu     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_10                   0x0002000fu     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_11                   0x00020010u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_12                   0x00020011u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_13                   0x00020012u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_14                   0x00020013u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_15                   0x00020014u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_INIT_ERROR_16                   0x00020015u     //!< Error during EEPROM initialization occurred
#define OSAL_EE_RD_BYTE_ADR_OUT_OF_RANGE        0x00020016u     //!< requested address is out of range
#define OSAL_EE_RD_WORD_ADR_OUT_OF_RANGE        0x00020017u     //!< requested address is out of range
#define OSAL_EE_RD_DWORD_ADR_OUT_OF_RANGE       0x00020018u     //!< requested address is out of range
#define OSAL_EE_WR_BYTE_ADR_OUT_OF_RANGE        0x00020019u     //!< requested address is out of range
#define OSAL_EE_WR_WORD_ADR_OUT_OF_RANGE        0x0002001au     //!< requested address is out of range
#define OSAL_EE_WR_DWORD_ADR_OUT_OF_RANGE       0x0002001bu     //!< requested address is out of range
#define OSAL_EE_DEF_DATA_ADR_OUT_OF_RANGE       0x0002001cu     //!< requested address is out of range
#define OSAL_EE_ADDRESES_OUT_OF_SPACE           0x0002001du     //!< Too much EEPROM addresses defined
#define OSAL_EE_FORMAT_FACTORY_RESET            0x0002001eu     //!< Format of EEPROM failed in BSP_EEPROM_factoryReset ()
#define OSAL_EE_WRITE_READY_TIME_OUT            0x0002001fu     //!< Ready signal after a write does not appear
#define OSAL_EE_ERASE_ALL_READY_TIME_OUT        0x00020020u     //!< Ready signal after an erase all does not appear

#define OSAL_FLASH_LOCK_ERR                     0x00030000u     //!< Unlock of Flash Erase Controller failed
#define OSAL_FLASH_ERASE_ERR                    0x00030001u     //!< Flash Erase Error
#define OSAL_FLASH_DEST_ALIGN                   0x00030002u     //!< Destination is not word aligned
#define OSAL_FLASH_PROG_ERR                     0x00030003u     //!< data program error
#define OSAL_FLASH_ERASE_ADDR_ERR               0x00030004u     //!< Flash Erase Address Error
#define OSAL_FLASH_ERASE_ADDR_ERR_2             0x00030005u     //!< Flash Erase Address Error

#define OSAL_UART_PORT_ERR                      0x00040000u     //!< Port Number is not supported
#define OSAL_UART_BAUD_RATE                     0x00040001u     //!< Baud Rate is not supported
#define OSAL_UART_STOP_BIT                      0x00040002u     //!< Number of Stop bits are not supported
#define OSAL_UART_FLOW_CONTROL                  0x00040003u     //!< Kind of Flow control is not supported
#define OSAL_UART_PARITY                        0x00040004u     //!< Parity Type is not supported
#define OSAL_UART_DATA_LEN                      0x00040005u     //!< Data Length is not supported
#define OSAL_UART_TX_NOT_EMPTY                  0x00040006u     //!< Transmitter Register is not empty during a send attempt
#define OSAL_UART_BUS_JOB_PENDING               0x00040007u     //!< Send Buffer: There is actually a job pending
#define OSAL_UART_RS485                         0x00040008u     //!< RS485: Option not supported

#define OSAL_TIMER_NUM_ERR                      0x00050000u     //!< Timer Number does not exist
#define OSAL_TIMER_PRESCALE_RANGE               0x00050001u     //!< Prescaler factor for timer initialization is out of range
#define OSAL_TIMER_USED_BY_SCHUDULER            0x00050002u     //!< the timer is already used by the scheduler
#define OSAL_TIMER_EXPIRED_BEFORE_STARTED       0x00050003u     //!< the timer is expired before started
#define OSAL_TIMER_NUM_ERR2                     0x00050004u     //!< Timer Number does not exist
#define OSAL_TIMER_NUM_ERR3                     0x00050005u     //!< Timer Number does not exist
#define OSAL_TIMER_NUM_ERR4                     0x00050006u     //!< Timer Number does not exist
#define OSAL_TIMER_NUM_ERR5                     0x00050007u     //!< Timer Number does not exist
#define OSAL_TIMER_ALL_IN_USE                   0x00050008u     //!< Available timers are all in use
#define OSAL_TIMER_MOD_NOT_INITIALIZED          0x00050009u     //!< Timer module was not initialized properly
#define OSAL_TIMER_NO_US_SUPPORT                0x0005000au     //!< compile module with define OSAL_OS_USE_TIMER_US

#define OSAL_LED_INVALID_STATE                  0x00060000u     //!< Set Led is called with an invalid state parameter
#define OSAL_LED_SET_OUT_OF_RANGE               0x00060001u     //!< Index of Set LED is out of Range
#define OSAL_LED_GET_OUT_OF_RANGE               0x00060002u     //!< Index of Get LED is out of Range

#define OSAL_SPI_PORT_ERR                       0x00070000u     //!< Port Number is not supported
#define OSAL_SPI_BUS_JOB_PENDING                0x00070001u     //!< Send Buffer: There is actually a job pending
#define OSAL_SPI_DMA_PERR			0x00070002u     ///< DMA Pointer invalid or missing
#define OSAL_SPI_INV_PRT			0x00070003u     ///< Invalid SPI Port
#define OSAL_SPI_PRT_RANGE			0x00070004u     ///< Invalid SPI Port
#define OSAL_SPI_INVALID_PHASE			0x00070005u     ///< Invalid SPI Configuration
#define OSAL_SPI_INVALID_POLARITY    		0x00070006u     ///< Invalid SPI Configuration
#define OSAL_SPI_INVALID_MISO_MOSI_DEF		0x00070007u     ///< Invalid SPI Configuration (in project.h)
#define OSAL_SPI_INVALID_SPIEN		        0x00070008u     ///< Invalid SPI Configuration (in project.h)

#define OSAL_CC_INIT_ERR                        0x00080000u     //!< Error in time of HW interface initialization
#define OSAL_CC_INIT_HSI_ERR                    0x00080001u     //!< internal HSI RC (8 MHz) error
#define OSAL_CC_INIT_HSE_ERR                    0x00080002u     //!< ext. high frequency OSC error
#define OSAL_CC_INIT_PLL_ERR                    0x00080003u     //!< PLL init error

#define OSAL_EXTI_LINE1                         0x00090000u     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_ALREADY_INIT                  0x00090001u     //!< Line is already initialized
#define OSAL_EXTI_PRIO_CONFLICT                 0x00090002u     //!< Interrupt Prioritization conflict
#define OSAL_EXTI_PORT                          0x00090003u     //!< Invalid Port
#define OSAL_EXTI_NO_EDGE                       0x00090004u     //!< Invalid Edge Setting
#define OSAL_EXTI_INVALID_PIN                   0x00090005u     //!< Invalid Edge Setting
#define OSAL_EXTI_INTERN1                       0x00090006u     //!< Internal error
#define OSAL_EXTI_INTERN2                       0x00090007u     //!< Internal error
#define OSAL_EXTI_LINE2                         0x00090008u     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_LINE3                         0x00090009u     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_LINE4                         0x0009000au     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_TEST                          0x0009000bu     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_LINE5                         0x0009000cu     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_LINE6                         0x0009000du     //!< EXTI; Invalid Line Parameter
#define OSAL_EXTI_LINE7                         0x0009000eu     //!< EXTI; Invalid Pin Argument
#define OSAL_EXTI_NOT_IMPLEMENTED1              0x0009000fu     //!< Function not implemented
#define OSAL_EXTI_NOT_IMPLEMENTED2              0x00090010u     //!< Function not implemented
#define OSAL_EXTI_NOT_IMPLEMENTED3              0x00090011u     //!< Function not implemented
#define OSAL_EXTI_INVALID_HANDLE1               0x00090012u     //!< Osal Handle is invalid
#define OSAL_EXTI_INVALID_HANDLE2               0x00090013u     //!< Osal Handle is invalid


#define OSAL_MFP3N_ADDRESS_PINS                 0x000A0000u     //!< MFP3N pins initialization, address pins not initialized ( pins are mixed or not on one GPIO port )
#define OSAL_MFP3N_DATA_PINS                    0x000A0001u     //!< MFP3N pins initialization, data pins not initialized ( pins are mixed or not on one GPIO port )
#define OSAL_MFP3N_SWx_PINS                     0x000A0002u     //!< MFP3N pins initialization, SWx pins not initialized ( pins are mixed or not on one GPIO port )
#define OSAL_MFP3N_SWx0_PINS                    0x000A0003u     //!< MFP3N pins initialization, SWx0 pins not initialized ( pins are mixed or not on one GPIO port )
#define OSAL_MFP3N_BSx_PINS                     0x000A0004u     //!< MFP3N pins initialization, BSx pins not initialized ( pins are mixed or not on one GPIO port )
#define OSAL_MFP3N_SENYUx_PINS                  0x000A0005u     //!< MFP3N pins initialization, SENYUx pins not initialized ( pins are mixed or not on one GPIO port )
#define OSAL_MFP3N_LED_PINS                     0x000A0006u     //!< MFP3N pins initialization, LED pins not initialized ( pins are not on one GPIO port )
#define OSAL_MFP3N_RESET_PIN                    0x000A0007u     //!< MFP3N pins initialization, Reset pin not initialized
#define OSAL_MFP3N_REFSTB_PIN                   0x000A0008u     //!< MFP3N pins initialization, REFSTB pin not initialized
#define OSAL_MFP3N_RD_PIN                       0x000A0009u     //!< MFP3N pins initialization, RD pin not initialized
#define OSAL_MFP3N_WR_PIN                       0x000A000Au     //!< MFP3N pins initialization, WR pin not initialized
#define OSAL_MFP3N_UC_CCLINK_RX_EN_PIN          0x000A000Bu     //!< MFP3N pins initialization, UC_CCLINK_RX_EN pin not initialized
#define OSAL_MFP3N_ADDRESS                      0x000A000Cu     //!< Invalid address of MFP3N chip was used


#define OSAL_GEN_SW_INDEX1                      0x000b0000u     //!< gen_sw; invalid index

#define OSAL_WAITTIMER_INV                      0x000c000Cu     //!< wait timer not supported

#define OSAL_ETH_STM_RESET                      0x000d0001u     //!< Timeout at reset of Ethernet unit in STM32
#define OSAL_ETH_SWITCH_RESET                   0x000d0002u     //!< error at reset or initialization of switch
#define OSAL_ETH_PHY_INIT                       0x000d0003u     //!< error at initialization of PHY
#define OSAL_ETH_LWIP_ASSERT                    0x000d0004u     //!< assert in LwIP stack
#define OSAL_ETH_OUT_OF_MEMORY                  0x000d0005u     //!< malloc failed
#define OSAL_ETH_DMA_CHAIN                      0x000d0006u     //!< DMA chain problem
#define OSAL_ETH_DUPLICATE_IP_ADDR              0x000d0007u     //!< Duplicate IP address detected
#define OSAL_ETH_FRAME_TO_LONG                  0x000d0008u     //!< Frame to send is longer than frame buffer
#define OSAL_ETH_OUT_OF_DESCR                   0x000d0009u     //!< Out of DMA descriptors
#define OSAL_ETH_INVALID_PORT                   0x000d000au     //!< Invalid destination port
#define OSAL_ETH_PHY_SW_RESTART_FUNC            0x000d000bu     //!< softwareRestart Function not registered for an particular Phy
#define OSAL_ETH_PHY_1GB_ADV_FUNC               0x000d000cu     //!< disable 1GBit Advertisment Function not registered for an particular Phy
#define OSAL_ETH_PHY_RGMII_LL_FUNC              0x000d000du     //!< rgmiiLowLatencyEnable Function not registered for an particular Phy
#define OSAL_ETH_PHY_TX_THVALUE                 0x000d000eu     //!< RGMII TX Threshold value out of range
#define OSAL_ETH_PHY_TX_TH_FUNC                 0x000d000fu     //!< rgmiiTxHalfFullThreshold Function not registered for an particular Phy
#define OSAL_ETH_PHY_RX_THVALUE                 0x000d0010u     //!< RGMII RX Threshold value out of range
#define OSAL_ETH_PHY_RX_TH_FUNC                 0x000d0011u     //!< rgmiiRxHalfFullThreshold Function not registered for an particular Phy
#define OSAL_ETH_PHY_SW_RESET_FUNC              0x000d0012u     //!< softwareReset Function not registered for an particular Phy
#define OSAL_ETH_PHY_EN_AUTO_MDIX_FUNC          0x000d0013u     //!< enableAutoMDIX Function not registered for an particular Phy
#define OSAL_ETH_PHY_SET_MII_MODE_FUNC          0x000d0014u     //!< setMiiMode Function not registered for an particular Phy
#define OSAL_ETH_PHY_SET_POWER_MODE_FUNC        0x000d0015u     //!< setPowerMode Function not registered for an particular Phy
#define OSAL_ETH_PHY_GET_POWER_MODE_FUNC        0x000d0016u     //!< getPowerMode Function not registered for an particular Phy
#define OSAL_ETH_PHY_CFG_MLED_FUNC              0x000d0017u     //!< configMLED Function not registered for an particular Phy
#define OSAL_ETH_PHY_EN_EXT_FD_FUNC             0x000d0018u     //!< enableExtFD Function not registered for an particular Phy
#define OSAL_ETH_PHY_EN_ODD_NIBBLE_FUNC         0x000d0019u     //!< enableODDNibbleDet Function not registered for an particular Phy
#define OSAL_ETH_PHY_EN_RX_ERR_IDLE_FUNC        0x000d001au     //!< enableRxErrIdle Function not registered for an particular Phy
#define OSAL_ETH_PHY_CFG_LED_FUNC               0x000d001bu     //!< configLed Function not registered for an particular Phy
#define OSAL_ETH_PHY_CFG_LED_BLINK_FUNC         0x000d001cu     //!< configLedBlink Function not registered for an particular Phy
#define OSAL_ETH_PHY_FAST_LNK_DOWN_FUNC         0x000d001du     //!< enableFastLinkDownDet Function not registered for an particular Phy
#define OSAL_ETH_PHY_FAST_RXD_FUNC              0x000d001eu     //!< enableFastRXDVDet Function not registered for an particular Phy
#define OSAL_ETH_PHY_CFG_STRAP_DONE_FUNC        0x000d001fu     //!< configSwStrapDone Function not registered for an particular Phy
#define OSAL_ETH_PHY_LINK_CFG_FUNC              0x000d0020u     //!< setLinkConfig Function not registered for an particular Phy
#define OSAL_ETH_PHY_GET_AUTO_NEG_FUNC          0x000d0021u     //!< getAutoNegotiation Function not registered for an particular Phy
#define OSAL_ETH_PHY_SET_MDIX_MODE_FUNC         0x000d0022u     //!< setMdixMode Function not registered for an particular Phy
#define OSAL_ETH_PHY_GET_MDIX_MODE_FUNC         0x000d0023u     //!< getMdixMode Function not registered for an particular Phy
#define OSAL_ETH_PHY_GET_SPEED_DUPLEX_FUNC      0x000d0024u     //!< getSpeedDuplex Function not registered for an particular Phy
#define OSAL_ETH_PHY_INVALID_PARAM              0x000d0025u     //!< parameter value is NULL or invalid
#define OSAL_ETH_PHY_OPENFXN_FUNC               0x000d0026u     //!< openFxn Function not registered for an particular Phy
#define OSAL_ETH_PHY_OPENFXN_FAILED             0x000d0027u     //!< openFxn Function reports failed status
#define OSAL_ETH_PHY_COMMANDFXN_FUNC            0x000d0028u     //!< commandFxn Function not registered for an particular Phy
#define OSAL_ETH_PHY_COMMANDFXN_FAILED          0x000d0029u     //!< commandFxn Function reports failed status
#define OSAL_ETH_PHY_CLOSEFXN_FUNC              0x000d002au     //!< closeFxn Function not registered for an particular Phy
#define OSAL_ETH_PHY_CLOSEFXN_FAILED            0x000d002bu     //!< closeFxn Function reports failed status

#define OSAL_TINYFS_INIT                        0x000e0000u     //!< error at initialization tiny flash file system on SPI flash
#define OSAL_TINYFS_TIMEOUT                     0x000e0001u     //!< timeout at write to SPI flash
#define OSAL_TINYFS_UNSUPPORTED                 0x000e0002u     //!< function is not supported

#define OSAL_MUTEX_MULTIPLE_WAIT                0x000f0001u     //!< mutex_get/wait is called more than once for the same mutex
#define OSAL_MUTEX_MULTIPLE_PUT                 0x000f0002u     //!< mutex_put is called more than once for the same mutex

#define OSAL_SCHED_RUN_WRONG_CTX                0x00100000u     //!< BSP_SCHED_run called in wrong context
#define OSAL_SCHED_SLEEP_WRONG_CTX              0x00100001u     //!< BSP_SCHED_sleep called in wrong context

#define OSAL_OS_TOO_MUCH_WAIT_EVT               0x00110000u     //!< BSP OS Too much waiting task for an event
#define OSAL_OS_CMD_QUEUE_FULL                  0x00110001u     //!< BSP OS command queue overflow
#define OSAL_OS_CMD_QUEUE_NULL                  0x00110002u     //!< BSP_OS Command queue Event or instance is NULL
#define OSAL_OS_START_TASK_ERR                  0x00110003u     //!< Try to start a task before everything in initialized
#define OSAL_OS_OWN_TCB_ERR                     0x00110004u     //!< TI-RTOS find own TCB failed

#define OSAL_STACK_INIT_ERROR                   0x00120000u     //!< Error on base initialize Stack
#define OSAL_STACK_PRU_CONFIGGET_ERROR          0x00120001u     //!< Error on get SOC specific PRU config
#define OSAL_STACK_PRECONFIG_ERROR              0x00120002u     //!< Error on base prepare Stack
#define OSAL_STACK_PHYDETECT_ERROR              0x00120003u     //!< Error Phy not found

#define OSAL_TPS1_SHDR_NO_RESPONSE              0x00130000u     //!< TPS1 Chip does not respond to a read/write request
#define OSAL_TPS1_SPI_PORT                      0x00130001u     //!< TPS1 Chip unsupported SPI chosen

#define OSAL_PN_INVALID_ARG1                    0x00140000u     //!< Invalid argument in function call
#define OSAL_PN_PTCP_DEFAULT1                   0x00140000u     //!< Profinet PTCP switch default reached
#define OSAL_PN_PTCP_DEFAULT2                   0x00140001u     //!< Profinet PTCP switch default reached
#define OSAL_PN_PTCP_ASSERT1                    0x00140002u     //!< Profinet PTCP invalid parameter
#define OSAL_PN_PTCP_NUM_OF_PORTS               0x00140003u     //!< Profinet PTCP switch default reached

#define OSAL_KSZ8851_RESET                      0x00150000u     //!< error at reset or initialization of ksz8851

#define OSAL_STARTUP_CRC                        0x00160000u     //!< CRC Error in Firmware detected
#define OSAL_STARTUP_RAM                        0x00160001u     //!< RAM Error during startup detected
#define OSAL_STARTUP_SERIAL                     0x00160002u     //!< Serial Error during startup detected

#define OSAL_USB_PORT_INVALID                   0x00170001u     //!< invalid usb port number
#define OSAL_USB_REQ_PENDING                    0x00170002u     //!< request is already pending

#define OSAL_DMA_INIT_FAILED                    0x00180001u     //!< DMA Init failed

#define OSAL_PRU_FRMWARE_NOTSUPPORTED           0x00190001u     //!< Firmware not supported
#define OSAL_PRU_FRMWARE_NOTSTARTED             0x00190002u     //!< Firmware not started

#define OSAL_TAP_DEV_INVALID_LINK_STATE         0x001a0001u     //!< Linkstate is neither UP nor DOWN

#define OSAL_VNET_DEINIT_HANDLE                 0x001b0001u     //!< Invalid Handle in deinit function
#define OSAL_VNET_CREATE_LX_HANDLE              0x001b0002u     //!< Invalid Linux Handle in create function
#define OSAL_VNET_CREATE_HANDLE                 0x001b0003u     //!< Invalid Handle in create function
#define OSAL_VNET_CREATE_ACCESS                 0x001b0004u     //!< No Access to /dev/net/tun
#define OSAL_VNET_CREATE_OPEN                   0x001b0005u     //!< Unable to open /dev/net/tun
#define OSAL_VNET_CREATE_IOCTL                  0x001b0005u     //!< Unable to create TAP witch ioctl
#define OSAL_VNET_CREATE_NO_MEM                 0x001b0005u     //!< Memory Allocation failed
#define OSAL_VNET_REMOVE_HANDLE                 0x001b0006u     //!< Invalid Handle in remove ()
#define OSAL_VNET_REMOVE_DEVICE_NAME            0x001b0007u     //!< Invalid device name in remove ()
#define OSAL_VNET_REMOVE_NOT_FOUND              0x001b0008u     //!< device not found in remove ()
#define OSAL_VNET_OPEN_LX_HANDLE                0x001b0009u     //!< Invalid Linux Handle in open ()
#define OSAL_VNET_OPEN_HANDLE                   0x001b000au     //!< Invalid Handle in open ()
#define OSAL_VNET_OPEN_DEVICE_NAME              0x001b0009u     //!< Invalid device name in open ()
#define OSAL_VNET_OPEN_NOT_FOUND                0x001b000au     //!< device not found in open ()
#define OSAL_VNET_CLOSE_LX_HANDLE               0x001b000bu     //!< Invalid Linux Handle in close ()
#define OSAL_VNET_CLOSE_HANDLE                  0x001b000cu     //!< Invalid Handle in close ()
#define OSAL_VNET_RX_WAIT_LX_HANDLE             0x001b000du     //!< Invalid Linux Handle in rxWait ()
#define OSAL_VNET_RX_WAIT_HANDLE                0x001b000eu     //!< Invalid Handle in rxWait ()
#define OSAL_VNET_RX_WAIT_SELECT                0x001b000fu     //!< select failed in rxWait ()
#define OSAL_VNET_RX_WAIT_TIMEOUT               0x001b0010u     //!< time out in rxWait ()
#define OSAL_VNET_READ_LX_HANDLE                0x001b0011u     //!< Invalid Linux Handle in read ()
#define OSAL_VNET_READ_HANDLE                   0x001b0012u     //!< Invalid Handle in read ()
#define OSAL_VNET_READ_DATA                     0x001b0013u     //!< read data error in read ()
#define OSAL_VNET_WRITE_LX_HANDLE               0x001b0014u     //!< Invalid Linux Handle in write ()
#define OSAL_VNET_WRITE_HANDLE                  0x001b0015u     //!< Invalid Handle in write ()
#define OSAL_VNET_WRITE_DATA                    0x001b0016u     //!< read data error in write ()
#define OSAL_VNET_MAC_LX_HANDLE                 0x001b0017u     //!< Invalid Linux Handle in mac ()
#define OSAL_VNET_MAC_HANDLE                    0x001b0018u     //!< Invalid Handle in mac ()
#define OSAL_VNET_MAC_FD_HANDLE                 0x001b0019u     //!< Invalid File Handle in mac ()
#define OSAL_VNET_MAC_SOCKET                    0x001b001au     //!< Error creating socket in mac ()
#define OSAL_VNET_MAC_SOCKET_OPT                0x001b001bu     //!< Error in socket opt in mac ()
#define OSAL_VNET_MAC_IOCTL1                    0x001b001cu     //!< Error ioctl in mac ()
#define OSAL_VNET_MAC_IOCTL2                    0x001b001du     //!< Error ioctl in mac ()
#define OSAL_VNET_IPV4_LX_HANDLE                0x001b0017u     //!< Invalid Linux Handle in ipv4 ()
#define OSAL_VNET_IPV4_HANDLE                   0x001b0018u     //!< Invalid Handle in ipv4 ()
#define OSAL_VNET_IPV4_FD_HANDLE                0x001b0019u     //!< Invalid File Handle in ipv4 ()
#define OSAL_VNET_IPV4_SOCKET                   0x001b001au     //!< Error creating socket in ipv4 ()
#define OSAL_VNET_IPV4_SOCKET_OPT               0x001b001bu     //!< Error in socket opt in ipv4 ()
#define OSAL_VNET_IPV4_IOCTL1                   0x001b001cu     //!< Error ioctl in ipv4 ()
#define OSAL_VNET_IPV4_IOCTL2                   0x001b001du     //!< Error ioctl in ipv4 ()
#define OSAL_VNET_ROUTE_LX_HANDLE               0x001b001eu     //!< Invalid Linux Handle in route ()
#define OSAL_VNET_ROUTE_HANDLE                  0x001b001fu     //!< Invalid Handle in route ()
#define OSAL_VNET_ROUTE_FD_HANDLE               0x001b0020u     //!< Invalid File Handle in route ()
#define OSAL_VNET_ROUTE_SOCKET                  0x001b0021u     //!< Error creating socket in route ()
#define OSAL_VNET_ROUTE_SOCKET_OPT              0x001b0022u     //!< Error in socket opt in route ()
#define OSAL_VNET_ROUTE_IOCTL1                  0x001b0023u     //!< Error ioctl in route ()
#define OSAL_VNET_ROUTE_IOCTL2                  0x001b0024u     //!< Error ioctl in route ()
#define OSAL_VNET_LINK_LX_HANDLE                0x001b0025u     //!< Invalid Linux Handle in link ()
#define OSAL_VNET_LINK_HANDLE                   0x001b0026u     //!< Invalid Handle in link ()
#define OSAL_VNET_LINK_FD_HANDLE                0x001b0027u     //!< Invalid File Handle in link ()
#define OSAL_VNET_LINK_VAL                      0x001b0028u     //!< Invalid Links state value link ()
#define OSAL_VNET_FLAGS_DEVICE_NAME             0x001b0029u     //!< Invalid device name in ifFlags ()
#define OSAL_VNET_FLAGS_SOCKET                  0x001b002au     //!< Error creating socket in ifFlags ()
#define OSAL_VNET_FLAGS_IOCTL1                  0x001b002bu     //!< Error ioctl in ifFlags ()
#define OSAL_VNET_FLAGS_IOCTL2                  0x001b002cu     //!< Error ioctl in ifFlags ()

#define OSAL_PRU_PN_NO_MEM                      0x001c0001u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM2                     0x001c0002u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM3                     0x001c0003u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM4                     0x001c0004u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM5                     0x001c0005u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM6                     0x001c0006u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM7                     0x001c0007u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM8                     0x001c0008u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM9                     0x001c0009u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM10                    0x001c000au     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM11                    0x001c000bu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM12                    0x001c000cu     //!< No Memory for allocation
#define OSAL_PRU_PN_ADDR_INFO_ERR               0x001c000du     //!< Get Pru Addressinfo failed
#define OSAL_PRU_PN_NO_MEM13                    0x001c000eu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM14                    0x001c000fu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM15                    0x001c0010u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM16                    0x001c0011u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM17                    0x001c0012u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM18                    0x001c0013u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM19                    0x001c0014u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM20                    0x001c0015u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM21                    0x001c0016u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM22                    0x001c0017u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM23                    0x001c0018u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM24                    0x001c0019u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM25                    0x001c001au     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM26                    0x001c001bu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM27                    0x001c001cu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM28                    0x001c001du     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM29                    0x001c001eu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM30                    0x001c001fu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM31                    0x001c0020u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM32                    0x001c0021u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM33                    0x001c0022u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM34                    0x001c0023u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM35                    0x001c0024u     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM36                    0x001c0025u     //!< No Memory for allocation
#define OSAL_PRU_PN_INV_INT_ARG                 0x001c0026u     //!< Interrupt registration invalid argument type
#define OSAL_PRU_PN_REGISTER_INT_FAILD          0x001c0027u     //!< Interrupt registration failed
#define OSAL_PRU_PN_START_PRU0_FAILED           0x001c0028u     //!< Start of PRU0 failed
#define OSAL_PRU_PN_START_PRU1_FAILED           0x001c0029u     //!< Start of PRU1 failed
#define OSAL_PRU_PN_REGISTER_RX_INT_FAILED      0x001c002au     //!< Interrupt registration failed
#define OSAL_PRU_PN_REGISTER_RX_INT_FAILED2     0x001c002bu     //!< Interrupt registration failed
#define OSAL_PRU_PN_REGISTER_LINK_INT_FAILED    0x001c002cu     //!< Interrupt registration failed
#define OSAL_PRU_PN_GET_IO_FRAME_BUF            0x001c002du     //!< Buffer to short
#define OSAL_PRU_PN_GET_IO_FRAME_ERR            0x001c002eu     //!< Error fetching IO-Frame
#define OSAL_PRU_PN_GET_IO_FRAME_ERR2           0x001c002fu     //!< Error fetching IO-Frame
#define OSAL_PRU_PN_SWITCH_INVALID_PORT         0x001c0030u     //!< Invalid port used for sending
#define OSAL_PRU_PN_ERR_SWITCH_INVALID_PARAM    0x001c0031u     //!< Invalid queue used for sending (obsolete)
#define OSAL_PRU_PN_ERR_BADPACKET               0x001c0032u     //!< Packet to send has invalid length (obsolete)
#define OSAL_PRU_PN_ERR_TX_NO_LINK              0x001c0033u     //!< No link while sending (obsolete)
#define OSAL_PRU_PN_ERR_COLLISION_FAIL          0x001c0034u     //!< No space in collision queue (obsolete)
#define OSAL_PRU_PN_ERR_TX_OUT_OF_BD            0x001c0035u     //!< No space in queue (obsolete)
#define OSAL_PRU_PN_ERR_TRANSLATE               0x001c0036u     //!< Unknown Emac Error (obsolete)
#define OSAL_PRU_PN_PST_PORT_ERR                0x001c0037u     //!< Invalid Portnumber
#define OSAL_PRU_PN_REGISTER_PPM_INT_FAILED     0x001c0038u     //!< Interrupt registration failed
#define OSAL_PRU_PN_REGISTER_CPM_INT_FAILED     0x001c0039u     //!< Interrupt registration failed
#define OSAL_PRU_PN_REGISTER_DHT_INT_FAILED     0x001c003au     //!< Interrupt registration failed
#define OSAL_PRU_PN_NO_MEM37                    0x001c003bu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM38                    0x001c003cu     //!< No Memory for allocation
#define OSAL_PRU_PN_NO_MEM39                    0x001c003du     //!< No Memory for allocation
#define OSAL_PRU_PN_TASK_PTCP_ERR               0x001c003eu     //!< Start of Task not successful
#define OSAL_PRU_PN_TASK_PTCP_SYNC_M_ERR        0x001c003fu     //!< Start of Task not successful
#define OSAL_PRU_PN_ERR_SEND_FAIL               0x001c0040u     //!< Packet sending failed

#define OSAL_PRU_EIP_NO_MEM                     0x001d0000u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM2                    0x001d0001u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM3                    0x001d0002u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM4                    0x001d0003u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM5                    0x001d0004u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM6                    0x001d0005u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM7                    0x001d0006u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM8                    0x001d0007u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM9                    0x001d0008u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM10                   0x001d0009u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM11                   0x001d000au     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM12                   0x001d000bu     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM13                   0x001d000cu     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM14                   0x001d000du     //!< No Memory for allocation
#define OSAL_PRU_EIP_INV_INT_ARG                0x001d000eu     //!< Interrupt registration invalid argument type
#define OSAL_PRU_EIP_REGISTER_INT_FAILD         0x001d000fu     //!< Interrupt registration failed
#define OSAL_PRU_EIP_START_PRU0_FAILED          0x001d0010u     //!< Start of PRU0 failed
#define OSAL_PRU_EIP_START_PRU1_FAILED          0x001d0011u     //!< Start of PRU1 failed
#define OSAL_PRU_EIP_NO_MEM15                   0x001d0012u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM16                   0x001d0013u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM17                   0x001d0014u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM18                   0x001d0015u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM19                   0x001d0016u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM20                   0x001d0017u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM21                   0x001d0018u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM22                   0x001d0019u     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM23                   0x001d001au     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM24                   0x001d001bu     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM25                   0x001d001cu     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM26                   0x001d001du     //!< No Memory for allocation
#define OSAL_PRU_EIP_NO_MEM27                   0x001d001eu     //!< No Memory for allocation
#define OSAL_PRU_EIP_STOP_PRU0_FAILED           0x001d001fu     //!< Stop of PRU0 failed
#define OSAL_PRU_EIP_STOP_PRU1_FAILED           0x001d0020u     //!< Stop of PRU1 failed
#define OSAL_PRU_EIP_WRITE_PRU0_FAILED          0x001d0021u     //!< Write to IRAM of PRU0 failed
#define OSAL_PRU_EIP_WRITE_PRU1_FAILED          0x001d0022u     //!< Write to IRAM of PRU1 failed
#define OSAL_PRU_EIP_ADDR_INFO_ERR              0x001d0023u     //!< Get Pru Addressinfo failed
#define OSAL_PRU_EIP_REGISTER_RX_INT_FAILED     0x001d0024u     //!< Interrupt registration failed
#define OSAL_PRU_EIP_REGISTER_LINK_INT_FAILED   0x001d0025u     //!< Interrupt registration failed
#define OSAL_PRU_EIP_SWITCH_INVALID_PORT        0x001d0026u     //!< Invalid port used for sending
#define OSAL_PRU_EIP_ERR_SWITCH_INVALID_PARAM   0x001d0027u     //!< Invalid queue used for sending (obsolete)
#define OSAL_PRU_EIP_ERR_BADPACKET              0x001d0028u     //!< Packet to send has invalid length (obsolete)
#define OSAL_PRU_EIP_ERR_TX_NO_LINK             0x001d0029u     //!< No link in time of send (obsolete)
#define OSAL_PRU_EIP_ERR_COLLISION_FAIL         0x001d002au     //!< No space in collision queue (obsolete)
#define OSAL_PRU_EIP_ERR_TX_OUT_OF_BD           0x001d002bu     //!< No space in queue (obsolete)
#define OSAL_PRU_EIP_ERR_TRANSLATE              0x001d002cu     //!< Unknown Emac Error (obsolete)
#define OSAL_PRU_EIP_PST_PORT_ERR               0x001d002du     //!< Invalid Portnumber
#define OSAL_PRU_EIP_GET_IO_FRAME_BUF           0x001d002eu     //!< Buffer to short
#define OSAL_PRU_EIP_GET_IO_FRAME_ERR           0x001d002fu     //!< Error fetching IO-Frame
#define OSAL_PRU_EIP_GET_IO_FRAME_ERR2          0x001d0030u     //!< Error fetching IO-Frame
#define OSAL_PRU_EIP_ERR_SEND_FAIL              0x001d0031u     //!< Packet sent failed

#define OSAL_SEM_CREATE_FAILED                  0x001e0001u     //!< Semaphore creation failed
#define OSAL_INTF_SHED_NO_IMPLEMENTATION        0x001e0000u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION2       0x001e0001u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION3       0x001e0002u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION4       0x001e0003u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION5       0x001e0004u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION6       0x001e0005u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION7       0x001e0006u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION8       0x001e0007u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION9       0x001e0008u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION10      0x001e0009u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION11      0x001e000au     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION12      0x001e000bu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION13      0x001e000cu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION14      0x001e000du     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION15      0x001e000eu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION16      0x001e000fu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION17      0x001e0010u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION18      0x001e0011u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION19      0x001e0012u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION20      0x001e0013u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION21      0x001e0014u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION22      0x001e0015u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION23      0x001e0016u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION24      0x001e0017u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION25      0x001e0018u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION26      0x001e0019u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION27      0x001e001au     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION28      0x001e001bu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION29      0x001e001cu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION30      0x001e001du     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION31      0x001e001eu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION32      0x001e001fu     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION33      0x001e0020u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION34      0x001e0021u     //!< There is no specific implementation of a function
#define OSAL_INTF_SHED_NO_IMPLEMENTATION35      0x001e0022u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION1        0x001e0023u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION2        0x001e0024u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION3        0x001e0025u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION4        0x001e0026u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION5        0x001e0027u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION6        0x001e0028u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION7        0x001e0029u     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION8        0x001e002au     //!< There is no specific implementation of a function
#define OSAL_INTF_IRQ_NO_IMPLEMENTATION9        0x001e002bu     //!< There is no specific implementation of a function

#define OSAL_ECSTUB_NOTINSTANTIATED             0x40000001u     //!< Interface functions not registered
#define OSAL_ECSTUB_NOTIMPLEMENTED              0x40000002u     //!< Interface function not implemented
#define OSAL_ECSTUB_INVALIDPARAMETER            0x40000003u     //!< Invalid parameter populate Interface

#define OSAL_CONTAINER_INVALIDPARAMETER         0x40800001u     //!< Invalid parameter in application
#define OSAL_CONTAINER_NOMEMORY                 0x40800002u     //!< No memory in application
#define OSAL_CONTAINER_NOSHAREDMEMORY           0x40800003u     //!< No shared memory in application
#define OSAL_CONTAINER_LOCALIMPLEMENTATION      0x40800004u     //!< Implementation is local not DPR
#define OSAL_CONTAINER_NOTSUPPORTED             0x40800005u     //!< Implementation not yet done

// if expression is false, handle error by trap into OSALrror(...)
#define OSAL_ASSERT(expr, errCode)   if (!(expr)) OSAL_error ( __func__, __LINE__, errCode, true, 0)

#if (defined __cplusplus)
extern "C" {
#endif

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_OSAL_ERROR_H */
