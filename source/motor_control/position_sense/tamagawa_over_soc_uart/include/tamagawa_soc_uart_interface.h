/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TAMAGAWA_UART_DRV_H_
#define TAMAGAWA_UART_DRV_H_
 /**
 * \defgroup MOTOR_CONTROL_API APIs for Motor Control Encoders
 *
 * This module contains APIs for device drivers for various motor control encoders supported in this SDK.
 */

/**
 *  \defgroup TAMAGAWA_UART_API_MODULE APIs for Tamagawa Encoder Over UART
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
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief  Used to set the maximum address that can be used for EEPROM Read/Write
 */
#define MAX_EEPROM_ADDRESS (127)
/**
* \brief Used to set the maximum value that can be written in EEPROM
*/
#define MAX_EEPROM_WRITE_DATA (255)
/**
 *    @brief    Data ID codes
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
 * @brief Tamagawa Interface Transmit data
 */
struct tamagawa_tx
{
    uint8_t  adf;   /**< EEPROM address */
    uint8_t  edf;   /**< EEPROM data */
};

/**
 * @brief Tamagawa Interface Received data
 */
struct tamagawa_rx
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
 * @brief Tamagawa Interface
 */
struct tamagawa_uart_interface
{
    uint8_t  data_id;  /**< Data ID code */
    struct tamagawa_tx tx;      /**< Transmit data */
    struct tamagawa_rx rx;      /**< Received data */
    uint8_t  rx_crc;   /**< Calculated CRC */
    uint32_t uart_instance; /**< Uart instance*/
    uint32_t gpio_base_address; /**< GPIO pin base address*/
    uint32_t gpio_pin_number;   /**<GPIO Pin number*/
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
/**
 *  \brief      Process The Tamagawa Command
 *
 *  \param[in]  tamagawa_interface         Tamagawa Interface
 *  \param[in]  gUartHandle    uart handle
 *  \param[in]  cmd             tamagawa command number
 *
 *
 *  \retval     0       success
 *  \retval     -EINVAL failure
 *
 */
int32_t tamagawa_command_process(volatile struct tamagawa_uart_interface *tamagawa_interface, UART_Handle *gUartHandle, int32_t cmd);
/**
 *  \brief      Compare Received CRC and Calculated CRC
 *
 *  \param[in]  tamagawa_interface         Tamagawa Interface
 *
 *
 *  \retval     1       CRC Success
 *  \retval     0       CRC failure
 *
 */
int32_t tamagawa_crc_verify(volatile struct tamagawa_uart_interface *tamagawa_interface);
/**
 *  \brief      Configure GOPIO pin for RTSn(sw flow control)
 *
 *  \param[in]  tamagawa_interface         Tamagawa Interface
 *  \param[in]  instance                   UART communication instance
 *  \param[in]  base_address               GPIO PIN address
 *  \param[in]  pin_number                 GPIO PIN Number
 *  \param[in]  pin_direction              GPIO pin direction (output/input)
 *
 *
 *
 */

void tamagawa_init(volatile struct tamagawa_uart_interface *tamagawa_interface, uint32_t instance , uint32_t base_address, uint32_t pin_number, uint32_t pin_direction);

/** @} */

#ifdef __cplusplus
}
#endif

#endif