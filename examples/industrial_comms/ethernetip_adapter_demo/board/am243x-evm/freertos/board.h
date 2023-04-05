/*
 *  Copyright (c) 2021, KUNBUS GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef BOARD_H_INC
#define BOARD_H_INC

#ifdef __cplusplus
extern "C" {
#endif

// attributes dependent on configuration/board
#define PRODUCT_CODE_OF_CONFIGURATION   0x2401
#define PRODUCT_NAME_OF_CONFIGURATION   "EtherNet/IP(tm) ADPT am243x-evm"
#define TIMESYNC_PRODUCT_DESCRIPTION_OF_CONFIGURATION   "Kunbus Ethernet/IP CIP Sync;buf_serial_number;";

/*
*  http://standards-oui.ieee.org/oui.txt
*  C8-3E-A7   (hex)        KUNBUS GmbH
*  C83EA7     (base 16)        KUNBUS GmbH
*  Heerweg 15C
*  Denkendorf  BW  D-73770
*  DE
*/
#define TIMESYNC_MANUFACTURE_ID_OF_CONFIGURATION        "\xC8\x3E\xA7\x00";
#define TIMESYNC_REVISION_DATA_OF_CONFIGURATION         "1.0;1.0;1.00";

// processor register address defines
#define IDK_CTRLMMR0_MAC_ID0 ((uint32_t*) 0x43000200u) // Ethernet MAC address lower 32 bit
#define IDK_CTRLMMR0_MAC_ID1 ((uint32_t*) 0x43000204u) // Ethernet MAC address upper 16 bit

// Application configuration
#define APP_MAIN_STACK_SIZE_IN_BYTES    (0x1000u)

// Board HW configuration
#define CONFIG_INSTANCE_INVALID         ~0

// Memory type for permanent data
#define PERMANENT_TYPE_FLASH  (0u)
#define PERMANENT_TYPE_EEPROM (1u)

#if (PERMANENT_TYPE_FLASH)
    #define PERMANENT_DATA_MEMORY_TYPE      CUST_DRIVERS_PRM_eTYPE_FLASH
    #define PERMANENT_DATA_MEMORY_INSTANCE  CONFIG_FLASH0
    #define PERMANENT_DATA_MEMORY_OFFSET    (0x200000u)
#elif (PERMANENT_TYPE_EEPROM)
    #define PERMANENT_DATA_MEMORY_TYPE      CUST_DRIVERS_PRM_eTYPE_EEPROM
    #define PERMANENT_DATA_MEMORY_INSTANCE  CONFIG_EEPROM0
    #define PERMANENT_DATA_MEMORY_OFFSET    (0x200u)
#else
    #define PERMANENT_DATA_MEMORY_TYPE      CUST_DRIVERS_PRM_eTYPE_UNDEFINED
    #define PERMANENT_DATA_MEMORY_INSTANCE  CONFIG_INSTANCE_INVALID
    #define PERMANENT_DATA_MEMORY_OFFSET    (0x0u)
#endif

#define INDUSTRIAL_LEDS_INSTANCE        CONFIG_LED0


#define PRU_ICSS_BLOCK_INSTANCE         CONFIG_PRU_ICSS1
#define PRU_ICSS_ETHPHY_0_INSTANCE      CONFIG_ETHPHY0
#define PRU_ICSS_ETHPHY_1_INSTANCE      CONFIG_ETHPHY1

#define PRINTF_CALLBACK_FUNCTION        CUST_DRIVERS_UART_printf
#define PRINTF_UART_CALLBACK_INSTANCE   CONFIG_UART_CONSOLE

#ifdef  __cplusplus 
}
#endif 

#endif // BOARD_H_INC
