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
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#ifndef _TIESCHW_H_
#define _TIESCHW_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <industrial_comms/ethercat_slave/icss_fwhal/tiescbsp.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DISABLE_ESC_INT()               ;//bsp_global_mutex_lock();
#define ENABLE_ESC_INT()                ;//bsp_global_mutex_unlock();

#ifndef TIMER_INT_HEADER
#define TIMER_INT_HEADER
#endif

#ifndef EE_BUSY_TIMEOUT_VALUE
#define EE_BUSY_TIMEOUT_VALUE 0x2000
#endif

#define ESC_RD                  0x02    /* read acces to ESC */
#define ESC_WR                  0x04    /* write acces to ESC */

#define ESC_PDI_INTERFACE_ON_CHIP_BUS       0x80
#define ESC_PDI_INTERFACE_SPI_SLAVE         0x05

#ifdef  _TIESCHW_
#undef PROTO
#define PROTO
#else
#define PROTO extern
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/** @addtogroup ECAT_MISC
@{ */
/**
*  @brief     Initializes EtherCAT slave interface by calling bsp_init. Wait for firmware to be ready. Reset AL event mask.
*  Register PDI and SYNC interrupt handlers. Disables all SyncManagers
*
*  @retval 0
*/
PROTO uint8_t HW_Init(void);
/**
*  @brief    Wrapper API for bsp_exit, delete RTOS resources, flush ESI EEPROM, disable PRUs
*
*/
PROTO void HW_Release(void);
/**
*  @brief    Perform the WARM RST of AM335x
*
*  <b>IMPORTANT NOTE</b> \n
*  This will result in application state to be lost and disrupt ongoing EtherCAT communication. Typical use is to upgrade the EtherCAT firmware
*/
PROTO void HW_RestartTarget(void);
#if UC_SET_ECAT_LED
/**
*  @brief    API to control onboard Run and ERR LEDs
*
*  @param RunLed 1 : on 0 : off
*  @param ErrLed   1 : on 0 : off
*/
PROTO void HW_SetLed(uint8_t RunLed, uint8_t ErrLed);
#endif
/**
@}
*/

#define HW_GetALEventRegister() (bsp_read_word(pruIcss1Handle, ESC_ADDR_AL_EVENT_REQ))
#define HW_GetALEventRegister_Isr() (bsp_read_word_isr(pruIcss1Handle, ESC_ADDR_AL_EVENT_REQ))
#define HW_GetALEventRegisterDword_Isr() (bsp_read_dword_isr(pruIcss1Handle, ESC_ADDR_AL_EVENT_REQ))

/** @addtogroup ECAT_INTMGMNT
 @{ */
/**
*  @brief  Reset AL event mask in AL event mask register (0x204), write AND of
*  intMask and current AL event mask value to AL event mask register
*
*  @param intMask AL event mask
*/
PROTO void HW_ResetALEventMask(uint16_t intMask);
/**
*  @brief  Set AL event mask in AL event mask register (0x204), write OR of
*  intMask and current AL event mask value to AL event mask register
*
*  @param intMask AL event mask
*/
PROTO void HW_SetALEventMask(uint16_t intMask);
/**
*  @brief   Return the current AL event mask used in Stack
*
*  @retval Current AL event mask
*/
PROTO uint16_t HW_GetALEventMask(void);
/**
*  @brief    PDI IRQ handler in the EtherCAT slave stack
*/
PROTO void HW_EcatIsr(void);
/**
*  @brief    PDI IRQ handler in the EtherCAT slave stack
*/
PROTO void PDI_Isr(void);
/**
*  @brief    SYNC0 IRQ handler in the EtherCAT slave stack
*/
PROTO void Sync0_Isr(void);
/**
*  @brief    SYNC1 IRQ handler in the EtherCAT slave stack
*/
PROTO void Sync1_Isr(void);
/**
*  @brief    SYNC1 IRQ handler in the EtherCAT slave stack
*/
/**
@}
*/

PROTO void HW_EscRead(uint8_t *pData, uint16_t Address, uint16_t Len);

/** @addtogroup ECAT_ESC_REG_ACCESS
 @{ */
/**
*  @brief  Used by application to read process data byte from ESC memory
*
*  @param Address ESC address to read from PDI
*  @param ByteValue at address
*
*/
PROTO void HW_EscReadByteIsr(uint8_t ByteValue, uint16_t Address);
/**
*  @brief  Used by application to read process data byte from ESC memory
*
*  @param Address ESC address to read from PDI
*  @param WordValue at address
*
*/
PROTO void HW_EscReadWordIsr(uint16_t WordValue, uint16_t Address);
/**
*  @brief  Used by application to read process data byte from ESC memory
*
*  @param Address ESC address to read from PDI
*  @param DWordValue at address
*
*/
PROTO void HW_EscReadDWordIsr(uint32_t DWordValue, uint16_t Address);
/**
*  @brief  Used by application to read process data byte from ESC memory
*
*  @param pData Pointer to the data array to read
*  @param Address ESC address to read from PDI
*  @param Len Length
*
*/
PROTO void HW_EscReadIsr(uint8_t *pData, uint16_t Address, uint16_t Len);

/**
*  @brief  Write a byte value at 'address' in ESC memory - SM mailbox (single buffer) mapped or register area
*
*  @param ByteValue Value to write
*  @param Address ESC address to write from PDI
*
*/
PROTO void HW_EscWriteByte(uint8_t ByteValue, uint16_t Address);

/**
*  @brief  Write a byte value at 'address' in ESC memory
*
*  @param ByteValue Value to write
*  @param Address ESC address to write from PDI
*
*/
PROTO void HW_EscWriteByteIsr(uint8_t ByteValue, uint16_t Address);

/**
*  @brief   Write a 16-bit value to 'address' in ESC memory - SM mailbox (single buffer) mapped or register area
*
*  @param WordValue Value at address
*  @param Address ESC address to write from PDI
*
*/
PROTO void HW_EscWriteWord(uint16_t WordValue, uint16_t Address);

/**
*  @brief    Write a word value at 'address' in ESC process data memory - SM buffer (3-buffer) mapped area
*
*  @param WordValue Value to write
*  @param Address ESC address to write from PDI
*
*/
PROTO void HW_EscWriteWordIsr(uint16_t WordValue, uint16_t Address);

/**
*  @brief    Write a 32-bit value to 'address' in ESC memory - SM mailbox (single buffer) mapped or register area
*
*  @param DWordValue Value at address
*  @param Address ESC address to write from PDI
*
*/
PROTO void HW_EscWriteDWord(uint32_t DWordValue, uint16_t Address);

/**
*  @brief   Write a 32-bit dword value at 'address' in ESC process data memory - SM buffer (3-buffer) mapped area
*
*  @param DWordValue Value to write
*  @param Address ESC address to write from PDI
*
*/
PROTO void HW_EscWriteDWordIsr(uint32_t DWordValue, uint16_t Address);

/**
*  @brief   Write 'len' bytes to 'address' in ESC memory from pdata
*
*  @param pData Pointer to data array in application
*  @param Address ESC address to write from PDI
*  @param Len Length of data array in application
*
*/
PROTO void HW_EscWrite(uint8_t *pData, uint16_t Address, uint16_t Len);

/**
*  @brief   Write 'len' bytes to 'address' in ESC memory from pdata, used to copy process data input values from slave application to ESC memory
*  for sending to EtherCAT master
*
*  @param pData Pointer to data array in application
*  @param Address ESC address to write from PDI
*  @param Len Length of data array in application
*
*/
PROTO void HW_EscWriteIsr(uint8_t *pData, uint16_t Address, uint16_t Len);
/**
@}
*/


PROTO uint8_t __HW_EscReadByteIsr(PRUICSS_Handle pruIcssHandle,
                                  uint16_t Address);
PROTO uint16_t __HW_EscReadWordIsr(PRUICSS_Handle pruIcssHandle,
                                   uint16_t Address);
PROTO uint32_t __HW_EscReadDWordIsr(PRUICSS_Handle pruIcssHandle,
                                    uint16_t Address);

#define HW_EscReadByteIsr(ByteValue, Address) ((ByteValue) = __HW_EscReadByteIsr(pruIcss1Handle, Address))
#define HW_EscReadWordIsr(WordValue, Address) ((WordValue) = __HW_EscReadWordIsr(pruIcss1Handle, Address))
#define HW_EscReadDWordIsr(DWordValue, Address)((DWordValue) = __HW_EscReadDWordIsr(pruIcss1Handle, Address))

#define HW_EscReadByte(ByteValue, Address) ((ByteValue) = bsp_read_byte(pruIcss1Handle, Address))
#define HW_EscReadWord(WordValue, Address) ((WordValue) = bsp_read_word(pruIcss1Handle, Address))
#define HW_EscReadDWord(DWordValue, Address) ((DWordValue) = bsp_read_dword(pruIcss1Handle, Address))



/** @addtogroup ECAT_SYNC_MAILBOX
 @{ */
/**
*  @brief  Read 'len' bytes from ESC mailbox @ 'address' into application buffer pointed by 'pdata'
*
*  @param pData Pointer to data array in application
*  @param Address Mailbox address to read form PDI
*  @param Len Length of data array in application
*
*/
PROTO void HW_EscReadMbxMem(uint8_t *pData, uint16_t Address, uint16_t Len);
/**
*  @brief  Write 'len' bytes from application buffer pointed by 'pdata' and copy to ESC mailbox @ 'address'
*
*  @param pData Pointer to data array in application
*  @param Address Mailbox address to write form PDI
*  @param Len Length of data array in application
*
*/
PROTO void HW_EscWriteMbxMem(uint8_t *pData, uint16_t Address, uint16_t Len);
/**
@}
*/








/** @addtogroup ECAT_SYNC_ACC_CTRL
 @{ */
/**
*  @brief  Disables SYNC manager channel from PDI
*
*  @param channel channel number(0-7)
*
*/
PROTO void HW_DisableSyncManChannel(uint16_t channel);
/**
*  @brief Enables SYNC manager channel from PDI
*
*  @param channel channel number(0-7)
*
*/
PROTO void HW_EnableSyncManChannel(uint16_t channel);
/**
*  @brief  Return SM properties like (address, length, control byte) to application from corresponding ESC registers
*
*  @param channel channel number(0-7)
*
*  @retval Pointer to requested SM channel settings
*
*/
PROTO void *HW_GetSyncMan(uint8_t channel);
/**
@}
*/

/** @addtogroup ECAT_TIMER
 @{ */
/**
*  @brief Returns the time difference from last call of "HW_ClearTimer" to this "HW_GetTimer". handles overflow.
*  This is a wrapper API used by SSC.
*
*  @retval 32-bit time elapsed since last call to HW_ClearTimer
*/
PROTO unsigned int HW_GetTimer();
/**
*  @brief Update the time when "HW_ClearTimer" last invoked.This is a wrapper API
*/
PROTO void HW_ClearTimer();

/**
@}
*/

/** @addtogroup ECAT_ESI_EEPROM
 @{ */
/**
*  @brief  Perform reload operation after validating EEPROM CRC
*
*/
PROTO uint16_t HW_EepromReload();
/**
@}
*/


#ifdef ENABLE_PDI_SWI
PROTO void PDI_Swi(void);
#endif

#undef PROTO

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern PRUICSS_Handle pruIcss1Handle;


#ifdef __cplusplus
}
#endif

#endif //_TIESCHW_H_
