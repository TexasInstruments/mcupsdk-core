/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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


/*===========================================================================
  ===========================================================================
  ==      File:    fss_m8051.h
  ==      Author:  Dave Brier
  ==
  ==      Purpose:  This file contains definitions for the various access
  ==                locations in the OptiFlash design that are accessed by
  ==                the M8051 processor. The attempt is to support two
  ==                compilers, the Keil compiler run in Windows and
  ==                the SDCC compiler run in Linux. There are different
  ==                syntax protocols between the two compilers and this
  ==                file will capture those.
  ==
  ==       Version:
  ==        0.1   03-16-22   dbrier
  ==
  ==========================================================================*/

#ifndef __FSS_M8051_H__
#define __FSS_M8051_H__

#include "fss_m8051_const.h"

/**
 * @brief Read OSPI Controller config registers
 * 
 * Based on rsel value, operation will done. Please go through the 
 * documentation for more details.
 * 
 * @param addr Address of the config register 
 * @param rsel Rsel value
 */
void FOTA_ReadCfg(uint16_t addr, uint8_t rsel);

/**
 * @brief Read OSPI Config register.
 * 
 * This API suppose the OSPI register address is already 
 * loaded in the correct place. Calling this API will 
 * simply read the value from the OSPI Controller register. 
 * Based on rsel value, operation will done. Please go through  
 * the documentation for more details.
 * 
 * @param rsel Rsel value
 */
void FOTA_ReadCfgPreload(uint8_t rsel);

/**
 * @brief Transfer write buffer to flash controller
 * 
 */
void FOTA_TransferDataWrRsel3();

/**
 * @brief Wait for GO bit ot set
 * 
 */
void FOTA_WaitGo();

/**
 * @brief request to get the OSPI controller config bug ownership.
 * 
 */
void FOTA_GetCfgBusOwnership();

/**
 * @brief release the OSPI controller config bug ownership.
 * 
 */
void FOTA_ReleaseCfgBusOwnership();

/**
 * @brief request to get the OSPI data bus ownership
 * 
 */
void FOTA_GetDatBusOwnership();

/**
 * @brief request to get the OSPI data bus ownership wBuf
 * 
 */
void FOTA_GetDatBusOwnershipWbuf();

/**
 * @brief release the OSPI data bus ownership
 * 
 */
void FOTA_ReleaseDatBusOwnership();

/**
 * @brief wait for some time 
 * 
 * @param time time 
 * @param mode timer mode 
 */
void FOTA_WaitTimer0(uint16_t time, uint8_t mode);

/**
 * @brief write OSPI controller registers
 * 
 * @param addr address of ospi controller register
 * @param rsel 
 */
void FOTA_WriteCfg(uint16_t addr, uint8_t rsel);

/**
 * @brief write pre config data to OSPI controller registers
 * 
 * this version of the function requires that the
 * address registers which are loaded above be already
 * set before it is called. This version of the
 * write function is used in any timing sensitive
 * section of code.
 * @param rsel rsel 
 */
void FOTA_WriteCfgPreload(uint8_t rsel);

/**
 * @brief get STIG status 
 * 
 */
void FOTA_GetFlashStatusSTIG();

/**
 * @brief  Polling flash flag_reg to deteremine if it is busy.
 * 
 * @param delay amount of delay to wait
 */
void FOTA_WaitFlashBusy(uint16_t delay);

/**
 * @brief This function will erase the flash sector
 * 
 * specified by the address passed in IS25LX064
 */
void FOTA_EraseSector();


void FOTASendGenericCommand();

#endif /*define __FSS_M8051_H__*/
