/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure" */
/**********************************************************************
 * Copyright (C) 2012-2021 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************
 * WARNING: This file is auto-generated using api-generator utility.
 *          api-generator: 12.02.13bb8d5
 *          Do not edit it manually.
 **********************************************************************
 * Cadence Core Driver for LPDDR4.
 **********************************************************************/
#ifndef LPDDR4_STRUCTS_IF_H
#define LPDDR4_STRUCTS_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include "lpddr4_if.h"

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
 * Structures and unions
 **********************************************************************/
/**
 * Configuration of device.
 * Object of this type is used for probe and init functions.
*/
struct LPDDR4_Config_s
{
    /** Base address of controller registers */
    struct LPDDR4_CtlRegs_s* ctlBase;
    /** Information/warning handler */
    LPDDR4_InfoCallback infoHandler;
    /** Controller interrupt handler */
    LPDDR4_CtlCallback ctlInterruptHandler;
    /** PHY Independent Module interrupt handler */
    LPDDR4_PhyIndepCallback phyIndepInterruptHandler;
};

/**
 * Structure contains private data for Core Driver that should not be used by
 * upper layers. This is not a part of API and manipulating of those data may cause
 * unpredictable behavior of Core Driver.
*/
struct LPDDR4_PrivateData_s
{
    /** Base address of controller registers */
    struct LPDDR4_CtlRegs_s* ctlBase;
    /** Information/warning handler */
    LPDDR4_InfoCallback infoHandler;
    /** Controller interrupt handler */
    LPDDR4_CtlCallback ctlInterruptHandler;
    /** PHY Independent Module interrupt handler */
    LPDDR4_PhyIndepCallback phyIndepInterruptHandler;
    /** Multiple instances of DDR controller */
    void* ddr_instance;
};

/** Structure to contain debug information reported by the driver. */
struct LPDDR4_DebugInfo_s
{
    /** PLL Lock error. */
    uint8_t pllError;
    /** I/O calibration error. */
    uint8_t ioCalibError;
    /** RX offset error. */
    uint8_t rxOffsetError;
    /** CA training error. */
    uint8_t caTraingError;
    /** Write levelling error. */
    uint8_t wrLvlError;
    /** Gate Level error. */
    uint8_t gateLvlError;
    /** Read Level error. */
    uint8_t readLvlError;
    /** Write DQ training error. */
    uint8_t dqTrainingError;
};

/** Frequency Set Point mode register values */
struct LPDDR4_FspModeRegs_s
{
    /** MR1 register data for the FSP. */
    uint8_t mr1Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR2 register data for the FSP. */
    uint8_t mr2Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR3 register data for the FSP. */
    uint8_t mr3Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR11 register data for the FSP. */
    uint8_t mr11Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR12 register data for the FSP. */
    uint8_t mr12Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR13 register data for the FSP. */
    uint8_t mr13Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR14 register data for the FSP. */
    uint8_t mr14Data_fN[LPDDR4_INTR_MAX_CS];
    /** MR22 register data for the selected frequency. */
    uint8_t mr22Data_fN[LPDDR4_INTR_MAX_CS];
};

/**
 *  @}
 */


#ifdef __cplusplus
}
#endif

#endif	/* LPDDR4_STRUCTS_IF_H */
