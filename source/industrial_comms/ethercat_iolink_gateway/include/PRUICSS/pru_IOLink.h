/*!
* \file pru_IOLink.h
*
* \brief
* PRU Integration: IOLink specific interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-20
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

#if !(defined __PRU_IOLINK_H__)
#define __PRU_IOLINK_H__		1

#include <stdint.h>
#include <stdbool.h>

#include <IOLM_Types.h>

#if (defined OSAL_LINUX)
#define ARM_INTERRUPT_OFFSET    0
#elif (defined OSAL_TIRTOS) || (defined OSAL_FREERTOS)
#if (defined SOC_AM335x)
#define ARM_INTERRUPT_OFFSET    0
#elif defined (SOC_AM437x)
#define ARM_INTERRUPT_OFFSET    32  /* 52 - 20 */
#elif defined (SOC_AM572x) || (defined SOC_AM574x)
#define ARM_INTERRUPT_OFFSET    132
#elif defined (SOC_K2G)
#define ARM_INTERRUPT_OFFSET_ICSS0 236
#define ARM_INTERRUPT_OFFSET_ICSS1 244
#elif defined(SOC_AM65XX)
#if defined (__aarch64__)
 /* A53 */
#define ARM_INTERRUPT_OFFSET_ICSS0 (286-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (294-20)
#define ARM_INTERRUPT_OFFSET_ICSS2 (302-20)
#else
 /* R5F */
#define ARM_INTERRUPT_OFFSET_ICSS0 (160-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (168-20)
#define ARM_INTERRUPT_OFFSET_ICSS2 (176-20)
#endif
#elif defined(SOC_AM64X) || defined(SOC_AM243X)
#if defined (__aarch64__)
 /* A53 */
#define ARM_INTERRUPT_OFFSET_ICSS0 (286-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (294-20)
#define ARM_INTERRUPT_OFFSET_ICSS2 (302-20)
#else
 /* R5F */
#define ARM_INTERRUPT_OFFSET_ICSS0 (120-20)
#define ARM_INTERRUPT_OFFSET_ICSS1 (248-20)
#endif
#else
#error Required to configure valid MCU platform
#endif
#else
#error "Unknown OS"
#endif

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#define MCU_getARMInterruptOffset(pruInst) \
    ( (PRUICSS_INSTANCE_ONE == (pruInst))?((int16_t)ARM_INTERRUPT_OFFSET_ICSS0):((int16_t)ARM_INTERRUPT_OFFSET_ICSS1) )

#elif (defined SOC_AM65XX)
#define MCU_getARMInterruptOffset       ((int16_t)(ARM_INTERRUPT_OFFSET_ICSS0)) /* always zero, IRQs are mapped to low, depending on PRU */
#else
#define MCU_getARMInterruptOffset       ((int16_t)(ARM_INTERRUPT_OFFSET))
#endif

// structures
typedef enum IOLM_PL_ePortMode {
    IOLM_PL_eModeSioInactive = 0,
    IOLM_PL_eModeSdci,
    IOLM_PL_eModeSioDI,
    IOLM_PL_eModeSioDO,
    IOLM_PL_eMode_Force32Bit = 0x7fffffff
} IOLM_PL_ePortMode_t;

typedef struct IOLM_PL_sGpioConfig
{
#ifndef OSAL_FREERTOS
    uint32_t    gpioID;
#else 
    uint32_t    gpioBase;
    uint32_t    gpioPin;
#endif
    int16_t     ctlRegOffset;
} IOLM_PL_sGpioConfig_t;

typedef struct IOLM_PL_sGpioTxConfig
{
    IOLM_PL_sGpioConfig_t   gpio;
    uint32_t                pruPin; // pru pin number
} IOLM_PL_sGpioTxConfig_t;

typedef struct IOLM_PL_sGpioEnConfig
{
    IOLM_PL_sGpioConfig_t   gpio;
    uint32_t                gpioPhysAddr; // physical address for pru to access gpio
    uint32_t                gpioPinMask; // mask to write on gpioPhysAddr
} IOLM_PL_sGpioEnConfig_t;

typedef struct IOLM_PL_sPortConfig
{
    IOLM_PL_sGpioConfig_t       rx;
    IOLM_PL_sGpioTxConfig_t     tx;
    IOLM_PL_sGpioEnConfig_t     txEn;
    IOLM_PL_sGpioConfig_t       pwrEn;
} IOLM_PL_sPortConfig_t;

typedef struct IOLM_PL_sEventIrq
{
    uint32_t pruIrqNum;
    uint32_t pruIrqPrio;
}IOLM_PL_sEventIrq_t;

typedef struct IOLM_PL_sPruIccsCfg
{
    uint32_t            pruIcssInst;
    IOLM_PL_sEventIrq_t eventIrq[8];
}IOLM_PL_sPruIccsCfg_t;

typedef void (*IOLM_PL_cbPRUsetMode_t)        (uint8_t instance_p, uint8_t portNum_p, IOLM_PL_ePortMode_t mode_p);
typedef const IOLM_PL_sPortConfig_t* (*IOLM_PL_cbPRUgetPortCfg_t)     (uint8_t instance_p, uint8_t portNum_p);
typedef const IOLM_PL_sPruIccsCfg_t* (*IOLM_PL_cbPRUgetPruCfg_t)      (uint8_t instance_p);
typedef void (*IOLM_PL_cbPRUsetDO_t)          (uint8_t instance_p, uint8_t portNum_p, bool portTargetState_p);
typedef bool (*IOLM_PL_cbPRUgetDI_t)          (uint8_t instance_p, uint8_t portNum_p);
typedef void (*IOLM_PL_cbPRUsetPower_t)       (uint8_t instance_p, uint8_t portNum_p, bool powerState_p);
typedef void (*IOLM_PL_cbPRUSetIQ_t)          (uint8_t instance_p, uint8_t portNum_p, bool outValue_p);
typedef bool (*IOLM_PL_cbPRUGetIQ_t)          (uint8_t instance_p, uint8_t portNum_p);
typedef void (*IOLM_PL_cbPRUSetIQMode_t)      (uint8_t instance_p, uint8_t portNum_p, IOL_EIQMode eIQMode_p);

typedef struct IOLM_PL_sCallbacks
{
    IOLM_PL_cbPRUsetMode_t      cbSetMode;
    IOLM_PL_cbPRUgetPortCfg_t   cbGetPortCfg;
    IOLM_PL_cbPRUgetPruCfg_t    cbGetPruCfg;
    IOLM_PL_cbPRUsetDO_t        cbSetDo;
    IOLM_PL_cbPRUgetDI_t        cbGetDi;
    IOLM_PL_cbPRUsetPower_t     cbSetPower;
    IOLM_PL_cbPRUSetIQ_t        cbSetIQ;
    IOLM_PL_cbPRUGetIQ_t        cbGetIQ;
    IOLM_PL_cbPRUSetIQMode_t    cbSetIQMode;
}IOLM_PL_sCallbacks_t;

void PRU_IOLM_registerSetModeCallback(IOLM_PL_cbPRUsetMode_t pCallback_p);
void PRU_IOLM_registerGetPortCfgCallback(IOLM_PL_cbPRUgetPortCfg_t pCallback_p);
void PRU_IOLM_registerGetPruCfgCallback(IOLM_PL_cbPRUgetPruCfg_t pCallback_p);
void PRU_IOLM_registerSetDoCallback(IOLM_PL_cbPRUsetDO_t pCallback_p);
void PRU_IOLM_registerGetDiCallback(IOLM_PL_cbPRUgetDI_t pCallback_p);
void PRU_IOLM_registerSetPowerCallback(IOLM_PL_cbPRUsetPower_t pCallback_p);
void PRU_IOLM_registerSetIQCallback(IOLM_PL_cbPRUSetIQ_t pCallback_p);
void PRU_IOLM_registerGetIQCallback(IOLM_PL_cbPRUGetIQ_t pCallback_p);
void PRU_IOLM_registerSetIQModeCallback(IOLM_PL_cbPRUSetIQMode_t pCallback_p);

void PRU_IOLM_init(uint8_t instance_p);
void PRU_IOLM_deInit(uint8_t instance_p);
void PRU_IOLM_initSitara(void);

#endif /* __PRU_IOLINK_H__ */
