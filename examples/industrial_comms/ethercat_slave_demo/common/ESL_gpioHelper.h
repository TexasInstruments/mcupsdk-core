/*
 *  Copyright (c) 2020, Kunbus GmbH
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

#if !(defined __ESL_GPIOHELPER_H__)
#define __ESL_GPIOHELPER_H__		1

#include <osal.h>

typedef enum ESL_GPIO_EModule
{
    ESL_GPIO_enMODULE_0,
    ESL_GPIO_enMODULE_1,
    ESL_GPIO_enMODULE_2,
    ESL_GPIO_enMODULE_3,
    ESL_GPIO_enMODULE_4,
    ESL_GPIO_enMODULE_5,
    ESL_GPIO_enMODULE_6,
    ESL_GPIO_enMODULE_7,
    ESL_GPIO_enMODULE_IOEXP,
    ESL_GPIO_enMODULE_LED,
    ESL_GPIO_enMODULE_MAX,
    ESL_GPIO_enMODULE_UNDEFINED = 0xFF
} ESL_GPIO_EModule_t;

typedef enum ESL_GPIO_EPin
{
    ESL_GPIO_enPIN_00,
    ESL_GPIO_enPIN_01,
    ESL_GPIO_enPIN_02,
    ESL_GPIO_enPIN_03,
    ESL_GPIO_enPIN_04,
    ESL_GPIO_enPIN_05,
    ESL_GPIO_enPIN_06,
    ESL_GPIO_enPIN_07,
    ESL_GPIO_enPIN_08,
    ESL_GPIO_enPIN_09,
    ESL_GPIO_enPIN_10,
    ESL_GPIO_enPIN_11,
    ESL_GPIO_enPIN_12,
    ESL_GPIO_enPIN_13,
    ESL_GPIO_enPIN_14,
    ESL_GPIO_enPIN_15,
    ESL_GPIO_enPIN_16,
    ESL_GPIO_enPIN_17,
    ESL_GPIO_enPIN_18,
    ESL_GPIO_enPIN_19,
    ESL_GPIO_enPIN_20,
    ESL_GPIO_enPIN_21,
    ESL_GPIO_enPIN_22,
    ESL_GPIO_enPIN_23,
    ESL_GPIO_enPIN_24,
    ESL_GPIO_enPIN_25,
    ESL_GPIO_enPIN_26,
    ESL_GPIO_enPIN_27,
    ESL_GPIO_enPIN_28,
    ESL_GPIO_enPIN_29,
    ESL_GPIO_enPIN_30,
    ESL_GPIO_enPIN_31,
    ESL_GPIO_enPIN_32,
    ESL_GPIO_enPIN_33,
    ESL_GPIO_enPIN_34,
    ESL_GPIO_enPIN_35,
    ESL_GPIO_enPIN_36,
    ESL_GPIO_enPIN_37,
    ESL_GPIO_enPIN_38,
    ESL_GPIO_enPIN_39,
    ESL_GPIO_enPIN_40,
    ESL_GPIO_enPIN_41,
    ESL_GPIO_enPIN_42,
    ESL_GPIO_enPIN_43,
    ESL_GPIO_enPIN_44,
    ESL_GPIO_enPIN_45,
    ESL_GPIO_enPIN_46,
    ESL_GPIO_enPIN_47,
    ESL_GPIO_enPIN_48,
    ESL_GPIO_enPIN_49,
    ESL_GPIO_enPIN_50,
    ESL_GPIO_enPIN_51,
    ESL_GPIO_enPIN_52,
    ESL_GPIO_enPIN_53,
    ESL_GPIO_enPIN_54,
    ESL_GPIO_enPIN_55,
    ESL_GPIO_enPIN_56,
    ESL_GPIO_enPIN_57,
    ESL_GPIO_enPIN_58,
    ESL_GPIO_enPIN_59,
    ESL_GPIO_enPIN_60,
    ESL_GPIO_enPIN_61,
    ESL_GPIO_enPIN_62,
    ESL_GPIO_enPIN_63,
    ESL_GPIO_enPIN_64,
    ESL_GPIO_enPIN_65,
    ESL_GPIO_enPIN_66,
    ESL_GPIO_enPIN_67,
    ESL_GPIO_enPIN_68,
    ESL_GPIO_enPIN_69,
    ESL_GPIO_enPIN_70,
    ESL_GPIO_enPIN_71,
    ESL_GPIO_enPIN_72,
    ESL_GPIO_enPIN_73,
    ESL_GPIO_enPIN_74,
    ESL_GPIO_enPIN_75,
    ESL_GPIO_enPIN_76,
    ESL_GPIO_enPIN_77,
    ESL_GPIO_enPIN_78,
    ESL_GPIO_enPIN_79,
    ESL_GPIO_enPIN_80,
    ESL_GPIO_enPIN_81,
    ESL_GPIO_enPIN_82,
    ESL_GPIO_enPIN_83,
    ESL_GPIO_enPIN_84,
    ESL_GPIO_enPIN_85,
    ESL_GPIO_enPIN_86,
    ESL_GPIO_enPIN_87,
    ESL_GPIO_enPIN_88,
    ESL_GPIO_enPIN_89,
    ESL_GPIO_enPIN_90,
    ESL_GPIO_enPIN_91,
    ESL_GPIO_enPIN_92,
    ESL_GPIO_enPIN_93,
    ESL_GPIO_enPIN_94,
    ESL_GPIO_enPIN_95,
    ESL_GPIO_enPIN_96,
    ESL_GPIO_enPIN_97,
    ESL_GPIO_enPIN_98,
    ESL_GPIO_enPIN_99,
    ESL_GPIO_enPIN_MAX,
    ESL_GPIO_enPIN_UNDEFINED = 0xFF
} ESL_GPIO_EPin_t;

typedef enum ESL_GPIO_EPinState
{
    ESL_GPIO_enPINSTATE_LOW,
    ESL_GPIO_enPINSTATE_HIGH
} ESL_GPIO_EPinState_t;

typedef enum ESL_GPIO_EDirectionMode
{
    ESL_GPIO_enDIRECTION_MODE_OUTPUT,
    ESL_GPIO_enDIRECTION_MODE_INPUT
} ESL_GPIO_EDirectionMode_t;

typedef enum ESL_GPIO_EInterruptMode
{
    ESL_GPIO_enIRQ_MODE_NONE,
    ESL_GPIO_enIRQ_MODE_RISING_EDGE,
    ESL_GPIO_enIRQ_MODE_FALLING_EDGE,
    ESL_GPIO_enIRQ_MODE_BOTH_EDGES,
    ESL_GPIO_enIRQ_MODE_LOW,
    ESL_GPIO_enIRQ_MODE_HIGH,
} ESL_GPIO_EInterruptMode_t;

/* Test pins logical indexes */
typedef enum ESL_GPIO_eTestPinNames {
    ESL_TESTPIN__FIRST,
    ESL_TESTPIN__0 = ESL_TESTPIN__FIRST,
    ESL_TESTPIN__1,
    ESL_TESTPIN__2,
    ESL_TESTPIN__3,
    ESL_TESTPIN__LAST = ESL_TESTPIN__3,
    ESL_TESTPIN__COUNT
} ESL_GPIO_eTestPinNames_t;

typedef void (*ESL_GPIO_CBIrq_t)(void);

#if (defined __cplusplus)
extern "C" {
#endif

extern void*    ESL_GPIO_init           (void);
extern bool     ESL_GPIO_setConfigMode  (void*                      pGpioHandle_p
                                        ,ESL_GPIO_EModule_t         moduleId_p
                                        ,ESL_GPIO_EPin_t            pinId_p
                                        ,ESL_GPIO_EDirectionMode_t  directionMode_p
                                        ,ESL_GPIO_EInterruptMode_t  intMode_p);
extern void     ESL_GPIO_apply          (void*                      pGpioHandle_p);
extern bool     ESL_GPIO_read           (void*                      pGpioHandle_p
                                        ,ESL_GPIO_EModule_t         moduleId_p
                                        ,ESL_GPIO_EPin_t            pinId_p);
extern bool     ESL_GPIO_write          (void*                      pGpioHandle_p
                                        ,ESL_GPIO_EModule_t         moduleId_p
                                        ,ESL_GPIO_EPin_t            pinId_p
                                        ,ESL_GPIO_EPinState_t       pinState_p);
extern uint32_t ESL_GPIO_getEntryIndex  (void*                      pGpioHandle_p
                                        ,ESL_GPIO_EModule_t         moduleId_p
                                        ,ESL_GPIO_EPin_t            pinId_p);
extern bool     ESL_GPIO_setCallback    (void*                      pGpioHandle_p
                                        ,ESL_GPIO_EModule_t         moduleId_p
                                        ,ESL_GPIO_EPin_t            pinId_p
                                        ,ESL_GPIO_CBIrq_t           callback_p);
extern bool     ESL_GPIO_enableIrq      (void*                      pGpioHandle_p
                                        ,ESL_GPIO_EModule_t         moduleId_p
                                        ,ESL_GPIO_EPin_t            pinId_p
                                        ,bool                       enable_p);

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
extern void     ESL_GPIO_testPins_setHigh
                                        (const
                                         ESL_GPIO_eTestPinNames_t   testpin_p);
extern void     ESL_GPIO_testPins_setLow(const
                                         ESL_GPIO_eTestPinNames_t   testpin_p);
extern void     ESL_GPIO_testPins_write (volatile
                                         CSL_GpioBank_registersRegs*
                                         const                      pGpioRegBank_p
                                        ,const uint32_t             pinMask_p);
extern void     ESL_GPIO_testPins_set   (volatile
                                         CSL_GpioBank_registersRegs*
                                         const                      pGpioRegBank_p
                                        ,const uint32_t             pinMask_p);
extern void     ESL_GPIO_testPins_clear (volatile
                                         CSL_GpioBank_registersRegs*
                                         const                      pGpioRegBank_p
                                        ,const uint32_t             pinMask_p);
#endif

#if (defined __cplusplus)
}
#endif

#endif /* __ESL_GPIOHELPER_H__ */
