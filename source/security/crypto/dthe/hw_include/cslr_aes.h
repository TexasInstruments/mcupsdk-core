/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *
*/
#ifndef CSLR_AES_H_
#define CSLR_AES_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>


/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
typedef struct {
    volatile uint32_t KEY2_6;
    volatile uint32_t KEY2_7;
    volatile uint32_t KEY2_4;
    volatile uint32_t KEY2_5;
    volatile uint32_t KEY2_2;
    volatile uint32_t KEY2_3;
    volatile uint32_t KEY2_0;
    volatile uint32_t KEY2_1;
    volatile uint32_t KEY1_6;
    volatile uint32_t KEY1_7;
    volatile uint32_t KEY1_4;
    volatile uint32_t KEY1_5;
    volatile uint32_t KEY1_2;
    volatile uint32_t KEY1_3;
    volatile uint32_t KEY1_0;
    volatile uint32_t KEY1_1;
    volatile uint32_t IV_IN_0;
    volatile uint32_t IV_IN_1;
    volatile uint32_t IV_IN_2;
    volatile uint32_t IV_IN_3;
    volatile uint32_t CTRL;
    volatile uint32_t C_LENGTH_0;
    volatile uint32_t C_LENGTH_1;
    volatile uint32_t AUTH_LENGTH;
    volatile uint32_t DATA_IN_0;
    volatile uint32_t DATA_IN_1;
    volatile uint32_t DATA_IN_2;
    volatile uint32_t DATA_IN_3;
    volatile uint32_t TAG_OUT_0;
    volatile uint32_t TAG_OUT_1;
    volatile uint32_t TAG_OUT_2;
    volatile uint32_t TAG_OUT_3;
    volatile uint32_t REVISION;
    volatile uint32_t SYSCONFIG;
    volatile uint32_t SYSSTS;
    volatile uint32_t IRQSTS;
    volatile uint32_t IRQEN;
    /* Only accessible for secure context*/
    volatile uint32_t DIRTYBITS;
    volatile uint32_t LOCKDOWN;
    volatile uint8_t  RSVD0[1048420];
} CSL_AesRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/* XTS second key / CBC-MAC third key */
#define CSL_AES_S_KEY2_6                                        (0x0U)

/* XTS second key (MSW for 256-bit key) / CBC-MAC third key (MSW) */
#define CSL_AES_S_KEY2_7                                        (0x4U)

/* XTS / CCM second key / CBC-MAC third key (LSW) */
#define CSL_AES_S_KEY2_4                                        (0x8U)

/* XTS second key (MSW for 192-bit key) / CBC-MAC third key */
#define CSL_AES_S_KEY2_5                                        (0xCU)

/* XTS / CCM / CBC-MAC second key / Hash Key input */
#define CSL_AES_S_KEY2_2                                        (0x10U)

/* XTS second key (MSW for 128-bit key) + CCM/CBC-MAC second key (MSW) / Hash
 * Key input (MSW) */
#define CSL_AES_S_KEY2_3                                        (0x14U)

/* XTS / CCM / CBC-MAC second key (LSW) / Hash Key input (LSW) */
#define CSL_AES_S_KEY2_0                                        (0x18U)

/* XTS / CCM / CBC-MAC second key / Hash Key input */
#define CSL_AES_S_KEY2_1                                        (0x1CU)

/* Key (LSW for 256-key) */
#define CSL_AES_S_KEY1_6                                        (0x20U)

/* Key (MSW for 256-bit key) */
#define CSL_AES_S_KEY1_7                                        (0x24U)

/* Key (LSW for 192-bit key) */
#define CSL_AES_S_KEY1_4                                        (0x28U)

/* Key (MSW for 192-bit key) */
#define CSL_AES_S_KEY1_5                                        (0x2CU)

/* Key */
#define CSL_AES_S_KEY1_2                                        (0x30U)

/* Key (MSW for 128-bit key) */
#define CSL_AES_S_KEY1_3                                        (0x34U)

/* Key (LSW for 128-bit key) */
#define CSL_AES_S_KEY1_0                                        (0x38U)

/* Key */
#define CSL_AES_S_KEY1_1                                        (0x3CU)

/* Initialization Vector input (LSW) */
#define CSL_AES_S_IV_IN_0                                       (0x40U)

/* Initialization vector input */
#define CSL_AES_S_IV_IN_1                                       (0x44U)

/* Initialization vector input */
#define CSL_AES_S_IV_IN_2                                       (0x48U)

/* Initialization Vector input (MSW) */
#define CSL_AES_S_IV_IN_3                                       (0x4CU)

/* register determines the mode of operation of the AES Engine */
#define CSL_AES_S_CTRL                                          (0x50U)

/* Crypto data length registers (LSW and MSW) store the cryptographic data
 * length in bytes for all modes. Once processing with this context is
 * started, this length decrements to zero. Data lengths up to (2^61 – 1)
 * bytes are allowed. For GCM, any value up to 2^36 - 32 bytes can be used.
 * This is because a 32-bit counter mode is used; the maximum number of
 * 128-bit blocks is 2^32 – 2, resulting in a maximum number of bytes of 2^36
 * - 32. A write to this register triggers the engine to start using this
 * context. This is valid for all modes except GCM and CCM. Note that for the
 * combined modes, this length does not include the authentication only data;
 * the authentication length is specified in the AES_AUTH_LENGTH register
 * below. All modes must have a length > 0. For the combined modes, it is
 * allowed to have one of the lengths equal to zero. For the basic encryption
 * modes (ECB/CBC/CTR/ICM/CFB128) it is allowed to program zero to the length
 * field; in that case the length is assumed infinite. All data must be byte
 * (8-bit) aligned; bit aligned data streams are not supported by the AES
 * Engine. For a Host read operation, these registers return all-zeroes. */
#define CSL_AES_S_C_LENGTH_0                                    (0x54U)

/* Crypto data length registers (LSW and MSW) store the cryptographic data
 * length in bytes for all modes. Once processing with this context is
 * started, this length decrements to zero. Data lengths up to (2^61 – 1)
 * bytes are allowed. For GCM, any value up to 2^36 - 32 bytes can be used.
 * This is because a 32-bit counter mode is used; the maximum number of
 * 128-bit blocks is 2^32 – 2, resulting in a maximum number of bytes of 2^36
 * - 32. A write to this register triggers the engine to start using this
 * context. This is valid for all modes except GCM and CCM. Note that for the
 * combined modes, this length does not include the authentication only data;
 * the authentication length is specified in the AES_AUTH_LENGTH register
 * below. All modes must have a length > 0. For the combined modes, it is
 * allowed to have one of the lengths equal to zero. For the basic encryption
 * modes (ECB/CBC/CTR/ICM/CFB128) it is allowed to program zero to the length
 * field; in that case the length is assumed infinite. All data must be byte
 * (8-bit) aligned; bit aligned data streams are not supported by the AES
 * Engine. For a Host read operation, these registers return all-zeroes. */
#define CSL_AES_S_C_LENGTH_1                                    (0x58U)

/* AAD data length. The authentication length register store the
 * authentication data length in bytes for combined modes only (GCM or CCM)
 * Supported AAD-lengths for CCM are from 0 to (2^16 - 2^8) bytes. For GCM any
 * value up to (2^32 - 1) bytes can be used. Once processing with this context
 * is started, this length decrements to zero. A write to this register
 * triggers the engine to start using this context for GCM and CCM. For XTS
 * this register is optionally used to load ‘j’. Loading of ‘j’ is only
 * required if ‘j’ != 0. ‘j’ is a 28-bit value and must be written to bits
 * [31-4] of this register. ‘j’ represents the sequential number of the
 * 128-bit block inside the data unit. For the first block in a unit, this
 * value is zero. It is not required to provide a ‘j’ for each new data block
 * within a unit. Note that it is possible to start with a ‘j’ unequal to
 * zero; refer to Table 4 for more details. For a Host read operation, these
 * registers return all-zeroes. */
#define CSL_AES_S_AUTH_LENGTH                                   (0x5CU)

/* Data register to read and write plaintext/ciphertext (MSW) */
#define CSL_AES_S_DATA_IN_0                                     (0x60U)

/* Data register to read and write plaintext/ciphertext */
#define CSL_AES_S_DATA_IN_1                                     (0x64U)

/* Data register to read and write plaintext/ciphertext */
#define CSL_AES_S_DATA_IN_2                                     (0x68U)

/* Data register to read and write plaintext/ciphertext (LSW) */
#define CSL_AES_S_DATA_IN_3                                     (0x6CU)

/* S_TAG_OUT_0 */
#define CSL_AES_S_TAG_OUT_0                                     (0x70U)

/* S_TAG_OUT_1 */
#define CSL_AES_S_TAG_OUT_1                                     (0x74U)

/* S_TAG_OUT_2 */
#define CSL_AES_S_TAG_OUT_2                                     (0x78U)

/* S_TAG_OUT_3 */
#define CSL_AES_S_TAG_OUT_3                                     (0x7CU)

/* Register AES_REVISION */
#define CSL_AES_S_REVISION                                      (0x80U)

/* Register AES_S_SYSCONFIG.This register configures the DMA signals and
 * controls the IDLE and reset logic */
#define CSL_AES_S_SYSCONFIG                                     (0x84U)

/* S_SYSSTS */
#define CSL_AES_S_SYSSTS                                        (0x88U)

/* This register indicates the interrupt status. If one of the interrupt bits
 * is set the interrupt output will be asserted */
#define CSL_AES_S_IRQSTS                                        (0x8CU)

/* This register contains an enable bit for each unique interrupt generated by
 * the module. It matches the layout of AES_IRQSTATUS register. An interrupt
 * is enabled when the bit in this register is set to ‘1’. An interrupt that
 * is enabled is propagated to the SINTREQUEST_x output. All interrupts need
 * to be enabled explicitly by writing this register. */
#define CSL_AES_S_IRQEN                                         (0x90U)

/* S_DIRTYBITS */
#define CSL_AES_S_DIRTYBITS                                     (0x94U)

/* S_LOCKDOWN */
#define CSL_AES_S_LOCKDOWN                                      (0x98U)

/* XTS second key / CBC-MAC third key */
#define CSL_AES_P_KEY2_6                                        (0x100000U)

/* XTS second key (MSW for 256-bit key) / CBC-MAC third key (MSW) */
#define CSL_AES_P_KEY2_7                                        (0x100004U)

/* XTS / CCM second key / CBC-MAC third key (LSW) */
#define CSL_AES_P_KEY2_4                                        (0x100008U)

/* XTS second key (MSW for 192-bit key) / CBC-MAC third key */
#define CSL_AES_P_KEY2_5                                        (0x10000CU)

/* XTS / CCM / CBC-MAC second key / Hash Key input */
#define CSL_AES_P_KEY2_2                                        (0x100010U)

/* XTS second key (MSW for 128-bit key) + CCM/CBC-MAC second key (MSW) / Hash
 * Key input (MSW) */
#define CSL_AES_P_KEY2_3                                        (0x100014U)

/* XTS / CCM / CBC-MAC second key (LSW) / Hash Key input (LSW) */
#define CSL_AES_P_KEY2_0                                        (0x100018U)

/* XTS / CCM / CBC-MAC second key / Hash Key input */
#define CSL_AES_P_KEY2_1                                        (0x10001CU)

/* Key (LSW for 256-bit key) */
#define CSL_AES_P_KEY1_6                                        (0x100020U)

/* Key (MSW for 256-bit key) */
#define CSL_AES_P_KEY1_7                                        (0x100024U)

/* Key (LSW for 192-bit key) */
#define CSL_AES_P_KEY1_4                                        (0x100028U)

/* Key (MSW for 192-bit key) */
#define CSL_AES_P_KEY1_5                                        (0x10002CU)

/* Key */
#define CSL_AES_P_KEY1_2                                        (0x100030U)

/* Key (MSW for 128-bit key) */
#define CSL_AES_P_KEY1_3                                        (0x100034U)

/* Key (LSW for 128-bit key) */
#define CSL_AES_P_KEY1_0                                        (0x100038U)

/* Key */
#define CSL_AES_P_KEY1_1                                        (0x10003CU)

/* Initialization Vector input (LSW) */
#define CSL_AES_P_IV_IN_0                                       (0x100040U)

/* Initialization vector input */
#define CSL_AES_P_IV_IN_1                                       (0x100044U)

/* Initialization vector input */
#define CSL_AES_P_IV_IN_2                                       (0x100048U)

/* Initialization Vector input (MSW) */
#define CSL_AES_P_IV_IN_3                                       (0x10004CU)

/* register determines the mode of operation of the AES Engine */
#define CSL_AES_P_CTRL                                          (0x100050U)

/* Crypto data length registers (LSW and MSW) store the cryptographic data
 * length in bytes for all modes. Once processing with this context is
 * started, this length decrements to zero. Data lengths up to (2^61 – 1)
 * bytes are allowed. For GCM, any value up to 2^36 - 32 bytes can be used.
 * This is because a 32-bit counter mode is used; the maximum number of
 * 128-bit blocks is 2^32 – 2, resulting in a maximum number of bytes of 2^36
 * - 32. A write to this register triggers the engine to start using this
 * context. This is valid for all modes except GCM and CCM. Note that for the
 * combined modes, this length does not include the authentication only data;
 * the authentication length is specified in the AES_AUTH_LENGTH register
 * below. All modes must have a length > 0. For the combined modes, it is
 * allowed to have one of the lengths equal to zero. For the basic encryption
 * modes (ECB/CBC/CTR/ICM/CFB128) it is allowed to program zero to the length
 * field; in that case the length is assumed infinite. All data must be byte
 * (8-bit) aligned; bit aligned data streams are not supported by the AES
 * Engine. For a Host read operation, these registers return all-zeroes. */
#define CSL_AES_P_C_LENGTH_0                                    (0x100054U)

/* Crypto data length registers (LSW and MSW) store the cryptographic data
 * length in bytes for all modes. Once processing with this context is
 * started, this length decrements to zero. Data lengths up to (2^61 – 1)
 * bytes are allowed. For GCM, any value up to 2^36 - 32 bytes can be used.
 * This is because a 32-bit counter mode is used; the maximum number of
 * 128-bit blocks is 2^32 – 2, resulting in a maximum number of bytes of 2^36
 * - 32. A write to this register triggers the engine to start using this
 * context. This is valid for all modes except GCM and CCM. Note that for the
 * combined modes, this length does not include the authentication only data;
 * the authentication length is specified in the AES_AUTH_LENGTH register
 * below. All modes must have a length > 0. For the combined modes, it is
 * allowed to have one of the lengths equal to zero. For the basic encryption
 * modes (ECB/CBC/CTR/ICM/CFB128) it is allowed to program zero to the length
 * field; in that case the length is assumed infinite. All data must be byte
 * (8-bit) aligned; bit aligned data streams are not supported by the AES
 * Engine. For a Host read operation, these registers return all-zeroes. */
#define CSL_AES_P_C_LENGTH_1                                    (0x100058U)

/* AAD data length. The authentication length register store the
 * authentication data length in bytes for combined modes only (GCM or CCM)
 * Supported AAD-lengths for CCM are from 0 to (2^16 - 2^8) bytes. For GCM any
 * value up to (2^32 - 1) bytes can be used. Once processing with this context
 * is started, this length decrements to zero. A write to this register
 * triggers the engine to start using this context for GCM and CCM. For XTS
 * this register is optionally used to load ‘j’. Loading of ‘j’ is only
 * required if ‘j’ != 0. ‘j’ is a 28-bit value and must be written to bits
 * [31-4] of this register. ‘j’ represents the sequential number of the
 * 128-bit block inside the data unit. For the first block in a unit, this
 * value is zero. It is not required to provide a ‘j’ for each new data block
 * within a unit. Note that it is possible to start with a ‘j’ unequal to
 * zero; refer to Table 4 for more details. For a Host read operation, these
 * registers return all-zeroes. */
#define CSL_AES_P_AUTH_LENGTH                                   (0x10005CU)

/* Data register to read and write plaintext/ciphertext (MSW) */
#define CSL_AES_P_DATA_IN_0                                     (0x100060U)

/* Data register to read and write plaintext/ciphertext */
#define CSL_AES_P_DATA_IN_1                                     (0x100064U)

/* Data register to read and write plaintext/ciphertext */
#define CSL_AES_P_DATA_IN_2                                     (0x100068U)

/* Data register to read and write plaintext/ciphertext (LSW) */
#define CSL_AES_P_DATA_IN_3                                     (0x10006CU)

/* P_TAG_OUT_0 */
#define CSL_AES_P_TAG_OUT_0                                     (0x100070U)

/* P_TAG_OUT_1 */
#define CSL_AES_P_TAG_OUT_1                                     (0x100074U)

/* P_TAG_OUT_2 */
#define CSL_AES_P_TAG_OUT_2                                     (0x100078U)

/* P_TAG_OUT_3 */
#define CSL_AES_P_TAG_OUT_3                                     (0x10007CU)

/* Register AES_REVISION */
#define CSL_AES_P_REVISION                                      (0x100080U)

/* Register AES_P_SYSCONFIG.This register configures the DMA signals. */
#define CSL_AES_P_SYSCONFIG                                     (0x100084U)

/* P_SYSSTS */
#define CSL_AES_P_SYSSTS                                        (0x100088U)

/* This register indicates the interrupt status. If one of the interrupt bits
 * is set the interrupt output will be asserted */
#define CSL_AES_P_IRQSTS                                        (0x10008CU)

/* This register contains an enable bit for each unique interrupt generated by
 * the module. It matches the layout of AES_IRQSTATUS register. An interrupt
 * is enabled when the bit in this register is set to ‘1’. An interrupt that
 * is enabled is propagated to the SINTREQUEST_x output. All interrupts need
 * to be enabled explicitly by writing this register. */
#define CSL_AES_P_IRQEN                                         (0x100090U)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* S_KEY2_6 */

#define CSL_AES_S_KEY2_6_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_6_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_6_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_6_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_6_RESETVAL                               (0x00000000U)

/* S_KEY2_7 */

#define CSL_AES_S_KEY2_7_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_7_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_7_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_7_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_7_RESETVAL                               (0x00000000U)

/* S_KEY2_4 */

#define CSL_AES_S_KEY2_4_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_4_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_4_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_4_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_4_RESETVAL                               (0x00000000U)

/* S_KEY2_5 */

#define CSL_AES_S_KEY2_5_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_5_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_5_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_5_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_5_RESETVAL                               (0x00000000U)

/* S_KEY2_2 */

#define CSL_AES_S_KEY2_2_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_2_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_2_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_2_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_2_RESETVAL                               (0x00000000U)

/* S_KEY2_3 */

#define CSL_AES_S_KEY2_3_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_3_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_3_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_3_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_3_RESETVAL                               (0x00000000U)

/* S_KEY2_0 */

#define CSL_AES_S_KEY2_0_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_0_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_0_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_0_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_0_RESETVAL                               (0x00000000U)

/* S_KEY2_1 */

#define CSL_AES_S_KEY2_1_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY2_1_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY2_1_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY2_1_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY2_1_RESETVAL                               (0x00000000U)

/* S_KEY1_6 */

#define CSL_AES_S_KEY1_6_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_6_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_6_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_6_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_6_RESETVAL                               (0x00000000U)

/* S_KEY1_7 */

#define CSL_AES_S_KEY1_7_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_7_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_7_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_7_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_7_RESETVAL                               (0x00000000U)

/* S_KEY1_4 */

#define CSL_AES_S_KEY1_4_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_4_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_4_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_4_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_4_RESETVAL                               (0x00000000U)

/* S_KEY1_5 */

#define CSL_AES_S_KEY1_5_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_5_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_5_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_5_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_5_RESETVAL                               (0x00000000U)

/* S_KEY1_2 */

#define CSL_AES_S_KEY1_2_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_2_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_2_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_2_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_2_RESETVAL                               (0x00000000U)

/* S_KEY1_3 */

#define CSL_AES_S_KEY1_3_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_3_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_3_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_3_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_3_RESETVAL                               (0x00000000U)

/* S_KEY1_0 */

#define CSL_AES_S_KEY1_0_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_0_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_0_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_0_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_0_RESETVAL                               (0x00000000U)

/* S_KEY1_1 */

#define CSL_AES_S_KEY1_1_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_S_KEY1_1_KEY_SHIFT                              (0U)
#define CSL_AES_S_KEY1_1_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_S_KEY1_1_KEY_MAX                                (0xffffffffU)

#define CSL_AES_S_KEY1_1_RESETVAL                               (0x00000000U)

/* S_IV_IN_0 */

#define CSL_AES_S_IV_IN_0_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_S_IV_IN_0_DATA_SHIFT                            (0U)
#define CSL_AES_S_IV_IN_0_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_S_IV_IN_0_DATA_MAX                              (0xffffffffU)

#define CSL_AES_S_IV_IN_0_RESETVAL                              (0x00000000U)

/* S_IV_IN_1 */

#define CSL_AES_S_IV_IN_1_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_S_IV_IN_1_DATA_SHIFT                            (0U)
#define CSL_AES_S_IV_IN_1_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_S_IV_IN_1_DATA_MAX                              (0xffffffffU)

#define CSL_AES_S_IV_IN_1_RESETVAL                              (0x00000000U)

/* S_IV_IN_2 */

#define CSL_AES_S_IV_IN_2_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_S_IV_IN_2_DATA_SHIFT                            (0U)
#define CSL_AES_S_IV_IN_2_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_S_IV_IN_2_DATA_MAX                              (0xffffffffU)

#define CSL_AES_S_IV_IN_2_RESETVAL                              (0x00000000U)

/* S_IV_IN_3 */

#define CSL_AES_S_IV_IN_3_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_S_IV_IN_3_DATA_SHIFT                            (0U)
#define CSL_AES_S_IV_IN_3_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_S_IV_IN_3_DATA_MAX                              (0xffffffffU)

#define CSL_AES_S_IV_IN_3_RESETVAL                              (0x00000000U)

/* S_CTRL */

#define CSL_AES_S_CTRL_OUTPUT_READY_MASK                        (0x00000001U)
#define CSL_AES_S_CTRL_OUTPUT_READY_SHIFT                       (0U)
#define CSL_AES_S_CTRL_OUTPUT_READY_RESETVAL                    (0x00000000U)
#define CSL_AES_S_CTRL_OUTPUT_READY_MAX                         (0x00000001U)

#define CSL_AES_S_CTRL_DIRECTION_MASK                           (0x00000004U)
#define CSL_AES_S_CTRL_DIRECTION_SHIFT                          (2U)
#define CSL_AES_S_CTRL_DIRECTION_RESETVAL                       (0x00000000U)
#define CSL_AES_S_CTRL_DIRECTION_DECRYPT                        (0x00000000U)
#define CSL_AES_S_CTRL_DIRECTION_ENCRYPT                        (0x00000001U)

#define CSL_AES_S_CTRL_INPUT_READY_MASK                         (0x00000002U)
#define CSL_AES_S_CTRL_INPUT_READY_SHIFT                        (1U)
#define CSL_AES_S_CTRL_INPUT_READY_RESETVAL                     (0x00000000U)
#define CSL_AES_S_CTRL_INPUT_READY_MAX                          (0x00000001U)

#define CSL_AES_S_CTRL_KEY_SIZE_MASK                            (0x00000018U)
#define CSL_AES_S_CTRL_KEY_SIZE_SHIFT                           (3U)
#define CSL_AES_S_CTRL_KEY_SIZE_RESETVAL                        (0x00000000U)
#define CSL_AES_S_CTRL_KEY_SIZE_RESERVED                        (0x00000000U)
#define CSL_AES_S_CTRL_KEY_SIZE_KEY128                          (0x00000001U)
#define CSL_AES_S_CTRL_KEY_SIZE_KEY192                          (0x00000002U)
#define CSL_AES_S_CTRL_KEY_SIZE_KEY256                          (0x00000003U)

#define CSL_AES_S_CTRL_MODE_MASK                                (0x00000020U)
#define CSL_AES_S_CTRL_MODE_SHIFT                               (5U)
#define CSL_AES_S_CTRL_MODE_RESETVAL                            (0x00000000U)
#define CSL_AES_S_CTRL_MODE_ECB                                 (0x00000000U)
#define CSL_AES_S_CTRL_MODE_CBC                                 (0x00000001U)

#define CSL_AES_S_CTRL_CTR_MASK                                 (0x00000040U)
#define CSL_AES_S_CTRL_CTR_SHIFT                                (6U)
#define CSL_AES_S_CTRL_CTR_RESETVAL                             (0x00000000U)
#define CSL_AES_S_CTRL_CTR_NOOP                                 (0x00000000U)
#define CSL_AES_S_CTRL_CTR_CTR                                  (0x00000001U)

#define CSL_AES_S_CTRL_CTR_WIDTH_MASK                           (0x00000180U)
#define CSL_AES_S_CTRL_CTR_WIDTH_SHIFT                          (7U)
#define CSL_AES_S_CTRL_CTR_WIDTH_RESETVAL                       (0x00000000U)
#define CSL_AES_S_CTRL_CTR_WIDTH_COUNTER32                      (0x00000000U)
#define CSL_AES_S_CTRL_CTR_WIDTH_COUNTER64                      (0x00000001U)
#define CSL_AES_S_CTRL_CTR_WIDTH_COUNTER96                      (0x00000002U)
#define CSL_AES_S_CTRL_CTR_WIDTH_COUNTER128                     (0x00000003U)

#define CSL_AES_S_CTRL_ICM_MASK                                 (0x00000200U)
#define CSL_AES_S_CTRL_ICM_SHIFT                                (9U)
#define CSL_AES_S_CTRL_ICM_RESETVAL                             (0x00000000U)
#define CSL_AES_S_CTRL_ICM_NO_ICM                               (0x00000000U)
#define CSL_AES_S_CTRL_ICM_ICM                                  (0x00000001U)

#define CSL_AES_S_CTRL_CFB_MASK                                 (0x00000400U)
#define CSL_AES_S_CTRL_CFB_SHIFT                                (10U)
#define CSL_AES_S_CTRL_CFB_RESETVAL                             (0x00000000U)
#define CSL_AES_S_CTRL_CFB_NO_CFB                               (0x00000000U)
#define CSL_AES_S_CTRL_CFB_CFB                                  (0x00000001U)

#define CSL_AES_S_CTRL_XTS_MASK                                 (0x00001800U)
#define CSL_AES_S_CTRL_XTS_SHIFT                                (11U)
#define CSL_AES_S_CTRL_XTS_RESETVAL                             (0x00000000U)
#define CSL_AES_S_CTRL_XTS_NOOP                                 (0x00000000U)
#define CSL_AES_S_CTRL_XTS_XTS01                                (0x00000001U)
#define CSL_AES_S_CTRL_XTS_XTS10                                (0x00000002U)
#define CSL_AES_S_CTRL_XTS_XTS11                                (0x00000003U)

#define CSL_AES_S_CTRL_F8_MASK                                  (0x00002000U)
#define CSL_AES_S_CTRL_F8_SHIFT                                 (13U)
#define CSL_AES_S_CTRL_F8_RESETVAL                              (0x00000000U)
#define CSL_AES_S_CTRL_F8_NO_F8                                 (0x00000000U)
#define CSL_AES_S_CTRL_F8_F8                                    (0x00000001U)

#define CSL_AES_S_CTRL_F9_MASK                                  (0x00004000U)
#define CSL_AES_S_CTRL_F9_SHIFT                                 (14U)
#define CSL_AES_S_CTRL_F9_RESETVAL                              (0x00000000U)
#define CSL_AES_S_CTRL_F9_NO_F9                                 (0x00000000U)
#define CSL_AES_S_CTRL_F9_F9                                    (0x00000001U)

#define CSL_AES_S_CTRL_CBCMAC_MASK                              (0x00008000U)
#define CSL_AES_S_CTRL_CBCMAC_SHIFT                             (15U)
#define CSL_AES_S_CTRL_CBCMAC_RESETVAL                          (0x00000000U)
#define CSL_AES_S_CTRL_CBCMAC_NO_CBCMAC                         (0x00000000U)
#define CSL_AES_S_CTRL_CBCMAC_CBCMAC                            (0x00000001U)

#define CSL_AES_S_CTRL_GCM_MASK                                 (0x00030000U)
#define CSL_AES_S_CTRL_GCM_SHIFT                                (16U)
#define CSL_AES_S_CTRL_GCM_RESETVAL                             (0x00000000U)
#define CSL_AES_S_CTRL_GCM_NOOP                                 (0x00000000U)
#define CSL_AES_S_CTRL_GCM_GCM01                                (0x00000001U)
#define CSL_AES_S_CTRL_GCM_GCMA10                               (0x00000002U)
#define CSL_AES_S_CTRL_GCM_GCM11                                (0x00000003U)

#define CSL_AES_S_CTRL_CCM_MASK                                 (0x00040000U)
#define CSL_AES_S_CTRL_CCM_SHIFT                                (18U)
#define CSL_AES_S_CTRL_CCM_RESETVAL                             (0x00000000U)
#define CSL_AES_S_CTRL_CCM_NO_CCM                               (0x00000000U)
#define CSL_AES_S_CTRL_CCM_CCM                                  (0x00000001U)

#define CSL_AES_S_CTRL_CCM_L_MASK                               (0x00380000U)
#define CSL_AES_S_CTRL_CCM_L_SHIFT                              (19U)
#define CSL_AES_S_CTRL_CCM_L_RESETVAL                           (0x00000000U)
#define CSL_AES_S_CTRL_CCM_L_MAX                                (0x00000007U)

#define CSL_AES_S_CTRL_CCM_M_MASK                               (0x01C00000U)
#define CSL_AES_S_CTRL_CCM_M_SHIFT                              (22U)
#define CSL_AES_S_CTRL_CCM_M_RESETVAL                           (0x00000000U)
#define CSL_AES_S_CTRL_CCM_M_MAX                                (0x00000007U)

#define CSL_AES_S_CTRL_SAVE_CONTEXT_MASK                        (0x20000000U)
#define CSL_AES_S_CTRL_SAVE_CONTEXT_SHIFT                       (29U)
#define CSL_AES_S_CTRL_SAVE_CONTEXT_RESETVAL                    (0x00000000U)
#define CSL_AES_S_CTRL_SAVE_CONTEXT_MAX                         (0x00000001U)

#define CSL_AES_S_CTRL_SAVE_CONTEXT_READY_MASK                  (0x40000000U)
#define CSL_AES_S_CTRL_SAVE_CONTEXT_READY_SHIFT                 (30U)
#define CSL_AES_S_CTRL_SAVE_CONTEXT_READY_RESETVAL              (0x00000000U)
#define CSL_AES_S_CTRL_SAVE_CONTEXT_READY_MAX                   (0x00000001U)

#define CSL_AES_S_CTRL_CONTEXT_READY_MASK                       (0x80000000U)
#define CSL_AES_S_CTRL_CONTEXT_READY_SHIFT                      (31U)
#define CSL_AES_S_CTRL_CONTEXT_READY_RESETVAL                   (0x00000001U)
#define CSL_AES_S_CTRL_CONTEXT_READY_MAX                        (0x00000001U)

#define CSL_AES_S_CTRL_RESETVAL                                 (0x80000000U)

/* S_C_LENGTH_0 */

#define CSL_AES_S_C_LENGTH_0_LENGTH_MASK                        (0xFFFFFFFFU)
#define CSL_AES_S_C_LENGTH_0_LENGTH_SHIFT                       (0U)
#define CSL_AES_S_C_LENGTH_0_LENGTH_RESETVAL                    (0x00000000U)
#define CSL_AES_S_C_LENGTH_0_LENGTH_MAX                         (0xffffffffU)

#define CSL_AES_S_C_LENGTH_0_RESETVAL                           (0x00000000U)

/* S_C_LENGTH_1 */

#define CSL_AES_S_C_LENGTH_1_LENGTH_MASK                        (0x1FFFFFFFU)
#define CSL_AES_S_C_LENGTH_1_LENGTH_SHIFT                       (0U)
#define CSL_AES_S_C_LENGTH_1_LENGTH_RESETVAL                    (0x00000000U)
#define CSL_AES_S_C_LENGTH_1_LENGTH_MAX                         (0x1fffffffU)

#define CSL_AES_S_C_LENGTH_1_RESETVAL                           (0x00000000U)

/* S_AUTH_LENGTH */

#define CSL_AES_S_AUTH_LENGTH_AUTH_MASK                         (0xFFFFFFFFU)
#define CSL_AES_S_AUTH_LENGTH_AUTH_SHIFT                        (0U)
#define CSL_AES_S_AUTH_LENGTH_AUTH_RESETVAL                     (0x00000000U)
#define CSL_AES_S_AUTH_LENGTH_AUTH_MAX                          (0xffffffffU)

#define CSL_AES_S_AUTH_LENGTH_RESETVAL                          (0x00000000U)

/* S_DATA_IN_0 */

#define CSL_AES_S_DATA_IN_0_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_DATA_IN_0_DATA_SHIFT                          (0U)
#define CSL_AES_S_DATA_IN_0_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_S_DATA_IN_0_DATA_MAX                            (0xffffffffU)

#define CSL_AES_S_DATA_IN_0_RESETVAL                            (0x00000000U)

/* S_DATA_IN_1 */

#define CSL_AES_S_DATA_IN_1_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_DATA_IN_1_DATA_SHIFT                          (0U)
#define CSL_AES_S_DATA_IN_1_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_S_DATA_IN_1_DATA_MAX                            (0xffffffffU)

#define CSL_AES_S_DATA_IN_1_RESETVAL                            (0x00000000U)

/* S_DATA_IN_2 */

#define CSL_AES_S_DATA_IN_2_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_DATA_IN_2_DATA_SHIFT                          (0U)
#define CSL_AES_S_DATA_IN_2_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_S_DATA_IN_2_DATA_MAX                            (0xffffffffU)

#define CSL_AES_S_DATA_IN_2_RESETVAL                            (0x00000000U)

/* S_DATA_IN_3 */

#define CSL_AES_S_DATA_IN_3_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_DATA_IN_3_DATA_SHIFT                          (0U)
#define CSL_AES_S_DATA_IN_3_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_S_DATA_IN_3_DATA_MAX                            (0xffffffffU)

#define CSL_AES_S_DATA_IN_3_RESETVAL                            (0x00000000U)

/* S_TAG_OUT_0 */

#define CSL_AES_S_TAG_OUT_0_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_TAG_OUT_0_HASH_SHIFT                          (0U)
#define CSL_AES_S_TAG_OUT_0_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_S_TAG_OUT_0_HASH_MAX                            (0xffffffffU)

#define CSL_AES_S_TAG_OUT_0_RESETVAL                            (0x00000000U)

/* S_TAG_OUT_1 */

#define CSL_AES_S_TAG_OUT_1_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_TAG_OUT_1_HASH_SHIFT                          (0U)
#define CSL_AES_S_TAG_OUT_1_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_S_TAG_OUT_1_HASH_MAX                            (0xffffffffU)

#define CSL_AES_S_TAG_OUT_1_RESETVAL                            (0x00000000U)

/* S_TAG_OUT_2 */

#define CSL_AES_S_TAG_OUT_2_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_TAG_OUT_2_HASH_SHIFT                          (0U)
#define CSL_AES_S_TAG_OUT_2_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_S_TAG_OUT_2_HASH_MAX                            (0xffffffffU)

#define CSL_AES_S_TAG_OUT_2_RESETVAL                            (0x00000000U)

/* S_TAG_OUT_3 */

#define CSL_AES_S_TAG_OUT_3_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_S_TAG_OUT_3_HASH_SHIFT                          (0U)
#define CSL_AES_S_TAG_OUT_3_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_S_TAG_OUT_3_HASH_MAX                            (0xffffffffU)

#define CSL_AES_S_TAG_OUT_3_RESETVAL                            (0x00000000U)

/* S_REVISION */

#define CSL_AES_S_REVISION_Y_MINOR_MASK                         (0x0000003FU)
#define CSL_AES_S_REVISION_Y_MINOR_SHIFT                        (0U)
#define CSL_AES_S_REVISION_Y_MINOR_RESETVAL                     (0x00000000U)
#define CSL_AES_S_REVISION_Y_MINOR_MAX                          (0x0000003fU)

#define CSL_AES_S_REVISION_CUSTOM_MASK                          (0x000000C0U)
#define CSL_AES_S_REVISION_CUSTOM_SHIFT                         (6U)
#define CSL_AES_S_REVISION_CUSTOM_RESETVAL                      (0x00000000U)
#define CSL_AES_S_REVISION_CUSTOM_STANDARD                      (0x00000000U)

#define CSL_AES_S_REVISION_X_MAJOR_MASK                         (0x00000700U)
#define CSL_AES_S_REVISION_X_MAJOR_SHIFT                        (8U)
#define CSL_AES_S_REVISION_X_MAJOR_RESETVAL                     (0x00000000U)
#define CSL_AES_S_REVISION_X_MAJOR_MAX                          (0x00000007U)

#define CSL_AES_S_REVISION_R_RTL_MASK                           (0x0000F800U)
#define CSL_AES_S_REVISION_R_RTL_SHIFT                          (11U)
#define CSL_AES_S_REVISION_R_RTL_RESETVAL                       (0x00000000U)
#define CSL_AES_S_REVISION_R_RTL_MAX                            (0x0000001fU)

#define CSL_AES_S_REVISION_FUNC_MASK                            (0x0FFF0000U)
#define CSL_AES_S_REVISION_FUNC_SHIFT                           (16U)
#define CSL_AES_S_REVISION_FUNC_RESETVAL                        (0x00000000U)
#define CSL_AES_S_REVISION_FUNC_MAX                             (0x00000fffU)

#define CSL_AES_S_REVISION_SCHEME_MASK                          (0xC0000000U)
#define CSL_AES_S_REVISION_SCHEME_SHIFT                         (30U)
#define CSL_AES_S_REVISION_SCHEME_RESETVAL                      (0x00000000U)
#define CSL_AES_S_REVISION_SCHEME_H08                           (0x00000001U)
#define CSL_AES_S_REVISION_SCHEME_LEGACY                        (0x00000000U)

#define CSL_AES_S_REVISION_RESETVAL                             (0x00000000U)

/* S_SYSCONFIG */

#define CSL_AES_S_SYSCONFIG_AUTOIDLE_MASK                       (0x00000001U)
#define CSL_AES_S_SYSCONFIG_AUTOIDLE_SHIFT                      (0U)
#define CSL_AES_S_SYSCONFIG_AUTOIDLE_RESETVAL                   (0x00000001U)
#define CSL_AES_S_SYSCONFIG_AUTOIDLE_CLOCKS_ON                  (0x00000000U)
#define CSL_AES_S_SYSCONFIG_AUTOIDLE_CLOCKS_OFF                 (0x00000001U)

#define CSL_AES_S_SYSCONFIG_SOFTRESET_MASK                      (0x00000002U)
#define CSL_AES_S_SYSCONFIG_SOFTRESET_SHIFT                     (1U)
#define CSL_AES_S_SYSCONFIG_SOFTRESET_RESETVAL                  (0x00000000U)
#define CSL_AES_S_SYSCONFIG_SOFTRESET_NOOP                      (0x00000000U)
#define CSL_AES_S_SYSCONFIG_SOFTRESET_SOFRESET                  (0x00000001U)

#define CSL_AES_S_SYSCONFIG_SIDLE_MASK                          (0x0000000CU)
#define CSL_AES_S_SYSCONFIG_SIDLE_SHIFT                         (2U)
#define CSL_AES_S_SYSCONFIG_SIDLE_RESETVAL                      (0x00000000U)
#define CSL_AES_S_SYSCONFIG_SIDLE_FORCEIDLE                     (0x00000000U)
#define CSL_AES_S_SYSCONFIG_SIDLE_NOIDLE                        (0x00000001U)
#define CSL_AES_S_SYSCONFIG_SIDLE_SMARTIDLE                     (0x00000002U)
#define CSL_AES_S_SYSCONFIG_SIDLE_RESERVED                      (0x00000003U)

#define CSL_AES_S_SYSCONFIG_DIRECTBUSEN_MASK                    (0x00000010U)
#define CSL_AES_S_SYSCONFIG_DIRECTBUSEN_SHIFT                   (4U)
#define CSL_AES_S_SYSCONFIG_DIRECTBUSEN_RESETVAL                (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DIRECTBUSEN_KEY                     (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DIRECTBUSEN_DIRECT                  (0x00000001U)

#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN_MASK             (0x00000020U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN_SHIFT            (5U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN_RESETVAL         (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN_DMA_DIS          (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_IN_EN_DMA_EN           (0x00000001U)

#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN_MASK            (0x00000040U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN_SHIFT           (6U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN_RESETVAL        (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN_DMA_DIS         (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_DATA_OUT_EN_DMA_EN          (0x00000001U)

#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_MASK          (0x00000080U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_SHIFT         (7U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_RESETVAL      (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_DMA_DIS       (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_DMA_EN        (0x00000001U)

#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_MASK         (0x00000100U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_SHIFT        (8U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_RESETVAL     (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_DMA_DIS      (0x00000000U)
#define CSL_AES_S_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_DMA_EN       (0x00000001U)

#define CSL_AES_S_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_MASK        (0x00000200U)
#define CSL_AES_S_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_SHIFT       (9U)
#define CSL_AES_S_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_RESETVAL    (0x00000000U)
#define CSL_AES_S_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_MAX         (0x00000001U)

#define CSL_AES_S_SYSCONFIG_KEK_MODE_MASK                       (0x00000400U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_SHIFT                      (10U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_RESETVAL                   (0x00000000U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_MAX                        (0x00000001U)

#define CSL_AES_S_SYSCONFIG_KEY_ENC_MASK                        (0x00000800U)
#define CSL_AES_S_SYSCONFIG_KEY_ENC_SHIFT                       (11U)
#define CSL_AES_S_SYSCONFIG_KEY_ENC_RESETVAL                    (0x00000000U)
#define CSL_AES_S_SYSCONFIG_KEY_ENC_MAX                         (0x00000001U)

#define CSL_AES_S_SYSCONFIG_K3_MASK                             (0x00001000U)
#define CSL_AES_S_SYSCONFIG_K3_SHIFT                            (12U)
#define CSL_AES_S_SYSCONFIG_K3_RESETVAL                         (0x00000000U)
#define CSL_AES_S_SYSCONFIG_K3_MAX                              (0x00000001U)

#define CSL_AES_S_SYSCONFIG_KEK_MODE_ID_MASK                    (0x00004000U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_ID_SHIFT                   (14U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_ID_RESETVAL                (0x00000000U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_ID_MAX                     (0x00000001U)

#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_MASK                   (0x00018000U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_SHIFT                  (15U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_RESETVAL               (0x00000000U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_RESERVED               (0x00000000U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_KEY128                 (0x00000001U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_KEY192                 (0x00000002U)
#define CSL_AES_S_SYSCONFIG_KEK_MODE_LEN_KEY256                 (0x00000003U)

#define CSL_AES_S_SYSCONFIG_RESETVAL                            (0x00000001U)

/* S_SYSSTS */

#define CSL_AES_S_SYSSTS_RESETDONE_MASK                         (0x00000001U)
#define CSL_AES_S_SYSSTS_RESETDONE_SHIFT                        (0U)
#define CSL_AES_S_SYSSTS_RESETDONE_RESETVAL                     (0x00000000U)
#define CSL_AES_S_SYSSTS_RESETDONE_MAX                          (0x00000001U)

#define CSL_AES_S_SYSSTS_RESETVAL                               (0x00000000U)

/* S_IRQSTS */

#define CSL_AES_S_IRQSTS_CONTEX_IN_MASK                         (0x00000001U)
#define CSL_AES_S_IRQSTS_CONTEX_IN_SHIFT                        (0U)
#define CSL_AES_S_IRQSTS_CONTEX_IN_RESETVAL                     (0x00000000U)
#define CSL_AES_S_IRQSTS_CONTEX_IN_MAX                          (0x00000001U)

#define CSL_AES_S_IRQSTS_DATA_IN_MASK                           (0x00000002U)
#define CSL_AES_S_IRQSTS_DATA_IN_SHIFT                          (1U)
#define CSL_AES_S_IRQSTS_DATA_IN_RESETVAL                       (0x00000000U)
#define CSL_AES_S_IRQSTS_DATA_IN_MAX                            (0x00000001U)

#define CSL_AES_S_IRQSTS_DATA_OUT_MASK                          (0x00000004U)
#define CSL_AES_S_IRQSTS_DATA_OUT_SHIFT                         (2U)
#define CSL_AES_S_IRQSTS_DATA_OUT_RESETVAL                      (0x00000000U)
#define CSL_AES_S_IRQSTS_DATA_OUT_MAX                           (0x00000001U)

#define CSL_AES_S_IRQSTS_CONTEXT_OUT_MASK                       (0x00000008U)
#define CSL_AES_S_IRQSTS_CONTEXT_OUT_SHIFT                      (3U)
#define CSL_AES_S_IRQSTS_CONTEXT_OUT_RESETVAL                   (0x00000000U)
#define CSL_AES_S_IRQSTS_CONTEXT_OUT_MAX                        (0x00000001U)

#define CSL_AES_S_IRQSTS_RESETVAL                               (0x00000000U)

/* S_IRQEN */

#define CSL_AES_S_IRQEN_CONTEX_IN_MASK                          (0x00000001U)
#define CSL_AES_S_IRQEN_CONTEX_IN_SHIFT                         (0U)
#define CSL_AES_S_IRQEN_CONTEX_IN_RESETVAL                      (0x00000000U)
#define CSL_AES_S_IRQEN_CONTEX_IN_MAX                           (0x00000001U)

#define CSL_AES_S_IRQEN_DATA_IN_MASK                            (0x00000002U)
#define CSL_AES_S_IRQEN_DATA_IN_SHIFT                           (1U)
#define CSL_AES_S_IRQEN_DATA_IN_RESETVAL                        (0x00000000U)
#define CSL_AES_S_IRQEN_DATA_IN_MAX                             (0x00000001U)

#define CSL_AES_S_IRQEN_DATA_OUT_MASK                           (0x00000004U)
#define CSL_AES_S_IRQEN_DATA_OUT_SHIFT                          (2U)
#define CSL_AES_S_IRQEN_DATA_OUT_RESETVAL                       (0x00000000U)
#define CSL_AES_S_IRQEN_DATA_OUT_MAX                            (0x00000001U)

#define CSL_AES_S_IRQEN_CONTEXT_OUT_MASK                        (0x00000008U)
#define CSL_AES_S_IRQEN_CONTEXT_OUT_SHIFT                       (3U)
#define CSL_AES_S_IRQEN_CONTEXT_OUT_RESETVAL                    (0x00000000U)
#define CSL_AES_S_IRQEN_CONTEXT_OUT_MAX                         (0x00000001U)

#define CSL_AES_S_IRQEN_RESETVAL                                (0x00000000U)

/* S_DIRTYBITS */

#define CSL_AES_S_DIRTYBITS_S_ACCESS_MASK                       (0x00000001U)
#define CSL_AES_S_DIRTYBITS_S_ACCESS_SHIFT                      (0U)
#define CSL_AES_S_DIRTYBITS_S_ACCESS_RESETVAL                   (0x00000000U)
#define CSL_AES_S_DIRTYBITS_S_ACCESS_MAX                        (0x00000001U)

#define CSL_AES_S_DIRTYBITS_S_DIRTY_MASK                        (0x00000002U)
#define CSL_AES_S_DIRTYBITS_S_DIRTY_SHIFT                       (1U)
#define CSL_AES_S_DIRTYBITS_S_DIRTY_RESETVAL                    (0x00000000U)
#define CSL_AES_S_DIRTYBITS_S_DIRTY_MAX                         (0x00000001U)

#define CSL_AES_S_DIRTYBITS_P_ACCESS_MASK                       (0x00000004U)
#define CSL_AES_S_DIRTYBITS_P_ACCESS_SHIFT                      (2U)
#define CSL_AES_S_DIRTYBITS_P_ACCESS_RESETVAL                   (0x00000000U)
#define CSL_AES_S_DIRTYBITS_P_ACCESS_MAX                        (0x00000001U)

#define CSL_AES_S_DIRTYBITS_P_DIRTY_MASK                        (0x00000008U)
#define CSL_AES_S_DIRTYBITS_P_DIRTY_SHIFT                       (3U)
#define CSL_AES_S_DIRTYBITS_P_DIRTY_RESETVAL                    (0x00000000U)
#define CSL_AES_S_DIRTYBITS_P_DIRTY_MAX                         (0x00000001U)

#define CSL_AES_S_DIRTYBITS_RESETVAL                            (0x00000000U)

/* S_LOCKDOWN */

#define CSL_AES_S_LOCKDOWN_KEY_LOCK_MASK                        (0x00000001U)
#define CSL_AES_S_LOCKDOWN_KEY_LOCK_SHIFT                       (0U)
#define CSL_AES_S_LOCKDOWN_KEY_LOCK_RESETVAL                    (0x00000000U)
#define CSL_AES_S_LOCKDOWN_KEY_LOCK_MAX                         (0x00000001U)

#define CSL_AES_S_LOCKDOWN_IV_LOCK_MASK                         (0x00000008U)
#define CSL_AES_S_LOCKDOWN_IV_LOCK_SHIFT                        (3U)
#define CSL_AES_S_LOCKDOWN_IV_LOCK_RESETVAL                     (0x00000000U)
#define CSL_AES_S_LOCKDOWN_IV_LOCK_MAX                          (0x00000001U)

#define CSL_AES_S_LOCKDOWN_CTRL_LOCK_MASK                       (0x00000010U)
#define CSL_AES_S_LOCKDOWN_CTRL_LOCK_SHIFT                      (4U)
#define CSL_AES_S_LOCKDOWN_CTRL_LOCK_RESETVAL                   (0x00000000U)
#define CSL_AES_S_LOCKDOWN_CTRL_LOCK_MAX                        (0x00000001U)

#define CSL_AES_S_LOCKDOWN_LENGTH_LOCK_MASK                     (0x00000020U)
#define CSL_AES_S_LOCKDOWN_LENGTH_LOCK_SHIFT                    (5U)
#define CSL_AES_S_LOCKDOWN_LENGTH_LOCK_RESETVAL                 (0x00000000U)
#define CSL_AES_S_LOCKDOWN_LENGTH_LOCK_MAX                      (0x00000001U)

#define CSL_AES_S_LOCKDOWN_KEY2_LOCK_MASK                       (0x00000002U)
#define CSL_AES_S_LOCKDOWN_KEY2_LOCK_SHIFT                      (1U)
#define CSL_AES_S_LOCKDOWN_KEY2_LOCK_RESETVAL                   (0x00000000U)
#define CSL_AES_S_LOCKDOWN_KEY2_LOCK_MAX                        (0x00000001U)

#define CSL_AES_S_LOCKDOWN_KEY3_LOCK_MASK                       (0x00000004U)
#define CSL_AES_S_LOCKDOWN_KEY3_LOCK_SHIFT                      (2U)
#define CSL_AES_S_LOCKDOWN_KEY3_LOCK_RESETVAL                   (0x00000000U)
#define CSL_AES_S_LOCKDOWN_KEY3_LOCK_MAX                        (0x00000001U)

#define CSL_AES_S_LOCKDOWN_RESETVAL                             (0x00000000U)

/* P_KEY2_6 */

#define CSL_AES_P_KEY2_6_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_6_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_6_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_6_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_6_RESETVAL                               (0x00000000U)

/* P_KEY2_7 */

#define CSL_AES_P_KEY2_7_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_7_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_7_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_7_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_7_RESETVAL                               (0x00000000U)

/* P_KEY2_4 */

#define CSL_AES_P_KEY2_4_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_4_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_4_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_4_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_4_RESETVAL                               (0x00000000U)

/* P_KEY2_5 */

#define CSL_AES_P_KEY2_5_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_5_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_5_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_5_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_5_RESETVAL                               (0x00000000U)

/* P_KEY2_2 */

#define CSL_AES_P_KEY2_2_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_2_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_2_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_2_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_2_RESETVAL                               (0x00000000U)

/* P_KEY2_3 */

#define CSL_AES_P_KEY2_3_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_3_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_3_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_3_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_3_RESETVAL                               (0x00000000U)

/* P_KEY2_0 */

#define CSL_AES_P_KEY2_0_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_0_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_0_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_0_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_0_RESETVAL                               (0x00000000U)

/* P_KEY2_1 */

#define CSL_AES_P_KEY2_1_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY2_1_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY2_1_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY2_1_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY2_1_RESETVAL                               (0x00000000U)

/* P_KEY1_6 */

#define CSL_AES_P_KEY1_6_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_6_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_6_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_6_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_6_RESETVAL                               (0x00000000U)

/* P_KEY1_7 */

#define CSL_AES_P_KEY1_7_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_7_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_7_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_7_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_7_RESETVAL                               (0x00000000U)

/* P_KEY1_4 */

#define CSL_AES_P_KEY1_4_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_4_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_4_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_4_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_4_RESETVAL                               (0x00000000U)

/* P_KEY1_5 */

#define CSL_AES_P_KEY1_5_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_5_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_5_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_5_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_5_RESETVAL                               (0x00000000U)

/* P_KEY1_2 */

#define CSL_AES_P_KEY1_2_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_2_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_2_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_2_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_2_RESETVAL                               (0x00000000U)

/* P_KEY1_3 */

#define CSL_AES_P_KEY1_3_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_3_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_3_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_3_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_3_RESETVAL                               (0x00000000U)

/* P_KEY1_0 */

#define CSL_AES_P_KEY1_0_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_0_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_0_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_0_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_0_RESETVAL                               (0x00000000U)

/* P_KEY1_1 */

#define CSL_AES_P_KEY1_1_KEY_MASK                               (0xFFFFFFFFU)
#define CSL_AES_P_KEY1_1_KEY_SHIFT                              (0U)
#define CSL_AES_P_KEY1_1_KEY_RESETVAL                           (0x00000000U)
#define CSL_AES_P_KEY1_1_KEY_MAX                                (0xffffffffU)

#define CSL_AES_P_KEY1_1_RESETVAL                               (0x00000000U)

/* P_IV_IN_0 */

#define CSL_AES_P_IV_IN_0_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_P_IV_IN_0_DATA_SHIFT                            (0U)
#define CSL_AES_P_IV_IN_0_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_P_IV_IN_0_DATA_MAX                              (0xffffffffU)

#define CSL_AES_P_IV_IN_0_RESETVAL                              (0x00000000U)

/* P_IV_IN_1 */

#define CSL_AES_P_IV_IN_1_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_P_IV_IN_1_DATA_SHIFT                            (0U)
#define CSL_AES_P_IV_IN_1_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_P_IV_IN_1_DATA_MAX                              (0xffffffffU)

#define CSL_AES_P_IV_IN_1_RESETVAL                              (0x00000000U)

/* P_IV_IN_2 */

#define CSL_AES_P_IV_IN_2_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_P_IV_IN_2_DATA_SHIFT                            (0U)
#define CSL_AES_P_IV_IN_2_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_P_IV_IN_2_DATA_MAX                              (0xffffffffU)

#define CSL_AES_P_IV_IN_2_RESETVAL                              (0x00000000U)

/* P_IV_IN_3 */

#define CSL_AES_P_IV_IN_3_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_AES_P_IV_IN_3_DATA_SHIFT                            (0U)
#define CSL_AES_P_IV_IN_3_DATA_RESETVAL                         (0x00000000U)
#define CSL_AES_P_IV_IN_3_DATA_MAX                              (0xffffffffU)

#define CSL_AES_P_IV_IN_3_RESETVAL                              (0x00000000U)

/* P_CTRL */

#define CSL_AES_P_CTRL_OUTPUT_READY_MASK                        (0x00000001U)
#define CSL_AES_P_CTRL_OUTPUT_READY_SHIFT                       (0U)
#define CSL_AES_P_CTRL_OUTPUT_READY_RESETVAL                    (0x00000000U)
#define CSL_AES_P_CTRL_OUTPUT_READY_MAX                         (0x00000001U)

#define CSL_AES_P_CTRL_DIRECTION_MASK                           (0x00000004U)
#define CSL_AES_P_CTRL_DIRECTION_SHIFT                          (2U)
#define CSL_AES_P_CTRL_DIRECTION_RESETVAL                       (0x00000000U)
#define CSL_AES_P_CTRL_DIRECTION_DECRYPT                        (0x00000000U)
#define CSL_AES_P_CTRL_DIRECTION_ENCRYPT                        (0x00000001U)

#define CSL_AES_P_CTRL_INPUT_READY_MASK                         (0x00000002U)
#define CSL_AES_P_CTRL_INPUT_READY_SHIFT                        (1U)
#define CSL_AES_P_CTRL_INPUT_READY_RESETVAL                     (0x00000000U)
#define CSL_AES_P_CTRL_INPUT_READY_MAX                          (0x00000001U)

#define CSL_AES_P_CTRL_KEY_SIZE_MASK                            (0x00000018U)
#define CSL_AES_P_CTRL_KEY_SIZE_SHIFT                           (3U)
#define CSL_AES_P_CTRL_KEY_SIZE_RESETVAL                        (0x00000000U)
#define CSL_AES_P_CTRL_KEY_SIZE_RESERVED                        (0x00000000U)
#define CSL_AES_P_CTRL_KEY_SIZE_KEY128                          (0x00000001U)
#define CSL_AES_P_CTRL_KEY_SIZE_KEY192                          (0x00000002U)
#define CSL_AES_P_CTRL_KEY_SIZE_KEY256                          (0x00000003U)

#define CSL_AES_P_CTRL_MODE_MASK                                (0x00000020U)
#define CSL_AES_P_CTRL_MODE_SHIFT                               (5U)
#define CSL_AES_P_CTRL_MODE_RESETVAL                            (0x00000000U)
#define CSL_AES_P_CTRL_MODE_ECB                                 (0x00000000U)
#define CSL_AES_P_CTRL_MODE_CBC                                 (0x00000001U)

#define CSL_AES_P_CTRL_CTR_MASK                                 (0x00000040U)
#define CSL_AES_P_CTRL_CTR_SHIFT                                (6U)
#define CSL_AES_P_CTRL_CTR_RESETVAL                             (0x00000000U)
#define CSL_AES_P_CTRL_CTR_NOOP                                 (0x00000000U)
#define CSL_AES_P_CTRL_CTR_CTR                                  (0x00000001U)

#define CSL_AES_P_CTRL_CTR_WIDTH_MASK                           (0x00000180U)
#define CSL_AES_P_CTRL_CTR_WIDTH_SHIFT                          (7U)
#define CSL_AES_P_CTRL_CTR_WIDTH_RESETVAL                       (0x00000000U)
#define CSL_AES_P_CTRL_CTR_WIDTH_COUNTER32                      (0x00000000U)
#define CSL_AES_P_CTRL_CTR_WIDTH_COUNTER64                      (0x00000001U)
#define CSL_AES_P_CTRL_CTR_WIDTH_COUNTER96                      (0x00000002U)
#define CSL_AES_P_CTRL_CTR_WIDTH_COUNTER128                     (0x00000003U)

#define CSL_AES_P_CTRL_ICM_MASK                                 (0x00000200U)
#define CSL_AES_P_CTRL_ICM_SHIFT                                (9U)
#define CSL_AES_P_CTRL_ICM_RESETVAL                             (0x00000000U)
#define CSL_AES_P_CTRL_ICM_NO_ICM                               (0x00000000U)
#define CSL_AES_P_CTRL_ICM_ICM                                  (0x00000001U)

#define CSL_AES_P_CTRL_CFB_MASK                                 (0x00000400U)
#define CSL_AES_P_CTRL_CFB_SHIFT                                (10U)
#define CSL_AES_P_CTRL_CFB_RESETVAL                             (0x00000000U)
#define CSL_AES_P_CTRL_CFB_NO_CFB                               (0x00000000U)
#define CSL_AES_P_CTRL_CFB_CFB                                  (0x00000001U)

#define CSL_AES_P_CTRL_XTS_MASK                                 (0x00001800U)
#define CSL_AES_P_CTRL_XTS_SHIFT                                (11U)
#define CSL_AES_P_CTRL_XTS_RESETVAL                             (0x00000000U)
#define CSL_AES_P_CTRL_XTS_NOOP                                 (0x00000000U)
#define CSL_AES_P_CTRL_XTS_XTS01                                (0x00000001U)
#define CSL_AES_P_CTRL_XTS_XTS10                                (0x00000002U)
#define CSL_AES_P_CTRL_XTS_XTS11                                (0x00000003U)

#define CSL_AES_P_CTRL_F8_MASK                                  (0x00002000U)
#define CSL_AES_P_CTRL_F8_SHIFT                                 (13U)
#define CSL_AES_P_CTRL_F8_RESETVAL                              (0x00000000U)
#define CSL_AES_P_CTRL_F8_NO_F8                                 (0x00000000U)
#define CSL_AES_P_CTRL_F8_F8                                    (0x00000001U)

#define CSL_AES_P_CTRL_F9_MASK                                  (0x00004000U)
#define CSL_AES_P_CTRL_F9_SHIFT                                 (14U)
#define CSL_AES_P_CTRL_F9_RESETVAL                              (0x00000000U)
#define CSL_AES_P_CTRL_F9_NO_F9                                 (0x00000000U)
#define CSL_AES_P_CTRL_F9_F9                                    (0x00000001U)

#define CSL_AES_P_CTRL_CBCMAC_MASK                              (0x00008000U)
#define CSL_AES_P_CTRL_CBCMAC_SHIFT                             (15U)
#define CSL_AES_P_CTRL_CBCMAC_RESETVAL                          (0x00000000U)
#define CSL_AES_P_CTRL_CBCMAC_NO_CBCMAC                         (0x00000000U)
#define CSL_AES_P_CTRL_CBCMAC_CBCMAC                            (0x00000001U)

#define CSL_AES_P_CTRL_GCM_MASK                                 (0x00030000U)
#define CSL_AES_P_CTRL_GCM_SHIFT                                (16U)
#define CSL_AES_P_CTRL_GCM_RESETVAL                             (0x00000000U)
#define CSL_AES_P_CTRL_GCM_NOOP                                 (0x00000000U)
#define CSL_AES_P_CTRL_GCM_GCM01                                (0x00000001U)
#define CSL_AES_P_CTRL_GCM_GCMA10                               (0x00000002U)
#define CSL_AES_P_CTRL_GCM_GCM11                                (0x00000003U)

#define CSL_AES_P_CTRL_CCM_MASK                                 (0x00040000U)
#define CSL_AES_P_CTRL_CCM_SHIFT                                (18U)
#define CSL_AES_P_CTRL_CCM_RESETVAL                             (0x00000000U)
#define CSL_AES_P_CTRL_CCM_NO_CCM                               (0x00000000U)
#define CSL_AES_P_CTRL_CCM_CCM                                  (0x00000001U)

#define CSL_AES_P_CTRL_CCM_L_MASK                               (0x00380000U)
#define CSL_AES_P_CTRL_CCM_L_SHIFT                              (19U)
#define CSL_AES_P_CTRL_CCM_L_RESETVAL                           (0x00000000U)
#define CSL_AES_P_CTRL_CCM_L_MAX                                (0x00000007U)

#define CSL_AES_P_CTRL_CCM_M_MASK                               (0x01C00000U)
#define CSL_AES_P_CTRL_CCM_M_SHIFT                              (22U)
#define CSL_AES_P_CTRL_CCM_M_RESETVAL                           (0x00000000U)
#define CSL_AES_P_CTRL_CCM_M_MAX                                (0x00000007U)

#define CSL_AES_P_CTRL_SAVE_CONTEXT_MASK                        (0x20000000U)
#define CSL_AES_P_CTRL_SAVE_CONTEXT_SHIFT                       (29U)
#define CSL_AES_P_CTRL_SAVE_CONTEXT_RESETVAL                    (0x00000000U)
#define CSL_AES_P_CTRL_SAVE_CONTEXT_MAX                         (0x00000001U)

#define CSL_AES_P_CTRL_SAVE_CONTEXT_READY_MASK                  (0x40000000U)
#define CSL_AES_P_CTRL_SAVE_CONTEXT_READY_SHIFT                 (30U)
#define CSL_AES_P_CTRL_SAVE_CONTEXT_READY_RESETVAL              (0x00000000U)
#define CSL_AES_P_CTRL_SAVE_CONTEXT_READY_MAX                   (0x00000001U)

#define CSL_AES_P_CTRL_CONTEXT_READY_MASK                       (0x80000000U)
#define CSL_AES_P_CTRL_CONTEXT_READY_SHIFT                      (31U)
#define CSL_AES_P_CTRL_CONTEXT_READY_RESETVAL                   (0x00000001U)
#define CSL_AES_P_CTRL_CONTEXT_READY_MAX                        (0x00000001U)

#define CSL_AES_P_CTRL_RESETVAL                                 (0x80000000U)

/* P_C_LENGTH_0 */

#define CSL_AES_P_C_LENGTH_0_LENGTH_MASK                        (0xFFFFFFFFU)
#define CSL_AES_P_C_LENGTH_0_LENGTH_SHIFT                       (0U)
#define CSL_AES_P_C_LENGTH_0_LENGTH_RESETVAL                    (0x00000000U)
#define CSL_AES_P_C_LENGTH_0_LENGTH_MAX                         (0xffffffffU)

#define CSL_AES_P_C_LENGTH_0_RESETVAL                           (0x00000000U)

/* P_C_LENGTH_1 */

#define CSL_AES_P_C_LENGTH_1_LENGTH_MASK                        (0x1FFFFFFFU)
#define CSL_AES_P_C_LENGTH_1_LENGTH_SHIFT                       (0U)
#define CSL_AES_P_C_LENGTH_1_LENGTH_RESETVAL                    (0x00000000U)
#define CSL_AES_P_C_LENGTH_1_LENGTH_MAX                         (0x1fffffffU)

#define CSL_AES_P_C_LENGTH_1_RESETVAL                           (0x00000000U)

/* P_AUTH_LENGTH */

#define CSL_AES_P_AUTH_LENGTH_AUTH_MASK                         (0xFFFFFFFFU)
#define CSL_AES_P_AUTH_LENGTH_AUTH_SHIFT                        (0U)
#define CSL_AES_P_AUTH_LENGTH_AUTH_RESETVAL                     (0x00000000U)
#define CSL_AES_P_AUTH_LENGTH_AUTH_MAX                          (0xffffffffU)

#define CSL_AES_P_AUTH_LENGTH_RESETVAL                          (0x00000000U)

/* P_DATA_IN_0 */

#define CSL_AES_P_DATA_IN_0_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_DATA_IN_0_DATA_SHIFT                          (0U)
#define CSL_AES_P_DATA_IN_0_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_P_DATA_IN_0_DATA_MAX                            (0xffffffffU)

#define CSL_AES_P_DATA_IN_0_RESETVAL                            (0x00000000U)

/* P_DATA_IN_1 */

#define CSL_AES_P_DATA_IN_1_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_DATA_IN_1_DATA_SHIFT                          (0U)
#define CSL_AES_P_DATA_IN_1_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_P_DATA_IN_1_DATA_MAX                            (0xffffffffU)

#define CSL_AES_P_DATA_IN_1_RESETVAL                            (0x00000000U)

/* P_DATA_IN_2 */

#define CSL_AES_P_DATA_IN_2_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_DATA_IN_2_DATA_SHIFT                          (0U)
#define CSL_AES_P_DATA_IN_2_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_P_DATA_IN_2_DATA_MAX                            (0xffffffffU)

#define CSL_AES_P_DATA_IN_2_RESETVAL                            (0x00000000U)

/* P_DATA_IN_3 */

#define CSL_AES_P_DATA_IN_3_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_DATA_IN_3_DATA_SHIFT                          (0U)
#define CSL_AES_P_DATA_IN_3_DATA_RESETVAL                       (0x00000000U)
#define CSL_AES_P_DATA_IN_3_DATA_MAX                            (0xffffffffU)

#define CSL_AES_P_DATA_IN_3_RESETVAL                            (0x00000000U)

/* P_TAG_OUT_0 */

#define CSL_AES_P_TAG_OUT_0_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_TAG_OUT_0_HASH_SHIFT                          (0U)
#define CSL_AES_P_TAG_OUT_0_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_P_TAG_OUT_0_HASH_MAX                            (0xffffffffU)

#define CSL_AES_P_TAG_OUT_0_RESETVAL                            (0x00000000U)

/* P_TAG_OUT_1 */

#define CSL_AES_P_TAG_OUT_1_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_TAG_OUT_1_HASH_SHIFT                          (0U)
#define CSL_AES_P_TAG_OUT_1_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_P_TAG_OUT_1_HASH_MAX                            (0xffffffffU)

#define CSL_AES_P_TAG_OUT_1_RESETVAL                            (0x00000000U)

/* P_TAG_OUT_2 */

#define CSL_AES_P_TAG_OUT_2_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_TAG_OUT_2_HASH_SHIFT                          (0U)
#define CSL_AES_P_TAG_OUT_2_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_P_TAG_OUT_2_HASH_MAX                            (0xffffffffU)

#define CSL_AES_P_TAG_OUT_2_RESETVAL                            (0x00000000U)

/* P_TAG_OUT_3 */

#define CSL_AES_P_TAG_OUT_3_HASH_MASK                           (0xFFFFFFFFU)
#define CSL_AES_P_TAG_OUT_3_HASH_SHIFT                          (0U)
#define CSL_AES_P_TAG_OUT_3_HASH_RESETVAL                       (0x00000000U)
#define CSL_AES_P_TAG_OUT_3_HASH_MAX                            (0xffffffffU)

#define CSL_AES_P_TAG_OUT_3_RESETVAL                            (0x00000000U)

/* P_REVISION */

#define CSL_AES_P_REVISION_Y_MINOR_MASK                         (0x0000003FU)
#define CSL_AES_P_REVISION_Y_MINOR_SHIFT                        (0U)
#define CSL_AES_P_REVISION_Y_MINOR_RESETVAL                     (0x00000000U)
#define CSL_AES_P_REVISION_Y_MINOR_MAX                          (0x0000003fU)

#define CSL_AES_P_REVISION_CUSTOM_MASK                          (0x000000C0U)
#define CSL_AES_P_REVISION_CUSTOM_SHIFT                         (6U)
#define CSL_AES_P_REVISION_CUSTOM_RESETVAL                      (0x00000000U)
#define CSL_AES_P_REVISION_CUSTOM_STANDARD                      (0x00000000U)

#define CSL_AES_P_REVISION_X_MAJOR_MASK                         (0x00000700U)
#define CSL_AES_P_REVISION_X_MAJOR_SHIFT                        (8U)
#define CSL_AES_P_REVISION_X_MAJOR_RESETVAL                     (0x00000000U)
#define CSL_AES_P_REVISION_X_MAJOR_MAX                          (0x00000007U)

#define CSL_AES_P_REVISION_R_RTL_MASK                           (0x0000F800U)
#define CSL_AES_P_REVISION_R_RTL_SHIFT                          (11U)
#define CSL_AES_P_REVISION_R_RTL_RESETVAL                       (0x00000000U)
#define CSL_AES_P_REVISION_R_RTL_MAX                            (0x0000001fU)

#define CSL_AES_P_REVISION_FUNC_MASK                            (0x0FFF0000U)
#define CSL_AES_P_REVISION_FUNC_SHIFT                           (16U)
#define CSL_AES_P_REVISION_FUNC_RESETVAL                        (0x00000000U)
#define CSL_AES_P_REVISION_FUNC_MAX                             (0x00000fffU)

#define CSL_AES_P_REVISION_SCHEME_MASK                          (0xC0000000U)
#define CSL_AES_P_REVISION_SCHEME_SHIFT                         (30U)
#define CSL_AES_P_REVISION_SCHEME_RESETVAL                      (0x00000000U)
#define CSL_AES_P_REVISION_SCHEME_H08                           (0x00000001U)
#define CSL_AES_P_REVISION_SCHEME_LEGACY                        (0x00000000U)

#define CSL_AES_P_REVISION_RESETVAL                             (0x00000000U)

/* P_SYSCONFIG */

#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_IN_EN_MASK             (0x00000020U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_IN_EN_SHIFT            (5U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_IN_EN_RESETVAL         (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_IN_EN_DMA_DIS          (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_IN_EN_DMA_EN           (0x00000001U)

#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_OUT_EN_MASK            (0x00000040U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_OUT_EN_SHIFT           (6U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_OUT_EN_RESETVAL        (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_OUT_EN_DMA_DIS         (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_DATA_OUT_EN_DMA_EN          (0x00000001U)

#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_MASK          (0x00000080U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_SHIFT         (7U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_RESETVAL      (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_DMA_DIS       (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_IN_EN_DMA_EN        (0x00000001U)

#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_MASK         (0x00000100U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_SHIFT        (8U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_RESETVAL     (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_DMA_DIS      (0x00000000U)
#define CSL_AES_P_SYSCONFIG_DMA_REQ_CONTEXT_OUT_EN_DMA_EN       (0x00000001U)

#define CSL_AES_P_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_MASK        (0x00000200U)
#define CSL_AES_P_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_SHIFT       (9U)
#define CSL_AES_P_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_RESETVAL    (0x00000000U)
#define CSL_AES_P_SYSCONFIG_MAP_CONTEXT_OUT_ON_DATA_MAX         (0x00000001U)

#define CSL_AES_P_SYSCONFIG_RESETVAL                            (0x00000000U)

/* P_SYSSTS */

#define CSL_AES_P_SYSSTS_RESETDONE_MASK                         (0x00000001U)
#define CSL_AES_P_SYSSTS_RESETDONE_SHIFT                        (0U)
#define CSL_AES_P_SYSSTS_RESETDONE_RESETVAL                     (0x00000000U)
#define CSL_AES_P_SYSSTS_RESETDONE_MAX                          (0x00000001U)

#define CSL_AES_P_SYSSTS_RESETVAL                               (0x00000000U)

/* P_IRQSTS */

#define CSL_AES_P_IRQSTS_CONTEX_IN_MASK                         (0x00000001U)
#define CSL_AES_P_IRQSTS_CONTEX_IN_SHIFT                        (0U)
#define CSL_AES_P_IRQSTS_CONTEX_IN_RESETVAL                     (0x00000000U)
#define CSL_AES_P_IRQSTS_CONTEX_IN_MAX                          (0x00000001U)

#define CSL_AES_P_IRQSTS_DATA_IN_MASK                           (0x00000002U)
#define CSL_AES_P_IRQSTS_DATA_IN_SHIFT                          (1U)
#define CSL_AES_P_IRQSTS_DATA_IN_RESETVAL                       (0x00000000U)
#define CSL_AES_P_IRQSTS_DATA_IN_MAX                            (0x00000001U)

#define CSL_AES_P_IRQSTS_DATA_OUT_MASK                          (0x00000004U)
#define CSL_AES_P_IRQSTS_DATA_OUT_SHIFT                         (2U)
#define CSL_AES_P_IRQSTS_DATA_OUT_RESETVAL                      (0x00000000U)
#define CSL_AES_P_IRQSTS_DATA_OUT_MAX                           (0x00000001U)

#define CSL_AES_P_IRQSTS_CONTEXT_OUT_MASK                       (0x00000008U)
#define CSL_AES_P_IRQSTS_CONTEXT_OUT_SHIFT                      (3U)
#define CSL_AES_P_IRQSTS_CONTEXT_OUT_RESETVAL                   (0x00000000U)
#define CSL_AES_P_IRQSTS_CONTEXT_OUT_MAX                        (0x00000001U)

#define CSL_AES_P_IRQSTS_RESETVAL                               (0x00000000U)

/* P_IRQEN */

#define CSL_AES_P_IRQEN_CONTEX_IN_MASK                          (0x00000001U)
#define CSL_AES_P_IRQEN_CONTEX_IN_SHIFT                         (0U)
#define CSL_AES_P_IRQEN_CONTEX_IN_RESETVAL                      (0x00000000U)
#define CSL_AES_P_IRQEN_CONTEX_IN_MAX                           (0x00000001U)

#define CSL_AES_P_IRQEN_DATA_IN_MASK                            (0x00000002U)
#define CSL_AES_P_IRQEN_DATA_IN_SHIFT                           (1U)
#define CSL_AES_P_IRQEN_DATA_IN_RESETVAL                        (0x00000000U)
#define CSL_AES_P_IRQEN_DATA_IN_MAX                             (0x00000001U)

#define CSL_AES_P_IRQEN_DATA_OUT_MASK                           (0x00000004U)
#define CSL_AES_P_IRQEN_DATA_OUT_SHIFT                          (2U)
#define CSL_AES_P_IRQEN_DATA_OUT_RESETVAL                       (0x00000000U)
#define CSL_AES_P_IRQEN_DATA_OUT_MAX                            (0x00000001U)

#define CSL_AES_P_IRQEN_CONTEXT_OUT_MASK                        (0x00000008U)
#define CSL_AES_P_IRQEN_CONTEXT_OUT_SHIFT                       (3U)
#define CSL_AES_P_IRQEN_CONTEXT_OUT_RESETVAL                    (0x00000000U)
#define CSL_AES_P_IRQEN_CONTEXT_OUT_MAX                         (0x00000001U)

#define CSL_AES_P_IRQEN_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
