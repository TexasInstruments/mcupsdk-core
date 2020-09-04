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
#ifndef CSLR_SHA_H_
#define CSLR_SHA_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <drivers/hw_include/cslr.h>

/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
/**
 *  The enumeration describes the list of supported algorithms which are
 *  supported by the EIP57T
 */
typedef enum CSL_EIP57T_SHAAlgo_e
{
    CSL_EIP57T_SHAAlgo_MD5      = 0x1,  /**< MD5    */
    CSL_EIP57T_SHAAlgo_SHA384,          /**< SHA384 */
    CSL_EIP57T_SHAAlgo_SHA1,            /**< SHA1   */
    CSL_EIP57T_SHAAlgo_SHA512,          /**< SHA512 */
    CSL_EIP57T_SHAAlgo_SHA224,          /**< SHA224 */
    CSL_EIP57T_SHAAlgo_SHA256           /**< SHA256 */
}CSL_EIP57T_SHAAlgo;

/**
 *  Structure type to access the EIP57T SHA Header file.
 */
typedef volatile struct CSL_EIP57T_SHARegs_t
{
    uint32_t ODIGEST[8U];           /**< 0x0000-0x001C:Secure World Outer Digest/HMAC Key Reg     */
    uint32_t IDIGEST[8U];           /**< 0x0020-0x003C:Secure World Inner Digest/HMAC Key Reg     */
    uint32_t DIGEST_COUNT;          /**< 0x0040:Secure World Digest Count Register                */
    uint32_t HASH_MODE;             /**< 0x0044:Secure World Mode Configuration Register          */
    uint32_t LENGTH;                /**< 0x0048:Secure World Block Length Register                */
    uint32_t RESERVED0[13U];        /**< 0x004C-0x007C:Reserved                                   */
    uint32_t DATA_IN[16U];          /**< 0x0080-0x00BC:Secure World Data Input Register           */
    uint32_t RESERVED1[16U];        /**< 0x00C0-0x00FC:Reserved                                   */
    uint32_t REVISION;              /**< 0x00100:Revision Register                                */
    uint32_t RESERVED2[3U];         /**< 0x0104-0x010C:Reserved                                   */
    uint32_t SYSCONFIG;             /**< 0x0110:System Configuration Register                     */
    uint32_t SYSSTATUS;             /**< 0x0114:System Status Register                            */
    uint32_t IRQSTATUS;             /**< 0x0118:Interrupt Request Register                        */
    uint32_t IRQENABLE;             /**< 0x0118:Interrupt Enable Register                         */
    uint32_t RESERVED3[56U];        /**< 0x011C-0x01FC:Reserved                                   */
    uint32_t HASH512_ODIGEST[16U];  /**< 0x0200-0x023C:SHA-384,512 Secure World Outer Digest Regs */
    uint32_t HASH512_IDIGEST[16U];  /**< 0x0240-0x027C:SHA-384,512 Secure World Inner Digest Regs */
    uint32_t HASH512_DIGEST_COUNT;  /**< 0x0280:SHA-384,512 Secure World Digest Count Reg         */
    uint32_t HASH512_MODE;          /**< 0x0284:SHA-384,512 Secure World Mode Config Reg          */
    uint32_t HASH512_LENGTH;        /**< 0x0288:SHA-384,512 Secure World Mode Block Length Reg    */
}CSL_EIP57T_SHARegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#ifdef __cplusplus
}
#endif
#endif
