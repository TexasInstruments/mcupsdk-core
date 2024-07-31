/********************************************************************
 * *
 * * HSM M4 memory map header file
 * *
 * * Copyright (C) 2022 Texas Instruments Incorporated.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *    Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * *
 * *    Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the
 * *    distribution.
 * *
 * *    Neither the name of Texas Instruments Incorporated nor the names of
 * *    its contributors may be used to endorse or promote products derived
 * *    from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * *
 * */
#ifndef CSLR_SOC_HSM_M4_BASEADDRESS_H_
#define CSLR_SOC_HSM_M4_BASEADDRESS_H_

#ifdef __cplusplus

extern "C"
{
#endif

/* Global addresses in unified address space */   
#define CSL_HSM_M4_ROM_BASE                 (0x00000000ul)        
#define CSL_HSM_M4_ROM_SIZE                 (0x00010000ul)

#define CSL_HSM_M4_SEC_ROM_BASE             (0x00010000ul) 
#define CSL_HSM_M4_SEC_ROM_SIZE             (0x00010000ul)

#define CSL_HSM_M4_RAM_BASE                 (0x00020000ul) 
#define CSL_HSM_M4_RAM_SIZE                 (0x00030000ul)

#define CSL_EXT_FLASH_SIZE                  (0x07FFFFFCU)

#define CSL_CM4_ICFG_BASE                   (0xE0000000ul) 



#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_R5_BASEADDRESS_H_ */

