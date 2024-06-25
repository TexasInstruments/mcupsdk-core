/******************************************************************************
*
*  (C) Copyright 2022 Texas Instruments, Inc.
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
******************************************************************************
*  @file ecc_test_utils_asm.asm
*
*  @brief
*   Implementation file for the ARM R5 utils
*
*   Contains the different control command and status query functions definitions
*
*   \par
******************************************************************************/
    .text


/****************************************************************************
* ECC_Test Read ATCM Region Register
*****************************************************************************/
	.section   ".text"
	.global ECC_Test_UTILS_TCMEnable

ECC_Test_UTILS_TCMEnable:
	
    //Enable ATCM @0x0
    MRC     p15, #0, r0, c9, c1, #1
    BFC     r0, #12, #20
    ORR     r0, r0, #0x1
    MCR     p15, #0, r0, c9, c1, #1

    //Enable BTCM @0x41010000
    LDR     r1, ecc_test_btcm_base
    MRC     p15, #0, r0, c9, c1, #0
    BFC     r0, #12, #20
    ORR     r0, r0, r1
    ORR     r0, r0, #0x1
    MCR     p15, #0, r0, c9, c1, #0

    BX      lr

ecc_test_btcm_base: .word 0x41010000

	
