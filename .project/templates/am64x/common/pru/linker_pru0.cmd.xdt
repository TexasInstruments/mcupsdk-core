/*
 * AM64x_PRU0.cmd
 *
 * Example Linker command file for linking programs built with the TI-PRU-CGT
 * on AM64x PRU0 cores
 *
 * Copyright (C) 2021-2022 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Specify the System Memory Map */
MEMORY
{
    PAGE 0:
      /* 12 KB PRU Instruction RAM */
      PRU_IMEM          : org = 0x00000000 len = 0x00003000

    PAGE 1:
      /* Data RAMs */
      /* 8 KB PRU Data RAM 0; use only the first 4 KB for PRU0 and reserve
       * the second 4 KB for RTU0 and Tx_PRU0 */
      PRU0_DMEM_0       : org = 0x00000000 len = 0x00001000
      /* 8 KB PRU Data RAM 1; reserved completely for Slice1 cores - PRU1,
       * RTU1 and Tx_PRU1; do not use for any Slice0 cores */
      PRU0_DMEM_1       : org = 0x00002000 len = 0x00001000
      /* NOTE: Custom split of the second 4 KB of ICSS Data RAMs 0 and 1
       * split equally between the corresponding RTU and Tx_PRU cores in
       * each slice */
      RTU_PRU0_DMEM_0   : org = 0x00001000 len = 0x00000800
      TX_PRU0_DMEM_0    : org = 0x00001800 len = 0x00000800
      RTU_PRU0_DMEM_1   : org = 0x00003000 len = 0x00000800
      TX_PRU0_DMEM_1    : org = 0x00003800 len = 0x00000800

    PAGE 2:
      /* C28 needs to be programmed to point to SHAREDMEM, default is 0 */
      /* 64 KB PRU Shared RAM */
      PRU_SHAREDMEM     : org = 0x00010000 len = 0x00010000
}

/* Specify the sections allocation into memory */
SECTIONS {
    /* Forces _c_int00 to the start of PRU IRAM. Not necessary when loading
       an ELF file, but useful when loading a binary */
    .text:_c_int00* >  0x0, PAGE 0

    .text           >  PRU_IMEM,    PAGE 0
    .stack          >  PRU_IMEM,    PAGE 0
    .bss            >  PRU0_DMEM_0, PAGE 1
    /*
    .cio            >  PRU0_DMEM_0, PAGE 1
    .data           >  PRU0_DMEM_0, PAGE 1
    .switch         >  PRU0_DMEM_0, PAGE 1
    .sysmem         >  PRU0_DMEM_0, PAGE 1
    .cinit          >  PRU0_DMEM_0, PAGE 1
    .rodata         >  PRU0_DMEM_0, PAGE 1
    .rofardata      >  PRU0_DMEM_0, PAGE 1
    .farbss         >  PRU0_DMEM_0, PAGE 1
    .fardata        >  PRU0_DMEM_0, PAGE 1
    */
}
