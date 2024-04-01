/*
 * AM263x_PRU1.cmd
 *
 * Example Linker command file for linking programs built with the C compiler
 * on AM263x PRU1 cores
 *
 */

/* Specify the System Memory Map */
MEMORY
{
    PAGE 0:
      /* 12 KB PRU Instruction RAM */
      PRU_IMEM          : org = 0x00000000 len = 0x00003000

    PAGE 1:
      /* Data RAMs */
      /* 8 KB PRU Data RAM 1 */
      PRU1_DMEM_1       : org = 0x00000000 len = 0x00002000
      /* 8 KB PRU Data RAM 0 */
      PRU1_DMEM_0       : org = 0x00002000 len = 0x00002000

    PAGE 2:
      /* C28 needs to be programmed to point to SHAREDMEM, default is 0 */
      /* 32 KB Shared general purpose memory RAM with ECC, shared between PRU0 and PRU1 */
      PRU_SHAREDMEM	    : org = 0x00010000 len = 0x00008000
}

/* Specify the sections allocation into memory */
SECTIONS {

    .text           >  PRU_IMEM, PAGE 0
    .bss            >  PRU1_DMEM_0, PAGE 1
    .stack          >  PRU1_DMEM_0, PAGE 1
}