/*
 * AM243x_PRU0.cmd
 *
 * Example Linker command file for linking programs built with the TI-PRU-CGT
 * on AM243x PRU0 cores
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
      /* 8 KB PRU Data RAM 0; use only the first 4 KB for PRU0 and reserve
       * the second 4 KB for RTU0 and Tx_PRU0 */
      PRU0_DMEM_0       : org = 0x00000000 len = 0x00001000
      /* 8 KB PRU Data RAM 1; reserved completely for Slice1 cores - PRU1,
       * RTU1 and Tx_PRU1; do not use for any Slice0 cores */
      PRU0_DMEM_1       : org = 0x00002000 len = 0x00001000
      /* NOTE: Custom split of the second 4 KB of ICSS Data RAMs 0 and 1
       * split equally between the corresponding RTU and Tx_PRU cores in
       * each slice */
      RTU0_DMEM_0       : org = 0x00001000 len = 0x00000800
      TX_PRU0_DMEM_0    : org = 0x00001800 len = 0x00000800
      RTU0_DMEM_1       : org = 0x00003000 len = 0x00000800
      TX_PRU0_DMEM_1    : org = 0x00003800 len = 0x00000800

    PAGE 2:
      /* C28 needs to be programmed to point to SHAREDMEM, default is 0 */
      /* 64 KB PRU Shared RAM */
      PRU_SHAREDMEM     : org = 0x00010000 len = 0x00010000
}

/* Specify the sections allocation into memory */
SECTIONS {

    .text		>  PRU_IMEM,    PAGE 0
    .stack		>  PRU0_DMEM_0, PAGE 1
    .bss		>  PRU0_DMEM_0, PAGE 1
    /*
    .cio		>  PRU0_DMEM_0, PAGE 1
    .data		>  PRU0_DMEM_0, PAGE 1
    .switch		>  PRU0_DMEM_0, PAGE 1
    .sysmem		>  PRU0_DMEM_0, PAGE 1
    .cinit		>  PRU0_DMEM_0, PAGE 1
    .rodata		>  PRU0_DMEM_0, PAGE 1
    .rofardata	>  PRU0_DMEM_0, PAGE 1
    .farbss		>  PRU0_DMEM_0, PAGE 1
    .fardata	>  PRU0_DMEM_0, PAGE 1
    */
    
}
