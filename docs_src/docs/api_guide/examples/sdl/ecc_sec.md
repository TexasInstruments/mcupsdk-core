# SDL ECC SEC {#EXAMPLES_SDL_ECC_SEC}

[TOC]

# Introduction

This example takes an aggregator index as input from the user and performs single bit error test for the corresponding aggregator.  It involves the following steps:

* Setup of an ESM application callback to receive Single Error Correction (SEC) and setup of ECC Aggregators in general
* Triggering of ECC events for all the RAM IDs, including Interconnect type and Wrapper type
* Printing out error information within the ECC callback upon reception of ECC events

# Aggregators Supported

The following aggregators can be tested using this example.

\cond SOC_AM64X

 Aggregator Index   |   ECC Aggregator
 -------------------|------------------------------------
 0                  |   SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR
 1                  |   SDL_MMCSD1_EMMCSD4SS_ECC_AGGR_RXMEM
 2                  |   SDL_ADC0_ADC12_CORE_FIFO_RAM_ECC_AGGR
 3                  |   SDL_ECC_AGGR1
 4                  |   SDL_ECC_AGGR0
 5                  |   SDL_SA2_UL0_SA2_UL_SA2_UL_ECC_AGGR
 6                  |   SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
 7                  |   SDL_DMASS0_DMSS_AM64_ECCAGGR
 8                  |   SDL_MMCSD1_EMMCSD4SS_ECC_AGGR_TXMEM
 9                  |   SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR
 10                 |   SDL_PRU_ICSSG1_ICSS_G_16FF_CORE_BORG_ECC_AGGR
 11                 |   SDL_PRU_ICSSG0_ICSS_G_16FF_CORE_BORG_ECC_AGGR
 12                 |   SDL_MSRAM_256K2_MSRAM32KX64E_ECC_AGGR
 13                 |   SDL_FSS0_FSS_UL_OSPI0_OSPI_WRAP_ECC_AGGR
 14                 |   SDL_CPSW0_CPSW_3GUSS_CORE_ECC_CPSW_ECC_AGGR
 15                 |   SDL_GICSS0_GIC500SS_1_2_ECC_AGGR
 16                 |   SDL_PCIE0_PCIE_G2X1_64_CORE_AXI_ECC_AGGR
 17                 |   SDL_PCIE0_PCIE_G2X1_64_CORE_CORE_ECC_AGGR
 18                 |   SDL_USB0_USB3P0SS64_16FFC_USB3P0SS64_CORE_A__ECC_AGGR
 19                 |   SDL_PDMA1_PDMA_AM64_MAIN1_ECCAGGR
 20                 |   SDL_DMSC0_DMSC_LITE
 21                 |   SDL_MSRAM_256K1_MSRAM32KX64E_ECC_AGGR
 22                 |   SDL_MSRAM_256K0_MSRAM32KX64E_ECC_AGGR
 23                 |   SDL_MSRAM_256K3_MSRAM32KX64E_ECC_AGGR
 24                 |   SDL_MSRAM_256K5_MSRAM32KX64E_ECC_AGGR
 25                 |   SDL_MSRAM_256K4_MSRAM32KX64E_ECC_AGGR
 26                 |   SDL_MSRAM_256K7_MSRAM32KX64E_ECC_AGGR
 27                 |   SDL_MSRAM_256K6_MSRAM32KX64E_ECC_AGGR
 28                 |   SDL_MCU_M4FSS0_BLAZAR_ECC
 29                 |   SDL_PDMA0_PDMA_AM64_MAIN0_ECCAGGR
 30                 |   SDL_MMCSD0_EMMC8SS_16FFC_ECC_AGGR_RXMEM
 31                 |   SDL_MMCSD0_EMMC8SS_16FFC_ECC_AGGR_TXMEM
 32                 |   SDL_VTM0_K3VTM_N16FFC_ECCAGGR
 33                 |   SDL_R5FSS1_PULSAR_LITE_CPU0_ECC_AGGR
 34                 |   SDL_R5FSS1_PULSAR_LITE_CPU1_ECC_AGGR
 35                 |   SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR
 36                 |   SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR
 37                 |   SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_CORE0
 38                 |   SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_COREPAC
 39                 |   SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_CORE1
 \endcond

\cond SOC_AM243X

 Aggregator Index   |   ECC Aggregator
 -------------------|------------------------------------
 0                  |   SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR
 1                  |   SDL_MMCSD1_EMMCSD4SS_ECC_AGGR_RXMEM
 2                  |   SDL_ADC0_ADC12_CORE_FIFO_RAM_ECC_AGGR
 3                  |   SDL_ECC_AGGR1
 4                  |   SDL_ECC_AGGR0
 5                  |   SDL_SA2_UL0_SA2_UL_SA2_UL_ECC_AGGR
 6                  |   SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
 7                  |   SDL_DMASS0_DMSS_AM64_ECCAGGR
 8                  |   SDL_MMCSD1_EMMCSD4SS_ECC_AGGR_TXMEM
 9                  |   SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR
 10                 |   SDL_PRU_ICSSG1_ICSS_G_16FF_CORE_BORG_ECC_AGGR
 11                 |   SDL_PRU_ICSSG0_ICSS_G_16FF_CORE_BORG_ECC_AGGR
 12                 |   SDL_MSRAM_256K2_MSRAM32KX64E_ECC_AGGR
 13                 |   SDL_FSS0_FSS_UL_OSPI0_OSPI_WRAP_ECC_AGGR
 14                 |   SDL_CPSW0_CPSW_3GUSS_CORE_ECC_CPSW_ECC_AGGR
 15                 |   SDL_GICSS0_GIC500SS_1_2_ECC_AGGR
 16                 |   SDL_PCIE0_PCIE_G2X1_64_CORE_AXI_ECC_AGGR
 17                 |   SDL_PCIE0_PCIE_G2X1_64_CORE_CORE_ECC_AGGR
 18                 |   SDL_USB0_USB3P0SS64_16FFC_USB3P0SS64_CORE_A__ECC_AGGR
 19                 |   SDL_PDMA1_PDMA_AM64_MAIN1_ECCAGGR
 20                 |   SDL_DMSC0_DMSC_LITE
 21                 |   SDL_MSRAM_256K1_MSRAM32KX64E_ECC_AGGR
 22                 |   SDL_MSRAM_256K0_MSRAM32KX64E_ECC_AGGR
 23                 |   SDL_MSRAM_256K3_MSRAM32KX64E_ECC_AGGR
 24                 |   SDL_MSRAM_256K5_MSRAM32KX64E_ECC_AGGR
 25                 |   SDL_MSRAM_256K4_MSRAM32KX64E_ECC_AGGR
 26                 |   SDL_MSRAM_256K7_MSRAM32KX64E_ECC_AGGR
 27                 |   SDL_MSRAM_256K6_MSRAM32KX64E_ECC_AGGR
 28                 |   SDL_MCU_M4FSS0_BLAZAR_ECC
 29                 |   SDL_PDMA0_PDMA_AM64_MAIN0_ECCAGGR
 30                 |   SDL_MMCSD0_EMMC8SS_16FFC_ECC_AGGR_RXMEM
 31                 |   SDL_MMCSD0_EMMC8SS_16FFC_ECC_AGGR_TXMEM
 32                 |   SDL_VTM0_K3VTM_N16FFC_ECCAGGR
 33                 |   SDL_R5FSS1_PULSAR_LITE_CPU0_ECC_AGGR
 34                 |   SDL_R5FSS1_PULSAR_LITE_CPU1_ECC_AGGR
 35                 |   SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR
 36                 |   SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR
 \endcond

# Supported Combinations {#EXAMPLES_SDL_ECC_SEC_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc_sec/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run for a couple of aggregators,

\cond SOC_AM64X || SOC_AM243X
\code
[MAIN_Cortex_R5_0_0] 
ECC_Test_init: Exception init complete 

ECC_Test_init: Init MCU ESM complete 

ECC_Test_init: Init MAIN ESM complete 

ECC_Test_init: ECC Callback Init complete for MCU ESM 

ECC_Test_init: ECC Callback Init complete for Main ESM 

 ECC SDL API tests: starting 


Refer the User Guide for the aggregator information

 Select the memory to test...2

...selected 2
PSC Domain STATE Result = 1PSC MOdule STATE Result = 3
 ecc_aggrtest: [2] single bit error self test: SDL_ADC0_ADC12_CORE_FIFO_RAM_ECC_AGGR starting 


ECC_Memory_init: [2] SDL_ADC0_ADC12_CORE_FIFO_RAM_ECC_AGGR ECC Init complete 

self test started not accessable RamId 0  starting 


  Got it

self test started not accessable RamId 1  starting 


  Got it

 Select the memory to test...6

...selected 6
PSC Domain STATE Result = 1PSC MOdule STATE Result = 3
 ecc_aggrtest: [6] single bit error self test: SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR starting 


ECC_Memory_init: [6] SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR ECC Init complete 

self test started accessable RamId 0  starting 

self test started RamId 1  starting 

 Select the memory to test...

\endcode
\endcond