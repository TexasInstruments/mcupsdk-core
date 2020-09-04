# SDL ECC BUS SAFETY {#EXAMPLES_SDL_ECC_BUS_SAFETY_MAIN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL ECC BUS SAFETY module. The example shows how to setup and use the ECC BUS Safety Diagnostic operation.
Shows the generation of SEC, DED and RED error on all MSS DSS and RSS nodes

# Note
Note :
1. SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction.
2. DSS Nodes are applicable to AWR294X and AM273X.

AM263x  : Supports MSS Nodes. (MSS_CR5C_AHB, MSS_CR5D_AHB , MSS_CR5C_AXI & MSS_CR5D_AXI)\
AM273X  : Supports MSS Nodes,DSS Nodes and RSS Nodes.\
AWR294x : Supports MSS Nodes,DSS Nodes and RSS Nodes.\

MSS Nodes Supported :
1. MSS_TPTC_A0_RD
2. MSS_TPTC_A1_RD
3. MSS_TPTC_B0_RD
4. MSS_TPTC_A0_WR
5. MSS_TPTC_A1_WR
6. MSS_TPTC_B0_WR
7. MSS_CR5A_AHB
8. MSS_CR5B_AHB
9. MSS_CR5C_AHB
10. MSS_CR5D_AHB
11. MSS_MBOX
12. MSS_CR5A_AXI_WR
13. MSS_CR5B_AXI_WR
14. MSS_CR5C_AXI_WR
15. MSS_CR5D_AXI_WR
16. MSS_CR5A_AXI_RD
17. MSS_CR5B_AXI_RD
18. MSS_CR5C_AXI_RD
19. MSS_CR5D_AXI_RD
20. MSS_CR5A_AXI_S
21. MSS_CR5B_AXI_S
22. MSS_CR5C_AXI_S
23. MSS_CR5D_AXI_S

DSS Nodes Supported :
1. DSS_DSP_MDMA
2. DSS_L3_BANKA
3. DSS_L3_BANKB
4. DSS_L3_BANKC
5. DSS_L3_BANKD
6. DSS_DSP_SDMA
7. DSS_TPTC_A0_RD
8. DSS_TPTC_A1_RD
9. DSS_TPTC_B0_RD
10. DSS_TPTC_B1_RD
11. DSS_TPTC_C0_RD
12. DSS_TPTC_C1_RD
13. DSS_TPTC_C2_RD
14. DSS_TPTC_C3_RD
15. DSS_TPTC_C4_RD
16. DSS_TPTC_C5_RD
17. DSS_TPTC_A0_WR
18. DSS_TPTC_A1_WR
19. DSS_TPTC_B0_WR
20. DSS_TPTC_B1_WR
21. DSS_TPTC_C0_WR
22. DSS_TPTC_C1_WR
23. DSS_TPTC_C2_WR
24. DSS_TPTC_C3_WR
25. DSS_TPTC_C4_WR
27. DSS_TPTC_C5_WR
28. DSS_CBUFF_FIFO
29. DSS_MCRC
30. DSS_PCR
31. DSS_HWA_DMA0
32. DSS_HWA_DMA1
33. DSS_MBOX
34. RSS_ADCBUF_RD applicable only for AWR294X
35. RSS_ADCBUF_WR applicable only for AWR294X
36. RSS_MBOX applicable only for AWR294X

Use Cases On AM263X in R5 Core
---------
Use Case | Description
---------|------------
UC-1     | CR5A_AHB_RED_Test in INTERRUPT  Method.
UC-2     | CR5B_AHB_RED_Test in INTERRUPT  Method.
UC-3     | CR5C_AHB_RED_Test in INTERRUPT  Method.
UC-4     | CR5D_AHB_RED_Test in INTERRUPT  Method.
UC-5     | TPTC_A0_WR_RED_Test in Interrupt Method.
UC-6     | TPTC_A1_WR_RED_Test in Interrupt Method.
UC-7     | TPTC_A0_RD_SEC_Test in Interrupt Method.
UC-8     | TPTC_A0_RD_DED_Test in Interrupt Method.
UC-9     | TPTC_A0_RD_RED_Test in Interrupt Method.
UC-10    | TPTC_A1_RD_SEC_Test in Interrupt Method.
UC-11    | TPTC_A1_RD_DED_Test in Interrupt Method.
UC-12    | TPTC_A1_RD_RED_Test in Interrupt Method.
UC-13    | MSS_CR5A_AXI_WR_RED_Test in Interrupt Method.
UC-14    | MSS_CR5B_AXI_WR_RED_Test in Interrupt Method.
UC-15    | MSS_CR5C_AXI_WR_RED_Test in Interrupt Method.
UC-16    | MSS_CR5D_AXI_WR_RED_Test in Interrupt Method.
UC-17    | MSS_CR5A_AXI_RD_SEC_Test in Interrupt Method.
UC-18    | MSS_CR5A_AXI_RD_RED_Test in Interrupt Method.
UC-19    | MSS_CR5B_AXI_RD_SEC_Test in Interrupt Method.
UC-20    | MSS_CR5B_AXI_RD_RED_Test in Interrupt Method.
UC-21    | MSS_CR5C_AXI_RD_SEC_Test in Interrupt Method.
UC-22    | MSS_CR5C_AXI_RD_RED_Test in Interrupt Method.
UC-23    | MSS_CR5D_AXI_RD_SEC_Test in Interrupt Method.
UC-24    | MSS_CR5D_AXI_RD_RED_Test in Interrupt Method.
UC-25    | MSS_CR5A_AXI_S_SEC_Test in Interrupt Method.
UC-26    | MSS_CR5A_AXI_S_RED_Test in Interrupt Method.
UC-27    | MSS_CR5B_AXI_S_SEC_Test in Interrupt Method.
UC-28    | MSS_CR5B_AXI_S_RED_Test in Interrupt Method.
UC-29    | MSS_CR5C_AXI_S_SEC_Test in Interrupt Method.
UC-30    | MSS_CR5C_AXI_S_RED_Test in Interrupt Method.
UC-31    | MSS_CR5D_AXI_S_SEC_Test in Interrupt Method.
UC-32    | MSS_CR5D_AXI_S_RED_Test in Interrupt Method.

Use Cases On AM273X in R5 Core
---------
Use Case | Description
---------|------------
UC-1     | CR5A_AHB_RED_Test in INTERRUPT  Method.
UC-2     | CR5B_AHB_RED_Test in INTERRUPT  Method.
UC-3     | TPTC_A0_WR_RED_Test in Interrupt Method.
UC-4     | TPTC_A1_WR_RED_Test in Interrupt Method.
UC-5     | TPTC_B0_WR_RED_Test in Interrupt Method.
UC-6     | TPTC_A0_RD_SEC_Test in Interrupt Method.
UC-7     | TPTC_A0_RD_DED_Test in Interrupt Method.
UC-8     | TPTC_A0_RD_RED_Test in Interrupt Method.
UC-9     | TPTC_A1_RD_SEC_Test in Interrupt Method.
UC-10    | TPTC_A1_RD_DED_Test in Interrupt Method.
UC-11    | TPTC_A1_RD_RED_Test in Interrupt Method.
UC-12    | TPTC_B0_RD_SEC_Test in Interrupt Method.
UC-13    | TPTC_B0_RD_DED_Test in Interrupt Method.
UC-14    | TPTC_B0_RD_RED_Test in Interrupt Method.
UC-15    | MSS_CR5A_AXI_WR_RED_Test in Interrupt Method.
UC-16    | MSS_CR5B_AXI_WR_RED_Test in Interrupt Method.
UC-17    | MSS_CR5A_AXI_RD_SEC_Test in Interrupt Method.
UC-18    | MSS_CR5A_AXI_RD_RED_Test in Interrupt Method.
UC-19    | MSS_CR5B_AXI_RD_SEC_Test in Interrupt Method.
UC-20    | MSS_CR5B_AXI_RD_RED_Test in Interrupt Method.
UC-21    | MSS_CR5A_AXI_S_SEC_Test in Interrupt Method.
UC-22    | MSS_CR5A_AXI_S_RED_Test in Interrupt Method.
UC-23    | MSS_CR5B_AXI_S_SEC_Test in Interrupt Method.
UC-24    | MSS_CR5B_AXI_S_RED_Test in Interrupt Method.
UC-25    | MSS_MBOX_SEC_Test in Interrupt Method.
UC-26    | MSS_MBOX_DED_Test in Interrupt Method.
UC-27    | MSS_MBOX_RED_Test in Interrupt Method.

Use Cases On AWR294X in R5 Core
---------
Use Case | Description
---------|------------
UC-1     | CR5A_AHB_RED_Test in INTERRUPT  Method.
UC-2     | CR5B_AHB_RED_Test in INTERRUPT  Method.
UC-3     | TPTC_A0_WR_RED_Test in Interrupt Method.
UC-4     | TPTC_A1_WR_RED_Test in Interrupt Method.
UC-5     | TPTC_B0_WR_RED_Test in Interrupt Method.
UC-6     | TPTC_A0_RD_SEC_Test in Interrupt Method.
UC-7     | TPTC_A0_RD_DED_Test in Interrupt Method.
UC-8     | TPTC_A0_RD_RED_Test in Interrupt Method.
UC-9     | TPTC_A1_RD_SEC_Test in Interrupt Method.
UC-10    | TPTC_A1_RD_DED_Test in Interrupt Method.
UC-11    | TPTC_A1_RD_RED_Test in Interrupt Method.
UC-12    | TPTC_B0_RD_SEC_Test in Interrupt Method.
UC-13    | TPTC_B0_RD_DED_Test in Interrupt Method.
UC-14    | TPTC_B0_RD_RED_Test in Interrupt Method.
UC-15    | MSS_CR5A_AXI_WR_RED_Test in Interrupt Method.
UC-16    | MSS_CR5B_AXI_WR_RED_Test in Interrupt Method.
UC-17    | MSS_CR5A_AXI_RD_SEC_Test in Interrupt Method.
UC-18    | MSS_CR5A_AXI_RD_RED_Test in Interrupt Method.
UC-19    | MSS_CR5B_AXI_RD_SEC_Test in Interrupt Method.
UC-20    | MSS_CR5B_AXI_RD_RED_Test in Interrupt Method.
UC-21    | MSS_CR5A_AXI_S_SEC_Test in Interrupt Method.
UC-22    | MSS_CR5A_AXI_S_RED_Test in Interrupt Method.
UC-23    | MSS_CR5B_AXI_S_SEC_Test in Interrupt Method.
UC-24    | MSS_CR5B_AXI_S_RED_Test in Interrupt Method.
UC-25    | MSS_MBOX_SEC_Test in Interrupt Method.
UC-26    | MSS_MBOX_DED_Test in Interrupt Method.
UC-27    | MSS_MBOX_RED_Test in Interrupt Method.

Use Cases On AWR294X in C66 Core
 ---------
 Use Case | Description
 ---------|------------
 UC-1     | MCRC_SEC_Test in Interrupt Method
 UC-2     | MCRC_DED_Test in Interrupt Method.
 UC-3     | MCRC_RED_Test in Interrupt Method.
 UC-4     | DSS_L3_BANKA_SEC_Test in Interrupt Method.
 UC-5     | DSS_L3_BANKA_DED_Test in Interrupt Method.
 UC-6     | DSS_L3_BANKA_RED_Test in Interrupt Method.
 UC-7     | DSS_L3_BANKB_SEC_Test in Interrupt Method.
 UC-8     | DSS_L3_BANKB_DED_Test in Interrupt Method.
 UC-9     | DSS_L3_BANKB_RED_Test in Interrupt Method.
 UC-10    | DSS_L3_BANKC_SEC_Test in Interrupt Method.
 UC-11    | DSS_L3_BANKC_DED_Test in Interrupt Method.
 UC-12    | DSS_L3_BANKC_RED_Test in Interrupt Method.
 UC-13    | DSS_L3_BANKD_SEC_Test in Interrupt Method.
 UC-14    | DSS_L3_BANKD_DED_Test in Interrupt Method.
 UC-15    | DSS_L3_BANKD_RED_Test in Interrupt Method.
 UC-16    | DSS_HWA_DMA0_SEC_Test in Interrupt Method.
 UC-17    | DSS_HWA_DMA0_DED_Test in Interrupt Method.
 UC-18    | DSS_HWA_DMA0_RED_Test in Interrupt Method.
 UC-19    | DSS_HWA_DMA1_SEC_Test in Interrupt Method.
 UC-20    | DSS_HWA_DMA1_DED_Test in Interrupt Method.
 UC-21    | DSS_HWA_DMA1_RED_Test in Interrupt Method.
 UC-22    | DSS_MBOX_SEC_Test in Interrupt Method.
 UC-23    | DSS_MBOX_DED_Test in Interrupt Method.
 UC-24    | DSS_MBOX_RED_Test in Interrupt Method.
 UC-25    | CBUFF_FIFO_SEC_Test in Interrupt Method.
 UC-26    | CBUFF_FIFO_DED_Test in Interrupt Method.
 UC-27    | CBUFF_FIFO_RED_Test in Interrupt Method.
 UC-28    | TPTC_A0_WR_RED_Test in Interrupt Method.
 UC-29    | TPTC_A1_WR_RED_Test in Interrupt Method.
 UC-30    | TPTC_B0_WR_RED_Test in Interrupt Method.
 UC-31    | TPTC_B1_WR_RED_Test in Interrupt Method.
 UC-32    | TPTC_C0_WR_RED_Test in Interrupt Method.
 UC-33    | TPTC_C1_WR_RED_Test in Interrupt Method.
 UC-34    | TPTC_C2_WR_RED_Test in Interrupt Method.
 UC-35    | TPTC_C3_WR_RED_Test in Interrupt Method.
 UC-36    | TPTC_C4_WR_RED_Test in Interrupt Method.
 UC-37    | TPTC_C5_WR_RED_Test in Interrupt Method.
 UC-38    | TPTC_A0_RD_SEC_Test in Interrupt Method.
 UC-39    | TPTC_A0_RD_DED_Test in Interrupt Method.
 UC-40    | TPTC_A0_RD_RED_Test in Interrupt Method.
 UC-41    | TPTC_A1_RD_SEC_Test in Interrupt Method.
 UC-42    | TPTC_A1_RD_DED_Test in Interrupt Method.
 UC-43    | TPTC_A1_RD_RED_Test in Interrupt Method.
 UC-44    | TPTC_B0_RD_SEC_Test in Interrupt Method.
 UC-45    | TPTC_B0_RD_DED_Test in Interrupt Method.
 UC-46    | TPTC_B0_RD_RED_Test in Interrupt Method.
 UC-47    | TPTC_B1_RD_SEC_Test in Interrupt Method.
 UC-48    | TPTC_B1_RD_DED_Test in Interrupt Method.
 UC-49    | TPTC_B1_RD_RED_Test in Interrupt Method.
 UC-50    | TPTC_C0_RD_SEC_Test in Interrupt Method.
 UC-51    | TPTC_C0_RD_DED_Test in Interrupt Method.
 UC-52    | TPTC_C0_RD_RED_Test in Interrupt Method.
 UC-53    | TPTC_C1_RD_SEC_Test in Interrupt Method.
 UC-54    | TPTC_C1_RD_DED_Test in Interrupt Method.
 UC-55    | TPTC_C1_RD_RED_Test in Interrupt Method
 UC-56    | TPTC_C2_RD_SEC_Test in Interrupt Method.
 UC-57    | TPTC_C2_RD_DED_Test in Interrupt Method.
 UC-58    | TPTC_C2_RD_RED_Test in Interrupt Method
 UC-59    | TPTC_C3_RD_SEC_Test in Interrupt Method.
 UC-60    | TPTC_C3_RD_DED_Test in Interrupt Method.
 UC-61    | TPTC_C3_RD_RED_Test in Interrupt Method
 UC-62    | TPTC_C4_RD_SEC_Test in Interrupt Method.
 UC-63    | TPTC_C4_RD_DED_Test in Interrupt Method.
 UC-64    | TPTC_C4_RD_RED_Test in Interrupt Method.
 UC-65    | TPTC_C5_RD_SEC_Test in Interrupt Method.
 UC-67    | TPTC_C5_RD_DED_Test in Interrupt Method.
 UC-68    | TPTC_C5_RD_RED_Test in Interrupt Method.
 UC-69    | PCR_SEC_Test in Interrupt Method.
 UC-70    | PCR_RED_Test in Interrupt Method.
 UC-71    | PCR_RED_Test in Interrupt Method.
 UC-72    | DSP_SDMA_SEC_Test in Interrupt Method.
 UC-73    | DSP_SDMA_DED_Test in Interrupt Method.
 UC-74    | DSP_SDMA_RED_Test in Interrupt Method.
 UC-75    | DSP_MDMA_RED_Test in Interrupt Method.
 UC-76    | RSS_MBOX_SEC_Test in Interrupt Method.
 UC-77    | RSS_MBOX_DED_Test in Interrupt Method.
 UC-78    | RSS_MBOX_RED_Test in Interrupt Method.
 UC-79    | RSS_ADCBUF_RD_RED_Test in Interrupt Method.
 UC-80    | RSS_ADCBUF_WR_SEC_Test in Interrupt Method.
 UC-81    | RSS_ADCBUF_WR_DED_Test in Interrupt Method.
 UC-82    | RSS_ADCBUF_WR_RED_Test in Interrupt Method.

Use Cases On AM273X in C66 Core
---------

 Use Case | Description
 ---------|------------
 UC-1     | MDO_FIFO_DED_Test in Polling Method.
 UC-2     | MDO_FIFO_RED_Test in Polling Method.
 UC-3     | MCRC_SEC_Test in Interrupt Method
 UC-4     | MCRC_DED_Test in Interrupt Method.
 UC-5     | MCRC_RED_Test in Interrupt Method.
 UC-6     | DSS_L3_BANKA_SEC_Test in Interrupt Method.
 UC-7     | DSS_L3_BANKA_DED_Test in Interrupt Method.
 UC-8     | DSS_L3_BANKA_RED_Test in Interrupt Method.
 UC-9     | DSS_L3_BANKB_SEC_Test in Interrupt Method.
 UC-10    | DSS_L3_BANKB_DED_Test in Interrupt Method.
 UC-11    | DSS_L3_BANKB_RED_Test in Interrupt Method.
 UC-12    | DSS_L3_BANKC_SEC_Test in Interrupt Method.
 UC-13    | DSS_L3_BANKC_DED_Test in Interrupt Method.
 UC-14    | DSS_L3_BANKC_RED_Test in Interrupt Method.
 UC-15    | DSS_L3_BANKD_SEC_Test in Interrupt Method.
 UC-16    | DSS_L3_BANKD_DED_Test in Interrupt Method.
 UC-17    | DSS_L3_BANKD_RED_Test in Interrupt Method.
 UC-18    | DSS_HWA_DMA0_SEC_Test in Interrupt Method.
 UC-19    | DSS_HWA_DMA0_DED_Test in Interrupt Method.
 UC-20    | DSS_HWA_DMA0_RED_Test in Interrupt Method.
 UC-21    | DSS_HWA_DMA1_SEC_Test in Interrupt Method.
 UC-22    | DSS_HWA_DMA1_DED_Test in Interrupt Method.
 UC-23    | DSS_HWA_DMA1_RED_Test in Interrupt Method.
 UC-24    | DSS_MBOX_SEC_Test in Interrupt Method.
 UC-25    | DSS_MBOX_DED_Test in Interrupt Method.
 UC-26    | DSS_MBOX_RED_Test in Interrupt Method.
 UC-29    | CBUFF_FIFO_SEC_Test in Interrupt Method.
 UC-30    | CBUFF_FIFO_DED_Test in Interrupt Method.
 UC-31    | CBUFF_FIFO_RED_Test in Interrupt Method.
 UC-32    | TPTC_A0_WR_RED_Test in Interrupt Method.
 UC-33    | TPTC_A1_WR_RED_Test in Interrupt Method.
 UC-32    | TPTC_B0_WR_RED_Test in Interrupt Method.
 UC-33    | TPTC_B1_WR_RED_Test in Interrupt Method.
 UC-34    | TPTC_C0_WR_RED_Test in Interrupt Method.
 UC-35    | TPTC_C1_WR_RED_Test in Interrupt Method.
 UC-36    | TPTC_C2_WR_RED_Test in Interrupt Method.
 UC-37    | TPTC_C3_WR_RED_Test in Interrupt Method.
 UC-38    | TPTC_C4_WR_RED_Test in Interrupt Method.
 UC-39    | TPTC_C5_WR_RED_Test in Interrupt Method.
 UC-40    | TPTC_A0_RD_SEC_Test in Interrupt Method.
 UC-41    | TPTC_A0_RD_DED_Test in Interrupt Method.
 UC-42    | TPTC_A0_RD_RED_Test in Interrupt Method.
 UC-43    | TPTC_A1_RD_SEC_Test in Interrupt Method.
 UC-44    | TPTC_A1_RD_DED_Test in Interrupt Method.
 UC-45    | TPTC_A1_RD_RED_Test in Interrupt Method.
 UC-46    | TPTC_B0_RD_SEC_Test in Interrupt Method.
 UC-47    | TPTC_B0_RD_DED_Test in Interrupt Method.
 UC-48    | TPTC_B0_RD_RED_Test in Interrupt Method.
 UC-49    | TPTC_B1_RD_SEC_Test in Interrupt Method.
 UC-50    | TPTC_B1_RD_DED_Test in Interrupt Method.
 UC-51    | TPTC_B1_RD_RED_Test in Interrupt Method.
 UC-52    | TPTC_C0_RD_SEC_Test in Interrupt Method.
 UC-53    | TPTC_C0_RD_DED_Test in Interrupt Method.
 UC-54    | TPTC_C0_RD_RED_Test in Interrupt Method.
 UC-55    | TPTC_C1_RD_SEC_Test in Interrupt Method.
 UC-56    | TPTC_C1_RD_DED_Test in Interrupt Method.
 UC-57    | TPTC_C1_RD_RED_Test in Interrupt Method
 UC-58    | TPTC_C2_RD_SEC_Test in Interrupt Method.
 UC-59    | TPTC_C2_RD_DED_Test in Interrupt Method.
 UC-60    | TPTC_C2_RD_RED_Test in Interrupt Method
 UC-61    | TPTC_C3_RD_SEC_Test in Interrupt Method.
 UC-62    | TPTC_C3_RD_DED_Test in Interrupt Method.
 UC-63    | TPTC_C3_RD_RED_Test in Interrupt Method
 UC-64    | TPTC_C4_RD_SEC_Test in Interrupt Method.
 UC-65    | TPTC_C4_RD_DED_Test in Interrupt Method.
 UC-66    | TPTC_C4_RD_RED_Test in Interrupt Method
 UC-67    | TPTC_C5_RD_SEC_Test in Interrupt Method.
 UC-68    | TPTC_C5_RD_DED_Test in Interrupt Method.
 UC-69    | TPTC_C5_RD_RED_Test in Interrupt Method
 UC-70    | PCR_SEC_Test in Interrupt Method.
 UC-71    | PCR_RED_Test in Interrupt Method.
 UC-72    | PCR_RED_Test in Interrupt Method.
 UC-73    | DSP_SDMA_SEC_Test in Interrupt Method.
 UC-74    | DSP_SDMA_DED_Test in Interrupt Method.
 UC-75    | DSP_SDMA_RED_Test in Interrupt Method.
 UC-76    | DSP_MDMA_RED_Test in Interrupt Method.
 UC-77    | MDO_FIFO_SEC_Test in Interrupt Method.

# Supported Combinations {#EXAMPLES_SDL_ECC_BUS_SAFETY_MAIN_COMBOS}

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | R5F, C66  nortos
 Toolchain      | ti-arm-clang,ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc_bus_safety/ecc_bus_safety_main/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_ECC_BUS_SAFETY_PAGE

# Sample Output
Shown below is a sample output when the application is run on AM263X  in R5 Core,

\code

 ECC BUS SAFETY  Application

 ECC BUS SAFETY TEST START : starting

Applications Name: CR5A_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 150  micro secs

Applications Name: CR5B_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 140  micro secs

Applications Name: CR5C_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 140  micro secs

Applications Name: CR5D_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 139  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 105  micro secs

Applications Name: TPTC_A1_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: TPTC_A0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 127  micro secs

Applications Name: TPTC_A0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 126  micro secs

Applications Name: TPTC_A0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 107  micro secs

Applications Name: TPTC_A1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 121  micro secs

Applications Name: TPTC_A1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 123  micro secs

Applications Name: TPTC_A1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 106  micro secs

Applications Name: MSS_CR5A_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 106  micro secs

Applications Name: MSS_CR5B_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5C_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5D_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5A_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 104  micro secs

Applications Name: MSS_CR5A_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5B_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 103  micro secs

Applications Name: MSS_CR5B_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 101  micro secs

Applications Name: MSS_CR5C_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5C_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5D_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 105  micro secs

Applications Name: MSS_CR5D_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5A_AXI_S_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5A_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 101  micro secs

Applications Name: MSS_CR5B_AXI_S_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 103  micro secs

Applications Name: MSS_CR5B_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5C_AXI_S_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 103  micro secs

Applications Name: MSS_CR5C_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 102  micro secs

Applications Name: MSS_CR5D_AXI_S_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 103  micro secs

Applications Name: MSS_CR5D_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 103  micro secs

 All tests have passed

\endcode

Shown below is a sample output when the application is run on AWR294X  in R5 Core,

\code
[Cortex_R5_0]
ECC BUS SAFETY  Application

ECC BUS SAFETY TEST START : starting

Applications Name: CR5A_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: CR5B_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 23  micro secs

Applications Name: TPTC_A1_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: TPTC_B0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: TPTC_A0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_A0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 49  micro secs

Applications Name: TPTC_A0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 26  micro secs

Applications Name: TPTC_A1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 49  micro secs

Applications Name: TPTC_A1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 48  micro secs

Applications Name: TPTC_A1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_B0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 49  micro secs

Applications Name: TPTC_B0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 46  micro secs

Applications Name: TPTC_B0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: MSS_CR5A_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: MSS_CR5B_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: MSS_CR5A_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: MSS_CR5A_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: MSS_CR5B_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: MSS_CR5B_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: MSS_CR5A_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: MSS_CR5B_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

All tests have passed

\endcode


Shown below is a sample output when the application is run on AM273X  in R5 Core,

\code
[Cortex_R5_0]
ECC BUS SAFETY  Application

ECC BUS SAFETY TEST START : starting

Applications Name: CR5A_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: CR5B_AHB_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 23  micro secs

Applications Name: TPTC_A1_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: TPTC_B0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: TPTC_A0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_A0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 49  micro secs

Applications Name: TPTC_A0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 26  micro secs

Applications Name: TPTC_A1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 49  micro secs

Applications Name: TPTC_A1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 48  micro secs

Applications Name: TPTC_A1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_B0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 49  micro secs

Applications Name: TPTC_B0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 46  micro secs

Applications Name: TPTC_B0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: MSS_CR5A_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: MSS_CR5B_AXI_WR_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: MSS_CR5A_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: MSS_CR5A_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: MSS_CR5B_AXI_RD_SEC_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 22  micro secs

Applications Name: MSS_CR5B_AXI_RD_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: MSS_CR5A_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: MSS_CR5B_AXI_S_RED_Test in INTERRUPT  Method  PASSED  and Time taken for the Test is 17  micro secs

All tests have passed

\endcode

Shown below is a sample output when the application is run on AWR294X  in C66 Core,

\code
[C66xx_DSP]
 ECC BUS SAFETY  Application

 ECC BUS SAFETY TEST START : starting

Applications Name: MCRC_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 39  micro secs

Applications Name: MCRC_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: MCRC_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: DSS_L3_BANKA_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 39  micro secs

Applications Name: DSS_L3_BANKA_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_L3_BANKA_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_L3_BANKB_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 39  micro secs

Applications Name: DSS_L3_BANKB_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: DSS_L3_BANKB_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: DSS_L3_BANKC_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 38  micro secs

Applications Name: DSS_L3_BANKC_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: DSS_L3_BANKC_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: DSS_L3_BANKD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 39  micro secs

Applications Name: DSS_L3_BANKD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_L3_BANKD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_HWA_DMA0_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 39  micro secs

Applications Name: DSS_HWA_DMA0_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: DSS_HWA_DMA0_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_HWA_DMA1_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 38  micro secs

Applications Name: DSS_HWA_DMA1_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_HWA_DMA1_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: DSS_MBOX_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 38  micro secs

Applications Name: DSS_MBOX_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSS_MBOX_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: CBUFF_FIFO_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 39  micro secs

Applications Name: CBUFF_FIFO_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: CBUFF_FIFO_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_A0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_A0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: TPTC_A0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_A1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_A1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: TPTC_A1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 12  micro secs

Applications Name: TPTC_B0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_B0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: TPTC_B0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_B1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_B1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: TPTC_B1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_C0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 59  micro secs

Applications Name: TPTC_C0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 31  micro secs

Applications Name: TPTC_C0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_C1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_C1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 32  micro secs

Applications Name: TPTC_C1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 14  micro secs

Applications Name: TPTC_C2_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_C2_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 32  micro secs

Applications Name: TPTC_C2_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_C3_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_C3_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 32  micro secs

Applications Name: TPTC_C3_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_C4_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_C4_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 32  micro secs

Applications Name: TPTC_C4_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: TPTC_C5_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: TPTC_C5_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 33  micro secs

Applications Name: TPTC_C5_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: PCR_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 57  micro secs

Applications Name: PCR_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 33  micro secs

Applications Name: PCR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 13  micro secs

Applications Name: DSP_SDMA_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 38  micro secs

Applications Name: DSP_SDMA_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

Applications Name: DSP_SDMA_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 14  micro secs

Applications Name: DSP_MDMA_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 14  micro secs

Applications Name: RSS_MBOX_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

Applications Name: RSS_MBOX_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

Applications Name: RSS_MBOX_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

Applications Name: RSS_ADCBUF_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

Applications Name: RSS_ADCBUF_WR_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

Applications Name: RSS_ADCBUF_WR_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: RSS_ADCBUF_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 15  micro secs

 All tests have passed

\endcode

Shown below is a sample output when the application is run on AM273X  in C66 Core,
\code
[C66xx_DSP]
 ECC BUS SAFETY  Application

 ECC BUS SAFETY TEST START : starting

Applications Name: MDO_FIFO_DED_Test in Polling Method  PASSED  and Time taken for the Test is 6  micro secs

Applications Name: MDO_FIFO_RED_Test in Polling Method  PASSED  and Time taken for the Test is 6  micro secs

Applications Name: MCRC_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 30  micro secs

Applications Name: MCRC_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: MCRC_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKA_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: DSS_L3_BANKA_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKA_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKB_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: DSS_L3_BANKB_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKB_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: DSS_L3_BANKC_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: DSS_L3_BANKC_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKC_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 30  micro secs

Applications Name: DSS_L3_BANKD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_L3_BANKD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: DSS_HWA_DMA0_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: DSS_HWA_DMA0_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: DSS_HWA_DMA0_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_HWA_DMA1_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: DSS_HWA_DMA1_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_HWA_DMA1_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_MBOX_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 30  micro secs

Applications Name: DSS_MBOX_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: DSS_MBOX_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: CBUFF_FIFO_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: CBUFF_FIFO_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: CBUFF_FIFO_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_WR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 8  micro secs

Applications Name: TPTC_A0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_A0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_A0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: TPTC_A1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_A1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_A1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 9  micro secs

Applications Name: TPTC_B0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_B0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_B0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: TPTC_B1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_B1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_B1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: TPTC_C0_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_C0_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_C0_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: TPTC_C1_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_C1_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_C1_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_C2_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 44  micro secs

Applications Name: TPTC_C2_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_C2_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: TPTC_C3_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_C3_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 24  micro secs

Applications Name: TPTC_C3_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_C4_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 45  micro secs

Applications Name: TPTC_C4_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_C4_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: TPTC_C5_RD_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: TPTC_C5_RD_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: TPTC_C5_RD_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: PCR_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 43  micro secs

Applications Name: PCR_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 25  micro secs

Applications Name: PCR_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 10  micro secs

Applications Name: DSP_SDMA_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 29  micro secs

Applications Name: DSP_SDMA_DED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSP_SDMA_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: DSP_MDMA_RED_Test in Interrupt Method  PASSED  and Time taken for the Test is 11  micro secs

Applications Name: MDO_FIFO_SEC_Test in Interrupt Method  PASSED  and Time taken for the Test is 30  micro secs

 All tests have passed
\endcode
