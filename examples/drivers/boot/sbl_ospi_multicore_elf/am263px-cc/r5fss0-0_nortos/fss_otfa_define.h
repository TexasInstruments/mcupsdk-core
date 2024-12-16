//###########################################################################
//
// FILE:   fss_defines.h
//
//AUTHOR : Rohit Ranjan(a0230425)
//
// TITLE:  commonly used variable definitions.
//
//###########################################################################
#include <drivers/hw_include/am263px/cslr_soc_baseaddress.h>

#define	WR_MEM_32(addr, data)   *(uint32_t*)(addr) = (uint32_t)(data)
#define	RD_MEM_32(addr)         *(uint32_t*)(addr)

//------------------------------------------------
//           DEFINES 
//------------------------------------------------

#define FSS_FSAS__FSAS_MMR_CFG__FSAS_GENREGS_BASE                                       CSL_FLASH_CONFIG_REG1_U_BASE      
#define FSS_FSAS__FSAS_OTFA_CFG__FSAS_OTFA_REGS_BASE                                    CSL_FLASH_CONFIG_REG2_U_BASE 
#define FSS_FSS_MMR__FSS_MMR_CFG__FSS_GENREGS_BASE                                      CSL_FLASH_CONFIG_REG0_U_BASE 
#define FSS_OSPI0__OSPI_CFG_VBUSP__MMR__MMRVBP__REGS_BASE                               CSL_FLASH_CONFIG_REG6_U_BASE 
#define FSS_OSPI0__OSPI_CFG_VBUSP__OSPI_WRAP__ECC_AGGR_VBP__REGS_BASE                   CSL_FLASH_CONFIG_REG7_U_BASE 
#define FSS_OSPI0__OSPI_CFG_VBUSP__VBP2APB_WRAP__OSPI_CFG_VBP__OSPI_FLASH_APB_REGS_BASE CSL_FLASH_CONFIG_REG8_U_BASE 
#define FSS_FSAS__FOTA_MMR_CFG__FOTA_GENREGS_BASE                                       CSL_FLASH_CONFIG_REG11_U_BASE 
#define FSS_FSAS__FOTA_PDMEM_CFG__PDMEM_GENREGS_BASE                                    CSL_FLASH_CONFIG_REG12_U_BASE 
#define FSS_FSAS__FOTA_IMEM_CFG__IMEM_GENREGS_BASE                                      CSL_FLASH_CONFIG_REG13_U_BASE 
#define FSS_FSAS__FOTA_WBUF_CFG__WBUF_GENREGS_BASE                                      CSL_FLASH_CONFIG_REG14_U_BASE 
#define FSS_FSAS__ECC_AGGR_CFG__REGS_BASE                                               CSL_FLASH_CONFIG_REG15_U_BASE 
#define FSS_DAT_REG0_BASE                                                               CSL_FLASH_DATA_REG0_U_BASE
#define FSS_DAT_REG1_BASE                                                               CSL_FLASH_DATA_REG1_U_BASE
#define FSS_DAT_REG3_BASE                                                               CSL_FLASH_DATA_REG3_U_BASE


////////////////////////////////////FSS ///////////////////////////////////////////////////
#define ECC_EN          1
#define ECC_DIS         0

#define AES_EN          1
#define AES_DIS         0

#define PREFETCHER_DIS          1
#define PREFETCHER_EN           0

#define ECC_ON_ADDR_DIS          1
#define ECC_ON_ADDR_EN           0

#define OSPI_DDR_DIS            1
#define OSPI_DDR_EN             0

#define ECC_REG_START_4KB(N)            N
#define ECC_REG_SIZE_4KB(N)             N

#define ECC_1B_ERR_CLR                  0x1
#define ECC_2B_ERR_CLR                  0x1 << 1
#define ECC_WR_NOALIGN_ERR_CLR          0x1 << 2

#define ECC_1B_ERR_EN                  0x1
#define ECC_2B_ERR_EN                  0x1 << 1
#define ECC_WR_NOALIGN_ERR_EN          0x1 << 2

#define ECC_ERR_SEC(N)                     N
#define ECC_ERR_DED(N)                     N
#define ECC_ERR_DA0(N)                     N
#define ECC_ERR_DA1(N)                     N
#define ECC_ERR_MAC(N)                     N
#define ECC_ERR_ADR(N)                     N

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////OTFA ////////////////////////////////////////////////////////////////
#define OTFA_EN         1
#define OTFA_DIS        0

#define MAC_ALLIGN_FOR_FSS      1
#define MAC_ALLIGN_DEFAULT      0

#define MAZ_SIZE_16B            3
#define MAZ_SIZE_12B            2
#define MAZ_SIZE_08B            1
#define MAZ_SIZE_04B            0

#define MAC_ERR_EN              1
#define MAC_ERR_DIS             0

#define KEY_SIZE_128B           0
#define KEY_SIZE_256B           1

#define OPTIMIZATION_DIS        0
#define OPTIMIZATION_EN         1

#define FE_PROC_EN              1
#define FE_PROC_DIS             0

#define REGION(N)               N

#define SEC(N)                  N
#define DED(N)                  N

#define WR_PROTECT_EN           1
#define WR_PROTECT_DIS          0

#define MAC_MODE_DIS               0
#define MAC_MODE_GMAC              1
#define MAC_MODE_CBC_MAC           2


#define ENC_MODE_DIS               0
#define ENC_MODE_AES_CTR           1
#define ENC_MODE_ECB_PLUS          2

#define MAC_REG_START_4KB(N)            N
#define REG_START_4KB(N)            N
#define MAC_REG_SIZE_4KB(N)             N-1















