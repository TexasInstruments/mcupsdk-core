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

        
#ifdef MX25
        #define DBOPCODE        1
#endif
#ifdef IS25
        #define DBOPCODE        0
#else
        #define DBOPCODE        1
#endif


#define DQS_CLK         0x0
#define INT_LPBK_CLK    0x7 

#define L2_ADDR1                 0x70100000
#define L2_ADDR2                 0x70100200
#define L2_ADDR3                 0x70100400

#define OSPI_EN                 1
#define OSPI_DIS                0

#define DRV_RST_PIN             1
#define RST_PIN_ONDQ3           0
#define RST_PIN_DEDICATED       1

#define DAC_EN                  1
#define DAC_DIS                 0

#define LEGACY_IP_MODE_EN       1
#define LEGACY_IP_MODE_DIS      0

#define DMA_INTF_EN             1
#define DMA_INTF_DIS            0

#define AHB_ADDR_REMAP_EN       1 
#define AHB_ADDR_REMAP_DIS      0

#define DIV2                    0x0 
#define DIV4                    0x1 
#define DIV6                    0x2 
#define DIV8                    0x3 
#define DIV10                   0x4 
#define DIV12                   0x5 
#define DIV14                   0x6 
#define DIV16                   0x7 
#define DIV18                   0x8 
#define DIV20                   0x9 
#define DIV22                   0xA 
#define DIV24                   0xB 
#define DIV26                   0xC 
#define DIV28                   0xD 
#define DIV30                   0xE 
#define DIV32                   0xF 

#define AHB_DEC_EN              1
#define AHB_DEC_DIS             0

#define DTR_PROTOCOL_EN         1
#define DTR_PROTOCOL_DIS        0

#define CRC_EN                  1
#define CRC_DIS                 0

#define DB_OPCODE_MODE_EN       1
#define DB_OPCODE_MODE_DIS      0

#define OSPI_IDLE               1

#define MODE_BIT_EN             1
#define MODE_BIT_DIS            0

#define AUTO_WEL_EN             0
#define AUTO_WEL_DIS            1

#define FLASH_SIZE_512M         0     
#define FLASH_SIZE_1G           1     
#define FLASH_SIZE_2G           2     
#define FLASH_SIZE_4G           3     

#define SPI_MODE                0
#define DUAL_SPI_MODE           1
#define QUAD_SPI_MODE           2
#define OCTAL_SPI_MODE          3

#define PER_BLOCK_1B            0 
#define PER_BLOCK_2B            1 
#define PER_BLOCK_4B            2 
#define PER_BLOCK_8B            3 
#define PER_BLOCK_64KB          0x10 

#define PER_PAGE_1B             0
#define PER_PAGE_2B             1
#define PER_PAGE_256B           0x100

#define ADDRESS_SIZE_1B         0
#define ADDRESS_SIZE_2B         1
#define ADDRESS_SIZE_3B         2
#define ADDRESS_SIZE_4B         3

#define POLL_DLY0               0
#define POLL_DLY1               1
#define POLL_DLY2               2
#define POLL_DLY3               3
#define POLL_DLY4               4
#define POLL_DLY5               5
#define POLL_DLY6               6
#define POLL_DLY7               7
#define POLL_DLY8               8
#define POLL_DLY9               9
#define POLL_DLY10              10
#define POLL_DLY11              11
#define POLL_DLY12              12
#define POLL_DLY13              13
#define POLL_DLY14              14
#define POLL_DLY15              15
#define POLL_DLY64              64
#define POLL_DLY230             230
#define POLL_DLY231             231
#define POLL_DLY232             232
#define POLL_DLY233             233
#define POLL_DLY234             234
#define POLL_DLY235             235
#define POLL_DLY236             236
#define POLL_DLY237             237
#define POLL_DLY238             238
#define POLL_DLY239             239
#define POLL_DLY240             240
#define POLL_DLY241             241
#define POLL_DLY242             242
#define POLL_DLY243             243
#define POLL_DLY244             244
#define POLL_DLY245             245
#define POLL_DLY246             246
#define POLL_DLY247             247
#define POLL_DLY248             248
#define POLL_DLY249             249
#define POLL_DLY250             250

#define POLL_CNT0               0
#define POLL_CNT1               1
#define POLL_CNT2               2
#define POLL_CNT3               3
#define POLL_CNT4               4
#define POLL_CNT5               5
#define POLL_CNT6               6
#define POLL_CNT7               7
#define POLL_CNT8               8
#define POLL_CNT9               9
#define POLL_CNT10              10
#define POLL_CNT11              11
#define POLL_CNT12              12
#define POLL_CNT13              13
#define POLL_CNT14              14
#define POLL_CNT15              15

#define POLL_CNT(N)             N

#define POLL_POL_LOW            0
#define POLL_POL_HIGH           1

#define POLL_BIT0               0
#define POLL_BIT1               1
#define POLL_BIT2               2
#define POLL_BIT3               3
#define POLL_BIT4               4
#define POLL_BIT5               5
#define POLL_BIT6               6
#define POLL_BIT7               7

#define DQS_EN                  1
#define DQS_DIS                 0


///STIG REG Defines///////////////
#define STIG_MEM_BANK_EN         1
#define STIG_MEM_BANK_DIS        0

#define DUMMY_CYC0              0
#define DUMMY_CYC1              1
#define DUMMY_CYC2              2
#define DUMMY_CYC3              3
#define DUMMY_CYC4              4
#define DUMMY_CYC5              5
#define DUMMY_CYC6              6
#define DUMMY_CYC7              7
#define DUMMY_CYC8              8
#define DUMMY_CYC9              9
#define DUMMY_CYC10             10
#define DUMMY_CYC11             11
#define DUMMY_CYC12             12
#define DUMMY_CYC13             13
#define DUMMY_CYC14             14
#define DUMMY_CYC15             15
#define DUMMY_CYC16             16
#define DUMMY_CYC17             17
#define DUMMY_CYC18             18
#define DUMMY_CYC19             19
#define DUMMY_CYC20             20
#define DUMMY_CYC21             21
#define DUMMY_CYC22             22
#define DUMMY_CYC23             23
#define DUMMY_CYC24             24
#define DUMMY_CYC25             25
#define DUMMY_CYC26             26
#define DUMMY_CYC27             27
#define DUMMY_CYC28             28
#define DUMMY_CYC29             29
#define DUMMY_CYC30             30

#define DATA_BYTE_CNT1          0
#define DATA_BYTE_CNT2          1
#define DATA_BYTE_CNT3          2
#define DATA_BYTE_CNT4          3
#define DATA_BYTE_CNT5          4
#define DATA_BYTE_CNT6          5
#define DATA_BYTE_CNT7          6
#define DATA_BYTE_CNT8          7

#define ADDR_BYTE_CNT1          0
#define ADDR_BYTE_CNT2          1
#define ADDR_BYTE_CNT3          2
#define ADDR_BYTE_CNT4          3


#define RD_CMD                  1
#define WR_CMD                  0
#define NO_DATA_CMD             2

#define MODE_BIT_EN             1
#define MODE_BIT_DIS            0

#define CMD_ADDR_EN             1
#define CMD_ADDR_DIS            0

#define POLL_DIS                1
#define POLL_EN                 0

///////////////////////////////////
#define RX_DLL_BYPASS_EN           1
#define RX_DLL_BYPASS_DIS          0


#define PHY_EN                  1
#define PHY_DIS                 0

#define DELAY(N)                N
#define TX_DLL_DLY(N)           N
#define RX_DLL_DLY(N)           N
#define WATERMARK(N)                    N
#define NUM_OF_BYTES(N)                 N
#define FLASH_START_ADDR(N)             N
#define INDAC_AHB_START_ADDR(N)         N
#define INDAC_ADDR_RANGE_2PWRN(N)       N

#define INDAC_EN                1
#define INDAC_DIS               0

#define INDAC_RD_SRAM_SIZE(N)           N

#define UNDERFLOW_DET            1 << 1
#define INDAC_COMPLETE           1 << 2
#define TWO_INDAC_PENDING        1 << 3
#define WR_TO_PROT_AREA          1 << 4
#define ILLEGAL_AHB_ACC          1 << 5
#define INDAC_WATERMARK_HIT      1 << 6
#define INDAC_RD_RAM_FULL        1 << 12
#define RX_CRC_DATA_ERR          1 << 16
#define ECC_ERR_FRM_FLASH        1 << 19
///////////////////////////////////
//////////FSS SRC CLK Defines /////
#define PER_PLL_CLKOUT1_192Mhz          0x3         
#define CORE_PLL_CLKOUT0_400Mhz         0x4         
#define CORE_PLL_CLKOUT3_200Mhz         0x6         
#define PER_PLL_CLKOUT2_160Mhz          0x7         
///////////////////////////////////
#define FLASH_SPI_MODE_EN               0x0
#define FLASH_OSPI_STR_MODE_EN          0x1
#define FLASH_OSPI_DTR_MODE_EN          0x2

#define WIP_STATUS                      0x1
#define WEL_STATUS                      0x2
#define BP0_STATUS                      0x4
#define BP1_STATUS                      0x8
#define BP2_STATUS                      0x10
#define BP3_STATUS                      0x20


/////// FLASH  COMMANDS /////////
#ifdef MX25
        #define NO_OPCODE               0x0
        #define CMD_WREN                0x06
        #define CMD_WREN_OP2            0xF9
        #define CMD_WR_CR2              0x72
        #define CMD_WR_CR2_OP2          0x80
        #define CMD_RDID_OPCODE1        0x9F
        #define CMD_RDID_OPCODE2        0x60
        #define CMD_PG_PROG_OP1         0x12
        #define CMD_PG_PROG_OP2         0xED
        #define CMD_RD8_OP1             0xEC
        #define CMD_RD8_OP2             0x13
        #define CMD_RDSR_OP1            0x05
        #define CMD_RDSR_OP2            0xFA
        
        #define CMD_DTRD8_OP1           0xEE
        #define CMD_DTRD8_OP2           0x11

        #define CMD_RD                  0x13

#endif
#ifdef IS25
        #define NO_OPCODE               0x0
        #define CMD_WREN                0x06
        #define CMD_WR_CR2              0x81
        #define CMD_RDID_OPCODE1        0x9F
        #define CMD_PG_PROG_OP1         0x84
        #define CMD_RD8_OP1             0x13
        #define CMD_RDSR_OP1            0x05
        
        #define CMD_DTRD8_OP1           0xFD
        #define CMD_FAST_RD_OP1         0x0B

        #define CMD_RD                  0x13
        
        #define CMD_WREN_OP2            0xF9
        #define CMD_WR_CR2_OP2          0x80
        #define CMD_RDID_OPCODE2        0x60
        #define CMD_PG_PROG_OP2         0xED
        #define CMD_RD8_OP2             0x13
        #define CMD_RDSR_OP2            0xFA
        #define CMD_DTRD8_OP2           0x11
       
        #define CMD_ENTER_4B_ADDR_MODE   0xB7
        #define CMD_EXIT_4B_ADDR_MODE    0xE9

        #define CMD_4B_ADDR_OCTAL_IO_FAST_RD     0xCC
        #define CMD_4B_ADDR_OCTAL_PP             0x8E

        #define FLASH_EXTENDED_SPI_MODE_EN                   0xFF
        #define FLASH_OSPI_IS25_DDR_MODE_EN                  0xE7
        #define FLASH_OSPI_IS25_DDR_WITHOUT_DQS              0xC7

        #define B4_4KB_SUBSECTOR_ERASE          0x21
        #define B4_32KB_SUBSECTOR_ERASE         0x5C
#else
        #define NO_OPCODE               0x0
        #define CMD_WREN                0x06
        #define CMD_WREN_OP2            0xF9
        #define CMD_WR_CR2              0x72
        #define CMD_WR_CR2_OP2          0x8D
        #define CMD_RDID_OPCODE1        0x9F
        #define CMD_RDID_OPCODE2        0x60
        #define CMD_PG_PROG_OP1         0x12
        #define CMD_PG_PROG_OP2         0xED
        #define CMD_RD8_OP1             0xEC
        #define CMD_RD8_OP2             0x13
        #define CMD_RDSR_OP1            0x05
        #define CMD_RDSR_OP2            0xFA
        
        #define CMD_DTRD8_OP1           0xEE
        #define CMD_DTRD8_OP2           0x11

        #define CMD_RD                  0x13
#endif

///////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////FOTA ////////////////////////////////////////////////////////////////
#define FOTA_CPU_CLK_EN         0
#define FOTA_CPU_CLK_DIS        1

#define FOTA_CPU_EN             0
#define FOTA_CPU_DIS            1

#define MEM_ACC_EN              1
#define MEM_ACC_DIS             0

#define FOTA_IRQ_EN             1
#define FOTA_IRQ_DIS            0

#define CFG_RD_ERR              1
#define CFG_WR_ERR              1
#define DAT_RD_ERR              1
#define DAT_WR_ERR              1
#define MCU_ERR                 1
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

#define AES_KEY0         0x0F1E2D3C
#define AES_KEY1         0x4B5A6978
#define AES_KEY2         0x01234567
#define AES_KEY3         0x8FEDCBA9

















