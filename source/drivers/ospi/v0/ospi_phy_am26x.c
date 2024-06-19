/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
 */

#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/ospi.h>
#include <drivers/ospi/v0/cslr_ospi.h>
#include <drivers/hw_include/cslr.h>

# ifndef MIN
# define MIN(x,y) \
    ({typeof (x) _x = (x);\
      typeof (y) _y = (y);\
      _x < _y ? _x : _y;})
# endif
# ifndef MAX
# define MAX(x,y) \
    ({typeof (x) _x = (x); \
      typeof (y) _y = (y); \
      _x > _y ? _x : _y; \
    })
# endif

#define OSPI_PHY_GRAPHER_INIT_RD_DELAY      (0U)
#define OSPI_PHY_GRAPHER_MAX_RD_DELAY       (4U)
#define OSPI_DLL_LOCK_TIMEOUT       (82U)
#define OSPI_DDR_SEARCH_STEP        (4U)

/* Mid range frequency to use different tuning point parameters */
#define OSPI_PHY_TUNING_FREQ_RANGE  (133333333U)


/**
 * \brief   OSPI controller master mode baud rate divisor.
 *          OSPI baud rate = master_ref_clk/BD, where BD is:
 *          0000 = /2
 *          0001 = /4
 *          0010 = /6
 *          ...
 *          1111 = /32
 */
#define CSL_OSPI_BAUD_RATE_DIVISOR(x)        (((x) - 2U) >> 1U)

/**
 * \brief   Number of delay elements to be inserted between phase detect flip-flops.
 *          One delay element = 0h
 *          Two delay element = 1h
 *          Three delay element = 2h
 *          Four delay element = 3h
 *          Five delay element = 4h
 *          Six delay element = 5h
 *          Seven delay element = 6h
 *          Eight delay element = 7h
 *
 *          Input corresponds number of delay element.
 *          Maximum delay element is 8.
 *
 */
#define OSPI_PHASE_DETECT_DLL_NUM_DELAY_ELEMENT(x)    ((uint32_t)((x) - 1))

#define OSPI_FLASH_ATTACK_VECTOR_SIZE       (128U)
#ifndef abs
    #define abs(x) ((x)>0 ? (x) : -(x))
#endif

static uint8_t gOspiFlashAttackVector[OSPI_FLASH_ATTACK_VECTOR_SIZE] =
{
 0xFEU,   // 0b11111110 @ 0x00000400 1024 bytes
 0xFF,   // 0b11111111 @ 0x00000401 1025 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000011
 0x01,   // 0b00000001 @ 0x00000402 1026 bytes
 0x01,   // 0b00000001 @ 0x00000403 1027 bytes
 0x01,   // 0b00000001 @ 0x00000404 1028 bytes
 0x01,   // 0b00000001 @ 0x00000405 1029 bytes
 0x01,   // 0b00000001 @ 0x00000406 1030 bytes
 0x00,   // 0b00000000 @ 0x00000407 1031 bytes
 0x00,   // 0b00000000 @ 0x00000408 1032 bytes
 0xFE,   // 0b11111110 @ 0x00000409 1033 bytes
 0xFE,   // 0b11111110 @ 0x0000040A 1034 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000011
 0x01,   // 0b00000001 @ 0x0000040B 1035 bytes
 0x01,   // 0b00000001 @ 0x0000040C 1036 bytes
 0x01,   // 0b00000001 @ 0x0000040D 1037 bytes
 0x01,   // 0b00000001 @ 0x0000040E 1038 bytes
 0x00,   // 0b00000000 @ 0x0000040F 1039 bytes
 0x00,   // 0b00000000 @ 0x00000410 1040 bytes
 0xFE,   // 0b11111110 @ 0x00000411 1041 bytes
 0xFE,   // 0b11111110 @ 0x00000412 1042 bytes
 0x01,   // 0b00000001 @ 0x00000413 1043 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110011
 0xFF,   // 0b11111111 @ 0x00000414 1044 bytes
 0xFF,   // 0b11111111 @ 0x00000415 1045 bytes
 0xFF,   // 0b11111111 @ 0x00000416 1046 bytes
 0xFF,   // 0b11111111 @ 0x00000417 1047 bytes
 0xFF,   // 0b11111111 @ 0x00000418 1048 bytes
 0x00,   // 0b00000000 @ 0x00000419 1049 bytes
 0x00,   // 0b00000000 @ 0x0000041A 1050 bytes
 0xFE,   // 0b11111110 @ 0x0000041B 1051 bytes
 0xFE,   // 0b11111110 @ 0x0000041C 1052 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110011
 0xFF,   // 0b11111111 @ 0x0000041D 1053 bytes
 0xFF,   // 0b11111111 @ 0x0000041E 1054 bytes
 0xFF,   // 0b11111111 @ 0x0000041F 1055 bytes
 0xFF,   // 0b11111111 @ 0x00000420 1056 bytes
 0x00,   // 0b00000000 @ 0x00000421 1057 bytes
 0x00,   // 0b00000000 @ 0x00000422 1058 bytes
 0xFE,   // 0b11111110 @ 0x00000423 1059 bytes
 0xFE,   // 0b11111110 @ 0x00000424 1060 bytes
 0xFF,   // 0b11111111 @ 0x00000425 1061 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001011
 0x01,   // 0b00000001 @ 0x00000426 1062 bytes
 0x01,   // 0b00000001 @ 0x00000427 1063 bytes
 0x01,   // 0b00000001 @ 0x00000428 1064 bytes
 0x01,   // 0b00000001 @ 0x00000429 1065 bytes
 0x01,   // 0b00000001 @ 0x0000042A 1066 bytes
 0xFE,   // 0b11111110 @ 0x0000042B 1067 bytes
 0x00,   // 0b00000000 @ 0x0000042C 1068 bytes
 0xFE,   // 0b11111110 @ 0x0000042D 1069 bytes
 0xFE,   // 0b11111110 @ 0x0000042E 1070 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001011
 0x01,   // 0b00000001 @ 0x0000042F 1071 bytes
 0x01,   // 0b00000001 @ 0x00000430 1072 bytes
 0x01,   // 0b00000001 @ 0x00000431 1073 bytes
 0x01,   // 0b00000001 @ 0x00000432 1074 bytes
 0xFE,   // 0b11111110 @ 0x00000433 1075 bytes
 0x00,   // 0b00000000 @ 0x00000434 1076 bytes
 0xFE,   // 0b11111110 @ 0x00000435 1077 bytes
 0xFE,   // 0b11111110 @ 0x00000436 1078 bytes
 0x01,   // 0b00000001 @ 0x00000437 1079 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111111011
 0xFF,   // 0b11111111 @ 0x00000438 1080 bytes
 0xFF,   // 0b11111111 @ 0x00000439 1081 bytes
 0xFF,   // 0b11111111 @ 0x0000043A 1082 bytes
 0xFF,   // 0b11111111 @ 0x0000043B 1083 bytes
 0xFF,   // 0b11111111 @ 0x0000043C 1084 bytes
 0xFE,   // 0b11111110 @ 0x0000043D 1085 bytes
 0x00,   // 0b00000000 @ 0x0000043E 1086 bytes
 0xFE,   // 0b11111110 @ 0x0000043F 1087 bytes
 0xFE,   // 0b11111110 @ 0x00000440 1088 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111111011
 0xFF,   // 0b11111111 @ 0x00000441 1089 bytes
 0xFF,   // 0b11111111 @ 0x00000442 1090 bytes
 0xFF,   // 0b11111111 @ 0x00000443 1091 bytes
 0xFF,   // 0b11111111 @ 0x00000444 1092 bytes
 0xFE,   // 0b11111110 @ 0x00000445 1093 bytes
 0x00,   // 0b00000000 @ 0x00000446 1094 bytes
 0xFE,   // 0b11111110 @ 0x00000447 1095 bytes
 0xFE,   // 0b11111110 @ 0x00000448 1096 bytes
 0xFF,   // 0b11111111 @ 0x00000449 1097 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000111
 0x01,   // 0b00000001 @ 0x0000044A 1098 bytes
 0x01,   // 0b00000001 @ 0x0000044B 1099 bytes
 0x01,   // 0b00000001 @ 0x0000044C 1100 bytes
 0x01,   // 0b00000001 @ 0x0000044D 1101 bytes
 0x01,   // 0b00000001 @ 0x0000044E 1102 bytes
 0x00,   // 0b00000000 @ 0x0000044F 1103 bytes
 0xFE,   // 0b11111110 @ 0x00000450 1104 bytes
 0xFE,   // 0b11111110 @ 0x00000451 1105 bytes
 0xFE,   // 0b11111110 @ 0x00000452 1106 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000000111
 0x01,   // 0b00000001 @ 0x00000453 1107 bytes
 0x01,   // 0b00000001 @ 0x00000454 1108 bytes
 0x01,   // 0b00000001 @ 0x00000455 1109 bytes
 0x01,   // 0b00000001 @ 0x00000456 1110 bytes
 0x00,   // 0b00000000 @ 0x00000457 1111 bytes
 0xFE,   // 0b11111110 @ 0x00000458 1112 bytes
 0xFE,   // 0b11111110 @ 0x00000459 1113 bytes
 0xFE,   // 0b11111110 @ 0x0000045A 1114 bytes
 0x01,   // 0b00000001 @ 0x0000045B 1115 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110111
 0xFF,   // 0b11111111 @ 0x0000045C 1116 bytes
 0xFF,   // 0b11111111 @ 0x0000045D 1117 bytes
 0xFF,   // 0b11111111 @ 0x0000045E 1118 bytes
 0xFF,   // 0b11111111 @ 0x0000045F 1119 bytes
 0xFF,   // 0b11111111 @ 0x00000460 1120 bytes
 0x00,   // 0b00000000 @ 0x00000461 1121 bytes
 0xFE,   // 0b11111110 @ 0x00000462 1122 bytes
 0xFE,   // 0b11111110 @ 0x00000463 1123 bytes
 0xFE,   // 0b11111110 @ 0x00000464 1124 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 111110111
 0xFF,   // 0b11111111 @ 0x00000465 1125 bytes
 0xFF,   // 0b11111111 @ 0x00000466 1126 bytes
 0xFF,   // 0b11111111 @ 0x00000467 1127 bytes
 0xFF,   // 0b11111111 @ 0x00000468 1128 bytes
 0x00,   // 0b00000000 @ 0x00000469 1129 bytes
 0xFE,   // 0b11111110 @ 0x0000046A 1130 bytes
 0xFE,   // 0b11111110 @ 0x0000046B 1131 bytes
 0xFE,   // 0b11111110 @ 0x0000046C 1132 bytes
 0xFF,   // 0b11111111 @ 0x0000046D 1133 bytes
 // Even Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001111
 0x01,   // 0b00000001 @ 0x0000046E 1134 bytes
 0x01,   // 0b00000001 @ 0x0000046F 1135 bytes
 0x01,   // 0b00000001 @ 0x00000470 1136 bytes
 0x01,   // 0b00000001 @ 0x00000471 1137 bytes
 0x01,   // 0b00000001 @ 0x00000472 1138 bytes
 0xFE,   // 0b11111110 @ 0x00000473 1139 bytes
 0xFE,   // 0b11111110 @ 0x00000474 1140 bytes
 0xFE,   // 0b11111110 @ 0x00000475 1141 bytes
 0xFE,   // 0b11111110 @ 0x00000476 1142 bytes
 // Odd  Aligned Victim bit: 0, vic time slice : 111110000, agr time slice : 000001111
 0x01,   // 0b00000001 @ 0x00000477 1143 bytes
 0x01,   // 0b00000001 @ 0x00000478 1144 bytes
 0x01,   // 0b00000001 @ 0x00000479 1145 bytes
 0x01,   // 0b00000001 @ 0x0000047A 1146 bytes
 0xFE,   // 0b11111110 @ 0x0000047B 1147 bytes
 0xFE,   // 0b11111110 @ 0x0000047C 1148 bytes
 0xFE,   // 0b11111110 @ 0x0000047D 1149 bytes
 0xFE,   // 0b11111110 @ 0x0000047E 1150 bytes
 0x01    // 0b00000001 @ 0x0000047F 1151 bytes
};

static uint32_t gReadBuf[OSPI_FLASH_ATTACK_VECTOR_SIZE/sizeof(uint32_t)] = {0U};

typedef struct
{
    int32_t txDLL;
    int32_t rxDLL;
    int32_t rdDelay;

} OSPI_PhyConfig;

void OSPI_phyBasicConfig(OSPI_Handle handle)
{
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                   OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_LOCK_MODE_FLD,
                   attrs->phyConfiguration.dllLockMode);

    /* Select the number of delay element to be inserted between
     * phase detect flip-flops.
     */
     CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                     OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_PHASE_DETECT_SELECTOR_FLD,
                     OSPI_PHASE_DETECT_DLL_NUM_DELAY_ELEMENT(attrs->phyConfiguration.phaseDelayElement));

    /* Configure PHY Control mode */
    CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                    attrs->phyConfiguration.phyControlMode);


    if(attrs->phyConfiguration.phyControlMode != OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_MODE &&
        attrs->phyConfiguration.phyControlMode != OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_BYPASS_MODE) /* default mode config */
    {
        if(attrs->inputClkFreq  >= OSPI_PHY_TUNING_FREQ_RANGE)
        {
            /* Master operational mode for OSPI clock frequency of 166MHz */
            CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_MODE);
        }
        else
        {
            /* Bypass mode for OSPI clock frequencies less than 166MHz */
            CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_BYPASS_MODE);
        }
    }

}

void OSPI_phyResyncDLL(OSPI_Handle handle)
{
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
    uint32_t idleFlag = 0;

    /* Wait for Idle */
    while (idleFlag == 0)
    {
        idleFlag = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                  OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD);
    }

    /* Disable OSPI Controller */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENB_SPI_FLD,
                   FALSE);

    /* Clear the delay line resync bit */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RESYNC_FLD,
                   0U);

    /* Reset DLL in master mode */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RESET_FLD,
                   0U);

    /* Set Initial delay for the master DLL */
    CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                   OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_INITIAL_DELAY_FLD,
                   0x10);

    /* DLL out of reset */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RESET_FLD,
                   1U);

    /* Wait DLL lock done */
    while ((CSL_REG32_FEXT(&pReg->DLL_OBSERVABLE_LOWER_REG,
            OSPI_FLASH_CFG_DLL_OBSERVABLE_LOWER_REG_DLL_OBSERVABLE_LOWER_DLL_LOCK_FLD) == 0U));
    /* Wait DLL loopback lock done */
    while ((CSL_REG32_FEXT(&pReg->DLL_OBSERVABLE_LOWER_REG,
            OSPI_FLASH_CFG_DLL_OBSERVABLE_LOWER_REG_DLL_OBSERVABLE_LOWER_LOOPBACK_LOCK_FLD) == 0U));

    /* Resync the Slave DLLs */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RESYNC_FLD,
                   1U);

    /* Enable the controller */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENB_SPI_FLD,
                   TRUE);

}

void OSPI_phySetRdDelayTxRxDLL(OSPI_Handle handle, OSPI_PhyConfig *configPoint)
{
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    uint32_t rdDelay = (uint32_t)configPoint->rdDelay;
    uint32_t txDLL   = (uint32_t)configPoint->txDLL;
    uint32_t rxDLL   = (uint32_t)configPoint->rxDLL;

    /* Set the read delay */
    CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                   rdDelay);

    /* Set the PHY rxDLL and txDLL */
    uint32_t dtrEnable = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                         OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD);

    /* Sampled on rising edge of clock */
    CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_SAMPLE_EDGE_SEL_FLD,
                   CSL_OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_SAMPLE_EDGE_SEL_FLD_MAX);

    /* If DTR is enabled, enable DQS */
    CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DQS_ENABLE_FLD,
                   dtrEnable);

    /* Set TX DLL delay */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_TX_DLL_DELAY_FLD,
                   txDLL);
    /* Set RX DLL delay */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RX_DLL_DELAY_FLD,
                   rxDLL);
    /* Re-sync DLL */
    OSPI_phyResyncDLL(handle);
}



int32_t OSPI_phyReadAttackVector(OSPI_Handle handle, uint32_t offset)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t flashDataBaseAddr = OSPI_getFlashDataBaseAddr(handle);
    uint32_t *src = (uint32_t *)(flashDataBaseAddr + offset);
    uint32_t *dst = (uint32_t *)gReadBuf;
    uint32_t count = OSPI_FLASH_ATTACK_VECTOR_SIZE;

    OSPI_enableDacMode(handle);

    for(int i=0;i<count/4;i++)
    {
        dst[i] = src[i];
        uint32_t temp = (gOspiFlashAttackVector[4*i + 3]<<24)|(gOspiFlashAttackVector[4*i + 2]<<16)|(gOspiFlashAttackVector[4*i + 1]<<8)|(gOspiFlashAttackVector[4*i + 0]<<0);
        if(dst[i] != temp)
            return SystemP_FAILURE;
    }
    return status;
}

void OSPI_phyFindRxLow(OSPI_Handle handle, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result)
{
    int32_t rdAttackStatus = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_PhyWindowParams *gPhyTuneWindowParams = (OSPI_PhyWindowParams *)&attrs->phyConfiguration.tuningWindowParams;

    result->txDLL = start->txDLL;
    result->rxDLL = start->rxDLL;
    result->rdDelay = start->rdDelay;

    OSPI_phySetRdDelayTxRxDLL(handle, result);

    rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);

    while(rdAttackStatus == SystemP_FAILURE)
    {
        result->rxDLL += gPhyTuneWindowParams->rxTxDLLSearchStep;
        if(result->rxDLL > gPhyTuneWindowParams->rxLowSearchEnd)
        {
            result->rxDLL = 128U;
            break;
        }
        OSPI_phySetRdDelayTxRxDLL(handle, result);
        rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);

    }

}

void OSPI_phyFindRxHigh(OSPI_Handle handle, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result)
{
    int32_t rdAttackStatus = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_PhyWindowParams *gPhyTuneWindowParams = (OSPI_PhyWindowParams *)&attrs->phyConfiguration.tuningWindowParams;

    result->txDLL = start->txDLL;
    result->rxDLL = start->rxDLL;
    result->rdDelay = start->rdDelay;

    OSPI_phySetRdDelayTxRxDLL(handle, result);

    rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);

    while(rdAttackStatus == SystemP_FAILURE)
    {
        result->rxDLL -= gPhyTuneWindowParams->rxTxDLLSearchStep;
        if(result->rxDLL < gPhyTuneWindowParams->rxHighSearchStart)
        {
            result->rxDLL = 128U;
            break;
        }
        OSPI_phySetRdDelayTxRxDLL(handle, result);
        rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);
    }
}

/*
 * Searches txDLL up from start until the tuning basis passes.
 * Does not look at the next rdDelay setting.  Returns txDLL=128 if fail.
 */
void OSPI_phyFindTxLow(OSPI_Handle handle, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result)
{
    int32_t rdAttackStatus = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_PhyWindowParams *gPhyTuneWindowParams = (OSPI_PhyWindowParams *)&attrs->phyConfiguration.tuningWindowParams;

    result->txDLL = start->txDLL;
    result->rxDLL = start->rxDLL;
    result->rdDelay = start->rdDelay;

    OSPI_phySetRdDelayTxRxDLL(handle, result);

    rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);

    while(rdAttackStatus == SystemP_FAILURE)
    {
        result->txDLL += gPhyTuneWindowParams->rxTxDLLSearchStep;
        if(result->txDLL > gPhyTuneWindowParams->txLowSearchEnd)
        {
            result->txDLL = 128U;
            break;
        }
        OSPI_phySetRdDelayTxRxDLL(handle, result);
        rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);
    }
}

/*
 * Searches txDLL down from start until the tuning basis passes.
 * Does not look at the next rdDelay setting.  Returns txDLL=128 if fail.
 */
void OSPI_phyFindTxHigh(OSPI_Handle handle, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result)
{
    int32_t rdAttackStatus = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_PhyWindowParams *gPhyTuneWindowParams = (OSPI_PhyWindowParams *)&attrs->phyConfiguration.tuningWindowParams;

    result->txDLL   = start->txDLL;
    result->rxDLL   = start->rxDLL;
    result->rdDelay = start->rdDelay;

    OSPI_phySetRdDelayTxRxDLL(handle, result);

    rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);

    while(rdAttackStatus == SystemP_FAILURE)
    {
        result->txDLL -= gPhyTuneWindowParams->rxTxDLLSearchStep;
        if(result->txDLL < gPhyTuneWindowParams->txHighSearchStart)
        {
            result->txDLL = 128U;
            break;
        }
        OSPI_phySetRdDelayTxRxDLL(handle, result);
        rdAttackStatus = OSPI_phyReadAttackVector(handle, offset);
    }
}

void OSPI_phyObserveDLL(OSPI_Handle handle)
{
    uint32_t dllLockMode;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    dllLockMode = CSL_REG32_FEXT(&pReg->DLL_OBSERVABLE_LOWER_REG,
                                   OSPI_FLASH_CFG_DLL_OBSERVABLE_LOWER_REG_DLL_OBSERVABLE_LOWER_LOCK_MODE_FLD);

    if(dllLockMode != 0U)
    {
        /* Put DLL into bypass mode */
        CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                      OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD, 1U);
    }
}

void OSPI_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize)
{
    *tuningData = (uint32_t)&gOspiFlashAttackVector[0];
    *tuningDataSize = OSPI_FLASH_ATTACK_VECTOR_SIZE;
}

/*
 *
  Shown below is an approximate txDLL vs rxDLL graph of a typical PHY.
  Regions P1-P2-BL and Q1-Q2-R2-TR-R1 are the passing regions.
  Each region corresponds to a different value of read data capture delay.
  The gap between the regions can move away or towards origin depending on various factors (like temperature).

  There can be just one region also. Or the orientation of the gap will be opposite.

  The tuning/calibration algorithm can work correctly only if we have a general idea of this graph.

  The grapher function blindly sweeps through all the tx and rx DLL values for rdDelays 0,1,2,3 and writes this
  data to the array passed into this function. This has to be a [4][128][128] array.

  This data can be then saved by reading the SOC memory region and saving it as binary data. Python script can be
  written to process this to give the PHY scatter graph of the particular flash.

 RX
  |
  |     R1
  |     _______________________________ TR
  |     |                               |
  |     | Q1                            |
  |   P1 \                              |
  |       \                             |
  |     |\ \                            |
  |     | \ \                           |
  |     |  \ \                          |
  |     |   \ \                         |
  |     |    \ \                        |
  |     |     \ \                       |
  |     |      \ \                      |
  |     |       \ \ Q2                  |
  |     |________\ \____________________|R2
  |     BL     P2
  |_________________________________________ TX

*/

int32_t OSPI_phyTuneGrapher(OSPI_Handle handle, uint32_t flashOffset, uint8_t arrays[4][128][128])
{
    int32_t status = SystemP_SUCCESS;
    OSPI_PhyConfig searchPoint;
    uint32_t rdDelay;
    uint8_t rxDll, txDll;

    OSPI_enablePhy(handle);
    /* keep phy pipeline disabled */
    OSPI_disablePhyPipeline(handle);

    /* Perform the Basic PHY configuration for the OSPI controller */
    OSPI_phyBasicConfig(handle);

    for(rdDelay = OSPI_PHY_GRAPHER_INIT_RD_DELAY; rdDelay < OSPI_PHY_GRAPHER_MAX_RD_DELAY; rdDelay++)
    {
        for(txDll = 0; txDll < 128; txDll++)
        {
            for(rxDll = 0; rxDll < 128; rxDll++)
            {
                searchPoint.rdDelay = rdDelay;
                searchPoint.rxDLL = rxDll;
                searchPoint.txDLL = txDll;

                OSPI_phySetRdDelayTxRxDLL(handle, &searchPoint);

                status = OSPI_phyReadAttackVector(handle, flashOffset);

                if(status == SystemP_SUCCESS)
                {
                    arrays[rdDelay][txDll][rxDll] = 1;
                }
                else
                {
                    arrays[rdDelay][txDll][rxDll] = 0;
                }
            }
        }
    }

    /* Disable PHY */
    OSPI_disablePhy(handle);

    return SystemP_SUCCESS;
}

int32_t OSPI_phyFindOTP1(OSPI_Handle handle, uint32_t flashOffset, OSPI_PhyConfig *otp)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_PhyWindowParams *gPhyTuneWindowParams = (OSPI_PhyWindowParams *)&attrs->phyConfiguration.tuningWindowParams;

    OSPI_PhyConfig searchPoint;
    OSPI_PhyConfig bottomLeft = {0,0,0}, topRight = {0,0,0};
    OSPI_PhyConfig gapLow = {0,0,0}, gapHigh = {0,0,0};
    OSPI_PhyConfig rxLow = {0,0,0}, rxHigh = {0,0,0};
    OSPI_PhyConfig txLow = {0,0,0}, txHigh = {0,0,0};
    OSPI_PhyConfig backupPoint = {0,0,0}, backupCornerPoint = {0,0,0};
    OSPI_PhyConfig sec_rxLow = {0,0,0}, sec_rxHigh = {0,0,0};
    float slope;

    /*
     * Finding RxDLL fails at some of the TxDLL values based on the HW platform.
     * A window of TxDLL values is used to find the RxDLL without errors.
     * This can increase the number of CPU cycles taken for the PHY tuning
     * in the cases where more TxDLL values need to be parsed to find a stable RxDLL.
     */

    /***************************** GOLDEN Primary Rx_Low Search **************/
    /*
     * To find the RxDLL boundaries, we fix a valid TxDLL and search through RxDLL range, rdDelay values
     * As we are not sure of a valid TxDLL we use a window of TxDLL values to find the RxDLL boundaries.
     */

    searchPoint.txDLL = gPhyTuneWindowParams->txDllLowWindowStart;

    while(searchPoint.txDLL <= gPhyTuneWindowParams->txDllLowWindowEnd)
    {
        searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMin;
        searchPoint.rxDLL = gPhyTuneWindowParams->rxLowSearchStart;
        OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &rxLow);

        while(rxLow.rxDLL == 128U)
        {
            searchPoint.rdDelay++;
            if(searchPoint.rdDelay > gPhyTuneWindowParams->rdDelayMax)
            {
                if(searchPoint.txDLL >= gPhyTuneWindowParams->txDllLowWindowEnd)
                {
                    status = SystemP_FAILURE;
                    return status;
                }
                else
                {
                    break;
                }
            }

            OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &rxLow);
        }

        if(rxLow.rxDLL != 128U)
        {
            break;
        }

        searchPoint.txDLL += gPhyTuneWindowParams->rxTxDLLSearchStep;
    }

    /***************************** GOLDEN Secondary Rx_Low Search *****************************/
    /* Search for one more rxLow at different txDll*/
    if(searchPoint.txDLL <= (gPhyTuneWindowParams->txDllLowWindowEnd - gPhyTuneWindowParams->txDLLSearchOffset))
    {
        searchPoint.txDLL += gPhyTuneWindowParams->txDLLSearchOffset;
    }
    else
    {
        searchPoint.txDLL = gPhyTuneWindowParams->txDllLowWindowEnd;
    }


    searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMin;
    searchPoint.rxDLL   = gPhyTuneWindowParams->rxLowSearchStart;

    OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &sec_rxLow);
    while(sec_rxLow.rxDLL == 128U)
    {
        searchPoint.rdDelay++;  /* For each TxDLL in the window, go through all the valid rdDelays until we find the RxLow */
        if(searchPoint.rdDelay > gPhyTuneWindowParams->rdDelayMax)
        {
            if(searchPoint.txDLL >= gPhyTuneWindowParams->txDllLowWindowEnd)
            {
                status = SystemP_FAILURE;
                return status; /* Not able to find RxLow as there is no valid TxDLL in the TxDLL window */
            }
            else
            {
                break;
            }
        }

        OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &sec_rxLow);
    }

    rxLow.rxDLL = MIN(rxLow.rxDLL, sec_rxLow.rxDLL);
    rxLow.rdDelay = MIN(rxLow.rdDelay, sec_rxLow.rdDelay);

    /*
     * Reset the search point txDLL to continue the Rx_High search
     */
    searchPoint.txDLL = rxLow.txDLL;

    /***************************** GOLDEN Primary Rx_High Search *********************/
    /*
     * To find rxHigh we use the txDLL values of rxLow
     * Start the rdDelay (Read delay) from maximum and decrement it.
     * As these are valid values and rxHigh rdDelay is always >= rxLow rdDelay
     */

    searchPoint.rxDLL = gPhyTuneWindowParams->rxHighSearchEnd;
    searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMax;
    OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &rxHigh);
    while(rxHigh.rxDLL == 128U)
    {
        searchPoint.rdDelay--;
        if(searchPoint.rdDelay < (int32_t) gPhyTuneWindowParams->rdDelayMin)
        {
            status = SystemP_FAILURE;
            break;
        }
        OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &rxHigh);
    }

    /***************************** GOLDEN Secondary Rx_High Search *********************/
    /*
     * To find Secondary rxHigh we use the txDLL + Search_offset value of rxLow
     * Start the rdDelay (Read delay) from maximum and decrement it.
     * As these are valid values and rxHigh rdDelay is always >= rxLow rdDelay
     */
    if(searchPoint.txDLL <= (gPhyTuneWindowParams->txDllLowWindowEnd - gPhyTuneWindowParams->txDLLSearchOffset))
    {

        searchPoint.txDLL += gPhyTuneWindowParams->txDLLSearchOffset;
    }
    else
    {
        searchPoint.txDLL = gPhyTuneWindowParams->txDllLowWindowEnd;
    }

    searchPoint.rxDLL = gPhyTuneWindowParams->rxHighSearchEnd;
    searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMax;


    OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &sec_rxHigh);

    while(sec_rxHigh.rxDLL == 128U)
    {
        searchPoint.rdDelay--;
        if(searchPoint.rdDelay < (int32_t)gPhyTuneWindowParams->rdDelayMin)
        {
            status = SystemP_FAILURE;
            break;
            /*
             * If we don't find a valid Secondary Rx_High, Don't return from tuning function
             * Check whether we have a valid Primary Rx_High and then take decision.
             */
        }
        OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &sec_rxHigh);
    }

    if(sec_rxHigh.rxDLL != 128U)
    {
        if(rxHigh.rxDLL == 128U)
        {
            rxHigh = sec_rxHigh;
        }
        else
        {
            if(sec_rxHigh.rxDLL > rxHigh.rxDLL)
            {
                rxHigh = sec_rxHigh;
            }
        }
    }
    else
    {
        if(rxHigh.rxDLL == 128)
        {
            status = SystemP_FAILURE;
            return status;
        }
    }
    /*
     * Check a different point if the rxLow and rxHigh are on the same rdDelay.
     * This avoids mistaking the metastability gap for an rxDLL boundary
     */
    if(rxLow.rdDelay == rxHigh.rdDelay)
    {
        /***************************** BACKUP Primary Rx_Low Search *********************/
        /*
         * Find the rxDLL boundaries using the TxDLL window at the higher end .
         * we start the window_end and decrement the TxDLL value until we find the valid point.
         */

        searchPoint.txDLL = gPhyTuneWindowParams->txDllHighWindowEnd;

        /* Find rxDLL Min */
        while(searchPoint.txDLL >= gPhyTuneWindowParams->txDllHighWindowStart)
        {
            searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMin;
            searchPoint.rxDLL = gPhyTuneWindowParams->rxLowSearchStart;
            OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &backupPoint);

            while(backupPoint.rxDLL == 128U)
            {
                searchPoint.rdDelay++;
                if(searchPoint.rdDelay > (int32_t)gPhyTuneWindowParams->rdDelayMax)
                {
                    if(searchPoint.txDLL <= gPhyTuneWindowParams->txDllHighWindowStart)
                    {
                        status = SystemP_FAILURE;
                        return status;
                    }
                    else
                    {
                        break;
                    }
                }
                OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &backupPoint);
            }

            if(backupPoint.rxDLL != (int32_t)128U)
            {
                break;
            }

            searchPoint.txDLL -= gPhyTuneWindowParams->rxTxDLLSearchStep;
        }

        /***************************** BACKUP Secondary Rx_Low Search *********************/
        /* Search for one more rxLow at different txDll*/
        if (searchPoint.txDLL >= (gPhyTuneWindowParams->txDllHighWindowStart + gPhyTuneWindowParams->txDLLSearchOffset ))
        {
            searchPoint.txDLL -= gPhyTuneWindowParams->txDLLSearchOffset;
        }
        else
        {
            searchPoint.txDLL = gPhyTuneWindowParams->txDllHighWindowStart;
        }

        searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMin;
        searchPoint.rxDLL   = gPhyTuneWindowParams->rxLowSearchStart;
        OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &sec_rxLow);
        while(sec_rxLow.rxDLL == 128U)
        {
            searchPoint.rdDelay++;  /* For each TxDLL in the window, go through all the valid rdDelays until we find the RxLow */
            if(searchPoint.rdDelay > (int32_t) gPhyTuneWindowParams->rdDelayMax)
            {
                if(searchPoint.txDLL <= gPhyTuneWindowParams->txDllHighWindowStart)
                {
                    status = SystemP_FAILURE;
                    return status; /* Not able to find RxLow as there is no valid TxDLL in the TxDLL window */
                }
                else
                {

                    break;
                }
            }
            OSPI_phyFindRxLow(handle, &searchPoint, flashOffset, &sec_rxLow);
        }

        backupPoint.rxDLL = MIN(backupPoint.rxDLL, sec_rxLow.rxDLL);
        backupPoint.rdDelay = MIN(backupPoint.rdDelay, sec_rxLow.rdDelay);

        if(backupPoint.rxDLL < rxLow.rxDLL)
        {
            rxLow = backupPoint;
        }

        /*
         * Reset the search point txDLL to continue the Rx_High search
         */
        searchPoint.txDLL = backupPoint.txDLL;

        /***************************** BACKUP Primary Rx_High Search *********************/
        /*
         * Find rxDLL Max
         * Start the rdDelay (Read delay) from maximum and decrement it.
         */

        searchPoint.rxDLL = gPhyTuneWindowParams->rxHighSearchEnd;
        searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMax;
        OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &backupPoint);

        while(backupPoint.rxDLL == 128U)
        {
            searchPoint.rdDelay--;
            if(searchPoint.rdDelay < (int32_t)gPhyTuneWindowParams->rdDelayMin)
            {
                status = SystemP_FAILURE;
                break;
            }
            OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &backupPoint);
        }

        /***************************** BACKUP Secondary Rx_High Search *********************/
        /*
         * Find rxDLL Max
         * Start the rdDelay (Read delay) from maximum and decrement it.
         */

        if (searchPoint.txDLL >= (gPhyTuneWindowParams->txDllHighWindowStart + gPhyTuneWindowParams->txDLLSearchOffset )){
            searchPoint.txDLL -= gPhyTuneWindowParams->txDLLSearchOffset;
        }
        else
        {
            searchPoint.txDLL = gPhyTuneWindowParams->txDllHighWindowStart;
        }

        searchPoint.rxDLL = gPhyTuneWindowParams->rxHighSearchEnd;
        searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMax;
        OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &sec_rxHigh);

        while(sec_rxHigh.rxDLL == 128U)
        {
            searchPoint.rdDelay--;
            if(searchPoint.rdDelay < (int32_t) gPhyTuneWindowParams->rdDelayMin)
            {
                status = SystemP_FAILURE;
                break;
            }
            OSPI_phyFindRxHigh(handle, &searchPoint, flashOffset, &sec_rxHigh);
        }

        /*
         * Compare the Primary and Secondary point
         * Pick the point which has passing maximum rxDll
        */

        if(sec_rxHigh.rxDLL != 128U)
        {
            if(backupPoint.rxDLL == 128U)
            {
                backupPoint = sec_rxHigh;
            }
            else
            {
                if(sec_rxHigh.rxDLL > backupPoint.rxDLL)
                {
                    backupPoint = sec_rxHigh;
                }
            }
        }
        else
        {
            if(backupPoint.rxDLL == 128)
            {
                status = SystemP_FAILURE;
                return status;
            }
        }

        if(backupPoint.rxDLL > rxHigh.rxDLL)
        {
            rxHigh = backupPoint;
        }
    }

    /***************************** GOLDEN Tx_Low Search *********************/
    /*
     * Look for txDLL boundaries at 1/4 of rxDLL window
     * Find txDLL Min
     */

    searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMin;
    searchPoint.rxDLL = rxLow.rxDLL+(rxHigh.rxDLL-rxLow.rxDLL)/4U;
    searchPoint.txDLL = gPhyTuneWindowParams->txLowSearchStart;
    OSPI_phyFindTxLow(handle, &searchPoint, flashOffset, &txLow);

    while(txLow.txDLL == 128U)
    {
        searchPoint.rdDelay++;
        OSPI_phyFindTxLow(handle, &searchPoint, flashOffset, &txLow);

        if(searchPoint.rdDelay > (int32_t)gPhyTuneWindowParams->rdDelayMax)
        {
            status = SystemP_FAILURE;

            return status;
        }
    }
    /***************************** GOLDEN Tx_High Search *********************/
    /*
     * Find txDLL Max
     * Start the rdDelay (Read delay) from maximum and decrement it.
     */
    searchPoint.txDLL = gPhyTuneWindowParams->txHighSearchEnd;
    searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMax;
    OSPI_phyFindTxHigh(handle, &searchPoint, flashOffset, &txHigh);

    while(txHigh.txDLL == 128U)
    {
        searchPoint.rdDelay--;
        OSPI_phyFindTxHigh(handle, &searchPoint, flashOffset, &txHigh);

        if(searchPoint.rdDelay <(int32_t) gPhyTuneWindowParams->rdDelayMin)
        {
            status = SystemP_FAILURE;
            return status;
        }
    }

    /*
     * Check a different point if the txLow and txHigh are on the same rdDelay.
     * This avoids mistaking the metastability gap for a txDLL boundary
     */
    if(txLow.rdDelay == txHigh.rdDelay)
    {

        /***************************** BACKUP Tx_Low Search *********************/
        /* Look for txDLL boundaries at 3/4 of rxDLL window */
        /* Find txDLL Min */

        searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMin;
        searchPoint.rxDLL = rxLow.rxDLL + 3U*(rxHigh.rxDLL-rxLow.rxDLL)/4U;
        searchPoint.txDLL = gPhyTuneWindowParams->txLowSearchStart;
        OSPI_phyFindTxLow(handle, &searchPoint, flashOffset, &backupPoint);
        while(backupPoint.txDLL==128U)
        {
            searchPoint.rdDelay++;
            OSPI_phyFindTxLow(handle, &searchPoint, flashOffset, &backupPoint);
            if(searchPoint.rdDelay > (int32_t)gPhyTuneWindowParams->rdDelayMax)
            {
                status = SystemP_FAILURE;
                return status;
            }
        }
        if(backupPoint.txDLL < txLow.txDLL)
        {
            txLow = backupPoint;
        }

        /***************************** BACKUP Tx_High Search *********************/
        /*
         * Find txDLL Max
         * Start the rdDelay (Read delay) from maximum and decrement it.
         */

        searchPoint.txDLL = gPhyTuneWindowParams->txHighSearchEnd;
        searchPoint.rdDelay = gPhyTuneWindowParams->rdDelayMax;
        OSPI_phyFindTxHigh(handle, &searchPoint, flashOffset, &backupPoint);
        while(backupPoint.txDLL==128U)
        {
            searchPoint.rdDelay--;
            OSPI_phyFindTxHigh(handle, &searchPoint, flashOffset, &backupPoint);
            if(searchPoint.rdDelay <(int32_t) gPhyTuneWindowParams->rdDelayMin)
            {
                status = SystemP_FAILURE;
                return status;
            }
        }
        if(backupPoint.txDLL > txHigh.txDLL)
        {
            txHigh = backupPoint;
        }
    }

    /*
     * Set bottom left and top right right corners.  These are theoretical corners. They may not actually be "good" points.
     * But the longest diagonal of the shmoo will be between these corners.
     */

    /* Bottom Left */
    bottomLeft.txDLL = txLow.txDLL;
    bottomLeft.rxDLL = rxLow.rxDLL;

    if(txLow.rdDelay <= rxLow.rdDelay)
    {
        bottomLeft.rdDelay = txLow.rdDelay;
    }
    else
    {
        bottomLeft.rdDelay = rxLow.rdDelay;
    }

    backupCornerPoint = bottomLeft;
    backupCornerPoint.txDLL += 4U;
    backupCornerPoint.rxDLL += 4U;

    OSPI_phySetRdDelayTxRxDLL(handle, &backupCornerPoint);

    status = OSPI_phyReadAttackVector(handle, flashOffset);

    if(status == SystemP_FAILURE)
    {
        backupCornerPoint.rdDelay--;
        OSPI_phySetRdDelayTxRxDLL(handle, &backupCornerPoint);
        status = OSPI_phyReadAttackVector(handle, flashOffset);
    }

    if(status == SystemP_SUCCESS)
    {
        bottomLeft.rdDelay = backupCornerPoint.rdDelay;
    }

    topRight.txDLL = txHigh.txDLL;
    topRight.rxDLL = rxHigh.rxDLL;

    if(txHigh.rdDelay > rxHigh.rdDelay)
    {
        topRight.rdDelay = txHigh.rdDelay;
    }
    else
    {
        topRight.rdDelay = rxHigh.rdDelay;
    }

    backupCornerPoint = topRight;
    backupCornerPoint.txDLL -= 4U;
    backupCornerPoint.rxDLL -= 4U;

    OSPI_phySetRdDelayTxRxDLL(handle, &backupCornerPoint);

    status = OSPI_phyReadAttackVector(handle, flashOffset);

    if(status == SystemP_FAILURE)
    {
        backupCornerPoint.rdDelay++;
        OSPI_phySetRdDelayTxRxDLL(handle, &backupCornerPoint);
        status = OSPI_phyReadAttackVector(handle, flashOffset);
    }

    if(status == SystemP_SUCCESS)
    {
        topRight.rdDelay = backupCornerPoint.rdDelay;
    }

    /* Find the equation of diagonal between topRight and bottomLeft */

    /* Slope and Intercept*/
    slope = ((float)topRight.rxDLL-(float)bottomLeft.rxDLL)/((float)topRight.txDLL-(float)bottomLeft.txDLL);
    /* Binary Search */
    OSPI_PhyConfig left, right;
    /* Search along the diagonal between corners */
    left = bottomLeft;
    right = topRight;
    searchPoint.txDLL = left.txDLL + ((right.txDLL - left.txDLL) / 2);
    searchPoint.rxDLL = left.rxDLL + ((right.rxDLL - left.rxDLL) / 2);
    searchPoint.rdDelay = left.rdDelay;

    do
    {
        OSPI_phySetRdDelayTxRxDLL(handle, &searchPoint);

        status = OSPI_phyReadAttackVector(handle, flashOffset);
        if(status == SystemP_FAILURE)
        {
            /*
            * As the read failed, we go to the lower half for finding the gap low
            */
            right.txDLL = searchPoint.txDLL;
            right.rxDLL = searchPoint.rxDLL;

            searchPoint.txDLL = left.txDLL + ((searchPoint.txDLL - left.txDLL)/2);
            searchPoint.rxDLL = left.rxDLL + ((searchPoint.rxDLL - left.rxDLL)/2);
        } else
        {
            /*
            * As the read is a success we go to the upper half for finding the gap low
            */
            left.txDLL = searchPoint.txDLL;
            left.rxDLL = searchPoint.rxDLL;

            searchPoint.txDLL = searchPoint.txDLL + ((right.txDLL - searchPoint.txDLL)/2);
            searchPoint.rxDLL = searchPoint.rxDLL + ((right.rxDLL - searchPoint.rxDLL)/2);
        }
    /* Break the loop if the window has closed. */
    } while ((right.txDLL - left.txDLL >= 2) && (right.rxDLL - left.rxDLL >= 2));

    gapLow = searchPoint;

    /* If there's only one segment, put tuning point in the middle and adjust for temperature */
    if(bottomLeft.rdDelay == topRight.rdDelay)
    {
        /* Start of the metastability gap is a good approximation for the topRight */
        topRight = gapLow;
        searchPoint.rdDelay = bottomLeft.rdDelay;
        searchPoint.txDLL = (bottomLeft.txDLL+topRight.txDLL)/2U;
        searchPoint.rxDLL = (bottomLeft.rxDLL+topRight.rxDLL)/2U;

        /* TODO: Temperature adjustment */
    }
    else
    {
        /* If there are two segments, find the start and end of the second one */
        left = bottomLeft;
        right = topRight;
        searchPoint.txDLL = left.txDLL + ((right.txDLL - left.txDLL) / 2);
        searchPoint.rxDLL = left.rxDLL + ((right.rxDLL - left.rxDLL) / 2);
        searchPoint.rdDelay = right.rdDelay;
        do{

            OSPI_phySetRdDelayTxRxDLL(handle, &searchPoint);
            status = OSPI_phyReadAttackVector(handle, flashOffset);
            if(status == SystemP_FAILURE)
            {
                /*
                * As the read failed, we go to the upper half for finding the gap high
                */
                left.txDLL = searchPoint.txDLL;
                left.rxDLL = searchPoint.rxDLL;

                searchPoint.txDLL = searchPoint.txDLL + ((right.txDLL - searchPoint.txDLL)/2);
                searchPoint.rxDLL = searchPoint.rxDLL + ((right.rxDLL - searchPoint.rxDLL)/2);
            }
            else
            {
                /*
                * As the read is a success we go to the lower half for finding the gap high
                */
                right.txDLL = searchPoint.txDLL;
                right.rxDLL = searchPoint.rxDLL;

                searchPoint.txDLL = left.txDLL + ((searchPoint.txDLL - left.txDLL)/2);
                searchPoint.rxDLL = left.rxDLL + ((searchPoint.rxDLL - left.rxDLL)/2);
            }
            /* Break the loop if the window has closed. */
        } while ((right.txDLL - left.txDLL >= 2) && (right.rxDLL - left.rxDLL >= 2));
        gapHigh = searchPoint;

        /* Place the final tuning point of the PHY in the corner furthest from the gap */
        int len1 = abs(gapLow.txDLL-bottomLeft.txDLL) + abs(gapLow.rxDLL-bottomLeft.rxDLL);
        int len2 = abs(gapHigh.txDLL-topRight.txDLL) + abs(gapHigh.rxDLL-topRight.rxDLL);

        if(len2 > len1)
        {
            searchPoint = topRight;
            searchPoint.txDLL -= 16;
            searchPoint.rxDLL -= (int32_t)((float)16*slope);
        }
        else
        {
            searchPoint = bottomLeft;
            searchPoint.txDLL += 16;
            searchPoint.rxDLL += (int32_t)((float)16*slope);
        }
    }

    OSPI_phySetRdDelayTxRxDLL(handle, &searchPoint);

    status = OSPI_phyReadAttackVector(handle, flashOffset);

    if(status == SystemP_SUCCESS)
    {
        otp->rxDLL = searchPoint.rxDLL;
        otp->txDLL = searchPoint.txDLL;
        otp->rdDelay = searchPoint.rdDelay;
    }
    else
    {
        otp->rxDLL = 0;
        otp->txDLL = 0;
        otp->rdDelay = 0;
    }

    return status;
}

int32_t OSPI_phyTuneDDR(OSPI_Handle handle, uint32_t flashOffset)
{
    int32_t status = SystemP_SUCCESS;
    OSPI_PhyConfig otp;
    OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    /* Enable PHY */
    OSPI_enablePhy(handle);
    /* keep phy pipeline disabled */
    OSPI_disablePhyPipeline(handle);
    /* Perform the Basic PHY configuration for the OSPI controller */
    OSPI_phyBasicConfig(handle);

    /* Use the normal algorithm */
    status = OSPI_phyFindOTP1(handle, flashOffset, &otp);
    /* Configure phy for the optimal tuning point */

    OSPI_phySetRdDelayTxRxDLL(handle, &otp);
    /* Update the phyRdDelay book-keeping. This is needed when we enable PHY later */

    obj->phyRdDataCapDelay = otp.rdDelay;

     /* Disable PHY */
    OSPI_disablePhy(handle);
    return status;

}

int32_t OSPI_phyTuneSDR(OSPI_Handle handle, uint32_t flashOffset)
{
    int32_t status = SystemP_SUCCESS;
    OSPI_PhyConfig otp;

    OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    /* Set Internal loopback mode */
    CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG, OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_BYPASS_FLD, TRUE);

    /* Set the baud rate div to zero. */
    CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD, 0);

    /* Enable phy mode. */
    CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD, TRUE);

    /* Disable PHY pipeline */
    CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD, FALSE);

    /* PHY DLL master operational mode */
    CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                    FALSE);

    /* Use the normal algorithm */
    status = OSPI_phyFindOTP1(handle, flashOffset, &otp);

    /* Configure phy for the optimal tuning point */
    OSPI_phySetRdDelayTxRxDLL(handle, &otp);

    /* Update the phyRdDelay book-keeping. This is needed when we enable PHY later */
    obj->phyRdDataCapDelay = otp.rdDelay;

    return status;
}
