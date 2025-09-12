/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
#include <drivers/ospi.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <drivers/hw_include/cslr.h>
#include "ospi_tuning/ospi_tuning_algo/algo_v1/ospi_phy_tuning.h"

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

#define OSPI_PHY_INIT_RD_DELAY      (1U)
#define OSPI_PHY_MAX_RD_DELAY       (4U)
#define OSPI_DLL_LOCK_TIMEOUT       (82U)
#define OSPI_DDR_SEARCH_STEP        (4U)

/* OSPI DLL lock timeout in micro seconds */
#define OSPI_DLL_LOCK_TIMEOUT_IN_US                     (500U)
/* OSPI Idle timeout in micro seconds */
#define OSPI_IDLE_TIMEOUT_IN_US                         (500U)
/* OSPI DLL loopback lock timeout in micro seconds */
#define OSPI_DLL_LOOPBACK_LOCK_TIMEOUT_IN_US            (500U)
/* OSPI read attack vector timeout in micro seconds */
#define OSPI_READ_ATTACK_VECTOR_TIMEOUT_IN_US           (500U)

/* Mid range frequency to use different tuning point parameters */
#define OSPI_PHY_TUNING_FREQ_RANGE  (166666666U)

/* This enum defines the PHY to operate in master/bypass mode.*/
#define OSPI_CFG_PHY_OP_MODE_DEFAULT           ((uint32_t) 0U)
/* Configure PHY to operate in Master mode */
#define OSPI_CFG_PHY_OP_MODE_MASTER            ((uint32_t) 1U)
/* Configure PHY to operate in Bypass mode */
#define OSPI_CFG_PHY_OP_MODE_BYPASS            ((uint32_t) 2U)

/* Value for master mode */
#define OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_MODE (0U)
/* Value for bypass mode */
#define OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_BYPASS_MODE (1U)

/* PHY operation mode based on OSPI functional clock */
#define OSPI_CFG_PHY_DLL_MODE_DEFAULT          (OSPI_LOCK_CYCLE_HALF)
/* Configure PHY to operate in Master mode */
#define OSPI_LOCK_CYCLE_FULL                   ((uint16_t) 0U)
/* Configure PHY to operate in Bypass mode */
#define OSPI_LOCK_CYCLE_HALF                   ((uint16_t) 1U)

/**
 * Number of delay elements to be inserted between phase detect flip-flops.
 * Do NOT Modify.
*/
#define PHASE_DELAY_ELEMENT         (3U)

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


typedef struct
{
    int32_t txDllLowWindowStart;
    int32_t txDllLowWindowEnd;
    int32_t txDllHighWindowStart;
    int32_t txDllHighWindowEnd;
    int32_t rxLowSearchStart;
    int32_t rxLowSearchEnd;
    int32_t rxHighSearchStart;
    int32_t rxHighSearchEnd;
    int32_t txLowSearchStart;
    int32_t txLowSearchEnd;
    int32_t txHighSearchStart;
    int32_t txHighSearchEnd;
    int32_t txDLLSearchOffset;

} OSPI_PhyTuneWindowParams;


OSPI_PhyTuneWindowParams gPhyTuneWindowParamsFixedOtp2  = {
    .txDllLowWindowStart  = 20,
    .txDllLowWindowEnd    = 30,
    .txDllHighWindowStart = 110,
    .txDllHighWindowEnd   = 55,
    .rxLowSearchStart     = 10,
    .rxLowSearchEnd       = 80,
    .txHighSearchStart    = 30,
    .rxHighSearchEnd      = 127,
    .txHighSearchEnd      = 127,
};

OSPI_PhyTuneWindowParams *gPhyTuneWindowParams = &gPhyTuneWindowParamsFixedOtp2;

OSPI_phyParams gPhyParams = {
    .radius                 = 10,
    .rxTxDllMin             = 0,
    .rxTxDllMax             = 127,
    .minReadDelay           = 1,
    .maxReadDelay           = 4,
    .minPassSize            = 100,
    .diagonalShift          = 10,
    .maxDiagonalShift       = 70,
    .numConsecutiveFail     = 5,
    .numConsecutivePass     = 10,
    .rdDelaySearchStep      = 16,
};

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

static uint32_t gReadBuf[OSPI_FLASH_ATTACK_VECTOR_SIZE/sizeof(uint32_t)] = { 0U };

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */

void OSPI_lld_phyFindRxHigh(OSPILLD_Handle hOspi, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result);
void OSPI_lld_phyFindRxLow(OSPILLD_Handle hOspi, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result);
static void OSPI_lld_phySetRdDelayTxRxDLL(OSPILLD_Handle hOspi, OSPI_PhyConfig *configPoint);



void OSPI_phyBasicConfig(OSPILLD_Handle hOspi)
{    
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

    uint32_t opMode;
    uint8_t isDtrEn;

    isDtrEn = CSL_REG32_FEXT(&pReg->CONFIG_REG,OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD);

    if(isDtrEn)
    {
        opMode   = OSPI_CFG_PHY_OP_MODE_MASTER;
    }
    else
    {
        opMode   = OSPI_CFG_PHY_OP_MODE_DEFAULT;
    }

    /* Force Half lock cycle */
    CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                   OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_LOCK_MODE_FLD,
                   OSPI_CFG_PHY_DLL_MODE_DEFAULT);

    /* Select the number of delay element to be inserted between
     * phase detect flip-flops.
     */
    CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_PHASE_DETECT_SELECTOR_FLD,
                    OSPI_PHASE_DETECT_DLL_NUM_DELAY_ELEMENT(PHASE_DELAY_ELEMENT));

    /* PHY DLL master operational mode */
    if(opMode == OSPI_CFG_PHY_OP_MODE_MASTER)
    {
        /* Configure PHY in Master operational mode */
        CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_MODE);
    }
    /* PHY DLL master operational mode */
    else if(opMode == OSPI_CFG_PHY_OP_MODE_BYPASS)
    {
        /* Configure PHY in Bypass operational mode */
        CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                    OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_BYPASS_MODE);
    }
    else /* default mode config */
    {
        if(hOspi->hOspiInit->inputClkFreq  >= 166666666U)
        {
            /* Master operational mode for OSPI clock frequency of 166MHz */
            CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                        0);
        }
        else
        {
            /* Bypass mode for OSPI clock frequencies less than 166MHz */
            CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD,
                        1);
        }
    }

}

void OSPI_lld_phyResyncDLL(OSPILLD_Handle hOspi)
{
    uint32_t startTicks, elapsedTicks = 0;

    /* Check if the handle is NULL */
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
    uint32_t idleFlag = 0;

    startTicks = hOspi->Clock_getTicks();
    /* Wait for Idle */
    while (((idleFlag == 0)  && (elapsedTicks < hOspi->Clock_usecToTicks(OSPI_IDLE_TIMEOUT_IN_US))))
    {
        idleFlag = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                 OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD);
        elapsedTicks = hOspi->Clock_getTicks() - startTicks;
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
                   0x10U);

    /* DLL out of reset */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RESET_FLD,
                   1U);

    startTicks = hOspi->Clock_getTicks();
    /* Wait DLL lock done */
    while ((CSL_REG32_FEXT(&pReg->DLL_OBSERVABLE_LOWER_REG,
        OSPI_FLASH_CFG_DLL_OBSERVABLE_LOWER_REG_DLL_OBSERVABLE_LOWER_DLL_LOCK_FLD) == 0U)
        && (elapsedTicks < hOspi->Clock_usecToTicks(OSPI_DLL_LOCK_TIMEOUT_IN_US)))
    {
            elapsedTicks = hOspi->Clock_getTicks() - startTicks;
    }

    startTicks = hOspi->Clock_getTicks();
    /* Wait DLL loopback lock done */
    while ((CSL_REG32_FEXT(&pReg->DLL_OBSERVABLE_LOWER_REG,
        OSPI_FLASH_CFG_DLL_OBSERVABLE_LOWER_REG_DLL_OBSERVABLE_LOWER_LOOPBACK_LOCK_FLD) == 0U)
        && (elapsedTicks < hOspi->Clock_usecToTicks(OSPI_DLL_LOOPBACK_LOCK_TIMEOUT_IN_US)))
    {
            elapsedTicks = hOspi->Clock_getTicks() - startTicks;
    }

    /* Resync the Slave DLLs */
    CSL_REG32_FINS(&pReg->PHY_CONFIGURATION_REG,
                   OSPI_FLASH_CFG_PHY_CONFIGURATION_REG_PHY_CONFIG_RESYNC_FLD,
                   1U);

    /* Enable the controller */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENB_SPI_FLD,
                   TRUE);
}

static void OSPI_lld_phySetRdDelayTxRxDLL(OSPILLD_Handle hOspi, OSPI_PhyConfig *configPoint)
{
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
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
    OSPI_lld_phyResyncDLL(hOspi);

}

int32_t OSPI_lld_phyReadAttackVector(OSPILLD_Handle hOspi, uint32_t offset)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    uint32_t flashDataBaseAddr = OSPI_lld_getFlashDataBaseAddr(hOspi);
    uint8_t *src = (uint8_t *)(flashDataBaseAddr + offset);
    uint8_t *dst = (uint8_t *)gReadBuf;
    uint32_t count = OSPI_FLASH_ATTACK_VECTOR_SIZE;
    uint32_t dacState;

    /* Check if the handle is NULL */
    if(NULL != hOspi)
    {
        dacState = hOspi->hOspiInit->dacEnable;
        if(dacState == FALSE)
        {
            OSPI_lld_enableDacMode(hOspi);
        }

        while(count--)
        {
            *dst++ = *src++;
        }

        if(memcmp(gReadBuf, gOspiFlashAttackVector, OSPI_FLASH_ATTACK_VECTOR_SIZE)!=0)
        {
            status = OSPI_SYSTEM_FAILURE;
        }

        /* Switch to INDAC mode if DAC was initially in disabled state */
        if(dacState == FALSE)
        {
            OSPI_lld_disableDacMode(hOspi);
        }
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_phySetAndRead(void* hOspi, uint32_t offset, OSPI_PhyConfig *configPoint)
{
    int32_t status = OSPI_SYSTEM_FAILURE;

    OSPI_lld_phySetRdDelayTxRxDLL((OSPILLD_Handle)hOspi, configPoint);
    status = OSPI_lld_phyReadAttackVector((OSPILLD_Handle)hOspi, offset);

    return status;
}

void OSPI_lld_phyFindRxLow(OSPILLD_Handle hOspi, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result)
{
    int32_t rdAttackStatus = OSPI_SYSTEM_SUCCESS;

    result->txDLL = start->txDLL;
    result->rxDLL = start->rxDLL;
    result->rdDelay = start->rdDelay;

    /* Check if the handle is NULL */
    if(NULL != hOspi)
    {
        OSPI_lld_phySetRdDelayTxRxDLL(hOspi, result);

        rdAttackStatus = OSPI_lld_phyReadAttackVector(hOspi, offset);

        while(rdAttackStatus == OSPI_SYSTEM_FAILURE)
        {
            result->rxDLL++;
            if(result->rxDLL > gPhyTuneWindowParams->rxLowSearchEnd)
            {
                result->rxDLL = 128U;
                break;
            }
            OSPI_lld_phySetRdDelayTxRxDLL(hOspi, result);
            rdAttackStatus = OSPI_lld_phyReadAttackVector(hOspi, offset);
        }
    }
    else
    {
        /* Nothing */
    }
}

void OSPI_lld_phyFindRxHigh(OSPILLD_Handle hOspi, OSPI_PhyConfig *start, uint32_t offset, OSPI_PhyConfig *result)
{
    int32_t rdAttackStatus = OSPI_SYSTEM_SUCCESS;

    result->txDLL = start->txDLL;
    result->rxDLL = start->rxDLL;
    result->rdDelay = start->rdDelay;

    /* Check if the handle is NULL */
    if(NULL != hOspi)
    {
        OSPI_lld_phySetRdDelayTxRxDLL(hOspi, result);

        rdAttackStatus = OSPI_lld_phyReadAttackVector(hOspi, offset);

        while(rdAttackStatus == OSPI_SYSTEM_FAILURE)
        {
            result->rxDLL--;
             if(result->rxDLL < gPhyTuneWindowParams->rxHighSearchStart)
            {
                result->rxDLL = 128U;
                break;
            }
            OSPI_lld_phySetRdDelayTxRxDLL(hOspi, result);
            rdAttackStatus = OSPI_lld_phyReadAttackVector(hOspi, offset);
        }
    }
    else
    {
        /* Nothing */
    }
}

void OSPI_lld_phyObserveDLL(OSPILLD_Handle hOspi)
{
    uint32_t dllLockMode;
    /* Check for the NULL handle */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        dllLockMode = CSL_REG32_FEXT(&pReg->DLL_OBSERVABLE_LOWER_REG,
                                    OSPI_FLASH_CFG_DLL_OBSERVABLE_LOWER_REG_DLL_OBSERVABLE_LOWER_LOCK_MODE_FLD);

        if(dllLockMode != 0U)
        {
            /* Put DLL into bypass mode */
            CSL_REG32_FINS(&pReg->PHY_MASTER_CONTROL_REG,
                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD, 1U);
        }
    }
    else
    {
        /* Nothing */
    }
}

void OSPI_lld_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize)
{
    /* Get the OSPI Phy Tuning data */
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

int32_t OSPI_lld_phyTuneGrapher(OSPILLD_Handle hOspi, uint32_t flashOffset, uint8_t arrays[5][128][128])
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    OSPI_PhyConfig searchPoint;
    uint32_t rdDelay;
    uint8_t rxDll, txDll;

    OSPI_enablePhy(hOspi);
    /* keep phy pipeline disabled */
    OSPI_disablePhyPipeline(hOspi);

    /* Perform the Basic PHY configuration for the OSPI controller */
    OSPI_phyBasicConfig(hOspi);

    for(rdDelay = OSPI_PHY_INIT_RD_DELAY; rdDelay <= OSPI_PHY_MAX_RD_DELAY; rdDelay++)
    {
        for(txDll = 0; txDll < 128; txDll++)
        {
            for(rxDll = 0; rxDll < 128; rxDll++)
            {
                searchPoint.rdDelay = rdDelay;
                searchPoint.rxDLL = rxDll;
                searchPoint.txDLL = txDll;

                OSPI_lld_phySetRdDelayTxRxDLL(hOspi, &searchPoint);

                status = OSPI_lld_phyReadAttackVector(hOspi, flashOffset);
                if(status == OSPI_SYSTEM_SUCCESS)
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
    OSPI_disablePhy(hOspi);

    return OSPI_SYSTEM_SUCCESS;
}

int32_t OSPI_lld_phyFindOTP2(OSPILLD_Handle handle, uint32_t flashOffset, OSPI_PhyConfig *otp)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    OSPI_PhyConfig searchPoint = {0,0,0};
    OSPI_PhyConfig topLeft = {0,0,0}, bottomRight = {0,0,0};
    OSPI_PhyConfig rxLow = {0,0,0}, rxHigh = {0,0,0};
    OSPI_PhyConfig gapLow = {0,0,0}, gapHigh = {0,0,0};
    float slope, intercept;

    /* Need to find topLeft corner. This will be a (txLow, rxHigh) point*/

    searchPoint.txDLL = gPhyTuneWindowParams->txDllLowWindowStart;

    while(searchPoint.txDLL <= gPhyTuneWindowParams->txDllLowWindowEnd)
    {
        searchPoint.rdDelay = OSPI_PHY_INIT_RD_DELAY;
        searchPoint.rxDLL = gPhyTuneWindowParams->rxHighSearchEnd;
        OSPI_lld_phyFindRxHigh(handle, &searchPoint, flashOffset, &rxHigh);

        while(rxHigh.rxDLL == 128U)
        {
            searchPoint.rdDelay++;
            if(searchPoint.rdDelay > OSPI_PHY_MAX_RD_DELAY)
            {
                if(searchPoint.txDLL >= gPhyTuneWindowParams->txDllLowWindowEnd)
                {
                    status = OSPI_SYSTEM_FAILURE;
                    return status;
                }
                break;
            }
            OSPI_lld_phyFindRxHigh(handle, &searchPoint, flashOffset, &rxHigh);
        }

        if(rxHigh.rxDLL != 128U)
        {
            break;
        }

        searchPoint.txDLL++;
    }

    topLeft = searchPoint;

    /* Need to find bottomRight corner. This will be a (txHigh, rxLow) point. */
    searchPoint.txDLL = gPhyTuneWindowParams->txDllHighWindowStart;

    while(searchPoint.txDLL >= gPhyTuneWindowParams->txDllHighWindowEnd)
    {
        searchPoint.rxDLL = 0;
        searchPoint.rdDelay = OSPI_PHY_INIT_RD_DELAY;
        OSPI_lld_phyFindRxLow(handle, &searchPoint, flashOffset, &rxLow);

        while(rxLow.rxDLL == 128U)
        {
            searchPoint.rdDelay++;
            if(searchPoint.rdDelay > OSPI_PHY_MAX_RD_DELAY)
            {
                if(searchPoint.txDLL <= gPhyTuneWindowParams->txDllHighWindowEnd)
                {
                    status = OSPI_SYSTEM_FAILURE;
                    return status;
                }
                break;
            }
            OSPI_lld_phyFindRxLow(handle, &searchPoint, flashOffset, &rxLow);
        }

        if(rxLow.rxDLL != 128U)
        {
            break;
        }

        searchPoint.txDLL--;
    }

    bottomRight = searchPoint;

    /* Find the equation of diagonal between topLeft and bottomRight */
    slope = ((float)topLeft.rxDLL-(float)bottomRight.rxDLL)/((float)topLeft.txDLL-(float)bottomRight.txDLL);
    intercept = (float)topLeft.rxDLL - slope*((float)topLeft.txDLL);

    searchPoint = topLeft;

    do
    {
        OSPI_lld_phySetRdDelayTxRxDLL(handle, &searchPoint);
        status = OSPI_lld_phyReadAttackVector(handle, flashOffset);
        searchPoint.txDLL += 1U;
        searchPoint.rxDLL = (int32_t)(slope*(float)(searchPoint.txDLL)+intercept);

    }while(status == OSPI_SYSTEM_FAILURE);

    do
    {
        OSPI_lld_phySetRdDelayTxRxDLL(handle, &searchPoint);
        status = OSPI_lld_phyReadAttackVector(handle, flashOffset);
        searchPoint.txDLL += 1U;
        searchPoint.rxDLL = (int32_t)(slope*(float)(searchPoint.txDLL)+intercept);

    }while(status == OSPI_SYSTEM_SUCCESS);

    searchPoint.txDLL -= 1U;
    searchPoint.rxDLL = (int32_t)(slope*(float)(searchPoint.txDLL)+intercept);
    gapHigh = searchPoint;

    /* If there's only one segment, put tuning point in the middle and adjust for temperature */
    if(bottomRight.rdDelay == topLeft.rdDelay)
    {
        /* Start of the metastability gap is a good approximation for the bottomRight */
        bottomRight = gapHigh;
        searchPoint.rdDelay = topLeft.rdDelay;
        searchPoint.txDLL = (topLeft.txDLL+bottomRight.txDLL)/2U;
        searchPoint.rxDLL = (topLeft.rxDLL+bottomRight.rxDLL)/2U;

        /* TODO: Temperature adjustment */
    }
    else
    {
        /* If there are two segments, find the start and end of the second one */
        searchPoint = bottomRight;

        do
        {
            OSPI_lld_phySetRdDelayTxRxDLL(handle, &searchPoint);
            status = OSPI_lld_phyReadAttackVector(handle, flashOffset);
            searchPoint.txDLL -= 1U;
            searchPoint.rxDLL = (int32_t)(slope*(float)(searchPoint.txDLL)+intercept);

        }while(status == OSPI_SYSTEM_FAILURE);

        do
        {
            OSPI_lld_phySetRdDelayTxRxDLL(handle, &searchPoint);
            status = OSPI_lld_phyReadAttackVector(handle, flashOffset);
            searchPoint.txDLL -= 1U;
            searchPoint.rxDLL = (int32_t)(slope*(float)(searchPoint.txDLL)+intercept);

        }while(status == OSPI_SYSTEM_SUCCESS);

        searchPoint.txDLL += 1U;
        searchPoint.rxDLL = (int32_t)(slope*(float)(searchPoint.txDLL)+intercept);
        gapLow = searchPoint;

        /* Place the final tuning point of the PHY in the corner furthest from the gap */
        float len1 = abs(gapLow.txDLL-bottomRight.txDLL) + abs(gapLow.rxDLL-bottomRight.rxDLL);
        float len2 = abs(gapHigh.txDLL-topLeft.txDLL) + abs(gapHigh.rxDLL-topLeft.rxDLL);

        if(len2 > len1)
        {
            searchPoint = topLeft;
            searchPoint.txDLL += (int32_t)(abs(topLeft.txDLL-gapHigh.txDLL)/2);
            searchPoint.rxDLL = (int32_t)((float)searchPoint.txDLL * slope + intercept);
        }
        else
        {
            searchPoint = bottomRight;
            searchPoint.txDLL -= (int32_t)(abs(bottomRight.txDLL-gapLow.txDLL)/2);
            searchPoint.rxDLL = (int32_t)((float)searchPoint.txDLL * slope + intercept);
        }
    }

    OSPI_lld_phySetRdDelayTxRxDLL(handle, &searchPoint);

    status = OSPI_lld_phyReadAttackVector(handle, flashOffset);

    if(status == OSPI_SYSTEM_SUCCESS)
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
int32_t OSPI_lld_phyTuneDDR(OSPILLD_Handle hOspi, uint32_t flashOffset)
{

    int32_t status = OSPI_SYSTEM_SUCCESS;
    OSPI_PhyConfig otp;
    uint8_t isDtrEn;
    OSPI_phyOps phyOps;
    phyOps.ops = OSPI_lld_phySetAndRead;
    phyOps.phyParams  = &gPhyParams;

    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)hOspi->baseAddr;

    /* Enable PHY */
    OSPI_lld_enablePhy(hOspi);
    /* keep phy pipeline disabled */
    OSPI_lld_disablePhyPipeline(hOspi);
    /* Perform the Basic PHY configuration for the OSPI controller */
    OSPI_phyBasicConfig(hOspi);

    isDtrEn = CSL_REG32_FEXT(&pReg->CONFIG_REG,OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD);

    if(isDtrEn)
    {
        /* Use the normal algorithm */
        status = OSPI_phyFindOTP4(hOspi, flashOffset, &phyOps, gPhyParams.radius, &otp);
    }
    else
    {
        gPhyTuneWindowParams = &gPhyTuneWindowParamsFixedOtp2;

       /* Use the second algorithm */
        status = OSPI_lld_phyFindOTP2(hOspi, flashOffset, &otp);
    }
    /* Configure phy for the optimal tuning point */
    OSPI_lld_phySetRdDelayTxRxDLL(hOspi, &otp);

    /* Update the phyRdDelay book-keeping. This is needed when we enable PHY later */
    hOspi->phyRdDataCapDelay = otp.rdDelay;

     /* Disable PHY */
    OSPI_lld_disablePhy(hOspi);

    return status;
}

int32_t OSPI_lld_phyTuneSDR(OSPILLD_Handle hOspi, uint32_t flashOffset)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    if(NULL != hOspi)
    {
        status = OSPI_SYSTEM_SUCCESS;
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }
    return status;
}
