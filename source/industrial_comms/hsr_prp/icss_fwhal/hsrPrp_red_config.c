/**
 * \file hsrPrp_red_config.c
 * \brief Contains HSR PRP Configuration interface routines
 *
 * \par
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 * \par
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <networking/icss_emac/icss_emac.h>

#include "hsrPrp_firmwareOffsets.h"
#include "hsrPrp_red_config.h"
#include "hsrPrp_red_nodeTable.h"
#include <drivers/hw_include/hw_types.h>

#ifdef BUILD_PRP
#if defined(HSR_PRP_RGMII)
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/prp/rgmii/icss_prp_pru0_bin.h>
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/prp/rgmii/icss_prp_pru1_bin.h>
#else
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/prp/mii/icss_prp_pru0_bin.h>
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/prp/mii/icss_prp_pru1_bin.h>
#endif
#endif

#ifdef BUILD_HSR_COMMON
#if defined(HSR_PRP_RGMII)
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/hsr/rgmii/icss_hsr_pru0_bin.h>
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/hsr/rgmii/icss_hsr_pru1_bin.h>
#else
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/hsr/mii/icss_hsr_pru0_bin.h>
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/hsr/mii/icss_hsr_pru1_bin.h>
#endif
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*name of the C struct in PRU header file*/
#define PRU0_FIRMWARE_NAME      PRU0_FIRMWARE
#define PRU1_FIRMWARE_NAME      PRU1_FIRMWARE

#ifdef ICSS_PROTOCOL_HSR
#ifdef HSR_MODE_H
#define HSR_MODE_INIT MODEH
#endif /* HSR_MODE_H */
#ifdef HSR_MODE_N
#define HSR_MODE_INIT MODEN
#endif /* HSR_MODE_N */
#ifdef HSR_MODE_T
#define HSR_MODE_INIT MODET
#endif /* HSR_MODE_T */
#ifdef HSR_MODE_U
#define HSR_MODE_INIT MODEU
#endif /* HSR_MODE_U */
#ifdef HSR_MODE_M
#define HSR_MODE_INIT MODEM
#endif /* HSR_MODE_M */
#endif /* ICSS_PROTOCOL_HSR */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Initialises PRU0 DMEM.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return None
 *
 */
static void RedPru0DmemInit(PRUICSS_Handle pruicssHandle)
{
    uint32_t reg = 0;
    uint32_t i;


    for(i = 0; i < DUPLICATE_HOST_TABLE_DMEM_SIZE; i += sizeof(reg))
    {
        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) +
                       DUPLICATE_HOST_TABLE + i)) = reg;
    }

    for(i = 0; i < DEBUG_COUNTER_DMEM_SIZE; i += sizeof(reg))
    {

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) + DBG_START + i))
            = reg;
    }
}

/**
 *
 *  \brief Initialises PRU1 DMEM.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return None
 *
 */
static void RedPru1DmemInit(PRUICSS_Handle pruicssHandle)
{
#ifdef ICSS_PROTOCOL_HSR
    uint32_t reg = 0;
    uint32_t i;

    memset((uint8_t *)((((PRUICSS_HwAttrs const *)pruicssHandle->hwAttrs)->pru1DramBase) +
                  MULTICAST_FILTER_MASK), MULTICAST_FILTER_INIT_VAL , ETHER_ADDR_LEN);

    *((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                  M_MULTICAST_TABLE_SEARCH_OP_CONTROL_BIT)) = MULTICAST_FILTER_DISABLED;

    *((uint16_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) + LRE_HSR_MODE)) =
                          HSR_MODE_INIT;

    for(i = 0; i < DUPLICATE_PORT_TABLE_DMEM_SIZE; i += sizeof(reg))
    {

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                       DUPLICATE_PORT_TABLE_PRU0 + i)) = reg;

        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                       DUPLICATE_PORT_TABLE_PRU1 + i)) = reg;
    }

#endif /* ICSS_PROTOCOL_HSR */

}

/**
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return None
 *
 */
static void RedSharedmemInit(PRUICSS_Handle pruicssHandle)
{
    uint32_t reg = 0;
    uint32_t i;

    for(i = 0; i < LRE_STATS_DMEM_SIZE; i += sizeof(reg))
    {
        *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) + LRE_START +
                       i)) = reg;
    }

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                   LRE_DUPLICATE_DISCARD)) = IEC62439_CONST_DUPLICATE_DISCARD;


    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                   LRE_TRANSPARENT_RECEPTION)) = IEC62439_CONST_TRANSPARENT_RECEPTION_REMOVE_RCT;

    *((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                  VLAN_FLTR_CTRL_BYTE)) = VLAN_FLTR_CTRL_BYTE_DFLT_VAL;

    for(i = 0; i < VLAN_FLTR_TBL_SIZE; i ++)
    {
        *((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                      VLAN_FLTR_TBL_BASE_ADDR +
                      i)) = 0;
    }
}

RED_STATUS RedGetConfiguration(RED_CONFIG *pCfg, PRUICSS_Handle pruicssHandle)
{
    if(pCfg == NULL)
    {
        return (RED_ERR);
    }


    pCfg->nodeTableSize                = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase)
                                           + NODE_TABLE_SIZE));

    pCfg->nodeTableArbitration         = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase)
                                           + NODE_TABLE_ARBITRATION));

    pCfg->duplicateHostTableSize       = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase)
                                           + DUPLICATE_HOST_TABLE_SIZE));
#ifdef ICSS_PROTOCOL_HSR

    pCfg->duplicatePortTableSize       = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase)
                                           + DUPLICATE_PORT_TABLE_SIZE));
#else /* ICSS_PROTOCOL_HSR */
    pCfg->duplicatePortTableSize     = 0; // Not used in PRP
#endif /* ICSS_PROTOCOL_HSR */

    pCfg->nodeForgetTime             = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         NODE_FORGET_TIME));

    pCfg->duplicateForgetTime        = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         DUPLI_FORGET_TIME));

    pCfg->brokenPathDifference       = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         PATH_BROKEN_NB_FRAM_DIFF));

    pCfg->duplicatePortCheckInterval = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         DUPLI_PORT_CHECK_RESO));

    pCfg->duplicateHostCheckInterval = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         DUPLI_HOST_CHECK_RESO));

    pCfg->nodeTableCheckInterval     = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         NODETABLE_CHECK_RESO));

    pCfg->supAddressHi               = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         SUP_ADDR));

    pCfg->supAddressLow              = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         SUP_ADDR_LOW));

    pCfg->hostDuplicateArbitration   = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                                         HOST_DUPLICATE_ARBITRATION));

    return (RED_OK);
}

/**
 *
 *  \brief Sets the default configuration.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return None
 *
 */
static void RedSetConfiguration(PRUICSS_Handle pruicssHandle)
{

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   NODE_TABLE_SIZE)) = NODE_TABLE_NT_MAX_ENTRIES;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   NODE_TABLE_ARBITRATION)) = MASTER_SLAVE_BUSY_BITS_CLEAR;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   DUPLICATE_HOST_TABLE_SIZE)) = DUPLICATE_HOST_TABLE_SIZE_INIT;
#ifdef ICSS_PROTOCOL_HSR

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   DUPLICATE_PORT_TABLE_SIZE)) = DUPLICATE_PORT_TABLE_SIZE_INIT;
#endif /* ICSS_PROTOCOL_HSR */

#ifdef SOC_AM65XX
    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   NODE_FORGET_TIME)) = NODE_FORGET_TIME_60000_MS + 1000;
    /*adding additional timeout as SNMP response is slower*/
#else
    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   NODE_FORGET_TIME)) = NODE_FORGET_TIME_60000_MS;
#endif

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   DUPLI_FORGET_TIME)) = DUPLICATE_FORGET_TIME_400_MS;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   DUPLI_PORT_CHECK_RESO)) = TABLE_CHECK_RESOLUTION_10_MS;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   DUPLI_HOST_CHECK_RESO)) = TABLE_CHECK_RESOLUTION_10_MS;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   NODETABLE_CHECK_RESO)) = TABLE_CHECK_RESOLUTION_10_MS;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) + SUP_ADDR)) =
                          SUP_ADDRESS_INIT_OCTETS_HIGH;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) + SUP_ADDR_LOW)) =
                          SUP_ADDRESS_INIT_OCTETS_LOW;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase) +
                   HOST_DUPLICATE_ARBITRATION)) = MASTER_SLAVE_BUSY_BITS_CLEAR;

    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                   INTR_PAC_PREV_TS_OFFSET_PRU0)) = INTR_PAC_PREV_TS_RESET_VAL;
    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) +
                   INTR_PAC_PREV_TS_OFFSET_PRU1)) = INTR_PAC_PREV_TS_RESET_VAL;
}

void RedInit(PRUICSS_Handle pruicssHandle)
{
    RedPru0DmemInit(pruicssHandle);
    RedPru1DmemInit(pruicssHandle);
    RedSharedmemInit(pruicssHandle);
    RedSetConfiguration(pruicssHandle);
}

uint8_t RedLoadFirmware(PRUICSS_Handle pruicssHandle)
{
     uint8_t firmwareLoad_done = SystemP_FAILURE;

    /*Load the firmware*/
    PRUICSS_disableCore(pruicssHandle, ICSS_EMAC_PORT_1 - 1);
    PRUICSS_disableCore(pruicssHandle, ICSS_EMAC_PORT_2 - 1);

    if(PRUICSS_writeMemory(pruicssHandle, PRUICSS_IRAM_PRU(0) , 0,
                              (uint32_t *) PRU0_FIRMWARE_NAME,
                              sizeof(PRU0_FIRMWARE_NAME)))
    {
        if(PRUICSS_writeMemory(pruicssHandle, PRUICSS_IRAM_PRU(1) , 0,
                                  (uint32_t *) PRU1_FIRMWARE_NAME,
                                  sizeof(PRU1_FIRMWARE_NAME)))
        {
            firmwareLoad_done = SystemP_SUCCESS;
        }
    }

    if(firmwareLoad_done == SystemP_SUCCESS)
    {
        PRUICSS_enableCore(pruicssHandle, ICSS_EMAC_PORT_1 - 1);
        PRUICSS_enableCore(pruicssHandle, ICSS_EMAC_PORT_2 - 1);
    }
    return (firmwareLoad_done);

}
