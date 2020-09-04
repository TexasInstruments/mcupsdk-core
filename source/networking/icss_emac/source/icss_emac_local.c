/*
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
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include "icss_emac_local.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define IEP_GLOBAL_CFG_REG_VAL          (0x0551U)

/* TX Minimum Inter packet gap */
#define TX_MIN_IPG                      (0x17U)

/* Minimum frame size cutoff in RGMII mode */
#define ICSS_EMAC_RGMII_MIN_FRAME_SIZE  (0x1F)

/* Maximum frame size cutoff in RGMII mode */
#define ICSS_EMAC_RGMII_MAX_FRAME_SIZE  (0x5F1)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**Default MAC for PRU-ICSS and NDK use!*/
static uint8_t ifMAC[] = { 0x00, 0x31, 0xDE, 0x00, 0x00, 0x00};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EMAC_memInitToZero(uint32_t *addr, uint32_t size)
{
    uint32_t i;

    if (addr != NULL)
    {
        for(i = 0U; i < (size/4u); i++)
        {
            *(addr + i) = 0x00000000U;
        }
    }
}

uint8_t ICSS_EMAC_memInit(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                *pTemp;
    uint32_t                shareDataRamSize;
    uint32_t                dataRamSize;
    uint32_t                ocmcRamSize;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    shareDataRamSize = pruicssHwAttrs->sharedDramSize;
    pTemp = (uint32_t *)(pruicssHwAttrs->sharedDramBase);
    /*clear all registers, clear all Shared RAM  */
    ICSS_EMAC_memInitToZero(pTemp, shareDataRamSize);

    /*clear data ram0 */
    dataRamSize = pruicssHwAttrs->pru0DramSize;
    pTemp = (uint32_t *)pruicssHwAttrs->pru0DramBase;
    ICSS_EMAC_memInitToZero(pTemp, dataRamSize);

    /*clear data ram1*/
    dataRamSize = pruicssHwAttrs->pru1DramSize;
    pTemp = (uint32_t *)pruicssHwAttrs->pru1DramBase;
    ICSS_EMAC_memInitToZero(pTemp, dataRamSize);

    ocmcRamSize = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcSize;
    pTemp = (uint32_t *)(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr);
    /*clear all registers, clear all OCMC (64K) */
    ICSS_EMAC_memInitToZero(pTemp, ocmcRamSize);

    return 0U;
}

void ICSS_EMAC_calcPort0BufferOffset(ICSS_EMAC_Handle icssEmacHandle,
                                     uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES],
                                     uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES])
{
    uint32_t                qCount = 1U;
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);

    bufferOffsets[ICSS_EMAC_QUEUE1] = pDynamicMMap->p0Q1BufferOffset;
    for (qCount = 1U; qCount < pDynamicMMap->numQueues; qCount++)
    {
        bufferOffsets[qCount] = bufferOffsets[qCount-1U] + pDynamicMMap->rxHostQueueSize[qCount-1U] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    }
    bufferOffsets[ICSS_EMAC_COLQUEUE] = pDynamicMMap->p0ColBufferOffset;

    bdOffsets[ICSS_EMAC_QUEUE1] = pDynamicMMap->p0Q1BufferDescOffset;
    for (qCount = 1U; qCount < pDynamicMMap->numQueues; qCount++)
    {
        bdOffsets[qCount] = bdOffsets[qCount-1U] + pDynamicMMap->rxHostQueueSize[qCount - 1U] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    }
    bdOffsets[ICSS_EMAC_COLQUEUE] = pDynamicMMap->p0ColBufferDescOffset;
}

void ICSS_EMAC_calcPort1BufferOffset(ICSS_EMAC_Handle icssEmacHandle,
                                     uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES],
                                     uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES])
{
    uint32_t qCount = 0U;
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);

    /* required to calcualate base of p1Q1BufferOffset which starts at the end of p0Q8BufferOffset*/
    uint32_t p0Q1BufferOffset = pDynamicMMap->p0Q1BufferOffset;
    uint32_t p0Q2BufferOffset = p0Q1BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE1] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q3BufferOffset = p0Q2BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE2] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q4BufferOffset = p0Q3BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE3] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q5BufferOffset = p0Q4BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE4] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q6BufferOffset = p0Q5BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE5] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q7BufferOffset = p0Q6BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE6] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q8BufferOffset = p0Q7BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE7] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    uint32_t p0Q9BufferOffset = p0Q8BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE8] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q10BufferOffset = p0Q9BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE9] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q11BufferOffset = p0Q10BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE10] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q12BufferOffset = p0Q11BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE11] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q13BufferOffset = p0Q12BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE12] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q14BufferOffset = p0Q13BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE13] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q15BufferOffset = p0Q14BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE14] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q16BufferOffset = p0Q15BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE15] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    bufferOffsets[ICSS_EMAC_QUEUE1] = p0Q16BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE16] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    for (qCount = (ICSS_EMAC_QUEUE1+1U); qCount < pDynamicMMap->numQueues; qCount++)
    {
        bufferOffsets[qCount] = bufferOffsets[qCount-1U] + pDynamicMMap->txQueueSize[qCount-1U] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    }

    bufferOffsets[ICSS_EMAC_COLQUEUE] = pDynamicMMap->p0ColBufferOffset + 1536U;

    /* required to calcualate base of p1Q1BdOffset which starts at the end of  p0Q8BdOffset */
    uint32_t p0Q1BdOffset = pDynamicMMap->p0Q1BufferDescOffset;
    uint32_t p0Q2BdOffset = p0Q1BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE1] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q3BdOffset = p0Q2BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE2] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q4BdOffset = p0Q3BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE3] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q5BdOffset = p0Q4BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE4] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q6BdOffset = p0Q5BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE5] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q7BdOffset = p0Q6BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE6] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q8BdOffset = p0Q7BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE7] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;

    uint32_t p0Q9BdOffset =   p0Q8BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE8] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q10BdOffset = p0Q9BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE9] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q11BdOffset = p0Q10BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE10] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q12BdOffset = p0Q11BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE11] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q13BdOffset = p0Q12BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE12] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q14BdOffset = p0Q13BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE13] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q15BdOffset = p0Q14BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE14] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q16BdOffset = p0Q15BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE15] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;

    bdOffsets[ICSS_EMAC_QUEUE1] = p0Q16BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE16] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;


    for (qCount = (ICSS_EMAC_QUEUE1+1U); qCount < pDynamicMMap->numQueues; qCount++)
    {
        bdOffsets[qCount] = bdOffsets[qCount-1U] + pDynamicMMap->txQueueSize[qCount-1U] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    }

    bdOffsets[ICSS_EMAC_COLQUEUE] = pDynamicMMap->p0ColBufferDescOffset + ICSS_EMAC_DEFAULT_FW_BD_SIZE * 48U ;
}

void ICSS_EMAC_calcPort2BufferOffset(ICSS_EMAC_Handle icssEmacHandle,
                                     uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES],
                                     uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES])
{
    uint32_t qCount = 0U;
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);

    /* required to calcualate base of p1Q1BufferOffset which starts at the end of   p0Q8BufferOffset*/
    uint32_t p0Q1BufferOffset = pDynamicMMap->p0Q1BufferOffset;
    uint32_t p0Q2BufferOffset = p0Q1BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE1] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q3BufferOffset = p0Q2BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE2] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q4BufferOffset = p0Q3BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE3] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q5BufferOffset = p0Q4BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE4] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q6BufferOffset = p0Q5BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE5] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q7BufferOffset = p0Q6BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE6] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q8BufferOffset = p0Q7BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE7] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    uint32_t p0Q9BufferOffset = p0Q8BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE8] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q10BufferOffset = p0Q9BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE9] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q11BufferOffset = p0Q10BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE10] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q12BufferOffset = p0Q11BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE11] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q13BufferOffset = p0Q12BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE12] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q14BufferOffset = p0Q13BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE13] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q15BufferOffset = p0Q14BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE14] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p0Q16BufferOffset = p0Q15BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE15] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    /* required to calcualate base of p2Q1Offset which starts at the end of  p1Q8BufferOffset*/
    uint32_t p1Q1BufferOffset = p0Q16BufferOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE16] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q2BufferOffset = p1Q1BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE1] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q3BufferOffset = p1Q2BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE2] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q4BufferOffset = p1Q3BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE3] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q5BufferOffset = p1Q4BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE4] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q6BufferOffset = p1Q5BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE5] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q7BufferOffset = p1Q6BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE6] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q8BufferOffset = p1Q7BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE7] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    uint32_t p1Q9BufferOffset = p1Q8BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE8] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q10BufferOffset = p1Q9BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE9] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q11BufferOffset = p1Q10BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE10] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q12BufferOffset = p1Q11BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE11] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q13BufferOffset = p1Q12BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE12] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q14BufferOffset = p1Q13BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE13] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q15BufferOffset = p1Q14BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE14] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    uint32_t p1Q16BufferOffset = p1Q15BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE15] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;

    bufferOffsets[ICSS_EMAC_QUEUE1] = p1Q16BufferOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE16] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    for (qCount = (ICSS_EMAC_QUEUE1+1U); qCount < pDynamicMMap->numQueues; qCount++)
    {
        bufferOffsets[qCount] = bufferOffsets[qCount-1U] + pDynamicMMap->txQueueSize[qCount-1U] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    }

    bufferOffsets[ICSS_EMAC_COLQUEUE] = pDynamicMMap->p0ColBufferOffset + (1536U*2U);

    /* required to calcualate base of p2Q1BdOffset */
    uint32_t p0Q1BdOffset = pDynamicMMap->p0Q1BufferDescOffset;
    uint32_t p0Q2BdOffset = p0Q1BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE1] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q3BdOffset = p0Q2BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE2] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q4BdOffset = p0Q3BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE3] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q5BdOffset = p0Q4BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE4] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q6BdOffset = p0Q5BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE5] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q7BdOffset = p0Q6BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE6] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q8BdOffset = p0Q7BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE7] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;

    uint32_t p0Q9BdOffset = p0Q8BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE8] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q10BdOffset = p0Q9BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE9] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q11BdOffset = p0Q10BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE10] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q12BdOffset = p0Q11BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE11] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q13BdOffset = p0Q12BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE12] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q14BdOffset = p0Q13BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE13] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q15BdOffset = p0Q14BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE14] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p0Q16BdOffset = p0Q15BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE15] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;

    uint32_t p1Q1BdOffset = p0Q16BdOffset + pDynamicMMap->rxHostQueueSize[ICSS_EMAC_QUEUE16] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q2BdOffset = p1Q1BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE1] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q3BdOffset = p1Q2BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE2] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q4BdOffset = p1Q3BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE3] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q5BdOffset = p1Q4BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE4] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q6BdOffset = p1Q5BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE5] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q7BdOffset = p1Q6BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE6] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q8BdOffset = p1Q7BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE7] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;

    uint32_t p1Q9BdOffset = p1Q8BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE8] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q10BdOffset = p1Q9BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE9] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q11BdOffset = p1Q10BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE10] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q12BdOffset = p1Q11BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE11] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q13BdOffset = p1Q12BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE12] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q14BdOffset = p1Q13BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE13] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q15BdOffset = p1Q14BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE14] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    uint32_t p1Q16BdOffset = p1Q15BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE15] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;

    uint32_t p1ColBDOffset = pDynamicMMap->p0ColBufferDescOffset + ICSS_EMAC_DEFAULT_FW_BD_SIZE * 48U;

    bdOffsets[ICSS_EMAC_QUEUE1] = p1Q16BdOffset + pDynamicMMap->txQueueSize[ICSS_EMAC_QUEUE16] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    for (qCount = (ICSS_EMAC_QUEUE1+1U); qCount < pDynamicMMap->numQueues; qCount++)
    {
        bdOffsets[qCount] = bdOffsets[qCount-1U] + pDynamicMMap->txQueueSize[qCount-1U] * ICSS_EMAC_DEFAULT_FW_BD_SIZE;
    }

    bdOffsets[ICSS_EMAC_COLQUEUE] = p1ColBDOffset + ICSS_EMAC_DEFAULT_FW_BD_SIZE * 48U;
}

void ICSS_EMAC_clearStatistics(ICSS_EMAC_Handle icssEmacHandle)
{
    ICSS_EMAC_PortParams *sPort;

    uint32_t i, j;
    for (j=0U; j<3U; j++)
    {
        sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[j]);
        sPort->errCount = 0U;
        sPort->rawCount = 0U;
        for (i=0U; i<ICSS_EMAC_NUMQUEUES; i++)
        {
            sPort->queue[i].qStat.errCount = 0U;
            sPort->queue[i].qStat.rawCount = 0U;
        }
    }
}

int32_t ICSS_EMAC_portInit(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                qCount = 0U;
    ICSS_EMAC_PortParams    *sPort;
    uint32_t                bufferOffsets[ICSS_EMAC_NUMQUEUES];
    uint32_t                bdOffsets[ICSS_EMAC_NUMQUEUES];
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);

    /* Clear counters */
    ICSS_EMAC_clearStatistics(icssEmacHandle);
    /* Initialize port 0*/
    sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);
    ICSS_EMAC_calcPort0BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
    {
        sPort->queue[qCount].buffer_offset      = bufferOffsets[qCount];
        sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
        sPort->queue[qCount].queue_desc_offset  = pStaticMMap->p0QueueDescOffset + (qCount *8U);
        sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->rxHostQueueSize[qCount]) << 2U) + (uint16_t)bdOffsets[qCount];        /* really the end of Queue */
    }

    /* Initialize port 1*/
    sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_1]);
    ICSS_EMAC_calcPort1BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
    {
        sPort->queue[qCount].buffer_offset      = bufferOffsets[qCount];
        sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
        sPort->queue[qCount].queue_desc_offset  = pStaticMMap->p0QueueDescOffset + (32U + qCount * 8U);
        sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->txQueueSize[qCount]) << 2U) +  (uint16_t)bdOffsets[qCount];
    }

    /*Collision Queue */
    sPort->queue[ICSS_EMAC_COLQUEUE].buffer_offset      = bufferOffsets[ICSS_EMAC_COLQUEUE];
    sPort->queue[ICSS_EMAC_COLQUEUE].buffer_desc_offset = bdOffsets[ICSS_EMAC_COLQUEUE];
    sPort->queue[ICSS_EMAC_COLQUEUE].queue_desc_offset  = pStaticMMap->p0ColQueueDescOffset + 8U;
    sPort->queue[ICSS_EMAC_COLQUEUE].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->collisionQueueSize) << 2U) + (uint16_t)bdOffsets[ICSS_EMAC_COLQUEUE];

    /* Initialize port 2*/
    sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_2]);
    ICSS_EMAC_calcPort2BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
    {
        sPort->queue[qCount].buffer_offset      = bufferOffsets[qCount];
        sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
        sPort->queue[qCount].queue_desc_offset  = pStaticMMap->p0QueueDescOffset + (64U + qCount * 8U);
        sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->txQueueSize[qCount]) << 2U) +  (uint16_t)bdOffsets[qCount];
    }

    /*Collision Queue */
    sPort->queue[ICSS_EMAC_COLQUEUE].buffer_offset      = bufferOffsets[ICSS_EMAC_COLQUEUE];
    sPort->queue[ICSS_EMAC_COLQUEUE].buffer_desc_offset = bdOffsets[ICSS_EMAC_COLQUEUE];
    sPort->queue[ICSS_EMAC_COLQUEUE].queue_desc_offset  = pStaticMMap->p0ColQueueDescOffset + 16U;
    sPort->queue[ICSS_EMAC_COLQUEUE].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->collisionQueueSize) << 2U) +  (uint16_t)bdOffsets[ICSS_EMAC_COLQUEUE];

    return 0;
}

uint8_t ICSS_EMAC_switchConfig(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                qCount = 0U;
    uint16_t                *pTemp16;
    uint8_t                 *pTemp8;
    int32_t                 i;
    ICSS_EMAC_IoctlCmd      ioctlParams;
    uint8_t                 portVal=0;

    uint8_t                 *charPtr1;
    uint8_t                 *charPtr2;

    uint32_t                bufferOffsetsPort0[ICSS_EMAC_NUMQUEUES];
    uint32_t                bdOffsetsPort0[ICSS_EMAC_NUMQUEUES];
    uint32_t                bufferOffsetsPort1[ICSS_EMAC_NUMQUEUES];
    uint32_t                bdOffsetsPort1[ICSS_EMAC_NUMQUEUES];
    uint32_t                bufferOffsetsPort2[ICSS_EMAC_NUMQUEUES];
    uint32_t                bdOffsetsPort2[ICSS_EMAC_NUMQUEUES];

    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    ICSS_EMAC_calcPort0BufferOffset(icssEmacHandle, bufferOffsetsPort0, bdOffsetsPort0);

    uint32_t dataRAM0BaseAddr = pruicssHwAttrs->pru0DramBase;
    uint32_t dataRAM1BaseAddr = pruicssHwAttrs->pru1DramBase;

    uint32_t temp = pruicssHwAttrs->pru1DramBase;

    uint32_t temp_addr = 0U;

    /* queue lookup table */
    temp_addr = (temp + pDynamicMMap->queueSizeOffset);
    pTemp16 = (uint16_t *)(temp_addr);
    /* host (port 2) queue */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(pDynamicMMap->rxHostQueueSize[qCount]);
        pTemp16++;
    }
    /* port 0 queue */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(pDynamicMMap->txQueueSize[qCount]);
        pTemp16++;
    }

    /* port 1 queue */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(pDynamicMMap->txQueueSize[qCount]);
        pTemp16++;
    }

    /********************** */
    /* Tx Context Initialize data. Port 1, (Q1,Q2,Q3,Q4,Qn) */
    ICSS_EMAC_calcPort1BufferOffset(icssEmacHandle, bufferOffsetsPort1, bdOffsetsPort1);
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 1 Queue 1 - Queue N (numQueues)*/
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort1[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 32U) + bufferOffsetsPort1[qCount] - 32U);
        pTemp16++;
        *pTemp16 =  (uint16_t)(bdOffsetsPort1[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 4U) + bdOffsetsPort1[qCount] - 4U);
        pTemp16++;
    }

    /********************** */
    /* Tx Context Initialize data. Port 2, (Q1,Q2,Q3,Q4,Qn) */
    /********************** */
    ICSS_EMAC_calcPort2BufferOffset(icssEmacHandle, bufferOffsetsPort2, bdOffsetsPort2);
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 32U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 2 Queue 1 - Queue N (numQueues)*/
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort2[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 32U) + bufferOffsetsPort2[qCount] - 32U);
        pTemp16++;
        *pTemp16 = (uint16_t)(bdOffsetsPort2[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 4U) + bdOffsetsPort2[qCount] - 4U);
        pTemp16++;
    }

    /********************** */
    /* Collision Tx Context Initialize data. Port 1, (Q1) */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 64U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 1 collision queue */
    *pTemp16 = (uint16_t)(bufferOffsetsPort1[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)(bufferOffsetsPort1[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)((pDynamicMMap->collisionQueueSize * 32U) + bufferOffsetsPort1[ICSS_EMAC_COLQUEUE] - 32U);
    pTemp16++;

    /********************** */
    /* Collision Tx Context Initialize data. Port 2, (Q1) */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 72U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 2 collision queue */
    *pTemp16 = (uint16_t)(bufferOffsetsPort2[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)(bufferOffsetsPort2[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)((pDynamicMMap->collisionQueueSize * 32U) + bufferOffsetsPort2[ICSS_EMAC_COLQUEUE] - 32U);
    pTemp16++;

    /********************** */
    /* Rx Context Initialize data for Host Port, (Q1,Q2,Q3,Q4,Qn) */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 80U);
    pTemp16 = (uint16_t *)(temp_addr);
   /* Host Port 1 Queue 1 - Queue N (numQueues)*/
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort0[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)(pStaticMMap->p0QueueDescOffset + (qCount * 8U));
        pTemp16++;
        *pTemp16 = (uint16_t)(bdOffsetsPort0[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->rxHostQueueSize[qCount] * 4U) + bdOffsetsPort0[qCount] - 4U);
        pTemp16++;
    }
    /********************** */
    /* Rx Context Initialize data for Port 1, (Q1,Q2,Q3,Q4,Qn) */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 112U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Host Port 1 Queue 1  - Queue N (numQueues)*/
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort1[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)(pStaticMMap->p0QueueDescOffset + (32U + (qCount * 8U)));
        pTemp16++;
        *pTemp16 = (uint16_t)(bdOffsetsPort1[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 4U) +  bdOffsetsPort1[qCount] - 4U);
        pTemp16++;
    }

    /********************** */
    /* Rx Context Initialize data for Port 2, (Q1,Q2,Q3,Q4,Qn) */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 144U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Host Port 2 Queue 1  - Queue N (numQueues)*/
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort2[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)(pStaticMMap->p0QueueDescOffset + (64U + (qCount * 8U)));
        pTemp16++;
        *pTemp16 = (uint16_t)(bdOffsetsPort2[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 4U) + bdOffsetsPort2[qCount] - 4U);
        pTemp16++;
    }

    /********************** */
    /* Collision Rx Context Initialize for Host port */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 176U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 0 collision queue */
    *pTemp16 = (uint16_t)(bufferOffsetsPort0[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)(bufferOffsetsPort0[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)(pStaticMMap->p0ColQueueDescOffset);
    pTemp16++;
    *pTemp16 = (uint16_t)(bdOffsetsPort0[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)((pDynamicMMap->collisionQueueSize * 4U) + bdOffsetsPort0[ICSS_EMAC_COLQUEUE] - 4U);
    pTemp16++;

    /********************** */
    /* Collision Rx Context Initialize for Port 1 */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 188U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 1 collision queue */
    *pTemp16 =  (uint16_t)(bufferOffsetsPort1[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 =  (uint16_t)(bufferOffsetsPort1[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)(pStaticMMap->p0ColQueueDescOffset + 8U);
    pTemp16++;
    *pTemp16 = (uint16_t)(bdOffsetsPort1[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)((pDynamicMMap->collisionQueueSize * 4U) + bdOffsetsPort1[ICSS_EMAC_COLQUEUE] - 4U);
    pTemp16++;

    /********************** */
    /* Collision Rx Context Initialize for Port 2 */
    /********************** */
    temp_addr = (temp + pDynamicMMap->p1Q1SwitchTxContextOffset + 200U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 2 collision queue */
    *pTemp16 =  (uint16_t)(bufferOffsetsPort2[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 =  (uint16_t)(bufferOffsetsPort2[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)(pStaticMMap->p0ColQueueDescOffset + 16U);
    pTemp16++;
    *pTemp16 = (uint16_t)(bdOffsetsPort2[ICSS_EMAC_COLQUEUE]);
    pTemp16++;
    *pTemp16 = (uint16_t)((pDynamicMMap->collisionQueueSize * 4U) + bdOffsetsPort2[ICSS_EMAC_COLQUEUE] - 4U);
    pTemp16++;

    /********************** */
    /* buffer offset table */
    /********************** */
    temp_addr = (temp + pDynamicMMap->queueOffset);
    pTemp16 = (uint16_t *)(temp_addr);
    /* host queue buffer */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort0[qCount]);
        pTemp16++;
    }

    /* port 1 queue */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort1[qCount]);
        pTemp16++;
    }
    /* port 2 queue */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bufferOffsetsPort2[qCount]);
        pTemp16++;
    }

    /************************** */
    /* buffer descriptor */
    /************************** */
    temp_addr = (temp + pDynamicMMap->queueDescriptorOffset);
    pTemp16 = (uint16_t *)(temp_addr);
    /* host buffer descriptor */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bdOffsetsPort0[qCount]);
        pTemp16++;
    }

    /* port 0 buffer descriptor */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bdOffsetsPort1[qCount]);
        pTemp16++;
    }
    /* port 1 buffer descriptor */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)(bdOffsetsPort2[qCount]);
        pTemp16++;
    }

    temp_addr = (dataRAM0BaseAddr + pStaticMMap->portMacAddr);
    charPtr1 = (uint8_t *)(temp_addr);
    temp_addr = (dataRAM1BaseAddr + pStaticMMap->portMacAddr);
    charPtr2 = (uint8_t *)(temp_addr);
    temp_addr = (dataRAM1BaseAddr + pStaticMMap->interfaceMacAddrOffset);
    pTemp8 = (uint8_t *)(temp_addr);

    for (i=0; i<6; i++)
    {
        ifMAC[i] = (((ICSS_EMAC_Object *)icssEmacHandle->object)->macId[i]);
        *charPtr1 = ifMAC[i];
        charPtr1++;
        *charPtr2 = ifMAC[i];
        charPtr2++;
        *pTemp8 = ifMAC[i];
        pTemp8++;
    }
    temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->p0ColQueueDescOffset);
    pTemp16 = (uint16_t *)(temp_addr);

    *pTemp16 = (uint16_t)bdOffsetsPort0[ICSS_EMAC_COLQUEUE];
    pTemp16++;
    *pTemp16 = (uint16_t)bdOffsetsPort0[ICSS_EMAC_COLQUEUE];
    pTemp16++;
    *pTemp16 = 0x0000;
    pTemp16++;
    *pTemp16 = 0x0000;
    pTemp16++;
    *pTemp16 = (uint16_t)bdOffsetsPort1[ICSS_EMAC_COLQUEUE];
    pTemp16++;
    *pTemp16 = (uint16_t)bdOffsetsPort1[ICSS_EMAC_COLQUEUE];
    pTemp16++;
    *pTemp16 = 0x0000;
    pTemp16++;
    *pTemp16 = 0x0000;
    pTemp16++;
    *pTemp16 = (uint16_t)bdOffsetsPort2[ICSS_EMAC_COLQUEUE];
    pTemp16++;
    *pTemp16 = (uint16_t)bdOffsetsPort2[ICSS_EMAC_COLQUEUE];
    pTemp16++;
    *pTemp16 = 0x0000;
    pTemp16++;
    *pTemp16 = 0x0000;
    pTemp16++;

    /* Port 0 */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)bdOffsetsPort0[qCount];
        pTemp16++;
        *pTemp16 = (uint16_t)bdOffsetsPort0[qCount];
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
    }

    /*Port1 */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)bdOffsetsPort1[qCount];
        pTemp16++;
        *pTemp16 = (uint16_t)bdOffsetsPort1[qCount];
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
    }
    /*Port2  */
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
    {
        *pTemp16 = (uint16_t)bdOffsetsPort2[qCount];
        pTemp16++;
        *pTemp16 = (uint16_t)bdOffsetsPort2[qCount];
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
    }

    portVal = ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE;
    ioctlParams.ioctlVal = &portVal;
    ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_1, (void*)&ioctlParams);
    ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_2, (void*)&ioctlParams);

    ICSS_EMAC_portInit(icssEmacHandle);

    return 0U;
}

static void ICSS_EMAC_pruicssCfgInit(ICSS_EMAC_Handle icssEmacHandle)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    /* Selecting MII-RT mode in GPCFG mux */
    PRUICSS_setGpMuxSelect(pruicssHandle, PRUICSS_PRU0, PRUICSS_GP_MUX_SEL_MODE_MII);
    PRUICSS_setGpMuxSelect(pruicssHandle, PRUICSS_PRU1, PRUICSS_GP_MUX_SEL_MODE_MII);

    PRUICSS_setGpiMode(pruicssHandle, PRUICSS_PRU0, PRUICSS_GPI_MODE_MII_RT);
    PRUICSS_setGpiMode(pruicssHandle, PRUICSS_PRU1, PRUICSS_GPI_MODE_MII_RT);

    /* Select ocp_clk for lower IEP Latency */
    PRUICSS_setIepClkSrc(pruicssHandle, 1U);

    HW_WR_FIELD32((pruicssHwAttrs->cfgRegBase) + CSL_ICSS_PR1_CFG_SLV_MII_RT_REG, CSL_ICSS_PR1_CFG_SLV_MII_RT_REG_MII_RT_EVENT_EN, 1);

    HW_WR_FIELD32((pruicssHwAttrs->cfgRegBase) + CSL_ICSS_PR1_CFG_SLV_SPP_REG, CSL_ICSS_PR1_CFG_SLV_SPP_REG_XFR_SHIFT_EN, 1);
}

static void ICSS_EMAC_pruicssMiiRtCfgInit(ICSS_EMAC_Handle icssEmacHandle)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                  CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_IPG_WIRE_CLK_EN0, 0x1);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                  CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_IPG_WIRE_CLK_EN1, 0x1);

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1,
                  CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG1_TX_IPG1, TX_MIN_IPG);
    /* Note that it is important to update TX_IPG1 before TX_IPG0 as in WIRE_CLK mode TX_IPG0 write is required to load the IPG
     * value to HW */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0,
                  CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TX_IPG0_TX_IPG0 , TX_MIN_IPG);

    /* Configuration of Port 0 */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0_RX_ENABLE0, 1);


    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0_RX_DATA_RDY_MODE_DIS0, 1);

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0_RX_MUX_SEL0, 0);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0_RX_L2_EN0, 1);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0_RX_CUT_PREAMBLE0, 1);

    /* Enable the clearing of RX_EOF from PRU R31 command */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0,
        CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG0_RX_EOF_SCLR_DIS0, 1);

    /* Enable the output FIFO and set a delay of 0x0 in nibbles, route TX back to Port0 */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_ENABLE0, 1);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_AUTO_PREAMBLE0, 1);

    /* Enable the 32 bit insert in Tx FIFO */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
        CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_32_MODE_EN0, 1);

    /* Need this change for tx pin swap in mii mode for AM64x, AM243x and AM263x only*/
    if( (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyToMacInterfaceMode == ICSS_EMAC_MII_MODE) && (((((ICSS_EMAC_Object *)icssEmacHandle->object)->icssRevision) == 0x103) || ((((ICSS_EMAC_Object *)icssEmacHandle->object)->icssRevision) == 0x203)) )
    {
        if(ICSS_EMAC_MODE_SWITCH == (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->portMask)
        { /*Switch mode*/
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_MUX_SEL0, 0x0);
        }
        else
        {
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_MUX_SEL0, 0x1);
        }
    }
    else
    {
        if(ICSS_EMAC_MODE_SWITCH == (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->portMask)
        { /*Switch mode*/
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_MUX_SEL0, 0x1);
        }
        else
        {
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
                CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_MUX_SEL0, 0x0);
        }
    }

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG0_TX_START_DELAY0, 0x0);

    /* Configuration of Port 1 */

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1_RX_ENABLE1, 1);


    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
        CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1_RX_DATA_RDY_MODE_DIS1, 1);

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1_RX_MUX_SEL1, 1);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1_RX_L2_EN1, 1);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1_RX_CUT_PREAMBLE1, 1);

    /* Enable the clearing of RX_EOF from PRU R31 command */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1,
        CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_RXCFG1_RX_EOF_SCLR_DIS1, 1);


    /* Enable the output FIFO and set a delay of 0x0 in nibbles, route TX back to Port0 */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_ENABLE1, 1);
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_AUTO_PREAMBLE1, 1);


    /* Enable the 32 bit insert in Tx FIFO */
    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
        CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_32_MODE_EN1, 1);


    /* Need this change for tx pin swap in mii mode for AM64x, AM243x and AM263x only*/
    if( (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyToMacInterfaceMode == ICSS_EMAC_MII_MODE) && (((((ICSS_EMAC_Object *)icssEmacHandle->object)->icssRevision) == 0x103) || ((((ICSS_EMAC_Object *)icssEmacHandle->object)->icssRevision) == 0x203)) )
    {
        if(ICSS_EMAC_MODE_SWITCH == (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->portMask)
        { /*Switch mode*/
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_MUX_SEL1, 0x1);
        }
        else
        {
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_MUX_SEL1, 0x0);
        }
    }
    else
    {
        if(ICSS_EMAC_MODE_SWITCH == (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->portMask)
        { /*Switch mode*/
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_MUX_SEL1, 0x0);
        }
        else
        {
            HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
                    CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_MUX_SEL1, 0x1);
        }
    }

    HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1,
            CSL_ICSS_PR1_MII_RT_PR1_MII_RT_CFG_TXCFG1_TX_START_DELAY1, 0x0);

    if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyToMacInterfaceMode == ICSS_EMAC_MII_MODE)
    {
        PRUICSS_setIcssCfgTxFifo(pruicssHandle, PRUICSS_TX_L1_FIFO, 0);
        PRUICSS_setIcssCfgTxFifo(pruicssHandle, PRUICSS_TX_L2_FIFO, 1);

        PRUICSS_setIcssCfgMiiMode(pruicssHandle, 0, PRUICSS_ICSS_CFG_MII_MODE_MII);
        PRUICSS_setIcssCfgMiiMode(pruicssHandle, 1, PRUICSS_ICSS_CFG_MII_MODE_MII);
    }
    else if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyToMacInterfaceMode == ICSS_EMAC_RGMII_MODE)
    {
        PRUICSS_setIcssCfgTxFifo(pruicssHandle, PRUICSS_TX_L1_FIFO, 1);
        PRUICSS_setIcssCfgTxFifo(pruicssHandle, PRUICSS_TX_L2_FIFO, 1);

        PRUICSS_setIcssCfgMiiMode(pruicssHandle, 0, PRUICSS_ICSS_CFG_MII_MODE_RGMII);
        PRUICSS_setIcssCfgMiiMode(pruicssHandle, 1, PRUICSS_ICSS_CFG_MII_MODE_RGMII);

        /*FIXME : Move this code to SoC specific files and remove ifdef*/

#if defined(SOC_AM64X) || defined (SOC_AM243X)
        /* Disable RGMII gigabit mode */
        HW_WR_FIELD32((pruicssHwAttrs->miiGRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG,
            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_GIG_IN, 0x0);
        HW_WR_FIELD32((pruicssHwAttrs->miiGRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG,
            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_GIG_IN, 0x0);

        /* RGMII full duplex override */
        HW_WR_FIELD32((pruicssHwAttrs->miiGRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG,
            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_FULLDUPLEX_IN, 0x1);
        HW_WR_FIELD32((pruicssHwAttrs->miiGRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG,
            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_FULLDUPLEX_IN, 0x1);

        /* Enable RGMII Inband */
        HW_WR_FIELD32((pruicssHwAttrs->miiGRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG,
            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII0_INBAND, 0x1);
        HW_WR_FIELD32((pruicssHwAttrs->miiGRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG,
            CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_RGMII_CFG_RGMII1_INBAND, 0x1);

        /* Setting min and max frame size */
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0,
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0_RX_MIN_FRM0, ICSS_EMAC_RGMII_MIN_FRAME_SIZE);
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0,
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0_RX_MAX_FRM0, ICSS_EMAC_RGMII_MAX_FRAME_SIZE);

        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1,
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1_RX_MIN_FRM1, ICSS_EMAC_RGMII_MIN_FRAME_SIZE);
        HW_WR_FIELD32((pruicssHwAttrs->miiRtCfgRegBase) + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1,
                CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1_RX_MAX_FRM1, ICSS_EMAC_RGMII_MAX_FRAME_SIZE);
#endif
    }

}

int8_t ICSS_EMAC_switchInit(ICSS_EMAC_Handle            icssEmacHandle,
                            const PRUICSS_IntcInitData  *pruicssIntcInitData)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                temp_addr = 0U;

    PRUICSS_disableCore(pruicssHandle, PRUICSS_PRU1);
    PRUICSS_disableCore(pruicssHandle, PRUICSS_PRU0);

    ICSS_EMAC_memInit(icssEmacHandle);
    ICSS_EMAC_switchConfig(icssEmacHandle);

    temp_addr = (pruicssHwAttrs->pru0CtrlRegBase + CSL_ICSS_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0);
    /* Set in constant table C28 to ICSS Shared memory 0x10000 */
    HW_WR_REG32(temp_addr, (pruicssHwAttrs->sharedDramBase & 0x0001FFFFU) >> 8U);
    temp_addr = (pruicssHwAttrs->pru1CtrlRegBase + CSL_ICSS_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0);
    HW_WR_REG32(temp_addr, (pruicssHwAttrs->sharedDramBase & 0x0001FFFFU) >> 8U);

    temp_addr = (pruicssHwAttrs->pru0CtrlRegBase + CSL_ICSS_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1);
    /* Set in constant table C30 to shared RAM 0x40300000 */
    HW_WR_REG32(temp_addr, (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr & 0x00FFFF00U) >> 8U);
    temp_addr = (pruicssHwAttrs->pru1CtrlRegBase + CSL_ICSS_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_1);
    HW_WR_REG32(temp_addr, (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr & 0x00FFFF00U) >> 8U);

    /* configure PRUICSS register CFG */
    ICSS_EMAC_pruicssCfgInit(icssEmacHandle);

    /* configure PRUICSS INTC */
    PRUICSS_intcInit(pruicssHandle, pruicssIntcInitData);

    /* configure PRUICSS register MII_RT */
    ICSS_EMAC_pruicssMiiRtCfgInit(icssEmacHandle);

    /* Enable IEP Counter */
    temp_addr = (pruicssHwAttrs->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    HW_WR_REG16(temp_addr, IEP_GLOBAL_CFG_REG_VAL);

    PRUICSS_disableCore(pruicssHandle, PRUICSS_PRU0);
    PRUICSS_disableCore(pruicssHandle, PRUICSS_PRU1);

    return 0;
}

void ICSS_EMAC_mdioIntrEnableSwitch(uint8_t             portNum,
                                    ICSS_EMAC_Handle    icssEmacHandle)
{
    uint32_t                phySel;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                baseAddr = pruicssHwAttrs->miiMdioRegBase + CSL_MDIO_USER_PHY_SEL_REG((uint32_t)portNum - (uint32_t)1U);
    uint32_t                phyAddr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[0];

    /* TODO: Use MDIO API directly */
    phySel = phyAddr;
    phySel |=  0x40U;

    HW_WR_REG32(baseAddr,phySel);

    baseAddr = pruicssHwAttrs->miiMdioRegBase + CSL_MDIO_USER_PHY_SEL_REG(portNum);
    phyAddr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[1];

    phySel = phyAddr;
    phySel |=  0x40U;

    HW_WR_REG32(baseAddr,phySel);
}

uint32_t ICSS_EMAC_calcTotalBufferPoolSize(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                qCount = 0U;
    uint32_t                hostBufferPoolSize = 0U;
    uint32_t                txPortBufferPoolSize = 0U;
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);

    for (qCount = 0U; qCount < pDynamicMMap->numQueues;qCount++)
    {
        hostBufferPoolSize += pDynamicMMap->rxHostQueueSize[qCount] * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
        hostBufferPoolSize  += pDynamicMMap->txQueueSize[qCount] * 2U * ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE;
    }

    return (hostBufferPoolSize + txPortBufferPoolSize);
}

uint8_t ICSS_EMAC_hostMemInit(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                totalBufferPoolSize;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    /*clear all registers, clear all Shared RAM */
    uint32_t sharedDataRamSize =  pruicssHwAttrs->sharedDramSize;
    uint32_t *pTemp = (uint32_t *)(pruicssHwAttrs->sharedDramBase);
    ICSS_EMAC_memInitToZero(pTemp, sharedDataRamSize);

    /*clear all registers, clear all OCMC region for  buffer pools */
    totalBufferPoolSize = ICSS_EMAC_calcTotalBufferPoolSize(icssEmacHandle);
    pTemp = (uint32_t *)(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr);
    ICSS_EMAC_memInitToZero(pTemp, totalBufferPoolSize);

    return 0U;
}

uint8_t ICSS_EMAC_hostConfig(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                qCount = 0U;
    uint16_t                *pTemp16;
    uint32_t                temp_addr = 0U;
    uint32_t                bufferOffsets[ICSS_EMAC_NUMQUEUES];
    uint32_t                bdOffsets[ICSS_EMAC_NUMQUEUES];
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                hostQDescOffset = pDynamicMMap->hostQ1RxContextOffset + 64U;
    uint32_t                hostQOffsetAddr = pDynamicMMap->hostQ1RxContextOffset + 40U;
    ICSS_EMAC_PortParams    *sPort;

    /* queue lookup table */
    temp_addr = (pruicssHwAttrs->sharedDramBase + pDynamicMMap->queueSizeOffset);
    pTemp16 = (uint16_t *)(temp_addr);
    /* host (port 2) queue */
    for (qCount = 0; qCount < pDynamicMMap->numQueues; qCount++)
    {
        *pTemp16 = (uint16_t)(pDynamicMMap->rxHostQueueSize[qCount]);
        pTemp16++;
     }

    /********************** */
    /* Rx Context Initialize data for Host Port, (Q1,Q2,Q3,Q4) */
    /********************** */
    ICSS_EMAC_calcPort0BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);
    temp_addr = (pruicssHwAttrs->sharedDramBase + pDynamicMMap->hostQ1RxContextOffset);
    pTemp16 = (uint16_t *)(temp_addr);
    for (qCount = 0; qCount < pDynamicMMap->numQueues; qCount++)
    {
        *pTemp16 = (uint16_t)(bufferOffsets[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)(hostQDescOffset + (qCount * 8U));
        pTemp16++;
        *pTemp16 = (uint16_t)(bdOffsets[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)((pDynamicMMap->rxHostQueueSize[qCount] * 4U) + bdOffsets[qCount] - 4U);
        pTemp16++;
    }

    /********************** */
    /* buffer offset table */
    /********************** */
    temp_addr = (pruicssHwAttrs->sharedDramBase + hostQOffsetAddr);
    pTemp16 = (uint16_t *)(temp_addr);
    /* host queue buffer */
    for (qCount = 0; qCount < pDynamicMMap->numQueues; qCount++)
    {
        *pTemp16 = (uint16_t)(bufferOffsets[qCount]);
        pTemp16++;
    }

    /************************** */
    /* buffer descriptor */
    /************************** */
    temp_addr = (pruicssHwAttrs->sharedDramBase + pDynamicMMap->hostQ1RxContextOffset + 32U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* host buffer descriptor */
    for (qCount = 0; qCount < pDynamicMMap->numQueues; qCount++)
    {
        *pTemp16 = (uint16_t)(bdOffsets[qCount]);
        pTemp16++;
    }

    temp_addr = (pruicssHwAttrs->sharedDramBase + pDynamicMMap->hostQ1RxContextOffset + 64U);
    pTemp16 = (uint16_t *)(temp_addr);
    /* Port 0 */
    for (qCount = 0; qCount < pDynamicMMap->numQueues; qCount++)
    {
        *pTemp16 = (uint16_t)(bdOffsets[qCount]);
        pTemp16++;
        *pTemp16 = (uint16_t)(bdOffsets[qCount]);
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
        *pTemp16 = 0x0000;
        pTemp16++;
    }

    qCount = 0U;
    hostQDescOffset = pDynamicMMap->hostQ1RxContextOffset + 64U;

    ICSS_EMAC_calcPort0BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);

    /* Initialize port 0*/
    sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);
    for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
    {
        sPort->queue[qCount].buffer_offset      = bufferOffsets[qCount];
        sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
        sPort->queue[qCount].queue_desc_offset  = hostQDescOffset + (qCount * 8U);
        sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->rxHostQueueSize[qCount]) << 2) + (uint16_t)bdOffsets[qCount];        /* really the end of Queue */
    }

    return 0U;
}

void ICSS_EMAC_hostInit(ICSS_EMAC_Handle icssEmacHandle,
                        const PRUICSS_IntcInitData *pruicssIntcInitData)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                temp_addr = 0U;

    ICSS_EMAC_hostMemInit(icssEmacHandle);
    ICSS_EMAC_hostConfig(icssEmacHandle);

    /* configure PRUICSS register CFG */
    ICSS_EMAC_pruicssCfgInit(icssEmacHandle);

    /* configure PRUICSS INTC */
    PRUICSS_intcInit(pruicssHandle, pruicssIntcInitData);

    /* configure PRUICSS register MII_RT */
    ICSS_EMAC_pruicssMiiRtCfgInit(icssEmacHandle);

    /* Enable IEP Counter */
    temp_addr = (pruicssHwAttrs->iep0RegBase + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    HW_WR_REG16(temp_addr, IEP_GLOBAL_CFG_REG_VAL);
}

void ICSS_EMAC_portMemInit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                *pTemp = NULL;
    uint32_t                dataRamSize = 0;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        dataRamSize = pruicssHwAttrs->pru0DramSize;
        pTemp = (uint32_t *)pruicssHwAttrs->pru0DramBase;
    }
    if((uint8_t)ICSS_EMAC_PORT_2 == portNum)
     {
        dataRamSize = pruicssHwAttrs->pru1DramSize;
        pTemp = (uint32_t *)pruicssHwAttrs->pru1DramBase;
    }

    /*clear Data Ram*/
    ICSS_EMAC_memInitToZero(pTemp, dataRamSize);
}

uint8_t ICSS_EMAC_macConfig(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    ICSS_EMAC_IoctlCmd      ioctlParams;
    uint8_t                 portVal = 0;
    uint16_t                *pTemp16;
    uint8_t                 *pTemp8=NULL;
    uint8_t                 i;
    uint32_t                bufferOffsets[ICSS_EMAC_NUMQUEUES];
    uint32_t                bdOffsets[ICSS_EMAC_NUMQUEUES];
    uint32_t                qCount = 0;
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                dataRAM0BaseAddr = (uint32_t)(pruicssHwAttrs->pru0DramBase);
    uint32_t                dataRAM1BaseAddr = (uint32_t)(pruicssHwAttrs->pru1DramBase);
    ICSS_EMAC_PortParams    *sPort;

    uint32_t temp_addr = 0U;

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        temp_addr = (dataRAM0BaseAddr + pStaticMMap->portMacAddr);
        pTemp8 = (uint8_t *)(temp_addr);
    }

    if((uint8_t)ICSS_EMAC_PORT_2 == portNum)
    {
        temp_addr = (dataRAM1BaseAddr + pStaticMMap->portMacAddr);
        pTemp8 = (uint8_t *)(temp_addr);
    }

    for (i=0; i<6U; i++)
    {
        ifMAC[i] = (((ICSS_EMAC_Object *)icssEmacHandle->object)->macId[i]);
        if(pTemp8 != NULL)
        {
            *pTemp8 = ifMAC[i];
            pTemp8++;
        }
    }

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        /********************** */
        /* Tx Context Initialize data. Port 1, (Q1,Q2,Q3,Q4 Qn) */
        ICSS_EMAC_calcPort1BufferOffset(icssEmacHandle, bufferOffsets,bdOffsets);
        /********************** */
        temp_addr = (dataRAM0BaseAddr + pDynamicMMap->q1EmacTxContextOffset);
        pTemp16 = (uint16_t *)(temp_addr);

        /* Port 1 Queue 1 - Queue N (numQueues)*/
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
        {
            *pTemp16 = (uint16_t)(bufferOffsets[qCount]);
            pTemp16++;
            *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 32U) + bufferOffsets[qCount]- 32U);
            pTemp16++;
            *pTemp16 = (uint16_t)(bdOffsets[qCount]);
            pTemp16++;
            *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 4U) + bdOffsets[qCount] - 4U);
            pTemp16++;
        }

        temp_addr = (dataRAM0BaseAddr + pDynamicMMap->portQueueDescOffset);
        pTemp16 = (uint16_t*)(temp_addr);
        /*Port1 */
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
        {
            *pTemp16 = (uint16_t)(bdOffsets[qCount]);
            pTemp16++;
            *pTemp16 = (uint16_t)(bdOffsets[qCount]);
            pTemp16++;
            *pTemp16 = 0x0000;
            pTemp16++;
            *pTemp16 = 0x0000;
            pTemp16++;
        }
        portVal = ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE;
        ioctlParams.ioctlVal = &portVal;
        ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_1, (void*)&ioctlParams);

    }

    if((uint8_t)ICSS_EMAC_PORT_2 == portNum)
    {
        /********************** */
        /* Tx Context Initialize data. Port 2, (Q1,Q2,Q3,Q4,Qn) */
        /********************** */
        ICSS_EMAC_calcPort2BufferOffset(icssEmacHandle, bufferOffsets,bdOffsets);
        temp_addr = (dataRAM1BaseAddr + pDynamicMMap->q1EmacTxContextOffset);
        pTemp16 = (uint16_t *)(temp_addr);
        /* Port 2Queue 1 - Queue N (numQueues)*/
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
        {
            *pTemp16 = (uint16_t)(bufferOffsets[qCount]);
            pTemp16++;
            *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 32U) + bufferOffsets[qCount] - 32U);
            pTemp16++;
            *pTemp16 = (uint16_t)(bdOffsets[qCount]);
            pTemp16++;
            *pTemp16 = (uint16_t)((pDynamicMMap->txQueueSize[qCount] * 4U) + bdOffsets[qCount] - 4U);
            pTemp16++;
        }

        temp_addr = (dataRAM1BaseAddr + pDynamicMMap->portQueueDescOffset);
        pTemp16 = (uint16_t *)(temp_addr);
        /*Port2 */
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount ++)
        {
            *pTemp16 = (uint16_t)(bdOffsets[qCount]);
            pTemp16++;
            *pTemp16 = (uint16_t)(bdOffsets[qCount]);
            pTemp16++;
            *pTemp16 = 0x0000;
            pTemp16++;
            *pTemp16 = 0x0000;
            pTemp16++;
        }

        portVal = ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE;
        ioctlParams.ioctlVal = &portVal;
        ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_2, (void*)&ioctlParams);
    }

    qCount = 0U;

    if(((uint8_t)(ICSS_EMAC_PORT_1)) == portNum)
    {
        /* Initialize port 1*/
        sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_1]);
        ICSS_EMAC_calcPort1BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
        {
            sPort->queue[qCount].buffer_offset       = bufferOffsets[qCount];
            sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
            sPort->queue[qCount].queue_desc_offset  = pDynamicMMap->portQueueDescOffset + (qCount * 8U);
            sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->txQueueSize[qCount]) << 2U) + (uint16_t)bdOffsets[qCount];
        }
    }

    if(((uint8_t)(ICSS_EMAC_PORT_2)) == portNum)
    {
        /* Initialize port 2*/
        sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_2]);
        ICSS_EMAC_calcPort2BufferOffset(icssEmacHandle, bufferOffsets, bdOffsets);
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
        {
            sPort->queue[qCount].buffer_offset       = bufferOffsets[qCount];
            sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
            sPort->queue[qCount].queue_desc_offset  = pDynamicMMap->portQueueDescOffset + (qCount * 8U);
            sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->txQueueSize[qCount]) << 2U) + (uint16_t)bdOffsets[qCount];
        }
    }

    return 0U;
}

/*TODO: Review this function*/
int8_t ICSS_EMAC_macInit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                temp_addr = 0U;
    uint32_t                sharedDataRamBase;
    uint32_t                l3OcmcBase;

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        PRUICSS_disableCore(pruicssHandle, PRUICSS_PRU0);
    }
    else if((uint8_t)ICSS_EMAC_PORT_2 == portNum)
    {
        PRUICSS_disableCore(pruicssHandle, PRUICSS_PRU1);
    }

    ICSS_EMAC_portMemInit(portNum, icssEmacHandle);

    ICSS_EMAC_macConfig(portNum, icssEmacHandle);

    sharedDataRamBase = pruicssHwAttrs->sharedDramBase;
    l3OcmcBase = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr;

    if((uint8_t)ICSS_EMAC_PORT_1 == portNum)
    {
        temp_addr = (pruicssHwAttrs->pru0CtrlRegBase + CSL_ICSS_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0);
        /* Set in constant table C28 to ICSS Shared memory 0x10000 */
        HW_WR_REG32(temp_addr, (sharedDataRamBase & 0x000FFFFFU) >> 8U);
        temp_addr = (pruicssHwAttrs->pru0CtrlRegBase + CSL_ICSS_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1);
        /* Set in constant table C30 to shared RAM 0x40300000 */
        HW_WR_REG32(temp_addr, (l3OcmcBase & 0x00FFFF00U) >> 8U);
    }

    if((uint8_t)ICSS_EMAC_PORT_2 == portNum)
    {
        temp_addr = (pruicssHwAttrs->pru1CtrlRegBase + CSL_ICSS_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0);
        HW_WR_REG32(temp_addr, (sharedDataRamBase & 0x000FFFFFU) >> 8U);
        temp_addr = (pruicssHwAttrs->pru1CtrlRegBase + CSL_ICSS_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_1);
        HW_WR_REG32(temp_addr, (l3OcmcBase & 0x00FFFF00U) >> 8U);
    }

    /*TODO: Review if following is needed*/
    temp_addr = (pruicssHwAttrs->pru0CtrlRegBase + CSL_ICSS_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0);
    HW_WR_REG32(temp_addr, 0x00000100U);
    temp_addr = (pruicssHwAttrs->pru1CtrlRegBase + CSL_ICSS_PR1_PDSP1_IRAM_CONSTANT_TABLE_PROG_PTR_0);
    HW_WR_REG32(temp_addr, 0x00000100U);

    return 0;
}

void ICSS_EMAC_mdioIntrEnable(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                phySel;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                baseAddr = pruicssHwAttrs->miiMdioRegBase + CSL_MDIO_USER_PHY_SEL_REG((uint32_t)portNum - (uint32_t)1U) ;
    uint32_t                phyAddr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[0];

    /* TODO: Use MDIO API directly */
    phySel = phyAddr;
    phySel |=  0x40U;

    HW_WR_REG32(baseAddr,phySel);
}

void ICSS_EMAC_initLinkState(ICSS_EMAC_Handle   icssEmacHandle,
                            uint8_t             interfaceId,
                            uint8_t             portNo)
{
    int32_t                 retVal = SystemP_FAILURE;
    uint32_t                linkStatus;
    ICSS_EMAC_IoctlCmd      ioctlParams;
    uint8_t                 portVal = 0U;
    volatile uint8_t        *portStatusPtr = NULL;
    uint32_t                temp_addr = 0U;
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = NULL;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    pStaticMMap = &((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap;

    linkStatus = 0;
    retVal = MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[interfaceId]);
    if(retVal == SystemP_SUCCESS)
        linkStatus = 1;

    if(linkStatus)
    {
        portVal = ICSS_EMAC_IOCTL_PORT_CTRL_ENABLE;
        ioctlParams.ioctlVal = &portVal;
        if (portNo == ICSS_EMAC_PORT_1)
        {
            ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, portNo, &ioctlParams);
            temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->portStatusOffset);
        }
        else if (portNo == ICSS_EMAC_PORT_2)
        {
            ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, portNo, &ioctlParams);
            temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->portStatusOffset);
        }
        /* switch use case */
        else
        {
            /* ICSS_EMAC_PORT_1 of switch */
            if (interfaceId ==0U)
            {
                ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, ICSS_EMAC_PORT_1, &ioctlParams);
                temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->portStatusOffset);
            }
            /* ICSS_EMAC_PORT_2 of switch */
            else
            {
                ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, ICSS_EMAC_PORT_2, &ioctlParams);
                temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->portStatusOffset);
            }
        }
        portStatusPtr = (uint8_t*)(temp_addr);
        *(portStatusPtr) = ICSS_EMAC_PORT_LINK_MASK;

        ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[interfaceId] = 1U;
        ((ICSS_EMAC_Object *)icssEmacHandle->object)->prevlinkStatus[interfaceId] = 0U;
    }
    else
    {
        ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[interfaceId] = 0U;
        ((ICSS_EMAC_Object *)icssEmacHandle->object)->prevlinkStatus[interfaceId] = 0U;
    }
}

/*TODO: Review this function*/
int32_t ICSS_EMAC_osInit(ICSS_EMAC_Handle icssEmacHandle)
{
    int32_t         retVal = SystemP_FAILURE;
    int32_t         retValSemLink = SystemP_FAILURE;
    int32_t         retValSemRx = SystemP_FAILURE;
    int32_t         retValSemTx = SystemP_FAILURE;
    int32_t         retValHwiLink = SystemP_FAILURE;
    int32_t         retValHwiRx = SystemP_FAILURE;
    int32_t         retValHwiTx = SystemP_FAILURE;
    int32_t         retValTaskLink = SystemP_FAILURE;
    int32_t         retValTaskRx = SystemP_FAILURE;
    int32_t         retValTaskTx = SystemP_FAILURE;
    HwiP_Params     hwiParams;
    TaskP_Params    taskParams;

    retValSemLink = SemaphoreP_constructBinary(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkSemaphoreObject), 0);

    if(SystemP_SUCCESS == retValSemLink)
    {
        retValSemRx = SemaphoreP_constructBinary(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxSemaphoreObject), 0);
    }

    if(SystemP_SUCCESS == retValSemRx)
    {
        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->linkIntNum;
        hwiParams.callback = (HwiP_FxnCallback)ICSS_EMAC_linkISR;
        hwiParams.args = (void *)(icssEmacHandle);
        /* TODO: Review if we need a  priority */
        // hwiParams.pri = 0x1;
        retValHwiLink = HwiP_construct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkInterruptObject), &hwiParams);
    }

    if(SystemP_SUCCESS == retValHwiLink)
    {
        HwiP_Params_init(&hwiParams);
        hwiParams.intNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->rxIntNum;
        hwiParams.callback = (HwiP_FxnCallback)ICSS_EMAC_rxInterruptHandler;
        hwiParams.args = (void *)(icssEmacHandle);
        /* TODO: Review if we need a  priority */
        // hwiParams.pri = 0x1;
        retValHwiRx = HwiP_construct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxInterruptObject), &hwiParams);
    }

    if(SystemP_SUCCESS == retValHwiRx)
    {
        TaskP_Params_init(&taskParams);
        taskParams.name = "LinkTask";
        taskParams.stackSize = LINK_TASK_STACK_SIZE;
        taskParams.stack = (uint8_t *)&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkTaskStack);
        taskParams.priority = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->linkTaskPriority;
        taskParams.args = (void *)icssEmacHandle;
        taskParams.taskMain = (TaskP_FxnMain)ICSS_EMAC_osLinkTaskFnc;
        retValTaskLink = TaskP_construct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkTaskObject), &taskParams);
    }

    if(SystemP_SUCCESS == retValTaskLink)
    {
        TaskP_Params_init(&taskParams);
        taskParams.name = "RxTask";
        taskParams.stackSize = RX_TASK_STACK_SIZE;
        taskParams.stack = (uint8_t *)&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxTaskStack);
        taskParams.priority = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->rxTaskPriority;
        taskParams.args = (void *)icssEmacHandle;
        taskParams.taskMain = (TaskP_FxnMain)ICSS_EMAC_osRxTaskFnc;
        retValTaskRx = TaskP_construct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxTaskObject), &taskParams);
    }

    retVal = retValTaskRx;

    if((1 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->txInterruptEnable) && (SystemP_SUCCESS == retVal))
    {
        retValSemTx = SemaphoreP_constructBinary(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txSemaphoreObject), 0);

        if(SystemP_SUCCESS == retValSemTx)
        {
            HwiP_Params_init(&hwiParams);
            hwiParams.intNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->txIntNum;
            hwiParams.callback = (HwiP_FxnCallback)ICSS_EMAC_txInterruptHandler;
            hwiParams.args = (void *)(icssEmacHandle);
            /* TODO: Review if we need a  priority */
            // hwiParams.pri = 0x1;
            retValHwiTx = HwiP_construct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txInterruptObject), &hwiParams);
        }

        if(SystemP_SUCCESS == retValHwiTx)
        {
            TaskP_Params_init(&taskParams);
            taskParams.name = "TxTask";
            taskParams.stackSize = TX_TASK_STACK_SIZE;
            taskParams.stack = (uint8_t *)&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txTaskStack);
            taskParams.priority = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->txTaskPriority;
            taskParams.args = (void *)icssEmacHandle;
            taskParams.taskMain = (TaskP_FxnMain)ICSS_EMAC_osTxTaskFnc;
            retValTaskTx = TaskP_construct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txTaskObject), &taskParams);
        }

        retVal = retValTaskTx;
    }

    if(SystemP_FAILURE == retVal)
    {
        if(SystemP_SUCCESS == retValSemLink)
        {
            SemaphoreP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkSemaphoreObject));
        }

        if(SystemP_SUCCESS == retValSemRx)
        {
            SemaphoreP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxSemaphoreObject));
        }

        if(SystemP_SUCCESS == retValSemTx)
        {
            SemaphoreP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txSemaphoreObject));
        }

        if(SystemP_SUCCESS == retValHwiLink)
        {
            HwiP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkInterruptObject));
        }

        if(SystemP_SUCCESS == retValHwiRx)
        {
            HwiP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxInterruptObject));
        }

        if(SystemP_SUCCESS == retValHwiTx)
        {
            HwiP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txInterruptObject));
        }

        if(SystemP_SUCCESS == retValTaskLink)
        {
            TaskP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkTaskObject));
        }

        if(SystemP_SUCCESS == retValTaskRx)
        {
            TaskP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxTaskObject));
        }

        if(SystemP_SUCCESS == retValTaskTx)
        {
            TaskP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txTaskObject));
        }

    }

    return retVal;
}

void ICSS_EMAC_mdioIntrDisableSwitch(uint8_t portNum,
                                     ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                phySel;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                baseAddr = pruicssHwAttrs->miiMdioRegBase + CSL_MDIO_USER_PHY_SEL_REG((uint32_t)portNum - (uint32_t)1U);
    uint32_t                phyAddr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[0];

    /* TODO: Add MDIO API for this*/
    phySel=phyAddr;
    phySel &=  ((uint8_t)~(0x40U));
    HW_WR_REG32(baseAddr,phySel);

    baseAddr = pruicssHwAttrs->miiMdioRegBase + CSL_MDIO_USER_PHY_SEL_REG(portNum);
    phyAddr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[1];

    phySel=phyAddr;
    phySel &=  ((uint8_t)~(0x40U));
    HW_WR_REG32(baseAddr,phySel);
}

void ICSS_EMAC_mdioIntrDisable(uint8_t portNum,
                               ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                phySel;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint32_t                baseAddr = pruicssHwAttrs->miiMdioRegBase + CSL_MDIO_USER_PHY_SEL_REG((uint32_t)portNum - (uint32_t)1U);
    uint32_t                phyAddr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs))->phyAddr[0];

    /* TODO: Add MDIO API for this*/
    phySel=phyAddr;
    phySel &=  ((uint8_t)~(0x40U));
    HW_WR_REG32(baseAddr,phySel);
}

/* TODO: Review this function */
int32_t ICSS_EMAC_osDeinit(ICSS_EMAC_Handle icssEmacHandle)
{
    SemaphoreP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkSemaphoreObject));
    SemaphoreP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxSemaphoreObject));
    if(1 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->txInterruptEnable)
    {
        SemaphoreP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txSemaphoreObject));
    }
    HwiP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkInterruptObject));
    HwiP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxInterruptObject));
    if(1 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->txInterruptEnable)
    {
        HwiP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txInterruptObject));
    }
    TaskP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkTaskObject));
    TaskP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxTaskObject));
    if(1 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->txInterruptEnable)
    {
        TaskP_destruct(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txTaskObject));
    }

    return SystemP_SUCCESS;
}

int32_t ICSS_EMAC_validateFeatureSet(ICSS_EMAC_Handle icssEmacHandle,
                                     uint8_t portNo,
                                     uint32_t featureCtrl)
{
    int32_t                 ret = SystemP_SUCCESS;
    uint32_t                icssFwRelease1 = 0U;
    uint32_t                icssFwRelease2 = 0U;
    uint32_t                icssFwFeatureSet = 0U;
    uint32_t                pruDataMem = 0U;
    uint32_t                pruDataMem1 = 0U;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    /* ICSS_EMAC_PORT_0 is switch mode, need to validate for both ports */
    if(ICSS_EMAC_PORT_0 == portNo)
    {
        pruDataMem = pruicssHwAttrs->pru0DramBase;

        icssFwRelease1 = HW_RD_REG32(pruDataMem + pStaticMMap->versionOffset);
        icssFwRelease2 = HW_RD_REG32(pruDataMem + pStaticMMap->version2Offset);
        icssFwFeatureSet = HW_RD_REG32(pruDataMem + pStaticMMap->featureOffset);
        /* Only check the feature set field if fw release settings are non-zero implying they are valid*/
        if ((icssFwRelease1 != 0U) && (icssFwRelease2 != 0U))
        {
            if (0U==(icssFwFeatureSet & featureCtrl))
            {
                ret = SystemP_FAILURE;
            }
        }
        if(ret == SystemP_SUCCESS)
        {
            icssFwRelease1 = 0U;
            icssFwRelease2 = 0U;
            icssFwFeatureSet = 0U;
            pruDataMem1 = pruicssHwAttrs->pru1DramBase;
            icssFwRelease1 = HW_RD_REG32(pruDataMem1 + pStaticMMap->versionOffset);
            icssFwRelease2 = HW_RD_REG32(pruDataMem1 + pStaticMMap->version2Offset);
            icssFwFeatureSet = HW_RD_REG32(pruDataMem1 + pStaticMMap->featureOffset);
            /* Only check the feature set field if fw release settings are non-zero implying they are valid*/
            if ((icssFwRelease1 != 0U) && (icssFwRelease2 != 0U))
            {
                if (0U==(icssFwFeatureSet & featureCtrl))
                {
                    ret = SystemP_FAILURE;
                }
            }
        }
    }
    else if(ICSS_EMAC_PORT_1 == portNo)
    {
        pruDataMem = pruicssHwAttrs->pru0DramBase;
        icssFwRelease1 = HW_RD_REG32(pruDataMem + pStaticMMap->versionOffset);
        icssFwRelease2 = HW_RD_REG32(pruDataMem + pStaticMMap->version2Offset);
        icssFwFeatureSet = HW_RD_REG32(pruDataMem + pStaticMMap->featureOffset);
        /* Only check the feature set field if fw release settings are non-zero implying they are valid*/
        if ((icssFwRelease1 != 0U) && (icssFwRelease2 != 0U))
        {
            if (0U==(icssFwFeatureSet & featureCtrl))
            {
                ret = SystemP_FAILURE;
            }
        }
    }
    else if(ICSS_EMAC_PORT_2 == portNo)    /*    ICSS_EMAC_PORT_2    */
    {
        pruDataMem1 = pruicssHwAttrs->pru1DramBase;
        icssFwRelease1 = HW_RD_REG32(pruDataMem1 +pStaticMMap->versionOffset);
        icssFwRelease2 = HW_RD_REG32(pruDataMem1 +pStaticMMap->version2Offset);
        icssFwFeatureSet = HW_RD_REG32(pruDataMem1 +pStaticMMap->featureOffset);
        /* Only check the feature set field if fw release settings are non-zero implying they are valid*/
        if ((icssFwRelease1 != 0U) && (icssFwRelease2 != 0U))
        {
            if (0U==(icssFwFeatureSet & featureCtrl))
            {
                ret = SystemP_FAILURE;
            }
        }
    }
    else
    {
        ret = SystemP_FAILURE;
    }

    return ret;
}

int32_t ICSS_EMAC_promiscuousModeInit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t                *pTemp32 = NULL;
    int8_t                  retVal = SystemP_SUCCESS;
    uint32_t                temp_addr = 0;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);

    retVal = ICSS_EMAC_validateFeatureSet(icssEmacHandle, portNum, ICSS_EMAC_FW_PROMISCOUS_MODE_FEATURE_CTRL);

    if(SystemP_SUCCESS == retVal)
    {
        temp_addr = (pruicssHwAttrs->sharedDramBase + pStaticMMap->promiscuousModeOffset);
        pTemp32 = (uint32_t *)(temp_addr);
        *pTemp32 = *pTemp32 | (((uint32_t)1U) << (portNum - ((uint8_t)1U)));
    }
    return retVal;
}

int32_t ICSS_EMAC_promiscuousModeDeinit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{

    uint32_t                *pTemp32 = NULL;
    int8_t                  retVal = SystemP_SUCCESS;
    uint32_t                temp_addr = 0;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);

    retVal = ICSS_EMAC_validateFeatureSet(icssEmacHandle, portNum, ICSS_EMAC_FW_PROMISCOUS_MODE_FEATURE_CTRL);

    if(SystemP_SUCCESS == retVal)
    {
        temp_addr = (pruicssHwAttrs->sharedDramBase + pStaticMMap->promiscuousModeOffset);
        pTemp32 = (uint32_t *)(temp_addr);
       *pTemp32 = *pTemp32 & (~(((uint32_t)1U) << (portNum - ((uint8_t)1U))));
    }
    return retVal;
}

static void ICSS_EMAC_multicastFilterFeatureCtrl(ICSS_EMAC_FwMulticastFilterParams    *pMulticastFilterParams,
                                                 uintptr_t                          dataRamAddr,
                                                 uint8_t                            value)
{
    uint8_t *multicastTableControl = ((uint8_t *)(dataRamAddr + pMulticastFilterParams->ctrlOffset));
    *multicastTableControl = value;
    return;
}

static void ICSS_EMAC_multicastFilterOverrideHashmask(ICSS_EMAC_FwMulticastFilterParams   *pMulticastFilterParams,
                                                      uintptr_t                         dataRamAddr,
                                                      uint8_t                           *mask)
{
    uint8_t *multicastFilterMask;
    multicastFilterMask = ((uint8_t *)(dataRamAddr + pMulticastFilterParams->maskOffset));
    memcpy(multicastFilterMask, mask, pMulticastFilterParams->maskSizeBytes);
    return;
}

static void ICSS_EMAC_multicastFilterUpdateMacId(ICSS_EMAC_FwMulticastFilterParams   *pMulticastFilterParams,
                                                 uintptr_t                          dataRamAddr,
                                                 uint8_t                            *multicastAddr,
                                                 uint8_t                            command)
{
    uint8_t *multicastTableBaseAddr = ((uint8_t *)(dataRamAddr + pMulticastFilterParams->tableOffset));
    uint8_t *multicastTablePtr;
    uint8_t *multicastFilterMask = ((uint8_t *)(dataRamAddr + pMulticastFilterParams->maskOffset));
    uint8_t multicastAddrTemp[pMulticastFilterParams->maskSizeBytes];
    uint8_t hashVal, i;

    /* compute the hashVal by XORing all 6 bytes of multicastAddr*/
    for(i = 0, hashVal = 0; i < pMulticastFilterParams->maskSizeBytes; i++)
    {
        multicastAddrTemp[i] = multicastFilterMask[i] & multicastAddr[i];
        hashVal = hashVal ^ multicastAddrTemp[i];
    }

    multicastTablePtr = multicastTableBaseAddr + hashVal;

    if(command == ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ADD_MACID)
    {
        *multicastTablePtr = pMulticastFilterParams->hostRcvAllowedValue;
    }

    else    /*ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID*/
    {
        *multicastTablePtr = pMulticastFilterParams->hostRcvNotAllowedValue ;
    }

    return;
}

int32_t ICSS_EMAC_multicastFilterConfig(ICSS_EMAC_FwMulticastFilterParams *pMulticastFilterParams,
                                        uintptr_t                       dataRamAddr,
                                        uint8_t                         ioctlCmd,
                                        void                            *ioctlVal)
{
    uint8_t *mcFilterPtr;
    uint8_t isMaskSet;
    int32_t retVal = 0;
    uint8_t defaultMask[pMulticastFilterParams->maskSizeBytes];

    memset((void *)defaultMask, pMulticastFilterParams->maskInitVal, pMulticastFilterParams->maskSizeBytes);

    switch(ioctlCmd)
    {
        case ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ENABLE:
            ICSS_EMAC_multicastFilterFeatureCtrl(pMulticastFilterParams, dataRamAddr, pMulticastFilterParams->ctrlEnabledValue);
            /* Check if Hash mask has been set already, else set to default value */
            mcFilterPtr = (uint8_t* ) (dataRamAddr + pMulticastFilterParams->overrideStatusOffset);
            isMaskSet = *mcFilterPtr;
            if (isMaskSet == pMulticastFilterParams->maskOverrideNotSetValue)
            {
                ICSS_EMAC_multicastFilterOverrideHashmask(pMulticastFilterParams, dataRamAddr, (uint8_t *)defaultMask);
            }
            break;
        case ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_DISABLE:
            ICSS_EMAC_multicastFilterFeatureCtrl(pMulticastFilterParams, dataRamAddr, pMulticastFilterParams->ctrlDisabledValue);
            break;
        case ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_OVERRIDE_HASHMASK:
            ICSS_EMAC_multicastFilterOverrideHashmask(pMulticastFilterParams, dataRamAddr, (uint8_t *)ioctlVal);
            mcFilterPtr = (uint8_t* ) (dataRamAddr + pMulticastFilterParams->overrideStatusOffset);
            /* Indicate Mask override is set */
            *mcFilterPtr = pMulticastFilterParams->maskOverrideSetValue;
            break;
        case ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ADD_MACID:
        case ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID:
            ICSS_EMAC_multicastFilterUpdateMacId(pMulticastFilterParams, dataRamAddr, (uint8_t*)ioctlVal, ioctlCmd);
            break;
        default:
           retVal = SystemP_FAILURE;
           break;
    }
    return retVal;
}

static void ICSS_EMAC_vlanFilterFeatureCtrl(ICSS_EMAC_FwVlanFilterParams  *pVlanFilterParams,
                                            uintptr_t                   dataRamAddr,
                                            uint8_t                     ioctlCmd)
{
    uint8_t *vlanFilterCtrlByte = ((uint8_t *)(dataRamAddr + pVlanFilterParams->ctrlBitmapOffset));
    uint8_t ctrlVal = *vlanFilterCtrlByte;
    uint8_t mask;

    switch(ioctlCmd)
    {
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ENABLE_CMD:
             mask     = (uint8_t)( 1 << pVlanFilterParams->ctrlEnableBit);
             ctrlVal |= mask;
             break;
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_DISABLE_CMD:
             mask     = (uint8_t) ( 1 << pVlanFilterParams->ctrlEnableBit);
             ctrlVal &= ~(mask);
             break;
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_NAL_CMD:
             mask     = (uint8_t) ( 1 << pVlanFilterParams->ctrlUntagHostRcvAllowBit);
             ctrlVal |= (mask);
             break;
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_ALL_CMD:
             mask     = (uint8_t) ( 1 << pVlanFilterParams->ctrlUntagHostRcvAllowBit);
             ctrlVal &= ~(mask);
             break;

        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_NAL_CMD:
            mask     = (uint8_t) ( 1 << pVlanFilterParams->ctrlPriotagHostRcvAllowBit);
            ctrlVal |= (mask);
            break;

        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_ALL_CMD:
            mask     = (uint8_t) ( 1 << pVlanFilterParams->ctrlPriotagHostRcvAllowBit);
            ctrlVal &= ~(mask);
            break;

        default:
             break;
    }
    *vlanFilterCtrlByte = ctrlVal;
}

static int32_t ICSS_EMAC_vlanFilterUpdateVID(ICSS_EMAC_FwVlanFilterParams     *pVlanFilterParams,
                                             uintptr_t                      dataRamAddr,
                                             uint8_t                        ioctlCmd,
                                             uint16_t                       *vlanId)
{
    int32_t     retVal = 0;
    uint8_t     *vlanTableBaseAddr = ((uint8_t *)(dataRamAddr + pVlanFilterParams->filterTableBaseAddress));
    uint16_t    vid = *vlanId;
    uint8_t     *vlanTableBytePtr;
    uint16_t    vlanTargetByte;
    uint8_t     vlanTargetBit;

    if (vid  > pVlanFilterParams->vidMaxValue)
    {
        retVal = SystemP_FAILURE;
    }

    if (retVal == SystemP_SUCCESS)
    {
        vlanTargetByte    = vid / 8;
        vlanTargetBit     = vid & 0x07;
        vlanTableBytePtr  = vlanTableBaseAddr + vlanTargetByte;

        if (ioctlCmd == ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ADD_VID_CMD)
        {
            *vlanTableBytePtr = *vlanTableBytePtr | (1 << vlanTargetBit);
        }
        else /* ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_REMOVE_VID_CMD */
        {
            *vlanTableBytePtr = *vlanTableBytePtr | (1 << vlanTargetBit);
            *vlanTableBytePtr = *vlanTableBytePtr ^ (1 << vlanTargetBit);
        }
    }
    return retVal;
}

int32_t ICSS_EMAC_vlanFilterConfig(ICSS_EMAC_FwVlanFilterParams   *pVlanFilterParams,
                                   uintptr_t                    dataRamAddr,
                                   uint8_t                      ioctlCmd,
                                   void                         *ioctlVal)
{
    int32_t retVal = SystemP_SUCCESS;

    switch(ioctlCmd)
    {
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ENABLE_CMD:
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_DISABLE_CMD:
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_ALL_CMD:
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_NAL_CMD:
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_ALL_CMD:
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_NAL_CMD:
            /* Set the appropriate control bit map in firmware to enable the feature */
            ICSS_EMAC_vlanFilterFeatureCtrl(pVlanFilterParams, dataRamAddr, ioctlCmd);
            break;
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ADD_VID_CMD:
        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_REMOVE_VID_CMD:
            retVal = ICSS_EMAC_vlanFilterUpdateVID(pVlanFilterParams, dataRamAddr, ioctlCmd, (uint16_t *)ioctlVal);
            break;

        default:
           retVal = SystemP_FAILURE;
           break;
    }
    return retVal;
}
