/*
*  Copyright (C) 2025 Texas Instruments Incorporated
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

#ifndef GPMC_EDMA_H_
#define GPMC_EDMA_H_

#include <stdint.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief GPMC EDMA Parameters
 *
 *  Used to store the EDMA parameters allocated for GPMC transfer.
 *
 */
typedef struct
{
	uint32_t edmaTcc;
	/**< EDMA TCC used for GPMC transfer */
	uint32_t edmaChId;
	/**< EDMA Channel used for GPMC transfer */
	uint32_t edmaParam;
	/**< EDMA Param ID used for GPMC transfer */
	uint32_t edmaRegionId;
	/**< EDMA Region used for GPMC transfer */
	uint32_t edmaBaseAddr;
	/**< EDMA Base address used for GPMC transfer */
	uint32_t isIntEnabled;
	/**< EDMA Interrupt enabled status */
	Edma_IntrObject edmaIntrObj;
	/**< EDMA Interrupt object */
	SemaphoreP_Object gEdmaTransferDoneSem;
	/**< EDMA transfer done Semaphore */
	uint32_t edmaInst;
} Gpmc_DmaArgs;

typedef struct GPMC_DmaConfig_s
{
	Gpmc_DmaArgs *gpmcDmaArgs;
	/* Arguments specific to a DMA driver. This will be typecasted to the specific DMA driver args struct
	 * when used by the appropriate callback. This struct will be defined in the specific DMA driver header file.
	 * Allocation of this struct will be done statically using Sysconfig code generation in the example code
	 */

} GPMC_DmaConfig;

int32_t GpmcDma_edmaOpen(Gpmc_DmaArgs* gpmcDmaArgs);
int32_t GpmcDma_edmaClose(Gpmc_DmaArgs* gpmcDmaArgs);
int32_t GpmcDma_edmaCopy(Gpmc_DmaArgs* gpmcDmaArgs, uint32_t *dst, uint32_t *src, uint32_t length, uint8_t fifoDrain);

#ifdef __cplusplus
}
#endif

#endif /* GPMC_EDMA_H_ */