/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include "fota_agent.h"
#include <security/security_common/drivers/hsmclient/hsmclient.h>

#define SIPC_QUEUE_LENGTH   (32u)
/* Total number of secure cores */
#define SIPC_NUM_R5_CORES   (2u)

#define ERASE_OPCODE        (0x21u)
#define ERASE_EXOPCODE		(0xDCu)

uint8_t gQueue_HsmToR5[SIPC_NUM_R5_CORES][SIPC_QUEUE_LENGTH*SIPC_MSG_SIZE] __attribute__((aligned(8),section(".bss.sipc_hsm_queue_mem")));
uint8_t gQueue_R5ToHsm[SIPC_NUM_R5_CORES][SIPC_QUEUE_LENGTH*SIPC_MSG_SIZE] __attribute__((aligned(8),section(".bss.sipc_r5f_queue_mem")));
HsmClient_t gHSMClient ;

int32_t add_HSM_firewall(void);

int32_t FOTAAgent_init(FOTAAgent_handle *pHandle)
{
    int32_t status;

    /* Spin till lock is acquired */
    while(1U)
    {
        status = Spinlock_lock(CSL_SPINLOCK0_BASE, LOCK_NUM0);
        if(status == SPINLOCK_LOCK_STATUS_FREE)
        {
            break;  /* Free and taken */
        }
    }

    status = Spinlock_lock(CSL_SPINLOCK0_BASE, LOCK_NUM1);
    if(status == SPINLOCK_LOCK_STATUS_FREE)
    {
        status = FLSOPSKD_Init(&(pHandle->FLSOPSKDhandle),ERASE_OPCODE,ERASE_EXOPCODE);
		if(status == SystemP_SUCCESS)
        status = add_HSM_firewall();
    }

    Spinlock_unlock(CSL_SPINLOCK0_BASE, LOCK_NUM0);

    return status;
}

void FOTAAgent_writeStart(FOTAAgent_handle *pHandle,uint32_t baseAddr,uint32_t wrOffset,uint32_t isXip)
{
	int32_t status = SystemP_FAILURE;
	while(1U)
	{
		status = Spinlock_lock(CSL_SPINLOCK0_BASE, LOCK_NUM0);
		if(status == SPINLOCK_LOCK_STATUS_FREE)
		{
			break;  /* Free and taken */
		}
	}
	pHandle->isXip = isXip;
	pHandle->chunkOffset = 0;
    pHandle->currWriteOffset = wrOffset;
	pHandle->prevWriteOffset = wrOffset;
    pHandle->flashBaseOffset = pHandle->isXip ? 0 : baseAddr;
	
	memset(pHandle->chunk,0xFF,CHUNK_SIZE);

    ELFUP_init(&(pHandle->elfuph), pHandle->pht, 20);

}

int32_t FOTAAgent_writeEnd(FOTAAgent_handle *pHandle)
{
	int32_t status = SystemP_SUCCESS;

	if(pHandle->currWriteOffset > 0)
	{
		status = FLSOPSKD_Erase(&(pHandle->FLSOPSKDhandle),pHandle->flashBaseOffset + pHandle->currWriteOffset);
		status = FLSOPSKD_Write(&(pHandle->FLSOPSKDhandle),pHandle->flashBaseOffset + pHandle->currWriteOffset,pHandle->chunk,CHUNK_SIZE);
	}

    memset(pHandle->chunk,0xFF,CHUNK_SIZE);
    pHandle->currWriteOffset = 0;
	pHandle->prevWriteOffset = 0;
    pHandle->chunkOffset = 0;
	Spinlock_unlock(CSL_SPINLOCK0_BASE,LOCK_NUM0);
	return status;
}

int32_t FOTAAgent_writeUpdate(FOTAAgent_handle *pHandle,uint8_t *buf,uint32_t size)
{
	int32_t status = SystemP_SUCCESS;
	static int32_t totalReceived = 0;

	if(pHandle->isXip == TRUE)
	{	
		totalReceived += size;
		uint32_t fileStart = totalReceived-size;

		for(uint32_t i = 0; i < size; i++)
		{
			ELFUP_ELFPH cpht;
			status |=ELFUP_update(&(pHandle->elfuph), buf[i]);
			uint32_t fileOffset = fileStart + i;
			if(ELFUP_isPartOfSegment(&(pHandle->elfuph), fileOffset, &cpht) == SystemP_SUCCESS)
			{
				if(cpht.ELFPH.type == PT_LOAD)
				{
					uint32_t dest_addr = cpht.ELFPH.paddr & ~(0xF0000000);
					pHandle->currWriteOffset = ((dest_addr + fileOffset - cpht.ELFPH.offset)/CHUNK_SIZE)*CHUNK_SIZE;
					if((pHandle->currWriteOffset != pHandle->prevWriteOffset) && pHandle->prevWriteOffset!=0)
					{
						status = FLSOPSKD_Erase(&(pHandle->FLSOPSKDhandle),pHandle->flashBaseOffset + pHandle->prevWriteOffset);
						status = FLSOPSKD_Write(&(pHandle->FLSOPSKDhandle),pHandle->flashBaseOffset + pHandle->prevWriteOffset,pHandle->chunk,CHUNK_SIZE);
						if(status == SystemP_SUCCESS)
						{
							memset(pHandle->chunk,0xFF,CHUNK_SIZE);
						}
					}
					pHandle->chunkOffset = (dest_addr + fileOffset - cpht.ELFPH.offset)%CHUNK_SIZE;
					pHandle->chunk[pHandle->chunkOffset] = buf[i];
					pHandle->prevWriteOffset = pHandle->currWriteOffset;
				}
			}
		}
	}
	else
	{
		uint32_t spaceLeft = CHUNK_SIZE - pHandle->chunkOffset;
		uint32_t bytesToCopy = size;
		uint32_t copySize = spaceLeft < bytesToCopy ? spaceLeft : bytesToCopy;

		memcpy(pHandle->chunk + pHandle->chunkOffset,buf,copySize);
		pHandle->chunkOffset += copySize;

		if(pHandle->chunkOffset == CHUNK_SIZE)
		{
			status = FLSOPSKD_Erase(&(pHandle->FLSOPSKDhandle),pHandle->flashBaseOffset + pHandle->currWriteOffset);
			status = FLSOPSKD_Write(&(pHandle->FLSOPSKDhandle),pHandle->flashBaseOffset + pHandle->currWriteOffset,pHandle->chunk,CHUNK_SIZE);

			if(status == SystemP_SUCCESS)
			{
				pHandle->currWriteOffset += CHUNK_SIZE;
				pHandle->chunkOffset = 0;
				memset(pHandle->chunk,0xFF,CHUNK_SIZE);

				bytesToCopy -= copySize;
				if(bytesToCopy>0)
				{
					memcpy(pHandle->chunk + pHandle->chunkOffset,buf + copySize,bytesToCopy);
					pHandle->chunkOffset += bytesToCopy;
				}
			}
		}
	}
	return status;
}

void HsmClient_config(void)
{
    SIPC_Params sipcParams;
    int32_t status;

    /* initialize parameters to default */
    SIPC_Params_init(&sipcParams);

    sipcParams.ipcQueue_eleSize_inBytes = SIPC_MSG_SIZE;
    sipcParams.ipcQueue_length = SIPC_QUEUE_LENGTH ;
    /* list the cores that will do SIPC communication with this core
    * Make sure to NOT list 'self' core in the list below
    */
    sipcParams.numCores = 1;
    sipcParams.coreIdList[0] = CORE_INDEX_HSM;

    /* specify the priority of SIPC Notify interrupt */
    sipcParams.intrPriority = 7U;


    /* This is HSM -> R5F queue */
    sipcParams.tx_SipcQueues[CORE_INDEX_HSM] = (uintptr_t)gQueue_R5ToHsm[0] ;
    sipcParams.rx_SipcQueues[CORE_INDEX_HSM] = (uintptr_t)gQueue_HsmToR5[0] ;
    sipcParams.secHostCoreId[CORE_INDEX_SEC_MASTER_0] = CORE_ID_R5FSS0_0;

    /* initialize the HsmClient module */
    status = HsmClient_init(&sipcParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* register a hsm client to detect bootnotify message and keyring import from HSM */
    status = HsmClient_register(&gHSMClient, 1);
    DebugP_assert(status==SystemP_SUCCESS);
}

void HsmClient_unRegister(void)
{
     /* Unregister bootnotify client */
    HsmClient_unregister(&gHSMClient, HSM_BOOT_NOTIFY_CLIENT_ID);
}

int32_t add_HSM_firewall(void)
{
    int32_t status;
    FirewallReq_t FirewallReqObj;
    FirewallRegionReq_t gMpuFirewallRegionConfig[1] = {
        {
            .firewallId = 14, // CSL bug
            .region = 2U,
            .permissionAttributes = (0xFF) | (((0x1<<PRIV_ID_R5FSS0_0))<<10U),
            .startAddress = CSL_FSS_PDMEM_GENREGS_REGS_BASE,
            .endAddress   = CSL_FSS_PDMEM_GENREGS_REGS_BASE + 0x7FFu,
        },
    };
    FirewallReqObj.regionCount = 1;
    FirewallReqObj.FirewallRegionArr = gMpuFirewallRegionConfig;

    HsmClient_config();
    status = HsmClient_setFirewall(&gHSMClient,&FirewallReqObj,SystemP_WAIT_FOREVER);
    return status;
}