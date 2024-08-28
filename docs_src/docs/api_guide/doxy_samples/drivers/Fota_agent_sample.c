
//! [include]
#include <stdio.h>
#include <drivers/fota_agent/fota_agent.h>
//! [include]

#define APP_OSPI_DATA_SIZE (20*1024U)
#define WRITE_CHUNK_SIZE   (4*1024U)
uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));
FOTAAgent_handle fotaAgentHandle;

//! [fota_init]
int32_t init_FOTAAgent(void)
{
    /* Initialize Fota Agent Driver*/
    int32_t status;
    status = FOTAAgent_init(&fotaAgentHandle);

    return status;
}
//! [fota_init]

void transfer_FOTAAgent(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t isXip = FALSE; 
    uint32_t flashBaseOffset = 0;
    uint32_t offset = 0x80000;
    uint32_t totalChunks = (APP_OSPI_DATA_SIZE/WRITE_CHUNK_SIZE) + ((APP_OSPI_DATA_SIZE%WRITE_CHUNK_SIZE) > 0);
    /* initialise buffer*/
    for(int i=0;i<APP_OSPI_DATA_SIZE;i++)
    {
        gOspiTxBuf[i] = i%256;
    }

//! [fota_write_start]
    FOTAAgent_writeStart(&fotaAgentHandle,flashBaseOffset,offset,isXip);
//! [fota_write_start]

//! [fota_write_update]
    for(int i=0;i<totalChunks;i++)
    {
        status += FOTAAgent_writeUpdate(&fotaAgentHandle,gOspiTxBuf + i*WRITE_CHUNK_SIZE,WRITE_CHUNK_SIZE);
        DebugP_assert(status==SystemP_SUCCESS);
    }    
//! [fota_write_update]

//! [fota_write_end]
    status += FOTAAgent_writeEnd(&fotaAgentHandle);
    DebugP_assert(status==SystemP_SUCCESS);
//! [fota_write_end]

}