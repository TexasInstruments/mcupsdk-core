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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <FreeRTOS.h>
#include <task.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include <drivers/fota_agent/fota_agent.h>

/*
 * This is an empty project provided for all cores present in the device.
 * User can use this project to start their application by adding more SysConfig modules.
 *
 * This application does driver and board init and just prints the pass string on the console.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */
#define WRITE_TASK_PRI  (30u)
#define LED_TASK_PRI  (31u)
#define FLASH_SECTOR_SIZE_2 (4096)
#define BOOT_INFO_BASE  (0x80000U)
#define TOTAL_FILES_TO_FLASH (2U) 
#define WRITE_TASK_SIZE (1024u)
#define LED_TASK_SIZE (1024u)
#define APP_OSPI_DATA_SIZE (5*1024U)
#define REGION_B (1U)

uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));
StackType_t gWriteTaskStack[WRITE_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gLedTaskStack[LED_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gWriteTaskObj;
TaskHandle_t gWriteTask;
StaticTask_t gLedTaskObj;
TaskHandle_t gLedTask;
FOTAAgent_handle fotaAgentHandle;

typedef struct bootinfo_sector_s_t
{
    uint8_t phyTuningVector[128];
    uint32_t bootRegion;
}bootinfo_sector_fields_t;

typedef union bootinfo_sector_u_t
{
    bootinfo_sector_fields_t fields;
    uint8_t bin[FLASH_SECTOR_SIZE_2];
}bootinfo_sector_t;

void write_to_flash();
void led_blink();
int32_t swap_to_b(uint32_t bootregion);
void mcanEnableTransceiver(void);
void i2c_flash_reset(void);
int32_t CANTransfer_TransmitResp(uint8_t *src);
int32_t CANTransfer_ReceiveFile(uint32_t *fileSize, uint8_t *dstBuf, uint32_t *run, uint8_t *lastChunk);
void CANTransfer_Init(uint64_t systemAddr);

void flashFixUpOspiBoot(OSPI_Handle oHandle)
{
    i2c_flash_reset();
    OSPI_enableSDR(oHandle);
    OSPI_clearDualOpCodeMode(oHandle);
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(1,1,1,0));
}

void write_to_flash()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset = 0;
    uint32_t gRunApp = 0;
    uint8_t done;
    uint32_t isXip; 
    uint32_t receivedChunks;
    Flash_Attrs *flashAttrs;
	flashAttrs = Flash_getAttrs(CONFIG_FLASH0);
    uint32_t flashBaseOffset;
    bootinfo_sector_t *bootSectorInfo = (bootinfo_sector_t *)BOOT_INFO_BASE;

    status = Flash_read(gFlashHandle[CONFIG_FLASH0], BOOT_INFO_BASE, bootSectorInfo->bin, FLASH_SECTOR_SIZE_2);
    DebugP_assert(status==SystemP_SUCCESS);

    uint32_t bootRegion = bootSectorInfo->fields.bootRegion;
    if(bootRegion == 1)
    {
        DebugP_log("BOOTING from Region B!!!! \r\n");
        flashBaseOffset = 0;
    }
    else
    {
        DebugP_log("BOOTING from Region A!!!! \r\n");
        flashBaseOffset = flashAttrs->flashSize/2;
    }
    for(int i=0;i<TOTAL_FILES_TO_FLASH;i++)
    {
        done = 0;
        isXip = FALSE; 
        receivedChunks = 0;
        while(!done)
        {
            uint32_t fileSize = 0;
            Bootloader_UniflashFileHeader fileHeader;
            Bootloader_UniflashResponseHeader respHeader = {
                .magicNumber = BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER,
                .statusCode  = BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS,
                .rsv0        = 0xDEADBABE,
                .rsv1        = 0xDEADBABE,
            };
            status = CANTransfer_ReceiveFile(&fileSize, gOspiTxBuf, &gRunApp, &done);
            DebugP_assert(status==SystemP_SUCCESS);
            memcpy(&fileHeader, gOspiTxBuf, sizeof(Bootloader_UniflashFileHeader));
            if(receivedChunks==0)
            {
                DebugP_log("File Transfer Started!!!! \r\n");
                uint32_t opType = (fileHeader.operationTypeAndFlags) & (uint32_t)0xFF;
                if(opType == BOOTLOADER_UNIFLASH_OPTYPE_FLASH_XIP)
                {
                    isXip = TRUE;
                    offset = 0;
                }
                else
                {
                    offset = fileHeader.offset;
                }
                FOTAAgent_writeStart(&fotaAgentHandle,flashBaseOffset,offset,isXip);
                DebugP_assert(status==SystemP_SUCCESS);
            }
            if(fileHeader.magicNumber != BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER)
            {
                respHeader.statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR;
                status = SystemP_FAILURE;
            }
            DebugP_assert(status==SystemP_SUCCESS);

            status = CANTransfer_TransmitResp((uint8_t *)&respHeader);
            DebugP_assert(status==SystemP_SUCCESS);

            status += FOTAAgent_writeUpdate(&fotaAgentHandle,gOspiTxBuf + sizeof(Bootloader_UniflashFileHeader),fileHeader.actualFileSize);
            DebugP_assert(status==SystemP_SUCCESS);            

            receivedChunks++;
            memset(gOspiTxBuf,0xFF,APP_OSPI_DATA_SIZE);
        }

        status += FOTAAgent_writeEnd(&fotaAgentHandle);
        DebugP_assert(status==SystemP_SUCCESS);
        DebugP_log("File %d received!!!! \r\n", i+1);
    }
	
    status += swap_to_b(bootRegion);
    DebugP_assert(status==SystemP_SUCCESS);

    if(status==SystemP_SUCCESS)
        DebugP_log("All Test Passed!! \r\n");
    vTaskDelete(NULL);
}

int32_t swap_to_b(uint32_t bootRegion)
{
    bootinfo_sector_t *bootSectorInfo = (bootinfo_sector_t *)BOOT_INFO_BASE;
    int32_t status = SystemP_SUCCESS;
    uint32_t offset = BOOT_INFO_BASE;
    FLSOPSKD_handle pHandle = fotaAgentHandle.FLSOPSKDhandle;

    DebugP_log("Updating boot info....\r\n");
    
    if(SystemP_SUCCESS == status)
    {
        status = Flash_read(gFlashHandle[CONFIG_FLASH0], offset, bootSectorInfo->bin, FLASH_SECTOR_SIZE_2);
    }

    if(bootRegion == REGION_B)
    {
        bootSectorInfo->fields.bootRegion = 0xFF;
    }
    else
    {
        bootSectorInfo->fields.bootRegion = 1;
    }

    status = FLSOPSKD_Erase(&pHandle,offset);

    if(SystemP_SUCCESS == status)
    {
        status = FLSOPSKD_Write(&pHandle, offset, bootSectorInfo->bin, FLASH_SECTOR_SIZE_2);
    }
    else
    {
        DebugP_log("Flash Write of %d bytes failed at 0x%X offset !!!", FLASH_SECTOR_SIZE_2, offset);
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("Please reset the board\r\n");
    }
    else
    {
        DebugP_log("ERROR: Failed to write boot info.\r\n");
    }

    return status;

}

void led_blink()
{
    uint32_t    loopcnt = 10, delaySec = 1;
    uint32_t    gpioBaseAddr, pinNum;

    DebugP_log("GPIO LED Blink Test Started ...\r\n");
    DebugP_log("LED will Blink for %d seconds ...\r\n", (loopcnt * delaySec * 2));

    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    pinNum       = GPIO_LED_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_LED_DIR);
    while(loopcnt > 0)
    {
        GPIO_pinWriteHigh(gpioBaseAddr, pinNum);
        ClockP_sleep(delaySec);
        GPIO_pinWriteLow(gpioBaseAddr, pinNum);
        ClockP_sleep(delaySec);
        loopcnt--;
    }

    DebugP_log("GPIO LED Blink Test Passed!!\r\n");
    vTaskDelete(NULL);
}

void fota_over_can_main(void *args)
{
    int32_t status;
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    flashFixUpOspiBoot(gOspiHandle[CONFIG_OSPI0]);
    Board_driversOpen();
    status = FOTAAgent_init(&fotaAgentHandle);
    DebugP_assert(status==SystemP_SUCCESS);

    mcanEnableTransceiver();
    CANTransfer_Init(CONFIG_MCAN0_BASE_ADDR);

    /* then create the tasks, order of task creation does not matter for this example */

    gLedTask = xTaskCreateStatic( led_blink,      /* Pointer to the function that implements the task. */
                                  "led",          /* Text name for the task.  This is to facilitate debugging only. */
                                  LED_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  LED_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gLedTaskStack,  /* pointer to stack base */
                                  &gLedTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gLedTask != NULL);

    gWriteTask = xTaskCreateStatic( write_to_flash,      /* Pointer to the function that implements the task. */
                                  "write",          /* Text name for the task.  This is to facilitate debugging only. */
                                  WRITE_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  WRITE_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gWriteTaskStack,  /* pointer to stack base */
                                  &gWriteTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gWriteTask != NULL);
    while(1);
    Board_driversClose();
    Drivers_close();
}

