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

#include <drivers/flsopskd/v0/flsopskd.h>
#include <drivers/hw_include/tistdtypes.h>
#include "base.h"

/**
 * @brief Writes to external flash using FLSOPSKD
 *
 * This App waits for an interrupt from other core and
 * once it receives it, starts writing calibration data
 * to the external flash. Here, each erase is of 4KB and
 * once entire write is complete, XIP downtime, measured
 * in the FLSOPSKD controller is then saved and printed
 * at the end of the program.
 */


#define FLASH_DATA_OFFSET           (6 * 1024 * 1024U)
#define CALIBRATION_DATA_LENGTH     (128 * 1024U)
#define EXT_FLASH_ERASE_OPCODE      (0x21U)
#define EXT_FLASH_ERASE_EXTOPCODE   (0xDEU)
#define EXT_FLASH_ERASE_SIZE        (4096U)
#define EXT_FLASH_PAGE_SIZE         (256U)

/**
 * @brief FLSOPSKD Handle.
 *
 */
FLSOPSKD_Handle gFlopsdkHandle;

/**
 * @brief calibration data to write.
 *
 */
uint32_t gWrCalibrationData[CALIBRATION_DATA_LENGTH/sizeof(uint32_t)] = {0};

/**
 * @brief Global variable to acting as a semaphore.abort
 *
 * This variable is set by ISR. mail function while loop is
 * is blocked on this variable.
 *
 */
int gStartWritingToFlash = 0;

/**
 * @brief ISR Callback function to start write to flash.
 *
 * @param args arguments.
 */
void startWrite(void* args)
{
    gStartWritingToFlash = 1;
    HwiP_clearInt(CSLR_R5FSS0_CORE1_INTR_SW_IRQ);
}

uint32_t FLSOPSKD_usrGetTicks()
{
    /* in syscfg, 1 tick is config as 1ms */
    return ClockP_getTicks();
}

/**
 * @brief Convert GP1 counter value to nanoseconds.
 *
 * @param counterValue GP1 Counter value.
 * @return nanoseconds
 */
uint32_t convertGP1ValueToTime(uint32_t counterValue)
{
    return counterValue * 10;
}

/**
 * @brief Write and verify calibartion data to flash.
 *
 * @param args arguments.
 */
void registerSoftwareInterrupt()
{
    HwiP_Params hwiPrms;
    HwiP_Object HwiObject;
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE1_INTR_SW_IRQ;
    hwiPrms.callback    = &startWrite;
    hwiPrms.isPulse     = 0;
    DebugP_assert(HwiP_construct(&HwiObject, &hwiPrms) == SystemP_SUCCESS);
}

/**
 * @brief Write and verify calibartion data to flash.
 *
 * @param args arguments.
 */
void write_verify_calibration_data(void *args)
{
    uint32_t flashSectorSize = EXT_FLASH_ERASE_SIZE;
    uint32_t nFlashSector = 0;
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint64_t timeToFlash = 0;
    uint32_t gWorstCaseDowntime = 0;
    uint32_t totalPolls = 0;

    nFlashSector = CALIBRATION_DATA_LENGTH / flashSectorSize;
    nFlashSector = nFlashSector == 0 ? 1 : nFlashSector;

    /* erase and write */
    start_time = ClockP_getTicks();
    for(uint32_t i = 0; i < nFlashSector; i++)
    {
        FLSOPSKD_erase(&gFlopsdkHandle, FLASH_DATA_OFFSET + flashSectorSize * i);
        gWorstCaseDowntime += gFlopsdkHandle.lastOperationXipDowntime;
        totalPolls += gFlopsdkHandle.lastOperationPollCount;
    }
    FLSOPSKD_write(&gFlopsdkHandle, FLASH_DATA_OFFSET, (uint8_t*)gWrCalibrationData, CALIBRATION_DATA_LENGTH);
    gWorstCaseDowntime += gFlopsdkHandle.lastOperationXipDowntime;
    totalPolls += gFlopsdkHandle.lastOperationPollCount;
    end_time = ClockP_getTicks();

    /* verify */
    uint32_t *gRdCalibrationData = (uint32_t *)(FLASH_DATA_OFFSET + SOC_getFlashDataBaseAddr());
    CacheP_inv((void*)(gRdCalibrationData), CALIBRATION_DATA_LENGTH, CacheP_TYPE_ALL);
    for(uint32_t i = 0; i < CALIBRATION_DATA_LENGTH/sizeof(uint32_t); i++)
    {
        volatile uint32_t actual = gRdCalibrationData[i];
        volatile uint32_t expected = gWrCalibrationData[i];
        DebugP_assert(actual == expected);
    }

    /* Print stats */
    timeToFlash = ClockP_ticksToUsec(end_time - start_time);
    timeToFlash /= 1000;
    timeToFlash /= 1000;

    DebugP_log("Time to flash : %d sec\r\n", timeToFlash);
    DebugP_log("Status Polls Sent: %d\r\n", totalPolls);
    DebugP_log("Worst Case XIP Downtime Measured: %d ns\r\n", gWorstCaseDowntime);
}

int main(void)
{
    uint32_t fw8051version = 0;

    System_init();
    Board_init();
    Drivers_open();
    Board_driversOpen();

    FLSOPSKD_Params params;
    FLSOPSKD_Params_init(&params);
    params.eraseOpCode = EXT_FLASH_ERASE_OPCODE;
    params.eraseExOpCode = EXT_FLASH_ERASE_EXTOPCODE;
    params.pageSizeInBytes = EXT_FLASH_PAGE_SIZE;
    params.eraseSizeInBytes = EXT_FLASH_ERASE_SIZE;
    FLSOPSKD_init(&gFlopsdkHandle, &params);

    FLSOPSKD_getFwVersion(&gFlopsdkHandle, &fw8051version);
    DebugP_assert(fw8051version == FLSOPSKD_EXPECTED_FW_VERSION);

    registerSoftwareInterrupt();


    for(uint32_t i = 0; i < CALIBRATION_DATA_LENGTH/4; i++)
    {
        gWrCalibrationData[i] = 0xffff0000 + i;
    }

    while(gStartWritingToFlash == 0);
    write_verify_calibration_data(NULL);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
    Board_deinit();
    System_deinit();

    return 0;
}
