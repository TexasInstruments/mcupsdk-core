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

#include "base.h"

#define FLASH_DATA_OFFSET (10*1024*1024U)

#define CALIBRATION_DATA_LENGTH (128*1024U)

CSL_fss_fota_genregsRegs *fotaregs;

FLSOPSKD_handle flopsdkHandle;

uint32_t gWrCalibrationData[CALIBRATION_DATA_LENGTH/sizeof(uint32_t)] = {0};
uint32_t gRdCalibrationData[CALIBRATION_DATA_LENGTH/sizeof(uint32_t)] = {0};
uint32_t erase_time[10] = {0};
uint32_t write_time[10] = {0};

int startWritingToFlash = 0;

void startWrite(void* args)
{
    startWritingToFlash = 1;
    HwiP_clearInt(CSLR_R5FSS0_CORE1_INTR_SW_IRQ);
}

volatile uint32_t totalFOTATime;
void write_verify_calibration_data(void *args)
{
    uint32_t flashSectorSize = 0;
    uint32_t nFlashSector ;

    FLSOPSKD_GetSectorSize(&flopsdkHandle, &flashSectorSize);
    nFlashSector = CALIBRATION_DATA_LENGTH / flashSectorSize;
    nFlashSector = nFlashSector == 0 ? 1 : nFlashSector;
    for(uint32_t i = 0; i < nFlashSector; i++)
    {
        FLSOPSKD_Erase(&flopsdkHandle, FLASH_DATA_OFFSET + flashSectorSize * i);
    }

    FLSOPSKD_Write(&flopsdkHandle, FLASH_DATA_OFFSET, (uint8_t*)gWrCalibrationData, CALIBRATION_DATA_LENGTH);

    /* verify by reading it back */

    FLSOPSKD_STIGRead(&flopsdkHandle, FLASH_DATA_OFFSET, (uint8_t*)gRdCalibrationData, CALIBRATION_DATA_LENGTH);

    for(uint32_t rdLen = 0; rdLen < sizeof(gRdCalibrationData)/sizeof(*gRdCalibrationData); rdLen++)
    {
        DebugP_assert(gWrCalibrationData[rdLen] == gRdCalibrationData[rdLen]);
    }

}

__attribute__((optnone))  int main(void)
{
    static unsigned int timesWritten = 0 ;
    fotaregs = (CSL_fss_fota_genregsRegs *)CSL_FSS_FOTA_GENREGS_REGS_BASE;
    uint32_t fw8051version = 0;
    System_init();
    Board_init();
    Drivers_open();

    Board_driversOpen();

    config_pmu();

    FLSOPSKD_Init(&flopsdkHandle);

    FLSOPSKD_Get8051Version(&flopsdkHandle, &fw8051version);

    DebugP_assert(fw8051version == 0x10000);

    DebugP_log("Init FLOPKD done\r\n");

    {
        HwiP_Params hwiPrms;
        HwiP_Object HwiObject;
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = CSLR_R5FSS0_CORE1_INTR_SW_IRQ;
        hwiPrms.callback    = &startWrite;
        hwiPrms.isPulse     = 0; //level interrupt
        DebugP_assert(HwiP_construct(&HwiObject, &hwiPrms) == SystemP_SUCCESS);
    }

    CycleCounterP_reset();

    while(1)
    {
        while(startWritingToFlash == 0);
        for(uint32_t i = 0; i < CALIBRATION_DATA_LENGTH/4; i++)
        {
            gWrCalibrationData[i] = 0xffff0000 + i;
            gRdCalibrationData[i] = 0;
        }
        totalFOTATime = CycleCounterP_getCount32();
        write_verify_calibration_data(NULL);
        totalFOTATime = CycleCounterP_getCount32() - totalFOTATime ;
        timesWritten += 1;
        startWritingToFlash = 0;
    }

    (void)timesWritten;
    (void)totalFOTATime;

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
    Board_deinit();
    System_deinit();

    return 0;
}
