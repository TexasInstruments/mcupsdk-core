/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <drivers/pruicss.h>
#include <pru_load_bin.h> // > PRUFirmware array

/**
 *  @brief PRU Core
 *  Wil come from sysconfig
 */
#define PRUICSS_PRUx                PRUICSS_PRU0

/*
 *  This is an example project to show R5F
 *  loading PRU firmware.
 */

/** \brief Global Structure pointer holding PRUSS1 memory Map. */

PRUICSS_Handle gPruIcss0Handle;

void load_pru_firmware(void *args)
{
     /* ----------------------------------------------------------------- */
     /* Program empty code on PRU Core/s;                                   */
     /* depends on usecase - might have to program multiple cores         */
     /* ----------------------------------------------------------------- */
     /* clear ICSS PRUx data RAM */
     int status;
     status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRUx));
     DebugP_assert(status != 0);
     status = PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
     DebugP_assert(SystemP_SUCCESS == status);
     status = PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);
     DebugP_assert(SystemP_SUCCESS == status);

     /* Load firmware. Set buffer = write to Pru memory */
     status = PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx), 0,
                        (uint32_t *) PRUFirmware_0, sizeof(PRUFirmware_0));
     DebugP_assert(status != 0);

     status = PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
     DebugP_assert(SystemP_SUCCESS == status);
     /* Run firmware */
     status = PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
     DebugP_assert(SystemP_SUCCESS == status);
}

void pru_io_empty_example_main(void *args)
{
     Drivers_open(); // check return status

     int status;
     status = Board_driversOpen();
     DebugP_assert(SystemP_SUCCESS == status);

     gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

     load_pru_firmware(NULL);
     
     while (1)
     {
        ClockP_usleep(1);
     }

     Board_driversClose();
     Drivers_close();
}
