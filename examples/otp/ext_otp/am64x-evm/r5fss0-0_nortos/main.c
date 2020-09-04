/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include "ext_otp.h"


/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

int main()
{
    int32_t status;
	uint32_t testMmrVal; 
	uint32_t testRowValRdBk; 

    status = Bootloader_socWaitForFWBoot();
    DebugP_assertNoLog(status == SystemP_SUCCESS);

    System_init();
    Drivers_open();

    DebugP_log("\r\n");
    DebugP_log("Starting EXT OTP writer\r\n");

	ext_otp_setVpp();
    DebugP_log("Enabled VPP\r\n");

	status = ext_otp_writeUsbVidPid(0x45a0, 0xe047);
    status += ext_otp_writePcieVidPid(0x1234, 0x4321);
    
    if(status != SystemP_SUCCESS )
    {
        DebugP_log("Error programming VID/PID \r\n");
    }
	else
	{
        DebugP_log("Success programming VID/PID \r\n");
	}

	/* Test reading OTP MMR */
    status = ext_otp_readMmr(11, &testMmrVal);
	status = ext_otp_printMmrs();

	/* Read USB - PCIE VID PID from control MMRs */
	status = ext_otp_getUsbVid(&testMmrVal);
	DebugP_log("USB VID: 0x%x \r\n", testMmrVal);
	status = ext_otp_getUsbPid(&testMmrVal);
	DebugP_log("USB PID: 0x%x \r\n", testMmrVal);
	status = ext_otp_getPcieVid(&testMmrVal);
	DebugP_log("PCIE VID: 0x%x \r\n", testMmrVal);
	status = ext_otp_getPciePid(&testMmrVal);
	DebugP_log("PCIE PID: 0x%x \r\n", testMmrVal);

	/* Test writing OTP ROW */
	status = ext_otp_writeRow(0x0, 0x10, 0x10, &testRowValRdBk);

    if(status != SystemP_SUCCESS )
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    else
    {
        DebugP_log("All tests have passed!!\r\n");
    }

    Drivers_close();
    System_deinit();

    return 0;
}


