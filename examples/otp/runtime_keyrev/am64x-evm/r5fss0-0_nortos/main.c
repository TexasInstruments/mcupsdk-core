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
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/sciclient/include/tisci/security/tisci_otp_revision.h>
#include <drivers/bootloader.h>
#include "runtime_keyrev.h"

#define EFUSE_VPP_PIN     (15U)
#define EFUSE_VPP_PIN_LED (16U)

/*
 *  Set this macro inorder to update the KEY Revision
 */
#define UPDATE_KEYREV     (0U)

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

    status = Bootloader_socWaitForFWBoot();
    DebugP_assertNoLog(status == SystemP_SUCCESS);

    System_init();
    Board_init();

    Drivers_open();
    Board_driversOpen();

    DebugP_log("\r\n");
    DebugP_log("Starting Runtime KEYREV writer\r\n");

    uint32_t keyrev = 0x0U, keycount = 0x0U;
    status = Sciclient_getVersionCheck(1);
    /* Fetch the current key revision and key count */
    status = runtime_keyrev_readKeyrevKeycnt(&keycount, &keyrev);

    if(status != SystemP_SUCCESS )
    {
        DebugP_log("Error reading  KEYREV and KEY Count \r\n");
    }
    else
    {
        DebugP_log("Value of KEYREV: 0x%x and KEY Count: 0x%x \r\n", keyrev, keycount);
    }

#if (UPDATE_KEYREV == 1U)

    if(status == SystemP_SUCCESS)
    {
        if(keycount == 2U && keyrev != 2U)
        {
            /* Enable VPP first */
            runtime_keyrev_setVpp(EFUSE_VPP_PIN, EFUSE_VPP_PIN_LED);

            /*Update Key revision*/
            status = runtime_keyrev_writeKeyrev();

            if(status == SystemP_SUCCESS)
            {
                DebugP_log("Updating KEYREV successfull \r\n");
            }
            else
            {
                DebugP_log("Updating key REV failed \r\n");
            }
        }
        else
        {
            if(keycount == 1U)
            {
                DebugP_log("Keycount must be 2 to update key REV \r\n");
            }
            if(keyrev == 2U)
            {
                DebugP_log("Key REV is already written to eFUSE \r\n");
            }
        }
    }

#endif

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


